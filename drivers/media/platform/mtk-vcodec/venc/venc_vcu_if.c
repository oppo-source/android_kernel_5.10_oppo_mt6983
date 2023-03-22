// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PoChun Lin <pochun.lin@mediatek.com>
 */

#include <linux/interrupt.h>
#include <media/v4l2-mem2mem.h>
#include <linux/mtk_vcu_controls.h>
#include <linux/delay.h>
#include "mtk_vcu.h"
#include "venc_vcu_if.h"
#include "mtk_vcodec_intr.h"
#include "mtk_vcodec_enc_pm.h"
#include "mtk_vcodec_enc.h"
#include "mtk_heap.h"


static void handle_enc_init_msg(struct venc_vcu_inst *vcu, void *data)
{
	struct venc_vcu_ipi_msg_init *msg = data;

	if (vcu == NULL)
		return;
	vcu->inst_addr = msg->vcu_inst_addr;
	vcu->vsi = VCU_FPTR(vcu_mapping_dm_addr)(vcu->dev, msg->vcu_inst_addr);
}

static void handle_query_cap_ack_msg(struct venc_vcu_ipi_query_cap_ack *msg)
{
	struct venc_vcu_inst *vcu = (struct venc_vcu_inst *)msg->ap_inst_addr;
	void *data;
	int size = 0;

	if (vcu == NULL)
		return;
	mtk_vcodec_debug(vcu, "+ ap_inst_addr = 0x%lx, vcu_data_addr = 0x%x, id = %d",
		(uintptr_t)msg->ap_inst_addr, msg->vcu_data_addr, msg->id);
	/* mapping VCU address to kernel virtual address */
	data = VCU_FPTR(vcu_mapping_dm_addr)(vcu->dev, msg->vcu_data_addr);
	if (data == NULL)
		return;
	switch (msg->id) {
	case GET_PARAM_VENC_CAP_SUPPORTED_FORMATS:
		size = sizeof(struct mtk_video_fmt);
		memcpy((void *)msg->ap_data_addr, data,
			size * MTK_MAX_ENC_CODECS_SUPPORT);
		break;
	case GET_PARAM_VENC_CAP_FRAME_SIZES:
		size = sizeof(struct mtk_codec_framesizes);
		memcpy((void *)msg->ap_data_addr, data,
			size * MTK_MAX_ENC_CODECS_SUPPORT);
		break;
	default:
		break;
	}
	mtk_vcodec_debug(vcu, "- vcu_inst_addr = 0x%x", vcu->inst_addr);
}

static void handle_enc_waitisr_msg(struct venc_vcu_inst *vcu,
	void *data, uint32_t timeout)
{
	struct venc_vcu_ipi_msg_waitisr *msg = data;
	struct mtk_vcodec_ctx *ctx = vcu->ctx;

	msg->irq_status = ctx->irq_status;
	msg->timeout = timeout;
}

static int check_codec_id(struct venc_vcu_ipi_msg_common *msg, unsigned int fmt)
{
	int codec_id = 0, ret = 0;

	switch (fmt) {
	case V4L2_PIX_FMT_H264:
		codec_id = VENC_H264;
		break;
	case V4L2_PIX_FMT_VP8:
		codec_id = VENC_VP8;
		break;
	case V4L2_PIX_FMT_MPEG4:
		codec_id = VENC_MPEG4;
		break;
	case V4L2_PIX_FMT_H263:
		codec_id = VENC_H263;
		break;
	case V4L2_PIX_FMT_H265:
		codec_id = VENC_H265;
		break;
	case V4L2_PIX_FMT_HEIF:
		codec_id = VENC_HEIF;
		break;
	default:
		pr_info("%s fourcc not supported", __func__);
		break;
	}

	if (codec_id == 0) {
		mtk_v4l2_err("[error] venc unsupported fourcc\n");
		ret = -1;
	} else if (msg->codec_id == codec_id) {
		pr_info("%s ipi id %d is correct\n", __func__, msg->codec_id);
		ret = 0;
	} else {
		mtk_v4l2_debug(2, "[Info] ipi id %d is incorrect\n", msg->codec_id);
		ret = -1;
	}

	return ret;
}

int vcu_enc_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct venc_vcu_ipi_msg_common *msg = data;
	struct venc_vcu_inst *vcu;
	struct mtk_vcodec_ctx *ctx;
	int ret = 0;
	unsigned int core_id;
	unsigned long flags;
	struct task_struct *task = NULL;

	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_msg_init) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_query_cap) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_msg_set_param) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_msg_enc) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_ap_ipi_msg_deinit) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(
		sizeof(struct venc_vcu_ipi_query_cap_ack) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_common) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_init) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(
		sizeof(struct venc_vcu_ipi_msg_set_param) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_enc) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_deinit) > SHARE_BUF_SIZE);
	BUILD_BUG_ON(sizeof(struct venc_vcu_ipi_msg_waitisr) > SHARE_BUF_SIZE);

	VCU_FPTR(vcu_get_task)(&task, 0);
	if (msg == NULL || task == NULL ||
	   task->tgid != current->tgid ||
	   (struct venc_vcu_inst *)(unsigned long)msg->venc_inst == NULL) {
		ret = -EINVAL;
		return ret;
	}

	vcu = (struct venc_vcu_inst *)(unsigned long)msg->venc_inst;
	if ((vcu != priv) && (msg->msg_id < VCU_IPIMSG_VENC_SEND_BASE)) {
		pr_info("%s, vcu:%p != priv:%p\n", __func__, vcu, priv);
		return 1;
	}

	if (vcu->daemon_pid != current->tgid) {
		pr_info("%s, vcu->daemon_pid:%d != current %d\n",
			__func__, vcu->daemon_pid, current->tgid);
		return 1;
	}

	mtk_vcodec_debug(vcu, "msg_id %x inst %p status %d",
					 msg->msg_id, vcu, msg->status);

	if (vcu->abort)
		return -EINVAL;

	ctx = vcu->ctx;
	switch (msg->msg_id) {
	case VCU_IPIMSG_ENC_INIT_DONE:
		handle_enc_init_msg(vcu, data);
		break;
	case VCU_IPIMSG_ENC_SET_PARAM_DONE:
		/* Prevent slowmotion with GCE mode on,
		 * user thread enter freezing while holding mutex (enc lock)
		 */
		current->flags |= PF_NOFREEZE;
		break;
	case VCU_IPIMSG_ENC_DEINIT_DONE:
		break;
	case VCU_IPIMSG_ENC_POWER_ON:
		/*use status to store core ID*/
		venc_encode_prepare(ctx, msg->status, &flags);
		msg->status = VENC_IPI_MSG_STATUS_OK;
		ret = 1;
		break;
	case VCU_IPIMSG_ENC_POWER_OFF:
		/*use status to store core ID*/
		venc_encode_unprepare(ctx, msg->status, &flags);
		msg->status = VENC_IPI_MSG_STATUS_OK;
		ret = 1;
		break;
	case VCU_IPIMSG_ENC_QUERY_CAP_DONE:
		handle_query_cap_ack_msg(data);
		break;
	case VCU_IPIMSG_ENC_WAIT_ISR:
		core_id = msg->status;
		if (core_id == MTK_VENC_CORE_0)
			vcodec_trace_count("VENC_HW_CORE_0", 2);
		else
			vcodec_trace_count("VENC_HW_CORE_1", 2);

		if (-1 == mtk_vcodec_wait_for_done_ctx(ctx, core_id,
			MTK_INST_IRQ_RECEIVED,
			WAIT_INTR_TIMEOUT_MS)) {
			handle_enc_waitisr_msg(vcu, data, 1);
			mtk_vcodec_debug(vcu,
				"irq_status %x <-", ctx->irq_status);
		} else
			handle_enc_waitisr_msg(vcu, data, 0);

		if (core_id == MTK_VENC_CORE_0)
			vcodec_trace_count("VENC_HW_CORE_0", 1);
		else
			vcodec_trace_count("VENC_HW_CORE_1", 1);

		ret = 1;
		break;
	case VCU_IPIMSG_ENC_PUT_BUFFER:
		mtk_enc_put_buf(ctx);
		ret = 1;
		break;
	case VCU_IPIMSG_ENC_ENCODE_DONE:
		break;
	case VCU_IPIMSG_ENC_CHECK_CODEC_ID:
		if (check_codec_id(msg, ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc) == 0)
			msg->status = 0;
		else
			msg->status = -1;
		ret = 1;
		break;
	default:
		mtk_vcodec_err(vcu, "unknown msg id %x", msg->msg_id);
		break;
	}

	/* deinit done timeout case handling do not touch vdec_vcu_inst
	 * or memory used after freed
	 */
	if (msg->msg_id != VCU_IPIMSG_ENC_DEINIT_DONE) {
		vcu->signaled = 1;
		vcu->failure = (msg->status != VENC_IPI_MSG_STATUS_OK);
	}

	mtk_vcodec_debug_leave(vcu);
	return ret;
}

static int vcu_enc_send_msg(struct venc_vcu_inst *vcu, void *msg,
							int len)
{
	int status;
	struct task_struct *task = NULL;
	unsigned int suspend_block_cnt = 0;

	mtk_vcodec_debug_enter(vcu);

	if (!vcu->dev) {
		mtk_vcodec_err(vcu, "inst dev is NULL");
		return -EINVAL;
	}

	if (vcu->abort)
		return -EIO;

	while (vcu->ctx->dev->is_codec_suspending == 1) {
		suspend_block_cnt++;
		if (suspend_block_cnt > SUSPEND_TIMEOUT_CNT) {
			mtk_v4l2_debug(4, "VENC blocked by suspend\n");
			suspend_block_cnt = 0;
		}
		usleep_range(10000, 20000);
	}

	VCU_FPTR(vcu_get_task)(&task, 0);
	if (task == NULL ||
		vcu->daemon_pid != task->tgid) {
		if (task)
			mtk_vcodec_err(vcu, "send fail pid: inst %d curr %d",
				vcu->daemon_pid, task->tgid);
		vcu->abort = 1;
		return -EIO;
	}

	status = VCU_FPTR(vcu_ipi_send)(vcu->dev, vcu->id, msg, len, vcu);
	if (status) {
		mtk_vcodec_err(vcu, "vcu_ipi_send msg_id %x len %d fail %d",
					   *(uint32_t *)msg, len, status);
		if (status == -EIO)
			vcu->abort = 1;
		return status;
	}
	else
		mtk_vcodec_debug(vcu, "vcu_ipi_send msg_id %x len %d success",
				*(uint32_t *)msg, len);
	if (vcu->failure)
		return -EINVAL;

	mtk_vcodec_debug_leave(vcu);

	return 0;
}


void vcu_enc_set_pid(struct venc_vcu_inst *vcu)
{
	struct task_struct *task = NULL;

	VCU_FPTR(vcu_get_task)(&task, 0);
	if (task != NULL)
		vcu->daemon_pid = task->tgid;
	else
		vcu->daemon_pid = -1;
}

int vcu_enc_set_ctx(struct venc_vcu_inst *vcu,
	struct venc_frm_buf *frm_buf,
	struct mtk_vcodec_mem *bs_buf)
{
	int err = 0;
	struct vb2_buffer *src_vb = NULL;
	struct vb2_buffer *dst_vb = NULL;
	struct mtk_video_enc_buf *temp;

	if (frm_buf != NULL) {
		temp = container_of(frm_buf, struct mtk_video_enc_buf, frm_buf);
		src_vb = &temp->vb.vb2_buf;
	}
	if (bs_buf != NULL) {
		temp = container_of(bs_buf, struct mtk_video_enc_buf, bs_buf);
		dst_vb = &temp->vb.vb2_buf;
	}
	VCU_FPTR(vcu_set_codec_ctx)(vcu->dev,
		(void *)vcu->ctx,
		src_vb, dst_vb,
		VCU_VENC);

	return err;
}

int vcu_enc_clear_ctx(struct venc_vcu_inst *vcu)
{
	int err = 0;

	VCU_FPTR(vcu_clear_codec_ctx)(vcu->dev,
		(void *)vcu->ctx, VCU_VENC);

	return err;
}

int vcu_enc_init(struct venc_vcu_inst *vcu)
{
	int status;
	struct venc_ap_ipi_msg_init out;

	mtk_vcodec_debug_enter(vcu);

	if (sizeof(out) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_msg_init cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	vcu->signaled = 0;
	vcu->failure = 0;
	VCU_FPTR(vcu_get_ctx_ipi_binding_lock)(vcu->dev, &vcu->ctx_ipi_lock, VCU_VENC);

	status = VCU_FPTR(vcu_ipi_register)(vcu->dev, vcu->id, vcu->handler,
							  NULL, vcu);
	if (status) {
		mtk_vcodec_err(vcu, "vcu_ipi_register fail %d", status);
		return -EINVAL;
	}

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_INIT;
	out.venc_inst = (unsigned long)vcu;

	vcu_enc_set_pid(vcu);
	status = vcu_enc_send_msg(vcu, &out, sizeof(out));

	if (status) {
		mtk_vcodec_err(vcu, "AP_IPIMSG_ENC_INIT fail");
		return -EINVAL;
	}
	mtk_vcodec_debug_leave(vcu);

	return 0;
}

int vcu_enc_query_cap(struct venc_vcu_inst *vcu, unsigned int id, void *out)
{
	struct venc_ap_ipi_query_cap msg;
	int err = 0;

	if (sizeof(msg) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_query_cap cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	mtk_vcodec_debug(vcu, "+ id=%X", AP_IPIMSG_ENC_QUERY_CAP);
	vcu->dev = VCU_FPTR(vcu_get_plat_device)(vcu->ctx->dev->plat_dev);
	if (vcu->dev  == NULL) {
		mtk_vcodec_err(vcu, "vcu device in not ready");
		return -EPROBE_DEFER;
	}

	vcu->id = (vcu->id == IPI_VCU_INIT) ? IPI_VENC_COMMON : vcu->id;
	vcu->handler = vcu_enc_ipi_handler;

	err = VCU_FPTR(vcu_ipi_register)(vcu->dev,
		vcu->id, vcu->handler, NULL, vcu);
	if (err != 0) {
		mtk_vcodec_err(vcu, "vcu_ipi_register fail status=%d", err);
		return err;
	}

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = AP_IPIMSG_ENC_QUERY_CAP;
	msg.id = id;
	msg.ap_inst_addr = (uintptr_t)vcu;
	msg.ap_data_addr = (uintptr_t)out;

	vcu_enc_set_pid(vcu);
	err = vcu_enc_send_msg(vcu, &msg, sizeof(msg));
	mtk_vcodec_debug(vcu, "- id=%X ret=%d", msg.msg_id, err);

	return err;
}

int vcu_enc_set_param(struct venc_vcu_inst *vcu,
					  enum venc_set_param_type id,
					  struct venc_enc_param *enc_param)
{
	struct venc_ap_ipi_msg_set_param out;

	mtk_vcodec_debug(vcu, "id %d ->", id);
	if (sizeof(out) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_msg_set_param cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_SET_PARAM;
	out.vcu_inst_addr = vcu->inst_addr;
	out.param_id = id;
	switch (id) {
	case VENC_SET_PARAM_ENC:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_FORCE_INTRA:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_ADJUST_BITRATE:
		out.data_item = 1;
		out.data[0] = enc_param->bitrate;
		break;
	case VENC_SET_PARAM_ADJUST_FRAMERATE:
		out.data_item = 1;
		out.data[0] = enc_param->frm_rate;
		break;
	case VENC_SET_PARAM_GOP_SIZE:
		out.data_item = 1;
		out.data[0] = enc_param->gop_size;
		break;
	case VENC_SET_PARAM_INTRA_PERIOD:
		out.data_item = 1;
		out.data[0] = enc_param->intra_period;
		break;
	case VENC_SET_PARAM_SKIP_FRAME:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_PREPEND_HEADER:
		out.data_item = 0;
		break;
	case VENC_SET_PARAM_SCENARIO:
		out.data_item = 1;
		out.data[0] = enc_param->scenario;
		break;
	case VENC_SET_PARAM_NONREFP:
		out.data_item = 1;
		out.data[0] = enc_param->nonrefp;
		break;
	case VENC_SET_PARAM_NONREFPFREQ:
		out.data_item = 1;
		out.data[0] = enc_param->nonrefpfreq;
		break;
	case VENC_SET_PARAM_DETECTED_FRAMERATE:
		out.data_item = 1;
		out.data[0] = enc_param->detectframerate;
		break;
	case VENC_SET_PARAM_RFS_ON:
		out.data_item = 1;
		out.data[0] = enc_param->rfs;
		break;
	case VENC_SET_PARAM_PREPEND_SPSPPS_TO_IDR:
		out.data_item = 1;
		out.data[0] = enc_param->prependheader;
		break;
	case VENC_SET_PARAM_OPERATION_RATE:
		out.data_item = 1;
		out.data[0] = enc_param->operationrate;
		break;
	case VENC_SET_PARAM_BITRATE_MODE:
		out.data_item = 1;
		out.data[0] = enc_param->bitratemode;
		break;
	case VENC_SET_PARAM_ROI_ON:
		out.data_item = 1;
		out.data[0] = enc_param->roion;
		break;
	case VENC_SET_PARAM_HEIF_GRID_SIZE:
		out.data_item = 1;
		out.data[0] = enc_param->heif_grid_size;
		break;
	case VENC_SET_PARAM_COLOR_DESC:
		out.data_item = 0; // passed via vsi
		break;
	case VENC_SET_PARAM_SEC_MODE:
		out.data_item = 1;
		out.data[0] = enc_param->svp_mode;
		break;
	case VENC_SET_PARAM_TSVC:
		out.data_item = 1;
		out.data[0] = enc_param->tsvc;
		break;
	case VENC_SET_PARAM_ENABLE_HIGHQUALITY:
		out.data_item = 1;
		out.data[0] = enc_param->highquality;
		break;
	case VENC_SET_PARAM_ADJUST_MAX_QP:
		out.data_item = 1;
		out.data[0] = enc_param->max_qp;
		break;
	case VENC_SET_PARAM_ADJUST_MIN_QP:
		out.data_item = 1;
		out.data[0] = enc_param->min_qp;
		break;
	case VENC_SET_PARAM_ADJUST_I_P_QP_DELTA:
		out.data_item = 1;
		out.data[0] = enc_param->ip_qpdelta;
		break;
	case VENC_SET_PARAM_ADJUST_FRAME_LEVEL_QP:
		out.data_item = 1;
		out.data[0] = enc_param->framelvl_qp;
		break;
	case VENC_SET_PARAM_ENABLE_DUMMY_NAL:
		out.data_item = 1;
		out.data[0] = enc_param->dummynal;
		break;
	default:
		mtk_vcodec_err(vcu, "id %d not supported", id);
		return -EINVAL;
	}

	if (vcu_enc_send_msg(vcu, &out, sizeof(out))) {
		mtk_vcodec_err(vcu,
			"AP_IPIMSG_ENC_SET_PARAM %d fail", id);
		return -EINVAL;
	}

	mtk_vcodec_debug(vcu, "id %d <-", id);

	return 0;
}

int vcu_enc_encode(struct venc_vcu_inst *vcu, unsigned int bs_mode,
				   struct venc_frm_buf *frm_buf,
				   struct mtk_vcodec_mem *bs_buf,
				   unsigned int *bs_size)
{

	struct venc_ap_ipi_msg_enc out;
	struct venc_vsi *vsi = (struct venc_vsi *)vcu->vsi;
	unsigned int i, ret;

	mtk_vcodec_debug(vcu, "bs_mode %d ->", bs_mode);

	if (sizeof(out) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_msg_enc cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_ENCODE;
	out.vcu_inst_addr = vcu->inst_addr;
	out.bs_mode = bs_mode;
	if (frm_buf) {
		out.fb_num_planes = frm_buf->num_planes;
		for (i = 0; i < frm_buf->num_planes; i++) {
			out.input_addr[i] =
				frm_buf->fb_addr[i].dma_addr;
			vsi->venc.fb_dma[i] =
				frm_buf->fb_addr[i].dma_addr;
			out.input_size[i] =
				frm_buf->fb_addr[i].size;
			out.data_offset[i] =
				frm_buf->fb_addr[i].data_offset;
		}
		if (frm_buf->has_meta) {
			vsi->meta_addr = frm_buf->meta_addr;
			vsi->meta_size = sizeof(struct mtk_hdr_dynamic_info);
			vsi->meta_offset = frm_buf->meta_offset;
		} else {
			vsi->meta_size = 0;
			vsi->meta_addr = 0;
		}

		if (frm_buf->has_qpmap) {
			vsi->qpmap_addr = frm_buf->qpmap_dma_addr;
			vsi->qpmap_size = frm_buf->qpmap_dma->size;
		} else {
			vsi->qpmap_addr = 0;
			vsi->qpmap_size = 0;
		}

		mtk_vcodec_debug(vcu, " num_planes = %d input (dmabuf:%lx), size %d %llx",
			frm_buf->num_planes,
			(unsigned long)frm_buf->fb_addr[0].dmabuf,
			vsi->meta_size,
			vsi->meta_addr);
		mtk_vcodec_debug(vcu, "vsi qpmap addr %llx size%d",
			vsi->meta_addr, vsi->qpmap_size);
	}
	else {
		mtk_vcodec_debug(vcu, "frm_buf is null");
	}

	if (bs_buf) {
		out.bs_addr = bs_buf->dma_addr;
		vsi->venc.bs_dma = bs_buf->dma_addr;
		out.bs_size = bs_buf->size;

		if (vsi->config.svp_mode) {
			out.sec_mem_handle = dmabuf_to_secure_handle(bs_buf->dmabuf);
			pr_info("%s %d out.sec_mem_handle 0x%x", __func__,
				 __LINE__, out.sec_mem_handle);
		}

		mtk_vcodec_debug(vcu, " output (dma:%lx)",
			(unsigned long)bs_buf->dmabuf);
	}
	else {
		mtk_vcodec_debug(vcu, "bs_buf is null");
	}

	mutex_lock(vcu->ctx_ipi_lock);
	vcu_enc_set_ctx(vcu, frm_buf, bs_buf);
	ret = vcu_enc_send_msg(vcu, &out, sizeof(out));
	mutex_unlock(vcu->ctx_ipi_lock);

	if (ret) {
		mtk_vcodec_err(vcu, "AP_IPIMSG_ENC_ENCODE %d fail %d",
					   bs_mode, ret);
		return ret;
	}
	else{
		mtk_vcodec_debug(vcu, " vcu_enc_send_msg done");
	}

	mtk_vcodec_debug(vcu, "bs_mode %d size %d key_frm %d <-",
		bs_mode, vcu->bs_size, vcu->is_key_frm);

	return 0;
}

int vcu_enc_deinit(struct venc_vcu_inst *vcu)
{
	struct venc_ap_ipi_msg_deinit out;
	int ret = 0;

	mtk_vcodec_debug_enter(vcu);

	if (sizeof(out) > SHARE_BUF_SIZE) {
		mtk_vcodec_err(vcu, "venc_ap_ipi_msg_deint cannot be large than %d",
					   SHARE_BUF_SIZE);
		return -EINVAL;
	}

	memset(&out, 0, sizeof(out));
	out.msg_id = AP_IPIMSG_ENC_DEINIT;
	out.vcu_inst_addr = vcu->inst_addr;

	mutex_lock(vcu->ctx_ipi_lock);
	ret = vcu_enc_send_msg(vcu, &out, sizeof(out));
	vcu_enc_clear_ctx(vcu);
	mutex_unlock(vcu->ctx_ipi_lock);
	current->flags &= ~PF_NOFREEZE;

	mtk_vcodec_debug_leave(vcu);

	return ret;
}

