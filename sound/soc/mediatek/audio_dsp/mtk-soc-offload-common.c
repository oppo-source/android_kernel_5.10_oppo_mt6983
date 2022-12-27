// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2021 MediaTek Inc.
// Author: Even Yang <even.yang@mediatek.com>
#include <linux/compat.h>

/* wake lock relate*/
#include <linux/device.h>
#include <linux/pm_wakeup.h>

#include "mtk-dsp-common.h"
#include "mtk-dsp-common_define.h"
#include "mtk-dsp-mem-control.h"
#include "audio_buf.h"
#include "mtk-auddrv-offloadcommon.h"
#include "mtk-base-dsp.h"

#include <audio_task_manager.h>
#include <audio_ipi_dma.h>

#include <trace/hooks/vendor_hooks.h>
#include <trace/hooks/snd_compr.h>

/*
 * Variable Definition
 */
#define USE_PERIODS_MAX        8192
#define OFFLOAD_SIZE_BYTES         (USE_PERIODS_MAX << 9) /* 4M */
#define FILL_BUFFERING             (USE_PERIODS_MAX << 3) /* 64K */
#define RESERVE_DRAMPLAYBACKSIZE   (USE_PERIODS_MAX << 2) /* 32 K*/
#define ID AUDIO_TASK_OFFLOAD_ID
#define GENPOOL_ID AUDIO_DSP_AFE_SHARE_MEM_ID

enum {
	TASK_SCENE_OFFLOAD_MP3,
	TASK_SCENE_OFFLOAD_AAC
};

typedef uint8_t task_offload_scene_t;

static task_offload_scene_t OFFLOAD_TYPE = TASK_SCENE_OFFLOAD_MP3;

static struct afe_offload_service_t afe_offload_service = {
	.write_blocked   = false,
	.enable          = false,
	.drain           = false,
	.tswait          = false,
	.needdata        = false,
	.decode_error    = false,
	.volume          = 0x10000,
	.scene           = TASK_SCENE_PLAYBACK_MP3,
};

static struct afe_offload_param_t afe_offload_block = {
	.state             = OFFLOAD_STATE_INIT,
	.samplerate        = 0,
	.transferred       = 0,
	.copied_total      = 0,
	.write_blocked_idx = 0,
	.wakelock          = false,
	.drain_state       = AUDIO_DRAIN_NONE,
};

static struct afe_offload_codec_t afe_offload_codec_info = {
	.codec_samplerate = 0,
	.codec_bitrate = 0,
	.target_samplerate = 0,
};

static struct snd_compr_stream *offload_stream;

static struct device *offload_dev;


static bool offload_playback_pause;
static bool offload_playback_resume;
#define use_wake_lock
static unsigned long ringbuf_writebk;
static unsigned long long ringbufbridge_writebk;

#ifdef use_wake_lock
static DEFINE_SPINLOCK(offload_lock);
struct wakeup_source *Offload_suspend_lock;
#endif
static struct mtk_base_dsp *dsp;
static unsigned int offload_buffer_size;

/*
 * Function  Declaration
 */
static void offloadservice_ipicmd_received(struct ipi_msg_t *ipi_msg);
static bool offloadservice_tswait(unsigned int id);
static int offloadservice_copydatatoram(void __user *buf, size_t count);
#ifdef use_wake_lock
static void mtk_compr_offload_int_wakelock(bool enable);
#endif

/*
 * Function Implementation
 */

static void offloadservice_setwriteblocked(bool flag)
{
	afe_offload_service.write_blocked = flag;
}

static void offloadservice_releasewriteblocked(void)
{
	offload_stream->runtime->state = SNDRV_PCM_STATE_RUNNING;
	wake_up(&offload_stream->runtime->sleep);
}

static int offloadservice_setvolume(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	afe_offload_service.volume =
		(unsigned int)ucontrol->value.integer.value[0];
	mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
			 AUDIO_IPI_MSG_ONLY,
			 AUDIO_IPI_MSG_BYPASS_ACK,
			 OFFLOAD_VOLUME, afe_offload_service.volume,
			 afe_offload_service.volume, NULL);
	return 0;
}

static int offloadservice_getvolume(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = afe_offload_service.volume;
	return 0;
}

static int offloadservice_setformat(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	OFFLOAD_TYPE = (task_offload_scene_t)ucontrol->value.integer.value[0];
	pr_debug("%s OFFLOAD_TYPE = %d\n", __func__, OFFLOAD_TYPE);
	return 0;
}

static int offloadservice_getformat(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	if (afe_offload_service.decode_error == true) {
		pr_info("offloadgetformat decode_error\n");
		ucontrol->value.integer.value[0] = 1;
	}
	return 0;
}

static int offloadservice_setbuffersize(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	offload_buffer_size = (unsigned int)ucontrol->value.integer.value[0];
	pr_info("%s offload_buffer_size = %d\n", __func__, offload_buffer_size);
	return 0;
}

static int offloadservice_getbuffersize(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = offload_buffer_size;
	return 0;
}

static int offloadservice_settargetrate(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	afe_offload_codec_info.target_samplerate = (unsigned int)ucontrol->value.integer.value[0];
	pr_debug("%s target_samplerate = %d\n", __func__, afe_offload_codec_info.target_samplerate);
	return 0;
}

static int offloadservice_gettargetrate(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = afe_offload_codec_info.target_samplerate;
	return 0;
}


static const struct snd_kcontrol_new Audio_snd_dloffload_controls[] = {
	SOC_SINGLE_EXT("offload digital volume", SND_SOC_NOPM, 0, 0x1000000, 0,
	offloadservice_getvolume, offloadservice_setvolume),
	SOC_SINGLE_EXT("offload set format", SND_SOC_NOPM, 0,
	TASK_SCENE_PLAYBACK_MP3, 0,
	offloadservice_getformat, offloadservice_setformat),
	SOC_SINGLE_EXT("offload_buffer_size", SND_SOC_NOPM, 0, 0x400000, 0,
	offloadservice_getbuffersize, offloadservice_setbuffersize),
	SOC_SINGLE_EXT("offload_target_rate", SND_SOC_NOPM, 0, 0x100000, 0,
	offloadservice_gettargetrate, offloadservice_settargetrate),
};

/*
 * offload driver operations
 */
#ifdef use_wake_lock
static void mtk_compr_offload_int_wakelock(bool enable)
{
	spin_lock(&offload_lock);
	if (enable ^ afe_offload_block.wakelock) {
		if (enable)
			aud_wake_lock(Offload_suspend_lock);
		else
			aud_wake_unlock(Offload_suspend_lock);
		afe_offload_block.wakelock = enable;
	}
	spin_unlock(&offload_lock);
}
#endif

static int mtk_compr_offload_draindone(void)
{
	if (afe_offload_block.state == OFFLOAD_STATE_DRAIN) {
		pr_info("%s\n", __func__);
		/* gapless mode clear vars */
		afe_offload_block.write_blocked_idx = 0;
		afe_offload_block.drain_state       = AUDIO_DRAIN_ALL;
		/* for gapless */
		offloadservice_setwriteblocked(false);
		offloadservice_releasewriteblocked();
	}
	return 0;
}

int mtk_compr_offload_copy(struct snd_soc_component *component,
			   struct snd_compr_stream *stream, char __user *buf,
			   size_t count)
{
	int ret = 0;

#ifdef use_wake_lock
	mtk_compr_offload_int_wakelock(true);
#endif
	ret = offloadservice_copydatatoram(buf, count);
	if (afe_offload_service.decode_error == true)
		ret = -1;
	return ret;
}

static int mtk_compr_offload_drain(struct snd_compr_stream *stream)
{
	struct RingBuf *ringbuf = &(dsp->dsp_mem[ID].ring_buf);
	struct ringbuf_bridge *buf_bridge =
		&(dsp->dsp_mem[ID].adsp_buf.aud_buffer.buf_bridge);

	int silence_length = 0;
	int ret;
	void *ipi_audio_buf; /* dsp <-> audio data struct */
	struct audio_dsp_dram *dsp_dram;

	dsp_dram = &dsp->dsp_mem[ID].msg_atod_share_buf;
	if (afe_offload_block.state != OFFLOAD_STATE_DRAIN) {
		if ((afe_offload_block.state != OFFLOAD_STATE_RUNNING) &&
			(afe_offload_block.transferred < 8 * USE_PERIODS_MAX)) {
			/* send audio_hw_buffer to SCP side, get writeIndx*/
			ipi_audio_buf = (void *)dsp_dram->va_addr;
			memcpy((void *)ipi_audio_buf,
					(void *)&dsp->dsp_mem[ID].adsp_buf,
					sizeof(struct audio_hw_buffer));
			ret = mtk_scp_ipi_send(
					get_dspscene_by_dspdaiid(ID),
					AUDIO_IPI_PAYLOAD,
					AUDIO_IPI_MSG_BYPASS_ACK,
					AUDIO_DSP_TASK_DLCOPY,
					sizeof(dsp->dsp_mem[ID].msg_atod_share_buf.phy_addr),
					0,
					(char *)
					&dsp->dsp_mem[ID].msg_atod_share_buf.phy_addr);
			pr_debug("%s(),MSG_DECODER_START, Update the final data, TRANSFERRED %lld\n",
					__func__, afe_offload_block.transferred);
		}

		silence_length = 0;
		RingBuf_update_writeptr(ringbuf, silence_length);
		RingBuf_Bridge_update_writeptr(buf_bridge, silence_length);
		ringbuf_writebk = (unsigned long)ringbuf->pWrite;
		ringbufbridge_writebk = buf_bridge->pWrite;
		afe_offload_service.needdata = false;

		pr_info("%s, OFFLOAD_DRAIN", __func__);
		ret = mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
			AUDIO_IPI_PAYLOAD,
			AUDIO_IPI_MSG_NEED_ACK,
			OFFLOAD_DRAIN,
			sizeof(buf_bridge->pWrite),
			0,
			(void *)&buf_bridge->pWrite);

		afe_offload_block.state = OFFLOAD_STATE_DRAIN;
		afe_offload_block.drain_state = AUDIO_DRAIN_EARLY_NOTIFY;
	}
#ifdef use_wake_lock
	mtk_compr_offload_int_wakelock(false);
#endif
	pr_info("%s-", __func__);
	//return -1;  /* make compress driver drain failed if use write_wait */
	return 0;
}

static int mtk_compr_offload_open(struct snd_soc_component *component,
				  struct snd_compr_stream *stream)
{
	int ret = 0;
#ifdef use_wake_lock
	mtk_compr_offload_int_wakelock(true);
#endif

	mtk_scp_ipi_send(TASK_SCENE_PLAYBACK_MP3,
			 AUDIO_IPI_MSG_ONLY,
			 AUDIO_IPI_MSG_NEED_ACK,
			 AUDIO_DSP_TASK_OPEN,
			 0,
			 0,
			 NULL);
	offload_stream = stream;

	if (dsp == NULL) {
		dsp = (struct mtk_base_dsp *)get_dsp_base();
		pr_debug("get_dsp_base again\n");
	}

	if (offload_buffer_size < 262144) { // 256K
		pr_debug("%s err offload_buffer_size = %u\n", __func__, offload_buffer_size);
		return -1;
	}

	/* gen pool related */
	dsp->dsp_mem[ID].gen_pool_buffer =
			mtk_get_adsp_dram_gen_pool(GENPOOL_ID);
	if (dsp->dsp_mem[ID].gen_pool_buffer != NULL) {
		pr_debug("gen_pool_avail = %zu poolsize = %zu\n",
			gen_pool_avail(
				dsp->dsp_mem[ID].gen_pool_buffer),
			gen_pool_size(
				dsp->dsp_mem[ID].gen_pool_buffer));

		/* allocate ring buffer wioth share memory*/
		ret = mtk_adsp_genpool_allocate_sharemem_ring(
			      &dsp->dsp_mem[ID],
			      offload_buffer_size,
			      ID);
		pr_debug("%s() allocate offload bufsize = %u\n", __func__, offload_buffer_size);

		if (ret < 0) {
			pr_debug("%s err\n", __func__);
			return -1;
		}
		pr_debug("gen_pool_avail = %zu poolsize = %zu\n",
			 gen_pool_avail(
			 dsp->dsp_mem[ID].gen_pool_buffer),
			 gen_pool_size(
			 dsp->dsp_mem[ID].gen_pool_buffer));
	}
	return 0;
}

static void mtk_compr_use_pause_in_drain(void *ignore, bool *use_pause_in_drain,
					 bool *leave_draining_state)
{
	*use_pause_in_drain = true;
	*leave_draining_state = true;
	pr_info("%s\n", __func__);
}

static int mtk_afe_dloffload_probe(struct snd_soc_component *component)
{
	int ret = 0;
	int ret_vh = 0;

	ret = snd_soc_add_component_controls(component,
					     Audio_snd_dloffload_controls,
					     ARRAY_SIZE(Audio_snd_dloffload_controls));
	if (ret)
		pr_info("%s add_component err ret = %d\n", __func__, ret);

	ret_vh = register_trace_android_vh_snd_compr_use_pause_in_drain(
				mtk_compr_use_pause_in_drain, NULL);
	if (ret_vh)
		pr_info("%s register vh err %d\n", __func__, ret_vh);

	return ret;
}

static int mtk_compr_offload_free(struct snd_soc_component *component,
				  struct snd_compr_stream *stream)
{
	pr_debug("%s()\n", __func__);
	offloadservice_setwriteblocked(false);
	if (dsp)
		mtk_adsp_genpool_free_sharemem_ring(&dsp->dsp_mem[ID], ID);
	afe_offload_block.state = OFFLOAD_STATE_INIT;
#ifdef use_wake_lock
	mtk_compr_offload_int_wakelock(false);
#endif
	return 0;
}

static int mtk_compr_offload_set_params(struct snd_soc_component *component,
					struct snd_compr_stream *stream,
					struct snd_compr_params *params)
{
	struct snd_codec codec;
	struct audio_hw_buffer *audio_hwbuf;
	struct mtk_base_dsp_mem *audio_dsp_mem;
	void *ipi_audio_buf; /* dsp <-> audio data struct*/
	int ret = 0;

	audio_task_register_callback(
			 TASK_SCENE_PLAYBACK_MP3,
			 offloadservice_ipicmd_received);

	codec = params->codec;
	afe_offload_block.samplerate = codec.sample_rate;

	//set shared Dram meme
	audio_hwbuf = &dsp->dsp_mem[ID].adsp_buf;
	audio_dsp_mem = &dsp->dsp_mem[ID];
	dump_audio_dsp_dram(&dsp->dsp_mem[ID].dsp_ring_share_buf);
	//set codec info
	afe_offload_codec_info.codec_samplerate = codec.sample_rate;
	afe_offload_codec_info.codec_bitrate = codec.bit_rate;
	audio_hwbuf->aud_buffer.buffer_attr.channel = codec.ch_out;
	audio_hwbuf->aud_buffer.buffer_attr.format = codec.format;
	audio_hwbuf->aud_buffer.buffer_attr.rate = afe_offload_codec_info.target_samplerate;

	ret = set_audiobuffer_hw(&dsp->dsp_mem[ID].adsp_buf,
				 BUFFER_TYPE_SHARE_MEM);
	if (ret < 0)
		goto ERROR;
	ret = set_audiobuffer_memorytype(&dsp->dsp_mem[ID].adsp_buf,
					 MEMORY_AUDIO_DRAM);
	if (ret < 0)
		goto ERROR;

	/* send codec info to SCP side */
	mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
			 AUDIO_IPI_MSG_ONLY,
			 AUDIO_IPI_MSG_BYPASS_ACK,
			 OFFLOAD_CODEC_INFO,
			 afe_offload_codec_info.codec_bitrate,
			 afe_offload_codec_info.codec_samplerate
			 , NULL);

	/* send audio_hw_buffer to SCP side */
	ipi_audio_buf =
		(void *)dsp->dsp_mem[ID].msg_atod_share_buf.va_addr;
	pr_debug("%s offload ipi_audio_buf = %p\n", __func__, ipi_audio_buf);
	memcpy((void *)ipi_audio_buf,
	       (void *)&dsp->dsp_mem[ID].adsp_buf,
	       sizeof(struct audio_hw_buffer));

	dump_audio_hwbuffer(ipi_audio_buf);
	dump_rbuf_s(__func__, &dsp->dsp_mem[ID].ring_buf);

	/* send to task with hw_param information , buffer and pcm attribute */
	mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
			 AUDIO_IPI_PAYLOAD,
			 AUDIO_IPI_MSG_NEED_ACK, AUDIO_DSP_TASK_HWPARAM,
			 sizeof(dsp->dsp_mem[ID].msg_atod_share_buf.phy_addr),
			 0,
			 (char *)
			 &dsp->dsp_mem[ID].msg_atod_share_buf.phy_addr);

	pr_debug("%s AUDIO_DSP_TASK_HWPARAM Done\n", __func__);
	mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
			 AUDIO_IPI_MSG_ONLY,
			 AUDIO_IPI_MSG_NEED_ACK,
			 OFFLOAD_SCENE, OFFLOAD_TYPE, OFFLOAD_TYPE, NULL);
	pr_debug("%s OFFLOAD_SCENE Done\n", __func__);
	return ret;

ERROR:
	pr_debug("%s err\n", __func__);
	return -1;
}

static int mtk_compr_offload_get_params(struct snd_soc_component *component,
					struct snd_compr_stream *stream,
					struct snd_codec *params)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int mtk_compr_offload_get_caps(struct snd_soc_component *component,
				      struct snd_compr_stream *stream,
				      struct snd_compr_caps *caps)
{
	pr_debug("%s\n", __func__);
	caps->num_codecs        = 2;
	caps->codecs[0]         = SND_AUDIOCODEC_PCM;
	caps->codecs[1]         = SND_AUDIOCODEC_MP3;
	caps->min_fragment_size = 8192;
	caps->max_fragment_size = 0x7FFFFFFF;
	caps->min_fragments     = 2;
	caps->max_fragments     = 1875;
	return 0;
}

static int mtk_compr_offload_get_codec_caps(struct snd_soc_component *component,
					    struct snd_compr_stream *stream,
					    struct snd_compr_codec_caps *codec)
{
	pr_debug("%s()\n", __func__);
	return 0;
}

static int mtk_compr_offload_set_metadata(struct snd_soc_component *component,
					  struct snd_compr_stream *stream,
					  struct snd_compr_metadata *metadata)
{
	pr_debug("%s()\n", __func__);
	return 0;
}

static int mtk_compr_offload_get_metadata(struct snd_soc_component *component,
					  struct snd_compr_stream *stream,
					  struct snd_compr_metadata *metadata)
{
	pr_debug("%s()\n", __func__);
	return 0;
}

static int mtk_compr_offload_mmap(struct snd_soc_component *component,
				  struct snd_compr_stream *stream,
				  struct vm_area_struct *vma)
{
	pr_debug("%s()\n", __func__);
	return 0;
}

static int mtk_dsp_mp3_dl_handler(struct mtk_base_dsp *dsp,
				   struct ipi_msg_t *ipi_msg, int id)
{
	/* get dsp_mem */
	struct mtk_base_dsp_mem *dsp_mem = &dsp->dsp_mem[id];
	struct RingBuf *ringbuf = &(dsp->dsp_mem[ID].ring_buf);
	struct ringbuf_bridge *buf_bridge =
		&(dsp->dsp_mem[ID].adsp_buf.aud_buffer.buf_bridge);
	char *readidx = NULL;
	spinlock_t *ringbuf_lock = &dsp_mem->ringbuf_lock;
	unsigned long flags = 0;
	int ret = 0;

#ifdef DEBUG_VERBOSE
	dump_rbuf_s(__func__, &dsp->dsp_mem[id].ring_buf);
	pr_debug("%s msg_id = %u param1 = %u param2 = %u", __func__,
		 ipi_msg->msg_id, ipi_msg->param1, ipi_msg->param2);
#endif
	if (ipi_msg->data_type == AUDIO_IPI_PAYLOAD) {
		struct audio_hw_buffer bufTemp;
		struct ringbuf_bridge *bufTmp_bridge;

		memcpy((void *)&bufTemp,
		       (void *)dsp_mem->msg_dtoa_share_buf.vir_addr,
		       sizeof(struct audio_hw_buffer));
		bufTmp_bridge = &(bufTemp.aud_buffer.buf_bridge);

		ringbuf->pWrite = (char *)ringbuf_writebk;
		buf_bridge->pWrite = ringbufbridge_writebk;
		buf_bridge->pRead = buf_bridge->pBufBase +
				    (bufTmp_bridge->pRead - bufTmp_bridge->pBufBase);

		readidx = ringbuf->pBufBase +
			  (bufTmp_bridge->pRead - bufTmp_bridge->pBufBase);
		if (readidx != ringbuf->pRead) {
			spin_lock_irqsave(ringbuf_lock, flags);
			sync_ringbuf_readidx(&dsp_mem->ring_buf,
					     &dsp_mem->adsp_buf.aud_buffer.buf_bridge);
			spin_unlock_irqrestore(ringbuf_lock, flags);
			pr_debug("%s update read ptr!", __func__);
			return ret;
		}
		pr_debug("%s no need to update read ptr!", __func__);
	}
#ifdef DEBUG_VERBOSE
	dump_rbuf_s(__func__, &dsp->dsp_mem[id].ring_buf);
#endif
	return -1;
}

static void offloadservice_ipicmd_received(struct ipi_msg_t *ipi_msg)
{
	struct mtk_base_dsp *dsp =
		(struct mtk_base_dsp *)get_ipi_recv_private();
	int id = 0;
	int retval = 0;

	if (ipi_msg == NULL) {
		pr_info("%s ipi_msg == NULL\n", __func__);
		return;
	}

	if (dsp == NULL) {
		pr_debug("%s dsp == NULL\n", __func__);
		return;
	}

	id = get_dspdaiid_by_dspscene(ipi_msg->task_scene);
	if (id < 0)
		return;

	switch (ipi_msg->msg_id) {
	case AUDIO_DSP_TASK_IRQDL:
		retval = mtk_dsp_mp3_dl_handler(dsp, ipi_msg, id);
		if (!retval) {
			offloadservice_setwriteblocked(false);
			offloadservice_releasewriteblocked();
			afe_offload_service.needdata = true;
		}
		break;
	case OFFLOAD_PCMCONSUMED:
		afe_offload_block.copied_total = ipi_msg->param1;
		afe_offload_block.time_pcm = ktime_get();
		afe_offload_block.time_pcm_delay_ms = ipi_msg->param2;
		mutex_lock(&afe_offload_service.ts_lock);
		afe_offload_service.tswait = false;
		wake_up_interruptible(&afe_offload_service.ts_wq);
		mutex_unlock(&afe_offload_service.ts_lock);
		break;
	case OFFLOAD_DRAINDONE:
		pr_info("%s mtk_compr_offload_draindone\n", __func__);
		mtk_compr_offload_draindone();
		break;
	case OFFLOAD_DECODE_ERROR:
		afe_offload_service.decode_error = true;
		offloadservice_setwriteblocked(false);
		offloadservice_releasewriteblocked();
		pr_info("%s decode_error\n", __func__);
		break;
	case OFFLOAD_CODEC_INFO:
		if (ipi_msg->param1) {
			afe_offload_codec_info.codec_bitrate =  ipi_msg->param1;
			pr_info("%s update bir_rate[%u]\n", __func__, ipi_msg->param1);
		}
		if (ipi_msg->param2) {
			afe_offload_codec_info.codec_samplerate = ipi_msg->param2;
			pr_info("%s sample_rate[%u]\n", __func__, ipi_msg->param2);
		}
	default:
		break;
	}
#ifdef DEBUG_VERBOSE
	pr_debug("%s msg_id:%d\n", __func__, ipi_msg->msg_id);
#endif
}

static bool offloadservice_tswait(unsigned int id)
{
	int retval;

	retval = wait_event_interruptible_timeout(
		 afe_offload_service.ts_wq,
		 !afe_offload_service.tswait,
		 msecs_to_jiffies(OFFLOAD_IPIMSG_TIMEOUT));
	if (!retval)
		pr_info("%s time out\n", __func__);

	return retval;
}

static int offloadservice_copydatatoram(void __user *buf, size_t count)
{
	void *ipi_audio_buf; /* dsp <-> audio data struct */
	int copy_size, availsize, ret = 0;
	static unsigned int u4round = 1;
	int transferred = 0;
	struct RingBuf *ringbuf = &(dsp->dsp_mem[ID].ring_buf);
	struct ringbuf_bridge *buf_bridge =
		&(dsp->dsp_mem[ID].adsp_buf.aud_buffer.buf_bridge);
	struct audio_dsp_dram *dsp_dram;
	struct mtk_base_dsp_mem *dsp_mem = &dsp->dsp_mem[ID];
	spinlock_t *ringbuf_lock = &dsp_mem->ringbuf_lock;
	unsigned long flags = 0;

	dsp_dram = &dsp->dsp_mem[ID].msg_atod_share_buf;
	copy_size = count;
	availsize = RingBuf_getFreeSpace(ringbuf);
#ifdef DEBUG_VERBOSE
	pr_debug("%s copy_size = %d availsize = %d\n",
		__func__, copy_size,
		RingBuf_getFreeSpace(ringbuf));

	dump_rbuf_s(__func__, &dsp->dsp_mem[ID].ring_buf);
#endif

	if (availsize >= copy_size) {
		RingBuf_copyFromUserLinear(ringbuf, buf, copy_size);
		spin_lock_irqsave(ringbuf_lock, flags);
		RingBuf_Bridge_update_writeptr(buf_bridge, copy_size);
		spin_unlock_irqrestore(ringbuf_lock, flags);
		afe_offload_block.transferred += count;
		ringbuf_writebk = (unsigned long)ringbuf->pWrite;
		ringbufbridge_writebk = buf_bridge->pWrite;
	} else {
		//Liang: checked below, should not happened
		pr_debug("%s fail copy_size = %d availsize = %d\n", __func__,
			 copy_size, RingBuf_getFreeSpace(ringbuf));
		goto Error;
	}
	//check for next time writable
	if (count >= RingBuf_getFreeSpace(ringbuf)) {
		offloadservice_setwriteblocked(true);
		afe_offload_block.write_blocked_idx =
			buf_bridge->pWrite;
		afe_offload_service.needdata = false;
		u4round = 1;
		mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
				AUDIO_IPI_PAYLOAD,
				AUDIO_IPI_MSG_BYPASS_ACK,
				OFFLOAD_SETWRITEBLOCK,
				sizeof(afe_offload_block.write_blocked_idx),
				0,
				(void *)&afe_offload_block.write_blocked_idx);
#ifdef use_wake_lock
		mtk_compr_offload_int_wakelock(false);
#endif
		pr_debug("%s buffer full , WIdx=%lld\n",
			__func__, buf_bridge->pWrite);

	}

#ifdef DEBUG_VERBOSE
	dump_rbuf_s(__func__, &dsp->dsp_mem[ID].ring_buf);
	Ringbuf_Check(&dsp->dsp_mem[ID].ring_buf);
	Ringbuf_Bridge_Check(
		&dsp->dsp_mem[ID].adsp_buf.aud_buffer.buf_bridge);
#endif

	if (afe_offload_service.needdata) {
		transferred = RingBuf_getDataCount(ringbuf);
		if (transferred >=
		    (32 * USE_PERIODS_MAX) * u4round) {
			/* notify writeIDX to SCP each 256K*/
			mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
				AUDIO_IPI_PAYLOAD,
				AUDIO_IPI_MSG_BYPASS_ACK,
				OFFLOAD_WRITEIDX,
				sizeof(buf_bridge->pWrite),
				0,
				(void *)&buf_bridge->pWrite);
			u4round++;
		}
	}
	if ((afe_offload_block.state != OFFLOAD_STATE_RUNNING) &&
		((afe_offload_block.transferred >= 8 * USE_PERIODS_MAX) ||
			(afe_offload_block.transferred < 8 * USE_PERIODS_MAX &&
			afe_offload_block.state == OFFLOAD_STATE_DRAIN))) {
		/* send audio_hw_buffer to SCP side, get writeIndx*/
		ipi_audio_buf = (void *)dsp_dram->va_addr;
		memcpy((void *)ipi_audio_buf,
			(void *)&dsp->dsp_mem[ID].adsp_buf,
			sizeof(struct audio_hw_buffer));
		ret = mtk_scp_ipi_send(
				get_dspscene_by_dspdaiid(ID),
				AUDIO_IPI_PAYLOAD,
				AUDIO_IPI_MSG_BYPASS_ACK,
				AUDIO_DSP_TASK_DLCOPY,
				sizeof(dsp->dsp_mem[ID].msg_atod_share_buf.phy_addr),
				0,
				(char *)
				&dsp->dsp_mem[ID].msg_atod_share_buf.phy_addr);
#ifdef DEBUG_VERBOSE
		pr_debug("%s copy_size = %d availsize = %d\n",
				__func__, copy_size, RingBuf_getFreeSpace(ringbuf));
#endif
		pr_debug("%s(),MSG_DECODER_START, TRANSFERRED %lld\n",
				__func__, afe_offload_block.transferred);
		afe_offload_block.state = OFFLOAD_STATE_RUNNING;
		u4round = 1;
	}
	return count;
Error:
	pr_debug("%s copy failed\n", __func__);
	return -1;
}

static int mtk_compr_send_query_tstamp(void)
{
	mutex_lock(&afe_offload_service.ts_lock);
	if (!afe_offload_service.tswait && !offload_playback_pause) {
		mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
		AUDIO_IPI_MSG_ONLY, AUDIO_IPI_MSG_BYPASS_ACK,
		OFFLOAD_TSTAMP, 0, 0, NULL);
		afe_offload_service.tswait = true;
	}
	mutex_unlock(&afe_offload_service.ts_lock);
	return 0;
}

static int mtk_compr_offload_pointer(struct snd_soc_component *component,
				     struct snd_compr_stream *stream,
				     struct snd_compr_tstamp *tstamp)
{
	int ret = 0;
	int data = 0;
	u64 pcm_compensate = 0;

	mtk_compr_send_query_tstamp();

	if (afe_offload_block.state == OFFLOAD_STATE_INIT ||
	    afe_offload_block.state == OFFLOAD_STATE_IDLE ||
	    afe_offload_block.state == OFFLOAD_STATE_PREPARE) {
		tstamp->copied_total  = 0;
		tstamp->sampling_rate = afe_offload_block.samplerate;
		tstamp->pcm_io_frames = 0;
		return 0;
	}

	if (afe_offload_block.state == OFFLOAD_STATE_RUNNING)
		offloadservice_tswait(OFFLOAD_PCMCONSUMED);

	if (!afe_offload_service.needdata) {
		tstamp->copied_total  =
			afe_offload_block.transferred;
	} else {
		tstamp->copied_total  =
			afe_offload_block.copied_total;
	}
	if (afe_offload_service.write_blocked ||  /* Dram full */
	    afe_offload_block.state == OFFLOAD_STATE_DRAIN) {
		data = (afe_offload_block.transferred - (8 * USE_PERIODS_MAX));
		if (afe_offload_block.drain_state == AUDIO_DRAIN_ALL) {
			tstamp->copied_total = data > 0 ? data : afe_offload_block.transferred;
		} else {
			tstamp->copied_total =	data > 0 ? data : 0;
		}
	}
	if (offload_playback_pause) {
		tstamp->copied_total =
			afe_offload_block.transferred;
	}
	if (afe_offload_block.samplerate != afe_offload_codec_info.codec_samplerate)
		tstamp->sampling_rate = afe_offload_codec_info.codec_samplerate;
	else
		tstamp->sampling_rate = afe_offload_block.samplerate;
	// check for if pcm_delay should < 100ms
	if (afe_offload_block.state == OFFLOAD_STATE_DRAIN ||
	    afe_offload_block.state == OFFLOAD_STATE_RUNNING) {
		if ((afe_offload_block.time_pcm_delay_ms > 0) &&
		    (afe_offload_block.time_pcm_delay_ms < 100) &&
		    (afe_offload_block.copied_total > 0)) {
			pcm_compensate = afe_offload_block.samplerate *
					 afe_offload_block.time_pcm_delay_ms / 1000;
		}
	}
	tstamp->pcm_io_frames = (afe_offload_block.copied_total >> 2)
				+ pcm_compensate; /* DSP return 16bit data */
	tstamp->pcm_io_frames = tstamp->pcm_io_frames&0Xffffff80;

	return ret;
}

/*
 * Offload trigger operations
 */
static int mtk_compr_offload_start(struct snd_compr_stream *stream)
{
	int ret = 0;

	afe_offload_block.state = OFFLOAD_STATE_PREPARE;
	offload_playback_pause = false;
	afe_offload_block.drain_state = AUDIO_DRAIN_NONE;
	memset(&afe_offload_block.time_pcm, 0, sizeof(ktime_t));
	ret = mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID), AUDIO_IPI_MSG_ONLY,
			       AUDIO_IPI_MSG_DIRECT_SEND, AUDIO_DSP_TASK_START,
			       1, 0, NULL);
	return 0;
}

static int mtk_compr_offload_resume(struct snd_compr_stream *stream)
{
	int ret = 0;
	if ((afe_offload_block.transferred >= 8 * USE_PERIODS_MAX) ||
	    (afe_offload_block.transferred < 8 * USE_PERIODS_MAX &&
	     ((afe_offload_block.drain_state == AUDIO_DRAIN_EARLY_NOTIFY) ||
	      (afe_offload_block.state == OFFLOAD_STATE_DRAIN)))) {
		ret = mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
				       AUDIO_IPI_MSG_ONLY,
				       AUDIO_IPI_MSG_NEED_ACK,
				       OFFLOAD_RESUME,
				       1, 0, NULL);

		if (afe_offload_block.drain_state != AUDIO_DRAIN_EARLY_NOTIFY)
			afe_offload_block.state = OFFLOAD_STATE_RUNNING;
	}
	offloadservice_releasewriteblocked();
	offload_playback_pause = false;
	offload_playback_resume = true;
	memset(&afe_offload_block.time_pcm, 0, sizeof(ktime_t));
	mtk_compr_send_query_tstamp();
	return 0;
}

static int mtk_compr_offload_pause(struct snd_compr_stream *stream)
{
	int ret = 0;
	pr_debug("%s\n", __func__);

	offloadservice_releasewriteblocked();
#ifdef use_wake_lock
	mtk_compr_offload_int_wakelock(false);
#endif
	if ((afe_offload_block.transferred >= 8 * USE_PERIODS_MAX) ||
	    (afe_offload_block.transferred < 8 * USE_PERIODS_MAX &&
	     ((afe_offload_block.drain_state == AUDIO_DRAIN_EARLY_NOTIFY) ||
	      (afe_offload_block.state == OFFLOAD_STATE_DRAIN)))) {
		ret = mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
					AUDIO_IPI_MSG_ONLY,
					AUDIO_IPI_MSG_BYPASS_ACK,
					OFFLOAD_PAUSE,
					1, 0, NULL);
		pr_debug("%s > transferred\n", __func__);
	}
	offload_playback_pause = true;
	offload_playback_resume = false;
	return 0;
}

static int mtk_compr_deinit_offload_service(struct afe_offload_service_t *service)
{
	if (!service)
		return -1;
	service->write_blocked = false;
	service->enable = false;
	service->drain = false;
	service->tswait = false;
	service->needdata = false;
	service->decode_error = false;
	service->pcmdump = false;
	return 0;
}

static int mtk_compr_offload_stop(struct snd_compr_stream *stream)
{
	int ret = 0;

	afe_offload_block.state = OFFLOAD_STATE_IDLE;
	//SetOffloadEnableFlag(false);
	/* stop hw */
	mtk_scp_ipi_send(get_dspscene_by_dspdaiid(ID),
				 AUDIO_IPI_MSG_ONLY,
				 AUDIO_IPI_MSG_NEED_ACK,
				 AUDIO_DSP_TASK_STOP, 1,
				 0, NULL);
	afe_offload_block.transferred       = 0;
	afe_offload_block.copied_total      = 0;
	afe_offload_block.write_blocked_idx = 0;
	afe_offload_block.drain_state       = AUDIO_DRAIN_NONE;
	mtk_compr_deinit_offload_service(&afe_offload_service);
	offloadservice_setwriteblocked(false);
	offloadservice_releasewriteblocked();
	clear_audiobuffer_hw(&dsp->dsp_mem[ID].adsp_buf);
	RingBuf_Reset(&dsp->dsp_mem[ID].ring_buf);

#ifdef use_wake_lock
	mtk_compr_offload_int_wakelock(false);
#endif
	return ret;
}

/*
 * mtk_compr_offload_trigger
 */
static int mtk_compr_offload_trigger(struct snd_soc_component *component,
				     struct snd_compr_stream *stream, int cmd)
{
	pr_debug("%s cmd:%x\n", __func__, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		return mtk_compr_offload_start(stream);
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_compr_offload_resume(stream);
	case SNDRV_PCM_TRIGGER_STOP:
		return mtk_compr_offload_stop(stream);
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_compr_offload_pause(stream);
	case SND_COMPR_TRIGGER_DRAIN:
		return mtk_compr_offload_drain(stream);
	}
	return 0;
}

static int mtk_asoc_dloffload_new(struct snd_soc_component *component,
								  struct snd_soc_pcm_runtime *rtd)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int mtk_dloffload_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
#ifdef use_wake_lock
	aud_wake_lock_destroy(Offload_suspend_lock);
#endif
	return 0;
}

static struct snd_compress_ops mtk_offload_compr_ops = {
	.open            = mtk_compr_offload_open,
	.free            = mtk_compr_offload_free,
	.set_params      = mtk_compr_offload_set_params,
	.get_params      = mtk_compr_offload_get_params,
	.set_metadata    = mtk_compr_offload_set_metadata,
	.get_metadata    = mtk_compr_offload_get_metadata,
	.trigger         = mtk_compr_offload_trigger,
	.pointer         = mtk_compr_offload_pointer,
	.copy            = mtk_compr_offload_copy,
	.mmap            = mtk_compr_offload_mmap,
	.ack             = NULL,
	.get_caps        = mtk_compr_offload_get_caps,
	.get_codec_caps  = mtk_compr_offload_get_codec_caps,
};

static const struct snd_soc_component_driver mtk_dloffload_soc_platform = {
	.compress_ops        = &mtk_offload_compr_ops,
	.pcm_construct    = mtk_asoc_dloffload_new,
	.probe      = mtk_afe_dloffload_probe,
};

static int mtk_offload_init(void)
{
	mutex_init(&afe_offload_service.ts_lock);
	init_waitqueue_head(&afe_offload_service.ts_wq);
	return 0;
}

static int mtk_dloffload_probe(struct platform_device *pdev)
{
	int ret = 0;
#ifdef use_wake_lock
	Offload_suspend_lock = aud_wake_lock_init(NULL, "Offload wakelock");
	if (Offload_suspend_lock == NULL)
		pr_info("%s init Offload_suspend_lock error\n", __func__);
#endif

	offload_dev = &pdev->dev;
	dsp = (struct mtk_base_dsp *)get_dsp_base();

	if (pdev->dev.of_node) {
		dev_set_name(&pdev->dev, "%s", "mt_soc_offload_common");
		pdev->name = pdev->dev.kobj.name;
	} else {
		pr_debug("%s(), offload_dev->dev.of_node = NULL!!!\n", __func__);
	}

	pr_info("%s: dev name %s\n", __func__, dev_name(offload_dev));

	ret = snd_soc_register_component(offload_dev, &mtk_dloffload_soc_platform,
					 NULL, 0);
	if (ret)
		dev_info(offload_dev, "%s() err_offload_platform: %d\n", __func__, ret);

	mtk_offload_init();

	return 0;
}


#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mt_soc_offload_common_dt_match[] = {
	{ .compatible = "mediatek,mt_soc_offload_common", },
	{}
};
MODULE_DEVICE_TABLE(of, mt_soc_offload_common_dt_match);
#endif


static struct platform_driver mtk_offloadplayback_driver = {
	.driver = {
		.name = "mt_soc_offload_common",
		.owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = mt_soc_offload_common_dt_match,
#endif
	},
	.probe = mtk_dloffload_probe,
	.remove = mtk_dloffload_remove,
};

module_platform_driver(mtk_offloadplayback_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek Offload Driver");
