/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PC Chen <pc.chen@mediatek.com>
 */

#ifndef _VDEC_VCU_IF_H_
#define _VDEC_VCU_IF_H_

#include <linux/dma-buf.h>
#include "mtk_vcu.h"
#include "vdec_ipi_msg.h"

/**
 * struct vdec_vcu_inst - VCU instance for video codec
 * @wq          : wait queue to wait VCU message ack
 * @ipi_id      : ipi id for decoder or encoder
 * @vsi         : driver structure allocated by VCU side and shared to AP side
 *                for control and info share
 * @failure     : VCU execution result status, 0: success, others: fail
 * @inst_addr   : VCU decoder instance address
 * @signaled    : 1 - Host has received ack message from VCU, 0 - not received
 * @ctx         : context for v4l2 layer integration
 * @dev         : platform device of VCU
 * @handler     : ipi handler for each decoder
 * @abort       : abort when vpud crashed stop this instance ipi_msg
 */
struct vdec_vcu_inst {
	wait_queue_head_t wq;
	wait_queue_head_t wq_res;
	enum ipi_id id;
	void *vsi;
	int32_t failure;
	uint32_t inst_addr;
	unsigned int signaled;
	unsigned int signaled_res;
	struct mtk_vcodec_ctx *ctx;
	struct platform_device *dev;
	ipi_handler_t handler;
	bool abort;
	int daemon_pid;
	struct mutex *ctx_ipi_lock;
	struct list_head bufs;
};

/**
 * vcu_dec_init - init decoder instance and allocate required resource in VCU.
 *
 * @vcu: instance for vdec_vcu_inst
 */
int vcu_dec_init(struct vdec_vcu_inst *vcu);

/**
 * vcu_dec_start - start decoding, basically the function will be invoked once
 *                 every frame.
 *
 * @vcu : instance for vdec_vcu_inst
 * @data: meta data to pass bitstream info to VCU decoder
 * @len : meta data length
 */
int vcu_dec_start(struct vdec_vcu_inst *vcu,
	uint32_t *data, unsigned int len,
	struct mtk_vcodec_mem *bs, struct vdec_fb *fb);

/**
 * vcu_dec_end - end decoding, basically the function will be invoked once
 *               when HW decoding done interrupt received successfully. The
 *               decoder in VCU will continute to do referene frame management
 *               and check if there is a new decoded frame available to display.
 *
 * @vcu : instance for vdec_vcu_inst
 */
int vcu_dec_end(struct vdec_vcu_inst *vcu);

/**
 * vcu_dec_deinit - deinit decoder instance and resource freed in VCU.
 *
 * @vcu: instance for vdec_vcu_inst
 */
int vcu_dec_deinit(struct vdec_vcu_inst *vcu);

/**
 * vcu_dec_reset - reset decoder, use for flush decoder when end of stream or
 *                 seek. Remainig non displayed frame will be pushed to display.
 *
 * @vcu: instance for vdec_vcu_inst
 * @drain_type: flush (0), drain (1),  drain with EOS (2)
 */
int vcu_dec_reset(struct vdec_vcu_inst *vcu, enum vdec_reset_type drain_type);

/**
 * vcu_dec_ipi_handler - Handler for VCU ipi message.
 *
 * @data: ipi message
 * @len : length of ipi message
 * @priv: callback private data which is passed by decoder when register.
 */
int vcu_dec_ipi_handler(void *data, unsigned int len, void *priv);
int vcu_dec_query_cap(struct vdec_vcu_inst *vcu, unsigned int id, void *out);
int vcu_dec_set_param(struct vdec_vcu_inst *vcu, unsigned int id,
					  void *param, unsigned int size);
int vcu_dec_set_frame_buffer(struct vdec_vcu_inst *vcu, void *fb);

#endif
