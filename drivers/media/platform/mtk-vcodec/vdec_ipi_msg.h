/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PC Chen <pc.chen@mediatek.com>
 */

#ifndef _VDEC_IPI_MSG_H_
#define _VDEC_IPI_MSG_H_
#include "vcodec_ipi_msg.h"

#define MTK_MAX_DEC_CODECS_SUPPORT       (128)
#define DEC_MAX_FB_NUM              VIDEO_MAX_FRAME
#define DEC_MAX_BS_NUM              VIDEO_MAX_FRAME

/**
 * enum vdec_src_chg_type - decoder src change type
 * @VDEC_NO_CHANGE      : no change
 * @VDEC_RES_CHANGE     : resolution change
 * @VDEC_REALLOC_MV_BUF : realloc mv buf
 * @VDEC_HW_NOT_SUPPORT : hw not support
 * @VDEC_NEED_MORE_OUTPUT_BUF: bs need more fm buffer to decode
 *          kernel should send the same bs buffer again with new fm buffer
 * @VDEC_CROP_CHANGED: notification to update frame crop info
 */
enum vdec_src_chg_type {
	VDEC_NO_CHANGE              = (0 << 0),
	VDEC_RES_CHANGE             = (1 << 0),
	VDEC_REALLOC_MV_BUF         = (1 << 1),
	VDEC_HW_NOT_SUPPORT         = (1 << 2),
	VDEC_NEED_SEQ_HEADER        = (1 << 3),
	VDEC_NEED_MORE_OUTPUT_BUF   = (1 << 4),
	VDEC_CROP_CHANGED           = (1 << 5),
	VDEC_OUTPUT_NOT_GENERATED   = (1 << 6),
};

enum vdec_ipi_msg_status {
	VDEC_IPI_MSG_STATUS_OK      = 0,
	VDEC_IPI_MSG_STATUS_FAIL    = -1,
	VDEC_IPI_MSG_STATUS_MAX_INST    = -2,
	VDEC_IPI_MSG_STATUS_ILSEQ   = -3,
	VDEC_IPI_MSG_STATUS_INVALID_ID  = -4,
	VDEC_IPI_MSG_STATUS_DMA_FAIL    = -5,
};

/**
 * enum vdec_ipi_msg_id - message id between AP and VCU
 * @AP_IPIMSG_XXX       : AP to VCU cmd message id
 * @VCU_IPIMSG_XXX_ACK  : VCU ack AP cmd message id
 */
enum vdec_ipi_msg_id {
	AP_IPIMSG_DEC_INIT = AP_IPIMSG_VDEC_SEND_BASE,
	AP_IPIMSG_DEC_START,
	AP_IPIMSG_DEC_END,
	AP_IPIMSG_DEC_DEINIT,
	AP_IPIMSG_DEC_RESET,
	AP_IPIMSG_DEC_SET_PARAM,
	AP_IPIMSG_DEC_QUERY_CAP,
	AP_IPIMSG_DEC_FRAME_BUFFER,
	AP_IPIMSG_DEC_BACKUP,

	VCU_IPIMSG_DEC_INIT_DONE = VCU_IPIMSG_VDEC_ACK_BASE,
	VCU_IPIMSG_DEC_START_DONE,
	VCU_IPIMSG_DEC_DONE,
	VCU_IPIMSG_DEC_DEINIT_DONE,
	VCU_IPIMSG_DEC_RESET_DONE,
	VCU_IPIMSG_DEC_SET_PARAM_DONE,
	VCU_IPIMSG_DEC_QUERY_CAP_DONE,
	VCU_IPIMSG_DEC_BACKUP_DONE,

	VCU_IPIMSG_DEC_PUT_FRAME_BUFFER = VCU_IPIMSG_VDEC_SEND_BASE,
	VCU_IPIMSG_DEC_LOCK_CORE,
	VCU_IPIMSG_DEC_UNLOCK_CORE,
	VCU_IPIMSG_DEC_LOCK_LAT,
	VCU_IPIMSG_DEC_UNLOCK_LAT,
	VCU_IPIMSG_DEC_MEM_ALLOC,
	VCU_IPIMSG_DEC_MEM_FREE,
	VCU_IPIMSG_DEC_WAITISR,
	VCU_IPIMSG_DEC_GET_FRAME_BUFFER,
	VCU_IPIMSG_DEC_CHECK_CODEC_ID,

	AP_IPIMSG_DEC_PUT_FRAME_BUFFER_DONE = AP_IPIMSG_VDEC_ACK_BASE,
	AP_IPIMSG_DEC_LOCK_CORE_DONE,
	AP_IPIMSG_DEC_UNLOCK_CORE_DONE,
	AP_IPIMSG_DEC_LOCK_LAT_DONE,
	AP_IPIMSG_DEC_UNLOCK_LAT_DONE,
	AP_IPIMSG_DEC_MEM_ALLOC_DONE,
	AP_IPIMSG_DEC_MEM_FREE_DONE,
	AP_IPIMSG_DEC_WAITISR_DONE,
	AP_IPIMSG_DEC_CHECK_CODEC_ID_DONE
 };

/**
 * enum vdec_reset_type - decoder reset type
 * @VDEC_FLUSH      : flush, no need to cotinue decode and return all frame buffers
 * @VDEC_DRAIN      : drain, need to decode done all inputs and return all decoded frames
 * @VDEC_DRAIN_EOS  : drain for EOS, except for drain, need to free one more buffer for EOS
 */
enum vdec_reset_type {
	VDEC_FLUSH = 0,
	VDEC_DRAIN = 1,
	VDEC_DRAIN_EOS = 2,
};

/* For GET_PARAM_DISP_FRAME_BUFFER and GET_PARAM_FREE_FRAME_BUFFER,
 * the caller does not own the returned buffer. The buffer will not be
 *                              released before vdec_if_deinit.
 * GET_PARAM_DISP_FRAME_BUFFER  : get next displayable frame buffer,
 *                              struct vdec_fb**
 * GET_PARAM_FREE_FRAME_BUFFER  : get non-referenced framebuffer, vdec_fb**
 * GET_PARAM_PIC_INFO           : get picture info, struct vdec_pic_info*
 * GET_PARAM_CROP_INFO          : get crop info, struct v4l2_crop*
 * GET_PARAM_DPB_SIZE           : get dpb size, __s32*
 * GET_PARAM_FRAME_INTERVAL     : get frame interval info*
 * GET_PARAM_ERRORMB_MAP        : get error mocroblock when decode error*
 * GET_PARAM_CAPABILITY_SUPPORTED_FORMATS: get codec supported format capability
 * GET_PARAM_CAPABILITY_FRAME_SIZES:
 *                       get codec supported frame size & alignment info
 */
enum vdec_get_param_type {
	GET_PARAM_DISP_FRAME_BUFFER,
	GET_PARAM_FREE_FRAME_BUFFER,
	GET_PARAM_FREE_BITSTREAM_BUFFER,
	GET_PARAM_PIC_INFO,
	GET_PARAM_CROP_INFO,
	GET_PARAM_DPB_SIZE,
	GET_PARAM_FRAME_INTERVAL,
	GET_PARAM_ERRORMB_MAP,
	GET_PARAM_VDEC_CAP_SUPPORTED_FORMATS,
	GET_PARAM_VDEC_CAP_FRAME_SIZES,
	GET_PARAM_COLOR_DESC,
	GET_PARAM_ASPECT_RATIO,
	GET_PARAM_PLATFORM_SUPPORTED_FIX_BUFFERS,
	GET_PARAM_PLATFORM_SUPPORTED_FIX_BUFFERS_SVP,
	GET_PARAM_INTERLACING,
	GET_PARAM_CODEC_TYPE,
	GET_PARAM_INPUT_DRIVEN
};

/*
 * enum vdec_set_param_type -
 *                  The type of set parameter used in vdec_if_set_param()
 * (VCU related: If you change the order, you must also update the VCU codes.)
 * SET_PARAM_DECODE_MODE: set decoder mode
 * SET_PARAM_FRAME_SIZE: set container frame size
 * SET_PARAM_SET_FIXED_MAX_OUTPUT_BUFFER: set fixed maximum buffer size
 * SET_PARAM_UFO_MODE: set UFO mode
 * SET_PARAM_CRC_PATH: set CRC path used for UT
 * SET_PARAM_GOLDEN_PATH: set Golden YUV path used for UT
 * SET_PARAM_FB_NUM_PLANES                      : frame buffer plane count
 */
enum vdec_set_param_type {
	SET_PARAM_DECODE_MODE,
	SET_PARAM_FRAME_SIZE,
	SET_PARAM_SET_FIXED_MAX_OUTPUT_BUFFER,
	SET_PARAM_UFO_MODE,
	SET_PARAM_CRC_PATH,
	SET_PARAM_GOLDEN_PATH,
	SET_PARAM_FB_NUM_PLANES,
	SET_PARAM_WAIT_KEY_FRAME,
	SET_PARAM_NAL_SIZE_LENGTH,
	SET_PARAM_OPERATING_RATE,
	SET_PARAM_TOTAL_FRAME_BUFQ_COUNT,
	SET_PARAM_FRAME_BUFFER,
	SET_PARAM_VDEC_PROPERTY,
	SET_PARAM_VDEC_VCP_LOG_INFO
};

/**
 * struct vdec_ap_ipi_cmd - generic AP to VCU ipi command format
 * @msg_id      : vdec_ipi_msg_id
 * @vcu_inst_addr       : VCU decoder instance address
 */
struct vdec_ap_ipi_cmd {
	__u32 msg_id;
	__u32 ctx_id;
	__u32 vcu_inst_addr;
	__u32 reserved;
};

/**
 * struct vdec_vcu_ipi_ack - generic VCU to AP ipi command format
 * @msg_id      : vdec_ipi_msg_id
 * @status      : VCU exeuction result, carries hw id when lock/unlock
 * @ap_inst_addr        : AP video decoder instance address
 */
struct vdec_vcu_ipi_ack {
	__u32 msg_id;
	__u32 ctx_id;
	__s32 status;
#ifndef CONFIG_64BIT
	union {
		__u64 ap_inst_addr_64;
		__u32 ap_inst_addr;
	};
#else
	__u64 ap_inst_addr;
#endif
	__s32 codec_id;
	__u32 reserved;
};

/**
 * struct vdec_vcu_ipi_mem_op -VCU/AP bi-direction memory operation cmd structure
 * @msg_id:   message id (VCU_IPIMSG_XXX_ENC_DEINIT_DONE)
 * @status:   cmd status (venc_ipi_msg_status)
 * @ap_inst_addr:	AP decoder instance (struct vdec_inst*)
 * @struct vcodec_mem_obj: encoder memories
 */
struct vdec_vcu_ipi_mem_op {
	__u32 msg_id;
	__u32 ctx_id;
	__s32 status;
	__u64 ap_inst_addr;
	struct vcodec_mem_obj mem;
	__u32 reserved[2];
};

/**
 * struct vdec_ap_ipi_init - for AP_IPIMSG_DEC_INIT
 * @msg_id      : AP_IPIMSG_DEC_INIT
 * @reserved    : Reserved field
 * @ap_inst_addr        : AP video decoder instance address
 */
struct vdec_ap_ipi_init {
	__u32 msg_id;
	__u32 ctx_id;
	__u32 reserved;
#ifndef CONFIG_64BIT
	union {
		__u64 ap_inst_addr_64;
		__u32 ap_inst_addr;
	};
#else
	__u64 ap_inst_addr;
#endif
};

/**
 * struct vdec_vcu_ipi_init_ack - for VCU_IPIMSG_DEC_INIT_ACK
 * @msg_id        : VCU_IPIMSG_DEC_INIT_ACK
 * @status        : VCU exeuction result
 * @ap_inst_addr        : AP vcodec_vcu_inst instance address
 * @vcu_inst_addr : VCU decoder instance address
 */
struct vdec_vcu_ipi_init_ack {
	__u32 msg_id;
	__u32 ctx_id;
	__s32 status;
#ifndef CONFIG_64BIT
	union {
		__u64 ap_inst_addr_64;
		__u32 ap_inst_addr;
	};
#else
	__u64 ap_inst_addr;
#endif
	__u32 vcu_inst_addr;
};

/**
 * struct vdec_ap_ipi_dec_start - for AP_IPIMSG_DEC_START
 * @msg_id      : AP_IPIMSG_DEC_START
 * @vcu_inst_addr       : VCU decoder instance address
 * @data        : Header info
 * @reserved    : Reserved field
 * @ack msg use vdec_vcu_ipi_ack
 */
struct vdec_ap_ipi_dec_start {
	__u32 msg_id;
	__u32 ctx_id;
	__u32 vcu_inst_addr;
	__u32 data[3];
	__u32 reserved;
};

/**
 * struct vdec_ap_ipi_set_param - for AP_IPIMSG_DEC_SET_PARAM
 * @msg_id        : AP_IPIMSG_DEC_SET_PARAM
 * @vcu_inst_addr : VCU decoder instance address
 * @id            : set param  type
 * @data          : param data
 */
struct vdec_ap_ipi_set_param {
	__u32 msg_id;
	__u32 ctx_id;
	__u32 vcu_inst_addr;
	__u32 id;
	__u32 data[12];
};

/**
 * struct vdec_ap_ipi_query_cap - for AP_IPIMSG_DEC_QUERY_CAP
 * @msg_id        : AP_IPIMSG_DEC_QUERY_CAP
 * @id      : query capability type
 * @vdec_inst     : AP query data address
 */
struct vdec_ap_ipi_query_cap {
	__u32 msg_id;
	__u32 ctx_id;
	__u32 id;
#ifndef CONFIG_64BIT
	union {
		__u64 ap_inst_addr_64;
		__u32 ap_inst_addr;
	};
	union {
		__u64 ap_data_addr_64;
		__u32 ap_data_addr;
	};
#else
	__u64 ap_inst_addr;
	__u64 ap_data_addr;
#endif
};

/**
 * struct vdec_vcu_ipi_query_cap_ack - for VCU_IPIMSG_DEC_QUERY_CAP_ACK
 * @msg_id      : VCU_IPIMSG_DEC_QUERY_CAP_ACK
 * @status      : VCU exeuction result
 * @ap_data_addr   : AP query data address
 * @vcu_data_addr  : VCU query data address
 */
struct vdec_vcu_ipi_query_cap_ack {
	__u32 msg_id;
	__u32 ctx_id;
	__s32 status;
#ifndef CONFIG_64BIT
	union {
		__u64 ap_inst_addr_64;
		__u32 ap_inst_addr;
	};
	__u32 id;
	union {
		__u64 ap_data_addr_64;
		__u32 ap_data_addr;
	};
#else
	__u64 ap_inst_addr;
	__u32 id;
	__u64 ap_data_addr;
#endif
	__u32 vcu_data_addr;
};

/*
 * struct vdec_ipi_fb - decoder frame buffer information
 * @vdec_fb_va  : virtual address of struct vdec_fb
 * @y_fb_dma    : dma address of Y frame buffer
 * @c_fb_dma    : dma address of C frame buffer
 * @dma_general_addr: dma address of meta
 * @general_size: meta size
 * @poc         : picture order count of frame buffer
 * @timestamp : timestamp of frame buffer
 * @reserved    : for 8 bytes alignment
 */
struct vdec_ipi_fb {
	__u64 vdec_fb_va;
	__u64 y_fb_dma;
	__u64 c_fb_dma;
	__u64 dma_general_addr;
	__u64 timestamp;
	__s32 general_size;
	__u32 reserved;
};


/**
 * struct ring_bs_list - ring bitstream buffer list
 * @vdec_bs_va_list   : bitstream buffer arrary
 * @read_idx  : read index
 * @write_idx : write index
 * @count     : buffer count in list
 */
struct ring_bs_list {
	__u64 vdec_bs_va_list[DEC_MAX_BS_NUM];
	__u32 read_idx;
	__u32 write_idx;
	__u32 count;
	__u32 reserved;
};

/**
 * struct ring_fb_list - ring frame buffer list
 * @fb_list   : frame buffer arrary
 * @read_idx  : read index
 * @write_idx : write index
 * @count     : buffer count in list
 */
struct ring_fb_list {
	struct vdec_ipi_fb fb_list[DEC_MAX_FB_NUM];
	__u32 read_idx;
	__u32 write_idx;
	__u32 count;
	__u32 reserved;
};

/**
 * struct vdec_vsi - shared memory for decode information exchange
 *                        between VCU and Host.
 *                        The memory is allocated by VCU and mapping to Host
 *                        in vcu_dec_init()
 * @ppl_buf_dma : HW working buffer ppl dma address
 * @mv_buf_dma  : HW working buffer mv dma address
 * @list_free   : free frame buffer ring list
 * @list_disp   : display frame buffer ring list
 * @dec         : decode information
 * @pic         : picture information
 * @crop        : crop information
 * @video_formats        : codec supported format info
 * @vdec_framesizes    : codec supported resolution info
 */
struct vdec_vsi {
	struct ring_bs_list list_free_bs;
	struct ring_fb_list list_free;
	struct ring_fb_list list_disp;
	struct vdec_dec_info dec;
	struct vdec_pic_info pic;
	struct mtk_color_desc color_desc;
	struct v4l2_rect crop;
	struct mtk_video_fmt video_formats[MTK_MAX_DEC_CODECS_SUPPORT];
	struct mtk_codec_framesizes vdec_framesizes[MTK_MAX_DEC_CODECS_SUPPORT];
	__u32 aspect_ratio;
	__u32 fix_buffers;
	__u32 fix_buffers_svp;
	__u32 interlacing;
	__u32 codec_type;
	__s8 crc_path[256];
	__s8 golden_path[256];
	__u8 input_driven;
	__u8 ipi_blocked;
	__s32 general_buf_fd;
	__u64 general_buf_dma;
	__u32 general_buf_size;
};

#endif
