/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek Controls Header
 *
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Yunfei Dong <yunfei.dong@mediatek.com>
 */

#ifndef __MTK_VCU_CONTROLS_H__
#define __MTK_VCU_CONTROLS_H__

#include <linux/types.h>
#include <linux/string.h>
#include <linux/compat.h>

#define SHARE_BUF_SIZE 80
#define LOG_INFO_SIZE 1024
#define VCODEC_CMDQ_CMD_MAX           (2048)

/**
 * struct mem_obj - memory buffer allocated in kernel
 *
 * @iova:	iova of buffer
 * @len:	buffer length
 * @pa:	physical address
 * @va: kernel virtual address
 */
struct mem_obj {
	uint64_t iova;
	uint32_t len;
	uint64_t pa;
	uint64_t va;
};

/**
 * struct map_obj - memory buffer mmaped in kernel
 *
 * @map_buf:	iova of buffer
 *	0: not mapped buf; 1: mapped buf
 * @map_type:	the type of mmap
 *	0: reserved; 1: MM_BASE;
 *	2: MM_CACHEABLE_BASE; 3: PA_BASE
 * @reserved: reserved
 */
struct map_obj {
	uint32_t map_buf;
	uint32_t map_type;
	uint64_t reserved;
};

/**
 * struct gce_cmds - cmds buffer
 *
 * @cmd:	gce cmd
 * @addr:	cmd operation addr
 * @data:	cmd operation data
 * @mask:  cmd operation mask
 * @cmd_cnt: cmdq total cmd count
 */
struct gce_cmds {
	uint8_t  cmd[VCODEC_CMDQ_CMD_MAX];
	uint64_t addr[VCODEC_CMDQ_CMD_MAX];
	uint64_t data[VCODEC_CMDQ_CMD_MAX];
	uint32_t mask[VCODEC_CMDQ_CMD_MAX];
	uint32_t dma_offset[VCODEC_CMDQ_CMD_MAX];
	uint32_t dma_size[VCODEC_CMDQ_CMD_MAX];
	uint32_t cmd_cnt;
};

/**
 * struct gce_cmdq_obj - cmdQ buffer allocated in kernel
 *
 * @cmds_user_ptr: user pointer to struct gce_cmds
 * @gce_handle: instance handle
 * @flush_order: cmdQ buffer order
 * @codec_type: decoder(1) or encoder(0)
 */
struct gce_cmdq_obj {
	uint64_t	cmds_user_ptr;
	uint64_t	gce_handle;
	uint32_t	flush_order;
	uint32_t	codec_type;
	uint32_t	core_id;
	uint32_t secure;
};

/**
 * struct gce_obj - gce allocated in kernel
 * @gce_handle: instance handle
 * @flush_order: cmdQ buffer order
 * @codec_type: decoder(1) or encoder(0)
 */
struct gce_obj {
	uint64_t	gce_handle;
	uint32_t	flush_order;
	uint32_t	codec_type;
};

struct disp_iova_obj {
	uint32_t	mapped_iova;
};

enum gce_cmd_id {
	CMD_READ = 0,    /* read register */
	CMD_WRITE,       /* write register */
	CMD_POLL_REG,
	/* polling register until get some value (no timeout, blocking wait) */
	CMD_WAIT_EVENT,      /* gce wait HW done event & clear */
	CMD_MEM_MV,      /* copy memory data from PA to another PA */
	CMD_POLL_ADDR,
	/* polling addr until get some value (with timeout) */
	CMD_SEC_WRITE,   /* sec dma write register */
	CMD_MAX
};

enum gce_event_id {
	VDEC_EVENT_0,    /* pic_start (each spec trigger decode will get) */
	VDEC_EVENT_1,    /* decode done, VDEC_TOP(41) bit16=1 */
	VDEC_EVENT_2,    /* vdec_pause (WDMA(9)bit0 or bit1=1) */
	VDEC_EVENT_3,    /* vdec_dec_error (each spec. decode error will get) */
	VDEC_EVENT_4,
	/* mc_busy_overflow | mdec_timeout
	 * (decode to VLD_TOP(20) or VLD_TOP(22) will get)
	 */
	VDEC_EVENT_5,
	/* all_dram_req & all_dram_cnt_0 & bits_proc_nop_1
	 * & bits_proc_nop_2, break or pic_finish need wait
	 */
	VDEC_EVENT_6,    /* ini_fetch_rdy VLD(58)bit0=1 */
	VDEC_EVENT_7,
	/* process_flag VLD(61)bit15=0 ||
	 * VLD(61)bit15=1 && VLD(61)bit0=0
	 */
	VDEC_EVENT_8,
	/* "search_start_code_done HEVC_VLD(37)bit8=0"
	 * "search_start_code_doneAVC_VLD(182)bit0=0"
	 * "ctx_count_dma_rdyVP9_VLD(170)bit0=1"
	 */
	VDEC_EVENT_9,
	/* "ref_reorder_doneHEVC_VLD(37)bit4=0"
	 * "ref_reorder_doneAVC_VLD(139)bit0=1"
	 * "& update_probs_rdy& VP9_VLD(51) = 1"
	 */
	VDEC_EVENT_10,
	/* "wp_tble_doneHEVC_VLD(37)bit0=0"
	 * "wp_tble_doneAVC_VLD(140)bit0=1"
	 * "bool_init_rdyVP9_VLD(68)bit16 = 1"
	 */
	VDEC_EVENT_11,
	/* "count_sram_clr_done &
	 * ctx_sram_clr_doneVP9_VLD(106)bit0 =0 &
	 * VP9_VLD(166)bit0 = 0"
	 */
	VDEC_EVENT_12,   /* reserved */
	VDEC_EVENT_13,   /* reserved */
	VDEC_EVENT_14,   /* reserved */
	VDEC_EVENT_15,   /* Queue Counter OP threshold */
	VDEC_LAT_EVENT_0,
	VDEC_LAT_EVENT_1,
	VDEC_LAT_EVENT_2,
	VDEC_LAT_EVENT_3,
	VDEC_LAT_EVENT_4,
	VDEC_LAT_EVENT_5,
	VDEC_LAT_EVENT_6,
	VDEC_LAT_EVENT_7,
	VDEC_LAT_EVENT_8,
	VDEC_LAT_EVENT_9,
	VDEC_LAT_EVENT_10,
	VDEC_LAT_EVENT_11,
	VDEC_LAT_EVENT_12,
	VDEC_LAT_EVENT_13,
	VDEC_LAT_EVENT_14,
	VDEC_LAT_EVENT_15,
	VDEC_EVENT_COUNT,
	VENC_EOF = VDEC_EVENT_COUNT,
	VENC_CMDQ_PAUSE_DONE,
	VENC_MB_DONE,
	VENC_128BYTE_CNT_DONE,
	VENC_EOF_C1,
	VENC_WP_2ND_DONE,
	VENC_WP_3ND_DONE,
	VENC_SPS_DONE,
	VENC_PPS_DONE
};


#define VCU_SET_OBJECT	_IOW('v', 0, struct share_obj)
#define VCU_MVA_ALLOCATION	_IOWR('v', 1, struct mem_obj)
#define VCU_MVA_FREE		_IOWR('v', 2, struct mem_obj)
#define VCU_CACHE_FLUSH_ALL	_IOWR('v', 3, struct mem_obj)
#define VCU_CACHE_FLUSH_BUFF	_IOWR('v', 4, struct mem_obj)
#define VCU_CACHE_INVALIDATE_BUFF	_IOWR('v', 5, struct mem_obj)
#define VCU_PA_ALLOCATION	_IOWR('v', 6, struct mem_obj)
#define VCU_PA_FREE		_IOWR('v', 7, struct mem_obj)
#define VCU_GCE_SET_CMD_FLUSH _IOW('v', 8, struct gce_cmdq_obj)
#define VCU_GCE_WAIT_CALLBACK _IOW('v', 9, struct gce_obj)
#define VCU_GET_OBJECT		_IOWR('v', 10, struct share_obj)
#define VCU_GET_LOG_OBJECT	_IOW('v', 11, struct log_test_nofuse)
#define VCU_SET_LOG_OBJECT	_IOW('v', 12, struct log_test)
#define VCU_SET_MMAP_TYPE	_IOW('v', 13, struct map_obj)
#define VCU_GET_DISP_MAPPED_IOVA	_IOWR('v', 14, struct disp_iova_obj)
#define VCU_CLEAR_DISP_MAPPED_IOVA	_IOW('v', 15, struct disp_iova_obj)
#define VCU_GET_DISP_WDMA_Y_ADDR	_IOWR('v', 16, struct disp_iova_obj)
#define VCU_UBE_MVA_ALLOCATION	_IOWR('v', 17, struct mem_obj)
#define VCU_UBE_MVA_FREE	_IOWR('v', 18, struct mem_obj)

#define COMPAT_VCU_SET_OBJECT		_IOW('v', 0, struct share_obj)
#define COMPAT_VCU_MVA_ALLOCATION	_IOWR('v', 1, struct compat_mem_obj)
#define COMPAT_VCU_MVA_FREE		_IOWR('v', 2, struct compat_mem_obj)
#define COMPAT_VCU_CACHE_FLUSH_ALL	_IOWR('v', 3, struct compat_mem_obj)
#define COMPAT_VCU_CACHE_FLUSH_BUFF	_IOWR('v', 4, struct compat_mem_obj)
#define COMPAT_VCU_CACHE_INVALIDATE_BUFF _IOWR('v', 5, struct compat_mem_obj)
#define COMPAT_VCU_PA_ALLOCATION	_IOWR('v', 6, struct compat_mem_obj)
#define COMPAT_VCU_PA_FREE		_IOWR('v', 7, struct compat_mem_obj)
#define COMPAT_VCU_SET_MMAP_TYPE	_IOW('v', 13, struct map_obj)
#define COMPAT_VCU_UBE_MVA_ALLOCATION	_IOWR('v', 17, struct compat_mem_obj)
#define COMPAT_VCU_UBE_MVA_FREE	_IOWR('v', 18, struct compat_mem_obj)

struct compat_mem_obj {
	uint64_t iova;
	uint32_t len;
	compat_u64 pa;
	compat_u64 va;
};

/**
 * struct share_obj - DTCM (Data Tightly-Coupled Memory) buffer shared with
 *		      AP and VCU
 *
 * @id:		IPI id
 * @len:	share buffer length
 * @share_buf:	share buffer data
 */
struct share_obj {
	int32_t id;
	uint32_t len;
	unsigned char share_buf[SHARE_BUF_SIZE];
};

struct log_test_nofuse {
	char log_info[LOG_INFO_SIZE];
};

#endif

