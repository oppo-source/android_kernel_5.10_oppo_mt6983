/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __MTK_FENCE_H__
#define __MTK_FENCE_H__

#include <linux/mutex.h>
#include <linux/list.h>
#include <drm/mediatek_drm.h>
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_plane.h"
#include <soc/mediatek/smi.h>
#ifdef IF_ZERO
#include "mtk_iommu_ext.h"
#include "pseudo_m4u.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define MTK_INVALID_ION_FD (-1)
#define MTK_INVALID_FENCE_FD (-1)

#define MTK_KERNEL_NO_ION_FD (-99)

struct fb_overlay_buffer_t {
	/* Input */
	int layer_id;
	unsigned int layer_en;
	unsigned int cache_sync;
	/* Output */
	unsigned int index;
	int fence_fd;
};

enum BUFFER_STATE { create, insert, reg_configed, reg_updated, read_done };

enum MTK_TIMELINE_ENUM {
	MTK_TIMELINE_OUTPUT_TIMELINE_ID = MTK_PLANE_INPUT_LAYER_COUNT,
	MTK_TIMELINE_PRIMARY_PRESENT_TIMELINE_ID,
	MTK_TIMELINE_OUTPUT_INTERFACE_TIMELINE_ID,
	MTK_TIMELINE_SECONDARY_PRESENT_TIMELINE_ID,
	MTK_TIMELINE_SF_PRIMARY_PRESENT_TIMELINE_ID,
	MTK_TIMELINE_SF_SECONDARY_PRESENT_TIMELINE_ID,
	MTK_TIMELINE_COUNT,
};

struct mtk_fence_buf_info {
	struct list_head list;
	unsigned int idx;
	int fence;
	struct dma_buf *buf_hnd;
	unsigned long mva;
	unsigned int size;
	unsigned int mva_offset;
	enum BUFFER_STATE buf_state;
	unsigned int set_input_ticket;
	unsigned int trigger_ticket; /* we can't update trigger_ticket_end,*/
	/*because can't gurantee ticket being updated before cmdq callback*/

	unsigned int release_ticket;
	unsigned int enable;
	unsigned long long ts_create;
	unsigned long long ts_period_keep;
	unsigned int seq;
	unsigned int layer_type;
};

struct mtk_fence_sync_info {
	unsigned int inited;
	struct mutex mutex_lock;
	unsigned int layer_id;
	unsigned int fence_idx;
	unsigned int timeline_idx;
	unsigned int inc;
	unsigned int cur_idx;
	struct sw_sync_timeline *timeline;
	struct list_head buf_list;
};

/* use another struct to avoid fence dependency with ddp_ovl.h */
struct FENCE_LAYER_INFO {
	unsigned int layer;
	unsigned int layer_en;
	unsigned int fmt;
	unsigned long addr;
	unsigned long vaddr;
	unsigned int src_x;
	unsigned int src_y;
	unsigned int src_w;
	unsigned int src_h;
	unsigned int src_pitch;
	unsigned int dst_x;
	unsigned int dst_y;
	unsigned int dst_w;
	unsigned int dst_h; /* clip region */
	unsigned int keyEn;
	unsigned int key;
	unsigned int aen;
	unsigned char alpha;

	unsigned int isDirty;

	unsigned int buff_idx;
	unsigned int security;
};

struct mtk_fence_info {
	unsigned int inited;
	struct mutex sync_lock;
	unsigned int layer_id;
	unsigned int fence_idx;
	unsigned int timeline_idx;
	unsigned int fence_fd;
	unsigned int inc;
	unsigned int cur_idx;
	struct sync_timeline *timeline;
	struct list_head buf_list;
	struct FENCE_LAYER_INFO cached_config;
};

struct mtk_fence_session_sync_info {
	unsigned int session_id;
	struct mtk_fence_info session_layer_info[MTK_TIMELINE_COUNT];
};

char *mtk_fence_session_mode_spy(unsigned int session_id);

void mtk_init_fence(void);
unsigned int mtk_query_frm_seq_by_addr(unsigned int session_id,
				       unsigned int layer_id,
				       unsigned long phy_addr);
bool mtk_update_buf_info(unsigned int session_id, unsigned int layer_id,
			 unsigned int idx, unsigned int mva_offset,
			 unsigned int seq);
struct mtk_fence_buf_info *mtk_init_buf_info(struct mtk_fence_buf_info *buf);
void mtk_release_fence(unsigned int session_id, unsigned int layer_id,
		       int fence);
void mtk_release_layer_fence(unsigned int session_id, unsigned int layer_id);
int mtk_release_present_fence(unsigned int session_id, unsigned int fence_idx, ktime_t time);
int mtk_release_sf_present_fence(unsigned int session_id,
				 unsigned int fence_idx);
int mtk_fence_get_output_timeline_id(void);
int mtk_fence_get_interface_timeline_id(void);

struct mtk_fence_buf_info *
mtk_fence_prepare_buf(struct drm_device *dev, struct drm_mtk_gem_submit *buf);
int mtk_fence_init(void);
int mtk_fence_get_cached_layer_info(unsigned int session_id,
				    unsigned int timeline_idx,
				    unsigned int *layer_en, unsigned long *addr,
				    unsigned int *fence_idx);
int mtk_fence_put_cached_layer_info(unsigned int session_id,
				    unsigned int timeline_idx,
				    struct mtk_plane_input_config *src,
				    unsigned long mva);

int mtk_fence_convert_input_to_fence_layer_info(
	struct mtk_plane_input_config *src, struct FENCE_LAYER_INFO *dst,
	unsigned long dst_mva);
unsigned int mtk_fence_query_buf_info(unsigned int session_id,
				      unsigned int timeline_id,
				      unsigned int idx, unsigned long *mva,
				      unsigned int *size, void **va,
				      int need_sync);

int mtk_fence_get_ovl_timeline_id(int layer_id);
int mtk_fence_get_present_timeline_id(unsigned int session_id);
int mtk_fence_get_sf_present_timeline_id(unsigned int session_id);
struct mtk_fence_session_sync_info *
disp_get_session_sync_info(unsigned int session_id);

void mtk_release_session_fence(unsigned int session_id);
struct mtk_fence_info *mtk_fence_get_layer_info(unsigned int session_id,
						unsigned int timeline_id);

#ifdef __cplusplus
} /* extern C */
#endif
#endif
