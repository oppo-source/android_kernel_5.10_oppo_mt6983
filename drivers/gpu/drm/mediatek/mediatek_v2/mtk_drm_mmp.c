// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <drm/drm_crtc.h>
#include <drm/drm_fourcc.h>
#include <linux/dma-buf.h>
#include "mtk_drm_drv.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_fb.h"
#include "mtk_log.h"

#define CRTC_NUM		3
#define DISP_REG_OVL_L0_PITCH (0x044UL)
#define L_PITCH_FLD_SRC_PITCH REG_FLD_MSB_LSB(15, 0)

static struct DRM_MMP_Events g_DRM_MMP_Events;
static struct CRTC_MMP_Events g_CRTC_MMP_Events[MMP_CRTC_NUM];

/* need to update if add new mmp_event in DRM_MMP_Events */
void init_drm_mmp_event(void)
{
	int i;

	if (g_DRM_MMP_Events.drm)
		return;

	g_DRM_MMP_Events.drm = mmprofile_register_event(MMP_ROOT_EVENT, "DRM");

	/* init DRM mmp events */
	g_DRM_MMP_Events.IRQ =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "IRQ");
	g_DRM_MMP_Events.ovl =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "OVL");
	g_DRM_MMP_Events.ovl0 =
		mmprofile_register_event(g_DRM_MMP_Events.ovl, "OVL0");
	g_DRM_MMP_Events.ovl1 =
		mmprofile_register_event(g_DRM_MMP_Events.ovl, "OVL1");
	g_DRM_MMP_Events.ovl0_2l =
		mmprofile_register_event(g_DRM_MMP_Events.ovl, "OVL0_2L");
	g_DRM_MMP_Events.ovl1_2l =
		mmprofile_register_event(g_DRM_MMP_Events.ovl, "OVL1_2L");
	g_DRM_MMP_Events.ovl2_2l =
		mmprofile_register_event(g_DRM_MMP_Events.ovl, "OVL2_2L");
	g_DRM_MMP_Events.ovl3_2l =
		mmprofile_register_event(g_DRM_MMP_Events.ovl, "OVL3_2L");
	g_DRM_MMP_Events.rdma =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "RDMA");
	g_DRM_MMP_Events.rdma0 =
		mmprofile_register_event(g_DRM_MMP_Events.rdma, "RDMA0");
	g_DRM_MMP_Events.rdma1 =
		mmprofile_register_event(g_DRM_MMP_Events.rdma, "RDMA1");
	g_DRM_MMP_Events.rdma2 =
		mmprofile_register_event(g_DRM_MMP_Events.rdma, "RDMA2");
	g_DRM_MMP_Events.rdma3 =
		mmprofile_register_event(g_DRM_MMP_Events.rdma, "RDMA3");
	g_DRM_MMP_Events.rdma4 =
		mmprofile_register_event(g_DRM_MMP_Events.rdma, "RDMA4");
	g_DRM_MMP_Events.rdma5 =
		mmprofile_register_event(g_DRM_MMP_Events.rdma, "RDMA5");
	g_DRM_MMP_Events.wdma =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "WDMA");
	g_DRM_MMP_Events.wdma0 =
		mmprofile_register_event(g_DRM_MMP_Events.wdma, "WDMA0");
	g_DRM_MMP_Events.dsi =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "DSI");
	g_DRM_MMP_Events.dsi0 =
		mmprofile_register_event(g_DRM_MMP_Events.dsi, "DSI0");
	g_DRM_MMP_Events.dsi1 =
		mmprofile_register_event(g_DRM_MMP_Events.dsi, "DSI1");
	g_DRM_MMP_Events.aal =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "AAL");
	g_DRM_MMP_Events.aal0 =
		mmprofile_register_event(g_DRM_MMP_Events.aal, "AAL0");
	g_DRM_MMP_Events.aal1 =
		mmprofile_register_event(g_DRM_MMP_Events.aal, "AAL1");
	g_DRM_MMP_Events.mmclk =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "MMCLK");
	g_DRM_MMP_Events.pmqos =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "PMQOS");
	g_DRM_MMP_Events.hrt_bw =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "HRT_BW");
	g_DRM_MMP_Events.layering =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "HRT");
	g_DRM_MMP_Events.layering_blob =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "HRT_BLOB");
	g_DRM_MMP_Events.mutex_lock =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "LOCK");
	g_DRM_MMP_Events.dma_alloc =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "D_ALLOC");
	g_DRM_MMP_Events.dma_free =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "D_FREE");
	g_DRM_MMP_Events.dma_get =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "D_GET");
	g_DRM_MMP_Events.dma_put =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "D_PUT");
	g_DRM_MMP_Events.ion_import_dma =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "I_DMA");
	g_DRM_MMP_Events.ion_import_fd =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "I_FD");
	g_DRM_MMP_Events.ion_import_free =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "I_FREE");
	g_DRM_MMP_Events.set_mode =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "SET_MODE");
	g_DRM_MMP_Events.top_clk =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "TOP_CLK");
	g_DRM_MMP_Events.ddp =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "MUTEX");
	g_DRM_MMP_Events.sram_alloc =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "S_ALLOC");
	g_DRM_MMP_Events.sram_free =
		mmprofile_register_event(g_DRM_MMP_Events.drm, "S_FREE");
	for (i = 0; i < DISP_MUTEX_DDP_COUNT; i++) {
		char name[32];

		snprintf(name, sizeof(name), "MUTEX%d", i);
		g_DRM_MMP_Events.mutex[i] =
			mmprofile_register_event(g_DRM_MMP_Events.ddp, name);
	}

	g_DRM_MMP_Events.postmask =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "POSTMASK");
	g_DRM_MMP_Events.postmask0 = mmprofile_register_event(
		g_DRM_MMP_Events.postmask, "POSTMASK0");
	g_DRM_MMP_Events.abnormal_irq =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "ABNORMAL_IRQ");
	g_DRM_MMP_Events.iova_tf =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "IOVA_TF");
	g_DRM_MMP_Events.dp_intf0 =
		mmprofile_register_event(g_DRM_MMP_Events.IRQ, "dp_intf0");
}

/* need to update if add new mmp_event in CRTC_MMP_Events */
void init_crtc_mmp_event(void)
{
	int i = 0;
	int r = 0;

	for (i = 0; i < MMP_CRTC_NUM; i++) {
		char name[32];
		mmp_event crtc_mmp_root;

		/* create i th root of CRTC mmp events */
		r = snprintf(name, sizeof(name), "crtc%d", i);
		if (r < 0) {
			/* Handle snprintf() error */
			DDPPR_ERR("%s:snprintf error\n", __func__);
			return;
		}
		crtc_mmp_root =
			mmprofile_register_event(g_DRM_MMP_Events.drm, name);
		g_DRM_MMP_Events.crtc[i] = crtc_mmp_root;

		/* init CRTC mmp events */
		g_CRTC_MMP_Events[i].trig_loop_done = mmprofile_register_event(
			crtc_mmp_root, "trig_loop_done");
		g_CRTC_MMP_Events[i].enable =
			mmprofile_register_event(crtc_mmp_root, "enable");
		g_CRTC_MMP_Events[i].disable =
			mmprofile_register_event(crtc_mmp_root, "disable");
		g_CRTC_MMP_Events[i].release_fence = mmprofile_register_event(
			crtc_mmp_root, "release_fence");
		g_CRTC_MMP_Events[i].update_present_fence =
			mmprofile_register_event(crtc_mmp_root,
				"update_present_fence");
		g_CRTC_MMP_Events[i].release_present_fence =
			mmprofile_register_event(crtc_mmp_root,
				"release_present_fence");
		g_CRTC_MMP_Events[i].present_fence_timestamp_same =
			mmprofile_register_event(crtc_mmp_root,
				"present_fence_timestamp_same");
		g_CRTC_MMP_Events[i].update_sf_present_fence =
			mmprofile_register_event(crtc_mmp_root,
				"update_sf_present_fence");
		g_CRTC_MMP_Events[i].release_sf_present_fence =
			mmprofile_register_event(crtc_mmp_root,
				"release_sf_present_fence");
		g_CRTC_MMP_Events[i].warn_sf_pf_0 =
			mmprofile_register_event(crtc_mmp_root, "warn_sf_pf_0");
		g_CRTC_MMP_Events[i].warn_sf_pf_2 =
			mmprofile_register_event(crtc_mmp_root, "warn_sf_pf_2");
		g_CRTC_MMP_Events[i].atomic_begin = mmprofile_register_event(
			crtc_mmp_root, "atomic_begin");
		g_CRTC_MMP_Events[i].atomic_flush = mmprofile_register_event(
			crtc_mmp_root, "atomic_flush");
		g_CRTC_MMP_Events[i].enable_vblank = mmprofile_register_event(
			crtc_mmp_root, "enable_vblank");
		g_CRTC_MMP_Events[i].disable_vblank = mmprofile_register_event(
			crtc_mmp_root, "disable_vblank");
		g_CRTC_MMP_Events[i].esd_check =
			mmprofile_register_event(crtc_mmp_root, "ESD check");
		g_CRTC_MMP_Events[i].esd_recovery =
			mmprofile_register_event(crtc_mmp_root, "ESD recovery");
		g_CRTC_MMP_Events[i].leave_idle = mmprofile_register_event(
			crtc_mmp_root, "leave_idle");
		g_CRTC_MMP_Events[i].enter_idle = mmprofile_register_event(
			crtc_mmp_root, "enter_idle");
		g_CRTC_MMP_Events[i].frame_cfg =
			mmprofile_register_event(crtc_mmp_root, "frame cfg");
		g_CRTC_MMP_Events[i].suspend = mmprofile_register_event(
			crtc_mmp_root, "suspend");
		g_CRTC_MMP_Events[i].resume = mmprofile_register_event(
			crtc_mmp_root, "resume");
		g_CRTC_MMP_Events[i].dsi_suspend = mmprofile_register_event(
			crtc_mmp_root, "dsi_suspend");
		g_CRTC_MMP_Events[i].dsi_resume = mmprofile_register_event(
			crtc_mmp_root, "dsi_resume");
		g_CRTC_MMP_Events[i].backlight = mmprofile_register_event(
			crtc_mmp_root, "backlight");
		g_CRTC_MMP_Events[i].backlight_grp = mmprofile_register_event(
			crtc_mmp_root, "backlight_grp");
		g_CRTC_MMP_Events[i].ddic_send_cmd = mmprofile_register_event(
			crtc_mmp_root, "ddic_send_cmd");
		g_CRTC_MMP_Events[i].ddic_read_cmd = mmprofile_register_event(
			crtc_mmp_root, "ddic_read_cmd");
		g_CRTC_MMP_Events[i].path_switch = mmprofile_register_event(
			crtc_mmp_root, "path_switch");
		g_CRTC_MMP_Events[i].user_cmd = mmprofile_register_event(
			crtc_mmp_root, "user_cmd");
		g_CRTC_MMP_Events[i].check_trigger = mmprofile_register_event(
			crtc_mmp_root, "check_trigger");
		g_CRTC_MMP_Events[i].kick_trigger = mmprofile_register_event(
			crtc_mmp_root, "kick_trigger");
		g_CRTC_MMP_Events[i].atomic_commit = mmprofile_register_event(
			crtc_mmp_root, "atomic_commit");
		g_CRTC_MMP_Events[i].user_cmd_cb =
			mmprofile_register_event(crtc_mmp_root, "user_cmd_cb");
		g_CRTC_MMP_Events[i].bl_cb =
			mmprofile_register_event(crtc_mmp_root, "bl_cb");
		g_CRTC_MMP_Events[i].clk_change = mmprofile_register_event(
			crtc_mmp_root, "clk_change");
		g_CRTC_MMP_Events[i].layerBmpDump =
					mmprofile_register_event(
					crtc_mmp_root, "LayerBmpDump");
		g_CRTC_MMP_Events[i].layer_dump[0] =
					mmprofile_register_event(
					g_CRTC_MMP_Events[i].layerBmpDump,
					"layer0_dump");
		g_CRTC_MMP_Events[i].layer_dump[1] =
					mmprofile_register_event(
					g_CRTC_MMP_Events[i].layerBmpDump,
					"layer1_dump");
		g_CRTC_MMP_Events[i].layer_dump[2] =
					mmprofile_register_event(
					g_CRTC_MMP_Events[i].layerBmpDump,
					"layer2_dump");
		g_CRTC_MMP_Events[i].layer_dump[3] =
					mmprofile_register_event(
					g_CRTC_MMP_Events[i].layerBmpDump,
					"layer3_dump");
		g_CRTC_MMP_Events[i].layer_dump[4] =
					mmprofile_register_event(
					g_CRTC_MMP_Events[i].layerBmpDump,
					"layer4_dump");
		g_CRTC_MMP_Events[i].layer_dump[5] =
					mmprofile_register_event(
					g_CRTC_MMP_Events[i].layerBmpDump,
					"layer5_dump");
		g_CRTC_MMP_Events[i].wbBmpDump =
			mmprofile_register_event(crtc_mmp_root, "wbBmpDump");
		g_CRTC_MMP_Events[i].wb_dump =
			mmprofile_register_event(g_CRTC_MMP_Events[i].wbBmpDump, "wb_dump");
		g_CRTC_MMP_Events[i].cwbBmpDump =
					mmprofile_register_event(
					crtc_mmp_root, "CwbBmpDump");
		g_CRTC_MMP_Events[i].cwb_dump =
					mmprofile_register_event(
					g_CRTC_MMP_Events[i].cwbBmpDump,
					"cwb_dump");
		/*Msync 2.0 mmp start*/
		g_CRTC_MMP_Events[i].ovl_status_err =
			mmprofile_register_event(crtc_mmp_root, "ovl_status_err");
		g_CRTC_MMP_Events[i].vfp_period =
			mmprofile_register_event(crtc_mmp_root, "vfp_period");
		g_CRTC_MMP_Events[i].not_vfp_period =
			mmprofile_register_event(crtc_mmp_root, "not_vfp_period");
		g_CRTC_MMP_Events[i].dsi_state_dbg7 =
			mmprofile_register_event(crtc_mmp_root, "dsi_state_dbg7");
		g_CRTC_MMP_Events[i].dsi_dbg7_after_sof =
			mmprofile_register_event(crtc_mmp_root, "dsi_dbg7_after_sof");
		g_CRTC_MMP_Events[i].msync_enable =
			mmprofile_register_event(crtc_mmp_root, "msync_enable");
		/*Msync 2.0 mmp end*/
		g_CRTC_MMP_Events[i].mode_switch = mmprofile_register_event(
			crtc_mmp_root, "mode_switch");
		g_CRTC_MMP_Events[i].ddp_clk = mmprofile_register_event(
			crtc_mmp_root, "ddp_clk");
		/*AAL MMP MARK*/
		g_CRTC_MMP_Events[i].aal_sof_thread = mmprofile_register_event(
			crtc_mmp_root, "aal_sof_thread");
		g_CRTC_MMP_Events[i].aal_dre30_rw = mmprofile_register_event(
			crtc_mmp_root, "aal_dre30_rw");
		g_CRTC_MMP_Events[i].aal_dre20_rh = mmprofile_register_event(
			crtc_mmp_root, "aal_dre20_rh");
		g_CRTC_MMP_Events[i].max_hrt_layers = mmprofile_register_event(
			crtc_mmp_root, "max_hrt_layers");
		/*Gamma MMP MARK*/
		g_CRTC_MMP_Events[i].gamma_ioctl = mmprofile_register_event(
			crtc_mmp_root, "gamma_ioctl");
		g_CRTC_MMP_Events[i].gamma_sof = mmprofile_register_event(
			crtc_mmp_root, "gamma_sof");
	}
}
void drm_mmp_init(void)
{
	DDPMSG("%s\n", __func__);

	mmprofile_enable(1);

	/* init mmp events */
	init_drm_mmp_event();
	init_crtc_mmp_event();

	/* enable all mmp events */
	mmprofile_enable_event_recursive(g_DRM_MMP_Events.drm, 1);

	mmprofile_start(1);
}

struct DRM_MMP_Events *get_drm_mmp_events(void)
{
	return &g_DRM_MMP_Events;
}

struct CRTC_MMP_Events *get_crtc_mmp_events(unsigned long id)
{
	return &g_CRTC_MMP_Events[id];
}

//#include <mtk_iommu_ext.h>
#include <mtk_drm_drv.h>

#define DISP_PAGE_MASK 0xfffL

int mtk_drm_check_fmt(unsigned int fmt, struct mmp_metadata_bitmap_t *bitmap)
{
	switch (fmt) {
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
		bitmap->format = MMPROFILE_BITMAP_RGB565;
		bitmap->bpp = 16;
		return 0;
	case DRM_FORMAT_RGB888:
	case DRM_FORMAT_BGR888:
	case DRM_FORMAT_C8:
		bitmap->format = MMPROFILE_BITMAP_RGB888;
		bitmap->bpp = 24;
		return 0;
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		bitmap->format = MMPROFILE_BITMAP_BGRA8888;
		bitmap->bpp = 32;
		return 0;
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		bitmap->format = MMPROFILE_BITMAP_RGBA8888;
		bitmap->bpp = 32;
		return 0;
	case DRM_FORMAT_YUYV:
		bitmap->format = MMPROFILE_BITMAP_RGB888;
		bitmap->bpp = 16;
		bitmap->data2 = MMPROFILE_BITMAP_YUYV;
		return 1;
	case DRM_FORMAT_YVYU:
		bitmap->format = MMPROFILE_BITMAP_RGB888;
		bitmap->bpp = 16;
		bitmap->data2 = MMPROFILE_BITMAP_YVYU;
		return 1;
	case DRM_FORMAT_UYVY:
		bitmap->format = MMPROFILE_BITMAP_RGB888;
		bitmap->bpp = 16;
		bitmap->data2 = MMPROFILE_BITMAP_UYVY;
		return 1;
	case DRM_FORMAT_VYUY:
		bitmap->format = MMPROFILE_BITMAP_RGB888;
		bitmap->bpp = 16;
		bitmap->data2 = MMPROFILE_BITMAP_VYUY;
		return 1;
	default:
		DDPINFO("[MMP]unknown fmt\n");
		return -1;
	}
}

int crtc_mva_map_kernel(unsigned int mva, unsigned int size,
			unsigned long *map_va, unsigned int *map_size)
{
#ifdef IF_ZERO
	struct disp_iommu_device *disp_dev = disp_get_iommu_dev();

	if ((disp_dev != NULL) && (disp_dev->iommu_pdev != NULL) && (mva != 0))
		mtk_iommu_iova_to_va(&(disp_dev->iommu_pdev->dev),
				     mva, map_va, size);
	else
		DDPINFO("%s, %d, disp_dev is null\n", __func__, __LINE__);
#endif

	return 0;
}

int crtc_mva_unmap_kernel(unsigned int mva, unsigned int size,
			  unsigned long map_va)
{
	vunmap((void *)(map_va & (~DISP_PAGE_MASK)));

	return 0;
}

void *mtk_drm_buffer_map_kernel(struct drm_framebuffer *fb)
{
	struct drm_gem_object *gem_obj = NULL;
	struct dma_buf *dmabuf = NULL;
	void *dma_va;

	if (!fb) {
		DDPINFO("[MMP]fb is null\n", __func__);
		return 0;
	}

	gem_obj = mtk_fb_get_gem_obj(fb);
	if (!gem_obj) {
		DDPINFO("[MMP]gem is null\n", __func__);
		return 0;
	}

	dmabuf = gem_obj->import_attach->dmabuf;
	if (!dmabuf) {
		DDPINFO("[MMP]dmabuf is null\n", __func__);
		return 0;
	}

	dma_va = dma_buf_vmap(dmabuf);

	return dma_va;
}

int mtk_drm_buffer_unmap_kernel(struct drm_framebuffer *fb, void *dma_va)
{
	struct drm_gem_object *gem_obj;
	struct dma_buf *dmabuf;

	gem_obj = mtk_fb_get_gem_obj(fb);
	dmabuf = gem_obj->import_attach->dmabuf;

	dma_buf_vunmap(dmabuf, dma_va);

	return 0;
}

int mtk_drm_mmp_ovl_layer(struct mtk_plane_state *state,
			  u32 downSampleX, u32 downSampleY, int global_lye_num)
{
	struct mtk_plane_pending_state *pending = &state->pending;
	struct drm_crtc *crtc = state->crtc;
	struct mtk_drm_private *private = crtc->dev->dev_private;
	unsigned int crtc_idx = drm_crtc_index(crtc);
	struct mmp_metadata_bitmap_t bitmap;
	struct mmp_metadata_t meta = {0};
	unsigned int fmt = pending->format;
	int raw = 0;
	int yuv = 0;
	void *dma_va;

	if (crtc_idx >= CRTC_NUM)
		return -1;

	if (!mtk_drm_helper_get_opt(private->helper_opt,
				MTK_DRM_OPT_USE_M4U)) {
		DDPPR_ERR("[MMP]display iommu is disabled\n");
		return -1;
	}

	if (!pending->enable) {
		DDPINFO("[MMP]layer is not disable\n");
		return -1;
	}

	if (pending->prop_val[PLANE_PROP_COMPRESS]) {
		DDPINFO("[MMP]layer is compress\n");
		return -1;
	}

	if (global_lye_num > 5) {
		DDPINFO("[MMP]not support layer over 6\n");
		return -1;
	}

	memset(&bitmap, 0, sizeof(struct mmp_metadata_bitmap_t));
	bitmap.data1 = 0;
	bitmap.width = pending->width;
	bitmap.height = pending->height;

	if (fmt == DRM_FORMAT_RGB565 || fmt == DRM_FORMAT_BGR565) {
		bitmap.format = MMPROFILE_BITMAP_RGB565;
		bitmap.bpp = 16;
	} else if (fmt == DRM_FORMAT_RGB888 || fmt == DRM_FORMAT_BGR888 ||
		   fmt == DRM_FORMAT_C8) {
		bitmap.format = MMPROFILE_BITMAP_RGB888;
		bitmap.bpp = 24;
	} else if (fmt == DRM_FORMAT_BGRA8888 || fmt == DRM_FORMAT_BGRX8888) {
		bitmap.format = MMPROFILE_BITMAP_BGRA8888;
		bitmap.bpp = 32;
	} else if (fmt == DRM_FORMAT_RGBA8888 ||
		   fmt == DRM_FORMAT_RGBX8888 ||
		   fmt == DRM_FORMAT_XRGB8888 ||
		   fmt == DRM_FORMAT_ARGB8888 ||
		   fmt == DRM_FORMAT_XBGR8888 ||
		   fmt == DRM_FORMAT_ABGR8888 ||
		   fmt == DRM_FORMAT_ABGR2101010 ||
		   fmt == DRM_FORMAT_ABGR16161616F) {
		bitmap.format = MMPROFILE_BITMAP_RGBA8888;
		bitmap.bpp = 32;
	} else if (fmt == DRM_FORMAT_BGRA8888 ||
		   fmt == DRM_FORMAT_BGRX8888){
		bitmap.format = MMPROFILE_BITMAP_BGRA8888;
		bitmap.bpp = 32;
	} else if (fmt == DRM_FORMAT_YUYV) {
		bitmap.format = MMPROFILE_BITMAP_RGB888;
		bitmap.bpp = 16;
		bitmap.data2 = MMPROFILE_BITMAP_YUYV;
		yuv = 1;
	} else if (fmt == DRM_FORMAT_YVYU) {
		bitmap.format = MMPROFILE_BITMAP_RGB888;
		bitmap.bpp = 16;
		bitmap.data2 = MMPROFILE_BITMAP_YVYU;
		yuv = 1;
	} else if (fmt == DRM_FORMAT_UYVY) {
		bitmap.format = MMPROFILE_BITMAP_RGB888;
		bitmap.bpp = 16;
		bitmap.data2 = MMPROFILE_BITMAP_UYVY;
		yuv = 1;
	} else if (fmt == DRM_FORMAT_VYUY) {
		bitmap.format = MMPROFILE_BITMAP_RGB888;
		bitmap.bpp = 16;
		bitmap.data2 = MMPROFILE_BITMAP_VYUY;
		yuv = 1;
	} else {
		DDPINFO("[MMP]unknown fmt\n");
		raw = 1;
	}

	CRTC_MMP_EVENT_START(crtc_idx, layerBmpDump,
			     global_lye_num, pending->enable);
	if (!raw) {
		mmp_event *event_base = NULL;
		bitmap.data1 = state->comp_state.comp_id;
		bitmap.pitch = pending->pitch;
		bitmap.start_pos = 0;
		bitmap.data_size = bitmap.pitch * bitmap.height;
		bitmap.down_sample_x = downSampleX;
		bitmap.down_sample_y = downSampleY;

		dma_va = mtk_drm_buffer_map_kernel(state->base.fb);
		if (!dma_va) {
			DDPINFO("[MMP]dma_va is null\n", __func__);
			goto end;
		}
		bitmap.p_data = dma_va;

		event_base = g_CRTC_MMP_Events[crtc_idx].layer_dump;
		if (event_base) {
			if (!yuv)
				mmprofile_log_meta_bitmap(
				event_base[global_lye_num],
				MMPROFILE_FLAG_PULSE,
				&bitmap);
			else
				mmprofile_log_meta_yuv_bitmap(
				event_base[global_lye_num],
				MMPROFILE_FLAG_PULSE,
				&bitmap);
		}
		mtk_drm_buffer_unmap_kernel(state->base.fb, dma_va);
	} else {
		mmp_event *event_base = NULL;

		meta.data_type = MMPROFILE_META_RAW;
		meta.size = pending->pitch * pending->height;
		if (crtc_mva_map_kernel(pending->addr, bitmap.data_size,
					(unsigned long *)&meta.p_data,
					&meta.size) != 0) {
			DDPINFO("%s,fail to dump rgb\n", __func__);
			goto end;
		}

		event_base = g_CRTC_MMP_Events[crtc_idx].layer_dump;
		if (event_base)
			mmprofile_log_meta(
			event_base[state->comp_state.lye_id],
			MMPROFILE_FLAG_PULSE, &meta);

		crtc_mva_unmap_kernel(pending->addr, meta.size,
				(unsigned long)meta.p_data);
	}

end:
	CRTC_MMP_EVENT_END(crtc_idx, layerBmpDump,
			   pending->addr, pending->format);

	return 0;
}

int mtk_drm_mmp_wdma_buffer(struct drm_crtc *crtc,
	struct drm_framebuffer *wb_fb, u32 downSampleX, u32 downSampleY)
{
	unsigned int crtc_idx = drm_crtc_index(crtc);
	struct mmp_metadata_bitmap_t bitmap;
	struct mmp_metadata_t meta;
	unsigned int fmt = wb_fb->format->format;
	int ret, raw = 0, yuv = 0;
	void *dma_va;

	memset(&bitmap, 0, sizeof(struct mmp_metadata_bitmap_t));

	ret = mtk_drm_check_fmt(fmt, &bitmap);
	if (ret == 1)
		yuv = 1;
	else if (ret == -1)
		raw = 1;

	CRTC_MMP_EVENT_START(crtc_idx, wbBmpDump, 0, 0);
	dma_va = mtk_drm_buffer_map_kernel(wb_fb);
	if (!dma_va) {
		DDPINFO("[MMP]dma_va is null\n", __func__);
		goto end;
	}
	bitmap.p_data = dma_va;
	bitmap.width = wb_fb->width;
	bitmap.height = wb_fb->height;
	bitmap.pitch = wb_fb->pitches[0];

	if (!raw) {
		bitmap.data_size = bitmap.pitch * bitmap.height;
		bitmap.down_sample_x = downSampleX;
		bitmap.down_sample_y = downSampleY;
		bitmap.data1 = wb_fb->height << 16 | wb_fb->width;
		bitmap.data2 = downSampleY << 16 | downSampleX;

		if (!yuv)
			CRTC_MMP_BITMAP_MARK(crtc_idx, wb_dump, &bitmap);
		else
			CRTC_MMP_YUV_BITMAP_MARK(crtc_idx, wb_dump, &bitmap);

	} else {
		memset(&meta, 0, sizeof(struct mmp_metadata_t));
		meta.data_type = MMPROFILE_META_RAW;
		meta.size = bitmap.pitch * bitmap.height;
		meta.p_data = bitmap.p_data;

		CRTC_MMP_META_MARK(crtc_idx, wb_dump, &meta);
	}
	mtk_drm_buffer_unmap_kernel(wb_fb, dma_va);

end:
	CRTC_MMP_EVENT_END(crtc_idx, wbBmpDump, 0, 0);

	return 0;
}

int mtk_drm_mmp_cwb_buffer(struct drm_crtc *crtc,
		struct mtk_cwb_info *cwb_info,
		void *buffer, unsigned int buf_idx)
{
	unsigned int crtc_idx = drm_crtc_index(crtc);
	enum CWB_BUFFER_TYPE type = cwb_info->type;
	struct mmp_metadata_bitmap_t bitmap;
	mmp_event event_base = 0;

	if (crtc_idx >= CRTC_NUM)
		return -EINVAL;

	memset(&bitmap, 0, sizeof(struct mmp_metadata_bitmap_t));
	bitmap.data1 = buf_idx;
	bitmap.width = cwb_info->copy_w;
	bitmap.height = cwb_info->copy_h;

	bitmap.format = MMPROFILE_BITMAP_RGB888;
	bitmap.bpp = 24;

	CRTC_MMP_EVENT_START(crtc_idx, cwbBmpDump,
			     0, 0);

	bitmap.pitch = bitmap.width * 3;
	bitmap.start_pos = 0;
	bitmap.data_size = bitmap.pitch * bitmap.height;
	bitmap.down_sample_x = 1;
	bitmap.down_sample_y = 1;
	if (type == IMAGE_ONLY) {
		bitmap.p_data = (void *)buffer;
	} else if (type == CARRY_METADATA) {
		struct user_cwb_buffer *tmp = (struct user_cwb_buffer *)buffer;

		bitmap.p_data = (void *)tmp->data.image;
	}

	event_base = g_CRTC_MMP_Events[crtc_idx].cwb_dump;
	if (event_base)
		mmprofile_log_meta_bitmap(
			event_base,
			MMPROFILE_FLAG_PULSE,
			&bitmap);

	CRTC_MMP_EVENT_END(crtc_idx, cwbBmpDump,
			   0, 0);
	return 0;
}
