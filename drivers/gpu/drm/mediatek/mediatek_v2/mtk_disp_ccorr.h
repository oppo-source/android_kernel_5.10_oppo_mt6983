/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __MTK_DISP_CCORR_H__
#define __MTK_DISP_CCORR_H__

#include <drm/mediatek_drm.h>

/*#ifdef OPLUS_BUG_STABILITY*/
#define OPLUS_SILKY_ON_START_FRAME
#include "../../oplus/oplus_drm_disp_panel.h"
/*#endif*/

struct mtk_disp_ccorr_data {
	bool support_shadow;
	bool need_bypass_shadow;
	int single_pipe_ccorr_num;
};

void ccorr_test(const char *cmd, char *debug_output);
int ccorr_interface_for_color(unsigned int ccorr_idx,
	unsigned int ccorr_coef[3][3], void *handle);
void disp_ccorr_on_end_of_frame(struct mtk_ddp_comp *comp);
void disp_pq_notify_backlight_changed(int bl_1024);
int disp_ccorr_set_color_matrix(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, int32_t matrix[16], int32_t hint, bool fte_flag, bool linear);
int disp_ccorr_set_RGB_Gain(struct mtk_ddp_comp *comp,
	struct cmdq_pkt *handle, int r, int g, int b);
int mtk_drm_ioctl_set_ccorr(struct drm_device *dev, void *data,
	struct drm_file *file_priv);
int mtk_drm_ioctl_ccorr_eventctl(struct drm_device *dev, void *data,
	struct drm_file *file_priv);
int mtk_drm_ioctl_ccorr_get_irq(struct drm_device *dev, void *data,
	struct drm_file *file_priv);
int mtk_drm_ioctl_support_color_matrix(struct drm_device *dev, void *data,
	struct drm_file *file_priv);
int mtk_drm_ioctl_aibld_cv_mode(struct drm_device *dev, void *data,
		struct drm_file *file_priv);
int mtk_get_ccorr_caps(struct drm_mtk_ccorr_caps *ccorr_caps);
int mtk_set_ccorr_caps(struct drm_mtk_ccorr_caps *ccorr_caps);
#ifdef OPLUS_SILKY_ON_START_FRAME
void disp_ccorr_on_start_of_frame(void);
#endif //OPLUS_SILKY_ON_START_FRAME

#endif

