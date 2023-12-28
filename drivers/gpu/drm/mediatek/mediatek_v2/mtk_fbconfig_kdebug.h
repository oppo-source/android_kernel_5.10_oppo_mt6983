/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __MTK_FBCONFIG_KDEBUG_H
#define __MTK_FBCONFIG_KDEBUG_H

#include <linux/types.h>
#include <drm/drm_mipi_dsi.h>
#include "mtk_drm_ddp_comp.h"



void PanelMaster_probe(void);
void PanelMaster_Init(struct drm_device *dev);
void PanelMaster_Deinit(void);


int fb_config_execute_cmd(void);
int fbconfig_get_esd_check_exec(void);


#define MAX_INSTRUCTION 35
#define NUM_OF_DSI 1

enum RECORD_TYPE {
	RECORD_CMD = 0,
	RECORD_MS = 1,
	RECORD_PIN_SET = 2,
};

enum DSI_INDEX {
	PM_DSI0 = 0,
	PM_DSI1 = 1,
	PM_DSI_MAX = 0XFF,
};


struct CONFIG_RECORD {
	enum RECORD_TYPE type;	/* msleep;cmd;setpin;resetpin. */
	int ins_num;
	int ins_array[MAX_INSTRUCTION];
};

struct CONFIG_RECORD_LIST {
	struct CONFIG_RECORD record;
	struct list_head list;
};

enum MIPI_SETTING_TYPE {
	MIPI_HS_PRPR = 0,
	MIPI_HS_ZERO = 1,
	MIPI_HS_TRAIL = 2,
	MIPI_TA_GO = 3,
	MIPI_TA_SURE = 4,
	MIPI_TA_GET = 5,
	MIPI_DA_HS_EXIT = 6,
	MIPI_CLK_ZERO = 7,
	MIPI_CLK_TRAIL = 8,
	MIPI_CONT_DET = 9,
	MIPI_CLK_HS_PRPR = 10,
	MIPI_CLK_HS_POST = 11,
	MIPI_CLK_HS_EXIT = 12,
	MIPI_HPW = 13,
	MIPI_HFP = 14,
	MIPI_HBP = 15,
	MIPI_VPW = 16,
	MIPI_VFP = 17,
	MIPI_VBP = 18,
	MIPI_LPX = 19,
	MIPI_SSC_EN = 0xFE,
	MIPI_MAX = 0XFF,
};

struct MIPI_TIMING {
	enum MIPI_SETTING_TYPE type;
	unsigned int value;
};

struct SETTING_VALUE {
	enum DSI_INDEX dsi_index;
	unsigned int value[NUM_OF_DSI];
};


struct PM_LAYER_INFO {
	int index;
	int height;
	int width;
	int fmt;
	unsigned int layer_size;
};

struct PM_MMQOS_REL_INFO {
	int vact;
	int hact;
	int vrefresh;
	int vtotal;
	int htotal;
	int data_rate;
	int isCphy;
	int mode;
	int lane_num;
	int compress_ratio;
	int scr_bpp;
	int is_dual_pipe;
	int idle;
	int bdg_rxtx_ratio;
	int dal_enable;
};

struct ESD_PARA {
	int addr;
	int type;
	int para_num;
	char *esd_ret_buffer;
};

#ifdef DRM_CMDQ_DISABLE
//drm_dev define in mtk_fbconfig_kdebug.c, PanelMaster_Init() must be called before crtc_create
extern struct drm_device *drm_dev;
#endif

#ifdef IF_ZERO
struct LAYER_H_SIZE {
	int layer_size;
	int height;
	int fmt;
};
#endif

struct MIPI_CLK_V2 {
	unsigned char div1;
	unsigned char div2;
	unsigned short fbk_div;
};

struct LCM_TYPE_FB {
	int clock;
	int lcm_type;
};

struct DSI_RET {
	int dsi[NUM_OF_DSI];	/* for there are totally 2 dsi. */
};


struct LCM_REG_READ {
	int check_addr;
	int check_para_num;
	int check_type;
	char *check_buffer;
};

struct FBCONFIG_DISP_IF {
	void (*set_cmd_mode)(void);
	int (*set_mipi_clk)(unsigned int clk);
	void (*set_dsi_post)(void);
	void (*set_lane_num)(unsigned int lane_num);
	void (*set_mipi_timing)(struct MIPI_TIMING timing);
	void (*set_te_enable)(char enable);
	void (*set_continuous_clock)(int enable);
	int (*set_spread_frequency)(unsigned int clk);
	int (*set_get_misc)(const char *name, void *parameter);

};

struct misc_property {
	unsigned int dual_port:1;
	unsigned int overall_layer_num:5;
	unsigned int reserved:26;
};

void Panel_Master_DDIC_config(void);

#include <linux/uaccess.h>
#include <linux/compat.h>



#ifdef CONFIG_COMPAT

struct compat_lcm_type_fb {
	compat_int_t clock;
	compat_int_t lcm_type;
};

struct compat_config_record {
	compat_int_t type;	/* msleep;cmd;setpin;resetpin. */
	compat_int_t ins_num;
	compat_int_t ins_array[MAX_INSTRUCTION];
};

struct compat_dsi_ret {
	compat_int_t dsi[NUM_OF_DSI];	/* for there are totally 2 dsi. */
};

struct compat_mipi_timing {
	compat_int_t type;
	compat_uint_t value;
};

/*
 * struct compat_pm_layer_en {
 * compat_int_t layer_en[TOTAL_OVL_LAYER_NUM];
 * };
 */
struct compat_pm_layer_info {
	compat_int_t index;
	compat_int_t height;
	compat_int_t width;
	compat_int_t fmt;
	compat_uint_t layer_size;
};

struct compat_pm_mmqos_rel_info {
	compat_int_t vact;
	compat_int_t hact;
	compat_int_t vrefresh;
	compat_int_t vtotal;
	compat_int_t htotal;
	compat_int_t data_rate;
	compat_int_t isCphy;
	compat_int_t mode;
	compat_int_t lane_num;
	compat_int_t compress_ratio;
	compat_int_t scr_bpp;
	compat_int_t is_dual_pipe;
	compat_int_t idle;
	compat_int_t bdg_rxtx_ratio;
	compat_int_t dal_enable;
};

struct compat_esd_para {
	compat_int_t addr;
	compat_int_t type;
	compat_int_t para_num;
	compat_uint_t esd_ret_buffer;
};

#endif
/* end CONFIG_COMPAT */
int Panel_Master_dsi_config_entry(struct drm_crtc *crtc,
	const char *name, int config_value);
u32 fbconfig_mtk_dsi_get_lanes_num(struct mtk_ddp_comp *comp);
int fbconfig_mtk_dsi_get_mode_type(struct mtk_ddp_comp *comp);
int fbconfig_get_esd_check_test(struct drm_crtc *crtc,
	uint32_t cmd, uint8_t *buffer, uint32_t num);
int fbconfig_mtk_dsi_get_bpp(struct mtk_ddp_comp *comp);

int Panel_Master_lcm_get_dsi_timing_entry(struct drm_crtc *crtc,
	int type);
int Panel_Master_mipi_set_timing_entry(struct drm_crtc *crtc,
	struct MIPI_TIMING timing);
int Panel_Master_mipi_set_cc_entry(struct drm_crtc *crtc,
	int enable);
int Panel_Master_mipi_get_cc_entry(struct drm_crtc *crtc);

#endif
/* __MTK_FBCONFIG_KDEBUG_H */
