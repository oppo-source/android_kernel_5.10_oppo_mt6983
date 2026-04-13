#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include <linux/regulator/consumer.h>
#include <mt-plat/mtk_boot_common.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "../oplus/oplus_display_panel_power.h"
#include "../mediatek/mediatek_v2/mtk_log.h"
#include "../mediatek/mediatek_v2/mtk_dsi.h"
#include "../mediatek/mediatek_v2/mtk_drm_ddp_comp.h"

#include "ac328_p_b_a0029_vdo_panel.h"
#include "oplus_bl.h"
#include "../oplus/oplus_display_onscreenfingerprint.h"
/* add for dips_drv log  */
#include "../oplus/oplus_display_mtk_debug.h"
#include "../oplus/oplus_drm_disp_panel.h"

static u32 flag_hbm = 0;

extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
static int current_fps = 120;
static bool aod_state = false;
static int mode_id = -1;
static unsigned int lhbm_last_backlight = 0;

#define MAX_NORMAL_BRIGHTNESS   3075
#define LCM_BRIGHTNESS_TYPE 2
#define FINGER_HBM_BRIGHTNESS 3730

extern void lcdinfo_notify(unsigned long val, void *v);

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr_enable_gpio;
	struct gpio_desc *vci_enable_gpio;
	struct drm_display_mode *m;
	struct gpio_desc *lcm_esd_err_fg_gpio;
	bool prepared;
	bool enabled;
	int error;
};
static unsigned int temp_seed_mode = 0;
#define lcm_dcs_write_seq(ctx, seq...)				\
	({												\
		const u8 d[] = { seq };						\
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,		\
				 "DCS sequence too big for stack");	\
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));		\
	})

#define lcm_dcs_write_seq_static(ctx, seq...)		\
	({												\
		static const u8 d[] = { seq };				\
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));		\
	})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static void push_table(struct lcm *ctx, struct LCM_setting_table *table, unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			usleep_range(table[i].count*1000, table[i].count*1000 + 100);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 100);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write(ctx, table[i].para_list, table[i].count);
			break;
		}
	}
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		DISP_ERR("ac328_p_b_a0029 %s display_mode m = NULL!\n", __func__);
		return -EINVAL;
	}

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60) {
		ret = FHD_SDC60;
	} else if (m_vrefresh == 90) {
		ret = FHD_SDC90;
	} else if (m_vrefresh == 120) {
		ret = FHD_SDC120;
	} else if (m_vrefresh == 30) {
		ret = FHD_SDC30;
	} else {
		ret = FHD_SDC60;
	}
	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	DISP_ERR("ac328_p_b_a0029 %s +, mode id=%d\n", __func__, mode_id);

	switch (mode_id) {
		case FHD_SDC60:
			push_table(ctx, init_setting_60hz, sizeof(init_setting_60hz)/sizeof(struct LCM_setting_table));
			break;
		case FHD_SDC90:
			push_table(ctx, init_setting_90hz, sizeof(init_setting_90hz)/sizeof(struct LCM_setting_table));
			break;
		case FHD_SDC120:
			push_table(ctx, init_setting_120hz, sizeof(init_setting_120hz)/sizeof(struct LCM_setting_table));
			break;
		default:
			push_table(ctx, init_setting_120hz, sizeof(init_setting_120hz)/sizeof(struct LCM_setting_table));
			break;
		}

	DISP_ERR("ac328_p_b_a0029 %s -\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	DISP_ERR("%s:prepared=%d\n", __func__, ctx->prepared);

	if (!ctx->prepared)
		return 0;

	usleep_range(1000, 1100);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x5A, 0xA5, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(20000, 21000);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(150*1000, 151*1000);

	ctx->error = 0;
	ctx->prepared = false;
	//ctx->hbm_en = false;
	DISP_ERR("%s:success\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	DISP_ERR("%s:prepared=%d\n", __func__, ctx->prepared);
	if (ctx->prepared)
		return 0;

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
	DISP_ERR("%s:success\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

static const struct drm_display_mode disp_mode_60Hz = {
		.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_60HZ + VBP + VSA) * 60) / 1000,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP,
		.hsync_end = FRAME_WIDTH + HFP + HSA,
		.htotal = FRAME_WIDTH + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_60HZ,
		.vsync_end = FRAME_HEIGHT + VFP_60HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_60HZ + VSA + VBP,
};

static const struct drm_display_mode disp_mode_90Hz = {
		.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_90HZ + VBP + VSA) * 90) / 1000,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP,
		.hsync_end = FRAME_WIDTH + HFP + HSA,
		.htotal = FRAME_WIDTH + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_90HZ,
		.vsync_end = FRAME_HEIGHT + VFP_90HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_90HZ + VSA + VBP,
};

static const struct drm_display_mode disp_mode_120Hz = {
		.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_120HZ + VBP + VSA) * 120) / 1000,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP,
		.hsync_end = FRAME_WIDTH + HFP + HSA,
		.htotal = FRAME_WIDTH + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_120HZ,
		.vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
};

static struct drm_display_mode disp_mode_30Hz = {
		.clock = ((FRAME_WIDTH + HFP_30HZ + HBP_30HZ + HSA_30HZ) * (FRAME_HEIGHT + VFP_30HZ + VBP_30HZ + VSA_30HZ) * 30) / 1000,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP_30HZ,
		.hsync_end = FRAME_WIDTH + HFP_30HZ + HSA_30HZ,
		.htotal = FRAME_WIDTH + HFP_30HZ + HSA_30HZ + HBP_30HZ,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_30HZ,
		.vsync_end = FRAME_HEIGHT + VFP_30HZ + VSA_30HZ,
		.vtotal = FRAME_HEIGHT + VFP_30HZ + VSA_30HZ + VBP_30HZ,
};

static struct mtk_panel_params ext_params_30Hz = {
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 30,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	// .lcm_esd_check_table[1] = {
	// 	.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	// },

	.vdo_per_frame_lp_enable = 0,

	.vendor = "AC328_A0025",
	.manufacture = "P_3",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.color_vivid_status = true,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2372,
		.pic_width = 1080,
		.slice_height = 593,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 594,
		.scale_value = 32,
		.increment_interval = 13214,
		.decrement_interval = 7,
		.line_bpg_offset = 15,
		.nfl_bpg_offset = 52,
		.slice_bpg_offset = 44,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 8,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 4,
		.rc_range_parameters[1].range_max_qp = 8,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 5,
		.rc_range_parameters[2].range_max_qp = 9,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 5,
		.rc_range_parameters[3].range_max_qp = 10,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 7,
		.rc_range_parameters[4].range_max_qp = 11,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 7,
		.rc_range_parameters[5].range_max_qp = 11,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 7,
		.rc_range_parameters[6].range_max_qp = 11,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 7,
		.rc_range_parameters[7].range_max_qp = 12,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 7,
		.rc_range_parameters[8].range_max_qp = 13,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 7,
		.rc_range_parameters[9].range_max_qp = 14,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 9,
		.rc_range_parameters[10].range_max_qp = 14,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 9,
		.rc_range_parameters[11].range_max_qp = 15,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 9,
		.rc_range_parameters[12].range_max_qp = 15,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 13,
		.rc_range_parameters[13].range_max_qp = 16,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 16,
		.rc_range_parameters[14].range_max_qp = 17,
		.rc_range_parameters[14].range_bpg_offset = -12,
	},
	.panel_bpp = 10,
	.skip_unnecessary_switch = true,
};

static struct mtk_panel_params ext_params_60Hz = {
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,

	.change_fps_by_vfp_send_cmd = 1,

	.color_vivid_status = true,
	.vendor = "AC328_A0029",
	.manufacture = "P_B",

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},

	.oplus_serial_para0 = 0x80,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
		.dfps_cmd_table[1] = {0, 2 , {0x38, 0x00}},
		.dfps_cmd_table[2] = {0, 2 , {0x60, 0xA0}},
	},

	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2372,
		.pic_width = 1080,
		.slice_height = 593,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 594,
		.scale_value = 32,
		.increment_interval = 13214,
		.decrement_interval = 7,
		.line_bpg_offset = 15,
		.nfl_bpg_offset = 52,
		.slice_bpg_offset = 44,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 8,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 4,
		.rc_range_parameters[1].range_max_qp = 8,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 5,
		.rc_range_parameters[2].range_max_qp = 9,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 5,
		.rc_range_parameters[3].range_max_qp = 10,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 7,
		.rc_range_parameters[4].range_max_qp = 11,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 7,
		.rc_range_parameters[5].range_max_qp = 11,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 7,
		.rc_range_parameters[6].range_max_qp = 11,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 7,
		.rc_range_parameters[7].range_max_qp = 12,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 7,
		.rc_range_parameters[8].range_max_qp = 13,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 7,
		.rc_range_parameters[9].range_max_qp = 14,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 9,
		.rc_range_parameters[10].range_max_qp = 14,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 9,
		.rc_range_parameters[11].range_max_qp = 15,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 9,
		.rc_range_parameters[12].range_max_qp = 15,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 13,
		.rc_range_parameters[13].range_max_qp = 16,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 16,
		.rc_range_parameters[14].range_max_qp = 17,
		.rc_range_parameters[14].range_bpg_offset = -12,
	},
	.panel_bpp = 10,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.skip_unnecessary_switch = true,
};

static struct mtk_panel_params ext_params_90Hz = {
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,

	.change_fps_by_vfp_send_cmd = 1,

	.color_vivid_status = true,
	.vendor = "AC328_A0029",
	.manufacture = "P_B",

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},

	.oplus_serial_para0 = 0x80,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
		.dfps_cmd_table[1] = {0, 2 , {0x38, 0x00}},
		.dfps_cmd_table[2] = {0, 2 , {0x60, 0x90}},
	},

	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2372,
		.pic_width = 1080,
		.slice_height = 593,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 594,
		.scale_value = 32,
		.increment_interval = 13214,
		.decrement_interval = 7,
		.line_bpg_offset = 15,
		.nfl_bpg_offset = 52,
		.slice_bpg_offset = 44,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 8,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 4,
		.rc_range_parameters[1].range_max_qp = 8,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 5,
		.rc_range_parameters[2].range_max_qp = 9,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 5,
		.rc_range_parameters[3].range_max_qp = 10,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 7,
		.rc_range_parameters[4].range_max_qp = 11,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 7,
		.rc_range_parameters[5].range_max_qp = 11,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 7,
		.rc_range_parameters[6].range_max_qp = 11,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 7,
		.rc_range_parameters[7].range_max_qp = 12,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 7,
		.rc_range_parameters[8].range_max_qp = 13,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 7,
		.rc_range_parameters[9].range_max_qp = 14,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 9,
		.rc_range_parameters[10].range_max_qp = 14,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 9,
		.rc_range_parameters[11].range_max_qp = 15,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 9,
		.rc_range_parameters[12].range_max_qp = 15,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 13,
		.rc_range_parameters[13].range_max_qp = 16,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 16,
		.rc_range_parameters[14].range_max_qp = 17,
		.rc_range_parameters[14].range_bpg_offset = -12,
	},
	.panel_bpp = 10,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.skip_unnecessary_switch = true,
};

static struct mtk_panel_params ext_params_120Hz = {
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,

	.change_fps_by_vfp_send_cmd = 1,

	.color_vivid_status = true,
	.vendor = "AC328_A0029",
	.manufacture = "P_B",

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},

	.oplus_serial_para0 = 0x80,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
		.dfps_cmd_table[1] = {0, 2 , {0x38, 0x00}},
		.dfps_cmd_table[2] = {0, 2 , {0x60, 0x00}},
	},

	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2372,
		.pic_width = 1080,
		.slice_height = 593,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 594,
		.scale_value = 32,
		.increment_interval = 13214,
		.decrement_interval = 7,
		.line_bpg_offset = 15,
		.nfl_bpg_offset = 52,
		.slice_bpg_offset = 44,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 8,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 4,
		.rc_range_parameters[1].range_max_qp = 8,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 5,
		.rc_range_parameters[2].range_max_qp = 9,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 5,
		.rc_range_parameters[3].range_max_qp = 10,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 7,
		.rc_range_parameters[4].range_max_qp = 11,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 7,
		.rc_range_parameters[5].range_max_qp = 11,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 7,
		.rc_range_parameters[6].range_max_qp = 11,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 7,
		.rc_range_parameters[7].range_max_qp = 12,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 7,
		.rc_range_parameters[8].range_max_qp = 13,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 7,
		.rc_range_parameters[9].range_max_qp = 14,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 9,
		.rc_range_parameters[10].range_max_qp = 14,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 9,
		.rc_range_parameters[11].range_max_qp = 15,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 9,
		.rc_range_parameters[12].range_max_qp = 15,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 13,
		.rc_range_parameters[13].range_max_qp = 16,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 16,
		.rc_range_parameters[14].range_max_qp = 17,
		.rc_range_parameters[14].range_bpg_offset = -12,
	},
	.panel_bpp = 10,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.skip_unnecessary_switch = true,
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 0) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	}

	if (level == 1) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = 2047;
	}

	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;

	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	DISP_ERR("ac328_p_b_a0029 backlight = %d bl_level[1]=%x, bl_level[2]=%x\n", level, bl_level[1], bl_level[2]);
	oplus_display_brightness = level;
	lhbm_last_backlight = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &level);

	return 0;
}

static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int mode)
{
	unsigned int i = 0;
	pr_info("[DISP][INFO][%s: mode=%d\n", __func__, mode);
	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	temp_seed_mode = mode;

	switch(mode) {
		case NATURAL:
			for(i = 0; i < sizeof(dsi_set_seed_natural)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_natural[i].para_list, dsi_set_seed_natural[i].count);
			}
		break;
		case EXPERT:
			for(i = 0; i < sizeof(dsi_set_seed_expert)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_expert[i].para_list, dsi_set_seed_expert[i].count);
			}
		break;
		default:
		break;
	}
	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char esd_bl_level[] = {0x51, 0x03, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	esd_bl_level[1] = level >> 8;
	esd_bl_level[2] = level & 0xFF;
	cb(dsi, handle, esd_bl_level, ARRAY_SIZE(esd_bl_level));
	lhbm_last_backlight = level;
	DISP_ERR("esd_bl_level[1]=%x, esd_bl_level[2]=%x backlight = %d\n", esd_bl_level[1], esd_bl_level[2], level);
	return 0;
}

/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;
	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	pr_info("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, hbm_mode);
	if (hbm_mode == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, hbm_on_cmd[i].para_list, hbm_on_cmd[i].count);
		}
	} else if (hbm_mode == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, hbm_off_cmd[i].para_list, hbm_off_cmd[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	return 0;
}

static int oplus_ofp_set_lhbm_pressed_icon(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	unsigned int reg_count = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *lhbm_pressed_icon_cmd = NULL;
	int i = 0;
	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, should not set lhbm pressed icon\n");
	}

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid ctx params\n");
	}

	OFP_INFO("%s,oplus_display_brightness=%d, hbm_mode=%d, lhbm_last_backlight %u\n",
			 __func__, oplus_display_brightness, en,  lhbm_last_backlight);
	if (en) {
		if(lhbm_last_backlight > 1154) {
			reg_count = sizeof(lhbm_pressed_icon_on_cmd_dc_mode) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_on_cmd_dc_mode;
		} else {
			reg_count = sizeof(lhbm_pressed_icon_on_cmd_pwm_mode) / sizeof(struct LCM_setting_table);
			lhbm_pressed_icon_cmd = lhbm_pressed_icon_on_cmd_pwm_mode;
		}
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_cmd[i].para_list, lhbm_pressed_icon_cmd[i].count);
		}
	} else if (en == 0) {
		reg_count = sizeof(lhbm_pressed_icon_off_cmd) / sizeof(struct LCM_setting_table);
		lhbm_pressed_icon_cmd = lhbm_pressed_icon_off_cmd;
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_cmd[i].para_list, lhbm_pressed_icon_cmd[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;

	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	for (i = 0; i < (sizeof(AOD_off_setting) / sizeof(struct LCM_setting_table)); i++) {

		cmd = AOD_off_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count * 1000, AOD_off_setting[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count, AOD_off_setting[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, AOD_off_setting[i].para_list, AOD_off_setting[i].count);
		}
	}
	aod_state = false;

	OFP_INFO("%s send aod off cmd\n", __func__);
	return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;
	unsigned int reg_count = 0;
	aod_state = true;
	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}
	if (oplus_ofp_local_hbm_is_enabled() && oplus_ofp_get_hbm_state()) {
		reg_count = sizeof(lhbm_pressed_icon_off_cmd) / sizeof(struct LCM_setting_table);
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_off_cmd[i].para_list, lhbm_pressed_icon_off_cmd[i].count);
		}
		OFP_INFO("should off hbm\n");
	}
	for (i = 0; i < (sizeof(AOD_on_setting)/sizeof(struct LCM_setting_table)); i++) {
		cmd = AOD_on_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(AOD_on_setting[i].count * 1000, AOD_on_setting[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(AOD_on_setting[i].count, AOD_on_setting[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
			{
				cb(dsi, handle, AOD_on_setting[i].para_list, AOD_on_setting[i].count);
			}
		}
	}

	OFP_INFO("%s send aod on cmd\n", __func__);
	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_bl_level[i].para_list, aod_high_bl_level[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_bl_level[i].para_list, aod_low_bl_level[i].count);
		}
	}
	OFP_INFO("%s level = %d\n", __func__ , level);
	return 0;
}
//#endif

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	return 0;
}

static struct vdo_aod_params vdo_aod_on = {
	.porch_change_flag = 0x03,
	.dst_hfp = 2100,
	.dst_vfp = 48, //30fps
	.mode_idx = FHD_SDC30,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x02}},
	.vdo_aod_cmd_table[1]={2, {0x88, 0x0C}},//GIP timing 延后一帧生效
	.vdo_aod_cmd_table[2]={4, {0xFF, 0x5A, 0xA5,0x2D}},
	.vdo_aod_cmd_table[3]={2, {0xDC,0x03}},//IC插黑立即生效
	.vdo_aod_cmd_table[4]={4, {0xFF, 0x5A, 0xA5, 0x22}},
	.vdo_aod_cmd_table[5]={2, {0xDD, 0x00}},
	.vdo_aod_cmd_table[6]={4, {0xFF, 0x5A, 0xA5,0x00}},
	.vdo_aod_cmd_table[7]={1, {0x39}},
	.vdo_aod_cmd_table[8]={3, {0xFD, 0x00, 0x80}},
	.vdo_aod_cmd_table[9]={3, {0x51, 0x07, 0xFF}},
	.vdo_aod_cmd_table[10]={3, {0xFD, 0x00, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_120hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 172,
	.dst_vfp = 48,
	.mode_idx = 0,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x02}},
	.vdo_aod_cmd_table[1]={2, {0x88, 0x08}},//对应进AOD
	.vdo_aod_cmd_table[2]={4, {0xFF, 0x5A, 0xA5,0x2D}},
	.vdo_aod_cmd_table[3]={2, {0xDC, 0x02}},//插黑功能关掉
	.vdo_aod_cmd_table[4]={4, {0xFF, 0x5A, 0xA5,0x00}},
	.vdo_aod_cmd_table[5]={1, {0x38}},
	.vdo_aod_cmd_table[6]={4, {0xFF, 0x5A, 0xA5, 0x22}},
	.vdo_aod_cmd_table[7]={2, {0xDD, 0x10}},
	.vdo_aod_cmd_table[8]={4, {0xFF, 0x5A, 0xA5, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_120hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 172,
	.dst_vfp = 48,
	.mode_idx = 0,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x02}},
	.vdo_aod_cmd_table[1]={2, {0x88, 0x08}},
	.vdo_aod_cmd_table[2]={4, {0xFF, 0x5A, 0xA5,0x2D}},
	.vdo_aod_cmd_table[3]={2, {0xDC, 0x02}},
	.vdo_aod_cmd_table[4]={4, {0xFF, 0x5A, 0xA5,0x00}},
	.vdo_aod_cmd_table[5]={1, {0x38}},
	.vdo_aod_cmd_table[6]={5, {0x51, 0x00, 0x00,0x00, 0x00}},
	.vdo_aod_cmd_table[7]={4, {0xFF, 0x5A, 0xA5, 0x22}},
	.vdo_aod_cmd_table[8]={2, {0xDD, 0x10}},
	.vdo_aod_cmd_table[9]={4, {0xFF, 0x5A, 0xA5, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_90hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 172,
	.dst_vfp = 864,
	 .mode_idx = 1,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x02}},
	.vdo_aod_cmd_table[1]={2, {0x88, 0x08}},
	.vdo_aod_cmd_table[2]={4, {0xFF, 0x5A, 0xA5,0x2D}},
	.vdo_aod_cmd_table[3]={2, {0xDC, 0x02}},
	.vdo_aod_cmd_table[4]={4, {0xFF, 0x5A, 0xA5,0x00}},
	.vdo_aod_cmd_table[5]={1, {0x38}},
	.vdo_aod_cmd_table[6]={4, {0xFF, 0x5A, 0xA5, 0x22}},
	.vdo_aod_cmd_table[7]={2, {0xDD, 0x10}},
	.vdo_aod_cmd_table[8]={4, {0xFF, 0x5A, 0xA5, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_90hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 172,
	.dst_vfp = 864,
	 .mode_idx = 1,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x02}},
	.vdo_aod_cmd_table[1]={2, {0x88, 0x08}},
	.vdo_aod_cmd_table[2]={4, {0xFF, 0x5A, 0xA5,0x2D}},
	.vdo_aod_cmd_table[3]={2, {0xDC, 0x02}},
	.vdo_aod_cmd_table[4]={4, {0xFF, 0x5A, 0xA5,0x00}},
	.vdo_aod_cmd_table[5]={1, {0x38}},
	.vdo_aod_cmd_table[6]={5, {0x51, 0x00, 0x00,0x00, 0x00}},
	.vdo_aod_cmd_table[7]={4, {0xFF, 0x5A, 0xA5, 0x22}},
	.vdo_aod_cmd_table[8]={2, {0xDD, 0x10}},
	.vdo_aod_cmd_table[9]={4, {0xFF, 0x5A, 0xA5, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_60hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 172,
	.dst_vfp = 2496,
	.mode_idx = 2,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x02}},
	.vdo_aod_cmd_table[1]={2, {0x88, 0x08}},
	.vdo_aod_cmd_table[2]={4, {0xFF, 0x5A, 0xA5,0x2D}},
	.vdo_aod_cmd_table[3]={2, {0xDC, 0x02}},
	.vdo_aod_cmd_table[4]={4, {0xFF, 0x5A, 0xA5,0x00}},
	.vdo_aod_cmd_table[5]={1, {0x38}},
	.vdo_aod_cmd_table[6]={4, {0xFF, 0x5A, 0xA5, 0x22}},
	.vdo_aod_cmd_table[7]={2, {0xDD, 0x10}},
	.vdo_aod_cmd_table[8]={4, {0xFF, 0x5A, 0xA5, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_60hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 172,
	.dst_vfp = 2496,
	.mode_idx = 2,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x02}},
	.vdo_aod_cmd_table[1]={2, {0x88, 0x08}},
	.vdo_aod_cmd_table[2]={4, {0xFF, 0x5A, 0xA5,0x2D}},
	.vdo_aod_cmd_table[3]={2, {0xDC, 0x02}},
	.vdo_aod_cmd_table[4]={4, {0xFF, 0x5A, 0xA5,0x00}},
	.vdo_aod_cmd_table[5]={1, {0x38}},
	.vdo_aod_cmd_table[6]={5, {0x51, 0x00, 0x00,0x00, 0x00}},
	.vdo_aod_cmd_table[7]={4, {0xFF, 0x5A, 0xA5, 0x22}},
	.vdo_aod_cmd_table[8]={2, {0xDD, 0x10}},
	.vdo_aod_cmd_table[9]={4, {0xFF, 0x5A, 0xA5, 0x00}},
};

static unsigned int aod_status = 0;
static int mtk_get_vdo_aod_param(int aod_en, struct vdo_aod_params **vdo_aod_param)
{
	static int mode_id_before_aod = 0;
	if (aod_en) {
		*vdo_aod_param = &vdo_aod_on;
		mode_id_before_aod = mode_id;
	} else {
		if (mode_id_before_aod == FHD_SDC60) {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_60hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_60hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		} else if (mode_id_before_aod == FHD_SDC90) {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_90hz_unlocking;
			else
				*vdo_aod_param = &vdo_aod_to_90hz;
		} else {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_120hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_120hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		}
		if (oplus_ofp_get_aod_unlocking())
			lhbm_last_backlight = 0;
	}
	OFP_INFO("%s:aod_en %d, mode_id %d, unlocking =%d\n", __func__, aod_en, mode_id, oplus_ofp_get_aod_unlocking());
	aod_status = aod_en;
	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared) {
		DISP_DEBUG("ctx->prepared:%d return! \n",ctx->prepared);
		return 0;
	}
	DISP_INFO("ac328_p_b_a0029 %s+\n", __func__);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(20000, 20100);
	DISP_INFO("ac328_p_b_a0029 %s-\n", __func__);

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared){
		return 0;
	}
	DISP_ERR("ac328_p_b_a0029 %s+\n", __func__);

	/* vddi 1.8v Alawys Enable */
	//usleep_range(5000, 5100);

	/* vddr 1.2v */
	ctx->vddr_enable_gpio = devm_gpiod_get(ctx->dev, "vddr", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr_enable_gpio)) {
		DISP_ERR("%s: cannot get bias_gpio %ld\n",
				 __func__, PTR_ERR(ctx->vddr_enable_gpio));
		return PTR_ERR(ctx->vddr_enable_gpio);
	}
	gpiod_set_value(ctx->vddr_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr_enable_gpio);
	usleep_range(5000, 5100);

	/* vci 3.0v */
	ctx->vci_enable_gpio = devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_enable_gpio)) {
		DISP_ERR("%s: cannot get vci_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vci_enable_gpio));
		return PTR_ERR(ctx->vci_enable_gpio);
	}
	gpiod_set_value(ctx->vci_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci_enable_gpio);
	usleep_range(10000, 10010);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	DISP_ERR("ac328_p_b_a0029 poweron Successful %s-\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared) {
		return 0;
	}
	DISP_ERR("ac328_p_b_a0029 %s+\n", __func__);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);
	/* vci 3.0v */
	ctx->vci_enable_gpio = devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vci_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vci_enable_gpio);
	usleep_range(5000, 5100);
	/* vddr 1.2v */
	ctx->vddr_enable_gpio = devm_gpiod_get(ctx->dev, "vddr", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr_enable_gpio);
	usleep_range(5000, 5100);
	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	usleep_range(70000, 70100);
	flag_hbm = 0;
	DISP_ERR("ac328_p_b_a0029 poweroff Successful %s-\n", __func__);
	return 0;
}

static int lcm_get_modes(struct drm_panel *panel,
                    struct drm_connector *connector) {
	struct drm_display_mode *mode[4];
	DISP_INFO("ac328_p_b_a0029 %s+\n", __func__);
	mode[0] = drm_mode_duplicate(connector->dev, &disp_mode_60Hz);
	if (!mode[0]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_60Hz.hdisplay, disp_mode_60Hz.vdisplay, drm_mode_vrefresh(&disp_mode_60Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[0]);
	pr_info("%s clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n", __func__, mode[0]->clock, mode[0]->htotal,
			mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

	mode[1] = drm_mode_duplicate(connector->dev, &disp_mode_90Hz);
	if (!mode[1]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_90Hz.hdisplay, disp_mode_90Hz.vdisplay, drm_mode_vrefresh(&disp_mode_90Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[1]);
	mode[1]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[1]);

	mode[2] = drm_mode_duplicate(connector->dev, &disp_mode_120Hz);
	if (!mode[2]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_120Hz.hdisplay, disp_mode_120Hz.vdisplay, drm_mode_vrefresh(&disp_mode_120Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[2]);
	mode[2]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[2]);

	mode[3] = drm_mode_duplicate(connector->dev, &disp_mode_30Hz);
	if (!mode[3]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_30Hz.hdisplay, disp_mode_30Hz.vdisplay, drm_mode_vrefresh(&disp_mode_30Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[3]);
	mode[3]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[3]);

	connector->display_info.width_mm = PHYSICAL_WIDTH;
	connector->display_info.height_mm = PHYSICAL_HEIGHT;
	DISP_INFO("ac328_p_b_a0029 %s-\n", __func__);
    return 1;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);
	DISP_INFO("%s: mode=%d, vrefresh=%d\n", __func__, mode, m_vrefresh);

	if (m_vrefresh == 60) {
		ext->params = &ext_params_60Hz;
		current_fps = 60;
	} else if (m_vrefresh == 90) {
		ext->params = &ext_params_90Hz;
		current_fps = 90;
	} else if (m_vrefresh == 120) {
		ext->params = &ext_params_120Hz;
		current_fps = 120;
	} else if (m_vrefresh == 30) {
		ext->params = &ext_params_30Hz;
		current_fps = 30;
	} else {
		ret = 1;
	}

	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);

	mode_id = get_mode_enum(m);

	if (mode_id == FHD_SDC60) {
		*ext_param = &ext_params_60Hz;
	} else if (mode_id == FHD_SDC90) {
		*ext_param = &ext_params_90Hz;
	} else if (mode_id == FHD_SDC120) {
		*ext_param = &ext_params_120Hz;
	} else {
		*ext_param = &ext_params_120Hz;
	}

	if (*ext_param)
		DISP_DEBUG("ac328_p_b_a0029 data_rate:%d\n", (*ext_param)->data_rate);
	else
		DISP_ERR("ac328_p_b_a0029 ext_param is NULL;\n");

	return ret;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_reset = lcm_panel_reset,
	.panel_poweroff = lcm_panel_poweroff,
	.ata_check = panel_ata_check,
	.ext_param_get = mtk_panel_ext_param_get,
	.ext_param_set = mtk_panel_ext_param_set,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.set_hbm = lcm_set_hbm,
	.oplus_ofp_set_lhbm_pressed_icon_single = oplus_ofp_set_lhbm_pressed_icon,
//	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.get_vdo_aod_param = mtk_get_vdo_aod_param,
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.set_seed = panel_set_seed,
};


static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};


static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	unsigned int fp_type = 0xA10;
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	DISP_INFO("ac328_p_b_a0029 %s+\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				DISP_ERR("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			DISP_ERR("device node name:%s, dev->of_node name:%s\n", remote_node->name, dev->of_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		DISP_ERR("skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = DSI_LANES;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		DISP_ERR("cannot get reset-gpios %ld\n",
				 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
	usleep_range(5000, 5100);

	ctx->vddr_enable_gpio = devm_gpiod_get(ctx->dev, "vddr", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr_enable_gpio)) {
		DISP_ERR("%s: cannot get vddr_enable_gpio %ld\n",
				 __func__, PTR_ERR(ctx->vddr_enable_gpio));
		return PTR_ERR(ctx->vddr_enable_gpio);
	}
	gpiod_set_value(ctx->vddr_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr_enable_gpio);
	usleep_range(5000, 5100);

	ctx->vci_enable_gpio = devm_gpiod_get(ctx->dev, "vci", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vci_enable_gpio)) {
		DISP_ERR("cannot get vci_enable_gpio %ld\n",
			PTR_ERR(ctx->vci_enable_gpio));
		return PTR_ERR(ctx->vci_enable_gpio);
	}
	gpiod_set_value(ctx->vci_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vci_enable_gpio);
	usleep_range(10000, 10100);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_120Hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	register_device_proc("lcd", "AC328_A0029", "P_B");
/* #ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	oplus_ofp_set_fp_type(&fp_type);
/* #endif  *//* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	DISP_INFO("ac328_p_b_a0029 %s-\n", __func__);
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{
		.compatible = "ac328,p,b,a0029,vdo,panel",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "ac328_p_b_a0029_vdo_panel",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
