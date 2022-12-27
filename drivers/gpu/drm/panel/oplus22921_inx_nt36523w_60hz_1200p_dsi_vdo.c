// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

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

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif
#include "ktz8866.h"

#define BRIGHTNESS_MAX 					4095
#define BRIGHTNESS_HALF 				2047
#define MAX_NORMAL_BRIGHTNESS 			2047
#define SILKY_MAX_NORMAL_BRIGHTNESS 	8191
#define LCM_BRIGHTNESS_TYPE 			2

#define FHD_LCM_WIDTH 					1200
#define FHD_LCM_HEIGHT 					2000


#define REGFLAG_CMD 					0xFFFA
#define REGFLAG_DELAY 					0xFFFC
#define REGFLAG_UDELAY 					0xFFFB
#define REGFLAG_END_OF_TABLE 			0xFFFD

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};


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

//void lcdinfo_notify(unsigned long val, void *v);

struct lcm_pmic_info {
	struct regulator *reg_vrfio18_aif;
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_en;
	struct regulator *reg_vrfio18_aif;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	int error;
};

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}
/*
static void lcm_dcs_write_ext(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0) {
		printk("[lcd_info]%s:ctx->error=%d\n",__func__,ctx->error);
		return;
	}

	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);

	if (ret < 0) {
		printk("[lcd_info]error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}
*/
static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0) {
		printk("[lcd_info]%s: ctx->error=%d, LINE=%d\n",__func__, ctx->error, __LINE__);
		return;
	}

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		printk("[lcd_info]error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

/*
static void push_table(struct lcm *ctx, struct LCM_setting_table *table,
		unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			msleep(table[i].count);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 1000);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write(ctx, table[i].para_list,
				table[i].count);
			break;
		}
	}
}
*/
#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0) {
		printk("[lcd_info]%s:ctx->error=%d LINE=%d\n",__func__, ctx->error, __LINE__);
		return;
	}

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		printk("[lcd_info]error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	DISP_INFO("%s+\n");

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		printk("%s return %d data(0x%08x) to dsi engine\n", __func__, ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static unsigned int lcm_enable_reg_vrfio18_aif(struct lcm *ctx, int en)
{
	unsigned int ret = 0,volt = 0;

	printk("[lcd_info]%s +\n", __func__);
	if(!ctx->reg_vrfio18_aif) {
		printk("%s error return -1\n", __func__);
		return -1;
	}

	if(en) {
		ret = regulator_set_voltage(ctx->reg_vrfio18_aif, 1800000, 1800000);
		volt = regulator_get_voltage(ctx->reg_vrfio18_aif);
		printk("[lcd_info]%s volt=%d ret=%d\n", __func__, volt, ret);
		ret = regulator_enable(ctx->reg_vrfio18_aif);
	} else {
		ret = regulator_disable(ctx->reg_vrfio18_aif);
		printk("[lcd_info]%s ret=%d\n", __func__, ret);
	}
	printk("[lcd_info]%s -\n", __func__);
	return ret;

}

/*
static struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
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

static struct LCM_setting_table init_setting[] = {
		{0xFF, 0x1, {0x27} },
		{REGFLAG_DELAY, 1, {}},
		{0xFB, 0x1, {0x01} },
		{0xD0, 0x1, {0x31} },
		{0xD1, 0x1, {0x88} },
		{0xD2, 0x1, {0x30} },
		{0xDE, 0x1, {0x43} },
		{0xDF, 0x1, {0x02} },
		{0xFB, 0x1, {0x01} },
		{0x00, 0x1, {0x02} },

		{0xFF, 0x1, {0x20} },
		{REGFLAG_DELAY, 1, {}},
		{0xFB, 0x1, {0x01} },
		{0x05, 0x1, {0xD9} },
		{0x0D, 0x1, {0x63} },
		{0x0E, 0x1, {0x91} },
		{0x0F, 0x1, {0x00} },
		{0x07, 0x1, {0x7D} },
		{0x08, 0x1, {0x5F} },
		{0x0E, 0x1, {0x7D} },
		{0x0F, 0x1, {0x7D} },
		{0x95, 0x1, {0xF5} },
		{0x96, 0x1, {0xF5} },
		{0x9D, 0x1, {0x00} },
		{0x9E, 0x1, {0x00} },
		{0x44, 0x1, {0x51} },

		{0xFF, 0x1, {0x20} },
		{REGFLAG_DELAY, 1, {}},
		{0xFE, 0x1, {0x01} },
		{0xB0, 0x10, {0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8} },
		{0xB1, 0x10, {0x00, 0xC8, 0x01, 0x00, 0x01, 0x2A, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d} },
		{0xB2, 0x10, {0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x5d, 0x03, 0x64} },
		{0xB3, 0x0c, {0x03, 0x6a, 0x03, 0x76, 0x03, 0x80, 0x03, 0x8a, 0x03, 0x96, 0x03, 0x9a} },
		{0xB4, 0x10, {0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8} },
		{0xB5, 0x10, {0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d} },
		{0xB6, 0x10, {0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x5d, 0x03, 0x64} },
		{0xB7, 0x10, {0x03, 0x6a, 0x03, 0x76, 0x03, 0x80, 0x03, 0x8a, 0x03, 0x96, 0x03, 0x9a} },
		{0xB8, 0x10, {0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8} },
		{0xB9, 0x10, {0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d} },
		{0xBA, 0x10, {0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x5d, 0x03, 0x64} },
		{0xBB, 0x0c, {0x03, 0x6a, 0x03, 0x76, 0x03, 0x80, 0x03, 0x8a, 0x03, 0x96, 0x03, 0x9a} },
		{0xFF, 0X01, {0X21} },
		{0xFB, 0X01, {0X01} },
		{0xB0, 0x10, {0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8} },
		{0xB1, 0x10, {0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d} },
		{0xB2, 0x10, {0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x61, 0x03, 0x74} },
		{0xB3, 0x0c, {0x03, 0x8c, 0x03, 0xa3, 0x03, 0xc8, 0x03, 0xde, 0x03, 0xe6, 0x03, 0xe6} },
		{0xB4, 0x10, {0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8} },
		{0xB5, 0x10, {0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d} },
		{0xB6, 0x10, {0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x61, 0x03, 0x74} },
		{0xB7, 0x10, {0x03, 0x8c, 0x03, 0xa3, 0x03, 0xc8, 0x03, 0xde, 0x03, 0xe6, 0x03, 0xe6} },
		{0xB8, 0x10, {0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8} },
		{0xB9, 0x10, {0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d} },
		{0xBA, 0x10, {0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x61, 0x03, 0x74} },
		{0xBB, 0x0c, {0x03, 0x8c, 0x03, 0xa3, 0x03, 0xc8, 0x03, 0xde, 0x03, 0xe6, 0x03, 0xe6} },

		{0XFF, 0x01, {0x24} },
		{REGFLAG_DELAY, 1, {}},
		{0XFB, 0x01, {0x01} },
		{0X91, 0x01, {0x44} },
		{0X92, 0x01, {0x77} },
		{0X93, 0x01, {0x1A} },
		{0X94, 0x01, {0x40} },
		{0X9A, 0x01, {0x08} },
		{0X60, 0x01, {0x96} },
		{0X61, 0x01, {0xD0} },
		{0X63, 0x01, {0x70} },
		{0XC2, 0x01, {0xC6} },
		{0X9B, 0x01, {0x0F} },
		{0X9A, 0x01, {0x08} },
		{0XA5, 0x01, {0x00} },
		{0XA6, 0x01, {0x41} },
		{0X00, 0x01, {0x03} },
		{0X01, 0x01, {0x03} },
		{0X02, 0x01, {0x03} },
		{0X03, 0x01, {0x03} },
		{0X04, 0x01, {0x03} },
		{0X05, 0x01, {0x03} },
		{0X06, 0x01, {0x03} },
		{0X07, 0x01, {0x03} },
		{0X08, 0x01, {0x22} },
		{0X09, 0x01, {0x08} },
		{0X0A, 0x01, {0x05} },
		{0X0B, 0x01, {0x1D} },
		{0X0C, 0x01, {0x1C} },
		{0X0D, 0x01, {0x11} },
		{0X0E, 0x01, {0x10} },
		{0X0F, 0x01, {0x0F} },
		{0X10, 0x01, {0x0E} },
		{0X11, 0x01, {0x0D} },
		{0X12, 0x01, {0x0C} },
		{0X13, 0x01, {0x04} },
		{0X14, 0x01, {0x03} },
		{0X15, 0x01, {0x03} },
		{0X16, 0x01, {0x03} },
		{0X17, 0x01, {0x03} },
		{0X18, 0x01, {0x03} },
		{0X19, 0x01, {0x03} },
		{0X1A, 0x01, {0x03} },
		{0X1B, 0x01, {0x03} },
		{0X1C, 0x01, {0x03} },
		{0X1D, 0x01, {0x03} },
		{0X1E, 0x01, {0x22} },
		{0X1F, 0x01, {0x08} },
		{0X20, 0x01, {0x05} },
		{0X21, 0x01, {0x1D} },
		{0X22, 0x01, {0x1C} },
		{0X23, 0x01, {0x11} },
		{0X24, 0x01, {0x10} },
		{0X25, 0x01, {0x0F} },
		{0X26, 0x01, {0x0E} },
		{0X27, 0x01, {0x0D} },
		{0X28, 0x01, {0x0C} },
		{0X29, 0x01, {0x04} },
		{0X2A, 0x01, {0x03} },
		{0X2B, 0x01, {0x03} },
		{0X2F, 0x01, {0x05} },
		{0X30, 0x01, {0x33} },
		{0X31, 0x01, {0x40} },
		{0X33, 0x01, {0x33} },
		{0X34, 0x01, {0x05} },
		{0X35, 0x01, {0x40} },
		{0X37, 0x01, {0x55} },
		{0X38, 0x01, {0x40} },
		{0X39, 0x01, {0x00} },
		{0X3A, 0x01, {0x51} },
		{0X3B, 0x01, {0x3E} },
		{0X3D, 0x01, {0x92} },
		{0XAB, 0x01, {0x55} },
		{0XAC, 0x01, {0x40} },
		{0X3F, 0x01, {0x40} },
		{0X43, 0x01, {0x40} },
		{0X47, 0x01, {0x55} },
		{0X4A, 0x01, {0x51} },
		{0X4B, 0x01, {0x3E} },
		{0X4C, 0x01, {0x91} },
		{0X4D, 0x01, {0x21} },
		{0X4E, 0x01, {0x43} },
		{0X4F, 0x01, {0x65} },
		{0X51, 0x01, {0x34} },
		{0X52, 0x01, {0x12} },
		{0X53, 0x01, {0x56} },
		{0X55, 0x02, {0x84, 0x04} },
		{0X56, 0x01, {0x06} },
		{0X58, 0x01, {0x21} },
		{0X59, 0x01, {0x50} },
		{0X5A, 0x01, {0x51} },
		{0X5B, 0x01, {0x3E} },
		{0X5E, 0x02, {0x00, 0x0C} },
		{0X5F, 0x01, {0x00} },
		{0X7A, 0x01, {0xFF} },
		{0X7B, 0x01, {0xFF} },
		{0X7C, 0x01, {0x00} },
		{0X7D, 0x01, {0x00} },
		{0X7E, 0x01, {0x20} },
		{0X7F, 0x01, {0x3C} },
		{0X80, 0x01, {0x00} },
		{0X81, 0x01, {0x00} },
		{0X82, 0x01, {0x08} },
		{0X97, 0x01, {0x02} },
		{0XC5, 0x01, {0x10} },
		{0XD7, 0x01, {0x55} },
		{0XD8, 0x01, {0x55} },
		{0XD9, 0x01, {0x23} },
		{0XDA, 0x01, {0x05} },
		{0XDB, 0x01, {0x01} },
		{0XDC, 0x01, {0x77} },
		{0XDD, 0x01, {0x55} },
		{0XDE, 0x01, {0x27} },
		{0XDF, 0x01, {0x01} },
		{0XE0, 0x01, {0x77} },
		{0XE1, 0x01, {0x01} },
		{0XE2, 0x01, {0x77} },
		{0XE3, 0x01, {0x01} },
		{0XE4, 0x01, {0x77} },
		{0XE5, 0x01, {0x01} },
		{0XE6, 0x01, {0x77} },
		{0XE7, 0x01, {0x00} },
		{0XE8, 0x01, {0x00} },
		{0XE9, 0x01, {0x01} },
		{0XEA, 0x01, {0x77} },
		{0XEB, 0x01, {0x01} },
		{0XEE, 0x01, {0x77} },
		{0XEF, 0x01, {0x01} },
		{0XF0, 0x01, {0x77} },

		{0XFF, 0x01, {0x25} },
		{REGFLAG_DELAY, 1, {}},
		{0XFB, 0x01, {0x01} },
		{0X05, 0x01, {0x00} },
		{0XF1, 0x01, {0x10} },
		{0X1E, 0x01, {0x00} },
		{0X1F, 0x01, {0x51} },
		{0X20, 0x01, {0x3E} },
		{0X25, 0x01, {0x00} },
		{0X26, 0x01, {0x51} },
		{0X27, 0x01, {0x3E} },
		{0X3F, 0x01, {0x80} },
		{0X40, 0x01, {0x00} },
		{0X43, 0x01, {0x00} },
		{0X44, 0x01, {0x51} },
		{0X45, 0x01, {0x3E} },
		{0X48, 0x01, {0x51} },
		{0X49, 0x01, {0x3E} },
		{0X5B, 0x01, {0x80} },
		{0X5C, 0x01, {0x00} },
		{0X5D, 0x01, {0x51} },
		{0X5E, 0x01, {0x3E} },
		{0X5F, 0x01, {0x51} },
		{0X60, 0x01, {0x3E} },
		{0X61, 0x01, {0x51} },
		{0X62, 0x01, {0x3E} },
		{0X68, 0x01, {0x0C} },

		{0XFF, 0x01, {0x26} },
		{REGFLAG_DELAY, 1, {}},
		{0XFB, 0x01, {0x01} },
		{0X00, 0x01, {0xA1} },
		{0X02, 0x01, {0x31} },
		{0X04, 0x01, {0x28} },
		{0X06, 0x01, {0x30} },
		{0X0C, 0x01, {0x13} },
		{0X0D, 0x01, {0x0A} },
		{0X0F, 0x01, {0x0A} },
		{0X11, 0x01, {0x00} },
		{0X12, 0x01, {0x50} },
		{0X13, 0x01, {0x51} },
		{0X14, 0x01, {0x65} },
		{0X15, 0x01, {0x00} },
		{0X16, 0x01, {0x10} },
		{0X17, 0x01, {0xA0} },
		{0X18, 0x01, {0x86} },
		{0X22, 0x01, {0x00} },
		{0X23, 0x01, {0x00} },
		{0X19, 0x01, {0x0A} },
		{0X1A, 0x01, {0xA0} },
		{0X1B, 0x01, {0x0A} },
		{0X1C, 0x01, {0x00} },
		{0X2A, 0x01, {0x0A} },
		{0X2B, 0x01, {0xA0} },
		{0X1D, 0x01, {0x00} },
		{0X1E, 0x01, {0x77} },
		{0X1F, 0x01, {0x77} },
		{0X2F, 0x01, {0x04} },
		{0X30, 0x01, {0x77} },
		{0X31, 0x01, {0x04} },
		{0X32, 0x01, {0x97} },
		{0X33, 0x01, {0x89} },
		{0X34, 0x01, {0x67} },
		{0X39, 0x01, {0x0C} },
		{0X3A, 0x01, {0x74} },
		{0X3B, 0x01, {0x06} },
		{0XC8, 0x01, {0x04} },
		{0XC9, 0x01, {0x8B} },
		{0XCA, 0x01, {0x4E} },
		{0XCB, 0x01, {0x00} },
		{0XA9, 0x01, {0x50} },
		{0XAA, 0x01, {0x4F} },
		{0XAB, 0x01, {0x4E} },
		{0XAC, 0x01, {0x4D} },
		{0XAD, 0x01, {0x4C} },
		{0XAE, 0x01, {0x4B} },
		{0XAF, 0x01, {0x4A} },
		{0XB0, 0x01, {0x49} },
		{0XB1, 0x01, {0x47} },
		{0XB2, 0x01, {0x45} },

		{0XFF, 0x01, {0x27} },
		{0XFB, 0x01, {0x01} },
		{REGFLAG_DELAY, 1, {}},
		{0XC0, 0x01, {0x18} },
		{0XC1, 0x01, {0x00} },
		{0XC2, 0x01, {0x00} },
		{0X56, 0x01, {0x06} },
		{0X58, 0x01, {0x80} },
		{0X59, 0x01, {0x52} },
		{0X5A, 0x01, {0x00} },
		{0X5B, 0x01, {0x14} },
		{0X5C, 0x01, {0x00} },
		{0X5D, 0x01, {0x00} },
		{0X5E, 0x01, {0x20} },
		{0X5F, 0x01, {0x10} },
		{0X60, 0x01, {0x00} },
		{0X61, 0x01, {0x1B} },
		{0X62, 0x01, {0x00} },
		{0X63, 0x01, {0x01} },
		{0X64, 0x01, {0x23} },
		{0X65, 0x01, {0x1A} },
		{0X66, 0x01, {0x00} },
		{0X67, 0x01, {0x01} },
		{0X68, 0x01, {0x23} },
		{0X00, 0x01, {0x00} },
		{0XC3, 0x01, {0x00} },
		{0X98, 0x01, {0x01} },
		{0XB4, 0x01, {0x03} },
		{0X9B, 0x01, {0xBE} },
		{0XAB, 0x01, {0x14} },
		{0XBC, 0x01, {0x08} },
		{0XBD, 0x01, {0x28} },

		{0XFF, 0x01, {0x2A} },
		{REGFLAG_DELAY, 1, {}},
		{0XFB, 0x01, {0x01} },
		{0X22, 0x01, {0x2F} },
		{0X23, 0x01, {0x08} },
		{0X24, 0x01, {0x00} },
		{0X25, 0x01, {0x76} },
		{0X26, 0x01, {0xF8} },
		{0X27, 0x01, {0x00} },
		{0X28, 0x01, {0x1A} },
		{0X29, 0x01, {0x00} },
		{0X2A, 0x01, {0x1A} },
		{0X2B, 0x01, {0x00} },
		{0X2D, 0x01, {0x1A} },
		{0X64, 0x01, {0x96} },
		{0X65, 0x01, {0x00} },
		{0X66, 0x01, {0x00} },
		{0X6A, 0x01, {0x96} },
		{0X6B, 0x01, {0x00} },
		{0X6C, 0x01, {0x00} },
		{0X70, 0x01, {0x92} },
		{0X71, 0x01, {0x00} },
		{0X72, 0x01, {0x00} },
		{0XA2, 0x01, {0x33} },
		{0XA3, 0x01, {0x30} },
		{0XA4, 0x01, {0xC0} },
		{0XE8, 0x01, {0x00} },
		{0X97, 0x01, {0x3C} },
		{0X98, 0x01, {0x02} },
		{0X99, 0x01, {0x95} },
		{0X9A, 0x01, {0x06} },
		{0X9B, 0x01, {0x00} },
		{0X9C, 0x01, {0x0B} },
		{0X9D, 0x01, {0x0A} },
		{0X9E, 0x01, {0x90} },
		{0XFF, 0x01, {0x25} },
		{0X13, 0x01, {0x02} },
		{0X14, 0x01, {0xF2} },

		{0XFF, 0x01, {0x25} },
		{REGFLAG_DELAY, 1, {}},
		{0XFB, 0x01, {0x01} },
		{0X17, 0x01, {0xCF} },
		{0X19, 0x01, {0x0F} },
		{0X1B, 0x01, {0x5B} },

		{0XFF, 0x01, {0x26} },
		{REGFLAG_DELAY, 1, {}},
		{0XFB, 0x01, {0x01} },
		{0X00, 0x01, {0x81} },
		{0X01, 0x01, {0x70} },
		{0X40, 0x01, {0x76} },
		{0X41, 0x01, {0x76} },
		{0X42, 0x01, {0x76} },
		{0X45, 0x01, {0x04} },
		{0X46, 0x01, {0x76} },
		{0X47, 0x01, {0x04} },
		{0X48, 0x01, {0x83} },
		{0X4D, 0x01, {0x51} },
		{0X4E, 0x01, {0x3E} },
		{0X4F, 0x01, {0x51} },
		{0X50, 0x01, {0x3E} },
		{0X51, 0x01, {0x51} },
		{0X52, 0x01, {0x3E} },
		{0X56, 0x01, {0x51} },
		{0X58, 0x01, {0x3E} },
		{0X5B, 0x01, {0x51} },
		{0X5C, 0x01, {0x3E} },
		{0X64, 0x01, {0x51} },
		{0X65, 0x01, {0x3E} },
		{0X6E, 0x01, {0x51} },
		{0X6F, 0x01, {0x3E} },
		{0X70, 0x01, {0x51} },
		{0X71, 0x01, {0x3E} },
		{0X72, 0x01, {0x51} },
		{0X73, 0x01, {0x3E} },
		{0X8B, 0x01, {0x50} },
		{0X8C, 0x01, {0x7C} },
		{0X92, 0x01, {0x50} },
		{0X93, 0x01, {0x10} },
		{0X94, 0x01, {0x10} },
		{0X96, 0x01, {0x10} },
		{0XC9, 0x01, {0x80} },

		{0XFF, 0x01, {0x2A} },
		{REGFLAG_DELAY, 1, {}},
		{0XFB, 0x01, {0x01} },
		{0X01, 0x01, {0x01} },
		{0X02, 0x01, {0x11} },
		{0X05, 0x01, {0x01} },
		{0X06, 0x01, {0x76} },
		{0X08, 0x01, {0x01} },
		{0X09, 0x01, {0x76} },
		{0X0C, 0x01, {0x01} },
		{0X0D, 0x01, {0x76} },
		{0XFF, 0x01, {0xF0} },
		{0XFB, 0x01, {0x01} },
		{0X3A, 0x01, {0x08} },
		{0XFF, 0x01, {0xD0} },
		{0XFB, 0x01, {0x01} },
		{0X02, 0x01, {0xAF} },
		{0X09, 0x01, {0xFF} },
		{0XFF, 0x01, {0x23} },
		{0XFB, 0x01, {0x01} },
		{0X00, 0x01, {0x68} },
		{0X07, 0x01, {0x00} },
		{0X08, 0x01, {0x01} },
		{0X09, 0x01, {0x90} },

		{0XFF, 0x01, {0x23} },
		{REGFLAG_DELAY, 1, {}},
		{0XFB, 0x01, {0x01} },
		{0X10, 0x01, {0x0C} },
		{0X11, 0x01, {0x02} },
		{0X12, 0x01, {0x80} },
		{0X15, 0x01, {0x83} },
		{0X16, 0x01, {0x0C} },
		{0X19, 0x01, {0x00} },
		{0X1A, 0x01, {0x04} },
		{0X1B, 0x01, {0x08} },
		{0X1C, 0x01, {0x0C} },
		{0X1D, 0x01, {0x10} },
		{0X1E, 0x01, {0x14} },
		{0X1F, 0x01, {0x18} },
		{0X20, 0x01, {0x1C} },
		{0X21, 0x01, {0x20} },
		{0X22, 0x01, {0x24} },
		{0X23, 0x01, {0x28} },
		{0X24, 0x01, {0x2C} },
		{0X25, 0x01, {0x30} },
		{0X26, 0x01, {0x34} },
		{0X27, 0x01, {0x38} },
		{0X28, 0x01, {0x3C} },
		{0X29, 0x01, {0x20} },
		{0X2A, 0x01, {0x20} },
		{0X2B, 0x01, {0x30} },
		{0X30, 0x01, {0xFF} },
		{0X31, 0x01, {0xFC} },
		{0X32, 0x01, {0xFA} },
		{0X33, 0x01, {0xF4} },
		{0X34, 0x01, {0xF2} },
		{0X35, 0x01, {0xF0} },
		{0X36, 0x01, {0xEF} },
		{0X37, 0x01, {0xEC} },
		{0X38, 0x01, {0xEC} },
		{0X39, 0x01, {0xE9} },
		{0X3A, 0x01, {0xE9} },
		{0X3B, 0x01, {0xE8} },
		{0X3D, 0x01, {0xE7} },
		{0X3F, 0x01, {0xE6} },
		{0X40, 0x01, {0xE5} },
		{0X41, 0x01, {0xE3} },
		{0X58, 0x01, {0xFF} },
		{0X59, 0x01, {0xFB} },
		{0X5A, 0x01, {0xEE} },
		{0X5B, 0x01, {0xE8} },
		{0X5C, 0x01, {0xE3} },
		{0X5D, 0x01, {0xDE} },
		{0X5E, 0x01, {0xD6} },
		{0X5F, 0x01, {0xD4} },
		{0X60, 0x01, {0xD2} },
		{0X61, 0x01, {0xD0} },
		{0X62, 0x01, {0xCC} },
		{0X63, 0x01, {0xCC} },
		{0X64, 0x01, {0xCA} },
		{0X65, 0x01, {0xC8} },
		{0X66, 0x01, {0xC4} },
		{0X67, 0x01, {0xC0} },

		{0XFF, 0x01, {0x20} },
		{REGFLAG_DELAY, 1, {}},
		{0XFB, 0x01, {0x01} },
		{0XC3, 0x0C, {0x27, 0x26, 0x1E, 0x20, 0x19, 0x18, 0x17, 0x16, 0x15, 0x13, 0x07, 0x03} },

		{0XFF, 0x01, {0x10} },
		{REGFLAG_DELAY, 1, {}},
		{0XB9, 0x01, {0x01} },

		{0XFF, 0x01, {0x20} },
		{REGFLAG_DELAY, 1, {}},
		{0X18, 0x01, {0x40} },

		{0XFF, 0x01, {0x10} },
		{REGFLAG_DELAY, 1, {}},
		{0XB9, 0x01, {0x02} },
		{0XFF, 0x01, {0x10} },
		{0XFB, 0x01, {0x01} },
		{0XB0, 0x01, {0x01} },
		{0X35, 0x01, {0x00} },
		{0X55, 0x01, {0x00} },
		{0X68, 0x02, {0x05, 0x01} },
		{0X3B, 0x05, {0x03, 0x5F, 0x1A, 0x04, 0x04} },
		{0X53, 0x01, {0x2C} },
		{0X51, 0x02, {0x07, 0xFF} },
		{0X11, 0x00, {0x00} },
		{REGFLAG_DELAY, 120, {}},
		{0X29, 0x00, {0x00} },
		{REGFLAG_DELAY, 10, {}}

	};
*/
static void lcm_panel_init(struct lcm *ctx)
{
	printk("[lcd_info]%s +\n", __func__);
//	push_table(ctx, init_setting, sizeof(init_setting)/sizeof(struct LCM_setting_table));
	lcm_dcs_write_seq_static(ctx, 0xFF, 0x20);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x00, 0xC8, 0x01, 0x00, 0x01, 0x2A, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d);
	lcm_dcs_write_seq_static(ctx, 0xB2, 0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x5d, 0x03, 0x64);
	lcm_dcs_write_seq_static(ctx, 0xB3, 0x03, 0x6a, 0x03, 0x76, 0x03, 0x80, 0x03, 0x8a, 0x03, 0x96, 0x03, 0x9a);
	lcm_dcs_write_seq_static(ctx, 0xB4, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8);
	lcm_dcs_write_seq_static(ctx, 0xB5, 0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d);
	lcm_dcs_write_seq_static(ctx, 0xB6, 0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x5d, 0x03, 0x64);
	lcm_dcs_write_seq_static(ctx, 0xB7, 0x03, 0x6a, 0x03, 0x76, 0x03, 0x80, 0x03, 0x8a, 0x03, 0x96, 0x03, 0x9a);
	lcm_dcs_write_seq_static(ctx, 0xB8, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8);
	lcm_dcs_write_seq_static(ctx, 0xB9, 0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d);
	lcm_dcs_write_seq_static(ctx, 0xBA, 0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x5d, 0x03, 0x64);
	lcm_dcs_write_seq_static(ctx, 0xBB, 0x03, 0x6a, 0x03, 0x76, 0x03, 0x80, 0x03, 0x8a, 0x03, 0x96, 0x03, 0x9a);
	lcm_dcs_write_seq_static(ctx, 0xFF, 0X21);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0X01);
	lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8);
	lcm_dcs_write_seq_static(ctx, 0xB1, 0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d);
	lcm_dcs_write_seq_static(ctx, 0xB2, 0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x61, 0x03, 0x74);
	lcm_dcs_write_seq_static(ctx, 0xB3, 0x03, 0x8c, 0x03, 0xa3, 0x03, 0xc8, 0x03, 0xde, 0x03, 0xe6, 0x03, 0xe6);
	lcm_dcs_write_seq_static(ctx, 0xB4, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8);
	lcm_dcs_write_seq_static(ctx, 0xB5, 0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d);
	lcm_dcs_write_seq_static(ctx, 0xB6, 0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x61, 0x03, 0x74);
	lcm_dcs_write_seq_static(ctx, 0xB7, 0x03, 0x8c, 0x03, 0xa3, 0x03, 0xc8, 0x03, 0xde, 0x03, 0xe6, 0x03, 0xe6);
	lcm_dcs_write_seq_static(ctx, 0xB8, 0x00, 0x00, 0x00, 0x1d, 0x00, 0x41, 0x00, 0x60, 0x00, 0x7a, 0x00, 0x91, 0x00, 0xa5, 0x00, 0xb8);
	lcm_dcs_write_seq_static(ctx, 0xB9, 0x00, 0xc8, 0x01, 0x00, 0x01, 0x2a, 0x01, 0x6c, 0x01, 0x9f, 0x01, 0xec, 0x02, 0x2b, 0x02, 0x2d);
	lcm_dcs_write_seq_static(ctx, 0xBA, 0x02, 0x68, 0x02, 0xa9, 0x02, 0xd2, 0x03, 0x06, 0x03, 0x2a, 0x03, 0x53, 0x03, 0x61, 0x03, 0x74);
	lcm_dcs_write_seq_static(ctx, 0xBB, 0x03, 0x8c, 0x03, 0xa3, 0x03, 0xc8, 0x03, 0xde, 0x03, 0xe6, 0x03, 0xe6);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x27);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xD0, 0x31);
	lcm_dcs_write_seq_static(ctx, 0xD1, 0x88);
	lcm_dcs_write_seq_static(ctx, 0xD2, 0x30);
	lcm_dcs_write_seq_static(ctx, 0xDE, 0x43);
	lcm_dcs_write_seq_static(ctx, 0xDF, 0x02);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x00, 0x02);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x20);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x05, 0xD9);
	lcm_dcs_write_seq_static(ctx, 0x0D, 0x63);
	lcm_dcs_write_seq_static(ctx, 0x0E, 0x91);
	lcm_dcs_write_seq_static(ctx, 0x0F, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x07, 0x7D);
	lcm_dcs_write_seq_static(ctx, 0x08, 0x5F);
	lcm_dcs_write_seq_static(ctx, 0x0E, 0x7D);
	lcm_dcs_write_seq_static(ctx, 0x0F, 0x7D);
	lcm_dcs_write_seq_static(ctx, 0x95, 0xF5);
	lcm_dcs_write_seq_static(ctx, 0x96, 0xF5);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x44, 0x51);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x24);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X91, 0x44);
	lcm_dcs_write_seq_static(ctx, 0X92, 0x77);
	lcm_dcs_write_seq_static(ctx, 0X93, 0x1A);
	lcm_dcs_write_seq_static(ctx, 0X94, 0x40);
	lcm_dcs_write_seq_static(ctx, 0X9A, 0x08);
	lcm_dcs_write_seq_static(ctx, 0X60, 0x96);
	lcm_dcs_write_seq_static(ctx, 0X61, 0xD0);
	lcm_dcs_write_seq_static(ctx, 0X63, 0x70);
	lcm_dcs_write_seq_static(ctx, 0XC2, 0xC6);
	lcm_dcs_write_seq_static(ctx, 0X9B, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0X9A, 0x08);
	lcm_dcs_write_seq_static(ctx, 0XA5, 0x00);
	lcm_dcs_write_seq_static(ctx, 0XA6, 0x41);
	lcm_dcs_write_seq_static(ctx, 0X00, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X01, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X02, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X03, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X04, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X05, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X06, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X07, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X08, 0x22);
	lcm_dcs_write_seq_static(ctx, 0X09, 0x08);
	lcm_dcs_write_seq_static(ctx, 0X0A, 0x05);
	lcm_dcs_write_seq_static(ctx, 0X0B, 0x1D);
	lcm_dcs_write_seq_static(ctx, 0X0C, 0x1C);
	lcm_dcs_write_seq_static(ctx, 0X0D, 0x11);
	lcm_dcs_write_seq_static(ctx, 0X0E, 0x10);
	lcm_dcs_write_seq_static(ctx, 0X0F, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0X10, 0x0E);
	lcm_dcs_write_seq_static(ctx, 0X11, 0x0D);
	lcm_dcs_write_seq_static(ctx, 0X12, 0x0C);
	lcm_dcs_write_seq_static(ctx, 0X13, 0x04);
	lcm_dcs_write_seq_static(ctx, 0X14, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X15, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X16, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X17, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X18, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X19, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X1A, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X1B, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X1C, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X1D, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X1E, 0x22);
	lcm_dcs_write_seq_static(ctx, 0X1F, 0x08);
	lcm_dcs_write_seq_static(ctx, 0X20, 0x05);
	lcm_dcs_write_seq_static(ctx, 0X21, 0x1D);
	lcm_dcs_write_seq_static(ctx, 0X22, 0x1C);
	lcm_dcs_write_seq_static(ctx, 0X23, 0x11);
	lcm_dcs_write_seq_static(ctx, 0X24, 0x10);
	lcm_dcs_write_seq_static(ctx, 0X25, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0X26, 0x0E);
	lcm_dcs_write_seq_static(ctx, 0X27, 0x0D);
	lcm_dcs_write_seq_static(ctx, 0X28, 0x0C);
	lcm_dcs_write_seq_static(ctx, 0X29, 0x04);
	lcm_dcs_write_seq_static(ctx, 0X2A, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X2B, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X2F, 0x05);
	lcm_dcs_write_seq_static(ctx, 0X30, 0x33);
	lcm_dcs_write_seq_static(ctx, 0X31, 0x40);
	lcm_dcs_write_seq_static(ctx, 0X33, 0x33);
	lcm_dcs_write_seq_static(ctx, 0X34, 0x05);
	lcm_dcs_write_seq_static(ctx, 0X35, 0x40);
	lcm_dcs_write_seq_static(ctx, 0X37, 0x55);
	lcm_dcs_write_seq_static(ctx, 0X38, 0x40);
	lcm_dcs_write_seq_static(ctx, 0X39, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X3A, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X3B, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X3D, 0x92);
	lcm_dcs_write_seq_static(ctx, 0XAB, 0x55);
	lcm_dcs_write_seq_static(ctx, 0XAC, 0x40);
	lcm_dcs_write_seq_static(ctx, 0X3F, 0x40);
	lcm_dcs_write_seq_static(ctx, 0X43, 0x40);
	lcm_dcs_write_seq_static(ctx, 0X47, 0x55);
	lcm_dcs_write_seq_static(ctx, 0X4A, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X4B, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X4C, 0x91);
	lcm_dcs_write_seq_static(ctx, 0X4D, 0x21);
	lcm_dcs_write_seq_static(ctx, 0X4E, 0x43);
	lcm_dcs_write_seq_static(ctx, 0X4F, 0x65);
	lcm_dcs_write_seq_static(ctx, 0X51, 0x34);
	lcm_dcs_write_seq_static(ctx, 0X52, 0x12);
	lcm_dcs_write_seq_static(ctx, 0X53, 0x56);
	lcm_dcs_write_seq_static(ctx, 0X55, 0x84, 0x04);
	lcm_dcs_write_seq_static(ctx, 0X56, 0x06);
	lcm_dcs_write_seq_static(ctx, 0X58, 0x21);
	lcm_dcs_write_seq_static(ctx, 0X59, 0x50);
	lcm_dcs_write_seq_static(ctx, 0X5A, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X5B, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X5E, 0x00, 0x0C);
	lcm_dcs_write_seq_static(ctx, 0X5F, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X7A, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0X7B, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0X7C, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X7D, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X7E, 0x20);
	lcm_dcs_write_seq_static(ctx, 0X7F, 0x3C);
	lcm_dcs_write_seq_static(ctx, 0X80, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X81, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X82, 0x08);
	lcm_dcs_write_seq_static(ctx, 0X97, 0x02);
	lcm_dcs_write_seq_static(ctx, 0XC5, 0x10);
	lcm_dcs_write_seq_static(ctx, 0XD7, 0x55);
	lcm_dcs_write_seq_static(ctx, 0XD8, 0x55);
	lcm_dcs_write_seq_static(ctx, 0XD9, 0x23);
	lcm_dcs_write_seq_static(ctx, 0XDA, 0x05);
	lcm_dcs_write_seq_static(ctx, 0XDB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XDC, 0x77);
	lcm_dcs_write_seq_static(ctx, 0XDD, 0x55);
	lcm_dcs_write_seq_static(ctx, 0XDE, 0x27);
	lcm_dcs_write_seq_static(ctx, 0XDF, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XE0, 0x77);
	lcm_dcs_write_seq_static(ctx, 0XE1, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XE2, 0x77);
	lcm_dcs_write_seq_static(ctx, 0XE3, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XE4, 0x77);
	lcm_dcs_write_seq_static(ctx, 0XE5, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XE6, 0x77);
	lcm_dcs_write_seq_static(ctx, 0XE7, 0x00);
	lcm_dcs_write_seq_static(ctx, 0XE8, 0x00);
	lcm_dcs_write_seq_static(ctx, 0XE9, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XEA, 0x77);
	lcm_dcs_write_seq_static(ctx, 0XEB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XEE, 0x77);
	lcm_dcs_write_seq_static(ctx, 0XEF, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XF0, 0x77);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x25);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X05, 0x00);
	lcm_dcs_write_seq_static(ctx, 0XF1, 0x10);
	lcm_dcs_write_seq_static(ctx, 0X1E, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X1F, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X20, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X25, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X26, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X27, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X3F, 0x80);
	lcm_dcs_write_seq_static(ctx, 0X40, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X43, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X44, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X45, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X48, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X49, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X5B, 0x80);
	lcm_dcs_write_seq_static(ctx, 0X5C, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X5D, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X5E, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X5F, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X60, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X61, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X62, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X68, 0x0C);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x26);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X00, 0xA1);
	lcm_dcs_write_seq_static(ctx, 0X02, 0x31);
	lcm_dcs_write_seq_static(ctx, 0X04, 0x28);
	lcm_dcs_write_seq_static(ctx, 0X06, 0x30);
	lcm_dcs_write_seq_static(ctx, 0X0C, 0x13);
	lcm_dcs_write_seq_static(ctx, 0X0D, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0X0F, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0X11, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X12, 0x50);
	lcm_dcs_write_seq_static(ctx, 0X13, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X14, 0x65);
	lcm_dcs_write_seq_static(ctx, 0X15, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X16, 0x10);
	lcm_dcs_write_seq_static(ctx, 0X17, 0xA0);
	lcm_dcs_write_seq_static(ctx, 0X18, 0x86);
	lcm_dcs_write_seq_static(ctx, 0X22, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X23, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X19, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0X1A, 0xA0);
	lcm_dcs_write_seq_static(ctx, 0X1B, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0X1C, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X2A, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0X2B, 0xA0);
	lcm_dcs_write_seq_static(ctx, 0X1D, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X1E, 0x77);
	lcm_dcs_write_seq_static(ctx, 0X1F, 0x77);
	lcm_dcs_write_seq_static(ctx, 0X2F, 0x04);
	lcm_dcs_write_seq_static(ctx, 0X30, 0x77);
	lcm_dcs_write_seq_static(ctx, 0X31, 0x04);
	lcm_dcs_write_seq_static(ctx, 0X32, 0x97);
	lcm_dcs_write_seq_static(ctx, 0X33, 0x89);
	lcm_dcs_write_seq_static(ctx, 0X34, 0x67);
	lcm_dcs_write_seq_static(ctx, 0X39, 0x0C);
	lcm_dcs_write_seq_static(ctx, 0X3A, 0x74);
	lcm_dcs_write_seq_static(ctx, 0X3B, 0x06);
	lcm_dcs_write_seq_static(ctx, 0XC8, 0x04);
	lcm_dcs_write_seq_static(ctx, 0XC9, 0x8B);
	lcm_dcs_write_seq_static(ctx, 0XCA, 0x4E);
	lcm_dcs_write_seq_static(ctx, 0XCB, 0x00);
	lcm_dcs_write_seq_static(ctx, 0XA9, 0x50);
	lcm_dcs_write_seq_static(ctx, 0XAA, 0x4F);
	lcm_dcs_write_seq_static(ctx, 0XAB, 0x4E);
	lcm_dcs_write_seq_static(ctx, 0XAC, 0x4D);
	lcm_dcs_write_seq_static(ctx, 0XAD, 0x4C);
	lcm_dcs_write_seq_static(ctx, 0XAE, 0x4B);
	lcm_dcs_write_seq_static(ctx, 0XAF, 0x4A);
	lcm_dcs_write_seq_static(ctx, 0XB0, 0x49);
	lcm_dcs_write_seq_static(ctx, 0XB1, 0x47);
	lcm_dcs_write_seq_static(ctx, 0XB2, 0x45);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x27);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XC0, 0x18);
	lcm_dcs_write_seq_static(ctx, 0XC1, 0x00);
	lcm_dcs_write_seq_static(ctx, 0XC2, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X56, 0x06);
	lcm_dcs_write_seq_static(ctx, 0X58, 0x80);
	lcm_dcs_write_seq_static(ctx, 0X59, 0x52);
	lcm_dcs_write_seq_static(ctx, 0X5A, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X5B, 0x14);
	lcm_dcs_write_seq_static(ctx, 0X5C, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X5D, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X5E, 0x20);
	lcm_dcs_write_seq_static(ctx, 0X5F, 0x10);
	lcm_dcs_write_seq_static(ctx, 0X60, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X61, 0x1B);
	lcm_dcs_write_seq_static(ctx, 0X62, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X63, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X64, 0x23);
	lcm_dcs_write_seq_static(ctx, 0X65, 0x1A);
	lcm_dcs_write_seq_static(ctx, 0X66, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X67, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X68, 0x23);
	lcm_dcs_write_seq_static(ctx, 0X00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0XC3, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X98, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XB4, 0x03);
	lcm_dcs_write_seq_static(ctx, 0X9B, 0xBE);
	lcm_dcs_write_seq_static(ctx, 0XAB, 0x14);
	lcm_dcs_write_seq_static(ctx, 0XBC, 0x08);
	lcm_dcs_write_seq_static(ctx, 0XBD, 0x28);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x2A);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X22, 0x2F);
	lcm_dcs_write_seq_static(ctx, 0X23, 0x08);
	lcm_dcs_write_seq_static(ctx, 0X24, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X25, 0x76);
	lcm_dcs_write_seq_static(ctx, 0X26, 0xF8);
	lcm_dcs_write_seq_static(ctx, 0X27, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X28, 0x1A);
	lcm_dcs_write_seq_static(ctx, 0X29, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X2A, 0x1A);
	lcm_dcs_write_seq_static(ctx, 0X2B, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X2D, 0x1A);
	lcm_dcs_write_seq_static(ctx, 0X64, 0x96);
	lcm_dcs_write_seq_static(ctx, 0X65, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X66, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X6A, 0x96);
	lcm_dcs_write_seq_static(ctx, 0X6B, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X6C, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X70, 0x92);
	lcm_dcs_write_seq_static(ctx, 0X71, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X72, 0x00);
	lcm_dcs_write_seq_static(ctx, 0XA2, 0x33);
	lcm_dcs_write_seq_static(ctx, 0XA3, 0x30);
	lcm_dcs_write_seq_static(ctx, 0XA4, 0xC0);
	lcm_dcs_write_seq_static(ctx, 0XE8, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X97, 0x3C);
	lcm_dcs_write_seq_static(ctx, 0X98, 0x02);
	lcm_dcs_write_seq_static(ctx, 0X99, 0x95);
	lcm_dcs_write_seq_static(ctx, 0X9A, 0x06);
	lcm_dcs_write_seq_static(ctx, 0X9B, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X9C, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0X9D, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0X9E, 0x90);
	lcm_dcs_write_seq_static(ctx, 0XFF, 0x25);
	lcm_dcs_write_seq_static(ctx, 0X13, 0x02);
	lcm_dcs_write_seq_static(ctx, 0X14, 0xF2);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x25);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X17, 0xCF);
	lcm_dcs_write_seq_static(ctx, 0X19, 0x0F);
	lcm_dcs_write_seq_static(ctx, 0X1B, 0x5B);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x26);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X00, 0x81);
	lcm_dcs_write_seq_static(ctx, 0X01, 0x70);
	lcm_dcs_write_seq_static(ctx, 0X40, 0x76);
	lcm_dcs_write_seq_static(ctx, 0X41, 0x76);
	lcm_dcs_write_seq_static(ctx, 0X42, 0x76);
	lcm_dcs_write_seq_static(ctx, 0X45, 0x04);
	lcm_dcs_write_seq_static(ctx, 0X46, 0x76);
	lcm_dcs_write_seq_static(ctx, 0X47, 0x04);
	lcm_dcs_write_seq_static(ctx, 0X48, 0x83);
	lcm_dcs_write_seq_static(ctx, 0X4D, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X4E, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X4F, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X50, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X51, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X52, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X56, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X58, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X5B, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X5C, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X64, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X65, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X6E, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X6F, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X70, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X71, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X72, 0x51);
	lcm_dcs_write_seq_static(ctx, 0X73, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0X8B, 0x50);
	lcm_dcs_write_seq_static(ctx, 0X8C, 0x7C);
	lcm_dcs_write_seq_static(ctx, 0X92, 0x50);
	lcm_dcs_write_seq_static(ctx, 0X93, 0x10);
	lcm_dcs_write_seq_static(ctx, 0X94, 0x10);
	lcm_dcs_write_seq_static(ctx, 0X96, 0x10);
	lcm_dcs_write_seq_static(ctx, 0XC9, 0x80);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x2A);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X01, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X02, 0x11);
	lcm_dcs_write_seq_static(ctx, 0X05, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X06, 0x76);
	lcm_dcs_write_seq_static(ctx, 0X08, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X09, 0x76);
	lcm_dcs_write_seq_static(ctx, 0X0C, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X0D, 0x76);
	lcm_dcs_write_seq_static(ctx, 0XFF, 0xF0);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X3A, 0x08);
	lcm_dcs_write_seq_static(ctx, 0XFF, 0xD0);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X02, 0xAF);
	lcm_dcs_write_seq_static(ctx, 0X09, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0XFF, 0x23);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X00, 0x68);
	lcm_dcs_write_seq_static(ctx, 0X07, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X08, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X09, 0x90);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x23);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X10, 0x0C);
	lcm_dcs_write_seq_static(ctx, 0X11, 0x02);
	lcm_dcs_write_seq_static(ctx, 0X12, 0x80);
	lcm_dcs_write_seq_static(ctx, 0X15, 0x83);
	lcm_dcs_write_seq_static(ctx, 0X16, 0x0C);
	lcm_dcs_write_seq_static(ctx, 0X19, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X1A, 0x04);
	lcm_dcs_write_seq_static(ctx, 0X1B, 0x08);
	lcm_dcs_write_seq_static(ctx, 0X1C, 0x0C);
	lcm_dcs_write_seq_static(ctx, 0X1D, 0x10);
	lcm_dcs_write_seq_static(ctx, 0X1E, 0x14);
	lcm_dcs_write_seq_static(ctx, 0X1F, 0x18);
	lcm_dcs_write_seq_static(ctx, 0X20, 0x1C);
	lcm_dcs_write_seq_static(ctx, 0X21, 0x20);
	lcm_dcs_write_seq_static(ctx, 0X22, 0x24);
	lcm_dcs_write_seq_static(ctx, 0X23, 0x28);
	lcm_dcs_write_seq_static(ctx, 0X24, 0x2C);
	lcm_dcs_write_seq_static(ctx, 0X25, 0x30);
	lcm_dcs_write_seq_static(ctx, 0X26, 0x34);
	lcm_dcs_write_seq_static(ctx, 0X27, 0x38);
	lcm_dcs_write_seq_static(ctx, 0X28, 0x3C);
	lcm_dcs_write_seq_static(ctx, 0X29, 0x20);
	lcm_dcs_write_seq_static(ctx, 0X2A, 0x20);
	lcm_dcs_write_seq_static(ctx, 0X2B, 0x30);
	lcm_dcs_write_seq_static(ctx, 0X30, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0X31, 0xFC);
	lcm_dcs_write_seq_static(ctx, 0X32, 0xFA);
	lcm_dcs_write_seq_static(ctx, 0X33, 0xF4);
	lcm_dcs_write_seq_static(ctx, 0X34, 0xF2);
	lcm_dcs_write_seq_static(ctx, 0X35, 0xF0);
	lcm_dcs_write_seq_static(ctx, 0X36, 0xEF);
	lcm_dcs_write_seq_static(ctx, 0X37, 0xEC);
	lcm_dcs_write_seq_static(ctx, 0X38, 0xEC);
	lcm_dcs_write_seq_static(ctx, 0X39, 0xE9);
	lcm_dcs_write_seq_static(ctx, 0X3A, 0xE9);
	lcm_dcs_write_seq_static(ctx, 0X3B, 0xE8);
	lcm_dcs_write_seq_static(ctx, 0X3D, 0xE7);
	lcm_dcs_write_seq_static(ctx, 0X3F, 0xE6);
	lcm_dcs_write_seq_static(ctx, 0X40, 0xE5);
	lcm_dcs_write_seq_static(ctx, 0X41, 0xE3);
	lcm_dcs_write_seq_static(ctx, 0X58, 0xFF);
	lcm_dcs_write_seq_static(ctx, 0X59, 0xFB);
	lcm_dcs_write_seq_static(ctx, 0X5A, 0xEE);
	lcm_dcs_write_seq_static(ctx, 0X5B, 0xE8);
	lcm_dcs_write_seq_static(ctx, 0X5C, 0xE3);
	lcm_dcs_write_seq_static(ctx, 0X5D, 0xDE);
	lcm_dcs_write_seq_static(ctx, 0X5E, 0xD6);
	lcm_dcs_write_seq_static(ctx, 0X5F, 0xD4);
	lcm_dcs_write_seq_static(ctx, 0X60, 0xD2);
	lcm_dcs_write_seq_static(ctx, 0X61, 0xD0);
	lcm_dcs_write_seq_static(ctx, 0X62, 0xCC);
	lcm_dcs_write_seq_static(ctx, 0X63, 0xCC);
	lcm_dcs_write_seq_static(ctx, 0X64, 0xCA);
	lcm_dcs_write_seq_static(ctx, 0X65, 0xC8);
	lcm_dcs_write_seq_static(ctx, 0X66, 0xC4);
	lcm_dcs_write_seq_static(ctx, 0X67, 0xC0);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x20);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XC3, 0x27, 0x26, 0x1E, 0x20, 0x19, 0x18, 0x17, 0x16, 0x15, 0x13, 0x07, 0x03);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x10);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XB9, 0x01);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x20);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0X18, 0x40);

	lcm_dcs_write_seq_static(ctx, 0XFF, 0x10);
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XB9, 0x02);
	lcm_dcs_write_seq_static(ctx, 0XFF, 0x10);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XB0, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X55, 0x00);
	lcm_dcs_write_seq_static(ctx, 0X68, 0x05, 0x01);
	lcm_dcs_write_seq_static(ctx, 0X3B, 0x03, 0x5F, 0x1A, 0x04, 0x04);
	lcm_dcs_write_seq_static(ctx, 0X53, 0x2C);
	lcm_dcs_write_seq_static(ctx, 0X51, 0x07, 0xFF);
/*
	lcm_dcs_write_seq_static(ctx, 0XFF, 0x25);//panel bist mode for test
	usleep_range(1000, 1010);
	lcm_dcs_write_seq_static(ctx, 0XFB, 0x01);
	lcm_dcs_write_seq_static(ctx, 0XEC, 0x01);
*/
	lcm_dcs_write_seq_static(ctx, 0X11);
	usleep_range(120000, 120100);
	lcm_dcs_write_seq_static(ctx, 0X29);
	usleep_range(10000, 10100);
	printk("[lcd_info]%s -\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s +\n", __func__);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	printk("[lcd_info]%s -\n", __func__);

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	printk("[lcd_info]%s +\n", __func__);
	if (!ctx->prepared) {
		printk("[lcd_info]%s ctx->prepared=%d\n", __func__, ctx->prepared);
		return 0;
	}

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(20000, 20100);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(110000, 110100);

	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(2000, 2100);

	lcd_set_bl_bias_reg(ctx->dev, 0);

	//disable 1.8V
	ret=lcm_enable_reg_vrfio18_aif(ctx, 0);

	ctx->error = 0;
	ctx->prepared = false;
	printk("[lcd_info]%s -\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	printk("[lcd_info]%s +\n", __func__);

	if (ctx->prepared){
		printk("[lcd_info]%s frist time ctx->prepared=%d\n", __func__, ctx->prepared);
		return 0;
	}

	//set vddi 1.8v
	ret=lcm_enable_reg_vrfio18_aif(ctx, 1);

	lcd_set_bl_bias_reg(ctx->dev, 1);
	usleep_range(20000, 21000);

	// lcd reset
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(10000,10100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000,5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000,10100);

	lcm_panel_init(ctx);
	usleep_range(2000, 2100);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	printk("[lcd_info]%s -\n", __func__);

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s +\n", __func__);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;
	printk("[lcd_info]%s -\n", __func__);

	return 0;
}

#define HFP (12)
#define HSA (2)
#define HBP (10)
#define VFP (26)
#define VSA (1)
#define VBP (94)

#define VAC_FHD (2000)
#define HAC_FHD (1200)

static const struct drm_display_mode display_mode = {
	//fhd_sdc_60_mode
	.clock = 156500,
	.hdisplay = HAC_FHD,
	.hsync_start = HAC_FHD + HFP,
	.hsync_end = HAC_FHD + HFP + HSA,
	.htotal = HAC_FHD + HFP + HSA + HBP,//1224
	.vdisplay = VAC_FHD,
	.vsync_start = VAC_FHD + VFP,
	.vsync_end = VAC_FHD + VFP + VSA,
	.vtotal = VAC_FHD + VFP + VSA + VBP,//2131
	.hskew = 1,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params= {
	//fhd_sdc_60_mode
	.data_rate = 980,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
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

	mapped_level = level * 0x7FF / 1024;

	bl_level[1] = mapped_level >> 8;
	bl_level[2] = mapped_level & 0xFF;

	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));

	printk("[lcd_info][%s:%d] mapped_level:%d level:%d \n", __func__, __LINE__, mapped_level, level);

	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	printk("[lcd_info]%s on=%d\n", __func__, on);

	gpiod_set_value(ctx->reset_gpio, on);

	return 0;
}

/*
static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared) {
		printk("ctx->prepared:%d return! \n",ctx->prepared);
		return 0;
	}
	usleep_range(3000,3100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(10000,10100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000,5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000,10100);
	printk("Successful\n");

	return 0;
}
*/
static int lcm_get_modes(struct drm_panel *panel,
			struct drm_connector *connector) {
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &display_mode);
	if (!mode) {
		printk("[lcd_info]failed to add mode %ux%ux@%u\n",
			display_mode.hdisplay, display_mode.vdisplay,
			drm_mode_vrefresh(&display_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);
	printk("[lcd_info]en=%u, clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n",mode, mode->clock, mode->htotal,
	mode->vtotal, mode->hskew, drm_mode_vrefresh(mode));

	connector->display_info.width_mm = 70;//align x3 panel physical w/h
	connector->display_info.height_mm = 156;

	return 1;
}


static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
};
#endif

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
	int ret = 0;

	printk("[lcd_info]%s +\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				printk("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			printk("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		printk("skip probe due to not current lcm\n");
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		printk("[lcd_info]cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->bias_en = devm_gpiod_get(ctx->dev, "pm-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_en)) {
		printk("[lcd_info]cannot get bias_en %ld\n",
			PTR_ERR(ctx->bias_en));
		return PTR_ERR(ctx->bias_en);
	}
	gpiod_set_value(ctx->bias_en, 1);
	printk("%s set GPIO68 to high befor devm_gpiod_put\n", __func__);
	devm_gpiod_put(ctx->dev, ctx->bias_en);

	ctx->reg_vrfio18_aif= regulator_get(dev, "1p8");
	if (IS_ERR(ctx->reg_vrfio18_aif)) {
		printk("[lcd_info]cannot get reg_vrfio18_aif %ld\n",
			PTR_ERR(ctx->reg_vrfio18_aif));
		return -517;
	} else {
			printk("[lcd_info]get reg_vrfio18_aif %ld succ\n",
			PTR_ERR(ctx->reg_vrfio18_aif));
	}
	ret=lcm_enable_reg_vrfio18_aif(ctx, 1);

	msleep(10);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		printk("[lcd_info]%s mipi_dsi_attach fail LINE=%d ret=%d\n", __func__, __LINE__, ret);
		drm_panel_remove(&ctx->panel);
	}
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0) {
		printk("[lcd_info]%s mtk_panel_ext_create fail LINE=%d ret=%d\n", __func__, __LINE__, ret);
		return ret;
	}
#endif

	printk("[lcd_info]%s -\n", __func__);
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
	    .compatible = "oplus22921_inx_nt36523w_60hz_1200p_dsi_vdo",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus22921_inx_nt36523w_60hz_1200p_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("liuwenqi");
MODULE_DESCRIPTION("lcm nt36523w Panel Driver");
MODULE_LICENSE("GPL v2");
