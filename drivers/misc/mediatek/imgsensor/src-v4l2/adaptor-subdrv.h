/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2020 MediaTek Inc. */

#ifndef __ADAPTOR_SUBDRV_H__
#define __ADAPTOR_SUBDRV_H__

#include <media/v4l2-subdev.h>

//#include "kd_imgsensor_define_v4l2.h"
#include "imgsensor-user.h"

#define DEBUG_LOG(ctx, ...) do {\
	imgsensor_info.sd = i2c_get_clientdata(ctx->i2c_client); \
	imgsensor_info.adaptor_ctx_ = to_ctx(imgsensor_info.sd);\
	if (unlikely(*((imgsensor_info.adaptor_ctx_)->sensor_debug_flag)))\
		LOG_INF(__VA_ARGS__);\
	} while (0)

/* def V4L2_MBUS_CSI2_IS_USER_DEFINED_DATA */
#define IMGSENSOR_VC_ROUTING

enum {
	HW_ID_AVDD = 0,
	HW_ID_DVDD,
	HW_ID_DOVDD,
	HW_ID_AFVDD,
	HW_ID_AVDD1,
	HW_ID_PDN,
	HW_ID_RST,
	HW_ID_MCLK,
	HW_ID_MCLK_DRIVING_CURRENT,
	HW_ID_MIPI_SWITCH,
	HW_ID_MAXCNT,
};

struct subdrv_pw_seq_entry {
	int id;
	int val;
	int delay;
};

#define HDR_CAP_IHDR 0x1
#define HDR_CAP_MVHDR 0x2
#define HDR_CAP_ZHDR 0x4
#define HDR_CAP_3HDR 0x8
#define HDR_CAP_ATR 0x10

#define PDAF_CAP_PIXEL_DATA_IN_RAW 0x1
#define PDAF_CAP_PIXEL_DATA_IN_VC 0x2
#define PDAF_CAP_DIFF_DATA_IN_VC 0x4
#define PDAF_CAP_PDFOCUS_AREA 0x10

struct subdrv_ctx {

	/* for i2c */
	struct i2c_client *i2c_client;

	unsigned int is_hflip:1;
	unsigned int is_vflip:1;
	unsigned int hdr_cap;
	unsigned int pdaf_cap;
	int max_frame_length;
	int ana_gain_min;
	int ana_gain_max;
	int ana_gain_step;
	int ana_gain_def;
	int exposure_min;
	int exposure_max;
	int exposure_step;
	int exposure_def;
	int le_shutter_def;
	int me_shutter_def;
	int se_shutter_def;
	int le_gain_def;
	int me_gain_def;
	int se_gain_def;

	u8 mirror; /* mirrorflip information */
	u8 sensor_mode; /* record IMGSENSOR_MODE enum value */
	u32 shutter; /* current shutter */
	u32 gain; /* current gain */
	u32 pclk; /* current pclk */
	u32 frame_length; /* current framelength */
	u32 line_length; /* current linelength */
	u32 min_frame_length;
	u8 margin; /* current (mode's) exp margin */
	u8 frame_time_delay_frame; /* EX: sony => 3 ; non-sony => 2 */
	u16 dummy_pixel; /* current dummypixel */
	u16 dummy_line; /* current dummline */
	u16 current_fps; /* current max fps */
	u8 autoflicker_en; /* record autoflicker enable or disable */
	u8 test_pattern; /* record test pattern mode or not */
	enum SENSOR_SCENARIO_ID_ENUM current_scenario_id;
	u8 ihdr_mode; /* ihdr enable or disable */
	u8 pdaf_mode; /* ihdr enable or disable */
	u8 hdr_mode; /* HDR mode : 0: disable HDR, 1:IHDR, 2:HDR, 9:ZHDR */
	struct IMGSENSOR_AE_FRM_MODE ae_frm_mode;
	u8 current_ae_effective_frame;
	u8 i2c_write_id;
	u32 readout_length; /* the current mode's readout line length */
	u8 read_margin; /* the read margin */

	u8 extend_frame_length_en;
	u8 fast_mode_on;
	u8 ae_ctrl_gph_en;
	u32 is_read_preload_eeprom;
	u32 is_read_four_cell;
	bool is_streaming;
};

struct subdrv_ops {
	int (*get_id)(struct subdrv_ctx *ctx, u32 *id);
	int (*init_ctx)(struct subdrv_ctx *ctx,
			struct i2c_client *i2c_client, u8 i2c_write_id);
	int (*open)(struct subdrv_ctx *ctx);
	int (*get_info)(struct subdrv_ctx *ctx,
			enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
			MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
	int (*get_resolution)(struct subdrv_ctx *ctx,
			MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
	int (*feature_control)(struct subdrv_ctx *ctx,
			MSDK_SENSOR_FEATURE_ENUM FeatureId,
			MUINT8 *pFeaturePara,
			MUINT32 *pFeatureParaLen);
	int (*control)(struct subdrv_ctx *ctx,
			enum MSDK_SCENARIO_ID_ENUM ScenarioId,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
			MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
	int (*close)(struct subdrv_ctx *ctx);
	int (*get_frame_desc)(struct subdrv_ctx *ctx,
			int scenario_id,
			struct mtk_mbus_frame_desc *fd);
	int (*get_temp)(struct subdrv_ctx *ctx, int *temp);
	int (*vsync_notify)(struct subdrv_ctx *ctx, unsigned int sof_cnt);
	int (*get_csi_param)(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id,
		struct mtk_csi_param *csi_param);

	int (*power_on)(struct subdrv_ctx *ctx, void *data);
	int (*power_off)(struct subdrv_ctx *ctx, void *data);
};

struct subdrv_entry {
	const char *name;
	unsigned int id;
	const struct subdrv_pw_seq_entry *pw_seq;
	const struct subdrv_ops *ops;
	int pw_seq_cnt;
};

#define subdrv_call(ctx, o, args...) \
({ \
	struct adaptor_ctx *__ctx = (ctx); \
	int __ret; \
	if (!__ctx || !__ctx->subdrv || !__ctx->subdrv->ops) \
		__ret = -ENODEV; \
	else if (!__ctx->subdrv->ops->o) \
		__ret = -ENOIOCTLCMD; \
	else \
		__ret = __ctx->subdrv->ops->o(&ctx->subctx, ##args); \
	__ret; \
})

#define subdrv_i2c_rd_u8(subctx, reg) \
({ \
	u8 __val = 0xff; \
	adaptor_i2c_rd_u8(subctx->i2c_client, \
		subctx->i2c_write_id >> 1, reg, &__val); \
	__val; \
})

#define subdrv_i2c_rd_u16(subctx, reg) \
({ \
	u16 __val = 0xffff; \
	adaptor_i2c_rd_u16(subctx->i2c_client, \
		subctx->i2c_write_id >> 1, reg, &__val); \
	__val; \
})

#define subdrv_i2c_wr_u8(subctx, reg, val) \
	adaptor_i2c_wr_u8(subctx->i2c_client, \
		subctx->i2c_write_id >> 1, reg, val)

#define subdrv_i2c_wr_u16(subctx, reg, val) \
	adaptor_i2c_wr_u16(subctx->i2c_client, \
		subctx->i2c_write_id >> 1, reg, val)

#define subdrv_i2c_wr_p8(subctx, reg, p_vals, n_vals) \
	adaptor_i2c_wr_p8(subctx->i2c_client, \
		subctx->i2c_write_id >> 1, reg, p_vals, n_vals)

#define subdrv_i2c_wr_p16(subctx, reg, p_vals, n_vals) \
	adaptor_i2c_wr_p16(subctx->i2c_client, \
		subctx->i2c_write_id >> 1, reg, p_vals, n_vals)

#define subdrv_i2c_wr_seq_p8(subctx, reg, p_vals, n_vals) \
	adaptor_i2c_wr_seq_p8(subctx->i2c_client, \
		subctx->i2c_write_id >> 1, reg, p_vals, n_vals)

#define subdrv_i2c_wr_regs_u8(subctx, list, len) \
	adaptor_i2c_wr_regs_u8(subctx->i2c_client, \
		subctx->i2c_write_id >> 1, list, len)

#define subdrv_i2c_wr_regs_u16(subctx, list, len) \
	adaptor_i2c_wr_regs_u16(subctx->i2c_client, \
		subctx->i2c_write_id >> 1, list, len)

#define FINE_INTEG_CONVERT(_shutter, _fine_integ) \
( \
	((_fine_integ) <= 0) ? \
	(_shutter) : \
	(((_shutter) > (_fine_integ)) ? (((_shutter) - (_fine_integ)) / 1000) : 0) \
)

#endif
