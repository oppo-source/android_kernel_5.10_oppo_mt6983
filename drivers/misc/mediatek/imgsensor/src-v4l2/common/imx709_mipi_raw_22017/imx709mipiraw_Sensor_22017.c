/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     IMX709mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"

//#include "imgsensor_eeprom.h"
#include "imx709mipiraw_Sensor_22017.h"
#include "imx709_ana_gain_table_22017.h"
#include "imx709_eeprom_22017.h"
#include "imx709_vcm_22017.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"
#include "adaptor.h"

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define write_cmos_sensor(...) subdrv_i2c_wr_u16(__VA_ARGS__)
//#define table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)
//#define imx709_table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u16(__VA_ARGS__)
#define imx709_table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)
#define imx709_seq_write_cmos_sensor(...)  subdrv_i2c_wr_seq_p8(__VA_ARGS__)
/***************Modify Following Strings for Debug**********************/
#define PFX "imx709_camera_sensor"
#define LOG_1 LOG_INF("IMX709,MIPI 4LANE\n")
/****************************Modify end**************************/

#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)
#define lrc_cal_en 1
static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable);
#define MODULE_ID_OFFSET 0x0000
//static kal_uint16 imx709_table_write_cmos_sensor(ctx, kal_uint16 *para, kal_uint32 len);

#define _I2C_BUF_SIZE 256
static kal_uint16 _i2c_data[_I2C_BUF_SIZE];
static unsigned int _size_to_write;

static kal_uint16 imx709_LRC_setting[LRC_SIZE*2];
static kal_uint8 imx709_QSC_setting[QSC_SIZE];
static kal_uint8 qsc_flag = 0, lrc_flag = 0;
static kal_uint8 qsc_is_valid = 0, lrc_is_valid = 0;
static kal_uint32 previous_exp[3];
static kal_uint16 previous_exp_cnt;
static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = IMX709_SENSOR_ID_22017,
    .checksum_value = 0x5a6563df,

    .pre = { /* reg_B8 3264*2448@30fps*/
        .pclk = 864000000,
        .linelength = 11760,
        .framelength = 2504,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 301200000,
        .max_framerate = 293, /* 30fps */
    },

    .cap = { /* reg_B3 3264*2448@30fps*/
        .pclk = 864000000,
        .linelength = 11760,
        .framelength = 2504,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 301200000,
        .max_framerate = 293, /* 30fps */
    },

    .normal_video = { /* reg_C4-3-11 3264*1856@30fps*/
        .pclk = 864000000,
        .linelength = 11760,
        .framelength = 2448,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 458000000,
        .max_framerate = 300, /* 30fps */
    },

    .hs_video = { /*reg_G2 3264x1856@60ps*/
        .pclk = 849600000,
        .linelength = 7400,
        .framelength = 1912,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 478628572,
        .max_framerate = 600, /* 60fps */
    },

    .slim_video = { /*reg_G2 3264x1856@60ps*/
        .pclk = 849600000,
        .linelength = 7400,
        .framelength = 1912,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 478628572,
        .max_framerate = 600, /* 60fps */
    },

    .custom1 = { /*reg H 1632x1224@15FPS */
        .pclk = 240000000,
        .linelength = 5880,
        .framelength = 2720,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 120000000,
        .max_framerate = 150, /* 15fps */
    },

    .custom2 = {  /*reg H1 1632x1224 @30FPS*/
        .pclk = 374400000,
        .linelength = 5880,
        .framelength = 2120,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 120000000,
        .max_framerate = 300, /* 30fps */
    },

    .custom3 = {/*reg-A_5: 6528x4896@15 for remosaic*/
        .pclk = 537600000,
        .linelength = 7120,
        .framelength = 5032,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 6528,
        .grabwindow_height = 4896,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 936000000,
        .max_framerate = 150,
    },

    .custom4 = { /*reg C4-5 3264*1856@30FPS 2DOL*/
        .pclk = 864000000,
        .linelength = 7456,
        .framelength = 3856,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 720000000,
        .max_framerate = 300, /* 30fps */
    },

    .custom5 = {/* reg_C3 3264*1856@30fps, reserved sensormode*/
        .pclk = 720000000,
        .linelength = 7400,
        .framelength = 3240,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 396342858,
        .max_framerate = 300, /* 30fps */
    },

    .custom6 = {/* reg_C4-3-11 3264*1856@30fps*/
        .pclk = 864000000,
        .linelength = 11760,
        .framelength = 2448,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 458000000,
        .max_framerate = 300, /* 30fps */
    },

    .custom7 = {  /*preisp , reg C4-5 3264*1856@30FPS 2DOL*/
        .pclk = 864000000,
        .linelength = 7456,
        .framelength = 3856,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 720000000,
        .max_framerate = 300, /* 30fps */
    },
    .custom8 = { /* reg_C4-3-1 3264*1856@30fps*/
        .pclk = 864000000,
        .linelength = 11760,
        .framelength = 2448,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        /* following for GetDefaultFramerateByScenario() */
        .mipi_pixel_rate = 483428572,
        .max_framerate = 300, /* 30fps */
    },
    .custom9 = {  /*preisp , reg I3 3264*1856@30FPS 2DOL*/
        .pclk = 859200000,
        .linelength = 7456,
        .framelength = 3840,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1856,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 720000000,
        .max_framerate = 300, /* 30fps */
    },
    .custom10 = {  /*reg H1 1632x1224 @30FPS with pd*/
        .pclk = 355200000,
        .linelength = 5880,
        .framelength = 2008,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 182800000,
        .max_framerate = 300, /* 30fps */
    },

    .margin = 24,        /* sensor framelength & shutter margin */
    .min_shutter = 8,    /* min shutter */
    .min_gain = BASEGAIN * 1, /*1x gain*/
    .max_gain = BASEGAIN * 32, /*64x gain*/
    .min_gain_iso = 100,
    .gain_step = 1,
    .gain_type = 0,
    .max_frame_length = 0xffff-5,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,    /* isp gain delay frame for AE cycle */
    .ihdr_support = 0,    /* 1, support; 0,not support */
    .ihdr_le_firstline = 0,    /* 1,le first ; 0, se first */
    .sensor_mode_num = 15,    /*support sensor mode num*/

    .cap_delay_frame = 2,  /*3 guanjd modify for cts*/
    .pre_delay_frame = 2,  /*3 guanjd modify for cts*/
    .video_delay_frame = 3,
    .hs_video_delay_frame = 3,
    .slim_video_delay_frame = 3,
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,
    .custom5_delay_frame = 2,
    .custom6_delay_frame = 2,
    .custom7_delay_frame = 2,
    .custom8_delay_frame = 2,
    .custom9_delay_frame = 2,
    .custom10_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_6MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_CPHY, /*only concern if it's cphy*/
    .mipi_settle_delay_mode = 0,
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
    //.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_UYVY,
    .mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
    .mipi_lane_num = SENSOR_MIPI_3_LANE,
    .i2c_addr_table = {0x20, 0xff},
    /* record sensor support all write id addr,
     * only supprt 4 must end with 0xff
     */
    .i2c_speed = 1000, /* i2c read/write speed */
};



/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
    {6560, 4928, 0,  16, 6560, 4896, 3280, 2448,  8,   0, 3264, 2448,  0,  0, 3264, 2448}, /* Preview  reg_B3*/
    {6560, 4928, 0,  16, 6560, 4896, 3280, 2448,  8,   0, 3264, 2448,  0,  0, 3264, 2448},/* capture  reg_B3*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* normal video reg_C3*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* hs_video  reg_G2*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* slim video reg_G2*/
    {6560, 4928, 0,   0, 6560, 4928, 1640, 1232,  4,   4, 1632, 1224,  0,  0, 1632, 1224}, /* custom1  reg_H*/
    {6560, 4928, 0,   0, 6560, 4928, 1640, 1232,  4,   4, 1632, 1224,  0,  0, 1632, 1224}, /* custom2  reg_H1*/
    {6560, 4928, 0,  16, 6560, 4896, 6560, 4896, 16,   0, 6528, 4896,  0,  0, 6528, 4896}, /* custom3  reg_A5*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* custom4  reg_I3*/
    {6560, 4928, 0,  16, 6560, 4896, 3280, 2448,  8,   0, 3264, 2448,  0,  0, 3264, 2448}, /* custom5 reg_C3*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* pre isp custom6 reg_C3*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* pre isp custom7  reg_I3*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* custom8 reg_C3*/
    {6560, 4928, 0, 608, 6560, 3712, 3280, 1856,  8,   0, 3264, 1856,  0,  0, 3264, 1856}, /* pre isp custom9  reg_I3*/
    {6560, 4928, 0,   0, 6560, 4928, 1640, 1232,  4,   4, 1632, 1224,  0,  0, 1632, 1224}, /* custom10  reg_H1*/
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 16,
	.i4OffsetY = 16,
	.i4PitchX  = 16,
	.i4PitchY  = 16,
	.i4PairNum  = 16,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = { {18, 17}, {26, 17}, {16, 19}, {24, 19},
                {20, 21}, {28, 21}, {22, 23}, {30, 23},
                {18, 25}, {26, 25}, {16, 27}, {24, 27},
                {20, 29}, {28, 29}, {22, 31}, {30, 31} },
	.i4PosR = { {19, 17}, {27, 17}, {17, 19}, {25, 19},
                {21, 21}, {29, 21}, {23, 23}, {31, 23},
                {19, 25}, {27, 25}, {17, 27}, {25, 27},
                {21, 29}, {29, 29}, {23, 31}, {31, 31} },
	.i4BlockNumX = 202,
	.i4BlockNumY = 152,
	.iMirrorFlip = 3,
	.i4Crop = { {8, 8}, {8, 8}, {8, 304}, {0, 0}, {0, 0},
		        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
                {0, 0}, {0, 0},{0, 0}, {0, 0}, {824, 620}},
};

static void commit_write_sensor(struct subdrv_ctx *ctx)
{
	if (_size_to_write) {
		imx709_table_write_cmos_sensor(ctx, _i2c_data, _size_to_write);
		memset(_i2c_data, 0x0, sizeof(_i2c_data));
		_size_to_write = 0;
	}
}

static void set_cmos_sensor_8(struct subdrv_ctx *ctx,
		kal_uint16 reg, kal_uint16 val)
{
	if (_size_to_write > _I2C_BUF_SIZE - 2)
		commit_write_sensor(ctx);

	_i2c_data[_size_to_write++] = reg;
	_i2c_data[_size_to_write++] = val;
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
    kal_uint16 get_byte = 0;

    adaptor_i2c_rd_u8(ctx->i2c_client, IMX709_EEPROM_SLAVE_ADDRESS >> 1, addr, (u8 *)&get_byte);
    return get_byte;
}

static BYTE imx709_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static void read_module_data(struct subdrv_ctx *ctx)
{
    kal_uint16 idx = 0;
    read_imx709_eeprom_info_22017(ctx, EEPROM_META_MODULE_ID,
                &(imx709_common_data[0]), 2);
    imgsensor_info.module_id = (kal_uint16)(imx709_common_data[1] << 8) |
                   imx709_common_data[0];
    read_imx709_eeprom_info_22017(ctx, EEPROM_META_SENSOR_ID,
                &(imx709_common_data[2]), 2);
    read_imx709_eeprom_info_22017(ctx, EEPROM_META_LENS_ID,
                &(imx709_common_data[4]), 2);
    read_imx709_eeprom_info_22017(ctx, EEPROM_META_VCM_ID,
                &(imx709_common_data[6]), 2);
    read_imx709_eeprom_info_22017(ctx, EEPROM_META_MODULE_SN,
                &(imx709_common_data[8]), 17);
    read_imx709_eeprom_info_22017(ctx, EEPROM_META_AF_CODE,
                &(imx709_common_data[25]), 6);
    qsc_is_valid = read_cmos_eeprom_8(ctx, 0x1F18);
    lrc_is_valid = read_cmos_eeprom_8(ctx, 0x2024);
    for (idx = 0; idx < 30; idx = idx + 4)
        LOG_INF("In %s: cam data: %02x %02x %02x %02x\n", __func__,
               imx709_common_data[idx], imx709_common_data[idx + 1],
               imx709_common_data[idx + 2],
               imx709_common_data[idx + 3]);
}

static kal_uint32 get_cur_exp_cnt(struct subdrv_ctx *ctx)
{
    kal_uint32 exp_cnt = 1;

    if (0x1 == (read_cmos_sensor_8(ctx, 0x3170) & 0x1)) { // DOL_EN
        exp_cnt = 2;
    }

    return exp_cnt;
}

static void write_frame_len(struct subdrv_ctx *ctx, kal_uint32 fll, kal_uint32 exp_cnt_value)
{
    kal_uint32  exp_cnt = get_cur_exp_cnt(ctx);
    ctx->frame_length = round_up(fll / exp_cnt, 4) * exp_cnt;
    LOG_INF("fll %d exp_cnt %d, extend %d, fastmode %d\n", ctx->frame_length, exp_cnt,
       ctx->extend_frame_length_en, ctx->fast_mode_on);
    if (ctx->extend_frame_length_en == KAL_FALSE) {
        write_cmos_sensor_8(ctx, 0x0340, (ctx->frame_length /exp_cnt) >> 8);
        write_cmos_sensor_8(ctx, 0x0341, (ctx->frame_length /exp_cnt) & 0xFF);
    }
    if (ctx->fast_mode_on == KAL_TRUE) {
        write_cmos_sensor_8(ctx, 0x3010, 0x00);
        ctx->fast_mode_on = KAL_FALSE;
    }
}

static void set_dummy(struct subdrv_ctx *ctx)
{
    LOG_INF("dummyline = %d, dummypixels = %d, frame_length:%d, line_length:%d\n",
        ctx->dummy_line, ctx->dummy_pixel, ctx->frame_length, ctx->line_length);
    /* return;*/ /* for test */
    if (ctx->shutter > ctx->min_frame_length - imgsensor_info.margin)
        ctx->frame_length = ctx->shutter + imgsensor_info.margin;
    write_frame_len(ctx, ctx->frame_length, 1);

}    /*    set_dummy  */

static void set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror)
{
    kal_uint8 itemp;

    LOG_INF("image_mirror = %d\n", image_mirror);
    itemp = read_cmos_sensor_8(ctx, 0x0101);
    itemp &= ~0x03;

    switch (image_mirror) {

    case IMAGE_NORMAL:
    write_cmos_sensor_8(ctx, 0x0101, itemp);
    break;

    case IMAGE_V_MIRROR:
    write_cmos_sensor_8(ctx, 0x0101, itemp | 0x02);
    break;

    case IMAGE_H_MIRROR:
    write_cmos_sensor_8(ctx, 0x0101, itemp | 0x01);
    break;

    case IMAGE_HV_MIRROR:
    write_cmos_sensor_8(ctx, 0x0101, itemp | 0x03);
    break;
    }
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
    /*kal_int16 dummy_line;*/
    kal_uint32 frame_length = ctx->frame_length;

    LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
        min_framelength_en);

    frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
    if (frame_length >= ctx->min_frame_length)
        ctx->frame_length = frame_length;
    else
        ctx->frame_length = ctx->min_frame_length;

    ctx->dummy_line =
            ctx->frame_length - ctx->min_frame_length;

    if (ctx->frame_length > imgsensor_info.max_frame_length) {
        ctx->frame_length = imgsensor_info.max_frame_length;
        ctx->dummy_line =
            ctx->frame_length - ctx->min_frame_length;
    }
    if (min_framelength_en)
        ctx->min_frame_length = ctx->frame_length;
    set_dummy(ctx);
}    /*    set_max_framerate  */

static kal_bool set_auto_flicker(struct subdrv_ctx *ctx)
{
	kal_uint16 realtime_fps = 0;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10
				/ ctx->frame_length;
		LOG_INF("autoflicker enable, realtime_fps = %d\n",
			realtime_fps);
		if (realtime_fps >= 593 && realtime_fps <= 615) {
			set_max_framerate(ctx, 592, 0);
			write_frame_len(ctx, ctx->frame_length,1);
			return KAL_TRUE;
		}
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(ctx, 296, 0);
			write_frame_len(ctx, ctx->frame_length,1);
			return KAL_TRUE;
		}
		if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(ctx, 146, 0);
			write_frame_len(ctx, ctx->frame_length,1);
			return KAL_TRUE;
		}
	}

	return KAL_FALSE;
}

static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    kal_uint16 i= 0;
    if (shutter > ctx->min_frame_length - imgsensor_info.margin)
        ctx->frame_length = shutter + imgsensor_info.margin;
    else
        ctx->frame_length = ctx->min_frame_length;
    if (ctx->frame_length > imgsensor_info.max_frame_length)
        ctx->frame_length = imgsensor_info.max_frame_length;
    if (shutter < imgsensor_info.min_shutter)
        shutter = imgsensor_info.min_shutter;

    for (i = 0; i < previous_exp_cnt; i++)
        previous_exp[i] = 0;
    previous_exp[0] = shutter;
    previous_exp_cnt = 1;

    if (gph) {
        write_cmos_sensor_8(ctx, 0x0104, 0x01);
    }

    if (ctx->autoflicker_en) {
        realtime_fps = ctx->pclk / ctx->line_length * 10
                / ctx->frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(ctx, 296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(ctx, 146, 0);
    } else {
        LOG_INF("(else)ctx->frame_length = %d\n",
            ctx->frame_length);
    }
    write_frame_len(ctx, ctx->frame_length, 1);

    /* Update Shutter */
    LOG_INF("fast_mode_on %d", ctx->fast_mode_on);
    if (ctx->fast_mode_on == KAL_TRUE) {
        write_cmos_sensor_8(ctx, 0x3010, 0x00);
        ctx->fast_mode_on = KAL_FALSE;
    }
    write_cmos_sensor_8(ctx, 0x0350, 0x01); /* Enable auto extend */
    write_cmos_sensor_8(ctx, 0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(ctx, 0x0203, shutter  & 0xFF);
    if (gph) {
        write_cmos_sensor_8(ctx, 0x0104, 0x00);
    }
    LOG_INF("shutter =%d, framelength =%d \n", shutter, ctx->frame_length);
}    /*    write_shutter  */

/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter, bool gph)
{
    ctx->shutter = shutter;

    write_shutter(ctx, shutter, gph);
} /* set_shutter */

/*************************************************************************
 * FUNCTION
 *    set_shutter_frame_length
 *
 * DESCRIPTION
 *    for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(struct subdrv_ctx *ctx, kal_uint16 shutter,
                     kal_uint16 frame_length)
{    kal_uint16 realtime_fps = 0;
    kal_int32 dummy_line = 0;

    ctx->shutter = shutter;

    /*0x3500, 0x3501, 0x3502 will increase VBLANK to
     *get exposure larger than frame exposure
     *AE doesn't update sensor gain at capture mode,
     *thus extra exposure lines must be updated here.
     */

    /* OV Recommend Solution */
    /*if shutter bigger than frame_length,
     *should extend frame length first
     */
    /* Change frame time */
    if (frame_length > 1)
        dummy_line = frame_length - ctx->frame_length;

    ctx->frame_length = ctx->frame_length + dummy_line;

    if (shutter > ctx->frame_length - imgsensor_info.margin)
        ctx->frame_length = shutter + imgsensor_info.margin;

    if (ctx->frame_length > imgsensor_info.max_frame_length)
        ctx->frame_length = imgsensor_info.max_frame_length;
    shutter = (shutter < imgsensor_info.min_shutter)
            ? imgsensor_info.min_shutter : shutter;
    shutter =
    (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
        ? (imgsensor_info.max_frame_length - imgsensor_info.margin)
        : shutter;
    write_cmos_sensor_8(ctx, 0x0104, 0x01);
    if (ctx->autoflicker_en) {
        realtime_fps = ctx->pclk / ctx->line_length * 10 /
                ctx->frame_length;
        if (realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(ctx, 296, 0);
        else if (realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(ctx, 146, 0);
        else {
            /* Extend frame length */

            write_cmos_sensor_8(ctx, 0x0340,
                    ctx->frame_length >> 8);
            write_cmos_sensor_8(ctx, 0x0341,
                    ctx->frame_length & 0xFF);
        }
    } else {
        /* Extend frame length */
        write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
        write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
    }

    /* Update Shutter */
    write_cmos_sensor_8(ctx, 0x0350, 0x00); /* Disable auto extend */
    write_cmos_sensor_8(ctx, 0x0202, (shutter >> 8) & 0xFF);
    write_cmos_sensor_8(ctx, 0x0203, shutter  & 0xFF);
    write_cmos_sensor_8(ctx, 0x0104, 0x00);
    LOG_INF(
        "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
        shutter, ctx->frame_length, frame_length,
        dummy_line, read_cmos_sensor(ctx, 0x0350));

}    /* set_shutter_frame_length */

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint16 gain)
{
     kal_uint16 reg_gain = 0x0;

    reg_gain = 16384 - (16384 * BASEGAIN) / gain;
    return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(struct subdrv_ctx *ctx, kal_uint16 gain, kal_bool gph)
{
    kal_uint16 reg_gain, max_gain = imgsensor_info.max_gain;

    if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3) {
        max_gain = 16 * BASEGAIN; //set 16x gain for full size mode.
    }

    if (gain < imgsensor_info.min_gain || gain > max_gain) {
        LOG_INF("Error gain setting");

        if (gain < imgsensor_info.min_gain)
            gain = imgsensor_info.min_gain;
        else if (gain > max_gain)
            gain = max_gain;
    }

    reg_gain = gain2reg(ctx, gain);
    ctx->gain = reg_gain;
    LOG_INF("gain = %d, reg_gain = 0x%x\n", gain, reg_gain);
    if (gph) {
        write_cmos_sensor_8(ctx, 0x0104, 0x01);
    }
    write_cmos_sensor_8(ctx, 0x0204, (reg_gain>>8) & 0xFF);
    write_cmos_sensor_8(ctx, 0x0205, reg_gain & 0xFF);
    if (gph) {
        write_cmos_sensor_8(ctx, 0x0104, 0x00);
    }

    return gain;
} /* set_gain */

/*************************************************************************
 * FUNCTION
 *    night_mode
 *
 * DESCRIPTION
 *    This function night mode of sensor.
 *
 * PARAMETERS
 *    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n",
        enable);
    if (enable) {
        //AI Video mode delay sensor stream on timing
        if (ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6 ||
            ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM7) {
            /* msleep(50); */
        }
        write_cmos_sensor_8(ctx, 0x0100, 0X01);
        write_cmos_sensor_8(ctx, 0x0601, 0X00);
        ctx->test_pattern = 0;
    } else {
        write_cmos_sensor_8(ctx, 0x0100, 0x00);
    }
    return ERROR_NONE;
}

static kal_uint16 imx709_rmsc_ip_setting[] = {
    //RMSC_IP_reg_210628
    0xD400, 0x00,
    0xD401, 0x00,
    0xD402, 0x00,
    0xD403, 0x00,
    0xD404, 0x00,
    0xD405, 0x00,
    0xD406, 0x00,
    0xD407, 0x00,
    0xD408, 0x00,
    0xD409, 0x00,
    0xD40A, 0x00,
    0xD40B, 0x00,
    0xD40C, 0x00,
    0xD40D, 0x00,
    0xD40E, 0x00,
    0xD40F, 0x00,
    0xD410, 0x00,
    0xD411, 0x00,
    0xD412, 0x00,
    0xD413, 0x00,
    0xD414, 0x00,
    0xD415, 0x00,
    0xD416, 0x00,
    0xD417, 0x00,
    0x393C, 0x00,
    0x393D, 0x00,
    0x393E, 0x04,
    0x393F, 0x00,
    0xD414, 0x00,
    0xD415, 0x00,
    0xD416, 0x00,
    0xD417, 0x00,
    0xD41C, 0x00,
    0xD41D, 0x00,
    0xD41E, 0x00,
    0xD41F, 0x00,
    0xD458, 0x0C,
    0xD459, 0x86,
    0xD45A, 0x42,
    0xD45B, 0x58,
    0xD45C, 0x04,
    0xD45D, 0xB1,
    0xD45E, 0x90,
    0xD45F, 0x96,
    0xD460, 0x03,
    0xD461, 0x00,
    0xD462, 0x60,
    0xD463, 0x32,
    0xD464, 0x0A,
    0xD465, 0x81,
    0xD466, 0xE0,
    0xD467, 0x50,
    0xD468, 0x16,
    0xD469, 0x84,
    0xD46A, 0x80,
    0xD46B, 0xE0,
    0xD46C, 0x27,
    0xD46D, 0x18,
    0xD46E, 0x41,
    0xD46F, 0xB8,
    0xD470, 0x3C,
    0xD471, 0x0D,
    0xD472, 0x22,
    0xD473, 0xD8,
    0x3900, 0x19,
    0x3901, 0x01,
    0x3902, 0x93,
    0x3903, 0xF7,
    0x3904, 0x03,
    0x3905, 0x20,
    0x3906, 0xC8,
    0x3907, 0x64,
    0x3908, 0x09,
    0x3909, 0x61,
    0x390A, 0x90,
    0x390B, 0xC8,
    0x390C, 0x01,
    0x390D, 0xE0,
    0x390E, 0xC9,
    0x390F, 0xF4,
    0x3910, 0x01,
    0x3911, 0xE0,
    0x3912, 0x78,
    0x3913, 0x1E,
    0x3914, 0x03,
    0x3915, 0x29,
    0x3916, 0x62,
    0x3917, 0x58,
    0x3918, 0x00,
    0x3919, 0x1E,
    0x391A, 0x00,
    0x391B, 0x1E,
    0x391C, 0x00,
    0x391D, 0x1E,
    0x391E, 0x00,
    0x391F, 0x1E,
    0x3920, 0x01,
    0x3921, 0xF4,
    0x3922, 0x01,
    0x3923, 0xF4,
    0x3924, 0x01,
    0x3925, 0xF4,
    0x3926, 0x01,
    0x3927, 0xF4,
    0x3928, 0x3F,
    0x3929, 0xA7,
    0x392A, 0xD3,
    0x392B, 0x84,
    0x392C, 0x07,
    0x392D, 0x8F,
    0x392E, 0xCA,
    0x392F, 0xEE,
    0x3930, 0x05,
    0x3931, 0xA0,
    0x3932, 0x78,
    0x3933, 0x78,
    0x3934, 0x05,
    0x3935, 0x01,
    0x3936, 0x40,
    0x3937, 0x50,
    0xD47C, 0x01,
    0xD47D, 0x60,
    0xD47E, 0x58,
    0xD47F, 0x34,
    0xD480, 0x00,
    0xD481, 0x80,
    0xD482, 0x20,
    0xD483, 0x14,
    0xD484, 0x01,
    0xD485, 0x80,
    0xD486, 0x60,
    0xD487, 0x20,
    0xD488, 0x02,
    0xD489, 0x40,
    0xD48A, 0x40,
    0xD48B, 0x20,
    0xD48C, 0x01,
    0xD48D, 0x40,
    0xD48E, 0x18,
    0xD48F, 0x28,
    0xD490, 0x01,
    0xD491, 0x40,
    0xD492, 0x50,
    0xD493, 0x12,
    0xD494, 0x01,
    0xD495, 0x40,
    0xD496, 0x30,
    0xD497, 0x12,
    0xD498, 0x00,
    0xD499, 0xC0,
    0xD49A, 0x30,
    0xD49B, 0x12,
    0xD49C, 0x00,
    0xD49D, 0xC0,
    0xD49E, 0x30,
    0xD49F, 0x12,
    0x3938, 0x01,
    0x3939, 0x2C,
    0x393A, 0x07,
    0x393B, 0xD0,
    0x3948, 0x00,
    0x3949, 0x00,
    0x394A, 0x03,
    0x394B, 0xD7,
    0x3960, 0x00,
    0x3961, 0x01,
    0x3962, 0x00,
    0x3963, 0x36,
    0x3964, 0x00,
    0x3965, 0x01,
    0x3966, 0x00,
    0x3967, 0x36,
    0x3954, 0x00,
    0x3955, 0x02,
    0x3956, 0x00,
    0x3957, 0xCC,
    0x3958, 0x00,
    0x3959, 0x02,
    0x395A, 0x00,
    0x395B, 0xCC,
    0xD500, 0x00,
    0xD501, 0x00,
    0xD502, 0x00,
    0xD503, 0x00,
    0xD504, 0x00,
    0xD505, 0x00,
    0xD506, 0x00,
    0xD507, 0x00,
    0xD508, 0x00,
    0xD509, 0x00,
    0xD50A, 0x00,
    0xD50B, 0x00,
    0xD50C, 0x00,
    0xD50D, 0x01,
    0xD50E, 0x21,
    0xD50F, 0x00,
    0xD510, 0x24,
    0xD511, 0x21,
    0xD512, 0x00,
    0xD513, 0x00,
    0xD514, 0x21,
    0xD515, 0x00,
    0xD516, 0x00,
    0xD517, 0x01,
    0xD518, 0x00,
    0xD519, 0x01,
    0xD51A, 0x24,
    0xD51B, 0x84,
    0xD51C, 0x24,
    0xD51D, 0x88,
    0xD51E, 0x84,
    0xD51F, 0x20,
    0xD520, 0x84,
    0xD521, 0x21,
    0xD522, 0x00,
    0xD523, 0x00,
    0xD524, 0x00,
    0xD525, 0x00,
    0xD526, 0x01,
    0xD527, 0x24,
    0xD528, 0x00,
    0xD529, 0x01,
    0xD52A, 0x24,
    0xD52B, 0x21,
    0xD52C, 0x01,
    0xD52D, 0x21,
    0xD52E, 0x00,
    0xD52F, 0x00,
    0xD530, 0x00,
    0xD531, 0x00,
    0xD532, 0x00,
    0xD533, 0x00,
    0xD534, 0x00,
    0xD535, 0x00,
    0xD536, 0x00,
    0xD537, 0x00,
    0xD538, 0x00,
    0xD539, 0x00,
    0xD53A, 0x00,
    0xD53B, 0x00,
    0xD53C, 0x00,
    0xD53D, 0x00,
    0xD53E, 0x00,
    0xD53F, 0x00,
    0xD540, 0x00,
    0xD541, 0x00,
    0xD542, 0x00,
    0xD543, 0x00,
    0xD544, 0x11,
    0xD545, 0x11,
    0xD546, 0x00,
    0xD547, 0x00,
    0xD548, 0x10,
    0xD549, 0x02,
    0xD54A, 0x22,
    0xD54B, 0x00,
    0xD54C, 0x28,
    0xD54D, 0x20,
    0xD54E, 0x01,
    0xD54F, 0x44,
    0xD550, 0x00,
    0xD551, 0x14,
    0xD552, 0x41,
    0xD553, 0x00,
    0xD554, 0x11,
    0xD555, 0x10,
    0xD556, 0x02,
    0xD557, 0x22,
    0xD558, 0x00,
    0xD559, 0x00,
    0xD55A, 0x00,
    0xD55B, 0x01,
    0xD55C, 0x00,
    0xD55D, 0x00,
    0xD55E, 0x00,
    0xD55F, 0x00,
    0xD560, 0x00,
    0xD561, 0x00,
    0xD562, 0x00,
    0xD563, 0x00,
    0xD564, 0x00,
    0xD565, 0x00,
    0xD566, 0x00,
    0xD567, 0x00,
    0xD568, 0x00,
    0xD569, 0x01,
    0xD56A, 0x11,
    0xD56B, 0x00,
    0xD56C, 0x14,
    0xD56D, 0x10,
    0xD56E, 0x00,
    0xD56F, 0x22,
    0xD570, 0x00,
    0xD571, 0x02,
    0xD572, 0x20,
    0xD573, 0x00,
    0xD574, 0x00,
    0xD575, 0x00,
    0xD576, 0x01,
    0xD577, 0x11,
    0xD578, 0x00,
    0xD579, 0x00,
    0xD57A, 0x00,
    0xD57B, 0x00,
    0xD57C, 0x00,
    0xD57D, 0x00,
    0xD57E, 0x00,
    0xD57F, 0x00,
    0xD580, 0x00,
    0xD581, 0x00,
    0xD582, 0x00,
    0xD583, 0x00,
    0xD584, 0x00,
    0xD585, 0x10,
    0xD586, 0x00,
    0xD587, 0x00,
    0xD588, 0x10,
    0xD589, 0x00,
    0xD58A, 0x22,
    0xD58B, 0x00,
    0xD58C, 0x28,
    0xD58D, 0x20,
    0xD58E, 0x00,
    0xD58F, 0x44,
    0xD590, 0x00,
    0xD591, 0x14,
    0xD592, 0x40,
    0xD593, 0x00,
    0xD594, 0x10,
    0xD595, 0x00,
    0xD596, 0x02,
    0xD597, 0x20,
    0xD598, 0x00,
    0xD599, 0x00,
    0xD59A, 0x00,
    0xD59B, 0x00,
    0xD59C, 0x00,
    0xD59D, 0x00,
    0xD59E, 0x00,
    0xD59F, 0x00,
    0xD5A0, 0x00,
    0xD5A1, 0x00,
    0xD5A2, 0x00,
    0xD5A3, 0x00,
    0xD5A4, 0x00,
    0xD5A5, 0x00,
    0xD5A6, 0x00,
    0xD5A7, 0x00,
    0xD5A8, 0x00,
    0xD5A9, 0x02,
    0xD5AA, 0x20,
    0xD5AB, 0x00,
    0xD5AC, 0x28,
    0xD5AD, 0x20,
    0xD5AE, 0x00,
    0xD5AF, 0x42,
    0xD5B0, 0x00,
    0xD5B1, 0x02,
    0xD5B2, 0x40,
    0xD5B3, 0x00,
    0xD5B4, 0x00,
    0xD5B5, 0x00,
    0xD5B6, 0x00,
    0xD5B7, 0x22,
    0xD5B8, 0x00,
    0xD5B9, 0x00,
    0xD5BA, 0x00,
    0xD5BB, 0x00,
    0xD5BC, 0x00,
    0xD5BD, 0x00,
    0xD5BE, 0x00,
    0xD5BF, 0x00,
    0xD5C0, 0x00,
    0xD5C1, 0x00,
    0xD5C2, 0x00,
    0xD5C3, 0x00,
    0xD5C4, 0x00,
    0xD5C5, 0x00,
    0xD5C6, 0x00,
    0xD5C7, 0x00,
    0xD5C8, 0x00,
    0xD5C9, 0x02,
    0xD5CA, 0x20,
    0xD5CB, 0x00,
    0xD5CC, 0x28,
    0xD5CD, 0x20,
    0xD5CE, 0x00,
    0xD5CF, 0x42,
    0xD5D0, 0x00,
    0xD5D1, 0x02,
    0xD5D2, 0x40,
    0xD5D3, 0x00,
    0xD5D4, 0x00,
    0xD5D5, 0x00,
    0xD5D6, 0x00,
    0xD5D7, 0x22,
    0xD5D8, 0x00,
    0xD5D9, 0x00,
    0xD5DA, 0x00,
    0xD5DB, 0x00,
    0xD5DC, 0x00,
    0xD5DD, 0x00,
    0xD5DE, 0x00,
    0xD5DF, 0x00,
    0xD5E0, 0x00,
    0xD5E1, 0x00,
    0xD5E2, 0x00,
    0xD5E3, 0x00,
    0xD5E4, 0x01,
    0xD5E5, 0x10,
    0xD5E6, 0x00,
    0xD5E7, 0x00,
    0xD5E8, 0x00,
    0xD5E9, 0x00,
    0xD5EA, 0x20,
    0xD5EB, 0x00,
    0xD5EC, 0x14,
    0xD5ED, 0x10,
    0xD5EE, 0x00,
    0xD5EF, 0x20,
    0xD5F0, 0x00,
    0xD5F1, 0x00,
    0xD5F2, 0x20,
    0xD5F3, 0x00,
    0xD5F4, 0x11,
    0xD5F5, 0x00,
    0xD5F6, 0x00,
    0xD5F7, 0x20,
    0xD5F8, 0x00,
    0xD5F9, 0x00,
    0xD5FA, 0x00,
    0xD5FB, 0x00,
    0xD5FC, 0x00,
    0xD5FD, 0x00,
    0xD5FE, 0x00,
    0xD5FF, 0x00,
    0xD600, 0x00,
    0xD601, 0x00,
    0xD602, 0x00,
    0xD603, 0x00,
    0xD604, 0x00,
    0xD605, 0x00,
    0xD606, 0x00,
    0xD607, 0x00,
    0xD608, 0x10,
    0xD609, 0x00,
    0xD60A, 0x10,
    0xD60B, 0x00,
    0xD60C, 0x24,
    0xD60D, 0x20,
    0xD60E, 0x01,
    0xD60F, 0x20,
    0xD610, 0x00,
    0xD611, 0x10,
    0xD612, 0x21,
    0xD613, 0x00,
    0xD614, 0x00,
    0xD615, 0x00,
    0xD616, 0x00,
    0xD617, 0x10,
    0xD618, 0x00,
    0xD619, 0x00,
    0xD61A, 0x00,
    0xD61B, 0x00,
    0xD61C, 0x00,
    0xD61D, 0x00,
    0xD61E, 0x00,
    0xD61F, 0x00,
    0xD620, 0x00,
    0xD621, 0x00,
    0xD622, 0x00,
    0xD623, 0x00,
    0xD624, 0x00,
    0xD625, 0x00,
    0xD626, 0x00,
    0xD627, 0x00,
    0xD628, 0x10,
    0xD629, 0x00,
    0xD62A, 0x10,
    0xD62B, 0x00,
    0xD62C, 0x24,
    0xD62D, 0x20,
    0xD62E, 0x01,
    0xD62F, 0x02,
    0xD630, 0x00,
    0xD631, 0x12,
    0xD632, 0x01,
    0xD633, 0x00,
    0xD634, 0x00,
    0xD635, 0x00,
    0xD636, 0x00,
    0xD637, 0x10,
    0xD638, 0x00,
    0xD639, 0x00,
    0xD63A, 0x00,
    0xD63B, 0x00,
    0xD63C, 0x00,
    0xD63D, 0x00,
    0xD63E, 0x00,
    0xD63F, 0x00,
    0xD640, 0x00,
    0xD641, 0x00,
    0xD642, 0x00,
    0xD643, 0x00,
    0xD644, 0x01,
    0xD645, 0x10,
    0xD646, 0x00,
    0xD647, 0x00,
    0xD648, 0x00,
    0xD649, 0x00,
    0xD64A, 0x20,
    0xD64B, 0x00,
    0xD64C, 0x14,
    0xD64D, 0x10,
    0xD64E, 0x00,
    0xD64F, 0x02,
    0xD650, 0x00,
    0xD651, 0x02,
    0xD652, 0x00,
    0xD653, 0x00,
    0xD654, 0x11,
    0xD655, 0x00,
    0xD656, 0x00,
    0xD657, 0x20,
    0xD658, 0x00,
    0xD659, 0x00,
    0xD65A, 0x00,
    0xD65B, 0x00,
    0xD65C, 0x00,
    0xD65D, 0x00,
    0xD65E, 0x00,
    0xD65F, 0x00,
    0xD660, 0x11,
    0xD661, 0x00,
    0xD662, 0x00,
    0xD663, 0x00,
    0xD664, 0x40,
    0xD665, 0x04,
    0xD666, 0x00,
    0xD667, 0x21,
    0xD668, 0x10,
    0xD669, 0x02,
    0xD66A, 0x12,
    0xD66B, 0x21,
    0xD66C, 0x00,
    0xD66D, 0x00,
    0xD66E, 0x00,
    0xD66F, 0x00,
    0xD670, 0x01,
    0xD671, 0x00,
    0xD672, 0x00,
    0xD673, 0x00,
    0xD674, 0x04,
    0xD675, 0x12,
    0xD676, 0x21,
    0xD677, 0x20,
    0xD678, 0x11,
    0xD679, 0x12,
    0xD67A, 0x00,
    0xD67B, 0x40,
    0xD67C, 0x00,
    0xD67D, 0x00,
    0xD67E, 0x00,
    0xD67F, 0x00,
    0xD680, 0x00,
    0xD681, 0x00,
    0xD682, 0x00,
    0xD683, 0x00,
    0xD684, 0x20,
    0xD685, 0x02,
    0xD686, 0x00,
    0xD687, 0x10,
    0xD688, 0x00,
    0xD689, 0x01,
    0xD68A, 0x01,
    0xD68B, 0x10,
    0xD68C, 0x00,
    0xD68D, 0x00,
    0xD68E, 0x00,
    0xD68F, 0x00,
    0xD690, 0x00,
    0xD691, 0x00,
    0xD692, 0x00,
    0xD693, 0x00,
    0xD694, 0x02,
    0xD695, 0x01,
    0xD696, 0x10,
    0xD697, 0x10,
    0xD698, 0x00,
    0xD699, 0x01,
    0xD69A, 0x00,
    0xD69B, 0x20,
    0xD69C, 0x00,
    0xD69D, 0x00,
    0xD69E, 0x00,
    0xD69F, 0x00,
    0xD6A0, 0x00,
    0xD6A1, 0x00,
    0xD6A2, 0x00,
    0xD6A3, 0x00,
    0xD6A4, 0x20,
    0xD6A5, 0x02,
    0xD6A6, 0x00,
    0xD6A7, 0x00,
    0xD6A8, 0x00,
    0xD6A9, 0x01,
    0xD6AA, 0x01,
    0xD6AB, 0x00,
    0xD6AC, 0x00,
    0xD6AD, 0x00,
    0xD6AE, 0x00,
    0xD6AF, 0x00,
    0xD6B0, 0x01,
    0xD6B1, 0x00,
    0xD6B2, 0x00,
    0xD6B3, 0x00,
    0xD6B4, 0x04,
    0xD6B5, 0x12,
    0xD6B6, 0x21,
    0xD6B7, 0x20,
    0xD6B8, 0x01,
    0xD6B9, 0x02,
    0xD6BA, 0x00,
    0xD6BB, 0x40,
    0xD6BC, 0x00,
    0xD6BD, 0x00,
    0xD6BE, 0x00,
    0xD6BF, 0x00,
    0xD6C0, 0x00,
    0xD6C1, 0x00,
    0xD6C2, 0x00,
    0xD6C3, 0x00,
    0xD6C4, 0x40,
    0xD6C5, 0x04,
    0xD6C6, 0x00,
    0xD6C7, 0x20,
    0xD6C8, 0x20,
    0xD6C9, 0x00,
    0xD6CA, 0x00,
    0xD6CB, 0x22,
    0xD6CC, 0x00,
    0xD6CD, 0x00,
    0xD6CE, 0x00,
    0xD6CF, 0x00,
    0xD6D0, 0x04,
    0xD6D1, 0x00,
    0xD6D2, 0x00,
    0xD6D3, 0x00,
    0xD6D4, 0x08,
    0xD6D5, 0x44,
    0xD6D6, 0x02,
    0xD6D7, 0x00,
    0xD6D8, 0x02,
    0xD6D9, 0x04,
    0xD6DA, 0x00,
    0xD6DB, 0x80,
    0xD6DC, 0x00,
    0xD6DD, 0x00,
    0xD6DE, 0x00,
    0xD6DF, 0x00,
    0xD6E0, 0x00,
    0xD6E1, 0x00,
    0xD6E2, 0x00,
    0xD6E3, 0x00,
    0xD6E4, 0x40,
    0xD6E5, 0x04,
    0xD6E6, 0x00,
    0xD6E7, 0x20,
    0xD6E8, 0x00,
    0xD6E9, 0x00,
    0xD6EA, 0x00,
    0xD6EB, 0x20,
    0xD6EC, 0x00,
    0xD6ED, 0x00,
    0xD6EE, 0x00,
    0xD6EF, 0x00,
    0xD6F0, 0x00,
    0xD6F1, 0x00,
    0xD6F2, 0x00,
    0xD6F3, 0x00,
    0xD6F4, 0x04,
    0xD6F5, 0x02,
    0xD6F6, 0x00,
    0xD6F7, 0x00,
    0xD6F8, 0x00,
    0xD6F9, 0x02,
    0xD6FA, 0x00,
    0xD6FB, 0x40,
    0xD6FC, 0x00,
    0xD6FD, 0x00,
    0xD6FE, 0x00,
    0xD6FF, 0x00,
    0xD700, 0x00,
    0xD701, 0x00,
    0xD702, 0x00,
    0xD703, 0x00,
    0xD704, 0x00,
    0xD705, 0x04,
    0xD706, 0x00,
    0xD707, 0x40,
    0xD708, 0x20,
    0xD709, 0x00,
    0xD70A, 0x00,
    0xD70B, 0x20,
    0xD70C, 0x00,
    0xD70D, 0x00,
    0xD70E, 0x00,
    0xD70F, 0x00,
    0xD710, 0x02,
    0xD711, 0x00,
    0xD712, 0x00,
    0xD713, 0x00,
    0xD714, 0x00,
    0xD715, 0x02,
    0xD716, 0x00,
    0xD717, 0x00,
    0xD718, 0x00,
    0xD719, 0x04,
    0xD71A, 0x00,
    0xD71B, 0x40,
    0xD71C, 0x00,
    0xD71D, 0x00,
    0xD71E, 0x00,
    0xD71F, 0x00,
    0xD720, 0x00,
    0xD721, 0x00,
    0xD722, 0x00,
    0xD723, 0x00,
    0xD724, 0x40,
    0xD725, 0x00,
    0xD726, 0x00,
    0xD727, 0x20,
    0xD728, 0x00,
    0xD729, 0x00,
    0xD72A, 0x00,
    0xD72B, 0x42,
    0xD72C, 0x00,
    0xD72D, 0x00,
    0xD72E, 0x00,
    0xD72F, 0x00,
    0xD730, 0x00,
    0xD731, 0x00,
    0xD732, 0x00,
    0xD733, 0x00,
    0xD734, 0x04,
    0xD735, 0x24,
    0xD736, 0x00,
    0xD737, 0x00,
    0xD738, 0x00,
    0xD739, 0x02,
    0xD73A, 0x00,
    0xD73B, 0x00,
    0xD73C, 0x00,
    0xD73D, 0x00,
    0xD73E, 0x00,
    0xD73F, 0x00,
    0xD740, 0x00,
    0xD741, 0x00,
    0xD742, 0x00,
    0xD743, 0x00,
    0xD744, 0x40,
    0xD745, 0x04,
    0xD746, 0x00,
    0xD747, 0x00,
    0xD748, 0x00,
    0xD749, 0x00,
    0xD74A, 0x02,
    0xD74B, 0x02,
    0xD74C, 0x00,
    0xD74D, 0x00,
    0xD74E, 0x00,
    0xD74F, 0x00,
    0xD750, 0x00,
    0xD751, 0x00,
    0xD752, 0x00,
    0xD753, 0x00,
    0xD754, 0x04,
    0xD755, 0x20,
    0xD756, 0x20,
    0xD757, 0x00,
    0xD758, 0x00,
    0xD759, 0x00,
    0xD75A, 0x00,
    0xD75B, 0x40,
    0xD75C, 0x00,
    0xD75D, 0x00,
    0xD75E, 0x00,
    0xD75F, 0x00,
    0xD760, 0x00,
    0xD761, 0x00,
    0xD762, 0x00,
    0xD763, 0x00,
    0xD764, 0x40,
    0xD765, 0x04,
    0xD766, 0x00,
    0xD767, 0x00,
    0xD768, 0x20,
    0xD769, 0x02,
    0xD76A, 0x00,
    0xD76B, 0x00,
    0xD76C, 0x00,
    0xD76D, 0x00,
    0xD76E, 0x00,
    0xD76F, 0x00,
    0xD770, 0x02,
    0xD771, 0x00,
    0xD772, 0x00,
    0xD773, 0x00,
    0xD774, 0x04,
    0xD775, 0x00,
    0xD776, 0x00,
    0xD777, 0x20,
    0xD778, 0x00,
    0xD779, 0x00,
    0xD77A, 0x00,
    0xD77B, 0x40,
    0xD77C, 0x00,
    0xD77D, 0x00,
    0xD77E, 0x00,
    0xD77F, 0x00,
    0xD780, 0x00,
    0xD781, 0x00,
    0xD782, 0x00,
    0xD783, 0x00,
    0xD784, 0x00,
    0xD785, 0x00,
    0xD786, 0x00,
    0xD787, 0x00,
    0xD788, 0x00,
    0xD789, 0x00,
    0xD78A, 0x00,
    0xD78B, 0x00,
    0xD78C, 0x00,
    0xD78D, 0x01,
    0xD78E, 0x21,
    0xD78F, 0x00,
    0xD790, 0x24,
    0xD791, 0x21,
    0xD792, 0x00,
    0xD793, 0x00,
    0xD794, 0x21,
    0xD795, 0x00,
    0xD796, 0x00,
    0xD797, 0x01,
    0xD798, 0x00,
    0xD799, 0x01,
    0xD79A, 0x24,
    0xD79B, 0x84,
    0xD79C, 0x24,
    0xD79D, 0x88,
    0xD79E, 0x84,
    0xD79F, 0x20,
    0xD7A0, 0x84,
    0xD7A1, 0x21,
    0xD7A2, 0x00,
    0xD7A3, 0x00,
    0xD7A4, 0x00,
    0xD7A5, 0x00,
    0xD7A6, 0x01,
    0xD7A7, 0x24,
    0xD7A8, 0x00,
    0xD7A9, 0x01,
    0xD7AA, 0x24,
    0xD7AB, 0x21,
    0xD7AC, 0x01,
    0xD7AD, 0x21,
    0xD7AE, 0x00,
    0xD7AF, 0x00,
    0xD7B0, 0x00,
    0xD7B1, 0x00,
    0xD7B2, 0x00,
    0xD7B3, 0x00,
    0xD7B4, 0x00,
    0xD7B5, 0x00,
    0xD7B6, 0x00,
    0xD7B7, 0x00,
    0xD7B8, 0x00,
    0xD7B9, 0x00,
    0xD7BA, 0x00,
    0xD7BB, 0x00,
    0xD7BC, 0x00,
    0xD7BD, 0x00,
    0xD7BE, 0x00,
    0xD7BF, 0x00,
    0xD7C0, 0x00,
    0xD7C1, 0x00,
    0xD7C2, 0x40,
    0xD7C3, 0x40,
    0xD7C4, 0x00,
    0xD7C5, 0x00,
    0xD7C6, 0x04,
    0xD7C7, 0x04,
    0xD7C8, 0x40,
    0xD7C9, 0x40,
    0xD7CA, 0x00,
    0xD7CB, 0x00,
    0xD7CC, 0x00,
    0xD7CD, 0x00,
    0xD7CE, 0x00,
    0xD7CF, 0x00,
    0xD7D0, 0x24,
    0xD7D1, 0x42,
    0xD7D2, 0x40,
    0xD7D3, 0x20,
    0xD7D4, 0x00,
    0xD7D5, 0x00,
    0xD7D6, 0x02,
    0xD7D7, 0x04,
    0x3940, 0x01,
    0x3941, 0x03,
    0x3942, 0x00,
    0x3943, 0x0A,
    0x3944, 0x01,
    0x3945, 0x03,
    0x3946, 0x00,
    0x3947, 0x0A,
    0xD7E0, 0x00,
    0xD7E1, 0x00,
    0xD7E2, 0x00,
    0xD7E3, 0x01,
    0x3A1A, 0x00,
    0x3A1B, 0x28,
    0x3A1C, 0x00,
    0x3A1D, 0x28,
    0xD7E8, 0x00,
    0xD7E9, 0x00,
    0xD7EA, 0x00,
    0xD7EB, 0x01,
    0x395C, 0x01,
    0x395D, 0x45,
    0x395E, 0x01,
    0x395F, 0xEE,
    0x394C, 0x03,
    0x394D, 0xCE,
    0x394E, 0x03,
    0x394F, 0xCE,
    0x3950, 0x00,
    0x3951, 0x01,
    0x3952, 0x00,
    0x3953, 0x01,
    0xD7FC, 0x00,
    0xD7FD, 0x00,
    0xD7FE, 0x00,
    0xD7FF, 0x01,
};

static kal_uint16 imx709_init_setting[] = {
    //stream off
    0x0100, 0x00,
    //External Clock Setting
    0x0136, 0x18,
    0x0137, 0x00,
    //PHY_VIF Setting
    0x3304, 0x00,
    //Register version
    0x33F0, 0x0C,
    0x33F1, 0x07,
    //Signaling mode setting
    0x0111, 0x02,
    //DIG_CROP_1PIX_RSHIFT_EN 1
    0x3215, 0x01,
    //Global Setting
    0x3379, 0x00,
    0x3724, 0x03,
    0x3732, 0x05,
    0x3733, 0xDC,
    0x3736, 0x09,
    0x3737, 0xC4,
    0x373A, 0x07,
    0x373B, 0xD0,
    0x373E, 0x03,
    0x373F, 0xE8,
    0x37A2, 0x8C,
    0x37B4, 0x03,
    0x37BA, 0x0F,
    0x37BB, 0xA0,
    0x37BE, 0x0D,
    0x37BF, 0xAC,
    0x37C2, 0x04,
    0x37C3, 0xB0,
    0x37C6, 0x3E,
    0x37C7, 0x80,
    0x37CA, 0x3A,
    0x37CB, 0x20,
    0x37CE, 0x03,
    0x37CF, 0x84,
    0x37D6, 0x09,
    0x37DB, 0x0F,
    0x37DC, 0x0F,
    0x37DD, 0x01,
    0x385E, 0x05,
    0x385F, 0xDC,
    0x3862, 0x09,
    0x3863, 0xC4,
    0x3866, 0x07,
    0x3867, 0xD0,
    0x386A, 0x03,
    0x386B, 0xE8,
    0x3A1B, 0x40,
    0x3A1D, 0x40,
    0x3A1E, 0x01,
    0x3A92, 0x01,
    0x3AA2, 0x01,
    0x640C, 0xFF,
    0x640D, 0xFF,
    0x6414, 0x00,
    0x6415, 0x9D,
    0x6418, 0xFF,
    0x6419, 0xFF,
    0x6420, 0x00,
    0x6421, 0x9D,
    0x6424, 0xFF,
    0x6425, 0xFF,
    0x642C, 0x00,
    0x642D, 0x9D,
    0x6430, 0xFF,
    0x6431, 0xFF,
    0x6438, 0x02,
    0x6439, 0x24,
    0x643C, 0xFF,
    0x643D, 0xFF,
    0x6444, 0x02,
    0x6445, 0x24,
    0x6454, 0xFF,
    0x6455, 0xFF,
    0x645C, 0x00,
    0x645D, 0x9D,
    0x646C, 0xFF,
    0x646D, 0xFF,
    0x6474, 0x00,
    0x6475, 0x9D,
    0x68B0, 0x00,
    0x68B2, 0x1A,
    0x68B5, 0x01,
    0x68B6, 0x00,
    0x68B8, 0x1A,
    0x69D3, 0x27,
    0x69DF, 0x27,
    0x6FD1, 0x09,
    0x6FD3, 0x09,
    0x6FD5, 0x09,
    0x6FD7, 0x09,
    0x6FD9, 0x09,
    0x7020, 0x03,
    0x710A, 0x17,
    0x710C, 0x18,
    0x73E9, 0x53,
    0x73ED, 0x50,
    0x73F1, 0x4B,
    0x73F5, 0x4D,
    0x73F9, 0x4E,
    0x741D, 0x56,
    0x7421, 0x55,
    0x7425, 0x53,
    0x7429, 0x56,
    0x742D, 0x56,
    0x7451, 0x58,
    0x7455, 0x56,
    0x7459, 0x57,
    0x745D, 0x58,
    0x7461, 0x58,
    0x7485, 0x5A,
    0x7489, 0x58,
    0x748D, 0x58,
    0x7491, 0x5B,
    0x7495, 0x5B,
    0x74B9, 0x5D,
    0x74BB, 0x5E,
    0x74BD, 0x5F,
    0x74BF, 0x60,
    0x74E0, 0x16,
    0x74E2, 0x19,
    0x74E4, 0x16,
    0x74E6, 0x17,
    0x74E8, 0x17,
    0x74FA, 0x20,
    0x74FC, 0x24,
    0x74FE, 0x1E,
    0x7500, 0x1F,
    0x7502, 0x30,
    0x7514, 0x3F,
    0x7516, 0x1E,
    0x7518, 0x19,
    0x751A, 0x1E,
    0x751C, 0x1E,
    0x752E, 0x57,
    0x7530, 0x26,
    0x7532, 0x14,
    0x7534, 0x1D,
    0x7536, 0x1D,
    0x7548, 0x53,
    0x7549, 0x19,
    0x754A, 0x52,
    0x754B, 0x53,
    0x757E, 0x07,
    0x7590, 0x0A,
    0x75AA, 0x0A,
    0x75AC, 0x05,
    0x75C4, 0x0A,
    0x75C5, 0x0D,
    0x75C6, 0x0A,
    0x75C7, 0x0A,
    0x7656, 0x28,
    0x765A, 0x14,
    0x765E, 0x05,
    0x7660, 0x05,
    0x7678, 0x0A,
    0x767A, 0x0A,
    0x7692, 0x14,
    0x7694, 0x14,
    0x76EA, 0x01,
    0x76EC, 0x01,
    0x76EE, 0x01,
    0x76F0, 0x01,
    0x76F2, 0x01,
    0x787A, 0x05,
    0x787C, 0x05,
    0x787E, 0x05,
    0x7880, 0x05,
    0x7B52, 0x14,
    0x7B53, 0x14,
    0x7B54, 0x14,
    0x7B55, 0x14,
    0x7B56, 0x14,
    0x7B57, 0x14,
    0x7B58, 0x14,
    0x7B59, 0x14,
    0x7B5A, 0x14,
    0x7B5C, 0x05,
    0x7B5E, 0x05,
    0x7B60, 0x14,
    0x7B61, 0x14,
    0x7B62, 0x14,
    0x7B63, 0x14,
    0x7B64, 0x14,
    0x7B65, 0x14,
    0x7B66, 0x14,
    0x7B67, 0x14,
    0x7B68, 0x14,
    0x7B69, 0x14,
    0x7B6A, 0x14,
    0x7B6B, 0x14,
    0x7B6C, 0x14,
    0x7B6D, 0x14,
    0x7B6E, 0x14,
    0x7B6F, 0x14,
    0x7B70, 0x14,
    0x7B71, 0x14,
    0x7B72, 0x14,
    0x7B73, 0x14,
    0x7B74, 0x14,
    0x7B76, 0x14,
    0x7B78, 0x14,
    0x7B7A, 0x14,
    0x7B7B, 0x14,
    0x7B7C, 0x14,
    0x7B7D, 0x14,
    0x7B7E, 0x14,
    0x7B7F, 0x14,
    0x7B80, 0x14,
    0x7B81, 0x14,
    0x7B82, 0x14,
    0x7B83, 0x14,
    0x7B84, 0x14,
    0x7B85, 0x14,
    0x7B86, 0x14,
    0x7B87, 0x14,
    0x7B88, 0x14,
    0x7B89, 0x14,
    0x7B8A, 0x14,
    0x7B8B, 0x14,
    0x7B8C, 0x14,
    0x7B8D, 0x14,
    0x7B8E, 0x14,
    0x7B90, 0x14,
    0x7B92, 0x14,
    0x7B94, 0x14,
    0x7B95, 0x14,
    0x7B96, 0x14,
    0x7B97, 0x14,
    0x7B98, 0x14,
    0x7B99, 0x14,
    0x7B9A, 0x14,
    0x7B9B, 0x14,
    0x7B9C, 0x14,
    0x7B9D, 0x14,
    0x7B9E, 0x14,
    0x7B9F, 0x14,
    0x7BA0, 0x14,
    0x7BA1, 0x14,
    0x7BA2, 0x14,
    0x7BA3, 0x14,
    0x7BA4, 0x14,
    0x7BA5, 0x14,
    0x7BA6, 0x14,
    0x7BA7, 0x14,
    0x7BA8, 0x14,
    0x7BAA, 0x14,
    0x7BAC, 0x14,
    0x7BAE, 0x14,
    0x7BAF, 0x14,
    0x7BB0, 0x14,
    0x7BB1, 0x14,
    0x7BB2, 0x14,
    0x7BB3, 0x14,
    0x7BB4, 0x14,
    0x7BB5, 0x14,
    0x7BB6, 0x14,
    0x7BB7, 0x14,
    0x7BB8, 0x14,
    0x7BB9, 0x14,
    0x7BBA, 0x14,
    0x7BBB, 0x14,
    0x7BBC, 0x14,
    0x7BBD, 0x14,
    0x7BBE, 0x14,
    0x7BC0, 0x14,
    0x7BC1, 0x14,
    0x7BC2, 0x14,
    0x7BC3, 0x14,
    0x7BC4, 0x14,
    0x7BC5, 0x14,
    0x7BC7, 0x14,
    0x7BC8, 0x14,
    0x7BC9, 0x14,
    0x7BCA, 0x14,
    0x7BCB, 0x14,
    0x9002, 0x0A,
    0x9003, 0x0A,
    0x9004, 0x0A,
    0x90E4, 0x01,
    0x90E7, 0x01,
    0x9200, 0x65,
    0x9201, 0xCE,
    0x9202, 0x65,
    0x9203, 0xC4,
    0x9204, 0x00,
    0x9205, 0x00,
    0x9206, 0x00,
    0x9207, 0x00,
    0x9208, 0x00,
    0x9209, 0x00,
    0x920A, 0x00,
    0x920B, 0x00,
    0x920C, 0x00,
    0x920D, 0x00,
    0x920E, 0x00,
    0x920F, 0x00,
    0x9210, 0x00,
    0x9211, 0x00,
    0x9212, 0x00,
    0x9213, 0x00,
    0x9214, 0x00,
    0x9215, 0x00,
    0x9216, 0x00,
    0x9217, 0x00,
    0x9218, 0x00,
    0x9219, 0x00,
    0x921A, 0x00,
    0x921B, 0x00,
    0x921C, 0x00,
    0x921D, 0x00,
    0x921E, 0x00,
    0x921F, 0x00,
    0x9220, 0x00,
    0x9221, 0x00,
    0x9222, 0x00,
    0x9223, 0x00,
    0x9224, 0x00,
    0x9225, 0x00,
    0x9226, 0x00,
    0x9227, 0x00,
    0x9228, 0x00,
    0x9229, 0x00,
    0x922A, 0x00,
    0x922B, 0x00,
    0x922C, 0x00,
    0x922D, 0x00,
    0x922E, 0x00,
    0x922F, 0x00,
    0x9230, 0x00,
    0x9231, 0x00,
    0x9232, 0x00,
    0x9233, 0x00,
    0x9234, 0xD7,
    0x9235, 0xE4,
    0x9236, 0xD7,
    0x9237, 0xE5,
    0x9238, 0xD7,
    0x9239, 0xE6,
    0x923A, 0xD7,
    0x923B, 0xE7,
    0x923C, 0x15,
    0x923D, 0x67,
    0xB0F9, 0x06,
    0xBC51, 0xC8,
    0xBCAF, 0x01,
    0xBD27, 0x28,
    0xBDC6, 0x00,
    0xBDC7, 0x00,
    0xBDC8, 0x00,
    0xBDC9, 0x00,
    0xBDCA, 0x00,
    0xBDCB, 0x00,
    0xBDCC, 0x00,
    0xBDCD, 0x00,
    0xBDCE, 0x00,
    0xBDCF, 0x00,
    0xBDD0, 0x10,
    0xBDD1, 0xDC,
    0xBDD2, 0x08,
    0xBDD3, 0x58,
    0xBDD4, 0x17,
    0xBDD5, 0xDC,
    0xBDD6, 0x0B,
    0xBDD7, 0x80,
    0xBDE0, 0x00,
    0xBDE1, 0x00,
    0xBDE2, 0x00,
    0xBDE3, 0x00,
    0xBDE4, 0x00,
    0xBDE5, 0xE0,
    0xBDE7, 0xC8,
    0xBDE8, 0x00,
    0xBDEA, 0x01,
    0xBDEB, 0xC8,
    0xC0D9, 0x12,
    0xC0DB, 0x28,
    0xC0DE, 0x28,
    0xC0E1, 0x2D,
    0xC0E3, 0x00,
    0xC0F2, 0x18,
    0xC0F3, 0x18,
    0xC546, 0x0A,
    0xC547, 0x08,
    0xC548, 0x28,
    0xC54B, 0x28,
    0xC54E, 0x3C,
    0xC551, 0x01,
    0xC55E, 0x18,
    0xC55F, 0x18,
    0xE981, 0x02,
    0xE982, 0x05,
    0xE983, 0x33,
    0xE984, 0x19,
    0xE985, 0x13,
    0xE986, 0x11,
    0xE987, 0x10,
    0xE98A, 0x01,
    0xEB00, 0x18,
    0xEB01, 0x0B,
    0xEB02, 0x04,
    //OB Setting
    0x3AC8, 0x02,
    0x3AC9, 0x2F,
    0x3ACA, 0x02,
    0x3ACB, 0x08,
    0x3A90, 0x00,
    0x3A91, 0x78,
    0x3A94, 0x01,
    0x3A95, 0xAE,
    // Image Quality
    0x9412, 0x03,
    0x9413, 0x02,
    0x9414, 0x02,
    0x950F, 0x8F,
    0x9533, 0x5F,
    0x953D, 0x9F,
    0x953F, 0xBF,
    0x9541, 0xFF,
    0x9AE7, 0x11,
    0x9AE9, 0x0E,
    0x9AED, 0x02,
    0x9AEF, 0x02,
    0x9AF9, 0x05,
    0x9AFB, 0x04,
    0x9B0B, 0x02,
    0x9B10, 0x02,
    0x9B15, 0x02,
    0x9B1A, 0x02,
    0x9B84, 0x1C,
    0x9B86, 0x06,
    0x9B87, 0x30,
    0x9B89, 0x0C,
    0x9B8A, 0x20,
    0x9B8C, 0x06,
    0x9B90, 0x24,
    0x9C01, 0x10,
    0x9C05, 0x1C,
    0x9C07, 0x06,
    0x9C0A, 0x0C,
    0x9C0E, 0x01,
    0x9C24, 0x00,
    0x9C3A, 0x00,
    0x9C50, 0x00,
    0x9C66, 0x00,
    0x9F00, 0xEF,
    0x9F01, 0xEF,
    0x9F02, 0xEF,
    0x9F03, 0xBF,
    0x9F04, 0xBF,
    0x9F05, 0xBF,
    0x9F06, 0xEF,
    0x9F07, 0xEF,
    0x9F08, 0xEF,
    0x9F09, 0xBF,
    0x9F0A, 0xBF,
    0x9F0B, 0xBF,
    0xA512, 0x2F,
    0xA513, 0x14,
    0xA514, 0x07,
    0xA518, 0x32,
    0xA519, 0x20,
    0xA51A, 0x0C,
    0xA521, 0x04,
    0xA527, 0x04,
    0xA528, 0x04,
    0xA529, 0x02,
    0xA52D, 0x2F,
    0xA52F, 0x06,
    0xA533, 0x31,
    0xA534, 0x15,
    0xA535, 0x08,
    0xA53C, 0x04,
    0xA53F, 0x04,
    0xA542, 0x04,
    0xA543, 0x04,
    0xA544, 0x02,
    0xA545, 0x04,
    0xA546, 0x04,
    0xA547, 0x02,
    0xA548, 0x2F,
    0xA54A, 0x06,
    0xA54B, 0x2F,
    0xA54D, 0x06,
    0xA54E, 0x31,
    0xA54F, 0x15,
    0xA550, 0x08,
    0xA557, 0x04,
    0xA55D, 0x04,
    0xA55E, 0x04,
    0xA55F, 0x02,
    0xA563, 0x1A,
    0xA565, 0x03,
    0xA569, 0x1B,
    0xA56A, 0x15,
    0xA56B, 0x05,
    0xA572, 0x04,
    0xA575, 0x04,
    0xA578, 0x04,
    0xA579, 0x04,
    0xA57A, 0x02,
    0xA57B, 0x04,
    0xA57C, 0x04,
    0xA57D, 0x02,
    0xA57E, 0x1A,
    0xA580, 0x03,
    0xA581, 0x1A,
    0xA583, 0x03,
    0xA584, 0x1B,
    0xA585, 0x15,
    0xA586, 0x05,
    0xA60D, 0x14,
    0xA60F, 0x08,
    0xA611, 0x01,
    0xA70A, 0x10,
    0xA70B, 0x04,
    0xA70F, 0x28,
    0xA710, 0x28,
    0xA711, 0x10,
    0xA716, 0x02,
    0xA717, 0x02,
    0xA71B, 0x31,
    0xA71C, 0x15,
    0xA71D, 0x08,
    0xA736, 0x31,
    0xA737, 0x15,
    0xA738, 0x08,
    0xA751, 0x1B,
    0xA752, 0x15,
    0xA753, 0x05,
    0xA76C, 0x1B,
    0xA76D, 0x15,
    0xA76E, 0x05,
    0xA801, 0x17,
    0xA803, 0x0C,
    0xA805, 0x00,
    0xA811, 0x08,
    0xA900, 0x02,
    0xA902, 0x02,
    0xA907, 0x03,
    0xA908, 0x03,
    0xA90D, 0x02,
    0xA90E, 0x02,
    0xA912, 0x00,
    0xA918, 0x38,
    0xA919, 0x38,
    0xA91A, 0x38,
    0xAA01, 0x18,
    0xAA03, 0x18,
    0xAA0F, 0x00,
    0xAA11, 0x00,
    0xAF01, 0x00,
    0xAF03, 0x00,
    0xAF05, 0x00,
    0xAF0D, 0xAF,
    0xAF0F, 0x9F,
    0xE80F, 0x93,
    0xE811, 0xFC,
    0xE827, 0x93,
    0xE829, 0xFC,
    0xE83F, 0x93,
    0xE841, 0xFC,
    0xE845, 0x93,
    0xE847, 0xFC,
    0xE857, 0x34,
    0xE859, 0xE9,
    0xE86F, 0x34,
    0xE871, 0xE9,
    0xE875, 0x34,
    0xE877, 0xE9,
};

/* reg_B8 3264*2448@29.3fps*/
static kal_uint16 imx709_preview_setting[] = {
    //3264x2448 @29.34fps Dbin RMSC w/ PD
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x2D,
    0x0343, 0xF0,
    0x0340, 0x09,
    0x0341, 0xC8,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x10,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x2F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x09,
    0x040F, 0x90,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x05,
    0x030F, 0xE2,
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x38B2, 0x00,
    0x38B3, 0x00,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x00,
    0x3A91, 0x00,
    0x3A94, 0x00,
    0x3A95, 0x00,
    0x3A96, 0x03,
    0x3A97, 0x97,
    0x3A9A, 0x03,
    0x3A9B, 0x97,
    0x3AA0, 0x00,
    0x3AA1, 0x00,
    0x3AA4, 0x00,
    0x3AA5, 0x00,
    0x3AB0, 0x01,
    0x3AB1, 0x06,
    0x3AB2, 0x00,
    0x3AB3, 0x00,
    0x3AB4, 0x01,
    0x3AB5, 0x06,
    0x3AB6, 0x00,
    0x3AB7, 0x00,
    0x3AC8, 0x07,
    0x3AC9, 0x20,
    0x3ACA, 0x07,
    0x3ACB, 0x20,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    0x0202, 0x09,
    0x0203, 0xB0,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x01,
    0x3080, 0x01,
    0x3081, 0x2b,
    0x3102, 0x01,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
    0x3207, 0x00, //Turn on the QSC Setting
    0x3963, 0x5A,
    0x3967, 0x5A,
};

/* reg_B8 3264*2448@29.3fps*/
static kal_uint16 imx709_capture_setting[] = {
    //3264x2448 @29.34fps Dbin RMSC w/ PD
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x2D,
    0x0343, 0xF0,
    0x0340, 0x09,
    0x0341, 0xC8,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x10,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x2F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x09,
    0x040F, 0x90,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x05,
    0x030F, 0xE2,
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x38B2, 0x00,
    0x38B3, 0x00,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x00,
    0x3A91, 0x00,
    0x3A94, 0x00,
    0x3A95, 0x00,
    0x3A96, 0x03,
    0x3A97, 0x97,
    0x3A9A, 0x03,
    0x3A9B, 0x97,
    0x3AA0, 0x00,
    0x3AA1, 0x00,
    0x3AA4, 0x00,
    0x3AA5, 0x00,
    0x3AB0, 0x01,
    0x3AB1, 0x06,
    0x3AB2, 0x00,
    0x3AB3, 0x00,
    0x3AB4, 0x01,
    0x3AB5, 0x06,
    0x3AB6, 0x00,
    0x3AB7, 0x00,
    0x3AC8, 0x07,
    0x3AC9, 0x20,
    0x3ACA, 0x07,
    0x3ACB, 0x20,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    0x0202, 0x09,
    0x0203, 0xB0,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x01,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
    0x3207, 0x00, //Turn on the QSC Setting
    0x3963, 0x5A,
    0x3967, 0x5A,
};

 /* reg_C4-3-11 3264*1856@30fps*/
static kal_uint16 imx709_normal_video_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x2D,
    0x0343, 0xF0,
    //Frame Length Lines Setting
    0x0340, 0x09,
    0x0341, 0x90,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0x79,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x01,
    0x3A91, 0xF6,
    0x3A94, 0x01,
    0x3A95, 0xF6,
    0x3A96, 0x04,
    0x3A97, 0x63,
    0x3A9A, 0x04,
    0x3A9B, 0x63,
    0x3AA0, 0x01,
    0x3AA1, 0xF6,
    0x3AA4, 0x01,
    0x3AA5, 0xF6,
    0x3AB0, 0x01,
    0x3AB1, 0x0E,
    0x3AB2, 0x00,
    0x3AB3, 0x00,
    0x3AB4, 0x01,
    0x3AB5, 0x0E,
    0x3AB6, 0x00,
    0x3AB7, 0x90,
    0x3AC8, 0x0B,
    0x3AC9, 0x2B,
    0x3ACA, 0x0B,
    0x3ACB, 0x2B,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x4F14, 0x01,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    0x38B2, 0x00,
    0x38B3, 0x90,
    //Integration Setting
    0x0202, 0x09,
    0x0203, 0x78,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x01,
    0x3080, 0x01,
    0x3081, 0x2b,
    0x3102, 0x01,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
    //IP setting
    0x3963, 0x0A,
    0x3967, 0x0A,
};

/*reg_G2 3264x1856@60ps*/
static kal_uint16 imx709_hs_video_setting[] = {
    //3264x1856 @60FPS
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting
    0x0340, 0x07,
    0x0341, 0x78,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB1,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x08,
    0x030F, 0x08,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A90, 0x03,
    0x3A91, 0x66,
    0x3A94, 0x03,
    0x3A95, 0x66,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3AC8, 0x03,
    0x3AC9, 0x0C,
    0x3ACA, 0x03,
    0x3ACB, 0x0C,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x07,
    0x0203, 0x60,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
};

/*reg_G2 3264x1856@60ps*/
static kal_uint16 imx709_slim_video_setting[] = {
    //3264x1856 @60FPS
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting
    0x0340, 0x07,
    0x0341, 0x78,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB1,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x08,
    0x030F, 0x08,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A90, 0x03,
    0x3A91, 0x66,
    0x3A94, 0x03,
    0x3A95, 0x66,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3AC8, 0x03,
    0x3AC9, 0x0C,
    0x3ACA, 0x03,
    0x3ACB, 0x0C,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x07,
    0x0203, 0x60,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
};

/*reg H 1632x1224@15FPS */
static kal_uint16 imx709_custom1_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x16,
    0x0343, 0xF8,
    //Frame Length Lines Setting
    0x0340, 0x0A,
    0x0341, 0xA0,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x3F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x24,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x42,
    0x31D1, 0x43,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x04,
    0x040A, 0x00,
    0x040B, 0x04,
    0x040C, 0x06,
    0x040D, 0x60,
    0x040E, 0x04,
    0x040F, 0xC8,
    //Output Size Setting
    0x034C, 0x06,
    0x034D, 0x60,
    0x034E, 0x04,
    0x034F, 0xC8,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x64,
    0x030B, 0x08,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0xB0,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A96, 0x02,
    0x3A97, 0xC2,
    0x3A9A, 0x02,
    0x3A9B, 0xC2,
    0x3AA0, 0x00,
    0x3AA1, 0x96,
    0x3AA4, 0x00,
    0x3AA5, 0x96,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0x64,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0x64,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x0A,
    0x0203, 0x88,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00, //turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00, //DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x36,
    0x3967, 0x36,
};

/*reg H1 1632x1224 @30FPS*/
static kal_uint16 imx709_custom2_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x16,
    0x0343, 0xF8,
    //Frame Length Lines Setting
    0x0340, 0x08,
    0x0341, 0x48,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x3F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x24,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x42,
    0x31D1, 0x43,
    0x0408, 0x00,
    0x0409, 0x04,
    0x040A, 0x00,
    0x040B, 0x04,
    0x040C, 0x06,
    0x040D, 0x60,
    0x040E, 0x04,
    0x040F, 0xC8,
    //Output Size Setting
    0x034C, 0x06,
    0x034D, 0x60,
    0x034E, 0x04,
    0x034F, 0xC8,
    //Clock Setting
    0x0303, 0x04,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x9C,
    0x030B, 0x08,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0xB0,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A96, 0x02,
    0x3A97, 0xC2,
    0x3A9A, 0x02,
    0x3A9B, 0xC2,
    0x3AA0, 0x00,
    0x3AA1, 0x96,
    0x3AA4, 0x00,
    0x3AA5, 0x96,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0x64,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0x64,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x08,
    0x0203, 0x30,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,//turn off PDAF
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL
    0x3170, 0x00, //DOL disable
    0x317C, 0x0A,
    0x317D, 0x0A,
    //Global setting
    0x3207, 0x00, //Turn off the QSC Setting
    //IP setting
    0x3963, 0x5A,
    0x3967, 0x5A,
};

/*reg A5 6528x4896@15fps*/
static kal_uint16 imx709_custom3_setting[] = {
    //6528x4896 @15FPS RMSC ( From A_5 )
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1B,
    0x0343, 0xD0,
    //Frame Length Lines Setting
    0x0340, 0x13,
    0x0341, 0xA8,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x10,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x2F,
    //Mode Setting
    0x0900, 0x00,
    0x0901, 0x11,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x4B,
    0x31D1, 0x4B,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x10,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x19,
    0x040D, 0x80,
    0x040E, 0x13,
    0x040F, 0x20,
    //Output Size Setting
    0x034C, 0x19,
    0x034D, 0x80,
    0x034E, 0x13,
    0x034F, 0x20,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x70,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x05,
    0x030F, 0x17,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x01,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x00,
    0x3A91, 0x00,
    0x3A94, 0x00,
    0x3A95, 0x00,
    0x3A96, 0x02,
    0x3A97, 0xF8,
    0x3A9A, 0x02,
    0x3A9B, 0xF8,
    0x3AA0, 0x03,
    0x3AA1, 0x70,
    0x3AA4, 0x03,
    0x3AA5, 0x70,
    0x3AB0, 0x00,
    0x3AB1, 0xDC,
    0x3AB2, 0x00,
    0x3AB3, 0xDC,
    0x3AB4, 0x00,
    0x3AB5, 0xDC,
    0x3AB6, 0x00,
    0x3AB7, 0xDC,
    0x3AC8, 0x00,
    0x3AC9, 0x7E,
    0x3ACA, 0x00,
    0x3ACB, 0x7E,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x13,
    0x0203, 0x90,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
};

/*reg C4-5 3264*1856@30FPS 2DOL*/
static kal_uint16 imx709_custom4_setting[] = {

    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1D,
    0x0343, 0x20,
    //Frame Length Lines Setting
    0x0340, 0x07,
    0x0341, 0x88,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0xAE,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x03,
    0x3A91, 0x66,
    0x3A94, 0x03,
    0x3A95, 0x66,
    0x3A96, 0x03,
    0x3A97, 0x88,
    0x3A9A, 0x03,
    0x3A9B, 0x88,
    0x3AA0, 0x01,
    0x3AA1, 0xAC,
    0x3AA4, 0x01,
    0x3AA5, 0xAC,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3AC8, 0x02,
    0x3AC9, 0xC8,
    0x3ACA, 0x02,
    0x3ACB, 0xBC,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x4F14, 0x01,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x06,
    0x0203, 0x80,
    0x0224, 0x00,
    0x0225, 0xD0,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x01,
    0x317C, 0x0A,
    0x317D, 0x0A,
    0xbcaf, 0x01,
    //IP setting
    0x3963, 0x0A,
    0x3967, 0x0A,
};

/* reg_C3 3264*1856@30fps reserved*/
static kal_uint16 imx709_custom5_setting[] = {
    //3264x2448@30FPS ( 870Mbps/lane )
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1C,
    0x0343, 0xE8,
    //Frame Length Lines Setting
    0x0340, 0x0C,
    0x0341, 0xA8,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x10,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x2F,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x09,
    0x040F, 0x90,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x09,
    0x034F, 0x90,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0x96,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x06,
    0x030F, 0xCC,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x59,
    0x3A01, 0x48,
    0x3A90, 0x03,
    0x3A91, 0x66,
    0x3A94, 0x03,
    0x3A95, 0x66,
    0x3A96, 0x03,
    0x3A97, 0x34,
    0x3A9A, 0x03,
    0x3A9B, 0x34,
    0x3AA0, 0x01,
    0x3AA1, 0x8B,
    0x3AA4, 0x01,
    0x3AA5, 0x8B,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3AC8, 0x03,
    0x3AC9, 0x0C,
    0x3ACA, 0x03,
    0x3ACB, 0x0C,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x0C,
    0x0203, 0x90,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
    0x3207, 0x00, //Turn on the QSC Setting
    0x3963, 0x36,
    0x3967, 0x36,
};

 /* reg_C4-3-1 3264*1856@30fps*/
static kal_uint16 imx709_custom6_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x2D,
    0x0343, 0xF0,
    //Frame Length Lines Setting
    0x0340, 0x09,
    0x0341, 0x90,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0x79,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x00,
    0x3A91, 0x00,
    0x3A94, 0x00,
    0x3A95, 0x00,
    0x3A96, 0x00,
    0x3A97, 0x00,
    0x3A9A, 0x00,
    0x3A9B, 0x00,
    0x3AA0, 0x00,
    0x3AA1, 0x00,
    0x3AA4, 0x00,
    0x3AA5, 0x00,
    0x3AB0, 0x00,
    0x3AB1, 0x00,
    0x3AB2, 0x00,
    0x3AB3, 0x00,
    0x3AB4, 0x00,
    0x3AB5, 0x00,
    0x3AB6, 0x00,
    0x3AB7, 0x00,
    0x3AC8, 0x00,
    0x3AC9, 0x00,
    0x3ACA, 0x00,
    0x3ACB, 0x00,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x4F14, 0x01,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x09,
    0x0203, 0x78,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x01,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
    //IP setting
    0x3963, 0x0A,
    0x3967, 0x0A,
};

/*reg C4-5  3264X1856@30fps 2DOL video for preisp*/
static kal_uint16 imx709_custom7_setting[] = {

    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1D,
    0x0343, 0x20,
    //Frame Length Lines Setting
    0x0340, 0x07,
    0x0341, 0x88,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0xAE,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x03,
    0x3A91, 0x66,
    0x3A94, 0x03,
    0x3A95, 0x66,
    0x3A96, 0x03,
    0x3A97, 0x88,
    0x3A9A, 0x03,
    0x3A9B, 0x88,
    0x3AA0, 0x01,
    0x3AA1, 0xAC,
    0x3AA4, 0x01,
    0x3AA5, 0xAC,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3AC8, 0x02,
    0x3AC9, 0xC8,
    0x3ACA, 0x02,
    0x3ACB, 0xBC,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x4F14, 0x01,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x06,
    0x0203, 0x80,
    0x0224, 0x00,
    0x0225, 0xD0,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x01,
    0x317C, 0x0A,
    0x317D, 0x0A,
    0xbcaf, 0x01,
    //IP setting
    0x3963, 0x0A,
    0x3967, 0x0A,
};

/*reg B3 3264x2448@30fps reserved for preisp*/
static kal_uint16 imx709_custom8_setting[] = {
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x2D,
    0x0343, 0xF0,
    //Frame Length Lines Setting
    0x0340, 0x09,
    0x0341, 0x90,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB4,
    0x030B, 0x02,
    0x030D, 0x0C,
    0x030E, 0x04,
    0x030F, 0x79,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x00,
    0x3A91, 0x00,
    0x3A94, 0x00,
    0x3A95, 0x00,
    0x3A96, 0x00,
    0x3A97, 0x00,
    0x3A9A, 0x00,
    0x3A9B, 0x00,
    0x3AA0, 0x00,
    0x3AA1, 0x00,
    0x3AA4, 0x00,
    0x3AA5, 0x00,
    0x3AB0, 0x00,
    0x3AB1, 0x00,
    0x3AB2, 0x00,
    0x3AB3, 0x00,
    0x3AB4, 0x00,
    0x3AB5, 0x00,
    0x3AB6, 0x00,
    0x3AB7, 0x00,
    0x3AC8, 0x00,
    0x3AC9, 0x00,
    0x3ACA, 0x00,
    0x3ACB, 0x00,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x4F14, 0x01,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x09,
    0x0203, 0x78,
    0x0224, 0x01,
    0x0225, 0xF4,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x01,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
    //IP setting
    0x3963, 0x0A,
    0x3967, 0x0A,
};

static kal_uint16 imx709_custom9_setting[] = {
    //3264x1856 @30FPS (Dbin 2-DOL), F-DOL 1ST
    //MIPI output setting
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    //Line Length PCK Setting
    0x0342, 0x1D,
    0x0343, 0x20,
    //Frame Length Lines Setting
    0x0340, 0x07,
    0x0341, 0x80,
    //ROI Setting
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x02,
    0x0347, 0x60,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x10,
    0x034B, 0xDF,
    //Mode Setting
    0x0900, 0x01,
    0x0901, 0x12,
    0x0902, 0x00,
    0x3148, 0x00,
    0x31D0, 0x40,
    0x31D1, 0x41,
    //Digital Crop & Scaling
    0x0408, 0x00,
    0x0409, 0x08,
    0x040A, 0x00,
    0x040B, 0x00,
    0x040C, 0x0C,
    0x040D, 0xC0,
    0x040E, 0x07,
    0x040F, 0x40,
    //Output Size Setting
    0x034C, 0x0C,
    0x034D, 0xC0,
    0x034E, 0x07,
    0x034F, 0x40,
    //Clock Setting
    0x0303, 0x02,
    0x0305, 0x02,
    0x0306, 0x00,
    0x0307, 0xB3,
    0x030B, 0x04,
    0x030D, 0x0C,
    0x030E, 0x08,
    0x030F, 0x20,
    //Other Setting
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x03,
    0x3A91, 0x66,
    0x3A94, 0x03,
    0x3A95, 0x66,
    0x3A96, 0x03,
    0x3A97, 0x88,
    0x3A9A, 0x03,
    0x3A9B, 0x88,
    0x3AA0, 0x01,
    0x3AA1, 0xAC,
    0x3AA4, 0x01,
    0x3AA5, 0xAC,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0xF0,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0xF0,
    0x3AC8, 0x02,
    0x3AC9, 0xC8,
    0x3ACA, 0x02,
    0x3ACB, 0xBC,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x4F14, 0x01,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    //Integration Setting
    0x0202, 0x06,
    0x0203, 0x80,
    0x0224, 0x00,
    0x0225, 0xD0,
    //Gain Setting
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    //PDAF TYPE2 Setting
    0x30C0, 0x00,
    0x3080, 0x00,
    0x3081, 0x30,
    0x3102, 0x00,
    0x347A, 0x03,
    0x347B, 0x2C,
    //DOL Setting
    0x3170, 0x01,
    0x317C, 0x0A,
    0x317D, 0x0A,
    0xbcaf, 0x01,
};

static kal_uint16 imx709_custom10_setting[] = {
    0x0112, 0x0A,
    0x0113, 0x0A,
    0x0114, 0x03,
    0x0342, 0x16,
    0x0343, 0xF8,
    0x0340, 0x07,
    0x0341, 0xD8,
    0x0344, 0x00,
    0x0345, 0x00,
    0x0346, 0x00,
    0x0347, 0x00,
    0x0348, 0x19,
    0x0349, 0x9F,
    0x034A, 0x13,
    0x034B, 0x3F,
    0x0900, 0x01,
    0x0901, 0x24,
    0x0902, 0x00,
    0x3148, 0x04,
    0x31D0, 0x42,
    0x31D1, 0x43,
    0x0408, 0x00,
    0x0409, 0x04,
    0x040A, 0x00,
    0x040B, 0x04,
    0x040C, 0x06,
    0x040D, 0x60,
    0x040E, 0x04,
    0x040F, 0xC8,
    0x034C, 0x06,
    0x034D, 0x60,
    0x034E, 0x04,
    0x034F, 0xC8,
    0x0303, 0x04,
    0x0305, 0x01,
    0x0306, 0x00,
    0x0307, 0x4A,
    0x030B, 0x08,
    0x030D, 0x0C,
    0x030E, 0x07,
    0x030F, 0x24,
    0x310C, 0x00,
    0x3207, 0x00,
    0x3214, 0x01,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x02,
    0x3A91, 0xA8,
    0x3A94, 0x02,
    0x3A95, 0xA8,
    0x3A96, 0x02,
    0x3A97, 0xC2,
    0x3A9A, 0x02,
    0x3A9B, 0xC2,
    0x3AA0, 0x00,
    0x3AA1, 0x96,
    0x3AA4, 0x00,
    0x3AA5, 0x96,
    0x3AB0, 0x00,
    0x3AB1, 0xF0,
    0x3AB2, 0x00,
    0x3AB3, 0x64,
    0x3AB4, 0x00,
    0x3AB5, 0xF0,
    0x3AB6, 0x00,
    0x3AB7, 0x64,
    0x3AC8, 0x03,
    0x3AC9, 0x20,
    0x3ACA, 0x03,
    0x3ACB, 0x02,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    0x0202, 0x07,
    0x0203, 0xC0,
    0x0224, 0x01,
    0x0225, 0xF4,
    0x0204, 0x00,
    0x0205, 0x00,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0216, 0x00,
    0x0217, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    0x30C0, 0x01,
    0x3080, 0x01,
    0x3081, 0x2B,
    0x3102, 0x01,
    0x347A, 0x03,
    0x347B, 0x2C,
    0x3170, 0x00,
    0x317C, 0x0A,
    0x317D, 0x0A,
};
static u16 imx709_aonhemd_clear_setting[] = {
    0x37B7, 0x01,
};
static u16 imx709_aonhemd_setting[] = {
    0x0136, 0x13,
    0x0137, 0x33,
    0x3304, 0x00,
    0x33F0, 0x08,
    0x33F1, 0x07,
    0x0111, 0x02,
    0x3140, 0x08,
    0x3A42, 0x01,
    0x4C24, 0x3F,
    0x4CFC, 0x00,
    0x4F0E, 0x01,
    0x4F0F, 0x01,
    0x3379, 0x00,
    0x3724, 0x03,
    0x3732, 0x05,
    0x3733, 0xDC,
    0x3736, 0x09,
    0x3737, 0xC4,
    0x373A, 0x07,
    0x373B, 0xD0,
    0x373E, 0x03,
    0x373F, 0xE8,
    0x37A2, 0x8C,
    0x37B4, 0x03,
    0x37BA, 0x0F,
    0x37BB, 0xA0,
    0x37BE, 0x0D,
    0x37BF, 0xAC,
    0x37C2, 0x04,
    0x37C3, 0xB0,
    0x37C6, 0x3E,
    0x37C7, 0x80,
    0x37CA, 0x3A,
    0x37CB, 0x20,
    0x37CE, 0x03,
    0x37CF, 0x84,
    0x37D6, 0x09,
    0x37DB, 0x0F,
    0x37DC, 0x0F,
    0x37DD, 0x01,
    0x385E, 0x05,
    0x385F, 0xDC,
    0x3862, 0x09,
    0x3863, 0xC4,
    0x3866, 0x07,
    0x3867, 0xD0,
    0x386A, 0x03,
    0x386B, 0xE8,
    0x3A1B, 0x40,
    0x3A1D, 0x40,
    0x3A1E, 0x01,
    0x3A92, 0x01,
    0x3AA2, 0x01,
    0x640C, 0xFF,
    0x640D, 0xFF,
    0x6414, 0x00,
    0x6415, 0x9D,
    0x6418, 0xFF,
    0x6419, 0xFF,
    0x6420, 0x00,
    0x6421, 0x9D,
    0x6424, 0xFF,
    0x6425, 0xFF,
    0x642C, 0x00,
    0x642D, 0x9D,
    0x6430, 0xFF,
    0x6431, 0xFF,
    0x6438, 0x02,
    0x6439, 0x24,
    0x643C, 0xFF,
    0x643D, 0xFF,
    0x6444, 0x02,
    0x6445, 0x24,
    0x6454, 0xFF,
    0x6455, 0xFF,
    0x645C, 0x00,
    0x645D, 0x9D,
    0x646C, 0xFF,
    0x646D, 0xFF,
    0x6474, 0x00,
    0x6475, 0x9D,
    0x68B0, 0x00,
    0x68B2, 0x1A,
    0x68B5, 0x01,
    0x68B6, 0x00,
    0x68B8, 0x1A,
    0x69D3, 0x27,
    0x69DF, 0x27,
    0x6FD1, 0x09,
    0x6FD3, 0x09,
    0x6FD5, 0x09,
    0x6FD7, 0x09,
    0x6FD9, 0x09,
    0x7020, 0x03,
    0x710A, 0x17,
    0x710C, 0x18,
    0x73E9, 0x53,
    0x73ED, 0x50,
    0x73F1, 0x4B,
    0x73F5, 0x4D,
    0x73F9, 0x4E,
    0x741D, 0x56,
    0x7421, 0x55,
    0x7425, 0x53,
    0x7429, 0x56,
    0x742D, 0x56,
    0x7451, 0x58,
    0x7455, 0x56,
    0x7459, 0x57,
    0x745D, 0x58,
    0x7461, 0x58,
    0x7485, 0x5A,
    0x7489, 0x58,
    0x748D, 0x58,
    0x7491, 0x5B,
    0x7495, 0x5B,
    0x74B9, 0x5D,
    0x74BB, 0x5E,
    0x74BD, 0x5F,
    0x74BF, 0x60,
    0x74E0, 0x16,
    0x74E2, 0x19,
    0x74E4, 0x16,
    0x74E6, 0x17,
    0x74E8, 0x17,
    0x74FA, 0x20,
    0x74FC, 0x24,
    0x74FE, 0x1E,
    0x7500, 0x1F,
    0x7502, 0x30,
    0x7514, 0x3F,
    0x7516, 0x1E,
    0x7518, 0x19,
    0x751A, 0x1E,
    0x751C, 0x1E,
    0x752E, 0x57,
    0x7530, 0x26,
    0x7532, 0x14,
    0x7534, 0x1D,
    0x7536, 0x1D,
    0x7548, 0x53,
    0x7549, 0x19,
    0x754A, 0x52,
    0x754B, 0x53,
    0x757E, 0x07,
    0x7590, 0x0A,
    0x75AA, 0x0A,
    0x75AC, 0x05,
    0x75C4, 0x0A,
    0x75C5, 0x0D,
    0x75C6, 0x0A,
    0x75C7, 0x0A,
    0x7656, 0x28,
    0x765A, 0x14,
    0x765E, 0x05,
    0x7660, 0x05,
    0x7678, 0x0A,
    0x767A, 0x0A,
    0x7692, 0x14,
    0x7694, 0x14,
    0x76EA, 0x01,
    0x76EC, 0x01,
    0x76EE, 0x01,
    0x76F0, 0x01,
    0x76F2, 0x01,
    0x787A, 0x05,
    0x787C, 0x05,
    0x787E, 0x05,
    0x7880, 0x05,
    0x7B52, 0x14,
    0x7B53, 0x14,
    0x7B54, 0x14,
    0x7B55, 0x14,
    0x7B56, 0x14,
    0x7B57, 0x14,
    0x7B58, 0x14,
    0x7B59, 0x14,
    0x7B5A, 0x14,
    0x7B5C, 0x05,
    0x7B5E, 0x05,
    0x7B60, 0x14,
    0x7B61, 0x14,
    0x7B62, 0x14,
    0x7B63, 0x14,
    0x7B64, 0x14,
    0x7B65, 0x14,
    0x7B66, 0x14,
    0x7B67, 0x14,
    0x7B68, 0x14,
    0x7B69, 0x14,
    0x7B6A, 0x14,
    0x7B6B, 0x14,
    0x7B6C, 0x14,
    0x7B6D, 0x14,
    0x7B6E, 0x14,
    0x7B6F, 0x14,
    0x7B70, 0x14,
    0x7B71, 0x14,
    0x7B72, 0x14,
    0x7B73, 0x14,
    0x7B74, 0x14,
    0x7B76, 0x14,
    0x7B78, 0x14,
    0x7B7A, 0x14,
    0x7B7B, 0x14,
    0x7B7C, 0x14,
    0x7B7D, 0x14,
    0x7B7E, 0x14,
    0x7B7F, 0x14,
    0x7B80, 0x14,
    0x7B81, 0x14,
    0x7B82, 0x14,
    0x7B83, 0x14,
    0x7B84, 0x14,
    0x7B85, 0x14,
    0x7B86, 0x14,
    0x7B87, 0x14,
    0x7B88, 0x14,
    0x7B89, 0x14,
    0x7B8A, 0x14,
    0x7B8B, 0x14,
    0x7B8C, 0x14,
    0x7B8D, 0x14,
    0x7B8E, 0x14,
    0x7B90, 0x14,
    0x7B92, 0x14,
    0x7B94, 0x14,
    0x7B95, 0x14,
    0x7B96, 0x14,
    0x7B97, 0x14,
    0x7B98, 0x14,
    0x7B99, 0x14,
    0x7B9A, 0x14,
    0x7B9B, 0x14,
    0x7B9C, 0x14,
    0x7B9D, 0x14,
    0x7B9E, 0x14,
    0x7B9F, 0x14,
    0x7BA0, 0x14,
    0x7BA1, 0x14,
    0x7BA2, 0x14,
    0x7BA3, 0x14,
    0x7BA4, 0x14,
    0x7BA5, 0x14,
    0x7BA6, 0x14,
    0x7BA7, 0x14,
    0x7BA8, 0x14,
    0x7BAA, 0x14,
    0x7BAC, 0x14,
    0x7BAE, 0x14,
    0x7BAF, 0x14,
    0x7BB0, 0x14,
    0x7BB1, 0x14,
    0x7BB2, 0x14,
    0x7BB3, 0x14,
    0x7BB4, 0x14,
    0x7BB5, 0x14,
    0x7BB6, 0x14,
    0x7BB7, 0x14,
    0x7BB8, 0x14,
    0x7BB9, 0x14,
    0x7BBA, 0x14,
    0x7BBB, 0x14,
    0x7BBC, 0x14,
    0x7BBD, 0x14,
    0x7BBE, 0x14,
    0x7BC0, 0x14,
    0x7BC1, 0x14,
    0x7BC2, 0x14,
    0x7BC3, 0x14,
    0x7BC4, 0x14,
    0x7BC5, 0x14,
    0x7BC7, 0x14,
    0x7BC8, 0x14,
    0x7BC9, 0x14,
    0x7BCA, 0x14,
    0x7BCB, 0x14,
    0x9002, 0x0A,
    0x9003, 0x0A,
    0x9004, 0x0A,
    0x90E4, 0x01,
    0x90E7, 0x01,
    0x9200, 0x65,
    0x9201, 0xCE,
    0x9202, 0x65,
    0x9203, 0xC4,
    0x9204, 0x00,
    0x9205, 0x00,
    0x9206, 0x00,
    0x9207, 0x00,
    0x9208, 0x00,
    0x9209, 0x00,
    0x920A, 0x00,
    0x920B, 0x00,
    0x920C, 0x00,
    0x920D, 0x00,
    0x920E, 0x00,
    0x920F, 0x00,
    0x9210, 0x00,
    0x9211, 0x00,
    0x9212, 0x00,
    0x9213, 0x00,
    0x9214, 0x00,
    0x9215, 0x00,
    0x9216, 0x00,
    0x9217, 0x00,
    0x9218, 0x00,
    0x9219, 0x00,
    0x921A, 0x00,
    0x921B, 0x00,
    0x921C, 0x00,
    0x921D, 0x00,
    0x921E, 0x00,
    0x921F, 0x00,
    0x9220, 0x00,
    0x9221, 0x00,
    0x9222, 0x00,
    0x9223, 0x00,
    0x9224, 0x00,
    0x9225, 0x00,
    0x9226, 0x00,
    0x9227, 0x00,
    0x9228, 0x00,
    0x9229, 0x00,
    0x922A, 0x00,
    0x922B, 0x00,
    0x922C, 0x00,
    0x922D, 0x00,
    0x922E, 0x00,
    0x922F, 0x00,
    0x9230, 0x00,
    0x9231, 0x00,
    0x9232, 0x00,
    0x9233, 0x00,
    0x9234, 0xD7,
    0x9235, 0xE4,
    0x9236, 0xD7,
    0x9237, 0xE5,
    0x9238, 0xD7,
    0x9239, 0xE6,
    0x923A, 0xD7,
    0x923B, 0xE7,
    0x923C, 0x15,
    0x923D, 0x67,
    0xB0F9, 0x06,
    0xBC51, 0xC8,
    0xBCAF, 0x01,
    0xBD27, 0x28,
    0xBDC6, 0x00,
    0xBDC7, 0x00,
    0xBDC8, 0x00,
    0xBDC9, 0x00,
    0xBDCA, 0x00,
    0xBDCB, 0x00,
    0xBDCC, 0x00,
    0xBDCD, 0x00,
    0xBDCE, 0x00,
    0xBDCF, 0x00,
    0xBDD0, 0x10,
    0xBDD1, 0xDC,
    0xBDD2, 0x08,
    0xBDD3, 0x58,
    0xBDD4, 0x17,
    0xBDD5, 0xDC,
    0xBDD6, 0x0B,
    0xBDD7, 0x80,
    0xBDE0, 0x00,
    0xBDE1, 0x00,
    0xBDE2, 0x00,
    0xBDE3, 0x00,
    0xBDE4, 0x00,
    0xBDE5, 0xE0,
    0xBDE7, 0xC8,
    0xBDE8, 0x00,
    0xBDEA, 0x01,
    0xBDEB, 0xC8,
    0xC0D9, 0x12,
    0xC0DB, 0x28,
    0xC0DE, 0x28,
    0xC0E1, 0x2D,
    0xC0E3, 0x00,
    0xC0F2, 0x18,
    0xC0F3, 0x18,
    0xC546, 0x0A,
    0xC547, 0x08,
    0xC548, 0x28,
    0xC54B, 0x28,
    0xC54E, 0x3C,
    0xC551, 0x01,
    0xC55E, 0x18,
    0xC55F, 0x18,
    0xE981, 0x02,
    0xE982, 0x05,
    0xE983, 0x33,
    0xE984, 0x19,
    0xE985, 0x13,
    0xE986, 0x11,
    0xE987, 0x10,
    0xE98A, 0x01,
    0xEB00, 0x18,
    0xEB01, 0x0B,
    0xEB02, 0x04,
    0x0112, 0x08,
    0x0113, 0x08,
    0x0114, 0x03,
    0x3762, 0x04,
    0x3763, 0xE2,
    0x3766, 0x00,
    0x3770, 0x02,
    0x3771, 0x71,
    0x3812, 0x04,
    0x3813, 0xE2,
    0x3816, 0x00,
    0x3820, 0x02,
    0x3821, 0x71,
    0x3882, 0x05,
    0x3883, 0x00,
    0x3886, 0x00,
    0x389E, 0x00,
    0x389F, 0x05,
    0x38C6, 0x05,
    0x38C7, 0x00,
    0x38CA, 0x00,
    0x38E0, 0x00,
    0x38E1, 0x05,
    0x3760, 0x0B,
    0x3761, 0x40,
    0x3810, 0x0B,
    0x3811, 0x40,
    0x3880, 0x0A,
    0x3881, 0xF8,
    0x38C4, 0x0A,
    0x38C5, 0xF8,
    0x3888, 0x00,
    0x3889, 0x00,
    0x388A, 0x00,
    0x388B, 0x00,
    0x388C, 0x19,
    0x388D, 0x9F,
    0x388E, 0x13,
    0x388F, 0x3F,
    0x38CC, 0x00,
    0x38CD, 0x00,
    0x38CE, 0x00,
    0x38CF, 0x00,
    0x38D0, 0x19,
    0x38D1, 0x9F,
    0x38D2, 0x13,
    0x38D3, 0x3F,
    0x0900, 0x01,
    0x0902, 0x00,
    0x3004, 0x0A,
    0x31D4, 0x08,
    0x31D5, 0x08,
    0x3894, 0x00,
    0x3895, 0x00,
    0x3896, 0x00,
    0x3897, 0x00,
    0x3898, 0x03,
    0x3899, 0x34,
    0x389A, 0x02,
    0x389B, 0x68,
    0x38D8, 0x00,
    0x38D9, 0x00,
    0x38DA, 0x00,
    0x38DB, 0x00,
    0x38DC, 0x01,
    0x38DD, 0x9A,
    0x38DE, 0x01,
    0x38DF, 0x34,
    0x3890, 0x03,
    0x3891, 0x34,
    0x3892, 0x02,
    0x3893, 0x68,
    0x38D4, 0x01,
    0x38D5, 0x98,
    0x38D6, 0x01,
    0x38D7, 0x34,
    0x0303, 0x02,
    0x0305, 0x01,
    0x0306, 0x00,
    0x0307, 0x42,
    0x387C, 0x01,
    0x387D, 0x01,
    0x387E, 0x00,
    0x387F, 0x81,
    0x31CC, 0x01,
    0x3213, 0x00,
    0x3A00, 0x42,
    0x3A01, 0xFF,
    0x3A90, 0x00,
    0x3A91, 0x00,
    0x3A94, 0x00,
    0x3A95, 0x00,
    0x3A96, 0x00,
    0x3A97, 0x00,
    0x3A9A, 0x00,
    0x3A9B, 0x00,
    0x3AA0, 0x00,
    0x3AA1, 0x00,
    0x3AA4, 0x00,
    0x3AA5, 0x00,
    0x3AB0, 0x00,
    0x3AB1, 0x00,
    0x3AB2, 0x00,
    0x3AB3, 0x00,
    0x3AB4, 0x00,
    0x3AB5, 0x00,
    0x3AB6, 0x00,
    0x3AB7, 0x00,
    0x3AC8, 0x00,
    0x3AC9, 0x00,
    0x3ACA, 0x00,
    0x3ACB, 0x00,
    0x3AD0, 0x00,
    0x3AD1, 0x00,
    0x3AD2, 0x00,
    0x3AD3, 0x00,
    0x3AD8, 0x00,
    0x3AD9, 0x00,
    0x3ADC, 0x00,
    0x3ADD, 0x00,
    0x3C70, 0x00,
    0x3C71, 0x00,
    0x3C72, 0x00,
    0x3C73, 0x00,
    0x3C84, 0x00,
    0x3C85, 0x00,
    0x3C88, 0x00,
    0x3C89, 0x00,
    0x3C8A, 0x00,
    0x3C8B, 0x00,
    0x3C9C, 0x00,
    0x3C9D, 0x00,
    0x3CA0, 0x00,
    0x3CA1, 0x00,
    0x3CA2, 0x00,
    0x3CA3, 0x00,
    0x3CB4, 0x00,
    0x3CB5, 0x00,
    0x3CB8, 0x00,
    0x3CB9, 0x00,
    0x3CBA, 0x00,
    0x3CBB, 0x00,
    0x3CCC, 0x00,
    0x3CCD, 0x00,
    0x4F14, 0x01,
    0x910E, 0x00,
    0x910F, 0x00,
    0x9110, 0x00,
    0x9154, 0x00,
    0xBD24, 0x02,
    0xBD25, 0x60,
    0x3750, 0x00,
    0x3751, 0x40,
    0x3756, 0x00,
    0x3757, 0x01,
    0x3788, 0x00,
    0x3789, 0xE0,
    0x378E, 0x00,
    0x378F, 0x04,
    0x3794, 0x01,
    0x3795, 0x80,
    0x379A, 0x00,
    0x379B, 0x06,
    0x3800, 0x00,
    0x3801, 0x16,
    0x3806, 0x00,
    0x3807, 0x16,
    0x3838, 0x00,
    0x3839, 0xED,
    0x383E, 0x00,
    0x383F, 0xED,
    0x3844, 0x0A,
    0x3845, 0x77,
    0x384A, 0x0A,
    0x384B, 0x77,
    0x3878, 0x00,
    0x3879, 0x40,
    0x38B6, 0x00,
    0x38B7, 0xE0,
    0x38BA, 0x01,
    0x38BB, 0x80,
    0x38C0, 0x00,
    0x38C1, 0x40,
    0x38F8, 0x00,
    0x38F9, 0xE0,
    0x38FC, 0x01,
    0x38FD, 0x80,
    0x020E, 0x01,
    0x020F, 0x00,
    0x0218, 0x01,
    0x0219, 0x00,
    0x3166, 0x01,
    0x3167, 0x00,
    0x3752, 0x00,
    0x3753, 0x00,
    0x3758, 0x00,
    0x3759, 0x00,
    0x378A, 0x20,
    0x378B, 0x00,
    0x3790, 0x20,
    0x3791, 0x00,
    0x3796, 0x30,
    0x3797, 0x00,
    0x379C, 0x30,
    0x379D, 0x00,
    0x3802, 0x3C,
    0x3803, 0x00,
    0x3808, 0x20,
    0x3809, 0x00,
    0x383A, 0x3C,
    0x383B, 0x00,
    0x3840, 0x20,
    0x3841, 0x00,
    0x3846, 0x3C,
    0x3847, 0x00,
    0x384C, 0x20,
    0x384D, 0x00,
    0x387A, 0x00,
    0x387B, 0x00,
    0x38B8, 0x20,
    0x38B9, 0x00,
    0x38BC, 0x30,
    0x38BD, 0x00,
    0x38C2, 0x00,
    0x38C3, 0x00,
    0x38FA, 0x20,
    0x38FB, 0x00,
    0x38FE, 0x30,
    0x38FF, 0x00,
    0x371C, 0x01,
    0x37B0, 0x01,
    0x3030, 0x01,
    0x4512, 0x03,
    0x4513, 0xFF,
    0x4501, 0x03,
    0x0100, 0x01,
};

static void extend_frame_length(struct subdrv_ctx *ctx, kal_uint32 ns)
{
    return;
    #if 0
    ctx->frame_length = ns;

    write_cmos_sensor_8(ctx, 0x0104, 0x01);
    write_frame_len(ctx, ctx->frame_length, get_cur_exp_cnt());
    write_cmos_sensor_8(ctx, 0x0104, 0x00);
    ctx->extend_frame_length_en = KAL_TRUE;
    //#else
    int i;
    kal_uint32 old_fl = ctx->frame_length;
    kal_uint32 calc_fl = 0;
    kal_uint32 readoutLength = ctx->readout_length;
    kal_uint32 readMargin = ctx->read_margin;
    kal_uint32 per_frame_ns = (kal_uint32)(((unsigned long long)ctx->frame_length *
        (unsigned long long)ctx->line_length * 1000000000) / (unsigned long long)ctx->pclk);

    /* NEED TO FIX start: support 1exp-2exp only; 3exp-?exp instead */
    if (previous_exp_cnt == 1)
        ns = 10000000;
    if (previous_exp_cnt > 1) {
        calc_fl = (readoutLength + readMargin);
        for (i = 1; i < previous_exp_cnt; i++)
            calc_fl += (previous_exp[i] + imgsensor_info.margin * previous_exp_cnt);

        ctx->frame_length = max(calc_fl, ctx->frame_length);
    }
    write_cmos_sensor_8(ctx, 0x0104, 0x01);
    write_frame_len(ctx, ctx->frame_length, get_cur_exp_cnt(ctx));
    write_cmos_sensor_8(ctx, 0x0104, 0x00);

    ctx->extend_frame_length_en = KAL_TRUE;

    ns = (kal_uint32)(((unsigned long long)(ctx->frame_length - old_fl) *
        (unsigned long long)ctx->line_length * 1000000000) / (unsigned long long)ctx->pclk);
    LOG_INF("new frame len = %d, old frame len = %d, per_frame_ns = %d, add more %d ns",
        ctx->frame_length, old_fl, per_frame_ns, ns);
    #endif
}

static void sensor_init(struct subdrv_ctx *ctx)
{
    LOG_INF("sensor_init start\n");
    imx709_table_write_cmos_sensor(ctx, imx709_init_setting,
        sizeof(imx709_init_setting)/sizeof(kal_uint16));
    imx709_table_write_cmos_sensor(ctx, imx709_rmsc_ip_setting,
        sizeof(imx709_rmsc_ip_setting)/sizeof(kal_uint16));
    set_mirror_flip(ctx, ctx->mirror);
    LOG_INF("sensor_init End\n");
}    /*sensor_init  */

static void preview_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(ctx, imx709_preview_setting,
        sizeof(imx709_preview_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
} /* preview_setting */

/*full size 30fps*/
static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(ctx, imx709_capture_setting,
        sizeof(imx709_capture_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(ctx, imx709_normal_video_setting,
        sizeof(imx709_normal_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(ctx, imx709_hs_video_setting,
        sizeof(imx709_hs_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(ctx, imx709_slim_video_setting,
        sizeof(imx709_slim_video_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom1_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(ctx, imx709_custom1_setting,
        sizeof(imx709_custom1_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom2_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(ctx, imx709_custom2_setting,
        sizeof(imx709_custom2_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom3_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(ctx, imx709_custom3_setting,
        sizeof(imx709_custom3_setting)/sizeof(kal_uint16));
    LOG_INF("%s end\n", __func__);
}

static void custom4_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(ctx, imx709_custom4_setting,
        sizeof(imx709_custom4_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom5_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(ctx, imx709_custom5_setting,
        sizeof(imx709_custom5_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom6_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(ctx, imx709_custom6_setting,
        sizeof(imx709_custom6_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom7_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(ctx, imx709_custom7_setting,
        sizeof(imx709_custom7_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom8_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    /*************MIPI output setting************/
    imx709_table_write_cmos_sensor(ctx, imx709_custom8_setting,
        sizeof(imx709_custom8_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom9_setting(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start\n", __func__);
    imx709_table_write_cmos_sensor(ctx, imx709_custom9_setting,
        sizeof(imx709_custom9_setting)/sizeof(kal_uint16));
     LOG_INF("%s end\n", __func__);
}

static void custom10_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("%s start\n", __func__);
	imx709_table_write_cmos_sensor(ctx, imx709_custom10_setting,
			sizeof(imx709_custom10_setting)/sizeof(kal_uint16));
	LOG_INF("%s end\n", __func__);
}

static void hdr_write_tri_shutter_w_gph(struct subdrv_ctx *ctx,
		kal_uint32 le, kal_uint32 me, kal_uint32 se, kal_bool gph)
{
	kal_uint16 exposure_cnt = 0;
	int i;

	if (le) {
		exposure_cnt++;
	}
	if (me) {
		exposure_cnt++;
	}
	if (se) {
		exposure_cnt++;
	}

	if (le) {
		le = (kal_uint16)max(imgsensor_info.min_shutter, (kal_uint32)le);
		le = round_up((le) / exposure_cnt, 4) * exposure_cnt;
	}
	if (me) {
		me = (kal_uint16)max(imgsensor_info.min_shutter, (kal_uint32)me);
		me = round_up((me) / exposure_cnt, 4) * exposure_cnt;
	}
	if (se) {
		se = (kal_uint16)max(imgsensor_info.min_shutter, (kal_uint32)se);
		se = round_up((se) / exposure_cnt, 4) * exposure_cnt;
	}

	ctx->frame_length =
		max((kal_uint32)(le + me + se + imgsensor_info.margin*exposure_cnt*exposure_cnt),
				ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, imgsensor_info.max_frame_length);

	for (i = 0; i < previous_exp_cnt; i++)
		previous_exp[i] = 0;
	previous_exp[0] = le;
	switch (exposure_cnt) {
		case 3:
			previous_exp[1] = me;
			previous_exp[2] = se;
			break;
		case 2:
			previous_exp[1] = se;
			previous_exp[2] = 0;
			break;
		case 1:
		default:
			previous_exp[1] = 0;
			previous_exp[2] = 0;
			break;
	}
	previous_exp_cnt = exposure_cnt;

	if (le)
		le = le / exposure_cnt;
	if (me)
		me = me / exposure_cnt;
	if (se)
		se = se / exposure_cnt;

	if (gph)
		set_cmos_sensor_8(ctx, 0x0104, 0x01);

	set_auto_flicker(ctx);
	// write_frame_len(ctx, ctx->frame_length);

	/* Long exposure */
	set_cmos_sensor_8(ctx, 0x0202, (le >> 8) & 0xFF);
	set_cmos_sensor_8(ctx, 0x0203, le & 0xFF);
	/* Muddle exposure */
	if (me) {
		/*MID_COARSE_INTEG_TIME[15:8]*/
		set_cmos_sensor_8(ctx, 0x313A, (me >> 8) & 0xFF);
		/*MID_COARSE_INTEG_TIME[7:0]*/
		set_cmos_sensor_8(ctx, 0x313B, me & 0xFF);
	}
	/* Short exposure */
	set_cmos_sensor_8(ctx, 0x0224, (se >> 8) & 0xFF);
	set_cmos_sensor_8(ctx, 0x0225, se & 0xFF);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			set_cmos_sensor_8(ctx, 0x0104, 0x00);

		commit_write_sensor(ctx);
	}

	LOG_INF("X! le:0x%x, me:0x%x, se:0x%x autoflicker_en %d frame_length %d\n",
			le, me, se, ctx->autoflicker_en, ctx->frame_length);
}

static void hdr_write_tri_shutter(struct subdrv_ctx *ctx,
		kal_uint32 le, kal_uint32 me, kal_uint32 se)
{
	hdr_write_tri_shutter_w_gph(ctx, le, me, se, KAL_TRUE);
}

static void hdr_write_tri_gain_w_gph(struct subdrv_ctx *ctx,
		kal_uint32 lg, kal_uint32 mg, kal_uint32 sg, kal_bool gph)
{
	kal_uint16 reg_lg, reg_mg, reg_sg;

	reg_lg = gain2reg(ctx, lg);
	reg_mg = mg ? gain2reg(ctx, mg) : 0;
	reg_sg = gain2reg(ctx, sg);

	ctx->gain = reg_lg;
	if (gph && !ctx->ae_ctrl_gph_en)
		set_cmos_sensor_8(ctx, 0x0104, 0x01);
	/* Long Gian */
	set_cmos_sensor_8(ctx, 0x0204, (reg_lg>>8) & 0xFF);
	set_cmos_sensor_8(ctx, 0x0205, reg_lg & 0xFF);
	/* Middle Gian */
	if (mg != 0) {
		set_cmos_sensor_8(ctx, 0x313C, (reg_mg>>8) & 0xFF);
		set_cmos_sensor_8(ctx, 0x313D, reg_mg & 0xFF);
	}
	/* Short Gian */
	set_cmos_sensor_8(ctx, 0x0216, (reg_sg>>8) & 0xFF);
	set_cmos_sensor_8(ctx, 0x0217, reg_sg & 0xFF);
	if (gph)
		set_cmos_sensor_8(ctx, 0x0104, 0x00);

	commit_write_sensor(ctx);

	LOG_INF(
			"lg:0x%x, reg_lg:0x%x, mg:0x%x, reg_mg:0x%x, sg:0x%x, reg_sg:0x%x\n",
			lg, reg_lg, mg, reg_mg, sg, reg_sg);
}

static void hdr_write_tri_gain(struct subdrv_ctx *ctx,
		kal_uint32 lg, kal_uint32 mg, kal_uint32 sg)
{
	hdr_write_tri_gain_w_gph(ctx, lg, mg, sg, KAL_TRUE);
}

static void write_sensor_QSC(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start write_sensor_QSC, qsc_is_valid:%d, qsc_flag:%d\n", __func__, qsc_is_valid, qsc_flag);
    if ((qsc_is_valid == 0x01) && !qsc_flag) {
        imx709_seq_write_cmos_sensor(ctx, 0x1000, imx709_QSC_setting,QSC_SIZE);
        qsc_flag = 1;
    }
    LOG_INF("%s end\n", __func__);
}

#if lrc_cal_en
static void write_sensor_lrc(struct subdrv_ctx *ctx)
{
    LOG_INF("%s start write_sensor_LRC, lrc_is_valid:%d, lrc_flag:%d", __func__, lrc_is_valid, lrc_flag);
    if ((lrc_is_valid == 0x01) && !lrc_flag) {
        imx709_table_write_cmos_sensor(ctx, imx709_LRC_setting,
            sizeof(imx709_LRC_setting) / sizeof(kal_uint16));
        lrc_flag = 1;
    }
    LOG_INF("%s end\n", __func__);
}
#endif
/*************************************************************************
 * FUNCTION
 *    get_imgsensor_id
 *
 * DESCRIPTION
 *    This function get the sensor ID
 *
 * PARAMETERS
 *    *sensorID : return the sensor ID
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    static bool first_read = KAL_TRUE;
    qsc_flag = 0;
    lrc_flag = 0;
    /*sensor have two i2c address 0x34 & 0x20,
     *we should detect the module used i2c address
     */
    LOG_INF("imx709 Enter %s.", __func__);
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
        do {
            *sensor_id = ((read_cmos_sensor_8(ctx, 0x0016) << 8)
                    | read_cmos_sensor_8(ctx, 0x0017));
            LOG_INF(
                "read_0x0000=0x%x, 0x0001=0x%x, 0x0000_0001=0x%x\n",
                read_cmos_sensor_8(ctx, 0x0016),
                read_cmos_sensor_8(ctx, 0x0017),
                read_cmos_sensor(ctx, 0x0000));
            if (*sensor_id == IMX709_SENSOR_ID) {
                *sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
                    ctx->i2c_write_id, *sensor_id);
                if(first_read){
                    read_module_data(ctx);
                    read_imx709_LRC_22017(ctx, imx709_LRC_setting);
                    read_imx709_QSC_22017(ctx, imx709_QSC_setting);
                    first_read = KAL_FALSE;
                }
                return ERROR_NONE;
            }

            LOG_INF("Read sensor id fail, id: 0x%x\n",
                ctx->i2c_write_id);
            retry--;
        } while (retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        /*if Sensor ID is not correct,
         *Must set *sensor_id to 0xFFFFFFFF
         */
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int open(struct subdrv_ctx *ctx)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    UINT32 sensor_id = 0;

    LOG_INF("IMX709 open start\n");
    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
     *we should detect the module used i2c address
     */
    if (ctx->is_esd_enable == true) {
        LOG_INF("Esd reset occur, reinit vcm");
        write_imx709_vcm_22017_init(ctx);
        ctx->is_esd_enable = false;
    }

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
        do {
            sensor_id = ((read_cmos_sensor_8(ctx, 0x0016) << 8)
                    | read_cmos_sensor_8(ctx, 0x0017));
            LOG_INF(
                "read_0x0000=0x%x, 0x0001=0x%x, 0x0000_0001=0x%x\n",
                read_cmos_sensor_8(ctx, 0x0016),
                read_cmos_sensor_8(ctx, 0x0017),
                read_cmos_sensor(ctx, 0x0000));
            if (sensor_id == IMX709_SENSOR_ID) {
                sensor_id = imgsensor_info.sensor_id;
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
                    ctx->i2c_write_id, sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n",
                ctx->i2c_write_id);
            retry--;
        } while (retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id) {
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    /* initail sequence write in  */
    sensor_init(ctx);
    write_sensor_QSC(ctx);
#if lrc_cal_en
    write_sensor_lrc(ctx);
#endif
    ctx->autoflicker_en = KAL_FALSE;
    ctx->sensor_mode = IMGSENSOR_MODE_INIT;
    ctx->shutter = 0x3D0;
    ctx->gain = 0x100;
    ctx->pclk = imgsensor_info.pre.pclk;
    ctx->frame_length = imgsensor_info.pre.framelength;
    ctx->line_length = imgsensor_info.pre.linelength;
    ctx->min_frame_length = imgsensor_info.pre.framelength;
    ctx->dummy_pixel = 0;
    ctx->dummy_line = 0;
    ctx->ihdr_mode = 0;
    ctx->test_pattern = 0;
    ctx->current_fps = imgsensor_info.pre.max_framerate;
    ctx->fast_mode_on = KAL_FALSE;
    LOG_INF("IMX709 open End\n");
    return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int close(struct subdrv_ctx *ctx)
{
    LOG_INF("E\n");
    /* No Need to implement this function */
    streaming_control(ctx, KAL_FALSE);
    qsc_flag = 0;
    lrc_flag = 0;
    return ERROR_NONE;
} /* close */

/*write AWB gain to sensor*/
static kal_uint16 imx709_feedback_awbgain[] = {
    0x0b90, 0x00,
    0x0b91, 0x01,
    0x0b92, 0x00,
    0x0b93, 0x01,
    0x0B8E, 0x01,
    0x0B8F, 0x00,
    0x0B94, 0x01,
    0x0B95, 0x00,
};

static void feedback_awbgain(struct subdrv_ctx *ctx, kal_uint32 r_gain, kal_uint32 b_gain)
{
    UINT32 r_gain_int = 0;
    UINT32 b_gain_int = 0;

    LOG_INF("IMX709 feedback_awbgain r_gain: %d, b_gain: %d\n", r_gain, b_gain);
    r_gain_int = r_gain / 512;
    b_gain_int = b_gain / 512;

    imx709_feedback_awbgain[1] = r_gain_int;
    imx709_feedback_awbgain[3] = (r_gain - r_gain_int * 512) / 2;
    imx709_feedback_awbgain[5] = b_gain_int;
    imx709_feedback_awbgain[7] = (b_gain - b_gain_int * 512) / 2;
    imx709_table_write_cmos_sensor(ctx, imx709_feedback_awbgain,
        sizeof(imx709_feedback_awbgain)/sizeof(kal_uint16));
}

/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("imx709:Enter %s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
    ctx->pclk = imgsensor_info.pre.pclk;
    ctx->line_length = imgsensor_info.pre.linelength;
    ctx->frame_length = imgsensor_info.pre.framelength;
    ctx->min_frame_length = imgsensor_info.pre.framelength;
    ctx->autoflicker_en = KAL_FALSE;

    preview_setting(ctx);
    LOG_INF("imx709:Leave %s., w*h:%d x %d.\n", __func__, ctx->line_length, ctx->frame_length);
    mdelay(8);

    return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (ctx->current_fps != imgsensor_info.cap.max_framerate){
        LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
                ctx->current_fps,
                imgsensor_info.cap.max_framerate / 10);
    }
    ctx->pclk = imgsensor_info.cap.pclk;
    ctx->line_length = imgsensor_info.cap.linelength;
    ctx->frame_length = imgsensor_info.cap.framelength;
    ctx->min_frame_length = imgsensor_info.cap.framelength;
    ctx->autoflicker_en = KAL_FALSE;


    capture_setting(ctx, ctx->current_fps);
    set_mirror_flip(ctx, ctx->mirror);

    return ERROR_NONE;
}    /* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
    ctx->pclk = imgsensor_info.normal_video.pclk;
    ctx->line_length = imgsensor_info.normal_video.linelength;
    ctx->frame_length = imgsensor_info.normal_video.framelength;
    ctx->min_frame_length = imgsensor_info.normal_video.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    normal_video_setting(ctx, ctx->current_fps);
    set_mirror_flip(ctx, ctx->mirror);

    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    ctx->pclk = imgsensor_info.hs_video.pclk;
    /*ctx->video_mode = KAL_TRUE;*/
    ctx->line_length = imgsensor_info.hs_video.linelength;
    ctx->frame_length = imgsensor_info.hs_video.framelength;
    ctx->min_frame_length = imgsensor_info.hs_video.framelength;
    ctx->dummy_line = 0;
    ctx->dummy_pixel = 0;
    /*ctx->current_fps = 300;*/
    ctx->autoflicker_en = KAL_FALSE;
    hs_video_setting(ctx);
    //set_mirror_flip(ctx, ctx->mirror);

    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s. 720P@240FPS\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    ctx->pclk = imgsensor_info.slim_video.pclk;
    /*ctx->video_mode = KAL_TRUE;*/
    ctx->line_length = imgsensor_info.slim_video.linelength;
    ctx->frame_length = imgsensor_info.slim_video.framelength;
    ctx->min_frame_length = imgsensor_info.slim_video.framelength;
    ctx->dummy_line = 0;
    ctx->dummy_pixel = 0;
    /*ctx->current_fps = 300;*/
    ctx->autoflicker_en = KAL_FALSE;
    slim_video_setting(ctx);
    //set_mirror_flip(ctx, ctx->mirror);

    return ERROR_NONE;
}    /* slim_video */

static kal_uint32 custom1(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    ctx->pclk = imgsensor_info.custom1.pclk;
    ctx->line_length = imgsensor_info.custom1.linelength;
    ctx->frame_length = imgsensor_info.custom1.framelength;
    ctx->min_frame_length = imgsensor_info.custom1.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom1_setting(ctx);
    set_mirror_flip(ctx, ctx->mirror);
    return ERROR_NONE;
}    /* custom1 */

static kal_uint32 custom2(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    ctx->pclk = imgsensor_info.custom2.pclk;
    ctx->line_length = imgsensor_info.custom2.linelength;
    ctx->frame_length = imgsensor_info.custom2.framelength;
    ctx->min_frame_length = imgsensor_info.custom2.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom2_setting(ctx);
    //write_sensor_QSC(ctx);

    set_mirror_flip(ctx, ctx->mirror);

    return ERROR_NONE;
}    /* custom2 */

static kal_uint32 custom3(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    ctx->pclk = imgsensor_info.custom3.pclk;
    ctx->line_length = imgsensor_info.custom3.linelength;
    ctx->frame_length = imgsensor_info.custom3.framelength;
    ctx->min_frame_length = imgsensor_info.custom3.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom3_setting(ctx);
    return ERROR_NONE;
}    /* custom3 */

static kal_uint32 custom4(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    ctx->pclk = imgsensor_info.custom4.pclk;
    ctx->line_length = imgsensor_info.custom4.linelength;
    ctx->frame_length = imgsensor_info.custom4.framelength;
    ctx->min_frame_length = imgsensor_info.custom4.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom4_setting(ctx);
    return ERROR_NONE;
}    /* custom4 */

static kal_uint32 custom5(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    ctx->pclk = imgsensor_info.custom5.pclk;
    ctx->line_length = imgsensor_info.custom5.linelength;
    ctx->frame_length = imgsensor_info.custom5.framelength;
    ctx->min_frame_length = imgsensor_info.custom5.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom5_setting(ctx);
    return ERROR_NONE;
}

static kal_uint32 custom6(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM6;
    ctx->pclk = imgsensor_info.custom6.pclk;
    ctx->line_length = imgsensor_info.custom6.linelength;
    ctx->frame_length = imgsensor_info.custom6.framelength;
    ctx->min_frame_length = imgsensor_info.custom6.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom6_setting(ctx);
    return ERROR_NONE;
}

static kal_uint32 custom7(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM7;
    ctx->pclk = imgsensor_info.custom7.pclk;
    ctx->line_length = imgsensor_info.custom7.linelength;
    ctx->frame_length = imgsensor_info.custom7.framelength;
    ctx->min_frame_length = imgsensor_info.custom7.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom7_setting(ctx);

    return ERROR_NONE;
}

static kal_uint32 custom8(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM8;
    ctx->pclk = imgsensor_info.custom8.pclk;
    ctx->line_length = imgsensor_info.custom8.linelength;
    ctx->frame_length = imgsensor_info.custom8.framelength;
    ctx->min_frame_length = imgsensor_info.custom8.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom8_setting(ctx);

    set_mirror_flip(ctx, ctx->mirror);

    return ERROR_NONE;
}

static kal_uint32 custom9(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM9;
    ctx->pclk = imgsensor_info.custom9.pclk;
    ctx->line_length = imgsensor_info.custom9.linelength;
    ctx->frame_length = imgsensor_info.custom9.framelength;
    ctx->min_frame_length = imgsensor_info.custom9.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom9_setting(ctx);
    set_mirror_flip(ctx, ctx->mirror);
    return ERROR_NONE;
}

static kal_uint32 custom10(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
              MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);
    ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM10;
    ctx->pclk = imgsensor_info.custom10.pclk;
    ctx->line_length = imgsensor_info.custom10.linelength;
    ctx->frame_length = imgsensor_info.custom10.framelength;
    ctx->min_frame_length = imgsensor_info.custom10.framelength;
    ctx->autoflicker_en = KAL_FALSE;
    custom10_setting(ctx);
    set_mirror_flip(ctx, ctx->mirror);
    return ERROR_NONE;
}

#define FMC_GPH_START        do { \
                    write_cmos_sensor_8(ctx, 0x0104, 0x01); \
                    write_cmos_sensor_8(ctx, 0x3010, 0x02); \
                } while (0)

#define FMC_GPH_END        do { \
                    write_cmos_sensor_8(ctx, 0x0104, 0x00); \
                } while (0)
enum {
    SHUTTER_LE_FRM_1 = 0,
    SHUTTER_ME_FRM_1,
    SHUTTER_SE_FRM_1,
    SHUTTER_SSE_FRM_1,
    SHUTTER_SSSE_FRM_1,
    GAIN_LE_FRM_1,
    GAIN_ME_FRM_1,
    GAIN_SE_FRM_1,
    GAIN_SSE_FRM_1,
    GAIN_SSSE_FRM_1,
};

static kal_uint32 seamless_switch(struct subdrv_ctx *ctx,
        enum SENSOR_SCENARIO_ID_ENUM scenario_id, uint32_t *ae_ctrl)
{
    ctx->extend_frame_length_en = KAL_FALSE;

    switch (scenario_id) {

    case SENSOR_SCENARIO_ID_CUSTOM6:
    {
        unsigned short imx709_seamless_custom6[] = {
            0x0342, 0x2D,
            0x0343, 0xF0,
            0x0340, 0x09,
            0x0341, 0x90,
            0x3148, 0x04,
            0x3A90, 0x00,
            0x3A91, 0x00,
            0x3A94, 0x00,
            0x3A95, 0x00,
            0x3A96, 0x00,
            0x3A97, 0x00,
            0x3A9A, 0x00,
            0x3A9B, 0x00,
            0x3AA0, 0x00,
            0x3AA1, 0x00,
            0x3AA4, 0x00,
            0x3AA5, 0x00,
            0x3AB1, 0x00,
            0x3AB3, 0x00,
            0x3AB5, 0x00,
            0x3AB7, 0x00,
            0x3AC8, 0x00,
            0x3AC9, 0x00,
            0x3ACA, 0x00,
            0x3ACB, 0x00,
            0x0202, 0x09,
            0x0203, 0x78,
            0x0224, 0x01,
            0x0225, 0xF4,
            0x3102, 0x01,
            0x3170, 0x00,
            0xBCAF, 0x00,
        };

        ctx->sensor_mode = scenario_id;
        ctx->autoflicker_en = KAL_FALSE;
        ctx->pclk = imgsensor_info.custom6.pclk;
        ctx->line_length = imgsensor_info.custom6.linelength;
        ctx->frame_length = imgsensor_info.custom6.framelength;
        ctx->min_frame_length = imgsensor_info.custom6.framelength;

        FMC_GPH_START;
        imx709_table_write_cmos_sensor(ctx, imx709_seamless_custom6,
                sizeof(imx709_seamless_custom6) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call SENSOR_SCENARIO_ID_CUSTOM6 %d %d",
                    ae_ctrl[SHUTTER_LE_FRM_1], ae_ctrl[GAIN_LE_FRM_1]);
            set_shutter(ctx, ae_ctrl[SHUTTER_LE_FRM_1], KAL_FALSE);
            set_gain(ctx, ae_ctrl[GAIN_LE_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM7:
    {
        unsigned short imx709_seamless_custom7[] = {
            0x0342, 0x1D,
            0x0343, 0x20,
            0x0340, 0x07,
            0x0341, 0x88,
            0x3148, 0x00,
            0x3A90, 0x03,
            0x3A91, 0x66,
            0x3A94, 0x03,
            0x3A95, 0x66,
            0x3A96, 0x03,
            0x3A97, 0x88,
            0x3A9A, 0x03,
            0x3A9B, 0x88,
            0x3AA0, 0x01,
            0x3AA1, 0xAC,
            0x3AA4, 0x01,
            0x3AA5, 0xAC,
            0x3AB1, 0xF0,
            0x3AB3, 0xF0,
            0x3AB5, 0xF0,
            0x3AB7, 0xF0,
            0x3AC8, 0x02,
            0x3AC9, 0xC8,
            0x3ACA, 0x02,
            0x3ACB, 0xBC,
            0x0202, 0x06,
            0x0203, 0x80,
            0x0224, 0x00,
            0x0225, 0xD0,
            0x3102, 0x00,
            0x3170, 0x01,
            0xbcaf, 0x01,
        };

        ctx->sensor_mode = scenario_id;
        ctx->autoflicker_en = KAL_FALSE;
        ctx->pclk = imgsensor_info.custom7.pclk;
        ctx->line_length = imgsensor_info.custom7.linelength;
        ctx->frame_length = imgsensor_info.custom7.framelength;
        ctx->min_frame_length = imgsensor_info.custom7.framelength;

        FMC_GPH_START;
        imx709_table_write_cmos_sensor(ctx, imx709_seamless_custom7,
                sizeof(imx709_seamless_custom7) / sizeof(kal_uint16));
        if (ae_ctrl) {
            LOG_INF("call SENSOR_SCENARIO_ID_CUSTOM7 %d %d %d %d",
                    ae_ctrl[SHUTTER_LE_FRM_1],
                    ae_ctrl[SHUTTER_ME_FRM_1],
                    ae_ctrl[GAIN_LE_FRM_1],
                    ae_ctrl[GAIN_ME_FRM_1]);
            hdr_write_tri_shutter_w_gph(ctx,
                    ae_ctrl[SHUTTER_LE_FRM_1],
                    0,
                    ae_ctrl[SHUTTER_ME_FRM_1], KAL_FALSE);
            hdr_write_tri_gain_w_gph(ctx,
                    ae_ctrl[GAIN_LE_FRM_1],
                    0,
                    ae_ctrl[GAIN_ME_FRM_1], KAL_FALSE);
        }
        FMC_GPH_END;
    }
        break;
	case SENSOR_SCENARIO_ID_CUSTOM8:
	{
		unsigned short imx709_seamless_custom8[] = {
			0x0342, 0x2D,
			0x0343, 0xF0,
			0x0340, 0x09,
			0x0341, 0x90,
			0x3148, 0x04,
			0x3A90, 0x00,
			0x3A91, 0x00,
			0x3A94, 0x00,
			0x3A95, 0x00,
			0x3A96, 0x00,
			0x3A97, 0x00,
			0x3A9A, 0x00,
			0x3A9B, 0x00,
			0x3AA0, 0x00,
			0x3AA1, 0x00,
			0x3AA4, 0x00,
			0x3AA5, 0x00,
			0x3AB1, 0x00,
			0x3AB3, 0x00,
			0x3AB5, 0x00,
			0x3AB7, 0x00,
			0x3AC8, 0x00,
			0x3AC9, 0x00,
			0x3ACA, 0x00,
			0x3ACB, 0x00,
			0x0202, 0x09,
			0x0203, 0x78,
			0x0224, 0x01,
			0x0225, 0xF4,
			0x3102, 0x01,
			0x3170, 0x00,
			0xBCAF, 0x00,
		};

		ctx->sensor_mode = scenario_id;
		ctx->autoflicker_en = KAL_FALSE;
		ctx->pclk = imgsensor_info.custom8.pclk;
		ctx->line_length = imgsensor_info.custom8.linelength;
		ctx->frame_length = imgsensor_info.custom8.framelength;
		ctx->min_frame_length = imgsensor_info.custom8.framelength;

		FMC_GPH_START;
		imx709_table_write_cmos_sensor(ctx, imx709_seamless_custom8,
				sizeof(imx709_seamless_custom8) / sizeof(kal_uint16));
		if (ae_ctrl) {
			LOG_INF("call SENSOR_SCENARIO_ID_CUSTOM8 %d %d",
					ae_ctrl[0], ae_ctrl[5]);
			set_shutter(ctx, ae_ctrl[0], KAL_FALSE);
			set_gain(ctx, ae_ctrl[5], KAL_FALSE);
		}
		FMC_GPH_END;
}
		break;
	case SENSOR_SCENARIO_ID_CUSTOM4:
	{
			unsigned short imx709_seamless_custom4[] = {
				0x0342, 0x1D,
				0x0343, 0x20,
				0x0340, 0x07,
				0x0341, 0x88,
				0x3148, 0x00,
				0x3A90, 0x03,
				0x3A91, 0x66,
				0x3A94, 0x03,
				0x3A95, 0x66,
				0x3A96, 0x03,
				0x3A97, 0x88,
				0x3A9A, 0x03,
				0x3A9B, 0x88,
				0x3AA0, 0x01,
				0x3AA1, 0xAC,
				0x3AA4, 0x01,
				0x3AA5, 0xAC,
				0x3AB1, 0xF0,
				0x3AB3, 0xF0,
				0x3AB5, 0xF0,
				0x3AB7, 0xF0,
				0x3AC8, 0x02,
				0x3AC9, 0xC8,
				0x3ACA, 0x02,
				0x3ACB, 0xBC,
				0x0202, 0x06,
				0x0203, 0x80,
				0x0224, 0x00,
				0x0225, 0xD0,
				0x3102, 0x00,
				0x3170, 0x01,
				0xbcaf, 0x01,
			};

			ctx->sensor_mode = scenario_id;
			ctx->autoflicker_en = KAL_FALSE;
			ctx->pclk = imgsensor_info.custom4.pclk;
			ctx->line_length = imgsensor_info.custom4.linelength;
			ctx->frame_length = imgsensor_info.custom4.framelength;
			ctx->min_frame_length = imgsensor_info.custom4.framelength;

			FMC_GPH_START;
			imx709_table_write_cmos_sensor(ctx, imx709_seamless_custom4,
					sizeof(imx709_seamless_custom4) / sizeof(kal_uint16));
			if (ae_ctrl) {
				LOG_INF("call SENSOR_SCENARIO_ID_CUSTOM4 %d %d %d %d %d %d",
						ae_ctrl[0],
						0,
						ae_ctrl[1],
						ae_ctrl[5],
						0,
						ae_ctrl[6]);
				hdr_write_tri_shutter_w_gph(ctx,
						ae_ctrl[0],
						0,
						ae_ctrl[1],
						KAL_FALSE);
				hdr_write_tri_gain_w_gph(ctx,
						ae_ctrl[5],
						0,
						ae_ctrl[6],
						KAL_FALSE);
			}
			FMC_GPH_END;
		}
		break;
    default:
    {
        LOG_INF("%s error! wrong setting in set_seamless_switch = %d",
                __func__, scenario_id);
        return 0xff;
    }
    }

    ctx->fast_mode_on = KAL_TRUE;
    LOG_INF("%s success, scenario is switched to %d", __func__, scenario_id);
    return 0;
}

static int get_resolution(struct subdrv_ctx *ctx, MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    int i = 0;
    LOG_INF("E\n");
    for (i = SENSOR_SCENARIO_ID_MIN; i < SENSOR_SCENARIO_ID_MAX; i++) {
        if (i < imgsensor_info.sensor_mode_num) {
            sensor_resolution->SensorWidth[i] = imgsensor_winsize_info[i].w2_tg_size;
            sensor_resolution->SensorHeight[i] = imgsensor_winsize_info[i].h2_tg_size;
        } else {
            sensor_resolution->SensorWidth[i] = 0;
            sensor_resolution->SensorHeight[i] = 0;
        }
    }
    return ERROR_NONE;
} /* get_resolution */

static int get_info(struct subdrv_ctx *ctx, enum SENSOR_SCENARIO_ID_ENUM scenario_id,
               MSDK_SENSOR_INFO_STRUCT *sensor_info,
               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    //LOG_INF("scenario_id = %d\n", scenario_id);

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SensorOutputDataFormat =
        imgsensor_info.sensor_output_dataformat;

    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_PREVIEW] =
        imgsensor_info.pre_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_CAPTURE] =
        imgsensor_info.cap_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_VIDEO] =
        imgsensor_info.video_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO] =
        imgsensor_info.hs_video_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_SLIM_VIDEO] =
        imgsensor_info.slim_video_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM1] =
        imgsensor_info.custom1_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM2] =
        imgsensor_info.custom2_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM3] =
        imgsensor_info.custom3_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM4] =
        imgsensor_info.custom4_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM5] =
        imgsensor_info.custom5_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM6] =
        imgsensor_info.custom6_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM7] =
        imgsensor_info.custom7_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM8] =
        imgsensor_info.custom8_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM9] =
        imgsensor_info.custom9_delay_frame;
    sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM10] =
        imgsensor_info.custom10_delay_frame;
    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
    sensor_info->AESensorGainDelayFrame =
        imgsensor_info.ae_sensor_gain_delay_frame;
    sensor_info->AEISPGainDelayFrame =
        imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV;

    sensor_info->HDR_Support = HDR_SUPPORT_STAGGER_FDOL;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
    sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
    sensor_info->SensorPacketECCOrder = 1;
    sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

    return ERROR_NONE;
}    /*    get_info  */

static int control(struct subdrv_ctx *ctx, enum SENSOR_SCENARIO_ID_ENUM scenario_id,
            MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("In control:scenario_id = %d\n", scenario_id);
    ctx->current_scenario_id = scenario_id;
    switch (scenario_id) {
    case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        preview(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        capture(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        normal_video(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        hs_video(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        slim_video(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM1:
        custom1(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM2:
        custom2(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM3:
        custom3(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM4:
        custom4(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM5:
        custom5(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM6:
        custom6(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM7:
        custom7(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM8:
        custom8(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM9:
        custom9(ctx, image_window, sensor_config_data);
        break;
    case SENSOR_SCENARIO_ID_CUSTOM10:
        custom10(ctx, image_window, sensor_config_data);
        break;
    default:
        LOG_INF("Error ScenarioId setting");
        preview(ctx, image_window, sensor_config_data);
        return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control(ctx) */

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    /* SetVideoMode Function should fix framerate */
    if (framerate == 0)
        /* Dynamic frame rate */
        return ERROR_NONE;
    if ((framerate == 300) && (ctx->autoflicker_en == KAL_TRUE))
        ctx->current_fps = 296;
    else if ((framerate == 150) && (ctx->autoflicker_en == KAL_TRUE))
        ctx->current_fps = 146;
    else
        ctx->current_fps = framerate;
    set_max_framerate(ctx, ctx->current_fps, 1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx, kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
    if (enable) /*enable auto flicker*/
        ctx->autoflicker_en = KAL_TRUE;
    else /*Cancel Auto flick*/
        ctx->autoflicker_en = KAL_FALSE;
    return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
        enum SENSOR_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
    case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        frame_length = imgsensor_info.pre.pclk / framerate * 10
                / imgsensor_info.pre.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
        ? (frame_length - imgsensor_info.pre.framelength) : 0;
        ctx->frame_length =
            imgsensor_info.pre.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        if (framerate == 0)
            return ERROR_NONE;
        frame_length = imgsensor_info.normal_video.pclk /
                framerate * 10 /
                imgsensor_info.normal_video.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.normal_video.framelength)
        ? (frame_length - imgsensor_info.normal_video.framelength)
        : 0;
        ctx->frame_length =
            imgsensor_info.normal_video.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        if (ctx->current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF(
                "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
                , framerate, imgsensor_info.cap.max_framerate/10);
        frame_length = imgsensor_info.cap.pclk / framerate * 10
                / imgsensor_info.cap.linelength;
        ctx->dummy_line =
                (frame_length > imgsensor_info.cap.framelength)
                ? (frame_length - imgsensor_info.cap.framelength) : 0;
        ctx->frame_length =
                imgsensor_info.cap.framelength
                + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        frame_length = imgsensor_info.hs_video.pclk / framerate * 10
                / imgsensor_info.hs_video.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.hs_video.framelength)
              ? (frame_length - imgsensor_info.hs_video.framelength)
              : 0;
        ctx->frame_length =
            imgsensor_info.hs_video.framelength
                + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        frame_length = imgsensor_info.slim_video.pclk / framerate * 10
            / imgsensor_info.slim_video.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.slim_video.framelength)
            ? (frame_length - imgsensor_info.slim_video.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.slim_video.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM1:
        frame_length = imgsensor_info.custom1.pclk / framerate * 10
                / imgsensor_info.custom1.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom1.framelength)
            ? (frame_length - imgsensor_info.custom1.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom1.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM2:
        frame_length = imgsensor_info.custom2.pclk / framerate * 10
                / imgsensor_info.custom2.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom2.framelength)
            ? (frame_length - imgsensor_info.custom2.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom2.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM3:
        frame_length = imgsensor_info.custom3.pclk / framerate * 10
                / imgsensor_info.custom3.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom3.framelength)
            ? (frame_length - imgsensor_info.custom3.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom3.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM4:
        frame_length = imgsensor_info.custom4.pclk / framerate * 10
                / imgsensor_info.custom4.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom4.framelength)
            ? (frame_length - imgsensor_info.custom4.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom4.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM5:
        frame_length = imgsensor_info.custom5.pclk / framerate * 10
                / imgsensor_info.custom5.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom5.framelength)
            ? (frame_length - imgsensor_info.custom5.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom5.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM6:
        frame_length = imgsensor_info.custom6.pclk / framerate * 10
                / imgsensor_info.custom6.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom6.framelength)
            ? (frame_length - imgsensor_info.custom6.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom6.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM7:
        frame_length = imgsensor_info.custom7.pclk / framerate * 10
                / imgsensor_info.custom7.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom7.framelength)
            ? (frame_length - imgsensor_info.custom7.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom7.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM8:
        frame_length = imgsensor_info.custom8.pclk / framerate * 10
                / imgsensor_info.custom8.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom8.framelength)
            ? (frame_length - imgsensor_info.custom8.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom8.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM9:
        frame_length = imgsensor_info.custom9.pclk / framerate * 10
                / imgsensor_info.custom9.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom9.framelength)
            ? (frame_length - imgsensor_info.custom9.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom9.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    case SENSOR_SCENARIO_ID_CUSTOM10:
        frame_length = imgsensor_info.custom10.pclk / framerate * 10
                / imgsensor_info.custom10.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.custom10.framelength)
            ? (frame_length - imgsensor_info.custom10.framelength)
            : 0;
        ctx->frame_length =
            imgsensor_info.custom10.framelength
            + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        break;
    default:  /*coding with  preview scenario by default*/
        frame_length = imgsensor_info.pre.pclk / framerate * 10
            / imgsensor_info.pre.linelength;
        ctx->dummy_line =
            (frame_length > imgsensor_info.pre.framelength)
            ? (frame_length - imgsensor_info.pre.framelength) : 0;
        ctx->frame_length =
            imgsensor_info.pre.framelength + ctx->dummy_line;
        ctx->min_frame_length = ctx->frame_length;
        if (ctx->frame_length > ctx->shutter) {
            write_cmos_sensor_8(ctx, 0x0104, 0x01);
            set_dummy(ctx);
            write_cmos_sensor_8(ctx, 0x0104, 0x00);
        }
        LOG_INF("error scenario_id = %d, we use preview scenario\n",
            scenario_id);
        break;
    }
    return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
        enum SENSOR_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
    case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        *framerate = imgsensor_info.pre.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        *framerate = imgsensor_info.normal_video.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        *framerate = imgsensor_info.cap.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        *framerate = imgsensor_info.hs_video.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        *framerate = imgsensor_info.slim_video.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM1:
        *framerate = imgsensor_info.custom1.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM2:
        *framerate = imgsensor_info.custom2.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM3:
        *framerate = imgsensor_info.custom3.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM4:
        *framerate = imgsensor_info.custom4.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM5:
        *framerate = imgsensor_info.custom5.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM6:
        *framerate = imgsensor_info.custom6.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM7:
        *framerate = imgsensor_info.custom7.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM8:
        *framerate = imgsensor_info.custom8.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM9:
        *framerate = imgsensor_info.custom9.max_framerate;
        break;
    case SENSOR_SCENARIO_ID_CUSTOM10:
        *framerate = imgsensor_info.custom10.max_framerate;
        break;
    default:
        break;
    }

    return ERROR_NONE;
}

static kal_uint32 get_fine_integ_line_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id, MUINT32 *fine_integ_line)
{
	/*
	   switch (scenario_id) {
	   case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	   case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	   case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	   case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	   case SENSOR_SCENARIO_ID_SLIM_VIDEO:
	   case SENSOR_SCENARIO_ID_CUSTOM1:
	   case SENSOR_SCENARIO_ID_CUSTOM2:
	   case SENSOR_SCENARIO_ID_CUSTOM3:
	   case SENSOR_SCENARIO_ID_CUSTOM4:
	   case SENSOR_SCENARIO_ID_CUSTOM5:
	   case SENSOR_SCENARIO_ID_CUSTOM6:
	   case SENSOR_SCENARIO_ID_CUSTOM7:
	   case SENSOR_SCENARIO_ID_CUSTOM8:
	   case SENSOR_SCENARIO_ID_CUSTOM9:
	   case SENSOR_SCENARIO_ID_CUSTOM10:
	 *fine_integ_line = fine_integ_line_table[scenario_id];
	 break;
	 default:
	 break;
	 }
	 */
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_uint32 mode)
{
    LOG_INF("mode: %d\n", mode);

    //1:Solid Color 2:Color bar 5:black
    if (mode) {
        switch(mode) {
        case 5:
            write_cmos_sensor_8(ctx, 0x020E, 0x00);
            write_cmos_sensor_8(ctx, 0x0218, 0x00);
            write_cmos_sensor_8(ctx, 0x3021, 0x00);
            break;
        default:
            write_cmos_sensor_8(ctx, 0x0601, mode);
            break;
        }
    } else if (ctx->test_pattern) {
        write_cmos_sensor_8(ctx, 0x0601, 0x00); /*No pattern*/
        write_cmos_sensor_8(ctx, 0x020E, 0x01);
        write_cmos_sensor_8(ctx, 0x0218, 0x01);
        write_cmos_sensor_8(ctx, 0x3021, 0x40);
    }

    ctx->test_pattern = mode;
    return ERROR_NONE;
}

static kal_uint32 set_test_pattern_data(struct subdrv_ctx *ctx, struct mtk_test_pattern_data *data)
{
    pr_debug("test_patterndata mode = %d  R = %x, Gr = %x,Gb = %x,B = %x\n", ctx->test_pattern,
        data->Channel_R >> 22, data->Channel_Gr >> 22,
        data->Channel_Gb >> 22, data->Channel_B >> 22);

    if (1 == ctx->test_pattern) {
        write_cmos_sensor_8(ctx, 0x0602, (data->Channel_R >> 30) & 0x3);
        write_cmos_sensor_8(ctx, 0x0603, (data->Channel_R >> 22) & 0xff);
        write_cmos_sensor_8(ctx, 0x0604, (data->Channel_Gr >> 30) & 0x3);
        write_cmos_sensor_8(ctx, 0x0605, (data->Channel_Gr >> 22) & 0xff);
        write_cmos_sensor_8(ctx, 0x0606, (data->Channel_B >> 30) & 0x3);
        write_cmos_sensor_8(ctx, 0x0607, (data->Channel_B >> 22) & 0xff);
        write_cmos_sensor_8(ctx, 0x0608, (data->Channel_Gb >> 30) & 0x3);
        write_cmos_sensor_8(ctx, 0x0609, (data->Channel_Gb >> 22) & 0xff);
    }

    return ERROR_NONE;
}

static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
                 UINT8 *feature_para, UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
    UINT16 *feature_data_16 = (UINT16 *) feature_para;
    UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
    UINT32 *feature_data_32 = (UINT32 *) feature_para;
    unsigned long long *feature_data = (unsigned long long *) feature_para;
    UINT32 *pAeCtrls;
    /* unsigned long long *feature_return_para
     *  = (unsigned long long *) feature_para;
     */
    struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    //struct SENSOR_VC_INFO_STRUCT *pvcinfo;
    /* SET_SENSOR_AWB_GAIN *pSetSensorAWB
     *  = (SET_SENSOR_AWB_GAIN *)feature_para;
     */
    //struct SENSOR_VC_INFO2_STRUCT *pvcinfo2;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
        = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
    case SENSOR_FEATURE_GET_OUTPUT_FORMAT_BY_SCENARIO:
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        case SENSOR_SCENARIO_ID_CUSTOM1:
        case SENSOR_SCENARIO_ID_CUSTOM2:
        case SENSOR_SCENARIO_ID_CUSTOM3:
        case SENSOR_SCENARIO_ID_CUSTOM4:
        case SENSOR_SCENARIO_ID_CUSTOM5:
        case SENSOR_SCENARIO_ID_CUSTOM6:
        case SENSOR_SCENARIO_ID_CUSTOM7:
        case SENSOR_SCENARIO_ID_CUSTOM8:
        case SENSOR_SCENARIO_ID_CUSTOM9:
        case SENSOR_SCENARIO_ID_CUSTOM10:
            *(feature_data + 1)
            = (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
                imgsensor_info.sensor_output_dataformat;
            break;
        }
    break;
    case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
        if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
            *(feature_data + 0) =
                sizeof(imx709_ana_gain_table_22017);
        } else {
            memcpy((void *)(uintptr_t) (*(feature_data + 1)),
            (void *)imx709_ana_gain_table_22017,
            sizeof(imx709_ana_gain_table_22017));
        }
        break;
    case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
        *(feature_data + 1) = imgsensor_info.min_gain;
        *(feature_data + 2) = imgsensor_info.max_gain;
        break;
    case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
        *(feature_data + 0) = imgsensor_info.min_gain_iso;
        *(feature_data + 1) = imgsensor_info.gain_step;
        *(feature_data + 2) = imgsensor_info.gain_type;
        break;
    case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
        *(feature_data + 1) = imgsensor_info.min_shutter;
        break;
    case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
        *(MINT32 *)(signed long)(*(feature_data + 1)) = 1411000;
        LOG_INF("exporsure");
    break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
        switch (*feature_data) {
            case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.cap.pclk;
                break;
            case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.normal_video.pclk;
                break;
            case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.hs_video.pclk;
                break;
            case SENSOR_SCENARIO_ID_SLIM_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.slim_video.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM1:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom1.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom2.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM3:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom3.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM4:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom4.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM5:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom5.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM6:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom6.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM7:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom7.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM8:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom8.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM9:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom9.pclk;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM10:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.custom10.pclk;
                break;
            case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
            default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = imgsensor_info.pre.pclk;
                break;
        }
        break;
    case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
        switch (*feature_data) {
            case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.cap.framelength << 16)
                                 + imgsensor_info.cap.linelength;
                break;
            case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.normal_video.framelength << 16)
                                + imgsensor_info.normal_video.linelength;
                break;
            case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.hs_video.framelength << 16)
                                 + imgsensor_info.hs_video.linelength;
                break;
            case SENSOR_SCENARIO_ID_SLIM_VIDEO:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.slim_video.framelength << 16)
                                 + imgsensor_info.slim_video.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM1:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom1.framelength << 16)
                                 + imgsensor_info.custom1.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM2:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom2.framelength << 16)
                                 + imgsensor_info.custom2.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM3:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom3.framelength << 16)
                                 + imgsensor_info.custom3.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM4:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom4.framelength << 16)
                                 + imgsensor_info.custom4.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM5:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom5.framelength << 16)
                                 + imgsensor_info.custom5.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM6:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom6.framelength << 16)
                                 + imgsensor_info.custom6.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM7:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom7.framelength << 16)
                                 + imgsensor_info.custom7.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM8:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom8.framelength << 16)
                                 + imgsensor_info.custom8.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM9:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom9.framelength << 16)
                                 + imgsensor_info.custom9.linelength;
                break;
            case SENSOR_SCENARIO_ID_CUSTOM10:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.custom10.framelength << 16)
                                 + imgsensor_info.custom10.linelength;
                break;
            case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
            default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                        = (imgsensor_info.pre.framelength << 16)
                                 + imgsensor_info.pre.linelength;
                break;
        }
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *feature_return_para_16++ = ctx->line_length;
        *feature_return_para_16 = ctx->frame_length;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        *feature_return_para_32 = ctx->pclk;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:
         set_shutter(ctx, *feature_data, KAL_TRUE);
        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
         /* night_mode((BOOL) *feature_data); */
        break;
    case SENSOR_FEATURE_SET_GAIN:
        set_gain(ctx, (UINT16) *feature_data, KAL_TRUE);
        break;
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        write_cmos_sensor_8(ctx, sensor_reg_data->RegAddr,
                    sensor_reg_data->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        sensor_reg_data->RegData =
            read_cmos_sensor_8(ctx, sensor_reg_data->RegAddr);
        break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        /*get the lens driver ID from EEPROM
         * or just return LENS_DRIVER_ID_DO_NOT_CARE
         * if EEPROM does not exist in camera module.
         */
        *feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:
        set_video_mode(ctx, *feature_data);
        break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        get_imgsensor_id(ctx, feature_return_para_32);
        break;
#ifdef OPLUS_FEATURE_CAMERA_COMMON
    case SENSOR_FEATURE_GET_MODULE_INFO:
    *feature_return_para_32++ = (imx709_common_data[1] << 24) |
        (imx709_common_data[0] << 16) |
        (imx709_common_data[3] << 8) |
        (imx709_common_data[2] & 0xFF);
    *feature_return_para_32 = (imx709_common_data[5] << 24) |
        (imx709_common_data[4] << 16) |
        (imx709_common_data[7] << 8) |
        (imx709_common_data[6] & 0xFF);
    *feature_para_len = 8;
    LOG_INF("imx709 GET_MODULE_CamInfo:%d %d\n", *feature_para_len,
            *feature_data_32);
    break;

    case SENSOR_FEATURE_GET_MODULE_SN:
    *feature_return_para_32++ = (imx709_common_data[11] << 24) |
        (imx709_common_data[10] << 16) |
        (imx709_common_data[9] << 8) |
        (imx709_common_data[8] & 0xFF);
    *feature_para_len = 4;
    LOG_INF("imx709 GET_MODULE_SN:%d %d\n", *feature_para_len,
            *feature_data_32);
    break;

    //case SENSOR_FEATURE_SET_SENSOR_OTP:
    //    kal_int32 ret = IMGSENSOR_RETURN_SUCCESS;
    //    LOG_INF("SENSOR_FEATURE_SET_SENSOR_OTP length :%d\n", (UINT32)*feature_para_len);
    //    ret = write_Module_data((ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
    //    if (ret == ERROR_NONE)
    //        return ERROR_NONE;
    //    else
    //        return ERROR_MSDK_IS_ACTIVATED;

    case SENSOR_FEATURE_CHECK_MODULE_ID:
    *feature_return_para_32 = (UINT32)imgsensor_info.module_id;
    *feature_para_len = 4;
    break;

    case SENSOR_FEATURE_GET_EEPROM_COMDATA:
    memcpy(feature_return_para_32, imx709_common_data,
            OPLUS_CAMERA_COMMON_DATA_LENGTH);
    *feature_para_len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
    break;
    case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
    break;
    case SENSOR_FEATURE_GET_DISTORTIONPARAMS:
    break;
#endif
    case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
        set_auto_flicker_mode(ctx, (BOOL)*feature_data_16,
                      *(feature_data_16+1));
        break;
    case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
         set_max_framerate_by_scenario(ctx,
                (enum SENSOR_SCENARIO_ID_ENUM)*feature_data,
                *(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
         get_default_framerate_by_scenario(ctx,
                (enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
                (MUINT32 *)(uintptr_t)(*(feature_data+1)));
        break;
	case SENSOR_FEATURE_GET_FINE_INTEG_LINE_BY_SCENARIO:
		get_fine_integ_line_by_scenario(ctx,
				(enum SENSOR_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN:
        set_test_pattern_mode(ctx, (UINT32)*feature_data);
        break;
    case SENSOR_FEATURE_SET_TEST_PATTERN_DATA:
        set_test_pattern_data(ctx, (struct mtk_test_pattern_data *)feature_data);
        break;
    case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
        /* for factory mode auto testing */
        *feature_return_para_32 = imgsensor_info.checksum_value;
        *feature_para_len = 4;
        break;
    case SENSOR_FEATURE_SET_FRAMERATE:
        LOG_INF("current fps :%d\n", (UINT32)*feature_data_32);
        ctx->current_fps = *feature_data_32;
        break;
    case SENSOR_FEATURE_SET_HDR:
        LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_32);
        ctx->ihdr_mode = *feature_data_32;
        break;
    case SENSOR_FEATURE_GET_CROP_INFO:
        LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
            (UINT32)*feature_data);
        wininfo =
            (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

        switch (*feature_data_32) {
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[1],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[2],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[3],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[4],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM1:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[5],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM2:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[6],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM3:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[7],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM4:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[8],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM5:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[9],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM6:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[10],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM7:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[11],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM8:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[12],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM9:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[13],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_CUSTOM10:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[14],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        default:
            memcpy((void *)wininfo,
            (void *)&imgsensor_winsize_info[0],
            sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
            break;
        }
        break;

    case SENSOR_FEATURE_GET_PDAF_INFO:
        LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
            (UINT16) *feature_data);
        PDAFinfo =
		  (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
                sizeof(struct SET_PD_BLOCK_INFO_T));
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
            PDAFinfo->i4BlockNumX = 202;
            PDAFinfo->i4BlockNumY = 152;
        break;
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
            PDAFinfo->i4BlockNumX = 202;
            PDAFinfo->i4BlockNumY = 116;
        break;
        case SENSOR_SCENARIO_ID_CUSTOM10:
            PDAFinfo->i4BlockNumX = 100;
            PDAFinfo->i4BlockNumY = 76;
        break;
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        case SENSOR_SCENARIO_ID_CUSTOM1:
        case SENSOR_SCENARIO_ID_CUSTOM2:
        case SENSOR_SCENARIO_ID_CUSTOM3:
        case SENSOR_SCENARIO_ID_CUSTOM4:
        case SENSOR_SCENARIO_ID_CUSTOM5:
        case SENSOR_SCENARIO_ID_CUSTOM6:
        case SENSOR_SCENARIO_ID_CUSTOM7:
        case SENSOR_SCENARIO_ID_CUSTOM8:
        case SENSOR_SCENARIO_ID_CUSTOM9:
        default:
			break;
        }
        break;
    case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
        LOG_INF(
        "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
            (UINT16) *feature_data);
        /*PDAF capacity enable or not, 2p8 only full size support PDAF*/
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        case SENSOR_SCENARIO_ID_CUSTOM10:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
            break;
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        case SENSOR_SCENARIO_ID_CUSTOM1:
        case SENSOR_SCENARIO_ID_CUSTOM2:
        case SENSOR_SCENARIO_ID_CUSTOM3:
        case SENSOR_SCENARIO_ID_CUSTOM4:
        case SENSOR_SCENARIO_ID_CUSTOM5:
        case SENSOR_SCENARIO_ID_CUSTOM6:
        case SENSOR_SCENARIO_ID_CUSTOM7:
        case SENSOR_SCENARIO_ID_CUSTOM8:
        case SENSOR_SCENARIO_ID_CUSTOM9:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
            break;
        }
        break;
    case SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH:
        pr_info("extend_frame_len %d\n", *feature_data);
        extend_frame_length(ctx, (MUINT32) *feature_data);
        pr_info("extend_frame_len done %d\n", *feature_data);
        break;
    case SENSOR_FEATURE_SEAMLESS_SWITCH:
    {
        LOG_INF("SENSOR_FEATURE_SEAMLESS_SWITCH");
        if ((feature_data + 1) != NULL)
            pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
        else
            LOG_INF("warning! no ae_ctrl input");

        if (feature_data == NULL) {
            LOG_INF("error! input scenario is null!");
            return ERROR_INVALID_SCENARIO_ID;
        }
        LOG_INF("call seamless_switch");
        seamless_switch(ctx, (*feature_data), pAeCtrls);
    }
        break;
    case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
        /*
            HDR_NONE = 0,
            HDR_RAW = 1,
            HDR_CAMSV = 2,
            HDR_RAW_ZHDR = 9,
            HDR_MultiCAMSV = 10,
            HDR_RAW_STAGGER_2EXP = 0xB,
            HDR_RAW_STAGGER_MIN = HDR_RAW_STAGGER_2EXP,
            HDR_RAW_STAGGER_3EXP = 0xC,
            HDR_RAW_STAGGER_MAX = HDR_RAW_STAGGER_3EXP,
         */
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0xb;
            break;
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        default:
            *(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
            break;
        }
        LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n",
            *feature_data, *(feature_data+1));
        break;
    case SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO:
        if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM8) {
            switch (*(feature_data + 1)) {
            case HDR_RAW_STAGGER_2EXP:
                *(feature_data + 2) = SENSOR_SCENARIO_ID_CUSTOM4;//custom4 was the 2 exp mode for preview mode
                break;
            default:
                break;
            }
        } else if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case HDR_NONE:
                *(feature_data + 2) = SENSOR_SCENARIO_ID_CUSTOM8;//normal_video mode for video preview mode
                break;
            default:
                break;
            }
        } else if (*feature_data == SENSOR_SCENARIO_ID_NORMAL_PREVIEW) {
            switch (*(feature_data + 1)) {
            case HDR_RAW_STAGGER_2EXP:
                *(feature_data + 2) = SENSOR_SCENARIO_ID_CUSTOM4;
                break;
            default:
                break;
            }
        }
        LOG_INF("SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO %d %d %d\n",
                (UINT16) *feature_data,
                (UINT16) *(feature_data + 1),
                (UINT16) *(feature_data + 2));
        break;
    case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
        if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM4) {
            switch (*(feature_data + 1)) {
            case VC_STAGGER_NE:
                *(feature_data + 2) = 32757;   //need repare
                break;
            case VC_STAGGER_ME:
                *(feature_data + 2) = 32757;
                break;
            case VC_STAGGER_SE:
                *(feature_data + 2) = 32757;
                break;
            default:
                *(feature_data + 2) = 32757;
                break;
            }
        } else {
            *(feature_data + 2) = 0;
        }
        break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER://for 2EXP
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER, LE=%d, SE=%d\n",
				(UINT32) *feature_data, (UINT32) *(feature_data + 1));
		// implement write shutter for NE/SE
		hdr_write_tri_shutter(ctx, (UINT32)*feature_data,
				0,
				(UINT32)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_DUAL_GAIN://for 2EXP
		LOG_INF("SENSOR_FEATURE_SET_DUAL_GAIN, LE=%d, SE=%d\n",
				(UINT32)*feature_data, (UINT32)*(feature_data + 1));
		// implement write gain for NE/SE
		hdr_write_tri_gain(ctx,
				(UINT32)*feature_data,
				0,
				(UINT32)*(feature_data+1));
		break;
    case SENSOR_FEATURE_SET_PDAF:
        LOG_INF("PDAF mode :%d\n", *feature_data_16);
        ctx->pdaf_mode = *feature_data_16;
        break;
    case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
        LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
            (UINT16)*feature_data,
            (UINT16)*(feature_data+1),
            (UINT16)*(feature_data+2));
        break;
    case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
        set_shutter_frame_length(ctx, (UINT16) (*feature_data),
                    (UINT16) (*(feature_data + 1)));
        break;
    case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
        streaming_control(ctx, KAL_FALSE);
        break;
    case SENSOR_FEATURE_SET_STREAMING_RESUME:
        LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
            *feature_data);
        if (*feature_data != 0)
            set_shutter(ctx, *feature_data, KAL_TRUE);
        streaming_control(ctx, KAL_TRUE);
        break;
    case SENSOR_FEATURE_GET_BINNING_TYPE:
        switch (*(feature_data + 1)) {
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        default:
            *feature_return_para_32 = 1; /*BINNING_AVERAGED*/
            break;
        }
        pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
            *feature_return_para_32);
        *feature_para_len = 4;

        break;
    case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
        *feature_return_para_32 = ctx->current_ae_effective_frame;
        break;
    case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
    {
        switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.cap.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.normal_video.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.hs_video.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_SLIM_VIDEO:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.slim_video.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM1:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom1.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM2:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom2.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM3:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom3.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM4:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom4.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM5:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom5.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom6.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM7:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom7.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM8:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom8.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM9:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom9.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_CUSTOM10:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.custom10.mipi_pixel_rate;
            break;
        case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        default:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = imgsensor_info.pre.mipi_pixel_rate;
            break;
        }
    }
    break;
    case SENSOR_FEATURE_GET_CUST_PIXEL_RATE:
    {
        switch (*feature_data) {
            case SENSOR_SCENARIO_ID_CUSTOM6:
            *(MUINT32 *)(uintptr_t)(*(feature_data + 1))
                = 518700000;
            break;
            default:
            break;
        }
    }
    break;
    case SENSOR_FEATURE_SET_AWB_GAIN:
        /* modify to separate 3hdr and remosaic */
        if (ctx->sensor_mode == IMGSENSOR_MODE_CUSTOM3) {
            /*write AWB gain to sensor*/
            feedback_awbgain(ctx, (UINT32)*(feature_data_32 + 1),
                    (UINT32)*(feature_data_32 + 2));
        }
    case SENSOR_FEATURE_SET_LSC_TBL:
        break;
    case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
        *(feature_data + 1) = 0; /* margin info by scenario */
        *(feature_data + 2) = imgsensor_info.margin;
        break;
    case SENSOR_FEATURE_ESD_RESET_BY_USER:
        switch (*feature_data) {
            case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
            case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
            case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
            case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
            case SENSOR_SCENARIO_ID_SLIM_VIDEO:
            case SENSOR_SCENARIO_ID_CUSTOM1:
            case SENSOR_SCENARIO_ID_CUSTOM2:
            case SENSOR_SCENARIO_ID_CUSTOM3:
            case SENSOR_SCENARIO_ID_CUSTOM4:
            case SENSOR_SCENARIO_ID_CUSTOM5:
            case SENSOR_SCENARIO_ID_CUSTOM6:
            case SENSOR_SCENARIO_ID_CUSTOM7:
            case SENSOR_SCENARIO_ID_CUSTOM8:
            case SENSOR_SCENARIO_ID_CUSTOM9:
            case SENSOR_SCENARIO_ID_CUSTOM10:
            default:
                *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
                break;
        }
        break;
    default:
        break;
    }

    return ERROR_NONE;
} /* feature_control(ctx) */

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 808, //uint:Byte, width*10/8
			.vsize = 1216,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 2448,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x30,
            .hsize = 1016, //uint:Byte, width*10/8
            .vsize = 1216,
            .user_data_desc = VC_PDAF_STATS,
        },
    },
};
//reg C4-3-1
static struct mtk_mbus_frame_desc_entry frame_desc_normal_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1856,
			.user_data_desc = VC_STAGGER_NE,
		},
    },
    {
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x2b,
            .hsize = 808, //uint:Byte, width*10/8
            .vsize = 928,
            .user_data_desc = VC_PDAF_STATS,
        },
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs_slim_vid[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1_cus2[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 1632,
            .vsize = 1224,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 6528,
            .vsize = 4896,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1856,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1856,
			.user_data_desc = VC_STAGGER_ME,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 2448,
            .user_data_desc = VC_STAGGER_NE,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_preisp_cus6[] = {
    {/*Virtual Channel for processed raw*/
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2c,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_RAW_PROCESSED_DATA,
        },
    },
#if 0
    {/*Virtual Channel for pure raw*/
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_RAW_DATA,
        },
    },
#endif
    {/*Virtual Channel for 3a_MetaData*/
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x12,
            .hsize = 1024,
            .vsize = 564,
            .user_data_desc = VC_GENERAL_EMBEDDED,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_preisp_cus7[] = {
    {/*Virtual Channel for processed raw*/
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2c,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_RAW_PROCESSED_DATA,
        },
    },
#if 0
    {/*Virtual Channel for pure raw*/
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x2b,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_RAW_DATA,
        },
    },
#endif
    {/*Virtual Channel for 3a_MetaData*/
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x12,
            .hsize = 1024,
            .vsize = 564,
            .user_data_desc = VC_GENERAL_EMBEDDED,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_preisp_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 1856,
			.user_data_desc = VC_STAGGER_NE,
		},
	}
};

static struct mtk_mbus_frame_desc_entry frame_desc_preisp_cus9[] = {
    {/*Virtual Channel for processed raw*/
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2c,
            .hsize = 3264,
            .vsize = 1856,
            .user_data_desc = VC_RAW_PROCESSED_DATA,
        },
    },
    {/*Virtual Channel for pure raw*/
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x2b,
            .hsize = 3272,
            .vsize = 3712,
            .user_data_desc = VC_RAW_DATA,
        },
    },
    {/*Virtual Channel for 3a_MetaData*/
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x12,
            .hsize = 1024,
            .vsize = 564,
            .user_data_desc = VC_GENERAL_EMBEDDED,
        },
    }
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
    {
        .bus.csi2 = {
            .channel = 0,
            .data_type = 0x2b,
            .hsize = 1632,
            .vsize = 1224,
            .user_data_desc = VC_STAGGER_NE,
        },
    },
    {
        .bus.csi2 = {
            .channel = 1,
            .data_type = 0x2b,
            .hsize = 400, //uint:Byte, width*10/8
            .vsize = 608,
            .user_data_desc = VC_PDAF_STATS,
        },
    }
};

static int get_frame_desc(struct subdrv_ctx *ctx,
        int scenario_id, struct mtk_mbus_frame_desc *fd)
{
    switch (scenario_id) {
    case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_prev);
        memcpy(fd->entry, frame_desc_prev, sizeof(frame_desc_prev));
        break;
    case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_cap);
        memcpy(fd->entry, frame_desc_cap, sizeof(frame_desc_cap));
        break;
    case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_normal_vid);
        memcpy(fd->entry, frame_desc_normal_vid, sizeof(frame_desc_normal_vid));
    break;
    case SENSOR_SCENARIO_ID_SLIM_VIDEO:
    case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_hs_slim_vid);
        memcpy(fd->entry, frame_desc_hs_slim_vid, sizeof(frame_desc_hs_slim_vid));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM1:
    case SENSOR_SCENARIO_ID_CUSTOM2:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_cus1_cus2);
        memcpy(fd->entry, frame_desc_cus1_cus2, sizeof(frame_desc_cus1_cus2));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM3:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_cus3);
        memcpy(fd->entry, (frame_desc_cus3), sizeof((frame_desc_cus3)));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM4:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_cus4);
        memcpy(fd->entry, frame_desc_cus4, sizeof(frame_desc_cus4));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM5:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_cus5);
        memcpy(fd->entry, frame_desc_cus5, sizeof(frame_desc_cus5));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM6:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_preisp_cus6);
        memcpy(fd->entry, frame_desc_preisp_cus6, sizeof(frame_desc_preisp_cus6));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM7:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_preisp_cus7);
        memcpy(fd->entry, frame_desc_preisp_cus7, sizeof(frame_desc_preisp_cus7));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM8:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_preisp_cus8);
        memcpy(fd->entry, frame_desc_preisp_cus8, sizeof(frame_desc_preisp_cus8));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM9:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_preisp_cus9);
        memcpy(fd->entry, frame_desc_preisp_cus9, sizeof(frame_desc_preisp_cus9));
        break;
    case SENSOR_SCENARIO_ID_CUSTOM10:
        fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
        fd->num_entries = ARRAY_SIZE(frame_desc_cus10);
        memcpy(fd->entry, frame_desc_cus10, sizeof(frame_desc_cus10));
        break;
    default:
        return -1;
    }

    return 0;
}
#endif

static const struct subdrv_ctx defctx = {

    .ana_gain_def = BASEGAIN * 4,
    .ana_gain_max = BASEGAIN * 32,
    .ana_gain_min = BASEGAIN * 1,
    .ana_gain_step = 1,
    .exposure_def = 0x3D0,
    .exposure_max = 0xffff - 24,
    .exposure_min = 8,
    .exposure_step = 1,
    //.frame_time_delay_frame = None,
    .is_hflip = 1,
    .is_vflip = 1,
    .margin = 24,
    .max_frame_length = 0xffff,

    .mirror = IMAGE_HV_MIRROR,    /* mirror and flip on information */
    .sensor_mode = IMGSENSOR_MODE_INIT,
    /* IMGSENSOR_MODE enum value,record current sensor mode,such as:
     * INIT, Preview, Capture, Video,High Speed Video, Slim Video
     */
    .shutter = 0x3D0,    /* current shutter */
    .gain = 0x100,        /* current gain */
    .dummy_pixel = 0,    /* current dummypixel */
    .dummy_line = 0,    /* current dummyline */
    .current_fps = 450,
    .autoflicker_en = KAL_FALSE,
    .test_pattern = 0,
    .current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
    .ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
    .i2c_write_id = 0x20, /* record current sensor's i2c write id */
    .current_ae_effective_frame = 2,
    .extend_frame_length_en = KAL_FALSE,
    .fast_mode_on = KAL_FALSE,
    .aonhemd_setting_table = imx709_aonhemd_setting,
	.aonhemd_setting_len = ARRAY_SIZE(imx709_aonhemd_setting),
	.aonhemd_clear_setting_table = imx709_aonhemd_clear_setting,
	.aonhemd_clear_setting_len   = ARRAY_SIZE(imx709_aonhemd_clear_setting),
};

static int init_ctx(struct subdrv_ctx *ctx,
        struct i2c_client *i2c_client, u8 i2c_write_id)
{
    memcpy(ctx, &defctx, sizeof(*ctx));
    ctx->i2c_client = i2c_client;
    ctx->i2c_write_id = i2c_write_id;
    return 0;
}

static int get_csi_param(struct subdrv_ctx *ctx,
    enum SENSOR_SCENARIO_ID_ENUM scenario_id,
    struct mtk_csi_param *csi_param)
{
    switch (scenario_id) {
    case SENSOR_SCENARIO_ID_CUSTOM4:
        csi_param->dphy_trail = 0x82;
        break;
    default:
        csi_param->dphy_trail = 0x34;
        break;
    }
    return 0;
}

static struct subdrv_ops ops = {
    .get_id = get_imgsensor_id,
    .init_ctx = init_ctx,
    .open = open,
    .get_info = get_info,
    .get_resolution = get_resolution,
    .control = control,
    .feature_control = feature_control,
    .close = close,
#ifdef IMGSENSOR_VC_ROUTING
    .get_frame_desc = get_frame_desc,
#endif
    .get_csi_param = get_csi_param,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
    {HW_ID_MCLK, 24, 0},
    {HW_ID_PDN, 0, 0},
    {HW_ID_RST, 0, 1},
    {HW_ID_AVDD, 2900000, 3},
    {HW_ID_AFVDD, 2800000, 3},
    {HW_ID_DVDD, 816000, 4},
    {HW_ID_DOVDD, 1800000, 3},
    {HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
    {HW_ID_PDN, 1, 1},
    {HW_ID_RST, 1, 4}
};

static struct subdrv_pw_seq_entry aon_pw_seq[] = {
    {HW_ID_MCLK, 24, 0},
    {HW_ID_PDN, 0, 0},
    {HW_ID_RST, 0, 4},
    {HW_ID_AVDD, 2900000, 3},
    {HW_ID_DVDD, 816000, 4},
    {HW_ID_DOVDD, 1800000, 3},
    {HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
    {HW_ID_PDN, 0, 1},
    {HW_ID_RST, 1, 4}
};

const struct subdrv_entry imx709_mipi_raw_22017_entry = {
    .name = "imx709_mipi_raw_22017",
    .id = IMX709_SENSOR_ID_22017,
    .pw_seq = pw_seq,
    .pw_seq_cnt = ARRAY_SIZE(pw_seq),
    .aon_pw_seq = aon_pw_seq,
	.aon_pw_seq_cnt = ARRAY_SIZE(aon_pw_seq),
    .ops = &ops,
};

