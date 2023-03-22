// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 imx766mipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"

#include "imx766mipiraw_Sensor.h"
#include "imx766_eeprom.h"
#include "imx766_ana_gain_table.h"
#include "imx766_Sensor_setting.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"
#include "adaptor.h"

#define PD_PIX_2_EN 0
#define SEQUENTIAL_WRITE_EN 1
#define DEBUG_LOG_EN 0

#define PFX "IMX766_camera_sensor"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)
#define LOG_DEBUG(...) do { if ((DEBUG_LOG_EN)) LOG_INF(__VA_ARGS__); } while (0)

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor_16(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define imx766_table_write_cmos_sensor_8(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)
#define imx766_seq_write_cmos_sensor_8(...) subdrv_i2c_wr_seq_p8(__VA_ARGS__)

#define IMX766_EEPROM_READ_ID  0xA1
#define IMX766_EEPROM_WRITE_ID 0xA0

#define _I2C_BUF_SIZE 256
static kal_uint16 _i2c_data[_I2C_BUF_SIZE];
static unsigned int _size_to_write;

static kal_uint8 otp_flag;

static kal_uint32 previous_exp[3];
static kal_uint16 previous_exp_cnt;

static void commit_write_sensor(struct subdrv_ctx *ctx)
{
	if (_size_to_write && !ctx->fast_mode_on) {
		imx766_table_write_cmos_sensor_8(ctx, _i2c_data, _size_to_write);
		memset(_i2c_data, 0x0, sizeof(_i2c_data));
		_size_to_write = 0;
	}
}

static void set_cmos_sensor_8(struct subdrv_ctx *ctx,
			kal_uint16 reg, kal_uint16 val)
{
	if (_size_to_write > _I2C_BUF_SIZE - 2)
		commit_write_sensor(ctx);

  if (!ctx->fast_mode_on) {
      _i2c_data[_size_to_write++] = reg;
      _i2c_data[_size_to_write++] = val;
  }
}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, IMX766_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

static bool read_cmos_eeprom_p8(struct subdrv_ctx *ctx, kal_uint16 addr,
			       BYTE *data, int size)
{

	if (adaptor_i2c_rd_p8(ctx->i2c_client, IMX766_EEPROM_READ_ID >> 1,
			addr, data, size) < 0) {
		return false;
	}
	return true;
}

#define   WRITE_DATA_MAX_LENGTH     (16)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, IMX766_EEPROM_WRITE_ID >> 1,
            addr, para, WRITE_DATA_MAX_LENGTH);

	return ret;
}

static kal_int32 write_1st_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
	kal_int32 ret = ERROR_NONE;
	kal_uint16 reg = 0xff35;
	u8 flag = 0;

	adaptor_i2c_wr_u8(ctx->i2c_client, (0xB0 | IMX766_EEPROM_WRITE_ID) >> 1, reg, 0x0);

	reg = 0x06ca;
	if (enable) {
		adaptor_i2c_wr_u8(ctx->i2c_client, (0xB0 | IMX766_EEPROM_WRITE_ID) >> 1, reg, 0x2);
	}
	else {
		adaptor_i2c_wr_u8(ctx->i2c_client, (0xB0 | IMX766_EEPROM_WRITE_ID) >> 1, reg, 0x0);
	}

	adaptor_i2c_rd_u8(ctx->i2c_client, IMX766_EEPROM_WRITE_ID >> 1, reg, &flag);
	LOG_INF("SET_SENSOR_OTP WRP: 0x%x\n", flag);

	return ret;
}

static kal_int32 write_2nd_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
	kal_int32 ret = ERROR_NONE;
	kal_uint16 reg = 0xa000;
	if (enable) {
		adaptor_i2c_wr_u8(ctx->i2c_client, IMX766_EEPROM_WRITE_ID >> 1, reg, 0x0E);
	}
	else {
		adaptor_i2c_wr_u8(ctx->i2c_client, IMX766_EEPROM_WRITE_ID >> 1, reg, 0x00);
	}

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
    u8 flag = 0;
	int ret = 0;
    adaptor_i2c_rd_u8(ctx->i2c_client, IMX766_EEPROM_READ_ID >> 1, 0x000D, &flag);
    if ( flag == 0x01 ) {
		ret = write_1st_eeprom_protect(ctx, enable);
    } else {
        ret = write_2nd_eeprom_protect(ctx, enable);
    }
	return ret;
}

static kal_int32 write_Module_data(struct subdrv_ctx *ctx,
            ACDK_SENSOR_ENGMODE_STEREO_STRUCT * pStereodata)
{
	kal_int32  ret = ERROR_NONE;
	kal_uint16 data_base, data_length;
	kal_uint32 idx, idy;
	kal_uint8 *pData;
	UINT32 i = 0;
	if(pStereodata != NULL) {
		LOG_INF("SET_SENSOR_OTP: 0x%x %d 0x%x %d\n",
					   pStereodata->uSensorId,
					   pStereodata->uDeviceId,
					   pStereodata->baseAddr,
					   pStereodata->dataLength);

		data_base = pStereodata->baseAddr;
		data_length = pStereodata->dataLength;
		pData = pStereodata->uData;
		if ((pStereodata->uSensorId == IMX766_SENSOR_ID) && (data_length == CALI_DATA_MASTER_LENGTH)
			&& (data_base == IMX766_STEREO_START_ADDR || data_base == IMX766_STEREO_MT_START_ADDR)) {
			LOG_INF("Write: %x %x %x %x\n", pData[0], pData[39], pData[40], pData[1556]);
			idx = data_length/WRITE_DATA_MAX_LENGTH;
			idy = data_length%WRITE_DATA_MAX_LENGTH;
			/* close write protect */
			write_eeprom_protect(ctx, 0);
			msleep(6);
			for (i = 0; i < idx; i++ ) {
				ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*i),
						&pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
				if (ret != ERROR_NONE) {
					LOG_INF("write_eeprom error: i= %d\n", i);
					/* open write protect */
					write_eeprom_protect(ctx, 1);
					msleep(6);
					return -1;
				}
				msleep(6);
			}
			ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*idx),
					&pData[WRITE_DATA_MAX_LENGTH*idx], idy);
			if (ret != ERROR_NONE) {
				LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
				/* open write protect */
				write_eeprom_protect(ctx, 1);
				msleep(6);
				return -1;
			}
			msleep(6);
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			msleep(6);
			LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, data_base));
			LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, data_base+39));
			LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, data_base+40));
			LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, data_base+1556));
			LOG_INF("write_Module_data Write end\n");
		} else if ((pStereodata->uSensorId == IMX766_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
			&& (data_base == IMX766_AESYNC_START_ADDR_21007)) {
				LOG_INF("write main aesync: %x %x %x %x %x %x %x %x\n", pData[0], pData[1],
				        pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
			idx = data_length/WRITE_DATA_MAX_LENGTH;
			idy = data_length%WRITE_DATA_MAX_LENGTH;
			/* close write protect */
			write_eeprom_protect(ctx, 0);
			msleep(6);
			for (i = 0; i < idx; i++ ) {
				ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*i),
						&pData[WRITE_DATA_MAX_LENGTH*i], WRITE_DATA_MAX_LENGTH);
				if (ret != ERROR_NONE) {
					LOG_INF("write_eeprom error: i= %d\n", i);
					/* open write protect */
					write_eeprom_protect(ctx, 1);
					msleep(6);
					return -1;
				}
				msleep(6);
			}
			ret = table_write_eeprom_30Bytes(ctx, (data_base+WRITE_DATA_MAX_LENGTH*idx),
					&pData[WRITE_DATA_MAX_LENGTH*idx], idy);
			if (ret != ERROR_NONE) {
				LOG_INF("write_eeprom error: idx= %d idy= %d\n", idx, idy);
				/* open write protect */
				write_eeprom_protect(ctx, 1);
				msleep(6);
				return -1;
			}
			msleep(6);
			/* open write protect */
			write_eeprom_protect(ctx, 1);
			msleep(6);
			LOG_INF("readback main aesync: %x %x %x %x %x %x %x %x\n",
			        read_cmos_eeprom_8(ctx, IMX766_AESYNC_START_ADDR_21007),
					read_cmos_eeprom_8(ctx, IMX766_AESYNC_START_ADDR_21007+1),
					read_cmos_eeprom_8(ctx, IMX766_AESYNC_START_ADDR_21007+2),
					read_cmos_eeprom_8(ctx, IMX766_AESYNC_START_ADDR_21007+3),
					read_cmos_eeprom_8(ctx, IMX766_AESYNC_START_ADDR_21007+4),
					read_cmos_eeprom_8(ctx, IMX766_AESYNC_START_ADDR_21007+5),
					read_cmos_eeprom_8(ctx, IMX766_AESYNC_START_ADDR_21007+6),
					read_cmos_eeprom_8(ctx, IMX766_AESYNC_START_ADDR_21007+7));
			LOG_INF("AESync write_Module_data Write end\n");
		} else {
			LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
			return -1;
		}
	} else {
		LOG_INF("imx766 write_Module_data pStereodata is null\n");
		return -1;
	}
	return ret;
}

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX766_SENSOR_ID,

	.checksum_value = 0xaf3e324f,

	.pre = { /* Reg_B4_4096x3072_30FPS */
		.pclk = 1920000000,
		.linelength = 15616,
		.framelength = 4098,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1440000000,
		.max_framerate = 300,
	},
	.cap = { /* Reg_B4_4096x3072_30FPS */
		.pclk = 1920000000,
		.linelength = 15616,
		.framelength = 4098,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1440000000,
		.max_framerate = 300,
	},
	.normal_video = { /* Reg_S_4096x2304_30FPS seamless K */
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4794,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 864000000,
		.max_framerate = 300,
	},
	.hs_video = { /* Reg_S_1_4096x2304_60FPS */
		.pclk = 2284800000,
		.linelength = 15616,
		.framelength = 2438,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 880460000,
		.max_framerate = 600,
	},
	.slim_video = { /* Reg_S_4096x2304_30FPS seamless K*/
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4794,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 864000000,
		.max_framerate = 300,
	},
	.custom1 = { /* Reg_K_4096x2304_30FPS */
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4792,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 864000000,
		.max_framerate = 300,
		/* ((y_end - y_start + 1) / 4 + 35) * 2 */
		.readout_length = 2374 * 2,
		.read_margin = 10 * 2,
	},
	.custom2 = { /* Reg_B2_4096x3072_24FPS */
		.pclk = 2390400000,
		.linelength = 31232,
		.framelength = 3188,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 451200000,
		.max_framerate = 240,
	},
	.custom3 = { /* Reg_A 8192x6144_11FPS */
		.pclk = 1603200000,
		.linelength = 23104,
		.framelength = 6308,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8192,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 784460000,
		.max_framerate = 110,
	},
	.custom4 = { /* Reg_L_1296x736_480FPS */
		.pclk = 3513600000,
		.linelength = 5568,
		.framelength = 1312,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 736,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1305600000,
		.max_framerate = 4800,
	},
	.custom5 = { /* Reg_M_2048x1152_240FPS */
		.pclk = 3283200000,
		.linelength = 8816,
		.framelength = 1548,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2048,
		.grabwindow_height = 1152,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1667660000,
		.max_framerate = 2400,
	},
	.custom6 = { /* Reg_D_2280x2160_24FPS */
		.pclk = 1296000000,
		.linelength = 23104,
		.framelength = 2336,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2280,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 255090000,
		.max_framerate = 240,
	},
	.custom7 = { /* Reg_E_1472x1104_24FPS */
		.pclk = 2486400000,
		.linelength = 31232,
		.framelength = 3316,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1472,
		.grabwindow_height = 1104,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 209830000,
		.max_framerate = 240,
	},
	.custom8 = { /* Reg_U_4096x3072_30FPS */
		.pclk = 1507200000,
		.linelength = 11552,
		.framelength = 4348,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 798170000,
		.max_framerate = 300,
	},
	.custom9 = { /* Reg_S3_4096x2304_30FPS */
		.pclk = 2265600000,
		.linelength = 31232,
		.framelength = 2418,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 438860000,
		.max_framerate = 300,
		.readout_length = 1580 * 2,
		.read_margin = 10 * 2,
	},
	.custom10 = { /* Reg_D1_2880x2160_12FPS */
		.pclk = 1584000000,
		.linelength = 34656,
		.framelength = 3808,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2880,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 205710000,
		.max_framerate = 120,
	},
	.custom11 = { /* Reg_E1_1472x1104_12FPS */
		.pclk = 2486400000,
		.linelength = 31232,
		.framelength = 6634,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1472,
		.grabwindow_height = 1104,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 205710000,
		.max_framerate = 120,
	},
	.custom12 = { /* Reg_B3_4096x3072_30FPS */
		.pclk = 3014400000,
		.linelength = 31232,
		.framelength = 3216,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 585600000,
		.max_framerate = 300,
	},
	.custom13 = { /* QBIN_HVBIN_V1H1_4096x3072_60FPS */
		.pclk = 3513600000,
		.linelength = 15616,
		.framelength = 3748,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 2122970000,
		.max_framerate = 600,
	},
	.min_gain = BASEGAIN * 1,
	.max_gain = BASEGAIN * 64,
	.min_gain_iso = 100,
	.margin = 48,
	.min_shutter = 16,
	.gain_step = 1,
	.gain_type = 0,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1: support; 0: not support */
	.ihdr_le_firstline = 0,	/* 1:le first; 0: se first */
	.temperature_support = 1, /* 1, support; 0,not support */
	.sensor_mode_num = 18,	/* support sensor mode num */
	.frame_time_delay_frame = 3,

	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,	/* enter hs video delay frame num */
	.slim_video_delay_frame = 2,/* enter slim video delay frame num */
	.custom1_delay_frame = 2,	/* enter custom1 delay frame num */
	.custom2_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom3_delay_frame = 2,	/* enter custom3 delay frame num */
	.custom4_delay_frame = 2,	/* enter custom4 delay frame num */
	.custom5_delay_frame = 2,	/* enter custom5 delay frame num */
	.custom6_delay_frame = 2,	/* enter custom6 delay frame num */
	.custom7_delay_frame = 2,	/* enter custom7 delay frame num */
	.custom8_delay_frame = 2,	/* enter custom8 delay frame num */
	.custom9_delay_frame = 2,	/* enter custom9 delay frame num */
	.custom10_delay_frame = 2,	/* enter custom10 delay frame num */
	.custom11_delay_frame = 2,	/* enter custom11 delay frame num */
	.custom12_delay_frame = 2,	/* enter custom12 delay frame num */
	.custom13_delay_frame = 2,	/* enter custom13 delay frame num */

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.mclk = 24, /* suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.i2c_addr_table = {0x20, 0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 1000, /* kbps */
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 384}, {0, 384}, {0, 384},
		{0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},  //{0, 1632}
	.iMirrorFlip = 3,
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[18] = {
	/* Preview */
	{8192, 6144,    0,    0, 8192, 6144, 4096, 3072,
	    0,    0, 4096, 3072,    0,    0, 4096, 3072},
	/* Capture */
	{8192, 6144,    0,    0, 8192, 6144, 4096, 3072,
	    0,    0, 4096, 3072,    0,    0, 4096, 3072},
	/* Video */
	{8192, 6144,    0,  768, 8192, 4608, 4096, 2304,
	    0,    0, 4096, 2304,    0,    0, 4096, 2304},
	/* hs_video */
	{8192, 6144,    0,  768, 8192, 4608, 4096, 2304,
	    0,    0, 4096, 2304,    0,    0, 4096, 2304},
	/* slim_video */
	{8192, 6144,    0,  768, 8192, 4608, 4096, 2304,
	    0,    0, 4096, 2304,    0,    0, 4096, 2304},
	/* custom1 */
	{8192, 6144,    0,  768, 8192, 4608, 4096, 2304,
	    0,    0, 4096, 2304,    0,    0, 4096, 2304},
	/* custom2 */
	{8192, 6144,    0,    0, 8192, 6144, 4096, 3072,
	    0,    0, 4096, 3072,    0,    0, 4096, 3072},
	/* custom3 */
	{8192, 6144,    0,    0, 8192, 6144, 8192, 6144,
	    0,    0, 8192, 6144,    0,    0, 8192, 6144},
	/* custom4 */
	{8192, 6144,    0, 1600, 8192, 2944, 2048,  736,
	  376,    0, 1296,  736,    0,    0, 1296,  736},
	/* custom5 */
	{8192, 6144,    0,  768, 8192, 4608, 2048, 1152,
	    0,    0, 2048, 1152,    0,    0, 2048, 1152},
	/* custom6 */
	{8192, 6144,    0, 1984, 8192, 2176, 8192, 2176,
	 2656,    8, 2880, 2160,    0,    0, 2880, 2160},
	/* custom7 */
	{8192, 6144,    0, 1952, 8192, 2240, 4096, 1120,
	 1312,    3, 1472, 1104,    0,    0, 1472, 1104},
	/* custom8 */
	{8192, 6144,    0, 1536, 8192, 3072, 8192, 3072,
	 2048,    0, 4096, 3072,    0,    0, 4096, 3072},
	/* custom9 */
	{8192, 6144,    0,  768, 8192, 4608, 4096, 2304,
	    0,    0, 4096, 2304,    0,    0, 4096, 2304},
	/* custom10 */
	{8192, 6144,    0, 1984, 8192, 2176, 8192, 2176,
	 2656,    8, 2880, 2160,    0,    0, 2880, 2160},
	/* custom11 */
	{8192, 6144,    0, 1952, 8192, 2240, 4096, 1120,
	 1312,    3, 1472, 1104,    0,    0, 1472, 1104},
	/* custom12 */
	{8192, 6144,    0,    0, 8192, 6144, 4096, 3072,
	    0,    0, 4096, 3072,    0,    0, 4096, 3072},
	/* custom13 */
	{8192, 6144,    0,    0, 8192, 6144, 4096, 3072,
	    0,    0, 4096, 3072,    0,    0, 4096, 3072},
};

//the index order of VC_STAGGER_NE/ME/SE in array identify the order of readout in MIPI transfer
static struct SENSOR_VC_INFO2_STRUCT SENSOR_VC_INFO2[18] = {
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //preivew
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
			{VC_PDAF_STATS_NE_PIX_1, 0x03, 0x2b, 0x1000, 0x300},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x300},
#endif
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //capture
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
			{VC_PDAF_STATS_NE_PIX_1, 0x03, 0x2b, 0x1000, 0x300},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x300},
#endif
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //normal_video
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
			{VC_PDAF_STATS_NE_PIX_1, 0x03, 0x2b, 0x1000, 0x240},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x800, 0x240},
#endif
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //hs_video
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0xa00, 0x240},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0xa00, 0x240},
#endif
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //slim_video
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0xa00, 0x240},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0xa00, 0x240},
#endif
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom1
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0xa00, 0x240},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0xa00, 0x240},
#endif
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom2
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0xa00, 0x300},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0xa00, 0x300},
#endif
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom3
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x2000, 0x1800},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0xa00, 0xc00},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0xa00, 0xc00},
#endif
		},
		1
	},
	{
		0x04, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom4
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x510, 0x2e0},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom5
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x800, 0x480},
			{VC_PDAF_STATS_NE_PIX_1, 0x03, 0x2b, 0x800, 0x120},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x06, 0x2b, 0x400, 0x120},
#endif
		},
		1
	},
	{
		0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom6
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0xb40, 0x870},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x384, 0x438},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0x384, 0x438},
#endif
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom7
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x5c0, 0x450},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x398, 0x114},
#if PD_PIX_2_EN
			{VC_PDAF_STATS_NE_PIX_2, 0x00, 0x31, 0x398, 0x114},
#endif
		},
		1
	},
	{
		0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom8
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x500, 0x600},
		},
		1
	},
	{
		0x02, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom9
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x900},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x500, 0x480},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom10
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0xb40, 0x870},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x384, 0x438},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom11
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x5c0, 0x450},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x398, 0x114},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom12
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x800, 0x600},
			{VC_PDAF_STATS_NE_PIX_1, 0x03, 0x2b, 0x800, 0x180},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom13
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0xc00},
			{VC_PDAF_STATS_NE_PIX_1, 0x03, 0x2b, 0x1000, 0x300},
		},
		1
	}
};
//mode    0,  1,  2,   3,  4,  5,  6,  7,   8,   9,  10, 11,  12,  13,  14, 15, 16, 17, 18, 19
static MUINT32 fine_integ_line_table[SENSOR_SCENARIO_ID_MAX] = {
		780,	//mode 0
		780,	//mode 1
		780,	//mode 2
		780,	//mode 3
		780,	//mode 4
		2826,	//mode 5
		413,	//mode 6
		751,	//mode 7
		1709,	//mode 8
		2555,	//mode 9
		751,	//mode 10
		413,	//mode 11
		413,	//mode 12
		413,	//mode 13
		834,	//mode 14
		413,	//mode 15
		826,	//mode 16
		826,	//mode 17
		826,	//mode 18
		826 	//mode 19

	};
static MUINT32 exposure_step_table[SENSOR_SCENARIO_ID_MAX] = {
		4,	//mode 0
		4,	//mode 1
		4,	//mode 2
		4,	//mode 3
		4,	//mode 4
		8,	//mode 5
		4,	//mode 6
		4,	//mode 7
		4,	//mode 8
		4,	//mode 9
		4,	//mode 10
		4,	//mode 11
		4,	//mode 12
		4,	//mode 13
		4,	//mode 14
		4,	//mode 15
		4,	//mode 16
		4,	//mode 17
		4,	//mode 18
		4 	//mode 19
	};
static MUINT32 min_shutter_table[SENSOR_SCENARIO_ID_MAX] = {
		 8,	//mode 0
		 8,	//mode 1
		16,	//mode 2
		16,	//mode 3
		16,	//mode 4
		16,	//mode 5
		16,	//mode 6
		 8,	//mode 7
		16,	//mode 8
		16,	//mode 9
		16,	//mode 10
		16,	//mode 11
		16,	//mode 12
		16,	//mode 13
		16,	//mode 14
		16,	//mode 15
		16,	//mode 16
		16,	//mode 17
		16,	//mode 18
		16 	//mode 19
	};

#define QSC_SIZE 3072
#define QSC_TABLE_SIZE (QSC_SIZE*2)
#define QSC_EEPROM_ADDR 0x1E30
#define QSC_OTP_ADDR 0xC800
#define SENSOR_ID_L 0x05
#define SENSOR_ID_H 0x00
#define LENS_ID_L 0x48
#define LENS_ID_H 0x01
#define SENSOR_ID_L_V2 0x0E

#if SEQUENTIAL_WRITE_EN
static kal_uint8 imx766_QSC_setting[QSC_SIZE];
#else
static kal_uint16 imx766_QSC_setting[QSC_TABLE_SIZE];
#endif

static void get_vc_info_2(struct SENSOR_VC_INFO2_STRUCT *pvcinfo2, kal_uint32 scenario)
{
	switch (scenario) {
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[1],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[2],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[3],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[4],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[5],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[6],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[7],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM4:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[8],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM5:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[9],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM6:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[10],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM7:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[11],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM8:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[12],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM9:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[13],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM10:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[14],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM11:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[15],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM12:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[16],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM13:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[17],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	default:
		memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[0],
			sizeof(struct SENSOR_VC_INFO2_STRUCT));
		break;
	}
}

static int get_frame_desc(struct subdrv_ctx *ctx,
	int scenario_id, struct mtk_mbus_frame_desc *fd);

static kal_uint32 get_exp_cnt_by_scenario(struct subdrv_ctx *ctx, kal_uint32 scenario)
{
	kal_uint32 exp_cnt = 0, i = 0;
	struct mtk_mbus_frame_desc frame_desc;

	get_frame_desc(ctx, scenario, &frame_desc);

	for (i = 0; i < frame_desc.num_entries; ++i) {
		if (frame_desc.entry[i].bus.csi2.user_data_desc > VC_STAGGER_MIN_NUM &&
			frame_desc.entry[i].bus.csi2.user_data_desc < VC_STAGGER_MAX_NUM) {
			exp_cnt++;
		}
	}

	LOG_DEBUG("%s exp_cnt %d\n", __func__, exp_cnt);
	return max(exp_cnt, (kal_uint32)1);
}

static kal_uint32 get_cur_exp_cnt(struct subdrv_ctx *ctx)
{
	kal_uint32 exp_cnt = 1;

	if (0x1 == (read_cmos_sensor_8(ctx, 0x33D0) & 0x1)) { // DOL_EN
		if (0x1 == (read_cmos_sensor_8(ctx, 0x33D1) & 0x3)) { // DOL_MODE
			exp_cnt = 3;
		} else {
			exp_cnt = 2;
		}
	}

	return exp_cnt;
}

static void imx766_get_pdaf_reg_setting(struct subdrv_ctx *ctx,
		MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		regDa[idx + 1] = read_cmos_sensor_8(ctx, regDa[idx]);
	}
}
static void imx766_set_pdaf_reg_setting(struct subdrv_ctx *ctx,
		MUINT32 regNum, kal_uint16 *regDa)
{
	imx766_table_write_cmos_sensor_8(ctx, regDa, regNum*2);
}
#if 0
static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, IMX766_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}
#endif

static void read_sensor_Cali(struct subdrv_ctx *ctx)
{
	kal_uint16 addr_qsc = QSC_EEPROM_ADDR;
#if SEQUENTIAL_WRITE_EN == 0
	kal_uint16 idx = 0;
	kal_uint8 tmp_QSC_setting[QSC_SIZE];
	kal_uint16 sensor_qsc = QSC_OTP_ADDR;
#endif
	kal_uint8 otp_data[9] = {0};
	int i = 0;

	/*read otp data to distinguish module*/
	otp_flag = OTP_QSC_NONE;

	for (i = 0; i < 7; i++)
		otp_data[i] = read_cmos_eeprom_8(ctx, 0x0006 + i);

	if ((otp_data[0] == SENSOR_ID_L || otp_data[0] == SENSOR_ID_L_V2) &&
		(otp_data[1] == SENSOR_ID_H) &&
		(otp_data[2] == LENS_ID_L) &&
		(otp_data[3] == LENS_ID_H)) {
		LOG_DEBUG("OTP type: Custom Only");
		otp_flag = OTP_QSC_CUSTOM;
#if SEQUENTIAL_WRITE_EN
		//imx766_QSC_setting[idx] = read_cmos_eeprom_8(ctx, addr_qsc);
		read_cmos_eeprom_p8(ctx, addr_qsc, imx766_QSC_setting, QSC_SIZE);
#else
		read_cmos_eeprom_p8(ctx, addr_qsc, tmp_QSC_setting, QSC_SIZE);
		for (idx = 0; idx < QSC_SIZE; idx++) {
			sensor_qsc = QSC_OTP_ADDR + idx;
			imx766_QSC_setting[2 * idx] = sensor_qsc;
			imx766_QSC_setting[2 * idx + 1] = tmp_QSC_setting[idx];
		}
#endif
	} else {
		LOG_INF("OTP type: No Data, 0x0008 = %d, 0x0009 = %d",
		read_cmos_eeprom_8(ctx, 0x0008), read_cmos_eeprom_8(ctx, 0x0009));
	}
	ctx->is_read_preload_eeprom = 1;
}

static void write_sensor_QSC(struct subdrv_ctx *ctx)
{
	// calibration tool version 3.0 -> 0x4E
	write_cmos_sensor_8(ctx, 0x86A9, 0x4E);
	// set QSC from EEPROM to sensor
	if ((otp_flag == OTP_QSC_CUSTOM) || (otp_flag == OTP_QSC_INTERNAL)) {
#if SEQUENTIAL_WRITE_EN
		imx766_seq_write_cmos_sensor_8(ctx, QSC_OTP_ADDR,
		imx766_QSC_setting, sizeof(imx766_QSC_setting));
#else
		imx766_table_write_cmos_sensor_8(ctx, imx766_QSC_setting,
		sizeof(imx766_QSC_setting) / sizeof(kal_uint16));
#endif
	}
	write_cmos_sensor_8(ctx, 0x32D2, 0x01);
}

static BYTE imx766_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static void read_module_data(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;
	read_imx766_eeprom_info(ctx, EEPROM_META_MODULE_ID,
				&(imx766_common_data[0]), 2);
	imgsensor_info.module_id = (kal_uint16)(imx766_common_data[1] << 8) |
				   imx766_common_data[0];
	read_imx766_eeprom_info(ctx, EEPROM_META_SENSOR_ID,
				&(imx766_common_data[2]), 2);
	read_imx766_eeprom_info(ctx, EEPROM_META_LENS_ID,
				&(imx766_common_data[4]), 2);
	read_imx766_eeprom_info(ctx, EEPROM_META_VCM_ID,
				&(imx766_common_data[6]), 2);
	read_imx766_eeprom_info(ctx, EEPROM_META_MODULE_SN,
				&(imx766_common_data[8]), 17);
	read_imx766_eeprom_info(ctx, EEPROM_META_AF_CODE,
				&(imx766_common_data[25]), 6);

	for (idx = 0; idx < 32; idx = idx + 4)
		LOG_INF("imx766 cam data: %02x %02x %02x %02x\n",
		       imx766_common_data[idx], imx766_common_data[idx + 1],
		       imx766_common_data[idx + 2], imx766_common_data[idx + 3]);

	imx766_common_data[25] = (kal_uint8)((imx766_common_data[26] << 6) | (imx766_common_data[25] >> 2));
	imx766_common_data[26] = (kal_uint8)(imx766_common_data[26] >> 2);
	imx766_common_data[27] = (kal_uint8)((imx766_common_data[28] << 6) | (imx766_common_data[27] >> 2));
	imx766_common_data[28] = (kal_uint8)(imx766_common_data[28] >> 2);
	imx766_common_data[29] = (kal_uint8)((imx766_common_data[30] << 6) | (imx766_common_data[29] >> 2));
	imx766_common_data[30] = (kal_uint8)(imx766_common_data[30] >> 2);
}

static void write_frame_len(struct subdrv_ctx *ctx, kal_uint32 fll)
{
	// //write_frame_len should be called inside GRP_PARAM_HOLD (0x0104)
	// FRM_LENGTH_LINES must be multiple of 4
	kal_uint32 exp_cnt = get_cur_exp_cnt(ctx);

	ctx->frame_length = round_up(fll / exp_cnt, 4) * exp_cnt;

	if (ctx->extend_frame_length_en == KAL_FALSE) {
		LOG_DEBUG("fll %d exp_cnt %d\n", ctx->frame_length, exp_cnt);
		set_cmos_sensor_8(ctx, 0x0340, ctx->frame_length / exp_cnt >> 8);
		set_cmos_sensor_8(ctx, 0x0341, ctx->frame_length / exp_cnt & 0xFF);
	}
}

static void set_dummy(struct subdrv_ctx *ctx)
{

	DEBUG_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	/* return;*/ /* for test */
	set_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_frame_len(ctx, ctx->frame_length);
	set_cmos_sensor_8(ctx, 0x0104, 0x00);

	commit_write_sensor(ctx);

}	/*	set_dummy  */

static void set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror)
{
	kal_uint8 itemp;

	LOG_DEBUG("image_mirror = %d\n", image_mirror);
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

	LOG_DEBUG("framerate = %d, min framelength should enable %d\n", framerate,
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
}	/*	set_max_framerate  */

static kal_bool set_auto_flicker(struct subdrv_ctx *ctx)
{
	kal_uint16 realtime_fps = 0;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10
				/ ctx->frame_length;
		LOG_DEBUG("autoflicker enable, realtime_fps = %d\n",
			realtime_fps);
		if (realtime_fps >= 593 && realtime_fps <= 615) {
			set_max_framerate(ctx, 592, 0);
			write_frame_len(ctx, ctx->frame_length);
			return KAL_TRUE;
		}
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(ctx, 296, 0);
			write_frame_len(ctx, ctx->frame_length);
			return KAL_TRUE;
		}
		if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(ctx, 146, 0);
			write_frame_len(ctx, ctx->frame_length);
			return KAL_TRUE;
		}
	}

	return KAL_FALSE;
}

#define MAX_CIT_LSHIFT 7
static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter, kal_bool gph)
{
	kal_uint16 l_shift = 1;
	kal_uint32 fineIntegTime = fine_integ_line_table[ctx->current_scenario_id];
	kal_uint32 min_shutter = imgsensor_info.min_shutter;
	int i;

	shutter = FINE_INTEG_CONVERT(shutter, fineIntegTime);
	shutter = round_up(shutter, 4);

	//if (shutter > ctx->min_frame_length - imgsensor_info.margin)
	//	ctx->frame_length = shutter + imgsensor_info.margin;
	//else
	ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	if (IMGSENSOR_MODE_PREVIEW == ctx->sensor_mode || IMGSENSOR_MODE_CAPTURE == ctx->sensor_mode)
		min_shutter = 8;

	if (shutter < min_shutter)
		shutter = min_shutter;

	/* restore current shutter value */
	for (i = 0; i < previous_exp_cnt; i++)
		previous_exp[i] = 0;
	previous_exp[0] = shutter;
	previous_exp_cnt = 1;

	if (gph)
		set_cmos_sensor_8(ctx, 0x0104, 0x01);

	set_auto_flicker(ctx);
	ctx->shutter = shutter;

	/* long expsoure */
	if (shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) {

		for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
			if ((shutter >> l_shift)
				< (imgsensor_info.max_frame_length - imgsensor_info.margin))
				break;
		}
		if (l_shift > MAX_CIT_LSHIFT) {
			LOG_INF(
				"Unable to set such a long exposure %d, set to max\n",
				shutter);

			l_shift = MAX_CIT_LSHIFT;
		}
		shutter = shutter >> l_shift;
		//ctx->frame_length = shutter + imgsensor_info.margin;
		LOG_INF("enter long exposure mode, time is %d", l_shift);
		set_cmos_sensor_8(ctx, 0x3128,
			read_cmos_sensor_16(ctx, 0x3128) | (l_shift & 0x7));
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		set_cmos_sensor_8(ctx, 0x3128, read_cmos_sensor_16(ctx, 0x3128) & 0xf8);
		// write_frame_len(ctx, ctx->frame_length);
		ctx->current_ae_effective_frame = 2;
	}

	/* Update Shutter */
	set_cmos_sensor_8(ctx, 0x0350, 0x01); /* Enable auto extend */
	set_cmos_sensor_8(ctx, 0x0202, (shutter >> 8) & 0xFF);
	set_cmos_sensor_8(ctx, 0x0203, shutter  & 0xFF);
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			set_cmos_sensor_8(ctx, 0x0104, 0x00);

		commit_write_sensor(ctx);
	}

	DEBUG_LOG(ctx, "shutter =%d, framelength =%d\n",
		shutter, ctx->frame_length);
}	/*	write_shutter  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter_w_gph(struct subdrv_ctx *ctx, kal_uint32 shutter, kal_bool gph)
{
	write_shutter(ctx, shutter, gph);
}
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	set_shutter_w_gph(ctx, shutter, KAL_TRUE);
} /* set_shutter */

static void set_frame_length(struct subdrv_ctx *ctx, kal_uint16 frame_length)
{
	if (frame_length > 1)
		ctx->frame_length = frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;
	/* Extend frame length */
	set_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_frame_len(ctx, ctx->frame_length);
	set_cmos_sensor_8(ctx, 0x0104, 0x00);
	commit_write_sensor(ctx);

	LOG_DEBUG("Framelength: set=%d/input=%d/min=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length);
}

static void set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
				kal_uint32 *shutters, kal_uint16 shutter_cnt,
				kal_uint16 frame_length)
{
	int i;
	int readoutDiff = 0;
	kal_uint32 calc_fl = 0;
	kal_uint32 calc_fl2 = 0;
	kal_uint32 calc_fl3 = 0;
	kal_uint16 le, me, se;
	kal_uint32 fineIntegTime = fine_integ_line_table[ctx->current_scenario_id];
	kal_uint32 readoutLength = ctx->readout_length;
	kal_uint32 readMargin = ctx->read_margin;

	/* convert & set current available shutter value */
	for (i = 0; i < shutter_cnt; i++) {
		shutters[i] = FINE_INTEG_CONVERT(shutters[i], fineIntegTime);
		shutters[i] = (kal_uint16)max(imgsensor_info.min_shutter,
					(kal_uint32)shutters[i]);
		shutters[i] = round_up((shutters[i]) / shutter_cnt, 4) * shutter_cnt;
	}

	/* fl constraint 1: previous se + previous me + current le */
	calc_fl = shutters[0];
	for (i = 1; i < previous_exp_cnt; i++)
		calc_fl += previous_exp[i];
	calc_fl += imgsensor_info.margin*shutter_cnt*shutter_cnt;

	/* fl constraint 2: current se + current me + current le */
	calc_fl2 = shutters[0];
	for (i = 1; i < shutter_cnt; i++)
		calc_fl2 += shutters[i];
	calc_fl2 += imgsensor_info.margin*shutter_cnt*shutter_cnt;

	/* fl constraint 3: readout time cannot be overlapped */
	calc_fl3 = (readoutLength + readMargin);
	if (previous_exp_cnt == shutter_cnt) {
		for (i = 1; i < shutter_cnt; i++) {
			readoutDiff = previous_exp[i] - shutters[i];
			calc_fl3 += readoutDiff > 0 ? readoutDiff : 0;
		}
	}

	/* using max fl of above value */
	calc_fl = max(calc_fl, calc_fl2);
	calc_fl = max(calc_fl, calc_fl3);

	/* set fl range */
	ctx->frame_length = max((kal_uint32)calc_fl, ctx->min_frame_length);
	ctx->frame_length = max(ctx->frame_length, (kal_uint32)frame_length);
	ctx->frame_length = min(ctx->frame_length, imgsensor_info.max_frame_length);

	/* restore current shutter value */
	for (i = 0; i < previous_exp_cnt; i++)
		previous_exp[i] = 0;
	for (i = 0; i < shutter_cnt; i++)
		previous_exp[i] = shutters[i];
	previous_exp_cnt = shutter_cnt;

	/* register value conversion */
	switch (shutter_cnt) {
	case 3:
		le = shutters[0]/3;
		me = shutters[1]/3;
		se = shutters[2]/3;
		break;
	case 2:
		le = shutters[0]/2;
		me = 0;
		se = shutters[1]/2;
		break;
	case 1:
		le = shutters[0];
		me = 0;
		se = 0;
		break;
	}

	set_cmos_sensor_8(ctx, 0x0104, 0x01);

	if (!set_auto_flicker(ctx))
		write_frame_len(ctx, ctx->frame_length);
	/* Long exposure */
	set_cmos_sensor_8(ctx, 0x0202, (le >> 8) & 0xFF);
	set_cmos_sensor_8(ctx, 0x0203, le & 0xFF);
	/* Middle exposure */
	if (me) {
		/*MID_COARSE_INTEG_TIME[15:8]*/
		set_cmos_sensor_8(ctx, 0x313A, (me >> 8) & 0xFF);
		/*MID_COARSE_INTEG_TIME[7:0]*/
		set_cmos_sensor_8(ctx, 0x313B, me & 0xFF);
	}
	/* Short exposure */
	if (se) {
		set_cmos_sensor_8(ctx, 0x0224, (se >> 8) & 0xFF);
		set_cmos_sensor_8(ctx, 0x0225, se & 0xFF);
	}
	if (!ctx->ae_ctrl_gph_en) {
		set_cmos_sensor_8(ctx, 0x0104, 0x00);

		commit_write_sensor(ctx);
	}

	DEBUG_LOG(ctx, "L! le:0x%x, me:0x%x, se:0x%x, fl:0x%x\n", le, me, se,
		ctx->frame_length);
}

static void set_shutter_frame_length(struct subdrv_ctx *ctx,
				kal_uint16 shutter, kal_uint16 frame_length,
				kal_bool auto_extend_en)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	ctx->shutter = shutter;

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger
	 * than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure
	 * lines must be updated here.
	 */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length,
	 * should extend frame length first
	 */
	/*Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;
	ctx->frame_length = ctx->frame_length + dummy_line;

	/*  */
	// if (shutter > ctx->frame_length - imgsensor_info.margin)
		// ctx->frame_length = shutter + imgsensor_info.margin;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;

	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
	? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	set_cmos_sensor_8(ctx, 0x0104, 0x01);
	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk
			/ ctx->line_length * 10 / ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
	}
	write_frame_len(ctx, ctx->frame_length);

	/* Update Shutter */
	if (auto_extend_en)
		set_cmos_sensor_8(ctx, 0x0350, 0x01); /* Enable auto extend */
	else
		set_cmos_sensor_8(ctx, 0x0350, 0x00); /* Disable auto extend */
	set_cmos_sensor_8(ctx, 0x0202, (shutter >> 8) & 0xFF);
	set_cmos_sensor_8(ctx, 0x0203, shutter & 0xFF);
	set_cmos_sensor_8(ctx, 0x0104, 0x00);

	commit_write_sensor(ctx);

	LOG_DEBUG(
		"shutter =%d, framelength =%d/%d, dummy_line=%d\n",
		shutter, ctx->frame_length,
		frame_length, dummy_line);
}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint16 reg_gain = 0x0;
	kal_uint32 gain_value = gain;
#ifdef USE_GAIN_TABLE
	int i = 0;
#endif

	if (gain_value < imgsensor_info.min_gain || gain_value > imgsensor_info.max_gain) {
		LOG_INF("Error: gain value out of range");

		if (gain_value < imgsensor_info.min_gain)
			gain_value = imgsensor_info.min_gain;
		else if (gain_value > imgsensor_info.max_gain)
			gain_value = imgsensor_info.max_gain;
	}

#ifdef USE_GAIN_TABLE
	reg_gain = imx766_gain_reg[IMX766_GAIN_TABLE_SIZE - 1];
	for (i = 0; i < IMX766_GAIN_TABLE_SIZE; i++) {
		if (gain_value <= imx766_gain_ratio[i]) {
			reg_gain = imx766_gain_reg[i];
			break;
		}
	}
#else
	reg_gain = 16384 - (16384 * BASEGAIN) / gain_value;
#endif

	return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x400)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 set_gain_w_gph(struct subdrv_ctx *ctx, kal_uint32 gain, kal_bool gph)
{
	kal_uint16 reg_gain;
	kal_uint32 min_gain, max_gain;

	min_gain = BASEGAIN;
	max_gain = imgsensor_info.max_gain;

	//16x for full size mode
	switch (ctx->sensor_mode) {
	/* non-binning */
	case IMGSENSOR_MODE_CUSTOM3:
	case IMGSENSOR_MODE_CUSTOM7:
		max_gain = 16 * BASEGAIN;
		break;
	/* binning */
	default:
		break;
	}

	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		LOG_INF("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}

	reg_gain = gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	DEBUG_LOG(ctx, "gain = %d, reg_gain = 0x%x\n", gain, reg_gain);

	if (gph && !ctx->ae_ctrl_gph_en)
		set_cmos_sensor_8(ctx, 0x0104, 0x01);
	set_cmos_sensor_8(ctx, 0x0204, (reg_gain>>8) & 0xFF);
	set_cmos_sensor_8(ctx, 0x0205, reg_gain & 0xFF);
	if (gph)
		set_cmos_sensor_8(ctx, 0x0104, 0x00);

	commit_write_sensor(ctx);

	return gain;
}

static kal_uint32 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
	return set_gain_w_gph(ctx, gain, KAL_TRUE);
} /* set_gain */

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n",
		enable);
	if (enable) {
		write_cmos_sensor_8(ctx, 0x0100, 0X01);
		write_cmos_sensor_8(ctx, 0x0601, 0X00);
		ctx->test_pattern = 0;
	} else {
		write_cmos_sensor_8(ctx, 0x0100, 0x00);
		if (ctx->fast_mode_on) {
			ctx->fast_mode_on = KAL_FALSE;
			ctx->ref_sof_cnt = 0;
			DEBUG_LOG(ctx, "seamless_switch disabled. ctx->fast_mode_on = %d", ctx->fast_mode_on);
			set_cmos_sensor_8(ctx, 0x3010, 0x00);
			commit_write_sensor(ctx);
		}
	}
	return ERROR_NONE;
}

static void extend_frame_length(struct subdrv_ctx *ctx, kal_uint32 ns)
{
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

	if (ns)
		ctx->frame_length = (kal_uint32)(((unsigned long long)(per_frame_ns + ns)) *
			ctx->frame_length / per_frame_ns);

	/* fl constraint: normal DOL behavior while stagger seamless switch */
	if (previous_exp_cnt > 1) {
		calc_fl = (readoutLength + readMargin);
		for (i = 1; i < previous_exp_cnt; i++)
			calc_fl += (previous_exp[i] + imgsensor_info.margin * previous_exp_cnt);

		ctx->frame_length = max(calc_fl, ctx->frame_length);
	}
	/* NEED TO FIX end */

	set_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_frame_len(ctx, ctx->frame_length);
	set_cmos_sensor_8(ctx, 0x0104, 0x00);

	commit_write_sensor(ctx);

	ctx->extend_frame_length_en = KAL_TRUE;

	ns = (kal_uint32)(((unsigned long long)(ctx->frame_length - old_fl) *
		(unsigned long long)ctx->line_length * 1000000000) / (unsigned long long)ctx->pclk);
	LOG_INF("new frame len = %d, old frame len = %d, per_frame_ns = %d, add more %d ns",
		ctx->frame_length, old_fl, per_frame_ns, ns);
}

static void sensor_init(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_init_setting,
		sizeof(imx766_init_setting)/sizeof(kal_uint16));
	/*enable temperature sensor, TEMP_SEN_CTL:*/
	write_cmos_sensor_8(ctx, 0x0138, 0x01);
	set_mirror_flip(ctx, ctx->mirror);
	LOG_INF("X\n");
}

static void preview_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_preview_setting,
		sizeof(imx766_preview_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void capture_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_capture_setting,
		sizeof(imx766_capture_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void normal_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_normal_video_setting,
		sizeof(imx766_normal_video_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_hs_video_setting,
		sizeof(imx766_hs_video_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_slim_video_setting,
		sizeof(imx766_slim_video_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom1_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom1_setting,
		sizeof(imx766_custom1_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom2_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom2_setting,
		sizeof(imx766_custom2_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom3_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom3_setting,
		sizeof(imx766_custom3_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom4_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom4_setting,
		sizeof(imx766_custom4_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom5_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom5_setting,
		sizeof(imx766_custom5_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom6_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom6_setting,
		sizeof(imx766_custom6_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom7_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom7_setting,
		sizeof(imx766_custom7_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom8_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom8_setting,
		sizeof(imx766_custom8_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom9_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom9_setting,
		sizeof(imx766_custom9_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom10_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom10_setting,
		sizeof(imx766_custom10_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom11_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom11_setting,
		sizeof(imx766_custom11_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom12_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom12_setting,
		sizeof(imx766_custom12_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void custom13_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	imx766_table_write_cmos_sensor_8(ctx, imx766_custom13_setting,
		sizeof(imx766_custom13_setting)/sizeof(kal_uint16));
	set_mirror_flip(ctx, ctx->mirror);
	if (otp_flag == OTP_QSC_NONE) {
		LOG_INF("OTP no QSC Data, close qsc register");
		write_cmos_sensor_8(ctx, 0x32D2, 0x00);
	}
	LOG_INF("X\n");
}

static void hdr_write_tri_shutter_w_gph(struct subdrv_ctx *ctx,
		kal_uint32 le, kal_uint32 me, kal_uint32 se, kal_bool gph)
{
	kal_uint16 exposure_cnt = 0;
	kal_uint32 fineIntegTime = fine_integ_line_table[ctx->current_scenario_id];
	int i;

	le = FINE_INTEG_CONVERT(le, fineIntegTime);
	me = FINE_INTEG_CONVERT(me, fineIntegTime);
	se = FINE_INTEG_CONVERT(se, fineIntegTime);

	if (le)
		exposure_cnt++;
	if (me)
		exposure_cnt++;
	if (se)
		exposure_cnt++;

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

	LOG_DEBUG("X! le:0x%x, me:0x%x, se:0x%x autoflicker_en %d frame_length %d\n",
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

	LOG_DEBUG(
		"lg:0x%x, reg_lg:0x%x, mg:0x%x, reg_mg:0x%x, sg:0x%x, reg_sg:0x%x\n",
		lg, reg_lg, mg, reg_mg, sg, reg_sg);
}

static void hdr_write_tri_gain(struct subdrv_ctx *ctx,
		kal_uint32 lg, kal_uint32 mg, kal_uint32 sg)
{
	hdr_write_tri_gain_w_gph(ctx, lg, mg, sg, KAL_TRUE);
}

#define FMC_GPH_START		do { \
					write_cmos_sensor_8(ctx, 0x0104, 0x01); \
					write_cmos_sensor_8(ctx, 0x3010, 0x02); \
				} while (0)

#define FMC_GPH_END		do { \
					write_cmos_sensor_8(ctx, 0x0104, 0x00); \
				} while (0)

static kal_uint32 seamless_switch(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id, uint32_t *ae_ctrl)
{
	ctx->extend_frame_length_en = KAL_FALSE;

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	{
		unsigned short imx766_seamless_capture[] = {
			0x0342, 0x3D,
			0x0343, 0x00,
			0x0340, 0x0C,
			0x0341, 0x90,
			0x0346, 0x00,
			0x034A, 0x17,
			0x0900, 0x01,
			0x0901, 0x22,
			0x0902, 0x08,
			0x3005, 0x02,
			0x3120, 0x04,
			0x3200, 0x41,
			0x3201, 0x41,
			0x32D6, 0x00,
			0x0408, 0x00,
			0x319C, 0x01,
			0x3800, 0x01,
			0x3801, 0x01,
			0x3802, 0x02,
			0x3847, 0x03,
			0x38C4, 0x01,
			0x38C5, 0x2C,
			0x0202, 0x0C,
			0x0203, 0x60,
			0x3803, 0x00,
			0x3804, 0x17,
			0x3805, 0xC0,
		};

		ctx->sensor_mode = scenario_id;
		ctx->autoflicker_en = KAL_FALSE;
		ctx->pclk = imgsensor_info.cap.pclk;
		ctx->line_length = imgsensor_info.cap.linelength;
		ctx->frame_length = imgsensor_info.cap.framelength;
		ctx->min_frame_length = imgsensor_info.cap.framelength;
		ctx->readout_length = imgsensor_info.cap.readout_length;

		FMC_GPH_START;
		imx766_table_write_cmos_sensor_8(ctx, imx766_seamless_capture,
				sizeof(imx766_seamless_capture) / sizeof(kal_uint16));
		if (ae_ctrl) {
			LOG_INF("call SENSOR_SCENARIO_ID_NORMAL_CAPTURE %d %d",
					ae_ctrl[0], ae_ctrl[5]);
			set_shutter_w_gph(ctx, ae_ctrl[0], KAL_FALSE);
			set_gain_w_gph(ctx, ae_ctrl[5], KAL_FALSE);
		}
		FMC_GPH_END;
	}
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	{
		unsigned short imx766_seamless_normal_video[] = {
			0x0340, 0x12,
			0x0341, 0xBA,
			0x0202, 0x12,
			0x0203, 0x8A,
			0x0224, 0x01,
			0x0225, 0xF4,
			0x30B4, 0x01,
			0x33D0, 0x00,
		};

		ctx->sensor_mode = scenario_id;
		ctx->autoflicker_en = KAL_FALSE;
		ctx->pclk = imgsensor_info.normal_video.pclk;
		ctx->line_length = imgsensor_info.normal_video.linelength;
		ctx->frame_length = imgsensor_info.normal_video.framelength;
		ctx->min_frame_length = imgsensor_info.normal_video.framelength;
		ctx->readout_length = imgsensor_info.normal_video.readout_length;
		ctx->read_margin = imgsensor_info.normal_video.read_margin;

		FMC_GPH_START;
		imx766_table_write_cmos_sensor_8(ctx, imx766_seamless_normal_video,
				sizeof(imx766_seamless_normal_video) / sizeof(kal_uint16));
		if (ae_ctrl) {
			LOG_INF("call SENSOR_SCENARIO_ID_NORMAL_VIDEO %d %d",
					ae_ctrl[0], ae_ctrl[5]);
			set_shutter_w_gph(ctx, ae_ctrl[0], KAL_FALSE);
			set_gain_w_gph(ctx, ae_ctrl[5], KAL_FALSE);
		}
		FMC_GPH_END;
	}
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
	{
		unsigned short imx766_seamless_custom1[] = {
			0x0340, 0x09,
			0x0341, 0x5C,
			0x0202, 0x07,
			0x0203, 0xE0,
			0x0224, 0x00,
			0x0225, 0xFC,
			0x30B4, 0x03,
			0x33D0, 0x01,
		};

		ctx->sensor_mode = scenario_id;
		ctx->autoflicker_en = KAL_FALSE;
		ctx->pclk = imgsensor_info.custom1.pclk;
		ctx->line_length = imgsensor_info.custom1.linelength;
		ctx->frame_length = imgsensor_info.custom1.framelength;
		ctx->min_frame_length = imgsensor_info.custom1.framelength;
		ctx->readout_length = imgsensor_info.custom1.readout_length;
		ctx->read_margin = imgsensor_info.custom1.read_margin;

		FMC_GPH_START;
		imx766_table_write_cmos_sensor_8(ctx, imx766_seamless_custom1,
				sizeof(imx766_seamless_custom1) / sizeof(kal_uint16));
		if (ae_ctrl) {
			LOG_INF("call SENSOR_SCENARIO_ID_CUSTOM1 %d %d %d %d %d %d",
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
	case SENSOR_SCENARIO_ID_CUSTOM8:
	{
		unsigned short imx766_seamless_custom8[] = {
			0x0342, 0x2D,
			0x0343, 0x20,
			0x0340, 0x10,
			0x0341, 0xFC,
			0x0346, 0x06,
			0x034A, 0x11,
			0x0900, 0x00,
			0x0901, 0x11,
			0x0902, 0x0A,
			0x3005, 0x00,
			0x3120, 0x00,
			0x3200, 0x00,
			0x3201, 0x00,
			0x32D6, 0x01,
			0x0408, 0x08,
			0x319C, 0x00,
			0x3800, 0x00,
			0x3801, 0x00,
			0x3802, 0x04,
			0x3847, 0x00,
			0x38C4, 0x02,
			0x38C5, 0x26,
			0x0202, 0x10,
			0x0203, 0xCC,
			0x3803, 0x01,
			0x3804, 0x16,
			0x3805, 0xB0,
		};

		ctx->sensor_mode = scenario_id;
		ctx->autoflicker_en = KAL_FALSE;
		ctx->pclk = imgsensor_info.custom8.pclk;
		ctx->line_length = imgsensor_info.custom8.linelength;
		ctx->frame_length = imgsensor_info.custom8.framelength;
		ctx->min_frame_length = imgsensor_info.custom8.framelength;
		ctx->readout_length = imgsensor_info.custom8.readout_length;
		ctx->read_margin = imgsensor_info.custom8.read_margin;

		FMC_GPH_START;
		imx766_table_write_cmos_sensor_8(ctx, imx766_seamless_custom8,
				sizeof(imx766_seamless_custom8) / sizeof(kal_uint16));
		if (ae_ctrl) {
			LOG_INF("call SENSOR_SCENARIO_ID_CUSTOM8 %d %d",
					ae_ctrl[0], ae_ctrl[5]);
			set_shutter_w_gph(ctx, ae_ctrl[0], KAL_FALSE);
			set_gain_w_gph(ctx, ae_ctrl[5], KAL_FALSE);
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
	ctx->ref_sof_cnt = ctx->sof_cnt;
	LOG_DEBUG("%s success, scenario is switched to %d", __func__, scenario_id);
	return 0;
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	static bool first_read = KAL_TRUE;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			*sensor_id = ((read_cmos_sensor_8(ctx, 0x0016) << 8)
					| read_cmos_sensor_8(ctx, 0x0017));
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, *sensor_id);
				read_sensor_Cali(ctx);
				if (first_read) {
					read_module_data(ctx);
					first_read = KAL_FALSE;
				}
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail. i2c_write_id: 0x%x\n", ctx->i2c_write_id);
			LOG_INF("sensor_id = 0x%x, imgsensor_info.sensor_id = 0x%x\n",
				*sensor_id, imgsensor_info.sensor_id);
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
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int open(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = ((read_cmos_sensor_8(ctx, 0x0016) << 8)
					| read_cmos_sensor_8(ctx, 0x0017));
			if (sensor_id == imgsensor_info.sensor_id) {
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
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init(ctx);

	LOG_INF("write_sensor_QSC Start\n");
	write_sensor_QSC(ctx);
	LOG_INF("write_sensor_QSC End\n");

	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->shutter = 0x3D0;
	ctx->gain = BASEGAIN * 4;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	ctx->ihdr_mode = 0;
	ctx->test_pattern = 0;
	ctx->current_fps = imgsensor_info.pre.max_framerate;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->fast_mode_on = KAL_FALSE;

	return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int close(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	write_cmos_sensor_8(ctx, 0x0100, 0x00);
	return ERROR_NONE;
} /* close */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->readout_length = imgsensor_info.pre.readout_length;
	ctx->read_margin = imgsensor_info.pre.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	preview_setting(ctx);

	return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	ctx->readout_length = imgsensor_info.cap.readout_length;
	ctx->read_margin = imgsensor_info.cap.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	capture_setting(ctx);

	return ERROR_NONE;
}	/* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	ctx->readout_length = imgsensor_info.normal_video.readout_length;
	ctx->read_margin = imgsensor_info.normal_video.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	normal_video_setting(ctx);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->readout_length = imgsensor_info.hs_video.readout_length;
	ctx->read_margin = imgsensor_info.hs_video.read_margin;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	hs_video_setting(ctx);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->readout_length = imgsensor_info.slim_video.readout_length;
	ctx->read_margin = imgsensor_info.slim_video.read_margin;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;
	slim_video_setting(ctx);

	return ERROR_NONE;
}	/* slim_video */

static kal_uint32 custom1(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	ctx->pclk = imgsensor_info.custom1.pclk;
	ctx->line_length = imgsensor_info.custom1.linelength;
	ctx->frame_length = imgsensor_info.custom1.framelength;
	ctx->min_frame_length = imgsensor_info.custom1.framelength;
	ctx->readout_length = imgsensor_info.custom1.readout_length;
	ctx->read_margin = imgsensor_info.custom1.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom1_setting(ctx);

	return ERROR_NONE;
}	/* custom1 */

static kal_uint32 custom2(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	ctx->pclk = imgsensor_info.custom2.pclk;
	ctx->line_length = imgsensor_info.custom2.linelength;
	ctx->frame_length = imgsensor_info.custom2.framelength;
	ctx->min_frame_length = imgsensor_info.custom2.framelength;
	ctx->readout_length = imgsensor_info.custom2.readout_length;
	ctx->read_margin = imgsensor_info.custom2.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom2_setting(ctx);

	return ERROR_NONE;
}	/* custom2 */

static kal_uint32 custom3(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	ctx->pclk = imgsensor_info.custom3.pclk;
	ctx->line_length = imgsensor_info.custom3.linelength;
	ctx->frame_length = imgsensor_info.custom3.framelength;
	ctx->min_frame_length = imgsensor_info.custom3.framelength;
	ctx->readout_length = imgsensor_info.custom3.readout_length;
	ctx->read_margin = imgsensor_info.custom3.read_margin;
	ctx->autoflicker_en = KAL_FALSE;

	custom3_setting(ctx);

	return ERROR_NONE;
}	/* custom3 */

static kal_uint32 custom4(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	ctx->pclk = imgsensor_info.custom4.pclk;
	ctx->line_length = imgsensor_info.custom4.linelength;
	ctx->frame_length = imgsensor_info.custom4.framelength;
	ctx->min_frame_length = imgsensor_info.custom4.framelength;
	ctx->readout_length = imgsensor_info.custom4.readout_length;
	ctx->read_margin = imgsensor_info.custom4.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom4_setting(ctx);

	return ERROR_NONE;
}	/* custom4 */

static kal_uint32 custom5(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	ctx->pclk = imgsensor_info.custom5.pclk;
	ctx->line_length = imgsensor_info.custom5.linelength;
	ctx->frame_length = imgsensor_info.custom5.framelength;
	ctx->min_frame_length = imgsensor_info.custom5.framelength;
	ctx->readout_length = imgsensor_info.custom5.readout_length;
	ctx->read_margin = imgsensor_info.custom5.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom5_setting(ctx);

	return ERROR_NONE;
}	/* custom5 */

static kal_uint32 custom6(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM6;
	ctx->pclk = imgsensor_info.custom6.pclk;
	ctx->line_length = imgsensor_info.custom6.linelength;
	ctx->frame_length = imgsensor_info.custom6.framelength;
	ctx->min_frame_length = imgsensor_info.custom6.framelength;
	ctx->readout_length = imgsensor_info.custom6.readout_length;
	ctx->read_margin = imgsensor_info.custom6.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom6_setting(ctx);

	return ERROR_NONE;
}	/* custom6 */

static kal_uint32 custom7(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM7;
	ctx->pclk = imgsensor_info.custom7.pclk;
	ctx->line_length = imgsensor_info.custom7.linelength;
	ctx->frame_length = imgsensor_info.custom7.framelength;
	ctx->min_frame_length = imgsensor_info.custom7.framelength;
	ctx->readout_length = imgsensor_info.custom7.readout_length;
	ctx->read_margin = imgsensor_info.custom7.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom7_setting(ctx);

	return ERROR_NONE;
}	/* custom7 */

static kal_uint32 custom8(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM8;
	ctx->pclk = imgsensor_info.custom8.pclk;
	ctx->line_length = imgsensor_info.custom8.linelength;
	ctx->frame_length = imgsensor_info.custom8.framelength;
	ctx->min_frame_length = imgsensor_info.custom8.framelength;
	ctx->readout_length = imgsensor_info.custom8.readout_length;
	ctx->read_margin = imgsensor_info.custom8.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom8_setting(ctx);

	return ERROR_NONE;
}	/* custom8 */

static kal_uint32 custom9(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM9;
	ctx->pclk = imgsensor_info.custom9.pclk;
	ctx->line_length = imgsensor_info.custom9.linelength;
	ctx->frame_length = imgsensor_info.custom9.framelength;
	ctx->min_frame_length = imgsensor_info.custom9.framelength;
	ctx->readout_length = imgsensor_info.custom9.readout_length;
	ctx->read_margin = imgsensor_info.custom9.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom9_setting(ctx);

	return ERROR_NONE;
}	/* custom9 */

static kal_uint32 custom10(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM10;
	ctx->pclk = imgsensor_info.custom10.pclk;
	ctx->line_length = imgsensor_info.custom10.linelength;
	ctx->frame_length = imgsensor_info.custom10.framelength;
	ctx->min_frame_length = imgsensor_info.custom10.framelength;
	ctx->readout_length = imgsensor_info.custom10.readout_length;
	ctx->read_margin = imgsensor_info.custom10.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom10_setting(ctx);

	return ERROR_NONE;
}	/* custom10 */

static kal_uint32 custom11(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM11;
	ctx->pclk = imgsensor_info.custom11.pclk;
	ctx->line_length = imgsensor_info.custom11.linelength;
	ctx->frame_length = imgsensor_info.custom11.framelength;
	ctx->min_frame_length = imgsensor_info.custom11.framelength;
	ctx->readout_length = imgsensor_info.custom11.readout_length;
	ctx->read_margin = imgsensor_info.custom11.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom11_setting(ctx);

	return ERROR_NONE;
}	/* custom11 */

static kal_uint32 custom12(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM12;
	ctx->pclk = imgsensor_info.custom12.pclk;
	ctx->line_length = imgsensor_info.custom12.linelength;
	ctx->frame_length = imgsensor_info.custom12.framelength;
	ctx->min_frame_length = imgsensor_info.custom12.framelength;
	ctx->readout_length = imgsensor_info.custom12.readout_length;
	ctx->read_margin = imgsensor_info.custom12.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom12_setting(ctx);

	return ERROR_NONE;
}	/* custom12 */

static kal_uint32 custom13(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM13;
	ctx->pclk = imgsensor_info.custom13.pclk;
	ctx->line_length = imgsensor_info.custom13.linelength;
	ctx->frame_length = imgsensor_info.custom13.framelength;
	ctx->min_frame_length = imgsensor_info.custom13.framelength;
	ctx->readout_length = imgsensor_info.custom13.readout_length;
	ctx->read_margin = imgsensor_info.custom13.read_margin;
	ctx->autoflicker_en = KAL_FALSE;
	custom13_setting(ctx);

	return ERROR_NONE;
}	/* custom13 */

static int get_resolution(struct subdrv_ctx *ctx,
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	int i = 0;

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

static int get_info(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_INFO_STRUCT *sensor_info,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("scenario_id = %d\n", scenario_id);

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
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM11] =
		imgsensor_info.custom11_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM12] =
		imgsensor_info.custom12_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM13] =
		imgsensor_info.custom13_delay_frame;

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
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV_QPD;

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
}	/*	get_info  */

static int control(struct subdrv_ctx *ctx, enum SENSOR_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	ctx->current_scenario_id = scenario_id;

	previous_exp_cnt = 0;

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
	case SENSOR_SCENARIO_ID_CUSTOM11:
		custom11(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM12:
		custom12(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM13:
		custom13(ctx, image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control(ctx) */

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	LOG_DEBUG("framerate = %d\n ", framerate);
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
	set_dummy(ctx);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx, kal_bool enable, UINT16 framerate)
{
	LOG_DEBUG("enable = %d, framerate = %d\n", enable, framerate);
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

	DEBUG_LOG(ctx, "scenario_id = %d, framerate = %d\n", scenario_id, framerate);

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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		if (ctx->current_fps != imgsensor_info.cap.max_framerate)
			LOG_DEBUG(
				"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
				, framerate
				, imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10
					/ imgsensor_info.cap.linelength;

		if (frame_length > imgsensor_info.max_frame_length) {
			LOG_DEBUG(
				"Warning: frame_length %d > max_frame_length %d!\n"
				, frame_length
				, imgsensor_info.max_frame_length);
			break;
		}

		ctx->dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			? (frame_length - imgsensor_info.cap.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.cap.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM11:
		frame_length = imgsensor_info.custom11.pclk / framerate * 10
				/ imgsensor_info.custom11.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom11.framelength)
			? (frame_length - imgsensor_info.custom11.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom11.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM12:
		frame_length = imgsensor_info.custom12.pclk / framerate * 10
				/ imgsensor_info.custom12.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom12.framelength)
			? (frame_length - imgsensor_info.custom12.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom12.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM13:
		frame_length = imgsensor_info.custom13.pclk / framerate * 10
				/ imgsensor_info.custom13.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom13.framelength)
			? (frame_length - imgsensor_info.custom13.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom13.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
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
		//if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
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
	case SENSOR_SCENARIO_ID_CUSTOM11:
		*framerate = imgsensor_info.custom11.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM12:
		*framerate = imgsensor_info.custom12.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM13:
		*framerate = imgsensor_info.custom13.max_framerate;
		break;
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_fine_integ_line_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id, MUINT32 *fine_integ_line)
{
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
	case SENSOR_SCENARIO_ID_CUSTOM11:
	case SENSOR_SCENARIO_ID_CUSTOM12:
	case SENSOR_SCENARIO_ID_CUSTOM13:
	case SENSOR_SCENARIO_ID_CUSTOM14:
	case SENSOR_SCENARIO_ID_CUSTOM15:
		*fine_integ_line = fine_integ_line_table[scenario_id];
		break;
	default:
		break;
	}
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
            write_cmos_sensor_8(ctx, 0x3015, 0x00);
            break;
        default:
            write_cmos_sensor_8(ctx, 0x0601, mode);
            break;
        }
    } else if (ctx->test_pattern) {
        write_cmos_sensor_8(ctx, 0x0601, 0x00); /*No pattern*/
        write_cmos_sensor_8(ctx, 0x020E, 0x01);
        write_cmos_sensor_8(ctx, 0x0218, 0x01);
        write_cmos_sensor_8(ctx, 0x3015, 0x40);
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
		set_cmos_sensor_8(ctx, 0x0602, (data->Channel_R >> 30) & 0x3);
		set_cmos_sensor_8(ctx, 0x0603, (data->Channel_R >> 22) & 0xff);
		set_cmos_sensor_8(ctx, 0x0604, (data->Channel_Gr >> 30) & 0x3);
		set_cmos_sensor_8(ctx, 0x0605, (data->Channel_Gr >> 22) & 0xff);
		set_cmos_sensor_8(ctx, 0x0606, (data->Channel_B >> 30) & 0x3);
		set_cmos_sensor_8(ctx, 0x0607, (data->Channel_B >> 22) & 0xff);
		set_cmos_sensor_8(ctx, 0x0608, (data->Channel_Gb >> 30) & 0x3);
		set_cmos_sensor_8(ctx, 0x0609, (data->Channel_Gb >> 22) & 0xff);
		commit_write_sensor(ctx);
	}

	return ERROR_NONE;
}

static kal_int32 get_sensor_temperature(struct subdrv_ctx *ctx)
{
	UINT8 temperature = 0;
	INT32 temperature_convert = 0;

	temperature = read_cmos_sensor_8(ctx, 0x013a);

	if (temperature >= 0x0 && temperature <= 0x60)
		temperature_convert = temperature;
	else if (temperature >= 0x61 && temperature <= 0x7F)
		temperature_convert = 97;
	else if (temperature >= 0x80 && temperature <= 0xE2)
		temperature_convert = -30;
	else
		temperature_convert = (INT8)temperature | 0xFFFFFF0;

	return temperature_convert;
}

static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
				UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	uint32_t *pAeCtrls;
	uint32_t *pScenarios;
	uint32_t ratio = 1;
	int ret = ERROR_NONE;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	struct SENSOR_VC_INFO2_STRUCT *pvcinfo2;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

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
		case SENSOR_SCENARIO_ID_CUSTOM11:
		case SENSOR_SCENARIO_ID_CUSTOM12:
		case SENSOR_SCENARIO_ID_CUSTOM13:
		case SENSOR_SCENARIO_ID_CUSTOM14:
		case SENSOR_SCENARIO_ID_CUSTOM15:
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(imx766_ana_gain_table);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)imx766_ana_gain_table,
			sizeof(imx766_ana_gain_table));
		}
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;

		switch (*feature_data) {
		/* non-binning */
		case SENSOR_SCENARIO_ID_CUSTOM3:
		case SENSOR_SCENARIO_ID_CUSTOM7:
			*(feature_data + 2) = BASEGAIN * 16;
			break;
		/* binning */
		default:
			*(feature_data + 2) = imgsensor_info.max_gain;
			break;
		}

		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
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
		case SENSOR_SCENARIO_ID_CUSTOM11:
		case SENSOR_SCENARIO_ID_CUSTOM12:
		case SENSOR_SCENARIO_ID_CUSTOM13:
		case SENSOR_SCENARIO_ID_CUSTOM14:
		case SENSOR_SCENARIO_ID_CUSTOM15:
			*(feature_data + 1) = min_shutter_table[*feature_data];
			*(feature_data + 2) = exposure_step_table[*feature_data];
			break;
		default:
			*(feature_data + 1) = imgsensor_info.min_shutter;
			*(feature_data + 2) = 4;
			break;
		}

		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 4211000;
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
		case SENSOR_SCENARIO_ID_CUSTOM11:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom11.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM12:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom12.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM13:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom13.pclk;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		if (*(feature_data + 2) & SENSOR_GET_LINELENGTH_FOR_READOUT)
			ratio = get_exp_cnt_by_scenario(ctx, (*feature_data));

		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ (ratio * imgsensor_info.cap.linelength);
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ (ratio * imgsensor_info.normal_video.linelength);
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ (ratio * imgsensor_info.hs_video.linelength);
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ (ratio * imgsensor_info.slim_video.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ (ratio * imgsensor_info.custom1.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ (ratio * imgsensor_info.custom2.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ (ratio * imgsensor_info.custom3.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom4.framelength << 16)
				+ (ratio * imgsensor_info.custom4.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom5.framelength << 16)
				+ (ratio * imgsensor_info.custom5.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom6.framelength << 16)
				+ (ratio * imgsensor_info.custom6.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom7.framelength << 16)
				+ (ratio * imgsensor_info.custom7.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom8.framelength << 16)
				+ (ratio * imgsensor_info.custom8.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom9.framelength << 16)
				+ (ratio * imgsensor_info.custom9.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom10.framelength << 16)
				+ (ratio * imgsensor_info.custom10.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM11:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom11.framelength << 16)
				+ (ratio * imgsensor_info.custom11.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM12:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom12.framelength << 16)
				+ (ratio * imgsensor_info.custom12.linelength);
			break;
		case SENSOR_SCENARIO_ID_CUSTOM13:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom13.framelength << 16)
				+ (ratio * imgsensor_info.custom13.linelength);
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ (ratio * imgsensor_info.pre.linelength);
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
		 set_shutter(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT32) * (feature_data));
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
		*feature_return_para_32++ = (imx766_common_data[1] << 24) |
					    (imx766_common_data[0] << 16) |
					    (imx766_common_data[3] << 8) |
					    (imx766_common_data[2] & 0xFF);
		*feature_return_para_32 = (imx766_common_data[5] << 24) |
					  (imx766_common_data[4] << 16) |
					  (imx766_common_data[7] << 8) |
					  (imx766_common_data[6] & 0xFF);
		*feature_para_len = 8;
		LOG_INF("imx766 GET_MODULE_CamInfo:%d %d\n", *feature_para_len,
			*feature_data_32);
		break;

	case SENSOR_FEATURE_GET_MODULE_SN:
		*feature_return_para_32++ = (imx766_common_data[11] << 24) |
					    (imx766_common_data[10] << 16) |
					    (imx766_common_data[9] << 8) |
					    (imx766_common_data[8] & 0xFF);
		*feature_para_len = 4;
		LOG_INF("imx766 GET_MODULE_SN:%d %d\n", *feature_para_len,
			*feature_data_32);
		break;

    case SENSOR_FEATURE_SET_SENSOR_OTP:
		ret = write_Module_data(ctx, (ACDK_SENSOR_ENGMODE_STEREO_STRUCT *)(feature_para));
		if (ret == ERROR_NONE)
		    return ERROR_NONE;
		else
		    return ERROR_MSDK_IS_ACTIVATED;
		break;
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = (UINT32)imgsensor_info.module_id;
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_GET_EEPROM_COMDATA:
		memcpy(feature_return_para_32, imx766_common_data,
		       OPLUS_CAMERA_COMMON_DATA_LENGTH);
		*feature_para_len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
		break;
	case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
		if(*feature_para_len > CALI_DATA_MASTER_LENGTH)
			*feature_para_len = CALI_DATA_MASTER_LENGTH;
		switch (*feature_data_16) {
		case EEPROM_STEREODATA_MT_MAIN:
			read_imx766_eeprom_info(ctx, EEPROM_META_STEREO_MT_MAIN_DATA,
					(BYTE *)feature_return_para_32, *feature_para_len);
			break;
		case EEPROM_STEREODATA_MW_MAIN:
		default:
			read_imx766_eeprom_info(ctx, EEPROM_META_STEREO_MW_MAIN_DATA,
					(BYTE *)feature_return_para_32, *feature_para_len);
			break;
		}
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
		LOG_DEBUG("SENSOR_FEATURE_GET_PDAF_DATA\n");
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
		LOG_DEBUG("current fps :%d\n", (UINT32)*feature_data_32);
		ctx->current_fps = *feature_data_32;
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_DEBUG("ihdr enable :%d\n", (BOOL)*feature_data_32);
		ctx->ihdr_mode = *feature_data_32;
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_DEBUG("SENSOR_FEATURE_GET_CROP_INFO, scenarioId:%d\n",
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
		case SENSOR_SCENARIO_ID_CUSTOM11:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[15],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM12:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[16],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM13:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[17],
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
		LOG_DEBUG("SENSOR_FEATURE_GET_PDAF_INFO, scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo =
		  (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
			sizeof(struct SET_PD_BLOCK_INFO_T));
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_DEBUG(
		"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY, scenarioId:%d\n",
			(UINT16) *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			/* video & capture use same setting */
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM11:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM12:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM13:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH:
		LOG_DEBUG("extend_frame_len %d\n", *feature_data);
		extend_frame_length(ctx, (MUINT32) *feature_data);
		LOG_DEBUG("extend_frame_len done %d\n", *feature_data);
		break;
	case SENSOR_FEATURE_SEAMLESS_SWITCH:
	{
		LOG_DEBUG("SENSOR_FEATURE_SEAMLESS_SWITCH");
		if ((feature_data + 1) != NULL)
			pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		else
			LOG_INF("warning! no ae_ctrl input");

		if (feature_data == NULL) {
			LOG_INF("error! input scenario is null!");
			return ERROR_INVALID_SCENARIO_ID;
		}
		LOG_DEBUG("call seamless_switch");
		seamless_switch(ctx, (*feature_data), pAeCtrls);
	}
		break;
	case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
		if ((feature_data + 1) != NULL)
			pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		else {
			LOG_INF("input pScenarios vector is NULL!\n");
			return ERROR_INVALID_SCENARIO_ID;
		}
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*pScenarios = SENSOR_SCENARIO_ID_CUSTOM8;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*pScenarios = SENSOR_SCENARIO_ID_CUSTOM1;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*pScenarios = SENSOR_SCENARIO_ID_NORMAL_VIDEO;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM8:
			*pScenarios = SENSOR_SCENARIO_ID_NORMAL_CAPTURE;
			break;
		default:
			*pScenarios = 0xff;
			break;
		}
		LOG_DEBUG("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %d %d\n",
				*feature_data, *pScenarios);
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0xB;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		}
		LOG_DEBUG(
			"SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY, scenarioId:%llu, HDR:%llu\n",
			*feature_data, *(MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
		/*END OF HDR CMD */
	case SENSOR_FEATURE_GET_VC_INFO2:
		LOG_DEBUG("SENSOR_FEATURE_GET_VC_INFO2 %d\n",
							(UINT16) (*feature_data));
		pvcinfo2 = (struct SENSOR_VC_INFO2_STRUCT *) (uintptr_t) (*(feature_data + 1));
		get_vc_info_2(pvcinfo2, *feature_data_32);
		break;
	case SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO:
		if (*feature_data == SENSOR_SCENARIO_ID_NORMAL_VIDEO) {
			switch (*(feature_data + 1)) {
			case HDR_RAW_STAGGER_2EXP:
				*(feature_data + 2) = SENSOR_SCENARIO_ID_CUSTOM1;
				break;
			default:
				break;
			}
		} else if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM1) {
			switch (*(feature_data + 1)) {
			case HDR_NONE:
				*(feature_data + 2) = SENSOR_SCENARIO_ID_NORMAL_VIDEO;
				break;
			default:
				break;
			}
		} else if (*feature_data == SENSOR_SCENARIO_ID_NORMAL_PREVIEW) {
			switch (*(feature_data + 1)) {
			case HDR_RAW_STAGGER_2EXP:
				*(feature_data + 2) = SENSOR_SCENARIO_ID_CUSTOM1;
				break;
			default:
				break;
			}
		}
		LOG_DEBUG("SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO %d %d %d\n",
				(UINT16) *feature_data,
				(UINT16) *(feature_data + 1),
				(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		*(feature_data + 1) = 1; //always 1
		/* margin info by scenario */
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_CUSTOM1:
			// 2dol
			*(feature_data + 2) = (imgsensor_info.margin * 2);
			break;
		default:
			*(feature_data + 2) = imgsensor_info.margin;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
		if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM1) {
			// see IMX766 SRM, table 5-22 constraints of COARSE_INTEG_TIME
			switch (*(feature_data + 1)) {
			case VC_STAGGER_NE:
			case VC_STAGGER_ME:
			case VC_STAGGER_SE:
			default:
				*(feature_data + 2) = 65532 - imgsensor_info.margin;
				break;
			}
		} else {
			*(feature_data + 2) = 0;
		}
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER://for 2EXP
		LOG_DEBUG("SENSOR_FEATURE_SET_HDR_SHUTTER, LE=%d, SE=%d\n",
				(UINT32) *feature_data, (UINT32) *(feature_data + 1));
		// implement write shutter for NE/SE
		hdr_write_tri_shutter(ctx, (UINT32)*feature_data,
					0,
					(UINT32)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_DUAL_GAIN://for 2EXP
		LOG_DEBUG("SENSOR_FEATURE_SET_DUAL_GAIN, LE=%d, SE=%d\n",
				(UINT32)*feature_data, (UINT32)*(feature_data + 1));
		// implement write gain for NE/SE
		hdr_write_tri_gain(ctx,
				(UINT32)*feature_data,
				0,
				(UINT32)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER://for 3EXP
		LOG_DEBUG("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER, LE=%d, ME=%d, SE=%d\n",
				(UINT32) *feature_data,
				(UINT32) *(feature_data + 1),
				(UINT32) *(feature_data + 2));
		hdr_write_tri_shutter(ctx,
				(UINT32) *feature_data,
				(UINT32) *(feature_data + 1),
				(UINT32) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
		LOG_DEBUG("SENSOR_FEATURE_SET_HDR_TRI_GAIN, LG=%d, SG=%d, MG=%d\n",
				(UINT32) *feature_data,
				(UINT32) *(feature_data + 1),
				(UINT32) *(feature_data + 2));
		hdr_write_tri_gain(ctx,
				(UINT32) *feature_data,
				(UINT32) *(feature_data + 1),
				(UINT32) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_32 = get_sensor_temperature(ctx);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		LOG_DEBUG("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx766_get_pdaf_reg_setting(ctx, (*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		imx766_set_pdaf_reg_setting(ctx, (*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_DEBUG("PDAF mode :%d\n", *feature_data_16);
		ctx->pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(ctx,
			(UINT16) (*feature_data),
			(UINT16) (*(feature_data + 1)),
			(BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(ctx, KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(ctx, *feature_data);
		streaming_control(ctx, KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		case SENSOR_SCENARIO_ID_CUSTOM5:
		case SENSOR_SCENARIO_ID_CUSTOM7:
		case SENSOR_SCENARIO_ID_CUSTOM9:
		case SENSOR_SCENARIO_ID_CUSTOM11:
		case SENSOR_SCENARIO_ID_CUSTOM12:
		case SENSOR_SCENARIO_ID_CUSTOM13:
			*feature_return_para_32 = 1465;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
		case SENSOR_SCENARIO_ID_CUSTOM6:
		case SENSOR_SCENARIO_ID_CUSTOM8:
		case SENSOR_SCENARIO_ID_CUSTOM10:
			*feature_return_para_32 = 1000;
			break;
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		LOG_DEBUG("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
		memcpy(feature_return_para_32,
		&ctx->ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
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
		case SENSOR_SCENARIO_ID_CUSTOM11:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom11.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM12:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom12.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM13:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom13.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
	break;
	case SENSOR_FEATURE_PRELOAD_EEPROM_DATA:
		/*get eeprom preloader data*/
		*feature_return_para_32 = ctx->is_read_preload_eeprom;
		*feature_para_len = 4;
		if (ctx->is_read_preload_eeprom != 1)
			read_sensor_Cali(ctx);
		break;
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME:
		set_multi_shutter_frame_length(ctx, (UINT32 *)(*feature_data),
					(UINT16) (*(feature_data + 1)),
					(UINT16) (*(feature_data + 2)));
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
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_hs[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_slim[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
	{
		.bus.csi2 = {
			.channel = 4,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.user_data_desc = VC_PDAF_STATS_ME_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2000,
			.vsize = 0x1800,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0510,
			.vsize = 0x02e0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0480,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0b40,
			.vsize = 0x0870,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x05c0,
			.vsize = 0x0450,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0600,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0480,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0b40,
			.vsize = 0x0870,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x05A0,
			.vsize = 0x0438,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus11[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x05c0,
			.vsize = 0x0450,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x05c0,
			.vsize = 0x0114,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus12[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0a00,
			.vsize = 0x0600,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus13[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS_NE_PIX_1,
		},
	},
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
		fd->num_entries = ARRAY_SIZE(frame_desc_vid);
		memcpy(fd->entry, frame_desc_vid, sizeof(frame_desc_vid));
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_hs);
		memcpy(fd->entry, frame_desc_hs, sizeof(frame_desc_hs));
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_slim);
		memcpy(fd->entry, frame_desc_slim, sizeof(frame_desc_slim));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus1);
		memcpy(fd->entry, frame_desc_cus1, sizeof(frame_desc_cus1));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus2);
		memcpy(fd->entry, frame_desc_cus2, sizeof(frame_desc_cus2));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus3);
		memcpy(fd->entry, frame_desc_cus3, sizeof(frame_desc_cus3));
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
		fd->num_entries = ARRAY_SIZE(frame_desc_cus6);
		memcpy(fd->entry, frame_desc_cus6, sizeof(frame_desc_cus6));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM7:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus7);
		memcpy(fd->entry, frame_desc_cus7, sizeof(frame_desc_cus7));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM8:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus8);
		memcpy(fd->entry, frame_desc_cus8, sizeof(frame_desc_cus8));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM9:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus9);
		memcpy(fd->entry, frame_desc_cus9, sizeof(frame_desc_cus9));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM10:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus10);
		memcpy(fd->entry, frame_desc_cus10, sizeof(frame_desc_cus10));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM11:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus11);
		memcpy(fd->entry, frame_desc_cus11, sizeof(frame_desc_cus11));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM12:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus12);
		memcpy(fd->entry, frame_desc_cus12, sizeof(frame_desc_cus12));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM13:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cus13);
		memcpy(fd->entry, frame_desc_cus13, sizeof(frame_desc_cus13));
		break;
	default:
		return -1;
	}

	return 0;
}
#endif

static const struct subdrv_ctx defctx = {

	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_step = 1,
	.exposure_def = 0x3D0,
	.exposure_max = 65532 - 48, /* exposure reg is limited to 4x. max = max - margin */
	.exposure_min = 24,
	.exposure_step = 1,
	.frame_time_delay_frame = 3,
	.margin = 48, /* exp margin */
	.max_frame_length = 0xffff,

	.mirror = IMAGE_HV_MIRROR,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,

	.shutter = 0x3D0,	/* current shutter */
	.gain = BASEGAIN * 4,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = 0,
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x20, /* record current sensor's i2c write id */
	.readout_length = 0,
	.read_margin = 10,
	.current_ae_effective_frame = 2,
	.extend_frame_length_en = KAL_FALSE,
	.ae_ctrl_gph_en = KAL_FALSE,
};

static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int get_temp(struct subdrv_ctx *ctx, int *temp)
{
	*temp = get_sensor_temperature(ctx) * 1000;
	return 0;
}

static int get_csi_param(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id,
	struct mtk_csi_param *csi_param)
{
	csi_param->legacy_phy = 0;
	csi_param->not_fixed_trail_settle = 0;

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	case SENSOR_SCENARIO_ID_CUSTOM3:
		csi_param->cphy_settle = 65;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	case SENSOR_SCENARIO_ID_CUSTOM4:
		csi_param->cphy_settle = 58;
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	case SENSOR_SCENARIO_ID_CUSTOM2:
	case SENSOR_SCENARIO_ID_CUSTOM5:
		csi_param->cphy_settle = 56;
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
	case SENSOR_SCENARIO_ID_CUSTOM1:
		csi_param->cphy_settle = 59;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM6:
		csi_param->cphy_settle = 67;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM7:
		csi_param->cphy_settle = 85;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM8:
	case SENSOR_SCENARIO_ID_CUSTOM9:
	case SENSOR_SCENARIO_ID_CUSTOM10:
	case SENSOR_SCENARIO_ID_CUSTOM11:
	case SENSOR_SCENARIO_ID_CUSTOM12:
	case SENSOR_SCENARIO_ID_CUSTOM13:
		csi_param->cphy_settle = 65;//0x12;
		break;
	default:
		break;
	}
	return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,
	unsigned int sof_cnt)
{
	DEBUG_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on = %d",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = KAL_FALSE;
		ctx->ref_sof_cnt = 0;
		DEBUG_LOG(ctx, "seamless_switch disabled. ctx->fast_mode_on = %d", ctx->fast_mode_on);
		set_cmos_sensor_8(ctx, 0x3010, 0x00);
		commit_write_sensor(ctx);
	}
	return 0;
}

static int update_sof_cnt(struct subdrv_ctx *ctx,
	unsigned int sof_cnt)
{
	DEBUG_LOG(ctx, "update ctx->sof_cnt(%u)", sof_cnt);
	ctx->sof_cnt = sof_cnt;
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
	.get_temp = get_temp,
	.get_csi_param = get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	//{HW_ID_PDN, 0, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2804000, 3},
	{HW_ID_AVDD1, 1804000, 3},
	{HW_ID_AFVDD, 2804000, 3},
	{HW_ID_DVDD, 1, 4},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	//{HW_ID_PDN, 1, 0},
	{HW_ID_RST, 1, 5}
};

const struct subdrv_entry imx766_mipi_raw_entry = {
	.name = "imx766_mipi_raw",
	.id = IMX766_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};
