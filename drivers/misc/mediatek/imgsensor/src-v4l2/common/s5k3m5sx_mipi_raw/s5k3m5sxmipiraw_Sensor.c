// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

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

#include "s5k3m5sxmipiraw_Sensor.h"
#include "s5k3m5_ana_gain_table.h"
#include "s5k3m5sx_eeprom.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"
#include "adaptor.h"

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define write_cmos_sensor(...) subdrv_i2c_wr_u16(__VA_ARGS__)
#define table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u16(__VA_ARGS__)
#define burst_table_write_cmos_sensor(...) subdrv_i2c_wr_p16(__VA_ARGS__)
#define I2C_BURST 1

#define S5K3M5SX_EEPROM_READ_ID  0xA1
#define S5K3M5SX_EEPROM_WRITE_ID 0xA0

#define PFX "S5K3M5SX_camera_sensor"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, S5K3M5SX_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}

#define   WRITE_DATA_MAX_LENGTH     (16)
static kal_int32 table_write_eeprom_30Bytes(struct subdrv_ctx *ctx,
        kal_uint16 addr, kal_uint8 *para, kal_uint32 len)
{
	kal_int32 ret = ERROR_NONE;
    ret = adaptor_i2c_wr_p8(ctx->i2c_client, S5K3M5SX_EEPROM_WRITE_ID >> 1,
            addr, para, WRITE_DATA_MAX_LENGTH);

	return ret;
}

static kal_int32 write_eeprom_protect(struct subdrv_ctx *ctx, kal_uint16 enable)
{
	kal_int32 ret = ERROR_NONE;
    kal_uint16 reg = 0x8000;
	if (enable) {
		adaptor_i2c_wr_u8(ctx->i2c_client, S5K3M5SX_EEPROM_WRITE_ID >> 1, reg, 0x0E);
	}
	else {
		adaptor_i2c_wr_u8(ctx->i2c_client, S5K3M5SX_EEPROM_WRITE_ID >> 1, reg, 0x00);
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
		if ((pStereodata->uSensorId == S5K3M5SX_SENSOR_ID) && (data_length == CALI_DATA_SLAVE_LENGTH)
			&& (data_base == S5K3M5SX_STEREO_START_ADDR)) {
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
			LOG_INF("com_0:0x%x\n", read_cmos_eeprom_8(ctx, S5K3M5SX_STEREO_START_ADDR));
			LOG_INF("com_39:0x%x\n", read_cmos_eeprom_8(ctx, S5K3M5SX_STEREO_START_ADDR+39));
			LOG_INF("innal_40:0x%x\n", read_cmos_eeprom_8(ctx, S5K3M5SX_STEREO_START_ADDR+40));
			LOG_INF("innal_1556:0x%x\n", read_cmos_eeprom_8(ctx, S5K3M5SX_STEREO_START_ADDR+1556));
			LOG_INF("write_Module_data Write end\n");
		} else if ((pStereodata->uSensorId == S5K3M5SX_SENSOR_ID) && (data_length < AESYNC_DATA_LENGTH_TOTAL)
			&& (data_base == S5K3M5SX_AESYNC_START_ADDR)) {
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
			        read_cmos_eeprom_8(ctx, S5K3M5SX_AESYNC_START_ADDR),
					read_cmos_eeprom_8(ctx, S5K3M5SX_AESYNC_START_ADDR+1),
					read_cmos_eeprom_8(ctx, S5K3M5SX_AESYNC_START_ADDR+2),
					read_cmos_eeprom_8(ctx, S5K3M5SX_AESYNC_START_ADDR+3),
					read_cmos_eeprom_8(ctx, S5K3M5SX_AESYNC_START_ADDR+4),
					read_cmos_eeprom_8(ctx, S5K3M5SX_AESYNC_START_ADDR+5),
					read_cmos_eeprom_8(ctx, S5K3M5SX_AESYNC_START_ADDR+6),
					read_cmos_eeprom_8(ctx, S5K3M5SX_AESYNC_START_ADDR+7));
			LOG_INF("AESync write_Module_data Write end\n");
		} else {
			LOG_INF("Invalid Sensor id:0x%x write eeprom\n", pStereodata->uSensorId);
			return -1;
		}
	} else {
		LOG_INF("s5k3m5sx write_Module_data pStereodata is null\n");
		return -1;
	}
	return ret;
}

#define _I2C_BUF_SIZE 256
static kal_uint16 _i2c_data[_I2C_BUF_SIZE];
static unsigned int _size_to_write;

static void commit_write_sensor(struct subdrv_ctx *ctx)
{
	if (_size_to_write) {
		table_write_cmos_sensor(ctx, _i2c_data, _size_to_write);
		memset(_i2c_data, 0x0, sizeof(_i2c_data));
		_size_to_write = 0;
	}
}

static void set_cmos_sensor(struct subdrv_ctx *ctx,
			kal_uint16 reg, kal_uint16 val)
{
	if (_size_to_write > _I2C_BUF_SIZE - 2)
		commit_write_sensor(ctx);

	_i2c_data[_size_to_write++] = reg;
	_i2c_data[_size_to_write++] = val;
}

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5K3M5SX_SENSOR_ID,
	.checksum_value = 0x350174bc,
	.pre = {
		.pclk = 482000000,
		.linelength = 4848,
		.framelength = 3312,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_pixel_rate = 576000000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 482000000,
		.linelength = 4848,
		.framelength = 3314,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_pixel_rate = 576000000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 482000000,
		.linelength = 4848,
		.framelength = 3314,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 2368,
		.mipi_pixel_rate = 576000000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 482000000,
		.linelength = 4848,
		.framelength = 1656,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1184,
		.mipi_pixel_rate = 576000000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
	},
	.slim_video = {
		.pclk = 482000000,
		.linelength = 4848,
		.framelength = 1656,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1184,
		.mipi_pixel_rate = 576000000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
	},
	.custom1 = {
		.pclk = 482000000,
		.linelength = 4848,
		.framelength = 4142,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_pixel_rate = 576000000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
	},
	.custom2 = {
		.pclk = 482000000,
		.linelength = 4848,
		.framelength = 3314,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 2368,
		.mipi_pixel_rate = 576000000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.custom3 = {
		.pclk = 482000000,
		.linelength = 4848,
		.framelength = 3314,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_pixel_rate = 576000000,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.margin = 4,
	.min_shutter = 4,
	.min_gain = BASEGAIN,
	.max_gain = 16*BASEGAIN,
	.min_gain_iso = 100,
	.gain_step = 2,
	.gain_type = 2,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 8,
	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x5a, 0x20, 0xff},
	.i2c_speed = 1000,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 24,
	.i4OffsetY = 24,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 16,
	.i4SubBlkW = 8,
	.i4SubBlkH = 8,
	.i4PosL = {
		{26, 25}, {34, 25}, {42, 25}, {50, 25}, {30, 37},
		{38, 37}, {46, 37}, {54, 37}, {26, 45}, {34, 45},
		{42, 45}, {50, 45}, {30, 49}, {38, 49}, {46, 49},
		{54, 49}
	 },
	.i4PosR = {
		{26, 29}, {34, 29}, {42, 29}, {50, 29}, {30, 33},
		{38, 33}, {46, 33}, {54, 33}, {26, 41}, {34, 41},
		{42, 41}, {50, 41}, {30, 53}, {38, 53}, {46, 53},
		{54, 53}
	},
	.i4BlockNumX = 130,
	.i4BlockNumY = 96,
	.i4LeFirst = 0,
	.i4Crop = {
		{104, 60}, {0, 0}, {0, 376}, {0, 0}, {0, 0},
		{0, 0}, {0, 376}, {0, 0}
	},
	.iMirrorFlip = 3,
};

static kal_uint16 sensor_init_setting_array1[] = {
	0x6028, 0x2000,
	0x602A, 0x3EAC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0549,
	0x6F12, 0x0448,
	0x6F12, 0x054A,
	0x6F12, 0xC1F8,
	0x6F12, 0xC804,
	0x6F12, 0x101A,
	0x6F12, 0xA1F8,
	0x6F12, 0xCC04,
	0x6F12, 0x00F0,
	0x6F12, 0x70BA,
	0x6F12, 0x2000,
	0x6F12, 0x4594,
	0x6F12, 0x2000,
	0x6F12, 0x2E50,
	0x6F12, 0x2000,
	0x6F12, 0x7000,
	0x6F12, 0x10B5,
	0x6F12, 0x00F0,
	0x6F12, 0xB7FA,
	0x6F12, 0xFF49,
	0x6F12, 0x0120,
	0x6F12, 0x0880,
	0x6F12, 0x10BD,
	0x6F12, 0x2DE9,
	0x6F12, 0xF041,
	0x6F12, 0xFD4C,
	0x6F12, 0xFB4F,
	0x6F12, 0x0026,
	0x6F12, 0xB4F8,
	0x6F12, 0x6A52,
	0x6F12, 0x3888,
	0x6F12, 0x08B1,
	0x6F12, 0xA4F8,
	0x6F12, 0x6A62,
	0x6F12, 0x00F0,
	0x6F12, 0xABFA,
	0x6F12, 0x3E80,
	0x6F12, 0xA4F8,
	0x6F12, 0x6A52,
	0x6F12, 0xBDE8,
	0x6F12, 0xF081,
	0x6F12, 0x2DE9,
	0x6F12, 0xF041,
	0x6F12, 0x0746,
	0x6F12, 0xF248,
	0x6F12, 0x0E46,
	0x6F12, 0x0022,
	0x6F12, 0x4068,
	0x6F12, 0x84B2,
	0x6F12, 0x050C,
	0x6F12, 0x2146,
	0x6F12, 0x2846,
	0x6F12, 0x00F0,
	0x6F12, 0x9EFA,
	0x6F12, 0x3146,
	0x6F12, 0x3846,
	0x6F12, 0x00F0,
	0x6F12, 0x9FFA,
	0x6F12, 0xED4F,
	0x6F12, 0x4DF2,
	0x6F12, 0x0C26,
	0x6F12, 0x4FF4,
	0x6F12, 0x8061,
	0x6F12, 0x3A78,
	0x6F12, 0x3046,
	0x6F12, 0x00F0,
	0x6F12, 0x91FA,
	0x6F12, 0x7878,
	0x6F12, 0xB8B3,
	0x6F12, 0x0022,
	0x6F12, 0x8021,
	0x6F12, 0x3046,
	0x6F12, 0x00F0,
	0x6F12, 0x8AFA,
	0x6F12, 0xE648,
	0x6F12, 0x0088,
	0x6F12, 0xE64B,
	0x6F12, 0xA3F8,
	0x6F12, 0x5C02,
	0x6F12, 0xE448,
	0x6F12, 0x001D,
	0x6F12, 0x0088,
	0x6F12, 0xA3F8,
	0x6F12, 0x5E02,
	0x6F12, 0xB3F8,
	0x6F12, 0x5C02,
	0x6F12, 0xB3F8,
	0x6F12, 0x5E12,
	0x6F12, 0x4218,
	0x6F12, 0x02D0,
	0x6F12, 0x8002,
	0x6F12, 0xB0FB,
	0x6F12, 0xF2F2,
	0x6F12, 0x91B2,
	0x6F12, 0xDE4A,
	0x6F12, 0xA3F8,
	0x6F12, 0x6012,
	0x6F12, 0xB2F8,
	0x6F12, 0x1602,
	0x6F12, 0xB2F8,
	0x6F12, 0x1422,
	0x6F12, 0xA3F8,
	0x6F12, 0x9805,
	0x6F12, 0xA3F8,
	0x6F12, 0x9A25,
	0x6F12, 0x8018,
	0x6F12, 0x04D0,
	0x6F12, 0x9202,
	0x6F12, 0xB2FB,
	0x6F12, 0xF0F0,
	0x6F12, 0xA3F8,
	0x6F12, 0x9C05,
	0x6F12, 0xB3F8,
	0x6F12, 0x9C05,
	0x6F12, 0x0A18,
	0x6F12, 0x01FB,
	0x6F12, 0x1020,
	0x6F12, 0x40F3,
	0x6F12, 0x9510,
	0x6F12, 0x1028,
	0x6F12, 0x06DC,
	0x6F12, 0x0028,
	0x6F12, 0x05DA,
	0x6F12, 0x0020,
	0x6F12, 0x03E0,
	0x6F12, 0xFFE7,
	0x6F12, 0x0122,
	0x6F12, 0xC5E7,
	0x6F12, 0x1020,
	0x6F12, 0xCE49,
	0x6F12, 0x0880,
	0x6F12, 0x2146,
	0x6F12, 0x2846,
	0x6F12, 0xBDE8,
	0x6F12, 0xF041,
	0x6F12, 0x0122,
	0x6F12, 0x00F0,
	0x6F12, 0x4ABA,
	0x6F12, 0xF0B5,
	0x6F12, 0xCA4C,
	0x6F12, 0xDDE9,
	0x6F12, 0x0565,
	0x6F12, 0x08B1,
	0x6F12, 0x2788,
	0x6F12, 0x0760,
	0x6F12, 0x09B1,
	0x6F12, 0x6088,
	0x6F12, 0x0860,
	0x6F12, 0x12B1,
	0x6F12, 0xA088,
	0x6F12, 0x401C,
	0x6F12, 0x1060,
	0x6F12, 0x0BB1,
	0x6F12, 0xE088,
	0x6F12, 0x1860,
	0x6F12, 0x0EB1,
	0x6F12, 0xA07B,
	0x6F12, 0x3060,
	0x6F12, 0x002D,
	0x6F12, 0x01D0,
	0x6F12, 0xE07B,
	0x6F12, 0x2860,
	0x6F12, 0xF0BD,
	0x6F12, 0x70B5,
	0x6F12, 0x0646,
	0x6F12, 0xB648,
	0x6F12, 0x0022,
	0x6F12, 0x8068,
	0x6F12, 0x84B2,
	0x6F12, 0x050C,
	0x6F12, 0x2146,
	0x6F12, 0x2846,
	0x6F12, 0x00F0,
	0x6F12, 0x26FA,
	0x6F12, 0x3046,
	0x6F12, 0x00F0,
	0x6F12, 0x2DFA,
	0x6F12, 0xB848,
	0x6F12, 0x0368,
	0x6F12, 0xB3F8,
	0x6F12, 0x7401,
	0x6F12, 0x010A,
	0x6F12, 0xB648,
	0x6F12, 0x4268,
	0x6F12, 0x82F8,
	0x6F12, 0x5010,
	0x6F12, 0x93F8,
	0x6F12, 0x7511,
	0x6F12, 0x82F8,
	0x6F12, 0x5210,
	0x6F12, 0xB3F8,
	0x6F12, 0x7811,
	0x6F12, 0x090A,
	0x6F12, 0x82F8,
	0x6F12, 0x5810,
	0x6F12, 0x93F8,
	0x6F12, 0x7911,
	0x6F12, 0x82F8,
	0x6F12, 0x5A10,
	0x6F12, 0x33F8,
	0x6F12, 0xF01F,
	0x6F12, 0x0068,
	0x6F12, 0x090A,
	0x6F12, 0x00F8,
	0x6F12, 0xCE1F,
	0x6F12, 0x5978,
	0x6F12, 0x8170,
	0x6F12, 0x5988,
	0x6F12, 0x090A,
	0x6F12, 0x0171,
	0x6F12, 0xD978,
	0x6F12, 0x8171,
	0x6F12, 0x988C,
	0x6F12, 0x000A,
	0x6F12, 0x9074,
	0x6F12, 0x93F8,
	0x6F12, 0x2500,
	0x6F12, 0x1075,
	0x6F12, 0xD88C,
	0x6F12, 0x000A,
	0x6F12, 0x9075,
	0x6F12, 0x93F8,
	0x6F12, 0x2700,
	0x6F12, 0x1076,
	0x6F12, 0xB3F8,
	0x6F12, 0xB000,
	0x6F12, 0x000A,
	0x6F12, 0x82F8,
	0x6F12, 0x7E00,
	0x6F12, 0x93F8,
	0x6F12, 0xB100,
	0x6F12, 0x82F8,
	0x6F12, 0x8000,
	0x6F12, 0x9548,
	0x6F12, 0x90F8,
	0x6F12, 0xB313,
	0x6F12, 0x82F8,
	0x6F12, 0x8210,
	0x6F12, 0x90F8,
	0x6F12, 0xB103,
	0x6F12, 0x82F8,
	0x6F12, 0x8400,
	0x6F12, 0x93F8,
	0x6F12, 0xB400,
	0x6F12, 0x82F8,
	0x6F12, 0x8600,
	0x6F12, 0x0020,
	0x6F12, 0x82F8,
	0x6F12, 0x8800,
	0x6F12, 0x93F8,
	0x6F12, 0x6211,
	0x6F12, 0x82F8,
	0x6F12, 0x9610,
	0x6F12, 0x93F8,
	0x6F12, 0x0112,
	0x6F12, 0x82F8,
	0x6F12, 0x9E10,
	0x6F12, 0x93F8,
	0x6F12, 0x0212,
	0x6F12, 0x82F8,
	0x6F12, 0xA010,
	0x6F12, 0x82F8,
	0x6F12, 0xA200,
	0x6F12, 0x82F8,
	0x6F12, 0xA400,
	0x6F12, 0x93F8,
	0x6F12, 0x0512,
	0x6F12, 0x82F8,
	0x6F12, 0xA610,
	0x6F12, 0x93F8,
	0x6F12, 0x0612,
	0x6F12, 0x82F8,
	0x6F12, 0xA810,
	0x6F12, 0x93F8,
	0x6F12, 0x0712,
	0x6F12, 0x82F8,
	0x6F12, 0xAA10,
	0x6F12, 0x82F8,
	0x6F12, 0xAC00,
	0x6F12, 0x5A20,
	0x6F12, 0x82F8,
	0x6F12, 0xAD00,
	0x6F12, 0x93F8,
	0x6F12, 0x0902,
	0x6F12, 0x82F8,
	0x6F12, 0xAE00,
	0x6F12, 0x2146,
	0x6F12, 0x2846,
	0x6F12, 0xBDE8,
	0x6F12, 0x7040,
	0x6F12, 0x0122,
	0x6F12, 0x00F0,
	0x6F12, 0xAFB9,
	0x6F12, 0x70B5,
	0x6F12, 0x7548,
	0x6F12, 0x0022,
	0x6F12, 0x0169,
	0x6F12, 0x0C0C,
	0x6F12, 0x8DB2,
	0x6F12, 0x2946,
	0x6F12, 0x2046,
	0x6F12, 0x00F0,
	0x6F12, 0xA5F9,
	0x6F12, 0x00F0,
	0x6F12, 0xB2F9,
	0x6F12, 0x7248,
	0x6F12, 0x8078,
	0x6F12, 0x08B1,
	0x6F12, 0x4F22,
	0x6F12, 0x00E0,
	0x6F12, 0x2522,
	0x6F12, 0x7748,
	0x6F12, 0x90F8,
	0x6F12, 0xE400,
	0x6F12, 0x0328,
	0x6F12, 0x07D1,
	0x6F12, 0x42F0,
	0x6F12, 0x8002,
	0x6F12, 0x4FF4,
	0x6F12, 0x8361,
	0x6F12, 0x48F6,
	0x6F12, 0x7A20,
	0x6F12, 0x00F0,
	0x6F12, 0xA4F9,
	0x6F12, 0x2946,
	0x6F12, 0x2046,
	0x6F12, 0xBDE8,
	0x6F12, 0x7040,
	0x6F12, 0x0122,
	0x6F12, 0x00F0,
	0x6F12, 0x89B9,
	0x6F12, 0x10B5,
	0x6F12, 0x0221,
	0x6F12, 0x7620,
	0x6F12, 0x00F0,
	0x6F12, 0x9DF9,
	0x6F12, 0x0221,
	0x6F12, 0x4420,
	0x6F12, 0x00F0,
	0x6F12, 0x99F9,
	0x6F12, 0x4021,
	0x6F12, 0x4520,
	0x6F12, 0x00F0,
	0x6F12, 0x95F9,
	0x6F12, 0x5D49,
	0x6F12, 0x0420,
	0x6F12, 0xA1F8,
	0x6F12, 0x3A06,
	0x6F12, 0x10BD,
	0x6F12, 0x7047,
	0x6F12, 0x7047,
	0x6F12, 0x08B5,
	0x6F12, 0x5949,
	0x6F12, 0x3120,
	0x6F12, 0x6A46,
	0x6F12, 0x81F8,
	0x6F12, 0x4306,
	0x6F12, 0x5A20,
	0x6F12, 0x8DF8,
	0x6F12, 0x0000,
	0x6F12, 0x0121,
	0x6F12, 0x7520,
	0x6F12, 0x00F0,
	0x6F12, 0x86F9,
	0x6F12, 0x9DF8,
	0x6F12, 0x0000,
	0x6F12, 0x08BD,
	0x6F12, 0x7047,
	0x6F12, 0x5248,
	0x6F12, 0x10B5,
	0x6F12, 0x8078,
	0x6F12, 0x18B1,
	0x6F12, 0x0021,
	0x6F12, 0x4420,
	0x6F12, 0x00F0,
	0x6F12, 0x75F9,
	0x6F12, 0x4D49,
	0x6F12, 0x0220,
	0x6F12, 0xA1F8,
	0x6F12, 0x3A06,
	0x6F12, 0x10BD,
	0x6F12, 0x5448,
	0x6F12, 0x90F8,
	0x6F12, 0xE400,
	0x6F12, 0x0328,
	0x6F12, 0x01D0,
	0x6F12, 0x00F0,
	0x6F12, 0x73B9,
	0x6F12, 0xAFF2,
	0x6F12, 0x2B01,
	0x6F12, 0x4648,
	0x6F12, 0xC0F8,
	0x6F12, 0x4C16,
	0x6F12, 0x4649,
	0x6F12, 0x8978,
	0x6F12, 0x11B1,
	0x6F12, 0xAFF2,
	0x6F12, 0x8501,
	0x6F12, 0x01E0,
	0x6F12, 0xAFF2,
	0x6F12, 0x6501,
	0x6F12, 0xC0F8,
	0x6F12, 0x4816,
	0x6F12, 0xAFF2,
	0x6F12, 0x6B01,
	0x6F12, 0xC0F8,
	0x6F12, 0x4416,
	0x6F12, 0xAFF2,
	0x6F12, 0x7101,
	0x6F12, 0xC0F8,
	0x6F12, 0x5016,
	0x6F12, 0xAFF2,
	0x6F12, 0x5901,
	0x6F12, 0xC0F8,
	0x6F12, 0x5416,
	0x6F12, 0x7047,
	0x6F12, 0x2DE9,
	0x6F12, 0xF041,
	0x6F12, 0x434C,
	0x6F12, 0x4249,
	0x6F12, 0x0646,
	0x6F12, 0xB4F8,
	0x6F12, 0x6670,
	0x6F12, 0xC989,
	0x6F12, 0xB4F8,
	0x6F12, 0x7E20,
	0x6F12, 0x0020,
	0x6F12, 0xC1B1,
	0x6F12, 0x2146,
	0x6F12, 0xD1F8,
	0x6F12, 0x9010,
	0x6F12, 0x72B1,
	0x6F12, 0x8FB1,
	0x6F12, 0x0846,
	0x6F12, 0x00F0,
	0x6F12, 0x48F9,
	0x6F12, 0x0546,
	0x6F12, 0xA06F,
	0x6F12, 0x00F0,
	0x6F12, 0x44F9,
	0x6F12, 0x8542,
	0x6F12, 0x02D2,
	0x6F12, 0xD4F8,
	0x6F12, 0x9000,
	0x6F12, 0x26E0,
	0x6F12, 0xA06F,
	0x6F12, 0x24E0,
	0x6F12, 0x002F,
	0x6F12, 0xFBD1,
	0x6F12, 0x002A,
	0x6F12, 0x24D0,
	0x6F12, 0x0846,
	0x6F12, 0x1EE0,
	0x6F12, 0x2849,
	0x6F12, 0x8D88,
	0x6F12, 0x8968,
	0x6F12, 0x4B42,
	0x6F12, 0x77B1,
	0x6F12, 0x2F48,
	0x6F12, 0x406F,
	0x6F12, 0x10E0,
	0x6F12, 0x4242,
	0x6F12, 0x00E0,
	0x6F12, 0x0246,
	0x6F12, 0x0029,
	0x6F12, 0x0FDB,
	0x6F12, 0x8A42,
	0x6F12, 0x0FDD,
	0x6F12, 0x3046,
	0x6F12, 0xBDE8,
	0x6F12, 0xF041,
	0x6F12, 0x00F0,
	0x6F12, 0x28B9,
	0x6F12, 0x002A,
	0x6F12, 0x0CD0,
	0x6F12, 0x2748,
	0x6F12, 0xD0F8,
	0x6F12, 0x8800,
	0x6F12, 0x25B1,
	0x6F12, 0x0028,
	0x6F12, 0xEDDA,
	0x6F12, 0xEAE7,
	0x6F12, 0x1946,
	0x6F12, 0xEDE7,
	0x6F12, 0x00F0,
	0x6F12, 0x20F9,
	0x6F12, 0xE060,
	0x6F12, 0x0120,
	0x6F12, 0x3DE6,
	0x6F12, 0x2DE9,
	0x6F12, 0xF047,
	0x6F12, 0x8146,
	0x6F12, 0x0F46,
	0x6F12, 0x0846,
	0x6F12, 0x00F0,
	0x6F12, 0x1BF9,
	0x6F12, 0x1B4C,
	0x6F12, 0x0026,
	0x6F12, 0x608A,
	0x6F12, 0x10B1,
	0x6F12, 0x00F0,
	0x6F12, 0x1AF9,
	0x6F12, 0x6682,
	0x6F12, 0x194D,
	0x6F12, 0x2888,
	0x6F12, 0x0128,
	0x6F12, 0x60D1,
	0x6F12, 0xA08B,
	0x6F12, 0x0028,
	0x6F12, 0x5DD1,
	0x6F12, 0x002F,
	0x6F12, 0x5BD1,
	0x6F12, 0x104F,
	0x6F12, 0x3868,
	0x6F12, 0xB0F8,
	0x6F12, 0x1403,
	0x6F12, 0x38B1,
	0x6F12, 0x2889,
	0x6F12, 0x401C,
	0x6F12, 0x80B2,
	0x6F12, 0x2881,
	0x6F12, 0xFF28,
	0x6F12, 0x01D9,
	0x6F12, 0xA08C,
	0x6F12, 0x2881,
	0x6F12, 0x0F48,
	0x6F12, 0xEE60,
	0x6F12, 0xB0F8,
	0x6F12, 0x5E80,
	0x6F12, 0x1BE0,
	0x6F12, 0x2000,
	0x6F12, 0x4580,
	0x6F12, 0x2000,
	0x6F12, 0x2E50,
	0x6F12, 0x2000,
	0x6F12, 0x6200,
	0x6F12, 0x4000,
	0x6F12, 0x9404,
	0x6F12, 0x2000,
	0x6F12, 0x38E0,
	0x6F12, 0x4000,
	0x6F12, 0xD000,
	0x6F12, 0x4000,
	0x6F12, 0xA410,
	0x6F12, 0x2000,
	0x6F12, 0x2C66,
	0x6F12, 0x2000,
	0x6F12, 0x0890,
	0x6F12, 0x2000,
	0x6F12, 0x3620,
	0x6F12, 0x2000,
	0x6F12, 0x0DE0,
	0x6F12, 0x2000,
	0x6F12, 0x2BC0,
	0x6F12, 0x2000,
	0x6F12, 0x3580,
	0x6F12, 0x4000,
	0x6F12, 0x7000,
	0x6F12, 0x40F2,
	0x6F12, 0xFF31,
	0x6F12, 0x0B20,
	0x6F12, 0x00F0,
	0x6F12, 0xE2F8,
	0x6F12, 0x3868,
	0x6F12, 0xB0F8,
	0x6F12, 0x1213,
	0x6F12, 0x19B1,
	0x6F12, 0x4846,
	0x6F12, 0x00F0,
	0x6F12, 0xC7F8,
	0x6F12, 0x0AE0,
	0x6F12, 0xB0F8,
	0x6F12, 0x1403,
	0x6F12, 0xC0B1,
	0x6F12, 0x2889,
	0x6F12, 0xB4F9,
	0x6F12, 0x2410,
	0x6F12, 0x8842,
	0x6F12, 0x13DB,
	0x6F12, 0x4846,
	0x6F12, 0xFFF7,
	0x6F12, 0x5AFF,
	0x6F12, 0x78B1,
	0x6F12, 0x2E81,
	0x6F12, 0x00F0,
	0x6F12, 0xD0F8,
	0x6F12, 0xE868,
	0x6F12, 0x2861,
	0x6F12, 0x208E,
	0x6F12, 0x18B1,
	0x6F12, 0x608E,
	0x6F12, 0x18B9,
	0x6F12, 0x00F0,
	0x6F12, 0xCDF8,
	0x6F12, 0x608E,
	0x6F12, 0x10B1,
	0x6F12, 0xE889,
	0x6F12, 0x2887,
	0x6F12, 0x6686,
	0x6F12, 0x4046,
	0x6F12, 0xBDE8,
	0x6F12, 0xF047,
	0x6F12, 0x00F0,
	0x6F12, 0xC8B8,
	0x6F12, 0xBDE8,
	0x6F12, 0xF087,
	0x6F12, 0x10B5,
	0x6F12, 0x6021,
	0x6F12, 0x0B20,
	0x6F12, 0x00F0,
	0x6F12, 0xC6F8,
	0x6F12, 0x8106,
	0x6F12, 0x4FEA,
	0x6F12, 0x4061,
	0x6F12, 0x05D5,
	0x6F12, 0x0029,
	0x6F12, 0x0BDA,
	0x6F12, 0xBDE8,
	0x6F12, 0x1040,
	0x6F12, 0x00F0,
	0x6F12, 0xC1B8,
	0x6F12, 0x0029,
	0x6F12, 0x03DA,
	0x6F12, 0xBDE8,
	0x6F12, 0x1040,
	0x6F12, 0x00F0,
	0x6F12, 0xC0B8,
	0x6F12, 0x8006,
	0x6F12, 0x03D5,
	0x6F12, 0xBDE8,
	0x6F12, 0x1040,
	0x6F12, 0x00F0,
	0x6F12, 0xBFB8,
	0x6F12, 0x10BD,
	0x6F12, 0x70B5,
	0x6F12, 0x1E4C,
	0x6F12, 0x0020,
	0x6F12, 0x2080,
	0x6F12, 0xAFF2,
	0x6F12, 0xDF40,
	0x6F12, 0x1C4D,
	0x6F12, 0x2861,
	0x6F12, 0xAFF2,
	0x6F12, 0xD940,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0xB941,
	0x6F12, 0xA861,
	0x6F12, 0x1948,
	0x6F12, 0x00F0,
	0x6F12, 0xB2F8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0xD531,
	0x6F12, 0x6060,
	0x6F12, 0x1748,
	0x6F12, 0x00F0,
	0x6F12, 0xABF8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0x1341,
	0x6F12, 0xA060,
	0x6F12, 0x1448,
	0x6F12, 0x00F0,
	0x6F12, 0xA4F8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0xED21,
	0x6F12, 0xE060,
	0x6F12, 0x1248,
	0x6F12, 0x00F0,
	0x6F12, 0x9DF8,
	0x6F12, 0x2061,
	0x6F12, 0xAFF2,
	0x6F12, 0x4920,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0x0B21,
	0x6F12, 0xE863,
	0x6F12, 0x0E48,
	0x6F12, 0x00F0,
	0x6F12, 0x93F8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0x8511,
	0x6F12, 0x0C48,
	0x6F12, 0x00F0,
	0x6F12, 0x8DF8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0xA701,
	0x6F12, 0xBDE8,
	0x6F12, 0x7040,
	0x6F12, 0x0948,
	0x6F12, 0x00F0,
	0x6F12, 0x85B8,
	0x6F12, 0x2000,
	0x6F12, 0x4580,
	0x6F12, 0x2000,
	0x6F12, 0x0840,
	0x6F12, 0x0001,
	0x6F12, 0x020D,
	0x6F12, 0x0000,
	0x6F12, 0x67CD,
	0x6F12, 0x0000,
	0x6F12, 0x3AE1,
	0x6F12, 0x0000,
	0x6F12, 0x72B1,
	0x6F12, 0x0000,
	0x6F12, 0x56D7,
	0x6F12, 0x0000,
	0x6F12, 0x5735,
	0x6F12, 0x0000,
	0x6F12, 0x0631,
	0x6F12, 0x45F6,
	0x6F12, 0x250C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F6,
	0x6F12, 0xF31C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x4AF2,
	0x6F12, 0xD74C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x40F2,
	0x6F12, 0x0D2C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x46F2,
	0x6F12, 0xCD7C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F2,
	0x6F12, 0xB12C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F2,
	0x6F12, 0x4F2C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F6,
	0x6F12, 0x017C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F6,
	0x6F12, 0x636C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F2,
	0x6F12, 0x0D0C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x4AF2,
	0x6F12, 0x5F4C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0xA56C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x1F5C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x7F5C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x312C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x40F2,
	0x6F12, 0xAB2C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0xF34C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x395C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x40F2,
	0x6F12, 0x117C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x40F2,
	0x6F12, 0xD92C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x054C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0xAF3C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x4B2C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x4AF6,
	0x6F12, 0xE75C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x30D5,
	0x6F12, 0x0103,
	0x6F12, 0x0000,
	0x6F12, 0x005E,
	0x602A, 0x1662,
	0x6F12, 0x1E00,
	0x602A, 0x1C9A,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0FF2,
	0x6F12, 0x0020,
	0x602A, 0x0EF6,
	0x6F12, 0x0100,
	0x602A, 0x23B2,
	0x6F12, 0x0001,
	0x602A, 0x0FE4,
	0x6F12, 0x0107,
	0x6F12, 0x07D0,
	0x602A, 0x12F8,
	0x6F12, 0x3D09,
	0x602A, 0x0E18,
	0x6F12, 0x0040,
	0x602A, 0x1066,
	0x6F12, 0x000C,
	0x602A, 0x13DE,
	0x6F12, 0x0000,
	0x602A, 0x12F2,
	0x6F12, 0x0F0F,
	0x602A, 0x13DC,
	0x6F12, 0x806F,
	0xF46E, 0x00C3,
	0xF46C, 0xBFA0,
	0xF44A, 0x0007,
	0xF456, 0x000A,
	0x6028, 0x2000,
	0x602A, 0x12F6,
	0x6F12, 0x7008,
	0x0BC6, 0x0000,
	0x0B36, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x2BC2,
	0x6F12, 0x0020,
	0x602A, 0x2BC4,
	0x6F12, 0x0020,
	0x602A, 0x6204,
	0x6F12, 0x0001,
	0x602A, 0x6208,
	0x6F12, 0x0000,
	0x6F12, 0x0030,
	0x6028, 0x2000,
	0x602A, 0x17C0,
	0x6F12, 0x143C,
};
static kal_uint16 sensor_init_setting_array1_burst[] = {
	0x0000,
	0x0000,
	0x0549,
	0x0448,
	0x054A,
	0xC1F8,
	0xC804,
	0x101A,
	0xA1F8,
	0xCC04,
	0x00F0,
	0x70BA,
	0x2000,
	0x4594,
	0x2000,
	0x2E50,
	0x2000,
	0x7000,
	0x10B5,
	0x00F0,
	0xB7FA,
	0xFF49,
	0x0120,
	0x0880,
	0x10BD,
	0x2DE9,
	0xF041,
	0xFD4C,
	0xFB4F,
	0x0026,
	0xB4F8,
	0x6A52,
	0x3888,
	0x08B1,
	0xA4F8,
	0x6A62,
	0x00F0,
	0xABFA,
	0x3E80,
	0xA4F8,
	0x6A52,
	0xBDE8,
	0xF081,
	0x2DE9,
	0xF041,
	0x0746,
	0xF248,
	0x0E46,
	0x0022,
	0x4068,
	0x84B2,
	0x050C,
	0x2146,
	0x2846,
	0x00F0,
	0x9EFA,
	0x3146,
	0x3846,
	0x00F0,
	0x9FFA,
	0xED4F,
	0x4DF2,
	0x0C26,
	0x4FF4,
	0x8061,
	0x3A78,
	0x3046,
	0x00F0,
	0x91FA,
	0x7878,
	0xB8B3,
	0x0022,
	0x8021,
	0x3046,
	0x00F0,
	0x8AFA,
	0xE648,
	0x0088,
	0xE64B,
	0xA3F8,
	0x5C02,
	0xE448,
	0x001D,
	0x0088,
	0xA3F8,
	0x5E02,
	0xB3F8,
	0x5C02,
	0xB3F8,
	0x5E12,
	0x4218,
	0x02D0,
	0x8002,
	0xB0FB,
	0xF2F2,
	0x91B2,
	0xDE4A,
	0xA3F8,
	0x6012,
	0xB2F8,
	0x1602,
	0xB2F8,
	0x1422,
	0xA3F8,
	0x9805,
	0xA3F8,
	0x9A25,
	0x8018,
	0x04D0,
	0x9202,
	0xB2FB,
	0xF0F0,
	0xA3F8,
	0x9C05,
	0xB3F8,
	0x9C05,
	0x0A18,
	0x01FB,
	0x1020,
	0x40F3,
	0x9510,
	0x1028,
	0x06DC,
	0x0028,
	0x05DA,
	0x0020,
	0x03E0,
	0xFFE7,
	0x0122,
	0xC5E7,
	0x1020,
	0xCE49,
	0x0880,
	0x2146,
	0x2846,
	0xBDE8,
	0xF041,
	0x0122,
	0x00F0,
	0x4ABA,
	0xF0B5,
	0xCA4C,
	0xDDE9,
	0x0565,
	0x08B1,
	0x2788,
	0x0760,
	0x09B1,
	0x6088,
	0x0860,
	0x12B1,
	0xA088,
	0x401C,
	0x1060,
	0x0BB1,
	0xE088,
	0x1860,
	0x0EB1,
	0xA07B,
	0x3060,
	0x002D,
	0x01D0,
	0xE07B,
	0x2860,
	0xF0BD,
	0x70B5,
	0x0646,
	0xB648,
	0x0022,
	0x8068,
	0x84B2,
	0x050C,
	0x2146,
	0x2846,
	0x00F0,
	0x26FA,
	0x3046,
	0x00F0,
	0x2DFA,
	0xB848,
	0x0368,
	0xB3F8,
	0x7401,
	0x010A,
	0xB648,
	0x4268,
	0x82F8,
	0x5010,
	0x93F8,
	0x7511,
	0x82F8,
	0x5210,
	0xB3F8,
	0x7811,
	0x090A,
	0x82F8,
	0x5810,
	0x93F8,
	0x7911,
	0x82F8,
	0x5A10,
	0x33F8,
	0xF01F,
	0x0068,
	0x090A,
	0x00F8,
	0xCE1F,
	0x5978,
	0x8170,
	0x5988,
	0x090A,
	0x0171,
	0xD978,
	0x8171,
	0x988C,
	0x000A,
	0x9074,
	0x93F8,
	0x2500,
	0x1075,
	0xD88C,
	0x000A,
	0x9075,
	0x93F8,
	0x2700,
	0x1076,
	0xB3F8,
	0xB000,
	0x000A,
	0x82F8,
	0x7E00,
	0x93F8,
	0xB100,
	0x82F8,
	0x8000,
	0x9548,
	0x90F8,
	0xB313,
	0x82F8,
	0x8210,
	0x90F8,
	0xB103,
	0x82F8,
	0x8400,
	0x93F8,
	0xB400,
	0x82F8,
	0x8600,
	0x0020,
	0x82F8,
	0x8800,
	0x93F8,
	0x6211,
	0x82F8,
	0x9610,
	0x93F8,
	0x0112,
	0x82F8,
	0x9E10,
	0x93F8,
	0x0212,
	0x82F8,
	0xA010,
	0x82F8,
	0xA200,
	0x82F8,
	0xA400,
	0x93F8,
	0x0512,
	0x82F8,
	0xA610,
	0x93F8,
	0x0612,
	0x82F8,
	0xA810,
	0x93F8,
	0x0712,
	0x82F8,
	0xAA10,
	0x82F8,
	0xAC00,
	0x5A20,
	0x82F8,
	0xAD00,
	0x93F8,
	0x0902,
	0x82F8,
	0xAE00,
	0x2146,
	0x2846,
	0xBDE8,
	0x7040,
	0x0122,
	0x00F0,
	0xAFB9,
	0x70B5,
	0x7548,
	0x0022,
	0x0169,
	0x0C0C,
	0x8DB2,
	0x2946,
	0x2046,
	0x00F0,
	0xA5F9,
	0x00F0,
	0xB2F9,
	0x7248,
	0x8078,
	0x08B1,
	0x4F22,
	0x00E0,
	0x2522,
	0x7748,
	0x90F8,
	0xE400,
	0x0328,
	0x07D1,
	0x42F0,
	0x8002,
	0x4FF4,
	0x8361,
	0x48F6,
	0x7A20,
	0x00F0,
	0xA4F9,
	0x2946,
	0x2046,
	0xBDE8,
	0x7040,
	0x0122,
	0x00F0,
	0x89B9,
	0x10B5,
	0x0221,
	0x7620,
	0x00F0,
	0x9DF9,
	0x0221,
	0x4420,
	0x00F0,
	0x99F9,
	0x4021,
	0x4520,
	0x00F0,
	0x95F9,
	0x5D49,
	0x0420,
	0xA1F8,
	0x3A06,
	0x10BD,
	0x7047,
	0x7047,
	0x08B5,
	0x5949,
	0x3120,
	0x6A46,
	0x81F8,
	0x4306,
	0x5A20,
	0x8DF8,
	0x0000,
	0x0121,
	0x7520,
	0x00F0,
	0x86F9,
	0x9DF8,
	0x0000,
	0x08BD,
	0x7047,
	0x5248,
	0x10B5,
	0x8078,
	0x18B1,
	0x0021,
	0x4420,
	0x00F0,
	0x75F9,
	0x4D49,
	0x0220,
	0xA1F8,
	0x3A06,
	0x10BD,
	0x5448,
	0x90F8,
	0xE400,
	0x0328,
	0x01D0,
	0x00F0,
	0x73B9,
	0xAFF2,
	0x2B01,
	0x4648,
	0xC0F8,
	0x4C16,
	0x4649,
	0x8978,
	0x11B1,
	0xAFF2,
	0x8501,
	0x01E0,
	0xAFF2,
	0x6501,
	0xC0F8,
	0x4816,
	0xAFF2,
	0x6B01,
	0xC0F8,
	0x4416,
	0xAFF2,
	0x7101,
	0xC0F8,
	0x5016,
	0xAFF2,
	0x5901,
	0xC0F8,
	0x5416,
	0x7047,
	0x2DE9,
	0xF041,
	0x434C,
	0x4249,
	0x0646,
	0xB4F8,
	0x6670,
	0xC989,
	0xB4F8,
	0x7E20,
	0x0020,
	0xC1B1,
	0x2146,
	0xD1F8,
	0x9010,
	0x72B1,
	0x8FB1,
	0x0846,
	0x00F0,
	0x48F9,
	0x0546,
	0xA06F,
	0x00F0,
	0x44F9,
	0x8542,
	0x02D2,
	0xD4F8,
	0x9000,
	0x26E0,
	0xA06F,
	0x24E0,
	0x002F,
	0xFBD1,
	0x002A,
	0x24D0,
	0x0846,
	0x1EE0,
	0x2849,
	0x8D88,
	0x8968,
	0x4B42,
	0x77B1,
	0x2F48,
	0x406F,
	0x10E0,
	0x4242,
	0x00E0,
	0x0246,
	0x0029,
	0x0FDB,
	0x8A42,
	0x0FDD,
	0x3046,
	0xBDE8,
	0xF041,
	0x00F0,
	0x28B9,
	0x002A,
	0x0CD0,
	0x2748,
	0xD0F8,
	0x8800,
	0x25B1,
	0x0028,
	0xEDDA,
	0xEAE7,
	0x1946,
	0xEDE7,
	0x00F0,
	0x20F9,
	0xE060,
	0x0120,
	0x3DE6,
	0x2DE9,
	0xF047,
	0x8146,
	0x0F46,
	0x0846,
	0x00F0,
	0x1BF9,
	0x1B4C,
	0x0026,
	0x608A,
	0x10B1,
	0x00F0,
	0x1AF9,
	0x6682,
	0x194D,
	0x2888,
	0x0128,
	0x60D1,
	0xA08B,
	0x0028,
	0x5DD1,
	0x002F,
	0x5BD1,
	0x104F,
	0x3868,
	0xB0F8,
	0x1403,
	0x38B1,
	0x2889,
	0x401C,
	0x80B2,
	0x2881,
	0xFF28,
	0x01D9,
	0xA08C,
	0x2881,
	0x0F48,
	0xEE60,
	0xB0F8,
	0x5E80,
	0x1BE0,
	0x2000,
	0x4580,
	0x2000,
	0x2E50,
	0x2000,
	0x6200,
	0x4000,
	0x9404,
	0x2000,
	0x38E0,
	0x4000,
	0xD000,
	0x4000,
	0xA410,
	0x2000,
	0x2C66,
	0x2000,
	0x0890,
	0x2000,
	0x3620,
	0x2000,
	0x0DE0,
	0x2000,
	0x2BC0,
	0x2000,
	0x3580,
	0x4000,
	0x7000,
	0x40F2,
	0xFF31,
	0x0B20,
	0x00F0,
	0xE2F8,
	0x3868,
	0xB0F8,
	0x1213,
	0x19B1,
	0x4846,
	0x00F0,
	0xC7F8,
	0x0AE0,
	0xB0F8,
	0x1403,
	0xC0B1,
	0x2889,
	0xB4F9,
	0x2410,
	0x8842,
	0x13DB,
	0x4846,
	0xFFF7,
	0x5AFF,
	0x78B1,
	0x2E81,
	0x00F0,
	0xD0F8,
	0xE868,
	0x2861,
	0x208E,
	0x18B1,
	0x608E,
	0x18B9,
	0x00F0,
	0xCDF8,
	0x608E,
	0x10B1,
	0xE889,
	0x2887,
	0x6686,
	0x4046,
	0xBDE8,
	0xF047,
	0x00F0,
	0xC8B8,
	0xBDE8,
	0xF087,
	0x10B5,
	0x6021,
	0x0B20,
	0x00F0,
	0xC6F8,
	0x8106,
	0x4FEA,
	0x4061,
	0x05D5,
	0x0029,
	0x0BDA,
	0xBDE8,
	0x1040,
	0x00F0,
	0xC1B8,
	0x0029,
	0x03DA,
	0xBDE8,
	0x1040,
	0x00F0,
	0xC0B8,
	0x8006,
	0x03D5,
	0xBDE8,
	0x1040,
	0x00F0,
	0xBFB8,
	0x10BD,
	0x70B5,
	0x1E4C,
	0x0020,
	0x2080,
	0xAFF2,
	0xDF40,
	0x1C4D,
	0x2861,
	0xAFF2,
	0xD940,
	0x0022,
	0xAFF2,
	0xB941,
	0xA861,
	0x1948,
	0x00F0,
	0xB2F8,
	0x0022,
	0xAFF2,
	0xD531,
	0x6060,
	0x1748,
	0x00F0,
	0xABF8,
	0x0022,
	0xAFF2,
	0x1341,
	0xA060,
	0x1448,
	0x00F0,
	0xA4F8,
	0x0022,
	0xAFF2,
	0xED21,
	0xE060,
	0x1248,
	0x00F0,
	0x9DF8,
	0x2061,
	0xAFF2,
	0x4920,
	0x0022,
	0xAFF2,
	0x0B21,
	0xE863,
	0x0E48,
	0x00F0,
	0x93F8,
	0x0022,
	0xAFF2,
	0x8511,
	0x0C48,
	0x00F0,
	0x8DF8,
	0x0022,
	0xAFF2,
	0xA701,
	0xBDE8,
	0x7040,
	0x0948,
	0x00F0,
	0x85B8,
	0x2000,
	0x4580,
	0x2000,
	0x0840,
	0x0001,
	0x020D,
	0x0000,
	0x67CD,
	0x0000,
	0x3AE1,
	0x0000,
	0x72B1,
	0x0000,
	0x56D7,
	0x0000,
	0x5735,
	0x0000,
	0x0631,
	0x45F6,
	0x250C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F6,
	0xF31C,
	0xC0F2,
	0x000C,
	0x6047,
	0x4AF2,
	0xD74C,
	0xC0F2,
	0x000C,
	0x6047,
	0x40F2,
	0x0D2C,
	0xC0F2,
	0x010C,
	0x6047,
	0x46F2,
	0xCD7C,
	0xC0F2,
	0x000C,
	0x6047,
	0x47F2,
	0xB12C,
	0xC0F2,
	0x000C,
	0x6047,
	0x47F2,
	0x4F2C,
	0xC0F2,
	0x000C,
	0x6047,
	0x47F6,
	0x017C,
	0xC0F2,
	0x000C,
	0x6047,
	0x47F6,
	0x636C,
	0xC0F2,
	0x000C,
	0x6047,
	0x47F2,
	0x0D0C,
	0xC0F2,
	0x000C,
	0x6047,
	0x4AF2,
	0x5F4C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F2,
	0xA56C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F2,
	0x1F5C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F2,
	0x7F5C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F2,
	0x312C,
	0xC0F2,
	0x000C,
	0x6047,
	0x40F2,
	0xAB2C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F2,
	0xF34C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F2,
	0x395C,
	0xC0F2,
	0x000C,
	0x6047,
	0x40F2,
	0x117C,
	0xC0F2,
	0x000C,
	0x6047,
	0x40F2,
	0xD92C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F2,
	0x054C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F2,
	0xAF3C,
	0xC0F2,
	0x000C,
	0x6047,
	0x45F2,
	0x4B2C,
	0xC0F2,
	0x000C,
	0x6047,
	0x4AF6,
	0xE75C,
	0xC0F2,
	0x000C,
	0x6047,
	0x0000,
	0x0000,
	0x0000,
	0x0000,
	0x0000,
	0x0000,
	0x30D5,
	0x0103,
	0x0000,
	0x005E,
};

static kal_uint16 sensor_init_setting_array2[] = {
	0x6028, 0x2000,
	0x602A, 0x3EAC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0549,
	0x6F12, 0x0448,
	0x6F12, 0x054A,
	0x6F12, 0xC1F8,
	0x6F12, 0xC804,
	0x6F12, 0x101A,
	0x6F12, 0xA1F8,
	0x6F12, 0xCC04,
	0x6F12, 0x00F0,
	0x6F12, 0x70BA,
	0x6F12, 0x2000,
	0x6F12, 0x4594,
	0x6F12, 0x2000,
	0x6F12, 0x2E50,
	0x6F12, 0x2000,
	0x6F12, 0x7000,
	0x6F12, 0x10B5,
	0x6F12, 0x00F0,
	0x6F12, 0xB7FA,
	0x6F12, 0xFF49,
	0x6F12, 0x0120,
	0x6F12, 0x0880,
	0x6F12, 0x10BD,
	0x6F12, 0x2DE9,
	0x6F12, 0xF041,
	0x6F12, 0xFD4C,
	0x6F12, 0xFB4F,
	0x6F12, 0x0026,
	0x6F12, 0xB4F8,
	0x6F12, 0x6A52,
	0x6F12, 0x3888,
	0x6F12, 0x08B1,
	0x6F12, 0xA4F8,
	0x6F12, 0x6A62,
	0x6F12, 0x00F0,
	0x6F12, 0xABFA,
	0x6F12, 0x3E80,
	0x6F12, 0xA4F8,
	0x6F12, 0x6A52,
	0x6F12, 0xBDE8,
	0x6F12, 0xF081,
	0x6F12, 0x2DE9,
	0x6F12, 0xF041,
	0x6F12, 0x0746,
	0x6F12, 0xF248,
	0x6F12, 0x0E46,
	0x6F12, 0x0022,
	0x6F12, 0x4068,
	0x6F12, 0x84B2,
	0x6F12, 0x050C,
	0x6F12, 0x2146,
	0x6F12, 0x2846,
	0x6F12, 0x00F0,
	0x6F12, 0x9EFA,
	0x6F12, 0x3146,
	0x6F12, 0x3846,
	0x6F12, 0x00F0,
	0x6F12, 0x9FFA,
	0x6F12, 0xED4F,
	0x6F12, 0x4DF2,
	0x6F12, 0x0C26,
	0x6F12, 0x4FF4,
	0x6F12, 0x8061,
	0x6F12, 0x3A78,
	0x6F12, 0x3046,
	0x6F12, 0x00F0,
	0x6F12, 0x91FA,
	0x6F12, 0x7878,
	0x6F12, 0xB8B3,
	0x6F12, 0x0022,
	0x6F12, 0x8021,
	0x6F12, 0x3046,
	0x6F12, 0x00F0,
	0x6F12, 0x8AFA,
	0x6F12, 0xE648,
	0x6F12, 0x0088,
	0x6F12, 0xE64B,
	0x6F12, 0xA3F8,
	0x6F12, 0x5C02,
	0x6F12, 0xE448,
	0x6F12, 0x001D,
	0x6F12, 0x0088,
	0x6F12, 0xA3F8,
	0x6F12, 0x5E02,
	0x6F12, 0xB3F8,
	0x6F12, 0x5C02,
	0x6F12, 0xB3F8,
	0x6F12, 0x5E12,
	0x6F12, 0x4218,
	0x6F12, 0x02D0,
	0x6F12, 0x8002,
	0x6F12, 0xB0FB,
	0x6F12, 0xF2F2,
	0x6F12, 0x91B2,
	0x6F12, 0xDE4A,
	0x6F12, 0xA3F8,
	0x6F12, 0x6012,
	0x6F12, 0xB2F8,
	0x6F12, 0x1602,
	0x6F12, 0xB2F8,
	0x6F12, 0x1422,
	0x6F12, 0xA3F8,
	0x6F12, 0x9805,
	0x6F12, 0xA3F8,
	0x6F12, 0x9A25,
	0x6F12, 0x8018,
	0x6F12, 0x04D0,
	0x6F12, 0x9202,
	0x6F12, 0xB2FB,
	0x6F12, 0xF0F0,
	0x6F12, 0xA3F8,
	0x6F12, 0x9C05,
	0x6F12, 0xB3F8,
	0x6F12, 0x9C05,
	0x6F12, 0x0A18,
	0x6F12, 0x01FB,
	0x6F12, 0x1020,
	0x6F12, 0x40F3,
	0x6F12, 0x9510,
	0x6F12, 0x1028,
	0x6F12, 0x06DC,
	0x6F12, 0x0028,
	0x6F12, 0x05DA,
	0x6F12, 0x0020,
	0x6F12, 0x03E0,
	0x6F12, 0xFFE7,
	0x6F12, 0x0122,
	0x6F12, 0xC5E7,
	0x6F12, 0x1020,
	0x6F12, 0xCE49,
	0x6F12, 0x0880,
	0x6F12, 0x2146,
	0x6F12, 0x2846,
	0x6F12, 0xBDE8,
	0x6F12, 0xF041,
	0x6F12, 0x0122,
	0x6F12, 0x00F0,
	0x6F12, 0x4ABA,
	0x6F12, 0xF0B5,
	0x6F12, 0xCA4C,
	0x6F12, 0xDDE9,
	0x6F12, 0x0565,
	0x6F12, 0x08B1,
	0x6F12, 0x2788,
	0x6F12, 0x0760,
	0x6F12, 0x09B1,
	0x6F12, 0x6088,
	0x6F12, 0x0860,
	0x6F12, 0x12B1,
	0x6F12, 0xA088,
	0x6F12, 0x401C,
	0x6F12, 0x1060,
	0x6F12, 0x0BB1,
	0x6F12, 0xE088,
	0x6F12, 0x1860,
	0x6F12, 0x0EB1,
	0x6F12, 0xA07B,
	0x6F12, 0x3060,
	0x6F12, 0x002D,
	0x6F12, 0x01D0,
	0x6F12, 0xE07B,
	0x6F12, 0x2860,
	0x6F12, 0xF0BD,
	0x6F12, 0x70B5,
	0x6F12, 0x0646,
	0x6F12, 0xB648,
	0x6F12, 0x0022,
	0x6F12, 0x8068,
	0x6F12, 0x84B2,
	0x6F12, 0x050C,
	0x6F12, 0x2146,
	0x6F12, 0x2846,
	0x6F12, 0x00F0,
	0x6F12, 0x26FA,
	0x6F12, 0x3046,
	0x6F12, 0x00F0,
	0x6F12, 0x2DFA,
	0x6F12, 0xB848,
	0x6F12, 0x0368,
	0x6F12, 0xB3F8,
	0x6F12, 0x7401,
	0x6F12, 0x010A,
	0x6F12, 0xB648,
	0x6F12, 0x4268,
	0x6F12, 0x82F8,
	0x6F12, 0x5010,
	0x6F12, 0x93F8,
	0x6F12, 0x7511,
	0x6F12, 0x82F8,
	0x6F12, 0x5210,
	0x6F12, 0xB3F8,
	0x6F12, 0x7811,
	0x6F12, 0x090A,
	0x6F12, 0x82F8,
	0x6F12, 0x5810,
	0x6F12, 0x93F8,
	0x6F12, 0x7911,
	0x6F12, 0x82F8,
	0x6F12, 0x5A10,
	0x6F12, 0x33F8,
	0x6F12, 0xF01F,
	0x6F12, 0x0068,
	0x6F12, 0x090A,
	0x6F12, 0x00F8,
	0x6F12, 0xCE1F,
	0x6F12, 0x5978,
	0x6F12, 0x8170,
	0x6F12, 0x5988,
	0x6F12, 0x090A,
	0x6F12, 0x0171,
	0x6F12, 0xD978,
	0x6F12, 0x8171,
	0x6F12, 0x988C,
	0x6F12, 0x000A,
	0x6F12, 0x9074,
	0x6F12, 0x93F8,
	0x6F12, 0x2500,
	0x6F12, 0x1075,
	0x6F12, 0xD88C,
	0x6F12, 0x000A,
	0x6F12, 0x9075,
	0x6F12, 0x93F8,
	0x6F12, 0x2700,
	0x6F12, 0x1076,
	0x6F12, 0xB3F8,
	0x6F12, 0xB000,
	0x6F12, 0x000A,
	0x6F12, 0x82F8,
	0x6F12, 0x7E00,
	0x6F12, 0x93F8,
	0x6F12, 0xB100,
	0x6F12, 0x82F8,
	0x6F12, 0x8000,
	0x6F12, 0x9548,
	0x6F12, 0x90F8,
	0x6F12, 0xB313,
	0x6F12, 0x82F8,
	0x6F12, 0x8210,
	0x6F12, 0x90F8,
	0x6F12, 0xB103,
	0x6F12, 0x82F8,
	0x6F12, 0x8400,
	0x6F12, 0x93F8,
	0x6F12, 0xB400,
	0x6F12, 0x82F8,
	0x6F12, 0x8600,
	0x6F12, 0x0020,
	0x6F12, 0x82F8,
	0x6F12, 0x8800,
	0x6F12, 0x93F8,
	0x6F12, 0x6211,
	0x6F12, 0x82F8,
	0x6F12, 0x9610,
	0x6F12, 0x93F8,
	0x6F12, 0x0112,
	0x6F12, 0x82F8,
	0x6F12, 0x9E10,
	0x6F12, 0x93F8,
	0x6F12, 0x0212,
	0x6F12, 0x82F8,
	0x6F12, 0xA010,
	0x6F12, 0x82F8,
	0x6F12, 0xA200,
	0x6F12, 0x82F8,
	0x6F12, 0xA400,
	0x6F12, 0x93F8,
	0x6F12, 0x0512,
	0x6F12, 0x82F8,
	0x6F12, 0xA610,
	0x6F12, 0x93F8,
	0x6F12, 0x0612,
	0x6F12, 0x82F8,
	0x6F12, 0xA810,
	0x6F12, 0x93F8,
	0x6F12, 0x0712,
	0x6F12, 0x82F8,
	0x6F12, 0xAA10,
	0x6F12, 0x82F8,
	0x6F12, 0xAC00,
	0x6F12, 0x5A20,
	0x6F12, 0x82F8,
	0x6F12, 0xAD00,
	0x6F12, 0x93F8,
	0x6F12, 0x0902,
	0x6F12, 0x82F8,
	0x6F12, 0xAE00,
	0x6F12, 0x2146,
	0x6F12, 0x2846,
	0x6F12, 0xBDE8,
	0x6F12, 0x7040,
	0x6F12, 0x0122,
	0x6F12, 0x00F0,
	0x6F12, 0xAFB9,
	0x6F12, 0x70B5,
	0x6F12, 0x7548,
	0x6F12, 0x0022,
	0x6F12, 0x0169,
	0x6F12, 0x0C0C,
	0x6F12, 0x8DB2,
	0x6F12, 0x2946,
	0x6F12, 0x2046,
	0x6F12, 0x00F0,
	0x6F12, 0xA5F9,
	0x6F12, 0x00F0,
	0x6F12, 0xB2F9,
	0x6F12, 0x7248,
	0x6F12, 0x8078,
	0x6F12, 0x08B1,
	0x6F12, 0x4F22,
	0x6F12, 0x00E0,
	0x6F12, 0x2522,
	0x6F12, 0x7748,
	0x6F12, 0x90F8,
	0x6F12, 0xE400,
	0x6F12, 0x0328,
	0x6F12, 0x07D1,
	0x6F12, 0x42F0,
	0x6F12, 0x8002,
	0x6F12, 0x4FF4,
	0x6F12, 0x8361,
	0x6F12, 0x48F6,
	0x6F12, 0x7A20,
	0x6F12, 0x00F0,
	0x6F12, 0xA4F9,
	0x6F12, 0x2946,
	0x6F12, 0x2046,
	0x6F12, 0xBDE8,
	0x6F12, 0x7040,
	0x6F12, 0x0122,
	0x6F12, 0x00F0,
	0x6F12, 0x89B9,
	0x6F12, 0x10B5,
	0x6F12, 0x0221,
	0x6F12, 0x7620,
	0x6F12, 0x00F0,
	0x6F12, 0x9DF9,
	0x6F12, 0x0221,
	0x6F12, 0x4420,
	0x6F12, 0x00F0,
	0x6F12, 0x99F9,
	0x6F12, 0x4021,
	0x6F12, 0x4520,
	0x6F12, 0x00F0,
	0x6F12, 0x95F9,
	0x6F12, 0x5D49,
	0x6F12, 0x0420,
	0x6F12, 0xA1F8,
	0x6F12, 0x3A06,
	0x6F12, 0x10BD,
	0x6F12, 0x7047,
	0x6F12, 0x7047,
	0x6F12, 0x08B5,
	0x6F12, 0x5949,
	0x6F12, 0x3120,
	0x6F12, 0x6A46,
	0x6F12, 0x81F8,
	0x6F12, 0x4306,
	0x6F12, 0x5A20,
	0x6F12, 0x8DF8,
	0x6F12, 0x0000,
	0x6F12, 0x0121,
	0x6F12, 0x7520,
	0x6F12, 0x00F0,
	0x6F12, 0x86F9,
	0x6F12, 0x9DF8,
	0x6F12, 0x0000,
	0x6F12, 0x08BD,
	0x6F12, 0x7047,
	0x6F12, 0x5248,
	0x6F12, 0x10B5,
	0x6F12, 0x8078,
	0x6F12, 0x18B1,
	0x6F12, 0x0021,
	0x6F12, 0x4420,
	0x6F12, 0x00F0,
	0x6F12, 0x75F9,
	0x6F12, 0x4D49,
	0x6F12, 0x0220,
	0x6F12, 0xA1F8,
	0x6F12, 0x3A06,
	0x6F12, 0x10BD,
	0x6F12, 0x5448,
	0x6F12, 0x90F8,
	0x6F12, 0xE400,
	0x6F12, 0x0328,
	0x6F12, 0x01D0,
	0x6F12, 0x00F0,
	0x6F12, 0x73B9,
	0x6F12, 0xAFF2,
	0x6F12, 0x2B01,
	0x6F12, 0x4648,
	0x6F12, 0xC0F8,
	0x6F12, 0x4C16,
	0x6F12, 0x4649,
	0x6F12, 0x8978,
	0x6F12, 0x11B1,
	0x6F12, 0xAFF2,
	0x6F12, 0x8501,
	0x6F12, 0x01E0,
	0x6F12, 0xAFF2,
	0x6F12, 0x6501,
	0x6F12, 0xC0F8,
	0x6F12, 0x4816,
	0x6F12, 0xAFF2,
	0x6F12, 0x6B01,
	0x6F12, 0xC0F8,
	0x6F12, 0x4416,
	0x6F12, 0xAFF2,
	0x6F12, 0x7101,
	0x6F12, 0xC0F8,
	0x6F12, 0x5016,
	0x6F12, 0xAFF2,
	0x6F12, 0x5901,
	0x6F12, 0xC0F8,
	0x6F12, 0x5416,
	0x6F12, 0x7047,
	0x6F12, 0x2DE9,
	0x6F12, 0xF041,
	0x6F12, 0x434C,
	0x6F12, 0x4249,
	0x6F12, 0x0646,
	0x6F12, 0xB4F8,
	0x6F12, 0x6670,
	0x6F12, 0xC989,
	0x6F12, 0xB4F8,
	0x6F12, 0x7E20,
	0x6F12, 0x0020,
	0x6F12, 0xC1B1,
	0x6F12, 0x2146,
	0x6F12, 0xD1F8,
	0x6F12, 0x9010,
	0x6F12, 0x72B1,
	0x6F12, 0x8FB1,
	0x6F12, 0x0846,
	0x6F12, 0x00F0,
	0x6F12, 0x48F9,
	0x6F12, 0x0546,
	0x6F12, 0xA06F,
	0x6F12, 0x00F0,
	0x6F12, 0x44F9,
	0x6F12, 0x8542,
	0x6F12, 0x02D2,
	0x6F12, 0xD4F8,
	0x6F12, 0x9000,
	0x6F12, 0x26E0,
	0x6F12, 0xA06F,
	0x6F12, 0x24E0,
	0x6F12, 0x002F,
	0x6F12, 0xFBD1,
	0x6F12, 0x002A,
	0x6F12, 0x24D0,
	0x6F12, 0x0846,
	0x6F12, 0x1EE0,
	0x6F12, 0x2849,
	0x6F12, 0x8D88,
	0x6F12, 0x8968,
	0x6F12, 0x4B42,
	0x6F12, 0x77B1,
	0x6F12, 0x2F48,
	0x6F12, 0x406F,
	0x6F12, 0x10E0,
	0x6F12, 0x4242,
	0x6F12, 0x00E0,
	0x6F12, 0x0246,
	0x6F12, 0x0029,
	0x6F12, 0x0FDB,
	0x6F12, 0x8A42,
	0x6F12, 0x0FDD,
	0x6F12, 0x3046,
	0x6F12, 0xBDE8,
	0x6F12, 0xF041,
	0x6F12, 0x00F0,
	0x6F12, 0x28B9,
	0x6F12, 0x002A,
	0x6F12, 0x0CD0,
	0x6F12, 0x2748,
	0x6F12, 0xD0F8,
	0x6F12, 0x8800,
	0x6F12, 0x25B1,
	0x6F12, 0x0028,
	0x6F12, 0xEDDA,
	0x6F12, 0xEAE7,
	0x6F12, 0x1946,
	0x6F12, 0xEDE7,
	0x6F12, 0x00F0,
	0x6F12, 0x20F9,
	0x6F12, 0xE060,
	0x6F12, 0x0120,
	0x6F12, 0x3DE6,
	0x6F12, 0x2DE9,
	0x6F12, 0xF047,
	0x6F12, 0x8146,
	0x6F12, 0x0F46,
	0x6F12, 0x0846,
	0x6F12, 0x00F0,
	0x6F12, 0x1BF9,
	0x6F12, 0x1B4C,
	0x6F12, 0x0026,
	0x6F12, 0x608A,
	0x6F12, 0x10B1,
	0x6F12, 0x00F0,
	0x6F12, 0x1AF9,
	0x6F12, 0x6682,
	0x6F12, 0x194D,
	0x6F12, 0x2888,
	0x6F12, 0x0128,
	0x6F12, 0x60D1,
	0x6F12, 0xA08B,
	0x6F12, 0x0028,
	0x6F12, 0x5DD1,
	0x6F12, 0x002F,
	0x6F12, 0x5BD1,
	0x6F12, 0x104F,
	0x6F12, 0x3868,
	0x6F12, 0xB0F8,
	0x6F12, 0x1403,
	0x6F12, 0x38B1,
	0x6F12, 0x2889,
	0x6F12, 0x401C,
	0x6F12, 0x80B2,
	0x6F12, 0x2881,
	0x6F12, 0xFF28,
	0x6F12, 0x01D9,
	0x6F12, 0xA08C,
	0x6F12, 0x2881,
	0x6F12, 0x0F48,
	0x6F12, 0xEE60,
	0x6F12, 0xB0F8,
	0x6F12, 0x5E80,
	0x6F12, 0x1BE0,
	0x6F12, 0x2000,
	0x6F12, 0x4580,
	0x6F12, 0x2000,
	0x6F12, 0x2E50,
	0x6F12, 0x2000,
	0x6F12, 0x6200,
	0x6F12, 0x4000,
	0x6F12, 0x9404,
	0x6F12, 0x2000,
	0x6F12, 0x38E0,
	0x6F12, 0x4000,
	0x6F12, 0xD000,
	0x6F12, 0x4000,
	0x6F12, 0xA410,
	0x6F12, 0x2000,
	0x6F12, 0x2C66,
	0x6F12, 0x2000,
	0x6F12, 0x0890,
	0x6F12, 0x2000,
	0x6F12, 0x3620,
	0x6F12, 0x2000,
	0x6F12, 0x0DE0,
	0x6F12, 0x2000,
	0x6F12, 0x2BC0,
	0x6F12, 0x2000,
	0x6F12, 0x3580,
	0x6F12, 0x4000,
	0x6F12, 0x7000,
	0x6F12, 0x40F2,
	0x6F12, 0xFF31,
	0x6F12, 0x0B20,
	0x6F12, 0x00F0,
	0x6F12, 0xE2F8,
	0x6F12, 0x3868,
	0x6F12, 0xB0F8,
	0x6F12, 0x1213,
	0x6F12, 0x19B1,
	0x6F12, 0x4846,
	0x6F12, 0x00F0,
	0x6F12, 0xC7F8,
	0x6F12, 0x0AE0,
	0x6F12, 0xB0F8,
	0x6F12, 0x1403,
	0x6F12, 0xC0B1,
	0x6F12, 0x2889,
	0x6F12, 0xB4F9,
	0x6F12, 0x2410,
	0x6F12, 0x8842,
	0x6F12, 0x13DB,
	0x6F12, 0x4846,
	0x6F12, 0xFFF7,
	0x6F12, 0x5AFF,
	0x6F12, 0x78B1,
	0x6F12, 0x2E81,
	0x6F12, 0x00F0,
	0x6F12, 0xD0F8,
	0x6F12, 0xE868,
	0x6F12, 0x2861,
	0x6F12, 0x208E,
	0x6F12, 0x18B1,
	0x6F12, 0x608E,
	0x6F12, 0x18B9,
	0x6F12, 0x00F0,
	0x6F12, 0xCDF8,
	0x6F12, 0x608E,
	0x6F12, 0x10B1,
	0x6F12, 0xE889,
	0x6F12, 0x2887,
	0x6F12, 0x6686,
	0x6F12, 0x4046,
	0x6F12, 0xBDE8,
	0x6F12, 0xF047,
	0x6F12, 0x00F0,
	0x6F12, 0xC8B8,
	0x6F12, 0xBDE8,
	0x6F12, 0xF087,
	0x6F12, 0x10B5,
	0x6F12, 0x6021,
	0x6F12, 0x0B20,
	0x6F12, 0x00F0,
	0x6F12, 0xC6F8,
	0x6F12, 0x8106,
	0x6F12, 0x4FEA,
	0x6F12, 0x4061,
	0x6F12, 0x05D5,
	0x6F12, 0x0029,
	0x6F12, 0x0BDA,
	0x6F12, 0xBDE8,
	0x6F12, 0x1040,
	0x6F12, 0x00F0,
	0x6F12, 0xC1B8,
	0x6F12, 0x0029,
	0x6F12, 0x03DA,
	0x6F12, 0xBDE8,
	0x6F12, 0x1040,
	0x6F12, 0x00F0,
	0x6F12, 0xC0B8,
	0x6F12, 0x8006,
	0x6F12, 0x03D5,
	0x6F12, 0xBDE8,
	0x6F12, 0x1040,
	0x6F12, 0x00F0,
	0x6F12, 0xBFB8,
	0x6F12, 0x10BD,
	0x6F12, 0x70B5,
	0x6F12, 0x1E4C,
	0x6F12, 0x0020,
	0x6F12, 0x2080,
	0x6F12, 0xAFF2,
	0x6F12, 0xDF40,
	0x6F12, 0x1C4D,
	0x6F12, 0x2861,
	0x6F12, 0xAFF2,
	0x6F12, 0xD940,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0xB941,
	0x6F12, 0xA861,
	0x6F12, 0x1948,
	0x6F12, 0x00F0,
	0x6F12, 0xB2F8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0xD531,
	0x6F12, 0x6060,
	0x6F12, 0x1748,
	0x6F12, 0x00F0,
	0x6F12, 0xABF8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0x1341,
	0x6F12, 0xA060,
	0x6F12, 0x1448,
	0x6F12, 0x00F0,
	0x6F12, 0xA4F8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0xED21,
	0x6F12, 0xE060,
	0x6F12, 0x1248,
	0x6F12, 0x00F0,
	0x6F12, 0x9DF8,
	0x6F12, 0x2061,
	0x6F12, 0xAFF2,
	0x6F12, 0x4920,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0x0B21,
	0x6F12, 0xE863,
	0x6F12, 0x0E48,
	0x6F12, 0x00F0,
	0x6F12, 0x93F8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0x8511,
	0x6F12, 0x0C48,
	0x6F12, 0x00F0,
	0x6F12, 0x8DF8,
	0x6F12, 0x0022,
	0x6F12, 0xAFF2,
	0x6F12, 0xA701,
	0x6F12, 0xBDE8,
	0x6F12, 0x7040,
	0x6F12, 0x0948,
	0x6F12, 0x00F0,
	0x6F12, 0x85B8,
	0x6F12, 0x2000,
	0x6F12, 0x4580,
	0x6F12, 0x2000,
	0x6F12, 0x0840,
	0x6F12, 0x0001,
	0x6F12, 0x020D,
	0x6F12, 0x0000,
	0x6F12, 0x67CD,
	0x6F12, 0x0000,
	0x6F12, 0x3AE1,
	0x6F12, 0x0000,
	0x6F12, 0x72B1,
	0x6F12, 0x0000,
	0x6F12, 0x56D7,
	0x6F12, 0x0000,
	0x6F12, 0x5735,
	0x6F12, 0x0000,
	0x6F12, 0x0631,
	0x6F12, 0x45F6,
	0x6F12, 0x250C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F6,
	0x6F12, 0xF31C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x4AF2,
	0x6F12, 0xD74C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x40F2,
	0x6F12, 0x0D2C,
	0x6F12, 0xC0F2,
	0x6F12, 0x010C,
	0x6F12, 0x6047,
	0x6F12, 0x46F2,
	0x6F12, 0xCD7C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F2,
	0x6F12, 0xB12C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F2,
	0x6F12, 0x4F2C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F6,
	0x6F12, 0x017C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F6,
	0x6F12, 0x636C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x47F2,
	0x6F12, 0x0D0C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x4AF2,
	0x6F12, 0x5F4C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0xA56C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x1F5C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x7F5C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x312C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x40F2,
	0x6F12, 0xAB2C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0xF34C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x395C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x40F2,
	0x6F12, 0x117C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x40F2,
	0x6F12, 0xD92C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x054C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0xAF3C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x45F2,
	0x6F12, 0x4B2C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x4AF6,
	0x6F12, 0xE75C,
	0x6F12, 0xC0F2,
	0x6F12, 0x000C,
	0x6F12, 0x6047,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x30D5,
	0x6F12, 0x0103,
	0x6F12, 0x0000,
	0x6F12, 0x005E,
	0x602A, 0x1662,
	0x6F12, 0x1E00,
	0x602A, 0x1C9A,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0FF2,
	0x6F12, 0x0020,
	0x602A, 0x0EF6,
	0x6F12, 0x0100,
	0x602A, 0x23B2,
	0x6F12, 0x0001,
	0x602A, 0x0FE4,
	0x6F12, 0x0107,
	0x6F12, 0x07D0,
	0x602A, 0x12F8,
	0x6F12, 0x3D09,
	0x602A, 0x0E18,
	0x6F12, 0x0040,
	0x602A, 0x1066,
	0x6F12, 0x000C,
	0x602A, 0x13DE,
	0x6F12, 0x0000,
	0x602A, 0x12F2,
	0x6F12, 0x0F0F,
	0x602A, 0x13DC,
	0x6F12, 0x806F,
	0xF46E, 0x00C3,
	0xF46C, 0xBFA0,
	0xF44A, 0x0007,
	0xF456, 0x000A,
	0x6028, 0x2000,
	0x602A, 0x12F6,
	0x6F12, 0x7008,
	0x0BC6, 0x0000,
	0x0B36, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x2BC2,
	0x6F12, 0x0020,
	0x602A, 0x2BC4,
	0x6F12, 0x0020,
	0x602A, 0x6204,
	0x6F12, 0x0001,
	0x602A, 0x6208,
	0x6F12, 0x0000,
	0x6F12, 0x0030,
	0x6028, 0x2000,
	0x602A, 0x17C0,
	0x6F12, 0x143C,
};

static kal_uint16 preview_setting_array[] = {
	0x6214, 0x7971,
	0x6218, 0x7150,
	0x0344, 0x0070,
	0x0346, 0x0044,
	0x0348, 0x100F,
	0x034A, 0x0BFB,
	0x034C, 0x0FA0,
	0x034E, 0x0BB8,
	0x0340, 0x0CF0,
	0x0342, 0x12F0,
	0x0900, 0x0011,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0001,
	0x0402, 0x1010,
	0x0404, 0x1000,
	0x0350, 0x0000,
	0x0352, 0x0000,
	0x0136, 0x1800,
	0x013E, 0x0000,
	0x0300, 0x0008,
	0x0302, 0x0001,
	0x0304, 0x0006,
	0x0306, 0x00F1,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005A,
	0x0312, 0x0000,
	0x0B06, 0x0101,
	0x6028, 0x2000,
	0x602A, 0x1FF6,
	0x6F12, 0x0000,
	0x021E, 0x0000,
	0x0202, 0x0100,
	0x0204, 0x0020,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0114, 0x0301,
	0x0D06, 0x01F0,
	0x0D08, 0x02E0,
	0x6028, 0x2000,
	0x602A, 0x0F10,
	0x6F12, 0x0003,
	0x602A, 0x0F12,
	0x6F12, 0x0200,
	0x602A, 0x2BC0,
	0x6F12, 0x0001,
	0x0B30, 0x0000,
	0x0B32, 0x0000,
	0x0B34, 0x0001,
	0x0804, 0x0200,
	0x0810, 0x0011,
	0x6028, 0x4000,
	0x0804, 0x0200,
	0x0810, 0x001e,
};

static kal_uint16 capture_setting_array[] = {
	0x6214, 0x7971,
	0x6218, 0x7150,
	0x0344, 0x0000,
	0x0346, 0x0008,
	0x0348, 0x1077,
	0x034A, 0x0C37,
	0x034C, 0x1070,
	0x034E, 0x0C30,
	0x0340, 0x0CF2,
	0x0342, 0x12F0,
	0x0900, 0x0011,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0001,
	0x0402, 0x1010,
	0x0404, 0x1000,
	0x0350, 0x0008,
	0x0352, 0x0000,
	0x0136, 0x1800,
	0x013E, 0x0000,
	0x0300, 0x0008,
	0x0302, 0x0001,
	0x0304, 0x0006,
	0x0306, 0x00F1,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005A,
	0x0312, 0x0000,
	0x0B06, 0x0101,
	0x6028, 0x2000,
	0x602A, 0x1FF6,
	0x6F12, 0x0000,
	0x021E, 0x0000,
	0x0202, 0x0100,
	0x0204, 0x0020,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0114, 0x0301,
	0x0D06, 0x0208,
	0x0D08, 0x0300,
	0x6028, 0x2000,
	0x602A, 0x0F10,
	0x6F12, 0x0003,
	0x602A, 0x0F12,
	0x6F12, 0x0200,
	0x602A, 0x2BC0,
	0x6F12, 0x0001,
	0x0B30, 0x0000,
	0x0B32, 0x0000,
	0x0B34, 0x0001,
	0x0804, 0x0200,
	0x0810, 0x0020,
	0x6028, 0x4000,
	0x0804, 0x0200,
	0x0810, 0x001e,
};

static kal_uint16 normal_video_setting_array[] = {
	0x6214, 0x7971,
	0x6218, 0x7150,
	0x0344, 0x0000,
	0x0346, 0x0180,
	0x0348, 0x1077,
	0x034A, 0x0ABF,
	0x034C, 0x1070,
	0x034E, 0x0940,
	0x0340, 0x0CF2,
	0x0342, 0x12F0,
	0x0900, 0x0011,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0001,
	0x0402, 0x1010,
	0x0404, 0x1000,
	0x0350, 0x0008,
	0x0352, 0x0000,
	0x0136, 0x1800,
	0x013E, 0x0000,
	0x0300, 0x0008,
	0x0302, 0x0001,
	0x0304, 0x0006,
	0x0306, 0x00F1,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005A,
	0x0312, 0x0000,
	0x0B06, 0x0101,
	0x6028, 0x2000,
	0x602A, 0x1FF6,
	0x6F12, 0x0000,
	0x021E, 0x0000,
	0x0202, 0x0100,
	0x0204, 0x0020,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0114, 0x0301,
	0x0D06, 0x0208,
	0x0D08, 0x0250,
	0x6028, 0x2000,
	0x602A, 0x0F10,
	0x6F12, 0x0003,
	0x602A, 0x0F12,
	0x6F12, 0x0200,
	0x602A, 0x2BC0,
	0x6F12, 0x0001,
	0x0B30, 0x0000,
	0x0B32, 0x0000,
	0x0B34, 0x0001,
	0x6028, 0x4000,
	0x0804, 0x0200,
	0x0810, 0x001e,
};

static kal_uint16 hs_video_setting_array[] = {
	0x6214, 0x7971,
	0x6218, 0x7150,
	0x0344, 0x0000,
	0x0346, 0x0180,
	0x0348, 0x1077,
	0x034A, 0x0ABF,
	0x034C, 0x0838,
	0x034E, 0x04A0,
	0x0340, 0x0678,
	0x0342, 0x12F0,
	0x0900, 0x0112,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0003,
	0x0402, 0x1010,
	0x0404, 0x2000,
	0x0350, 0x0004,
	0x0352, 0x0000,
	0x0136, 0x1800,
	0x013E, 0x0000,
	0x0300, 0x0008,
	0x0302, 0x0001,
	0x0304, 0x0006,
	0x0306, 0x00F1,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005A,
	0x0312, 0x0000,
	0x0B06, 0x0101,
	0x6028, 0x2000,
	0x602A, 0x1FF6,
	0x6F12, 0x0000,
	0x021E, 0x0000,
	0x0202, 0x0100,
	0x0204, 0x0020,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0114, 0x0301,
	0x0D06, 0x0208,
	0x0D08, 0x0250,
	0x6028, 0x2000,
	0x602A, 0x0F10,
	0x6F12, 0x0003,
	0x602A, 0x0F12,
	0x6F12, 0x0200,
	0x602A, 0x2BC0,
	0x6F12, 0x0001,
	0x0B30, 0x0000,
	0x0B32, 0x0000,
	0x0B34, 0x0001,
	0x6028, 0x4000,
	0x0804, 0x0200,
	0x0810, 0x001e,
};

static kal_uint16 slim_video_setting_array[] = {
	0x6214, 0x7971,
	0x6218, 0x7150,
	0x0344, 0x0000,
	0x0346, 0x0180,
	0x0348, 0x1077,
	0x034A, 0x0ABF,
	0x034C, 0x0838,
	0x034E, 0x04A0,
	0x0340, 0x0678,
	0x0342, 0x12F0,
	0x0900, 0x0112,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0003,
	0x0402, 0x1010,
	0x0404, 0x2000,
	0x0350, 0x0004,
	0x0352, 0x0000,
	0x0136, 0x1800,
	0x013E, 0x0000,
	0x0300, 0x0008,
	0x0302, 0x0001,
	0x0304, 0x0006,
	0x0306, 0x00F1,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005A,
	0x0312, 0x0000,
	0x0B06, 0x0101,
	0x6028, 0x2000,
	0x602A, 0x1FF6,
	0x6F12, 0x0000,
	0x021E, 0x0000,
	0x0202, 0x0100,
	0x0204, 0x0020,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0114, 0x0301,
	0x0D06, 0x0208,
	0x0D08, 0x0250,
	0x6028, 0x2000,
	0x602A, 0x0F10,
	0x6F12, 0x0003,
	0x602A, 0x0F12,
	0x6F12, 0x0200,
	0x602A, 0x2BC0,
	0x6F12, 0x0001,
	0x0B30, 0x0000,
	0x0B32, 0x0000,
	0x0B34, 0x0001,
	0x6028, 0x4000,
	0x0804, 0x0200,
	0x0810, 0x001e,
};

static kal_uint16 custom1_setting_array[] = {
	0x6214, 0x7971,
	0x6218, 0x7150,
	0x0344, 0x0000,
	0x0346, 0x0008,
	0x0348, 0x1077,
	0x034A, 0x0C37,
	0x034C, 0x1070,
	0x034E, 0x0C30,
	0x0340, 0x102E,
	0x0342, 0x12F0,
	0x0900, 0x0011,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0001,
	0x0402, 0x1010,
	0x0404, 0x1000,
	0x0350, 0x0008,
	0x0352, 0x0000,
	0x0136, 0x1800,
	0x013E, 0x0000,
	0x0300, 0x0008,
	0x0302, 0x0001,
	0x0304, 0x0006,
	0x0306, 0x00F1,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005A,
	0x0312, 0x0000,
	0x0B06, 0x0101,
	0x6028, 0x2000,
	0x602A, 0x1FF6,
	0x6F12, 0x0000,
	0x021E, 0x0000,
	0x0202, 0x0100,
	0x0204, 0x0020,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0114, 0x0301,
	0x0D06, 0x0208,
	0x0D08, 0x0300,
	0x6028, 0x2000,
	0x602A, 0x0F10,
	0x6F12, 0x0003,
	0x602A, 0x0F12,
	0x6F12, 0x0200,
	0x602A, 0x2BC0,
	0x6F12, 0x0001,
	0x0B30, 0x0000,
	0x0B32, 0x0000,
	0x0B34, 0x0001,
	0x6028, 0x4000,
	0x0804, 0x0200,
	0x0810, 0x001e,
};

static kal_uint16 custom2_setting_array[] = {
	0x6214, 0x7971,
	0x6218, 0x7150,
	0x0344, 0x0000,
	0x0346, 0x0180,
	0x0348, 0x1077,
	0x034A, 0x0ABF,
	0x034C, 0x1070,
	0x034E, 0x0940,
	0x0340, 0x0CF2,
	0x0342, 0x12F0,
	0x0900, 0x0011,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0001,
	0x0402, 0x1010,
	0x0404, 0x1000,
	0x0350, 0x0008,
	0x0352, 0x0000,
	0x0136, 0x1800,
	0x013E, 0x0000,
	0x0300, 0x0008,
	0x0302, 0x0001,
	0x0304, 0x0006,
	0x0306, 0x00F1,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005A,
	0x0312, 0x0000,
	0x0B06, 0x0101,
	0x6028, 0x2000,
	0x602A, 0x1FF6,
	0x6F12, 0x0000,
	0x021E, 0x0000,
	0x0202, 0x0100,
	0x0204, 0x0020,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0114, 0x0301,
	0x0D06, 0x0208,
	0x0D08, 0x0250,
	0x6028, 0x2000,
	0x602A, 0x0F10,
	0x6F12, 0x0003,
	0x602A, 0x0F12,
	0x6F12, 0x0200,
	0x602A, 0x2BC0,
	0x6F12, 0x0001,
	0x0B30, 0x0000,
	0x0B32, 0x0000,
	0x0B34, 0x0001,
	0x6028, 0x4000,
	0x0804, 0x0200,
	0x0810, 0x001e,
};

static kal_uint16 custom3_setting_array[] = {
	0x6214, 0x7971,
	0x6218, 0x7150,
	0x0344, 0x0000,
	0x0346, 0x0008,
	0x0348, 0x1077,
	0x034A, 0x0C37,
	0x034C, 0x1070,
	0x034E, 0x0C30,
	0x0340, 0x0CF2,
	0x0342, 0x12F0,
	0x0900, 0x0011,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0001,
	0x0402, 0x1010,
	0x0404, 0x1000,
	0x0350, 0x0008,
	0x0352, 0x0000,
	0x0136, 0x1800,
	0x013E, 0x0000,
	0x0300, 0x0008,
	0x0302, 0x0001,
	0x0304, 0x0006,
	0x0306, 0x00F1,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005A,
	0x0312, 0x0000,
	0x0B06, 0x0101,
	0x6028, 0x2000,
	0x602A, 0x1FF6,
	0x6F12, 0x0000,
	0x021E, 0x0000,
	0x0202, 0x0100,
	0x0204, 0x0020,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0114, 0x0301,
	0x0D06, 0x0208,
	0x0D08, 0x0300,
	0x6028, 0x2000,
	0x602A, 0x0F10,
	0x6F12, 0x0003,
	0x602A, 0x0F12,
	0x6F12, 0x0200,
	0x602A, 0x2BC0,
	0x6F12, 0x0001,
	0x0B30, 0x0000,
	0x0B32, 0x0000,
	0x0B34, 0x0001,
	0x6028, 0x4000,
	0x0804, 0x0200,
	0x0810, 0x001e,
};

/* VC2 for PDAF */
static struct SENSOR_VC_INFO_STRUCT vc_info_preview = {
	0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	0x00, 0x2b, 0x0fa0, 0x0bb8, /* VC0 */
	0x00, 0x00, 0x0000, 0x0000, /* VC1 */
	0x01, 0x2b, 0x01f0, 0x02e0, /* VC2 */
	0x00, 0x00, 0x0000, 0x0000, /* VC3 */
};

static struct SENSOR_VC_INFO_STRUCT vc_info_capture = {
	0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	0x00, 0x2b, 0x1070, 0x0c30, /* VC0 */
	0x00, 0x00, 0x0000, 0x0000, /* VC1 */
	0x01, 0x2b, 0x0208, 0x0300, /* VC2 */
	0x00, 0x00, 0x0000, 0x0000, /* VC3 */
};

static struct SENSOR_VC_INFO_STRUCT vc_info_video = {
	0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	0x00, 0x2b, 0x1070, 0x0940, /* VC0 */
	0x00, 0x00, 0x0000, 0x0000, /* VC1 */
	0x01, 0x2b, 0x0208, 0x0250, /* VC2 */
	0x00, 0x00, 0x0000, 0x0000, /* VC3 */
};

static struct SENSOR_VC_INFO_STRUCT vc_info_hs_video = {
	0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	0x00, 0x2b, 0x0838, 0x04a0, /* VC0 */
	0x00, 0x00, 0x0000, 0x0000, /* VC1 */
	0x01, 0x2b, 0x0208, 0x0250, /* VC2 */
	0x00, 0x00, 0x0000, 0x0000, /* VC3 */
};

static struct SENSOR_VC_INFO_STRUCT vc_info_slim_video = {
	0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	0x00, 0x2b, 0x0838, 0x04a0, /* VC0 */
	0x00, 0x00, 0x0000, 0x0000, /* VC1 */
	0x01, 0x2b, 0x0208, 0x0250, /* VC2 */
	0x00, 0x00, 0x0000, 0x0000, /* VC3 */
};

static struct SENSOR_VC_INFO_STRUCT vc_info_cus1 = {
	0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	0x00, 0x2b, 0x1070, 0x0c30, /* VC0 */
	0x00, 0x00, 0x0000, 0x0000, /* VC1 */
	0x01, 0x2b, 0x0208, 0x0300, /* VC2 */
	0x00, 0x00, 0x0000, 0x0000, /* VC3 */
};

static struct SENSOR_VC_INFO_STRUCT vc_info_cus2 = {
	0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	0x00, 0x2b, 0x1070, 0x0940, /* VC0 */
	0x00, 0x00, 0x0000, 0x0000, /* VC1 */
	0x01, 0x2b, 0x0208, 0x0250, /* VC2 */
	0x00, 0x00, 0x0000, 0x0000, /* VC3 */
};

static struct SENSOR_VC_INFO_STRUCT vc_info_cus3 = {
	0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	0x00, 0x2b, 0x1070, 0x0c30, /* VC0 */
	0x00, 0x00, 0x0000, 0x0000, /* VC1 */
	0x01, 0x2b, 0x0208, 0x0300, /* VC2 */
	0x00, 0x00, 0x0000, 0x0000, /* VC3 */
};
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[] = {
	{4208, 3120, 104, 60, 4000, 3000, 4000, 3000,
		0, 0, 4000, 3000, 0, 0, 4000, 3000},
	{4208, 3120, 0, 0, 4208, 3120, 4208, 3120,
		0, 0, 4208, 3120, 0, 0, 4208, 3120},
	{4208, 3120, 0, 376, 4208, 2368, 4208, 2368,
		0, 0, 4208, 2368, 0, 0, 4208, 2368},
	{4208, 3120, 0, 376, 4208, 2368, 2104, 1184,
		0, 0, 2104, 1184, 0, 0, 2104, 1184},
	{4208, 3120, 0, 376, 4208, 2368, 2104, 1184,
		0, 0, 2104, 1184, 0, 0, 2104, 1184},
	{4208, 3120, 0, 0, 4208, 3120, 4208, 3120,
		0, 0, 4208, 3120, 0, 0, 4208, 3120},
	{4208, 3120, 0, 376, 4208, 2368, 4208, 2368,
		0, 0, 4208, 2368, 0, 0, 4208, 2368},
	{4208, 3120, 0, 0, 4208, 3120, 4208, 3120,
		0, 0, 4208, 3120, 0, 0, 4208, 3120},
};

static BYTE s5k3m5sx_common_data[OPLUS_CAMERA_COMMON_DATA_LENGTH] = { 0 };
static void read_module_data(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;
	read_s5k3m5sx_eeprom_info(ctx, EEPROM_META_MODULE_ID,
				  &(s5k3m5sx_common_data[0]), 2);
	imgsensor_info.module_id = (kal_uint16)(s5k3m5sx_common_data[1] << 8) |
				   s5k3m5sx_common_data[0];
	read_s5k3m5sx_eeprom_info(ctx, EEPROM_META_SENSOR_ID,
				  &(s5k3m5sx_common_data[2]), 2);
	read_s5k3m5sx_eeprom_info(ctx, EEPROM_META_LENS_ID,
				  &(s5k3m5sx_common_data[4]), 2);
	read_s5k3m5sx_eeprom_info(ctx, EEPROM_META_VCM_ID,
				  &(s5k3m5sx_common_data[6]), 2);
	read_s5k3m5sx_eeprom_info(ctx, EEPROM_META_MODULE_SN,
				  &(s5k3m5sx_common_data[8]), 17);
	read_s5k3m5sx_eeprom_info(ctx, EEPROM_META_AF_CODE,
				&(s5k3m5sx_common_data[25]), 6);

	for (idx = 0; idx < 30; idx = idx + 4)
		LOG_INF("cam data: %02x %02x %02x %02x\n",
		       s5k3m5sx_common_data[idx], s5k3m5sx_common_data[idx + 1],
		       s5k3m5sx_common_data[idx + 2],
		       s5k3m5sx_common_data[idx + 3]);
}

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

static kal_uint16 gain2reg(struct subdrv_ctx *ctx, const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0;

	reg_gain = gain / 32;
	return (kal_uint16) reg_gain;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_uint32 mode)
{
	DEBUG_LOG(ctx, "mode: %d\n", mode);

	if (mode)
		write_cmos_sensor(ctx, 0x0600, mode); /*100% Color bar*/
	else if (ctx->test_pattern)
		write_cmos_sensor(ctx, 0x0600, 0x0000); /*No pattern*/

	ctx->test_pattern = mode;
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_data(struct subdrv_ctx *ctx, struct mtk_test_pattern_data *data)
{
	pr_debug("test_patterndata mode = %d  R = %x, Gr = %x,Gb = %x,B = %x\n", ctx->test_pattern,
		data->Channel_R >> 22, data->Channel_Gr >> 22,
		data->Channel_Gb >> 22, data->Channel_B >> 22);

	set_cmos_sensor(ctx, 0x0602, (data->Channel_R >> 22) & 0x3ff);
	//set_cmos_sensor(ctx, 0x0603, (data->Channel_R >> 22) & 0xff);
	set_cmos_sensor(ctx, 0x0604, (data->Channel_Gr >> 22) & 0x3ff);
	//set_cmos_sensor(ctx, 0x0605, (data->Channel_Gr >> 22) & 0xff);
	set_cmos_sensor(ctx, 0x0606, (data->Channel_B >> 22) & 0x3ff);
	//set_cmos_sensor(ctx, 0x0607, (data->Channel_B >> 22) & 0xff);
	set_cmos_sensor(ctx, 0x0608, (data->Channel_Gb >> 22) & 0x3ff);
	//set_cmos_sensor(ctx, 0x0609, (data->Channel_Gb >> 22) & 0xff);
	commit_write_sensor(ctx);
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

static void set_dummy(struct subdrv_ctx *ctx)
{
	DEBUG_LOG(ctx, "dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	set_cmos_sensor(ctx, 0x0340, ctx->frame_length & 0xFFFF);
	set_cmos_sensor(ctx, 0x0342, ctx->line_length & 0xFFFF);

	commit_write_sensor(ctx);
}	/*  set_dummy  */

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
	/*  kal_int16 dummy_line;  */
	kal_uint32 frame_length = ctx->frame_length;

	DEBUG_LOG(ctx, "framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	if (frame_length >= ctx->min_frame_length)
		ctx->frame_length = frame_length;
	else
		ctx->frame_length = ctx->min_frame_length;

	ctx->dummy_line
		= ctx->frame_length - ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length) {
		ctx->frame_length = imgsensor_info.max_frame_length;
		ctx->dummy_line
			= ctx->frame_length - ctx->min_frame_length;
	}

	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;

}	/*  set_max_framerate  */

#define MAX_CIT_LSHIFT 7
static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint16 l_shift = 1;
	static bool is_long_exposure = KAL_FALSE;

	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (ctx->autoflicker_en) {
		realtime_fps
			= ctx->pclk
			/ ctx->line_length * 10
			/ ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
	}

	/* long expsoure */
	if (shutter
	    > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {

		for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
			if ((shutter >> l_shift)
			    < (imgsensor_info.max_frame_length
			       - imgsensor_info.margin))

				break;
		}
		if (l_shift > MAX_CIT_LSHIFT) {
			LOG_INF(
				"Unable to set such a long exposure %d, set to max\n",
				shutter);

			l_shift = MAX_CIT_LSHIFT;
		}
		shutter = shutter >> l_shift;
		ctx->frame_length = shutter + imgsensor_info.margin;

		LOG_INF("enter long exposure mode, time is %d", l_shift);

		set_cmos_sensor(ctx, 0x0702, l_shift << 8);
		set_cmos_sensor(ctx, 0x0704, l_shift << 8);
		is_long_exposure = KAL_TRUE;

		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		ctx->current_ae_effective_frame = 2;
		if (is_long_exposure) {
			set_cmos_sensor(ctx, 0x0702, 0);
			set_cmos_sensor(ctx, 0x0704, 0);
			is_long_exposure = KAL_FALSE;
		}

		LOG_INF("exit long exposure mode");
	}

	/* Update Shutter */
	set_cmos_sensor(ctx, 0x0104, 0x01);//gph start
	set_cmos_sensor(ctx, 0x0340, ctx->frame_length & 0xFFFF);
	set_cmos_sensor(ctx, 0X0202, shutter & 0xFFFF);
	if (!ctx->ae_ctrl_gph_en)
		set_cmos_sensor(ctx, 0x0104, 0x00);//grouphold end
	commit_write_sensor(ctx);

	DEBUG_LOG(ctx, "shutter =%d, framelength =%d\n",
		shutter, ctx->frame_length);

}	/*  write_shutter  */

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
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	ctx->shutter = shutter;

	write_shutter(ctx, shutter);
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
	write_cmos_sensor(ctx, 0x0340, ctx->frame_length & 0xFFFF);

	LOG_INF("Framelength: set=%d/input=%d/min=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length);
}

static void set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
				kal_uint32 *shutters, kal_uint16 shutter_cnt,
				kal_uint16 frame_length)
{
	if (shutter_cnt == 1) {
		ctx->shutter = shutters[0];

		if (shutters[0] > ctx->min_frame_length - imgsensor_info.margin)
			ctx->frame_length = shutters[0] + imgsensor_info.margin;
		else
			ctx->frame_length = ctx->min_frame_length;

		if (frame_length > ctx->frame_length)
			ctx->frame_length = frame_length;
		if (ctx->frame_length > imgsensor_info.max_frame_length)
			ctx->frame_length = imgsensor_info.max_frame_length;

		if (shutters[0] < imgsensor_info.min_shutter)
			shutters[0] = imgsensor_info.min_shutter;

		/* Update Shutter */
		set_cmos_sensor(ctx, 0x0104, 0x01);//grouphold start
		set_cmos_sensor(ctx, 0x0340, ctx->frame_length & 0xFFFF);
		set_cmos_sensor(ctx, 0X0202, shutters[0] & 0xFFFF);
		if (!ctx->ae_ctrl_gph_en)
			set_cmos_sensor(ctx, 0x0104, 0x00);//grouphold end
		commit_write_sensor(ctx);

		DEBUG_LOG(ctx, "shutters[0] =%d, framelength =%d\n",
			shutters[0], ctx->frame_length);
	}
}

/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(struct subdrv_ctx *ctx, kal_uint32 shutter,
				     kal_uint32 frame_length,
				     kal_bool auto_extend_en)
{	kal_uint16 realtime_fps = 0;

	ctx->shutter = shutter;


	/* Change frame time */
	/* dummy_line = frame_length - ctx->frame_length;
	 * ctx->frame_length = ctx->frame_length + dummy_line;
	 * ctx->min_frame_length = ctx->frame_length;

	 * if (shutter > ctx->min_frame_length - imgsensor_info.margin)
	 *	ctx->frame_length = shutter + imgsensor_info.margin;
	 */

	if (frame_length > 1)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;

	shutter = (shutter < imgsensor_info.min_shutter)
		? imgsensor_info.min_shutter
		: shutter;

	shutter = (shutter > (imgsensor_info.max_frame_length
			      - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	if (ctx->autoflicker_en) {
		realtime_fps
			= ctx->pclk
			/ ctx->line_length * 10
			/ ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
	}

	/* Update Shutter */
	set_cmos_sensor(ctx, 0x0340, ctx->frame_length & 0xFFFF);
	set_cmos_sensor(ctx, 0X0202, shutter & 0xFFFF);

	commit_write_sensor(ctx);

	LOG_INF("Exit! shutter =%d, framelength =%d\n",
		shutter, ctx->frame_length);

}	/* set_shutter_frame_length */

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
	kal_uint16 reg_gain;

	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		LOG_INF("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}

	reg_gain = gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	DEBUG_LOG(ctx, "gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

	set_cmos_sensor(ctx, 0x0204, (reg_gain & 0xFFFF));
	if (ctx->ae_ctrl_gph_en)
		set_cmos_sensor(ctx, 0x0104, 0x00); //grouphold end
	commit_write_sensor(ctx);
	return gain;
} /* set_gain */

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void check_streamoff(struct subdrv_ctx *ctx)
{
	unsigned int i = 0, framecnt = 0;
	int timeout = ctx->current_fps ? (10000 / ctx->current_fps) + 1 : 101;

	for (i = 0; i < timeout; i++) {
		framecnt = read_cmos_sensor_8(ctx, 0x0005);
		if (framecnt == 0xFF)
			return;
		mdelay(1);
	}
	LOG_INF(" Stream Off Fail1!\n");
}

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	unsigned int tmp;

	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor_8(ctx, 0x0100, 0X01);
	else {
		tmp = read_cmos_sensor_8(ctx, 0x0100);
		if (tmp)
			write_cmos_sensor_8(ctx, 0x0100, 0x00);
		//check_streamoff(ctx);
	}
	return ERROR_NONE;
}

static void sensor_init(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	write_cmos_sensor(ctx, 0x6028, 0x4000);
	write_cmos_sensor(ctx, 0x0000, 0x0006);
	write_cmos_sensor(ctx, 0x0000, 0x30D5);
	write_cmos_sensor(ctx, 0x6214, 0x7971);
	write_cmos_sensor(ctx, 0x6218, 0x7150);
	mdelay(3);
	write_cmos_sensor(ctx, 0x0A02, 0x7800);
	write_cmos_sensor(ctx, 0x6028, 0x2000);
	write_cmos_sensor(ctx, 0x602A, 0x3EAC);
	if (I2C_BURST)
		burst_table_write_cmos_sensor(ctx, 0x6F12,
		(kal_uint16 *)sensor_init_setting_array1_burst,
		sizeof(sensor_init_setting_array1_burst)/sizeof(kal_uint16));
	else
		table_write_cmos_sensor(ctx,
		sensor_init_setting_array1,
		sizeof(sensor_init_setting_array1)/sizeof(kal_uint16));

	table_write_cmos_sensor(ctx,
		sensor_init_setting_array2,
		sizeof(sensor_init_setting_array2)/sizeof(kal_uint16));

	/* set pdaf DT to 0x2B */
	write_cmos_sensor_8(ctx, 0x0116, 0x2B);
}

static void preview_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(ctx,
		preview_setting_array,
		sizeof(preview_setting_array)/sizeof(kal_uint16));
	LOG_INF("X");
} /* preview_setting */

static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_INF(" E! currefps:%d\n",  currefps);
	table_write_cmos_sensor(ctx,
		custom3_setting_array,
		sizeof(custom3_setting_array)/sizeof(kal_uint16));
        if(0)
	table_write_cmos_sensor(ctx,
		capture_setting_array,
		sizeof(capture_setting_array)/sizeof(kal_uint16));
}

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_INF(" E! currefps:%d\n",  currefps);
	table_write_cmos_sensor(ctx,
		normal_video_setting_array,
		sizeof(normal_video_setting_array)/sizeof(kal_uint16));
}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(ctx,
		hs_video_setting_array,
		sizeof(hs_video_setting_array)/sizeof(kal_uint16));
}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(ctx,
		slim_video_setting_array,
		sizeof(slim_video_setting_array)/sizeof(kal_uint16));
}

static void custom1_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(ctx,
		custom1_setting_array,
		sizeof(custom1_setting_array)/sizeof(kal_uint16));
}

static void custom2_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(ctx,
		custom2_setting_array,
		sizeof(custom2_setting_array)/sizeof(kal_uint16));
}

static void custom3_setting(struct subdrv_ctx *ctx)
{
	LOG_INF("E\n");
	table_write_cmos_sensor(ctx,
		custom3_setting_array,
		sizeof(custom3_setting_array)/sizeof(kal_uint16));
}

static kal_uint32 return_sensor_id(struct subdrv_ctx *ctx)
{
	return ((read_cmos_sensor_8(ctx, 0x0000) << 8) | read_cmos_sensor_8(ctx, 0x0001));
}

static void read_sensor_Cali(struct subdrv_ctx *ctx)
{
	LOG_INF("return data\n");
	ctx->is_read_preload_eeprom = 1;
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
			*sensor_id = return_sensor_id(ctx);
			//if (*sensor_id == imgsensor_info.sensor_id) {
			if (*sensor_id == S5K3M5_SENSOR_ID) {
				*sensor_id = S5K3M5SX_SENSOR_ID;
				LOG_INF(
					"i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, *sensor_id);
				if (ctx->i2c_write_id == 0x5a) {
					ctx->mirror = IMAGE_HV_MIRROR;
					imgsensor_info.sensor_output_dataformat =
						SENSOR_OUTPUT_FORMAT_RAW_Gb;
				}
				if (first_read) {
					read_module_data(ctx);
					//read_s5k3m5sx_LRC(ctx, s5k3m5sx_LRC_setting);
					//read_s5k3m5sx_QSC(ctx, s5k3m5sx_QSC_setting);
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
	kal_uint32 sensor_id = 0;

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			get_imgsensor_id(ctx, &sensor_id);
			if (sensor_id == imgsensor_info.sensor_id) {
			//if (sensor_id == S5K3M5_SENSOR_ID) {
				//sensor_id = S5K3M5SX_SENSOR_ID;
				LOG_INF(
					"i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, sensor_id);
				if (ctx->i2c_write_id == 0x5a) {
					ctx->mirror = IMAGE_HV_MIRROR;
					/*imgsensor_info.sensor_output_dataformat =
						SENSOR_OUTPUT_FORMAT_RAW_Gb;*/
				}
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


	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->shutter = 0x3D0;
	ctx->gain = 0x1000;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	ctx->ihdr_mode = 0;
	ctx->test_pattern = 0;
	ctx->current_fps = imgsensor_info.pre.max_framerate;

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
	streaming_control(ctx, 0);
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
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	preview_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	LOG_INF("X\n");
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
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	capture_setting(ctx, ctx->current_fps);
	set_mirror_flip(ctx, ctx->mirror);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
	       MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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

	LOG_INF("X\n");
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;

	hs_video_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	ctx->autoflicker_en = KAL_FALSE;

	slim_video_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/* slim_video */

static kal_uint32 custom1(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	ctx->pclk = imgsensor_info.custom1.pclk;
	ctx->line_length = imgsensor_info.custom1.linelength;
	ctx->frame_length = imgsensor_info.custom1.framelength;
	ctx->min_frame_length = imgsensor_info.custom1.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	custom1_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/* custom1 */

static kal_uint32 custom2(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	ctx->pclk = imgsensor_info.custom2.pclk;
	ctx->line_length = imgsensor_info.custom2.linelength;
	ctx->frame_length = imgsensor_info.custom2.framelength;
	ctx->min_frame_length = imgsensor_info.custom2.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	custom2_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/* custom2 */

static kal_uint32 custom3(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	ctx->pclk = imgsensor_info.custom3.pclk;
	ctx->line_length = imgsensor_info.custom3.linelength;
	ctx->frame_length = imgsensor_info.custom3.framelength;
	ctx->min_frame_length = imgsensor_info.custom3.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	custom3_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	LOG_INF("X\n");
	return ERROR_NONE;
}	/* custom1 */

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

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

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
	sensor_info->PDAF_Support = 2;
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



	return ERROR_NONE;
}	/*	get_info  */

static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	ctx->current_scenario_id = scenario_id;

	/* streamoff should be finished before mode changes */
	check_streamoff(ctx);

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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control(ctx) */

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
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

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx,
		kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	if (enable) /*enable auto flicker*/
		ctx->autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
		ctx->autoflicker_en = KAL_FALSE;
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	DEBUG_LOG(ctx, "scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;

		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength)
			: 0;

		ctx->frame_length =
			imgsensor_info.pre.framelength
			+ ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);

		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		if (framerate == 0)
			return ERROR_NONE;

		frame_length
			= imgsensor_info.normal_video.pclk
			/ framerate * 10
			/ imgsensor_info.normal_video.linelength;


		ctx->dummy_line
			= (frame_length
			   > imgsensor_info.normal_video.framelength)
			? (frame_length
			   - imgsensor_info.normal_video.framelength)
			: 0;

		ctx->frame_length =
			imgsensor_info.normal_video.framelength
			+ ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);

		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		frame_length
			= imgsensor_info.cap.pclk
			/ framerate * 10
			/ imgsensor_info.cap.linelength;

		ctx->dummy_line
			= (frame_length > imgsensor_info.cap.framelength)
			? (frame_length - imgsensor_info.cap.framelength)
			: 0;

		ctx->frame_length
			= imgsensor_info.cap.framelength
			+ ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);

		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		frame_length
			= imgsensor_info.hs_video.pclk
			/ framerate * 10
			/ imgsensor_info.hs_video.linelength;

		ctx->dummy_line
			= (frame_length > imgsensor_info.hs_video.framelength)
			? (frame_length - imgsensor_info.hs_video.framelength)
			: 0;

		ctx->frame_length
			= imgsensor_info.hs_video.framelength
			+ ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);

		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		frame_length
			= imgsensor_info.slim_video.pclk
			/ framerate * 10
			/ imgsensor_info.slim_video.linelength;

		ctx->dummy_line
			= (frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;

		ctx->frame_length
			= imgsensor_info.slim_video.framelength
			+ ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);

		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		frame_length
			= imgsensor_info.custom1.pclk
			/ framerate * 10
			/ imgsensor_info.custom1.linelength;

		ctx->dummy_line
			= (frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength)
			: 0;

		ctx->frame_length
			= imgsensor_info.custom1.framelength
			+ ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);

		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		frame_length
			= imgsensor_info.custom2.pclk
			/ framerate * 10
			/ imgsensor_info.custom2.linelength;

		ctx->dummy_line
			= (frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength)
			: 0;

		ctx->frame_length
			= imgsensor_info.custom2.framelength
			+ ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);

		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		frame_length
			= imgsensor_info.custom3.pclk
			/ framerate * 10
			/ imgsensor_info.custom3.linelength;

		ctx->dummy_line
			= (frame_length > imgsensor_info.custom3.framelength)
			? (frame_length - imgsensor_info.custom3.framelength)
			: 0;

		ctx->frame_length =
			imgsensor_info.custom3.framelength
			+ ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);

		break;
	default:  /*coding with  preview scenario by default*/
		frame_length
			= imgsensor_info.pre.pclk
			/ framerate * 10
			/ imgsensor_info.pre.linelength;

		ctx->dummy_line
			= (frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength)
			: 0;

		ctx->frame_length
			= imgsensor_info.pre.framelength
			+ ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);

		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);

		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
		    enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
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
	default:
		break;
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
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	/* SET_SENSOR_AWB_GAIN *pSetSensorAWB
	 *  = (SET_SENSOR_AWB_GAIN *)feature_para;
	 */
	int ret = ERROR_NONE;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_INF("feature_id = %d\n", feature_id);*/
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
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(s5k3m5_ana_gain_table);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)s5k3m5_ana_gain_table,
			sizeof(s5k3m5_ana_gain_table));
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
				= (imgsensor_info.normal_video.framelength
				   << 16)
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
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 3000000;
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
		/* night_mode((BOOL) *feature_data); */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT32) * feature_data);
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
		*feature_return_para_32++ = (s5k3m5sx_common_data[1] << 24) |
					    (s5k3m5sx_common_data[0] << 16) |
					    (s5k3m5sx_common_data[3] << 8) |
					    (s5k3m5sx_common_data[2] & 0xFF);
		*feature_return_para_32 = (s5k3m5sx_common_data[5] << 24) |
					  (s5k3m5sx_common_data[4] << 16) |
					  (s5k3m5sx_common_data[7] << 8) |
					  (s5k3m5sx_common_data[6] & 0xFF);
		*feature_para_len = 8;
		LOG_INF("s5k3m5sx GET_MODULE_CamInfo:%d %d\n",
			*feature_para_len, *feature_data_32);
		break;

	case SENSOR_FEATURE_GET_MODULE_SN:
		*feature_return_para_32++ = (s5k3m5sx_common_data[11] << 24) |
					    (s5k3m5sx_common_data[10] << 16) |
					    (s5k3m5sx_common_data[9] << 8) |
					    (s5k3m5sx_common_data[8] & 0xFF);
		*feature_para_len = 4;
		LOG_INF("s5k3m5sx GET_MODULE_SN:%d %d\n", *feature_para_len,
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
		memcpy(feature_return_para_32, s5k3m5sx_common_data,
		       OPLUS_CAMERA_COMMON_DATA_LENGTH);
		*feature_para_len = OPLUS_CAMERA_COMMON_DATA_LENGTH;
		break;
	case SENSOR_FEATURE_GET_EEPROM_STEREODATA:
		if(*feature_para_len > CALI_DATA_SLAVE_LENGTH)
			*feature_para_len = CALI_DATA_SLAVE_LENGTH;
		read_s5k3m5sx_eeprom_info(ctx, EEPROM_META_STEREO_DATA,
						(BYTE *)feature_return_para_32, *feature_para_len);
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
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(ctx,
				(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		// LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
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
		wininfo =
			(struct SENSOR_WINSIZE_INFO_STRUCT *)
				(uintptr_t)(*(feature_data+1));

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
			PDAFinfo->i4BlockNumX = 124;
			PDAFinfo->i4BlockNumY = 92;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM2:
			PDAFinfo->i4BlockNumX = 130;
			PDAFinfo->i4BlockNumY = 74;
			break;
		}
		break;
		/*LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);

		PDAFinfo =
			(struct SET_PD_BLOCK_INFO_T *)
				(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		default:
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info_binning,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		}
		break;*/
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		// LOG_INF(
			// "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
			// (UINT16) *feature_data);

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
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
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
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_32 = get_sensor_temperature(ctx);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));

		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		/*LOG_INF("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
		 *	(*feature_para_len));
		 */

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
					 (UINT16) (*(feature_data + 1)),
					 (BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length(ctx) support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
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
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		// LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			 // *feature_return_para_32);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
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
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		pvcinfo =
			(struct SENSOR_VC_INFO_STRUCT *)
				(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&vc_info_preview,
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			memcpy((void *)pvcinfo, (void *)&vc_info_capture,
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			memcpy((void *)pvcinfo, (void *)&vc_info_video,
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			memcpy((void *)pvcinfo, (void *)&vc_info_hs_video,
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)pvcinfo, (void *)&vc_info_slim_video,
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			memcpy((void *)pvcinfo, (void *)&vc_info_cus1,
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			memcpy((void *)pvcinfo, (void *)&vc_info_cus2,
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			memcpy((void *)pvcinfo, (void *)&vc_info_cus3,
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		default:
			break;
		}
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME:
		set_multi_shutter_frame_length(ctx, (UINT32 *)(*feature_data),
					(UINT16) (*(feature_data + 1)),
					(UINT16) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_PRELOAD_EEPROM_DATA:
		/*get eeprom preloader data*/
		*feature_return_para_32 = ctx->is_read_preload_eeprom;
		*feature_para_len = 4;
		if (ctx->is_read_preload_eeprom != 1)
			read_sensor_Cali(ctx);
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
			.hsize = 0x0fa0,
			.vsize = 0x0bb8,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x01f0,
			.vsize = 0x02e0,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0c30,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0208,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0940,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0208,
			.vsize = 0x0250,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0838,
			.vsize = 0x04a0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0208,
			.vsize = 0x0250,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0838,
			.vsize = 0x04a0,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0208,
			.vsize = 0x0250,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0c30,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0208,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0940,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0208,
			.vsize = 0x0250,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1070,
			.vsize = 0x0c30,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0208,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS,
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
		fd->num_entries = ARRAY_SIZE(frame_desc_hs_vid);
		memcpy(fd->entry, frame_desc_hs_vid, sizeof(frame_desc_hs_vid));
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_slim_vid);
		memcpy(fd->entry, frame_desc_slim_vid, sizeof(frame_desc_slim_vid));
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
	default:
		return -1;
	}

	return 0;
}
#endif


static const struct subdrv_ctx defctx = {

	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_min = BASEGAIN,
	.ana_gain_step = 1,
	.exposure_def = 0x3D0,
	.exposure_max = (0xffff * 128) - 4,
	.exposure_min = 4,
	.exposure_step = 1,
	.frame_time_delay_frame = 2,
	.margin = 4,
	.max_frame_length = 0xffff,

	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
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
	.current_ae_effective_frame = 2,
	.ae_ctrl_gph_en = 0,
};

static int get_temp(struct subdrv_ctx *ctx, int *temp)
{
	*temp = get_sensor_temperature(ctx) * 1000;
	return 0;
}

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
	csi_param->legacy_phy = 0;
	csi_param->not_fixed_trail_settle = 0;
	csi_param->dphy_trail = 0;

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
	.get_csi_param = get_csi_param,
#ifdef IMGSENSOR_VC_ROUTING
	.get_frame_desc = get_frame_desc,
#endif
	.get_temp = get_temp,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2804000, 0},
	{HW_ID_AFVDD, 2804000, 0},
	{HW_ID_DVDD, 1056000, 0},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_RST, 1, 2},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 1},
};

const struct subdrv_entry s5k3m5sx_mipi_raw_entry = {
	.name = "s5k3m5sx_mipi_raw",
	.id = S5K3M5SX_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};
