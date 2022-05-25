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
#include <linux/slab.h>
#include "kd_camera_typedef.h"

#define PFX "imx350_pdafotp"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"
#include "imx350_eeprom.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define imx350_EEPROM_READ_ID 0xA0
#define imx350_EEPROM_WRITE_ID 0xA1
#define imx350_I2C_SPEED 100
#define imx350_MAX_OFFSET 4096

#define DATA_SIZE 2048
#define SPC_START_ADDR 0x763
#define DCC_START_ADDR 0x8c3

BYTE imx350_DCC_data[384] = { 0 };	/* 16x12x2 */

static bool get_done_dcc;
static int last_size_dcc;
static int last_offset_dcc;

static bool get_done_spc;
static int last_size_spc;
static int last_offset_spc;

static bool imx350_selective_read_eeprom(struct subdrv_ctx *ctx,
		kal_uint16 addr, BYTE *data)
{
	if (addr > imx350_MAX_OFFSET)
		return false;
	if (adaptor_i2c_rd_u8(ctx->i2c_client,
		imx350_EEPROM_READ_ID >> 1, addr, data) < 0)
		return false;
	return true;
}

static bool imx350_read_eeprom(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size)
{
	int i = 0;
	int offset = addr;

	LOG_INF("enter _read_eeprom size = %d\n", size);
	for (i = 0; i < size; i++) {
		if (!imx350_selective_read_eeprom(ctx, offset, &data[i]))
			return false;
		/* LOG_INF("read_eeprom 0x%0x %d\n",offset, data[i]); */
		offset++;
	}

	if (addr == SPC_START_ADDR) {
		get_done_spc = true;
		last_size_spc = size;
		last_offset_spc = offset;
	} else {
		get_done_dcc = true;
		last_size_dcc = size;
		last_offset_dcc = offset;
	}
	return true;
}

void imx350_read_SPC(struct subdrv_ctx *ctx, BYTE *data)
{

	int addr = SPC_START_ADDR;
	int size = 352;

	if (!get_done_spc || last_size_spc != size) {
		if (!imx350_read_eeprom(ctx, addr, data, size)) {
			get_done_spc = 0;
			last_size_spc = 0;
			last_offset_spc = 0;
			/* return false; */
		}
	}
	/* memcpy(data, imx350_SPC_data , size); */
	/* return true; */
}

void imx350_read_DCC(struct subdrv_ctx *ctx,
		kal_uint16 addr, BYTE *data, kal_uint32 size)
{
	/* int i; */
	addr = DCC_START_ADDR;
	size = 384;
	if (!get_done_dcc || last_size_dcc != size) {
		if (!imx350_read_eeprom(ctx, addr, imx350_DCC_data, size)) {
			get_done_dcc = 0;
			last_size_dcc = 0;
			last_offset_dcc = 0;
			/* return false; */
		}
	}

	memcpy(data, imx350_DCC_data, size);
	/* return true; */
}

