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

#define PFX "imx355_pdafotp"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"
#include "imx355_eeprom_22047.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define imx355_EEPROM_READ_ID 0xA3
#define imx355_EEPROM_WRITE_ID 0xA2
#define imx355_I2C_SPEED 100
#define imx355_MAX_OFFSET 0x8000

#define DATA_SIZE 2048
#define SPC_START_ADDR 0x763
#define DCC_START_ADDR 0x8c3

static BYTE imx355_DCC_data[384] = { 0 };    /* 16x12x2 */

static bool get_done_dcc;
static int last_size_dcc;
static int last_offset_dcc;

static bool get_done_spc;
static int last_size_spc;
static int last_offset_spc;

bool imx355_read_eeprom_22047(struct subdrv_ctx *ctx, kal_uint16 addr, BYTE *data, int size)
{
    LOG_INF("enter _read_eeprom size = %d\n", size);
    if (addr + size > imx355_MAX_OFFSET) {
        return false;
    }

    if (adaptor_i2c_rd_p8(ctx->i2c_client,
            imx355_EEPROM_READ_ID >> 1, addr, data, size) < 0) {
        return false;
    }

    if (addr == SPC_START_ADDR) {
        get_done_spc = true;
        last_size_spc = size;
        last_offset_spc = addr + size;
    } else {
        get_done_dcc = true;
        last_size_dcc = size;
        last_offset_dcc = addr + size;
    }
    return true;
}

void imx355_read_SPC_22047(struct subdrv_ctx *ctx, BYTE *data)
{

    int addr = SPC_START_ADDR;
    int size = 352;

    if (!get_done_spc || last_size_spc != size) {
        if (!imx355_read_eeprom_22047(ctx, addr, data, size)) {
            get_done_spc = 0;
            last_size_spc = 0;
            last_offset_spc = 0;
            /* return false; */
        }
    }
    /* memcpy(data, imx355_SPC_data , size); */
    /* return true; */
}

void imx355_read_DCC_22047(struct subdrv_ctx *ctx,
        kal_uint16 addr, BYTE *data, kal_uint32 size)
{
    /* int i; */
    addr = DCC_START_ADDR;
    size = 384;
    if (!get_done_dcc || last_size_dcc != size) {
        if (!imx355_read_eeprom_22047(ctx, addr, imx355_DCC_data, size)) {
            get_done_dcc = 0;
            last_size_dcc = 0;
            last_offset_dcc = 0;
            /* return false; */
        }
    }

    memcpy(data, imx355_DCC_data, size);
    /* return true; */
}

struct eeprom_map_info imx355_eeprom_info_22047[] = {
    { EEPROM_META_MODULE_ID, 0x0000, 0x000F, 0x0010, 2, true },
    { EEPROM_META_SENSOR_ID, 0x0006, 0x000F, 0x0010, 2, true },
    { EEPROM_META_LENS_ID, 0x0008, 0x000F, 0x0010, 2, true },
    { EEPROM_META_VCM_ID, 0x000A, 0x000F, 0x0010, 2, true },
    { EEPROM_META_MIRROR_FLIP, 0x000E, 0x000F, 0x0010, 1, true },
    { EEPROM_META_MODULE_SN, 0x00B0, 0x00C1, 0x00C2, 17, true },
};

unsigned int read_imx355_eeprom_info_22047(struct subdrv_ctx *ctx, kal_uint16 meta_id,
                     BYTE *data, int size)
{
    kal_uint16 addr;
    int readsize;

    if (meta_id != imx355_eeprom_info_22047[meta_id].meta)
        return -1;

    if (size != imx355_eeprom_info_22047[meta_id].size)
        return -1;

    addr = imx355_eeprom_info_22047[meta_id].start;
    readsize = imx355_eeprom_info_22047[meta_id].size;

    imx355_read_eeprom_22047(ctx, addr, data, readsize);

    return 0;
}
