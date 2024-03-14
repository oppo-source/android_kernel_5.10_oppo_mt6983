/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __S5K3M5SX_EEPROM_H__
#define __S5K3M5SX_EEPROM_H__

#include "kd_camera_typedef.h"

#include "adaptor-subdrv.h"

#include "kd_eeprom_oplus.h"

/*
 * LRC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_s5k3m5sx_LRC(struct subdrv_ctx *ctx, u8 *data);

/*
 * DCC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_s5k3m5sx_DCC(struct subdrv_ctx *ctx, u8 *data);

/*
 * read eeprom data
 *
 * @param meta_id info_id
 * @param data Buffer
 * @param size ReadSize
 * @return size of data
 */
unsigned int read_s5k3m5sx_eeprom_info(struct subdrv_ctx *ctx,
				       kal_uint16 meta_id, BYTE *data,
				       int size);
#endif
