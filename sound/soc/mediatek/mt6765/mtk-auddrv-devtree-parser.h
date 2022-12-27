/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Michael Hsiao <michael.hsiao@mediatek.com>
 */

/******************************************************************************
 *
 * Filename:
 * ---------
 *   auddrv_devtree_parser.h
 *
 * Project:
 * --------
 *   devtree config parser
 *
 * Description:
 * ------------
 *  devtree config parser
 *
 * Author:
 * -------
 *   Chipeng Chang
 *
 *-----------------------------------------------------------------------------
 *
 *
 ******************************************************************************
 */

#ifndef _MT_AUDDRV_DEVTREE_PARSER_H_
#define _MT_AUDDRV_DEVTREE_PARSER_H_

#if IS_ENABLED(CONFIG_OF)
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#define AUDDRV_I2S0_MCLKGPIO "i2s0mclk-gpio"
#define AUDDRV_I2S0_CLKGPIO "i2s0clk-gpio"
#define AUDDRV_I2S0_WSGPIO "i2s0ws-gpio"
#define AUDDRV_I2S0_DATGPIO "i2s0dat-gpio"
#define AUDDRV_I2S0_DAT1GPIO "i2s0dat1-gpio"
#define AUDDRV_I2S0_DATAINGPIO "i2s0datain-gpio"

#define AUDDRV_I2S1_MCLKGPIO "i2s1mclk-gpio"
#define AUDDRV_I2S1_CLKGPIO "i2s1clk-gpio"
#define AUDDRV_I2S1_WSGPIO "i2s1ws-gpio"
#define AUDDRV_I2S1_DATGPIO "i2s1dat-gpio"
#define AUDDRV_I2S1_DAT1GPIO "i2s1dat1-gpio"

#define AUDDRV_I2S2_MCLKGPIO "i2s2mclk-gpio"
#define AUDDRV_I2S2_CLKGPIO "i2s2clk-gpio"
#define AUDDRV_I2S2_WSGPIO "i2s2ws-gpio"
#define AUDDRV_I2S2_DATGPIO "i2s2dat-gpio"
#define AUDDRV_I2S2_DAT1GPIO "i2s2dat1-gpio"

#define AUDDRV_I2S3_MCLKGPIO "i2s3mclk-gpio"
#define AUDDRV_I2S3_CLKGPIO "i2s3clk-gpio"
#define AUDDRV_I2S3_WSGPIO "i2s3ws-gpio"
#define AUDDRV_I2S3_DATGPIO "i2s3dat-gpio"
#define AUDDRV_I2S3_DAT1GPIO "i2s3dat1-gpio"

#define AUDDRV_AUD_CLKGPIO "audclk-gpio"
#define AUDDRV_AUD_DATIGPIO "audmosi-gpio"
#define AUDDRV_AUD_DATOGPIO "audmopi-gpio"

enum Auddrv_I2S_NUMBER {
	Auddrv_I2S0_Setting = 0,
	Auddrv_I2S1_Setting,
	Auddrv_I2S2_Setting,
	Auddrv_I2S3_Setting,
	Auddrv_I2S_Num,
};

enum Auddrv_I2S_Setting {
	Auddrv_I2S_Setting_ws = 0,
	Auddrv_I2S_Setting_bck,
	Auddrv_I2S_Setting_D00,
	Auddrv_I2S_Setting_D01,
	Auddrv_I2S_Setting_I00,
	Auddrv_I2S_Setting_I01,
	Auddrv_I2S_Setting_Mclk,
	Auddrv_I2S_Attribute_Num
};

enum Auddrv_CLK_Setting {
	Auddrv_CLK_Mosi = 0,
	Auddrv_DataIn1_Mosi,
	Auddrv_DataOut1_Mosi,
	Auddrv_Attribute_num,
};

struct auddrv_i2s_attribute {
	unsigned int Gpio_Number;
	unsigned int Gpio_Mode;
};

void Auddrv_Devtree_Init(void);
void Auddrv_DevTree_I2S_Setting(const char *DevTreeName);
void Auddrv_Devtree_Dump(void);
struct auddrv_i2s_attribute *GetI2SSetting(uint32_t I2S_Number,
					   uint32_t I2S_Setting);

#endif
