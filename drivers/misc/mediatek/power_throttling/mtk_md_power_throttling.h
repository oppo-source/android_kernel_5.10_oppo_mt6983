/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */

#ifndef _MTK_MD_POWER_THROTTLING_H_
#define _MTK_MD_POWER_THROTTLING_H_

enum POWER_THROTTLE_NOTIFY_TYPE {
	PT_LOW_BATTERY_VOLTAGE,
	PT_BATTERY_PERCENT,
	PT_OVER_CURRENT,
};

enum tmc_ctrl_low_pwr_enum {
	TMC_CTRL_LOW_POWER_LOW_BATTERY_EVENT = 0,
	TMC_CTRL_LOW_POWER_RECHARGE_BATTERY_EVENT,
	TMC_CTRL_LOW_POWER_MAX
};

#define TMC_CTRL_CMD_TX_POWER	10
#define TMC_CTRL_CMD_LOW_POWER_IND 11

#define LBAT_REDUCE_TX_POWER	6
#define OC_REDUCE_TX_POWER	6

#endif
