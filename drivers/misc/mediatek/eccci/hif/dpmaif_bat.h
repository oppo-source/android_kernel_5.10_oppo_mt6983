/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#ifndef __MODEM_DPMA_BAT_V2_H__
#define __MODEM_DPMA_BAT_V2_H__

#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/dmapool.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/skbuff.h>
#include <mt-plat/mtk_ccci_common.h>

#include "ccci_config.h"

extern struct hif_dpmaif_ctrl *dpmaif_ctrl;



#define BAT_ALLOC_NO_PAUSED  0
#define BAT_ALLOC_IS_PAUSED  1
#define BAT_ALLOC_PAUSE_SUCC 2



void ccci_dpmaif_bat_hw_init(void);

int ccci_dpmaif_bat_sw_init(void);

int ccci_dpmaif_bat_start(void);

void ccci_dpmaif_bat_stop(void);

inline void ccci_dpmaif_bat_wakeup_thread(void);

#endif /* __MODEM_DPMA_BAT_V2_H__ */

