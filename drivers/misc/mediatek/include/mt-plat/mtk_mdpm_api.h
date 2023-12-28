/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2017 MediaTek Inc.
 */

#ifndef _MTK_MDPM_API_H_
#define _MTK_MDPM_API_H_

#include "mtk_dlpt.h"
#include "mtk_pbm.h"

enum mdpm_power_type {
	MAX_POWER = 0,
	AVG_POWER,
	POWER_TYPE_NUM
};


extern void init_md_section_level(enum pbm_kicker kicker);
extern int get_md1_power(enum mdpm_power_type power_type, bool need_update);

#endif
