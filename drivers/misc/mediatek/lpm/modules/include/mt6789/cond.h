/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __COND_H__
#define __COND_H__

/* Definition about SPM_COND_CHECK_BLOCKED
 * bit [00 ~ 15]: cg blocking index
 * bit [16 ~ 29]: pll blocking index
 * bit [30]     : pll blocking information
 * bit [31]    : idle condition check fail
 */

#define SPM_COND_BLOCKED_CG_IDX		(0)
#define SPM_COND_BLOCKED_PLL_IDX	(16)
#define SPM_COND_BLOCKED_PLL		(1<<30L)
#define SPM_COND_CHECK_FAIL		(1<<31L)

enum PLAT_SPM_COND {
	PLAT_SPM_COND_MTCMOS_0 = 0,
	PLAT_SPM_COND_CG_INFRA_0,
	PLAT_SPM_COND_CG_INFRA_1,
	PLAT_SPM_COND_CG_INFRA_2,
	PLAT_SPM_COND_CG_INFRA_3,
	PLAT_SPM_COND_CG_INFRA_4,
	PLAT_SPM_COND_CG_MMSYS_0,
	PLAT_SPM_COND_CG_MMSYS_3,
	PLAT_SPM_COND_MAX,
};

enum PLAT_SPM_PLL_COND {
	PLAT_SPM_COND_UNIVPLL = 0,
	PLAT_SPM_COND_MFGPLL,
	PLAT_SPM_COND_MSDCPLL,
	PLAT_SPM_COND_TVPLL,
	PLAT_SPM_COND_MMPLL,
	PLAT_SPM_COND_PLL_MAX,
};

#endif
