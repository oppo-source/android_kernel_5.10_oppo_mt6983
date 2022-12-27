/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _SSPM_RESERVEDMEM_DEFINE_H_
#define _SSPM_RESERVEDMEM_DEFINE_H_
#include <sspm_reservedmem.h>

#define _SSPM_INTERNAL_

#ifdef _SSPM_INTERNAL_
/* The total size of sspm_reserve_mblock should less equal than
 * reserve-memory-sspm_share of device tree
 */
static struct sspm_reserve_mblock sspm_reserve_mblock[NUMS_MEM_ID] = {
	{
		.num = SSPM_MEM_ID,
		.size = 0x400 + SSPM_PLT_LOGGER_BUF_LEN,
		/* logger header + 1M log buffer */
	},
	{
		.num = PWRAP_MEM_ID,
		.size = 0x300,  /* 768 bytes */
	},
	{
		.num = PMIC_MEM_ID,
		.size = 0xC00,  /* 3K */
	},
	{
		.num = UPD_MEM_ID,
		.size = 0x1800, /* 6K */
	},
	{
		.num = QOS_MEM_ID,
		.size = 0x1000, /* 4K */
	},
	{
		.num = SWPM_MEM_ID,
		.size = 0xC00,  /* 3K */
	},
#if IS_ENABLED(CONFIG_MTK_GMO_RAM_OPTIMIZE) || IS_ENABLED(CONFIG_MTK_MET_MEM_ALLOC)
#else
	{
		.num = MET_MEM_ID,
		.size = 0x400000, /* 4M */
	},
#endif
	/* TO align 64K, total is 1M+64K. The remaining size = 0xB900. */
};
#endif
#endif

