// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/**
 * @file    mtk_gpufreq_common.c
 * @brief   Driver for GPU-DVFS
 */

/**
 * ===============================================
 * SECTION : Include files
 * ===============================================
 */
#include <linux/slab.h>
#include <aee.h>
#include <aed.h>
#include "mtk_gpufreq_common.h"

#if IS_ENABLED(CONFIG_MTK_AEE_AED)
static struct gpu_assert_info g_pending_info;
static int g_have_pending_info;
#endif

static void dump_except(enum g_exception_enum except_type, char *except_str)
{
#if IS_ENABLED(CONFIG_MTK_AEE_AED)
	if (except_str == NULL) {
		pr_info("%s: NULL string\n", __func__);
		return;
	}
	if (aee_get_mode() != AEE_MODE_NOT_INIT) {
		aee_kernel_warning("GPU_FREQ",
			"\n\n%s\nCRDISPATCH_KEY:%s\n",
			except_str,
			g_exception_string[except_type]);
	} else if (g_have_pending_info == 0) {
		/* the aee driver is not ready
		 * we will call kernel api later
		 */
		g_pending_info.exception_type = except_type;
		strncpy(g_pending_info.exception_string, except_str, 1023);
		g_have_pending_info = 1;
	}
#else
	(void)except_type;
	(void)except_str;
#endif
}

void gpu_assert(bool cond, enum g_exception_enum except_type,
	const char *except_str, ...)
{
	va_list args;
	int cx;
	char tmp_string[1024];

	if (unlikely(!(cond))) {
		va_start(args, except_str);
		cx = vsnprintf(tmp_string, sizeof(tmp_string),
			except_str, args);
		va_end(args);

		pr_info("[GPU/DVFS] assert:%s", tmp_string);
		if (cx >= 0)
			dump_except(except_type, tmp_string);
	}
}

/* check if there have pending info
 * return: the count of pending info
 */
void check_pending_info(void)
{
#if IS_ENABLED(CONFIG_MTK_AEE_AED)
	if (g_have_pending_info &&
		aee_get_mode() != AEE_MODE_NOT_INIT) {
		g_have_pending_info = 0;
		dump_except(
			g_pending_info.exception_type,
			g_pending_info.exception_string);
	}
#endif
}

