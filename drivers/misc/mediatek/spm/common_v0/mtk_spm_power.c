// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>

#if IS_ENABLED(CONFIG_MTK_PMIC) || IS_ENABLED(CONFIG_MTK_PMIC_NEW_ARCH)
#include <mt-plat/upmu_common.h>
#endif

#include <mtk_idle.h>
#include <mtk_idle_internal.h> /* MTK_IDLE_OPT_XXX */
#include <mtk_spm_internal.h>
#include <mtk_sspm.h>

void mtk_idle_power_pre_process(int idle_type, unsigned int op_cond)
{
	int ret;
	struct spm_data spm_d;
	unsigned int spm_opt = 0;
	int cmd;

	cmd = (idle_type == IDLE_TYPE_DP) ? SPM_DPIDLE_ENTER :
		(idle_type == IDLE_TYPE_SO) ? SPM_ENTER_SODI :
		(idle_type == IDLE_TYPE_SO3) ? SPM_ENTER_SODI3 : -1;

	memset(&spm_d, 0, sizeof(struct spm_data));

	spm_opt |= (op_cond & MTK_IDLE_OPT_SLEEP_DPIDLE) ?
		PWR_OPT_SLEEP_DPIDLE : 0;
	spm_opt |= (op_cond & MTK_IDLE_OPT_XO_UFS_ON_OFF) ?
		PWR_OPT_XO_UFS_OFF : 0;
	spm_opt |= (op_cond & MTK_IDLE_OPT_CLKBUF_BBLPM) ?
		PWR_OPT_CLKBUF_ENTER_BBLPM : 0;
	spm_opt |= (op_cond & MTK_IDLE_OPT_VCORE_LP_MODE) ?
		PWR_OPT_VCORE_LP_MODE : 0;

	spm_d.u.suspend.spm_opt = spm_opt;

	ret = spm_to_sspm_command_async(cmd, &spm_d);
	if (ret < 0)
		printk_deferred("[name:spm&]%s: ret %d", __func__, ret);
}

void mtk_idle_power_pre_process_async_wait(int idle_type, unsigned int op_cond)
{
	int ret = 0;
	int cmd;

	cmd = (idle_type == IDLE_TYPE_DP) ? SPM_DPIDLE_ENTER :
		(idle_type == IDLE_TYPE_SO) ? SPM_ENTER_SODI :
		(idle_type == IDLE_TYPE_SO3) ? SPM_ENTER_SODI3 : -1;

	ret = spm_to_sspm_command_async_wait(cmd);
	if (ret < 0)
		printk_deferred("[name:spm&]%s: ret %d", __func__, ret);
}

void mtk_idle_power_post_process(int idle_type, unsigned int op_cond)
{
	int ret;
	struct spm_data spm_d;
	unsigned int spm_opt = 0;
	int cmd;

	cmd = (idle_type == IDLE_TYPE_DP) ? SPM_DPIDLE_LEAVE :
		(idle_type == IDLE_TYPE_SO) ? SPM_LEAVE_SODI :
		(idle_type == IDLE_TYPE_SO3) ? SPM_LEAVE_SODI3 : -1;

	memset(&spm_d, 0, sizeof(struct spm_data));

	spm_opt |= (op_cond & MTK_IDLE_OPT_SLEEP_DPIDLE) ?
		PWR_OPT_SLEEP_DPIDLE : 0;
	spm_opt |= (op_cond & MTK_IDLE_OPT_XO_UFS_ON_OFF) ?
		PWR_OPT_XO_UFS_OFF : 0;
	spm_opt |= (op_cond & MTK_IDLE_OPT_CLKBUF_BBLPM) ?
		PWR_OPT_CLKBUF_ENTER_BBLPM : 0;
	spm_opt |= (op_cond & MTK_IDLE_OPT_VCORE_LP_MODE) ?
		PWR_OPT_VCORE_LP_MODE : 0;

	spm_d.u.suspend.spm_opt = spm_opt;

	ret = spm_to_sspm_command_async(cmd, &spm_d);
	if (ret < 0)
		printk_deferred("[name:spm&]%s: ret %d", __func__, ret);
}

void mtk_idle_power_post_process_async_wait(int idle_type, unsigned int op_cond)
{
	int ret = 0;
	int cmd;

	cmd = (idle_type == IDLE_TYPE_DP) ? SPM_DPIDLE_LEAVE :
		(idle_type == IDLE_TYPE_SO) ? SPM_LEAVE_SODI :
		(idle_type == IDLE_TYPE_SO3) ? SPM_LEAVE_SODI3 : -1;

	ret = spm_to_sspm_command_async_wait(cmd);
	if (ret < 0)
		printk_deferred("[name:spm&]%s: ret %d", __func__, ret);
}

