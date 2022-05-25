/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/ktime.h>

#include <swpm_module.h>
#include <swpm_module_ext.h>

#define SWPM_EXT_OPS (swpm_ext_m.plat_ops)

struct swpm_ext_manager {
	bool func_ready;
	struct swpm_internal_ops *plat_ops;
};

static struct swpm_ext_manager swpm_ext_m;

static int swpm_ops_func_ready_chk(void)
{
	bool func_ready = false;
	struct swpm_internal_ops *ops_chk = swpm_ext_m.plat_ops;

	if (ops_chk &&
	    ops_chk->cmd &&
	    ops_chk->ddr_act_times_get &&
	    ops_chk->ddr_sr_pd_times_get &&
	    ops_chk->ddr_freq_data_ip_stats_get &&
	    ops_chk->vcore_ip_vol_stats_get &&
	    ops_chk->vcore_vol_duration_get &&
	    ops_chk->num_get)
		func_ready = true;

	return func_ready;
}

int32_t sync_latest_data(void)
{
	if (swpm_ext_m.func_ready)
		SWPM_EXT_OPS->cmd(SYNC_DATA, 0);

	return 0;
}
EXPORT_SYMBOL(sync_latest_data);

int32_t get_ddr_act_times(int32_t freq_num,
			  struct ddr_act_times *ddr_times)
{
	if (swpm_ext_m.func_ready && ddr_times != NULL)
		SWPM_EXT_OPS->ddr_act_times_get(freq_num, ddr_times);

	return 0;
}
EXPORT_SYMBOL(get_ddr_act_times);

int32_t get_ddr_sr_pd_times(struct ddr_sr_pd_times *ddr_times)
{
	if (swpm_ext_m.func_ready && ddr_times != NULL)
		SWPM_EXT_OPS->ddr_sr_pd_times_get(ddr_times);

	return 0;
}
EXPORT_SYMBOL(get_ddr_sr_pd_times);

int32_t get_ddr_data_ip_num(void)
{
	if (swpm_ext_m.func_ready)
		return SWPM_EXT_OPS->num_get(DDR_DATA_IP);

	return 0;
}
EXPORT_SYMBOL(get_ddr_data_ip_num);

int32_t get_ddr_freq_num(void)
{
	if (swpm_ext_m.func_ready)
		return SWPM_EXT_OPS->num_get(DDR_FREQ);

	return 0;
}
EXPORT_SYMBOL(get_ddr_freq_num);

int32_t get_ddr_freq_data_ip_stats(int32_t data_ip_num,
				   int32_t freq_num,
				   void *stats)
{
	if (swpm_ext_m.func_ready)
		return SWPM_EXT_OPS->ddr_freq_data_ip_stats_get(data_ip_num,
			freq_num, stats);

	return 0;
}
EXPORT_SYMBOL(get_ddr_freq_data_ip_stats);

int32_t get_vcore_ip_num(void)
{
	if (swpm_ext_m.func_ready)
		return SWPM_EXT_OPS->num_get(CORE_IP);

	return 0;
}
EXPORT_SYMBOL(get_vcore_ip_num);

int32_t get_vcore_vol_num(void)
{
	if (swpm_ext_m.func_ready)
		return SWPM_EXT_OPS->num_get(CORE_VOL);

	return 0;
}
EXPORT_SYMBOL(get_vcore_vol_num);

int32_t get_vcore_ip_vol_stats(int32_t ip_num,
				int32_t vol_num,
				void *stats)
{
	if (swpm_ext_m.func_ready)
		return SWPM_EXT_OPS->vcore_ip_vol_stats_get(ip_num,
			vol_num, stats);

	return 0;
}
EXPORT_SYMBOL(get_vcore_ip_vol_stats);

int32_t get_vcore_vol_duration(int32_t vol_num,
			       struct vol_duration *duration)
{
	if (swpm_ext_m.func_ready && duration != NULL)
		return SWPM_EXT_OPS->vcore_vol_duration_get(vol_num,
							duration);

	return 0;
}
EXPORT_SYMBOL(get_vcore_vol_duration);

int mtk_register_swpm_ops(struct swpm_internal_ops *ops)
{
	if (!swpm_ext_m.plat_ops && ops) {
		swpm_ext_m.plat_ops = ops;
		swpm_ext_m.func_ready = swpm_ops_func_ready_chk();
	} else
		return -1;

	return 0;
}
EXPORT_SYMBOL(mtk_register_swpm_ops);
