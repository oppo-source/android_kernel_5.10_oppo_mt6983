// SPDX-License-Identifier: GPL-2.0-only
/*===========================================================================
#  Copyright (c) 2021, OPPO Technologies Inc. All rights reserved.
===========================================================================

                              EDIT HISTORY

 when       who        what, where, why
 --------   ---        ----------------------------------------------------------
 06/24/21   Yang.Wang   Created file
=============================================================================*/
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>
#include <linux/of.h>
#include "oplus_pmic_info_mtk.h"
#define PMIC_INFO_SECTOR_START_UFS  (1161*4096)
#define OPPO_PARTITION_OPPORESERVE_1                      "/dev/block/by-name/opporeserve1"
#define OPPO_PARTITION_OPLUSRESERVE_1                     "/dev/block/by-name/oplusreserve1"
#define OPPORESERVER1_PATH                               (!access(OPPO_PARTITION_OPPORESERVE_1, 0) ? OPPO_PARTITION_OPPORESERVE_1 : OPPO_PARTITION_OPLUSRESERVE_1 )
//#define OPPORESERVER1_PATH                                OPPO_PARTITION_OPLUSRESERVE_1

static struct PMICHistoryKernelStruct PMICHistory;
static struct PMICHistoryKernelStruct *format = NULL;

static int init_pmic_history_fdt(void)
{
	struct device_node *np = NULL;
	unsigned int log_count = 0;
	int ret = 0;
	char pmic_string[32] = {0};
	int i;
	struct PMICRegStruct *reg;

	np = of_find_node_by_name(NULL, "pmic_history_count");

	if (!np) {
		pr_err("init_pmic_history_fdt find node error\n");
		return -1;
	}

	ret = of_property_read_u32(np, "count", &log_count);

	if (ret) {
		pr_err("init_pmic_history_fdt count failed\n");
		return ret;
	}

	if (4 < log_count) {
		pr_err("init_pmic_history_fdt history count is incorrect!\n");
		return -1;
	}

	PMICHistory.log_count = log_count;

	for (i = 0; i < log_count; i++) {
		snprintf(pmic_string, 32, "pmic_history%u", i);
		np = of_find_node_by_name(NULL, pmic_string);

		if (!np) {
			pr_err("init_pmic_history_fdt don't fine node %s\n", pmic_string);
			return -1;
		}

		reg = PMICHistory.pmic_record[i].pmic_pon_poff_reason;

		ret = of_property_read_u32(np, "data_is_valid", &reg->data_is_valid);

		if (ret) {
			pr_err("%s don't find data_is_valid\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "chip_code", &reg->chip_code);

		if (ret) {
			pr_err("%s don't find poff_reason\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "pon_reason", &reg->pon_reason);

		if (ret) {
			pr_err("%s don't find pon_reason\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "poff_reason", &reg->poff_reason);

		if (ret) {
			pr_err("%s don't find poff_reason\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "pg_sdn_sts0", &reg->pg_sdn_sts0);

		if (ret) {
			pr_err("%s don't find pg_sdn_sts0\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "pg_sdn_sts1", &reg->pg_sdn_sts1);

		if (ret) {
			pr_err("%s don't find pg_sdn_sts1\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "oc_sdn_sts0", &reg->oc_sdn_sts0);

		if (ret) {
			pr_err("%s don't find oc_sdn_sts0\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "oc_sdn_sts1", &reg->oc_sdn_sts1);

		if (ret) {
			pr_err("%s don't find oc_sdn_sts1\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "buck_oc_sdn_status", &reg->buck_oc_sdn_status);

		if (ret) {
			pr_err("%s don't find buck_oc_sdn_status\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "buck_oc_sdn_en", &reg->buck_oc_sdn_en);

		if (ret) {
			pr_err("%s don't find buck_oc_sdn_en\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "thermalstatus", &reg->thermalstatus);

		if (ret) {
			pr_err("%s don't find thermalstatus\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "strup_con4", &reg->strup_con4);

		if (ret) {
			pr_err("%s don't find strup_con4\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "strup_con12", &reg->strup_con12);

		if (ret) {
			pr_err("%s don't find strup_con12\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "top_rst_misc", &reg->top_rst_misc);

		if (ret) {
			pr_err("%s don't find top_rst_misc\n", __func__);
			return ret;
		}

		ret = of_property_read_u32(np, "top_clk_trim", &reg->top_clk_trim);

		if (ret) {
			pr_err("%s don't find top_clk_trim\n", __func__);
			return ret;
		}
	}

	format = &PMICHistory;
	return 0;
}

struct PMICHistoryKernelStruct *get_pmic_history(void)
{
	if (format == NULL)
		init_pmic_history_fdt();

	return format;
}

MODULE_DESCRIPTION("OPLUS ocp status");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lee <wangyang4@oppo.com>");


