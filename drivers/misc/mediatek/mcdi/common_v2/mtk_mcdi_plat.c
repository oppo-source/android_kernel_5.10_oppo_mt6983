// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 MediaTek Inc.
 */
#include <linux/bug.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/cpuidle.h>

#include "mtk_mcdi_governor.h"
#include "mcdi_v1/mtk_mcdi_util.h"
#include "mtk_mcdi_plat.h"

void __iomem *cpc_base;

static unsigned int dt_init_sta;
static const char mcdi_node_name[] = "mediatek,mt6765-mcdi";

static int cluster_idx_map[NF_CPU] = {
	0,
	0,
	0,
	0,
	1,
	1,
	1,
	1
};

static int cpu_type_idx_map[NF_CPU] = {
	CPU_TYPE_L,
	CPU_TYPE_L,
	CPU_TYPE_L,
	CPU_TYPE_L,
	CPU_TYPE_LL,
	CPU_TYPE_LL,
	CPU_TYPE_LL,
	CPU_TYPE_LL
};

static unsigned int cpu_cluster_pwr_stat_map[NF_PWR_STAT_MAP_TYPE][NF_CPU] = {
	[ALL_CPU_IN_CLUSTER] = {
		0x000F,     /* Cluster 0 */
		0x00F0,     /* Cluster 1 */
		0x0000,     /* N/A */
		0x0000,
		0x0000,
		0x0000,
		0x0000,
		0x0000
	},
	[CPU_CLUSTER] = {
		0x200FE,     /* Only CPU 0 */
		0x200FD,     /* Only CPU 1 */
		0x200FB,
		0x200F7,
		0x100EF,
		0x100DF,
		0x100BF,
		0x1007F      /* Only CPU 7 */
	},
	[CPU_IN_OTHER_CLUSTER] = {
		0x000F0,
		0x000F0,
		0x000F0,
		0x000F0,
		0x0000F,
		0x0000F,
		0x0000F,
		0x0000F
	}
};

unsigned int get_pwr_stat_check_map(int type, int idx)
{
	if (!(type >= 0 && type < NF_PWR_STAT_MAP_TYPE))
		return 0;

	if (cpu_is_invalid(idx))
		return 0;

	return cpu_cluster_pwr_stat_map[type][idx];
}

int cluster_idx_get(int cpu)
{
	if (cpu_is_invalid(cpu)) {
		WARN_ON(cpu_is_invalid(cpu));

		return 0;
	}

	return cluster_idx_map[cpu];
}

int cpu_type_idx_get(int cpu)
{
	if (unlikely(cpu_is_invalid(cpu)))
		return 0;

	return cpu_type_idx_map[cpu];
}

void mcdi_status_init(void)
{
	if (cpc_base)
		set_mcdi_enable_status(!!dt_init_sta);
}

void mcdi_of_init(void **base)
{
	struct device_node *node = NULL;

	if (base == NULL)
		return;

	/* MCDI sysram base */
	node = of_find_compatible_node(NULL, NULL, mcdi_node_name);

	if (!node) {
		pr_info("node '%s' not found!\n", mcdi_node_name);
		cpc_base = NULL;
		return;
	}

	*base = of_iomap(node, 0);

	if (!*base)
		pr_info("node '%s' can not iomap!\n", mcdi_node_name);

	cpc_base = of_iomap(node, 1);

	if (!cpc_base)
		pr_info("node cpc_base can not iomap!\n");

	pr_info("mcdi_sysram_base = %p, cpc_base = %p\n",
			*base,
			cpc_base);

	/* The state value is modified only if the property exists */
	of_property_read_u32(node, "mediatek,enabled", &dt_init_sta);

}
