// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/types.h>


/* local include */
#include "inc/mtk_cpufreq_api.h"
#include "mtk_unified_power.h"
#include "mtk_unified_power_data.h"
#include "mtk_devinfo.h"
#include "mtk_static_power.h"
#if IS_ENABLED(CONFIG_MTK_EEM_READY)
#include "mtk_eem.h"
#endif
#ifndef EARLY_PORTING_SPOWER
#include "mtk_common_static_power.h"
#endif

#undef  BIT
#define BIT(bit)	(1U << (bit))

#define MSB(range)	(1 ? range)
#define LSB(range)	(0 ? range)
/**
 * Genearte a mask wher MSB to LSB are all 0b1
 * @r:	Range in the form of MSB:LSB
 */
#define BITMASK(r)	\
	(((unsigned int) -1 >> (31 - MSB(r))) & ~((1U << LSB(r)) - 1))

/**
 * Set value at MSB:LSB. For example, BITS(7:3, 0x5A)
 * will return a value where bit 3 to bit 7 is 0x5A
 * @r:	Range in the form of MSB:LSB
 */
/* BITS(MSB:LSB, value) => Set value at MSB:LSB  */
#define BITS(r, val)	((val << LSB(r)) & BITMASK(r))

#define GET_BITS_VAL(_bits_, _val_)   \
	(((_val_) & (BITMASK(_bits_))) >> ((0) ? _bits_))
/* #if (NR_UPOWER_TBL_LIST <= 1) */
struct upower_tbl final_upower_tbl[NR_UPOWER_BANK] = {};
/* #endif */

int degree_set[NR_UPOWER_DEGREE] = {
		UPOWER_DEGREE_0,
		UPOWER_DEGREE_1,
		UPOWER_DEGREE_2,
		UPOWER_DEGREE_3,
		UPOWER_DEGREE_4,
		UPOWER_DEGREE_5,
};

/* collect all the raw tables */
#define INIT_UPOWER_TBL_INFOS(name, tbl) {__stringify(name), &tbl}
struct upower_tbl_info
	upower_tbl_infos_list[NR_UPOWER_TBL_LIST][NR_UPOWER_BANK] = {
	/* MT6768 */
	[0] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL,
				upower_tbl_l_6768),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L,
				upower_tbl_b_6768),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
				upower_tbl_cluster_l_6768),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
				upower_tbl_cluster_b_6768),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI,
				upower_tbl_cci_6768),
	},
	/* MT6767 */
	[1] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL,
				upower_tbl_l_6767),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L,
				upower_tbl_b_6767),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
				upower_tbl_cluster_l_6767),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
				upower_tbl_cluster_b_6767),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI,
				upower_tbl_cci_6767),
	},

	/* MT6768_PRO */
	[2] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL,
				upower_tbl_l_PRO),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L,
				upower_tbl_b_PRO),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
				upower_tbl_cluster_l_PRO),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
				upower_tbl_cluster_b_PRO),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI,
				upower_tbl_cci_PRO),
	},
	/* MT6768 */
	[3] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL,
				upower_tbl_l_6768_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L,
				upower_tbl_b_6768_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
				upower_tbl_cluster_l_6768_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
				upower_tbl_cluster_b_6768_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI,
				upower_tbl_cci_6768_v7),
	},
	/* MT6767 */
	[4] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL,
				upower_tbl_l_6767_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L,
				upower_tbl_b_6767_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
				upower_tbl_cluster_l_6767_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
				upower_tbl_cluster_b_6767_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI,
				upower_tbl_cci_6767_v7),
	},

	/* MT6768_PRO */
	[5] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL,
				upower_tbl_l_PRO_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L,
				upower_tbl_b_PRO_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
				upower_tbl_cluster_l_PRO_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
				upower_tbl_cluster_b_PRO_v7),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI,
				upower_tbl_cci_PRO_v7),
	},

	/* MT6768_G75 */
	[6] = {
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_LL,
				upower_tbl_l_G75),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_L,
				upower_tbl_b_G75),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_LL,
				upower_tbl_cluster_l_G75),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CLS_L,
				upower_tbl_cluster_b_G75),
		INIT_UPOWER_TBL_INFOS(UPOWER_BANK_CCI,
				upower_tbl_cci_G75),
	},


};
/* Upower will know how to apply voltage that comes from EEM */
unsigned char upower_recognize_by_eem[NR_UPOWER_BANK] = {
	UPOWER_BANK_LL, /* LL EEM apply voltage to LL upower bank */
	UPOWER_BANK_L, /* L EEM apply voltage to L upower bank */
	UPOWER_BANK_LL, /* LL EEM apply voltage to CLS_LL upower bank */
	UPOWER_BANK_L, /* L EEM apply voltage to CLS_L upower bank */
	UPOWER_BANK_CCI, /* CCI EEM apply voltage to CCI upower bank */
};

/* Used for rcu lock, points to all the raw tables list*/
struct upower_tbl_info *p_upower_tbl_infos = &upower_tbl_infos_list[0][0];

#ifndef EARLY_PORTING_SPOWER
int upower_bank_to_spower_bank(int upower_bank)
{
	int ret;

	switch (upower_bank) {
	case UPOWER_BANK_LL:
		ret = MTK_SPOWER_CPULL;
		break;
	case UPOWER_BANK_L:
		ret = MTK_SPOWER_CPUL;
		break;

	case UPOWER_BANK_CCI:
		ret = MTK_SPOWER_CCI;
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}
#endif

static void upower_scale_l_cap(void)
{
	unsigned int ratio;
	unsigned int temp;
	unsigned int max_cap = 1024;
	int i, j;
	struct upower_tbl *tbl;

	/* get L opp0's cap and calculate scaling ratio */
	/* ratio = round_up(1024 * 1000 / opp0 cap) */
	/* new cap = orig cap * ratio / 1000 */
	tbl = upower_tbl_infos[UPOWER_BANK_L].p_upower_tbl;
	temp = tbl->row[UPOWER_OPP_NUM - 1].cap;
	ratio = ((max_cap * 1000) + (temp - 1)) / temp;
	upower_error("scale ratio = %d, orig cap = %d\n", ratio, temp);

	/* if L opp0's cap is 1024, no need to scale cap anymore */
	if (temp == 1024)
		return;

	/* scaling L and cluster L cap value */
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		if ((i == UPOWER_BANK_L) || (i == UPOWER_BANK_CLS_L)) {
			tbl = upower_tbl_infos[i].p_upower_tbl;
			for (j = 0; j < UPOWER_OPP_NUM; j++) {
				temp = tbl->row[j].cap;
				tbl->row[j].cap = temp * ratio / 1000;
			}
		}
	}

	/* check opp0's cap after scaling */
	tbl = upower_tbl_infos[UPOWER_BANK_L].p_upower_tbl;
	temp = tbl->row[UPOWER_OPP_NUM - 1].cap;
	if (temp != 1024) {
		upower_error("Notice: new cap is not 1024 after scaling(%d)\n",
				ratio);
		tbl->row[UPOWER_OPP_NUM - 1].cap = 1024;
	}
}

int cpu_cluster_mapping(unsigned int cpu)
{
	enum upower_bank bank = UPOWER_BANK_LL;

	if (cpu < 6) /* cpu 0-3 */
		bank = UPOWER_BANK_LL;
	else if (cpu < 8) /* cpu 4-7 */
		bank = UPOWER_BANK_L;

	return bank;
}

/****************************************************
 * According to chip version get the raw upower tbl *
 * and let upower_tbl_infos points to it.           *
 * Choose a non used upower tbl location and let    *
 * upower_tbl_ref points to it to store target      *
 * power tbl.                                       *
 ***************************************************/

void get_original_table(void)
{
	unsigned short idx = 0; /* default use MT6771T_FY */
	int i, j;

	idx = mt_cpufreq_get_cpu_level();

	if (idx >= NR_UPOWER_TBL_LIST)
		idx = 0;

	/* get location of reference table */
	upower_tbl_infos = &upower_tbl_infos_list[idx][0];

	/* get location of target table */
/* #if (NR_UPOWER_TBL_LIST <= 1) */
	upower_tbl_ref = &final_upower_tbl[0];
/* #else
 *	upower_tbl_ref = upower_tbl_infos_list[(idx+1) %
 *	NR_UPOWER_TBL_LIST][0].p_upower_tbl;
 * #endif
 */

	upower_debug("idx %d dest:%p, src:%p\n",
			(idx+1)%NR_UPOWER_TBL_LIST,
			upower_tbl_ref, upower_tbl_infos);
	/* p_upower_tbl_infos = upower_tbl_infos; */

	/*
	 *  Clear volt fields before eem run.                                  *
	 *  If eem is enabled, it will apply volt into it. If eem is disabled, *
	 *  the values of volt are 0 , and upower will apply orig volt into it *
	 */
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		for (j = 0; j < UPOWER_OPP_NUM; j++)
			upower_tbl_ref[i].row[j].volt = 0;
	}
	for (i = 0; i < NR_UPOWER_BANK; i++)
		upower_debug("bank[%d] dest:%p dyn_pwr:%u, volt[0]%u\n",
				i, &upower_tbl_ref[i],
				upower_tbl_ref[i].row[0].dyn_pwr,
				upower_tbl_ref[i].row[0].volt);

	/* Not support L+ now, scale L and cluster L cap to 1024 */
	upower_scale_l_cap();
}
MODULE_DESCRIPTION("MediaTek Unified Power Driver v0.0");
MODULE_LICENSE("GPL");
