/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2017 MediaTek Inc.
 */

#ifndef __MTK_SPOWER_DATA_H__
#define __MTK_SPOWER_DATA_H__

#include "mtk_static_power.h"

struct spower_raw_t {
	int vsize;
	int tsize;
	int table_size;
	int *table[3];
	unsigned int devinfo_domain;
	unsigned int spower_id;
	unsigned int leakage_id;
	unsigned int instance;
	bool print_leakage;
};

#if IS_ENABLED(CONFIG_MTK_PLAT_POWER_MT6765)
#include "mtk_spower_data_6765.h"
#else
#include "mtk_spower_data_plat.h"
#endif



struct voltage_row_s {
	int mV[VSIZE];
};

struct temperature_row_s {
	int deg;
	int mA[VSIZE];
};

struct sptab_s {
	int vsize;
	int tsize;
	int *data;	/* array[VSIZE + TSIZE + (VSIZE*TSIZE)] */
	struct voltage_row_s *vrow;	/* pointer to voltage row of data */
	struct temperature_row_s *trow;	/* pointer to temperature row of data */
	unsigned int devinfo_domain;
	unsigned int spower_id;
	unsigned int leakage_id;
	unsigned int instance;
	bool print_leakage;
};

struct sptab_list {
	struct sptab_s tab_raw[MAX_TABLE_SIZE];
};

#define trow(tab, ti)		((tab)->trow[ti])
#define mA(tab, vi, ti)		((tab)->trow[ti].mA[vi])
#define mV(tab, vi)		((tab)->vrow[0].mV[vi])
#define deg(tab, ti)		((tab)->trow[ti].deg)
#define vsize(tab)		((tab)->vsize)
#define tsize(tab)		((tab)->tsize)
#define tab_validate(tab)	(!!(tab) && (tab)->data != NULL)

static inline void spower_tab_construct(struct sptab_s *tab,
				struct spower_raw_t *raw, unsigned int id)
{
	int i;
	struct sptab_s *ptab = (struct sptab_s *)tab;

	for (i = 0; i < raw->table_size; i++) {
		ptab->vsize = raw->vsize;
		ptab->tsize = raw->tsize;
		ptab->data = raw->table[i];
		ptab->vrow = (struct voltage_row_s *)ptab->data;
		ptab->trow = (struct temperature_row_s *)
				(ptab->data + ptab->vsize);
		ptab->devinfo_domain = raw->devinfo_domain;
		ptab->spower_id = id;
		ptab->leakage_id = raw->leakage_id;
		ptab->instance = raw->instance;
		ptab->print_leakage = raw->print_leakage;
		ptab++;
	}
}

#endif

