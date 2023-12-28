/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MT6763_THERMAL_H__
#define __MT6763_THERMAL_H__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <linux/io.h>
#include <linux/uaccess.h>

#if IS_ENABLED(CONFIG_MTK_GPU_SUPPORT)
#include "mtk_gpufreq.h"
#endif

/*=============================================================
 * LVTS SW Configs
 *=============================================================
 */
#define CFG_THERM_LVTS				(0)

#if CFG_THERM_LVTS
#define	CFG_LVTS_DOMINATOR			(1)
#define	LVTS_THERMAL_CONTROLLER_HW_FILTER	(1) /* 1, 2, 4, 8, 16 */
#else
#define	CFG_LVTS_DOMINATOR			(0)
#define	LVTS_THERMAL_CONTROLLER_HW_FILTER	(0)
#endif

/* public thermal sensor enum */
enum thermal_sensor {
	TS_MCU0 = 0,	/* TSMCU0 */
	TS_MCU1,	/* TSMCU1 */
	TS_MCU2,	/* TSMCU2 */
	TS_MCU3,	/* TSMCU3 */
	TS_MCU4,	/* TSMCU4 */
	TS_MCU5,	/* TSMCU5 */
	TS_MCU6,	/* TSMCU6 */
	TS_MCU7,	/* TSMCU7 */
	TS_MCU8,	/* TSMCU8 */
	TS_MCU9,	/* TSMCU9 */
#if CFG_THERM_LVTS
	TS_LVTS1_0,	/* LVTS1-0 */
	TS_LVTS1_1,	/* LVTS1-1 */
	TS_LVTS2_0,	/* LVTS2-0 */
	TS_LVTS2_1,	/* LVTS2-1 */
	TS_LVTS2_2,	/* LVTS2-2 */
	TS_LVTS3_0,	/* LVTS3-0 */
	TS_LVTS3_1,	/* LVTS3-1 */
	TS_LVTS4_0,	/* LVTS4-0 */
	TS_LVTS4_1,	/* LVTS4-1 */
	TS_LVTS9_0,	/* LVTS9-0 */
#endif
	TS_ABB,		/* ABB */
	TS_ENUM_MAX,
};

enum thermal_bank_name {
	THERMAL_BANK0 = 0,
	THERMAL_BANK1,
	THERMAL_BANK2,
	THERMAL_BANK3,
	THERMAL_BANK4,
	THERMAL_BANK5,
	THERMAL_BANK6,
	THERMAL_BANK7,
	THERMAL_BANK_NUM
};

struct TS_PTPOD {
	unsigned int ts_MTS;
	unsigned int ts_BTS;
};

#if !IS_ENABLED(CONFIG_MTK_GPU_SUPPORT)
struct mt_gpufreq_power_table_info {
	unsigned int gpufreq_khz;
	unsigned int gpufreq_volt;
	unsigned int gpufreq_power;
};
#endif

extern int mtktscpu_limited_dmips;

/* Valid if it returns 1, invalid if it returns 0. */
extern int tscpu_is_temp_valid(void);

extern void get_thermal_slope_intercept(
	struct TS_PTPOD *ts_info, enum thermal_bank_name ts_bank);

#if CFG_THERM_LVTS
extern void get_lvts_slope_intercept(
		struct TS_PTPOD *ts_info, enum thermal_bank_name ts_bank);
#endif

extern void set_taklking_flag(bool flag);

extern int tscpu_get_cpu_temp(void);

extern int tscpu_get_temp_by_bank(enum thermal_bank_name ts_bank);

#define THERMAL_WRAP_WR32(val, addr)	\
	mt_reg_sync_writel((val), ((void *)addr))

extern int get_immediate_cpuL_wrap(void);
extern int get_immediate_cpuB_wrap(void);
extern int get_immediate_mcucci_wrap(void);
extern int get_immediate_gpu_wrap(void);
extern int get_immediate_mdla_wrap(void);
extern int get_immediate_vpu_wrap(void);
extern int get_immediate_top_wrap(void);
extern int get_immediate_md_wrap(void);

/* Added for DLPT. */
extern int tscpu_get_min_cpu_pwr(void);

extern int tscpu_get_min_gpu_pwr(void);
extern int tscpu_get_min_vpu_pwr(void);
extern int tscpu_get_min_mdla_pwr(void);

/* Five thermal sensors. */
enum mtk_thermal_sensor_cpu_id_met {
	MTK_THERMAL_SENSOR_TS0 = 0,
	MTK_THERMAL_SENSOR_TS1,
	MTK_THERMAL_SENSOR_TS2,
	MTK_THERMAL_SENSOR_TS3,
	MTK_THERMAL_SENSOR_TS4,
	MTK_THERMAL_SENSOR_TS5,
	MTK_THERMAL_SENSOR_TS6,
	MTK_THERMAL_SENSOR_TS7,
	MTK_THERMAL_SENSOR_TS8,
	MTK_THERMAL_SENSOR_TS9,
#if CFG_THERM_LVTS
	MTK_THERMAL_SENSOR_LVTS1_0,	/* LVTS1-0 */
	MTK_THERMAL_SENSOR_LVTS1_1,	/* LVTS1-1 */
	MTK_THERMAL_SENSOR_LVTS2_0,	/* LVTS2-0 */
	MTK_THERMAL_SENSOR_LVTS2_1,	/* LVTS2-1 */
	MTK_THERMAL_SENSOR_LVTS2_2,	/* LVTS2-2 */
	MTK_THERMAL_SENSOR_LVTS3_0,	/* LVTS3-0 */
	MTK_THERMAL_SENSOR_LVTS3_1,	/* LVTS3-1 */
	MTK_THERMAL_SENSOR_LVTS4_0,	/* LVTS4-0 */
	MTK_THERMAL_SENSOR_LVTS4_1,	/* LVTS4-1 */
	MTK_THERMAL_SENSOR_LVTS9_0,	/* LVTS9-0 */
#endif
	MTK_THERMAL_SENSOR_TSABB,

	ATM_CPU_LIMIT,
	ATM_GPU_LIMIT,

	MTK_THERMAL_SENSOR_CPU_COUNT
};

extern int tscpu_get_cpu_temp_met(enum mtk_thermal_sensor_cpu_id_met id);

typedef void (*met_thermalsampler_funcMET)(void);

extern void mt_thermalsampler_registerCB(met_thermalsampler_funcMET pCB);

extern void mtkTTimer_cancel_timer(void);

extern void mtkTTimer_start_timer(void);

extern int mtkts_bts_get_hw_temp(void);

extern int get_immediate_ts0_wrap(void);

extern int get_immediate_ts1_wrap(void);

extern int get_immediate_ts2_wrap(void);

extern int get_immediate_ts3_wrap(void);

extern int get_immediate_ts4_wrap(void);

extern int get_immediate_ts5_wrap(void);

extern int get_immediate_ts6_wrap(void);

extern int get_immediate_ts7_wrap(void);

extern int get_immediate_ts8_wrap(void);

extern int get_immediate_ts9_wrap(void);

#if CFG_THERM_LVTS
extern int get_immediate_tslvts1_0_wrap(void);	/* LVTS1-0 */
extern int get_immediate_tslvts1_1_wrap(void);	/* LVTS1-1 */
extern int get_immediate_tslvts2_0_wrap(void);	/* LVTS2-0 */
extern int get_immediate_tslvts2_1_wrap(void);	/* LVTS2-1 */
extern int get_immediate_tslvts2_2_wrap(void);	/* LVTS2-2 */
extern int get_immediate_tslvts3_0_wrap(void);	/* LVTS3-0 */
extern int get_immediate_tslvts3_1_wrap(void);	/* LVTS3-1 */
extern int get_immediate_tslvts4_0_wrap(void);	/* LVTS4-0 */
extern int get_immediate_tslvts4_1_wrap(void);	/* LVTS4-1 */
extern int get_immediate_tslvts9_0_wrap(void);	/* LVTS9-0 */
#endif

extern int get_immediate_tsabb_wrap(void);

extern int (*get_immediate_tsX[TS_ENUM_MAX])(void);

extern int is_cpu_power_unlimit(void);	/* in mtk_ts_cpu.c */

extern int is_cpu_power_min(void);	/* in mtk_ts_cpu.c */

extern int get_cpu_target_tj(void);

extern int get_cpu_target_offset(void);

extern int mtk_gpufreq_register(
	struct mt_gpufreq_power_table_info *freqs, int num);

extern int get_target_tj(void);

extern int mtk_thermal_get_tpcb_target(void);

extern void thermal_set_big_core_speed(
unsigned int tempMonCtl1, unsigned int tempMonCtl2, unsigned int tempAhbPoll);

/*
 * return value(1): cooler of abcct/abcct_lcmoff is deactive,
 * and no thermal current limit.
 */
extern int mtk_cooler_is_abcct_unlimit(void);

#endif /* __MT6763_THERMAL_H__ */
