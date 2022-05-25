/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __MT6879_APUPWR_PROT_H__
#define __MT6879_APUPWR_PROT_H__

#include "apu_top.h"
#include "mt6879_apupwr.h"
// mbox offset define (for data exchange with remote)
#define SPARE_DBG_REG10		0x428	// mbox4_dummy10
#define SPARE_DBG_REG11		0x42C	// mbox4_dummy11
#define SPARE_DBG_REG12		0x430	// mbox4_dummy12
#define SPARE_DBG_REG13		0x434	// mbox4_dummy13
#define SPARE_DBG_REG14		0x438	// mbox4_dummy14
#define SPARE_DBG_REG15		0x43C	// mbox4_dummy15
#define SPARE_DBG_REG16		0x440	// mbox4_dummy16

#define ACX0_LIMIT_OPP_REG      SPARE_DBG_REG10
#define ACX1_LIMIT_OPP_REG      SPARE_DBG_REG11
#define DEV_OPP_SYNC_REG        SPARE_DBG_REG12
#define HW_RES_SYNC_REG         SPARE_DBG_REG13
#define PLAT_CFG_SYNC_REG	SPARE_DBG_REG14
#define DRV_CFG_SYNC_REG	SPARE_DBG_REG15

#define PWR_FLOW_SYNC_REG	SPARE_DBG_REG16

enum {
	APUPWR_DBG_DEV_CTL = 0,
	APUPWR_DBG_DEV_SET_OPP,
	APUPWR_DBG_DVFS_DEBUG,
	APUPWR_DBG_DUMP_OPP_TBL,
	APUPWR_DBG_CURR_STATUS,
	APUPWR_DBG_PROFILING,
};

enum apu_opp_limit_type {
	OPP_LIMIT_THERMAL = 0,	// limit by power API
	OPP_LIMIT_HAL,		// limit by i/o ctl
	OPP_LIMIT_DEBUG,	// limit by i/o ctl
};

struct drv_cfg_data {
	int8_t log_level;
	int8_t dvfs_debounce;	// debounce unit : ms
	int8_t disable_hw_meter;// 1: disable hw meter, bypass to read volt/freq
};

struct plat_cfg_data {
	int8_t aging_flag:4,
	       hw_id:4;
};

struct device_opp_limit {
	int8_t vpu_max:4,
	       vpu_min:4;
	int8_t dla_max:4,
	       dla_min:4;
	int8_t lmt_type; // limit reason
};

struct cluster_dev_opp_info {
	uint32_t opp_lmt_reg;
	struct device_opp_limit dev_opp_lmt;
};

struct hw_resource_status {
	int32_t vapu_opp:4,
		vsram_opp:4,
		vcore_opp:4,
		fconn_opp:4,
		fvpu_opp:4,
		fdla_opp:4,
		fup_opp:4,
		reserved:4;
};

/*
 * due to this struct will be used to do data exchange through rpmsg
 * so the struct size can't over than 256 bytes
 * 4 bytes * 14 struct members = 56 bytes
 */
struct apu_pwr_curr_info {
	int buck_volt[BUCK_NUM];
	int buck_opp[BUCK_NUM];
	int pll_freq[PLL_NUM];
	int pll_opp[PLL_NUM];
};

/*
 * for satisfy size limitation of rpmsg data exchange is 256 bytes
 * we only put necessary information for opp table here
 * opp entries : 4 bytes * 5 struct members * 10 opp entries = 200 bytes
 * tbl_size : 4 bytes
 * total : 200 + 4 = 204 bytes
 */
struct tiny_dvfs_opp_entry {
	int vapu;       // = volt_bin - volt_age + volt_avs
	int pll_freq[PLL_NUM];
};

struct tiny_dvfs_opp_tbl {
	int tbl_size;   // entry number
	struct tiny_dvfs_opp_entry opp[USER_MIN_OPP_VAL + 1];   // entry data
};

void mt6879_aputop_opp_limit(struct aputop_func_param *aputop,
		enum apu_opp_limit_type type);

#if IS_ENABLED(CONFIG_DEBUG_FS)
int mt6879_apu_top_dbg_open(struct inode *inode, struct file *file);
ssize_t mt6879_apu_top_dbg_write(
		struct file *flip, const char __user *buffer,
		size_t count, loff_t *f_pos);
#endif

int mt6879_init_remote_data_sync(void __iomem *reg_base);
int mt6879_drv_cfg_remote_sync(struct aputop_func_param *aputop);
int mt6879_chip_data_remote_sync(struct plat_cfg_data *plat_cfg);
int mt6879_apu_top_rpmsg_cb(int cmd, void *data, int len, void *priv, u32 src);
int mt6879_pwr_flow_remote_sync(uint32_t cfg);

#endif
