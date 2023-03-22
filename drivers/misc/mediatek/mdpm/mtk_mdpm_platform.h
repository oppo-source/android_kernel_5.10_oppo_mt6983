/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */

#ifndef _MTK_MDPM_PLATFORM_H_
#define _MTK_MDPM_PLATFORM_H_

#ifdef DISABLE_DLPT_FEATURE
#define MD_POWER_METER_ENABLE 0
#else
#define MD_POWER_METER_ENABLE 1
#endif

#define GET_MD_SCEANRIO_BY_SHARE_MEMORY

#define MAX_MD1_POWER	4000	/* mW */
#define MAX_TX_POWER	3500	/* mW */

#define SHARE_REG_MASK 0xFFFF

#define DBM_TABLE_SIZE 2
#define DBM_SECTION_MASK	0x1F

#define END_GUARDING_PATERN (2)
#define DBM_RESERVE_OFFSET (128 - SHARE_MEM_BLOCK_NUM - END_GUARDING_PATERN)

#define DBM_GEN95_OFFSET (90 + DBM_RESERVE_OFFSET)

#define MDPM_SHARE_MEMORY_MASK		0xFFFFFFFF
#define MDPM_SHARE_MEMORY_SHIFT		0
#define VERSION_CHECK_VERSION_MASK	0xFFFF
#define VERSION_CHECK_VERSION_SHIFT	0
#define VERSION_CHECK_VALID_MASK	0x3
#define VERSION_CHECK_VALID_SHIFT	16
#define RF_HW_VERSION_MASK		0x3
#define RF_HW_VERSION_SHIFT		0
#define RF_HW_VALID_MASK		0x3
#define RF_HW_VALID_SHIFT		2

#define AP_MD_MDPM_VERSION		0x1001
#define VERSION_VALID			0x3
#define VERSION_INVALID			0x1
#define VERSION_INIT			0x0
#define RF_HW_VALID			0x3
#define RF_HW_INVALID			0x1
#define RF_HW_INIT			0x0

enum section_level_tbl {
	DBM_SECTION_1 = 0,
	DBM_SECTION_2 = 5,
	DBM_SECTION_3 = 10,
	DBM_SECTION_4 = 15,
	DBM_SECTION_5 = 20,
	DBM_SECTION_6 = 25,
	DBM_SECTION_7 = 32,
	DBM_SECTION_8 = 37,
	DBM_SECTION_9 = 42,
	DBM_SECTION_10 = 47,
	DBM_SECTION_11 = 52,
	DBM_SECTION_12 = 57,
	DBM_SECTION_NUM = 12
};

enum md_scenario {
	S_STANDBY = 0,
	S_2G_IDLE,
	S_2G_NON_IDLE,
	S_2G_NON_CONN,
	S_2G_CONN,
	S_C2K_DATALINK,
	S_C2K_SHDR,
	S_C2K_1X_TRAFFIC,
	S_C2K_PAGING,
	S_C2K_EVDO,
	S_C2K_1X,
	S_3G_TDD_PAGING,
	S_3G_TDD_TALKING,
	S_3G_TDD_DATALINK,
	S_3G_IDLE,
	S_3G_WCDMA_TALKING,
	S_3G_1C,
	S_3G_2C,
	S_3G_UL1_TALKING,
	S_3G_UL1_PAGING,
	S_3G_UL1_DATA_1C,
	S_3G_UL1_DATA_2C,
	S_4G_0D0U,
	S_4G_1CC,
	S_4G_2CC,
	S_4G_3CC,
	S_4G_4CC,
	S_4G_5CC,
	S_4G_3D1U,
	S_4G_3D2U,
	S_4G_4D1U,
	S_4G_4D2U,
	S_4G_4D3U,
	S_4G_POS,
	S_5G_1CC_2CC,
	S_5G_1CC_2CC_4G_4CC,
	S_5G_1CC_2CC_4G_1CC,
	S_5G_NR_0CC_1TG,
	S_5G_NR_0CC_2TG,
	S_5G_NR_0CC_3TG,
	S_5G_FR1_1CC_1TG,
	S_5G_FR1_2CC_1TG,
	S_5G_FR1_3_4CC_1TG,
	S_5G_FR1_1_4CC_2TG,
	S_5G_FR2_1CC_1TG,
	S_5G_FR2_2_4CC_1TG,
	S_5G_FR2_5_8CC_1TG,
	S_5G_FR2_1_8CC_2TG,
	S_5G_FR1_FR2_3TG,
	S_5G_FR1_FR2,
	S_5G_FR1_1CC_1TG_4G_1CC,
	S_5G_FR2_1CC_1TG_4G_1CC_NR_0CC,
	S_5G_FR2_2_4CC_1TG_4G_1CC,
	S_5G_FR1_FR2_4G,
	S_4G_POS_URGENT,
	S_4G_5G_POS_URGENT,
	SCENARIO_NUM
};

enum tx_rat_type {
	RAT_2G = 1,
	RAT_3G,
	RAT_3GTDD,
	RAT_4G,
	RAT_C2K,
	RAT_5G,
	RAT_MMW,
	RAT_NUM = RAT_MMW,
};

enum tx_power_table {
	TX_2G_DBM = 0,
	TX_3G_DBM,
	TX_3GTDD_DBM,
	TX_4G_CC0_DBM,
	TX_4G_CC1_DBM,
	TX_C2K_DBM,
	TX_NR_CC0_DBM,
	TX_NR_CC1_DBM,
	TX_MMW_TX1_DBM,
	TX_MMW_TX2_DBM,
	TX_DBM_NUM
};

enum share_mem_mapping {	/* each of 8 bytes */
	DBM_TABLE_2_START = 0,
	DBM_TABLE_2_END = 40,
	SECTION_LEVEL_2_START = 41,
	M_MMW_SECTION_LEVEL = 78,
	M_MMW_SECTION_1_LEVEL,
	M_MMW_SECTION_2_LEVEL,
	M_MMW_SECTION_3_LEVEL,
	SECTION_LEVEL_2_END = M_MMW_SECTION_3_LEVEL,
	DBM_TABLE_START,
	M_MMW_DBM_TABLE = DBM_TABLE_START,
	M_MMW_DBM_1_TABLE,
	M_2G_DBM_TABLE,
	M_3G_DBM_TABLE,
	M_4G_DBM_TABLE,
	M_4G_DBM_1_TABLE,
	M_4G_DBM_2_TABLE,
	M_4G_DBM_3_TABLE,
	M_NR_DBM_TABLE,
	M_NR_DBM_1_TABLE,
	M_NR_DBM_2_TABLE,
	M_NR_DBM_3_TABLE,
	M_MMW_DBM_2_TABLE,
	M_MMW_DBM_3_TABLE,
	M_4G_DBM_10_TABLE,
	M_2G_DBM_1_TABLE,
	M_3G_DBM_1_TABLE,
	M_TDD_DBM_TABLE,
	M_C2K_DBM_1_TABLE,
	M_C2K_DBM_2_TABLE,
	M_C2K_DBM_3_TABLE,
	M_TDD_DBM_1_TABLE,
	DBM_TABLE_END = M_TDD_DBM_1_TABLE,
	SECTION_LEVEL_START,
	M_2G_SECTION_LEVEL = SECTION_LEVEL_START,
	M_3G_SECTION_LEVEL,
	M_4G_SECTION_LEVEL,
	M_NR_SECTION_LEVEL,
	M_NR_SECTION_1_LEVEL,
	M_NR_SECTION_2_LEVEL,
	M_NR_SECTION_3_LEVEL,
	M_4G_SECTION_5_LEVEL,
	M_4G_SECTION_6_LEVEL,
	M_4G_SECTION_7_LEVEL,
	M_MD_SCENARIO,
	M_2G_SECTION_1_LEVEL,
	M_3G_SECTION_1_LEVEL,
	M_4G_SECTION_9_LEVEL,
	M_VERSION_CHECK,
	M_TDD_SECTION_LEVEL,
	M_C2K_SECTION_1_LEVEL,
	M_RF_HW,
	M_C2K_SECTION_2_LEVEL,
	M_TDD_SECTION_1_LEVEL,
	SECTION_LEVEL_END = M_TDD_SECTION_1_LEVEL,
	SHARE_MEM_BLOCK_NUM
};

#define MAX_DBM_FUNC_NUM 5
#define MAX_MDPM_NAME_LEN 32
#define RF_HW_NUM 2
#define SHARE_MEM_SIZE (SHARE_MEM_BLOCK_NUM)

struct md_power_status {
	char scenario_name[MAX_MDPM_NAME_LEN];
	enum md_scenario scenario_id;
	enum tx_rat_type rat;
	enum mdpm_power_type power_type;
	int dbm_section;
	int scanario_power;
	int rfhw_sel;
	int pa_power;
	int rf_power;
	int tx_power;
	int total_power;
};

struct tx_power_type_t {
	int max[DBM_SECTION_NUM];
	int avg[DBM_SECTION_NUM];
};

struct rfhw_power_t {
	struct tx_power_type_t pa_power;
	struct tx_power_type_t rf_power;
	int section[DBM_SECTION_NUM];

};

struct tx_power {
	char dbm_name[MAX_MDPM_NAME_LEN];
	enum share_mem_mapping shm_dbm_idx[DBM_TABLE_SIZE];
	enum share_mem_mapping shm_sec_idx[DBM_TABLE_SIZE];
	struct rfhw_power_t *rfhw;
};

struct scenario_power_type_t {
	int max;
	int avg;
};

struct mdpm_scenario {
	u32 scenario_reg;
	char scenario_name[MAX_MDPM_NAME_LEN];
	struct scenario_power_type_t *scenario_power;
	enum tx_rat_type tx_power_rat[MAX_DBM_FUNC_NUM];
	int tx_power_rat_sum;
	int (*tx_power_func)(u32 *dbm_mem, u32 *old_dbm_mem,
		enum tx_rat_type rat, enum mdpm_power_type power_type,
		struct md_power_status *md_power_s);
};

struct mdpm_data {
	struct device *dev;
	unsigned int platform;
	struct mdpm_scenario *scenario_power_t;
	struct tx_power *tx_power_t;
	void *prority_t;
};

extern void mt_mdpm_init(void);
extern void init_md1_section_level(u32 *share_mem);
extern void init_version_check(u32 *share_mem);
extern unsigned int get_md1_status_reg(void);
extern unsigned int get_md1_scenario(u32 share_reg,
	enum mdpm_power_type power_type);
extern int get_md1_scenario_power(unsigned int scenario,
	enum mdpm_power_type power_type, struct md_power_status *mdpm_pwr_sta);
extern int get_md1_tx_power(enum md_scenario scenario, u32 *share_mem,
	enum mdpm_power_type power_type, struct md_power_status *mdpm_pwr_sta);
#ifdef GET_MD_SCEANRIO_BY_SHARE_MEMORY
extern unsigned int get_md1_scenario_by_shm(u32 *share_mem);
#endif
#endif /* _MTK_MDPM_PLATFORM_H_ */
