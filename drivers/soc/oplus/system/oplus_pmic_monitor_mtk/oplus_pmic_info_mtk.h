/* SPDX-License-Identifier: GPL-2.0-only */
/*===========================================================================
#  Copyright (c) 2021, OPPO Technologies Inc. All rights reserved.
===========================================================================

                              EDIT HISTORY

 when       who        what, where, why
 --------   ---        ----------------------------------------------------------
 06/24/21   Yang.Wang   Created file
=============================================================================*/
#ifndef __OPLUS_PMIC_INFO_MTK_H__
#define __OPLUS_PMIC_INFO_MTK_H__

#define PMIC_INFO_MAGIC 0x43494D504F50504F
#define DATA_VALID_FLAG 0xCC
#define MAX_HISTORY_COUNT 4

#define PMIC_INFO_SECTOR_START_UFS  (1161*4096)
#define PMIC_INFO_SECTOR_START_EMMC (9020*512)

typedef unsigned short uint16;

typedef enum {
	CHIP_CODE = 0,
	TOP_RST_STATUS,
	PONSTS,
	POFFSTS,
	PG_SDN_STS0,
	PG_SDN_STS1,
	OC_SDN_STS0,
	OC_SDN_STS1,
	BUCK_OC_SDN_STATUS,
	BUCK_OC_SDN_EN,
	THERMALSTATUS,
	STRUP_CON4,
	STRUP_CON12,
	TOP_RST_MISC,
	TOP_CLK_TRIM,
} pmic_regs_t;

struct PMICRegStruct {
	unsigned int data_is_valid;
	unsigned int chip_code;
	unsigned int top_rst_status;
	unsigned int pon_reason;
	unsigned int poff_reason;
	unsigned int pg_sdn_sts0;
	unsigned int pg_sdn_sts1;
	unsigned int oc_sdn_sts0;
	unsigned int oc_sdn_sts1;
	unsigned int buck_oc_sdn_status;
	unsigned int buck_oc_sdn_en;
	unsigned int thermalstatus;
	unsigned int strup_con4;
	unsigned int strup_con12;
	unsigned int top_rst_misc;
	unsigned int top_clk_trim;
};


struct PMICRecordKernelStruct {
	struct PMICRegStruct pmic_pon_poff_reason[1];//MAX_HISTORY_COUNT
};

struct PMICHistoryKernelStruct {
	long long unsigned int pmic_magic;
	unsigned int  log_count;
	unsigned int  chip_code;
	long long unsigned int log_count_bak;
	struct PMICRecordKernelStruct pmic_record[MAX_HISTORY_COUNT];
};

#define MAX_RECORD_LDO_NUM 32
#define MAX_RECORD_SPMS_NUM 8
#define MAX_RECORD_BOB_NUM 8

#define pmic_info_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define pmic_info_attr_ro(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = S_IRUGO,		\
	},					\
	.show	= _name##_show,			\
}

//PON_OFF_REASON |0x8C7
#define INDEX_RAW_XVDD_RB_OCCURED 		3
#define INDEX_RAW_DVDD_RB_OCCURED 		4
#define INDEX_IMMEDIATE_XVDD_SHUTDOWN	5
#define INDEX_S3_RESET					6
#define INDEX_FAULT_SEQ					7
#define INDEX_POFF_SEQ					8

//MTK start
#define INDEX_PON_OFF_REASON_STS_OVLO             14
#define INDEX_PON_OFF_REASON_STS_PKSP             13
#define INDEX_PON_OFF_REASON_STS_KEYPWR           12
#define INDEX_PON_OFF_REASON_STS_PUPSRC           11
#define INDEX_PON_OFF_REASON_STS_WDT              10
#define INDEX_PON_OFF_REASON_STS_DDLO             9
#define INDEX_PON_OFF_REASON_STS_BWDT             8
#define INDEX_PON_OFF_REASON_STS_NORMOFF          7
#define INDEX_PON_OFF_REASON_STS_PKEYLP           6
#define INDEX_PON_OFF_REASON_STS_CRST             5
#define INDEX_PON_OFF_REASON_STS_WRST             4
#define INDEX_PON_OFF_REASON_STS_THRDN            3
#define INDEX_PON_OFF_REASON_STS_PSOC             2
#define INDEX_PON_OFF_REASON_STS_PGFAIL           1
#define INDEX_PON_OFF_REASON_STS_UVLO             0



extern struct PMICHistoryKernelStruct *get_pmic_history(void);

#endif

