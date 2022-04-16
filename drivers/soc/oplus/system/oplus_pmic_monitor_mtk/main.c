// SPDX-License-Identifier: GPL-2.0-only
/*===========================================================================
#  Copyright (c) 2021, OPPO Technologies Inc. All rights reserved.
===========================================================================

                              EDIT HISTORY

 when       who        what, where, why
 --------   ---        ----------------------------------------------------------
 06/24/21   Yang.Wang   Created file
=============================================================================*/
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/module.h>
#include "oplus_pmic_info_mtk.h"

/**********************************************/
static ssize_t pmic_history_magic_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	char page[16] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr)
		len += snprintf(&page[len], 16 - len, "NULL\n");

	else
		len += snprintf(&page[len], 16 - len, "%s\n", &(pmic_history_ptr->pmic_magic));

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(pmic_history_magic);

/**********************************************/

/**********************************************/
static ssize_t pmic_history_count_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	char page[16] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr)
		len += snprintf(&page[len], 16 - len, "NULL\n");

	else
		len += snprintf(&page[len], 16 - len, "%ld\n", pmic_history_ptr->log_count_bak);

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(pmic_history_count);

/**********************************************/

#if 0
/**********************************************
PG_SDN_STS1[0x18]
15 STRUP_EXT_PMIC_PG_STATUS				Shutdown PG status 0: Power not good  1: Power good
14 STRUP_VEMC_PG_STATUS						0: Power not good  1: Power good
13 STRUP_VSRAM_OTHERS_PG_STATUS             0: Power not good  1: Power good
12 STRUP_VSRAM_MD_PG_STATUS				    0: Power not good  1: Power good
11 STRUP_VPROC2_PG_STATUS					0: Power not good  1: Power good
10 STRUP_VSRAM_PROC2_PG_STATUS		        0: Power not good  1: Power good
9  STRUP_VPROC1_PG_STATUS					0: Power not good  1: Power good
8  STRUP_VSRAM_PROC1_PG_STATUS		        0: Power not good  1: Power good
7  STRUP_VAUD18_PG_STATUS					0: Power not good  1: Power good
6  STRUP_VUSB_PG_STATUS						0: Power not good  1: Power good
5  STRUP_VRF18_PG_STATUS					0: Power not good  1: Power good

**********************************************/
#define PG_SDN_STS0_MAX 16
static char *const PG_SDN_STS1_reason_str[] = {
	[0] = ":UNKNOW REASON",
	[1] = ":UNKNOW REASON",
	[2] = ":UNKNOW REASON",
	[3] = ":UNKNOW REASON",
	[4] = ":UNKNOW REASON",
	[5] = ":STRUP_VRF18_PG_STATUS Power good",
	[6] = ":STRUP_VUSB_PG_STATUS Power good",
	[7] = ":STRUP_VAUD18_PG_STATUS Power good",
	[8] = ":STRUP_VSRAM_PROC1_PG_STATUS Power good",
	[9] = ":STRUP_VPROC1_PG_STATUS  Power good",
	[10] = ":STRUP_VSRAM_PROC2_PG_STATUS Power good",
	[11] = ":STRUP_VPROC2_PG_STATUS Power good",
	[12] = ":STRUP_VSRAM_MD_PG_STATUS Power good",
	[13] = ":STRUP_VSRAM_OTHERS_PG_STATUS Power good",
	[14] = ":STRUP_VEMC_PG_STATUS Power good",
	[15] = ":STRUP_EXT_PMIC_PG_STATUS Power good",
};


/**********************************************
OC_SDN_STS0[0x1a]
15 STRUP_VXO22_OC_STATUS         Shutdown OC status 0: No OC  1: OC occurs
14 STRUP_VAUX18_OC_STATUS        0: No OC  1: OC occurs
13 STRUP_VCORE_OC_STATUS         0: No OC  1: OC occurs
12 STRUP_VGPU11_OC_STATUS        0: No OC  1: OC occurs
11 STRUP_VGPU12_OC_STATUS        0: No OC  1: OC occurs
10 STRUP_VPU_OC_STATUS           0: No OC  1: OC occurs
9  STRUP_VMODEM_OC_STATUS        0: No OC  1: OC occurs
8  STRUP_VS2_OC_STATUS           0: No OC  1: OC occurs
7  STRUP_VA09_OC_STATUS          0: No OC  1: OC occurs
6  STRUP_VA12_OC_STATUS          0: No OC  1: OC occurs
5  STRUP_VS1_OC_STATUS           0: No OC  1: OC occurs
4  STRUP_VRFCK_1_OC_STATUS       0: No OC  1: OC occurs
3  STRUP_VBBCK_OC_STATUS         0: No OC  1: OC occurs
2  STRUP_VUFS_OC_STATUS          0: No OC  1: OC occurs
1  STRUP_VIO18_OC_STATUS         0: No OC  1: OC occurs
0  STRUP_VM18_OC_STATUS          0: No OC  1: OC occurs
**********************************************/
#define OC_SDN_STS0_MAX 16
static char *const OC_SDN_STS0_reason_str[] = {
	[0] = ":STRUP_VM18_OC_STATUS    OC occurs",
	[1] = ":STRUP_VIO18_OC_STATUS   OC occurs",
	[2] = ":STRUP_VUFS_OC_STATUS    OC occurs",
	[3] = ":STRUP_VBBCK_OC_STATUS   OC occurs",
	[4] = ":STRUP_VRFCK_1_OC_STATUS OC occurs",
	[5] = ":STRUP_VS1_OC_STATUS     OC occurs",
	[6] = ":STRUP_VA12_OC_STATUS    OC occurs",
	[7] = ":STRUP_VA09_OC_STATUS    OC occurs",
	[8] = ":STRUP_VS2_OC_STATUS     OC occurs",
	[9] = ":STRUP_VMODEM_OC_STATUS  OC occurs",
	[10] = ":STRUP_VPU_OC_STATUS     OC occurs",
	[11] = ":STRUP_VGPU12_OC_STATUS  OC occurs",
	[12] = ":STRUP_VGPU11_OC_STATUS  OC occurs",
	[13] = ":STRUP_VCORE_OC_STATUS   OC occurs ",
	[14] = ":STRUP_VAUX18_OC_STATUS  OC occurs",
	[15] = ":STRUP_VXO22_OC_STATUS   OC occurs",
};

/**********************************************
BUCK_OC_SDN_STATUS[0x1430]  		 BUCK_TOP_OC_CON0
9 RG_BUCK_VPA_OS_SDN_STATUS  		 OC status condition 0: No OC  1: OC occurs
8 RG_BUCK_VS2_OS_SDN_STATUS  		 0: No OC  1: OC occurs
7 RG_BUCK_VS1_OS_SDN_STATUS  		 0: No OC  1: OC occurs
6 RG_BUCK_VPROC2_OS_SDN_STATUS   0: No OC  1: OC occurs
5 RG_BUCK_VPROC1_OS_SDN_STATUS   0: No OC  1: OC occurs
4 RG_BUCK_VMODEM_OS_SDN_STATUS   0: No OC  1: OC occurs
3 RG_BUCK_VGPU12_OS_SDN_STATUS   0: No OC  1: OC occurs
2 RG_BUCK_VGPU11_OS_SDN_STATUS   0: No OC  1: OC occurs
1 RG_BUCK_VCORE_OS_SDN_STATUS    0: No OC  1: OC occurs
0 RG_BUCK_VPU_OS_SDN_STATUS      0: No OC  1: OC occurs

**********************************************/
#define BUCK_OC_SDN_STATUS_MAX 10
static char *const BUCK_OC_SDN_STATUS_reason_str[] = {
	[0] = ":RG_BUCK_VPU_OS_SDN_STATUS    OC occurs",
	[1] = ":RG_BUCK_VCORE_OS_SDN_STATUS   OC occurs",
	[2] = ":RG_BUCK_VGPU11_OS_SDN_STATUS    OC occurs",
	[3] = ":RG_BUCK_VGPU12_OS_SDN_STATUS   OC occurs",
	[4] = ":RG_BUCK_VMODEM_OS_SDN_STATUS OC occurs",
	[5] = ":RG_BUCK_VPROC1_OS_SDN_STATUS     OC occurs",
	[6] = ":RG_BUCK_VPROC2_OS_SDN_STATUS    OC occurs",
	[7] = ":RG_BUCK_VS1_OS_SDN_STATUS    OC occurs",
	[8] = ":RG_BUCK_VS2_OS_SDN_STATUS     OC occurs",
	[9] = ":RG_BUCK_VPA_OS_SDN_STATUS  OC occurs",
};


/**********************************************
THERMALSTATUS[0x1e]
15 STRUP_THERMAL_STATUS 				Shutdown Thermal status 0: No Thermal  1:Thermal occurs
14 PMU_THERMAL_DEB						Debounced Thermal status
**********************************************/
#define THERMALSTATUS_MAX 2
static char *const THERMALSTATUS_reason_str[] = {
	[0] = ":PMU_THERMAL_DEB  Debounced Thermal status",
	[1] = ":STRUP_THERMAL_STATUS Thermal occurs",
};


/**********************************************
STRUP_CON4[0xa1a]
2  JUST_PWRKEY_RST							    Long pressed reset indicator
1  JUST_SMART_RST								Smart reset indicator
**********************************************/
#define STRUP_CON4_MAX 2
static char *const STRUP_CON4_reason_str[] = {
	[0] = ":JUST_SMART_RST Smart reset indicator",
	[1] = ":JUST_PWRKEY_RST Long pressed reset indicator",
};

/**********************************************
STRUP_CON12[0xa12]
4   RG_STRUP_LONG_PRESS_EXT_EN   Enables re-power-on scenario function 1'b0: Disable long_press re-power on case  1'b1: Enable long_press re-power on case
3:2 RG_STRUPO_LONG_PRESS_EXT_TD  Selects delay time   2'b00:0.5s   2'b01:1s  2'b10:2s
1:0 RG_STRUPO_LONG_PRESS_EXT_SEL Selects Re-Power-On Scenario   2'b00:Debounce Pwrkey   2'b01:After releasing pwrkey  2'b10:After releasing pwrkey and pressing pwrkey again
**********************************************/
#define STRUP_CON4_MAX 2
static char *const STRUP_CON4_reason_str[] = {
	[0] = ":RG_STRUPO_LONG_PRESS_EXT_SEL Selects Re-Power-On Scenario   2'b00:Debounce Pwrkey   2'b01:After releasing pwrkey  2'b10:After releasing pwrkey and pressing pwrkey again",
	[1] = ":RG_STRUPO_LONG_PRESS_EXT_SEL Selects Re-Power-On Scenario   2'b00:Debounce Pwrkey   2'b01:After releasing pwrkey  2'b10:After releasing pwrkey and pressing pwrkey again",
	[2] = ":RG_STRUPO_LONG_PRESS_EXT_TD  Selects delay time   2'b00:0.5s   2'b01:1s  2'b10:2s",
	[3] = ":RG_STRUPO_LONG_PRESS_EXT_TD  Selects delay time   2'b00:0.5s   2'b01:1s  2'b10:2s",
	[4] = ":RG_STRUP_LONG_PRESS_EXT_EN  Enable long_press re-power on case",
};

#endif

/**********************************************
[PMIC]POFFSTS[0xE]
14		STS_OVLO	   Power off for OVLO event
13		STS_PKSP	   PWRKEY short press
12		STS_KEYPWR	   Critical power is turned off during system on
11		STS_PUPSRC	   Power off for power source missing
10		STS_WDT 	   AP WDT
9		STS_DDLO	   DDLO occurs after system on
8		STS_BWDT	   Power off for BWDT
7		STS_NORMOFF    Power off for PWRHOLD clear
6		STS_PKEYLP	   Power off for power key(s) long press
5		STS_CRST	   Power off for cold reset
4		STS_WRST	   Power reset for warm reset
3		STS_THRDN	   Power off for thermal shutdown
2		STS_PSOC	   Power off for default on BUCK OC
1		STS_PGFAIL	   Power off for PWRGOOD failure
0		STS_UVLO	   Power off for UVLO event
**********************************************/
#define PON_POFF_REASON_MAX 15
static char *const pon_poff_reason_str[] = {
	[0] = ":Power off for UVLO event",
	[1] = ":Power off for PWRGOOD failure",
	[2] = ":Power off for default on BUCKOC",
	[3] = ":Power off for thermal shutdown",
	[4] = ":Power reset for warm reset",
	[5] = ":Power off for cold reset",
	[6] = ":Power off for powerkey(s) long press",
	[7] = ":Power off for PWRHOLD clear",
	[8] = ":Power off for BWDT",
	[9] = ":DDLO occurs after system on",
	[10] = ":APWDT",
	[11] = ":Power off for power source missing",
	[12] = ":Critical power is turned off during system on",
	[13] = ":PWRKEY short press",
	[14] = ":Power off for OVLO event",
};

static ssize_t poff_reason_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	char page[512] = {0};
	int len = 0;
	int bit = -1;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u64 pmic_history_count = 0;
	unsigned int cur_poff_reason, cur_poff_reason_bak;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 512 - len, "PMIC|0|0x00|0x0000|NULL\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	printk(KERN_INFO "pmic_history_count = %d\n", pmic_history_count);

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		cur_poff_reason = pmic_first_record.pmic_pon_poff_reason[0].poff_reason;

		if (!cur_poff_reason) {
			len += snprintf(&page[len], 512 - len, "PMIC|%d|0x%0x|:%s\n",
					pmic_device_index,
					cur_poff_reason,
					"Battery Loss");
			continue;
		}

		cur_poff_reason_bak = cur_poff_reason;

		while (0 != cur_poff_reason_bak) {
			cur_poff_reason_bak = cur_poff_reason_bak >> 1;
			bit++;
		}

		len += snprintf(&page[len], 512 - len, "PMIC|%d|0x%x|:%s\n",
				pmic_device_index,
				cur_poff_reason,
				pon_poff_reason_str[bit]);
		bit = -1;

	}

	memcpy(buf, page, len);
	return len;

}
pmic_info_attr_ro(poff_reason);
/**********************************************/


/**********************************************
PONSTS[0XC]
4 		STS_RBOOT		Power on for cold reset
3 		STS_SPAR	    Power on for SPAR event
2		STS_CHRIN 		Power on for charger insertion
1 		STS_RTCA		Power on for RTC alarm
0		STS_PWRKEY		Power on for PWREKY press
**********************************************/
#define PON_PON_REASON_MAX 5
static char *const pon_pon_fault_reason_str[] = {
	[0] = ":Power on for PWREKY press",
	[1] = ":Power on for RTC alarm",
	[2] = ":Power on for charger insertion",
	[3] = ":Power on for SPAR event",
	[4] = ":Power on for cold reset",
};

#define QPNP_WARM_SEQ BIT(6)

static ssize_t pon_reason_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	char page[512] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u8 pon_index = 0;
	u8 pon_pon_reason1_reg_vaule = 0;
	u64 pmic_history_count = 0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 512 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		pon_pon_reason1_reg_vaule =
			pmic_first_record.pmic_pon_poff_reason[0].pon_reason;

		len += snprintf(&page[len], 512 - len, "PMIC|%d|0x%02X| (1/%ld)\n",
				pmic_device_index,
				pon_pon_reason1_reg_vaule,
				pmic_history_count);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(pon_reason);
/**********************************************/


/**********************************************
PG_SDN_STS0[0x16]
15 STRUP_VXO22_PG_STATUS         Shutdown PG status 0: Power not good  1: Power good
14 STRUP_VAUX18_PG_STATUS        0: Power not good  1: Power good
13 STRUP_VCORE_PG_STATUS         0: Power not good  1: Power good
12 STRUP_VGPU11_PG_STATUS        0: Power not good  1: Power good
11 STRUP_VGPU12_PG_STATUS        0: Power not good  1: Power good
10 STRUP_VPU_PG_STATUS           0: Power not good  1: Power good
9  STRUP_VMODEM_PG_STATUS        0: Power not good  1: Power good
8  STRUP_VS2_PG_STATUS           0: Power not good  1: Power good
7  STRUP_VA09_PG_STATUS          0: Power not good  1: Power good
6  STRUP_VA12_PG_STATUS          0: Power not good  1: Power good
5  STRUP_VS1_PG_STATUS           0: Power not good  1: Power good
4  STRUP_VRFCK_1_PG_STATUS       0: Power not good  1: Power good
3  STRUP_VBBCK_PG_STATUS         0: Power not good  1: Power good
2  STRUP_VUFS_PG_STATUS          0: Power not good  1: Power good
1  STRUP_VIO18_PG_STATUS         0: Power not good  1: Power good
0  STRUP_VM18_PG_STATUS          0: Power not good  1: Power good
**********************************************/
#define PG_SDN_STS0_MAX 16
static char *const pg_sdn_sts0_reason_str[] = {
	[0] = ":STRUP_VM18_PG_STATUS Power good",
	[1] = ":STRUP_VIO18_PG_STATUS Power good",
	[2] = ":STRUP_VUFS_PG_STATUS Power good",
	[3] = ":STRUP_VBBCK_PG_STATUS Power good",
	[4] = ":STRUP_VRFCK_1_PG_STATUS Power good",
	[5] = ":STRUP_VS1_PG_STATUS Power good",
	[6] = ":STRUP_VA12_PG_STATUS Power good",
	[7] = ":STRUP_VA09_PG_STATUS Power good",
	[8] = ":STRUP_VS2_PG_STATUS Power good",
	[9] = ":STRUP_VMODEM_PG_STATUS Power good",
	[10] = ":STRUP_VPU_PG_STATUS Power good",
	[11] = ":STRUP_VGPU12_PG_STATUS Power good",
	[12] = ":STRUP_VGPU11_PG_STATUS Power good",
	[13] = ":STRUP_VCORE_PG_STATUS Power good",
	[14] = ":STRUP_VAUX18_PG_STATUS Power good",
	[15] = ":STRUP_VXO22_PG_STATUS Power good",
};

/**********************************************/
static ssize_t pg_sdn_sts0_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	char page[256] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u64 pmic_history_count = 0;
	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 256 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		len += snprintf(&page[len], 256 - len, "PMIC|%d|0x%08X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[0].pg_sdn_sts0);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(pg_sdn_sts0);


/**********************************************
PG_SDN_STS1[0x18]
15 STRUP_EXT_PMIC_PG_STATUS				Shutdown PG status 0: Power not good  1: Power good
14 STRUP_VEMC_PG_STATUS						0: Power not good  1: Power good
13 STRUP_VSRAM_OTHERS_PG_STATUS   			0: Power not good  1: Power good
12 STRUP_VSRAM_MD_PG_STATUS					0: Power not good  1: Power good
11 STRUP_VPROC2_PG_STATUS					0: Power not good  1: Power good
10 STRUP_VSRAM_PROC2_PG_STATUS				0: Power not good  1: Power good
9  STRUP_VPROC1_PG_STATUS					0: Power not good  1: Power good
8  STRUP_VSRAM_PROC1_PG_STATUS				0: Power not good  1: Power good
7  STRUP_VAUD18_PG_STATUS					0: Power not good  1: Power good
6  STRUP_VUSB_PG_STATUS						0: Power not good  1: Power good
5  STRUP_VRF18_PG_STATUS					0: Power not good  1: Power good

**********************************************/
#define PG_SDN_STS1_MAX 16
static char *const pg_sdn_sts1_reason_str[] = {
	[0] = ":UNKNOW REASON",
	[1] = ":UNKNOW REASON",
	[2] = ":UNKNOW REASON",
	[3] = ":UNKNOW REASON",
	[4] = ":UNKNOW REASON",
	[5] = ":STRUP_VRF18_PG_STATUS Power good",
	[6] = ":STRUP_VUSB_PG_STATUS Power good",
	[7] = ":STRUP_VAUD18_PG_STATUS Power good",
	[8] = ":STRUP_VSRAM_PROC1_PG_STATUS Power good",
	[9] = ":STRUP_VPROC1_PG_STATUS  Power good",
	[10] = ":STRUP_VSRAM_PROC2_PG_STATUS Power good",
	[11] = ":STRUP_VPROC2_PG_STATUS Power good",
	[12] = ":STRUP_VSRAM_MD_PG_STATUS Power good",
	[13] = ":STRUP_VSRAM_OTHERS_PG_STATUS Power good",
	[14] = ":STRUP_VEMC_PG_STATUS Power good",
	[15] = ":STRUP_EXT_PMIC_PG_STATUS Power good",
};

/**********************************************/
static ssize_t pg_sdn_sts1_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	char page[256] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u64 pmic_history_count = 0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 256 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		len += snprintf(&page[len], 256 - len, "PMIC|%d|0x%08X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[0].pg_sdn_sts1);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(pg_sdn_sts1);


/**********************************************
OC_SDN_STS0[0x1a]
15 STRUP_VXO22_OC_STATUS         Shutdown OC status 0: No OC  1: OC occurs
14 STRUP_VAUX18_OC_STATUS        0: No OC  1: OC occurs
13 STRUP_VCORE_OC_STATUS         0: No OC  1: OC occurs
12 STRUP_VGPU11_OC_STATUS        0: No OC  1: OC occurs
11 STRUP_VGPU12_OC_STATUS        0: No OC  1: OC occurs
10 STRUP_VPU_OC_STATUS           0: No OC  1: OC occurs
9  STRUP_VMODEM_OC_STATUS        0: No OC  1: OC occurs
8  STRUP_VS2_OC_STATUS           0: No OC  1: OC occurs
7  STRUP_VA09_OC_STATUS          0: No OC  1: OC occurs
6  STRUP_VA12_OC_STATUS          0: No OC  1: OC occurs
5  STRUP_VS1_OC_STATUS           0: No OC  1: OC occurs
4  STRUP_VRFCK_1_OC_STATUS       0: No OC  1: OC occurs
3  STRUP_VBBCK_OC_STATUS         0: No OC  1: OC occurs
2  STRUP_VUFS_OC_STATUS          0: No OC  1: OC occurs
1  STRUP_VIO18_OC_STATUS         0: No OC  1: OC occurs
0  STRUP_VM18_OC_STATUS          0: No OC  1: OC occurs
**********************************************/
#define OC_SDN_STS0_MAX 16
static char *const OC_SDN_STS0_reason_str[] = {
	[0] = ":STRUP_VM18_OC_STATUS    OC occurs",
	[1] = ":STRUP_VIO18_OC_STATUS   OC occurs",
	[2] = ":STRUP_VUFS_OC_STATUS    OC occurs",
	[3] = ":STRUP_VBBCK_OC_STATUS   OC occurs",
	[4] = ":STRUP_VRFCK_1_OC_STATUS OC occurs",
	[5] = ":STRUP_VS1_OC_STATUS     OC occurs",
	[6] = ":STRUP_VA12_OC_STATUS    OC occurs",
	[7] = ":STRUP_VA09_OC_STATUS    OC occurs",
	[8] = ":STRUP_VS2_OC_STATUS     OC occurs",
	[9] = ":STRUP_VMODEM_OC_STATUS  OC occurs",
	[10] = ":STRUP_VPU_OC_STATUS     OC occurs",
	[11] = ":STRUP_VGPU12_OC_STATUS  OC occurs",
	[12] = ":STRUP_VGPU11_OC_STATUS  OC occurs",
	[13] = ":STRUP_VCORE_OC_STATUS   OC occurs ",
	[14] = ":STRUP_VAUX18_OC_STATUS  OC occurs",
	[15] = ":STRUP_VXO22_OC_STATUS   OC occurs",
};

/**********************************************/
static ssize_t OC_SDN_STS0_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	char page[256] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u8 index = 0, pon_pon_reason1_reg_vaule = 0;
	u64 pmic_history_count = 0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 256 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		len += snprintf(&page[len], 256 - len, "PMIC|%d|0x%08X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[0].oc_sdn_sts0);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(OC_SDN_STS0);


/**********************************************
BUCK_OC_SDN_STATUS[0x1430]  		 BUCK_TOP_OC_CON0
9 RG_BUCK_VPA_OS_SDN_STATUS  		 OC status condition 0: No OC  1: OC occurs
8 RG_BUCK_VS2_OS_SDN_STATUS  		 0: No OC  1: OC occurs
7 RG_BUCK_VS1_OS_SDN_STATUS  		 0: No OC  1: OC occurs
6 RG_BUCK_VPROC2_OS_SDN_STATUS   0: No OC  1: OC occurs
5 RG_BUCK_VPROC1_OS_SDN_STATUS   0: No OC  1: OC occurs
4 RG_BUCK_VMODEM_OS_SDN_STATUS   0: No OC  1: OC occurs
3 RG_BUCK_VGPU12_OS_SDN_STATUS   0: No OC  1: OC occurs
2 RG_BUCK_VGPU11_OS_SDN_STATUS   0: No OC  1: OC occurs
1 RG_BUCK_VCORE_OS_SDN_STATUS    0: No OC  1: OC occurs
0 RG_BUCK_VPU_OS_SDN_STATUS      0: No OC  1: OC occurs

**********************************************/
#define BUCK_OC_SDN_STATUS_MAX 10
static char *const BUCK_OC_SDN_STATUS_reason_str[] = {
	[0] = ":RG_BUCK_VPU_OS_SDN_STATUS    OC occurs",
	[1] = ":RG_BUCK_VCORE_OS_SDN_STATUS   OC occurs",
	[2] = ":RG_BUCK_VGPU11_OS_SDN_STATUS    OC occurs",
	[3] = ":RG_BUCK_VGPU12_OS_SDN_STATUS   OC occurs",
	[4] = ":RG_BUCK_VMODEM_OS_SDN_STATUS OC occurs",
	[5] = ":RG_BUCK_VPROC1_OS_SDN_STATUS     OC occurs",
	[6] = ":RG_BUCK_VPROC2_OS_SDN_STATUS    OC occurs",
	[7] = ":RG_BUCK_VS1_OS_SDN_STATUS    OC occurs",
	[8] = ":RG_BUCK_VS2_OS_SDN_STATUS     OC occurs",
	[9] = ":RG_BUCK_VPA_OS_SDN_STATUS  OC occurs",
};


/**********************************************/
static ssize_t BUCK_OC_SDN_STATUS_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	char page[256] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u64 pmic_history_count = 0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 256 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		len += snprintf(&page[len], 256 - len, "PMIC|%d|0x%08X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[0].buck_oc_sdn_status);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(BUCK_OC_SDN_STATUS);


/**********************************************
THERMALSTATUS[0x1e]
15 STRUP_THERMAL_STATUS 				Shutdown Thermal status 0: No Thermal  1:Thermal occurs
14 PMU_THERMAL_DEB						Debounced Thermal status
**********************************************/
#define THERMALSTATUS_MAX 2
static char *const THERMALSTATUS_reason_str[] = {
	[0] = ":PMU_THERMAL_DEB  Debounced Thermal status",
	[1] = ":STRUP_THERMAL_STATUS Thermal occurs",
};


/**********************************************/
static ssize_t THERMALSTATUS_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	char page[256] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u64 pmic_history_count = 0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 256 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		len += snprintf(&page[len], 256 - len, "PMIC|%d|0x%08X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[0].thermalstatus);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(THERMALSTATUS);



/**********************************************
STRUP_CON4[0xa1a]
2  JUST_PWRKEY_RST							    Long pressed reset indicator
1  JUST_SMART_RST								Smart reset indicator
**********************************************/
#define STRUP_CON4_MAX 2
static char *const STRUP_CON4_reason_str[] = {
	[0] = ":JUST_SMART_RST Smart reset indicator",
	[1] = ":JUST_PWRKEY_RST Long pressed reset indicator",
};

/**********************************************/
static ssize_t STRUP_CON4_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	char page[256] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u64 pmic_history_count = 0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 256 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		len += snprintf(&page[len], 256 - len, "PMIC|%d|0x%08X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[0].strup_con4);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(STRUP_CON4);

/**********************************************/
static ssize_t TOP_RST_MISC_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	char page[256] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u64 pmic_history_count = 0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 256 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		len += snprintf(&page[len], 256 - len, "PMIC|%d|0x%08X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[0].top_rst_misc);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(TOP_RST_MISC);


/**********************************************/
static ssize_t TOP_CLK_TRIM_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	char page[256] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u64 pmic_history_count = 0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 256 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];

		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[0].data_is_valid)
			continue;

		len += snprintf(&page[len], 256 - len, "PMIC|%d|0x%08X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[0].top_clk_trim);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(TOP_CLK_TRIM);
#if 0
///no STRUP_CON12
/**********************************************
STRUP_CON12[0xa12]
4   RG_STRUP_LONG_PRESS_EXT_EN   Enables re-power-on scenario function 1'b0: Disable long_press re-power on case  1'b1: Enable long_press re-power on case
3:2 RG_STRUPO_LONG_PRESS_EXT_TD  Selects delay time   2'b00:0.5s   2'b01:1s  2'b10:2s
1:0 RG_STRUPO_LONG_PRESS_EXT_SEL Selects Re-Power-On Scenario   2'b00:Debounce Pwrkey   2'b01:After releasing pwrkey  2'b10:After releasing pwrkey and pressing pwrkey again
**********************************************/
#define STRUP_CON12_MAX 2
static char *const STRUP_CON12_reason_str[] = {
	[0] = ":RG_STRUPO_LONG_PRESS_EXT_SEL Selects Re-Power-On Scenario   2'b00:Debounce Pwrkey   2'b01:After releasing pwrkey  2'b10:After releasing pwrkey and pressing pwrkey again",
	[1] = ":RG_STRUPO_LONG_PRESS_EXT_SEL Selects Re-Power-On Scenario   2'b00:Debounce Pwrkey   2'b01:After releasing pwrkey  2'b10:After releasing pwrkey and pressing pwrkey again",
	[2] = ":RG_STRUPO_LONG_PRESS_EXT_TD  Selects delay time   2'b00:0.5s   2'b01:1s  2'b10:2s",
	[3] = ":RG_STRUPO_LONG_PRESS_EXT_TD  Selects delay time   2'b00:0.5s   2'b01:1s  2'b10:2s",
	[4] = ":RG_STRUP_LONG_PRESS_EXT_EN  Enable long_press re-power on case",
};
/**********************************************/
static ssize_t STRUP_CON12_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	char page[256] = {0};
	int len = 0;
	struct PMICHistoryKernelStruct *pmic_history_ptr = NULL;
	struct PMICRecordKernelStruct pmic_first_record;
	u8 pmic_device_index = 0;
	u8 index = 0, pon_pon_reason1_reg_vaule = 0;
	u64 pmic_history_count = 0;

	pmic_history_ptr = (struct PMICHistoryKernelStruct *)get_pmic_history();

	if (NULL == pmic_history_ptr) {
		len += snprintf(&page[len], 256 - len, "PMIC|0|0x000000000000\n");
		memcpy(buf, page, len);
		return len;
	}

	//pmic_first_record = pmic_history_ptr->pmic_record[0];
	pmic_history_count = pmic_history_ptr->log_count;

	for (pmic_device_index = 0; pmic_device_index < pmic_history_count;
			pmic_device_index++) {
		pmic_first_record = pmic_history_ptr->pmic_record[pmic_device_index];
		/*
		if (DATA_VALID_FLAG != pmic_first_record.pmic_pon_poff_reason[pmic_device_index].data_is_valid) {
			continue;
		}
		*/
		//pon_pon_reason1_reg_vaule = pmic_first_record.pmic_pon_poff_reason[pmic_device_index].pg_sdn_sts0;
		//index = ffs(pon_pon_reason1_reg_vaule);
		len += snprintf(&page[len], 256 - len, "PMIC|%d|0x%08X\n",
				pmic_device_index,
				pmic_first_record.pmic_pon_poff_reason[0].strup_con12);
	}

	memcpy(buf, page, len);
	return len;
}
pmic_info_attr_ro(STRUP_CON12);
//MTK end
#endif
/**********************************************/

static struct attribute *g[] = {
	&pmic_history_magic_attr.attr,
	&pmic_history_count_attr.attr,
	&poff_reason_attr.attr,
	&pon_reason_attr.attr,
	&pg_sdn_sts0_attr.attr,
	&pg_sdn_sts1_attr.attr,
	&OC_SDN_STS0_attr.attr,
	&BUCK_OC_SDN_STATUS_attr.attr,
	&THERMALSTATUS_attr.attr,
	&STRUP_CON4_attr.attr,
	&TOP_RST_MISC_attr.attr,
	&TOP_CLK_TRIM_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

struct kobject *pmic_info_kobj;

static int __init pmic_info_init(void)
{
	int error;

	pmic_info_kobj = kobject_create_and_add("pmic_info", NULL);

	if (!pmic_info_kobj)
		return -ENOMEM;

	error = sysfs_create_group(pmic_info_kobj, &attr_group);

	if (error)
		return error;

	printk(KERN_INFO "pmic_info_init ok \n");
	return 0;
}

core_initcall(pmic_info_init);

MODULE_LICENSE("GPL v2");


