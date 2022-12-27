/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MFD_MT6357_CORE_H__
#define __MFD_MT6357_CORE_H__

#include <linux/kernel.h>

#define MT6357_REG_WIDTH 16

enum mt6357_irq_top_status_shift {
	MT6357_BUCK_TOP = 0,
	MT6357_LDO_TOP = 1,
	MT6357_PSC_TOP = 2,
	MT6357_SCK_TOP = 3,
	MT6357_BM_TOP = 4,
	MT6357_HK_TOP = 5,
	MT6357_AUD_TOP = 7,
	MT6357_MISC_TOP = 8,
};

enum mt6357_irq_numbers {
	MT6357_IRQ_VPROC_OC = 0,
	MT6357_IRQ_VCORE_OC = 1,
	MT6357_IRQ_VMODEM_OC = 2,
	MT6357_IRQ_VS1_OC = 3,
	MT6357_IRQ_VPA_OC = 4,
	MT6357_IRQ_VCORE_PREOC = 5,
	MT6357_IRQ_VFE28_OC = 16,
	MT6357_IRQ_VXO22_OC = 17,
	MT6357_IRQ_VRF18_OC = 18,
	MT6357_IRQ_VRF12_OC = 19,
	MT6357_IRQ_VEFUSE_OC = 20,
	MT6357_IRQ_VCN33_OC = 21,
	MT6357_IRQ_VCN28_OC = 22,
	MT6357_IRQ_VCN18_OC = 23,
	MT6357_IRQ_VCAMA_OC = 24,
	MT6357_IRQ_VCAMD_OC = 25,
	MT6357_IRQ_VCAMIO_OC = 26,
	MT6357_IRQ_VLDO28_OC = 27,
	MT6357_IRQ_VUSB33_OC = 28,
	MT6357_IRQ_VAUX18_OC = 29,
	MT6357_IRQ_VAUD28_OC = 30,
	MT6357_IRQ_VIO28_OC = 31,
	MT6357_IRQ_VIO18_OC = 32,
	MT6357_IRQ_VSRAM_PROC_OC = 33,
	MT6357_IRQ_VSRAM_OTHERS_OC = 34,
	MT6357_IRQ_VIBR_OC = 35,
	MT6357_IRQ_VDRAM_OC = 36,
	MT6357_IRQ_VMC_OC = 37,
	MT6357_IRQ_VMCH_OC = 38,
	MT6357_IRQ_VEMC_OC = 39,
	MT6357_IRQ_VSIM1_OC = 40,
	MT6357_IRQ_VSIM2_OC = 41,
	MT6357_IRQ_PWRKEY = 48,
	MT6357_IRQ_HOMEKEY = 49,
	MT6357_IRQ_PWRKEY_R = 50,
	MT6357_IRQ_HOMEKEY_R = 51,
	MT6357_IRQ_NI_LBAT_INT = 52,
	MT6357_IRQ_CHRDET = 53,
	MT6357_IRQ_CHRDET_EDGE = 54,
	MT6357_IRQ_VCDT_HV_DET = 55,
	MT6357_IRQ_WATCHDOG = 58,
	MT6357_IRQ_VBATON_UNDET = 59,
	MT6357_IRQ_BVALID_DET = 60,
	MT6357_IRQ_OV = 61,
	MT6357_IRQ_RTC = 64,
	MT6357_IRQ_FG_BAT0_H = 80,
	MT6357_IRQ_FG_BAT0_L = 81,
	MT6357_IRQ_FG_CUR_H = 82,
	MT6357_IRQ_FG_CUR_L = 83,
	MT6357_IRQ_FG_ZCV = 84,
	MT6357_IRQ_BATON_LV = 96,
	MT6357_IRQ_BATON_HT = 97,
	MT6357_IRQ_BAT_H = 114,
	MT6357_IRQ_BAT_L = 115,
	MT6357_IRQ_AUXADC_IMP = 120,
	MT6357_IRQ_NAG_C_DLTV = 121,
	MT6357_IRQ_AUDIO = 128,
	MT6357_IRQ_ACCDET = 133,
	MT6357_IRQ_ACCDET_EINT0 = 134,
	MT6357_IRQ_ACCDET_EINT1 = 135,
	MT6357_IRQ_SPI_CMD_ALERT = 144,
	MT6357_IRQ_NR = 145,
};

#define MT6357_IRQ_BUCK_BASE ALIGN_DOWN(MT6357_IRQ_VPROC_OC, MT6357_REG_WIDTH)
#define MT6357_IRQ_LDO_BASE ALIGN_DOWN(MT6357_IRQ_VFE28_OC, MT6357_REG_WIDTH)
#define MT6357_IRQ_PSC_BASE ALIGN_DOWN(MT6357_IRQ_PWRKEY, MT6357_REG_WIDTH)
#define MT6357_IRQ_SCK_BASE ALIGN_DOWN(MT6357_IRQ_RTC, MT6357_REG_WIDTH)
#define MT6357_IRQ_BM_BASE ALIGN_DOWN(MT6357_IRQ_FG_BAT0_H, MT6357_REG_WIDTH)
#define MT6357_IRQ_HK_BASE ALIGN_DOWN(MT6357_IRQ_BAT_H, MT6357_REG_WIDTH)
#define MT6357_IRQ_AUD_BASE ALIGN_DOWN(MT6357_IRQ_AUDIO, MT6357_REG_WIDTH)
#define MT6357_IRQ_MISC_BASE \
	ALIGN_DOWN(MT6357_IRQ_SPI_CMD_ALERT, MT6357_REG_WIDTH)

#define MT6357_IRQ_BUCK_BITS (MT6357_IRQ_VCORE_PREOC - MT6357_IRQ_BUCK_BASE + 1)
#define MT6357_IRQ_LDO_BITS (MT6357_IRQ_VSIM2_OC - MT6357_IRQ_LDO_BASE + 1)
#define MT6357_IRQ_PSC_BITS (MT6357_IRQ_OV - MT6357_IRQ_PSC_BASE + 1)
#define MT6357_IRQ_SCK_BITS (MT6357_IRQ_RTC - MT6357_IRQ_SCK_BASE + 1)
#define MT6357_IRQ_BM_BITS (MT6357_IRQ_BATON_HT - MT6357_IRQ_BM_BASE + 1)
#define MT6357_IRQ_HK_BITS (MT6357_IRQ_NAG_C_DLTV - MT6357_IRQ_HK_BASE + 1)
#define MT6357_IRQ_AUD_BITS (MT6357_IRQ_ACCDET_EINT1 - MT6357_IRQ_AUD_BASE + 1)
#define MT6357_IRQ_MISC_BITS \
	(MT6357_IRQ_SPI_CMD_ALERT - MT6357_IRQ_MISC_BASE + 1)

#define MT6357_TOP_GEN(sp)	\
{	\
	.hwirq_base = MT6357_IRQ_##sp##_BASE,	\
	.num_int_regs =	\
		(MT6357_IRQ_##sp##_BITS / MT6357_REG_WIDTH) + 1,	\
	.en_reg = MT6357_##sp##_TOP_INT_CON0,		\
	.en_reg_shift = 0x6,	\
	.sta_reg = MT6357_##sp##_TOP_INT_STATUS0,		\
	.sta_reg_shift = 0x2,	\
	.top_offset = MT6357_##sp##_TOP,	\
}

#define MT6357_IRQ_NAME_GEN()	\
{	\
	[MT6357_IRQ_VPROC_OC] = {.name = "vproc_oc"},	\
	[MT6357_IRQ_VCORE_OC] = {.name = "vcore_oc"},	\
	[MT6357_IRQ_VMODEM_OC] = {.name = "vmodem_oc"},	\
	[MT6357_IRQ_VS1_OC] = {.name = "vs1_oc"},	\
	[MT6357_IRQ_VPA_OC] = {.name = "vpa_oc"},	\
	[MT6357_IRQ_VCORE_PREOC] = {.name = "vcore_preoc"},	\
	[MT6357_IRQ_VFE28_OC] = {.name = "vfe28_oc"},	\
	[MT6357_IRQ_VXO22_OC] = {.name = "vxo22_oc"},	\
	[MT6357_IRQ_VRF18_OC] = {.name = "vrf18_oc"},	\
	[MT6357_IRQ_VRF12_OC] = {.name = "vrf12_oc"},	\
	[MT6357_IRQ_VEFUSE_OC] = {.name = "vefuse_oc"},	\
	[MT6357_IRQ_VCN33_OC] = {.name = "vcn33_oc"},	\
	[MT6357_IRQ_VCN28_OC] = {.name = "vcn28_oc"},	\
	[MT6357_IRQ_VCN18_OC] = {.name = "vcn18_oc"},	\
	[MT6357_IRQ_VCAMA_OC] = {.name = "vcama_oc"},	\
	[MT6357_IRQ_VCAMD_OC] = {.name = "vcamd_oc"},	\
	[MT6357_IRQ_VCAMIO_OC] = {.name = "vcamio_oc"},	\
	[MT6357_IRQ_VLDO28_OC] = {.name = "vldo28_oc"},	\
	[MT6357_IRQ_VUSB33_OC] = {.name = "vusb33_oc"},	\
	[MT6357_IRQ_VAUX18_OC] = {.name = "vaux18_oc"},	\
	[MT6357_IRQ_VAUD28_OC] = {.name = "vaud28_oc"},	\
	[MT6357_IRQ_VIO28_OC] = {.name = "vio28_oc"},	\
	[MT6357_IRQ_VIO18_OC] = {.name = "vio18_oc"},	\
	[MT6357_IRQ_VSRAM_PROC_OC] = {.name = "vsram_proc_oc"},	\
	[MT6357_IRQ_VSRAM_OTHERS_OC] = {.name = "vsram_others_oc"},	\
	[MT6357_IRQ_VIBR_OC] = {.name = "vibr_oc"},	\
	[MT6357_IRQ_VDRAM_OC] = {.name = "vdram_oc"},	\
	[MT6357_IRQ_VMC_OC] = {.name = "vmc_oc"},	\
	[MT6357_IRQ_VMCH_OC] = {.name = "vmch_oc"},	\
	[MT6357_IRQ_VEMC_OC] = {.name = "vemc_oc"},	\
	[MT6357_IRQ_VSIM1_OC] = {.name = "vsim1_oc"},	\
	[MT6357_IRQ_VSIM2_OC] = {.name = "vsim2_oc"},	\
	[MT6357_IRQ_PWRKEY] = {.name = "pwrkey"},	\
	[MT6357_IRQ_HOMEKEY] = {.name = "homekey"},	\
	[MT6357_IRQ_PWRKEY_R] = {.name = "pwrkey_r"},	\
	[MT6357_IRQ_HOMEKEY_R] = {.name = "homekey_r"},	\
	[MT6357_IRQ_NI_LBAT_INT] = {.name = "ni_lbat_int"},	\
	[MT6357_IRQ_CHRDET] = {.name = "chrdet"},	\
	[MT6357_IRQ_CHRDET_EDGE] = {.name = "chrdet_edge"},	\
	[MT6357_IRQ_VCDT_HV_DET] = {.name = "vcdt_hv_det"},	\
	[MT6357_IRQ_WATCHDOG] = {.name = "watchdog"},	\
	[MT6357_IRQ_VBATON_UNDET] = {.name = "vbaton_undet"},	\
	[MT6357_IRQ_BVALID_DET] = {.name = "bvalid_det"},	\
	[MT6357_IRQ_OV] = {.name = "ov"},	\
	[MT6357_IRQ_RTC] = {.name = "rtc"},	\
	[MT6357_IRQ_FG_BAT0_H] = {.name = "fg_bat0_h"},	\
	[MT6357_IRQ_FG_BAT0_L] = {.name = "fg_bat0_l"},	\
	[MT6357_IRQ_FG_CUR_H] = {.name = "fg_cur_h"},	\
	[MT6357_IRQ_FG_CUR_L] = {.name = "fg_cur_l"},	\
	[MT6357_IRQ_FG_ZCV] = {.name = "fg_zcv"},	\
	[MT6357_IRQ_BATON_LV] = {.name = "baton_lv"},	\
	[MT6357_IRQ_BATON_HT] = {.name = "baton_ht"},	\
	[MT6357_IRQ_BAT_H] = {.name = "bat_h"},	\
	[MT6357_IRQ_BAT_L] = {.name = "bat_l"},	\
	[MT6357_IRQ_AUXADC_IMP] = {.name = "auxadc_imp"},	\
	[MT6357_IRQ_NAG_C_DLTV] = {.name = "nag_c_dltv"},	\
	[MT6357_IRQ_AUDIO] = {.name = "audio"},	\
	[MT6357_IRQ_ACCDET] = {.name = "accdet"},	\
	[MT6357_IRQ_ACCDET_EINT0] = {.name = "accdet_eint0"},	\
	[MT6357_IRQ_ACCDET_EINT1] = {.name = "accdet_eint1"},	\
	[MT6357_IRQ_SPI_CMD_ALERT] = {.name = "spi_cmd_alert"},	\
}

#endif /* __MFD_MT6357_CORE_H__ */
