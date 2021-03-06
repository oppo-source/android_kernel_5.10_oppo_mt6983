// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


/* This file is generated by GenLP_setting.pl v1.5.7 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
/*FIXME 6368 coda update*/
const unsigned int AP_PMIC_REG_MT6368_gs_suspend_32kless_data[] = {
/*  Address     Mask        Golden Setting Value */
	0x16, 0xA, 0xA,/* TOP_CON */
	0x19, 0x1F, 0x1F,/* TEST_CON0 */
	0x21, 0x1, 0x1,/* SMT_CON0 */
	0x10F, 0x10, 0x10,/* TOP_CKPDN_CON1 */
	0x112, 0x4, 0x4,/* TOP_CKPDN_CON2 */
	0x11B, 0xC, 0x0,/* TOP_CKSEL_CON2 */
	0x128, 0x55, 0x0,/* TOP_CLK_CON0 */
	0x129, 0x5, 0x0,/* TOP_CLK_CON1 */
	0x142, 0x1, 0x0,/* TOP_DCXO_CKEN_SW */
	0x143, 0x1, 0x0,/* TOP_SPMI_CON0 */
	0x240, 0xFF, 0x1,/* TOP_VRCTL_VR0_EN */
	0x243, 0x3, 0x0,/* TOP_VRCTL_VR1_EN */
	0x246, 0xFF, 0x0,/* TOP_VRCTL_VR0_LP */
	0x249, 0x3, 0x0,/* TOP_VRCTL_VR1_LP */
	0xF8C, 0xFF, 0x15,/* HK_TOP_CLK_CON0 */
	0xF8D, 0x23, 0x21,/* HK_TOP_CLK_CON1 */
	0x1188, 0xE0, 0x0,/* AUXADC_CON0 */
	0x1189, 0x3F, 0x14,/* AUXADC_CON1 */
	0x1190, 0xFF, 0x13,/* AUXADC_SPL_CON6 */
	0x119D, 0x70, 0x60,/* AUXADC_AVG_CON5 */
	0x1208, 0x1, 0x0,/* AUXADC_IMP0 */
	0x120C, 0x2, 0x0,/* AUXADC_IMP4 */
	0x120D, 0x1, 0x0,/* AUXADC_LBAT0 */
	0x1218, 0x2, 0x0,/* AUXADC_LBAT11 */
	0x1219, 0x1, 0x0,/* AUXADC_LBAT2_0 */
	0x121A, 0x3, 0x2,/* AUXADC_LBAT2_1 */
	0x121B, 0x2, 0x2,/* AUXADC_LBAT2_2 */
	0x121E, 0x2, 0x2,/* AUXADC_LBAT2_5 */
	0x1224, 0x2, 0x0,/* AUXADC_LBAT2_11 */
	0x1225, 0x1, 0x0,/* AUXADC_BAT_TEMP_0 */
	0x1227, 0x3, 0x2,/* AUXADC_BAT_TEMP_2 */
	0x1228, 0x2, 0x2,/* AUXADC_BAT_TEMP_3 */
	0x122B, 0x2, 0x2,/* AUXADC_BAT_TEMP_6 */
	0x1231, 0x2, 0x0,/* AUXADC_BAT_TEMP_12 */
	0x1232, 0x1, 0x0,/* AUXADC_THR0 */
	0x123D, 0x2, 0x0,/* AUXADC_THR11 */
	0x1247, 0x19, 0x10,/* AUXADC_NAG_0 */
	0x1259, 0x2, 0x0,/* AUXADC_NAG_18 */
	0x140B, 0x7, 0x0,/* BUCK_TOP_CLK_CON0 */
	0x140E, 0x7, 0x7,/* BUCK_TOP_CLK_HWEN_CON0 */
	0x1488, 0x10, 0x0,/* BUCK_VBUCK0_SLP_CON */
	0x148D, 0x1, 0x1,/* BUCK_VBUCK0_OP_EN_0 */
	0x148F, 0x80, 0x80,/* BUCK_VBUCK0_OP_EN_2 */
	0x1490, 0x1, 0x1,/* BUCK_VBUCK0_OP_CFG_0 */
	0x1493, 0x1, 0x1,/* BUCK_VBUCK0_OP_MODE_0 */
	0x1508, 0x10, 0x0,/* BUCK_VBUCK1_SLP_CON */
	0x150F, 0x80, 0x80,/* BUCK_VBUCK1_OP_EN_2 */
	0x1588, 0x10, 0x0,/* BUCK_VBUCK2_SLP_CON */
	0x158F, 0x80, 0x80,/* BUCK_VBUCK2_OP_EN_2 */
	0x1608, 0x10, 0x0,/* BUCK_VBUCK3_SLP_CON */
	0x160F, 0x80, 0x80,/* BUCK_VBUCK3_OP_EN_2 */
	0x1688, 0x10, 0x0,/* BUCK_VBUCK4_SLP_CON */
	0x168F, 0x80, 0x80,/* BUCK_VBUCK4_OP_EN_2 */
	0x1708, 0x10, 0x0,/* BUCK_VBUCK5_SLP_CON */
	0x170F, 0x80, 0x80,/* BUCK_VBUCK5_OP_EN_2 */
	0x1788, 0x10, 0x0,/* BUCK_VBUCK6_SLP_CON */
	0x178F, 0x80, 0x80,/* BUCK_VBUCK6_OP_EN_2 */
	0x1808, 0x10, 0x0,/* BUCK_VBUCK7_SLP_CON */
	0x180F, 0x80, 0x80,/* BUCK_VBUCK7_OP_EN_2 */
	0x1888, 0x10, 0x0,/* BUCK_VBUCK8_SLP_CON */
	0x188F, 0x80, 0x80,/* BUCK_VBUCK8_OP_EN_2 */
	0x1908, 0x10, 0x0,/* BUCK_VBUCK9_SLP_CON */
	0x190F, 0x80, 0x80,/* BUCK_VBUCK9_OP_EN_2 */
	0x1B0D, 0x3, 0x3,/* TOP_TOP_CKHWEN_CON0 */
	0x1B0E, 0x1, 0x1,/* LDO_TOP_CLK_DCM_CON0 */
	0x1B87, 0x3, 0x0,/* LDO_VAUD18_CON0 */
	0x1B88, 0x8, 0x0,/* LDO_VAUD18_CON1 */
	0x1B89, 0x80, 0x0,/* LDO_VAUD18_CON2 */
	0x1B8E, 0x80, 0x80,/* LDO_VAUD18_OP_EN2 */
	0x1B95, 0x3, 0x1,/* LDO_VUSB_CON0 */
	0x1B96, 0x8, 0x0,/* LDO_VUSB_CON1 */
	0x1B97, 0x80, 0x0,/* LDO_VUSB_CON2 */
	0x1B9A, 0x1, 0x1,/* LDO_VUSB_OP_EN0 */
	0x1B9C, 0x80, 0x80,/* LDO_VUSB_OP_EN2 */
	0x1B9D, 0x1, 0x1,/* LDO_VUSB_OP_CFG0 */
	0x1BA0, 0x1, 0x1,/* LDO_VUSB_OP_MODE0 */
	0x1BA3, 0x3, 0x0,/* LDO_VAUX18_CON0 */
	0x1BA4, 0x8, 0x0,/* LDO_VAUX18_CON1 */
	0x1BA5, 0x80, 0x0,/* LDO_VAUX18_CON2 */
	0x1BAA, 0x80, 0x80,/* LDO_VAUX18_OP_EN2 */
	0x1BB1, 0x3, 0x0,/* LDO_VRF13_AIF_CON0 */
	0x1BB2, 0x8, 0x0,/* LDO_VRF13_AIF_CON1 */
	0x1BB3, 0x80, 0x0,/* LDO_VRF13_AIF_CON2 */
	0x1BB8, 0x80, 0x80,/* LDO_VRF13_AIF_OP_EN2 */
	0x1BBF, 0x3, 0x0,/* LDO_VRF18_AIF_CON0 */
	0x1BC0, 0x8, 0x0,/* LDO_VRF18_AIF_CON1 */
	0x1BC1, 0x80, 0x0,/* LDO_VRF18_AIF_CON2 */
	0x1BC6, 0x80, 0x80,/* LDO_VRF18_AIF_OP_EN2 */
	0x1BCD, 0x3, 0x0,/* LDO_VRFIO18_AIF_CON0 */
	0x1BCE, 0x8, 0x0,/* LDO_VRFIO18_AIF_CON1 */
	0x1BCF, 0x80, 0x0,/* LDO_VRFIO18_AIF_CON2 */
	0x1BD4, 0x80, 0x80,/* LDO_VRFIO18_AIF_OP_EN2 */
	0x1C07, 0x3, 0x1,/* LDO_VCN33_1_CON0 */
	0x1C08, 0x8, 0x0,/* LDO_VCN33_1_CON1 */
	0x1C09, 0x80, 0x0,/* LDO_VCN33_1_CON2 */
	0x1C0C, 0x80, 0x80,/* LDO_VCN33_1_OP_EN0 */
	0x1C0D, 0x1, 0x1,/* LDO_VCN33_1_OP_EN1 */
	0x1C0E, 0x80, 0x80,/* LDO_VCN33_1_OP_EN2 */
	0x1C0F, 0x80, 0x0,/* LDO_VCN33_1_OP_CFG0 */
	0x1C10, 0x1, 0x0,/* LDO_VCN33_1_OP_CFG1 */
	0x1C12, 0x80, 0x80,/* LDO_VCN33_1_OP_MODE0 */
	0x1C13, 0x1, 0x0,/* LDO_VCN33_1_OP_MODE1 */
	0x1C15, 0x3, 0x1,/* LDO_VCN33_2_CON0 */
	0x1C16, 0x8, 0x0,/* LDO_VCN33_2_CON1 */
	0x1C17, 0x80, 0x0,/* LDO_VCN33_2_CON2 */
	0x1C1B, 0x1, 0x1,/* LDO_VCN33_2_OP_EN1 */
	0x1C1C, 0x80, 0x80,/* LDO_VCN33_2_OP_EN2 */
	0x1C1E, 0x1, 0x0,/* LDO_VCN33_2_OP_CFG1 */
	0x1C21, 0x1, 0x1,/* LDO_VCN33_2_OP_MODE1 */
	0x1C23, 0x3, 0x0,/* LDO_VCN33_3_CON0 */
	0x1C24, 0x8, 0x0,/* LDO_VCN33_3_CON1 */
	0x1C25, 0x80, 0x0,/* LDO_VCN33_3_CON2 */
	0x1C2A, 0x80, 0x80,/* LDO_VCN33_3_OP_EN2 */
	0x1C31, 0x3, 0x0,/* LDO_VCN18IO_CON0 */
	0x1C32, 0x8, 0x0,/* LDO_VCN18IO_CON1 */
	0x1C33, 0x80, 0x0,/* LDO_VCN18IO_CON2 */
	0x1C38, 0x80, 0x80,/* LDO_VCN18IO_OP_EN2 */
	0x1C3F, 0x3, 0x0,/* LDO_VRF09_AIF_CON0 */
	0x1C40, 0x8, 0x0,/* LDO_VRF09_AIF_CON1 */
	0x1C41, 0x80, 0x0,/* LDO_VRF09_AIF_CON2 */
	0x1C46, 0x80, 0x80,/* LDO_VRF09_AIF_OP_EN2 */
	0x1C4D, 0x3, 0x0,/* LDO_VRF12_AIF_CON0 */
	0x1C4E, 0x8, 0x0,/* LDO_VRF12_AIF_CON1 */
	0x1C4F, 0x80, 0x0,/* LDO_VRF12_AIF_CON2 */
	0x1C54, 0x80, 0x80,/* LDO_VRF12_AIF_OP_EN2 */
	0x1C87, 0x3, 0x0,/* LDO_VANT18_CON0 */
	0x1C88, 0x8, 0x0,/* LDO_VANT18_CON1 */
	0x1C89, 0x80, 0x0,/* LDO_VANT18_CON2 */
	0x1C8E, 0x80, 0x80,/* LDO_VANT18_OP_EN2 */
	0x1C95, 0x3, 0x3,/* LDO_VMDDR_CON0 */
	0x1C96, 0x8, 0x0,/* LDO_VMDDR_CON1 */
	0x1C97, 0x80, 0x0,/* LDO_VMDDR_CON2 */
	0x1C9C, 0x80, 0x80,/* LDO_VMDDR_OP_EN2 */
	0x1CA3, 0x3, 0x0,/* LDO_VEFUSE_CON0 */
	0x1CA4, 0x8, 0x0,/* LDO_VEFUSE_CON1 */
	0x1CA5, 0x80, 0x0,/* LDO_VEFUSE_CON2 */
	0x1CAA, 0x80, 0x80,/* LDO_VEFUSE_OP_EN2 */
	0x1CB1, 0x3, 0x0,/* LDO_VMCH_CON0 */
	0x1CB2, 0x8, 0x0,/* LDO_VMCH_CON1 */
	0x1CB3, 0x80, 0x0,/* LDO_VMCH_CON2 */
	0x1CB8, 0x80, 0x80,/* LDO_VMCH_OP_EN2 */
	0x1CC0, 0x3, 0x0,/* LDO_VMC_CON0 */
	0x1CC1, 0x8, 0x0,/* LDO_VMC_CON1 */
	0x1CC2, 0x80, 0x0,/* LDO_VMC_CON2 */
	0x1CC7, 0x80, 0x80,/* LDO_VMC_OP_EN2 */
	0x1CCE, 0x3, 0x0,/* LDO_VIBR_CON0 */
	0x1CCF, 0x8, 0x0,/* LDO_VIBR_CON1 */
	0x1CD0, 0x80, 0x0,/* LDO_VIBR_CON2 */
	0x1CD5, 0x80, 0x80,/* LDO_VIBR_OP_EN2 */
	0x1D07, 0x3, 0x1,/* LDO_VIO28_CON0 */
	0x1D08, 0x8, 0x0,/* LDO_VIO28_CON1 */
	0x1D09, 0x80, 0x0,/* LDO_VIO28_CON2 */
	0x1D0E, 0x80, 0x80,/* LDO_VIO28_OP_EN2 */
	0x1D15, 0x3, 0x0,/* LDO_VFP_CON0 */
	0x1D16, 0x8, 0x0,/* LDO_VFP_CON1 */
	0x1D17, 0x80, 0x0,/* LDO_VFP_CON2 */
	0x1D1C, 0x80, 0x80,/* LDO_VFP_OP_EN2 */
	0x1D23, 0x3, 0x0,/* LDO_VTP_CON0 */
	0x1D24, 0x8, 0x0,/* LDO_VTP_CON1 */
	0x1D25, 0x80, 0x0,/* LDO_VTP_CON2 */
	0x1D2A, 0x80, 0x80,/* LDO_VTP_OP_EN2 */
	0x1D31, 0x3, 0x0,/* LDO_VSIM1_CON0 */
	0x1D32, 0x8, 0x0,/* LDO_VSIM1_CON1 */
	0x1D33, 0x80, 0x0,/* LDO_VSIM1_CON2 */
	0x1D38, 0x80, 0x80,/* LDO_VSIM1_OP_EN2 */
	0x1D40, 0x3, 0x0,/* LDO_VSIM2_CON0 */
	0x1D41, 0x8, 0x0,/* LDO_VSIM2_CON1 */
	0x1D42, 0x80, 0x0,/* LDO_VSIM2_CON2 */
	0x1D47, 0x80, 0x80,/* LDO_VSIM2_OP_EN2 */
	0x1D87, 0x3, 0x0,/* LDO_VSRAM_DIGRF_AIF_CON0 */
	0x1D88, 0x8, 0x0,/* LDO_VSRAM_DIGRF_AIF_CON1 */
	0x1D89, 0x80, 0x0,/* LDO_VSRAM_DIGRF_AIF_CON2 */
	0x1D96, 0x80, 0x80 /* LDO_VSRAM_DIGRF_AIF_OP_EN2 */
};

const unsigned int *AP_PMIC_REG_6368_gs_suspend_32kless =
				AP_PMIC_REG_MT6368_gs_suspend_32kless_data;

unsigned int AP_PMIC_REG_6368_gs_suspend_32kless_len = 519;

const unsigned int AP_PMIC_REG_MT6368_gs_deepidle___lp_mp3_32kless_data[] = {
/*  Address     Mask        Golden Setting Value */
	0x10F, 0x4, 0x0,/* TOP_CKPDN_CON1 */
	0x148, 0xF, 0x0,/* TOP_CFG_ELR3 */
	0x3AA, 0x3F, 0x0,/* PLT_CFG_ELR0 */
	0x413, 0xFF, 0x0,/* SPMI_EXT_ADDR1 */
	0x416, 0xFF, 0x0,/* SPMI_EXT_ADDR2 */
	0x417, 0x7, 0x0,/* SPMI_EXT_ADDR2_H */
	0xA16, 0xC0, 0x0,/* STRUP_CON8 */
	0xA37, 0x20, 0x0,/* PSEQ_ELR4 */
	0x144A, 0xFF, 0x0,/* BUCK_TOP_ELR0 */
	0x144B, 0xFF, 0x0,/* BUCK_TOP_ELR1 */
	0x144C, 0xF, 0x0,/* BUCK_TOP_ELR2 */
	0x1989, 0x56, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON0 */
	0x198A, 0x2F, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON1 */
	0x198B, 0x2, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON2 */
	0x198C, 0xFF, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON3 */
	0x198D, 0x3F, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON4 */
	0x198F, 0xC, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON6 */
	0x1991, 0xFF, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON8 */
	0x1992, 0xF, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON9 */
	0x1993, 0x56, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON10 */
	0x1994, 0x2F, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON11 */
	0x1995, 0x1, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON12 */
	0x1996, 0xFF, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON13 */
	0x1997, 0x3F, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON14 */
	0x1999, 0xC, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON16 */
	0x199B, 0xFF, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON18 */
	0x199C, 0xF, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON19 */
	0x19A0, 0xC, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON23 */
	0x19A1, 0xFF, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON24 */
	0x19A2, 0xFF, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON25 */
	0x19A3, 0xC, 0x0,/* BUCK_TOP_2PHASE_1_ANA_CON26 */
	0x1A08, 0x50, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON0 */
	0x1A09, 0x2F, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON1 */
	0x1A0B, 0xF8, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON3 */
	0x1A0C, 0x3F, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON4 */
	0x1A0E, 0xC, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON6 */
	0x1A0F, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON7 */
	0x1A10, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON8 */
	0x1A12, 0x56, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON10 */
	0x1A13, 0x2F, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON11 */
	0x1A14, 0x1, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON12 */
	0x1A15, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON13 */
	0x1A16, 0x3F, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON14 */
	0x1A18, 0xC, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON16 */
	0x1A1A, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON18 */
	0x1A1B, 0xF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON19 */
	0x1A1C, 0x56, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON20 */
	0x1A1D, 0x2F, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON21 */
	0x1A1E, 0x1, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON22 */
	0x1A1F, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON23 */
	0x1A20, 0x3F, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON24 */
	0x1A22, 0xC, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON26 */
	0x1A24, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON28 */
	0x1A25, 0xF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON29 */
	0x1A26, 0x56, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON30 */
	0x1A27, 0x2F, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON31 */
	0x1A28, 0x1, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON32 */
	0x1A29, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON33 */
	0x1A2A, 0x3F, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON34 */
	0x1A2C, 0xC, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON36 */
	0x1A2E, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON38 */
	0x1A2F, 0xF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON39 */
	0x1A37, 0xF1, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON47 */
	0x1A39, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON49 */
	0x1A3A, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON50 */
	0x1A3B, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON51 */
	0x1A3C, 0xFF, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON52 */
	0x1A3D, 0xF0, 0x0,/* BUCK_TOP_4PHASE_1_ANA_CON53 */
	0x1A89, 0x2C, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON1 */
	0x1A8B, 0xF8, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON3 */
	0x1A8C, 0x3F, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON4 */
	0x1A8E, 0xC, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON6 */
	0x1A8F, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON7 */
	0x1A90, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON8 */
	0x1A91, 0xF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON9 */
	0x1A92, 0x56, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON10 */
	0x1A93, 0x2F, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON11 */
	0x1A94, 0x1, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON12 */
	0x1A95, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON13 */
	0x1A96, 0x3F, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON14 */
	0x1A98, 0xC, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON16 */
	0x1A9A, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON18 */
	0x1A9B, 0xF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON19 */
	0x1A9C, 0x56, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON20 */
	0x1A9D, 0x2F, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON21 */
	0x1A9E, 0x1, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON22 */
	0x1A9F, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON23 */
	0x1AA0, 0x3F, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON24 */
	0x1AA2, 0xC, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON26 */
	0x1AA4, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON28 */
	0x1AA5, 0xF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON29 */
	0x1AA6, 0x56, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON30 */
	0x1AA7, 0x2F, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON31 */
	0x1AA8, 0x1, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON32 */
	0x1AA9, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON33 */
	0x1AAA, 0x3F, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON34 */
	0x1AAC, 0xC, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON36 */
	0x1AAE, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON38 */
	0x1AAF, 0xF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON39 */
	0x1AB7, 0xF1, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON47 */
	0x1AB9, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON49 */
	0x1ABA, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON50 */
	0x1ABB, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON51 */
	0x1ABC, 0xFF, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON52 */
	0x1ABD, 0xF0, 0x0,/* BUCK_TOP_4PHASE_2_ANA_CON53 */
};

const unsigned int *AP_PMIC_REG_6368_gs_deepidle___lp_mp3_32kless =
			AP_PMIC_REG_MT6368_gs_deepidle___lp_mp3_32kless_data;

unsigned int AP_PMIC_REG_6368_gs_deepidle___lp_mp3_32kless_len = 315;

const unsigned int AP_PMIC_REG_MT6368_gs_sodi3p0_32kless_data[] = {
/*  Address     Mask        Golden Setting Value */
	0x16, 0xA, 0xA,/* TOP_CON */
	0x19, 0x1F, 0x1F,/* TEST_CON0 */
	0x21, 0x1, 0x1,/* SMT_CON0 */
	0x10F, 0x10, 0x10,/* TOP_CKPDN_CON1 */
	0x112, 0x4, 0x4,/* TOP_CKPDN_CON2 */
	0x11B, 0xC, 0x0,/* TOP_CKSEL_CON2 */
	0x128, 0x55, 0x0,/* TOP_CLK_CON0 */
	0x129, 0x5, 0x0,/* TOP_CLK_CON1 */
	0x142, 0x1, 0x0,/* TOP_DCXO_CKEN_SW */
	0x143, 0x1, 0x0,/* TOP_SPMI_CON0 */
	0x240, 0xFF, 0x1,/* TOP_VRCTL_VR0_EN */
	0x243, 0x3, 0x0,/* TOP_VRCTL_VR1_EN */
	0x246, 0xFF, 0x0,/* TOP_VRCTL_VR0_LP */
	0x249, 0x3, 0x0,/* TOP_VRCTL_VR1_LP */
	0xF8C, 0xFF, 0x15,/* HK_TOP_CLK_CON0 */
	0xF8D, 0x23, 0x21,/* HK_TOP_CLK_CON1 */
	0x1188, 0xE0, 0x0,/* AUXADC_CON0 */
	0x1189, 0x3F, 0x14,/* AUXADC_CON1 */
	0x1190, 0xFF, 0x13,/* AUXADC_SPL_CON6 */
	0x119D, 0x70, 0x60,/* AUXADC_AVG_CON5 */
	0x1208, 0x1, 0x0,/* AUXADC_IMP0 */
	0x120C, 0x2, 0x0,/* AUXADC_IMP4 */
	0x120D, 0x1, 0x0,/* AUXADC_LBAT0 */
	0x1218, 0x2, 0x0,/* AUXADC_LBAT11 */
	0x1219, 0x1, 0x0,/* AUXADC_LBAT2_0 */
	0x121A, 0x3, 0x2,/* AUXADC_LBAT2_1 */
	0x121B, 0x2, 0x2,/* AUXADC_LBAT2_2 */
	0x121E, 0x2, 0x2,/* AUXADC_LBAT2_5 */
	0x1224, 0x2, 0x0,/* AUXADC_LBAT2_11 */
	0x1225, 0x1, 0x0,/* AUXADC_BAT_TEMP_0 */
	0x1227, 0x3, 0x2,/* AUXADC_BAT_TEMP_2 */
	0x1228, 0x2, 0x2,/* AUXADC_BAT_TEMP_3 */
	0x122B, 0x2, 0x2,/* AUXADC_BAT_TEMP_6 */
	0x1231, 0x2, 0x0,/* AUXADC_BAT_TEMP_12 */
	0x1232, 0x1, 0x0,/* AUXADC_THR0 */
	0x123D, 0x2, 0x0,/* AUXADC_THR11 */
	0x1247, 0x19, 0x10,/* AUXADC_NAG_0 */
	0x1259, 0x2, 0x0,/* AUXADC_NAG_18 */
	0x140B, 0x7, 0x0,/* BUCK_TOP_CLK_CON0 */
	0x140E, 0x7, 0x7,/* BUCK_TOP_CLK_HWEN_CON0 */
	0x1488, 0x10, 0x0,/* BUCK_VBUCK0_SLP_CON */
	0x148D, 0x1, 0x1,/* BUCK_VBUCK0_OP_EN_0 */
	0x148F, 0x80, 0x80,/* BUCK_VBUCK0_OP_EN_2 */
	0x1490, 0x1, 0x1,/* BUCK_VBUCK0_OP_CFG_0 */
	0x1493, 0x1, 0x1,/* BUCK_VBUCK0_OP_MODE_0 */
	0x1508, 0x10, 0x0,/* BUCK_VBUCK1_SLP_CON */
	0x150F, 0x80, 0x80,/* BUCK_VBUCK1_OP_EN_2 */
	0x1588, 0x10, 0x0,/* BUCK_VBUCK2_SLP_CON */
	0x158F, 0x80, 0x80,/* BUCK_VBUCK2_OP_EN_2 */
	0x1608, 0x10, 0x0,/* BUCK_VBUCK3_SLP_CON */
	0x160F, 0x80, 0x80,/* BUCK_VBUCK3_OP_EN_2 */
	0x1688, 0x10, 0x0,/* BUCK_VBUCK4_SLP_CON */
	0x168F, 0x80, 0x80,/* BUCK_VBUCK4_OP_EN_2 */
	0x1708, 0x10, 0x0,/* BUCK_VBUCK5_SLP_CON */
	0x170F, 0x80, 0x80,/* BUCK_VBUCK5_OP_EN_2 */
	0x1788, 0x10, 0x0,/* BUCK_VBUCK6_SLP_CON */
	0x178F, 0x80, 0x80,/* BUCK_VBUCK6_OP_EN_2 */
	0x1808, 0x10, 0x0,/* BUCK_VBUCK7_SLP_CON */
	0x180F, 0x80, 0x80,/* BUCK_VBUCK7_OP_EN_2 */
	0x1888, 0x10, 0x0,/* BUCK_VBUCK8_SLP_CON */
	0x188F, 0x80, 0x80,/* BUCK_VBUCK8_OP_EN_2 */
	0x1908, 0x10, 0x0,/* BUCK_VBUCK9_SLP_CON */
	0x190F, 0x80, 0x80,/* BUCK_VBUCK9_OP_EN_2 */
	0x1B0D, 0x3, 0x3,/* TOP_TOP_CKHWEN_CON0 */
	0x1B0E, 0x1, 0x1,/* LDO_TOP_CLK_DCM_CON0 */
	0x1B87, 0x3, 0x0,/* LDO_VAUD18_CON0 */
	0x1B88, 0x8, 0x0,/* LDO_VAUD18_CON1 */
	0x1B89, 0x80, 0x0,/* LDO_VAUD18_CON2 */
	0x1B8E, 0x80, 0x80,/* LDO_VAUD18_OP_EN2 */
	0x1B95, 0x3, 0x1,/* LDO_VUSB_CON0 */
	0x1B96, 0x8, 0x0,/* LDO_VUSB_CON1 */
	0x1B97, 0x80, 0x0,/* LDO_VUSB_CON2 */
	0x1B9A, 0x1, 0x1,/* LDO_VUSB_OP_EN0 */
	0x1B9C, 0x80, 0x80,/* LDO_VUSB_OP_EN2 */
	0x1B9D, 0x1, 0x1,/* LDO_VUSB_OP_CFG0 */
	0x1BA0, 0x1, 0x1,/* LDO_VUSB_OP_MODE0 */
	0x1BA3, 0x3, 0x0,/* LDO_VAUX18_CON0 */
	0x1BA4, 0x8, 0x0,/* LDO_VAUX18_CON1 */
	0x1BA5, 0x80, 0x0,/* LDO_VAUX18_CON2 */
	0x1BAA, 0x80, 0x80,/* LDO_VAUX18_OP_EN2 */
	0x1BB1, 0x3, 0x0,/* LDO_VRF13_AIF_CON0 */
	0x1BB2, 0x8, 0x0,/* LDO_VRF13_AIF_CON1 */
	0x1BB3, 0x80, 0x0,/* LDO_VRF13_AIF_CON2 */
	0x1BB8, 0x80, 0x80,/* LDO_VRF13_AIF_OP_EN2 */
	0x1BBF, 0x3, 0x0,/* LDO_VRF18_AIF_CON0 */
	0x1BC0, 0x8, 0x0,/* LDO_VRF18_AIF_CON1 */
	0x1BC1, 0x80, 0x0,/* LDO_VRF18_AIF_CON2 */
	0x1BC6, 0x80, 0x80,/* LDO_VRF18_AIF_OP_EN2 */
	0x1BCD, 0x3, 0x0,/* LDO_VRFIO18_AIF_CON0 */
	0x1BCE, 0x8, 0x0,/* LDO_VRFIO18_AIF_CON1 */
	0x1BCF, 0x80, 0x0,/* LDO_VRFIO18_AIF_CON2 */
	0x1BD4, 0x80, 0x80,/* LDO_VRFIO18_AIF_OP_EN2 */
	0x1C07, 0x3, 0x0,/* LDO_VCN33_1_CON0 */
	0x1C08, 0x8, 0x0,/* LDO_VCN33_1_CON1 */
	0x1C09, 0x80, 0x0,/* LDO_VCN33_1_CON2 */
	0x1C0E, 0x80, 0x80,/* LDO_VCN33_1_OP_EN2 */
	0x1C15, 0x3, 0x0,/* LDO_VCN33_2_CON0 */
	0x1C16, 0x8, 0x0,/* LDO_VCN33_2_CON1 */
	0x1C17, 0x80, 0x0,/* LDO_VCN33_2_CON2 */
	0x1C1C, 0x80, 0x80,/* LDO_VCN33_2_OP_EN2 */
	0x1C23, 0x3, 0x0,/* LDO_VCN33_3_CON0 */
	0x1C24, 0x8, 0x0,/* LDO_VCN33_3_CON1 */
	0x1C25, 0x80, 0x0,/* LDO_VCN33_3_CON2 */
	0x1C2A, 0x80, 0x80,/* LDO_VCN33_3_OP_EN2 */
	0x1C31, 0x3, 0x0,/* LDO_VCN18IO_CON0 */
	0x1C32, 0x8, 0x0,/* LDO_VCN18IO_CON1 */
	0x1C33, 0x80, 0x0,/* LDO_VCN18IO_CON2 */
	0x1C38, 0x80, 0x80,/* LDO_VCN18IO_OP_EN2 */
	0x1C3F, 0x3, 0x0,/* LDO_VRF09_AIF_CON0 */
	0x1C40, 0x8, 0x0,/* LDO_VRF09_AIF_CON1 */
	0x1C41, 0x80, 0x0,/* LDO_VRF09_AIF_CON2 */
	0x1C46, 0x80, 0x80,/* LDO_VRF09_AIF_OP_EN2 */
	0x1C4D, 0x3, 0x0,/* LDO_VRF12_AIF_CON0 */
	0x1C4E, 0x8, 0x0,/* LDO_VRF12_AIF_CON1 */
	0x1C4F, 0x80, 0x0,/* LDO_VRF12_AIF_CON2 */
	0x1C54, 0x80, 0x80,/* LDO_VRF12_AIF_OP_EN2 */
	0x1C87, 0x3, 0x0,/* LDO_VANT18_CON0 */
	0x1C88, 0x8, 0x0,/* LDO_VANT18_CON1 */
	0x1C89, 0x80, 0x0,/* LDO_VANT18_CON2 */
	0x1C8E, 0x80, 0x80,/* LDO_VANT18_OP_EN2 */
	0x1C95, 0x3, 0x3,/* LDO_VMDDR_CON0 */
	0x1C96, 0x8, 0x0,/* LDO_VMDDR_CON1 */
	0x1C97, 0x80, 0x0,/* LDO_VMDDR_CON2 */
	0x1C9C, 0x80, 0x80,/* LDO_VMDDR_OP_EN2 */
	0x1CA3, 0x3, 0x0,/* LDO_VEFUSE_CON0 */
	0x1CA4, 0x8, 0x0,/* LDO_VEFUSE_CON1 */
	0x1CA5, 0x80, 0x0,/* LDO_VEFUSE_CON2 */
	0x1CAA, 0x80, 0x80,/* LDO_VEFUSE_OP_EN2 */
	0x1CB1, 0x3, 0x0,/* LDO_VMCH_CON0 */
	0x1CB2, 0x8, 0x0,/* LDO_VMCH_CON1 */
	0x1CB3, 0x80, 0x0,/* LDO_VMCH_CON2 */
	0x1CB8, 0x80, 0x80,/* LDO_VMCH_OP_EN2 */
	0x1CC0, 0x3, 0x0,/* LDO_VMC_CON0 */
	0x1CC1, 0x8, 0x0,/* LDO_VMC_CON1 */
	0x1CC2, 0x80, 0x0,/* LDO_VMC_CON2 */
	0x1CC7, 0x80, 0x80,/* LDO_VMC_OP_EN2 */
	0x1CCE, 0x3, 0x0,/* LDO_VIBR_CON0 */
	0x1CCF, 0x8, 0x0,/* LDO_VIBR_CON1 */
	0x1CD0, 0x80, 0x0,/* LDO_VIBR_CON2 */
	0x1CD5, 0x80, 0x80,/* LDO_VIBR_OP_EN2 */
	0x1D07, 0x3, 0x1,/* LDO_VIO28_CON0 */
	0x1D08, 0x8, 0x0,/* LDO_VIO28_CON1 */
	0x1D09, 0x80, 0x0,/* LDO_VIO28_CON2 */
	0x1D0E, 0x80, 0x80,/* LDO_VIO28_OP_EN2 */
	0x1D15, 0x3, 0x0,/* LDO_VFP_CON0 */
	0x1D16, 0x8, 0x0,/* LDO_VFP_CON1 */
	0x1D17, 0x80, 0x0,/* LDO_VFP_CON2 */
	0x1D1C, 0x80, 0x80,/* LDO_VFP_OP_EN2 */
	0x1D23, 0x3, 0x0,/* LDO_VTP_CON0 */
	0x1D24, 0x8, 0x0,/* LDO_VTP_CON1 */
	0x1D25, 0x80, 0x0,/* LDO_VTP_CON2 */
	0x1D2A, 0x80, 0x80,/* LDO_VTP_OP_EN2 */
	0x1D31, 0x3, 0x0,/* LDO_VSIM1_CON0 */
	0x1D32, 0x8, 0x0,/* LDO_VSIM1_CON1 */
	0x1D33, 0x80, 0x0,/* LDO_VSIM1_CON2 */
	0x1D38, 0x80, 0x80,/* LDO_VSIM1_OP_EN2 */
	0x1D40, 0x3, 0x0,/* LDO_VSIM2_CON0 */
	0x1D41, 0x8, 0x0,/* LDO_VSIM2_CON1 */
	0x1D42, 0x80, 0x0,/* LDO_VSIM2_CON2 */
	0x1D47, 0x80, 0x80,/* LDO_VSIM2_OP_EN2 */
	0x1D87, 0x3, 0x0,/* LDO_VSRAM_DIGRF_AIF_CON0 */
	0x1D88, 0x8, 0x0,/* LDO_VSRAM_DIGRF_AIF_CON1 */
	0x1D89, 0x80, 0x0,/* LDO_VSRAM_DIGRF_AIF_CON2 */
	0x1D96, 0x80, 0x80 /* LDO_VSRAM_DIGRF_AIF_OP_EN2 */
};

const unsigned int *AP_PMIC_REG_6368_gs_sodi3p0_32kless =
				AP_PMIC_REG_MT6368_gs_sodi3p0_32kless_data;

unsigned int AP_PMIC_REG_6368_gs_sodi3p0_32kless_len = 492;
