/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#ifndef _CLK_MT6855_FMETER_H
#define _CLK_MT6855_FMETER_H

/* generate from clock_table.xlsx from TOPCKGEN DE */

/* CKGEN Part */
#define FM_AXI_CK				1
#define FM_AXIP_CK				2
#define FM_HAXI_U_CK				3
#define FM_B					4
#define FM_DISP0_CK				5
#define FM_MDP0_CK				6
#define FM_MMINFRA_CK				7
#define FM_MMUP_CK				8
#define FM_CAMTG_CK				9
#define FM_CAMTG2_CK				10
#define FM_CAMTG3_CK				11
#define FM_CAMTG4_CK				12
#define FM_CAMTG5_CK				13
#define FM_CAMTG6_CK				14
#define FM_UART_CK				15
#define FM_SPI_CK				16
#define FM_MSDC_0P_MACRO_CK			17
#define FM_MSDC5HCLK_CK				18
#define FM_MSDC50_0_CK				19
#define FM_AES_MSDCFDE_CK			20
#define FM_MSDC_MACRO_CK			21
#define FM_MSDC30_1_CK				22
#define FM_AUDIO_CK				23
#define FM_AUD_INTBUS_CK			24
#define FM_ATB_CK				25
#define FM_DISP_PWM_CK				26
#define FM_USB_CK				27
#define FM_USB_XHCI_CK				28
#define FM_I2C_CK				29
#define FM_SENINF_CK				30
#define FM_SENINF1_CK				31
#define FM_SENINF2_CK				32
#define FM_SENINF3_CK				33
#define FM_DXCC_CK				34
#define FM_AUD_ENGEN1_CK			35
#define FM_AUD_ENGEN2_CK			36
#define FM_AES_UFSFDE_CK			37
#define FM_U_CK					38
#define FM_U_MBIST_CK				39
#define FM_AUD_1_CK				40
#define FM_AUD_2_CK				41
#define FM_DPMAIF_MAIN_CK			42
#define FM_VENC_CK				43
#define FM_VDEC_CK				44
#define FM_PWM_CK				45
#define FM_AUDIO_H_CK				46
#define FM_MCUPM_CK				47
#define FM_MEM_SUB_CK				48
#define FM_MEM_SUBP_CK				49
#define FM_MEM_SUB_U_CK				50
#define FM_EMI_N_CK				51
#define FM_DSI_OCC_CK				52
#define FM_AP2CONN_HOST_CK			53
#define FM_MCU_ACP_CK				54
#define FM_IMG1_CK				55
#define FM_IPE_CK				56
#define FM_CAM_CK				57
#define FM_CAMTM_CK				58
#define FM_MSDC_1P_RX_CK			59
/* ABIST Part */
#define FM_APLL1_CK				2
#define FM_APLL2_CK				3
#define FM_APPLLGP_MON_FM_CK			4
#define FM_ARMPLL_BL_CK				6
#define FM_ARMPLL_BL_CKDIV_CK			7
#define FM_ARMPLL_LL_CK				8
#define FM_ARMPLL_LL_CKDIV_CK			9
#define FM_CCIPLL_CK				10
#define FM_CCIPLL_CKDIV_CK			11
#define FM_CSI0A_DELAYCAL_CK			12
#define FM_CSI0B_DELAYCAL_CK			13
#define FM_CSI1A_DEKAYCAL_CK			14
#define FM_CSI1B_DELAYCAL_CK			15
#define FM_CSI2A_DELAYCAL_CK			16
#define FM_CSI2B_DELAYCAL_CK			17
#define FM_CSI3A_DELAYCAL_CK			18
#define FM_CSI3B_DELAYCAL_CK			19
#define FM_DSI0_LNTC_DSICLK			20
#define FM_DSI0_MPPLL_TST_CK			21
#define FM_MAINPLL_CKDIV_CK			23
#define FM_MAINPLL_CK				24
#define FM_MDPLL1_FS26M_GUIDE			25
#define FM_MMPLL_CKDIV_CK			26
#define FM_MMPLL_CK				27
#define FM_MMPLL_D3_CK				28
#define FM_MPLL_CK				29
#define FM_MSDCPLL_CK				30
#define FM_RCKRPLL_DIV4_CH01			31
#define FM_IMGPLL_CK				32
#define FM_RPHYPLL_DIV4_CH01			33
#define FM_EMIPLL_CK				34
#define FM_TVDPLL_CK				35
#define FM_ULPOSC2_MON_V_VCORE_CK		36
#define FM_ULPOSC_MON_VCORE_CK			37
#define FM_UNIVPLL_CK				38
#define FM_UNIVPLL_192M_CK			40
#define FM_U_CLK2FREQ				41
#define FM_WBG_DIG_BPLL_CK			42
#define FM_WBG_DIG_WPLL_CK960			43
#define FMEM_AFT_CH0				44
#define FMEM_AFT_CH1				45
#define FMEM_BFE_CH0				48
#define FM_GMEM_BFE_CH1				49
#define FM_466M_FMEM_INFRASYS			50
#define FM_MCUSYS_ARM_OUT_ALL			51
#define FM_MSDC11_IN_CK				54
#define FM_MSDC12_IN_CK				55
#define FM_MSDC21_IN_CK				56
#define FM_MSDC22_IN_CK				57
#define FM_F32K_VCORE_CK			58
#define FM_ADA_LVTS_TO_PLLGP_MONCK_L6		64
#define FM_ADA_LVTS_TO_PLLGP_MONCK_L5		65
#define FM_ADA_LVTS_TO_PLLGP_MONCK_L4		66
#define FM_ADA_LVTS_TO_PLLGP_MONCK_L3		67
#define FM_ADA_LVTS_TO_PLLGP_MONCK_L2		68
#define FM_ADA_LVTS_TO_PLLGP_MONCK_L1		69
#define FM_LVTS_TO_PLLGP_MON_LM			70
#define FM_APLL1_CKDIV_CK			71
#define FM_APLL2_CKDIV_CK			72
#define FM_MPLL_CKDIV_CK			73
#define FM_TVDPLL_CKDIV_CK			74
#define FM_IMGPLL_CKDIV_CK			75
#define FM_EMIPLL_CKDIV_CK			76
#define FM_MSDCPLL_CKDIV_CK			77
/* ABIST2 Part */
#define FM_CKMON4_CK				1
#define FM_CKMON3_CK				2
#define FM_CKMON2_CK				3
#define FM_CKMON1_CK				4
#define FM_PMSRCK_CK				5
#define FM_MMPLL_D9_CK				6
#define FM_MMPLL_D7_CK				7
#define FM_MMPLL_D6_CK				8
#define FM_MMPLL_D5_CK				9
#define FM_MMPLL_D4_CK				10
#define FM_MMPLL_D3_CK_2			11
#define FM_UNIV_416M_CK				12
#define FM_UNIV_499M_CK				13
#define FM_UNIV_624M_CK				14
#define FM_UNIV_832M_CK				15
#define FM_UNIV_1248M_CK			16
#define FM_MAIN_H312M_CK			17
#define FM_MAIN_H436P8M_CK			18
#define FM_MAIN_H546M_CK			19
#define FM_MAIN_H728M_CK			20
#define FM_MAINPLL_1092M_CORE_CK		21
#define FM_SPMI_MST_32K_CK			22
#define FM_SRCK_CK				23
#define FM_UNIPLL_SES_CK			24
#define FM_APLL_I2S5_M_CK			25
#define FM_APLL_I2S4_B_CK			26
#define FM_APLL_I2S4_M_CK			27
#define FM_APLL_I2S3_M_CK			28
#define FM_APLL_I2S2_M_CK			29
#define FM_APLL_I2S1_M_CK			30
#define FM_APLL_I2S0_M_CK			31

enum fm_sys_id {
	FM_GPU_PLL_CTRL = 0,
	FM_APU_PLL_CTRL = 1,
	FM_SYS_NUM = 2,
};

#endif /* _CLK_MT6855_FMETER_H */
