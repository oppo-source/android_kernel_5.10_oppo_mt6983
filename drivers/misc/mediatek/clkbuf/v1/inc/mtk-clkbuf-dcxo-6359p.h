/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: ren-ting.wang <ren-ting.wang@mediatek.com>
 */
#ifndef CLKBUF_DCXO_6359P_H
#define CLKBUF_DCXO_6359P_H

#include "mtk-clkbuf-dcxo.h"

extern struct dcxo_hw mt6359p_dcxo;

#define MT6359P_PMRC_CON0			(0x1A6)
#define MT6359P_TOP_SPI_CON1			(0x458)
#define MT6359P_XO_BUF_CTL0			(0x54c)
#define MT6359P_XO_BUF_CTL1			(0x54e)
#define MT6359P_XO_BUF_CTL2			(0x550)
#define MT6359P_XO_BUF_CTL3			(0x552)
#define MT6359P_XO_BUF_CTL4			(0x554)
#define MT6359P_DCXO_CW00			(0x788)
#define MT6359P_DCXO_CW08			(0x79c)
#define MT6359P_DCXO_CW09			(0x79e)
#define MT6359P_DCXO_CW12			(0x7a8)
#define MT6359P_DCXO_CW13			(0x7aa)
#define MT6359P_DCXO_CW16			(0x7b0)
#define MT6359P_DCXO_CW17			(0x7b2)
#define MT6359P_DCXO_CW19			(0x7b6)
#define MT6359P_LDO_VRFCK_ELR			(0x1b40)
#define MT6359P_LDO_VRFCK_CON0			(0x1d1c)
#define MT6359P_LDO_VRFCK_OP_EN			(0x1d22)
#define MT6359P_LDO_VRFCK_OP_EN_SET		(0x1d24)
#define MT6359P_LDO_VRFCK_OP_EN_CLR		(0x1d26)
#define MT6359P_LDO_VBBCK_CON0			(0x1d2e)
#define MT6359P_LDO_VBBCK_OP_EN			(0x1d34)
#define MT6359P_LDO_VBBCK_OP_EN_SET		(0x1d36)
#define MT6359P_LDO_VBBCK_OP_EN_CLR		(0x1d38)
#define MT6359P_DCXO_ADLDO_BIAS_ELR_0		(0x209c)
#define MT6359P_DCXO_ADLDO_BIAS_ELR_1		(0x209e)

/* Register_TOP_REG */
#define MT6359P_PMRC_EN_ADDR			(MT6359P_PMRC_CON0)
#define MT6359P_PMRC_EN_MASK			(0xFFFF)
#define MT6359P_PMRC_EN_SHIFT			(0)
/* Register_PLT_REG*/
#define MT6359P_RG_SRCLKEN_IN3_EN_ADDR		(MT6359P_TOP_SPI_CON1)
#define MT6359P_RG_SRCLKEN_IN3_EN_MASK		(0x1)
#define MT6359P_RG_SRCLKEN_IN3_EN_SHIFT		(0)
/* Register_SCK_REG*/
#define MT6359P_XO_SOC_VOTE_ADDR		(MT6359P_XO_BUF_CTL0)
#define MT6359P_XO_SOC_VOTE_MASK		(0x7ff)
#define MT6359P_XO_SOC_VOTE_SHIFT		(0)
#define MT6359P_XO_WCN_VOTE_ADDR		(MT6359P_XO_BUF_CTL1)
#define MT6359P_XO_WCN_VOTE_MASK		(0x7ff)
#define MT6359P_XO_WCN_VOTE_SHIFT		(0)
#define MT6359P_XO_NFC_VOTE_ADDR		(MT6359P_XO_BUF_CTL2)
#define MT6359P_XO_NFC_VOTE_MASK		(0x7ff)
#define MT6359P_XO_NFC_VOTE_SHIFT		(0)
#define MT6359P_XO_CEL_VOTE_ADDR		(MT6359P_XO_BUF_CTL3)
#define MT6359P_XO_CEL_VOTE_MASK		(0x7ff)
#define MT6359P_XO_CEL_VOTE_SHIFT		(0)
#define MT6359P_XO_EXT_VOTE_ADDR		(MT6359P_XO_BUF_CTL4)
#define MT6359P_XO_EXT_VOTE_MASK		(0x7ff)
#define MT6359P_XO_EXT_VOTE_SHIFT		(0)
/* Register_DCXO_REG*/
#define MT6359P_XO_EXTBUF1_MODE_ADDR		(MT6359P_DCXO_CW00)
#define MT6359P_XO_EXTBUF1_MODE_MASK		(0x3)
#define MT6359P_XO_EXTBUF1_MODE_SHIFT		(0)
#define MT6359P_XO_EXTBUF1_EN_M_ADDR		(MT6359P_DCXO_CW00)
#define MT6359P_XO_EXTBUF1_EN_M_MASK		(0x1)
#define MT6359P_XO_EXTBUF1_EN_M_SHIFT		(2)
#define MT6359P_XO_EXTBUF2_MODE_ADDR		(MT6359P_DCXO_CW00)
#define MT6359P_XO_EXTBUF2_MODE_MASK		(0x3)
#define MT6359P_XO_EXTBUF2_MODE_SHIFT		(3)
#define MT6359P_XO_EXTBUF2_EN_M_ADDR		(MT6359P_DCXO_CW00)
#define MT6359P_XO_EXTBUF2_EN_M_MASK		(0x1)
#define MT6359P_XO_EXTBUF2_EN_M_SHIFT		(5)
#define MT6359P_XO_EXTBUF3_MODE_ADDR		(MT6359P_DCXO_CW00)
#define MT6359P_XO_EXTBUF3_MODE_MASK		(0x3)
#define MT6359P_XO_EXTBUF3_MODE_SHIFT		(6)
#define MT6359P_XO_EXTBUF3_EN_M_ADDR		(MT6359P_DCXO_CW00)
#define MT6359P_XO_EXTBUF3_EN_M_MASK		(0x1)
#define MT6359P_XO_EXTBUF3_EN_M_SHIFT		(8)
#define MT6359P_XO_EXTBUF4_MODE_ADDR		(MT6359P_DCXO_CW00)
#define MT6359P_XO_EXTBUF4_MODE_MASK		(0x3)
#define MT6359P_XO_EXTBUF4_MODE_SHIFT		(9)
#define MT6359P_XO_EXTBUF4_EN_M_ADDR		(MT6359P_DCXO_CW00)
#define MT6359P_XO_EXTBUF4_EN_M_MASK		(0x1)
#define MT6359P_XO_EXTBUF4_EN_M_SHIFT		(11)
#define MT6359P_XO_BB_LPM_EN_M_ADDR		(MT6359P_DCXO_CW00)
#define MT6359P_XO_BB_LPM_EN_M_MASK		(0x1)
#define MT6359P_XO_BB_LPM_EN_M_SHIFT		(12)
#define MT6359P_XO_PMIC_TOP_DIG_SW_ADDR		(MT6359P_DCXO_CW08)
#define MT6359P_XO_PMIC_TOP_DIG_SW_MASK		(0x1)
#define MT6359P_XO_PMIC_TOP_DIG_SW_SHIFT	(2)
#define MT6359P_XO_EXTBUF6_MODE_ADDR		(MT6359P_DCXO_CW09)
#define MT6359P_XO_EXTBUF6_MODE_MASK		(0x3)
#define MT6359P_XO_EXTBUF6_MODE_SHIFT		(9)
#define MT6359P_XO_EXTBUF6_EN_M_ADDR		(MT6359P_DCXO_CW09)
#define MT6359P_XO_EXTBUF6_EN_M_MASK		(0x1)
#define MT6359P_XO_EXTBUF6_EN_M_SHIFT		(11)
#define MT6359P_XO_EXTBUF7_MODE_ADDR		(MT6359P_DCXO_CW09)
#define MT6359P_XO_EXTBUF7_MODE_MASK		(0x3)
#define MT6359P_XO_EXTBUF7_MODE_SHIFT		(12)
#define MT6359P_XO_EXTBUF7_EN_M_ADDR		(MT6359P_DCXO_CW09)
#define MT6359P_XO_EXTBUF7_EN_M_MASK		(0x1)
#define MT6359P_XO_EXTBUF7_EN_M_SHIFT		(14)
#define MT6359P_XO_BB_LPM_EN_SEL_ADDR		(MT6359P_DCXO_CW12)
#define MT6359P_XO_BB_LPM_EN_SEL_MASK		(0x1)
#define MT6359P_XO_BB_LPM_EN_SEL_SHIFT		(0)
#define MT6359P_XO_EXTBUF1_BBLPM_EN_MASK_ADDR	(MT6359P_DCXO_CW12)
#define MT6359P_XO_EXTBUF1_BBLPM_EN_MASK_MASK	(0x1)
#define MT6359P_XO_EXTBUF1_BBLPM_EN_MASK_SHIFT	(1)
#define MT6359P_XO_EXTBUF2_BBLPM_EN_MASK_ADDR	(MT6359P_DCXO_CW12)
#define MT6359P_XO_EXTBUF2_BBLPM_EN_MASK_MASK	(0x1)
#define MT6359P_XO_EXTBUF2_BBLPM_EN_MASK_SHIFT	(2)
#define MT6359P_XO_EXTBUF3_BBLPM_EN_MASK_ADDR	(MT6359P_DCXO_CW12)
#define MT6359P_XO_EXTBUF3_BBLPM_EN_MASK_MASK	(0x1)
#define MT6359P_XO_EXTBUF3_BBLPM_EN_MASK_SHIFT	(3)
#define MT6359P_XO_EXTBUF4_BBLPM_EN_MASK_ADDR	(MT6359P_DCXO_CW12)
#define MT6359P_XO_EXTBUF4_BBLPM_EN_MASK_MASK	(0x1)
#define MT6359P_XO_EXTBUF4_BBLPM_EN_MASK_SHIFT	(4)
#define MT6359P_XO_EXTBUF6_BBLPM_EN_MASK_ADDR	(MT6359P_DCXO_CW12)
#define MT6359P_XO_EXTBUF6_BBLPM_EN_MASK_MASK	(0x1)
#define MT6359P_XO_EXTBUF6_BBLPM_EN_MASK_SHIFT	(5)
#define MT6359P_XO_EXTBUF7_BBLPM_EN_MASK_ADDR	(MT6359P_DCXO_CW12)
#define MT6359P_XO_EXTBUF7_BBLPM_EN_MASK_MASK	(0x1)
#define MT6359P_XO_EXTBUF7_BBLPM_EN_MASK_SHIFT	(6)
#define MT6359P_RG_XO_EXTBUF2_SRSEL_ADDR	(MT6359P_DCXO_CW13)
#define MT6359P_RG_XO_EXTBUF2_SRSEL_MASK	(0x7)
#define MT6359P_RG_XO_EXTBUF2_SRSEL_SHIFT	(0)
#define MT6359P_RG_XO_EXTBUF4_SRSEL_ADDR	(MT6359P_DCXO_CW13)
#define MT6359P_RG_XO_EXTBUF4_SRSEL_MASK	(0x7)
#define MT6359P_RG_XO_EXTBUF4_SRSEL_SHIFT	(4)
#define MT6359P_RG_XO_EXTBUF3_HD_ADDR		(MT6359P_DCXO_CW13)
#define MT6359P_RG_XO_EXTBUF3_HD_MASK		(0x3)
#define MT6359P_RG_XO_EXTBUF3_HD_SHIFT		(10)
#define MT6359P_XO_STATIC_AUXOUT_SEL_ADDR	(MT6359P_DCXO_CW16)
#define MT6359P_XO_STATIC_AUXOUT_SEL_MASK	(0x3f)
#define MT6359P_XO_STATIC_AUXOUT_SEL_SHIFT	(0)
#define MT6359P_XO_AUXOUT_SEL_ADDR		(MT6359P_DCXO_CW16)
#define MT6359P_XO_AUXOUT_SEL_MASK		(0x3ff)
#define MT6359P_XO_AUXOUT_SEL_SHIFT		(6)
#define MT6359P_XO_STATIC_AUXOUT_ADDR		(MT6359P_DCXO_CW17)
#define MT6359P_XO_STATIC_AUXOUT_MASK		(0xffff)
#define MT6359P_XO_STATIC_AUXOUT_SHIFT		(0)
#define MT6359P_RG_XO_EXTBUF1_RSEL_ADDR		(MT6359P_DCXO_CW19)
#define MT6359P_RG_XO_EXTBUF1_RSEL_MASK		(0x7)
#define MT6359P_RG_XO_EXTBUF1_RSEL_SHIFT	(1)
#define MT6359P_RG_XO_EXTBUF2_RSEL_ADDR		(MT6359P_DCXO_CW19)
#define MT6359P_RG_XO_EXTBUF2_RSEL_MASK		(0x7)
#define MT6359P_RG_XO_EXTBUF2_RSEL_SHIFT	(4)
#define MT6359P_RG_XO_EXTBUF3_RSEL_ADDR		(MT6359P_DCXO_CW19)
#define MT6359P_RG_XO_EXTBUF3_RSEL_MASK		(0x7)
#define MT6359P_RG_XO_EXTBUF3_RSEL_SHIFT	(7)
#define MT6359P_RG_XO_EXTBUF4_RSEL_ADDR		(MT6359P_DCXO_CW19)
#define MT6359P_RG_XO_EXTBUF4_RSEL_MASK		(0x7)
#define MT6359P_RG_XO_EXTBUF4_RSEL_SHIFT	(10)
#define MT6359P_RG_XO_EXTBUF7_RSEL_ADDR		(MT6359P_DCXO_CW19)
#define MT6359P_RG_XO_EXTBUF7_RSEL_MASK		(0x7)
#define MT6359P_RG_XO_EXTBUF7_RSEL_SHIFT	(13)
/* Register_LDO_REG*/
#define MT6359P_RG_LDO_VRFCK_ANA_SEL_ADDR	(MT6359P_LDO_VRFCK_ELR)
#define MT6359P_RG_LDO_VRFCK_ANA_SEL_MASK	(0x1)
#define MT6359P_RG_LDO_VRFCK_ANA_SEL_SHIFT	(0)
#define MT6359P_RG_LDO_VRFCK_EN_ADDR		(MT6359P_LDO_VRFCK_CON0)
#define MT6359P_RG_LDO_VRFCK_EN_MASK		(0x1)
#define MT6359P_RG_LDO_VRFCK_EN_SHIFT		(0)
#define MT6359P_RG_LDO_VRFCK_HW14_OP_EN_ADDR	(MT6359P_LDO_VRFCK_OP_EN)
#define MT6359P_RG_LDO_VRFCK_HW14_OP_EN_MASK	(0x1)
#define MT6359P_RG_LDO_VRFCK_HW14_OP_EN_SHIFT	(14)
#define MT6359P_RG_LDO_VBBCK_EN_ADDR		(MT6359P_LDO_VBBCK_CON0)
#define MT6359P_RG_LDO_VBBCK_EN_MASK		(0x1)
#define MT6359P_RG_LDO_VBBCK_EN_SHIFT		(0)
#define MT6359P_RG_LDO_VBBCK_HW14_OP_EN_ADDR	(MT6359P_LDO_VBBCK_OP_EN)
#define MT6359P_RG_LDO_VBBCK_HW14_OP_EN_MASK	(0x1)
#define MT6359P_RG_LDO_VBBCK_HW14_OP_EN_SHIFT	(14)
#define MT6359P_RG_VRFCK_HV_EN_ADDR		(MT6359P_DCXO_ADLDO_BIAS_ELR_0)
#define MT6359P_RG_VRFCK_HV_EN_MASK		(0x1)
#define MT6359P_RG_VRFCK_HV_EN_SHIFT		(9)
#define MT6359P_RG_VRFCK_NDIS_EN_ADDR		(MT6359P_DCXO_ADLDO_BIAS_ELR_0)
#define MT6359P_RG_VRFCK_NDIS_EN_MASK		(0x1)
#define MT6359P_RG_VRFCK_NDIS_EN_SHIFT		(11)
#define MT6359P_RG_VRFCK_1_NDIS_EN_ADDR		(MT6359P_DCXO_ADLDO_BIAS_ELR_1)
#define MT6359P_RG_VRFCK_1_NDIS_EN_MASK		(0x1)
#define MT6359P_RG_VRFCK_1_NDIS_EN_SHIFT	(0)

#endif /* CLKBUF_DCXO_6359P_H */
