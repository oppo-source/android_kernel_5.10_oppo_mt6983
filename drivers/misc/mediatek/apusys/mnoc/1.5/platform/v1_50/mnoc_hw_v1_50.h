/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __APUSYS_MNOC_HW_H__
#define __APUSYS_MNOC_HW_H__

enum apu_qos_mni {
	MNI_VPU0,
	MNI_VPU1,
	MNI_VPU2,
	MNI_MDLA0_0,
	MNI_MDLA0_1,
	MNI_MDLA1_0,
	MNI_MDLA1_1,
	MNI_EDMA0,
	MNI_EDMA1,
	MNI_MD32,

	NR_APU_QOS_MNI
};

enum apu_qos_engine {
	APU_QOS_ENGINE_VPU0,
	APU_QOS_ENGINE_VPU1,
	APU_QOS_ENGINE_VPU2,
	APU_QOS_ENGINE_MDLA0,
	APU_QOS_ENGINE_MDLA1,
	APU_QOS_ENGINE_EDMA0,
	APU_QOS_ENGINE_EDMA1,
	APU_QOS_ENGINE_MD32,

	NR_APU_QOS_ENGINE
};

enum mni_int_sta {
	MNOC_INT_MNI_QOS_IRQ_FLAG,
	MNOC_INT_ADDR_DEC_ERR_FLAG,
	MNOC_INT_MST_PARITY_ERR_FLAG,
	MNOC_INT_MST_MISRO_ERR_FLAG,
	MNOC_INT_MST_CRDT_ERR_FLAG,

	NR_MNI_INT_STA
};

enum sni_int_sta {
	MNOC_INT_SLV_PARITY_ERR_FLA,
	MNOC_INT_SLV_MISRO_ERR_FLAG,
	MNOC_INT_SLV_CRDT_ERR_FLAG,

	NR_SNI_INT_STA
};

enum rt_int_sta {
	MNOC_INT_REQRT_MISRO_ERR_FLAG,
	MNOC_INT_RSPRT_MISRO_ERR_FLAG,
	MNOC_INT_REQRT_TO_ERR_FLAG,
	MNOC_INT_RSPRT_TO_ERR_FLAG,
	MNOC_INT_REQRT_CBUF_ERR_FLAG,
	MNOC_INT_RSPRT_CBUF_ERR_FLAG,
	MNOC_INT_REQRT_CRDT_ERR_FLAG,
	MNOC_INT_RSPRT_CRDT_ERR_FLAG,

	NR_RT_INT_STA
};

#define NR_APU_ENGINE_VPU (3)
#define NR_APU_ENGINE_MDLA (2)
#define NR_APU_ENGINE_EDMA (2)

#define NR_MNOC_RT (5)
#define NR_GROUP (1)
#define NR_MNOC_MNI (16)
#define NR_MNOC_SNI (16)
#define NR_MNOC_PMU_CNTR (16)

/* 0x1906E000 */
#define APU_NOC_TOP_BASEADDR mnoc_base
/* 0x19001000 */
#define MNOC_INT_BASEADDR mnoc_int_base
/* 0x19020000 */
#define MNOC_APU_CONN_BASEADDR mnoc_apu_conn_base
/* 0x10001000 */
#define MNOC_SLP_PROT_BASEADDR1 mnoc_slp_prot_base1
/* 0x10215000 */
#define MNOC_SLP_PROT_BASEADDR2 mnoc_slp_prot_base2

/* MNoC register definition */
#define APUSYS_INT_EN (MNOC_INT_BASEADDR + 0x80)
#define APUSYS_INT_STA (MNOC_INT_BASEADDR + 0x34)
#define MNOC_INT_MAP (0x3)

#define APU_TCM_HASH_TRUNCATE_CTRL0 (MNOC_APU_CONN_BASEADDR + 0x7C)

/* #define APU_NOC_TOP_BASEADDR			(0x1906E000) */
#define APU_NOC_TOP_ADDR (0x1906E000)
#define APU_NOC_TOP_RANGE (0x2000)

#define APU_NOC_PMU_ADDR (0x1906E200)
#define APU_NOC_PMU_RANGE (0x48C)

#define MNI_QOS_CTRL_BASE (APU_NOC_TOP_BASEADDR + 0x1000)
#define MNI_QOS_INFO_BASE (APU_NOC_TOP_BASEADDR + 0x1800)
#define MNI_QOS_REG(base, reg_num, mni_offset) \
(base + reg_num*16*4 + mni_offset*4)

#define REQ_RT_PMU_BASE (APU_NOC_TOP_BASEADDR + 0x500)
#define RSP_RT_PMU_BASE (APU_NOC_TOP_BASEADDR + 0x600)
#define MNOC_RT_PMU_REG(base, reg_num, rt_num)	(base + reg_num*5*4 + rt_num*4)

#define MISC_CTRL (0x0)
#define SLV_QOS_CTRL1 (0x14)
#define MNI_QOS_IRQ_FLAG (0x18)
#define ADDR_DEC_ERR_FLAG (0x30)
#define MST_PARITY_ERR_FLAG (0x38)
#define SLV_PARITY_ERR_FLA (0x3C)
#define MST_MISRO_ERR_FLAG (0x40)
#define SLV_MISRO_ERR_FLAG (0x44)
#define REQRT_MISRO_ERR_FLAG (0x48)
#define RSPRT_MISRO_ERR_FLAG (0x4C)
#define REQRT_TO_ERR_FLAG (0x50)
#define RSPRT_TO_ERR_FLAG (0x54)
#define REQRT_CBUF_ERR_FLAG (0x188)
#define RSPRT_CBUF_ERR_FLAG (0x18C)
#define MST_CRDT_ERR_FLAG (0x190)
#define SLV_CRDT_ERR_FLAG (0x194)
#define REQRT_CRDT_ERR_FLAG (0x198)
#define RSPRT_CRDT_ERR_FLAG (0x19C)

#define MNOC_REG(offset) (APU_NOC_TOP_BASEADDR + offset)

#define PMU_COUNTER0_OUT (APU_NOC_TOP_BASEADDR + 0x240)

#define QG_LT_THL_PRE_ULTRA (0x1FFF)
#define QG_LT_THH_PRE_ULTRA (0x1FFF)

/* to deal with dual IOMMU tfrp addr hash aware problem */
#define NR_IOMMU (2)
#define APU_TFRP_ALIGN (0x1000)
#define INFRA_AO_BASE (0x10001000)
#define INFRA_AO_REG_SIZE (0x2000)
#define EMI_HASH_RULE_OFFSET (0x1050)
#define F_VAL(val, msb, lsb) (((val)&((1<<(msb-lsb+1))-1))<<lsb)
#define F_MSK(msb, lsb)	F_VAL(0xffffffff, msb, lsb)
/* F_RP_PA_REG_BIT32 = 0x7 */
#define F_RP_PA_REG_BIT32 F_MSK(2, 0)

struct int_sta_info {
	uint32_t reg_val;
	uint64_t timestamp;
};

struct mnoc_int_dump {
	uint32_t count;
	struct int_sta_info apusys_int_sta;
	struct int_sta_info mni_int_sta[NR_MNI_INT_STA];
	struct int_sta_info sni_int_sta[NR_SNI_INT_STA];
	struct int_sta_info rt_int_sta[NR_RT_INT_STA];
	struct int_sta_info sw_irq_sta;
};

#endif
