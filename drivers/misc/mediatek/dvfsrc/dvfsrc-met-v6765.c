// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/mtk_dvfsrc.h>
#include <linux/regulator/consumer.h>

#include "dvfsrc-met.h"
#include "dvfsrc-common.h"

static inline u32 dvfsrc_met_read(struct mtk_dvfsrc_met *dvfs, u32 offset)
{
	return readl(dvfs->regs + offset);
}

enum met_src_index {
	SRC_MD2SPM_IDX,
	DDR_OPP_IDX,
	DDR_SW_REQ1_PMQOS_IDX,
	DDR_SW_REQ2_CM_IDX,
	DDR_EMI_TOTAL_IDX,
	DDR_QOS_BW_IDX,
	VCORE_OPP_IDX,
	VCORE_SW_REQ1_PMQOS_IDX,
	VCORE_SW_REQ2_CM_IDX,
	VCORE_SCP_IDX,
	SRC_TOTAL_EMI_BW_IDX,
	SRC_PMQOS_TOTAL_IDX,
	SRC_PMQOS_BW0_IDX,
	SRC_PMQOS_BW1_IDX,
	SRC_PMQOS_BW2_IDX,
	SRC_PMQOS_BW3_IDX,
	SRC_PMQOS_BW4_IDX,
	SRC_MD_REQ_OPP,
	BWMON_TOTAL_BW_IDX,
	BWMON_CPU_BW_IDX,
	BWMON_GPU_BW_IDX,
	BWMON_MM_BW_IDX,
	SRC_MAX
};

/* met profile table */
static unsigned int met_vcorefs_src[SRC_MAX];

static char *met_src_name[SRC_MAX] = {
	"MD2SPM",
	"SRC_DDR_OPP",
	"DDR__SW_REQ1_PMQOS",
	"DDR__SW_REQ2_CM",
	"DDR__EMI_TOTAL",
	"DDR__QOS_BW",
	"SRC_VCORE_OPP",
	"VCORE__SW_REQ1_PMQOS",
	"VCORE__SW_REQ2_CM",
	"VCORE__SCP",
	"TOTAL_EMI_BW",
	"PMQOS_TOTAL",
	"PMQOS_BW0",
	"PMQOS_BW1",
	"PMQOS_BW2",
	"PMQOS_BW3",
	"PMQOS_BW4",
	"MD_REQ_OPP",
	"BWMON__TOTAL_BW",
	"BWMON__CPU_BW",
	"BWMON__GPU_BW",
	"BWMON__MM_BW",
};

#define DVFSRC_SW_REQ	(0x4)
#define DVFSRC_SW_REQ2	(0x8)
#define DVFSRC_VCORE_REQUEST (0x48)
#define DVFSRC_EMI_QOS0            (0x24)
#define DVFSRC_EMI_QOS1            (0x28)
#define DVFSRC_SW_BW_0	(0x160)
#define DVFSRC_SW_BW_1	(0x164)
#define DVFSRC_SW_BW_2	(0x168)
#define DVFSRC_SW_BW_3	(0x16C)
#define DVFSRC_SW_BW_4	(0x170)
#define DVFSRC_MD_SCENARIO         (0X310)
#define DVFSRC_RSRV_0              (0x600)
#define DVFSRC_LEVEL               (0xDC)
#define DVFSRC_EMI_MD2SPM0_T       (0x3C)
#define DVFSRC_EMI_MD2SPM1_T       (0x40)
#define DVFSRC_BASIC_CONTROL       (0x0)

#define DVFSRC_CURRENT_LEVEL(x)	(((x) >> 16) & 0x0000ffff)

/* DVFSRC_SW_REQ 0x4 */
#define EMI_SW_AP_SHIFT		0
#define EMI_SW_AP_MASK		0x3
#define VCORE_SW_AP_SHIFT	2
#define VCORE_SW_AP_MASK	0x3

/* DVFSRC_VCORE_REQUEST  */
#define VCORE_SCP_GEAR_SHIFT	30
#define VCORE_SCP_GEAR_MASK	0x3

/* met profile function */
static int dvfsrc_get_src_req_num(void)
{
	return SRC_MAX;
}

static char **dvfsrc_get_src_req_name(void)
{
	return met_src_name;
}

static u32 dvfsrc_get_current_level(struct mtk_dvfsrc_met *dvfsrc)
{
	return DVFSRC_CURRENT_LEVEL(dvfsrc_met_read(dvfsrc, DVFSRC_LEVEL));
}

static u32 dvfsrc_mt6768_ddr_qos(struct mtk_dvfsrc_met *dvfs, u32 bw)
{
	unsigned int qos0_thres = dvfsrc_met_read(dvfs, DVFSRC_EMI_QOS0);
	unsigned int qos1_thres = dvfsrc_met_read(dvfs, DVFSRC_EMI_QOS1);

	if (bw > qos1_thres)
		return 2;
	else if (bw > qos0_thres)
		return 1;
	else
		return 0;
}


static int dvfsrc_emi_mon_gear(struct mtk_dvfsrc_met *dvfs)
{
	//unsigned int total_bw_status = get_emi_bwst(0);
	//if (((total_bw_status >> 1) & 0x1) != 0)
	//	return 2;
	//else if ((total_bw_status & 0x1) != 0)
	//	return 1;
	//else
	//	return 0;
	return 0;
}

static void vcorefs_get_src_ddr_req(struct mtk_dvfsrc_met *dvfs)
{
	unsigned int sw_req;

	sw_req = dvfsrc_met_read(dvfs, DVFSRC_SW_REQ);
	met_vcorefs_src[DDR_SW_REQ1_PMQOS_IDX] =
		(sw_req >> EMI_SW_AP_SHIFT) & EMI_SW_AP_MASK;

	sw_req = dvfsrc_met_read(dvfs, DVFSRC_SW_REQ2);

	met_vcorefs_src[DDR_SW_REQ2_CM_IDX] =
		(sw_req >> EMI_SW_AP_SHIFT) & EMI_SW_AP_MASK;

	met_vcorefs_src[DDR_EMI_TOTAL_IDX] =
		dvfsrc_emi_mon_gear(dvfs);
}

static void vcorefs_get_src_vcore_req(struct mtk_dvfsrc_met *dvfs)
{
	u32 sw_req;

	sw_req = dvfsrc_met_read(dvfs, DVFSRC_SW_REQ);
	met_vcorefs_src[VCORE_SW_REQ1_PMQOS_IDX] =
		(sw_req >> VCORE_SW_AP_SHIFT) & VCORE_SW_AP_MASK;
	sw_req = dvfsrc_met_read(dvfs, DVFSRC_SW_REQ2);
	met_vcorefs_src[VCORE_SW_REQ2_CM_IDX] =
		(sw_req >> VCORE_SW_AP_SHIFT) & VCORE_SW_AP_MASK;
	sw_req = dvfsrc_met_read(dvfs, DVFSRC_VCORE_REQUEST);
	met_vcorefs_src[VCORE_SCP_IDX] =
		(sw_req >> VCORE_SCP_GEAR_SHIFT) & VCORE_SCP_GEAR_MASK;
}

static void vcorefs_get_src_misc_info(struct mtk_dvfsrc_met *dvfs)
{
	u32 qos_bw0, qos_bw1, qos_bw2, qos_bw3, qos_bw4;
	u32 total_bw;

	qos_bw0 = dvfsrc_met_read(dvfs, DVFSRC_SW_BW_0);
	qos_bw1 = dvfsrc_met_read(dvfs, DVFSRC_SW_BW_1);
	qos_bw2 = dvfsrc_met_read(dvfs, DVFSRC_SW_BW_2);
	qos_bw3 = dvfsrc_met_read(dvfs, DVFSRC_SW_BW_3);
	qos_bw4 = dvfsrc_met_read(dvfs, DVFSRC_SW_BW_4);
	total_bw = qos_bw0 + qos_bw1 + qos_bw2 + qos_bw3 + qos_bw4;
	met_vcorefs_src[SRC_MD2SPM_IDX] =
		dvfsrc_met_read(dvfs, DVFSRC_MD_SCENARIO);
	met_vcorefs_src[SRC_PMQOS_TOTAL_IDX] = total_bw;
	met_vcorefs_src[DDR_QOS_BW_IDX] =
		dvfsrc_mt6768_ddr_qos(dvfs, total_bw);
	met_vcorefs_src[SRC_PMQOS_BW0_IDX] = qos_bw0;
	met_vcorefs_src[SRC_PMQOS_BW1_IDX] = qos_bw1;
	met_vcorefs_src[SRC_PMQOS_BW2_IDX] = qos_bw2;
	met_vcorefs_src[SRC_PMQOS_BW3_IDX] = qos_bw3;
	met_vcorefs_src[SRC_PMQOS_BW4_IDX] = qos_bw4;
	met_vcorefs_src[SRC_MD_REQ_OPP] =
		dvfsrc_met_read(dvfs, DVFSRC_RSRV_0) & 0x3F;
}

static unsigned int *dvfsrc_get_src_req(struct mtk_dvfsrc_met *dvfs)
{
	vcorefs_get_src_ddr_req(dvfs);
	vcorefs_get_src_vcore_req(dvfs);
	vcorefs_get_src_misc_info(dvfs);

	return met_vcorefs_src;
}

static int dvfsrc_get_ddr_ratio(struct mtk_dvfsrc_met *dvfs)
{
	return 0;
}

const struct dvfsrc_met_config mt6768_met_config = {
	.dvfsrc_get_src_req_num = dvfsrc_get_src_req_num,
	.dvfsrc_get_src_req_name = dvfsrc_get_src_req_name,
	.dvfsrc_get_src_req = dvfsrc_get_src_req,
	.dvfsrc_get_ddr_ratio = dvfsrc_get_ddr_ratio,
	.get_current_level = dvfsrc_get_current_level,
};

const struct dvfsrc_met_config mt6765_met_config = {
	.dvfsrc_get_src_req_num = dvfsrc_get_src_req_num,
	.dvfsrc_get_src_req_name = dvfsrc_get_src_req_name,
	.dvfsrc_get_src_req = dvfsrc_get_src_req,
	.dvfsrc_get_ddr_ratio = dvfsrc_get_ddr_ratio,
	.get_current_level = dvfsrc_get_current_level,
};

