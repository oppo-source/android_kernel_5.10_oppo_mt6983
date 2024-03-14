// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#ifndef DRM_CMDQ_DISABLE
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#else
#include "mtk-cmdq-ext.h"
#endif

#include <soc/mediatek/smi.h>
#if IS_ENABLED(CONFIG_MTK_IOMMU_MISC_DBG)
#include "iommu_debug.h"
#endif

#include "mtk_drm_drv.h"
#include "mtk_drm_plane.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_gem.h"
#include "mtk_drm_mmp.h"
#include "mtk_dump.h"
#include "platform/mtk_drm_6789.h"

#define MTK_SMI_CLK_CTRL
#ifdef MTK_SMI_CLK_CTRL
#include <soc/mediatek/smi.h>
#endif

#define DISP_OD_EN 0x0000
#define DISP_OD_INTEN 0x0008
#define DISP_OD_INTSTA 0x000c
#define DISP_OD_CFG 0x0020
#define DISP_OD_SIZE 0x0030
#define DISP_DITHER_5 0x0114
#define DISP_DITHER_7 0x011c
#define DISP_DITHER_15 0x013c
#define DISP_DITHER_16 0x0140

#define DISP_REG_SPLIT_START 0x0000

#define DISP_REG_UFO_START 0x0000
#define DISP_REG_UFO_WIDTH 0x0050
#define DISP_REG_UFO_HEIGHT 0x0054

#define OD_RELAYMODE BIT(0)

#define UFO_BYPASS BIT(2)
#define UFO_LR (BIT(3) | BIT(0))

#define DISP_DITHERING BIT(2)
#define DITHER_LSB_ERR_SHIFT_R(x) (((x)&0x7) << 28)
#define DITHER_OVFLW_BIT_R(x) (((x)&0x7) << 24)
#define DITHER_ADD_LSHIFT_R(x) (((x)&0x7) << 20)
#define DITHER_ADD_RSHIFT_R(x) (((x)&0x7) << 16)
#define DITHER_NEW_BIT_MODE BIT(0)
#define DITHER_LSB_ERR_SHIFT_B(x) (((x)&0x7) << 28)
#define DITHER_OVFLW_BIT_B(x) (((x)&0x7) << 24)
#define DITHER_ADD_LSHIFT_B(x) (((x)&0x7) << 20)
#define DITHER_ADD_RSHIFT_B(x) (((x)&0x7) << 16)
#define DITHER_LSB_ERR_SHIFT_G(x) (((x)&0x7) << 12)
#define DITHER_OVFLW_BIT_G(x) (((x)&0x7) << 8)
#define DITHER_ADD_LSHIFT_G(x) (((x)&0x7) << 4)
#define DITHER_ADD_RSHIFT_G(x) (((x)&0x7) << 0)

#define MT6873_SODI_REQ_SEL_ALL                   REG_FLD_MSB_LSB(9, 8)

#define MT6873_SODI_REQ_VAL_ALL                   REG_FLD_MSB_LSB(13, 12)

#define MT6765_HRT_URGENT_CTL_SEL_ALL             REG_FLD_MSB_LSB(7, 0)
	#define MT6765_HRT_URGENT_CTL_SEL_RDMA0       REG_FLD_MSB_LSB(0, 0)
	#define MT6765_HRT_URGENT_CTL_SEL_WDMA0       REG_FLD_MSB_LSB(2, 2)
	#define MT6765_HRT_URGENT_CTL_SEL_DSI0        REG_FLD_MSB_LSB(5, 5)

#define MT6765_HRT_URGENT_CTL_VAL_ALL             REG_FLD_MSB_LSB(15, 8)
	#define MT6765_HRT_URGENT_CTL_VAL_RDMA0       REG_FLD_MSB_LSB(8, 8)
	#define MT6765_HRT_URGENT_CTL_VAL_WDMA0       REG_FLD_MSB_LSB(10, 10)
	#define MT6765_HRT_URGENT_CTL_VAL_DSI0        REG_FLD_MSB_LSB(13, 13)

#define MT6768_HRT_URGENT_CTL_SEL_ALL             REG_FLD_MSB_LSB(7, 0)
	#define MT6768_HRT_URGENT_CTL_SEL_RDMA0       REG_FLD_MSB_LSB(0, 0)
	#define MT6768_HRT_URGENT_CTL_SEL_WDMA0       REG_FLD_MSB_LSB(2, 2)
	#define MT6768_HRT_URGENT_CTL_SEL_DSI0        REG_FLD_MSB_LSB(5, 5)

#define MT6768_HRT_URGENT_CTL_VAL_ALL             REG_FLD_MSB_LSB(15, 8)
	#define MT6768_HRT_URGENT_CTL_VAL_RDMA0       REG_FLD_MSB_LSB(8, 8)
	#define MT6768_HRT_URGENT_CTL_VAL_WDMA0       REG_FLD_MSB_LSB(10, 10)
	#define MT6768_HRT_URGENT_CTL_VAL_DSI0        REG_FLD_MSB_LSB(13, 13)

#define MT6879_DVFS_HALT_MASK_SEL_ALL             REG_FLD_MSB_LSB(21, 16)
	#define MT6879_DVFS_HALT_MASK_SEL_RDMA0       REG_FLD_MSB_LSB(16, 16)
	#define MT6879_DVFS_HALT_MASK_SEL_RDMA1       REG_FLD_MSB_LSB(17, 17)
	#define MT6879_DVFS_HALT_MASK_SEL_RDMA4       REG_FLD_MSB_LSB(18, 18)
	#define MT6879_DVFS_HALT_MASK_SEL_RDMA5       REG_FLD_MSB_LSB(19, 19)
	#define MT6879_DVFS_HALT_MASK_SEL_WDMA0       REG_FLD_MSB_LSB(20, 20)
	#define MT6879_DVFS_HALT_MASK_SEL_WDMA1       REG_FLD_MSB_LSB(21, 21)

#define MT6879_HRT_URGENT_CTL_SEL_ALL             REG_FLD_MSB_LSB(7, 0)
	#define MT6879_HRT_URGENT_CTL_SEL_RDMA0       REG_FLD_MSB_LSB(0, 0)
	#define MT6879_HRT_URGENT_CTL_SEL_RDMA1       REG_FLD_MSB_LSB(1, 1)
	#define MT6879_HRT_URGENT_CTL_SEL_WDMA0       REG_FLD_MSB_LSB(2, 2)
	#define MT6879_HRT_URGENT_CTL_SEL_WDMA1       REG_FLD_MSB_LSB(3, 3)
	#define MT6879_HRT_URGENT_CTL_SEL_DSI0        REG_FLD_MSB_LSB(5, 5)
	#define MT6879_HRT_URGENT_CTL_SEL_MDP_RDMA0   REG_FLD_MSB_LSB(6, 6)

#define MT6855_HRT_URGENT_CTL_SEL_ALL             REG_FLD_MSB_LSB(7, 0)
	#define MT6855_HRT_URGENT_CTL_SEL_RDMA0       REG_FLD_MSB_LSB(0, 0)
	#define MT6855_HRT_URGENT_CTL_SEL_WDMA0       REG_FLD_MSB_LSB(2, 2)
	#define MT6855_HRT_URGENT_CTL_SEL_DSI0        REG_FLD_MSB_LSB(5, 5)

#define MT6879_HRT_URGENT_CTL_VAL_ALL             REG_FLD_MSB_LSB(15, 8)
	#define MT6879_HRT_URGENT_CTL_VAL_RDMA0       REG_FLD_MSB_LSB(8, 8)
	#define MT6879_HRT_URGENT_CTL_VAL_RDMA1       REG_FLD_MSB_LSB(9, 9)
	#define MT6879_HRT_URGENT_CTL_VAL_WDMA0       REG_FLD_MSB_LSB(10, 10)
	#define MT6879_HRT_URGENT_CTL_VAL_WDMA1       REG_FLD_MSB_LSB(11, 11)
	#define MT6879_HRT_URGENT_CTL_VAL_DSI0        REG_FLD_MSB_LSB(13, 13)
	#define MT6879_HRT_URGENT_CTL_VAL_MDP_RDMA4   REG_FLD_MSB_LSB(15, 15)

#define MT6855_HRT_URGENT_CTL_VAL_ALL             REG_FLD_MSB_LSB(15, 8)
	#define MT6855_HRT_URGENT_CTL_VAL_RDMA0       REG_FLD_MSB_LSB(8, 8)
	#define MT6855_HRT_URGENT_CTL_VAL_WDMA0       REG_FLD_MSB_LSB(10, 10)
	#define MT6855_HRT_URGENT_CTL_VAL_DSI0        REG_FLD_MSB_LSB(13, 13)

#define MT6833_INFRA_DISP_DDR_CTL  0x2C
#define MT6833_INFRA_FLD_DDR_MASK  REG_FLD_MSB_LSB(7, 4)

#define MT6895_FLD_OVL0_RDMA_ULTRA_SEL            REG_FLD_MSB_LSB(5, 2)
#define MT6895_FLD_OVL0_2L_RDMA_ULTRA_SEL         REG_FLD_MSB_LSB(9, 6)
#define MT6895_FLD_OVL1_2L_RDMA_ULTRA_SEL         REG_FLD_MSB_LSB(13, 10)
#define MT6879_FLD_OVL0_RDMA_ULTRA_SEL            REG_FLD_MSB_LSB(5, 2)
#define MT6879_FLD_OVL0_2L_RDMA_ULTRA_SEL         REG_FLD_MSB_LSB(9, 6)
#define MT6879_FLD_OVL0_2L_NWCG_RDMA_ULTRA_SEL    REG_FLD_MSB_LSB(17, 14)
#define MT6855_FLD_OVL0_RDMA_ULTRA_SEL            REG_FLD_MSB_LSB(5, 2)
#define MT6855_FLD_OVL1_2L_RDMA_ULTRA_SEL         REG_FLD_MSB_LSB(13, 10)

#define SMI_LARB_NON_SEC_CON 0x0380

#define MTK_DDP_COMP_USER "DISP"

#if defined(CONFIG_PXLW_IRIS)
int mtk_ddp_write(struct mtk_ddp_comp *comp, unsigned int value,
		   unsigned int offset, void *handle)
{
	int ret = 0;
#ifndef DRM_CMDQ_DISABLE
	ret = cmdq_pkt_write((struct cmdq_pkt *)handle, comp->cmdq_base,
		       comp->regs_pa + offset, value, ~0);
	if (ret < 0)
		DDPPR_ERR("%s:%d, cmdq error! ret:%d\n",
				__func__, __LINE__, ret);
#else
	writel(value, comp->regs + offset);
#endif
	return ret;
}

int mtk_ddp_write_relaxed(struct mtk_ddp_comp *comp, unsigned int value,
			   unsigned int offset, void *handle)
{
	int ret = 0;
#ifndef DRM_CMDQ_DISABLE
	if (handle) {
		ret = cmdq_pkt_write((struct cmdq_pkt *)handle, comp->cmdq_base,
		       comp->regs_pa + offset, value, ~0);
		if (ret < 0)
			DDPPR_ERR("%s:%d, cmdq error! ret:%d\n",
					__func__, __LINE__, ret);
		return ret;
	}
#endif
	writel_relaxed(value, comp->regs + offset);
	return ret;
}

int mtk_ddp_write_mask(struct mtk_ddp_comp *comp, unsigned int value,
			unsigned int offset, unsigned int mask, void *handle)
{
	int ret = 0;
	unsigned int tmp;
#ifndef DRM_CMDQ_DISABLE
	if(handle) {
		ret = cmdq_pkt_write((struct cmdq_pkt *)handle, comp->cmdq_base,
		    comp->regs_pa + offset, value, mask);
		if (ret < 0)
		DDPPR_ERR("%s:%d, cmdq error! ret:%d\n",
				__func__, __LINE__, ret);
		return ret;
	}
#endif
	tmp = readl(comp->regs + offset);
	tmp = (tmp & ~mask) | (value & mask);
	writel(tmp, comp->regs + offset);
	return ret;
}
#else
void mtk_ddp_write(struct mtk_ddp_comp *comp, unsigned int value,
		   unsigned int offset, void *handle)
{
#ifndef DRM_CMDQ_DISABLE
	cmdq_pkt_write((struct cmdq_pkt *)handle, comp->cmdq_base,
		       comp->regs_pa + offset, value, ~0);
#else
	writel(value, comp->regs + offset);
#endif
}

//#ifdef OPLUS_ADFR
void mtk_ddp_write_relaxed(struct mtk_ddp_comp *comp, unsigned int value,
			   unsigned int offset, void *handle)
{
#ifndef DRM_CMDQ_DISABLE
	if (handle) {
		cmdq_pkt_write((struct cmdq_pkt *)handle, comp->cmdq_base,
		       comp->regs_pa + offset, value, ~0);
		return;
	}
#endif
	writel_relaxed(value, comp->regs + offset);

}
//#endif

void mtk_ddp_write_mask(struct mtk_ddp_comp *comp, unsigned int value,
			unsigned int offset, unsigned int mask, void *handle)
{
	unsigned int tmp;

#ifndef DRM_CMDQ_DISABLE
	if (handle) {
		cmdq_pkt_write((struct cmdq_pkt *)handle, comp->cmdq_base,
		       comp->regs_pa + offset, value, mask);
		return;
	}
#endif
	tmp = readl(comp->regs + offset);
	tmp = (tmp & ~mask) | (value & mask);
	writel(tmp, comp->regs + offset);
}
#endif /* CONFIG_PXLW_IRIS */

void mtk_ddp_write_mask_cpu(struct mtk_ddp_comp *comp,
	unsigned int value, unsigned int offset, unsigned int mask)
{
	unsigned int tmp = readl(comp->regs + offset);

	tmp = (tmp & ~mask) | (value & mask);
	writel(tmp, comp->regs + offset);
}

void mtk_dither_set(struct mtk_ddp_comp *comp, unsigned int bpc,
		    unsigned int CFG, struct cmdq_pkt *handle)
{
	/* If bpc equal to 0, the dithering function didn't be enabled */
	if (bpc == 0)
		return;

	if (bpc >= MTK_MIN_BPC) {
		cmdq_pkt_write(handle, comp->cmdq_base,
			       comp->regs_pa + DISP_DITHER_5, 0, ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			       comp->regs_pa + DISP_DITHER_7, 0, ~0);
		cmdq_pkt_write(handle, comp->cmdq_base,
			       comp->regs_pa + DISP_DITHER_15,
			       DITHER_LSB_ERR_SHIFT_R(MTK_MAX_BPC - bpc) |
				       DITHER_ADD_LSHIFT_R(MTK_MAX_BPC - bpc) |
				       DITHER_NEW_BIT_MODE,
			       ~0);
		cmdq_pkt_write(
			handle, comp->cmdq_base, comp->regs_pa + DISP_DITHER_16,
			DITHER_LSB_ERR_SHIFT_B(MTK_MAX_BPC - bpc) |
				DITHER_ADD_LSHIFT_B(MTK_MAX_BPC - bpc) |
				DITHER_LSB_ERR_SHIFT_G(MTK_MAX_BPC - bpc) |
				DITHER_ADD_LSHIFT_G(MTK_MAX_BPC - bpc),
			~0);
		cmdq_pkt_write(handle, comp->cmdq_base, comp->regs_pa + CFG,
			       DISP_DITHERING, ~0);
	}
}

static void mtk_od_config(struct mtk_ddp_comp *comp, struct mtk_ddp_config *cfg,
			  struct cmdq_pkt *handle)
{
	cmdq_pkt_write(handle, comp->cmdq_base, comp->regs_pa + DISP_OD_SIZE,
		       cfg->w << 16 | cfg->h, ~0);
	cmdq_pkt_write(handle, comp->cmdq_base, comp->regs_pa + DISP_OD_CFG,
		       OD_RELAYMODE, ~0);
	mtk_dither_set(comp, cfg->bpc, DISP_OD_CFG, handle);
}

static void mtk_od_start(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle)
{
	cmdq_pkt_write(handle, comp->cmdq_base, comp->regs_pa + DISP_OD_EN, 1,
		       ~0);
}

static void mtk_ufoe_config(struct mtk_ddp_comp *comp,
			    struct mtk_ddp_config *cfg, struct cmdq_pkt *handle)
{
	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_UFO_WIDTH, cfg->w, ~0);
	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_UFO_HEIGHT, cfg->h, ~0);
}

static void mtk_ufoe_start(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle)
{
	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_UFO_START, UFO_BYPASS, ~0);
}

static void mtk_split_start(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle)
{
	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_REG_SPLIT_START, 1, ~0);
}

static const struct mtk_ddp_comp_funcs ddp_od = {
	.config = mtk_od_config, .start = mtk_od_start,
};

static const struct mtk_ddp_comp_funcs ddp_ufoe = {
	.start = mtk_ufoe_start, .config = mtk_ufoe_config,
};

static const struct mtk_ddp_comp_funcs ddp_split = {
	.start = mtk_split_start,
};

static const char *const mtk_ddp_comp_stem[MTK_DDP_COMP_TYPE_MAX] = {
	[MTK_DISP_OVL] = "ovl",
	[MTK_DISP_RDMA] = "rdma",
	[MTK_DISP_WDMA] = "wdma",
	[MTK_DISP_COLOR] = "color",
	[MTK_DISP_CCORR] = "ccorr",
	[MTK_DISP_AAL] = "aal",
	[MTK_DISP_GAMMA] = "gamma",
	[MTK_DISP_DITHER] = "dither",
	[MTK_DISP_UFOE] = "ufoe",
	[MTK_DSI] = "dsi",
	[MTK_DP_INTF] = "dp_intf",
	[MTK_DPI] = "dpi",
	[MTK_DISP_PWM] = "pwm",
	[MTK_DISP_MUTEX] = "mutex",
	[MTK_DISP_OD] = "od",
	[MTK_DISP_BLS] = "bls",
	[MTK_DISP_RSZ] = "rsz",
	[MTK_DISP_POSTMASK] = "postmask",
	[MTK_DMDP_RDMA] = "mrdma",
	[MTK_DMDP_HDR] = "mhdr",
	[MTK_DMDP_AAL] = "maal",
	[MTK_DMDP_RSZ] = "mrsz",
	[MTK_DMDP_TDSHP] = "mtdshp",
	[MTK_DISP_CM] = "cm",
	[MTK_DISP_SPR] = "spr",
	[MTK_DISP_DSC] = "dsc",
	[MTK_DISP_MERGE] = "merge",
	[MTK_DISP_DPTX] = "dptx",
	[MTK_DISP_RDMA_OUT_RELAY] = "rmda_out_relay",
	[MTK_DISP_VIRTUAL] = "virtual",
	[MTK_DISP_CHIST] = "chist",
	[MTK_DISP_C3D] = "c3d",
	[MTK_DISP_TDSHP] = "tdshp",
	[MTK_DISP_Y2R] = "y2r",
	[MTK_DISP_DLO_ASYNC] = "dlo_async",
	[MTK_DISP_DLI_ASYNC] = "dli_async",
	[MTK_DISP_INLINE_ROTATE] = "inlinerotate",
	[MTK_MMLSYS_BYPASS] = "mmlsys_bypass",
	[MTK_MML_RSZ] = "mml_rsz",
	[MTK_MML_HDR] = "mml_hdr",
	[MTK_MML_AAL] = "mml_aal",
	[MTK_MML_TDSHP] = "mml_tdshp",
	[MTK_MML_COLOR] = "mml_color",
	[MTK_MML_MML] = "mml_mml",
	[MTK_MML_MUTEX] = "mml_mutex",
	[MTK_MML_WROT] = "mml_wrot",
};

struct mtk_ddp_comp_match {
	enum mtk_ddp_comp_id index;
	enum mtk_ddp_comp_type type;
	int alias_id;
	const struct mtk_ddp_comp_funcs *funcs;
	bool is_output;
};

static const struct mtk_ddp_comp_match mtk_ddp_matches[DDP_COMPONENT_ID_MAX] = {
	{DDP_COMPONENT_AAL0, MTK_DISP_AAL, 0, NULL, 0},
	{DDP_COMPONENT_AAL1, MTK_DISP_AAL, 1, NULL, 0},
	{DDP_COMPONENT_BLS, MTK_DISP_BLS, 0, NULL, 0},
	{DDP_COMPONENT_CHIST0, MTK_DISP_CHIST, 0, NULL, 0},
	{DDP_COMPONENT_CHIST1, MTK_DISP_CHIST, 1, NULL, 0},
	{DDP_COMPONENT_CHIST2, MTK_DISP_CHIST, 2, NULL, 0},
	{DDP_COMPONENT_CHIST3, MTK_DISP_CHIST, 3, NULL, 0},
	{DDP_COMPONENT_TDSHP0, MTK_DISP_TDSHP, 0, NULL, 0},
	{DDP_COMPONENT_TDSHP1, MTK_DISP_TDSHP, 1, NULL, 0},
	{DDP_COMPONENT_C3D0, MTK_DISP_C3D, 0, NULL, 0},
	{DDP_COMPONENT_C3D1, MTK_DISP_C3D, 1, NULL, 0},
	{DDP_COMPONENT_CCORR0, MTK_DISP_CCORR, 0, NULL, 0},
	{DDP_COMPONENT_CCORR1, MTK_DISP_CCORR, 1, NULL, 0},
	{DDP_COMPONENT_CCORR2, MTK_DISP_CCORR, 2, NULL, 0},
	{DDP_COMPONENT_CCORR3, MTK_DISP_CCORR, 3, NULL, 0},
	{DDP_COMPONENT_COLOR0, MTK_DISP_COLOR, 0, NULL, 0},
	{DDP_COMPONENT_COLOR1, MTK_DISP_COLOR, 1, NULL, 0},
	{DDP_COMPONENT_COLOR2, MTK_DISP_COLOR, 2, NULL, 0},
	{DDP_COMPONENT_DITHER0, MTK_DISP_DITHER, 0, NULL, 0},
	{DDP_COMPONENT_DITHER1, MTK_DISP_DITHER, 1, NULL, 0},
	{DDP_COMPONENT_DPI0, MTK_DPI, 0, NULL, 1},
	{DDP_COMPONENT_DPI1, MTK_DPI, 1, NULL, 1},
	{DDP_COMPONENT_DSI0, MTK_DSI, 0, NULL, 1},
	{DDP_COMPONENT_DSI1, MTK_DSI, 1, NULL, 1},
	{DDP_COMPONENT_GAMMA0, MTK_DISP_GAMMA, 0, NULL, 0},
	{DDP_COMPONENT_GAMMA1, MTK_DISP_GAMMA, 1, NULL, 0},
	{DDP_COMPONENT_OD, MTK_DISP_OD, 0, &ddp_od, 0},
	{DDP_COMPONENT_OD1, MTK_DISP_OD, 1, &ddp_od, 0},
	{DDP_COMPONENT_OVL0, MTK_DISP_OVL, 0, NULL, 0},
	{DDP_COMPONENT_OVL1, MTK_DISP_OVL, 1, NULL, 0},
	{DDP_COMPONENT_OVL2, MTK_DISP_OVL, 2, NULL, 0},
	{DDP_COMPONENT_OVL0_2L, MTK_DISP_OVL, 3, NULL, 0},
	{DDP_COMPONENT_OVL1_2L, MTK_DISP_OVL, 4, NULL, 0},
	{DDP_COMPONENT_OVL2_2L, MTK_DISP_OVL, 5, NULL, 0},
	{DDP_COMPONENT_OVL3_2L, MTK_DISP_OVL, 6, NULL, 0},
	{DDP_COMPONENT_OVL0_2L_NWCG, MTK_DISP_OVL, 7, NULL, 0},
	{DDP_COMPONENT_OVL1_2L_NWCG, MTK_DISP_OVL, 8, NULL, 0},
	{DDP_COMPONENT_OVL2_2L_NWCG, MTK_DISP_OVL, 9, NULL, 0},
	{DDP_COMPONENT_OVL3_2L_NWCG, MTK_DISP_OVL, 10, NULL, 0},
	{DDP_COMPONENT_OVL0_2L_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL1_2L_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL2_2L_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL3_2L_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL0_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL0_VIRTUAL1, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL1_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL1_VIRTUAL1, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL0_2L_NWCG_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL2_2L_NWCG_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_OVL0_OVL0_2L_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_PWM0, MTK_DISP_PWM, 0, NULL, 0},
	{DDP_COMPONENT_PWM1, MTK_DISP_PWM, 1, NULL, 0},
	{DDP_COMPONENT_PWM2, MTK_DISP_PWM, 2, NULL, 0},
	{DDP_COMPONENT_RDMA0, MTK_DISP_RDMA, 0, NULL, 0},
	{DDP_COMPONENT_RDMA1, MTK_DISP_RDMA, 1, NULL, 0},
	{DDP_COMPONENT_RDMA2, MTK_DISP_RDMA, 2, NULL, 0},
	{DDP_COMPONENT_RDMA3, MTK_DISP_RDMA, 3, NULL, 0},
	{DDP_COMPONENT_RDMA4, MTK_DISP_RDMA, 4, NULL, 0},
	{DDP_COMPONENT_RDMA5, MTK_DISP_RDMA, 5, NULL, 0},
	{DDP_COMPONENT_RDMA0_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_RDMA1_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_RDMA2_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_RSZ0, MTK_DISP_RSZ, 0, NULL, 0},
	{DDP_COMPONENT_RSZ1, MTK_DISP_RSZ, 1, NULL, 0},
	{DDP_COMPONENT_UFOE, MTK_DISP_UFOE, 0, &ddp_ufoe, 0},
	{DDP_COMPONENT_WDMA0, MTK_DISP_WDMA, 0, NULL, 1},
	{DDP_COMPONENT_WDMA1, MTK_DISP_WDMA, 1, NULL, 1},
	{DDP_COMPONENT_WDMA2, MTK_DISP_WDMA, 2, NULL, 1},
	{DDP_COMPONENT_WDMA3, MTK_DISP_WDMA, 3, NULL, 1},
	{DDP_COMPONENT_UFBC_WDMA0, MTK_DISP_WDMA, 2, NULL, 1},
	{DDP_COMPONENT_WDMA_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_WDMA_VIRTUAL1, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_POSTMASK0, MTK_DISP_POSTMASK, 0, NULL, 0},
	{DDP_COMPONENT_POSTMASK1, MTK_DISP_POSTMASK, 1, NULL, 0},
	{DDP_COMPONENT_DMDP_RDMA0, MTK_DMDP_RDMA, 0, NULL, 0},
	{DDP_COMPONENT_DMDP_HDR0, MTK_DMDP_HDR, 0, NULL, 0},
	{DDP_COMPONENT_DMDP_AAL0, MTK_DMDP_AAL, 0, NULL, 0},
	{DDP_COMPONENT_DMDP_RSZ0, MTK_DMDP_RSZ, 0, NULL, 0},
	{DDP_COMPONENT_DMDP_TDSHP0, MTK_DMDP_TDSHP, 0, NULL, 0},
	{DDP_COMPONENT_DMDP_RDMA1, MTK_DMDP_RDMA, 1, NULL, 0},
	{DDP_COMPONENT_DMDP_HDR1, MTK_DMDP_HDR, 1, NULL, 0},
	{DDP_COMPONENT_DMDP_AAL1, MTK_DMDP_AAL, 1, NULL, 0},
	{DDP_COMPONENT_DMDP_RSZ1, MTK_DMDP_RSZ, 1, NULL, 0},
	{DDP_COMPONENT_DMDP_TDSHP1, MTK_DMDP_TDSHP, 1, NULL, 0},
	{DDP_COMPONENT_CM0, MTK_DISP_CM, 0, NULL, 0},
	{DDP_COMPONENT_CM1, MTK_DISP_CM, 1, NULL, 0},
	{DDP_COMPONENT_SPR0, MTK_DISP_SPR, 0, NULL, 0},
	{DDP_COMPONENT_SPR1, MTK_DISP_SPR, 1, NULL, 0},
	{DDP_COMPONENT_DSC0, MTK_DISP_DSC, 0, NULL, 0},
	{DDP_COMPONENT_DSC1, MTK_DISP_DSC, 1, NULL, 0},
	{DDP_COMPONENT_DLO_ASYNC0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLO_ASYNC1, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLO_ASYNC2, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLO_ASYNC3, MTK_DISP_DLO_ASYNC, 3, NULL, 0},
	{DDP_COMPONENT_DLO_ASYNC4, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLO_ASYNC5, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLO_ASYNC6, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLO_ASYNC7, MTK_DISP_DLO_ASYNC, 7, NULL, 0},
	{DDP_COMPONENT_DLI_ASYNC0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLI_ASYNC1, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLI_ASYNC2, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLI_ASYNC3, MTK_DISP_DLI_ASYNC, 3, NULL, 0},
	{DDP_COMPONENT_DLI_ASYNC4, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLI_ASYNC5, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLI_ASYNC6, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLI_ASYNC7, MTK_DISP_DLI_ASYNC, 7, NULL, 0},
	{DDP_COMPONENT_MERGE0, MTK_DISP_MERGE, 0, NULL, 0},
	{DDP_COMPONENT_DPTX, MTK_DISP_DPTX, 0, NULL, 1},
	{DDP_COMPONENT_DP_INTF0, MTK_DP_INTF, 0, NULL, 1},
	{DDP_COMPONENT_RDMA4_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_RDMA5_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_MERGE1, MTK_DISP_MERGE, 1, NULL, 0},
	{DDP_COMPONENT_SPR0_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_RDMA0_OUT_RELAY, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_RDMA2_OUT_RELAY, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_PQ0_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_PQ1_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_TV0_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_TV1_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_MAIN0_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_MAIN1_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_SUB0_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_SUB1_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_SUB0_VIRTUAL1, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_SUB1_VIRTUAL1, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_PQ0_RDMA0_POS_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_MAIN_OVL_DISP_PQ0_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_Y2R0, MTK_DISP_Y2R, 0, NULL, 0},
	{DDP_COMPONENT_Y2R1, MTK_DISP_Y2R, 1, NULL, 0},
	{DDP_COMPONENT_Y2R0_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_Y2R1_VIRTUAL0, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_DLO_ASYNC, MTK_DISP_DLO_ASYNC, 0, NULL, 0},
	{DDP_COMPONENT_DLI_ASYNC, MTK_DISP_DLI_ASYNC, 0, NULL, 0},
	{DDP_COMPONENT_INLINE_ROTATE0, MTK_DISP_INLINE_ROTATE, 0, NULL, 0},
	{DDP_COMPONENT_INLINE_ROTATE1, MTK_DISP_INLINE_ROTATE, 1, NULL, 0},
	{DDP_COMPONENT_MMLSYS_BYPASS, MTK_MMLSYS_BYPASS, 0, NULL, 0},
	{DDP_COMPONENT_MAIN_OVL_DISP_WDMA_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_MAIN_OVL_DISP1_WDMA_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_SUB_OVL_DISP0_PQ0_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_SUB_OVL_DISP1_PQ0_VIRTUAL, MTK_DISP_VIRTUAL, -1, NULL, 0},
	{DDP_COMPONENT_MML_RSZ0, MTK_MML_RSZ, 0, NULL, 0},
	{DDP_COMPONENT_MML_RSZ1, MTK_MML_RSZ, 1, NULL, 0},
	{DDP_COMPONENT_MML_RSZ2, MTK_MML_RSZ, 2, NULL, 0},
	{DDP_COMPONENT_MML_RSZ3, MTK_MML_RSZ, 3, NULL, 0},
	{DDP_COMPONENT_MML_HDR0, MTK_MML_HDR, 0, NULL, 0},
	{DDP_COMPONENT_MML_HDR1, MTK_MML_HDR, 1, NULL, 0},
	{DDP_COMPONENT_MML_AAL0, MTK_MML_AAL, 0, NULL, 0},
	{DDP_COMPONENT_MML_AAL1, MTK_MML_AAL, 1, NULL, 0},
	{DDP_COMPONENT_MML_TDSHP0, MTK_MML_TDSHP, 0, NULL, 0},
	{DDP_COMPONENT_MML_TDSHP1, MTK_MML_TDSHP, 1, NULL, 0},
	{DDP_COMPONENT_MML_COLOR0, MTK_MML_COLOR, 0, NULL, 0},
	{DDP_COMPONENT_MML_COLOR1, MTK_MML_COLOR, 1, NULL, 0},
	{DDP_COMPONENT_MML_MML0, MTK_MML_MML, 0, NULL, 0},
	{DDP_COMPONENT_MML_MUTEX0, MTK_MML_MUTEX, 0, NULL, 0},
	{DDP_COMPONENT_MML_WROT0, MTK_MML_WROT, 0, NULL, 0},
	{DDP_COMPONENT_MML_WROT1, MTK_MML_WROT, 1, NULL, 0},
	{DDP_COMPONENT_MML_WROT2, MTK_MML_WROT, 2, NULL, 0},
	{DDP_COMPONENT_MML_WROT3, MTK_MML_WROT, 3, NULL, 0},
};
void mtk_irq_time_handle(struct work_struct *data)
{
	struct mtk_irq_ts_debug_workqueue *mtk_irq_work =
		container_of(data, struct mtk_irq_ts_debug_workqueue, work);
	int i = 0;

	mtk_irq_work->is_busy = true;

	mtk_dprec_logger_pr(DPREC_LOGGER_STATUS, "%s > %d us, [%d]:\n",
		mtk_dump_comp_str_id(mtk_irq_work->comp_id),
		DO_COMMON_DIV(MTK_IRQ_TS_THRESHOLD, 1000), mtk_irq_work->number);
	for (i = 0; i < MTK_IRQ_TS_MAX && mtk_irq_work->irq_time[i].line != 0; i++)
		mtk_dprec_logger_pr(DPREC_LOGGER_STATUS, "[%d]%llu ns\n",
		mtk_irq_work->irq_time[i].line, mtk_irq_work->irq_time[i].ts);

	if (i > 0)
		mtk_dprec_logger_pr(DPREC_LOGGER_STATUS, "total : %llu ns\n",
		mtk_irq_work->irq_time[i - 1].ts - mtk_irq_work->irq_time[0].ts);

	for (i = 0; i < MTK_IRQ_TS_MAX; i++) {    //clean
		mtk_irq_work->irq_time[i].ts = 0;
		mtk_irq_work->irq_time[i].line = 0;
	}

	mtk_irq_work->is_busy = false;

}

static int mtk_ddp_comp_irq_work_init(struct mtk_ddp_comp *ddp_comp, int index)
{
	int i;

	if (index < 0 || index >= MTK_IRQ_WORK_MAX) {
		DDPMSG("%s index %d out of bounds\n", __func__, index);
		return -EINVAL;
	}
	INIT_WORK(&ddp_comp->ts_works[index].work, mtk_irq_time_handle);
	ddp_comp->ts_works[index].number = 0;
	ddp_comp->ts_works[index].is_busy = FALSE;
	for (i = 0; i < MTK_IRQ_TS_MAX; i++) {
		ddp_comp->ts_works[index].irq_time[i].ts = 0;
		ddp_comp->ts_works[index].irq_time[i].line = 0;
	}
	return 0;
}

int mtk_ddp_comp_create_workqueue(struct mtk_ddp_comp *ddp_comp)
{
	int i = 0;
	int ret = 0;
	char wq_buf[64] = {0};

	memset(wq_buf, 0, sizeof(wq_buf));
	ret = snprintf(wq_buf, sizeof(wq_buf), "mtk_%s_wq", mtk_dump_comp_str_id(ddp_comp->id));
	if (ret < 0) {
		DDPPR_ERR("%s snprintf fail: %d\n", __func__, ret);
		/* Handle snprintf() error */
		return -EINVAL;
	}
	DDPMSG("begin mtk_ddp_comp_create %d: %s\n", ddp_comp->id, wq_buf);

	ddp_comp->wq = create_singlethread_workqueue(wq_buf);
	if (IS_ERR_OR_NULL(ddp_comp->wq)) {
		DDPPR_ERR("Failed to create dsi workqueue\n");
		ddp_comp->irq_debug = false;
		return -ENOMEM;
	}
	for (i = 0; i < MTK_IRQ_WORK_MAX; i++)
		mtk_ddp_comp_irq_work_init(ddp_comp, i);
	ddp_comp->irq_debug = true;
	return 0;
}

bool mtk_ddp_comp_is_output(struct mtk_ddp_comp *comp)
{
	if (comp->id >= DDP_COMPONENT_ID_MAX)
		return false;

	return mtk_ddp_matches[comp->id].is_output;
}

bool mtk_ddp_comp_is_output_by_id(enum mtk_ddp_comp_id id)
{
	if (id >= DDP_COMPONENT_ID_MAX)
		return false;

	return mtk_ddp_matches[id].is_output;
}

void mtk_ddp_comp_get_name(struct mtk_ddp_comp *comp, char *buf, int buf_len)
{
	int r;

	if (comp->id < 0 || comp->id >= DDP_COMPONENT_ID_MAX) {
		DDPPR_ERR("%s(), invalid id %d, set buf to 0\n",
			  __func__, comp->id);
		memset(buf, 0, buf_len);
		return;
	}

	if (buf_len > sizeof(buf))
		buf_len = sizeof(buf);

	r = snprintf(buf, buf_len, "%s%d",
		  mtk_ddp_comp_stem[mtk_ddp_matches[comp->id].type],
		  mtk_ddp_matches[comp->id].alias_id);
	if (r < 0) {
		/* Handle snprintf() error */
		DDPPR_ERR("snprintf error\n");
	}
}

int mtk_ddp_comp_get_type(enum mtk_ddp_comp_id comp_id)
{
	if (comp_id >= DDP_COMPONENT_ID_MAX)
		return -EINVAL;

	return mtk_ddp_matches[comp_id].type;
}

int mtk_ddp_comp_get_alias(enum mtk_ddp_comp_id comp_id)
{
	if (comp_id >= DDP_COMPONENT_ID_MAX)
		return -EINVAL;

	return mtk_ddp_matches[comp_id].alias_id;
}

static bool mtk_drm_find_comp_in_ddp(struct mtk_ddp_comp ddp_comp,
				     const struct mtk_crtc_path_data *path_data)
{
	unsigned int i, j, ddp_mode;
	const enum mtk_ddp_comp_id *path = NULL;

	if (path_data == NULL)
		return false;

	for (ddp_mode = 0U; ddp_mode < DDP_MODE_NR; ddp_mode++)
		for (i = 0U; i < DDP_PATH_NR; i++) {
			path = path_data->path[ddp_mode][i];
			for (j = 0U; j < path_data->path_len[ddp_mode][i]; j++)
				if (ddp_comp.id == path[j])
					return true;
		}

	return false;
}

enum mtk_ddp_comp_id mtk_ddp_comp_get_id(struct device_node *node,
					 enum mtk_ddp_comp_type comp_type)
{
	int id;
	int i;

	id = of_alias_get_id(node, mtk_ddp_comp_stem[comp_type]);

	DDPINFO("id:%d, comp_type:%d\n", id, comp_type);
	for (i = 0; i < ARRAY_SIZE(mtk_ddp_matches); i++) {
		if (comp_type == mtk_ddp_matches[i].type &&
		    (id < 0 || id == mtk_ddp_matches[i].alias_id))
			return mtk_ddp_matches[i].index;
	}

	return -EINVAL;
}

struct mtk_ddp_comp *mtk_ddp_comp_find_by_id(struct drm_crtc *crtc,
					     enum mtk_ddp_comp_id comp_id)
{
	unsigned int i = 0, j = 0, ddp_mode = 0;
	struct mtk_drm_crtc *mtk_crtc =
		container_of(crtc, struct mtk_drm_crtc, base);
	struct mtk_ddp_comp *comp;

	for_each_comp_in_all_crtc_mode(comp, mtk_crtc, i, j,
				       ddp_mode)
		if (comp_id == comp->id)
			return comp;

	return NULL;
}

static void mtk_ddp_comp_set_larb(struct device *dev, struct device_node *node,
				  struct mtk_ddp_comp *comp)
{
	int ret;
	struct device_node *larb_node = NULL;
	struct platform_device *larb_pdev = NULL;
	enum mtk_ddp_comp_type type = mtk_ddp_comp_get_type(comp->id);
	unsigned int larb_id;

	comp->larb_dev = NULL;

	larb_node = of_parse_phandle(node, "mediatek,larb", 0);

	if (larb_node) {
		larb_pdev = of_find_device_by_node(larb_node);
		if (larb_pdev)
			comp->larb_dev = &larb_pdev->dev;
		of_node_put(larb_node);
	}

	if (!comp->larb_dev)
		return;

	ret = of_property_read_u32(node,
				"mediatek,smi-id", &larb_id);
	if (ret) {
		dev_err(comp->larb_dev,
			"read smi-id failed:%d\n", ret);
		return;
	}
	comp->larb_id = larb_id;

	/* check if this module need larb_dev */
	if (type == MTK_DISP_OVL || type == MTK_DISP_RDMA ||
	    type == MTK_DISP_WDMA || type == MTK_DISP_POSTMASK) {
		dev_warn(dev, "%s: %s need larb device\n", __func__,
				mtk_dump_comp_str(comp));
		DDPMSG("%s: smi-id:%d\n", mtk_dump_comp_str(comp),
				comp->larb_id);
	}
}

unsigned int mtk_drm_find_possible_crtc_by_comp(struct drm_device *drm,
						struct mtk_ddp_comp ddp_comp)
{
	struct mtk_drm_private *private = drm->dev_private;
	unsigned int ret;

	if (mtk_drm_find_comp_in_ddp(ddp_comp, private->data->main_path_data) ==
	    true) {
		ret = BIT(0);
	} else if (mtk_drm_find_comp_in_ddp(
			   ddp_comp, private->data->ext_path_data) == true) {
		ret = BIT(1);
	} else if (mtk_drm_find_comp_in_ddp(
			   ddp_comp, private->data->third_path_data) == true) {
		ret = BIT(2);
	} else {
		DRM_INFO("Failed to find comp in ddp table\n");
		ret = 0;
	}

	return ret;
}

#if IS_ENABLED(CONFIG_MTK_IOMMU_MISC_DBG)
static int mtk_ddp_iommu_callback(int port, dma_addr_t mva, void *data)
{
	struct mtk_ddp_comp *comp = (struct mtk_ddp_comp *)data;

	DDPPR_ERR("fault call port=0x%x, mva=0x%lx, data=0x%p\n", port,
		  (unsigned long)mva, data);
	if (mva == 0x0) {
		/*
		 * when ovl underflow, ovl maybe send 0x0 header address to iommu, can cause
		 * translation fault
		 * please check :1)ovl header address != 0
		 *				2)ovl fifo underflow
		 *				3)mva == 0
		 * ignore the translation fault, just resolve the ovl underflow issue
		 */
		DDPPR_ERR(
			"When had translation fault & mva = 0, check the code comment:[%s] line (%d)\n"
			, __func__, __LINE__);
	}
	DRM_MMP_EVENT_START(iova_tf, port, mva);
	if (comp) {
#ifdef CONFIG_MTK_IOMMU_MISC_DBG_DETAIL
		struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
		struct drm_crtc *crtc = NULL;

		if (mtk_crtc == NULL) {
			mtk_dump_analysis(comp);
			mtk_dump_reg(comp);
		} else {
			crtc = &mtk_crtc->base;
			mtk_drm_crtc_analysis(crtc);
			mtk_drm_crtc_dump(crtc);
		}
#else
		mtk_dump_analysis(comp);
		mtk_dump_reg(comp);
#endif
		DRM_MMP_EVENT_END(iova_tf, comp->id, comp->regs_pa);
	} else
		DRM_MMP_EVENT_END(iova_tf, 0, 0);

	return 0;
}

static void mtk_ddp_comp_iommu_register(struct mtk_ddp_comp *comp)
{
	int port = 0, index = 0, ret = 0;

	if (!comp || !comp->dev)
		return;

	while (1) {
		ret = of_property_read_u32_index(comp->dev->of_node,
				"iommus", index * 2 + 1, &port);
		if (ret < 0)
			break;
		if (disp_helper_get_stage() ==
			DISP_HELPER_STAGE_NORMAL)
			mtk_iommu_register_fault_callback(
						port, mtk_ddp_iommu_callback,
						comp, false);
		DDPINFO("%s, comp:%u, register the %d port:0x%x\n",
			__func__, comp->id, index, port);
		index++;
	}
}
#endif

int mtk_ddp_comp_init(struct device *dev, struct device_node *node,
		      struct mtk_ddp_comp *comp, enum mtk_ddp_comp_id comp_id,
		      const struct mtk_ddp_comp_funcs *funcs)
{
	enum mtk_ddp_comp_type type;
	struct platform_device *comp_pdev = NULL;
	struct resource res;

	DDPINFO("%s+\n", __func__);

	if (comp_id >= DDP_COMPONENT_ID_MAX)
		return -EINVAL;

	type = mtk_ddp_matches[comp_id].type;

	comp->id = comp_id;
	comp->funcs = funcs ?: mtk_ddp_matches[comp_id].funcs;
	comp->dev = dev;

	/* get the first clk in the device node */
	comp->clk = of_clk_get(node, 0);
	if (IS_ERR(comp->clk)) {
		comp->clk = NULL;
		DDPPR_ERR("comp:%d get clock fail!\n", comp_id);
	}

	if (comp_id == DDP_COMPONENT_BLS || comp_id == DDP_COMPONENT_PWM0) {
		comp->regs_pa = 0;
		comp->regs = NULL;
		comp->irq = 0;
		return 0;
	}

	if (of_address_to_resource(node, 0, &res) != 0) {
		dev_err(dev, "Missing reg in %s node\n", node->full_name);
		return -EINVAL;
	}

	comp->regs_pa = res.start;

	if (comp_id == DDP_COMPONENT_DPI0 || comp_id == DDP_COMPONENT_DPI1 ||
	    comp_id == DDP_COMPONENT_DSI0 || comp_id == DDP_COMPONENT_DSI1)
		comp->irq = 0;
	else
		comp->irq = of_irq_get(node, 0);

	comp->regs = of_iomap(node, 0);
	DDPINFO("[DRM]regs_pa:0x%lx, regs:0x%p, node:%s\n",
		(unsigned long)comp->regs_pa, comp->regs, node->full_name);

	/* handle cmdq related resources */
	comp_pdev = of_find_device_by_node(node);
	if (!comp_pdev) {
		dev_warn(dev, "Waiting for comp device %s\n", node->full_name);
		return -EPROBE_DEFER;
	}

	comp->cmdq_base = cmdq_register_device(&comp_pdev->dev);

#ifdef IF_ZERO
	/* TODO: if no subsys id, use 99 instead. CMDQ owner would define 99 in
	 * DTS afterward.
	 */
	if (of_property_read_u8(node, "my_subsys_id", &comp->cmdq_subsys))
		comp->cmdq_subsys = 99;
#endif

	/* handle larb resources */
	mtk_ddp_comp_set_larb(dev, node, comp);

#if IS_ENABLED(CONFIG_MTK_IOMMU_MISC_DBG)
	mtk_ddp_comp_iommu_register(comp);
#endif

	DDPINFO("%s-\n", __func__);

	return 0;
}

int mtk_ddp_comp_register(struct drm_device *drm, struct mtk_ddp_comp *comp)
{
	struct mtk_drm_private *private = drm->dev_private;

	if (private->ddp_comp[comp->id])
		return -EBUSY;

	private->ddp_comp[comp->id] = comp;
	return 0;
}

void mtk_ddp_comp_unregister(struct drm_device *drm, struct mtk_ddp_comp *comp)
{
	struct mtk_drm_private *private = drm->dev_private;
	if (comp)
		private->ddp_comp[comp->id] = NULL;
}

void mtk_ddp_comp_pm_enable(struct mtk_ddp_comp *comp)
{
	if (comp->larb_dev)
		pm_runtime_enable(comp->dev);
}

void mtk_ddp_comp_pm_disable(struct mtk_ddp_comp *comp)
{
	if (comp->larb_dev)
		pm_runtime_disable(comp->dev);
}

void mtk_ddp_comp_clk_prepare(struct mtk_ddp_comp *comp)
{
	unsigned int index = 0;
	int ret = 0;

	if (comp == NULL)
		return;

	if (comp->larb_dev)
#ifdef MTK_SMI_CLK_CTRL
		ret = mtk_smi_larb_get(comp->larb_dev);
#else
		ret = pm_runtime_get_sync(comp->dev);
#endif

	if (ret)
		DDPPR_ERR("larb or pm_runtime get fail:%s\n", mtk_dump_comp_str(comp));

	if (comp->clk) {
		ret = clk_prepare_enable(comp->clk);
		if (ret)
			DDPPR_ERR("clk prepare enable failed:%s\n",
				mtk_dump_comp_str(comp));
	}

	if (comp->mtk_crtc)
		index = drm_crtc_index(&comp->mtk_crtc->base);
	CRTC_MMP_MARK(index, ddp_clk, comp->id, 1);
}

void mtk_ddp_comp_clk_unprepare(struct mtk_ddp_comp *comp)
{
	unsigned int index = 0;

	if (comp == NULL)
		return;

	if (comp->clk)
		clk_disable_unprepare(comp->clk);
	DDPINFO("%s: comp %d unprepare done\n", __func__, comp->id);

	if (comp->larb_dev)
#ifdef MTK_SMI_CLK_CTRL
		mtk_smi_larb_put(comp->larb_dev);
#else
		pm_runtime_put_sync(comp->dev);
#endif

	if (comp->mtk_crtc)
		index = drm_crtc_index(&comp->mtk_crtc->base);
	CRTC_MMP_MARK(index, ddp_clk, comp->id, 0);
}

#define GET_M4U_PORT 0x1F
void mtk_ddp_comp_iommu_enable(struct mtk_ddp_comp *comp,
			       struct cmdq_pkt *handle)
{
	int port = 0, index, ret;
	struct resource res;
	struct mtk_drm_private *priv;

	if (!comp->dev || !comp->larb_dev || !comp->mtk_crtc)
		return;

	priv = comp->mtk_crtc->base.dev->dev_private;

	index = 0;
	while (1) {
		ret = of_property_read_u32_index(comp->dev->of_node,
				"iommus", index * 2 + 1, &port);
		if (ret < 0)
			break;

		port &= (unsigned int)GET_M4U_PORT;
		if (of_address_to_resource(comp->larb_dev->of_node, 0, &res) !=
		    0) {
			dev_err(comp->dev, "Missing reg in %s node\n",
				comp->larb_dev->of_node->full_name);
			return;
		}

		if (!mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_USE_M4U))
			//bypass m4u
			cmdq_pkt_write(handle, NULL,
				res.start + SMI_LARB_NON_SEC_CON + port * 4, 0,
				0x1);
		else
			cmdq_pkt_write(handle, NULL,
				res.start + SMI_LARB_NON_SEC_CON + port * 4, 0x1,
				0x1);

		index++;
	}
}

void mt6765_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_VAL_ALL);

		/* apply sodi hrt with rdma fifo*/
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_HRT_FIFO_SEL_DISP0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_HRT_FIFO_SEL_DISP0_CG_MODE);

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_PD_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0xFF, MT6765_HRT_URGENT_CTL_SEL_ALL);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, MT6765_HRT_URGENT_CTL_VAL_ALL);
	} else if (id == DDP_COMPONENT_RDMA0) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!en),
					SODI_REQ_SEL_RDMA0_CG_MODE);
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
					MT6765_HRT_URGENT_CTL_SEL_RDMA0);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
					MT6765_HRT_URGENT_CTL_SEL_WDMA0);
	} else
		return;

	if (handle == NULL) {
		unsigned int v;

		v = (readl(priv->config_regs + MMSYS_SODI_REQ_MASK)
			& (~sodi_req_mask));
		v += (sodi_req_val & sodi_req_mask);
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);

		v = (readl(priv->config_regs +  MMSYS_EMI_REQ_CTL)
			& (~emi_req_mask));
		v += (emi_req_val & emi_req_mask);
		writel_relaxed(v, priv->config_regs +  MMSYS_EMI_REQ_CTL);
	} else {
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, sodi_req_val, sodi_req_mask);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, emi_req_val, emi_req_mask);
	}
}

void mt6768_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_VAL_ALL);

		/* apply sodi hrt with rdma fifo*/
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_HRT_FIFO_SEL_DISP0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_HRT_FIFO_SEL_DISP0_CG_MODE);

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_PD_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0xFF, MT6768_HRT_URGENT_CTL_SEL_ALL);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, MT6768_HRT_URGENT_CTL_VAL_ALL);
	} else if (id == DDP_COMPONENT_RDMA0) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!en),
					SODI_REQ_SEL_RDMA0_CG_MODE);
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
					MT6768_HRT_URGENT_CTL_SEL_RDMA0);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
					MT6768_HRT_URGENT_CTL_SEL_WDMA0);
	} else
		return;

	if (handle == NULL) {
		unsigned int v;

		v = (readl(priv->config_regs + MMSYS_SODI_REQ_MASK)
			& (~sodi_req_mask));
		v += (sodi_req_val & sodi_req_mask);
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);

		v = (readl(priv->config_regs +  MMSYS_EMI_REQ_CTL)
			& (~emi_req_mask));
		v += (emi_req_val & emi_req_mask);
		writel_relaxed(v, priv->config_regs +  MMSYS_EMI_REQ_CTL);
	} else {
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, sodi_req_val, sodi_req_mask);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, emi_req_val, emi_req_mask);
	}
}

void mt6779_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int val = 0, mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;
		val = 0x0F005506;
		mask = 0xFFFFFFFF;
	} else if (id == DDP_COMPONENT_RDMA0) {
		mask |= (BIT(9) + BIT(16));
		val |= (((!(unsigned int)en) << 9) + ((en) << 16));
	} else if (id == DDP_COMPONENT_RDMA1) {
		mask |= (BIT(11) + BIT(17));
		val |= (((!(unsigned int)en) << 11) + ((en) << 17));
	} else if (id == DDP_COMPONENT_WDMA0) {
		mask |= BIT(18);
		val |= ((en) << 18);
	} else
		return;

	if (handle == NULL) {
		unsigned int v = (readl(priv->config_regs + 0xF8) & (~mask));

		v += (val & mask);
		writel_relaxed(v, priv->config_regs + 0xF8);
	} else
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa + 0xF8, val,
			       mask);
}

void mt6853_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, MT6873_SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, MT6873_SODI_REQ_VAL_ALL);

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_PD_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0x1, HRT_URGENT_CTL_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0x1, HRT_URGENT_CTL_SEL_WDMA0);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_MDP_RDMA4);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_WDMA0);
	} else if (id == DDP_COMPONENT_RDMA0) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!en),
					SODI_REQ_SEL_RDMA0_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
				HRT_URGENT_CTL_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
				DVFS_HALT_MASK_SEL_RDMA0);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
					HRT_URGENT_CTL_SEL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_WDMA0);
	} else
		return;

	if (handle == NULL) {
		unsigned int v;

		v = (readl(priv->config_regs + MMSYS_SODI_REQ_MASK)
			& (~sodi_req_mask));
		v += (sodi_req_val & sodi_req_mask);
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);

		v = (readl(priv->config_regs +  MMSYS_EMI_REQ_CTL)
			& (~emi_req_mask));
		v += (emi_req_val & emi_req_mask);
		writel_relaxed(v, priv->config_regs +  MMSYS_EMI_REQ_CTL);
	} else {
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, sodi_req_val, sodi_req_mask);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, emi_req_val, emi_req_mask);
	}
}

void mt6833_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	unsigned int infra_req_val = 0, infra_req_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, MT6873_SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, MT6873_SODI_REQ_VAL_ALL);

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_PD_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0x1, HRT_URGENT_CTL_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0x1, HRT_URGENT_CTL_SEL_WDMA0);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_MDP_RDMA4);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_WDMA0);
	} else if (id == DDP_COMPONENT_RDMA0) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!en),
					SODI_REQ_SEL_RDMA0_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
				HRT_URGENT_CTL_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
				DVFS_HALT_MASK_SEL_RDMA0);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
					HRT_URGENT_CTL_SEL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_WDMA0);
	} else
		return;

	if (priv->data->bypass_infra_ddr_control)
		SET_VAL_MASK(infra_req_val, infra_req_mask,
				0xf, MT6833_INFRA_FLD_DDR_MASK);

	if (handle == NULL) {
		unsigned int v;

		v = (readl(priv->config_regs + MMSYS_SODI_REQ_MASK)
			& (~sodi_req_mask));
		v += (sodi_req_val & sodi_req_mask);
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);

		v = (readl(priv->config_regs +  MMSYS_EMI_REQ_CTL)
			& (~emi_req_mask));
		v += (emi_req_val & emi_req_mask);
		writel_relaxed(v, priv->config_regs +  MMSYS_EMI_REQ_CTL);
		if (priv->data->bypass_infra_ddr_control) {
			if (!IS_ERR(priv->infra_regs)) {
				v = (readl(priv->infra_regs + MT6833_INFRA_DISP_DDR_CTL)
					| MT6833_INFRA_FLD_DDR_MASK);
				writel_relaxed(v, priv->infra_regs + MT6833_INFRA_DISP_DDR_CTL);
			} else
				DDPINFO("%s: failed to disable infra ddr control\n", __func__);
		}
	} else {
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, sodi_req_val, sodi_req_mask);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, emi_req_val, emi_req_mask);
		if (priv->data->bypass_infra_ddr_control) {
			if (priv->infra_regs_pa) {
				cmdq_pkt_write(handle, NULL,  priv->infra_regs_pa +
						MT6833_INFRA_DISP_DDR_CTL,
						infra_req_val, infra_req_mask);
			} else
				DDPINFO("%s: failed to disable infra ddr control\n", __func__);
		}
	}
}

void mt6879_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	unsigned int ultra_ovl_val = 0, ultra_ovl_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_VAL_ALL);

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_HRT_FIFO_SEL_DISP0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_HRT_FIFO_SEL_DISP0_CG_MODE);

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, MT6879_DVFS_HALT_MASK_SEL_ALL);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0xFF, MT6879_HRT_URGENT_CTL_SEL_ALL);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, MT6879_HRT_URGENT_CTL_VAL_ALL);
	} else if (id == DDP_COMPONENT_RDMA0) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!(unsigned int)en),
					SODI_REQ_SEL_RDMA0_CG_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, en,
					MT6879_DVFS_HALT_MASK_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					MT6879_HRT_URGENT_CTL_SEL_DSI0);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					MT6879_HRT_URGENT_CTL_SEL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					MT6879_DVFS_HALT_MASK_SEL_WDMA0);
	} else if (id == DDP_COMPONENT_WDMA1) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					MT6879_HRT_URGENT_CTL_SEL_WDMA1);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					MT6879_DVFS_HALT_MASK_SEL_WDMA1);
	} else
		return;

	if (handle == NULL) {
		unsigned int v;

		v = (readl(priv->config_regs + MMSYS_SODI_REQ_MASK)
			& (~sodi_req_mask));
		v += (sodi_req_val & sodi_req_mask);
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);

		v = (readl(priv->config_regs +  MMSYS_EMI_REQ_CTL)
			& (~emi_req_mask));
		v += (emi_req_val & emi_req_mask);
		writel_relaxed(v, priv->config_regs +  MMSYS_EMI_REQ_CTL);

		/* enable ultra signal from rdma to ovl0 and ovl0_2l */
		v = readl(priv->config_regs +  DISP_REG_CONFIG_MMSYS_MISC);
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask,
			0, MT6879_FLD_OVL0_RDMA_ULTRA_SEL);
		v = (v & ~ultra_ovl_mask) | (ultra_ovl_val & ultra_ovl_mask);
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask,
			0, MT6879_FLD_OVL0_2L_RDMA_ULTRA_SEL);
		v = (v & ~ultra_ovl_mask) | (ultra_ovl_val & ultra_ovl_mask);
		writel_relaxed(v, priv->config_regs +  DISP_REG_CONFIG_MMSYS_MISC);

	} else {
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, sodi_req_val, sodi_req_mask);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, emi_req_val, emi_req_mask);

		/* enable ultra signal from rdma to ovl0 */
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask,
			0, MT6879_FLD_OVL0_RDMA_ULTRA_SEL);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			DISP_REG_CONFIG_MMSYS_MISC, ultra_ovl_val, ultra_ovl_mask);

		/* enable ultra signal from rdma to ovl1_2l */
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask,
			0, MT6879_FLD_OVL0_2L_RDMA_ULTRA_SEL);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			DISP_REG_CONFIG_MMSYS_MISC, ultra_ovl_val, ultra_ovl_mask);
	}
}

void mt6855_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	unsigned int ultra_ovl_val = 0, ultra_ovl_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_VAL_ALL);

		/* apply sodi hrt with rdma fifo*/
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_HRT_FIFO_SEL_DISP0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_HRT_FIFO_SEL_DISP0_CG_MODE);

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_PD_MODE);

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0xFF, MT6855_HRT_URGENT_CTL_SEL_ALL);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, MT6855_HRT_URGENT_CTL_VAL_ALL);
	} else if (id == DDP_COMPONENT_RDMA0) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!en),
					SODI_REQ_SEL_RDMA0_CG_MODE);
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
					MT6855_HRT_URGENT_CTL_SEL_RDMA0);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!en),
					MT6855_HRT_URGENT_CTL_SEL_WDMA0);
	} else
		return;

	if (handle == NULL) {
		unsigned int v;

		v = (readl(priv->config_regs + MMSYS_SODI_REQ_MASK)
			& (~sodi_req_mask));
		v += (sodi_req_val & sodi_req_mask);
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);

		v = (readl(priv->config_regs +  MMSYS_EMI_REQ_CTL)
			& (~emi_req_mask));
		v += (emi_req_val & emi_req_mask);
		writel_relaxed(v, priv->config_regs +  MMSYS_EMI_REQ_CTL);

		/* enable ultra signal from rdma to ovl0 and ovl1_2l */
		v = readl(priv->config_regs +  DISP_REG_CONFIG_MMSYS_MISC);
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask, 0,
			MT6855_FLD_OVL0_RDMA_ULTRA_SEL);
		v = (v & ~ultra_ovl_mask) | (ultra_ovl_val & ultra_ovl_mask);
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask, 0,
			MT6855_FLD_OVL1_2L_RDMA_ULTRA_SEL);
		v = (v & ~ultra_ovl_mask) | (ultra_ovl_val & ultra_ovl_mask);
		writel_relaxed(v, priv->config_regs +  DISP_REG_CONFIG_MMSYS_MISC);
	} else {
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, sodi_req_val, sodi_req_mask);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, emi_req_val, emi_req_mask);

		/* enable ultra signal from rdma to ovl0 and ovl1_2l*/
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask, 0,
			MT6855_FLD_OVL0_RDMA_ULTRA_SEL);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa + DISP_REG_CONFIG_MMSYS_MISC,
			       ultra_ovl_val, ultra_ovl_mask);
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask, 0,
			MT6855_FLD_OVL1_2L_RDMA_ULTRA_SEL);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa + DISP_REG_CONFIG_MMSYS_MISC,
			       ultra_ovl_val, ultra_ovl_mask);
	}
}

void mt6873_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, MT6873_SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, MT6873_SODI_REQ_VAL_ALL);

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_PD_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0x1, HRT_URGENT_CTL_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0x1, HRT_URGENT_CTL_SEL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0x1, HRT_URGENT_CTL_SEL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0x1, HRT_URGENT_CTL_SEL_MDP_RDMA4);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_MDP_RDMA4);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_WDMA0);
	} else if (id == DDP_COMPONENT_RDMA0) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!(unsigned int)en),
					SODI_REQ_SEL_RDMA0_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
				HRT_URGENT_CTL_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
				DVFS_HALT_MASK_SEL_RDMA0);
	} else if (id == DDP_COMPONENT_RDMA4) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask, (unsigned int)en,
					DVFS_HALT_MASK_SEL_RDMA4);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_WDMA0);
	} else
		return;

	if (handle == NULL) {
		unsigned int v;

		v = (readl(priv->config_regs + MMSYS_SODI_REQ_MASK)
			& (~sodi_req_mask));
		v += (sodi_req_val & sodi_req_mask);
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);

		v = (readl(priv->config_regs +  MMSYS_EMI_REQ_CTL)
			& (~emi_req_mask));
		v += (emi_req_val & emi_req_mask);
		writel_relaxed(v, priv->config_regs +  MMSYS_EMI_REQ_CTL);
	} else {
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, sodi_req_val, sodi_req_mask);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, emi_req_val, emi_req_mask);
	}
}

void mt6885_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_VAL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA1_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA1_PD_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0xFF, HRT_URGENT_CTL_SEL_ALL);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_ALL);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_ALL);
	} else if (id == DDP_COMPONENT_RDMA0) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!(unsigned int)en),
					SODI_REQ_SEL_RDMA0_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
				HRT_URGENT_CTL_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
				DVFS_HALT_MASK_SEL_RDMA0);
	} else if (id == DDP_COMPONENT_RDMA1) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!(unsigned int)en),
					SODI_REQ_SEL_RDMA1_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_RDMA1);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_RDMA1);
	} else if (id == DDP_COMPONENT_RDMA4) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_RDMA4);
	} else if (id == DDP_COMPONENT_RDMA5) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_RDMA5);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_RDMA5);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_WDMA0);
	} else if (id == DDP_COMPONENT_WDMA1) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_WDMA1);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_WDMA1);
	} else
		return;

	if (handle == NULL) {
		unsigned int v;

		v = (readl(priv->config_regs + MMSYS_SODI_REQ_MASK)
			& (~sodi_req_mask));
		v += (sodi_req_val & sodi_req_mask);
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);

		v = (readl(priv->config_regs +  MMSYS_EMI_REQ_CTL)
			& (~emi_req_mask));
		v += (emi_req_val & emi_req_mask);
		writel_relaxed(v, priv->config_regs +  MMSYS_EMI_REQ_CTL);
	} else {
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, sodi_req_val, sodi_req_mask);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, emi_req_val, emi_req_mask);
	}
}

void mt6983_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_VAL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_SEL_RDMA1_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					1, SODI_REQ_VAL_RDMA1_PD_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0xFF, HRT_URGENT_CTL_SEL_ALL);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_ALL);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, DVFS_HALT_MASK_SEL_ALL);
	} else if (id == DDP_COMPONENT_RDMA0) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!(unsigned int)en),
					SODI_REQ_SEL_RDMA0_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
				HRT_URGENT_CTL_SEL_RDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
				DVFS_HALT_MASK_SEL_RDMA0);
	} else if (id == DDP_COMPONENT_RDMA1) {
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!(unsigned int)en),
					SODI_REQ_SEL_RDMA1_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_RDMA1);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_RDMA1);
	} else if (id == DDP_COMPONENT_RDMA4) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_RDMA4);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_RDMA4);
	} else if (id == DDP_COMPONENT_RDMA5) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_RDMA5);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_RDMA5);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_WDMA0);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_WDMA0);
	} else if (id == DDP_COMPONENT_WDMA1) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_WDMA1);
		SET_VAL_MASK(emi_req_val, emi_req_mask, en,
					DVFS_HALT_MASK_SEL_WDMA1);
	} else
		return;

	if (handle == NULL) {
		unsigned int v;

		v = 0xF500;
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);
		writel_relaxed(0x7, priv->config_regs + MMSYS_DUMMY0);
		if (priv->side_config_regs) {
			writel_relaxed(v, priv->side_config_regs + MMSYS_SODI_REQ_MASK);
			writel_relaxed(0x7, priv->side_config_regs + MMSYS_DUMMY0);
		}
		v = 0xDF;
		writel_relaxed(v, priv->config_regs +  MMSYS_EMI_REQ_CTL);
		if (priv->side_config_regs)
			writel_relaxed(v, priv->side_config_regs +  MMSYS_EMI_REQ_CTL);
		v = (readl(priv->config_regs + MMSYS_MISC)
			& (~0x3FFFFC));
		writel_relaxed(v, priv->config_regs + MMSYS_MISC);
		if (priv->side_config_regs) {
			v = (readl(priv->side_config_regs + MMSYS_MISC)
				& (~0x3FFFFC));
			writel_relaxed(v, priv->side_config_regs + MMSYS_MISC);
		}
	} else {
		/* TODO: HARD CODE for RDMA0 scenario */
		// cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
		//	MMSYS_SODI_REQ_MASK, sodi_req_val, sodi_req_mask);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, 0xf500, ~0);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_DUMMY0, 0x7, ~0);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, 0xdf, ~0);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_MISC, 0x0, 0x3FFFFC);
		if (priv->side_config_regs_pa) {
			cmdq_pkt_write(handle, NULL, priv->side_config_regs_pa +
				MMSYS_SODI_REQ_MASK, 0xf500, ~0);
			cmdq_pkt_write(handle, NULL, priv->side_config_regs_pa +
				MMSYS_DUMMY0, 0x7, ~0);
			cmdq_pkt_write(handle, NULL, priv->side_config_regs_pa +
				MMSYS_EMI_REQ_CTL, 0xdf, ~0);
			cmdq_pkt_write(handle, NULL, priv->side_config_regs_pa +
				MMSYS_MISC, 0x0, 0x3FFFFC);
		}
	}
}

void mt6895_mtk_sodi_config(struct drm_device *drm, enum mtk_ddp_comp_id id,
			    struct cmdq_pkt *handle, void *data)
{
	struct mtk_drm_private *priv = drm->dev_private;
	unsigned int sodi_req_val = 0, sodi_req_mask = 0;
	unsigned int emi_req_val = 0, emi_req_mask = 0;
	unsigned int ultra_ovl_val = 0, ultra_ovl_mask = 0;
	bool en = *((bool *)data);

	if (id == DDP_COMPONENT_ID_MAX) { /* config when top clk on */
		if (!en)
			return;

		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_SEL_ALL);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask,
					0, SODI_REQ_VAL_ALL);

		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0xFF, HRT_URGENT_CTL_SEL_ALL);
		SET_VAL_MASK(emi_req_val, emi_req_mask,
					0, HRT_URGENT_CTL_VAL_ALL);
	} else if (id == DDP_COMPONENT_RDMA0 ||
	    id == DDP_COMPONENT_RDMA2) {
		/* Select ddren smi req source from dsi */
		/* Todo: judge by dsi_buf on/off */
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, 0,
					SODI_REQ_SEL_DDREN);

		/* apsrc */
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, 1,
					SODI_REQ_SEL_RDMA0_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, ((unsigned int)en),
					SODI_REQ_VAL_RDMA0_PD_MODE);

		/* ddren */
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!(unsigned int)en),
					SODI_REQ_SEL_RDMA0_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
				HRT_URGENT_CTL_SEL_RDMA0);
	} else if (id == DDP_COMPONENT_RDMA1 ||
	    id == DDP_COMPONENT_RDMA3) {
		/* apsrc */
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, 1,
					SODI_REQ_SEL_RDMA1_PD_MODE);
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, ((unsigned int)en),
					SODI_REQ_VAL_RDMA1_PD_MODE);

		/* ddren */
		SET_VAL_MASK(sodi_req_val, sodi_req_mask, (!(unsigned int)en),
					SODI_REQ_SEL_RDMA1_CG_MODE);

		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_RDMA1);
	} else if (id == DDP_COMPONENT_WDMA0) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_WDMA0);
	} else if (id == DDP_COMPONENT_WDMA1) {
		SET_VAL_MASK(emi_req_val, emi_req_mask, (!(unsigned int)en),
					HRT_URGENT_CTL_SEL_WDMA1);
	} else
		return;

	if (handle == NULL) {
		unsigned int v;

		v = 0xF500;
		writel_relaxed(v, priv->config_regs + MMSYS_SODI_REQ_MASK);
		writel_relaxed(0x7, priv->config_regs + MMSYS_DUMMY0);
		if (priv->side_config_regs) {
			writel_relaxed(v, priv->side_config_regs +  MMSYS_SODI_REQ_MASK);
			writel_relaxed(0x7, priv->side_config_regs + MMSYS_DUMMY0);
		}

		/* enable urgent signal from mmsys0 dsi buffer*/
		writel_relaxed(0xdf, priv->config_regs +  MMSYS_EMI_REQ_CTL);
		if (priv->side_config_regs)
			writel_relaxed(0xff, priv->side_config_regs +  MMSYS_EMI_REQ_CTL);

		/* enable ultra signal from rdma to ovl0 and ovl1_2l */
		v = readl(priv->config_regs +  DISP_REG_CONFIG_MMSYS_MISC);
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask,
				0, MT6895_FLD_OVL0_RDMA_ULTRA_SEL);
		v = (v & ~ultra_ovl_mask) | (ultra_ovl_val & ultra_ovl_mask);
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask,
				0, MT6895_FLD_OVL1_2L_RDMA_ULTRA_SEL);
		v = (v & ~ultra_ovl_mask) | (ultra_ovl_val & ultra_ovl_mask);
		writel_relaxed(v, priv->config_regs +  DISP_REG_CONFIG_MMSYS_MISC);
		if (priv->side_config_regs)
			writel_relaxed(v, priv->side_config_regs +  DISP_REG_CONFIG_MMSYS_MISC);
	} else {
		/* enable ultra signal from rdma to ovl0 */
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask,
				0, MT6895_FLD_OVL0_RDMA_ULTRA_SEL);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			DISP_REG_CONFIG_MMSYS_MISC, ultra_ovl_val, ultra_ovl_mask);
		if (priv->side_config_regs_pa)
			cmdq_pkt_write(handle, NULL, priv->side_config_regs_pa +
				DISP_REG_CONFIG_MMSYS_MISC, ultra_ovl_val, ultra_ovl_mask);

		/* enable ultra signal from rdma to ovl1_2l */
		SET_VAL_MASK(ultra_ovl_val, ultra_ovl_mask,
				0, MT6895_FLD_OVL1_2L_RDMA_ULTRA_SEL);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			DISP_REG_CONFIG_MMSYS_MISC, ultra_ovl_val, ultra_ovl_mask);
		if (priv->side_config_regs_pa)
			cmdq_pkt_write(handle, NULL, priv->side_config_regs_pa +
				DISP_REG_CONFIG_MMSYS_MISC, ultra_ovl_val, ultra_ovl_mask);

		/* enable urgent signal from mmsys0 dsi buffer*/
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_SODI_REQ_MASK, 0xf500, ~0);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_DUMMY0, 0x7, ~0);
		cmdq_pkt_write(handle, NULL, priv->config_regs_pa +
			MMSYS_EMI_REQ_CTL, 0xdf, ~0);
		if (priv->side_config_regs_pa) {
			cmdq_pkt_write(handle, NULL, priv->side_config_regs_pa +
				MMSYS_SODI_REQ_MASK, 0xf500, ~0);
			cmdq_pkt_write(handle, NULL, priv->side_config_regs_pa +
				MMSYS_DUMMY0, 0x7, ~0);
			cmdq_pkt_write(handle, NULL, priv->side_config_regs_pa +
				MMSYS_EMI_REQ_CTL, 0xff, ~0);
		}
	}
}

int mtk_ddp_comp_helper_get_opt(struct mtk_ddp_comp *comp,
				enum MTK_DRM_HELPER_OPT option)
{
	struct mtk_drm_crtc *mtk_crtc = comp->mtk_crtc;
	struct mtk_drm_private *priv = NULL;
	struct mtk_drm_helper *helper_opt = NULL;

	if (!mtk_crtc) {
		DDPINFO("%s: crtc is empty\n", __func__);
		return -EINVAL;
	}

	priv = mtk_crtc->base.dev->dev_private;
	helper_opt = priv->helper_opt;

	return mtk_drm_helper_get_opt(helper_opt, option);
}
