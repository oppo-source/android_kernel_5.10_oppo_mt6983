// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#ifndef DRM_CMDQ_DISABLE
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#else
#include "mtk-cmdq-ext.h"
#endif

#include "mtk_drm_crtc.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_drv.h"
#include "mtk_log.h"
#include "mtk_dump.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_gem.h"

#define POSTMASK_MASK_MAX_NUM 96
#define POSTMASK_GRAD_MAX_NUM 192
#define POSTMASK_DRAM_MODE

#define DISP_POSTMASK_EN 0x0
#define DISP_POSTMASK_INTEN 0x8
#define INTEN_FLD_PM_IF_FME_END_INTEN REG_FLD_MSB_LSB(0, 0)
#define INTEN_FLD_PM_FME_CPL_INTEN REG_FLD_MSB_LSB(1, 1)
#define INTEN_FLD_PM_START_INTEN REG_FLD_MSB_LSB(2, 2)
#define INTEN_FLD_PM_ABNORMAL_SOF_INTEN REG_FLD_MSB_LSB(4, 4)
#define INTEN_FLD_RDMA_FME_UND_INTEN REG_FLD_MSB_LSB(8, 8)
#define INTEN_FLD_RDMA_FME_SWRST_DONE_INTEN REG_FLD_MSB_LSB(9, 9)
#define INTEN_FLD_RDMA_FME_HWRST_DONE_INTEN REG_FLD_MSB_LSB(10, 10)
#define INTEN_FLD_RDMA_EOF_ABNORMAL_INTEN REG_FLD_MSB_LSB(11, 11)
#define INTEN_FLD_RDMA_SMI_UNDERFLOW_INTEN REG_FLD_MSB_LSB(12, 12)
#define DISP_POSTMASK_INTSTA 0xC
#define DISP_POSTMASK_CFG 0x20
#define CFG_FLD_STALL_CG_ON REG_FLD_MSB_LSB(8, 8)
#define CFG_FLD_GCLAST_EN REG_FLD_MSB_LSB(6, 6)
#define CFG_FLD_BGCLR_IN_SEL REG_FLD_MSB_LSB(2, 2)
#define CFG_FLD_DRAM_MODE REG_FLD_MSB_LSB(1, 1)
#define CFG_FLD_RELAY_MODE REG_FLD_MSB_LSB(0, 0)
#define DISP_POSTMASK_SIZE 0x30
#define DISP_POSTMASK_SRAM_CFG 0x40
#define SRAM_CFG_FLD_MASK_NUM_SW_SET REG_FLD_MSB_LSB(11, 4)
#define SRAM_CFG_FLD_MASK_L_TOP_EN REG_FLD_MSB_LSB(3, 3)
#define SRAM_CFG_FLD_MASK_L_BOTTOM_EN REG_FLD_MSB_LSB(2, 2)
#define SRAM_CFG_FLD_MASK_R_TOP_EN REG_FLD_MSB_LSB(1, 1)
#define SRAM_CFG_FLD_MASK_R_BOTTOM_EN REG_FLD_MSB_LSB(0, 0)
#define DISP_POSTMASK_BLEND_CFG 0x50
#define BLEND_CFG_FLD_CONST_BLD REG_FLD_MSB_LSB(2, 2)
#define BLEND_CFG_FLD_PARGB_BLD REG_FLD_MSB_LSB(1, 1)
#define BLEND_CFG_FLD_A_EN REG_FLD_MSB_LSB(0, 0)
#define DISP_POSTMASK_ROI_BGCLR 0x54
#define DISP_POSTMASK_MASK_CLR 0x58
#define DISP_REG_POSTMASK_SODI 0x60
#define PM_MASK_THRESHOLD_LOW_FOR_SODI  REG_FLD_MSB_LSB(13, 0)
#define PM_MASK_THRESHOLD_HIGH_FOR_SODI REG_FLD_MSB_LSB(29, 16)
#define DISP_POSTMASK_STATUS 0xA0
#define DISP_POSTMASK_INPUT_COUNT 0xA4
#define DISP_POSTMASK_MEM_ADDR 0x100
#define DISP_POSTMASK_MEM_LENGTH 0x104
#define DISP_POSTMASK_RDMA_FIFO_CTRL 0x108
#define DISP_POSTMASK_MEM_GMC_SETTING2 0x10C
#define MEM_GMC_FLD_FORCE_REQ_TH REG_FLD_MSB_LSB(30, 30)
#define MEM_GMC_FLD_REQ_TH_ULTRA REG_FLD_MSB_LSB(29, 29)
#define MEM_GMC_FLD_REQ_TH_PREULTRA REG_FLD_MSB_LSB(28, 28)
#define MEM_GMC_FLD_ISSUE_REQ_TH_URG REG_FLD_MSB_LSB(27, 16)
#define MEM_GMC_FLD_ISSUE_REQ_TH REG_FLD_MSB_LSB(11, 0)
#define DISP_POSTMASK_PAUSE_REGION 0x110
#define PAUSE_REGION_FLD_RDMA_PAUSE_END REG_FLD_MSB_LSB(27, 16)
#define PAUSE_REGION_FLD_RDMA_PAUSE_START REG_FLD_MSB_LSB(11, 0)
#define DISP_POSTMASK_MEM_ADDR_MSB 0x114
#define DISP_POSTMASK_RDMA_GREQ_NUM 0x130
#define GREQ_FLD_IOBUF_FLUSH_ULTRA REG_FLD_MSB_LSB(31, 31)
#define GREQ_FLD_IOBUF_FLUSH_PREULTRA REG_FLD_MSB_LSB(30, 30)
#define GREQ_FLD_GRP_BRK_STOP REG_FLD_MSB_LSB(29, 29)
#define GREQ_FLD_GRP_END_STOP REG_FLD_MSB_LSB(28, 28)
#define GREQ_FLD_GREQ_STOP_EN REG_FLD_MSB_LSB(27, 27)
#define GREQ_FLD_GREQ_DIS_CNT REG_FLD_MSB_LSB(26, 24)
#define GREQ_FLD_OSTD_GREQ_NUM REG_FLD_MSB_LSB(23, 16)
#define GREQ_FLD_GREQ_NUM_SHT REG_FLD_MSB_LSB(14, 13)
#define GREQ_FLD_GREQ_NUM_SHT_VAL REG_FLD_MSB_LSB(12, 12)
#define GREQ_FLD_GREQ_URG_NUM REG_FLD_MSB_LSB(7, 4)
#define GREQ_FLD_GREQ_NUM REG_FLD_MSB_LSB(3, 0)
#define DISP_POSTMASK_RDMA_GREQ_URG_NUM 0x134
#define GREQ_URG_FLD_ARB_URG_BIAS REG_FLD_MSB_LSB(12, 12)
#define GREQ_URG_FLD_ARB_GREQ_URG_TH REG_FLD_MSB_LSB(11, 0)
#define DISP_POSTMASK_RDMA_ULTRA_SRC 0x140
#define ULTRA_FLD_ULTRA_RDMA_SRC REG_FLD_MSB_LSB(15, 14)
#define ULTRA_FLD_ULTRA_ROI_END_SRC REG_FLD_MSB_LSB(13, 12)
#define ULTRA_FLD_ULTRA_SMI_SRC REG_FLD_MSB_LSB(11, 10)
#define ULTRA_FLD_ULTRA_BUF_SRC REG_FLD_MSB_LSB(9, 8)
#define ULTRA_FLD_PREULTRA_RDMA_SRC REG_FLD_MSB_LSB(7, 6)
#define ULTRA_FLD_PREULTRA_ROI_END_SRC REG_FLD_MSB_LSB(5, 4)
#define ULTRA_FLD_PREULTRA_SMI_SRC REG_FLD_MSB_LSB(3, 2)
#define ULTRA_FLD_PREULTRA_BUF_SRC REG_FLD_MSB_LSB(1, 0)
#define DISP_POSTMASK_RDMA_BUF_LOW_TH 0x144
#define TH_FLD_RDMA_PREULTRA_LOW_TH REG_FLD_MSB_LSB(23, 12)
#define TH_FLD_RDMA_ULTRA_LOW_TH REG_FLD_MSB_LSB(11, 0)
#define DISP_POSTMASK_RDMA_BUF_HIGH_TH 0x148
#define TH_FLD_RDMA_PREULTRA_HIGH_DIS REG_FLD_MSB_LSB(31, 31)
#define TH_FLD_RDMA_PREULTRA_HIGH_TH REG_FLD_MSB_LSB(23, 12)
#define DISP_POSTMASK_NUM_0 0x800
#define DISP_POSTMASK_NUM(n) (DISP_POSTMASK_NUM_0 + (0x4 * (n)))
#define DISP_POSTMASK_GRAD_VAL_0 0xA00
#define DISP_POSTMASK_GRAD_VAL(n) (DISP_POSTMASK_GRAD_VAL_0 + (0x4 * (n)))

static irqreturn_t mtk_postmask_irq_handler(int irq, void *dev_id) __attribute__((unused));

struct mtk_disp_postmask_data {
	bool is_support_34bits;
};

struct mtk_disp_postmask {
	struct mtk_ddp_comp ddp_comp;
	struct drm_crtc *crtc;
	unsigned int underflow_cnt;
	unsigned int abnormal_cnt;
	const struct mtk_disp_postmask_data *data;
};

static inline struct mtk_disp_postmask *comp_to_postmask(struct mtk_ddp_comp *comp)
{
	return container_of(comp, struct mtk_disp_postmask, ddp_comp);
}

static irqreturn_t mtk_postmask_irq_handler(int irq, void *dev_id)
{
	struct mtk_disp_postmask *priv = dev_id;
	struct mtk_ddp_comp *postmask = NULL;
	unsigned int val = 0;
	unsigned int ret = 0;

	if (IS_ERR_OR_NULL(priv))
		return IRQ_NONE;

	postmask = &priv->ddp_comp;
	if (IS_ERR_OR_NULL(postmask))
		return IRQ_NONE;

	if (mtk_drm_top_clk_isr_get("postmask_irq") == false) {
		DDPIRQ("%s, top clk off\n", __func__);
		return IRQ_NONE;
	}

	val = readl(postmask->regs + DISP_POSTMASK_INTSTA);
	if (!val) {
		ret = IRQ_NONE;
		goto out;
	}

	DRM_MMP_MARK(IRQ, postmask->regs_pa, val);
	DRM_MMP_MARK(postmask0, val, 0);

	if (val & 0x110)
		DRM_MMP_MARK(abnormal_irq, val, postmask->id);

	DDPIRQ("%s irq, val:0x%x\n", mtk_dump_comp_str(postmask), val);

	writel(~val, postmask->regs + DISP_POSTMASK_INTSTA);

	if (val & (1 << 0))
		DDPIRQ("[IRQ] %s: input frame end!\n",
		       mtk_dump_comp_str(postmask));
	if (val & (1 << 1))
		DDPIRQ("[IRQ] %s: output frame end!\n",
		       mtk_dump_comp_str(postmask));
	if (val & (1 << 2))
		DDPIRQ("[IRQ] %s: frame start!\n", mtk_dump_comp_str(postmask));
	if (val & (1 << 4)) {
		DDPPR_ERR("[IRQ] %s: abnormal SOF! cnt=%d\n",
			  mtk_dump_comp_str(postmask), priv->abnormal_cnt);
		priv->abnormal_cnt++;
	}

	//if (val & (1 << 8)) {
		//DDPPR_ERR("[IRQ] %s: frame underflow! cnt=%d\n",
			  //mtk_dump_comp_str(postmask), priv->underflow_cnt);
		//priv->underflow_cnt++;
	//}

	ret = IRQ_HANDLED;

out:
	mtk_drm_top_clk_isr_put("postmask_irq");

	return ret;
}

static void mtk_postmask_config(struct mtk_ddp_comp *comp,
				struct mtk_ddp_config *cfg,
				struct cmdq_pkt *handle)
{
	unsigned int value;
	struct mtk_panel_params *panel_ext =
		mtk_drm_get_lcm_ext_params(&comp->mtk_crtc->base);
#ifndef POSTMASK_DRAM_MODE
	unsigned int i = 0;
	unsigned int num = 0;
#else
	struct mtk_drm_gem_obj *gem;
	unsigned int size = 0;
	dma_addr_t addr = 0;
	unsigned int force_relay = 0;
	struct mtk_disp_postmask *postmask = comp_to_postmask(comp);
#endif
	unsigned int width;

	if (comp->mtk_crtc->is_dual_pipe)
		width = cfg->w / 2;
	else
		width = cfg->w;

	value = (REG_FLD_VAL((BLEND_CFG_FLD_A_EN), 1) |
		 REG_FLD_VAL((BLEND_CFG_FLD_PARGB_BLD), 0) |
		 REG_FLD_VAL((BLEND_CFG_FLD_CONST_BLD), 0));
	mtk_ddp_write_relaxed(comp, value, DISP_POSTMASK_BLEND_CFG, handle);

	mtk_ddp_write_relaxed(comp, 0xff000000, DISP_POSTMASK_ROI_BGCLR,
			      handle);
	mtk_ddp_write_relaxed(comp, 0xff000000, DISP_POSTMASK_MASK_CLR, handle);

	value = (width << 16) + cfg->h;
	mtk_ddp_write_relaxed(comp, value, DISP_POSTMASK_SIZE, handle);

	if (!panel_ext)
		DDPPR_ERR("%s:panel_ext not found\n", __func__);

	if (panel_ext && panel_ext->round_corner_en) {
		value = (REG_FLD_VAL((PAUSE_REGION_FLD_RDMA_PAUSE_START),
				     panel_ext->corner_pattern_height) |
			 REG_FLD_VAL(
				 (PAUSE_REGION_FLD_RDMA_PAUSE_END),
				 cfg->h -
					 panel_ext->corner_pattern_height_bot));
		mtk_ddp_write_relaxed(comp, value, DISP_POSTMASK_PAUSE_REGION,
				      handle);

		value = (REG_FLD_VAL((MEM_GMC_FLD_ISSUE_REQ_TH), 63) |
			 REG_FLD_VAL((MEM_GMC_FLD_ISSUE_REQ_TH_URG), 63) |
			 REG_FLD_VAL((MEM_GMC_FLD_REQ_TH_PREULTRA), 0) |
			 REG_FLD_VAL((MEM_GMC_FLD_REQ_TH_ULTRA), 1) |
			 REG_FLD_VAL((MEM_GMC_FLD_FORCE_REQ_TH), 0));
		mtk_ddp_write_relaxed(comp, value,
				      DISP_POSTMASK_MEM_GMC_SETTING2, handle);

		value = (REG_FLD_VAL((GREQ_FLD_GREQ_NUM), 7) |
			 REG_FLD_VAL((GREQ_FLD_GREQ_URG_NUM), 7) |
			 REG_FLD_VAL((GREQ_FLD_GREQ_NUM_SHT_VAL), 1) |
			 REG_FLD_VAL((GREQ_FLD_GREQ_NUM_SHT), 0) |
			 REG_FLD_VAL((GREQ_FLD_OSTD_GREQ_NUM), 0xFF) |
			 REG_FLD_VAL((GREQ_FLD_GREQ_DIS_CNT), 1) |
			 REG_FLD_VAL((GREQ_FLD_GREQ_STOP_EN), 0) |
			 REG_FLD_VAL((GREQ_FLD_GRP_END_STOP), 1) |
			 REG_FLD_VAL((GREQ_FLD_GRP_BRK_STOP), 1) |
			 REG_FLD_VAL((GREQ_FLD_IOBUF_FLUSH_PREULTRA), 1) |
			 REG_FLD_VAL((GREQ_FLD_IOBUF_FLUSH_ULTRA), 1));
		mtk_ddp_write_relaxed(comp, value, DISP_POSTMASK_RDMA_GREQ_NUM,
				      handle);

		value = (REG_FLD_VAL((GREQ_URG_FLD_ARB_GREQ_URG_TH), 0) |
			 REG_FLD_VAL((GREQ_URG_FLD_ARB_URG_BIAS), 0));
		mtk_ddp_write_relaxed(comp, value,
				      DISP_POSTMASK_RDMA_GREQ_URG_NUM, handle);

		value = (REG_FLD_VAL((ULTRA_FLD_PREULTRA_BUF_SRC), 0) |
			 REG_FLD_VAL((ULTRA_FLD_PREULTRA_SMI_SRC), 1) |
			 REG_FLD_VAL((ULTRA_FLD_PREULTRA_ROI_END_SRC), 0) |
			 REG_FLD_VAL((ULTRA_FLD_PREULTRA_RDMA_SRC), 0) |
			 REG_FLD_VAL((ULTRA_FLD_ULTRA_BUF_SRC), 0) |
			 REG_FLD_VAL((ULTRA_FLD_ULTRA_SMI_SRC), 1) |
			 REG_FLD_VAL((ULTRA_FLD_ULTRA_ROI_END_SRC), 0) |
			 REG_FLD_VAL((ULTRA_FLD_ULTRA_RDMA_SRC), 0));
		mtk_ddp_write_relaxed(comp, value, DISP_POSTMASK_RDMA_ULTRA_SRC,
				      handle);

		value = (REG_FLD_VAL((TH_FLD_RDMA_ULTRA_LOW_TH), 0xFFF) |
			 REG_FLD_VAL((TH_FLD_RDMA_PREULTRA_LOW_TH), 0xFFF));
		mtk_ddp_write_relaxed(comp, value,
				      DISP_POSTMASK_RDMA_BUF_LOW_TH, handle);

		value = (REG_FLD_VAL((TH_FLD_RDMA_PREULTRA_HIGH_TH), 0xFFF) |
			 REG_FLD_VAL((TH_FLD_RDMA_PREULTRA_HIGH_DIS), 0));
		mtk_ddp_write_relaxed(comp, value,
				      DISP_POSTMASK_RDMA_BUF_HIGH_TH, handle);

#ifdef POSTMASK_DRAM_MODE
		if (comp->mtk_crtc->is_dual_pipe) {
			if (comp->id == DDP_COMPONENT_POSTMASK0 &&
			    comp->mtk_crtc->round_corner_gem_l &&
			    panel_ext->corner_pattern_tp_size_l) {
				gem = comp->mtk_crtc->round_corner_gem_l;
				addr = gem->dma_addr;
				size = panel_ext->corner_pattern_tp_size_l;
			} else if (comp->id == DDP_COMPONENT_POSTMASK1 &&
			    comp->mtk_crtc->round_corner_gem_r &&
			    panel_ext->corner_pattern_tp_size_r) {
				gem = comp->mtk_crtc->round_corner_gem_r;
				addr = gem->dma_addr;
				size = panel_ext->corner_pattern_tp_size_r;
			}
		} else if (comp->mtk_crtc->round_corner_gem &&
			   panel_ext->corner_pattern_tp_size) {
			gem = comp->mtk_crtc->round_corner_gem;
			addr = gem->dma_addr;
			size = panel_ext->corner_pattern_tp_size;
		}

		if (addr == 0 || size == 0) {
			DDPPR_ERR("invalid postmaks addr/size\n");
			force_relay = 1;
		}
		value = (REG_FLD_VAL((CFG_FLD_RELAY_MODE), force_relay) |
			 REG_FLD_VAL((CFG_FLD_DRAM_MODE), 1) |
			 REG_FLD_VAL((CFG_FLD_BGCLR_IN_SEL), 1) |
			 REG_FLD_VAL((CFG_FLD_GCLAST_EN), 1) |
			 REG_FLD_VAL((CFG_FLD_STALL_CG_ON), 1));
		mtk_ddp_write_relaxed(comp, value, DISP_POSTMASK_CFG, handle);

		mtk_ddp_write_relaxed(comp, addr, DISP_POSTMASK_MEM_ADDR,
				      handle);

		if (postmask->data->is_support_34bits)
			mtk_ddp_write_relaxed(comp, (addr >> 32),
					DISP_POSTMASK_MEM_ADDR_MSB, handle);

		mtk_ddp_write_relaxed(comp, size, DISP_POSTMASK_MEM_LENGTH,
				      handle);
#else
		value = (REG_FLD_VAL((CFG_FLD_RELAY_MODE), 0) |
			 REG_FLD_VAL((CFG_FLD_DRAM_MODE), 0) |
			 REG_FLD_VAL((CFG_FLD_BGCLR_IN_SEL), 1) |
			 REG_FLD_VAL((CFG_FLD_GCLAST_EN), 1) |
			 REG_FLD_VAL((CFG_FLD_STALL_CG_ON), 1));
		mtk_ddp_write_relaxed(comp, value, DISP_POSTMASK_CFG, handle);

		value = (REG_FLD_VAL((SRAM_CFG_FLD_MASK_NUM_SW_SET),
				     panel_ext->corner_pattern_height) |
			 REG_FLD_VAL((SRAM_CFG_FLD_MASK_L_TOP_EN), 1) |
			 REG_FLD_VAL((SRAM_CFG_FLD_MASK_L_BOTTOM_EN), 1) |
			 REG_FLD_VAL((SRAM_CFG_FLD_MASK_R_TOP_EN), 1) |
			 REG_FLD_VAL((SRAM_CFG_FLD_MASK_R_BOTTOM_EN), 1));
		mtk_ddp_write_relaxed(comp, value, DISP_POSTMASK_SRAM_CFG,
				      handle);

		num = POSTMASK_MASK_MAX_NUM;
		for (i = 0; i < num; i++) {
			mtk_ddp_write_relaxed(comp, 0x1F001F00,
					      DISP_POSTMASK_NUM(i), handle);
		}

		num = POSTMASK_GRAD_MAX_NUM;
		for (i = 0; i < num; i++) {
			mtk_ddp_write_relaxed(
				comp, 0x0, DISP_POSTMASK_GRAD_VAL(i), handle);
		}
#endif
		/* config relay mode */
	} else {
		value = (REG_FLD_VAL((CFG_FLD_RELAY_MODE), 1) |
			 REG_FLD_VAL((CFG_FLD_DRAM_MODE), 1) |
			 REG_FLD_VAL((CFG_FLD_BGCLR_IN_SEL), 1) |
			 REG_FLD_VAL((CFG_FLD_GCLAST_EN), 1) |
			 REG_FLD_VAL((CFG_FLD_STALL_CG_ON), 1));
		mtk_ddp_write_relaxed(comp, value, DISP_POSTMASK_CFG, handle);
	}
}

int mtk_postmask_dump(struct mtk_ddp_comp *comp)
{
	void __iomem *baddr = comp->regs;

	DDPDUMP("== %s REGS:0x%x ==\n", mtk_dump_comp_str(comp), comp->regs_pa);

	mtk_serial_dump_reg(baddr, 0x0, 4);
	mtk_serial_dump_reg(baddr, 0x20, 1);
	mtk_serial_dump_reg(baddr, 0x30, 1);
	mtk_serial_dump_reg(baddr, 0x40, 3);
	mtk_serial_dump_reg(baddr, 0x50, 3);
	mtk_serial_dump_reg(baddr, 0xA0, 2);
	mtk_serial_dump_reg(baddr, 0xB0, 3);
	mtk_serial_dump_reg(baddr, 0x100, 4);
	mtk_serial_dump_reg(baddr, 0x110, 2);
	mtk_serial_dump_reg(baddr, 0x130, 2);
	mtk_serial_dump_reg(baddr, 0x140, 3);

	return 0;
}

int mtk_postmask_analysis(struct mtk_ddp_comp *comp)
{
	void __iomem *baddr = comp->regs;
	struct mtk_disp_postmask *postmask = comp_to_postmask(comp);
	dma_addr_t addr = 0;

	DDPDUMP("== %s ANALYSIS:0x%x ==\n", mtk_dump_comp_str(comp), comp->regs_pa);
	DDPDUMP("en=%d,cfg=0x%x,size=(%dx%d)\n",
		readl(DISP_POSTMASK_EN + baddr) & 0x1,
		readl(DISP_POSTMASK_CFG + baddr),
		(readl(DISP_POSTMASK_SIZE + baddr) >> 16) & 0x1fff,
		readl(DISP_POSTMASK_SIZE + baddr) & 0x1fff);
	DDPDUMP("blend_cfg=0x%x,bg=0x%x,mask=0x%x\n",
		readl(DISP_POSTMASK_BLEND_CFG + baddr),
		readl(DISP_POSTMASK_ROI_BGCLR + baddr),
		readl(DISP_POSTMASK_MASK_CLR + baddr));
	DDPDUMP("fifo_cfg=%d,gmc=0x%x,threshold=(0x%x,0x%x)\n",
		readl(DISP_POSTMASK_RDMA_FIFO_CTRL + baddr),
		readl(DISP_POSTMASK_MEM_GMC_SETTING2 + baddr),
		readl(DISP_POSTMASK_RDMA_BUF_LOW_TH + baddr),
		readl(DISP_POSTMASK_RDMA_BUF_HIGH_TH + baddr));

	if (postmask->data->is_support_34bits) {
		addr = readl(DISP_POSTMASK_MEM_ADDR_MSB + baddr);
		addr = (addr << 32);
	}

	addr += readl(DISP_POSTMASK_MEM_ADDR + baddr);

	DDPDUMP("mem_addr=0x%x,length=0x%x\n",
		addr, readl(DISP_POSTMASK_MEM_LENGTH + baddr));

	DDPDUMP("status=0x%x,cur_pos=0x%x\n",
		readl(DISP_POSTMASK_STATUS + baddr),
		readl(DISP_POSTMASK_INPUT_COUNT + baddr));

	return 0;
}

static int mtk_postmask_io_cmd(struct mtk_ddp_comp *comp,
			       struct cmdq_pkt *handle,
			       enum mtk_ddp_io_cmd io_cmd, void *params);

static void mtk_postmask_start(struct mtk_ddp_comp *comp,
			       struct cmdq_pkt *handle)
{
	DDPDBG("%s\n", __func__);

	mtk_postmask_io_cmd(comp, handle, IRQ_LEVEL_NORMAL, NULL);

	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_POSTMASK_EN, 1, ~0);
}

static void mtk_postmask_stop(struct mtk_ddp_comp *comp,
			      struct cmdq_pkt *handle)
{
	DDPDBG("%s\n", __func__);


	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_POSTMASK_INTEN, 0, ~0);
	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_POSTMASK_EN, 0, ~0);
	cmdq_pkt_write(handle, comp->cmdq_base,
		       comp->regs_pa + DISP_POSTMASK_INTSTA, 0, ~0);
}

static int mtk_disp_postmask_bind(struct device *dev, struct device *master,
				  void *data)
{
	struct mtk_disp_postmask *priv = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	int ret;

	DDPINFO("%s\n", __func__);

	ret = mtk_ddp_comp_register(drm_dev, &priv->ddp_comp);
	if (ret < 0) {
		dev_err(dev, "Failed to register component %s: %d\n",
			dev->of_node->full_name, ret);
		return ret;
	}

	return 0;
}

static void mtk_disp_postmask_unbind(struct device *dev, struct device *master,
				     void *data)
{
	struct mtk_disp_postmask *priv = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;

	mtk_ddp_comp_unregister(drm_dev, &priv->ddp_comp);
}

static void mtk_postmask_prepare(struct mtk_ddp_comp *comp)
{
	mtk_ddp_comp_clk_prepare(comp);
}

static void mtk_postmask_unprepare(struct mtk_ddp_comp *comp)
{
	mtk_ddp_comp_clk_unprepare(comp);
}

static int mtk_postmask_io_cmd(struct mtk_ddp_comp *comp,
			       struct cmdq_pkt *handle,
			       enum mtk_ddp_io_cmd io_cmd, void *params)
{
	switch (io_cmd) {
	case IRQ_LEVEL_NORMAL: {
		unsigned int inten;

		inten = REG_FLD_VAL(INTEN_FLD_PM_IF_FME_END_INTEN, 1) |
			REG_FLD_VAL(INTEN_FLD_PM_FME_CPL_INTEN, 1) |
			REG_FLD_VAL(INTEN_FLD_PM_START_INTEN, 1) |
			REG_FLD_VAL(INTEN_FLD_PM_ABNORMAL_SOF_INTEN, 1) |
			REG_FLD_VAL(INTEN_FLD_RDMA_FME_UND_INTEN, 1) |
			REG_FLD_VAL(INTEN_FLD_RDMA_EOF_ABNORMAL_INTEN, 1);
		cmdq_pkt_write(handle, comp->cmdq_base,
			       comp->regs_pa + DISP_POSTMASK_INTEN, inten,
			       inten);
		break;
	}
	case IRQ_LEVEL_IDLE: {
		unsigned int inten;

		inten = REG_FLD_VAL(INTEN_FLD_PM_IF_FME_END_INTEN, 1) |
			REG_FLD_VAL(INTEN_FLD_PM_FME_CPL_INTEN, 1) |
			REG_FLD_VAL(INTEN_FLD_PM_START_INTEN, 1);
		cmdq_pkt_write(handle, comp->cmdq_base,
			       comp->regs_pa + DISP_POSTMASK_INTEN, 0, inten);
		break;
	}
	default:
		break;
	}

	return 0;
}

static const struct mtk_ddp_comp_funcs mtk_disp_postmask_funcs = {
	.config = mtk_postmask_config,
	.start = mtk_postmask_start,
	.stop = mtk_postmask_stop,
	.prepare = mtk_postmask_prepare,
	.unprepare = mtk_postmask_unprepare,
	.io_cmd = mtk_postmask_io_cmd,
};

static const struct component_ops mtk_disp_postmask_component_ops = {
	.bind = mtk_disp_postmask_bind, .unbind = mtk_disp_postmask_unbind,
};

static int mtk_disp_postmask_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_disp_postmask *priv;
	enum mtk_ddp_comp_id comp_id;
	int ret;

	DDPINFO("%s+\n", __func__);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	comp_id = mtk_ddp_comp_get_id(dev->of_node, MTK_DISP_POSTMASK);
	if ((int)comp_id < 0) {
		DDPPR_ERR("Failed to identify by alias: %d\n", comp_id);
		return comp_id;
	}

	ret = mtk_ddp_comp_init(dev, dev->of_node, &priv->ddp_comp, comp_id,
				&mtk_disp_postmask_funcs);
	if (ret != 0) {
		DDPPR_ERR("Failed to initialize component: %d\n", ret);
		return ret;
	}

	priv->data = of_device_get_match_data(dev);

	platform_set_drvdata(pdev, priv);

	mtk_ddp_comp_pm_enable(&priv->ddp_comp);

	ret = component_add(dev, &mtk_disp_postmask_component_ops);
	if (ret != 0) {
		dev_err(dev, "Failed to add component: %d\n", ret);
		mtk_ddp_comp_pm_disable(&priv->ddp_comp);
	}
	DDPINFO("%s-\n", __func__);

	return ret;
}

static int mtk_disp_postmask_remove(struct platform_device *pdev)
{
	struct mtk_disp_postmask *priv = dev_get_drvdata(&pdev->dev);

	component_del(&pdev->dev, &mtk_disp_postmask_component_ops);
	mtk_ddp_comp_pm_disable(&priv->ddp_comp);

	return 0;
}

static const struct mtk_disp_postmask_data mt6779_postmask_driver_data = {
	.is_support_34bits = false,
};

static const struct mtk_disp_postmask_data mt6885_postmask_driver_data = {
	.is_support_34bits = false,
};

static const struct mtk_disp_postmask_data mt6983_postmask_driver_data = {
	.is_support_34bits = true,
};

static const struct mtk_disp_postmask_data mt6895_postmask_driver_data = {
	.is_support_34bits = true,
};

static const struct mtk_disp_postmask_data mt6873_postmask_driver_data = {
	.is_support_34bits = false,
};

static const struct mtk_disp_postmask_data mt6853_postmask_driver_data = {
	.is_support_34bits = false,
};

static const struct mtk_disp_postmask_data mt6833_postmask_driver_data = {
	.is_support_34bits = false,
};

static const struct mtk_disp_postmask_data mt6879_postmask_driver_data = {
	.is_support_34bits = true,
};

static const struct mtk_disp_postmask_data mt6855_postmask_driver_data = {
	.is_support_34bits = false,
};

static const struct of_device_id mtk_disp_postmask_driver_dt_match[] = {
	{ .compatible = "mediatek,mt6779-disp-postmask",
	  .data = &mt6779_postmask_driver_data},
	{ .compatible = "mediatek,mt6885-disp-postmask",
	  .data = &mt6885_postmask_driver_data},
	{ .compatible = "mediatek,mt6983-disp-postmask",
	  .data = &mt6983_postmask_driver_data},
	{ .compatible = "mediatek,mt6895-disp-postmask",
	  .data = &mt6895_postmask_driver_data},
	{ .compatible = "mediatek,mt6873-disp-postmask",
	  .data = &mt6873_postmask_driver_data},
	{ .compatible = "mediatek,mt6853-disp-postmask",
	  .data = &mt6853_postmask_driver_data},
	{ .compatible = "mediatek,mt6833-disp-postmask",
	  .data = &mt6833_postmask_driver_data},
	{ .compatible = "mediatek,mt6879-disp-postmask",
	  .data = &mt6879_postmask_driver_data},
	{ .compatible = "mediatek,mt6855-disp-postmask",
	  .data = &mt6855_postmask_driver_data},
	{},
};

struct platform_driver mtk_disp_postmask_driver = {
	.probe = mtk_disp_postmask_probe,
	.remove = mtk_disp_postmask_remove,
	.driver = {

			.name = "mediatek-disp-postmask",
			.owner = THIS_MODULE,
			.of_match_table = mtk_disp_postmask_driver_dt_match,
		},
};
