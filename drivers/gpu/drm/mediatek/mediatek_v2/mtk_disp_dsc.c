// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>

#ifndef DRM_CMDQ_DISABLE
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#else
#include "mtk-cmdq-ext.h"
#endif

#include "mtk_drm_crtc.h"
#include "mtk_drm_drv.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_dump.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_gem.h"
#include "mtk_drm_fb.h"
#include "mtk_dp_api.h"
#include "mtk_disp_dsc.h"
#include "platform/mtk_drm_6789.h"

#define DISP_REG_DSC_CON			0x0000
	#define DSC_EN BIT(0)
	#define DSC_DUAL_INOUT BIT(2)
	#define DSC_IN_SRC_SEL BIT(3)
	#define DSC_BYPASS BIT(4)
	#define DSC_RELAY BIT(5)
	#define DSC_EMPTY_FLAG_SEL REG_FLD_MSB_LSB(15, 14)
	#define DSC_UFOE_SEL BIT(16)
	#define DSC_OUTPUT_SWAP BIT(18)
	#define CON_FLD_DSC_EN		REG_FLD_MSB_LSB(0, 0)
	#define CON_FLD_DISP_DSC_BYPASS		REG_FLD_MSB_LSB(4, 4)

#define DISP_REG_DSC_INTEN			0x0004
#define DISP_REG_DSC_INTSTA			0x0008
	#define DSC_DONE BIT(0)
	#define DSC_ERR BIT(1)
	#define DSC_ZERO_FIFO BIT(2)
	#define DSC_ABN_EOF BIT(3)

#define DISP_REG_DSC_INTACK			0x000C

#define DISP_REG_DSC_SPR			0x0014
#define CFG_FLD_DSC_SPR_EN	BIT(26)
#define CFG_FLD_DSC_SPR_FORMAT_SEL	BIT(24)

#define DISP_REG_DSC_PIC_W			0x0018
	#define CFG_FLD_PIC_WIDTH	REG_FLD_MSB_LSB(15, 0)
	#define CFG_FLD_PIC_HEIGHT_M1	REG_FLD_MSB_LSB(31, 16)

#define DISP_REG_DSC_PIC_H			0x001C

#define DISP_REG_DSC_SLICE_W		0x0020
	#define CFG_FLD_SLICE_WIDTH	REG_FLD_MSB_LSB(15, 0)

#define DISP_REG_DSC_SLICE_H		0x0024

#define DISP_REG_DSC_CHUNK_SIZE		0x0028

#define DISP_REG_DSC_BUF_SIZE		0x002C

#define DISP_REG_DSC_MODE			0x0030
	#define DSC_SLICE_MODE BIT(0)
	#define DSC_RGB_SWAP BIT(2)
#define DISP_REG_DSC_CFG			0x0034

#define DISP_REG_DSC_PAD			0x0038
#define DISP_REG_DSC_ENC_WIDTH		0x003C


#define DISP_REG_DSC_DBG_CON		0x0060
	#define DSC_CKSM_CAL_EN BIT(9)
#define DISP_REG_DSC_OBUF			0x0070
#define DISP_REG_DSC_PPS0			0x0080
#define DISP_REG_DSC_PPS1			0x0084
#define DISP_REG_DSC_PPS2			0x0088
#define DISP_REG_DSC_PPS3			0x008C
#define DISP_REG_DSC_PPS4			0x0090
#define DISP_REG_DSC_PPS5			0x0094
#define DISP_REG_DSC_PPS6			0x0098
#define DISP_REG_DSC_PPS7			0x009C
#define DISP_REG_DSC_PPS8			0x00A0
#define DISP_REG_DSC_PPS9			0x00A4
#define DISP_REG_DSC_PPS10			0x00A8
#define DISP_REG_DSC_PPS11			0x00AC
#define DISP_REG_DSC_PPS12			0x00B0
#define DISP_REG_DSC_PPS13			0x00B4
#define DISP_REG_DSC_PPS14			0x00B8
#define DISP_REG_DSC_PPS15			0x00BC
#define DISP_REG_DSC_PPS16			0x00C0
#define DISP_REG_DSC_PPS17			0x00C4
#define DISP_REG_DSC_PPS18			0x00C8
#define DISP_REG_DSC_PPS19			0x00CC

#define DISP_REG_DSC_SHADOW			0x0200
#define DSC_FORCE_COMMIT	BIT(0)
#define DSC_BYPASS_SHADOW	BIT(1)
#define DSC_READ_WORKING	BIT(2)
#define DISP_REG_DSC1_OFFSET        0x0400
#define DISP_REG_SHADOW_CTRL(module)	((module)->data->shadow_ctrl_reg)

/**
 * struct mtk_disp_dsc - DISP_DSC driver structure
 * @ddp_comp - structure containing type enum and hardware resources
 */
struct mtk_disp_dsc {
	struct mtk_ddp_comp	 ddp_comp;
	const struct mtk_disp_dsc_data *data;
	int enable;
};

static inline struct mtk_disp_dsc *comp_to_dsc(struct mtk_ddp_comp *comp)
{
	return container_of(comp, struct mtk_disp_dsc, ddp_comp);
}

static irqreturn_t mtk_dsc_irq_handler(int irq, void *dev_id)
{
	struct mtk_disp_dsc *priv = dev_id;
	struct mtk_ddp_comp *dsc = NULL;
	unsigned int val = 0;
	unsigned int ret = 0;

	if (IS_ERR_OR_NULL(priv))
		return IRQ_NONE;

	dsc = &priv->ddp_comp;
	if (IS_ERR_OR_NULL(dsc))
		return IRQ_NONE;

	if (mtk_drm_top_clk_isr_get("dsc_irq") == false) {
		DDPIRQ("%s, top clk off\n", __func__);
		return IRQ_NONE;
	}

	val = readl(dsc->regs + DISP_REG_DSC_INTSTA);
	if (!val) {
		ret = IRQ_NONE;
		goto out;
	}

	DDPIRQ("%s irq, val:0x%x\n", mtk_dump_comp_str(dsc), val);

	writel(val, dsc->regs + DISP_REG_DSC_INTACK);
	writel(0x0, dsc->regs + DISP_REG_DSC_INTACK);

	if (val & (1 << 0))
		DDPIRQ("[IRQ] %s: frame complete!\n",
			mtk_dump_comp_str(dsc));

	if (val & (1 << 1)) {
		static DEFINE_RATELIMIT_STATE(err_ratelimit, 1 * HZ, 20);

		if (__ratelimit(&err_ratelimit))
			DDPPR_ERR("[IRQ] %s: err!\n",
				  mtk_dump_comp_str(dsc));
	}
	if (val & (1 << 2)) {
		static DEFINE_RATELIMIT_STATE(zfifo_ratelimit, 1 * HZ, 20);

		if (__ratelimit(&zfifo_ratelimit))
			DDPPR_ERR("[IRQ] %s: zero fifo!\n",
				  mtk_dump_comp_str(dsc));
	}
	if (val & (1 << 3))
		DDPPR_ERR("[IRQ] %s: abnormal EOF!\n",
			  mtk_dump_comp_str(dsc));

	ret = IRQ_HANDLED;
out:
	mtk_drm_top_clk_isr_put("dsc_irq");

	return ret;
}

static void mtk_dsc_start(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle)
{
	void __iomem *baddr = comp->regs;
	struct mtk_disp_dsc *dsc = comp_to_dsc(comp);

	mtk_ddp_write_mask(comp, DSC_FORCE_COMMIT,
		DISP_REG_DSC_SHADOW, DSC_FORCE_COMMIT, handle);
	mtk_ddp_write_mask(comp, DSC_FORCE_COMMIT,
		DISP_REG_DSC_SHADOW + DISP_REG_DSC1_OFFSET, DSC_FORCE_COMMIT, handle);

	if (dsc->enable) {
		mtk_ddp_write_mask(comp, DSC_EN, DISP_REG_DSC_CON,
				DSC_EN, handle);

		if (comp->mtk_crtc->is_dual_pipe &&
			!comp->mtk_crtc->panel_ext->params->dsc_params.dual_dsc_enable &&
			comp->mtk_crtc->panel_ext->params->output_mode
				== MTK_PANEL_DUAL_PORT) {
			mtk_ddp_write_mask(comp, DSC_EN, DISP_REG_DSC_CON + DISP_REG_DSC1_OFFSET,
					DSC_EN, handle);
		}
	}
	DDPINFO("%s, dsc_start:0x%x\n",
		mtk_dump_comp_str(comp), readl(baddr + DISP_REG_DSC_CON));
}

static void mtk_dsc_stop(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle)
{
	void __iomem *baddr = comp->regs;

	mtk_ddp_write_mask(comp, 0x0, DISP_REG_DSC_CON, DSC_EN, handle);

	if (comp->mtk_crtc && comp->mtk_crtc->panel_ext &&
		comp->mtk_crtc->is_dual_pipe &&
		!comp->mtk_crtc->panel_ext->params->dsc_params.dual_dsc_enable &&
		comp->mtk_crtc->panel_ext->params->output_mode
				== MTK_PANEL_DUAL_PORT)
		mtk_ddp_write_mask(comp, 0x0, DISP_REG_DSC_CON +
				   DISP_REG_DSC1_OFFSET, DSC_EN, handle);

	DDPINFO("%s, dsc_stop:0x%x\n",
		mtk_dump_comp_str(comp), readl(baddr + DISP_REG_DSC_CON));
}

static void mtk_dsc_prepare(struct mtk_ddp_comp *comp)
{
	struct mtk_disp_dsc *dsc = comp_to_dsc(comp);

	mtk_ddp_comp_clk_prepare(comp);

	/* Bypass shadow register and read shadow register */
	if (dsc->data->need_bypass_shadow) {
		mtk_ddp_write_mask_cpu(comp, DSC_BYPASS_SHADOW,
			DISP_REG_SHADOW_CTRL(dsc), DSC_BYPASS_SHADOW);
		mtk_ddp_write_mask_cpu(comp, DSC_BYPASS_SHADOW,
			DISP_REG_SHADOW_CTRL(dsc) + DISP_REG_DSC1_OFFSET, DSC_BYPASS_SHADOW);
	}
}

static void mtk_dsc_unprepare(struct mtk_ddp_comp *comp)
{
	mtk_ddp_comp_clk_unprepare(comp);
}

struct mtk_panel_dsc_params *mtk_dsc_default_setting(void)
{
	u8 dsc_cap[16];
	static struct mtk_panel_dsc_params dsc_params = {
		.enable = 1,
		.ver = 2,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 0x12,//flatness_det_thr, 8bit
		.rct_on = 1,//default
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 13, //9,//11 for 10bit
		.bp_enable = 1,//align vend
		.bit_per_pixel = 128,//16*bpp
		.pic_height = 2160,
		.pic_width = 3840, /*for dp port 4k scenario*/
		.slice_height = 8,
		.slice_width = 1920,// frame_width/slice mode
		.chunk_size = 1920,
		.xmit_delay = 512, //410,
		.dec_delay = 1216, //526,
		.scale_value = 32,
		.increment_interval = 286, //488,
		.decrement_interval = 26, //7,
		.line_bpg_offset = 12, //12,
		.nfl_bpg_offset = 3511, //1294,
		.slice_bpg_offset = 916, //1302,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	};

	mtk_dp_get_dsc_capability(dsc_cap);
	dsc_params.bp_enable = dsc_cap[6];
	//dsc_params.ver = dsc_cap[1];

	return &dsc_params;
}
EXPORT_SYMBOL(mtk_dsc_default_setting);

//extern void mtk_dp_dsc_pps_send(u8 *PPS);
u8 PPS[128] = {
	0x12, 0x00, 0x00, 0x8d, 0x30, 0x80, 0x08, 0x70,
	0x0f, 0x00, 0x00, 0x08, 0x07, 0x80, 0x07, 0x80,
	0x02, 0x00, 0x04, 0xc0, 0x00, 0x20, 0x01, 0x1e,
	0x00, 0x1a, 0x00, 0x0c, 0x0d, 0xb7, 0x03, 0x94,
	0x18, 0x00, 0x10, 0xf0, 0x03, 0x0c, 0x20, 0x00,
	0x06, 0x0b, 0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b,
	0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa, 0x19, 0xf8,
	0x1a, 0x38, 0x1a, 0x78, 0x22, 0xb6, 0x2a, 0xb6,
	0x2a, 0xf6, 0x2a, 0xf4, 0x43, 0x34, 0x63, 0x74,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void mtk_dsc1_config(struct mtk_ddp_comp *comp,
				 struct mtk_ddp_config *cfg,
				 struct cmdq_pkt *handle)
{
	u32 reg_val, i;
	struct mtk_disp_dsc *dsc = comp_to_dsc(comp);
	unsigned int dsc_con = 0;
	unsigned int pic_group_width, slice_width, slice_height;
	unsigned int enc_slice_width, enc_pic_width;
	unsigned int pic_height_ext_num, slice_group_width;
	unsigned int bit_per_pixel, chrunk_size, pad_num;
	unsigned int init_delay_limit, init_delay_height_min;
	unsigned int init_delay_height;
	struct mtk_panel_dsc_params *dsc_params;
	struct mtk_panel_spr_params *spr_params;

	if (!comp->mtk_crtc || (!comp->mtk_crtc->panel_ext
				&& !comp->mtk_crtc->is_dual_pipe))
		return;
//	mtk_dsc_default_setting();

	dsc_params = &comp->mtk_crtc->panel_ext->params->dsc_params;
	spr_params = &comp->mtk_crtc->panel_ext->params->spr_params;

	if (dsc_params->enable == 1) {
		DDPINFO("%s, w:%d, h:%d, slice_mode:%d,slice(%d,%d),bpp:%d\n",
			mtk_dump_comp_str(comp), cfg->w, cfg->h,
			dsc_params->slice_mode,	dsc_params->slice_width,
			dsc_params->slice_height, dsc_params->bit_per_pixel);

		pic_group_width = (dsc_params->slice_width *
				   (dsc_params->slice_mode + 1) + 2)/3;
		slice_width = dsc_params->slice_width;
		slice_height = dsc_params->slice_height;
		pic_height_ext_num = (cfg->h + slice_height - 1) / slice_height;
		slice_group_width = (slice_width + 2)/3;
		/* 128=1/3, 196=1/2 */
		bit_per_pixel = dsc_params->bit_per_pixel;
		chrunk_size = (slice_width*bit_per_pixel/8/16);

		if (spr_params->enable && spr_params->relay == 0
			&& disp_spr_bypass == 0) {
			reg_val = ((spr_params->relay == 0) ?
				0x1 : 0) << 26;
			switch (spr_params->spr_format_type) {
			case MTK_PANEL_RGBG_BGRG_TYPE:
				reg_val |= 0x00e10d2;
				enc_slice_width = (slice_width + 1) / 2;
				break;
			case MTK_PANEL_BGRG_RGBG_TYPE:
				reg_val |= 0x00d20e1;
				enc_slice_width = (slice_width + 1) / 2;
				break;
			case MTK_PANEL_RGBRGB_BGRBGR_TYPE:
				reg_val |= 0x1924186;
				enc_slice_width = (slice_width * 2 + 2) / 3;
				break;
			case MTK_PANEL_BGRBGR_RGBRGB_TYPE:
				reg_val |= 0x1186924;
				enc_slice_width = (slice_width * 2 + 2) / 3;
				break;
			case MTK_PANEL_RGBRGB_BRGBRG_TYPE:
				reg_val |= 0x1924492;
				enc_slice_width = (slice_width * 2 + 2) / 3;
				break;
			case MTK_PANEL_BRGBRG_RGBRGB_TYPE:
				reg_val |= 0x1492924;
				enc_slice_width = (slice_width * 2 + 2) / 3;
				break;
			default:
				reg_val = 0x0;
				enc_slice_width = slice_width;
				break;
			}
			mtk_ddp_write_relaxed(comp, reg_val,
				DISP_REG_DSC_SPR + DISP_REG_DSC1_OFFSET, handle);
			chrunk_size = (enc_slice_width*bit_per_pixel / 16 + 7)/8;
		} else {
			mtk_ddp_write_relaxed(comp, 0x0,
				DISP_REG_DSC_SPR + DISP_REG_DSC1_OFFSET, handle);
			enc_slice_width = slice_width;
		}
		enc_pic_width = enc_slice_width * (dsc_params->slice_mode + 1);
		if (spr_params->enable && spr_params->relay == 0
			&& disp_spr_bypass == 0) {
			pic_group_width = (enc_pic_width + 2) / 3;
			slice_group_width = (enc_slice_width + 2) / 3;
		}
		if (dsc->data->need_obuf_sw && enc_pic_width < 1440) {
			mtk_ddp_write_relaxed(comp, 0x800002d9,
				DISP_REG_DSC_OBUF + DISP_REG_DSC1_OFFSET,
				handle);
		}
		pad_num = (chrunk_size * (dsc_params->slice_mode + 1) + 2)/3*3
			- chrunk_size * (dsc_params->slice_mode + 1);
		mtk_ddp_write_relaxed(comp,
			enc_pic_width << 16 | enc_slice_width,
			DISP_REG_DSC_ENC_WIDTH + DISP_REG_DSC1_OFFSET, handle);

		if (dsc_params->ver == 2)
			dsc_con = 0x4080;
		else
			dsc_con = 0x0080;
		dsc_con |= DSC_UFOE_SEL;
		if (comp->mtk_crtc->is_dual_pipe && !dsc_params->dual_dsc_enable) {
			if (comp->mtk_crtc->panel_ext->params->output_mode
				== MTK_PANEL_DUAL_PORT)
				dsc_con |= DSC_DUAL_INOUT;
			else
				dsc_con |= DSC_IN_SRC_SEL;
		}

		mtk_ddp_write_relaxed(comp,
			dsc_con, DISP_REG_DSC_CON + DISP_REG_DSC1_OFFSET, handle);

		mtk_ddp_write_relaxed(comp, (pic_group_width - 1) << 16 |
			dsc_params->slice_width * (dsc_params->slice_mode + 1),
			DISP_REG_DSC_PIC_W + DISP_REG_DSC1_OFFSET, handle);
		mtk_ddp_write_relaxed(comp,
			(pic_height_ext_num * slice_height - 1) << 16 |
			(cfg->h - 1),
			DISP_REG_DSC_PIC_H + DISP_REG_DSC1_OFFSET, handle);

		mtk_ddp_write_relaxed(comp,
			(slice_group_width - 1) << 16 | slice_width,
			DISP_REG_DSC_SLICE_W + DISP_REG_DSC1_OFFSET, handle);

		mtk_ddp_write_relaxed(comp,
			(slice_width % 3) << 30 |
			(pic_height_ext_num - 1) << 16 |
			(slice_height - 1),
			DISP_REG_DSC_SLICE_H + DISP_REG_DSC1_OFFSET, handle);

		mtk_ddp_write_relaxed(comp, chrunk_size,
			DISP_REG_DSC_CHUNK_SIZE + DISP_REG_DSC1_OFFSET, handle);

		mtk_ddp_write_relaxed(comp,	pad_num,
			DISP_REG_DSC_PAD + DISP_REG_DSC1_OFFSET, handle);

		mtk_ddp_write_relaxed(comp,	chrunk_size * slice_height,
			DISP_REG_DSC_BUF_SIZE + DISP_REG_DSC1_OFFSET, handle);

		init_delay_limit =
			((128 + (dsc_params->xmit_delay + 2) / 3) * 3 +
			dsc_params->slice_width-1) / dsc_params->slice_width;
		init_delay_height_min =
			(init_delay_limit > 15) ? 15 : init_delay_limit;
		if (dsc->data->dsi_buffer)
			init_delay_height = 0;
		else if (!mtk_crtc_is_frame_trigger_mode(&comp->mtk_crtc->base))
			init_delay_height = 1;
		else
			init_delay_height = 4;

		reg_val = (!!dsc_params->slice_mode) |
					(!!dsc_params->rgb_swap << 2) |
					(init_delay_height << 8);
		mtk_ddp_write_mask(comp, reg_val,
			DISP_REG_DSC_MODE + DISP_REG_DSC1_OFFSET, 0xFFFF, handle);

		mtk_ddp_write_relaxed(comp,
			(dsc_params->dsc_cfg == 0) ? 0x22 : dsc_params->dsc_cfg,
			DISP_REG_DSC_CFG + DISP_REG_DSC1_OFFSET, handle);

		mtk_ddp_write_mask(comp, DSC_CKSM_CAL_EN,
			DISP_REG_DSC_DBG_CON + DISP_REG_DSC1_OFFSET,
			DSC_CKSM_CAL_EN, handle);

		mtk_ddp_write_mask(comp,
			(((dsc_params->ver & 0xf) == 2) ? 0x40 : 0x20),
			DISP_REG_DSC_SHADOW + DISP_REG_DSC1_OFFSET, 0x60, handle);

		reg_val = dsc_params->dsc_line_buf_depth;
		if (dsc_params->bit_per_channel == 0)
			reg_val |= (0x8 << 4);
		else
			reg_val |= (dsc_params->bit_per_channel << 4);
		if (dsc_params->bit_per_pixel == 0)
			reg_val |= (0x80 << 8);
		else
			reg_val |= (dsc_params->bit_per_pixel << 8);

		reg_val |= (dsc_params->rct_on << 18);
		reg_val |= (dsc_params->bp_enable << 19);

		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS0 + DISP_REG_DSC1_OFFSET, handle);

		if (dsc_params->xmit_delay == 0)
			reg_val = 0x200;
		else
			reg_val = (dsc_params->xmit_delay);
		if (dsc_params->dec_delay == 0)
			reg_val |= (0x268 << 16);
		else
			reg_val |= (dsc_params->dec_delay << 16);
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS1 + DISP_REG_DSC1_OFFSET, handle);

		reg_val = ((dsc_params->scale_value == 0) ?
			0x20 : dsc_params->scale_value);
		reg_val |= ((dsc_params->increment_interval == 0) ?
			0x387 : dsc_params->increment_interval) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS2 + DISP_REG_DSC1_OFFSET, handle);

		reg_val = ((dsc_params->decrement_interval == 0) ?
			0xa : dsc_params->decrement_interval);
		reg_val |= ((dsc_params->line_bpg_offset == 0) ?
			0xc : dsc_params->line_bpg_offset) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS3 + DISP_REG_DSC1_OFFSET, handle);

		reg_val = ((dsc_params->nfl_bpg_offset == 0) ?
			0x319 : dsc_params->nfl_bpg_offset);
		reg_val |= ((dsc_params->slice_bpg_offset == 0) ?
			0x263 : dsc_params->slice_bpg_offset) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS4 + DISP_REG_DSC1_OFFSET, handle);

		reg_val = ((dsc_params->initial_offset == 0) ?
			0x1800 : dsc_params->initial_offset);
		reg_val |= ((dsc_params->final_offset == 0) ?
			0x10f0 : dsc_params->final_offset) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS5 + DISP_REG_DSC1_OFFSET, handle);

		reg_val = ((dsc_params->flatness_minqp == 0) ?
			0x3 : dsc_params->flatness_minqp);
		reg_val |= ((dsc_params->flatness_maxqp == 0) ?
			0xc : dsc_params->flatness_maxqp) << 8;
		reg_val |= ((dsc_params->rc_model_size == 0) ?
			0x2000 : dsc_params->rc_model_size) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS6 + DISP_REG_DSC1_OFFSET, handle);

		DDPINFO("%s, bit_per_channel:%d\n",
			mtk_dump_comp_str(comp), dsc_params->bit_per_channel);
		reg_val = (dsc_params->rc_edge_factor) + (dsc_params->rc_quant_incr_limit0 << 8) +
			(dsc_params->rc_quant_incr_limit1 << 16) +
			(dsc_params->rc_tgt_offset_hi << 24) +
			(dsc_params->rc_tgt_offset_lo << 28);
		mtk_ddp_write_relaxed(comp, reg_val,
			DISP_REG_DSC_PPS7 + DISP_REG_DSC1_OFFSET, handle);
		if (dsc_params->rc_buf_thresh[0]) {
			reg_val = (dsc_params->rc_buf_thresh[3] << 24) |
				(dsc_params->rc_buf_thresh[2] << 16) |
				(dsc_params->rc_buf_thresh[1] << 8) |
				dsc_params->rc_buf_thresh[0];

			mtk_ddp_write_relaxed(comp, reg_val,
						DISP_REG_DSC_PPS8 + DISP_REG_DSC1_OFFSET, handle);

			reg_val = (dsc_params->rc_buf_thresh[7] << 24) |
				(dsc_params->rc_buf_thresh[6] << 16) |
				(dsc_params->rc_buf_thresh[5] << 8) |
				dsc_params->rc_buf_thresh[4];

			mtk_ddp_write_relaxed(comp, reg_val,
						DISP_REG_DSC_PPS9 + DISP_REG_DSC1_OFFSET, handle);

			reg_val = (dsc_params->rc_buf_thresh[11] << 24) |
				(dsc_params->rc_buf_thresh[10] << 16) |
				(dsc_params->rc_buf_thresh[9] << 8) |
				dsc_params->rc_buf_thresh[8];

			mtk_ddp_write_relaxed(comp, reg_val,
				DISP_REG_DSC_PPS10 + DISP_REG_DSC1_OFFSET, handle);

			reg_val = (dsc_params->rc_buf_thresh[13] << 8)
				| dsc_params->rc_buf_thresh[12];

			mtk_ddp_write_relaxed(comp, reg_val,
						DISP_REG_DSC_PPS11 + DISP_REG_DSC1_OFFSET, handle);

			//pps12~pps19
			for (i = 0; i < 7; i++) {
				reg_val = ((dsc_params->rc_range_parameters[i * 2 + 1]
					.range_bpg_offset & 0x3f) << 26) |
					((dsc_params->rc_range_parameters[i * 2 + 1].range_max_qp
						& 0x1f) << 21) |
					((dsc_params->rc_range_parameters[i * 2 + 1].range_min_qp
						& 0x1f) << 16) |
					((dsc_params->rc_range_parameters[i * 2].range_bpg_offset
						& 0x3f) << 10) |
					((dsc_params->rc_range_parameters[i * 2].range_max_qp
						& 0x1f) << 5) |
					(dsc_params->rc_range_parameters[i * 2].range_min_qp
						& 0x1f);

				mtk_ddp_write_relaxed(comp, reg_val,
					i * 4 +  DISP_REG_DSC_PPS12 + DISP_REG_DSC1_OFFSET, handle);
			}

			reg_val = ((dsc_params->rc_range_parameters[14].range_bpg_offset
				& 0x3f) << 10) |
				((dsc_params->rc_range_parameters[14].range_max_qp & 0x1f) << 5) |
				(dsc_params->rc_range_parameters[14].range_min_qp & 0x1f);
			mtk_ddp_write_relaxed(comp,	reg_val,
						DISP_REG_DSC_PPS19 + DISP_REG_DSC1_OFFSET, handle);
		} else if (dsc_params->bit_per_channel == 10) {
			//10bpc_to_8bpp_20_slice_h
			mtk_ddp_write(comp, 0x20001007, DISP_REG_DSC_PPS6 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x330F0F06, DISP_REG_DSC_PPS7 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x382a1c0e, DISP_REG_DSC_PPS8 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x69625446, DISP_REG_DSC_PPS9 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x7b797770, DISP_REG_DSC_PPS10 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x00007e7d, DISP_REG_DSC_PPS11 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x01040900, DISP_REG_DSC_PPS12 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xF9450125, DISP_REG_DSC_PPS13 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xE967F167, DISP_REG_DSC_PPS14 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xE187E167, DISP_REG_DSC_PPS15 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xD9C7E1A7, DISP_REG_DSC_PPS16 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xD1E9D9C9, DISP_REG_DSC_PPS17 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xD20DD1E9, DISP_REG_DSC_PPS18 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x0000D230, DISP_REG_DSC_PPS19 +
				      DISP_REG_DSC1_OFFSET, handle);
		} else {
			/* 8bpc_to_8bpp_20_slice_h */
			mtk_ddp_write(comp, 0x20000c03, DISP_REG_DSC_PPS6 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x330b0b06, DISP_REG_DSC_PPS7 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x382a1c0e, DISP_REG_DSC_PPS8 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x69625446, DISP_REG_DSC_PPS9 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x7b797770, DISP_REG_DSC_PPS10 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x00007e7d, DISP_REG_DSC_PPS11 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x00800880, DISP_REG_DSC_PPS12 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xf8c100a1, DISP_REG_DSC_PPS13 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xe8e3f0e3, DISP_REG_DSC_PPS14 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xe103e0e3, DISP_REG_DSC_PPS15 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd943e123,	DISP_REG_DSC_PPS16 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd185d965,	DISP_REG_DSC_PPS17 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd1a7d1a5,	DISP_REG_DSC_PPS18 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x0000d1ed,	DISP_REG_DSC_PPS19 +
				      DISP_REG_DSC1_OFFSET, handle);
		}

		if (spr_params->enable && spr_params->relay == 0
					&& disp_spr_bypass == 0) {
			/* VESA1.2 needed */
			mtk_ddp_write(comp, 0x0001d822, DISP_REG_DSC_CFG +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x800840, DISP_REG_DSC_PPS12 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd923e103,	DISP_REG_DSC_PPS16 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd125d925,	DISP_REG_DSC_PPS17 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd147d125,	DISP_REG_DSC_PPS18 +
				      DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd16a,	DISP_REG_DSC_PPS19 +
				      DISP_REG_DSC1_OFFSET, handle);
		}

#ifdef IF_ZERO
		if (comp->mtk_crtc->is_dual_pipe) {
			mtk_ddp_write(comp, 0xe8e3f0e3,
				DISP_REG_DSC_PPS14 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xe103e0e3,
				DISP_REG_DSC_PPS15 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd944e123,
				DISP_REG_DSC_PPS16 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd965d945,
				DISP_REG_DSC_PPS17 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd188d165,
				DISP_REG_DSC_PPS18 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x0000d1ac,
				DISP_REG_DSC_PPS19 + DISP_REG_DSC1_OFFSET, handle);

			///mtk_dp_dsc_pps_send(PPS);
		} else {
			mtk_ddp_write(comp, 0xe8e3f0e3,
				DISP_REG_DSC_PPS14 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xe103e0e3,
				DISP_REG_DSC_PPS15 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd943e123,
				DISP_REG_DSC_PPS16 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd185d965,
				DISP_REG_DSC_PPS17 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0xd1a7d1a5,
				DISP_REG_DSC_PPS18 + DISP_REG_DSC1_OFFSET, handle);
			mtk_ddp_write(comp, 0x0000d1ed,
				DISP_REG_DSC_PPS19 + DISP_REG_DSC1_OFFSET, handle);
		}
#endif

		dsc->enable = true;
	} else {
		/*enable dsc relay mode*/
		mtk_ddp_write_mask(comp, DSC_RELAY + DISP_REG_DSC1_OFFSET,
				   DISP_REG_DSC_CON, DSC_RELAY, handle);
		dsc->enable = false;
	}
}

static void mtk_dsc_config(struct mtk_ddp_comp *comp,
				 struct mtk_ddp_config *cfg,
				 struct cmdq_pkt *handle)
{
	u32 reg_val, i;
	struct mtk_disp_dsc *dsc = comp_to_dsc(comp);
	unsigned int dsc_con = 0;
	unsigned int pic_group_width, slice_width, slice_height;
	unsigned int enc_slice_width, enc_pic_width;
	unsigned int pic_height_ext_num, slice_group_width;
	unsigned int bit_per_pixel, chrunk_size, pad_num;
	unsigned int init_delay_limit, init_delay_height_min;
	unsigned int init_delay_height;
	struct mtk_panel_dsc_params *dsc_params;
	struct mtk_panel_spr_params *spr_params;

	if (!comp->mtk_crtc || (!comp->mtk_crtc->panel_ext
				&& !comp->mtk_crtc->is_dual_pipe))
		return;
//	mtk_dsc_default_setting();


	dsc_params = &comp->mtk_crtc->panel_ext->params->dsc_params;
	spr_params = &comp->mtk_crtc->panel_ext->params->spr_params;

	if (dsc_params->enable == 1) {
		DDPINFO("%s, w:%d, h:%d, slice_mode:%d,slice(%d,%d),bpp:%d\n",
			mtk_dump_comp_str(comp), cfg->w, cfg->h,
			dsc_params->slice_mode,	dsc_params->slice_width,
			dsc_params->slice_height, dsc_params->bit_per_pixel);

		pic_group_width = (dsc_params->slice_width *
				  (dsc_params->slice_mode + 1) + 2)/3;
		slice_width = dsc_params->slice_width;
		slice_height = dsc_params->slice_height;
		pic_height_ext_num = (cfg->h + slice_height - 1) / slice_height;
		slice_group_width = (slice_width + 2)/3;
		/* 128=1/3, 196=1/2 */
		bit_per_pixel = dsc_params->bit_per_pixel;
		chrunk_size = (slice_width*bit_per_pixel/8/16);

		if (spr_params->enable && spr_params->relay == 0
			&& disp_spr_bypass == 0) {
			reg_val = ((spr_params->relay == 0) ?
				0x1 : 0) << 26;
			switch (spr_params->spr_format_type) {
			case MTK_PANEL_RGBG_BGRG_TYPE:
				reg_val |= 0x00e10d2;
				enc_slice_width = (slice_width + 1) / 2;
				break;
			case MTK_PANEL_BGRG_RGBG_TYPE:
				reg_val |= 0x00d20e1;
				enc_slice_width = (slice_width + 1) / 2;
				break;
			case MTK_PANEL_RGBRGB_BGRBGR_TYPE:
				reg_val |= 0x1924186;
				enc_slice_width = (slice_width * 2 + 2) / 3;
				break;
			case MTK_PANEL_BGRBGR_RGBRGB_TYPE:
				reg_val |= 0x1186924;
				enc_slice_width = (slice_width * 2 + 2) / 3;
				break;
			case MTK_PANEL_RGBRGB_BRGBRG_TYPE:
				reg_val |= 0x1924492;
				enc_slice_width = (slice_width * 2 + 2) / 3;
				break;
			case MTK_PANEL_BRGBRG_RGBRGB_TYPE:
				reg_val |= 0x1492924;
				enc_slice_width = (slice_width * 2 + 2) / 3;
				break;
			default:
				reg_val = 0x0;
				enc_slice_width = slice_width;
				break;
			}
			mtk_ddp_write_relaxed(comp, reg_val,
				DISP_REG_DSC_SPR, handle);
			chrunk_size = (enc_slice_width*bit_per_pixel / 16 + 7)/8;
		} else {
			mtk_ddp_write_relaxed(comp, 0x0,
				DISP_REG_DSC_SPR, handle);
			enc_slice_width = slice_width;
		}
		enc_pic_width = enc_slice_width * (dsc_params->slice_mode + 1);
		if (spr_params->enable && spr_params->relay == 0
			&& disp_spr_bypass == 0) {
			pic_group_width = (enc_pic_width + 2) / 3;
			slice_group_width = (enc_slice_width + 2) / 3;
		}
		if (dsc->data->need_obuf_sw && enc_pic_width < 1440) {
			mtk_ddp_write_relaxed(comp,
				0x800002d9, DISP_REG_DSC_OBUF, handle);
		}
		pad_num = (chrunk_size * (dsc_params->slice_mode + 1) + 2) / 3 * 3
			- chrunk_size * (dsc_params->slice_mode + 1);
		mtk_ddp_write_relaxed(comp,
			enc_pic_width << 16 | enc_slice_width,
			DISP_REG_DSC_ENC_WIDTH, handle);

		if (dsc_params->ver == 2)
			dsc_con = 0x4080;
		else
			dsc_con = 0x0080;
		dsc_con |= DSC_UFOE_SEL;
		if (comp->mtk_crtc->is_dual_pipe && !dsc_params->dual_dsc_enable) {
			if (comp->mtk_crtc->panel_ext->params->output_mode
				== MTK_PANEL_DUAL_PORT)
				dsc_con |= DSC_DUAL_INOUT;
			else
				dsc_con |= DSC_IN_SRC_SEL;
		}
		if (comp->mtk_crtc->is_dsc_output_swap)
			dsc_con |= DSC_OUTPUT_SWAP;

		mtk_ddp_write_relaxed(comp,
			dsc_con, DISP_REG_DSC_CON, handle);

		mtk_ddp_write_relaxed(comp, (pic_group_width - 1) << 16 |
			dsc_params->slice_width * (dsc_params->slice_mode + 1),
			DISP_REG_DSC_PIC_W, handle);
		mtk_ddp_write_relaxed(comp,
			(pic_height_ext_num * slice_height - 1) << 16 |
			(cfg->h - 1),
			DISP_REG_DSC_PIC_H, handle);

		mtk_ddp_write_relaxed(comp,
			(slice_group_width - 1) << 16 | slice_width,
			DISP_REG_DSC_SLICE_W, handle);

		mtk_ddp_write_relaxed(comp,
			(slice_group_width - 1) << 16 | slice_width,
			DISP_REG_DSC_SLICE_W, handle);

		mtk_ddp_write_relaxed(comp,
			(slice_width % 3) << 30 |
			(pic_height_ext_num - 1) << 16 |
			(slice_height - 1),
			DISP_REG_DSC_SLICE_H, handle);

		mtk_ddp_write_relaxed(comp, chrunk_size,
			DISP_REG_DSC_CHUNK_SIZE, handle);

		mtk_ddp_write_relaxed(comp,	pad_num,
			DISP_REG_DSC_PAD, handle);

		mtk_ddp_write_relaxed(comp,	chrunk_size * slice_height,
			DISP_REG_DSC_BUF_SIZE, handle);

		init_delay_limit =
			((128 + (dsc_params->xmit_delay + 2) / 3) * 3 +
			dsc_params->slice_width-1) / dsc_params->slice_width;
		init_delay_height_min =
			(init_delay_limit > 15) ? 15 : init_delay_limit;
		if (dsc->data->dsi_buffer)
			init_delay_height = 0;
		else if (!mtk_crtc_is_frame_trigger_mode(&comp->mtk_crtc->base))
			init_delay_height = 1;
		else
			init_delay_height = 4;

		reg_val = (!!dsc_params->slice_mode) |
					(!!dsc_params->rgb_swap << 2) |
					(init_delay_height << 8);
		mtk_ddp_write_mask(comp, reg_val,
					DISP_REG_DSC_MODE, 0xFFFF, handle);

		mtk_ddp_write_relaxed(comp,
			(dsc_params->dsc_cfg == 0) ? 0x22 : dsc_params->dsc_cfg,
			DISP_REG_DSC_CFG, handle);

		mtk_ddp_write_mask(comp, DSC_CKSM_CAL_EN,
					DISP_REG_DSC_DBG_CON, DSC_CKSM_CAL_EN,
					handle);

		mtk_ddp_write_mask(comp,
			(((dsc_params->ver & 0xf) == 2) ? 0x40 : 0x20),
			DISP_REG_DSC_SHADOW, 0x60, handle);

		reg_val = dsc_params->dsc_line_buf_depth;
		if (dsc_params->bit_per_channel == 0)
			reg_val |= (0x8 << 4);
		else
			reg_val |= (dsc_params->bit_per_channel << 4);
		if (dsc_params->bit_per_pixel == 0)
			reg_val |= (0x80 << 8);
		else
			reg_val |= (dsc_params->bit_per_pixel << 8);

		reg_val |= (dsc_params->rct_on << 18);
		reg_val |= (dsc_params->bp_enable << 19);

		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS0, handle);

		if (dsc_params->xmit_delay == 0)
			reg_val = 0x200;
		else
			reg_val = (dsc_params->xmit_delay);
		if (dsc_params->dec_delay == 0)
			reg_val |= (0x268 << 16);
		else
			reg_val |= (dsc_params->dec_delay << 16);
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS1, handle);

		reg_val = ((dsc_params->scale_value == 0) ?
			0x20 : dsc_params->scale_value);
		reg_val |= ((dsc_params->increment_interval == 0) ?
			0x387 : dsc_params->increment_interval) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS2, handle);

		reg_val = ((dsc_params->decrement_interval == 0) ?
			0xa : dsc_params->decrement_interval);
		reg_val |= ((dsc_params->line_bpg_offset == 0) ?
			0xc : dsc_params->line_bpg_offset) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS3, handle);

		reg_val = ((dsc_params->nfl_bpg_offset == 0) ?
			0x319 : dsc_params->nfl_bpg_offset);
		reg_val |= ((dsc_params->slice_bpg_offset == 0) ?
			0x263 : dsc_params->slice_bpg_offset) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS4, handle);

		reg_val = ((dsc_params->initial_offset == 0) ?
			0x1800 : dsc_params->initial_offset);
		reg_val |= ((dsc_params->final_offset == 0) ?
			0x10f0 : dsc_params->final_offset) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS5, handle);

		reg_val = ((dsc_params->flatness_minqp == 0) ?
			0x3 : dsc_params->flatness_minqp);
		reg_val |= ((dsc_params->flatness_maxqp == 0) ?
			0xc : dsc_params->flatness_maxqp) << 8;
		reg_val |= ((dsc_params->rc_model_size == 0) ?
			0x2000 : dsc_params->rc_model_size) << 16;
		mtk_ddp_write_relaxed(comp,	reg_val,
			DISP_REG_DSC_PPS6, handle);

		DDPINFO("%s, bit_per_channel:%d\n",
			mtk_dump_comp_str(comp), dsc_params->bit_per_channel);
		reg_val = (dsc_params->rc_edge_factor) + (dsc_params->rc_quant_incr_limit0 << 8) +
			(dsc_params->rc_quant_incr_limit1 << 16) +
			(dsc_params->rc_tgt_offset_hi << 24) +
			(dsc_params->rc_tgt_offset_lo << 28);
		mtk_ddp_write_relaxed(comp, reg_val, DISP_REG_DSC_PPS7, handle);

		if (dsc_params->rc_buf_thresh[0]) {
			reg_val = (dsc_params->rc_buf_thresh[3] << 24) |
				(dsc_params->rc_buf_thresh[2] << 16) |
				(dsc_params->rc_buf_thresh[1] << 8) |
				dsc_params->rc_buf_thresh[0];

			mtk_ddp_write_relaxed(comp, reg_val, DISP_REG_DSC_PPS8, handle);

			reg_val = (dsc_params->rc_buf_thresh[7] << 24) |
				(dsc_params->rc_buf_thresh[6] << 16) |
				(dsc_params->rc_buf_thresh[5] << 8) |
				dsc_params->rc_buf_thresh[4];

			mtk_ddp_write_relaxed(comp, reg_val,
						DISP_REG_DSC_PPS9, handle);

			reg_val = (dsc_params->rc_buf_thresh[11] << 24) |
				(dsc_params->rc_buf_thresh[10] << 16) |
				(dsc_params->rc_buf_thresh[9] << 8) |
				dsc_params->rc_buf_thresh[8];

			mtk_ddp_write_relaxed(comp, reg_val, DISP_REG_DSC_PPS10, handle);

			reg_val = (dsc_params->rc_buf_thresh[13] << 8)
				| dsc_params->rc_buf_thresh[12];

			mtk_ddp_write_relaxed(comp, reg_val, DISP_REG_DSC_PPS11, handle);

			//pps12~pps19
			for (i = 0; i < 7; i++) {
				reg_val = ((dsc_params->rc_range_parameters[i * 2 + 1]
					.range_bpg_offset & 0x3f) << 26) |
					((dsc_params->rc_range_parameters[i * 2 + 1].range_max_qp
					& 0x1f) << 21) |
					((dsc_params->rc_range_parameters[i * 2 + 1].range_min_qp
					& 0x1f) << 16) |
					((dsc_params->rc_range_parameters[i * 2].range_bpg_offset
					& 0x3f) << 10) |
					((dsc_params->rc_range_parameters[i * 2].range_max_qp
					& 0x1f) << 5) |
					(dsc_params->rc_range_parameters[i * 2].range_min_qp
					& 0x1f);

				mtk_ddp_write_relaxed(comp, reg_val,
					i * 4 + DISP_REG_DSC_PPS12, handle);
			}

			reg_val = ((dsc_params->rc_range_parameters[14].range_bpg_offset
				& 0x3f) << 10) |
				((dsc_params->rc_range_parameters[14].range_max_qp & 0x1f) << 5) |
				(dsc_params->rc_range_parameters[14].range_min_qp & 0x1f);
			mtk_ddp_write_relaxed(comp, reg_val, DISP_REG_DSC_PPS19, handle);
		} else if (dsc_params->bit_per_channel == 10) {
			if (dsc_params->bit_per_pixel == 160) {
/* #ifdef OPLUS_FEATURE_DISPLAY */
				if (40 == dsc_params->dsc_cfg) {
					//10bpc_to_10bpp_12_slice_h DSC1.1 version and DSC1.2 rc_10bpc_10bpp.cfg dsc_cfg = 40
					mtk_ddp_write(comp, 0x000ca0ab, DISP_REG_DSC_PPS0, handle);
					mtk_ddp_write(comp, 0x01d8019a, DISP_REG_DSC_PPS1, handle);
					mtk_ddp_write(comp, 0x01030019, DISP_REG_DSC_PPS2, handle);
					mtk_ddp_write(comp, 0x000c000a, DISP_REG_DSC_PPS3, handle);
					mtk_ddp_write(comp, 0x0a5f08bb, DISP_REG_DSC_PPS4, handle);
					mtk_ddp_write(comp, 0x10ec1600, DISP_REG_DSC_PPS5, handle);
					mtk_ddp_write(comp, 0x20001007, DISP_REG_DSC_PPS6, handle);
					mtk_ddp_write(comp, 0x330f0f06, DISP_REG_DSC_PPS7, handle);
					mtk_ddp_write(comp, 0x382a1c0e, DISP_REG_DSC_PPS8, handle);
					mtk_ddp_write(comp, 0x69625446, DISP_REG_DSC_PPS9, handle);
					mtk_ddp_write(comp, 0x7b797770, DISP_REG_DSC_PPS10, handle);
					mtk_ddp_write(comp, 0x00007e7d, DISP_REG_DSC_PPS11, handle);
					mtk_ddp_write(comp, 0x010408e0, DISP_REG_DSC_PPS12, handle);
					mtk_ddp_write(comp, 0xf9460125, DISP_REG_DSC_PPS13, handle);
					mtk_ddp_write(comp, 0xe967f167, DISP_REG_DSC_PPS14, handle);
					mtk_ddp_write(comp, 0xe187e167, DISP_REG_DSC_PPS15, handle);
					mtk_ddp_write(comp, 0xd9a7e1a7, DISP_REG_DSC_PPS16, handle);
					mtk_ddp_write(comp, 0xd9c9d9c9, DISP_REG_DSC_PPS17, handle);
					mtk_ddp_write(comp, 0xd1ebd1e9, DISP_REG_DSC_PPS18, handle);
					mtk_ddp_write(comp, 0x0000d20f, DISP_REG_DSC_PPS19, handle);
				} else if (34 == dsc_params->dsc_cfg) {
					//10bpc_to_10bpp_12_slice_h DSC1.1 version and DSC1.1 rc_10bpc_10bpp.cfg dsc_cfg = 34
					mtk_ddp_write(comp, 0x20001007, DISP_REG_DSC_PPS6, handle);
					mtk_ddp_write(comp, 0x330F0F06, DISP_REG_DSC_PPS7, handle);
					mtk_ddp_write(comp, 0x382a1c0e, DISP_REG_DSC_PPS8, handle);
					mtk_ddp_write(comp, 0x69625446, DISP_REG_DSC_PPS9, handle);
					mtk_ddp_write(comp, 0x7b797770, DISP_REG_DSC_PPS10, handle);
					mtk_ddp_write(comp, 0x00007e7d, DISP_REG_DSC_PPS11, handle);
					mtk_ddp_write(comp, 0x010408E0, DISP_REG_DSC_PPS12, handle);
					mtk_ddp_write(comp, 0xF9460125, DISP_REG_DSC_PPS13, handle);
					mtk_ddp_write(comp, 0xE967F167, DISP_REG_DSC_PPS14, handle);
					mtk_ddp_write(comp, 0xE187E167, DISP_REG_DSC_PPS15, handle);
					mtk_ddp_write(comp, 0xD9A7E1A7, DISP_REG_DSC_PPS16, handle);
					mtk_ddp_write(comp, 0xD1E9D9C9, DISP_REG_DSC_PPS17, handle);
					mtk_ddp_write(comp, 0xD22BD209, DISP_REG_DSC_PPS18, handle);
					mtk_ddp_write(comp, 0x0000D271, DISP_REG_DSC_PPS19, handle);
				}
			} else {
				if (dsc_params->dsc_cfg_change == 1) {
					//10bpc_to_8bpp_20_slice_h
					mtk_ddp_write(comp, 0x20001007, DISP_REG_DSC_PPS6, handle);
					mtk_ddp_write(comp, 0x330F0F06, DISP_REG_DSC_PPS7, handle);
					mtk_ddp_write(comp, 0x382a1c0e, DISP_REG_DSC_PPS8, handle);
					mtk_ddp_write(comp, 0x69625446, DISP_REG_DSC_PPS9, handle);
					mtk_ddp_write(comp, 0x7b797770, DISP_REG_DSC_PPS10, handle);
					mtk_ddp_write(comp, 0x00007e7d, DISP_REG_DSC_PPS11, handle);
					mtk_ddp_write(comp, 0x01040900, DISP_REG_DSC_PPS12, handle);
					mtk_ddp_write(comp, 0xF9450125, DISP_REG_DSC_PPS13, handle);
					mtk_ddp_write(comp, 0xE967F167, DISP_REG_DSC_PPS14, handle);
					mtk_ddp_write(comp, 0xE187E167, DISP_REG_DSC_PPS15, handle);
					mtk_ddp_write(comp, 0xD9C8E1A7, DISP_REG_DSC_PPS16, handle);
					mtk_ddp_write(comp, 0xD9E9D9C9, DISP_REG_DSC_PPS17, handle);
					mtk_ddp_write(comp, 0xD20CD1E9, DISP_REG_DSC_PPS18, handle);
					mtk_ddp_write(comp, 0x0000D230, DISP_REG_DSC_PPS19, handle);
				} else {
					//10bpc_to_8bpp_20_slice_h
					mtk_ddp_write(comp, 0x20001007, DISP_REG_DSC_PPS6, handle);
					mtk_ddp_write(comp, 0x330F0F06, DISP_REG_DSC_PPS7, handle);
					mtk_ddp_write(comp, 0x382a1c0e, DISP_REG_DSC_PPS8, handle);
					mtk_ddp_write(comp, 0x69625446, DISP_REG_DSC_PPS9, handle);
					mtk_ddp_write(comp, 0x7b797770, DISP_REG_DSC_PPS10, handle);
					mtk_ddp_write(comp, 0x00007e7d, DISP_REG_DSC_PPS11, handle);
					mtk_ddp_write(comp, 0x01040900, DISP_REG_DSC_PPS12, handle);
					mtk_ddp_write(comp, 0xF9450125, DISP_REG_DSC_PPS13, handle);
					mtk_ddp_write(comp, 0xE967F167, DISP_REG_DSC_PPS14, handle);
					mtk_ddp_write(comp, 0xE187E167, DISP_REG_DSC_PPS15, handle);
					mtk_ddp_write(comp, 0xD9C7E1A7, DISP_REG_DSC_PPS16, handle);
					mtk_ddp_write(comp, 0xD1E9D9C9, DISP_REG_DSC_PPS17, handle);
					mtk_ddp_write(comp, 0xD20DD1E9, DISP_REG_DSC_PPS18, handle);
					mtk_ddp_write(comp, 0x0000D230, DISP_REG_DSC_PPS19, handle);
				}
			}
/* endif */
		} else if (4 == dsc_params->dsc_scr_version) {
			//8bpc_to_8bpp_20_slice_h
			mtk_ddp_write(comp, 0x20000c03, DISP_REG_DSC_PPS6, handle);
			mtk_ddp_write(comp, 0x330b0b06, DISP_REG_DSC_PPS7, handle);
			mtk_ddp_write(comp, 0x382a1c0e, DISP_REG_DSC_PPS8, handle);
			mtk_ddp_write(comp, 0x69625446, DISP_REG_DSC_PPS9, handle);
			mtk_ddp_write(comp, 0x7b797770, DISP_REG_DSC_PPS10, handle);
			mtk_ddp_write(comp, 0x00007e7d, DISP_REG_DSC_PPS11, handle);
			mtk_ddp_write(comp, 0x00800880, DISP_REG_DSC_PPS12, handle);
			mtk_ddp_write(comp, 0xf8c100a1, DISP_REG_DSC_PPS13, handle);
			mtk_ddp_write(comp, 0xe8e3f0e3, DISP_REG_DSC_PPS14, handle);
			mtk_ddp_write(comp, 0xe103e0e3, DISP_REG_DSC_PPS15, handle);
			mtk_ddp_write(comp, 0xd943e123,	DISP_REG_DSC_PPS16, handle);
			mtk_ddp_write(comp, 0xd165d945,	DISP_REG_DSC_PPS17, handle);
			mtk_ddp_write(comp, 0xd189d165,	DISP_REG_DSC_PPS18, handle);
			mtk_ddp_write(comp, 0x0000d1ac,	DISP_REG_DSC_PPS19, handle);
		} else if (5 == dsc_params->dsc_scr_version) {
			//8bpc_to_8bpp_20_slice_h
			mtk_ddp_write(comp, 0x20000c03, DISP_REG_DSC_PPS6, handle);
			mtk_ddp_write(comp, 0x330b0b06, DISP_REG_DSC_PPS7, handle);
			mtk_ddp_write(comp, 0x382a1c0e, DISP_REG_DSC_PPS8, handle);
			mtk_ddp_write(comp, 0x69625446, DISP_REG_DSC_PPS9, handle);
			mtk_ddp_write(comp, 0x7b797770, DISP_REG_DSC_PPS10, handle);
			mtk_ddp_write(comp, 0x00007e7d, DISP_REG_DSC_PPS11, handle);
			mtk_ddp_write(comp, 0x00800880, DISP_REG_DSC_PPS12, handle);
			mtk_ddp_write(comp, 0xf8c100a1, DISP_REG_DSC_PPS13, handle);
			mtk_ddp_write(comp, 0xe8e3f0e3, DISP_REG_DSC_PPS14, handle);
			mtk_ddp_write(comp, 0xe103e0e3, DISP_REG_DSC_PPS15, handle);
			mtk_ddp_write(comp, 0xd944e123,	DISP_REG_DSC_PPS16, handle);
			mtk_ddp_write(comp, 0xd965d945,	DISP_REG_DSC_PPS17, handle);
			mtk_ddp_write(comp, 0xd188d165,	DISP_REG_DSC_PPS18, handle);
			mtk_ddp_write(comp, 0x0000d1ac,	DISP_REG_DSC_PPS19, handle);
		} else {
			//8bpc_to_8bpp_20_slice_h
			mtk_ddp_write(comp, 0x20000c03, DISP_REG_DSC_PPS6, handle);
			mtk_ddp_write(comp, 0x330b0b06, DISP_REG_DSC_PPS7, handle);
			mtk_ddp_write(comp, 0x382a1c0e, DISP_REG_DSC_PPS8, handle);
			mtk_ddp_write(comp, 0x69625446, DISP_REG_DSC_PPS9, handle);
			mtk_ddp_write(comp, 0x7b797770, DISP_REG_DSC_PPS10, handle);
			mtk_ddp_write(comp, 0x00007e7d, DISP_REG_DSC_PPS11, handle);
			mtk_ddp_write(comp, 0x00800880, DISP_REG_DSC_PPS12, handle);
			mtk_ddp_write(comp, 0xf8c100a1, DISP_REG_DSC_PPS13, handle);
			mtk_ddp_write(comp, 0xe8e3f0e3, DISP_REG_DSC_PPS14, handle);
			mtk_ddp_write(comp, 0xe103e0e3, DISP_REG_DSC_PPS15, handle);
			mtk_ddp_write(comp, 0xd943e123,	DISP_REG_DSC_PPS16, handle);
			mtk_ddp_write(comp, 0xd185d965,	DISP_REG_DSC_PPS17, handle);
			mtk_ddp_write(comp, 0xd1a7d1a5,	DISP_REG_DSC_PPS18, handle);
			mtk_ddp_write(comp, 0x0000d1ed,	DISP_REG_DSC_PPS19, handle);
		}

		if (spr_params->enable && spr_params->relay == 0
					&& disp_spr_bypass == 0) {
			mtk_ddp_write(comp, 0x0001d822, DISP_REG_DSC_CFG, handle);//VESA1.2 needed
			//mtk_ddp_write(comp, 0x00014001, DISP_REG_DSC_CON, handle);
			mtk_ddp_write(comp, 0x800840, DISP_REG_DSC_PPS12, handle);
			mtk_ddp_write(comp, 0xd923e103,	DISP_REG_DSC_PPS16, handle);
			mtk_ddp_write(comp, 0xd125d925,	DISP_REG_DSC_PPS17, handle);
			mtk_ddp_write(comp, 0xd147d125,	DISP_REG_DSC_PPS18, handle);
			mtk_ddp_write(comp, 0xd16a,	DISP_REG_DSC_PPS19, handle);
		}

#ifdef IF_ZERO
		if (comp->mtk_crtc->is_dual_pipe) {
			mtk_ddp_write(comp, 0xe8e3f0e3,
				DISP_REG_DSC_PPS14, handle);
			mtk_ddp_write(comp, 0xe103e0e3,
				DISP_REG_DSC_PPS15, handle);
			mtk_ddp_write(comp, 0xd944e123,
				DISP_REG_DSC_PPS16, handle);
			mtk_ddp_write(comp, 0xd965d945,
				DISP_REG_DSC_PPS17, handle);
			mtk_ddp_write(comp, 0xd188d165,
				DISP_REG_DSC_PPS18, handle);
			mtk_ddp_write(comp, 0x0000d1ac,
				DISP_REG_DSC_PPS19, handle);

			///mtk_dp_dsc_pps_send(PPS);
		} else {
			mtk_ddp_write(comp, 0xe8e3f0e3,
				DISP_REG_DSC_PPS14, handle);
			mtk_ddp_write(comp, 0xe103e0e3,
				DISP_REG_DSC_PPS15, handle);
			mtk_ddp_write(comp, 0xd943e123,
				DISP_REG_DSC_PPS16, handle);
			mtk_ddp_write(comp, 0xd185d965,
				DISP_REG_DSC_PPS17, handle);
			mtk_ddp_write(comp, 0xd1a7d1a5,
				DISP_REG_DSC_PPS18, handle);
			mtk_ddp_write(comp, 0x0000d1ed,
				DISP_REG_DSC_PPS19, handle);
			}
#endif
		dsc->enable = true;
	} else {
		/*enable dsc relay mode*/
		mtk_ddp_write_mask(comp, DSC_RELAY, DISP_REG_DSC_CON,
				DSC_RELAY, handle);
		mtk_ddp_write_relaxed(comp, cfg->w, DISP_REG_DSC_PIC_W, handle);
		mtk_ddp_write_relaxed(comp, (cfg->h - 1), DISP_REG_DSC_PIC_H, handle);
		mtk_ddp_write_relaxed(comp, cfg->w, DISP_REG_DSC_SLICE_W, handle);
		mtk_ddp_write_relaxed(comp, (cfg->h - 1), DISP_REG_DSC_SLICE_H, handle);

		if (comp->mtk_crtc->is_dsc_output_swap)
			mtk_ddp_write_mask(comp, DSC_OUTPUT_SWAP, DISP_REG_DSC_CON,
				DSC_OUTPUT_SWAP, handle);
		dsc->enable = true;    /*need to enable dsc in relay mode*/
	}

	if (comp->mtk_crtc->is_dual_pipe &&
	    comp->mtk_crtc->panel_ext->params->output_mode == MTK_PANEL_DUAL_PORT &&
	    !dsc_params->dual_dsc_enable)
		mtk_dsc1_config(comp, cfg, handle);
}

void mtk_dsc_dump(struct mtk_ddp_comp *comp)
{
	void __iomem *baddr = comp->regs;
	int i, id = 0, offset = 0;

DUMP:
	DDPDUMP("== %s REGS:0x%x ==\n", mtk_dump_comp_str(comp), comp->regs_pa + offset);
	DDPDUMP("(0x%03x)DSC_START=0x%x\n", offset + DISP_REG_DSC_CON,
		readl(baddr + DISP_REG_DSC_CON));
	DDPDUMP("(0x%03x)DSC_SLICE_WIDTH=0x%x\n", offset + DISP_REG_DSC_SLICE_W,
		readl(baddr + DISP_REG_DSC_SLICE_W));
	DDPDUMP("(0x%03x)DSC_SLICE_HIGHT=0x%x\n", offset + DISP_REG_DSC_SLICE_H,
		readl(baddr + DISP_REG_DSC_SLICE_H));
	DDPDUMP("(0x%03x)DSC_WIDTH=0x%x\n", offset + DISP_REG_DSC_PIC_W,
		readl(baddr + DISP_REG_DSC_PIC_W));
	DDPDUMP("(0x%03x)DSC_HEIGHT=0x%x\n", offset + DISP_REG_DSC_PIC_H,
		readl(baddr + DISP_REG_DSC_PIC_H));
	DDPDUMP("(0x%03x)DSC_SHADOW=0x%x\n", offset + DISP_REG_DSC_SHADOW,
		readl(baddr + DISP_REG_DSC_SHADOW));
	DDPDUMP("-- Start dump dsc registers --\n");
	for (i = 0; i < 0x200; i += 16) {
		DDPDUMP("DSC+0x%03x: 0x%x 0x%x 0x%x 0x%x\n", offset + i, readl(baddr + i),
			 readl(baddr + i + 0x4), readl(baddr + i + 0x8),
			 readl(baddr + i + 0xc));
	}

	if (id > 0)
		return;

	if (comp->mtk_crtc && comp->mtk_crtc->is_dual_pipe &&
	    comp->mtk_crtc->panel_ext->params->output_mode == MTK_PANEL_DUAL_PORT) {
		id = 1;
		offset = DISP_REG_DSC1_OFFSET;
		baddr += offset;
		goto DUMP;
	}
}

int mtk_dsc_analysis(struct mtk_ddp_comp *comp)
{
	void __iomem *baddr = comp->regs;

	DDPDUMP("== %s ANALYSIS:0x%p ==\n", mtk_dump_comp_str(comp), comp->regs);
	DDPDUMP("en=%d, pic_w=%d, pic_h=%d, slice_w=%d, bypass=%d\n",
		 DISP_REG_GET_FIELD(CON_FLD_DSC_EN,
				baddr + DISP_REG_DSC_CON),
		 DISP_REG_GET_FIELD(CFG_FLD_PIC_WIDTH,
				baddr + DISP_REG_DSC_PIC_W),
		 DISP_REG_GET_FIELD(CFG_FLD_PIC_HEIGHT_M1,
				baddr + DISP_REG_DSC_PIC_H),
		 DISP_REG_GET_FIELD(CFG_FLD_SLICE_WIDTH,
				baddr + DISP_REG_DSC_SLICE_W),
		 DISP_REG_GET_FIELD(CON_FLD_DISP_DSC_BYPASS,
				baddr + DISP_REG_DSC_CON));

	return 0;
}

static const struct mtk_ddp_comp_funcs mtk_disp_dsc_funcs = {
	.config = mtk_dsc_config,
	.start = mtk_dsc_start,
	.stop = mtk_dsc_stop,
	.prepare = mtk_dsc_prepare,
	.unprepare = mtk_dsc_unprepare,
};

static int mtk_disp_dsc_bind(struct device *dev, struct device *master,
				  void *data)
{
	struct mtk_disp_dsc *priv = dev_get_drvdata(dev);
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

static void mtk_disp_dsc_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct mtk_disp_dsc *priv = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;

	mtk_ddp_comp_unregister(drm_dev, &priv->ddp_comp);
}

static const struct component_ops mtk_disp_dsc_component_ops = {
	.bind = mtk_disp_dsc_bind,
	.unbind = mtk_disp_dsc_unbind,
};

static int mtk_disp_dsc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_disp_dsc *priv;
	enum mtk_ddp_comp_id comp_id;
	int irq;
	int ret;

	DDPMSG("%s+\n", __func__);
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	comp_id = mtk_ddp_comp_get_id(dev->of_node, MTK_DISP_DSC);
	if ((int)comp_id < 0) {
		dev_err(dev, "Failed to identify by alias: %d\n", comp_id);
		return comp_id;
	}

	ret = mtk_ddp_comp_init(dev, dev->of_node, &priv->ddp_comp, comp_id,
				&mtk_disp_dsc_funcs);
	if (ret) {
		dev_err(dev, "Failed to initialize component: %d\n", ret);
		return ret;
	}

	priv->data = of_device_get_match_data(dev);

	platform_set_drvdata(pdev, priv);

	ret = devm_request_irq(dev, irq, mtk_dsc_irq_handler,
			       IRQF_TRIGGER_NONE | IRQF_SHARED, dev_name(dev),
			       priv);
	if (ret < 0) {
		DDPAEE("%s:%d, failed to request irq:%d ret:%d comp_id:%d\n",
				__func__, __LINE__,
				irq, ret, comp_id);
		return ret;
	}

	mtk_ddp_comp_pm_enable(&priv->ddp_comp);

	ret = component_add(dev, &mtk_disp_dsc_component_ops);
	if (ret != 0) {
		dev_err(dev, "Failed to add component: %d\n", ret);
		mtk_ddp_comp_pm_disable(&priv->ddp_comp);
	}

	DDPMSG("%s-\n", __func__);
	return ret;
}

static int mtk_disp_dsc_remove(struct platform_device *pdev)
{
	struct mtk_disp_dsc *priv = dev_get_drvdata(&pdev->dev);

	component_del(&pdev->dev, &mtk_disp_dsc_component_ops);
	mtk_ddp_comp_pm_disable(&priv->ddp_comp);

	return 0;
}

static const struct mtk_disp_dsc_data mt6885_dsc_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = false,
	.dsi_buffer = false,
	.shadow_ctrl_reg = 0x0200,
};

static const struct mtk_disp_dsc_data mt6983_dsc_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = false,
	.need_obuf_sw = true,
	.dsi_buffer = true,
	.shadow_ctrl_reg = 0x0228,
};

static const struct mtk_disp_dsc_data mt6895_dsc_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = false,
	.need_obuf_sw = false,
	.dsi_buffer = true,
	.shadow_ctrl_reg = 0x0228,
};

static const struct mtk_disp_dsc_data mt6873_dsc_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = true,
	.dsi_buffer = false,
	.shadow_ctrl_reg = 0x0200,
};

static const struct mtk_disp_dsc_data mt6853_dsc_driver_data = {
	.support_shadow     = false,
	.need_bypass_shadow = true,
	.dsi_buffer = false,
	.shadow_ctrl_reg = 0x0200,
};

static const struct mtk_disp_dsc_data mt6879_dsc_driver_data = {
	.support_shadow = false,
	.need_bypass_shadow = false,
	.dsi_buffer = true,
	.shadow_ctrl_reg = 0x0200,
};

static const struct mtk_disp_dsc_data mt6855_dsc_driver_data = {
	.support_shadow = false,
	.need_bypass_shadow = false,
	.dsi_buffer = false,
	.shadow_ctrl_reg = 0x0200,
};

static const struct of_device_id mtk_disp_dsc_driver_dt_match[] = {
	{ .compatible = "mediatek,mt6885-disp-dsc",
	  .data = &mt6885_dsc_driver_data},
	{ .compatible = "mediatek,mt6983-disp-dsc",
	  .data = &mt6983_dsc_driver_data},
	{ .compatible = "mediatek,mt6895-disp-dsc",
	  .data = &mt6895_dsc_driver_data},
	{ .compatible = "mediatek,mt6873-disp-dsc",
	  .data = &mt6873_dsc_driver_data},
	{ .compatible = "mediatek,mt6853-disp-dsc",
	  .data = &mt6853_dsc_driver_data},
	{ .compatible = "mediatek,mt6789-disp-dsc",
	  .data = &mt6789_dsc_driver_data},
	{ .compatible = "mediatek,mt6879-disp-dsc",
	  .data = &mt6879_dsc_driver_data},
	{ .compatible = "mediatek,mt6855-disp-dsc",
	  .data = &mt6855_dsc_driver_data},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_disp_dsc_driver_dt_match);

struct platform_driver mtk_disp_dsc_driver = {
	.probe = mtk_disp_dsc_probe,
	.remove = mtk_disp_dsc_remove,
	.driver = {
		.name = "mediatek-disp-dsc",
		.owner = THIS_MODULE,
		.of_match_table = mtk_disp_dsc_driver_dt_match,
	},
};
