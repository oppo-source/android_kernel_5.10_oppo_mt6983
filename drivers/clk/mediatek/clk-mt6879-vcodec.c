// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2021 MediaTek Inc.
// Author: Owen Chen <owen.chen@mediatek.com>

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mt6879-clk.h>

#define MT_CCF_BRINGUP		1

/* Regular Number Definition */
#define INV_OFS			-1
#define INV_BIT			-1

static const struct mtk_gate_regs vde20_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x4,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs vde21_cg_regs = {
	.set_ofs = 0x190,
	.clr_ofs = 0x190,
	.sta_ofs = 0x190,
};

static const struct mtk_gate_regs vde22_cg_regs = {
	.set_ofs = 0x8,
	.clr_ofs = 0xC,
	.sta_ofs = 0x8,
};

static const struct mtk_gate_regs vde20_hwv_regs = {
	.set_ofs = 0x58,
	.clr_ofs = 0x5C,
	.sta_ofs = 0x1C2C,
};

static const struct mtk_gate_regs vde22_hwv_regs = {
	.set_ofs = 0x48,
	.clr_ofs = 0x4C,
	.sta_ofs = 0x1C24,
};

#define GATE_VDE20(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &vde20_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr_inv,	\
	}

#define GATE_VDE21(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &vde21_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_no_setclr_inv,	\
	}

#define GATE_VDE22(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &vde22_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr_inv,	\
	}

#define GATE_HWV_VDE20(_id, _name, _parent, _shift) {			\
		.id = _id,						\
		.name = _name,						\
		.parent_name = _parent,					\
		.regs = &vde20_cg_regs,				\
		.hwv_regs = &vde20_hwv_regs,				\
		.shift = _shift,					\
		.ops = &mtk_clk_gate_ops_hwv,			\
		.flags = CLK_USE_HW_VOTER,				\
	}

#define GATE_HWV_VDE22(_id, _name, _parent, _shift) {			\
		.id = _id,						\
		.name = _name,						\
		.parent_name = _parent,					\
		.regs = &vde22_cg_regs,				\
		.hwv_regs = &vde22_hwv_regs,				\
		.shift = _shift,					\
		.ops = &mtk_clk_gate_ops_hwv,			\
		.flags = CLK_USE_HW_VOTER,				\
	}

static const struct mtk_gate vde2_clks[] = {
	/* VDE20 */
	GATE_VDE20(CLK_VDE2_VDEC_CKEN, "vde2_vdec_cken",
			"vdec_ck"/* parent */, 0),
	GATE_VDE20(CLK_VDE2_VDEC_ACTIVE, "vde2_vdec_active",
			"vdec_ck"/* parent */, 4),
	/* VDE21 */
	GATE_VDE21(CLK_VDE2_MINI_MDP_CKEN_CFG_RG, "vde2_mini_mdp_cken",
			"vdec_ck"/* parent */, 0),
	/* VDE22 */
	GATE_VDE22(CLK_VDE2_LARB1_CKEN, "vde2_larb1_cken",
			"vdec_ck"/* parent */, 0),
};

static const struct mtk_clk_desc vde2_mcd = {
	.clks = vde2_clks,
	.num_clks = CLK_VDE2_NR_CLK,
};

static const struct mtk_gate_regs ven1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs ven1_hwv_regs = {
	.set_ofs = 0x68,
	.clr_ofs = 0x6C,
	.sta_ofs = 0x1C34,
};

#define GATE_VEN1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &ven1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr_inv,	\
	}

#define GATE_HWV_VEN1(_id, _name, _parent, _shift) {			\
		.id = _id,						\
		.name = _name,						\
		.parent_name = _parent,					\
		.regs = &ven1_cg_regs,					\
		.hwv_regs = &ven1_hwv_regs,				\
		.shift = _shift,					\
		.ops = &mtk_clk_gate_ops_hwv,				\
		.flags = CLK_USE_HW_VOTER,				\
	}

static const struct mtk_gate ven1_clks[] = {
	GATE_VEN1(CLK_VEN1_CKE0_LARB, "ven1_cke0_larb",
			"venc_ck"/* parent */, 0),
	GATE_VEN1(CLK_VEN1_CKE1_VENC, "ven1_cke1_venc",
			"venc_ck"/* parent */, 4),
	GATE_VEN1(CLK_VEN1_CKE2_JPGENC, "ven1_cke2_jpgenc",
			"venc_ck"/* parent */, 8),
	GATE_VEN1(CLK_VEN1_CKE5_GALS, "ven1_cke5_gals",
			"venc_ck"/* parent */, 28),
};

static const struct mtk_clk_desc ven1_mcd = {
	.clks = ven1_clks,
	.num_clks = CLK_VEN1_NR_CLK,
};

static const struct of_device_id of_match_clk_mt6879_vcodec[] = {
	{
		.compatible = "mediatek,mt6879-vdec_gcon_base",
		.data = &vde2_mcd,
	}, {
		.compatible = "mediatek,mt6879-vencsys",
		.data = &ven1_mcd,
	}, {
		/* sentinel */
	}
};


static int clk_mt6879_vcodec_grp_probe(struct platform_device *pdev)
{
	int r;

#if MT_CCF_BRINGUP
	pr_notice("%s: %s init begin\n", __func__, pdev->name);
#endif

	r = mtk_clk_simple_probe(pdev);
	if (r)
		dev_err(&pdev->dev,
			"could not register clock provider: %s: %d\n",
			pdev->name, r);

#if MT_CCF_BRINGUP
	pr_notice("%s: %s init end\n", __func__, pdev->name);
#endif

	return r;
}

static struct platform_driver clk_mt6879_vcodec_drv = {
	.probe = clk_mt6879_vcodec_grp_probe,
	.driver = {
		.name = "clk-mt6879-vcodec",
		.of_match_table = of_match_clk_mt6879_vcodec,
	},
};

static int __init clk_mt6879_vcodec_init(void)
{
	return platform_driver_register(&clk_mt6879_vcodec_drv);
}

static void __exit clk_mt6879_vcodec_exit(void)
{
	platform_driver_unregister(&clk_mt6879_vcodec_drv);
}

arch_initcall(clk_mt6879_vcodec_init);
module_exit(clk_mt6879_vcodec_exit);
MODULE_LICENSE("GPL");
