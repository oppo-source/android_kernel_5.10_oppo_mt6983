// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2020 MediaTek Inc.
// Author: Owen Chen <owen.chen@mediatek.com>

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mt6853-clk.h>

#define MT_CLKMGR_MODULE_INIT	0

#define MT_CCF_BRINGUP		1

#define INV_OFS			-1

static const struct mtk_gate_regs vdec0_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x4,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs vdec1_cg_regs = {
	.set_ofs = 0x8,
	.clr_ofs = 0xc,
	.sta_ofs = 0x8,
};

#define GATE_VDEC0(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &vdec0_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr_inv,	\
	}

#define GATE_VDEC1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &vdec1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr_inv,	\
	}

static const struct mtk_gate vdec_clks[] = {
	/* VDEC0 */
	GATE_VDEC0(CLK_VDEC_CKEN, "vdec_cken",
			"vdec_ck"/* parent */, 0),
	GATE_VDEC0(CLK_VDEC_ACTIVE, "vdec_active",
			"vdec_ck"/* parent */, 4),
	/* VDEC1 */
	GATE_VDEC1(CLK_VDEC_LARB1_CKEN, "vdec_larb1_cken",
			"vdec_ck"/* parent */, 0),
};

static int clk_mt6853_vdec_probe(struct platform_device *pdev)
{
	struct clk_onecell_data *clk_data;
	int r;
	struct device_node *node = pdev->dev.of_node;

#if MT_CCF_BRINGUP
	pr_notice("%s init begin\n", __func__);
#endif

	clk_data = mtk_alloc_clk_data(CLK_VDEC_NR_CLK);

	mtk_clk_register_gates_with_dev(node, vdec_clks, ARRAY_SIZE(vdec_clks),
			clk_data, &pdev->dev);

	r = of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);

	if (r)
		pr_err("%s(): could not register clock provider: %d\n",
			__func__, r);

#if MT_CCF_BRINGUP
	pr_notice("%s init end\n", __func__);
#endif

	return r;
}

static const struct of_device_id of_match_clk_mt6853_vdec[] = {
	{ .compatible = "mediatek,mt6853-vdec_gcon", },
	{}
};

#if MT_CLKMGR_MODULE_INIT

static struct platform_driver clk_mt6853_vdec_drv = {
	.probe = clk_mt6853_vdec_probe,
	.driver = {
		.name = "clk-mt6853-vdec",
		.of_match_table = of_match_clk_mt6853_vdec,
	},
};

builtin_platform_driver(clk_mt6853_vdec_drv);

#else

static struct platform_driver clk_mt6853_vdec_drv = {
	.probe = clk_mt6853_vdec_probe,
	.driver = {
		.name = "clk-mt6853-vdec",
		.of_match_table = of_match_clk_mt6853_vdec,
	},
};

static int __init clk_mt6853_vdec_init(void)
{
	return platform_driver_register(&clk_mt6853_vdec_drv);
}

static void __exit clk_mt6853_vdec_exit(void)
{
	platform_driver_unregister(&clk_mt6853_vdec_drv);
}

arch_initcall(clk_mt6853_vdec_init);
module_exit(clk_mt6853_vdec_exit);
MODULE_LICENSE("GPL");
#endif	/* MT_CLKMGR_MODULE_INIT */
