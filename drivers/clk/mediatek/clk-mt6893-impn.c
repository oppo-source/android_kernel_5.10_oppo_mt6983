// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2020 MediaTek Inc.
// Author: Owen Chen <owen.chen@mediatek.com>

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mt6893-clk.h>

#define MT_CLKMGR_MODULE_INIT	0

#define MT_CCF_BRINGUP		1

#define INV_OFS			-1

static const struct mtk_gate_regs impn_cg_regs = {
	.set_ofs = 0xe08,
	.clr_ofs = 0xe04,
	.sta_ofs = 0xe00,
};

#define GATE_IMPN(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &impn_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate impn_clks[] = {
	GATE_IMPN(CLK_IMPN_AP_I2C5_RO, "impn_ap_i2c5_ro",
			"i2c_ck"/* parent */, 0),
	GATE_IMPN(CLK_IMPN_AP_I2C6_RO, "impn_ap_i2c6_ro",
			"i2c_ck"/* parent */, 1),
};

static int clk_mt6893_impn_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk_onecell_data *clk_data;
	int r;

#if MT_CCF_BRINGUP
	pr_notice("%s init begin\n", __func__);
#endif

	clk_data = mtk_alloc_clk_data(CLK_IMPN_NR_CLK);

	mtk_clk_register_gates(node, impn_clks, ARRAY_SIZE(impn_clks),
			clk_data);

	r = of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);

	if (r)
		pr_err("%s(): could not register clock provider: %d\n",
			__func__, r);

#if MT_CCF_BRINGUP
	pr_notice("%s init end\n", __func__);
#endif

	return r;
}

static const struct of_device_id of_match_clk_mt6893_impn[] = {
	{ .compatible = "mediatek,mt6893-imp_iic_wrap_n", },
	{}
};

#if MT_CLKMGR_MODULE_INIT

static struct platform_driver clk_mt6893_impn_drv = {
	.probe = clk_mt6893_impn_probe,
	.driver = {
		.name = "clk-mt6893-impn",
		.of_match_table = of_match_clk_mt6893_impn,
	},
};

builtin_platform_driver(clk_mt6893_impn_drv);

#else

static struct platform_driver clk_mt6893_impn_drv = {
	.probe = clk_mt6893_impn_probe,
	.driver = {
		.name = "clk-mt6893-impn",
		.of_match_table = of_match_clk_mt6893_impn,
	},
};

static int __init clk_mt6893_impn_init(void)
{
	return platform_driver_register(&clk_mt6893_impn_drv);
}

static void __exit clk_mt6893_impn_exit(void)
{
	platform_driver_unregister(&clk_mt6893_impn_drv);
}

arch_initcall(clk_mt6893_impn_init);
module_exit(clk_mt6893_impn_exit);
MODULE_LICENSE("GPL");
#endif	/* MT_CLKMGR_MODULE_INIT */
