// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2021 MediaTek Inc.
// Author: Owen Chen <owen.chen@mediatek.com>

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mt6855-clk.h>

#define MT_CCF_BRINGUP		1

/* Regular Number Definition */
#define INV_OFS			-1
#define INV_BIT			-1

static const struct mtk_gate_regs cam_m_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_CAM_M(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &cam_m_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate cam_m_clks[] = {
	GATE_CAM_M(CLK_CAM_M_LARB13, "cam_m_larb13",
			"cam_ck"/* parent */, 0),
	GATE_CAM_M(CLK_CAM_M_DFP_VAD, "cam_m_dfp_vad",
			"cam_ck"/* parent */, 1),
	GATE_CAM_M(CLK_CAM_M_LARB14, "cam_m_larb14",
			"cam_ck"/* parent */, 2),
	GATE_CAM_M(CLK_CAM_M_CAM, "cam_m_cam",
			"cam_ck"/* parent */, 6),
	GATE_CAM_M(CLK_CAM_M_CAMTG, "cam_m_camtg",
			"cam_ck"/* parent */, 7),
	GATE_CAM_M(CLK_CAM_M_SENINF, "cam_m_seninf",
			"cam_ck"/* parent */, 8),
	GATE_CAM_M(CLK_CAM_M_CAMSV0, "cam_m_camsv0",
			"cam_ck"/* parent */, 9),
	GATE_CAM_M(CLK_CAM_M_CAMSV1, "cam_m_camsv1",
			"cam_ck"/* parent */, 10),
	GATE_CAM_M(CLK_CAM_M_CAMSV2, "cam_m_camsv2",
			"cam_ck"/* parent */, 11),
	GATE_CAM_M(CLK_CAM_M_CAMSV3, "cam_m_camsv3",
			"cam_ck"/* parent */, 12),
	GATE_CAM_M(CLK_CAM_M_CCU0, "cam_m_ccu0",
			"cam_ck"/* parent */, 13),
	GATE_CAM_M(CLK_CAM_M_CCU1, "cam_m_ccu1",
			"cam_ck"/* parent */, 14),
	GATE_CAM_M(CLK_CAM_M_MRAW0, "cam_m_mraw0",
			"cam_ck"/* parent */, 15),
	GATE_CAM_M(CLK_CAM_M_FAKE_ENG, "cam_m_fake_eng",
			"cam_ck"/* parent */, 17),
	GATE_CAM_M(CLK_CAM_M_CCU_GALS, "cam_m_ccu_gals",
			"cam_ck"/* parent */, 18),
	GATE_CAM_M(CLK_CAM_M_CAM2MM_GALS, "cam_m_cam2mm_gals",
			"cam_ck"/* parent */, 19),
	GATE_CAM_M(CLK_CAM_M_CAMSV4, "cam_m_camsv4",
			"cam_ck"/* parent */, 20),
	GATE_CAM_M(CLK_CAM_M_PDA, "cam_m_pda",
			"cam_ck"/* parent */, 21),
};

static const struct mtk_clk_desc cam_m_mcd = {
	.clks = cam_m_clks,
	.num_clks = CLK_CAM_M_NR_CLK,
};

static int clk_mt6855_cam_m_probe(struct platform_device *pdev)
{
	int r;

#if MT_CCF_BRINGUP
	pr_notice("%s init begin\n", __func__);
#endif

	r = mtk_clk_simple_probe(pdev);
	if (r)
		dev_err(&pdev->dev,
			"could not register clock provider: %s: %d\n",
			pdev->name, r);

#if MT_CCF_BRINGUP
	pr_notice("%s init end\n", __func__);
#endif

	return r;
}

static const struct of_device_id of_match_clk_mt6855_cam_m[] = {
	{
		.compatible = "mediatek,mt6855-camsys_main",
		.data = &cam_m_mcd,
	},
	{}
};

static struct platform_driver clk_mt6855_cam_m_drv = {
	.probe = clk_mt6855_cam_m_probe,
	.driver = {
		.name = "clk-mt6855-cam_m",
		.of_match_table = of_match_clk_mt6855_cam_m,
	},
};

static int __init clk_mt6855_cam_m_init(void)
{
	return platform_driver_register(&clk_mt6855_cam_m_drv);
}

static void __exit clk_mt6855_cam_m_exit(void)
{
	platform_driver_unregister(&clk_mt6855_cam_m_drv);
}

arch_initcall(clk_mt6855_cam_m_init);
module_exit(clk_mt6855_cam_m_exit);
MODULE_LICENSE("GPL");
