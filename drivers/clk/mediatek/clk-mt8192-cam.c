// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2020 MediaTek Inc.
// Author: Weiyi Lu <weiyi.lu@mediatek.com>

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mt8192-clk.h>

static const struct mtk_gate_regs cam_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_CAM(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &cam_cg_regs, _shift,	\
		&mtk_clk_gate_ops_setclr)

static const struct mtk_gate cam_clks[] = {
	GATE_CAM(CLK_CAM_LARB13, "cam_larb13", "cam_sel", 0),
	GATE_CAM(CLK_CAM_DFP_VAD, "cam_dfp_vad", "cam_sel", 1),
	GATE_CAM(CLK_CAM_LARB14, "cam_larb14", "cam_sel", 2),
	GATE_CAM(CLK_CAM_CAM, "cam_cam", "cam_sel", 6),
	GATE_CAM(CLK_CAM_CAMTG, "cam_camtg", "cam_sel", 7),
	GATE_CAM(CLK_CAM_SENINF, "cam_seninf", "cam_sel", 8),
	GATE_CAM(CLK_CAM_CAMSV0, "cam_camsv0", "cam_sel", 9),
	GATE_CAM(CLK_CAM_CAMSV1, "cam_camsv1", "cam_sel", 10),
	GATE_CAM(CLK_CAM_CAMSV2, "cam_camsv2", "cam_sel", 11),
	GATE_CAM(CLK_CAM_CAMSV3, "cam_camsv3", "cam_sel", 12),
	GATE_CAM(CLK_CAM_CCU0, "cam_ccu0", "cam_sel", 13),
	GATE_CAM(CLK_CAM_CCU1, "cam_ccu1", "cam_sel", 14),
	GATE_CAM(CLK_CAM_MRAW0, "cam_mraw0", "cam_sel", 15),
	GATE_CAM(CLK_CAM_FAKE_ENG, "cam_fake_eng", "cam_sel", 17),
	GATE_CAM(CLK_CAM_CCU_GALS, "cam_ccu_gals", "cam_sel", 18),
	GATE_CAM(CLK_CAM_CAM2MM_GALS, "cam2mm_gals", "cam_sel", 19),
};

static int clk_mt8192_cam_probe(struct platform_device *pdev)
{
	struct clk_onecell_data *clk_data;
	struct device_node *node = pdev->dev.of_node;

	clk_data = mtk_alloc_clk_data(CLK_CAM_NR_CLK);

	mtk_clk_register_gates(node, cam_clks, ARRAY_SIZE(cam_clks),
			clk_data);

	return of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);
}

static const struct of_device_id of_match_clk_mt8192_cam[] = {
	{ .compatible = "mediatek,mt8192-camsys", },
	{}
};

static struct platform_driver clk_mt8192_cam_drv = {
	.probe = clk_mt8192_cam_probe,
	.driver = {
		.name = "clk-mt8192-cam",
		.of_match_table = of_match_clk_mt8192_cam,
	},
};

static int __init clk_mt8192_cam_init(void)
{
	return platform_driver_register(&clk_mt8192_cam_drv);
}

static void __exit clk_mt8192_cam_exit(void)
{
	platform_driver_unregister(&clk_mt8192_cam_drv);
}

arch_initcall(clk_mt8192_cam_init);
module_exit(clk_mt8192_cam_exit);
MODULE_LICENSE("GPL");
