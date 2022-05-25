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

#include <dt-bindings/clock/mt6855-clk.h>

#define MT_CCF_BRINGUP		1

/* Regular Number Definition */
#define INV_OFS			-1
#define INV_BIT			-1

static const struct mtk_gate_regs afe0_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x0,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs afe1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x4,
	.sta_ofs = 0x4,
};

static const struct mtk_gate_regs afe2_cg_regs = {
	.set_ofs = 0x8,
	.clr_ofs = 0x8,
	.sta_ofs = 0x8,
};

#define GATE_AFE0(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &afe0_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_no_setclr,	\
	}

#define GATE_AFE1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &afe1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_no_setclr,	\
	}

#define GATE_AFE2(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &afe2_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_no_setclr,	\
	}

static const struct mtk_gate afe_clks[] = {
	/* AFE0 */
	GATE_AFE0(CLK_AFE_AFE, "afe_afe",
			"audio_ck"/* parent */, 2),
	GATE_AFE0(CLK_AFE_22M, "afe_22m",
			"aud_engen1_ck"/* parent */, 8),
	GATE_AFE0(CLK_AFE_24M, "afe_24m",
			"aud_engen2_ck"/* parent */, 9),
	GATE_AFE0(CLK_AFE_APLL2_TUNER, "afe_apll2_tuner",
			"aud_engen2_ck"/* parent */, 18),
	GATE_AFE0(CLK_AFE_APLL_TUNER, "afe_apll_tuner",
			"aud_engen1_ck"/* parent */, 19),
	GATE_AFE0(CLK_AFE_TDM, "afe_tdm_ck",
			"aud_1_ck"/* parent */, 20),
	GATE_AFE0(CLK_AFE_ADC, "afe_adc",
			"audio_ck"/* parent */, 24),
	GATE_AFE0(CLK_AFE_DAC, "afe_dac",
			"audio_ck"/* parent */, 25),
	GATE_AFE0(CLK_AFE_DAC_PREDIS, "afe_dac_predis",
			"audio_ck"/* parent */, 26),
	GATE_AFE0(CLK_AFE_TML, "afe_tml",
			"audio_ck"/* parent */, 27),
	GATE_AFE0(CLK_AFE_NLE, "afe_nle",
			"audio_ck"/* parent */, 28),
	/* AFE1 */
	GATE_AFE1(CLK_AFE_GENERAL3_ASRC, "afe_general3_asrc",
			"audio_ck"/* parent */, 11),
	GATE_AFE1(CLK_AFE_CONNSYS_I2S_ASRC, "afe_connsys_i2s_asrc",
			"audio_ck"/* parent */, 12),
	GATE_AFE1(CLK_AFE_GENERAL1_ASRC, "afe_general1_asrc",
			"audio_ck"/* parent */, 13),
	GATE_AFE1(CLK_AFE_GENERAL2_ASRC, "afe_general2_asrc",
			"audio_ck"/* parent */, 14),
	GATE_AFE1(CLK_AFE_DAC_HIRES, "afe_dac_hires",
			"audio_h_ck"/* parent */, 15),
	GATE_AFE1(CLK_AFE_ADC_HIRES, "afe_adc_hires",
			"audio_h_ck"/* parent */, 16),
	GATE_AFE1(CLK_AFE_ADC_HIRES_TML, "afe_adc_hires_tml",
			"audio_h_ck"/* parent */, 17),
	GATE_AFE1(CLK_AFE_3RD_DAC, "afe_3rd_dac",
			"audio_ck"/* parent */, 28),
	GATE_AFE1(CLK_AFE_3RD_DAC_PREDIS, "afe_3rd_dac_predis",
			"audio_ck"/* parent */, 29),
	GATE_AFE1(CLK_AFE_3RD_DAC_TML, "afe_3rd_dac_tml",
			"audio_ck"/* parent */, 30),
	GATE_AFE1(CLK_AFE_3RD_DAC_HIRES, "afe_3rd_dac_hires",
			"audio_h_ck"/* parent */, 31),
	/* AFE2 */
	GATE_AFE2(CLK_AFE_I2S5_BCLK, "afe_i2s5_bclk",
			"audio_ck"/* parent */, 0),
	GATE_AFE2(CLK_AFE_I2S6_BCLK, "afe_i2s6_bclk",
			"audio_ck"/* parent */, 1),
	GATE_AFE2(CLK_AFE_I2S7_BCLK, "afe_i2s7_bclk",
			"audio_ck"/* parent */, 2),
	GATE_AFE2(CLK_AFE_I2S8_BCLK, "afe_i2s8_bclk",
			"audio_ck"/* parent */, 3),
	GATE_AFE2(CLK_AFE_I2S9_BCLK, "afe_i2s9_bclk",
			"audio_ck"/* parent */, 4),
	GATE_AFE2(CLK_AFE_ETDM_IN0_BCLK, "afe_etdm_in0_bclk",
			"audio_ck"/* parent */, 5),
	GATE_AFE2(CLK_AFE_ETDM_OUT0_BCLK, "afe_etdm_out0_bclk",
			"audio_ck"/* parent */, 6),
	GATE_AFE2(CLK_AFE_I2S1_BCLK, "afe_i2s1_bclk",
			"audio_ck"/* parent */, 7),
	GATE_AFE2(CLK_AFE_I2S2_BCLK, "afe_i2s2_bclk",
			"audio_ck"/* parent */, 8),
	GATE_AFE2(CLK_AFE_I2S3_BCLK, "afe_i2s3_bclk",
			"audio_ck"/* parent */, 9),
	GATE_AFE2(CLK_AFE_I2S4_BCLK, "afe_i2s4_bclk",
			"audio_ck"/* parent */, 10),
	GATE_AFE2(CLK_AFE_ETDM_IN1_BCLK, "afe_etdm_in1_bclk",
			"audio_ck"/* parent */, 23),
	GATE_AFE2(CLK_AFE_ETDM_OUT1_BCLK, "afe_etdm_out1_bclk",
			"audio_ck"/* parent */, 24),
};

static const struct mtk_clk_desc afe_mcd = {
	.clks = afe_clks,
	.num_clks = CLK_AFE_NR_CLK,
};

static const struct of_device_id of_match_clk_mt6855_adsp[] = {
	{
		.compatible = "mediatek,mt6855-afe",
		.data = &afe_mcd,
	}, {
		/* sentinel */
	}
};


static int clk_mt6855_adsp_grp_probe(struct platform_device *pdev)
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

static struct platform_driver clk_mt6855_adsp_drv = {
	.probe = clk_mt6855_adsp_grp_probe,
	.driver = {
		.name = "clk-mt6855-adsp",
		.of_match_table = of_match_clk_mt6855_adsp,
	},
};

static int __init clk_mt6855_adsp_init(void)
{
	return platform_driver_register(&clk_mt6855_adsp_drv);
}

static void __exit clk_mt6855_adsp_exit(void)
{
	platform_driver_unregister(&clk_mt6855_adsp_drv);
}

arch_initcall(clk_mt6855_adsp_init);
module_exit(clk_mt6855_adsp_exit);
MODULE_LICENSE("GPL");
