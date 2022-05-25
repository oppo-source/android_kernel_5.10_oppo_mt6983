/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6853-afe-clk.h  --  Mediatek 6853 afe clock ctrl definition
 *
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Shane Chien <shane.chien@mediatek.com>
 */

#ifndef _MT6853_AFE_CLOCK_CTRL_H_
#define _MT6853_AFE_CLOCK_CTRL_H_

#define AP_PLL_CON3 0x0014
#define APLL1_CON0 0x0318
#define APLL1_CON1 0x031c
#define APLL1_CON2 0x0320
#define APLL1_CON4 0x0328
#define APLL1_TUNER_CON0 0x0040

#define APLL2_CON0 0x032c
#define APLL2_CON1 0x0330
#define APLL2_CON2 0x0334
#define APLL2_CON4 0x033c
#define APLL2_TUNER_CON0 0x0044

#define CLK_CFG_7 0x0080
#define CLK_CFG_8 0x0090
#define CLK_CFG_11 0x00c0
#define CLK_CFG_12 0x00d0
#define CLK_CFG_13 0x00e0
#define CLK_CFG_15 0x0100

#define CLK_AUDDIV_0 0x0320
#define CLK_AUDDIV_2 0x0328
#define CLK_AUDDIV_3 0x0334
#define CLK_AUDDIV_4 0x0338
#define CKSYS_AUD_TOP_CFG 0x032c
#define CKSYS_AUD_TOP_MON 0x0330

#define PERI_BUS_DCM_CTRL 0x0074
#define MODULE_SW_CG_1_STA 0x0094
#define MODULE_SW_CG_2_STA 0x00ac

/* CLK_AUDDIV_0 */
#define APLL12_DIV0_PDN_SFT                0
#define APLL12_DIV0_PDN_MASK               0x1
#define APLL12_DIV0_PDN_MASK_SFT           (0x1 << 0)
#define APLL12_DIV1_PDN_SFT                1
#define APLL12_DIV1_PDN_MASK               0x1
#define APLL12_DIV1_PDN_MASK_SFT           (0x1 << 1)
#define APLL12_DIV2_PDN_SFT                2
#define APLL12_DIV2_PDN_MASK               0x1
#define APLL12_DIV2_PDN_MASK_SFT           (0x1 << 2)
#define APLL12_DIV3_PDN_SFT                3
#define APLL12_DIV3_PDN_MASK               0x1
#define APLL12_DIV3_PDN_MASK_SFT           (0x1 << 3)
#define APLL12_DIV4_PDN_SFT                4
#define APLL12_DIV4_PDN_MASK               0x1
#define APLL12_DIV4_PDN_MASK_SFT           (0x1 << 4)
#define APLL12_DIVB_PDN_SFT                5
#define APLL12_DIVB_PDN_MASK               0x1
#define APLL12_DIVB_PDN_MASK_SFT           (0x1 << 5)
#define APLL12_DIV5_PDN_SFT                6
#define APLL12_DIV5_PDN_MASK               0x1
#define APLL12_DIV5_PDN_MASK_SFT           (0x1 << 6)
#define APLL_I2S0_MCK_SEL_SFT              16
#define APLL_I2S0_MCK_SEL_MASK             0x1
#define APLL_I2S0_MCK_SEL_MASK_SFT         (0x1 << 16)
#define APLL_I2S1_MCK_SEL_SFT              17
#define APLL_I2S1_MCK_SEL_MASK             0x1
#define APLL_I2S1_MCK_SEL_MASK_SFT         (0x1 << 17)
#define APLL_I2S2_MCK_SEL_SFT              18
#define APLL_I2S2_MCK_SEL_MASK             0x1
#define APLL_I2S2_MCK_SEL_MASK_SFT         (0x1 << 18)
#define APLL_I2S3_MCK_SEL_SFT              19
#define APLL_I2S3_MCK_SEL_MASK             0x1
#define APLL_I2S3_MCK_SEL_MASK_SFT         (0x1 << 19)
#define APLL_I2S4_MCK_SEL_SFT              20
#define APLL_I2S4_MCK_SEL_MASK             0x1
#define APLL_I2S4_MCK_SEL_MASK_SFT         (0x1 << 20)
#define APLL_I2S5_MCK_SEL_SFT              21
#define APLL_I2S5_MCK_SEL_MASK             0x1
#define APLL_I2S5_MCK_SEL_MASK_SFT         (0x1 << 21)

/* CLK_AUDDIV_2 */
#define APLL12_CK_DIV0_SFT                 0
#define APLL12_CK_DIV0_MASK                0xff
#define APLL12_CK_DIV0_MASK_SFT            (0xff << 0)
#define APLL12_CK_DIV1_SFT                 8
#define APLL12_CK_DIV1_MASK                0xff
#define APLL12_CK_DIV1_MASK_SFT            (0xff << 8)
#define APLL12_CK_DIV2_SFT                 16
#define APLL12_CK_DIV2_MASK                0xff
#define APLL12_CK_DIV2_MASK_SFT            (0xff << 16)
#define APLL12_CK_DIV3_SFT                 24
#define APLL12_CK_DIV3_MASK                0xff
#define APLL12_CK_DIV3_MASK_SFT            (0xff << 24)

/* CLK_AUDDIV_3 */
#define APLL12_CK_DIV4_SFT                 0
#define APLL12_CK_DIV4_MASK                0xff
#define APLL12_CK_DIV4_MASK_SFT            (0xff << 0)
#define APLL12_CK_DIVB_SFT                 8
#define APLL12_CK_DIVB_MASK                0xff
#define APLL12_CK_DIVB_MASK_SFT            (0xff << 8)
#define APLL12_CK_DIV5_SFT                 16
#define APLL12_CK_DIV5_MASK                0xff
#define APLL12_CK_DIV5_MASK_SFT            (0xff << 16)

/* AUD_TOP_CFG */
#define AUD_TOP_CFG_SFT                    0
#define AUD_TOP_CFG_MASK                   0xffffffff
#define AUD_TOP_CFG_MASK_SFT               (0xffffffff << 0)

/* AUD_TOP_MON */
#define AUD_TOP_MON_SFT                    0
#define AUD_TOP_MON_MASK                   0xffffffff
#define AUD_TOP_MON_MASK_SFT               (0xffffffff << 0)

/* CLK_AUDDIV_3 */
#define APLL12_CK_DIV5_MSB_SFT             0
#define APLL12_CK_DIV5_MSB_MASK            0xf
#define APLL12_CK_DIV5_MSB_MASK_SFT        (0xf << 0)
#define RESERVED0_SFT                      4
#define RESERVED0_MASK                     0xfffffff
#define RESERVED0_MASK_SFT                 (0xfffffff << 4)

/* APLL */
#define APLL1_W_NAME "APLL1"
#define APLL2_W_NAME "APLL2"
enum {
	MT6853_APLL1 = 0,
	MT6853_APLL2,
};

struct mtk_base_afe;

int mt6853_init_clock(struct mtk_base_afe *afe);
int mt6853_afe_enable_clock(struct mtk_base_afe *afe);
void mt6853_afe_disable_clock(struct mtk_base_afe *afe);

int mt6853_afe_dram_request(struct device *dev);
int mt6853_afe_dram_release(struct device *dev);

int mt6853_apll1_enable(struct mtk_base_afe *afe);
void mt6853_apll1_disable(struct mtk_base_afe *afe);

int mt6853_apll2_enable(struct mtk_base_afe *afe);
void mt6853_apll2_disable(struct mtk_base_afe *afe);

int mt6853_get_apll_rate(struct mtk_base_afe *afe, int apll);
int mt6853_get_apll_by_rate(struct mtk_base_afe *afe, int rate);
int mt6853_get_apll_by_name(struct mtk_base_afe *afe, const char *name);

extern void aud_intbus_mux_sel(unsigned int aud_idx);

/* these will be replaced by using CCF */
int mt6853_mck_enable(struct mtk_base_afe *afe, int mck_id, int rate);
void mt6853_mck_disable(struct mtk_base_afe *afe, int mck_id);

int mt6853_set_audio_int_bus_parent(struct mtk_base_afe *afe,
				    int clk_id);

#endif
