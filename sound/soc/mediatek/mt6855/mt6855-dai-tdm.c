// SPDX-License-Identifier: GPL-2.0
/*
 *  MediaTek ALSA SoC Audio DAI TDM Control
 *
 *  Copyright (c) 2021 MediaTek Inc.
 *  Author: Tina Tsai <tina.tsai@mediatek.com>
 */

#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include "mt6855-afe-clk.h"
#include "mt6855-afe-common.h"
#include "mt6855-interconnection.h"

struct mtk_afe_tdm_priv {
	int bck_id;
	int bck_rate;

	int mclk_id;
	int mclk_multiple; /* according to sample rate */
	int mclk_rate;
	int mclk_apll;
};

enum {
	TDM_WLEN_16_BIT = 1,
	TDM_WLEN_32_BIT = 2,
};

enum {
	TDM_CHANNEL_BCK_16 = 0,
	TDM_CHANNEL_BCK_24 = 1,
	TDM_CHANNEL_BCK_32 = 2,
};

enum {
	TDM_CHANNEL_NUM_2 = 0,
	TDM_CHANNEL_NUM_4 = 1,
	TDM_CHANNEL_NUM_8 = 2,
};

enum  {
	TDM_CH_START_O30_O31 = 0,
	TDM_CH_START_O32_O33,
	TDM_CH_START_O34_O35,
	TDM_CH_START_O36_O37,
	TDM_CH_ZERO,
};

enum {
	DPTX_CHANNEL_2,
	DPTX_CHANNEL_8,
};

enum {
	DPTX_WLEN_24_BIT,
	DPTX_WLEN_16_BIT,
};

enum {
	DPTX_CH_EN_MASK_2CH = 0x3,
	DPTX_CH_EN_MASK_4CH = 0xf,
	DPTX_CH_EN_MASK_6CH = 0x3f,
	DPTX_CH_EN_MASK_8CH = 0xff,
};

static unsigned int get_tdm_wlen(snd_pcm_format_t format)
{
	return snd_pcm_format_physical_width(format) <= 16 ?
	       TDM_WLEN_16_BIT : TDM_WLEN_32_BIT;
}

static unsigned int get_tdm_channel_bck(snd_pcm_format_t format)
{
	return snd_pcm_format_physical_width(format) <= 16 ?
	       TDM_CHANNEL_BCK_16 : TDM_CHANNEL_BCK_32;
}

static unsigned int get_tdm_lrck_width(snd_pcm_format_t format)
{
	return snd_pcm_format_physical_width(format) - 1;
}

static unsigned int get_tdm_ch(unsigned int ch)
{
	switch (ch) {
	case 1:
	case 2:
		return TDM_CHANNEL_NUM_2;
	case 3:
	case 4:
		return TDM_CHANNEL_NUM_4;
	case 5:
	case 6:
	case 7:
	case 8:
	default:
		return TDM_CHANNEL_NUM_8;
	}
}

static unsigned int get_dptx_ch_enable_mask(unsigned int ch)
{
	switch (ch) {
	case 1:
	case 2:
		return DPTX_CH_EN_MASK_2CH;
	case 3:
	case 4:
		return DPTX_CH_EN_MASK_4CH;
	case 5:
	case 6:
		return DPTX_CH_EN_MASK_6CH;
	case 7:
	case 8:
		return DPTX_CH_EN_MASK_8CH;
	default:
		pr_info("%s(), invalid channel num, default use 2ch\n",
			__func__);
		return DPTX_CH_EN_MASK_2CH;
	}
}

static unsigned int get_dptx_ch(unsigned int ch)
{
	if (ch == 2)
		return DPTX_CHANNEL_2;
	else
		return DPTX_CHANNEL_8;
}

static unsigned int get_dptx_wlen(snd_pcm_format_t format)
{
	return snd_pcm_format_physical_width(format) <= 16 ?
	       DPTX_WLEN_16_BIT : DPTX_WLEN_24_BIT;
}

/* interconnection */
enum {
	HDMI_CONN_CH0 = 0,
	HDMI_CONN_CH1,
	HDMI_CONN_CH2,
	HDMI_CONN_CH3,
	HDMI_CONN_CH4,
	HDMI_CONN_CH5,
	HDMI_CONN_CH6,
	HDMI_CONN_CH7,
};

static const char *const hdmi_conn_mux_map[] = {
	"CH0", "CH1", "CH2", "CH3",
	"CH4", "CH5", "CH6", "CH7",
};

static int hdmi_conn_mux_map_value[] = {
	HDMI_CONN_CH0,
	HDMI_CONN_CH1,
	HDMI_CONN_CH2,
	HDMI_CONN_CH3,
	HDMI_CONN_CH4,
	HDMI_CONN_CH5,
	HDMI_CONN_CH6,
	HDMI_CONN_CH7,
};

static SOC_VALUE_ENUM_SINGLE_DECL(hdmi_ch0_mux_map_enum,
				  AFE_HDMI_CONN0,
				  HDMI_O_0_SFT,
				  HDMI_O_0_MASK,
				  hdmi_conn_mux_map,
				  hdmi_conn_mux_map_value);

static const struct snd_kcontrol_new hdmi_ch0_mux_control =
	SOC_DAPM_ENUM("HDMI_CH0_MUX", hdmi_ch0_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(hdmi_ch1_mux_map_enum,
				  AFE_HDMI_CONN0,
				  HDMI_O_1_SFT,
				  HDMI_O_1_MASK,
				  hdmi_conn_mux_map,
				  hdmi_conn_mux_map_value);

static const struct snd_kcontrol_new hdmi_ch1_mux_control =
	SOC_DAPM_ENUM("HDMI_CH1_MUX", hdmi_ch1_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(hdmi_ch2_mux_map_enum,
				  AFE_HDMI_CONN0,
				  HDMI_O_2_SFT,
				  HDMI_O_2_MASK,
				  hdmi_conn_mux_map,
				  hdmi_conn_mux_map_value);

static const struct snd_kcontrol_new hdmi_ch2_mux_control =
	SOC_DAPM_ENUM("HDMI_CH2_MUX", hdmi_ch2_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(hdmi_ch3_mux_map_enum,
				  AFE_HDMI_CONN0,
				  HDMI_O_3_SFT,
				  HDMI_O_3_MASK,
				  hdmi_conn_mux_map,
				  hdmi_conn_mux_map_value);

static const struct snd_kcontrol_new hdmi_ch3_mux_control =
	SOC_DAPM_ENUM("HDMI_CH3_MUX", hdmi_ch3_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(hdmi_ch4_mux_map_enum,
				  AFE_HDMI_CONN0,
				  HDMI_O_4_SFT,
				  HDMI_O_4_MASK,
				  hdmi_conn_mux_map,
				  hdmi_conn_mux_map_value);

static const struct snd_kcontrol_new hdmi_ch4_mux_control =
	SOC_DAPM_ENUM("HDMI_CH4_MUX", hdmi_ch4_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(hdmi_ch5_mux_map_enum,
				  AFE_HDMI_CONN0,
				  HDMI_O_5_SFT,
				  HDMI_O_5_MASK,
				  hdmi_conn_mux_map,
				  hdmi_conn_mux_map_value);

static const struct snd_kcontrol_new hdmi_ch5_mux_control =
	SOC_DAPM_ENUM("HDMI_CH5_MUX", hdmi_ch5_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(hdmi_ch6_mux_map_enum,
				  AFE_HDMI_CONN0,
				  HDMI_O_6_SFT,
				  HDMI_O_6_MASK,
				  hdmi_conn_mux_map,
				  hdmi_conn_mux_map_value);

static const struct snd_kcontrol_new hdmi_ch6_mux_control =
	SOC_DAPM_ENUM("HDMI_CH6_MUX", hdmi_ch6_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_DECL(hdmi_ch7_mux_map_enum,
				  AFE_HDMI_CONN0,
				  HDMI_O_7_SFT,
				  HDMI_O_7_MASK,
				  hdmi_conn_mux_map,
				  hdmi_conn_mux_map_value);

static const struct snd_kcontrol_new hdmi_ch7_mux_control =
	SOC_DAPM_ENUM("HDMI_CH7_MUX", hdmi_ch7_mux_map_enum);

static const char *const tdm_out_mux_map[] = {
	"Disconnect", "Connect",
};

static int tdm_out_mux_map_value[] = {
	0, 1,
};

static SOC_VALUE_ENUM_SINGLE_AUTODISABLE_DECL(hdmi_out_mux_map_enum,
		SND_SOC_NOPM,
		0,
		1,
		tdm_out_mux_map,
		tdm_out_mux_map_value);
static const struct snd_kcontrol_new hdmi_out_mux_control =
	SOC_DAPM_ENUM("HDMI_OUT_MUX", hdmi_out_mux_map_enum);

static SOC_VALUE_ENUM_SINGLE_AUTODISABLE_DECL(dptx_out_mux_map_enum,
		SND_SOC_NOPM,
		0,
		1,
		tdm_out_mux_map,
		tdm_out_mux_map_value);
static const struct snd_kcontrol_new dptx_out_mux_control =
	SOC_DAPM_ENUM("DPTX_OUT_MUX", dptx_out_mux_map_enum);


static SOC_VALUE_ENUM_SINGLE_AUTODISABLE_DECL(dptx_virtual_out_mux_map_enum,
		SND_SOC_NOPM,
		0,
		1,
		tdm_out_mux_map,
		tdm_out_mux_map_value);

static const struct snd_kcontrol_new dptx_virtual_out_mux_control =
	SOC_DAPM_ENUM("DPTX_VIRTUAL_OUT_MUX", dptx_virtual_out_mux_map_enum);

enum {
	SUPPLY_SEQ_APLL,
	SUPPLY_SEQ_TDM_MCK_EN,
	SUPPLY_SEQ_TDM_BCK_EN,
	SUPPLY_SEQ_TDM_DPTX_MCK_EN,
	SUPPLY_SEQ_TDM_DPTX_BCK_EN,
	SUPPLY_SEQ_TDM_EN,
};

static int get_tdm_id_by_name(const char *name)
{
	if (strstr(name, "DPTX"))
		return MT6855_DAI_TDM_DPTX;
	else
		return MT6855_DAI_TDM;
}

static int mtk_tdm_bck_en_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol,
				int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6855_afe_private *afe_priv = afe->platform_priv;
	int dai_id = get_tdm_id_by_name(w->name);
	struct mtk_afe_tdm_priv *tdm_priv = afe_priv->dai_priv[dai_id];

	dev_info(cmpnt->dev, "%s(), name %s, event 0x%x, dai_id %d\n",
		 __func__, w->name, event, dai_id);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mt6855_mck_enable(afe, tdm_priv->bck_id, tdm_priv->bck_rate);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mt6855_mck_disable(afe, tdm_priv->bck_id);
		break;
	default:
		break;
	}

	return 0;
}

static int mtk_tdm_mck_en_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol,
				int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6855_afe_private *afe_priv = afe->platform_priv;
	int dai_id = get_tdm_id_by_name(w->name);
	struct mtk_afe_tdm_priv *tdm_priv = afe_priv->dai_priv[dai_id];

	dev_info(cmpnt->dev, "%s(), name %s, event 0x%x, dai_id %d\n",
		 __func__, w->name, event, dai_id);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mt6855_mck_enable(afe, tdm_priv->mclk_id, tdm_priv->mclk_rate);
		break;
	case SND_SOC_DAPM_POST_PMD:
		tdm_priv->mclk_rate = 0;
		mt6855_mck_disable(afe, tdm_priv->mclk_id);
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget mtk_dai_tdm_widgets[] = {
	SND_SOC_DAPM_MUX("HDMI_CH0_MUX", SND_SOC_NOPM, 0, 0,
			 &hdmi_ch0_mux_control),
	SND_SOC_DAPM_MUX("HDMI_CH1_MUX", SND_SOC_NOPM, 0, 0,
			 &hdmi_ch1_mux_control),
	SND_SOC_DAPM_MUX("HDMI_CH2_MUX", SND_SOC_NOPM, 0, 0,
			 &hdmi_ch2_mux_control),
	SND_SOC_DAPM_MUX("HDMI_CH3_MUX", SND_SOC_NOPM, 0, 0,
			 &hdmi_ch3_mux_control),
	SND_SOC_DAPM_MUX("HDMI_CH4_MUX", SND_SOC_NOPM, 0, 0,
			 &hdmi_ch4_mux_control),
	SND_SOC_DAPM_MUX("HDMI_CH5_MUX", SND_SOC_NOPM, 0, 0,
			 &hdmi_ch5_mux_control),
	SND_SOC_DAPM_MUX("HDMI_CH6_MUX", SND_SOC_NOPM, 0, 0,
			 &hdmi_ch6_mux_control),
	SND_SOC_DAPM_MUX("HDMI_CH7_MUX", SND_SOC_NOPM, 0, 0,
			 &hdmi_ch7_mux_control),

	SND_SOC_DAPM_MUX("HDMI_OUT_MUX", SND_SOC_NOPM, 0, 0,
			 &hdmi_out_mux_control),
	SND_SOC_DAPM_MUX("DPTX_OUT_MUX", SND_SOC_NOPM, 0, 0,
			 &dptx_out_mux_control),
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	SND_SOC_DAPM_CLOCK_SUPPLY("aud_tdm_clk"),
#endif
	SND_SOC_DAPM_SUPPLY_S("TDM_EN", SUPPLY_SEQ_TDM_EN,
			      AFE_TDM_CON1, TDM_EN_SFT, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY_S("TDM_BCK", SUPPLY_SEQ_TDM_BCK_EN,
			      SND_SOC_NOPM, 0, 0,
			      mtk_tdm_bck_en_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY_S("TDM_MCK", SUPPLY_SEQ_TDM_MCK_EN,
			      SND_SOC_NOPM, 0, 0,
			      mtk_tdm_mck_en_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY_S("TDM_DPTX_BCK", SUPPLY_SEQ_TDM_DPTX_BCK_EN,
			      SND_SOC_NOPM, 0, 0,
			      mtk_tdm_bck_en_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY_S("TDM_DPTX_MCK", SUPPLY_SEQ_TDM_DPTX_MCK_EN,
			      SND_SOC_NOPM, 0, 0,
			      mtk_tdm_mck_en_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("DPTX_VIRTUAL_OUT_MUX",
			 SND_SOC_NOPM, 0, 0, &dptx_virtual_out_mux_control),
	SND_SOC_DAPM_OUTPUT("DPTX_VIRTUAL_OUT"),
};

static int mtk_afe_tdm_apll_connect(struct snd_soc_dapm_widget *source,
				    struct snd_soc_dapm_widget *sink)
{
	struct snd_soc_dapm_widget *w = sink;
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6855_afe_private *afe_priv = afe->platform_priv;
	int dai_id = get_tdm_id_by_name(w->name);
	struct mtk_afe_tdm_priv *tdm_priv = afe_priv->dai_priv[dai_id];
	int cur_apll;

	/* which apll */
	cur_apll = mt6855_get_apll_by_name(afe, source->name);

	return (tdm_priv->mclk_apll == cur_apll) ? 1 : 0;
}

static const struct snd_soc_dapm_route mtk_dai_tdm_routes[] = {
	{"HDMI_CH0_MUX", "CH0", "HDMI"},
	{"HDMI_CH0_MUX", "CH1", "HDMI"},
	{"HDMI_CH0_MUX", "CH2", "HDMI"},
	{"HDMI_CH0_MUX", "CH3", "HDMI"},
	{"HDMI_CH0_MUX", "CH4", "HDMI"},
	{"HDMI_CH0_MUX", "CH5", "HDMI"},
	{"HDMI_CH0_MUX", "CH6", "HDMI"},
	{"HDMI_CH0_MUX", "CH7", "HDMI"},

	{"HDMI_CH1_MUX", "CH0", "HDMI"},
	{"HDMI_CH1_MUX", "CH1", "HDMI"},
	{"HDMI_CH1_MUX", "CH2", "HDMI"},
	{"HDMI_CH1_MUX", "CH3", "HDMI"},
	{"HDMI_CH1_MUX", "CH4", "HDMI"},
	{"HDMI_CH1_MUX", "CH5", "HDMI"},
	{"HDMI_CH1_MUX", "CH6", "HDMI"},
	{"HDMI_CH1_MUX", "CH7", "HDMI"},

	{"HDMI_CH2_MUX", "CH0", "HDMI"},
	{"HDMI_CH2_MUX", "CH1", "HDMI"},
	{"HDMI_CH2_MUX", "CH2", "HDMI"},
	{"HDMI_CH2_MUX", "CH3", "HDMI"},
	{"HDMI_CH2_MUX", "CH4", "HDMI"},
	{"HDMI_CH2_MUX", "CH5", "HDMI"},
	{"HDMI_CH2_MUX", "CH6", "HDMI"},
	{"HDMI_CH2_MUX", "CH7", "HDMI"},

	{"HDMI_CH3_MUX", "CH0", "HDMI"},
	{"HDMI_CH3_MUX", "CH1", "HDMI"},
	{"HDMI_CH3_MUX", "CH2", "HDMI"},
	{"HDMI_CH3_MUX", "CH3", "HDMI"},
	{"HDMI_CH3_MUX", "CH4", "HDMI"},
	{"HDMI_CH3_MUX", "CH5", "HDMI"},
	{"HDMI_CH3_MUX", "CH6", "HDMI"},
	{"HDMI_CH3_MUX", "CH7", "HDMI"},

	{"HDMI_CH4_MUX", "CH0", "HDMI"},
	{"HDMI_CH4_MUX", "CH1", "HDMI"},
	{"HDMI_CH4_MUX", "CH2", "HDMI"},
	{"HDMI_CH4_MUX", "CH3", "HDMI"},
	{"HDMI_CH4_MUX", "CH4", "HDMI"},
	{"HDMI_CH4_MUX", "CH5", "HDMI"},
	{"HDMI_CH4_MUX", "CH6", "HDMI"},
	{"HDMI_CH4_MUX", "CH7", "HDMI"},

	{"HDMI_CH5_MUX", "CH0", "HDMI"},
	{"HDMI_CH5_MUX", "CH1", "HDMI"},
	{"HDMI_CH5_MUX", "CH2", "HDMI"},
	{"HDMI_CH5_MUX", "CH3", "HDMI"},
	{"HDMI_CH5_MUX", "CH4", "HDMI"},
	{"HDMI_CH5_MUX", "CH5", "HDMI"},
	{"HDMI_CH5_MUX", "CH6", "HDMI"},
	{"HDMI_CH5_MUX", "CH7", "HDMI"},

	{"HDMI_CH6_MUX", "CH0", "HDMI"},
	{"HDMI_CH6_MUX", "CH1", "HDMI"},
	{"HDMI_CH6_MUX", "CH2", "HDMI"},
	{"HDMI_CH6_MUX", "CH3", "HDMI"},
	{"HDMI_CH6_MUX", "CH4", "HDMI"},
	{"HDMI_CH6_MUX", "CH5", "HDMI"},
	{"HDMI_CH6_MUX", "CH6", "HDMI"},
	{"HDMI_CH6_MUX", "CH7", "HDMI"},

	{"HDMI_CH7_MUX", "CH0", "HDMI"},
	{"HDMI_CH7_MUX", "CH1", "HDMI"},
	{"HDMI_CH7_MUX", "CH2", "HDMI"},
	{"HDMI_CH7_MUX", "CH3", "HDMI"},
	{"HDMI_CH7_MUX", "CH4", "HDMI"},
	{"HDMI_CH7_MUX", "CH5", "HDMI"},
	{"HDMI_CH7_MUX", "CH6", "HDMI"},
	{"HDMI_CH7_MUX", "CH7", "HDMI"},

	{"HDMI_OUT_MUX", "Connect", "HDMI_CH0_MUX"},
	{"HDMI_OUT_MUX", "Connect", "HDMI_CH1_MUX"},
	{"HDMI_OUT_MUX", "Connect", "HDMI_CH2_MUX"},
	{"HDMI_OUT_MUX", "Connect", "HDMI_CH3_MUX"},
	{"HDMI_OUT_MUX", "Connect", "HDMI_CH4_MUX"},
	{"HDMI_OUT_MUX", "Connect", "HDMI_CH5_MUX"},
	{"HDMI_OUT_MUX", "Connect", "HDMI_CH6_MUX"},
	{"HDMI_OUT_MUX", "Connect", "HDMI_CH7_MUX"},


	{"DPTX_OUT_MUX", "Connect", "HDMI_CH0_MUX"},
	{"DPTX_OUT_MUX", "Connect", "HDMI_CH1_MUX"},
	{"DPTX_OUT_MUX", "Connect", "HDMI_CH2_MUX"},
	{"DPTX_OUT_MUX", "Connect", "HDMI_CH3_MUX"},
	{"DPTX_OUT_MUX", "Connect", "HDMI_CH4_MUX"},
	{"DPTX_OUT_MUX", "Connect", "HDMI_CH5_MUX"},
	{"DPTX_OUT_MUX", "Connect", "HDMI_CH6_MUX"},
	{"DPTX_OUT_MUX", "Connect", "HDMI_CH7_MUX"},

	{"TDM", NULL, "HDMI_OUT_MUX"},
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	{"TDM", NULL, "aud_tdm_clk"},
#endif
	{"TDM", NULL, "TDM_BCK"},
	{"TDM", NULL, "TDM_EN"},

	{"TDM_DPTX", NULL, "DPTX_OUT_MUX"},
#if !defined(CONFIG_FPGA_EARLY_PORTING)
	{"TDM_DPTX", NULL, "aud_tdm_clk"},
#endif
	{"TDM_DPTX", NULL, "TDM_DPTX_BCK"},

	{"TDM_BCK", NULL, "TDM_MCK"},
	{"TDM_DPTX_BCK", NULL, "TDM_DPTX_MCK"},
	{"TDM_MCK", NULL, APLL1_W_NAME, mtk_afe_tdm_apll_connect},
	{"TDM_MCK", NULL, APLL2_W_NAME, mtk_afe_tdm_apll_connect},
	{"TDM_DPTX_MCK", NULL, APLL1_W_NAME, mtk_afe_tdm_apll_connect},
	{"TDM_DPTX_MCK", NULL, APLL2_W_NAME, mtk_afe_tdm_apll_connect},

	{"DPTX_VIRTUAL_OUT_MUX", "Connect", "TDM_DPTX"},
	{"DPTX_VIRTUAL_OUT", NULL, "DPTX_VIRTUAL_OUT_MUX"},
};

/* dai ops */
static int mtk_dai_tdm_cal_mclk(struct mtk_base_afe *afe,
				struct mtk_afe_tdm_priv *tdm_priv,
				int freq)
{
	int apll;
	int apll_rate;

	apll = mt6855_get_apll_by_rate(afe, freq);
	apll_rate = mt6855_get_apll_rate(afe, apll);

	if (freq > apll_rate) {
		AUDIO_AEE("freq > apll rate");
		return -EINVAL;
	}

	if (apll_rate % freq != 0) {
		AUDIO_AEE("APLL cannot generate freq Hz");
		return -EINVAL;
	}

	tdm_priv->mclk_rate = freq;
	tdm_priv->mclk_apll = apll;

	return 0;
}

static int mtk_dai_tdm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt6855_afe_private *afe_priv = afe->platform_priv;
	int tdm_id = dai->id;
	struct mtk_afe_tdm_priv *tdm_priv = afe_priv->dai_priv[tdm_id];
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params);
	snd_pcm_format_t format = params_format(params);
	unsigned int tdm_con = 0;

	/* calculate mclk_rate, if not set explicitly */
	if (!tdm_priv->mclk_rate) {
		tdm_priv->mclk_rate = rate * tdm_priv->mclk_multiple;
		mtk_dai_tdm_cal_mclk(afe,
				     tdm_priv,
				     tdm_priv->mclk_rate);
	}

	/* calculate bck */
	tdm_priv->bck_rate = rate *
			     channels *
			     snd_pcm_format_physical_width(format);

	if (tdm_priv->bck_rate > tdm_priv->mclk_rate)
		AUDIO_AEE("bck_rate > mclk_rate rate");

	if (tdm_priv->mclk_rate % tdm_priv->bck_rate != 0)
		AUDIO_AEE("bck cannot generate");

	dev_info(afe->dev, "%s(), id %d, rate %d, channels %d, format %d, mclk_rate %d, bck_rate %d\n",
		 __func__,
		 tdm_id, rate, channels, format,
		 tdm_priv->mclk_rate, tdm_priv->bck_rate);

	/* set tdm */
	tdm_con = 1 << BCK_INVERSE_SFT;
	tdm_con |= 1 << LRCK_INVERSE_SFT;
	tdm_con |= 1 << DELAY_DATA_SFT;
	tdm_con |= 1 << LEFT_ALIGN_SFT;
	tdm_con |= get_tdm_wlen(format) << WLEN_SFT;
	tdm_con |= get_tdm_ch(channels) << CHANNEL_NUM_SFT;
	tdm_con |= get_tdm_channel_bck(format) << CHANNEL_BCK_CYCLES_SFT;
	tdm_con |= get_tdm_lrck_width(format) << LRCK_TDM_WIDTH_SFT;
	regmap_write(afe->regmap, AFE_TDM_CON1, tdm_con);

	/* set dptx */
	if (tdm_id == MT6855_DAI_TDM_DPTX) {
		regmap_update_bits(afe->regmap, AFE_DPTX_CON,
				   DPTX_CHANNEL_ENABLE_MASK_SFT,
				   get_dptx_ch_enable_mask(channels) <<
				   DPTX_CHANNEL_ENABLE_SFT);
		regmap_update_bits(afe->regmap, AFE_DPTX_CON,
				   DPTX_CHANNEL_NUM_MASK_SFT,
				   get_dptx_ch(channels) <<
				   DPTX_CHANNEL_NUM_SFT);
		regmap_update_bits(afe->regmap, AFE_DPTX_CON,
				   DPTX_16_BIT_MASK_SFT,
				   get_dptx_wlen(format) << DPTX_16_BIT_SFT);
	} else {
		switch (channels) {
		case 1:
		case 2:
			tdm_con = TDM_CH_START_O30_O31 << ST_CH_PAIR_SOUT0_SFT;
			tdm_con |= TDM_CH_ZERO << ST_CH_PAIR_SOUT1_SFT;
			tdm_con |= TDM_CH_ZERO << ST_CH_PAIR_SOUT2_SFT;
			tdm_con |= TDM_CH_ZERO << ST_CH_PAIR_SOUT3_SFT;
			break;
		case 3:
		case 4:
			tdm_con = TDM_CH_START_O30_O31 << ST_CH_PAIR_SOUT0_SFT;
			tdm_con |= TDM_CH_START_O32_O33 << ST_CH_PAIR_SOUT1_SFT;
			tdm_con |= TDM_CH_ZERO << ST_CH_PAIR_SOUT2_SFT;
			tdm_con |= TDM_CH_ZERO << ST_CH_PAIR_SOUT3_SFT;
			break;
		case 5:
		case 6:
			tdm_con = TDM_CH_START_O30_O31 << ST_CH_PAIR_SOUT0_SFT;
			tdm_con |= TDM_CH_START_O32_O33 << ST_CH_PAIR_SOUT1_SFT;
			tdm_con |= TDM_CH_START_O34_O35 << ST_CH_PAIR_SOUT2_SFT;
			tdm_con |= TDM_CH_ZERO << ST_CH_PAIR_SOUT3_SFT;
			break;
		case 7:
		case 8:
			tdm_con = TDM_CH_START_O30_O31 << ST_CH_PAIR_SOUT0_SFT;
			tdm_con |= TDM_CH_START_O32_O33 << ST_CH_PAIR_SOUT1_SFT;
			tdm_con |= TDM_CH_START_O34_O35 << ST_CH_PAIR_SOUT2_SFT;
			tdm_con |= TDM_CH_START_O36_O37 << ST_CH_PAIR_SOUT3_SFT;
			break;
		default:
			tdm_con = 0;
		}
		regmap_write(afe->regmap, AFE_TDM_CON2, tdm_con);
	}

	return 0;
}

static int mtk_dai_tdm_set_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dai->dev);
	struct mt6855_afe_private *afe_priv = afe->platform_priv;
	struct mtk_afe_tdm_priv *tdm_priv = afe_priv->dai_priv[dai->id];

	if (!tdm_priv) {
		AUDIO_AEE("tdm_priv == NULL");
		return -EINVAL;
	}

	if (dir != SND_SOC_CLOCK_OUT) {
		AUDIO_AEE("dir != SND_SOC_CLOCK_OUT");
		return -EINVAL;
	}

	dev_info(afe->dev, "%s(), freq %d\n", __func__, freq);

	return mtk_dai_tdm_cal_mclk(afe, tdm_priv, freq);
}

static const struct snd_soc_dai_ops mtk_dai_tdm_ops = {
	.hw_params = mtk_dai_tdm_hw_params,
	.set_sysclk = mtk_dai_tdm_set_sysclk,
};

/* dai driver */
#define MTK_TDM_RATES (SNDRV_PCM_RATE_8000_48000 |\
		       SNDRV_PCM_RATE_88200 |\
		       SNDRV_PCM_RATE_96000 |\
		       SNDRV_PCM_RATE_176400 |\
		       SNDRV_PCM_RATE_192000)

#define MTK_TDM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mtk_dai_tdm_driver[] = {
	{
		.name = "TDM",
		.id = MT6855_DAI_TDM,
		.playback = {
			.stream_name = "TDM",
			.channels_min = 2,
			.channels_max = 8,
			.rates = MTK_TDM_RATES,
			.formats = MTK_TDM_FORMATS,
		},
		.ops = &mtk_dai_tdm_ops,
	},
	{
		.name = "TDM_DPTX",
		.id = MT6855_DAI_TDM_DPTX,
		.playback = {
			.stream_name = "TDM_DPTX",
			.channels_min = 2,
			.channels_max = 8,
			.rates = MTK_TDM_RATES,
			.formats = MTK_TDM_FORMATS,
		},
		.ops = &mtk_dai_tdm_ops,
	},
};

static struct mtk_afe_tdm_priv *init_tdm_priv_data(struct mtk_base_afe *afe,
		int id)
{
	struct mtk_afe_tdm_priv *tdm_priv;

	tdm_priv = devm_kzalloc(afe->dev, sizeof(struct mtk_afe_tdm_priv),
				GFP_KERNEL);
	if (!tdm_priv)
		return NULL;

	if (id == MT6855_DAI_TDM_DPTX)
		tdm_priv->mclk_multiple = 256;
	else
		tdm_priv->mclk_multiple = 128;

	tdm_priv->bck_id = MT6855_I2S4_BCK;
	tdm_priv->mclk_id = MT6855_I2S4_MCK;

	return tdm_priv;
}

int mt6855_dai_tdm_register(struct mtk_base_afe *afe)
{
	struct mt6855_afe_private *afe_priv = afe->platform_priv;
	struct mtk_afe_tdm_priv *tdm_priv, *tdm_dptx_priv;
	struct mtk_base_afe_dai *dai;

	dev_info(afe->dev, "%s()\n", __func__);

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mtk_dai_tdm_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_tdm_driver);

	dai->dapm_widgets = mtk_dai_tdm_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mtk_dai_tdm_widgets);
	dai->dapm_routes = mtk_dai_tdm_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_tdm_routes);

	tdm_priv = init_tdm_priv_data(afe, MT6855_DAI_TDM);
	if (!tdm_priv)
		return -ENOMEM;

	tdm_dptx_priv = init_tdm_priv_data(afe, MT6855_DAI_TDM_DPTX);
	if (!tdm_dptx_priv)
		return -ENOMEM;

	afe_priv->dai_priv[MT6855_DAI_TDM] = tdm_priv;
	afe_priv->dai_priv[MT6855_DAI_TDM_DPTX] = tdm_dptx_priv;

	return 0;
}
