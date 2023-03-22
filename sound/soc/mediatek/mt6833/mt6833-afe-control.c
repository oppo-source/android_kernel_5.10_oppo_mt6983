// SPDX-License-Identifier: GPL-2.0
/*
 *  MediaTek ALSA SoC Audio Control
 *
 *  Copyright (c) 2021 MediaTek Inc.
 *  Author: Yujie Xiao <yujie.xiao@mediatek.com>
 */

#include "mt6833-afe-common.h"
#include <linux/pm_runtime.h>

#include "../common/mtk-sram-manager.h"

/* don't use this directly if not necessary */
static struct mtk_base_afe *local_afe;

int mt6833_set_local_afe(struct mtk_base_afe *afe)
{
	local_afe = afe;
	return 0;
}

unsigned int mt6833_general_rate_transform(struct device *dev,
					   unsigned int rate)
{
	switch (rate) {
	case 8000:
		return MTK_AFE_RATE_8K;
	case 11025:
		return MTK_AFE_RATE_11K;
	case 12000:
		return MTK_AFE_RATE_12K;
	case 16000:
		return MTK_AFE_RATE_16K;
	case 22050:
		return MTK_AFE_RATE_22K;
	case 24000:
		return MTK_AFE_RATE_24K;
	case 32000:
		return MTK_AFE_RATE_32K;
	case 44100:
		return MTK_AFE_RATE_44K;
	case 48000:
		return MTK_AFE_RATE_48K;
	case 88200:
		return MTK_AFE_RATE_88K;
	case 96000:
		return MTK_AFE_RATE_96K;
	case 176400:
		return MTK_AFE_RATE_176K;
	case 192000:
		return MTK_AFE_RATE_192K;
	case 260000:
		return MTK_AFE_RATE_260K;
	case 352800:
		return MTK_AFE_RATE_352K;
	case 384000:
		return MTK_AFE_RATE_384K;
	default:
		dev_warn(dev, "%s(), rate %u invalid, use %d!!!\n",
			 __func__,
			 rate, MTK_AFE_RATE_48K);
		return MTK_AFE_RATE_48K;
	}
}

static unsigned int dai_memif_rate_transform(struct device *dev,
					     unsigned int rate)
{
	switch (rate) {
	case 8000:
		return MTK_AFE_DAI_MEMIF_RATE_8K;
	case 16000:
		return MTK_AFE_DAI_MEMIF_RATE_16K;
	case 32000:
		return MTK_AFE_DAI_MEMIF_RATE_32K;
	case 48000:
		return MTK_AFE_DAI_MEMIF_RATE_48K;
	default:
		dev_warn(dev, "%s(), rate %u invalid, use %d!!!\n",
			 __func__,
			 rate, MTK_AFE_DAI_MEMIF_RATE_16K);
		return MTK_AFE_DAI_MEMIF_RATE_16K;
	}
}

static unsigned int pcm_rate_transform(struct device *dev,
				       unsigned int rate)
{
	switch (rate) {
	case 8000:
		return MTK_AFE_PCM_RATE_8K;
	case 16000:
		return MTK_AFE_PCM_RATE_16K;
	case 32000:
		return MTK_AFE_PCM_RATE_32K;
	case 48000:
		return MTK_AFE_PCM_RATE_48K;
	default:
		dev_warn(dev, "%s(), rate %u invalid, use %d!!!\n",
			 __func__,
			 rate, MTK_AFE_PCM_RATE_32K);
		return MTK_AFE_PCM_RATE_32K;
	}
}

unsigned int mt6833_rate_transform(struct device *dev,
				   unsigned int rate, int aud_blk)
{
	switch (aud_blk) {
	case MT6833_MEMIF_DAI:
	case MT6833_MEMIF_MOD_DAI:
		return dai_memif_rate_transform(dev, rate);
	case MT6833_DAI_PCM_1:
	case MT6833_DAI_PCM_2:
		return pcm_rate_transform(dev, rate);
	default:
		return mt6833_general_rate_transform(dev, rate);
	}
}

int mt6833_dai_set_priv(struct mtk_base_afe *afe, int id,
			int priv_size, const void *priv_data)
{
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	void *temp_data;

	temp_data = devm_kzalloc(afe->dev,
				 priv_size,
				 GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;

	if (priv_data)
		memcpy(temp_data, priv_data, priv_size);

	if (id >= 0)
		afe_priv->dai_priv[id] = temp_data;

	return 0;
}

/* DC compensation */
int mt6833_enable_dc_compensation(bool enable)
{
	if (!local_afe)
		return -EPERM;

	if (pm_runtime_status_suspended(local_afe->dev))
		dev_warn(local_afe->dev, "%s(), status suspended\n", __func__);


	pm_runtime_get_sync(local_afe->dev);
	regmap_update_bits(local_afe->regmap,
			   AFE_ADDA_DL_SDM_DCCOMP_CON,
			   AUD_DC_COMP_EN_MASK_SFT,
			   (enable ? 1 : 0) << AUD_DC_COMP_EN_SFT);
	pm_runtime_put(local_afe->dev);
	return 0;
}
EXPORT_SYMBOL(mt6833_enable_dc_compensation);

int mt6833_set_lch_dc_compensation(int value)
{
	if (!local_afe)
		return -EPERM;

	if (pm_runtime_status_suspended(local_afe->dev))
		dev_warn(local_afe->dev, "%s(), status suspended\n", __func__);

	pm_runtime_get_sync(local_afe->dev);
	regmap_write(local_afe->regmap,
		     AFE_ADDA_DL_DC_COMP_CFG0,
		     value);
	pm_runtime_put(local_afe->dev);
	return 0;
}
EXPORT_SYMBOL(mt6833_set_lch_dc_compensation);

int mt6833_set_rch_dc_compensation(int value)
{
	if (!local_afe)
		return -EPERM;

	if (pm_runtime_status_suspended(local_afe->dev))
		dev_warn(local_afe->dev, "%s(), status suspended\n", __func__);

	pm_runtime_get_sync(local_afe->dev);
	regmap_write(local_afe->regmap,
		     AFE_ADDA_DL_DC_COMP_CFG1,
		     value);
	pm_runtime_put(local_afe->dev);
	return 0;
}
EXPORT_SYMBOL(mt6833_set_rch_dc_compensation);

int mt6833_adda_dl_gain_control(bool mute)
{
	unsigned int dl_2_gain_ctl;

	if (!local_afe)
		return -EPERM;

	if (pm_runtime_status_suspended(local_afe->dev))
		dev_warn(local_afe->dev, "%s(), status suspended\n", __func__);

	pm_runtime_get_sync(local_afe->dev);

	if (mute)
		dl_2_gain_ctl = MTK_AFE_ADDA_DL_GAIN_MUTE;
	else
		dl_2_gain_ctl = MTK_AFE_ADDA_DL_GAIN_NORMAL;

	regmap_update_bits(local_afe->regmap,
			   AFE_ADDA_DL_SRC2_CON1,
			   DL_2_GAIN_CTL_PRE_MASK_SFT,
			   dl_2_gain_ctl << DL_2_GAIN_CTL_PRE_SFT);

	dev_info(local_afe->dev, "%s(), adda_dl_gain %x\n",
		 __func__, dl_2_gain_ctl);

	pm_runtime_put(local_afe->dev);
	return 0;
}
EXPORT_SYMBOL(mt6833_adda_dl_gain_control);
