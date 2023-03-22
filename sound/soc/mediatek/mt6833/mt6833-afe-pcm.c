// SPDX-License-Identifier: GPL-2.0
/*
 *  Mediatek ALSA SoC AFE platform driver for 6833
 *
 *  Copyright (c) 2021 MediaTek Inc.
 *  Author: Yujie Xiao <yujie.xiao@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <linux/arm-smccc.h> /* for Kernel Native SMC API */
#include <linux/soc/mediatek/mtk_sip_svc.h> /* for SMC ID table */

#include "../common/mtk-afe-debug.h"
#include "../common/mtk-afe-platform-driver.h"
#include "../common/mtk-afe-fe-dai.h"
#include "../common/mtk-sp-pcm-ops.h"
#include "../common/mtk-sram-manager.h"
#include "../common/mtk-mmap-ion.h"

#include "mt6833-afe-common.h"
#include "mt6833-afe-clk.h"
#include "mt6833-afe-gpio.h"
#include "mt6833-interconnection.h"
#if IS_ENABLED(CONFIG_SND_SOC_MTK_AUDIO_DSP)
#include "../audio_dsp/mtk-dsp-common.h"
#endif

#define SKIP_SWRDG
#if IS_ENABLED(CONFIG_MTK_ULTRASND_PROXIMITY) && !defined(SKIP_SWRDG)
#include "../ultrasound/ultra_scp/mtk-scp-ultra-common.h"
#endif
/* FORCE_FPGA_ENABLE_IRQ use irq in fpga */
/* #define FORCE_FPGA_ENABLE_IRQ */


#define AFE_SYS_DEBUG_SIZE (1024 * 32) // 32K
#define MAX_DEBUG_WRITE_INPUT 256

static ssize_t mt6833_debug_read_reg(char *buffer, int size, struct mtk_base_afe *afe);

static const struct snd_pcm_hardware mt6833_afe_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_NO_PERIOD_WAKEUP |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE |
		    SNDRV_PCM_FMTBIT_S24_LE |
		    SNDRV_PCM_FMTBIT_S32_LE),
	.period_bytes_min = 96,
	.period_bytes_max = 4 * 48 * 1024,
	.periods_min = 2,
	.periods_max = 256,
	.buffer_bytes_max = 4 * 48 * 1024,
	.fifo_size = 0,
};

static int mt6833_fe_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	int memif_num = cpu_dai->id;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];
	const struct snd_pcm_hardware *mtk_afe_hardware = afe->mtk_afe_hardware;
	int ret;

	memif->substream = substream;

	snd_pcm_hw_constraint_step(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 16);

	snd_soc_set_runtime_hwparams(substream, mtk_afe_hardware);

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		dev_err(afe->dev, "snd_pcm_hw_constraint_integer failed\n");

	/* dynamic allocate irq to memif */
	if (memif->irq_usage < 0) {
		int irq_id = mtk_dynamic_irq_acquire(afe);

		if (irq_id != afe->irqs_size) {
			/* link */
			memif->irq_usage = irq_id;
		} else {
			dev_err(afe->dev, "%s() error: no more asys irq\n",
				__func__);
			ret = -EBUSY;
		}
	}

	return ret;
}

void mt6833_fe_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	int memif_num = cpu_dai->id;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];
	int irq_id = memif->irq_usage;

	memif->substream = NULL;
	afe_priv->irq_cnt[memif_num] = 0;
	afe_priv->xrun_assert[memif_num] = 0;

	if (!memif->const_irq) {
		mtk_dynamic_irq_release(afe, irq_id);
		memif->irq_usage = -1;
		memif->substream = NULL;
	}
}

int mt6833_fe_trigger(struct snd_pcm_substream *substream, int cmd,
		      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime * const runtime = substream->runtime;
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	int id = cpu_dai->id;
	struct mtk_base_afe_memif *memif = &afe->memif[id];
	int irq_id = memif->irq_usage;
	struct mtk_base_afe_irq *irqs = &afe->irqs[irq_id];
	const struct mtk_base_irq_data *irq_data = irqs->irq_data;
	unsigned int counter = runtime->period_size;
	unsigned int rate = runtime->rate;
	int fs;
	int ret = 0;

	if (!in_interrupt())
		dev_info(afe->dev,
			 "%s(), %s cmd %d, irq_id %d, is_afe_need_triggered %d, no_period_wakeup %d\n",
			 __func__, memif->data->name, cmd, irq_id,
			 is_afe_need_triggered(memif),
			 runtime->no_period_wakeup);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (is_afe_need_triggered(memif)) {
			ret = mtk_memif_set_enable(afe, id);
			if (ret) {
				dev_err(afe->dev,
					"%s(), error, id %d, memif enable, ret %d\n",
					__func__, id, ret);
				return ret;
			}
		}

		/*
		 * for small latency record
		 * ul memif need read some data before irq enable
		 */
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			if ((runtime->period_size * 1000) / rate <= 10)
				udelay(300);
		}

		/* set irq counter */
		if (afe_priv->irq_cnt[id] > 0)
			counter = afe_priv->irq_cnt[id];

		mtk_regmap_update_bits(afe->regmap, irq_data->irq_cnt_reg,
				   irq_data->irq_cnt_maskbit,
				   counter, irq_data->irq_cnt_shift);

		/* set irq fs */
		fs = afe->irq_fs(substream, runtime->rate);
		if (fs < 0)
			return -EINVAL;

		mtk_regmap_update_bits(afe->regmap, irq_data->irq_fs_reg,
				   irq_data->irq_fs_maskbit,
				   fs, irq_data->irq_fs_shift);

		if (!runtime->no_period_wakeup)
			mtk_irq_set_enable(afe, irq_data, id);

		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (afe_priv->xrun_assert[id] > 0) {
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
				int avail = snd_pcm_capture_avail(runtime);

				if (avail >= runtime->buffer_size) {
					dev_warn(afe->dev, "%s(), id %d, xrun assert\n",
						 __func__, id);
					AUDIO_AEE("xrun assert");
				}
			}
		}

		if (is_afe_need_triggered(memif)) {
			ret = mtk_memif_set_disable(afe, id);
			if (ret) {
				dev_err(afe->dev,
					"%s(), error, id %d, memif enable, ret %d\n",
					__func__, id, ret);
			}
		}

		if (!runtime->no_period_wakeup) {
			/* disable interrupt */
			mtk_irq_set_disable(afe, irq_data, id);

			/* clear pending IRQ */
			regmap_write(afe->regmap, irq_data->irq_clr_reg,
				     1 << irq_data->irq_clr_shift);
		}

		return ret;
	default:
		return -EINVAL;
	}
}

static int mt6833_memif_fs(struct snd_pcm_substream *substream,
			   unsigned int rate)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	int id = cpu_dai->id;

	return mt6833_rate_transform(afe->dev, rate, id);
}

static int mt6833_get_dai_fs(struct mtk_base_afe *afe,
			     int dai_id, unsigned int rate)
{
	return mt6833_rate_transform(afe->dev, rate, dai_id);
}

static int mt6833_irq_fs(struct snd_pcm_substream *substream, unsigned int rate)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);

	return mt6833_general_rate_transform(afe->dev, rate);
}

int mt6833_get_memif_pbuf_size(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	if ((runtime->period_size * 1000) / runtime->rate > 10)
		return MT6833_MEMIF_PBUF_SIZE_256_BYTES;
	else
		return MT6833_MEMIF_PBUF_SIZE_32_BYTES;
}

/* FE DAIs */
static const struct snd_soc_dai_ops mt6833_memif_dai_ops = {
	.startup	= mt6833_fe_startup,
	.shutdown	= mt6833_fe_shutdown,
	.hw_params	= mtk_afe_fe_hw_params,
	.hw_free	= mtk_afe_fe_hw_free,
	.prepare	= mtk_afe_fe_prepare,
	.trigger	= mt6833_fe_trigger,
};

#define MTK_PCM_RATES (SNDRV_PCM_RATE_8000_48000 |\
				SNDRV_PCM_RATE_88200 |\
				SNDRV_PCM_RATE_96000 |\
				SNDRV_PCM_RATE_176400 |\
				SNDRV_PCM_RATE_192000)

#define MTK_PCM_DAI_RATES (SNDRV_PCM_RATE_8000 |\
				SNDRV_PCM_RATE_16000 |\
				SNDRV_PCM_RATE_32000 |\
				SNDRV_PCM_RATE_48000)

#define MTK_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
				SNDRV_PCM_FMTBIT_S24_LE |\
				SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mt6833_memif_dai_driver[] = {
	/* FE DAIs: memory intefaces to CPU */
	{
		.name = "DL1",
		.id = MT6833_MEMIF_DL1,
		.playback = {
			.stream_name = "DL1",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "DL12",
		.id = MT6833_MEMIF_DL12,
		.playback = {
			.stream_name = "DL12",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "DL2",
		.id = MT6833_MEMIF_DL2,
		.playback = {
			.stream_name = "DL2",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "DL3",
		.id = MT6833_MEMIF_DL3,
		.playback = {
			.stream_name = "DL3",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "DL4",
		.id = MT6833_MEMIF_DL4,
		.playback = {
			.stream_name = "DL4",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "DL5",
		.id = MT6833_MEMIF_DL5,
		.playback = {
			.stream_name = "DL5",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "DL6",
		.id = MT6833_MEMIF_DL6,
		.playback = {
			.stream_name = "DL6",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "DL7",
		.id = MT6833_MEMIF_DL7,
		.playback = {
			.stream_name = "DL7",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "DL8",
		.id = MT6833_MEMIF_DL8,
		.playback = {
			.stream_name = "DL8",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "DL9",
		.id = MT6833_MEMIF_DL9,
		.playback = {
			.stream_name = "DL9",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL1",
		.id = MT6833_MEMIF_VUL12,
		.capture = {
			.stream_name = "UL1",
			.channels_min = 1,
			.channels_max = 4,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL2",
		.id = MT6833_MEMIF_AWB,
		.capture = {
			.stream_name = "UL2",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL3",
		.id = MT6833_MEMIF_VUL2,
		.capture = {
			.stream_name = "UL3",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL4",
		.id = MT6833_MEMIF_AWB2,
		.capture = {
			.stream_name = "UL4",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL5",
		.id = MT6833_MEMIF_VUL3,
		.capture = {
			.stream_name = "UL5",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL6",
		.id = MT6833_MEMIF_VUL4,
		.capture = {
			.stream_name = "UL6",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL7",
		.id = MT6833_MEMIF_VUL5,
		.capture = {
			.stream_name = "UL7",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL8",
		.id = MT6833_MEMIF_VUL6,
		.capture = {
			.stream_name = "UL8",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL_MONO_1",
		.id = MT6833_MEMIF_MOD_DAI,
		.capture = {
			.stream_name = "UL_MONO_1",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_DAI_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL_MONO_2",
		.id = MT6833_MEMIF_DAI,
		.capture = {
			.stream_name = "UL_MONO_2",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_DAI_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
	{
		.name = "UL_MONO_3",
		.id = MT6833_MEMIF_DAI2,
		.capture = {
			.stream_name = "UL_MONO_3",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_DAI_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mt6833_memif_dai_ops,
	},
};

/* kcontrol */
static int mt6833_irq_cnt1_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] =
		afe_priv->irq_cnt[MT6833_PRIMARY_MEMIF];
	return 0;
}

static int mt6833_irq_cnt1_set(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_PRIMARY_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];
	int irq_id = memif->irq_usage;
	int irq_cnt = afe_priv->irq_cnt[memif_num];

	dev_info(afe->dev, "%s(), irq_id %d, irq_cnt = %d, value = %ld\n",
		 __func__,
		 irq_id, irq_cnt,
		 ucontrol->value.integer.value[0]);

	if (irq_cnt == ucontrol->value.integer.value[0])
		return 0;

	irq_cnt = ucontrol->value.integer.value[0];
	afe_priv->irq_cnt[memif_num] = irq_cnt;

	if (pm_runtime_status_suspended(afe->dev) || irq_id < 0) {
		dev_info(afe->dev, "%s(), suspended || irq_id %d, not set\n",
			 __func__, irq_id);
	} else {
		struct mtk_base_afe_irq *irqs = &afe->irqs[irq_id];
		const struct mtk_base_irq_data *irq_data = irqs->irq_data;

		regmap_update_bits(afe->regmap, irq_data->irq_cnt_reg,
				   irq_data->irq_cnt_maskbit
				   << irq_data->irq_cnt_shift,
				   irq_cnt << irq_data->irq_cnt_shift);
	}

	return 0;
}

static int mt6833_irq_cnt2_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] =
		afe_priv->irq_cnt[MT6833_RECORD_MEMIF];
	return 0;
}

static int mt6833_irq_cnt2_set(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_RECORD_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];
	int irq_id = memif->irq_usage;
	int irq_cnt = afe_priv->irq_cnt[memif_num];

	dev_info(afe->dev, "%s(), irq_id %d, irq_cnt = %d, value = %ld\n",
		 __func__,
		 irq_id, irq_cnt,
		 ucontrol->value.integer.value[0]);

	if (irq_cnt == ucontrol->value.integer.value[0])
		return 0;

	irq_cnt = ucontrol->value.integer.value[0];
	afe_priv->irq_cnt[memif_num] = irq_cnt;

	if (pm_runtime_status_suspended(afe->dev) || irq_id < 0) {
		dev_info(afe->dev, "%s(), suspended || irq_id %d, not set\n",
			 __func__, irq_id);
	} else {
		struct mtk_base_afe_irq *irqs = &afe->irqs[irq_id];
		const struct mtk_base_irq_data *irq_data = irqs->irq_data;

		regmap_update_bits(afe->regmap, irq_data->irq_cnt_reg,
				   irq_data->irq_cnt_maskbit
				   << irq_data->irq_cnt_shift,
				   irq_cnt << irq_data->irq_cnt_shift);
	}

	return 0;
}

static int mt6833_deep_irq_cnt_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] = afe_priv->irq_cnt[MT6833_DEEP_MEMIF];
	return 0;
}

static int mt6833_deep_irq_cnt_set(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_DEEP_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];
	int irq_id = memif->irq_usage;
	int irq_cnt = afe_priv->irq_cnt[memif_num];

	dev_info(afe->dev, "%s(), irq_id %d, irq_cnt = %d, value = %ld\n",
		 __func__,
		 irq_id, irq_cnt,
		 ucontrol->value.integer.value[0]);

	if (irq_cnt == ucontrol->value.integer.value[0])
		return 0;

	irq_cnt = ucontrol->value.integer.value[0];
	afe_priv->irq_cnt[memif_num] = irq_cnt;

	if (pm_runtime_status_suspended(afe->dev) || irq_id < 0) {
		dev_info(afe->dev, "%s(), suspended || irq_id %d, not set\n",
			 __func__, irq_id);
	} else {
		struct mtk_base_afe_irq *irqs = &afe->irqs[irq_id];
		const struct mtk_base_irq_data *irq_data = irqs->irq_data;

		regmap_update_bits(afe->regmap, irq_data->irq_cnt_reg,
				   irq_data->irq_cnt_maskbit
				   << irq_data->irq_cnt_shift,
				   irq_cnt << irq_data->irq_cnt_shift);
	}

	return 0;
}

static int mt6833_voip_rx_irq_cnt_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] = afe_priv->irq_cnt[MT6833_VOIP_MEMIF];
	return 0;
}

static int mt6833_voip_rx_irq_cnt_set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_VOIP_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];
	int irq_id = memif->irq_usage;
	int irq_cnt = afe_priv->irq_cnt[memif_num];

	dev_info(afe->dev, "%s(), irq_id %d, irq_cnt = %d, value = %ld\n",
		 __func__,
		 irq_id, irq_cnt,
		 ucontrol->value.integer.value[0]);

	if (irq_cnt == ucontrol->value.integer.value[0])
		return 0;

	irq_cnt = ucontrol->value.integer.value[0];
	afe_priv->irq_cnt[memif_num] = irq_cnt;

	if (pm_runtime_status_suspended(afe->dev) || irq_id < 0) {
		dev_info(afe->dev, "%s(), suspended || irq_id %d, not set\n",
			 __func__, irq_id);
	} else {
		struct mtk_base_afe_irq *irqs = &afe->irqs[irq_id];
		const struct mtk_base_irq_data *irq_data = irqs->irq_data;

		regmap_update_bits(afe->regmap, irq_data->irq_cnt_reg,
				   irq_data->irq_cnt_maskbit
				   << irq_data->irq_cnt_shift,
				   irq_cnt << irq_data->irq_cnt_shift);
	}

	return 0;
}

static int mt6833_deep_scene_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] = afe_priv->deep_playback_state;
	return 0;
}

static int mt6833_deep_scene_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_DEEP_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];

	afe_priv->deep_playback_state = ucontrol->value.integer.value[0];

	if (afe_priv->deep_playback_state == 1)
		memif->ack_enable = true;
	else
		memif->ack_enable = false;

	return 0;
}

static int mt6833_fast_scene_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] = afe_priv->fast_playback_state;
	return 0;
}

static int mt6833_fast_scene_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_FAST_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];

	afe_priv->fast_playback_state = ucontrol->value.integer.value[0];

	if (afe_priv->fast_playback_state == 1)
		memif->use_dram_only = 1;
	else
		memif->use_dram_only = 0;

	return 0;
}

static int mt6833_primary_scene_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] = afe_priv->primary_playback_state;
	return 0;
}

static int mt6833_primary_scene_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_PRIMARY_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];

	afe_priv->primary_playback_state = ucontrol->value.integer.value[0];

	if (afe_priv->primary_playback_state == 1)
		memif->use_dram_only = 1;
	else
		memif->use_dram_only = 0;

	return 0;
}

static int mt6833_voip_scene_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] = afe_priv->voip_rx_state;
	return 0;
}

static int mt6833_voip_scene_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_VOIP_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];

	afe_priv->voip_rx_state = ucontrol->value.integer.value[0];

	if (afe_priv->voip_rx_state == 1)
		memif->use_dram_only = 1;
	else
		memif->use_dram_only = 0;

	return 0;
}

static int mt6833_record_xrun_assert_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int xrun_assert = afe_priv->xrun_assert[MT6833_RECORD_MEMIF];

	ucontrol->value.integer.value[0] = xrun_assert;
	return 0;
}

static int mt6833_record_xrun_assert_set(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int xrun_assert = ucontrol->value.integer.value[0];

	dev_info(afe->dev, "%s(), xrun_assert %d\n", __func__, xrun_assert);
	afe_priv->xrun_assert[MT6833_RECORD_MEMIF] = xrun_assert;
	return 0;
}

static int mt6833_echo_ref_xrun_assert_get(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int xrun_assert = afe_priv->xrun_assert[MT6833_ECHO_REF_MEMIF];

	ucontrol->value.integer.value[0] = xrun_assert;
	return 0;
}

static int mt6833_echo_ref_xrun_assert_set(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int xrun_assert = ucontrol->value.integer.value[0];

	dev_info(afe->dev, "%s(), xrun_assert %d\n", __func__, xrun_assert);
	afe_priv->xrun_assert[MT6833_ECHO_REF_MEMIF] = xrun_assert;
	return 0;
}

static int mt6833_sram_size_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mtk_audio_sram *sram = afe->sram;

	ucontrol->value.integer.value[0] =
		mtk_audio_sram_get_size(sram, sram->prefer_mode);

	return 0;
}

static int mt6833_vow_barge_in_irq_id_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	int memif_num = MT6833_BARGE_INMEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];
	int irq_id = memif->irq_usage;

	ucontrol->value.integer.value[0] = irq_id;
	return 0;
}

static int mt6833_mmap_dl_scene_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] = afe_priv->mmap_playback_state;
	return 0;
}

static int mt6833_mmap_dl_scene_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_MMAP_DL_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];

	afe_priv->mmap_playback_state = ucontrol->value.integer.value[0];

	if (afe_priv->mmap_playback_state == 1) {
		unsigned long phy_addr;
		void *vir_addr;

		mtk_get_mmap_dl_buffer(&phy_addr, &vir_addr);

		if (phy_addr != 0x0 && vir_addr)
			memif->use_mmap_share_mem = 1;
	} else {
		memif->use_mmap_share_mem = 0;
	}

	dev_info(afe->dev, "%s(), state %d, mem %d\n", __func__,
		 afe_priv->mmap_playback_state, memif->use_mmap_share_mem);
	return 0;
}

static int mt6833_mmap_ul_scene_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	ucontrol->value.integer.value[0] = afe_priv->mmap_record_state;
	return 0;
}

static int mt6833_mmap_ul_scene_set(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int memif_num = MT6833_MMAP_UL_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];

	afe_priv->mmap_record_state = ucontrol->value.integer.value[0];

	if (afe_priv->mmap_record_state == 1) {
		unsigned long phy_addr;
		void *vir_addr;

		mtk_get_mmap_ul_buffer(&phy_addr, &vir_addr);

		if (phy_addr != 0x0 && vir_addr)
			memif->use_mmap_share_mem = 2;
	} else {
		memif->use_mmap_share_mem = 0;
	}

	dev_info(afe->dev, "%s(), state %d, mem %d\n", __func__,
		 afe_priv->mmap_record_state, memif->use_mmap_share_mem);
	return 0;
}

static int mt6833_mmap_ion_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

static int mt6833_mmap_ion_set(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);

	dev_info(afe->dev, "%s()\n", __func__);
	mtk_exporter_init(afe->dev);
	return 0;
}

static int mt6833_dl_mmap_fd_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	int memif_num = MT6833_MMAP_DL_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];

	ucontrol->value.integer.value[0] = (memif->use_mmap_share_mem == 1) ?
					    mtk_get_mmap_dl_fd() : 0;
	dev_info(afe->dev, "%s, fd %ld\n", __func__,
		 ucontrol->value.integer.value[0]);
	return 0;
}

static int mt6833_dl_mmap_fd_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int mt6833_ul_mmap_fd_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	int memif_num = MT6833_MMAP_UL_MEMIF;
	struct mtk_base_afe_memif *memif = &afe->memif[memif_num];

	ucontrol->value.integer.value[0] = (memif->use_mmap_share_mem == 2) ?
					    mtk_get_mmap_ul_fd() : 0;
	dev_info(afe->dev, "%s, fd %ld\n", __func__,
		 ucontrol->value.integer.value[0]);
	return 0;
}

static int mt6833_ul_mmap_fd_set(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}


static const struct snd_kcontrol_new mt6833_pcm_kcontrols[] = {
	SOC_SINGLE_EXT("Audio IRQ1 CNT", SND_SOC_NOPM, 0, 0x3ffff, 0,
		       mt6833_irq_cnt1_get, mt6833_irq_cnt1_set),
	SOC_SINGLE_EXT("Audio IRQ2 CNT", SND_SOC_NOPM, 0, 0x3ffff, 0,
		       mt6833_irq_cnt2_get, mt6833_irq_cnt2_set),
	SOC_SINGLE_EXT("deep_buffer_irq_cnt", SND_SOC_NOPM, 0, 0x3ffff, 0,
		       mt6833_deep_irq_cnt_get, mt6833_deep_irq_cnt_set),
	SOC_SINGLE_EXT("voip_rx_irq_cnt", SND_SOC_NOPM, 0, 0x3ffff, 0,
		       mt6833_voip_rx_irq_cnt_get, mt6833_voip_rx_irq_cnt_set),
	SOC_SINGLE_EXT("deep_buffer_scenario", SND_SOC_NOPM, 0, 0x1, 0,
		       mt6833_deep_scene_get, mt6833_deep_scene_set),
	SOC_SINGLE_EXT("record_xrun_assert", SND_SOC_NOPM, 0, 0x1, 0,
		       mt6833_record_xrun_assert_get,
		       mt6833_record_xrun_assert_set),
	SOC_SINGLE_EXT("echo_ref_xrun_assert", SND_SOC_NOPM, 0, 0x1, 0,
		       mt6833_echo_ref_xrun_assert_get,
		       mt6833_echo_ref_xrun_assert_set),
	SOC_SINGLE_EXT("fast_play_scenario", SND_SOC_NOPM, 0, 0x1, 0,
		       mt6833_fast_scene_get, mt6833_fast_scene_set),
	SOC_SINGLE_EXT("primary_play_scenario", SND_SOC_NOPM, 0, 0x1, 0,
		       mt6833_primary_scene_get, mt6833_primary_scene_set),
	SOC_SINGLE_EXT("voip_rx_scenario", SND_SOC_NOPM, 0, 0x1, 0,
		       mt6833_voip_scene_get, mt6833_voip_scene_set),
	SOC_SINGLE_EXT("sram_size", SND_SOC_NOPM, 0, 0xffffffff, 0,
		       mt6833_sram_size_get, NULL),
	SOC_SINGLE_EXT("vow_barge_in_irq_id", SND_SOC_NOPM, 0, 0x3ffff, 0,
			   mt6833_vow_barge_in_irq_id_get, NULL),
	SOC_SINGLE_EXT("mmap_play_scenario", SND_SOC_NOPM, 0, 0x1, 0,
		       mt6833_mmap_dl_scene_get, mt6833_mmap_dl_scene_set),
	SOC_SINGLE_EXT("mmap_record_scenario", SND_SOC_NOPM, 0, 0x1, 0,
		       mt6833_mmap_ul_scene_get, mt6833_mmap_ul_scene_set),
	SOC_SINGLE_EXT("aaudio_ion",
		       SND_SOC_NOPM, 0, 0xffffffff, 0,
		       mt6833_mmap_ion_get,
		       mt6833_mmap_ion_set),
	SOC_SINGLE_EXT("aaudio_dl_mmap_fd",
		       SND_SOC_NOPM, 0, 0xffffffff, 0,
		       mt6833_dl_mmap_fd_get,
		       mt6833_dl_mmap_fd_set),
	SOC_SINGLE_EXT("aaudio_ul_mmap_fd",
		       SND_SOC_NOPM, 0, 0xffffffff, 0,
		       mt6833_ul_mmap_fd_get,
		       mt6833_ul_mmap_fd_set),
};

/* dma widget & routes*/
static const struct snd_kcontrol_new memif_ul1_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN21,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN21,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul1_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN22,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN22,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul1_ch3_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN9,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN9,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul1_ch4_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN10,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN10,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul2_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("I2S0_CH1", AFE_CONN5,
				    I_I2S0_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH1", AFE_CONN5,
				    I_DL1_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL12_CH1", AFE_CONN5,
				    I_DL12_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL2_CH1", AFE_CONN5,
				    I_DL2_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL3_CH1", AFE_CONN5,
				    I_DL3_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL4_CH1", AFE_CONN5_1,
				    I_DL4_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL5_CH1", AFE_CONN5_1,
				    I_DL5_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL6_CH1", AFE_CONN5_1,
				    I_DL6_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL7_CH1", AFE_CONN5_1,
				    I_DL7_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_1_CAP_CH1", AFE_CONN5,
				    I_PCM_1_CAP_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_2_CAP_CH1", AFE_CONN5,
				    I_PCM_2_CAP_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I2S2_CH1", AFE_CONN5,
				    I_I2S2_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("CONNSYS_I2S_CH1", AFE_CONN5_1,
				    I_CONNSYS_I2S_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("SRC_1_OUT_CH1", AFE_CONN5_1,
				    I_SRC_1_OUT_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul2_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("I2S0_CH2", AFE_CONN6,
				    I_I2S0_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH2", AFE_CONN6,
				    I_DL1_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL12_CH2", AFE_CONN6,
				    I_DL12_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL2_CH2", AFE_CONN6,
				    I_DL2_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL3_CH2", AFE_CONN6,
				    I_DL3_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL4_CH2", AFE_CONN6_1,
				    I_DL4_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL5_CH2", AFE_CONN6_1,
				    I_DL5_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL6_CH2", AFE_CONN6_1,
				    I_DL6_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL7_CH2", AFE_CONN6_1,
				    I_DL7_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_1_CAP_CH1", AFE_CONN6,
				    I_PCM_1_CAP_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_2_CAP_CH1", AFE_CONN6,
				    I_PCM_2_CAP_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I2S2_CH2", AFE_CONN6,
				    I_I2S2_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("CONNSYS_I2S_CH2", AFE_CONN6_1,
				    I_CONNSYS_I2S_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("SRC_1_OUT_CH2", AFE_CONN6_1,
				    I_SRC_1_OUT_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul3_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("CONNSYS_I2S_CH1", AFE_CONN32_1,
				    I_CONNSYS_I2S_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH1", AFE_CONN32,
				    I_DL1_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL2_CH1", AFE_CONN32,
				    I_DL2_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul3_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("CONNSYS_I2S_CH2", AFE_CONN33_1,
				    I_CONNSYS_I2S_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul4_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN38,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I2S0_CH1", AFE_CONN38,
				    I_I2S0_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul4_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN39,
				    I_ADDA_UL_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("I2S0_CH2", AFE_CONN39,
				    I_I2S0_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul5_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN44,
				    I_ADDA_UL_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul5_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN45,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul6_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN46,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH1", AFE_CONN46,
				    I_DL1_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL12_CH1", AFE_CONN46,
				    I_DL12_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL6_CH1", AFE_CONN46_1,
				    I_DL6_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL2_CH1", AFE_CONN46,
				    I_DL2_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL3_CH1", AFE_CONN46,
				    I_DL3_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL4_CH1", AFE_CONN46_1,
				    I_DL4_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_1_CAP_CH1", AFE_CONN46,
				    I_PCM_1_CAP_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_2_CAP_CH1", AFE_CONN46,
				    I_PCM_2_CAP_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul6_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN47,
				    I_ADDA_UL_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH2", AFE_CONN47,
				    I_DL1_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL12_CH2", AFE_CONN47,
				    I_DL12_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL6_CH2", AFE_CONN47_1,
				    I_DL6_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL2_CH2", AFE_CONN47,
				    I_DL2_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL3_CH2", AFE_CONN47,
				    I_DL3_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("DL4_CH2", AFE_CONN47_1,
				    I_DL4_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_1_CAP_CH1", AFE_CONN47,
				    I_PCM_1_CAP_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_2_CAP_CH1", AFE_CONN47,
				    I_PCM_2_CAP_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul7_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN48,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("HW_GAIN2_OUT_CH1", AFE_CONN48,
				    I_GAIN2_OUT_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("HW_SRC_2_OUT_CH1", AFE_CONN48_1,
				    I_SRC_2_OUT_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul7_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN49,
				    I_ADDA_UL_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("HW_GAIN2_OUT_CH2", AFE_CONN49,
				    I_GAIN2_OUT_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("HW_SRC_2_OUT_CH2", AFE_CONN49_1,
				    I_SRC_2_OUT_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul8_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN50,
				    I_ADDA_UL_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul8_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN51,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new memif_ul_mono_1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_1_CAP_CH1", AFE_CONN12,
				    I_PCM_1_CAP_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_2_CAP_CH1", AFE_CONN12,
				    I_PCM_2_CAP_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul_mono_2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN11,
				    I_ADDA_UL_CH1, 1, 0),
};

static const struct snd_kcontrol_new memif_ul_mono_3_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN35,
				    I_ADDA_UL_CH1, 1, 0),
};

static const struct snd_soc_dapm_widget mt6833_memif_widgets[] = {
	/* inter-connections */
	SND_SOC_DAPM_MIXER("UL1_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul1_ch1_mix, ARRAY_SIZE(memif_ul1_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL1_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul1_ch2_mix, ARRAY_SIZE(memif_ul1_ch2_mix)),
	SND_SOC_DAPM_MIXER("UL1_CH3", SND_SOC_NOPM, 0, 0,
			   memif_ul1_ch3_mix, ARRAY_SIZE(memif_ul1_ch3_mix)),
	SND_SOC_DAPM_MIXER("UL1_CH4", SND_SOC_NOPM, 0, 0,
			   memif_ul1_ch4_mix, ARRAY_SIZE(memif_ul1_ch4_mix)),

	SND_SOC_DAPM_MIXER("UL2_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul2_ch1_mix, ARRAY_SIZE(memif_ul2_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL2_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul2_ch2_mix, ARRAY_SIZE(memif_ul2_ch2_mix)),

	SND_SOC_DAPM_MIXER("UL3_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul3_ch1_mix, ARRAY_SIZE(memif_ul3_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL3_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul3_ch2_mix, ARRAY_SIZE(memif_ul3_ch2_mix)),

	SND_SOC_DAPM_MIXER("UL4_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul4_ch1_mix, ARRAY_SIZE(memif_ul4_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL4_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul4_ch2_mix, ARRAY_SIZE(memif_ul4_ch2_mix)),

	SND_SOC_DAPM_MIXER("UL5_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul5_ch1_mix, ARRAY_SIZE(memif_ul5_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL5_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul5_ch2_mix, ARRAY_SIZE(memif_ul5_ch2_mix)),

	SND_SOC_DAPM_MIXER("UL6_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul6_ch1_mix, ARRAY_SIZE(memif_ul6_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL6_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul6_ch2_mix, ARRAY_SIZE(memif_ul6_ch2_mix)),

	SND_SOC_DAPM_MIXER("UL7_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul7_ch1_mix, ARRAY_SIZE(memif_ul7_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL7_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul7_ch2_mix, ARRAY_SIZE(memif_ul7_ch2_mix)),

	SND_SOC_DAPM_MIXER("UL8_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul8_ch1_mix, ARRAY_SIZE(memif_ul8_ch1_mix)),
	SND_SOC_DAPM_MIXER("UL8_CH2", SND_SOC_NOPM, 0, 0,
			   memif_ul8_ch2_mix, ARRAY_SIZE(memif_ul8_ch2_mix)),

	SND_SOC_DAPM_MIXER("UL_MONO_1_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul_mono_1_mix,
			   ARRAY_SIZE(memif_ul_mono_1_mix)),

	SND_SOC_DAPM_MIXER("UL_MONO_2_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul_mono_2_mix,
			   ARRAY_SIZE(memif_ul_mono_2_mix)),

	SND_SOC_DAPM_MIXER("UL_MONO_3_CH1", SND_SOC_NOPM, 0, 0,
			   memif_ul_mono_3_mix,
			   ARRAY_SIZE(memif_ul_mono_3_mix)),

	SND_SOC_DAPM_INPUT("UL1_VIRTUAL_INPUT"),
	SND_SOC_DAPM_INPUT("UL2_VIRTUAL_INPUT"),
	SND_SOC_DAPM_INPUT("UL6_VIRTUAL_INPUT"),
};

static const struct snd_soc_dapm_route mt6833_memif_routes[] = {
	{"UL1", NULL, "UL1_CH1"},
	{"UL1", NULL, "UL1_CH2"},
	{"UL1", NULL, "UL1_CH3"},
	{"UL1", NULL, "UL1_CH4"},
	{"UL1_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"UL1_CH1", "ADDA_UL_CH2", "ADDA_UL_Mux"},
	{"UL1_CH2", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"UL1_CH2", "ADDA_UL_CH2", "ADDA_UL_Mux"},
	{"UL1_CH3", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"UL1_CH3", "ADDA_UL_CH2", "ADDA_UL_Mux"},
	{"UL1_CH4", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"UL1_CH4", "ADDA_UL_CH2", "ADDA_UL_Mux"},

	{"UL2", NULL, "UL2_CH1"},
	{"UL2", NULL, "UL2_CH2"},

	/* cannot connect FE to FE directly */
	{"UL2_CH1", "DL1_CH1", "Hostless_UL2 UL"},
	{"UL2_CH2", "DL1_CH2", "Hostless_UL2 UL"},
	{"UL2_CH1", "DL12_CH1", "Hostless_UL2 UL"},
	{"UL2_CH2", "DL12_CH2", "Hostless_UL2 UL"},
	{"UL2_CH1", "DL6_CH1", "Hostless_UL2 UL"},
	{"UL2_CH2", "DL6_CH2", "Hostless_UL2 UL"},
	{"UL2_CH1", "DL2_CH1", "Hostless_UL2 UL"},
	{"UL2_CH2", "DL2_CH2", "Hostless_UL2 UL"},
	{"UL2_CH1", "DL3_CH1", "Hostless_UL2 UL"},
	{"UL2_CH2", "DL3_CH2", "Hostless_UL2 UL"},
	{"UL2_CH1", "DL4_CH1", "Hostless_UL2 UL"},
	{"UL2_CH2", "DL4_CH2", "Hostless_UL2 UL"},
	{"UL2_CH1", "DL5_CH1", "Hostless_UL2 UL"},
	{"UL2_CH2", "DL5_CH2", "Hostless_UL2 UL"},
	{"UL2_CH1", "DL7_CH1", "Hostless_UL2 UL"},
	{"UL2_CH2", "DL7_CH2", "Hostless_UL2 UL"},

	{"Hostless_UL2 UL", NULL, "UL2_VIRTUAL_INPUT"},

	{"UL2_CH1", "I2S0_CH1", "I2S0"},
	{"UL2_CH2", "I2S0_CH2", "I2S0"},
	{"UL2_CH1", "I2S2_CH1", "I2S2"},
	{"UL2_CH2", "I2S2_CH2", "I2S2"},

	{"UL2_CH1", "PCM_1_CAP_CH1", "PCM 1 Capture"},
	{"UL2_CH2", "PCM_1_CAP_CH1", "PCM 1 Capture"},
	{"UL2_CH1", "PCM_2_CAP_CH1", "PCM 2 Capture"},
	{"UL2_CH2", "PCM_2_CAP_CH1", "PCM 2 Capture"},

	{"UL_MONO_1", NULL, "UL_MONO_1_CH1"},
	{"UL_MONO_1_CH1", "PCM_1_CAP_CH1", "PCM 1 Capture"},
	{"UL_MONO_1_CH1", "PCM_2_CAP_CH1", "PCM 2 Capture"},

	{"UL_MONO_2", NULL, "UL_MONO_2_CH1"},
	{"UL_MONO_2_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux"},

	{"UL_MONO_3", NULL, "UL_MONO_3_CH1"},
	{"UL_MONO_3_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux"},

	{"UL2_CH1", "CONNSYS_I2S_CH1", "Connsys I2S"},
	{"UL2_CH2", "CONNSYS_I2S_CH2", "Connsys I2S"},

	{"UL2_CH1", "SRC_1_OUT_CH1", "HW_SRC_1_Out"},
	{"UL2_CH2", "SRC_1_OUT_CH2", "HW_SRC_1_Out"},

	{"UL3", NULL, "UL3_CH1"},
	{"UL3", NULL, "UL3_CH2"},
	{"UL3_CH1", "CONNSYS_I2S_CH1", "Connsys I2S"},
	{"UL3_CH2", "CONNSYS_I2S_CH2", "Connsys I2S"},

	{"UL4", NULL, "UL4_CH1"},
	{"UL4", NULL, "UL4_CH2"},
	{"UL4_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"UL4_CH2", "ADDA_UL_CH2", "ADDA_UL_Mux"},
	{"UL4_CH1", "I2S0_CH1", "I2S0"},
	{"UL4_CH2", "I2S0_CH2", "I2S0"},

	{"UL5", NULL, "UL5_CH1"},
	{"UL5", NULL, "UL5_CH2"},
	{"UL5_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"UL5_CH2", "ADDA_UL_CH2", "ADDA_UL_Mux"},

	{"UL6", NULL, "UL6_CH1"},
	{"UL6", NULL, "UL6_CH2"},

	{"UL6_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"UL6_CH2", "ADDA_UL_CH2", "ADDA_UL_Mux"},
	{"UL6_CH1", "DL1_CH1", "Hostless_UL6 UL"},
	{"UL6_CH2", "DL1_CH2", "Hostless_UL6 UL"},
	{"UL6_CH1", "DL2_CH1", "Hostless_UL6 UL"},
	{"UL6_CH2", "DL2_CH2", "Hostless_UL6 UL"},
	{"UL6_CH1", "DL12_CH1", "Hostless_UL6 UL"},
	{"UL6_CH2", "DL12_CH2", "Hostless_UL6 UL"},
	{"UL6_CH1", "DL6_CH1", "Hostless_UL6 UL"},
	{"UL6_CH2", "DL6_CH2", "Hostless_UL6 UL"},
	{"UL6_CH1", "DL3_CH1", "Hostless_UL6 UL"},
	{"UL6_CH2", "DL3_CH2", "Hostless_UL6 UL"},
	{"UL6_CH1", "DL4_CH1", "Hostless_UL6 UL"},
	{"UL6_CH2", "DL4_CH2", "Hostless_UL6 UL"},
	{"Hostless_UL6 UL", NULL, "UL6_VIRTUAL_INPUT"},
	{"UL6_CH1", "PCM_1_CAP_CH1", "PCM 1 Capture"},
	{"UL6_CH2", "PCM_1_CAP_CH1", "PCM 1 Capture"},
	{"UL6_CH1", "PCM_2_CAP_CH1", "PCM 2 Capture"},
	{"UL6_CH2", "PCM_2_CAP_CH1", "PCM 2 Capture"},

	{"UL7", NULL, "UL7_CH1"},
	{"UL7", NULL, "UL7_CH2"},
	{"UL7_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"UL7_CH2", "ADDA_UL_CH2", "ADDA_UL_Mux"},
	{"UL7_CH1", "HW_GAIN2_OUT_CH1", "HW Gain 2 Out"},
	{"UL7_CH2", "HW_GAIN2_OUT_CH2", "HW Gain 2 Out"},
	{"UL7_CH1", "HW_SRC_2_OUT_CH1", "HW_SRC_2_Out"},
	{"UL7_CH2", "HW_SRC_2_OUT_CH2", "HW_SRC_2_Out"},

	{"UL8", NULL, "UL8_CH1"},
	{"UL8", NULL, "UL8_CH2"},
	{"UL8_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"UL8_CH2", "ADDA_UL_CH2", "ADDA_UL_Mux"},

	{"HW_GAIN2_IN_CH1", "ADDA_UL_CH1", "ADDA_UL_Mux"},
	{"HW_GAIN2_IN_CH2", "ADDA_UL_CH2", "ADDA_UL_Mux"},
	{"UL7_CH1", "HW_GAIN2_OUT_CH1", "HW Gain 2 Out"},
	{"UL7_CH2", "HW_GAIN2_OUT_CH2", "HW Gain 2 Out"},
};

static const struct mtk_base_memif_data memif_data[MT6833_MEMIF_NUM] = {
	[MT6833_MEMIF_DL1] = {
		.name = "DL1",
		.id = MT6833_MEMIF_DL1,
		.reg_ofs_base = AFE_DL1_BASE,
		.reg_ofs_cur = AFE_DL1_CUR,
		.reg_ofs_end = AFE_DL1_END,
		.reg_ofs_base_msb = AFE_DL1_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL1_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL1_END_MSB,
		.fs_reg = AFE_DL1_CON0,
		.fs_shift = DL1_MODE_SFT,
		.fs_maskbit = DL1_MODE_MASK,
		.mono_reg = AFE_DL1_CON0,
		.mono_shift = DL1_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL1_ON_SFT,
		.hd_reg = AFE_DL1_CON0,
		.hd_shift = DL1_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL1_CON0,
		.pbuf_mask = DL1_PBUF_SIZE_MASK,
		.pbuf_shift = DL1_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL1_CON0,
		.minlen_mask = DL1_MINLEN_MASK,
		.minlen_shift = DL1_MINLEN_SFT,
	},
	[MT6833_MEMIF_DL12] = {
		.name = "DL12",
		.id = MT6833_MEMIF_DL12,
		.reg_ofs_base = AFE_DL12_BASE,
		.reg_ofs_cur = AFE_DL12_CUR,
		.reg_ofs_end = AFE_DL12_END,
		.reg_ofs_base_msb = AFE_DL12_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL12_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL12_END_MSB,
		.fs_reg = AFE_DL12_CON0,
		.fs_shift = DL12_MODE_SFT,
		.fs_maskbit = DL12_MODE_MASK,
		.mono_reg = AFE_DL12_CON0,
		.mono_shift = DL12_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL12_ON_SFT,
		.hd_reg = AFE_DL12_CON0,
		.hd_shift = DL12_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL12_CON0,
		.pbuf_mask = DL12_PBUF_SIZE_MASK,
		.pbuf_shift = DL12_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL12_CON0,
		.minlen_mask = DL12_MINLEN_MASK,
		.minlen_shift = DL12_MINLEN_SFT,
	},
	[MT6833_MEMIF_DL2] = {
		.name = "DL2",
		.id = MT6833_MEMIF_DL2,
		.reg_ofs_base = AFE_DL2_BASE,
		.reg_ofs_cur = AFE_DL2_CUR,
		.reg_ofs_end = AFE_DL2_END,
		.reg_ofs_base_msb = AFE_DL2_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL2_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL2_END_MSB,
		.fs_reg = AFE_DL2_CON0,
		.fs_shift = DL2_MODE_SFT,
		.fs_maskbit = DL2_MODE_MASK,
		.mono_reg = AFE_DL2_CON0,
		.mono_shift = DL2_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL2_ON_SFT,
		.hd_reg = AFE_DL2_CON0,
		.hd_shift = DL2_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL2_CON0,
		.pbuf_mask = DL2_PBUF_SIZE_MASK,
		.pbuf_shift = DL2_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL2_CON0,
		.minlen_mask = DL2_MINLEN_MASK,
		.minlen_shift = DL2_MINLEN_SFT,
	},
	[MT6833_MEMIF_DL3] = {
		.name = "DL3",
		.id = MT6833_MEMIF_DL3,
		.reg_ofs_base = AFE_DL3_BASE,
		.reg_ofs_cur = AFE_DL3_CUR,
		.reg_ofs_end = AFE_DL3_END,
		.reg_ofs_base_msb = AFE_DL3_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL3_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL3_END_MSB,
		.fs_reg = AFE_DL3_CON0,
		.fs_shift = DL3_MODE_SFT,
		.fs_maskbit = DL3_MODE_MASK,
		.mono_reg = AFE_DL3_CON0,
		.mono_shift = DL3_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL3_ON_SFT,
		.hd_reg = AFE_DL3_CON0,
		.hd_shift = DL3_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL3_CON0,
		.pbuf_mask = DL3_PBUF_SIZE_MASK,
		.pbuf_shift = DL3_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL3_CON0,
		.minlen_mask = DL3_MINLEN_MASK,
		.minlen_shift = DL3_MINLEN_SFT,
	},
	[MT6833_MEMIF_DL4] = {
		.name = "DL4",
		.id = MT6833_MEMIF_DL4,
		.reg_ofs_base = AFE_DL4_BASE,
		.reg_ofs_cur = AFE_DL4_CUR,
		.reg_ofs_end = AFE_DL4_END,
		.reg_ofs_base_msb = AFE_DL4_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL4_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL4_END_MSB,
		.fs_reg = AFE_DL4_CON0,
		.fs_shift = DL4_MODE_SFT,
		.fs_maskbit = DL4_MODE_MASK,
		.mono_reg = AFE_DL4_CON0,
		.mono_shift = DL4_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL4_ON_SFT,
		.hd_reg = AFE_DL4_CON0,
		.hd_shift = DL4_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL4_CON0,
		.pbuf_mask = DL4_PBUF_SIZE_MASK,
		.pbuf_shift = DL4_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL4_CON0,
		.minlen_mask = DL4_MINLEN_MASK,
		.minlen_shift = DL4_MINLEN_SFT,
	},
	[MT6833_MEMIF_DL5] = {
		.name = "DL5",
		.id = MT6833_MEMIF_DL5,
		.reg_ofs_base = AFE_DL5_BASE,
		.reg_ofs_cur = AFE_DL5_CUR,
		.reg_ofs_end = AFE_DL5_END,
		.reg_ofs_base_msb = AFE_DL5_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL5_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL5_END_MSB,
		.fs_reg = AFE_DL5_CON0,
		.fs_shift = DL5_MODE_SFT,
		.fs_maskbit = DL5_MODE_MASK,
		.mono_reg = AFE_DL5_CON0,
		.mono_shift = DL5_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL5_ON_SFT,
		.hd_reg = AFE_DL5_CON0,
		.hd_shift = DL5_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL5_CON0,
		.pbuf_mask = DL5_PBUF_SIZE_MASK,
		.pbuf_shift = DL5_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL5_CON0,
		.minlen_mask = DL5_MINLEN_MASK,
		.minlen_shift = DL5_MINLEN_SFT,
	},
	[MT6833_MEMIF_DL6] = {
		.name = "DL6",
		.id = MT6833_MEMIF_DL6,
		.reg_ofs_base = AFE_DL6_BASE,
		.reg_ofs_cur = AFE_DL6_CUR,
		.reg_ofs_end = AFE_DL6_END,
		.reg_ofs_base_msb = AFE_DL6_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL6_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL6_END_MSB,
		.fs_reg = AFE_DL6_CON0,
		.fs_shift = DL6_MODE_SFT,
		.fs_maskbit = DL6_MODE_MASK,
		.mono_reg = AFE_DL6_CON0,
		.mono_shift = DL6_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL6_ON_SFT,
		.hd_reg = AFE_DL6_CON0,
		.hd_shift = DL6_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL6_CON0,
		.pbuf_mask = DL6_PBUF_SIZE_MASK,
		.pbuf_shift = DL6_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL6_CON0,
		.minlen_mask = DL6_MINLEN_MASK,
		.minlen_shift = DL6_MINLEN_SFT,
	},
	[MT6833_MEMIF_DL7] = {
		.name = "DL7",
		.id = MT6833_MEMIF_DL7,
		.reg_ofs_base = AFE_DL7_BASE,
		.reg_ofs_cur = AFE_DL7_CUR,
		.reg_ofs_end = AFE_DL7_END,
		.reg_ofs_base_msb = AFE_DL7_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL7_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL7_END_MSB,
		.fs_reg = AFE_DL7_CON0,
		.fs_shift = DL7_MODE_SFT,
		.fs_maskbit = DL7_MODE_MASK,
		.mono_reg = AFE_DL7_CON0,
		.mono_shift = DL7_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL7_ON_SFT,
		.hd_reg = AFE_DL7_CON0,
		.hd_shift = DL7_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL7_CON0,
		.pbuf_mask = DL7_PBUF_SIZE_MASK,
		.pbuf_shift = DL7_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL7_CON0,
		.minlen_mask = DL7_MINLEN_MASK,
		.minlen_shift = DL7_MINLEN_SFT,
	},
	[MT6833_MEMIF_DL8] = {
		.name = "DL8",
		.id = MT6833_MEMIF_DL8,
		.reg_ofs_base = AFE_DL8_BASE,
		.reg_ofs_cur = AFE_DL8_CUR,
		.reg_ofs_end = AFE_DL8_END,
		.reg_ofs_base_msb = AFE_DL8_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL8_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL8_END_MSB,
		.fs_reg = AFE_DL8_CON0,
		.fs_shift = DL8_MODE_SFT,
		.fs_maskbit = DL8_MODE_MASK,
		.mono_reg = AFE_DL8_CON0,
		.mono_shift = DL8_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL8_ON_SFT,
		.hd_reg = AFE_DL8_CON0,
		.hd_shift = DL8_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL8_CON0,
		.pbuf_mask = DL8_PBUF_SIZE_MASK,
		.pbuf_shift = DL8_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL8_CON0,
		.minlen_mask = DL8_MINLEN_MASK,
		.minlen_shift = DL8_MINLEN_SFT,
	},
	[MT6833_MEMIF_DL9] = {
		.name = "DL9",
		.id = MT6833_MEMIF_DL9,
		.reg_ofs_base = AFE_DL9_BASE,
		.reg_ofs_cur = AFE_DL9_CUR,
		.reg_ofs_end = AFE_DL9_END,
		.reg_ofs_base_msb = AFE_DL9_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DL9_CUR_MSB,
		.reg_ofs_end_msb = AFE_DL9_END_MSB,
		.fs_reg = AFE_DL9_CON0,
		.fs_shift = DL9_MODE_SFT,
		.fs_maskbit = DL9_MODE_MASK,
		.mono_reg = AFE_DL9_CON0,
		.mono_shift = DL9_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DL9_ON_SFT,
		.hd_reg = AFE_DL9_CON0,
		.hd_shift = DL9_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
		.pbuf_reg = AFE_DL9_CON0,
		.pbuf_mask = DL9_PBUF_SIZE_MASK,
		.pbuf_shift = DL9_PBUF_SIZE_SFT,
		.minlen_reg = AFE_DL9_CON0,
		.minlen_mask = DL9_MINLEN_MASK,
		.minlen_shift = DL9_MINLEN_SFT,
	},
	[MT6833_MEMIF_DAI] = {
		.name = "DAI",
		.id = MT6833_MEMIF_DAI,
		.reg_ofs_base = AFE_DAI_BASE,
		.reg_ofs_cur = AFE_DAI_CUR,
		.reg_ofs_end = AFE_DAI_END,
		.reg_ofs_base_msb = AFE_DAI_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DAI_CUR_MSB,
		.reg_ofs_end_msb = AFE_DAI_END_MSB,
		.fs_reg = AFE_DAI_CON0,
		.fs_shift = DAI_MODE_SFT,
		.fs_maskbit = DAI_MODE_MASK,
		.mono_reg = AFE_DAI_CON0,
		.mono_shift = DAI_DUPLICATE_WR_SFT,
		.mono_invert = 1,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DAI_ON_SFT,
		.hd_reg = AFE_DAI_CON0,
		.hd_shift = DAI_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_MOD_DAI] = {
		.name = "MOD_DAI",
		.id = MT6833_MEMIF_MOD_DAI,
		.reg_ofs_base = AFE_MOD_DAI_BASE,
		.reg_ofs_cur = AFE_MOD_DAI_CUR,
		.reg_ofs_end = AFE_MOD_DAI_END,
		.reg_ofs_base_msb = AFE_MOD_DAI_BASE_MSB,
		.reg_ofs_cur_msb = AFE_MOD_DAI_CUR_MSB,
		.reg_ofs_end_msb = AFE_MOD_DAI_END_MSB,
		.fs_reg = AFE_MOD_DAI_CON0,
		.fs_shift = MOD_DAI_MODE_SFT,
		.fs_maskbit = MOD_DAI_MODE_MASK,
		.mono_reg = AFE_MOD_DAI_CON0,
		.mono_shift = MOD_DAI_DUPLICATE_WR_SFT,
		.mono_invert = 1,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = MOD_DAI_ON_SFT,
		.hd_reg = AFE_MOD_DAI_CON0,
		.hd_shift = MOD_DAI_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_DAI2] = {
		.name = "DAI2",
		.id = MT6833_MEMIF_DAI2,
		.reg_ofs_base = AFE_DAI2_BASE,
		.reg_ofs_cur = AFE_DAI2_CUR,
		.reg_ofs_end = AFE_DAI2_END,
		.reg_ofs_base_msb = AFE_DAI2_BASE_MSB,
		.reg_ofs_cur_msb = AFE_DAI2_CUR_MSB,
		.reg_ofs_end_msb = AFE_DAI2_END_MSB,
		.fs_reg = AFE_DAI2_CON0,
		.fs_shift = DAI2_MODE_SFT,
		.fs_maskbit = DAI2_MODE_MASK,
		.mono_reg = AFE_DAI2_CON0,
		.mono_shift = DAI2_DUPLICATE_WR_SFT,
		.mono_invert = 1,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = DAI2_ON_SFT,
		.hd_reg = AFE_DAI2_CON0,
		.hd_shift = DAI2_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_VUL12] = {
		.name = "VUL12",
		.id = MT6833_MEMIF_VUL12,
		.reg_ofs_base = AFE_VUL12_BASE,
		.reg_ofs_cur = AFE_VUL12_CUR,
		.reg_ofs_end = AFE_VUL12_END,
		.reg_ofs_base_msb = AFE_VUL12_BASE_MSB,
		.reg_ofs_cur_msb = AFE_VUL12_CUR_MSB,
		.reg_ofs_end_msb = AFE_VUL12_END_MSB,
		.fs_reg = AFE_VUL12_CON0,
		.fs_shift = VUL12_MODE_SFT,
		.fs_maskbit = VUL12_MODE_MASK,
		.mono_reg = AFE_VUL12_CON0,
		.mono_shift = VUL12_MONO_SFT,
		.quad_ch_reg = AFE_VUL12_CON0,
		.quad_ch_mask = VUL12_4CH_EN_MASK,
		.quad_ch_shift = VUL12_4CH_EN_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = VUL12_ON_SFT,
		.hd_reg = AFE_VUL12_CON0,
		.hd_shift = VUL12_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_VUL2] = {
		.name = "VUL2",
		.id = MT6833_MEMIF_VUL2,
		.reg_ofs_base = AFE_VUL2_BASE,
		.reg_ofs_cur = AFE_VUL2_CUR,
		.reg_ofs_end = AFE_VUL2_END,
		.reg_ofs_base_msb = AFE_VUL2_BASE_MSB,
		.reg_ofs_cur_msb = AFE_VUL2_CUR_MSB,
		.reg_ofs_end_msb = AFE_VUL2_END_MSB,
		.fs_reg = AFE_VUL2_CON0,
		.fs_shift = VUL2_MODE_SFT,
		.fs_maskbit = VUL2_MODE_MASK,
		.mono_reg = AFE_VUL2_CON0,
		.mono_shift = VUL2_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = VUL2_ON_SFT,
		.hd_reg = AFE_VUL2_CON0,
		.hd_shift = VUL2_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_AWB] = {
		.name = "AWB",
		.id = MT6833_MEMIF_AWB,
		.reg_ofs_base = AFE_AWB_BASE,
		.reg_ofs_cur = AFE_AWB_CUR,
		.reg_ofs_end = AFE_AWB_END,
		.reg_ofs_base_msb = AFE_AWB_BASE_MSB,
		.reg_ofs_cur_msb = AFE_AWB_CUR_MSB,
		.reg_ofs_end_msb = AFE_AWB_END_MSB,
		.fs_reg = AFE_AWB_CON0,
		.fs_shift = AWB_MODE_SFT,
		.fs_maskbit = AWB_MODE_MASK,
		.mono_reg = AFE_AWB_CON0,
		.mono_shift = AWB_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = AWB_ON_SFT,
		.hd_reg = AFE_AWB_CON0,
		.hd_shift = AWB_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_AWB2] = {
		.name = "AWB2",
		.id = MT6833_MEMIF_AWB2,
		.reg_ofs_base = AFE_AWB2_BASE,
		.reg_ofs_cur = AFE_AWB2_CUR,
		.reg_ofs_end = AFE_AWB2_END,
		.reg_ofs_base_msb = AFE_AWB2_BASE_MSB,
		.reg_ofs_cur_msb = AFE_AWB2_CUR_MSB,
		.reg_ofs_end_msb = AFE_AWB2_END_MSB,
		.fs_reg = AFE_AWB2_CON0,
		.fs_shift = AWB2_MODE_SFT,
		.fs_maskbit = AWB2_MODE_MASK,
		.mono_reg = AFE_AWB2_CON0,
		.mono_shift = AWB2_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = AWB2_ON_SFT,
		.hd_reg = AFE_AWB2_CON0,
		.hd_shift = AWB2_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_VUL3] = {
		.name = "VUL3",
		.id = MT6833_MEMIF_VUL3,
		.reg_ofs_base = AFE_VUL3_BASE,
		.reg_ofs_cur = AFE_VUL3_CUR,
		.reg_ofs_end = AFE_VUL3_END,
		.reg_ofs_base_msb = AFE_VUL3_BASE_MSB,
		.reg_ofs_cur_msb = AFE_VUL3_CUR_MSB,
		.reg_ofs_end_msb = AFE_VUL3_END_MSB,
		.fs_reg = AFE_VUL3_CON0,
		.fs_shift = VUL3_MODE_SFT,
		.fs_maskbit = VUL3_MODE_MASK,
		.mono_reg = AFE_VUL3_CON0,
		.mono_shift = VUL3_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = VUL3_ON_SFT,
		.hd_reg = AFE_VUL3_CON0,
		.hd_shift = VUL3_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_VUL4] = {
		.name = "VUL4",
		.id = MT6833_MEMIF_VUL4,
		.reg_ofs_base = AFE_VUL4_BASE,
		.reg_ofs_cur = AFE_VUL4_CUR,
		.reg_ofs_end = AFE_VUL4_END,
		.reg_ofs_base_msb = AFE_VUL4_BASE_MSB,
		.reg_ofs_cur_msb = AFE_VUL4_CUR_MSB,
		.reg_ofs_end_msb = AFE_VUL4_END_MSB,
		.fs_reg = AFE_VUL4_CON0,
		.fs_shift = VUL4_MODE_SFT,
		.fs_maskbit = VUL4_MODE_MASK,
		.mono_reg = AFE_VUL4_CON0,
		.mono_shift = VUL4_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = VUL4_ON_SFT,
		.hd_reg = AFE_VUL4_CON0,
		.hd_shift = VUL4_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_VUL5] = {
		.name = "VUL5",
		.id = MT6833_MEMIF_VUL5,
		.reg_ofs_base = AFE_VUL5_BASE,
		.reg_ofs_cur = AFE_VUL5_CUR,
		.reg_ofs_end = AFE_VUL5_END,
		.reg_ofs_base_msb = AFE_VUL5_BASE_MSB,
		.reg_ofs_cur_msb = AFE_VUL5_CUR_MSB,
		.reg_ofs_end_msb = AFE_VUL5_END_MSB,
		.fs_reg = AFE_VUL5_CON0,
		.fs_shift = VUL5_MODE_SFT,
		.fs_maskbit = VUL5_MODE_MASK,
		.mono_reg = AFE_VUL5_CON0,
		.mono_shift = VUL5_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = VUL5_ON_SFT,
		.hd_reg = AFE_VUL5_CON0,
		.hd_shift = VUL5_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
	[MT6833_MEMIF_VUL6] = {
		.name = "VUL6",
		.id = MT6833_MEMIF_VUL6,
		.reg_ofs_base = AFE_VUL6_BASE,
		.reg_ofs_cur = AFE_VUL6_CUR,
		.reg_ofs_end = AFE_VUL6_END,
		.reg_ofs_base_msb = AFE_VUL6_BASE_MSB,
		.reg_ofs_cur_msb = AFE_VUL6_CUR_MSB,
		.reg_ofs_end_msb = AFE_VUL6_END_MSB,
		.fs_reg = AFE_VUL6_CON0,
		.fs_shift = VUL6_MODE_SFT,
		.fs_maskbit = VUL6_MODE_MASK,
		.mono_reg = AFE_VUL6_CON0,
		.mono_shift = VUL6_MONO_SFT,
		.enable_reg = AFE_DAC_CON0,
		.enable_shift = VUL6_ON_SFT,
		.hd_reg = AFE_VUL6_CON0,
		.hd_shift = VUL6_HD_MODE_SFT,
		.agent_disable_reg = -1,
		.agent_disable_shift = -1,
		.msb_reg = -1,
		.msb_shift = -1,
	},
};

static const struct mtk_base_irq_data irq_data[MT6833_IRQ_NUM] = {
	[MT6833_IRQ_0] = {
		.id = MT6833_IRQ_0,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT0,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = IRQ0_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ0_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ0_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ0_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ0_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ0_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_1] = {
		.id = MT6833_IRQ_1,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT1,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = IRQ1_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ1_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ1_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ1_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ1_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ1_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_2] = {
		.id = MT6833_IRQ_2,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT2,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = IRQ2_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ2_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ2_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ2_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ2_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ2_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_3] = {
		.id = MT6833_IRQ_3,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT3,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = IRQ3_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ3_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ3_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ3_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ3_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ3_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_4] = {
		.id = MT6833_IRQ_4,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT4,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = IRQ4_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ4_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ4_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ4_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ4_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ4_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_5] = {
		.id = MT6833_IRQ_5,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT5,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = IRQ5_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ5_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ5_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ5_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ5_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ5_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_6] = {
		.id = MT6833_IRQ_6,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT6,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = IRQ6_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ6_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ6_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ6_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ6_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ6_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_7] = {
		.id = MT6833_IRQ_7,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT7,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON1,
		.irq_fs_shift = IRQ7_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ7_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ7_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ7_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ7_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ7_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_8] = {
		.id = MT6833_IRQ_8,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT8,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = IRQ8_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ8_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ8_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ8_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ8_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ8_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_9] = {
		.id = MT6833_IRQ_9,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT9,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = IRQ9_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ9_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ9_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ9_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ9_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ9_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_10] = {
		.id = MT6833_IRQ_10,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT10,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = IRQ10_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ10_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ10_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ10_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ10_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ10_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_11] = {
		.id = MT6833_IRQ_11,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT11,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = IRQ11_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ11_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ11_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ11_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ11_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ11_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_12] = {
		.id = MT6833_IRQ_12,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT12,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = IRQ12_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ12_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ12_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ12_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ12_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ12_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_13] = {
		.id = MT6833_IRQ_13,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT13,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = IRQ13_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ13_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ13_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ13_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ13_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ13_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_14] = {
		.id = MT6833_IRQ_14,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT14,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = IRQ14_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ14_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ14_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ14_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ14_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ14_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_15] = {
		.id = MT6833_IRQ_15,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT15,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON2,
		.irq_fs_shift = IRQ15_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ15_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ15_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ15_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ15_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ15_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_16] = {
		.id = MT6833_IRQ_16,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT16,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON3,
		.irq_fs_shift = IRQ16_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ16_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ16_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ16_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ16_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ16_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_17] = {
		.id = MT6833_IRQ_17,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT17,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON3,
		.irq_fs_shift = IRQ17_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ17_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ17_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ17_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ17_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ17_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_18] = {
		.id = MT6833_IRQ_18,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT18,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON3,
		.irq_fs_shift = IRQ18_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ18_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ18_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ18_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ18_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ18_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_19] = {
		.id = MT6833_IRQ_19,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT19,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON3,
		.irq_fs_shift = IRQ19_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ19_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ19_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ19_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ19_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ19_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_20] = {
		.id = MT6833_IRQ_20,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT20,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON3,
		.irq_fs_shift = IRQ20_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ20_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ20_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ20_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ20_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ20_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_21] = {
		.id = MT6833_IRQ_21,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT21,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON3,
		.irq_fs_shift = IRQ21_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ21_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ21_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ21_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ21_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ21_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_22] = {
		.id = MT6833_IRQ_22,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT22,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON3,
		.irq_fs_shift = IRQ22_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ22_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ22_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ22_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ22_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ22_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_23] = {
		.id = MT6833_IRQ_23,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT23,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON3,
		.irq_fs_shift = IRQ23_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ23_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ23_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ23_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ23_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ23_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_24] = {
		.id = MT6833_IRQ_24,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT24,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON4,
		.irq_fs_shift = IRQ24_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ24_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ24_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ24_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ24_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ24_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_25] = {
		.id = MT6833_IRQ_25,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT25,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON4,
		.irq_fs_shift = IRQ25_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ25_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ25_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ25_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ25_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ25_MCU_SCP_EN_SFT,
	},
	[MT6833_IRQ_26] = {
		.id = MT6833_IRQ_26,
		.irq_cnt_reg = AFE_IRQ_MCU_CNT26,
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT,
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK,
		.irq_fs_reg = AFE_IRQ_MCU_CON4,
		.irq_fs_shift = IRQ26_MCU_MODE_SFT,
		.irq_fs_maskbit = IRQ26_MCU_MODE_MASK,
		.irq_en_reg = AFE_IRQ_MCU_CON0,
		.irq_en_shift = IRQ26_MCU_ON_SFT,
		.irq_clr_reg = AFE_IRQ_MCU_CLR,
		.irq_clr_shift = IRQ26_MCU_CLR_SFT,
		.irq_ap_en_reg = AFE_IRQ_MCU_EN,
		.irq_ap_en_shift = IRQ26_MCU_EN_SFT,
		.irq_scp_en_reg = AFE_IRQ_MCU_SCP_EN,
		.irq_scp_en_shift = IRQ26_MCU_SCP_EN_SFT,
	},
};

static const int memif_irq_usage[MT6833_MEMIF_NUM] = {
	/* TODO: verify each memif & irq */
	[MT6833_MEMIF_DL1] = MT6833_IRQ_0,
	[MT6833_MEMIF_DL2] = MT6833_IRQ_1,
	[MT6833_MEMIF_DL3] = MT6833_IRQ_2,
	[MT6833_MEMIF_DL4] = MT6833_IRQ_3,
	[MT6833_MEMIF_DL5] = MT6833_IRQ_4,
	[MT6833_MEMIF_DL6] = MT6833_IRQ_5,
	[MT6833_MEMIF_DL7] = MT6833_IRQ_6,
	[MT6833_MEMIF_DL8] = MT6833_IRQ_7,
	[MT6833_MEMIF_DL9] = MT6833_IRQ_8,
	[MT6833_MEMIF_DL12] = MT6833_IRQ_9,
	[MT6833_MEMIF_DAI] = MT6833_IRQ_10,
	[MT6833_MEMIF_MOD_DAI] = MT6833_IRQ_11,
	[MT6833_MEMIF_DAI2] = MT6833_IRQ_12,
	[MT6833_MEMIF_VUL12] = MT6833_IRQ_13,
	[MT6833_MEMIF_VUL2] = MT6833_IRQ_14,
	[MT6833_MEMIF_AWB] = MT6833_IRQ_15,
	[MT6833_MEMIF_AWB2] = MT6833_IRQ_16,
	[MT6833_MEMIF_VUL3] = MT6833_IRQ_17,
	[MT6833_MEMIF_VUL4] = MT6833_IRQ_18,
	[MT6833_MEMIF_VUL5] = MT6833_IRQ_19,
	[MT6833_MEMIF_VUL6] = MT6833_IRQ_20,
};

static bool mt6833_is_volatile_reg(struct device *dev, unsigned int reg)
{
	/* these auto-gen reg has read-only bit, so put it as volatile */
	/* volatile reg cannot be cached, so cannot be set when power off */
	switch (reg) {
	case AUDIO_TOP_CON0:	/* reg bit controlled by CCF */
	case AUDIO_TOP_CON1:	/* reg bit controlled by CCF */
	case AUDIO_TOP_CON2:
	case AUDIO_TOP_CON3:
	case AFE_DL1_CUR_MSB:
	case AFE_DL1_CUR:
	case AFE_DL1_END:
	case AFE_DL2_CUR_MSB:
	case AFE_DL2_CUR:
	case AFE_DL2_END:
	case AFE_DL3_CUR_MSB:
	case AFE_DL3_CUR:
	case AFE_DL3_END:
	case AFE_DL4_CUR_MSB:
	case AFE_DL4_CUR:
	case AFE_DL4_END:
	case AFE_DL12_CUR_MSB:
	case AFE_DL12_CUR:
	case AFE_DL12_END:
	case AFE_ADDA_SRC_DEBUG_MON0:
	case AFE_ADDA_SRC_DEBUG_MON1:
	case AFE_ADDA_UL_SRC_MON0:
	case AFE_ADDA_UL_SRC_MON1:
	case AFE_SECURE_CON0:
	case AFE_SRAM_BOUND:
	case AFE_SECURE_CON1:
	case AFE_VUL_CUR_MSB:
	case AFE_VUL_CUR:
	case AFE_VUL_END:
	case AFE_SIDETONE_MON:
	case AFE_SIDETONE_CON0:
	case AFE_SIDETONE_COEFF:
	case AFE_VUL2_CUR_MSB:
	case AFE_VUL2_CUR:
	case AFE_VUL2_END:
	case AFE_VUL3_CUR_MSB:
	case AFE_VUL3_CUR:
	case AFE_VUL3_END:
	case AFE_I2S_MON:
	case AFE_DAC_MON:
	case AFE_IRQ0_MCU_CNT_MON:
	case AFE_IRQ6_MCU_CNT_MON:
	case AFE_VUL4_CUR_MSB:
	case AFE_VUL4_CUR:
	case AFE_VUL4_END:
	case AFE_VUL12_CUR_MSB:
	case AFE_VUL12_CUR:
	case AFE_VUL12_END:
	case AFE_IRQ3_MCU_CNT_MON:
	case AFE_IRQ4_MCU_CNT_MON:
	case AFE_IRQ_MCU_STATUS:
	case AFE_IRQ_MCU_CLR:
	case AFE_IRQ_MCU_MON2:
	case AFE_IRQ1_MCU_CNT_MON:
	case AFE_IRQ2_MCU_CNT_MON:
	case AFE_IRQ5_MCU_CNT_MON:
	case AFE_IRQ7_MCU_CNT_MON:
	case AFE_IRQ_MCU_MISS_CLR:
	case AFE_GAIN1_CUR:
	case AFE_GAIN2_CUR:
	case AFE_SRAM_DELSEL_CON1:
	case PCM_INTF_CON2:
	case FPGA_CFG0:
	case FPGA_CFG1:
	case FPGA_CFG2:
	case FPGA_CFG3:
	case AUDIO_TOP_DBG_MON0:
	case AUDIO_TOP_DBG_MON1:
	case AFE_IRQ8_MCU_CNT_MON:
	case AFE_IRQ11_MCU_CNT_MON:
	case AFE_IRQ12_MCU_CNT_MON:
	case AFE_IRQ9_MCU_CNT_MON:
	case AFE_IRQ10_MCU_CNT_MON:
	case AFE_IRQ13_MCU_CNT_MON:
	case AFE_IRQ14_MCU_CNT_MON:
	case AFE_IRQ15_MCU_CNT_MON:
	case AFE_IRQ16_MCU_CNT_MON:
	case AFE_IRQ17_MCU_CNT_MON:
	case AFE_IRQ18_MCU_CNT_MON:
	case AFE_IRQ19_MCU_CNT_MON:
	case AFE_IRQ20_MCU_CNT_MON:
	case AFE_IRQ21_MCU_CNT_MON:
	case AFE_IRQ22_MCU_CNT_MON:
	case AFE_IRQ23_MCU_CNT_MON:
	case AFE_IRQ24_MCU_CNT_MON:
	case AFE_IRQ25_MCU_CNT_MON:
	case AFE_IRQ26_MCU_CNT_MON:
	case AFE_IRQ31_MCU_CNT_MON:
	case AFE_CBIP_MON0:
	case AFE_CBIP_SLV_MUX_MON0:
	case AFE_CBIP_SLV_DECODER_MON0:
	case AFE_ADDA6_MTKAIF_MON0:
	case AFE_ADDA6_MTKAIF_MON1:
	case AFE_AWB_CUR_MSB:
	case AFE_AWB_CUR:
	case AFE_AWB_END:
	case AFE_AWB2_CUR_MSB:
	case AFE_AWB2_CUR:
	case AFE_AWB2_END:
	case AFE_DAI_CUR_MSB:
	case AFE_DAI_CUR:
	case AFE_DAI_END:
	case AFE_DAI2_CUR_MSB:
	case AFE_DAI2_CUR:
	case AFE_DAI2_END:
	case AFE_ADDA6_SRC_DEBUG_MON0:
	case AFE_ADD6A_UL_SRC_MON0:
	case AFE_ADDA6_UL_SRC_MON1:
	case AFE_MOD_DAI_CUR_MSB:
	case AFE_MOD_DAI_CUR:
	case AFE_MOD_DAI_END:
	case AFE_AWB_RCH_MON:
	case AFE_AWB_LCH_MON:
	case AFE_VUL_RCH_MON:
	case AFE_VUL_LCH_MON:
	case AFE_VUL12_RCH_MON:
	case AFE_VUL12_LCH_MON:
	case AFE_VUL2_RCH_MON:
	case AFE_VUL2_LCH_MON:
	case AFE_DAI_DATA_MON:
	case AFE_MOD_DAI_DATA_MON:
	case AFE_DAI2_DATA_MON:
	case AFE_AWB2_RCH_MON:
	case AFE_AWB2_LCH_MON:
	case AFE_VUL3_RCH_MON:
	case AFE_VUL3_LCH_MON:
	case AFE_VUL4_RCH_MON:
	case AFE_VUL4_LCH_MON:
	case AFE_VUL5_RCH_MON:
	case AFE_VUL5_LCH_MON:
	case AFE_VUL6_RCH_MON:
	case AFE_VUL6_LCH_MON:
	case AFE_DL1_RCH_MON:
	case AFE_DL1_LCH_MON:
	case AFE_DL2_RCH_MON:
	case AFE_DL2_LCH_MON:
	case AFE_DL12_RCH1_MON:
	case AFE_DL12_LCH1_MON:
	case AFE_DL12_RCH2_MON:
	case AFE_DL12_LCH2_MON:
	case AFE_DL3_RCH_MON:
	case AFE_DL3_LCH_MON:
	case AFE_DL4_RCH_MON:
	case AFE_DL4_LCH_MON:
	case AFE_DL5_RCH_MON:
	case AFE_DL5_LCH_MON:
	case AFE_DL6_RCH_MON:
	case AFE_DL6_LCH_MON:
	case AFE_DL7_RCH_MON:
	case AFE_DL7_LCH_MON:
	case AFE_DL8_RCH_MON:
	case AFE_DL8_LCH_MON:
	case AFE_VUL5_CUR_MSB:
	case AFE_VUL5_CUR:
	case AFE_VUL5_END:
	case AFE_VUL6_CUR_MSB:
	case AFE_VUL6_CUR:
	case AFE_VUL6_END:
	case AFE_ADDA_DL_SDM_FIFO_MON:
	case AFE_ADDA_DL_SRC_LCH_MON:
	case AFE_ADDA_DL_SRC_RCH_MON:
	case AFE_ADDA_DL_SDM_OUT_MON:
	case AFE_CONNSYS_I2S_MON:
	case AFE_ASRC_2CH_CON0:
	case AFE_ASRC_2CH_CON2:
	case AFE_ASRC_2CH_CON3:
	case AFE_ASRC_2CH_CON4:
	case AFE_ASRC_2CH_CON5:
	case AFE_ASRC_2CH_CON7:
	case AFE_ASRC_2CH_CON8:
	case AFE_ASRC_2CH_CON12:
	case AFE_ASRC_2CH_CON13:
	case AFE_DL9_CUR_MSB:
	case AFE_DL9_CUR:
	case AFE_DL9_END:
	case AFE_ADDA_MTKAIF_MON0:
	case AFE_ADDA_MTKAIF_MON1:
	case AFE_AUD_PAD_TOP:
	case AFE_DL_NLE_R_MON0:
	case AFE_DL_NLE_R_MON1:
	case AFE_DL_NLE_R_MON2:
	case AFE_DL_NLE_L_MON0:
	case AFE_DL_NLE_L_MON1:
	case AFE_DL_NLE_L_MON2:
	case AFE_GENERAL1_ASRC_2CH_CON0:
	case AFE_GENERAL1_ASRC_2CH_CON2:
	case AFE_GENERAL1_ASRC_2CH_CON3:
	case AFE_GENERAL1_ASRC_2CH_CON4:
	case AFE_GENERAL1_ASRC_2CH_CON5:
	case AFE_GENERAL1_ASRC_2CH_CON7:
	case AFE_GENERAL1_ASRC_2CH_CON8:
	case AFE_GENERAL1_ASRC_2CH_CON12:
	case AFE_GENERAL1_ASRC_2CH_CON13:
	case AFE_GENERAL2_ASRC_2CH_CON0:
	case AFE_GENERAL2_ASRC_2CH_CON2:
	case AFE_GENERAL2_ASRC_2CH_CON3:
	case AFE_GENERAL2_ASRC_2CH_CON4:
	case AFE_GENERAL2_ASRC_2CH_CON5:
	case AFE_GENERAL2_ASRC_2CH_CON7:
	case AFE_GENERAL2_ASRC_2CH_CON8:
	case AFE_GENERAL2_ASRC_2CH_CON12:
	case AFE_GENERAL2_ASRC_2CH_CON13:
	case AFE_DL9_RCH_MON:
	case AFE_DL9_LCH_MON:
	case AFE_DL5_CUR_MSB:
	case AFE_DL5_CUR:
	case AFE_DL5_END:
	case AFE_DL6_CUR_MSB:
	case AFE_DL6_CUR:
	case AFE_DL6_END:
	case AFE_DL7_CUR_MSB:
	case AFE_DL7_CUR:
	case AFE_DL7_END:
	case AFE_DL8_CUR_MSB:
	case AFE_DL8_CUR:
	case AFE_DL8_END:
	case AFE_PROT_SIDEBAND_MON:
	case AFE_DOMAIN_SIDEBAND0_MON:
	case AFE_DOMAIN_SIDEBAND1_MON:
	case AFE_DOMAIN_SIDEBAND2_MON:
	case AFE_DOMAIN_SIDEBAND3_MON:
	case AFE_APLL1_TUNER_CFG:	/* [20:31] is monitor */
	case AFE_APLL2_TUNER_CFG:	/* [20:31] is monitor */
	/* these reg would change in scp/adsp */
	case AFE_DAC_CON0:
	case AFE_IRQ_MCU_CON0:
	case AFE_IRQ_MCU_EN:
	case AFE_IRQ_MCU_DSP_EN:
	case AFE_IRQ_MCU_SCP_EN:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config mt6833_afe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.volatile_reg = mt6833_is_volatile_reg,

	.max_register = AFE_MAX_REGISTER,
	.num_reg_defaults_raw = AFE_MAX_REGISTER,

	.cache_type = REGCACHE_FLAT,
};

#if !defined(CONFIG_FPGA_EARLY_PORTING) || defined(FORCE_FPGA_ENABLE_IRQ)
static irqreturn_t mt6833_afe_irq_handler(int irq_id, void *dev)
{
	struct mtk_base_afe *afe = dev;
	struct mtk_base_afe_irq *irq;
	unsigned int status = 0;
	unsigned int status_mcu;
	unsigned int mcu_en = 0;
	int ret;
	int i;

	/* get irq that is sent to MCU */
	regmap_read(afe->regmap, AFE_IRQ_MCU_EN, &mcu_en);

	ret = regmap_read(afe->regmap, AFE_IRQ_MCU_STATUS, &status);
	/* only care IRQ which is sent to MCU */
	status_mcu = status & mcu_en & AFE_IRQ_STATUS_BITS;

	if (ret || status_mcu == 0) {
		dev_err(afe->dev, "%s(), irq status err, ret %d, status 0x%x, mcu_en 0x%x\n",
			__func__, ret, status, mcu_en);

		goto err_irq;
	}

	for (i = 0; i < MT6833_MEMIF_NUM; i++) {
		struct mtk_base_afe_memif *memif = &afe->memif[i];

		if (!memif->substream)
			continue;

		if (memif->irq_usage < 0)
			continue;

		irq = &afe->irqs[memif->irq_usage];

		if (status_mcu & (1 << irq->irq_data->irq_en_shift))
			snd_pcm_period_elapsed(memif->substream);
	}

err_irq:
	/* clear irq */
	regmap_write(afe->regmap,
		     AFE_IRQ_MCU_CLR,
		     status_mcu);

	return IRQ_HANDLED;
}
#endif

static int mt6833_afe_runtime_suspend(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	unsigned int value = 0;
	int ret;

	dev_info(afe->dev, "%s()\n", __func__);

	if (!afe->regmap)
		goto skip_regmap;

	/* disable AFE */
	regmap_update_bits(afe->regmap, AFE_DAC_CON0, 0x1, 0x0);

	ret = regmap_read_poll_timeout(afe->regmap,
				       AFE_DAC_MON,
				       value,
				       (value & AFE_ON_RETM_MASK_SFT) == 0,
				       20,
				       1 * 1000 * 1000);
	if (ret)
		dev_warn(afe->dev, "%s(), ret %d\n", __func__, ret);

	/* make sure all irq status are cleared */
	regmap_write(afe->regmap, AFE_IRQ_MCU_CLR, 0xffffffff);
	regmap_write(afe->regmap, AFE_IRQ_MCU_CLR, 0xffffffff);

	/* reset sgen */
	regmap_write(afe->regmap, AFE_SINEGEN_CON0, 0x0);
	regmap_update_bits(afe->regmap, AFE_SINEGEN_CON2,
			   INNER_LOOP_BACK_MODE_MASK_SFT,
			   0x3f << INNER_LOOP_BACK_MODE_SFT);

	/* cache only */
	regcache_cache_only(afe->regmap, true);
	regcache_mark_dirty(afe->regmap);

skip_regmap:
	mt6833_afe_disable_clock(afe);
	return 0;
}

static int mt6833_afe_runtime_resume(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	struct mt6833_afe_private *afe_priv = afe->platform_priv;
	int ret;

	dev_info(afe->dev, "%s()\n", __func__);

	ret = mt6833_afe_enable_clock(afe);
	if (ret)
		return ret;

	if (!afe->regmap)
		goto skip_regmap;

	regcache_cache_only(afe->regmap, false);
	regcache_sync(afe->regmap);
	/* enable audio sys DCM for power saving */
	regmap_update_bits(afe_priv->infracfg,
			   PERI_BUS_DCM_CTRL, 0x1 << 29, 0x1 << 29);
	regmap_update_bits(afe->regmap, AUDIO_TOP_CON0, 0x1 << 29, 0x1 << 29);
	/* force cpu use 8_24 format when writing 32bit data */
	regmap_update_bits(afe->regmap, AFE_MEMIF_CON0,
			   CPU_HD_ALIGN_MASK_SFT, 0 << CPU_HD_ALIGN_SFT);
	/* set all output port to 24bit */
	regmap_write(afe->regmap, AFE_CONN_24BIT, 0xffffffff);
	regmap_write(afe->regmap, AFE_CONN_24BIT_1, 0xffffffff);
	/* enable AFE */
	regmap_update_bits(afe->regmap, AFE_DAC_CON0, 0x1, 0x1);

skip_regmap:
	return 0;
}

static int mt6833_afe_pcm_copy(struct snd_pcm_substream *substream,
			       int channel, unsigned long hwoff,
			       void *buf, unsigned long bytes,
			       mtk_sp_copy_f sp_copy)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	int ret = 0;

	mt6833_set_audio_int_bus_parent(afe, CLK_TOP_MAINPLL_D4_D4);

	ret = sp_copy(substream, channel, hwoff, buf, bytes);

	mt6833_set_audio_int_bus_parent(afe, CLK_CLK26M);

	return ret;
}

static int mt6833_set_memif_sram_mode(struct device *dev,
				      enum mtk_audio_sram_mode sram_mode)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	int reg_bit = sram_mode == MTK_AUDIO_SRAM_NORMAL_MODE ? 1 : 0;

	regmap_update_bits(afe->regmap, AFE_DL1_CON0,
			   DL1_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL1_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DL2_CON0,
			   DL2_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL2_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DL3_CON0,
			   DL3_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL3_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DL4_CON0,
			   DL4_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL4_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DL5_CON0,
			   DL5_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL5_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DL6_CON0,
			   DL6_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL6_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DL7_CON0,
			   DL7_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL7_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DL8_CON0,
			   DL8_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL8_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DL9_CON0,
			   DL9_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL9_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DL12_CON0,
			   DL12_NORMAL_MODE_MASK_SFT,
			   reg_bit << DL12_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_AWB_CON0,
			   AWB_NORMAL_MODE_MASK_SFT,
			   reg_bit << AWB_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_AWB2_CON0,
			   AWB2_NORMAL_MODE_MASK_SFT,
			   reg_bit << AWB2_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_VUL12_CON0,
			   VUL12_NORMAL_MODE_MASK_SFT,
			   reg_bit << VUL12_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_VUL_CON0,
			   VUL_NORMAL_MODE_MASK_SFT,
			   reg_bit << VUL_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_VUL2_CON0,
			   VUL2_NORMAL_MODE_MASK_SFT,
			   reg_bit << VUL2_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_VUL3_CON0,
			   VUL3_NORMAL_MODE_MASK_SFT,
			   reg_bit << VUL3_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_VUL4_CON0,
			   VUL4_NORMAL_MODE_MASK_SFT,
			   reg_bit << VUL4_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_VUL5_CON0,
			   VUL5_NORMAL_MODE_MASK_SFT,
			   reg_bit << VUL5_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_VUL6_CON0,
			   VUL6_NORMAL_MODE_MASK_SFT,
			   reg_bit << VUL6_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DAI_CON0,
			   DAI_NORMAL_MODE_MASK_SFT,
			   reg_bit << DAI_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_DAI2_CON0,
			   DAI2_NORMAL_MODE_MASK_SFT,
			   reg_bit << DAI2_NORMAL_MODE_SFT);
	regmap_update_bits(afe->regmap, AFE_MOD_DAI_CON0,
			   MOD_DAI_NORMAL_MODE_MASK_SFT,
			   reg_bit << MOD_DAI_NORMAL_MODE_SFT);
	return 0;
}

static int mt6833_set_sram_mode(struct device *dev,
				enum mtk_audio_sram_mode sram_mode)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);

	/* set memif sram mode */
	mt6833_set_memif_sram_mode(dev, sram_mode);

	if (sram_mode == MTK_AUDIO_SRAM_COMPACT_MODE)
		/* cpu use compact mode when access sram data */
		regmap_update_bits(afe->regmap, AFE_MEMIF_CON0,
				   CPU_COMPACT_MODE_MASK_SFT,
				   0x1 << CPU_COMPACT_MODE_SFT);
	else
		/* cpu use normal mode when access sram data */
		regmap_update_bits(afe->regmap, AFE_MEMIF_CON0,
				   CPU_COMPACT_MODE_MASK_SFT,
				   0x0 << CPU_COMPACT_MODE_SFT);

	return 0;
}

static const struct mtk_audio_sram_ops mt6833_sram_ops = {
	.set_sram_mode = mt6833_set_sram_mode,
};

static u32 copy_from_buffer_request(void *dest, size_t destsize, const void *src,
				    size_t srcsize, u32 offset, size_t request)
{
	/* if request == -1, offset == 0, copy full srcsize */
	if (offset + request > srcsize)
		request = srcsize - offset;

	/* if destsize == -1, don't check the request size */
	if (!dest || destsize < request) {
		pr_info("%s, buffer null or not enough space", __func__);
		return 0;
	}

	memcpy(dest, src + offset, request);
	return request;
}

/*
 * sysfs bin_attribute node
 */

static ssize_t afe_sysfs_debug_read(struct file *filep, struct kobject *kobj,
				    struct bin_attribute *attr,
				    char *buf, loff_t offset, size_t size)
{
	size_t read_size, ceil_size, page_mask;
	ssize_t ret;
	struct mtk_base_afe *afe = (struct mtk_base_afe *)attr->private;
	char *buffer = NULL; /* for reduce kernel stack */

	buffer = kmalloc(AFE_SYS_DEBUG_SIZE, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	// sys fs op align with page size
	read_size = mt6833_debug_read_reg(buffer, AFE_SYS_DEBUG_SIZE, afe);
	page_mask = ~(PAGE_SIZE-1);
	ceil_size = (read_size&page_mask) + PAGE_SIZE;

	ret = copy_from_buffer_request(buf, -1, buffer, ceil_size, offset, size);
	kfree(buffer);

	return ret;
}

/*
 * sysfs bin_attribute node
 */
static ssize_t afe_sysfs_debug_write(struct file *filep, struct kobject *kobj,
				     struct bin_attribute *attr,
				     char *buf, loff_t offset, size_t size)
{
	struct mtk_base_afe *afe = (struct mtk_base_afe *)attr->private;

	char input[MAX_DEBUG_WRITE_INPUT];
	char *temp, *command, *str_begin;
	char delim[] = " ,";

	if (!size) {
		dev_info(afe->dev, "%s(), count is 0, return directly\n",
			 __func__);
		goto exit;
	}

	if (size > MAX_DEBUG_WRITE_INPUT)
		size = MAX_DEBUG_WRITE_INPUT;

	memset((void *)input, 0, MAX_DEBUG_WRITE_INPUT);
	memcpy(input, buf, size);

	str_begin = kstrndup(input, MAX_DEBUG_WRITE_INPUT - 1,
			     GFP_KERNEL);

	if (!str_begin) {
		dev_info(afe->dev, "%s(), kstrdup fail\n", __func__);
		goto exit;
	}
	temp = str_begin;

	command = strsep(&temp, delim);

	if (strcmp("write_reg", command) == 0)
		mtk_afe_write_reg(afe, (void *)temp);
exit:

	return size;
}

struct bin_attribute bin_attr_afe_dump = {
	.attr = {
		.name = "mtk_afe_node",
		.mode = 0444,
	},
	.size = AFE_SYS_DEBUG_SIZE,
	.read = afe_sysfs_debug_read,
	.write = afe_sysfs_debug_write,
};

static struct bin_attribute *afe_bin_attrs[] = {
	&bin_attr_afe_dump,
	NULL,
};

struct attribute_group afe_bin_attr_group = {
	.name = "mtk_afe_attrs",
	.bin_attrs = afe_bin_attrs,
};


static int mt6833_afe_component_probe(struct snd_soc_component *component)
{
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(component);
	struct snd_soc_card *sndcard = component->card;
	struct snd_card *card = sndcard->snd_card;
	int ret = 0;

	mtk_afe_add_sub_dai_control(component);
	mt6833_add_misc_control(component);

	bin_attr_afe_dump.private = (void *)afe;
	ret = snd_card_add_dev_attr(card, &afe_bin_attr_group);
	if (ret)
		pr_info("snd_card_add_dev_attr fail\n");

	return 0;
}

static const struct snd_soc_component_driver mt6833_afe_component = {
	.name = AFE_PCM_NAME,
	.probe = mt6833_afe_component_probe,
	.pcm_construct = mtk_afe_pcm_new,
	.pcm_destruct = mtk_afe_pcm_free,
	.open = mtk_afe_pcm_open,
	.pointer = mtk_afe_pcm_pointer,
	.copy_user = mtk_afe_pcm_copy_user,
};

static ssize_t mt6833_debug_read_reg(char *buffer, int size, struct mtk_base_afe *afe)
{
	int n = 0, i = 0;
	unsigned int value;
	struct mt6833_afe_private *afe_priv = afe->platform_priv;

	if (!buffer)
		return -ENOMEM;

	n += scnprintf(buffer + n, size - n,
		       "mtkaif calibration phase %d, %d, %d\n",
		       afe_priv->mtkaif_chosen_phase[0],
		       afe_priv->mtkaif_chosen_phase[1],
		       afe_priv->mtkaif_chosen_phase[2]);

	n += scnprintf(buffer + n, size - n,
		       "mtkaif calibration cycle %d, %d, %d\n",
		       afe_priv->mtkaif_phase_cycle[0],
		       afe_priv->mtkaif_phase_cycle[1],
		       afe_priv->mtkaif_phase_cycle[2]);

	for (i = 0; i < afe->memif_size; i++) {
		n += scnprintf(buffer + n, size - n,
			       "memif[%d], irq_usage %d\n",
			       i, afe->memif[i].irq_usage);
	}

	regmap_read(afe_priv->topckgen, CLK_CFG_7, &value);
	n += scnprintf(buffer + n, size - n,
		       "CLK_CFG_7 = 0x%x\n", value);
	regmap_read(afe_priv->topckgen, CLK_CFG_8, &value);
	n += scnprintf(buffer + n, size - n,
		       "CLK_CFG_8 = 0x%x\n", value);
	regmap_read(afe_priv->topckgen, CLK_CFG_11, &value);
	n += scnprintf(buffer + n, size - n,
		       "CLK_CFG_11 = 0x%x\n", value);
	regmap_read(afe_priv->topckgen, CLK_CFG_12, &value);
	n += scnprintf(buffer + n, size - n,
		       "CLK_CFG_12 = 0x%x\n", value);
	regmap_read(afe_priv->topckgen, CLK_CFG_13, &value);
	n += scnprintf(buffer + n, size - n,
		       "CLK_CFG_13 = 0x%x\n", value);
	regmap_read(afe_priv->topckgen, CLK_CFG_15, &value);
	n += scnprintf(buffer + n, size - n,
		       "CLK_CFG_15 = 0x%x\n", value);
	regmap_read(afe_priv->topckgen, CLK_AUDDIV_0, &value);
	n += scnprintf(buffer + n, size - n,
		       "CLK_AUDDIV_0 = 0x%x\n", value);
	regmap_read(afe_priv->topckgen, CLK_AUDDIV_2, &value);
	n += scnprintf(buffer + n, size - n,
		       "CLK_AUDDIV_2 = 0x%x\n", value);
	regmap_read(afe_priv->topckgen, CLK_AUDDIV_3, &value);
	n += scnprintf(buffer + n, size - n,
		       "CLK_AUDDIV_3 = 0x%x\n", value);

	regmap_read(afe_priv->apmixed, AP_PLL_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AP_PLL_CON3 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL1_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL1_CON0 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL1_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL1_CON1 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL1_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL1_CON2 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL1_CON4, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL1_CON4 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL2_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL2_CON0 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL2_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL2_CON1 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL2_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL2_CON2 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL2_CON4, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL2_CON4 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL1_TUNER_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL1_TUNER_CON0 = 0x%x\n", value);
	regmap_read(afe_priv->apmixed, APLL2_TUNER_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "APLL2_TUNER_CON0 = 0x%x\n", value);

	regmap_read(afe_priv->infracfg, PERI_BUS_DCM_CTRL, &value);
	n += scnprintf(buffer + n, size - n,
		       "PERI_BUS_DCM_CTRL = 0x%x\n", value);
	regmap_read(afe_priv->infracfg, MODULE_SW_CG_1_STA, &value);
	n += scnprintf(buffer + n, size - n,
		       "MODULE_SW_CG_1_STA = 0x%x\n", value);
	regmap_read(afe_priv->infracfg, MODULE_SW_CG_2_STA, &value);
	n += scnprintf(buffer + n, size - n,
		       "MODULE_SW_CG_2_STA = 0x%x\n", value);

	regmap_read(afe->regmap, AUDIO_TOP_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AUDIO_TOP_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AUDIO_TOP_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AUDIO_TOP_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AUDIO_TOP_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AUDIO_TOP_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AUDIO_TOP_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AUDIO_TOP_CON3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAC_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAC_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_I2S_CON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_I2S_CON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN4, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN4 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_I2S_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_I2S_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_I2S_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_I2S_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_I2S_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_I2S_CON3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN5, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN5 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN_24BIT, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN_24BIT = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL1_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL1_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL1_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL1_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL1_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL1_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL1_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL1_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL1_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL1_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL1_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL1_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL1_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL1_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL2_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL2_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL2_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL2_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL2_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL2_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL2_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL2_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL2_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL2_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL2_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL2_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL2_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL2_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL3_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL3_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL3_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL3_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL3_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL3_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL3_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL3_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL3_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL3_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL3_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL3_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL3_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL3_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN6, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN6 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL4_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL4_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL4_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL4_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL4_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL4_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL4_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL4_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL4_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL4_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL4_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL4_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL4_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL4_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SRC2_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SRC2_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SRC2_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SRC2_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_UL_SRC_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_UL_SRC_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_UL_SRC_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_UL_SRC_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_TOP_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_TOP_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_UL_DL_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_UL_DL_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_SRC_DEBUG, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_SRC_DEBUG = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_SRC_DEBUG_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_SRC_DEBUG_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_SRC_DEBUG_MON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_SRC_DEBUG_MON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_UL_SRC_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_UL_SRC_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_UL_SRC_MON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_UL_SRC_MON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SRAM_BOUND, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SRAM_BOUND = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_CONN0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_CONN0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SIDETONE_DEBUG, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SIDETONE_DEBUG = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SIDETONE_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SIDETONE_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SINEGEN_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SINEGEN_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SIDETONE_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SIDETONE_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SIDETONE_COEFF, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SIDETONE_COEFF = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SIDETONE_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SIDETONE_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SIDETONE_GAIN, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SIDETONE_GAIN = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SINEGEN_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SINEGEN_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_TOP_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_TOP_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL2_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL2_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL2_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL2_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL2_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL2_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL2_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL2_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL2_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL2_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL2_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL2_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL2_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL2_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL3_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL3_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL3_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL3_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL3_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL3_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL3_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL3_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL3_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL3_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL3_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL3_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL3_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL3_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_BUSY, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_BUSY = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_BUS_CFG, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_BUS_CFG = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_PREDIS_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_PREDIS_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_PREDIS_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_PREDIS_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_I2S_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_I2S_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_IIR_COEF_02_01, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_IIR_COEF_02_01 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_IIR_COEF_04_03, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_IIR_COEF_04_03 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_IIR_COEF_06_05, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_IIR_COEF_06_05 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_IIR_COEF_08_07, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_IIR_COEF_08_07 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_IIR_COEF_10_09, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_IIR_COEF_10_09 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAC_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAC_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CON3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CON4, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CON4 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT6, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT6 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT8, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT8 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_DSP2_EN, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_DSP2_EN = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ0_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ0_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ6_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ6_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL4_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL4_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL4_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL4_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL4_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL4_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL4_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL4_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL4_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL4_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL4_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL4_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL4_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL4_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL12_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL12_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL12_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL12_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL12_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL12_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL12_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL12_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL12_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL12_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL12_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL12_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL12_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL12_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ3_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ3_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ4_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ4_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_STATUS, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_STATUS = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CLR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CLR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_EN, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_EN = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_MON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_MON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT5, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT5 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ1_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ1_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ2_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ2_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ5_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ5_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_DSP_EN, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_DSP_EN = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_SCP_EN, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_SCP_EN = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT7, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT7 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ7_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ7_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT4, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT4 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT11, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT11 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_APLL1_TUNER_CFG, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_APLL1_TUNER_CFG = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_APLL2_TUNER_CFG, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_APLL2_TUNER_CFG = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_MISS_CLR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_MISS_CLR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN33, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN33 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT12, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT12 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN1_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN1_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN1_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN1_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN1_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN1_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN1_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN1_CON3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN7, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN7 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN1_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN1_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN2_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN2_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN2_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN2_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN2_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN2_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN2_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN2_CON3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN8, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN8 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GAIN2_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GAIN2_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN9, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN9 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN10, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN10 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN11, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN11 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN12, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN12 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN13, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN13 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN14, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN14 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN15, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN15 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN16, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN16 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN17, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN17 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN18, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN18 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN19, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN19 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN20, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN20 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN21, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN21 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN22, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN22 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN23, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN23 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN24, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN24 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN_RS, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN_RS = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN_DI, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN_DI = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN25, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN25 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN26, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN26 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN27, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN27 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN28, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN28 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN29, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN29 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN30, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN30 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN31, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN31 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN32, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN32 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SRAM_DELSEL_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SRAM_DELSEL_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN56, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN56 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN57, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN57 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN56_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN56_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN57_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN57_1 = 0x%x\n", value);
	regmap_read(afe->regmap, PCM_INTF_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "PCM_INTF_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, PCM_INTF_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "PCM_INTF_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, PCM2_INTF_CON, &value);
	n += scnprintf(buffer + n, size - n,
		       "PCM2_INTF_CON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN34, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN34 = 0x%x\n", value);
	regmap_read(afe->regmap, AUDIO_TOP_DBG_CON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AUDIO_TOP_DBG_CON = 0x%x\n", value);
	regmap_read(afe->regmap, AUDIO_TOP_DBG_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AUDIO_TOP_DBG_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AUDIO_TOP_DBG_MON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AUDIO_TOP_DBG_MON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ8_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ8_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ11_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ11_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ12_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ12_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT9, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT9 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT10, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT10 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT13, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT13 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT14, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT14 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT15, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT15 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT16, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT16 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT17, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT17 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT18, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT18 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT19, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT19 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT20, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT20 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT21, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT21 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT22, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT22 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT23, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT23 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT24, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT24 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT25, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT25 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ_MCU_CNT26, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ_MCU_CNT26 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ9_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ9_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ10_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ10_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ13_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ13_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ14_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ14_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ15_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ15_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ16_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ16_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ17_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ17_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ18_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ18_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ19_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ19_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ20_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ20_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ21_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ21_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ22_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ22_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ23_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ23_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ24_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ24_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ25_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ25_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ26_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ26_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_IRQ31_MCU_CNT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_IRQ31_MCU_CNT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG4, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG4 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG5, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG5 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG6, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG6 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG7, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG7 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG8, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG8 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG9, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG9 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG10, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG10 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG11, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG11 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG12, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG12 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG13, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG13 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG14, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG14 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL_REG15, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL_REG15 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CBIP_CFG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CBIP_CFG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CBIP_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CBIP_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CBIP_SLV_MUX_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CBIP_SLV_MUX_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CBIP_SLV_DECODER_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CBIP_SLV_DECODER_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_MTKAIF_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_MTKAIF_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_MTKAIF_MON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_MTKAIF_MON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB2_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB2_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB2_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB2_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB2_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB2_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB2_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB2_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB2_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB2_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB2_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB2_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB2_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB2_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI2_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI2_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI2_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI2_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI2_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI2_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI2_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI2_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI2_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI2_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI2_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI2_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI2_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI2_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_MEMIF_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_MEMIF_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN0_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN0_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN1_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN1_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN2_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN2_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN3_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN3_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN4_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN4_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN5_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN5_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN6_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN6_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN7_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN7_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN8_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN8_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN9_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN9_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN10_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN10_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN11_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN11_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN12_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN12_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN13_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN13_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN14_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN14_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN15_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN15_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN16_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN16_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN17_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN17_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN18_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN18_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN19_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN19_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN20_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN20_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN21_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN21_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN22_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN22_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN23_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN23_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN24_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN24_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN25_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN25_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN26_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN26_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN27_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN27_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN28_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN28_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN29_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN29_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN30_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN30_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN31_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN31_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN32_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN32_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN33_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN33_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN34_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN34_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN_RS_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN_RS_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN_DI_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN_DI_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN_24BIT_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN_24BIT_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN_REG, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN_REG = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN35, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN35 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN36, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN36 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN37, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN37 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN38, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN38 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN35_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN35_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN36_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN36_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN37_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN37_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN38_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN38_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN39, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN39 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN40, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN40 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN41, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN41 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN42, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN42 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN39_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN39_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN40_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN40_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN41_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN41_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN42_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN42_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_I2S_CON4, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_I2S_CON4 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_TOP_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_TOP_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_UL_SRC_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_UL_SRC_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_UL_SRC_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_UL_SRC_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_SRC_DEBUG, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_SRC_DEBUG = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_SRC_DEBUG_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_SRC_DEBUG_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_02_01, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_02_01 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_04_03, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_04_03 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_06_05, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_06_05 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_08_07, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_08_07 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_10_09, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_10_09 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_12_11, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_12_11 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_14_13, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_14_13 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_16_15, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_16_15 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_18_17, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_18_17 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_20_19, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_20_19 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_22_21, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_22_21 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_24_23, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_24_23 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_26_25, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_26_25 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_28_27, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_28_27 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_ULCF_CFG_30_29, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_ULCF_CFG_30_29 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADD6A_UL_SRC_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADD6A_UL_SRC_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_UL_SRC_MON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_UL_SRC_MON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN43, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN43 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN43_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN43_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_MOD_DAI_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_MOD_DAI_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_MOD_DAI_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_MOD_DAI_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_MOD_DAI_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_MOD_DAI_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_MOD_DAI_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_MOD_DAI_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_MOD_DAI_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_MOD_DAI_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_MOD_DAI_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_MOD_DAI_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_MOD_DAI_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_MOD_DAI_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL12_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL12_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL12_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL12_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL2_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL2_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL2_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL2_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI_DATA_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI_DATA_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_MOD_DAI_DATA_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_MOD_DAI_DATA_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DAI2_DATA_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DAI2_DATA_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB2_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB2_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AWB2_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AWB2_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL3_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL3_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL3_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL3_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL4_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL4_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL4_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL4_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL5_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL5_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL5_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL5_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL6_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL6_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL6_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL6_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL1_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL1_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL1_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL1_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL2_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL2_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL2_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL2_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_RCH1_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_RCH1_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_LCH1_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_LCH1_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_RCH2_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_RCH2_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL12_LCH2_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL12_LCH2_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL3_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL3_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL3_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL3_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL4_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL4_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL4_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL4_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL5_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL5_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL5_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL5_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL6_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL6_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL6_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL6_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL7_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL7_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL7_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL7_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL8_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL8_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL8_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL8_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL5_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL5_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL5_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL5_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL5_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL5_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL5_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL5_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL5_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL5_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL5_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL5_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL5_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL5_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL6_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL6_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL6_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL6_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL6_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL6_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL6_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL6_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL6_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL6_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL6_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL6_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_VUL6_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_VUL6_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SDM_DCCOMP_CON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SDM_DCCOMP_CON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SDM_TEST, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SDM_TEST = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_DC_COMP_CFG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_DC_COMP_CFG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_DC_COMP_CFG1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_DC_COMP_CFG1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SDM_FIFO_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SDM_FIFO_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SRC_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SRC_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SRC_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SRC_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SDM_OUT_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SDM_OUT_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SDM_DITHER_CON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SDM_DITHER_CON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_SDM_AUTO_RESET_CON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_SDM_AUTO_RESET_CON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONNSYS_I2S_CON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONNSYS_I2S_CON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONNSYS_I2S_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONNSYS_I2S_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON4, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON4 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON5, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON5 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON6, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON6 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON7, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON7 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON8, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON8 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON9, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON9 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON10, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON10 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON12, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON12 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ASRC_2CH_CON13, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ASRC_2CH_CON13 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_IIR_COEF_02_01, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_IIR_COEF_02_01 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_IIR_COEF_04_03, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_IIR_COEF_04_03 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_IIR_COEF_06_05, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_IIR_COEF_06_05 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_IIR_COEF_08_07, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_IIR_COEF_08_07 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_IIR_COEF_10_09, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_IIR_COEF_10_09 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SE_PROT_SIDEBAND, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SE_PROT_SIDEBAND = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SE_DOMAIN_SIDEBAND0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SE_DOMAIN_SIDEBAND0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_PREDIS_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_PREDIS_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_PREDIS_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_PREDIS_CON3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SE_DOMAIN_SIDEBAND1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SE_DOMAIN_SIDEBAND1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SE_DOMAIN_SIDEBAND2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SE_DOMAIN_SIDEBAND2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SE_DOMAIN_SIDEBAND3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SE_DOMAIN_SIDEBAND3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN44, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN44 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN45, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN45 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN46, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN46 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN47, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN47 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN44_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN44_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN45_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN45_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN46_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN46_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN47_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN47_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL9_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL9_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL9_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL9_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL9_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL9_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL9_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL9_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_HD_ENGEN_ENABLE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_HD_ENGEN_ENABLE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_DL_NLE_FIFO_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_DL_NLE_FIFO_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_MTKAIF_CFG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_MTKAIF_CFG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_MTKAIF_SYNCWORD_CFG, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_MTKAIF_SYNCWORD_CFG = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_MTKAIF_RX_CFG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_MTKAIF_RX_CFG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_MTKAIF_RX_CFG1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_MTKAIF_RX_CFG1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_MTKAIF_RX_CFG2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_MTKAIF_RX_CFG2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_MTKAIF_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_MTKAIF_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA_MTKAIF_MON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA_MTKAIF_MON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_AUD_PAD_TOP, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_AUD_PAD_TOP = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_R_CFG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_R_CFG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_R_CFG1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_R_CFG1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_L_CFG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_L_CFG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_L_CFG1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_L_CFG1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_R_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_R_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_R_MON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_R_MON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_R_MON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_R_MON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_L_MON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_L_MON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_L_MON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_L_MON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_L_MON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_L_MON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL_NLE_GAIN_CFG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL_NLE_GAIN_CFG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_MTKAIF_CFG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_MTKAIF_CFG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_MTKAIF_RX_CFG0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_MTKAIF_RX_CFG0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_MTKAIF_RX_CFG1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_MTKAIF_RX_CFG1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_ADDA6_MTKAIF_RX_CFG2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_ADDA6_MTKAIF_RX_CFG2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON4, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON4 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON5, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON5 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON6, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON6 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON7, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON7 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON8, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON8 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON9, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON9 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON10, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON10 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON12, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON12 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL1_ASRC_2CH_CON13, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL1_ASRC_2CH_CON13 = 0x%x\n", value);
	regmap_read(afe->regmap, GENERAL_ASRC_MODE, &value);
	n += scnprintf(buffer + n, size - n,
		       "GENERAL_ASRC_MODE = 0x%x\n", value);
	regmap_read(afe->regmap, GENERAL_ASRC_EN_ON, &value);
	n += scnprintf(buffer + n, size - n,
		       "GENERAL_ASRC_EN_ON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN48, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN48 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN49, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN49 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN50, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN50 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN51, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN51 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN52, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN52 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN53, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN53 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN54, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN54 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN55, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN55 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN48_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN48_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN49_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN49_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN50_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN50_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN51_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN51_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN52_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN52_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN53_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN53_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN54_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN54_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_CONN55_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_CONN55_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON4, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON4 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON5, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON5 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON6, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON6 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON7, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON7 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON8, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON8 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON9, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON9 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON10, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON10 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON12, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON12 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_GENERAL2_ASRC_2CH_CON13, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_GENERAL2_ASRC_2CH_CON13 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL9_RCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL9_RCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL9_LCH_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL9_LCH_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL5_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL5_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL5_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL5_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL5_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL5_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL5_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL5_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL5_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL5_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL5_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL5_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL5_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL5_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL6_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL6_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL6_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL6_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL6_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL6_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL6_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL6_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL6_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL6_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL6_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL6_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL6_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL6_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL7_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL7_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL7_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL7_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL7_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL7_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL7_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL7_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL7_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL7_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL7_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL7_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL7_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL7_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL8_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL8_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL8_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL8_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL8_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL8_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL8_CUR_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL8_CUR_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL8_CUR, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL8_CUR = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL8_END_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL8_END_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL8_END, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL8_END = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL9_CON0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL9_CON0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL9_BASE_MSB, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL9_BASE_MSB = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DL9_BASE, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DL9_BASE = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SE_SECURE_CON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SE_SECURE_CON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_PROT_SIDEBAND_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_PROT_SIDEBAND_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DOMAIN_SIDEBAND0_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DOMAIN_SIDEBAND0_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DOMAIN_SIDEBAND1_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DOMAIN_SIDEBAND1_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DOMAIN_SIDEBAND2_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DOMAIN_SIDEBAND2_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_DOMAIN_SIDEBAND3_MON, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_DOMAIN_SIDEBAND3_MON = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN0, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN0 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN2, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN2 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN3, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN3 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN4, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN4 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN5, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN5 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN6, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN6 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN7, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN7 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN8, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN8 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN9, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN9 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN10, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN10 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN11, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN11 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN12, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN12 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN13, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN13 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN14, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN14 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN15, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN15 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN16, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN16 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN17, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN17 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN18, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN18 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN19, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN19 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN20, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN20 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN21, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN21 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN22, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN22 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN23, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN23 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN24, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN24 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN25, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN25 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN26, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN26 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN27, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN27 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN28, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN28 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN29, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN29 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN30, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN30 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN31, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN31 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN32, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN32 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN33, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN33 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN34, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN34 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN35, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN35 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN36, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN36 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN37, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN37 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN38, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN38 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN39, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN39 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN40, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN40 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN41, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN41 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN42, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN42 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN43, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN43 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN44, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN44 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN45, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN45 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN46, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN46 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN47, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN47 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN48, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN48 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN49, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN49 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN50, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN50 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN51, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN51 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN52, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN52 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN53, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN53 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN54, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN54 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN55, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN55 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN56, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN56 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN57, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN57 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN0_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN0_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN1_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN1_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN2_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN2_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN3_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN3_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN4_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN4_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN5_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN5_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN6_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN6_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN7_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN7_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN8_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN8_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN9_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN9_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN10_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN10_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN11_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN11_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN12_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN12_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN13_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN13_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN14_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN14_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN15_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN15_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN16_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN16_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN17_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN17_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN18_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN18_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN19_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN19_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN20_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN20_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN21_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN21_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN22_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN22_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN23_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN23_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN24_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN24_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN25_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN25_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN26_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN26_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN27_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN27_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN28_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN28_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN29_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN29_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN30_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN30_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN31_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN31_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN32_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN32_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN33_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN33_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN34_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN34_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN35_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN35_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN36_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN36_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN37_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN37_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN38_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN38_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN39_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN39_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN40_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN40_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN41_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN41_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN42_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN42_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN43_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN43_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN44_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN44_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN45_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN45_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN46_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN46_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN47_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN47_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN48_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN48_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN49_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN49_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN50_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN50_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN51_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN51_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN52_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN52_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN53_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN53_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN54_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN54_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN55_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN55_1 = 0x%x\n", value);
	regmap_read(afe->regmap, AFE_SECURE_MASK_CONN56_1, &value);
	n += scnprintf(buffer + n, size - n,
		       "AFE_SECURE_MASK_CONN56_1 = 0x%x\n", value);
	return n;
}

#ifdef CONFIG_DEBUG_FS
static ssize_t mt6833_debugfs_read(struct file *file, char __user *buf,
				   size_t count, loff_t *pos)
{
	struct mtk_base_afe *afe = file->private_data;
	const int size = AFE_SYS_DEBUG_SIZE;
	char *buffer = NULL; /* for reduce kernel stack */
	int n = 0, ret = 0;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	n = mt6833_debug_read_reg(buffer, size, afe);

	ret = simple_read_from_buffer(buf, count, pos, buffer, n);
	kfree(buffer);
	return ret;
}

static const struct mtk_afe_debug_cmd mt6833_debug_cmds[] = {
	MTK_AFE_DBG_CMD("write_reg", mtk_afe_debug_write_reg),
	{}
};

static const struct file_operations mt6833_debugfs_ops = {
	.open = mtk_afe_debugfs_open,
	.write = mtk_afe_debugfs_write,
	.read = mt6833_debugfs_read,
};
#endif


static int mt6833_dai_memif_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mt6833_memif_dai_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mt6833_memif_dai_driver);

	dai->controls = mt6833_pcm_kcontrols;
	dai->num_controls = ARRAY_SIZE(mt6833_pcm_kcontrols);
	dai->dapm_widgets = mt6833_memif_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mt6833_memif_widgets);
	dai->dapm_routes = mt6833_memif_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mt6833_memif_routes);
	return 0;
}

typedef int (*dai_register_cb)(struct mtk_base_afe *);
static const dai_register_cb dai_register_cbs[] = {
	mt6833_dai_adda_register,
	mt6833_dai_i2s_register,
	mt6833_dai_hw_gain_register,
	mt6833_dai_src_register,
	mt6833_dai_pcm_register,
	mt6833_dai_hostless_register,
	mt6833_dai_memif_register,
};

static int mt6833_afe_pcm_dev_probe(struct platform_device *pdev)
{
	int ret, i;
#if !defined(CONFIG_FPGA_EARLY_PORTING) || defined(FORCE_FPGA_ENABLE_IRQ)
	int irq_id;
#endif
	struct mtk_base_afe *afe;
	struct mt6833_afe_private *afe_priv;
	struct resource *res;
	struct device *dev;
	struct arm_smccc_res smccc_res;

	pr_info("+%s()\n", __func__);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(34));
	if (ret)
		return ret;

	afe = devm_kzalloc(&pdev->dev, sizeof(*afe), GFP_KERNEL);
	if (!afe)
		return -ENOMEM;

	platform_set_drvdata(pdev, afe);
	mt6833_set_local_afe(afe);

	afe->platform_priv = devm_kzalloc(&pdev->dev, sizeof(*afe_priv),
					  GFP_KERNEL);
	if (!afe->platform_priv)
		return -ENOMEM;

	afe_priv = afe->platform_priv;

	afe->dev = &pdev->dev;
	dev = afe->dev;

	/* init audio related clock */
	ret = mt6833_init_clock(afe);
	if (ret) {
		dev_err(dev, "init clock error: %d\n", ret);
		return ret;
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		goto err_pm_disable;

	/* Audio device is part of genpd.
	 * Set audio as syscore device to prevent
	 * genpd automatically power off audio
	 * device when suspend
	 */
	dev_pm_syscore_device(&pdev->dev, true);

	/* regmap init */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	afe->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(afe->base_addr))
		return PTR_ERR(afe->base_addr);

	/* enable clock for regcache get default value from hw */
	pm_runtime_get_sync(&pdev->dev);

	afe->regmap = devm_regmap_init_mmio(&pdev->dev, afe->base_addr,
					    &mt6833_afe_regmap_config);
	if (IS_ERR(afe->regmap))
		return PTR_ERR(afe->regmap);

	pm_runtime_put_sync(&pdev->dev);

	regcache_cache_only(afe->regmap, true);
	regcache_mark_dirty(afe->regmap);

	/* init gpio */
	ret = mt6833_afe_gpio_init(afe);
	if (ret)
		dev_err(dev, "init gpio error\n");

	/* init sram */
	afe->sram = devm_kzalloc(&pdev->dev, sizeof(struct mtk_audio_sram),
				 GFP_KERNEL);
	if (!afe->sram)
		return -ENOMEM;

	ret = mtk_audio_sram_init(dev, afe->sram, &mt6833_sram_ops);
	if (ret)
		return ret;

	/* init memif */
	afe->is_memif_bit_banding = 0;
	afe->memif_32bit_supported = 1;
	afe->memif_size = MT6833_MEMIF_NUM;
	afe->memif = devm_kcalloc(dev, afe->memif_size, sizeof(*afe->memif),
				  GFP_KERNEL);

	if (!afe->memif)
		return -ENOMEM;

	for (i = 0; i < afe->memif_size; i++) {
		afe->memif[i].data = &memif_data[i];
		afe->memif[i].irq_usage = memif_irq_usage[i];
		afe->memif[i].const_irq = 1;
	}
	afe->memif[MT6833_DEEP_MEMIF].ack = mtk_sp_clean_written_buffer_ack;

	mutex_init(&afe->irq_alloc_lock);	/* needed when dynamic irq */

	/* init irq */
	afe->irqs_size = MT6833_IRQ_NUM;
	afe->irqs = devm_kcalloc(dev, afe->irqs_size, sizeof(*afe->irqs),
				 GFP_KERNEL);

	if (!afe->irqs)
		return -ENOMEM;

	for (i = 0; i < afe->irqs_size; i++)
		afe->irqs[i].irq_data = &irq_data[i];

#if !defined(CONFIG_FPGA_EARLY_PORTING) || defined(FORCE_FPGA_ENABLE_IRQ)
	/* request irq */
	irq_id = platform_get_irq(pdev, 0);
	if (irq_id <= 0) {
		dev_err(dev, "%pOFn no irq found\n", dev->of_node);
		return irq_id < 0 ? irq_id : -ENXIO;
	}
	ret = devm_request_irq(dev, irq_id, mt6833_afe_irq_handler,
			       IRQF_TRIGGER_NONE,
			       "Afe_ISR_Handle", (void *)afe);
	if (ret) {
		dev_err(dev, "could not request_irq for Afe_ISR_Handle\n");
		return ret;
	}
	ret = enable_irq_wake(irq_id);
	if (ret < 0)
		dev_err(dev, "enable_irq_wake %d err: %d\n", irq_id, ret);
#endif

	/* init arm_smccc_smc call */
	arm_smccc_smc(MTK_SIP_AUDIO_CONTROL, MTK_AUDIO_SMC_OP_INIT,
		      0, 0, 0, 0, 0, 0, &smccc_res);

	/* init sub_dais */
	INIT_LIST_HEAD(&afe->sub_dais);

	for (i = 0; i < ARRAY_SIZE(dai_register_cbs); i++) {
		ret = dai_register_cbs[i](afe);
		if (ret) {
			dev_warn(afe->dev, "dai register i %d fail, ret %d\n",
				 i, ret);
			goto err_pm_disable;
		}
	}

	/* init dai_driver and component_driver */
	ret = mtk_afe_combine_sub_dai(afe);
	if (ret) {
		dev_warn(afe->dev, "mtk_afe_combine_sub_dai fail, ret %d\n",
			 ret);
		goto err_pm_disable;
	}

	/* others */
	afe->mtk_afe_hardware = &mt6833_afe_hardware;
	afe->memif_fs = mt6833_memif_fs;
	afe->irq_fs = mt6833_irq_fs;
	afe->get_dai_fs = mt6833_get_dai_fs;
	afe->get_memif_pbuf_size = mt6833_get_memif_pbuf_size;

	afe->runtime_resume = mt6833_afe_runtime_resume;
	afe->runtime_suspend = mt6833_afe_runtime_suspend;

	afe->request_dram_resource = mt6833_afe_dram_request;
	afe->release_dram_resource = mt6833_afe_dram_release;

	afe->copy = mt6833_afe_pcm_copy;

#if IS_ENABLED(CONFIG_DEBUG_FS)
	/* debugfs */
	afe->debug_cmds = mt6833_debug_cmds;
	afe->debugfs = debugfs_create_file("mtksocaudio",
					   S_IFREG | 0444, NULL,
					   afe, &mt6833_debugfs_ops);
#endif
	/* register component */
	ret = devm_snd_soc_register_component(&pdev->dev,
					      &mt6833_afe_component,
					      afe->dai_drivers,
					      afe->num_dai_drivers);
	if (ret) {
		dev_warn(dev, "afe component err: %d\n", ret);
		goto err_pm_disable;
	}

#if IS_ENABLED(CONFIG_SND_SOC_MTK_AUDIO_DSP)
	audio_set_dsp_afe(afe);
#endif
#if IS_ENABLED(CONFIG_MTK_ULTRASND_PROXIMITY) && !defined(SKIP_SWRDG)
	ultra_set_dsp_afe(afe);
#endif
	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int mt6833_afe_pcm_dev_remove(struct platform_device *pdev)
{
	struct mtk_base_afe *afe = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		mt6833_afe_runtime_suspend(&pdev->dev);

	/* disable afe clock */
	mt6833_afe_disable_clock(afe);
	return 0;
}

static const struct of_device_id mt6833_afe_pcm_dt_match[] = {
	{ .compatible = "mediatek,mt6833-sound", },
	{},
};
MODULE_DEVICE_TABLE(of, mt6833_afe_pcm_dt_match);

static const struct dev_pm_ops mt6833_afe_pm_ops = {
	SET_RUNTIME_PM_OPS(mt6833_afe_runtime_suspend,
			   mt6833_afe_runtime_resume, NULL)
};

static struct platform_driver mt6833_afe_pcm_driver = {
	.driver = {
		   .name = "mt6833-audio",
		   .of_match_table = mt6833_afe_pcm_dt_match,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &mt6833_afe_pm_ops,
#endif
	},
	.probe = mt6833_afe_pcm_dev_probe,
	.remove = mt6833_afe_pcm_dev_remove,
};

module_platform_driver(mt6833_afe_pcm_driver);

MODULE_DESCRIPTION("Mediatek ALSA SoC AFE platform driver for 6833");
MODULE_AUTHOR("Yujie Xiao <yujie.xiao@mediatek.com>");
MODULE_LICENSE("GPL v2");
