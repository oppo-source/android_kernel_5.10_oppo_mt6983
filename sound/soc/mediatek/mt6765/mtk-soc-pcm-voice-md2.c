// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Michael Hsiao <michael.hsiao@mediatek.com>
 */

/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mt_soc_pcm_voice_md2.c
 *
 * Project:
 * --------
 *   voice call platform driver
 *
 * Description:
 * ------------
 *
 *
 * Author:
 * -------
 * Tina Tsai
 *
 *------------------------------------------------------------------------------
 *
 ******************************************************************************
 */

/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/

/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include "mtk-auddrv-afe.h"
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-common.h"
#include "mtk-auddrv-def.h"
#include "mtk-auddrv-kernel.h"
#include "mtk-soc-afe-control.h"
#include "mtk-soc-analog-type.h"
#include "mtk-soc-digital-type.h"
#include "mtk-soc-pcm-common.h"
#include "mtk-soc-pcm-platform.h"
#include <linux/dma-mapping.h>

/*
 *    function implementation
 */

static int mtk_voice_md2_probe(struct platform_device *pdev);
static int mtk_voice_md2_close(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream);
static int mtk_voice_md2_component_probe(struct snd_soc_component *component);

#define MAX_PCM_DEVICES 4
#define MAX_PCM_SUBSTREAMS 128

/* defaults */
#define MAX_BUFFER_SIZE_MD2 (16 * 1024)
#define MIN_PERIOD_SIZE_MD2 64
#define USE_FORMATS (SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)
#define USE_RATE (SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000)
#define USE_RATE_MIN 8000
#define USE_RATE_MAX 48000
#define USE_CHANNELS_MIN 1
#define USE_CHANNELS_MAX 2
#define USE_PERIODS_MIN 512
#define USE_PERIODS_MAX 2048

static bool voice_md2_Status;

bool get_voice_md2_status(void)
{
	return voice_md2_Status;
}
EXPORT_SYMBOL(get_voice_md2_status);
/* for 6752 internal md 2 bring up */
static struct audio_digital_pcm Voice2IntPcm = {
	.mBclkOutInv = false,
	.mTxLchRepeatSel = Soc_Aud_TX_LCH_RPT_TX_LCH_NO_REPEAT,
	.mVbt16kModeSel = Soc_Aud_VBT_16K_MODE_VBT_16K_MODE_DISABLE,
	.mExtModemSel = Soc_Aud_EXT_MODEM_MODEM_2_USE_INTERNAL_MODEM,
	.mExtendBckSyncLength = 0,
	.mExtendBckSyncTypeSel = Soc_Aud_PCM_SYNC_TYPE_BCK_CYCLE_SYNC,
	.mSingelMicSel = Soc_Aud_BT_MODE_DUAL_MIC_ON_TX,
	/* .mAsyncFifoSel = Soc_Aud_BYPASS_SRC_SLAVE_USE_ASRC, */
	.mAsyncFifoSel =
		Soc_Aud_BYPASS_SRC_SLAVE_USE_ASYNC_FIFO,
	/* TODO: KC/TINA: For BringUp Setting */
	.mSlaveModeSel = Soc_Aud_PCM_CLOCK_SOURCE_SALVE_MODE,
	.mPcmWordLength = Soc_Aud_PCM_WLEN_LEN_PCM_16BIT,
	.mPcmModeWidebandSel = false,
	.mPcmFormat = Soc_Aud_PCM_FMT_PCM_MODE_B,
	.mModemPcmOn = false,
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(soc_voice_supported_sample_rates),
	.list = soc_voice_supported_sample_rates,
	.mask = 0,
};

static struct snd_pcm_hardware mtk_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SND_SOC_STD_MT_FMTS,
	.rates = SOC_NORMAL_USE_RATE,
	.rate_min = SOC_NORMAL_USE_RATE_MIN,
	.rate_max = SOC_NORMAL_USE_RATE_MAX,
	.channels_min = SOC_HIGH_USE_CHANNELS_MIN,
	.channels_max = SOC_HIGH_USE_CHANNELS_MAX,
	.buffer_bytes_max = MAX_BUFFER_SIZE_MD2,
	.period_bytes_max = MAX_BUFFER_SIZE_MD2,
	.periods_min = 1,
	.periods_max = 4096,
	.fifo_size = 0,
};

static int mtk_voice_md2_pcm_open(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	AudDrv_Clk_On();

	pr_debug("%s(), stream(%d)\n", __func__, substream->stream);

	runtime->hw = mtk_pcm_hardware;
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_pcm_hardware,
	       sizeof(struct snd_pcm_hardware));

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraints_sample_rates);
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0) {
		pr_err("%s(), stream(%d) snd_pcm_hw_constraint_integer failed, ret(%d)\n",
		       __func__, substream->stream, ret);
	}

	runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
	runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;

	if (ret < 0) {
		mtk_voice_md2_close(component, substream);
		return ret;
	}
	return 0;
}

static int mtk_voice_md2_close(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	pr_debug("%s(), stream(%d)\n", __func__, substream->stream);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* 3-mic setting */
		if (substream->runtime->channels > 2) {
			SetIntfConnection(
				Soc_Aud_InterCon_DisConnect,
				Soc_Aud_AFE_IO_Block_ADDA_UL2,
				Soc_Aud_AFE_IO_Block_MODEM_PCM_1_O_CH3);

			SetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_UL2,
					    false);
			if (GetMemoryPathEnable(
				    Soc_Aud_Digital_Block_ADDA_UL2) == false)
				set_adc2_enable(false);
		}
		AudDrv_Clk_Off();
		return 0;
	}

	/* todo : enable sidetone */
	/* here start digital part */
	SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			  Soc_Aud_AFE_IO_Block_ADDA_UL,
			  Soc_Aud_AFE_IO_Block_MODEM_PCM_1_O);
	SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			  Soc_Aud_AFE_IO_Block_MODEM_PCM_1_I_CH1,
			  Soc_Aud_AFE_IO_Block_I2S1_DAC);
	SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			  Soc_Aud_AFE_IO_Block_MODEM_PCM_1_I_CH1,
			  Soc_Aud_AFE_IO_Block_I2S1_DAC_2);

	SetI2SDacEnable(false);
	SetModemPcmEnable(MODEM_EXTERNAL, false);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, false);

	SetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_UL, false);
	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_UL) == false)
		set_adc_enable(false);

	EnableAfe(false);
	AudDrv_Clk_Off();

	voice_md2_Status = false;

	return 0;
}

static int mtk_voice_md2_trigger(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream, int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	}
	return 0;
}

static int mtk_voice_md2_pcm_copy(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream,
				  int channel,
				  unsigned long pos,
				  void __user *buf,
				  unsigned long bytes)
{
	return 0;
}


static void *dummy_page[2];
static struct page *mtk_pcm_page(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream,
				 unsigned long offset)
{
	return virt_to_page(dummy_page[substream->stream]); /* the same page */
}

static int mtk_voice1_ext_prepare(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtimeStream = substream->runtime;

	pr_debug("%s(), stream(%d), rate = %d  channels = %d period_size = %lu\n",
		__func__, substream->stream, runtimeStream->rate,
		runtimeStream->channels, runtimeStream->period_size);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* 3-mic setting */
		if (substream->runtime->channels > 2) {
			SetIntfConnection(
				Soc_Aud_InterCon_Connection,
				Soc_Aud_AFE_IO_Block_ADDA_UL2,
				Soc_Aud_AFE_IO_Block_MODEM_PCM_1_O_CH3);

			if (GetMemoryPathEnable(
				    Soc_Aud_Digital_Block_ADDA_UL2) == false) {
				SetMemoryPathEnable(
					Soc_Aud_Digital_Block_ADDA_UL2, true);
				set_adc2_in(substream->runtime->rate);
				set_adc2_enable(true);
			} else {
				SetMemoryPathEnable(
					Soc_Aud_Digital_Block_ADDA_UL2, true);
			}
		}
		return 0;
	}
	/* here start digital part */
	SetIntfConnection(Soc_Aud_InterCon_Connection,
			  Soc_Aud_AFE_IO_Block_ADDA_UL,
			  Soc_Aud_AFE_IO_Block_MODEM_PCM_1_O);
	SetIntfConnection(Soc_Aud_InterCon_Connection,
			  Soc_Aud_AFE_IO_Block_MODEM_PCM_1_I_CH1,
			  Soc_Aud_AFE_IO_Block_I2S1_DAC);
	SetIntfConnection(Soc_Aud_InterCon_Connection,
			  Soc_Aud_AFE_IO_Block_MODEM_PCM_1_I_CH1,
			  Soc_Aud_AFE_IO_Block_I2S1_DAC_2);

	/* start I2S DAC out */
	SetI2SDacOut(substream->runtime->rate, false,
		     Soc_Aud_I2S_WLEN_WLEN_16BITS);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_OUT_DAC, true);
	SetI2SDacEnable(true);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_UL) == false) {
		SetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_UL, true);
		set_adc_in(substream->runtime->rate);
		set_adc_enable(true);
	} else {
		SetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_UL, true);
	}

	EnableAfe(true);

	Voice2IntPcm.mPcmModeWidebandSel = SampleRateTransform(
		runtimeStream->rate, Soc_Aud_Digital_Block_MODEM_PCM_1_O);

	/* Voice2IntPcm.mAsyncFifoSel = Soc_Aud_BYPASS_SRC_SLAVE_USE_ASYNC_FIFO;
	 */
	SetModemPcmConfig(MODEM_EXTERNAL, Voice2IntPcm);
	SetModemPcmEnable(MODEM_EXTERNAL, true);

	voice_md2_Status = true;

	return 0;
}

static int mtk_pcm_hw_params(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *hw_params)
{
	int ret = 0;
	return ret;
}

static int mtk_voice_md2_hw_free(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream)
{
	pr_debug("%s()\n", __func__);
	return snd_pcm_lib_free_pages(substream);
}

static struct snd_soc_component_driver mtk_soc_voice_md2_component = {
	.name = AFE_PCM_NAME,
	.probe = mtk_voice_md2_component_probe,
	.open = mtk_voice_md2_pcm_open,
	.close = mtk_voice_md2_close,
	.hw_params = mtk_pcm_hw_params,
	.hw_free = mtk_voice_md2_hw_free,
	.prepare = mtk_voice1_ext_prepare,
	.trigger = mtk_voice_md2_trigger,
	.copy_user = mtk_voice_md2_pcm_copy,
	.page = mtk_pcm_page,

};

static int mtk_voice_md2_probe(struct platform_device *pdev)
{
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_VOICE_MD2);
	pdev->name = pdev->dev.kobj.name;

	pr_debug("%s(), dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_component(&pdev->dev,
					  &mtk_soc_voice_md2_component,
					  NULL,
					  0);
}

static int mtk_voice_md2_component_probe(struct snd_soc_component *component)
{
	pr_debug("%s()\n", __func__);
	return 0;
}

static int mtk_voice_md2_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mt_soc_pcm_voice_md2_of_ids[] = {
	{
		.compatible = "mediatek,mt_soc_pcm_voice_md2",
	},
	{} };
#endif

static struct platform_driver mtk_voice_md2_driver = {
	.driver = {

			.name = MT_SOC_VOICE_MD2,
			.owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_OF)
			.of_match_table = mt_soc_pcm_voice_md2_of_ids,
#endif
		},
	.probe = mtk_voice_md2_probe,
	.remove = mtk_voice_md2_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtk_voice_md2_dev;
#endif

int mtk_soc_voice_md2_platform_init(void)
{
	int ret = 0;

	pr_debug("%s()\n", __func__);
#ifndef CONFIG_OF
	soc_mtk_voice_md2_dev = platform_device_alloc(MT_SOC_VOICE_MD2, -1);
	if (!soc_mtk_voice_md2_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_mtk_voice_md2_dev);
	if (ret != 0) {
		platform_device_put(soc_mtk_voice_md2_dev);
		return ret;
	}
#endif
	ret = platform_driver_register(&mtk_voice_md2_driver);

	return ret;
}

void mtk_soc_voice_md2_platform_exit(void)
{

	pr_debug("%s()\n", __func__);
	platform_driver_unregister(&mtk_voice_md2_driver);
}

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");
