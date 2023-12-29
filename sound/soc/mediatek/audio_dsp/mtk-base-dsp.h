/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mtk-base-dsp.h --  Mediatek ADSP dsp base
 *
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Chipeng <Chipeng.chang@mediatek.com>
 */



#ifndef _MTK_BASE_DSP_H_
#define _MTK_BASE_DSP_H_

struct device;
struct snd_pcm_substream;
struct snd_soc_dai;
struct snd_soc_dai_driver;
struct mtk_base_dsp;
struct gen_pool;

#include <linux/genalloc.h>
#include <sound/soc.h>
#include <linux/interrupt.h>

#include "audio_buf.h"
#include <audio_messenger_ipi.h>
#include "audio_task.h"
#include "mtk-dsp-common.h"
#include "adsp_helper.h"

#define MAX_PAYLOAD_SIZE (32) /* 32bytes */

struct mtk_dsp_ipi_ops {
	void (*ipi_message_callback)(struct ipi_msg_t *ipi_msg);
	void (*ipi_handler)(struct mtk_base_dsp *dsp,
			    struct ipi_msg_t *ipi_msg);
};

struct mtk_base_dsp_mem {
	struct audio_hw_buffer adsp_buf;    /* dsp <-> audio data struct */
	struct audio_hw_buffer adsp_work_buf; /* working buffer */
	struct audio_hw_buffer audio_afepcm_buf; /* dsp <-> audio data struct */
	struct RingBuf ring_buf;
	struct snd_pcm_substream *substream;
	struct gen_pool *gen_pool_buffer;
	struct audio_dsp_dram msg_atod_share_buf;
	struct audio_dsp_dram msg_dtoa_share_buf;
	struct audio_dsp_dram dsp_ring_share_buf;
	unsigned char ipi_payload_buf[MAX_PAYLOAD_SIZE];
	unsigned int dsp_feature_counter;
	int underflowed;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
	int underflow_cnt;
#endif /* CONFIG_OPLUS_FEATURE_MM_FEEDBACK */
	spinlock_t ringbuf_lock;
	void *dsp_copy_buf;
};

struct audio_core_flag {
	/*
	 * flag with which irq occur
	 * bit mask with adsp scene
	 */
	unsigned long long atod_flag;
	unsigned long long dtoa_flag;
};

struct mtk_ap_adsp_mem {
	struct gen_pool *gen_pool_buffer;

	/* share dram  */
	struct audio_core_flag *ap_adsp_core_mem[ADSP_CORE_TOTAL];
	struct audio_dsp_dram ap_adsp_share_buf[ADSP_CORE_TOTAL];
};

struct mtk_base_dsp {
	struct device *dev;
	const struct snd_pcm_hardware *mtk_dsp_hardware;
	struct mtk_base_dsp_mem dsp_mem[AUDIO_TASK_DAI_NUM];
	struct snd_soc_dai_driver *dai_drivers;
	unsigned int num_dai_drivers;

	int (*runtime_suspend)(struct device *dev);
	int (*runtime_resume)(struct device *dev);

	int (*request_dram_resource)(struct device *dev);
	int (*release_dram_resource)(struct device *dev);

	bool suspended;
	int dsp_dram_resource_counter;
	struct mtk_dsp_ipi_ops dsp_ipi_ops;

	struct mtk_ap_adsp_mem core_share_mem;
	struct tasklet_struct dsp_tasklet[ADSP_CORE_TOTAL];

	/*
	 * 0: playback task direct to AFE DL ,
	 * aud_playback task get data from UL and write to AFE HW
	 * 1: playback task as a source write to target
	 * aud_playback task get data from source and write to AFE HW
	 */
	int dsp_ver;
};

struct mtk_adsp_task_attr {
	unsigned int default_enable; /* default setting */
	int afe_memif_dl;
	int afe_memif_ul;
	int afe_memif_ref;
	int adsp_feature_id;
	int runtime_enable;
	int ref_runtime_enable;
	unsigned int spk_protect_in_dsp;
};

#endif
