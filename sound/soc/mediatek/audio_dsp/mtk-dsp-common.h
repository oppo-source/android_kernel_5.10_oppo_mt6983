/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mtk-dsp-common.h  --  Mediatek dsp common function
 *
 * Copyright (c) 2018 MediaTek Inc.
 * Author: ChiPeng Chang <chipeng.chang@mediatek.com>
 */

#ifndef AUDIO_DSP_COMMON_H
#define AUDIO_DSP_COMMON_H

#include <linux/delay.h>
#include <linux/kernel.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/module.h>

#include "mtk-dsp-common_define.h"

#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#include <mt-plat/aee.h>
#endif

#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#define AUD_ASSERT(exp) \
do { \
	if (!(exp)) { \
		aee_kernel_exception_api(__FILE__, __LINE__, DB_OPT_DEFAULT, \
					 "[Audio]", "ASSERT("#exp") fail!!"); \
	} \
} while (0)
#else

#define AUD_ASSERT(exp) \
do { \
	if (!(exp)) { \
		pr_notice("ASSERT("#exp") fail: \""  __FILE__ "\", %uL\n", \
		__LINE__); \
	} \
} while (0)
#endif

/* wake lock relate*/
/* wake lock relate*/
#define aud_wake_lock_init(dev, name) wakeup_source_register(dev, name)
#define aud_wake_lock_destroy(ws) wakeup_source_destroy(ws)
#define aud_wake_lock(ws) __pm_stay_awake(ws)
#define aud_wake_unlock(ws) __pm_relax(ws)

struct mtk_base_dsp;
struct mtk_base_afe;
struct snd_dma_buffer;
struct snd_pcm_substream;
struct snd_soc_dai;
struct mtk_base_afe;
struct audio_hw_buffer;
struct platform_device;
struct ipi_msg_t;

int mtk_scp_ipi_send(int task_scene, int data_type, int ack_type,
		     uint16_t msg_id, uint32_t param1, uint32_t param2,
		     char *payload);

/* set priv data when receive IPI message */
void *get_ipi_recv_private(void);
void set_ipi_recv_private(void *priv);

void mtk_dsp_pcm_ipi_recv(struct ipi_msg_t *ipi_msg);
void mtk_dsp_handler(struct mtk_base_dsp *dsp,
		     struct ipi_msg_t *ipi_msg);

/* dsp dai id ==> task scene mapping */
int get_dspscene_by_dspdaiid(int id);
/* dsp scene  ==> dsp dai id */
int get_dspdaiid_by_dspscene(int dspscene);

/* platform dependdent , should implement in platform folder */
int dai_dsp_register(struct platform_device *pdev, struct mtk_base_dsp *dsp);
int copy_ipi_payload(void *dst, void *src, unsigned int size);

/* function warp playback buffer information send to dsp */
int afe_pcm_ipi_to_dsp(int command, struct snd_pcm_substream *substream,
		       struct snd_pcm_hw_params *params,
		       struct snd_soc_dai *dai,
		       struct mtk_base_afe *afe);

int set_dsp_base(struct mtk_base_dsp *pdsp);
void *get_dsp_base(void);

int mtk_adsp_allocate_mem(struct snd_pcm_substream *substream,
			  unsigned int size);
int mtk_adsp_free_mem(struct snd_pcm_substream *substream);

int mtk_adsp_genpool_allocate_memory(unsigned char **vaddr,
				     dma_addr_t *paddr,
				     unsigned int size,
				     int id);
int mtk_adsp_genpool_free_memory(unsigned char **vaddr,
				 size_t *size, int id);
int afe_get_pcmdir(int dir, struct audio_hw_buffer buf);
int get_dsp_task_attr(int dsp_id, int task_attr);
int get_dsp_task_id_from_str(const char *task_name);
const char *get_str_by_dsp_dai_id(const int task_id);

int audio_set_dsp_afe(struct mtk_base_afe *afe);
struct mtk_base_afe *get_afe_base(void);

int mtk_dsp_register_feature(int id);
int mtk_dsp_deregister_feature(int id);

int mtk_audio_register_notify(void);

int mtk_spk_send_ipi_buf_to_dsp(void *data_buffer, uint32_t data_size);
int mtk_spk_recv_ipi_buf_from_dsp(int8_t *buffer,
				  int16_t size,
				  uint32_t *buf_len);

#endif
