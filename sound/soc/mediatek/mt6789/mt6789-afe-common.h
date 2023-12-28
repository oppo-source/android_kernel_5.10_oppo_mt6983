/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6789-afe-common.h  --  Mediatek 6885 audio driver definitions
 *
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Yujie Xiao <yujie.xiao@mediatek.com>
 */

#ifndef _MT_6789_AFE_COMMON_H_
#define _MT_6789_AFE_COMMON_H_
#include <sound/soc.h>
#include <linux/list.h>
#include <linux/regmap.h>
#include <mt-plat/aee.h>
#include "mt6789-reg.h"
#include "../common/mtk-base-afe.h"

#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#define AUDIO_AEE(message) \
	(aee_kernel_exception_api(__FILE__, \
				  __LINE__, \
				  DB_OPT_FTRACE, message, \
				  "audio assert"))
#else
#define AUDIO_AEE(message) WARN_ON(true)
#endif

#define SKIP_SB

enum {
	MT6789_MEMIF_DL1,
	MT6789_MEMIF_DL12,
	MT6789_MEMIF_DL2,
	MT6789_MEMIF_DL3,
	MT6789_MEMIF_DL4,
	MT6789_MEMIF_DL5,
	MT6789_MEMIF_DL6,
	MT6789_MEMIF_DL7,
	MT6789_MEMIF_DL8,
	MT6789_MEMIF_DAI,
	MT6789_MEMIF_DAI2,
	MT6789_MEMIF_MOD_DAI,
	MT6789_MEMIF_VUL12,
	MT6789_MEMIF_VUL2,
	MT6789_MEMIF_VUL3,
	MT6789_MEMIF_VUL4,
	MT6789_MEMIF_VUL5,
	MT6789_MEMIF_VUL6,
	MT6789_MEMIF_AWB,
	MT6789_MEMIF_AWB2,
	MT6789_MEMIF_NUM,
	MT6789_DAI_ADDA = MT6789_MEMIF_NUM,
	MT6789_DAI_AP_DMIC,
	MT6789_DAI_VOW,
	MT6789_DAI_CONNSYS_I2S,
	MT6789_DAI_I2S_0,
	MT6789_DAI_I2S_1,
	MT6789_DAI_I2S_2,
	MT6789_DAI_I2S_3,
	MT6789_DAI_I2S_5,
	MT6789_DAI_HW_GAIN_1,
	MT6789_DAI_HW_GAIN_2,
	MT6789_DAI_SRC_1,
	MT6789_DAI_SRC_2,
	MT6789_DAI_PCM_2,
	MT6789_DAI_HOSTLESS_LPBK,
	MT6789_DAI_HOSTLESS_FM,
	MT6789_DAI_HOSTLESS_HW_GAIN_AAUDIO,
	MT6789_DAI_HOSTLESS_SRC_AAUDIO,
	MT6789_DAI_HOSTLESS_SPEECH,
	MT6789_DAI_HOSTLESS_SPH_ECHO_REF,
	MT6789_DAI_HOSTLESS_SPK_INIT,
	MT6789_DAI_HOSTLESS_IMPEDANCE,
	MT6789_DAI_HOSTLESS_SRC_1,	/* just an exmpale */
	MT6789_DAI_HOSTLESS_SRC_BARGEIN,
	MT6789_DAI_HOSTLESS_UL1,
	MT6789_DAI_HOSTLESS_UL2,
	MT6789_DAI_HOSTLESS_UL3,
	MT6789_DAI_HOSTLESS_UL6,
	MT6789_DAI_NUM,
};

#define MT6789_DAI_I2S_MAX_NUM 5
#define MT6789_RECORD_MEMIF MT6789_MEMIF_VUL12
#define MT6789_ECHO_REF_MEMIF MT6789_MEMIF_AWB
#define MT6789_PRIMARY_MEMIF MT6789_MEMIF_DL1
#define MT6789_FAST_MEMIF MT6789_MEMIF_DL2
#define MT6789_DEEP_MEMIF MT6789_MEMIF_DL3
#define MT6789_VOIP_MEMIF MT6789_MEMIF_DL12
#define MT6789_MMAP_DL_MEMIF MT6789_MEMIF_DL5
#define MT6789_MMAP_UL_MEMIF MT6789_MEMIF_VUL5
#define MT6789_BARGE_INMEMIF MT6789_MEMIF_AWB

enum {
	MT6789_IRQ_0,
	MT6789_IRQ_1,
	MT6789_IRQ_2,
	MT6789_IRQ_3,
	MT6789_IRQ_4,
	MT6789_IRQ_5,
	MT6789_IRQ_6,
	MT6789_IRQ_7,
	MT6789_IRQ_8,
	MT6789_IRQ_9,
	MT6789_IRQ_10,
	MT6789_IRQ_11,
	MT6789_IRQ_12,
	MT6789_IRQ_13,
	MT6789_IRQ_14,
	MT6789_IRQ_15,
	MT6789_IRQ_16,
	MT6789_IRQ_17,
	MT6789_IRQ_18,
	MT6789_IRQ_19,
	MT6789_IRQ_20,
	MT6789_IRQ_21,
	MT6789_IRQ_22,
	MT6789_IRQ_23,
	MT6789_IRQ_24,
	MT6789_IRQ_25,
	MT6789_IRQ_26,
	MT6789_IRQ_NUM,
};

/* MCLK */
enum {
	MT6789_I2S0_MCK = 0,
	MT6789_I2S1_MCK,
	MT6789_I2S2_MCK,
	MT6789_I2S3_MCK,
	MT6789_I2S4_MCK,
	MT6789_I2S4_BCK,
	MT6789_MCK_NUM,
};

struct snd_pcm_substream;
struct mtk_base_irq_data;
struct clk;

struct mt6789_afe_private {
	struct clk **clk;
	struct regmap *topckgen;
	struct regmap *apmixed;
	struct regmap *infracfg;
	int irq_cnt[MT6789_MEMIF_NUM];
	int stf_positive_gain_db;
	int dram_resource_counter;
	int sgen_mode;
	int sgen_rate;
	int sgen_amplitude;
	/* usb call */
	int usb_call_echo_ref_enable;
	int usb_call_echo_ref_size;
	bool usb_call_echo_ref_reallocate;
	/* deep buffer playback */
	int deep_playback_state;
	/* fast playback */
	int fast_playback_state;
	/* mmap playback */
	int mmap_playback_state;
	/* mmap record */
	int mmap_record_state;
	/* primary playback */
	int primary_playback_state;
	/* voip rx */
	int voip_rx_state;
	/* xrun assert */
	int xrun_assert[MT6789_MEMIF_NUM];

	/* dai */
	bool dai_on[MT6789_DAI_NUM];
	void *dai_priv[MT6789_DAI_NUM];

	/* adda */
	int mtkaif_protocol;
	int mtkaif_chosen_phase[3];
	int mtkaif_phase_cycle[3];
	int mtkaif_calibration_num_phase;
	int mtkaif_dmic;

	/* mck */
	int mck_rate[MT6789_MCK_NUM];

	/* speech mixctrl instead property usage */
	int speech_a2m_msg_id;
	int speech_md_status;
	int speech_adsp_status;
	int speech_mic_mute;
	int speech_dl_mute;
	int speech_ul_mute;
	int speech_phone1_md_idx;
	int speech_phone2_md_idx;
	int speech_phone_id;
	int speech_md_epof;
	int speech_bt_sco_wb;
	int speech_shm_init;
	int speech_shm_usip;
	int speech_shm_widx;
	int speech_md_headversion;
	int speech_md_version;
	int speech_cust_param_init;
	int speech_dynamic_dl_mute;
};

int mt6789_dai_adda_register(struct mtk_base_afe *afe);
int mt6789_dai_i2s_register(struct mtk_base_afe *afe);
int mt6789_dai_hw_gain_register(struct mtk_base_afe *afe);
int mt6789_dai_src_register(struct mtk_base_afe *afe);
int mt6789_dai_pcm_register(struct mtk_base_afe *afe);

int mt6789_dai_hostless_register(struct mtk_base_afe *afe);

int mt6789_add_misc_control(struct snd_soc_component *component);

int mt6789_set_local_afe(struct mtk_base_afe *afe);

unsigned int mt6789_general_rate_transform(struct device *dev,
					   unsigned int rate);
unsigned int mt6789_rate_transform(struct device *dev,
				   unsigned int rate, int aud_blk);
int mt6789_enable_dc_compensation(bool enable);
int mt6789_set_lch_dc_compensation(int value);
int mt6789_set_rch_dc_compensation(int value);
int mt6789_adda_dl_gain_control(bool mute);

int mt6789_dai_set_priv(struct mtk_base_afe *afe, int id,
			int priv_size, const void *priv_data);

#endif
