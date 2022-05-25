/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 MediaTek Inc.
 */

#ifndef _MTK_SP_SPK_AMP_H
#define _MTK_SP_SPK_AMP_H

struct mtk_spk_i2c_ctrl {
	int (*i2c_probe)(struct i2c_client *client,
			 const struct i2c_device_id *device_id);
	int (*i2c_remove)(struct i2c_client *client);
	void (*i2c_shutdown)(struct i2c_client *client);
	const char *stream_name;
	const char *codec_dai_name;
	const char *codec_name;
};

#define MTK_SPK_NOT_SMARTPA_STR "MTK_SPK_NOT_SMARTPA"
#define MTK_SPK_RICHTEK_RT5509_STR "MTK_SPK_RICHTEK_RT5509"
#define MTK_SPK_MEDIATEK_MT6660_STR "MTK_SPK_MEDIATEK_MT6660"
#define MTK_SPK_RICHTEK_RT5512_STR "MTK_SPK_MEDIATEK_RT5512"
#define MTK_SPK_GOODIX_TFA98XX_STR "MTK_SPK_GOODIX_TFAXXXX"

#define MTK_SPK_I2S_0_STR "MTK_SPK_I2S_0"
#define MTK_SPK_I2S_1_STR "MTK_SPK_I2S_1"
#define MTK_SPK_I2S_2_STR "MTK_SPK_I2S_2"
#define MTK_SPK_I2S_3_STR "MTK_SPK_I2S_3"
#define MTK_SPK_I2S_5_STR "MTK_SPK_I2S_5"
#define MTK_SPK_I2S_6_STR "MTK_SPK_I2S_6"
#define MTK_SPK_I2S_7_STR "MTK_SPK_I2S_7"
#define MTK_SPK_I2S_8_STR "MTK_SPK_I2S_8"
#define MTK_SPK_I2S_9_STR "MTK_SPK_I2S_9"
#define MTK_SPK_TINYCONN_I2S_0_STR "MTK_SPK_TINYCONN_I2S_0"

enum mtk_spk_type {
	MTK_SPK_NOT_SMARTPA = 0,
	MTK_SPK_RICHTEK_RT5509,
	MTK_SPK_MEDIATEK_MT6660,
	MTK_SPK_GOODIX_TFA98XX,
	MTK_SPK_MEDIATEK_RT5512,
	MTK_SPK_TYPE_NUM
};

enum mtk_spk_i2s_type {
	MTK_SPK_I2S_TYPE_INVALID = -1,
	MTK_SPK_I2S_0,
	MTK_SPK_I2S_1,
	MTK_SPK_I2S_2,
	MTK_SPK_I2S_3,
	MTK_SPK_I2S_5,
	MTK_SPK_I2S_6,
	MTK_SPK_I2S_7,
	MTK_SPK_I2S_8,
	MTK_SPK_I2S_9,
	MTK_SPK_TINYCONN_I2S_0,
	MTK_SPK_TINYCONN_I2S_1,
	MTK_SPK_TINYCONN_I2S_2,
	MTK_SPK_TINYCONN_I2S_3,
	MTK_SPK_TINYCONN_I2S_5,
	MTK_SPK_TINYCONN_I2S_6,
	MTK_SPK_TINYCONN_I2S_7,
	MTK_SPK_TINYCONN_I2S_8,
	MTK_SPK_TINYCONN_I2S_9,
	MTK_SPK_I2S_TYPE_NUM
};

int mtk_spk_get_type(void);
extern void mtk_spk_set_type(int spk_type);
int mtk_spk_get_i2s_out_type(void);
int mtk_spk_get_i2s_in_type(void);
int mtk_ext_spk_get_status(void);
void mtk_ext_spk_enable(int enable);
int mtk_spk_update_info(struct snd_soc_card *card,
			struct platform_device *pdev);
#endif

