// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: CY Huang <cy_huang@richtek.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
/* alsa sound header */
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>

#include "rt5509.h"

struct reg_size_table {
	u32 addr;
	u8 size;
};

static const struct reg_size_table rt5509_reg_size_table[] = {
	{ RT5509_REG_CHIPREV, 1 },
	{ RT5509_REG_EVENTINFO, 1 },
	{ RT5509_REG_DMGFLAG, 1 },
	{ RT5509_REG_CHIPEN, 1, },
	{ RT5509_REG_AUDFMT, 1, },
	{ RT5509_REG_AUDSR, 1 },
	{ RT5509_REG_I2SSEL, 1, },
	{ RT5509_REG_I2SDOLRSEL, 1 },
	{ RT5509_REG_I2SDOSEL, 1, },
	{ RT5509_REG_FUNCEN, 1, },
	{ RT5509_REG_CLIP_THR, 1, },
	{ RT5509_REG_CLIP_CTRL, 1, },
	{ RT5509_REG_CLIP_SLOPE, 2, },
	{ RT5509_REG_CLIP_VOMIN, 2 },
	{ RT5509_REG_CLIP_SIGMAX, 2 },
	{ RT5509_REG_AMPCONF, 1 },
	{ RT5509_REG_DACRNKGAIN, 1, },
	{ RT5509_REG_SAMPOFFS, 1, },
	{ RT5509_REG_SAMPCONF, 1, },
	{ RT5509_REG_DAGAIN, 2, },
	{ RT5509_REG_FFGAIN, 2, },
	{ RT5509_REG_VBATGAIN, 1 },
	{ RT5509_REG_RLDCOEF1, 1 },
	{ RT5509_REG_RLDCOEF2, 1 },
	{ RT5509_REG_BST_MODE, 1 },
	{ RT5509_REG_BST_TH1, 2 },
	{ RT5509_REG_BST_TH2, 2 },
	{ RT5509_REG_BST_TH3, 2 },
	{ RT5509_REG_BST_CONF1, 1 },
	{ RT5509_REG_BST_SIG_GAIN, 1 },
	{ RT5509_REG_BST_CONF2, 1 },
	{ RT5509_REG_BST_CONF3, 1 },
	{ RT5509_REG_OCPOTPEN, 1 },
	{ RT5509_REG_IDAC1TST, 1, },
	{ RT5509_REG_IDAC2TST, 1, },
	{ RT5509_REG_IDAC3TST, 1, },
	{ RT5509_REG_IDACTSTEN, 1, },
	{ RT5509_REG_CCMAX, 1, },
	{ RT5509_REG_OCPMAX, 1 },
	{ RT5509_REG_INTERRUPT, 2 },
	{ RT5509_REG_INTRMASK, 2 },
	{ RT5509_REG_DEGLITCH, 2 },
	{ RT5509_REG_SICRTNSTHACT, 1 },
	{ RT5509_REG_TIMEDET, 1 },
	{ RT5509_REG_TDELAY, 2 },
	{ RT5509_REG_TATKSEL, 1 },
	{ RT5509_REG_TREL, 2 },
	{ RT5509_REG_THOLDREL, 2 },
	{ RT5509_REG_STHLMT, 1 },
	{ RT5509_REG_XTHLMT, 1 },
	{ RT5509_REG_STHALC, 1 },
	{ RT5509_REG_XTHALC, 1 },
	{ RT5509_REG_INITUDT, 1 },
	{ RT5509_REG_UDT, 1 },
	{ RT5509_REG_DNHALFT, 1 },
	{ RT5509_REG_ALCGAIN, 1 },
	{ RT5509_REG_ADAPTCONF, 1 },
	{ RT5509_REG_INITIMPLDMU, 1 },
	{ RT5509_REG_IMPLDMU, 1 },
	{ RT5509_REG_GPILOT, 1 },
	{ RT5509_REG_PILOTEN, 1 },
	{ RT5509_REG_PILOTNISENSE, 1 },
	{ RT5509_REG_ISENSEGAIN, 3 },
	{ RT5509_REG_RAPP, 3 },
	{ RT5509_REG_DCR_MAX, 1 },
	{ RT5509_REG_DCR_KD, 2 },
	{ RT5509_REG_DCR_KP, 2 },
	{ RT5509_REG_DCR_KI, 1 },
	{ RT5509_REG_INITDCRIDMU, 1 },
	{ RT5509_REG_DCRIDMU, 1 },
	{ RT5509_REG_CALIB_DCR, 3 },
	{ RT5509_REG_CALIB_BL, 2 },
	{ RT5509_REG_CALIB_CTRL, 1 },
	{ RT5509_REG_CALIB_REQ, 2 },
	{ RT5509_REG_CALIB_GAIN, 2 },
	{ RT5509_REG_CALIB_OUT0, 4 },
	{ RT5509_REG_CALIB_OUT1, 4 },
	{ RT5509_REG_XTHLMTDAM, 1 },
	{ RT5509_REG_RMAXDAM, 1 },
	{ RT5509_REG_TSCALEDAM, 1 },
	{ RT5509_REG_RECOVERT, 1 },
	{ RT5509_REG_SETRESFREQ, 2 },
	{ RT5509_REG_GETRESFREQ, 2 },
	{ RT5509_REG_VOLCTL, 1 },
	{ RT5509_REG_VOLUME, 1 },
	{ RT5509_REG_CALIB_OUTX, 4 },
	{ RT5509_REG_CALIB_OUTY, 4 },
	{ RT5509_REG_BQ1, 20 },
	{ RT5509_REG_BQ2, 20 },
	{ RT5509_REG_BQ3, 20 },
	{ RT5509_REG_BQ4, 20 },
	{ RT5509_REG_BQ5, 20 },
	{ RT5509_REG_BQ6, 20 },
	{ RT5509_REG_BQ7, 20 },
	{ RT5509_REG_BQ8, 20 },
	{ RT5509_REG_BQ9, 20 },
	{ RT5509_REG_BQ10, 20 },
	{ RT5509_REG_VBBQ1, 20 },
	{ RT5509_REG_VBBQ2, 20 },
	{ RT5509_REG_VBBQ3, 20 },
	{ RT5509_REG_VBBQ4, 20 },
	{ RT5509_REG_VBBQ5, 20 },
	{ RT5509_REG_VBBQ6, 20 },
	{ RT5509_REG_VBBQ7, 20 },
	{ RT5509_REG_VBBQ8, 20 },
	{ RT5509_REG_VBBQ9, 20 },
	{ RT5509_REG_VBFCN, 24 },
	{ RT5509_REG_VBGAIN1, 4 },
	{ RT5509_REG_VBGAIN2, 4 },
	{ RT5509_REG_VBGAIN3, 4 },
	{ RT5509_REG_VBGAIN4, 4 },
	{ RT5509_REG_VBGAIN5, 4 },
	{ RT5509_REG_VBGAIN6, 4 },
	{ RT5509_REG_VBGAIN7, 4 },
	{ RT5509_REG_VBGAIN8, 4 },
	{ RT5509_REG_VBGAIN9, 4 },
	{ RT5509_REG_VBGAIN10, 4 },
	{ RT5509_REG_SLOPCONST, 1 },
	{ RT5509_REG_BWCOEFF, 1 },
	{ RT5509_REG_SWRESET, 1 },
	{ RT5509_REG_SPKGAIN, 1 },
	{ RT5509_REG_DSPKCONF1, 1 },
	{ RT5509_REG_DSPKCONF2, 1 },
	{ RT5509_REG_DSPKCONF3, 1 },
	{ RT5509_REG_DSPKCONF4, 1 },
	{ RT5509_REG_DSPKVMID, 1 },
	{ RT5509_REG_DSPKZCBOOST, 1 },
	{ RT5509_REG_ISENSE_CTRL, 1 },
	{ RT5509_REG_DIMADC, 1 },
	{ RT5509_REG_DSPKEN1, 1 },
	{ RT5509_REG_VBATDATA, 2 },
	{ RT5509_REG_VTHRMDATA, 2 },
	{ RT5509_REG_VBATSENSE, 1 },
	{ RT5509_REG_IDACTSTNINFO, 1 },
	{ RT5509_REG_IDACBOOST, 2 },
	{ RT5509_REG_DSPKEN2, 1 },
	{ RT5509_REG_DSPKIBCONF1, 1 },
	{ RT5509_REG_DSPKIBCONF2, 1 },
	{ RT5509_REG_DSPKIBCONF3, 1 },
	{ RT5509_REG_DSPKCONF5, 1 },
	{ RT5509_REG_DSPKCONF6, 1 },
	{ RT5509_REG_OVPUVPCTRL, 1 },
	{ RT5509_REG_PLLCONF1, 1 },
	{ RT5509_REG_PLLCONF2, 1 },
	{ RT5509_REG_PLLCONF3, 1 },
	{ RT5509_REG_PLLCONF4, 1 },
	{ RT5509_REG_PLLINFO, 1 },
	{ RT5509_REG_PLLDIVISOR, 4 },
	{ RT5509_REG_ZCCONF, 1 },
	{ RT5509_REG_DCADJ, 1 },
	{ RT5509_REG_I2CBCKLRCKCONF, 1 },
	{ RT5509_REG_TDEN, 1 },
	{ RT5509_REG_ALPHACONF, 1 },
	{ RT5509_REG_SPKRPTSEL, 1 },
	{ RT5509_REG_SPKRPT, 3 },
	{ RT5509_REG_NDELAY, 3 },
	{ RT5509_REG_DELAYRES, 3 },
	{ RT5509_REG_PHI1, 3 },
	{ RT5509_REG_PHI2, 3 },
	{ RT5509_REG_PHI3, 3 },
	{ RT5509_REG_PHI4, 3 },
	{ RT5509_REG_PHI5, 3 },
	{ RT5509_REG_ADAPTB0, 3 },
	{ RT5509_REG_ADAPTB1, 3 },
	{ RT5509_REG_ADAPTB2, 3 },
	{ RT5509_REG_ADAPTB3, 3 },
	{ RT5509_REG_ADAPTB4, 3 },
	{ RT5509_REG_ADAPTB5, 3 },
	{ RT5509_REG_COEFSIERA, 1 },
	{ RT5509_REG_COEFHPF, 1 },
	{ RT5509_REG_MIMATC_CTRL, 1 },
	{ RT5509_REG_TDM_CTRL, 1 },
	{ RT5509_REG_ECO_CTRL, 1 },
	{ RT5509_REG_BSTTM, 1 },
	{ RT5509_REG_ALCMINGAIN, 1 },
	{ RT5509_REG_RESVECO0, 1 },
	{ RT5509_REG_OTPCONF, 1 },
	{ RT5509_REG_OTPDIN, 3 },
	{ RT5509_REG_VBG_TRIM, 1 },
	{ RT5509_REG_VTEMP_TRIM, 2 },
	{ RT5509_REG_TCOEFF, 2 },
	{ RT5509_REG_SPSCONF, 1 },
	{ RT5509_REG_SPSTHR, 3 },
	{ RT5509_REG_VTHERMBATEN, 1 },
	{ RT5509_REG_DBGADS, 2 },
	{ RT5509_REG_TESTDAC, 3 },
	{ RT5509_REG_SPKDCS, 2 },
	{ RT5509_REG_MSKFLAG, 1 },
	{ RT5509_REG_DRCMINGAIN, 1 },
	{ RT5509_REG_DRC_SEL, 1 },
	{ RT5509_REG_DRC_ATTACK, 16 },
	{ RT5509_REG_DRC_PARAM, 7 },
	{ RT5509_REG_DRCBQ1, 20 },
	{ RT5509_REG_DRCBQ2, 20 },
	{ RT5509_REG_DRCBQ3, 20 },
	{ RT5509_REG_DRCBQ4, 20 },
	{ RT5509_REG_DRCBQ5, 20 },
	{ RT5509_REG_DRCBQ6, 20 },
	{ RT5509_REG_DRCBQ7, 20 },
	{ RT5509_REG_DRCBQ8, 20 },
	{ RT5509_REG_DRCBQ9, 20 },
	{ RT5509_REG_DRCBQ10, 20 },
	{ RT5509_REG_DRCBQ11, 20 },
	{ RT5509_REG_DRCBQ12, 20 },
	{ RT5509_REG_DRCEN, 2 },
	{ RT5509_REG_MTPFLOW1, 1 },
	{ RT5509_REG_MTPFLOW2, 1 },
	{ RT5509_REG_MTPFLOW3, 1 },
	{ RT5509_REG_MTPFLOW4, 1 },
	{ RT5509_REG_MTPFLOW5, 1 },
	{ RT5509_REG_MTPFLOW6, 1 },
	{ RT5509_REG_MTPFLOW7, 1 },
	{ RT5509_REG_MTPFLOW8, 1 },
	{ RT5509_REG_MTPFLOW9, 1 },
	{ RT5509_REG_MTPFLOWA, 1 },
	{ RT5509_REG_MTPFLOWB, 1 },
	{ RT5509_REG_MTPFLOWC, 2 },
	{ RT5509_REG_MTPFLOWD, 1 },
	{ RT5509_REG_MTPFLOWE, 1 },
	{ RT5509_REG_MTPFLOWF, 3 },
	{ RT5509_REG_TESTMODE1, 1 },
	{ RT5509_REG_RAMIND1, 1 },
	{ RT5509_REG_RAMIND2, 1 },
	{ RT5509_REG_SCANMODE, 1 },
	{ RT5509_REG_CLKEN1, 1 },
	{ RT5509_REG_CLKEN2, 1 },
	{ RT5509_REG_PADDRV, 1 },
	{ RT5509_REG_TESTMODE2, 1 },
	{ RT5509_REG_SLEWRATE1, 1 },
	{ RT5509_REG_SLEWRATE2, 1 },
	{ RT5509_REG_BIASRESISTOR, 2 },
	{ RT5509_REG_SPKDRV, 2 },
	{ RT5509_REG_BLOCKREF1, 2 },
	{ RT5509_REG_BLOCKREF2, 1 },
	{ RT5509_REG_BIASCURRENT, 1 },
	{ RT5509_REG_BIASOPTION, 1 },
};

static int rt5509_block_read(void *client, u32 reg, int bytes, void *dest)
{
	return i2c_smbus_read_i2c_block_data(client, reg, bytes, dest);
}

static int rt5509_block_write(void *client, u32 reg, int bytes, const void *src)
{
	return i2c_smbus_write_i2c_block_data(client, reg, bytes, src);
}

static int rt5509_update_bits(struct i2c_client *i2c, u8 reg, u8 mask,
			      u8 data)
{
	struct rt5509_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
	u8 read_data = 0;

	down(&chip->io_semaphore);
	ret = rt5509_block_read(i2c, reg, 1, &read_data);
	if (ret < 0)
		goto err_bits;
	read_data &= ~mask;
	read_data |= (data & mask);
	ret = rt5509_block_write(i2c, reg, 1, &read_data);
	if (ret < 0)
		goto err_bits;
err_bits:
	up(&chip->io_semaphore);
	return ret;
}

static unsigned int rt5509_io_read(struct snd_soc_component *component,
				  unsigned int reg)
{
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	int ret = 0;
	u8 data = 0;

	ret = rt5509_block_read(chip->i2c, reg, 1, &data);
	if (ret < 0)
		dev_err(chip->dev, "%s read failed, reg = %x\n", __func__, reg);
	return data;
}

static int rt5509_io_write(struct snd_soc_component *component,
			   unsigned int reg,
			   unsigned int data)
{
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);

	return rt5509_block_write(chip->i2c, reg, 1, &data);
}

struct reg_config {
	uint8_t reg_addr;
	uint32_t reg_data;
};

static const char * const prop_str[RT5509_CFG_MAX] = {
	"general",
	"boost_converter",
	"speaker_protect",
	"safe_guard",
	"eq",
	"bwe",
	"dcr",
	"mbdrc",
	"alc",
};

static const struct reg_config battmode_config[] = {
};

static const struct reg_config adaptive_config[] = {
};

static const struct reg_config general_config[] = {
	{ 0x1e, 0x0b},
	{ 0x8a, 0x02},
	{ 0x94, 0x12},
	{ 0x15, 0x00},
	{ 0x81, 0x99},
	{ 0x50, 0x00},
	{ 0x2c, 0x00},
	{ 0x5b, 0x00},
	{ 0x82, 0x42},
	{ 0x83, 0xc0},
	{ 0x84, 0xb5},
	{ 0x85, 0xd5},
	{ 0xb3, 0x00},
	{ 0x88, 0x90},
	{ 0x96, 0xfd},
	{ 0x93, 0x55},
	{ 0x26, 0x00},
	{ 0x8d, 0xe0},
	{ 0x14, 0x7fff},
	{ 0xea, 0x81},
	{ 0xeb, 0xa1},
	{ 0xc3, 0x10},
	{ 0xfd, 0xf0},
	{ 0xee, 0x06},
	{ 0x86, 0x18},
	{ 0x13, 0x3bbb},
	{ 0x14, 0x6c00},
	{ 0x16, 0x4c},
	{ 0x18, 0x4a},
	{ 0x19, 0x016a},
	{ 0x1a, 0x0200},
	{ 0x1b, 0x77},
	{ 0x1c, 0x70},
	{ 0x1d, 0x80},
	{ 0x1f, 0x05d3},
	{ 0x20, 0x03cd},
	{ 0x21, 0x0209},
	{ 0x22, 0x07},
	{ 0x23, 0x47},
	{ 0x24, 0xc1},
	{ 0x25, 0x4f},
	{ 0x43, 0x44},
	{ 0x44, 0xb3},
	{ 0x45, 0x1a},
	{ 0x46, 0x800000},
	{ 0x81, 0xb9},
	{ 0x84, 0x35},
	{ 0x86, 0x14},
	{ 0x87, 0x04},
	{ 0x92, 0x57},
	{ 0x93, 0x54},
	{ 0xc0, 0x20},
	{ 0xc2, 0x000097},
	{ 0xeb, 0xa2},
	{ 0xec, 0x7474},
	{ 0xef, 0x001201},
	{ 0xf8, 0x55},
	{ 0xf9, 0x01},
	{ 0xfa, 0x0711},
};

static const struct reg_config revc_general_config[] = {
	{ 0x14, 0x6000},
	{ 0x15, 0xB8},
	{ 0x16, 0x5d},
	{ 0x18, 0x0a},
	{ 0x19, 0x016a},
	{ 0x1a, 0x0000},
	{ 0x1c, 0x70},
	{ 0x1d, 0x80},
	{ 0x1e, 0x0b},
	{ 0x1f, 0x085b},
	{ 0x20, 0x05cf},
	{ 0x21, 0x02c7},
	{ 0x22, 0x0d},
	{ 0x23, 0x6f},
	{ 0x24, 0xb1},
	{ 0x25, 0x08},
	{ 0x2c, 0x00},
	{ 0x2d, 0x0100},
	{ 0x2f, 0x0559},
	{ 0x45, 0x0b},
	{ 0x5a, 0x07dc},
	{ 0x81, 0xbb},
	{ 0x82, 0x43},
	{ 0x83, 0x54},
	{ 0x86, 0x3f},
	{ 0x87, 0x1f},
	{ 0x8a, 0x02},
	{ 0x8d, 0xc0},
	{ 0x93, 0xd4},
	{ 0x94, 0x12},
	{ 0x96, 0x0d},
	{ 0x97, 0xa5},
	{ 0xb8, 0x80},
	{ 0xea, 0x91},
	{ 0xeb, 0xa4},
	{ 0xec, 0x7474},
	{ 0xee, 0x06},
	{ 0xf8, 0xd7},
	{ 0xfd, 0xd0},
	{ 0xef, 0x001201},
	{ 0xb4, 0x81},
};

static const struct reg_config revd_general_config[] = {
	{ 0x14, 0x6800},
	{ 0x16, 0x95},
	{ 0x18, 0xaa},
	{ 0x19, 0x016a},
	{ 0x1a, 0x0000},
	{ 0x1c, 0x50},
	{ 0x1d, 0x00},
	{ 0x1e, 0x0b},
	{ 0x1f, 0x085b},
	{ 0x20, 0x05cf},
	{ 0x21, 0x02c7},
	{ 0x22, 0x05},
	{ 0x24, 0xb1},
	{ 0x25, 0x04},
	{ 0x2c, 0x00},
	{ 0x2f, 0x0559},
	{ 0x45, 0x08},
	{ 0x5b, 0x00},
	{ 0x81, 0xbb},
	{ 0x82, 0x43},
	{ 0x83, 0x54},
	{ 0x86, 0x38},
	{ 0x87, 0x1f},
	{ 0x92, 0x54},
	{ 0x93, 0xd4},
	{ 0x94, 0x12},
	{ 0x9a, 0xdc},
	{ 0xb3, 0x00},
	{ 0xea, 0x80},
	{ 0xf9, 0x01},
	{ 0xfd, 0xd0},
	{ 0xef, 0x001201},
	{ 0xee, 0x05},
	{ 0xb4, 0x81},
	{ 0xba, 0x80},
	{ 0x2b, 0x67},
};

static inline int rt5509_power_on(struct rt5509_chip *chip, bool en)
{
	int ret = 0;

	dev_dbg(chip->dev, "%s: en %d\n", __func__, en);
	if (en) {
		ret = rt5509_update_bits(chip->i2c, RT5509_REG_CHIPEN,
					 RT5509_CHIPPD_ENMASK, 0);
	} else {
		ret = rt5509_update_bits(chip->i2c, RT5509_REG_CHIPEN,
					 RT5509_CHIPPD_ENMASK,
					 RT5509_CHIPPD_ENMASK);
	}
	mdelay(1);
	return ret;
}

static int rt5509_set_bias_level(struct snd_soc_component *component,
	enum snd_soc_bias_level level)
{
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_get_dapm(component);
	int ret = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		dapm->bias_level = level;
		break;
	case SND_SOC_BIAS_STANDBY:
		if (dapm->bias_level != SND_SOC_BIAS_OFF) {
			dapm->bias_level = level;
			break;
		}
		ret = rt5509_power_on(chip, true);
		if (ret < 0)
			goto out_set_bias;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKCONF5,
						    RT5509_VBG_ENMASK,
						    RT5509_VBG_ENMASK);
		if (ret < 0)
			goto out_set_bias;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKEN1,
						    RT5509_BIAS_ENMASK,
						    RT5509_BIAS_ENMASK);
		if (ret < 0)
			goto out_set_bias;
		ret = snd_soc_component_write(component, RT5509_REG_BST_MODE,
				    chip->mode_store);
		if (ret < 0)
			goto out_set_bias;
		dapm->bias_level = level;
		ret = 0;
		break;
	case SND_SOC_BIAS_OFF:
		ret = snd_soc_component_read(component, RT5509_REG_BST_MODE);
		if (ret < 0)
			goto out_set_bias;
		chip->mode_store = ret;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_BST_MODE,
						    0x03, 0x00);
		if (ret < 0)
			goto out_set_bias;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKEN1,
						    RT5509_BIAS_ENMASK,
						    ~RT5509_BIAS_ENMASK);
		if (ret < 0)
			goto out_set_bias;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKCONF5,
						    RT5509_VBG_ENMASK,
						    ~RT5509_VBG_ENMASK);
		if (ret < 0)
			goto out_set_bias;
		ret = snd_soc_component_read(component, RT5509_REG_INTERRUPT);
		if (ret < 0)
			goto out_set_bias;
		ret = rt5509_power_on(chip, false);
		if (ret < 0)
			goto out_set_bias;
		dapm->bias_level = level;
		ret = 0;
		break;
	default:
		ret = -EINVAL;
	}
out_set_bias:
	dev_info(component->dev, "%s: bias_level %d\n", __func__, level);
	return ret;
}

static int rt5509_init_battmode_setting(struct snd_soc_component *component)
{
	int i = 0, ret = 0;

	dev_dbg(component->dev, "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(battmode_config); i++) {
		ret = snd_soc_component_write(component,
					      battmode_config[i].reg_addr,
					      battmode_config[i].reg_data);
		if (ret < 0)
			break;
	}
	return ret;
}

static int rt5509_init_adaptive_setting(struct snd_soc_component *component)
{
	int i = 0, ret = 0;

	dev_dbg(component->dev, "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(adaptive_config); i++) {
		ret = snd_soc_component_write(component,
					      adaptive_config[i].reg_addr,
					      adaptive_config[i].reg_data);
		if (ret < 0)
			break;
	}
	return ret;
}

static int rt5509_init_general_setting(struct snd_soc_component *component)
{
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	const struct reg_config *reg_cfg;
	int reg_cfg_size = 0;
	int i = 0, ret = 0;

	dev_dbg(component->dev, "%s\n", __func__);
	if (chip->chip_rev >= RT5509_CHIP_REVD) {
		reg_cfg = revd_general_config;
		reg_cfg_size = ARRAY_SIZE(revd_general_config);
	} else if (chip->chip_rev >= RT5509_CHIP_REVC) {
		reg_cfg = revc_general_config;
		reg_cfg_size = ARRAY_SIZE(revc_general_config);
	} else {
		reg_cfg = general_config;
		reg_cfg_size = ARRAY_SIZE(general_config);
	}
	for (i = 0; i < reg_cfg_size; i++) {
		ret = snd_soc_component_write(component, reg_cfg[i].reg_addr,
				    reg_cfg[i].reg_data);
		if (ret < 0)
			break;
	}
	return ret;
}

static int rt5509_do_tcsense_fix(struct snd_soc_component *component)
{
	uint32_t tc_sense = 0, vtemp = 0;
	int ret = 0;

	dev_dbg(component->dev, "%s\n", __func__);
	ret = snd_soc_component_update_bits(component,
					    RT5509_REG_MTPFLOWB, 0x40, 0x40);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_write(component, RT5509_REG_OTPCONF, 0x82);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_write(component, RT5509_REG_OTPCONF, 0x00);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_update_bits(component,
					    RT5509_REG_MTPFLOWB, 0x40, 0x00);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_read(component, RT5509_REG_VTEMP_TRIM);
	if (ret < 0)
		return ret;
	vtemp = ret & 0xffff;
	if (vtemp < 0x4250) {
		tc_sense = 0xffff;
		goto bypass_tcsense_overflow;
	}
	tc_sense = (1073741824U / vtemp * (273 - 40) / (265 - 40)) << 1;
	if (tc_sense & 0x10000)
		tc_sense = (tc_sense & 0xffff) + 1;
	tc_sense &= 0xffff;
bypass_tcsense_overflow:
	dev_dbg(component->dev, "tc_sense %04x\n", tc_sense);
	return snd_soc_component_write(component, RT5509_REG_TCOEFF, tc_sense);
}

static int rt5509_adap_coefficent_fix(struct snd_soc_component *component)
{
	int i, ret = 0;
	int64_t x = 0, y = 0, z = 0, w = 0;

	dev_dbg(component->dev, "%s\n", __func__);
	ret = snd_soc_component_read(component, RT5509_REG_ISENSEGAIN);
	ret &= 0xffffff;
	dev_info(component->dev, "gsense otp -> 0x%08x\n", ret);
	/* gsense otp value */
	x = ret;
	ret = snd_soc_component_read(component, RT5509_REG_CALIB_DCR);
	ret &= 0xffffff;
	dev_info(component->dev, "dcr otp -> 0x%08x\n", ret);
	if (ret == 0xffffff || ret == 0)
		ret = 0x800000;
	/* rspk otp value */
	w = ret;
	for (i = 0; i < 6; i++) {
		ret = snd_soc_component_read(component, RT5509_REG_ADAPTB0 + i);
		ret &= 0xffffff;
		dev_info(component->dev, "b factor before 0x%08x\n", ret);
		/* y = phi factor */
		y = ret;
		if (ret < 0x800000) {
			z = div64_s64(x * 1000000, w) * y;
			z = div_s64(z, 1000000);
		} else {
			y = ((int64_t)0xffffff - y) * div64_s64(x * 1000000, w);
			z = (int64_t)0xffffff * (int64_t)1000000;
			z = z - y;
			z = div_s64(z, 1000000);
		}
		ret = z & 0xffffff;
		dev_info(component->dev, "b factor after 0x%08x\n", ret);
		ret = snd_soc_component_write(component,
					      RT5509_REG_ADAPTB0 + i, ret);
		if (ret < 0) {
			dev_err(component->dev, "fix b factor fail %d\n", i);
			return ret;
		}
	}
	return 0;
}

static int rt5509_init_impedance_ctrl_fix(struct snd_soc_component *component)
{
	u32 gsense_otp, rspk_otp, result;
	int ret = 0;

	dev_dbg(component->dev, "%s\n", __func__);
	ret = snd_soc_component_read(component, RT5509_REG_ISENSEGAIN);
	if (ret == 0)
		ret = 0x800000;
	gsense_otp = ret & 0xffffff;
	dev_dbg(component->dev, "gsense otp 0x%08x\n", gsense_otp);
	ret = snd_soc_component_read(component, RT5509_REG_CALIB_DCR);
	if (ret == 0)
		ret = 0x800000;
	rspk_otp = ret & 0xffffff;
	dev_dbg(component->dev, "rspk otp 0x%08x\n", rspk_otp);
	result = ((rspk_otp << 7) / gsense_otp) << 16;
	result &= 0xffffff;
	dev_dbg(component->dev, "final result 0x%08x\n", result);
	return snd_soc_component_write(component, RT5509_REG_DELAYRES, result);
}

static int rt5509_init_proprietary_setting(struct snd_soc_component *component)
{
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	struct rt5509_proprietary_param *p_param = chip->pdata->p_param;
	const u8 *cfg;
	u32 cfg_size;
	int i = 0, j = 0;
	int ret = 0;

	dev_dbg(component->dev, "%s\n", __func__);
	if (!p_param)
		goto out_init_proprietary;
	for (i = 0; i < RT5509_CFG_MAX; i++) {
		cfg = p_param->cfg[i];
		cfg_size = p_param->cfg_size[i];
		if (!cfg)
			continue;
		dev_dbg(chip->dev, "%s start\n", prop_str[i]);
		for (j = 0; j < cfg_size;) {
			ret = rt5509_block_write(chip->i2c, cfg[0], cfg[1],
					      cfg + 2);
			if (ret < 0)
				dev_err(chip->dev, "set %02x fail\n", cfg[0]);
			j += (2 + cfg[1]);
			cfg += (2 + cfg[1]);
		}
		dev_dbg(chip->dev, "%s end\n", prop_str[i]);
	}
	ret = rt5509_adap_coefficent_fix(component);
	if (ret < 0)
		dev_err(chip->dev, "fix adap coefficient fail\n");
	ret = rt5509_init_impedance_ctrl_fix(component);
	if (ret < 0)
		dev_err(chip->dev, "init impedance ctrl fix fail\n");
	if (p_param->cfg_size[RT5509_CFG_SPEAKERPROT]) {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_SPKPROT_ENMASK,
						    RT5509_SPKPROT_ENMASK);
	}
out_init_proprietary:
	return ret;
}

static ssize_t rt5509_proprietary_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct rt5509_chip *chip = dev_get_drvdata(dev);
	struct rt5509_proprietary_param *param = chip->pdata->p_param;
	int i = 0, j = 0;
	const u8 *cfg;
	u32 cfg_size;

	dev_dbg(chip->dev, "%s\n", __func__);
	if (!param) {
		i += scnprintf(buf + i, PAGE_SIZE - i, "no proprietary parm\n");
		goto out_show;
	}
	for (j = 0; j < RT5509_CFG_MAX; j++) {
		cfg = param->cfg[j];
		cfg_size = param->cfg_size[j];
		if (!cfg) {
			i += scnprintf(buf + i, PAGE_SIZE - i, "no %s cfg\n",
				       prop_str[j]);
			continue;
		}
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s size %d\n",
			       prop_str[j], cfg_size);
	}
out_show:
	return i;
}

static ssize_t rt5509_proprietary_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t cnt)
{
	struct rt5509_chip *chip = dev_get_drvdata(dev);
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_get_dapm(chip->component);
	struct rt5509_proprietary_param *param = NULL;
	int i = 0, size = 0;
	const u8 *bin_offset;
	u32 *sptr;

	dev_dbg(chip->dev, "%s, size %d\n", __func__, (int)cnt);
	/* do mark check */
	if (cnt < 7) {
		dev_err(chip->dev, "data is invalid\n");
		goto out_param_write;
	}
	if (strncmp("richtek", buf, 7)) {
		dev_err(chip->dev, "data is invalid\n");
		goto out_param_write;
	}
	/* do size check */
	sptr = (u32 *)(buf + cnt - 4);
	size = *sptr;
	dev_dbg(chip->dev, "size %d\n", size);
	if (cnt < 47 || (size + 47) != cnt) {
		dev_err(chip->dev, "sorry bin size is wrong\n");
		goto out_param_write;
	}
	for (i = 0; i < RT5509_CFG_MAX; i++) {
		sptr = (u32 *)(buf + cnt - 40 + i * 4);
		size -= *sptr;
	}
	if (size != 0) {
		dev_err(chip->dev, "sorry, bin format is wrong\n");
		goto out_param_write;
	}
	/* if previous one is existed, release it */
	param = chip->pdata->p_param;
	if (param) {
		dev_dbg(chip->dev, "previous existed\n");
		for (i = 0; i < RT5509_CFG_MAX; i++)
			devm_kfree(chip->dev, param->cfg[i]);
		devm_kfree(chip->dev, param);
		chip->pdata->p_param = NULL;
	}
	/* start to copy */
	param = devm_kzalloc(chip->dev, sizeof(*param), GFP_KERNEL);
	if (!param)
		goto out_param_write;

	bin_offset = buf + 7;
	for (i = 0; i < RT5509_CFG_MAX; i++) {
		sptr = (u32 *)(buf + cnt - 40 + i * 4);
		param->cfg_size[i] = *sptr;
		param->cfg[i] = devm_kzalloc(chip->dev,
					     sizeof(u8) * param->cfg_size[i],
					     GFP_KERNEL);
		memcpy(param->cfg[i], bin_offset, param->cfg_size[i]);
		bin_offset += param->cfg_size[i];
	}
	chip->pdata->p_param = param;
	if (dapm->bias_level != SND_SOC_BIAS_OFF)
		goto out_param_write;
	if (rt5509_power_on(chip, true) < 0)
		goto out_param_write;
	rt5509_do_tcsense_fix(chip->component);
	rt5509_init_proprietary_setting(chip->component);
	if (rt5509_power_on(chip, false) < 0)
		goto out_param_write;
	return cnt;
out_param_write:
	return -EINVAL;
}

static struct device_attribute rt5509_proprietary_attr = {
	.attr = {
		.name = "prop_param",
		.mode = 0644,
	},
	.show = rt5509_proprietary_show,
	.store = rt5509_proprietary_store,
};

static int rt5509_param_probe(struct platform_device *pdev)
{
	struct rt5509_chip *chip = dev_get_drvdata(pdev->dev.parent);
	int ret = 0;

	ret = device_create_file(&pdev->dev, &rt5509_proprietary_attr);
	if (ret < 0) {
		dev_err(&pdev->dev, "create file error\n");
		return ret;
	}
	platform_set_drvdata(pdev, chip);
	return 0;
}

static int rt5509_param_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &rt5509_proprietary_attr);
	return 0;
}

static struct platform_driver rt5509_param_driver = {
	.driver = {
		.name = "rt5509_param",
		.owner = THIS_MODULE,
	},
	.probe = rt5509_param_probe,
	.remove = rt5509_param_remove,
};
static int param_drv_registered;

static int rt5509_param_create(struct rt5509_chip *chip)
{
	chip->pdev = platform_device_register_data(chip->dev, "rt5509_param",
						   chip->dev_cnt, NULL, 0);
	if (!chip->pdev)
		return -EFAULT;
	if (!param_drv_registered) {
		param_drv_registered = 1;
		return platform_driver_register(&rt5509_param_driver);
	}
	return 0;
}

static void rt5509_param_destroy(struct rt5509_chip *chip)
{
	platform_device_unregister(chip->pdev);
	platform_driver_unregister(&rt5509_param_driver);
}

static int rt5509_component_probe(struct snd_soc_component *component)
{
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(component->dev, "%s\n", __func__);

	/* CHIP Enable */
	ret = snd_soc_component_update_bits(component, RT5509_REG_CHIPEN,
		RT5509_CHIPPD_ENMASK, ~RT5509_CHIPPD_ENMASK);
	if (ret < 0)
		goto err_out_probe;
	ret = rt5509_init_general_setting(component);
	if (ret < 0)
		goto err_out_probe;
	ret = rt5509_init_adaptive_setting(component);
	if (ret < 0)
		goto err_out_probe;
	ret = rt5509_init_battmode_setting(component);
	if (ret < 0)
		goto err_out_probe;
	ret = rt5509_do_tcsense_fix(component);
	if (ret < 0)
		goto err_out_probe;
	ret = rt5509_init_proprietary_setting(component);
	if (ret < 0)
		goto err_out_probe;
	ret = rt5509_param_create(chip);
	if (ret < 0)
		goto err_out_probe;
	chip->component = component;
	ret = rt5509_calib_create(chip);
	if (ret < 0)
		goto err_out_probe;
	dev_info(component->dev, "%s\n", __func__);
	return rt5509_set_bias_level(component, SND_SOC_BIAS_OFF);
err_out_probe:
	dev_info(component->dev, "chip io error\n");
	/* Chip Disable */
	ret = snd_soc_component_update_bits(component, RT5509_REG_CHIPEN,
		RT5509_CHIPPD_ENMASK, RT5509_CHIPPD_ENMASK);
	return ret < 0 ? ret : -EINVAL;
}

static void rt5509_component_remove(struct snd_soc_component *component)
{
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	int ret = 0;

	rt5509_calib_destroy(chip);
	rt5509_param_destroy(chip);
	ret = rt5509_set_bias_level(component, SND_SOC_BIAS_OFF);
	if (ret)
		dev_err(component->dev, "%s set bias level failed\n", __func__);
}

#ifdef CONFIG_PM
static int rt5509_component_suspend(struct snd_soc_component *component)
{
	return 0;
}

static int rt5509_component_resume(struct snd_soc_component *component)
{
	return 0;
}
#else
#define rt5509_component_suspend NULL
#define rt5509_component_resume NULL
#endif /* #ifdef CONFIG_PM */

static int rt5509_clk_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = snd_soc_component_read(component, RT5509_REG_OTPCONF);
		if (ret < 0)
			return ret;
		/* check bit 6 and 5 */
		ret &= 0x60;
		if (ret) {
			dev_err(component->dev,
				"MTP check fail, val 0x%x\n", ret);
			return -EINVAL;
		}
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CLKEN1,
						    0x40, 0x00);
		if (ret < 0)
			return ret;
		break;
	default:
		break;
	}
	return 0;
}

static int rt5509_boost_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKVMID,
						    RT5509_VMID_ENMASK,
						    RT5509_VMID_ENMASK);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_TRIWAVE_ENMASK,
						    RT5509_TRIWAVE_ENMASK);
		if (ret < 0)
			goto out_boost_event;
		mdelay(1);
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_MSKFLAG,
						    0x3F, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component, RT5509_REG_BSTTM,
						    0x40, 0x40);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_OCPOTPEN,
						    0x03, 0x03);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_write(component,
					      RT5509_REG_OCPMAX, 0x7f);
		if (ret < 0)
			goto out_boost_event;
		if (chip->chip_rev >= RT5509_CHIP_REVD) {
			ret = snd_soc_component_update_bits(component,
				RT5509_REG_OVPUVPCTRL, 0xe0, 0xe0);
			if (ret < 0)
				goto out_boost_event;
		} else {
			ret = snd_soc_component_update_bits(component,
				RT5509_REG_OVPUVPCTRL, 0x60, 0x60);
			if (ret < 0)
				goto out_boost_event;
		}
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CLKEN1,
						    0x40, 0x40);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_write(component,
					      RT5509_REG_DSPKCONF4, 0xd9);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_MSKFLAG,
						    0x3F, 0x3F);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_BLOCKREF1,
						    0x0300, 0x0100);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_MTPFLOWA,
						    0x10, 0x10);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKEN1,
						    RT5509_BUF_ENMASK,
						    RT5509_BUF_ENMASK);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_AMPCONF,
						    0x80, 0x80);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_VBATSENSE,
						    0x20, 0x20);
		if (ret < 0)
			goto out_boost_event;
		if (chip->chip_rev >= RT5509_CHIP_REVD) {
			ret = snd_soc_component_update_bits(component,
				RT5509_REG_BIASRESISTOR, 0x0001, 0x0000);
			if (ret < 0)
				goto out_boost_event;
		}
		mdelay(6);
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_AMPCONF,
						    0x38, 0x38);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_read(component, RT5509_REG_BST_MODE);
		if (ret < 0)
			goto out_boost_event;
		chip->mode_store = ret;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_BST_MODE,
						    0x03, 0x01);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_write(component,
					      RT5509_REG_ISENSE_CTRL, 0x97);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_MTPFLOWA,
						    0x04, 0x04);
		if (ret < 0)
			goto out_boost_event;
		mdelay(1);
		dev_info(chip->dev, "amp turn on\n");
		break;
	case SND_SOC_DAPM_POST_PMU:
		mdelay(11);
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_PILOTNISENSE,
						    0x07, 0x03);
		if (ret < 0)
			goto out_boost_event;
		if (chip->chip_rev >= RT5509_CHIP_REVD) {
			ret = snd_soc_component_update_bits(component,
							    RT5509_REG_MTPFLOWA,
							    0x01, 0x01);
			if (ret < 0)
				goto out_boost_event;
		}
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_BST_MODE,
						    0x03, chip->mode_store);
		if (ret < 0)
			goto out_boost_event;
		if (!chip->recv_spec_set) {
			ret = snd_soc_component_update_bits(component,
							    RT5509_REG_PILOTEN,
							    0x01, 0x01);
			if (ret < 0)
				goto out_boost_event;
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ret = snd_soc_component_read(component, RT5509_REG_CHIPEN);
		if (ret < 0)
			goto out_boost_event;
		if (ret & RT5509_SPKPROT_ENMASK) {
			ret = snd_soc_component_update_bits(component,
				RT5509_REG_CHIPEN,  RT5509_SPKPROT_ENMASK,
				~RT5509_SPKPROT_ENMASK);
			if (ret < 0)
				goto out_boost_event;
			ret = snd_soc_component_update_bits(component,
				RT5509_REG_CHIPEN, RT5509_SPKPROT_ENMASK,
				RT5509_SPKPROT_ENMASK);
			if (ret < 0)
				goto out_boost_event;
		}
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_PILOTEN,
						    0x01, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_PILOTNISENSE,
						    0x07, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKCONF1,
						    0x20, 0x20);
		if (ret < 0)
			goto out_boost_event;
		mdelay(1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		dev_info(chip->dev, "amp turn off\n");
		mdelay(1);
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKCONF1,
						    0x20, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_SPKAMP_ENMASK,
						    RT5509_SPKAMP_ENMASK);
		if (ret < 0)
			goto out_boost_event;
		mdelay(11);
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKCONF1,
						    0x20, 0x20);
		if (ret < 0)
			goto out_boost_event;
		mdelay(1);
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_SPKAMP_ENMASK,
						    ~RT5509_SPKAMP_ENMASK);
		if (ret < 0)
			goto out_boost_event;
		mdelay(1);
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKCONF1,
						    0x20, 0x00);
		if (ret < 0)
			goto out_boost_event;
		mdelay(1);
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_MTPFLOWA,
						    0x05, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_write(component,
					      RT5509_REG_ISENSE_CTRL, 0x03);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_AMPCONF,
						    0x38, 0x00);
		if (ret < 0)
			goto out_boost_event;
		if (chip->chip_rev >= RT5509_CHIP_REVD) {
			ret = snd_soc_component_update_bits(component,
				RT5509_REG_BIASRESISTOR, 0x0001, 0x0001);
			if (ret < 0)
				goto out_boost_event;
		}
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_VBATSENSE,
						    0x20, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_AMPCONF,
						    0x80, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKEN1,
						    RT5509_BUF_ENMASK,
						    ~RT5509_BUF_ENMASK);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_MTPFLOWA,
						    0x10, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_BLOCKREF1,
						    0x0300, 0x0000);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_write(component,
					      RT5509_REG_DSPKCONF4, 0x15);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_OVPUVPCTRL,
						    0xe0, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_write(component,
					      RT5509_REG_OCPMAX, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_OCPOTPEN,
						    0x03, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component, RT5509_REG_BSTTM,
			0x40, 0x00);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_TRIWAVE_ENMASK,
						    ~RT5509_TRIWAVE_ENMASK);
		if (ret < 0)
			goto out_boost_event;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKVMID,
						    RT5509_VMID_ENMASK,
						    ~RT5509_VMID_ENMASK);
		if (ret < 0)
			goto out_boost_event;
		break;
	default:
		break;
	}
out_boost_event:
	return ret;
}

static const char * const rt5509_i2smux_text[] = { "I2S1", "I2S2"};
static const char * const rt5509_i2sdomux_text[] = { "I2SDOR/L", "DATAI3"};
static SOC_ENUM_SINGLE_DECL(rt5509_i2s_muxsel,
	SND_SOC_NOPM, 0, rt5509_i2smux_text);
static SOC_ENUM_SINGLE_DECL(rt5509_i2s_dosel,
	RT5509_REG_I2SDOSEL, 1, rt5509_i2sdomux_text);
static const struct snd_kcontrol_new rt5509_i2smux_ctrl =
	SOC_DAPM_ENUM("Switch", rt5509_i2s_muxsel);
static const struct snd_kcontrol_new rt5509_i2sdo_ctrl =
	SOC_DAPM_ENUM("Switch", rt5509_i2s_dosel);
static const struct snd_soc_dapm_widget rt5509_dapm_widgets[] = {
	SND_SOC_DAPM_MUX("I2S Mux", SND_SOC_NOPM, 0, 0, &rt5509_i2smux_ctrl),
	SND_SOC_DAPM_MUX("I2SDO Mux", RT5509_REG_I2SDOSEL, 0, 0,
		&rt5509_i2sdo_ctrl),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA("PGA", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_OUT_DRV_E("BOOST", RT5509_REG_CHIPEN, RT5509_SPKAMP_ENSHFT,
		0, NULL, 0, rt5509_boost_event, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("CLK", RT5509_REG_PLLCONF1, 0, 1, rt5509_clk_event,
		SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const struct snd_soc_dapm_route rt5509_dapm_routes[] = {
	{ "I2S Mux", "I2S1", "AIF1 Playback"},
	{ "I2S Mux", "I2S2", "AIF2 Playback"},
	/* DATAO path start */
	{ "I2SDO Mux", "I2SDOR/L", "I2S Mux"},
	{ "I2SDO Mux", "DATAI3", "I2S Mux"},
	{ "AIF1 Capture", NULL, "I2SDO Mux"},
	/* DATAO path end */
	{ "DAC", NULL, "I2S Mux"},
	{ "DAC", NULL, "CLK"},
	{ "PGA", NULL, "DAC"},
	{ "BOOST", NULL, "PGA"},
	{ "Speaker", NULL, "BOOST"},
};

static int rt5509_alcfixed_gain_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	int ret = 0;

	if (!chip->rlr_func)
		return -EINVAL;
	ret = snd_soc_component_read(component, RT5509_REG_ALCGAIN);
	if (ret < 0)
		return ret;
	ucontrol->value.integer.value[0] = ret & 0x0f;
	return 0;
}

static int rt5509_alcfixed_gain_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	struct soc_enum *se = (struct soc_enum *)kcontrol->private_value;
	int orig_pwron = 0, val = 0, ret = 0;

	if (ucontrol->value.enumerated.item[0] >= se->items)
		return -EINVAL;
	if (!chip->rlr_func)
		return -EINVAL;
	ret = snd_soc_component_read(component, RT5509_REG_CHIPEN);
	if (ret < 0)
		return ret;
	orig_pwron = (ret & RT5509_CHIPPD_ENMASK) ? 0 : 1;
	ret = snd_soc_component_update_bits(component, RT5509_REG_CHIPEN,
				  RT5509_CHIPPD_ENMASK, ~RT5509_CHIPPD_ENMASK);
	if (ret < 0)
		return ret;
	val = ucontrol->value.enumerated.item[0];
	ret = snd_soc_component_write(component, RT5509_REG_ALCMINGAIN, val);
	if (ret < 0)
		return ret;
	val += (val << 4);
	ret = snd_soc_component_write(component, RT5509_REG_ALCGAIN, val);
	if (ret < 0)
		return ret;
	if (!orig_pwron) {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_CHIPPD_ENMASK,
						    RT5509_CHIPPD_ENMASK);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int rt5509_rlrfunc_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = chip->rlr_func;
	return 0;
}

static int rt5509_rlrfunc_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	struct soc_enum *se = (struct soc_enum *)kcontrol->private_value;
	int orig_pwron = 0, orig_proton = 0, ret = 0;

	if (ucontrol->value.enumerated.item[0] >= se->items)
		return -EINVAL;
	if (ucontrol->value.enumerated.item[0] == chip->rlr_func)
		return 0;
	ret = snd_soc_component_read(component, RT5509_REG_CHIPEN);
	if (ret < 0)
		return ret;
	orig_pwron = (ret & RT5509_CHIPPD_ENMASK) ? 0 : 1;
	orig_proton = (ret & RT5509_SPKPROT_ENMASK) ? 1 : 0;
	ret = snd_soc_component_update_bits(component, RT5509_REG_CHIPEN,
				  RT5509_CHIPPD_ENMASK | RT5509_SPKPROT_ENMASK,
				  0);
	if (ret < 0)
		return ret;
	if (ucontrol->value.enumerated.item[0]) {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_FUNCEN,
						    0x1f, 0x12);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_read(component, RT5509_REG_FUNCEN);
		if (ret < 0)
			return ret;
		if (!(ret & 0x80)) {
			ret = snd_soc_component_read(component,
						     RT5509_REG_NDELAY);
			if (ret < 0)
				return ret;
			ret = snd_soc_component_write(component,
						      RT5509_REG_NDELAY,
						      ret + 0x128f5c);
			if (ret < 0)
				return ret;
		}
		ret = snd_soc_component_read(component, RT5509_REG_ALCGAIN);
		if (ret < 0)
			return ret;
		chip->alc_gain = (u8)ret;
		ret = snd_soc_component_read(component, RT5509_REG_ALCMINGAIN);
		if (ret < 0)
			return ret;
		chip->alc_min_gain = (u8)ret;
		ret = snd_soc_component_write(component,
					      RT5509_REG_ALCGAIN, 0x00);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component,
					      RT5509_REG_ALCMINGAIN, 0x00);
		if (ret < 0)
			return ret;
	} else {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_FUNCEN,
						    0x1f, 0x1f);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_read(component, RT5509_REG_FUNCEN);
		if (ret < 0)
			return ret;
		if (!(ret & 0x80)) {
			ret = snd_soc_component_read(component,
						     RT5509_REG_NDELAY);
			if (ret < 0)
				return ret;
			ret = snd_soc_component_write(component,
						      RT5509_REG_NDELAY,
						      ret - 0x128f5c);
			if (ret < 0)
				return ret;
		}
		ret = snd_soc_component_write(component, RT5509_REG_ALCGAIN,
					      chip->alc_gain);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component, RT5509_REG_ALCMINGAIN,
				    chip->alc_min_gain);
		if (ret < 0)
			return ret;
	}
	if (orig_proton) {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_SPKPROT_ENMASK,
						    0xff);
		if (ret < 0)
			return ret;
	}
	if (!orig_pwron) {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_CHIPPD_ENMASK, 0xff);
		if (ret < 0)
			return ret;
	}
	chip->rlr_func = ucontrol->value.enumerated.item[0];
	return 0;
}

static int rt5509_recv_config_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = chip->recv_spec_set;
	return 0;
}

static int rt5509_recv_config_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	struct soc_enum *se = (struct soc_enum *)kcontrol->private_value;
	int orig_pwron = 0, ret = 0;

	if (ucontrol->value.enumerated.item[0] >= se->items)
		return -EINVAL;
	if (ucontrol->value.enumerated.item[0] == chip->recv_spec_set)
		return 0;
	ret = snd_soc_component_read(component, RT5509_REG_CHIPEN);
	if (ret < 0)
		return ret;
	orig_pwron = (ret & RT5509_CHIPPD_ENMASK) ? 0 : 1;
	ret = snd_soc_component_update_bits(component, RT5509_REG_CHIPEN,
				  RT5509_CHIPPD_ENMASK, ~RT5509_CHIPPD_ENMASK);
	if (ret < 0)
		return ret;
	if (ucontrol->value.enumerated.item[0]) {
		/* backup gain ++ */
		ret = snd_soc_component_read(component, RT5509_REG_SPKGAIN);
		if (ret < 0)
			return ret;
		chip->classd_gain_store = ret;
		ret = snd_soc_component_read(component, RT5509_REG_DSPKCONF1);
		if (ret < 0)
			return ret;
		chip->pgain_gain_store = ret;
		ret = snd_soc_component_read(component,
					     RT5509_REG_BST_SIG_GAIN);
		if (ret < 0)
			return ret;
		chip->sig_gain_store = ret;
		ret = snd_soc_component_read(component,
					     RT5509_REG_CLIP_SIGMAX);
		if (ret < 0)
			return ret;
		chip->sig_max_store = ret;
		/* backup gain -- */
		/* default set to model 1 */
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_SPKGAIN,
						    0xe0, 0x00);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component,
					      RT5509_REG_BST_SIG_GAIN, 0x12);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_DSPKCONF1,
						    0x03, 0x02);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CLIP_CTRL,
						    0x80, 0x00);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component,
					      RT5509_REG_CLIP_SIGMAX, 0x7fff);
		if (ret < 0)
			return ret;
		if (orig_pwron) {
			ret = snd_soc_component_update_bits(component,
							    RT5509_REG_PILOTEN,
							    0x01, 0x00);
			if (ret < 0)
				return ret;
		}
		ret = snd_soc_component_update_bits(component, RT5509_REG_TDEN,
						    0x20, 0x00);
		if (ret < 0)
			return ret;
	} else {
		ret = snd_soc_component_update_bits(component, RT5509_REG_TDEN,
						    0x20, 0x20);
		if (ret < 0)
			return ret;
		if (orig_pwron) {
			ret = snd_soc_component_update_bits(component,
							    RT5509_REG_PILOTEN,
							    0x01, 0x00);
			if (ret < 0)
				return ret;
		}
		/* restore gain ++ */
		ret = snd_soc_component_write(component, RT5509_REG_CLIP_SIGMAX,
				   chip->sig_max_store);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component,
					      RT5509_REG_BST_SIG_GAIN,
					      chip->sig_gain_store);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component, RT5509_REG_DSPKCONF1,
				    chip->pgain_gain_store);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component, RT5509_REG_SPKGAIN,
				    chip->classd_gain_store);
		if (ret < 0)
			return ret;
		/* restore gain -- */
	}
	if (!orig_pwron) {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_CHIPPD_ENMASK,
						    RT5509_CHIPPD_ENMASK);
		if (ret < 0)
			return ret;
	}
	chip->recv_spec_set = ucontrol->value.enumerated.item[0];
	return 0;
}

static int rt5509_bypassdsp_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = chip->bypass_dsp;
	return 0;
}

static int rt5509_bypassdsp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	struct soc_enum *se = (struct soc_enum *)kcontrol->private_value;
	int orig_pwron = 0, ret = 0;

	if (ucontrol->value.enumerated.item[0] >= se->items)
		return -EINVAL;
	if (ucontrol->value.enumerated.item[0] == chip->bypass_dsp)
		return 0;
	ret = snd_soc_component_read(component, RT5509_REG_CHIPEN);
	if (ret < 0)
		return ret;
	orig_pwron = (ret & RT5509_CHIPPD_ENMASK) ? 0 : 1;
	ret = snd_soc_component_update_bits(component, RT5509_REG_CHIPEN,
				  RT5509_CHIPPD_ENMASK, ~RT5509_CHIPPD_ENMASK);
	if (ret < 0)
		return ret;
	if (ucontrol->value.enumerated.item[0]) {
		ret = snd_soc_component_read(component, RT5509_REG_FUNCEN);
		if (ret < 0)
			return ret;
		chip->func_en = ret;
		ret = snd_soc_component_read(component, RT5509_REG_CHIPEN);
		if (ret < 0)
			return ret;
		chip->spk_prot_en = ret & RT5509_SPKPROT_ENMASK;
		ret = snd_soc_component_write(component, RT5509_REG_FUNCEN, 0);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_SPKPROT_ENMASK, 0);
		if (ret < 0)
			return ret;
	} else {
		ret = snd_soc_component_write(component, RT5509_REG_FUNCEN,
					      chip->func_en);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component, RT5509_REG_FUNCEN, 0);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component, RT5509_REG_FUNCEN,
					      chip->func_en);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_SPKPROT_ENMASK,
						    chip->spk_prot_en);
		if (ret < 0)
			return ret;
	}
	if (!orig_pwron) {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_CHIPPD_ENMASK,
						    RT5509_CHIPPD_ENMASK);
		if (ret < 0)
			return ret;
	}
	chip->bypass_dsp = ucontrol->value.enumerated.item[0];
	return 0;
}

static int rt5509_put_spk_volsw(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_get_dapm(component);
	int orig_pwron = 0, ret = 0;

	orig_pwron = (dapm->bias_level == SND_SOC_BIAS_OFF) ? 0 : 1;
	if (!orig_pwron) {
		ret = rt5509_set_bias_level(component, SND_SOC_BIAS_STANDBY);
		if (ret < 0)
			return ret;
	}
	ret = snd_soc_put_volsw(kcontrol, ucontrol);
	if (ret < 0)
		return ret;
	if (!orig_pwron) {
		ret = rt5509_set_bias_level(component, SND_SOC_BIAS_OFF);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int rt5509_put_enum_double(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_get_dapm(component);
	int orig_pwron = 0, ret = 0;

	orig_pwron = (dapm->bias_level == SND_SOC_BIAS_OFF) ? 0 : 1;
	if (!orig_pwron) {
		ret = rt5509_set_bias_level(component, SND_SOC_BIAS_STANDBY);
		if (ret < 0)
			return ret;
	}
	ret = snd_soc_put_enum_double(kcontrol, ucontrol);
	if (ret < 0)
		return ret;
	if (!orig_pwron) {
		ret = rt5509_set_bias_level(component, SND_SOC_BIAS_OFF);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int rt5509_recv_model_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	int ret = 0;

	if (!chip->recv_spec_set)
		return -EINVAL;
	ret = snd_soc_component_read(component, RT5509_REG_SPKGAIN);
	if (ret < 0)
		return ret;
	ucontrol->value.integer.value[0] = (ret & 0xe0) >> 5;
	return 0;
}

static int rt5509_recv_model_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct rt5509_chip *chip = snd_soc_component_get_drvdata(component);
	struct soc_enum *se = (struct soc_enum *)kcontrol->private_value;
	int orig_pwron = 0, ret = 0;

	if (ucontrol->value.enumerated.item[0] >= se->items)
		return -EINVAL;
	if (!chip->recv_spec_set)
		return -EINVAL;
	ret = snd_soc_component_read(component, RT5509_REG_CHIPEN);
	if (ret < 0)
		return ret;
	orig_pwron = (ret & RT5509_CHIPPD_ENMASK) ? 0 : 1;
	ret = snd_soc_component_update_bits(component, RT5509_REG_CHIPEN,
					    RT5509_CHIPPD_ENMASK,
					    ~RT5509_CHIPPD_ENMASK);
	if (ret < 0)
		return ret;
	if (ucontrol->value.enumerated.item[0]) {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_SPKGAIN,
						    0xe0, 0x20);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component,
					      RT5509_REG_BST_SIG_GAIN, 0x1a);
		if (ret < 0)
			return ret;
	} else {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_SPKGAIN,
						    0xe0, 0x00);
		if (ret < 0)
			return ret;
		ret = snd_soc_component_write(component,
					      RT5509_REG_BST_SIG_GAIN, 0x12);
		if (ret < 0)
			return ret;
	}
	if (!orig_pwron) {
		ret = snd_soc_component_update_bits(component,
						    RT5509_REG_CHIPEN,
						    RT5509_CHIPPD_ENMASK,
						    RT5509_CHIPPD_ENMASK);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static const DECLARE_TLV_DB_SCALE(dacvol_tlv, -1275, 5, 0);
static const char * const rt5509_enable_text[] = { "Disable", "Enable"};
static const char * const rt5509_slots_text[] = { "Slot 0", "Slot 1"};
static const char * const rt5509_alcgain_text[] = {
	"0dB", "3dB", "6dB", "9dB", "12dB", "15dB", "18dB", "21dB" };
static const DECLARE_TLV_DB_SCALE(predspvol_tlv, 0, 6, 0);
static const char * const rt5509_recvmodel_text[] = { "model1", "model2"};
static const struct soc_enum rt5509_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rt5509_enable_text), rt5509_enable_text),
	SOC_ENUM_SINGLE(RT5509_REG_TDM_CTRL, 2, ARRAY_SIZE(rt5509_slots_text),
		rt5509_slots_text),
	SOC_ENUM_SINGLE(RT5509_REG_TDM_CTRL, 1, ARRAY_SIZE(rt5509_slots_text),
		rt5509_slots_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rt5509_alcgain_text),
		rt5509_alcgain_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rt5509_recvmodel_text),
		rt5509_recvmodel_text),
};
static const struct snd_kcontrol_new rt5509_controls[] = {
	SOC_SINGLE_EXT_TLV("DAC Volume", RT5509_REG_VOLUME, 0, 255, 1,
		snd_soc_get_volsw, rt5509_put_spk_volsw, dacvol_tlv),
	SOC_SINGLE_EXT("Speaker Protection", RT5509_REG_CHIPEN,
		RT5509_SPKPROT_ENSHFT,
		1, 0, snd_soc_get_volsw, rt5509_put_spk_volsw),
	SOC_SINGLE_EXT("Limiter Func", RT5509_REG_FUNCEN,
		RT5509_LMTEN_SHFT, 1, 0,
		snd_soc_get_volsw, rt5509_put_spk_volsw),
	SOC_SINGLE_EXT("ALC Func", RT5509_REG_FUNCEN, RT5509_ALCEN_SHFT, 1, 0,
		snd_soc_get_volsw, rt5509_put_spk_volsw),
	SOC_SINGLE_EXT("CLIP Func", RT5509_REG_CLIP_CTRL,
		RT5509_CLIPEN_SHFT, 1, 0,
		snd_soc_get_volsw, rt5509_put_spk_volsw),
	SOC_SINGLE_EXT("BoostMode", RT5509_REG_BST_MODE,
		RT5509_BSTMODE_SHFT, 3, 0,
		snd_soc_get_volsw, rt5509_put_spk_volsw),
	SOC_SINGLE_EXT("I2S_Channel", RT5509_REG_I2SSEL,
		RT5509_I2SLRSEL_SHFT, 3, 0,
		snd_soc_get_volsw, rt5509_put_spk_volsw),
	SOC_SINGLE_EXT("Ext_DO_Enable", RT5509_REG_I2SDOSEL, 0, 1, 0,
		snd_soc_get_volsw, rt5509_put_spk_volsw),
	SOC_SINGLE_EXT("I2SDOL Mux", RT5509_REG_I2SDOLRSEL, 0, 15, 0,
		snd_soc_get_volsw, rt5509_put_spk_volsw),
	SOC_SINGLE_EXT("I2SDOR Mux", RT5509_REG_I2SDOLRSEL, 4, 15, 0,
		snd_soc_get_volsw, rt5509_put_spk_volsw),
	SOC_ENUM_EXT("BypassDSP", rt5509_enum[0], rt5509_bypassdsp_get,
		rt5509_bypassdsp_put),
	SOC_ENUM_EXT("Recv_Special_Set", rt5509_enum[0], rt5509_recv_config_get,
		rt5509_recv_config_put),
	SOC_ENUM_EXT("RLR Func", rt5509_enum[0], rt5509_rlrfunc_get,
		rt5509_rlrfunc_put),
	SOC_ENUM_EXT("TDM_ADC_SEL", rt5509_enum[1], snd_soc_get_enum_double,
		rt5509_put_enum_double),
	SOC_ENUM_EXT("TDM_DAC_SEL", rt5509_enum[2], snd_soc_get_enum_double,
		rt5509_put_enum_double),
	SOC_ENUM_EXT("ALC Fixed Gain", rt5509_enum[3], rt5509_alcfixed_gain_get,
		rt5509_alcfixed_gain_put),
	SOC_SINGLE_EXT_TLV("PreDSP Volume", RT5509_REG_ALCMINGAIN, 4, 2, 0,
		snd_soc_get_volsw, rt5509_put_spk_volsw, predspvol_tlv),
	SOC_ENUM_EXT("Recv_Model_Set", rt5509_enum[4], rt5509_recv_model_get,
		rt5509_recv_model_put),
};

static const struct snd_soc_component_driver rt5509_component_drv = {
	.probe = rt5509_component_probe,
	.remove = rt5509_component_remove,
	.suspend = rt5509_component_suspend,
	.resume = rt5509_component_resume,

	.controls = rt5509_controls,
	.num_controls = ARRAY_SIZE(rt5509_controls),
	.dapm_widgets = rt5509_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rt5509_dapm_widgets),
	.dapm_routes = rt5509_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(rt5509_dapm_routes),

	.set_bias_level = rt5509_set_bias_level,
	.idle_bias_on = false,
	.read = rt5509_io_read,
	.write = rt5509_io_write,
};

static int rt5509_aif_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	u8 regval = 0;
	int ret = 0;

	dev_dbg(dai->dev, "%s: fmt:%d\n", __func__, fmt);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		regval |= (RT5509_AUDFMT_I2S << RT5509_AUDFMT_SHFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		regval |= (RT5509_AUDFMT_RIGHTJ << RT5509_AUDFMT_SHFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		regval |= (RT5509_AUDFMT_LEFTJ << RT5509_AUDFMT_SHFT);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		regval |= (RT5509_DSP_MODEA << RT5509_DSPMODE_SHFT);
		break;
	case SND_SOC_DAIFMT_DSP_B:
		regval |= (RT5509_DSP_MODEB << RT5509_DSPMODE_SHFT);
		break;
	default:
		break;
	}
	ret = snd_soc_component_update_bits(dai->component, RT5509_REG_AUDFMT,
			RT5509_DSPMODE_MASK | RT5509_AUDFMT_MASK, regval);
	if (ret < 0)
		dev_err(dai->dev, "config dac audfmt error\n");
	return ret;
}

static int rt5509_aif_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hw_params, struct snd_soc_dai *dai)
{
	struct rt5509_chip *chip =
		snd_soc_component_get_drvdata(dai->component);
	unsigned int rate = params_rate(hw_params);
	snd_pcm_format_t format = params_format(hw_params);
	/* 0 for sr and bckfs, 1 for audbits */
	u8 regval[2] = {0};
	u32 pll_divider = 0;
	u8 word_len = 0;
	int ret = 0;

	dev_info(dai->dev, "%s(), format %d, rate %u\n",
		 __func__, format, rate);
	switch (format) {
	case SNDRV_PCM_FORMAT_S16:
	case SNDRV_PCM_FORMAT_U16:
		regval[0] |= (RT5509_BCKMODE_32FS << RT5509_BCKMODE_SHFT);
		regval[1] |= (RT5509_AUDBIT_16 << RT5509_AUDBIT_SHFT);
		pll_divider = 0x00100000;
		word_len = 16 * 4;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
	case SNDRV_PCM_FORMAT_U18_3LE:
	case SNDRV_PCM_FORMAT_S18_3BE:
	case SNDRV_PCM_FORMAT_U18_3BE:
		regval[0] |= (RT5509_BCKMODE_48FS << RT5509_BCKMODE_SHFT);
		regval[1] |= (RT5509_AUDBIT_18 << RT5509_AUDBIT_SHFT);
		pll_divider = 0x000c0000;
		word_len = 18 * 4;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_U20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
	case SNDRV_PCM_FORMAT_U20_3BE:
		regval[0] |= (RT5509_BCKMODE_48FS << RT5509_BCKMODE_SHFT);
		regval[1] |= (RT5509_AUDBIT_20 << RT5509_AUDBIT_SHFT);
		pll_divider = 0x000c0000;
		word_len = 20 * 4;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
	case SNDRV_PCM_FORMAT_U24_3LE:
	case SNDRV_PCM_FORMAT_U24_3BE:
		regval[0] |= (RT5509_BCKMODE_48FS << RT5509_BCKMODE_SHFT);
		regval[1] |= (RT5509_AUDBIT_24 << RT5509_AUDBIT_SHFT);
		pll_divider = 0x000c0000;
		word_len = 24 * 4;
		break;
	case SNDRV_PCM_FORMAT_S32:
	case SNDRV_PCM_FORMAT_U32:
		regval[0] |= (RT5509_BCKMODE_64FS << RT5509_BCKMODE_SHFT);
		regval[1] |= (RT5509_AUDBIT_24 << RT5509_AUDBIT_SHFT);
		pll_divider = 0x00080000;
		word_len = 24 * 4;
		break;
	default:
		ret = -EINVAL;
		goto out_hw_params;
	}

	switch (rate) {
	case 8000:
		regval[0] |= (RT5509_SRMODE_8K << RT5509_SRMODE_SHFT);
		pll_divider *= 6;
		break;
	case 11025:
	case 12000:
		regval[0] |= (RT5509_SRMODE_12K << RT5509_SRMODE_SHFT);
		pll_divider *= 4;
		break;
	case 16000:
		regval[0] |= (RT5509_SRMODE_16K << RT5509_SRMODE_SHFT);
		pll_divider *= 3;
		break;
	case 22050:
	case 24000:
		regval[0] |= (RT5509_SRMODE_24K << RT5509_SRMODE_SHFT);
		pll_divider *= 2;
		break;
	case 32000:
		regval[0] |= (RT5509_SRMODE_32K << RT5509_SRMODE_SHFT);
		pll_divider = (pll_divider * 3) >> 1;
		break;
	case 44100:
	case 48000:
		regval[0] |= (RT5509_SRMODE_48K << RT5509_SRMODE_SHFT);
		break;
	case 88200:
	case 96000:
		regval[0] |= (RT5509_SRMODE_96K << RT5509_SRMODE_SHFT);
		pll_divider >>= 1;
		break;
	case 176400:
	case 192000:
		regval[0] |= (RT5509_SRMODE_192K << RT5509_SRMODE_SHFT);
		pll_divider >>= 2;
		break;
	default:
		ret = -EINVAL;
		goto out_hw_params;
	}
	if (chip->tdm_mode)
		pll_divider >>= 1;
	ret = snd_soc_component_update_bits(dai->component, RT5509_REG_AUDSR,
			RT5509_BCKMODE_MASK | RT5509_SRMODE_MASK, regval[0]);
	if (ret < 0) {
		dev_err(dai->dev, "configure bck and sr fail\n");
		goto out_hw_params;
	}
	ret = snd_soc_component_update_bits(dai->component, RT5509_REG_AUDFMT,
			RT5509_AUDBIT_MASK, regval[1]);
	if (ret < 0) {
		dev_err(dai->dev, "configure audbit fail\n");
		goto out_hw_params;
	}
	ret = snd_soc_component_write(dai->component,
				      RT5509_REG_PLLDIVISOR, pll_divider);
	if (ret < 0) {
		dev_err(dai->dev, "configure pll divider fail\n");
		goto out_hw_params;
	}
	ret = snd_soc_component_write(dai->component,
				      RT5509_REG_DMGFLAG, word_len);
	if (ret < 0)
		dev_err(dai->dev, "configure word len fail\n");
out_hw_params:
	return ret;
}

static int rt5509_aif_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	return 0;
}

static int rt5509_aif_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s\n", __func__);
	return rt5509_set_bias_level(dai->component, SND_SOC_BIAS_STANDBY);
}

static void rt5509_aif_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s\n", __func__);
}

static int rt5509_aif_trigger(struct snd_pcm_substream *substream,
	int cmd, struct snd_soc_dai *dai)
{
	int capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);

	dev_dbg(dai->dev, "%s: cmd=%d\n", __func__, cmd);
	dev_dbg(dai->dev, "%s: %c\n", __func__, capture ? 'c' : 'p');
	return 0;
}

static int rt5509_aif_set_tdm_slot(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct rt5509_chip *chip =
		snd_soc_component_get_drvdata(dai->component);

	dev_dbg(dai->dev, "%s: slots %d\n", __func__, slots);
	if (!slots) {
		dev_dbg(dai->dev, "disable TDM\n");
		chip->tdm_mode = 0;
	} else if (slots == 4) {
		dev_dbg(dai->dev, "enable TDM\n");
		chip->tdm_mode = 1;
	} else
		return -EINVAL;
	return snd_soc_component_update_bits(dai->component,
					     RT5509_REG_TDM_CTRL,
					     RT5509_TDM_ENMASK,
					     chip->tdm_mode ? 0xff : 0);
}

static const struct snd_soc_dai_ops rt5509_dai_ops = {
	.set_fmt = rt5509_aif_set_fmt,
	.hw_params = rt5509_aif_hw_params,
	.startup = rt5509_aif_startup,
	.shutdown = rt5509_aif_shutdown,
	.trigger = rt5509_aif_trigger,
	.prepare = rt5509_aif_prepare,
	.set_tdm_slot = rt5509_aif_set_tdm_slot,
};

#define RT5509_RATES SNDRV_PCM_RATE_8000_192000
#define RT5509_FORMATS (SNDRV_PCM_FMTBIT_S16 | SNDRV_PCM_FMTBIT_S18_3LE |\
	SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_3LE |\
	SNDRV_PCM_FMTBIT_S32)

static struct snd_soc_dai_driver rt5509_i2s_dais[] = {
	{
		.name = "rt5509-aif1",
		.playback = {
			.stream_name = "AIF1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5509_RATES,
			.formats = RT5509_FORMATS,
		},
		.capture = {
			.stream_name = "AIF1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5509_RATES,
			.formats = RT5509_FORMATS,
		},
		.ops = &rt5509_dai_ops,
	},
	{
		.name = "rt5509-aif2",
		.playback = {
			.stream_name = "AIF2 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5509_RATES,
			.formats = RT5509_FORMATS,
		},
		.ops = &rt5509_dai_ops,
	},
};

static inline int rt5509_component_register(struct rt5509_chip *chip)
{
	return snd_soc_register_component(chip->dev, &rt5509_component_drv,
		rt5509_i2s_dais, ARRAY_SIZE(rt5509_i2s_dais));
}

static inline int rt5509_component_unregister(struct rt5509_chip *chip)
{
	snd_soc_unregister_component(chip->dev);
	return 0;
}

static int rt5509_handle_pdata(struct rt5509_chip *chip)
{
	return 0;
}

static int rt5509_i2c_initreg(struct rt5509_chip *chip)
{
	return rt5509_update_bits(chip->i2c, RT5509_REG_CHIPEN,
				  RT5509_TRIWAVE_ENMASK, 0);
}

static int rt5509_get_chip_rev(struct rt5509_chip *chip)
{
	int ret = 0;
	u8 data = 0;

	ret = rt5509_block_read(chip->i2c, RT5509_REG_CHIPREV, 1, &data);
	if (ret < 0)
		return ret;
	if ((data & RT5509_CHIPID_MASK) != RT5509_CHIP_ID)
		return -ENODEV;
	chip->chip_rev = (data & RT5509_CHIPREV_MASK) >> RT5509_CHIPREV_SHFT;
	dev_dbg(chip->dev, "chip revision %d\n", chip->chip_rev);
	return 0;
}

static int rt5509_sw_reset(struct rt5509_chip *chip)
{
	int ret = 0;
	u8 data = 0;

	dev_dbg(chip->dev, "%s\n", __func__);
	ret = rt5509_block_read(chip->i2c, RT5509_REG_SWRESET, 1, &data);
	if (ret < 0)
		return ret;
	data |= RT5509_SWRST_MASK;
	ret = rt5509_block_write(chip->i2c, RT5509_REG_SWRESET, 1, &data);
	mdelay(30);
	return ret;
}

static inline int _rt5509_power_on(struct rt5509_chip *chip, bool en)
{
	int ret = 0;
	u8 data = 0;

	dev_dbg(chip->dev, "%s: en %d\n", __func__, en);
	ret = rt5509_block_read(chip->i2c, RT5509_REG_CHIPEN, 1, &data);
	if (ret < 0)
		return ret;
	data = (en ? (data & ~0x01) : (data | 0x01));
	return rt5509_block_write(chip->i2c, RT5509_REG_CHIPEN, 1, &data);
}

#ifdef CONFIG_OF
static inline int rt5509_parse_dt(struct device *dev,
				  struct rt5509_pdata *pdata)
{
	struct device_node *param_np;
	struct property *prop;
	struct rt5509_proprietary_param *p_param;
	u32 len = 0;
	int i = 0;

	param_np = of_find_node_by_name(dev->of_node, "proprietary_param");
	if (!param_np)
		goto OUT_PARSE_DT;
	p_param = devm_kzalloc(dev, sizeof(*p_param), GFP_KERNEL);
	if (!p_param)
		return -ENOMEM;
	for (i = 0; i < RT5509_CFG_MAX; i++) {
		prop = of_find_property(param_np, prop_str[i], &len);
		if (!prop)
			dev_warn(dev, "no %s setting\n", prop_str[i]);
		else if (!len)
			dev_warn(dev, "%s cfg size is zero\n", prop_str[i]);
		else {
			p_param->cfg[i] = devm_kzalloc(dev, len * sizeof(u8),
						     GFP_KERNEL);
			if (!p_param->cfg[i])
				return -ENOMEM;

			memcpy(p_param->cfg[i], prop->value, len);
			p_param->cfg_size[i] = len;
		}
	}
	pdata->p_param = p_param;
OUT_PARSE_DT:
	return 0;
}
#else
static inline int rt5509_parse_dt(struct device *dev,
				  struct rt5509_pdata *pdata)
{
	return 0;
}
#endif /* #ifdef CONFIG_OF */

static int rt5509_dbg_io_read(void *drvdata, u16 reg, void *val, u16 size)
{
	struct rt5509_chip *chip = (struct rt5509_chip *)drvdata;

	return rt5509_block_read(chip->i2c, reg, size, val);
}

static int rt5509_dbg_io_write(void *drvdata, u16 reg, const void *val, u16 size)
{
	struct rt5509_chip *chip = (struct rt5509_chip *)drvdata;

	return rt5509_block_write(chip->i2c, reg, size, val);
}

static ssize_t dump_attr_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct rt5509_chip *chip = dev_get_drvdata(dev);
	int i = 0, ret = 0, j = 0;
	u8 data[24] = {0};

	buf[0] = '\0';
	for (i = 0; i < ARRAY_SIZE(rt5509_reg_size_table); i++) {
		ret = rt5509_block_read(chip->i2c, rt5509_reg_size_table[i].addr,
					rt5509_reg_size_table[i].size, data);
		if (ret < 0)
			return ret;
		ret = snprintf(buf + strlen(buf), PAGE_SIZE, "reg0x02%x = 0x",
			       rt5509_reg_size_table[i].addr);
		for (j = 0; j < rt5509_reg_size_table[i].size; j++)
			ret = snprintf(buf + strlen(buf), PAGE_SIZE, "02%x,", data[j]);
		snprintf(buf + strlen(buf), PAGE_SIZE, "\n");
	}
	return strlen(buf);
}

static const struct device_attribute dump_attr = {
	.attr = { .name = "dump", .mode = 0444 },
	.show = dump_attr_show,
};

int rt5509_i2c_probe(struct i2c_client *client,
		     const struct i2c_device_id *id)
{
	struct rt5509_pdata *pdata = client->dev.platform_data;
	struct rt5509_chip *chip;
	static int dev_cnt;
	int ret = 0;

	pr_info("+%s\n", __func__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		ret = rt5509_parse_dt(&client->dev, pdata);
		if (ret < 0)
			goto err_parse_dt;

		client->dev.platform_data = pdata;
	} else {
		if (!pdata) {
			dev_err(&client->dev, "Failed, no pdata specified\n");
			return -EINVAL;
		}
	}
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		goto err_parse_dt;

	chip->i2c = client;
	chip->dev = &client->dev;
	chip->pdata = pdata;
	chip->dev_cnt = dev_cnt;
	i2c_set_clientdata(client, chip);

	sema_init(&chip->io_semaphore, 1);
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_active(chip->dev);
	pm_runtime_enable(chip->dev);
#else
	atomic_set(&chip->power_count, 1);
#endif /* #ifdef CONFIG_PM_RUNTIME */
	/* before sw_reset, set CHIP_PD = 0 */
	ret = _rt5509_power_on(chip, true);
	if (ret < 0) {
		dev_err(chip->dev, "power on fail 1\n");
		goto err_sw_reset;
	}
	/* do software reset at default */
	ret = rt5509_sw_reset(chip);
	if (ret < 0) {
		dev_err(chip->dev, "sw_reset fail\n");
		goto err_sw_reset;
	}
	ret = _rt5509_power_on(chip, true);
	if (ret < 0) {
		dev_err(chip->dev, "power on fail 2\n");
		goto err_pm_init;
	}
	/* get chip revisioin first */
	ret = rt5509_get_chip_rev(chip);
	if (ret < 0) {
		dev_err(chip->dev, "get chip rev fail\n");
		goto err_sw_reset;
	}

	chip->extdev_desc.dirname = devm_kasprintf(&client->dev,
						   GFP_KERNEL, "RT5509.%s",
						   dev_name(&client->dev));
	chip->extdev_desc.devname = dev_name(&client->dev);
	chip->extdev_desc.typestr = devm_kasprintf(&client->dev, GFP_KERNEL,
						   "I2C,RT5509");
	chip->extdev_desc.rmap = (struct regmap *)chip;
	chip->extdev_desc.io_read = rt5509_dbg_io_read;
	chip->extdev_desc.io_write = rt5509_dbg_io_write;
	chip->extdev = devm_extdev_io_device_register(&client->dev,
						      &chip->extdev_desc);
	if (IS_ERR(chip->extdev)) {
		dev_err(&client->dev, "Failed to register extdev_io device\n");
		return PTR_ERR(chip->extdev);
	}

	ret = device_create_file(chip->dev, &dump_attr);
	if (ret < 0)
		dev_err(&client->dev, "Failed to add dump attr\n");

	ret = rt5509_i2c_initreg(chip);
	if (ret < 0) {
		dev_err(chip->dev, "init_reg fail\n");
		goto err_initreg;
	}
	ret = rt5509_handle_pdata(chip);
	if (ret < 0) {
		dev_err(chip->dev, "init_pdata fail\n");
		goto err_pdata;
	}
	ret = rt5509_power_on(chip, false);
	if (ret < 0) {
		dev_err(chip->dev, "power off fail\n");
		goto err_put_sync;
	}
	dev_set_name(chip->dev, "RT5509_MT_%d", chip->dev_cnt);
	ret = rt5509_component_register(chip);
	if (ret < 0) {
		dev_err(chip->dev, "component register fail\n");
		goto err_put_sync;
	}
	dev_info(&client->dev, "RT5509_MT_%d driver probed\n",
		dev_cnt);
	dev_cnt++;
	return 0;
err_put_sync:
err_pdata:
err_initreg:
err_pm_init:
	_rt5509_power_on(chip, false);
err_sw_reset:
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(chip->dev);
	pm_runtime_set_suspended(chip->dev);
#else
	atomic_dec(&chip->power_count);
#endif /* #ifdef CONFIG_PM_RUNTIME */
	devm_kfree(&client->dev, chip);
err_parse_dt:
	if (client->dev.of_node)
		devm_kfree(&client->dev, pdata);
	dev_err(&client->dev, "error %d\n", ret);
	i2c_set_clientdata(client, NULL);
	return ret;
}
EXPORT_SYMBOL(rt5509_i2c_probe);

int rt5509_i2c_remove(struct i2c_client *client)
{
	struct rt5509_chip *chip = i2c_get_clientdata(client);

	rt5509_component_unregister(chip);
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(chip->dev);
	pm_runtime_set_suspended(chip->dev);
#else
	atomic_set(&chip->power_count, 0);
#endif /* #ifdef CONFIG_PM_RUNTIME */
	_rt5509_power_on(chip, false);
	devm_kfree(chip->dev, chip->pdata);
	chip->pdata = client->dev.platform_data = NULL;
	dev_dbg(&client->dev, "driver removed\n");
	return 0;
}
EXPORT_SYMBOL(rt5509_i2c_remove);

void rt5509_i2c_shutdown(struct i2c_client *client)
{
	struct rt5509_chip *chip = i2c_get_clientdata(client);
	struct snd_soc_dapm_context *dapm = NULL;

	dev_dbg(&client->dev, "%s\n", __func__);
	if (chip && chip->component) {
		dapm = snd_soc_component_get_dapm(chip->component);
		snd_soc_dapm_disable_pin(dapm, "Speaker");
		snd_soc_dapm_sync(dapm);
	}
}
EXPORT_SYMBOL(rt5509_i2c_shutdown);

MODULE_AUTHOR("CY_Huang <cy_huang@richtek.com>");
MODULE_DESCRIPTION("RT5509 SPKAMP Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.0.0_M");
