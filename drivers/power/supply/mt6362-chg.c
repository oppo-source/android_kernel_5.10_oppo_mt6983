// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <dt-bindings/mfd/mt6362.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

#include "charger_class.h"
#include "mtk_charger.h"

#define MT6362_CHG_DRV_VERSION		"1.0.2_MTK"

static bool dbg_log_en;
module_param(dbg_log_en, bool, 0644);
#define mt_dbg(dev, fmt, ...) \
	do { \
		if (dbg_log_en) \
			dev_info(dev, fmt, ##__VA_ARGS__); \
	} while (0)

#define PHY_MODE_BC11_SET 1
#define PHY_MODE_BC11_CLR 2

/* Register Table */
#define MT6362_REG_CORE_CTRL2		(0x06)
#define MT6362_REG_TM_PASCODE1		(0x07)
#define MT6362_REG_CHG_TOP1		(0x20)
#define MT6362_REG_CHG_TOP2		(0x21)
#define MT6362_REG_CHG_AICR		(0x22)
#define MT6362_REG_CHG_MIVR		(0x23)
#define MT6362_REG_CHG_PREC		(0x24)
#define MT6362_REG_CHG_VCHG		(0x25)
#define MT6362_REG_CHG_ICHG		(0x26)
#define MT6362_REG_CHG_TMR		(0x27)
#define MT6362_REG_CHG_EOC		(0x28)
#define MT6362_REG_CHG_WDT		(0x2A)
#define MT6362_REG_CHG_PUMPX		(0x2B)
#define MT6362_REG_CHG_AICC1		(0x2C)
#define MT6362_REG_CHG_AICC2		(0x2D)
#define MT6362_REG_OTG_V		(0x31)
#define MT6362_REG_OTG_C		(0x32)
#define MT6362_REG_BAT_COMP		(0x33)
#define MT6362_REG_CHG_STAT		(0x34)
#define MT6362_REG_CHG_DUMMY0		(0x35)
#define MT6362_REG_CHG_HD_TOP1		(0x3B)
#define MT6362_REG_CHG_HD_BUCK5		(0x40)
#define MT6362_REG_CHG_HD_DRV3		(0x49)
#define MT6362_REG_BC12_FUNC		(0x50)
#define MT6362_REG_BC12_STAT		(0x51)
#define MT6362_REG_FLED_EN		(0x7E)
#define MT6362_REG_ADC_CONFIG1		(0xA4)
#define MT6362_REG_ADC_ZCV_RPT_H	(0xCA)
#define MT6362_REG_CHG_STAT0		(0xE0)
#define MT6362_REG_CHG_STAT1		(0xE1)
#define MT6362_PD_I2C_TO_RST_CTRL	(0x4BF)

/* Mask & Shift */
/* 0x06 */
#define MT6362_MASK_SHIP_RST_DIS	BIT(0)
/* 0x20 */
#define MT6362_MASK_PP_PG_FLAG		BIT(7)
#define MT6362_MASK_BATFET_DIS		BIT(6)
#define MT6362_MASK_BATFET_DIS_DLY	BIT(5)
#define MT6362_SHFT_BATFET_DIS_DLY	(5)
#define MT6362_MASK_OTG_EN		BIT(2)
#define MT6362_MASK_CHG_BUCK_EN		BIT(1)
#define MT6362_MASK_CHG_EN		BIT(0)
/* 0x21 */
#define MT6362_MASK_VBUS_OV		(0x03)
#define MT6362_SHFT_VBUS_OV		(0)
#define MT6362_MASK_SEL_CLK_FREQ	(0xC0)
#define MT6362_SHFT_SEL_CLK_FREQ	(6)
/* 0x22 */
#define MT6362_MASK_ILIM_EN		BIT(7)
#define MT6362_SHFT_ILIM_EN		(7)
#define MT6362_MASK_AICR		(0x7F)
#define MT6362_SHFT_AICR		(0)
/* 0x23 */
#define MT6362_MASK_MIVR		(0x7F)
#define MT6362_SHFT_MIVR		(0)
/* 0x24 */
#define MT6362_MASK_IPREC		(0x1F)
#define MT6362_SHFT_IPREC		(0)
/* 0x25 */
#define MT6362_MASK_CV			(0x7F)
#define MT6362_SHFT_CV			(0)
/* 0x26 */
#define MT6362_MASK_CC			(0x3F)
#define MT6362_SHFT_CC			(0)
/* 0x27 */
#define MT6362_MASK_CHG_TMR_EN		BIT(7)
#define MT6362_MASK_CHG_TMR_TIME	(0x30)
#define MT6362_SHFT_CHG_TMR_TIME	(4)
/* 0x28 */
#define MT6362_MASK_IEOC		(0xF0)
#define MT6362_SHFT_IEOC		(4)
#define MT6362_MASK_TE			BIT(1)
#define MT6362_MASK_EOC_RST		BIT(0)
/* 0x2A */
#define MT6362_MASK_WDT_EN		BIT(3)
#define MT6362_SHFT_WDT_EN		(3)
#define MT6362_MASK_WDT_CNT_RST		BIT(2)
/* 0x2B */
#define MT6362_MASK_PE_EN		BIT(7)
#define MT6362_MASK_PE_SEL		BIT(6)
#define MT6362_MASK_PE10_INC		BIT(5)
#define MT6362_MASK_PE20_CODE		(0x1F)
#define MT6362_SHFT_PE20_CODE		(0)
/* 0x2C */
#define MT6362_MASK_AICC_EN		BIT(7)
#define MT6362_MASK_AICC_VTH		(0x7F)
/* 0x2D */
#define MT6362_MASK_AICC_ONESHOT	BIT(7)
#define MT6362_SHFT_AICC_ONESHOT	(7)
#define MT6362_MASK_AICC_RPT		(0x7F)
#define MT6362_SHFT_AICC_RPT		(0)
/* 0x31 */
#define MT6362_MASK_OTG_CV		(0x3F)
/* 0x32 */
#define MT6362_MASK_OTG_CC		(0x07)
#define MT6362_SHFT_OTG_CC		(0)
/* 0x33 */
#define MT6362_MASK_BAT_IRCOMP_R	(0x70)
#define MT6362_SHFT_BAT_IRCOMP_R	(4)
#define MT6362_MASK_BAT_IRCOMP_V	(0x07)
#define MT6362_SHFT_BAT_IRCOMP_V	(0)
/* 0x34 */
#define MT6362_MASK_IC_STAT		(0x0F)
#define MT6362_SHFT_IC_STAT		(0)
/* 0x35 */
#define MT6362_MASK_COMP_CLAMP		(0x03)
#define MT6362_SHFT_COMP_CLAMP		(0)
/* 0x3B */
#define MT6362_MASK_DISCHG		BIT(6)
/* 0x40 */
#define MT6362_MASK_BUCK_RAMPOFT	(0xC0)
#define MT6362_SHFT_BUCK_RAMPOFT	(6)
/* 0x49 */
#define MT6362_MASK_DISDRV_UG2LG	BIT(1)
/* 0x50 */
#define MT6362_MASK_BC12_EN		BIT(7)
#define MT6362_MASK_SPECTA_EN		(0x40)
#define MT6362_SHFT_SPECTA_EN		(6)
#define MT6362_MASK_DCDT_SEL		(0x30)
#define MT6362_SHFT_DCDT_SEL		(4)
/* 0x51 */
#define MT6362_MASK_PORT_STAT		(0x0F)
#define MT6362_SHFT_PORT_STAT		(0)
/* 0x7E */
#define MT6362_MASK_FL_STROBE		BIT(2)
/* 0xA4 */
#define MT6362_MASK_ZCV_EN		BIT(6)
/* 0xE0 */
#define MT6362_MASK_PWR_RDY		BIT(0)
/* 0xE1 */
#define MT6362_MASK_MIVR_STAT		BIT(7)
/* 0x4BF */
#define MT6362_MASK_BLEEDDIS_EN		BIT(6)

/* Engineer Spec */
/* uA */
#define MT6362_AICR_MIN		50000
#define MT6362_AICR_MAX		3225000
#define MT6362_AICR_STEP	25000
/* uA */
#define MT6362_AICC_MIN		50000
#define MT6362_AICC_MAX		3225000
#define MT6362_AICC_STEP	25000
/* uV */
#define MT6362_MIVR_MIN		3900000
#define MT6362_MIVR_MAX		13400000
#define MT6362_MIVR_STEP	100000
/* uV */
#define MT6362_VPREC_MIN	2600000
#define MT6362_VPREC_MAX	3300000
#define MT6362_VPREC_STEP	100000
/* uA */
#define MT6362_IPREC_MIN	50000
#define MT6362_IPREC_MAX	1600000
#define MT6362_IPREC_STEP	50000
/* uV */
#define MT6362_CV_MIN		3900000
#define MT6362_CV_MAX		4710000
#define MT6362_CV_STEP		10000
/* uA */
#define MT6362_CC_MIN		0
#define MT6362_CC_MAX		3150000
#define MT6362_CC_STEP		50000
/* hr */
#define MT6362_TMR_MIN		5
#define MT6362_TMR_MAX		20
#define MT6362_TMR_STEP		5
/* uA */
#define MT6362_IEOC_MIN		50000
#define MT6362_IEOC_MAX		800000
#define MT6362_IEOC_STEP	50000
/* uV */
#define MT6362_PE20_MIN		5500000
#define MT6362_PE20_MAX		20000000
#define MT6362_PE20_STEP	500000
/* uohm */
#define MT6362_IR_R_MIN		0
#define MT6362_IR_R_MAX		25000
#define MT6362_IR_R_STEP	175000
/* uV */
#define MT6362_IR_V_MIN		0
#define MT6362_IR_V_MAX		32000
#define MT6362_IR_V_STEP	224000

struct mt6362_chg_platform_data {
	u32 ichg;
	u32 aicr;
	u32 mivr;
	u32 cv;
	u32 ieoc;
	u32 safety_timer;
	u32 ircmp_resistor;
	u32 ircmp_vclamp;
	u32 dcdt_sel;
	u32 specta_det;
	u32 vbusov_sel;
	u32 en_te;
	u32 en_wdt;
	u32 aicc_oneshot;
	u32 post_aicc;
	u32 post_aicc_thr;
	u32 shipping_dly_en;
	u32 batoc_notify;
	u32 bc12_sel;
	const char *chg_name;
};

static const struct mt6362_chg_platform_data def_platform_data = {
	.ichg = 2000000,		/* uA */
	.aicr = 500000,			/* uA */
	.mivr = 4400000,		/* uV */
	.cv = 4350000,			/* uA */
	.ieoc = 150000,			/* uA */
	.safety_timer = 10,		/* hour */
	.ircmp_resistor = 25000,	/* uohm */
	.ircmp_vclamp = 32000,		/* uV */
	.dcdt_sel = 2,
	.specta_det = 0,
	.en_te = true,
	.en_wdt = true,
	.aicc_oneshot = true,
	.post_aicc = true,
	.post_aicc_thr = 200000,
	.shipping_dly_en = true,
	.batoc_notify = false,
	.bc12_sel = 0,
	.chg_name = "primary_chg",
};

struct mt6362_chg_data {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply_desc psy_desc;
	struct power_supply *psy;
	struct charger_device *chg_dev;
	struct regulator_dev *otg_rdev;
	struct iio_channel *iio_ch;
	struct mutex hidden_mode_lock;
	struct mutex ichg_lock;
	struct mutex tchg_lock;
	struct mutex pe_lock;
	struct mutex bd_lock;
	struct mutex bc12_lock;
	struct mutex otg_lock;
	struct completion aicc_done;
	struct completion pe_done;
	struct completion bc12_start;
	bool bd_flag;
	bool pwr_rdy;
	int hidden_mode_cnt;
	int otg_mode_cnt;
	int tchg;
	u32 zcv;
	u32 ichg;
	u32 ichg_dis_chg;
	u32 bd_mivr;
	struct task_struct *bc12_task;
	bool bc12_update;
	bool attach;
	struct power_supply *chg_psy;
	int psy_usb_type;
	/* mivr */
	atomic_t mivr_cnt;
	wait_queue_head_t waitq;
	struct task_struct *mivr_task;

	/* boot_mode */
	u32 bootmode;
	u32 boottype;
};

static const struct regulator_ops mt6362_chg_otg_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static const struct regulator_desc mt6362_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &mt6362_chg_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.min_uV = 4350000,
	.uV_step = 25000, /* 25mV per step */
	.n_voltages = 59, /* 4350mV to 5800mV */
	.vsel_reg = MT6362_REG_OTG_V,
	.vsel_mask = MT6362_MASK_OTG_CV,
	.enable_reg = MT6362_REG_CHG_TOP1,
	.enable_mask = MT6362_MASK_OTG_EN,
};

static const u32 otg_cc_table[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000, 3000000
};

enum mt6362_clk_freq {
	MT6362_CLK_FREQ_1500K,
	MT6362_CLK_FREQ_1000K,
	MT6362_CLK_FREQ_750K,
	MT6362_CLK_FREQ_MAX,
};

enum mt6362_usbsw_state {
	MT6362_USBSW_CHG = 0,
	MT6362_USBSW_USB,
};

enum pe_sel {
	MT6362_PE_SEL_10,
	MT6362_PE_SEL_20,
	MT6362_PE_SEL_MAX,
};

enum mt6362_chg_type {
	MT6362_CHG_TYPE_NO_INFO = 0x0,
	MT6362_CHG_TYPE_APPLE_10W = 0x8,
	MT6362_CHG_TYPE_SAMSUNG_10W = 0x9,
	MT6362_CHG_TYPE_APPLE_5W = 0xA,
	MT6362_CHG_TYPE_APPLE_12W = 0xB,
	MT6362_CHG_TYPE_UNKNOWN_TA = 0xC,
	MT6362_CHG_TYPE_SDP = 0xD,
	MT6362_CHG_TYPE_CDP = 0xE,
	MT6362_CHG_TYPE_DCP = 0xF,
};

enum mt6362_ic_stat {
	MT6362_STAT_HZ = 0x0,
	MT6362_STAT_READY = 0x1,
	MT6362_STAT_TRI_CHG = 0x2,
	MT6362_STAT_PRE_CHG = 0x3,
	MT6362_STAT_FAST_CHG = 0x4,
	MT6362_STAT_EOC_CHG = 0x5,
	MT6362_STAT_BACKGND_CHG = 0x6,
	MT6362_STAT_CHG_DONE = 0x7,
	MT6362_STAT_CHG_FAULT = 0x8,
	MT6362_STAT_OTG = 0xf,
};

static const char * const mt6362_ic_stat_list[] = {
	"hz", "ready", "trickle_chg", "pre_chg", "fast_chg",
	"ieoc_chg", "backgnd_chg", "chg_done", "chg_fault", "otg",
};

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

enum mt6362_chg_adc_channel {
	MT6362_CHG_ADCCH_CHGVINDIV5,
	MT6362_CHG_ADCCH_VSYS,
	MT6362_CHG_ADCCH_VBAT,
	MT6362_CHG_ADCCH_IBUS,
	MT6362_CHG_ADCCH_IBAT,
	MT6362_CHG_ADCCH_TEMP_JC,
	MT6362_CHG_ADCCH_ZCV,
};

enum mt6362_bc12_trigger {
	MT6362_BC12_TRIG_MICROB,
	MT6362_BC12_TRIG_TYPEC,
	MT6362_BC12_TRIG_NONE,
};

static const char * const mt6362_adcch_list[] = {
	"VBUSDIV5", "VSYS", "VBAT", "IBUS", "IBAT", "TEMP_JC", "ZCV"
};

/*
 * =================
 * internal funciton
 * =================
 */

static inline enum mt6362_bc12_trigger mt6362_get_bc12_trigger(
	struct mt6362_chg_data *data)
{
	struct mt6362_chg_platform_data *pdata = dev_get_platdata(data->dev);

	/*
	 * 0 means bc12 owner is THIS_MODULE, if it is not 0, always ignore
	 */
	if (pdata->bc12_sel != 0)
		return MT6362_BC12_TRIG_NONE;
	return IS_ENABLED(CONFIG_TCPC_CLASS) ? MT6362_BC12_TRIG_TYPEC :
					       MT6362_BC12_TRIG_MICROB;
}

static inline u32 mt6362_map_reg_sel(u32 data, u32 min, u32 max, u32 step)
{
	u32 target = 0, max_sel;

	if (data >= min) {
		target = (data - min) / step;
		max_sel = (max - min) / step;
		if (target > max_sel)
			target = max_sel;
	}
	return target;
}

static inline u32 mt6362_map_real_val(u32 sel, u32 min, u32 max, u32 step)
{
	u32 target = 0;

	target = min + (sel * step);
	if (target > max)
		target = max;
	return target;
}

static int mt6362_enable_hidden_mode(struct mt6362_chg_data *data, bool en)
{
	int ret = 0;

	mt_dbg(data->dev, "%s: en = %d\n", __func__, en);
	mutex_lock(&data->hidden_mode_lock);
	if (en) {
		if (data->hidden_mode_cnt == 0) {
			ret = regmap_write(data->regmap,
					   MT6362_REG_TM_PASCODE1, 0x69);
			if (ret < 0)
				goto err;
		}
		data->hidden_mode_cnt++;
	} else {
		if (data->hidden_mode_cnt == 1) {
			ret = regmap_write(data->regmap,
					   MT6362_REG_TM_PASCODE1, 0x00);
			if (ret < 0)
				goto err;
			data->hidden_mode_cnt--;
		}
	}
	goto out;
err:
	dev_err(data->dev, "%s: fail\n", __func__);
out:
	mutex_unlock(&data->hidden_mode_lock);
	return ret;
}

static int mt6362_enable_wdt(struct mt6362_chg_data *data, bool en)
{
	struct mt6362_chg_platform_data *pdata = dev_get_platdata(data->dev);

	mt_dbg(data->dev, "%s: en = %d\n", __func__, en);
	if (!pdata->en_wdt)
		return 0;
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_WDT,
				  MT6362_MASK_WDT_EN,
				  en ? 0xff : 0);
}

static inline int mt6362_read_zcv(struct mt6362_chg_data *data)
{
	int ret;

	dev_dbg(data->dev, "%s\n", __func__);
	ret = iio_read_channel_processed(&data->iio_ch[MT6362_CHG_ADCCH_ZCV],
					 &data->zcv);
	if (ret < 0) {
		dev_info(data->dev, "%s: fail(%d)\n", __func__, ret);
		return ret;
	}
	dev_info(data->dev, "%s: zcv = %d mV\n", __func__, data->zcv/1000);
	return ret;
}

static inline int mt6362_is_charger_enabled(struct mt6362_chg_data *data,
					    bool *en)
{
	int ret = 0;
	u32 regval;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_TOP1, &regval);
	if (ret < 0)
		return ret;
	*en = (regval & MT6362_MASK_CHG_EN) ? true : false;
	return 0;
}

static int __mt6362_set_ichg(struct mt6362_chg_data *data, u32 uA)
{
	u8 sel;
	int ret;

	/* mapping datasheet define */
	if (uA < 300000)
		return -EINVAL;
	sel = mt6362_map_reg_sel(uA, MT6362_CC_MIN, MT6362_CC_MAX,
				 MT6362_CC_STEP);
	ret = regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_ICHG,
				 MT6362_MASK_CC,
				 sel << MT6362_SHFT_CC);
	if (ret < 0)
		dev_err(data->dev, "%s: fail\n", __func__);
	else
		data->ichg = uA;
	return ret;
}

/* Switch D+D- to AP/MT6362 */
static int mt6362_set_usbsw_state(struct mt6362_chg_data *data, int state)
{
	struct phy *phy;
	int ret;
	int mode = (state == MT6362_USBSW_CHG) ? PHY_MODE_BC11_SET :
						 PHY_MODE_BC11_CLR;

	dev_info(data->dev, "%s: state = %d\n", __func__, state);
	phy = phy_get(data->dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_info(data->dev, "phy_get fail\n");
		return -EINVAL;
	}
	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, mode);
	if (ret)
		dev_info(data->dev, "phy_set_mode_ext fail\n");
	phy_put(data->dev, phy);
	return 0;
}

static int __mt6362_enable_bc12(struct mt6362_chg_data *data, bool en)
{
	enum mt6362_usbsw_state usbsw =
				       en ? MT6362_USBSW_CHG : MT6362_USBSW_USB;

	mt6362_set_usbsw_state(data, usbsw);
	return regmap_update_bits(data->regmap, MT6362_REG_BC12_FUNC,
				  MT6362_MASK_BC12_EN, en ? 0xff : 0);
}

static bool is_usb_rdy(struct device *dev)
{
	struct device_node *node;
	bool ready = true;

	node = of_parse_phandle(dev->of_node, "usb", 0);
	if (node) {
		ready = !of_property_read_bool(node, "cdp-block");
		dev_info(dev, "usb ready=%d\n", ready);
	} else
		dev_info(dev, "usb node missing or invalid\n");

	return ready;
}

static int mt6362_enable_bc12(struct mt6362_chg_data *data, bool en)
{
	struct mt6362_chg_platform_data *pdata = dev_get_platdata(data->dev);
	int i;
	const int max_wait_cnt = 250;

	if (en) {
		if (data->bootmode == 5) {
			/* Skip charger type detection to speed up meta boot.*/
			dev_notice(data->dev,
				   "%s: force Standard USB Host in meta\n",
				   __func__);
			data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
			power_supply_changed(data->psy);
			return 0;
		}
		/* add delay for disable DCD timeout */
		if (!pdata->dcdt_sel)
			msleep(180);
		/* Workaround for CDP port */
		for (i = 0; i < max_wait_cnt; i++) {
			if (is_usb_rdy(data->dev))
				break;
			dev_info(data->dev, "%s: CDP block\n", __func__);
			if (!data->attach) {
				dev_info(data->dev, "%s: plug out\n", __func__);
				return 0;
			}
			msleep(100);
		}
		if (i == max_wait_cnt)
			dev_err(data->dev, "%s: CDP timeout\n", __func__);
		else
			dev_info(data->dev, "%s: CDP free\n", __func__);
	} else {
		data->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		data->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		power_supply_changed(data->psy);
	}
	return __mt6362_enable_bc12(data, en);
}

static int mt6362_bc12_thread(void *data)
{
	struct mt6362_chg_data *cdata = data;
	bool bc12_en;
	int ret;

	while (!kthread_should_stop()) {
		wait_for_completion(&cdata->bc12_start);
		mutex_lock(&cdata->bc12_lock);
		reinit_completion(&cdata->bc12_start);
		bc12_en = cdata->attach;
		mutex_unlock(&cdata->bc12_lock);
		ret = mt6362_enable_bc12(cdata, bc12_en);
		if (ret < 0)
			dev_err(cdata->dev, "%s: handle bc12 fail, en = %d\n",
				__func__, bc12_en);
	}
	return 0;
}

static void mt6362_run_bc12_thread(struct mt6362_chg_data *data,
				   enum mt6362_bc12_trigger trigger, bool en)
{
	/* If trigger does not match system's setting, ignore it */
	if (mt6362_get_bc12_trigger(data) != trigger)
		return;
	if (en == data->attach) {
		dev_info(data->dev,
			 "%s: attach is the same, ignore\n", __func__);
		return;
	}
	mutex_lock(&data->bc12_lock);
	data->attach = en;
	data->bc12_update = true;
	complete(&data->bc12_start);
	mutex_unlock(&data->bc12_lock);
}

static inline int mt6362_get_pwr_rdy_stat(struct mt6362_chg_data *data,
					  bool *pwr_rdy)
{
	int ret;
	u32 regval = 0;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_STAT0, &regval);
	if (ret < 0)
		return ret;
	*pwr_rdy = (regval & MT6362_MASK_PWR_RDY) ? true : false;
	return 0;
}

static int mt6362_get_charging_status(struct mt6362_chg_data *data,
				      enum mt6362_ic_stat *ic_stat)
{
	int ret;
	u32 regval = 0;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_STAT, &regval);
	if (ret < 0)
		return ret;
	*ic_stat = (regval & MT6362_MASK_IC_STAT) >> MT6362_SHFT_IC_STAT;
	return 0;
}

static int mt6362_toggle_cfo(struct mt6362_chg_data *data)
{
	int ret;
	u32 regval;

	/* check if strobe mode */
	ret = regmap_read(data->regmap, MT6362_REG_FLED_EN, &regval);
	if (ret < 0)
		return ret;
	if (regval & MT6362_MASK_FL_STROBE) {
		dev_err(data->dev, "%s: fled in strobe mode\n", __func__);
		return ret;
	}
	/* chg_buck off */
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_TOP1,
				 MT6362_MASK_CHG_BUCK_EN, 0);
	if (ret < 0) {
		dev_err(data->dev, "%s: set chg_buck off fail\n", __func__);
		return ret;
	}
	/* chg_buck on */
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_TOP1,
				 MT6362_MASK_CHG_BUCK_EN, 0xff);
	if (ret < 0)
		dev_err(data->dev, "%s: set chg_buck on fail\n", __func__);
	return ret;
}

static void mt6362_chg_irq_enable(const char *name, int en);
static int mt6362_get_ibus(struct charger_device *chg_dev, u32 *ibus);
static int mt6362_get_mivr_state(struct charger_device *chg_dev, bool *in_loop);
static int mt6362_chg_mivr_task_threadfn(void *data)
{
	struct mt6362_chg_data *cdata = data;
	u32 ibus;
	int ret;
	bool mivr_stat;

	dev_info(cdata->dev, "%s ++\n", __func__);
	while (!kthread_should_stop()) {
		wait_event(cdata->waitq, atomic_read(&cdata->mivr_cnt) > 0);
		mt_dbg(cdata->dev, "%s: enter mivr thread\n", __func__);
		pm_stay_awake(cdata->dev);
		/* check real mivr stat or not */
		ret = mt6362_get_mivr_state(cdata->chg_dev, &mivr_stat);
		if (ret < 0)
			goto loop_cont;
		if (!mivr_stat) {
			mt_dbg(cdata->dev, "%s: mivr stat not act\n", __func__);
			goto loop_cont;
		}
		/* read ibus adc */
		ret = mt6362_get_ibus(cdata->chg_dev, &ibus);
		if (ret < 0) {
			dev_err(cdata->dev,
				"%s: get ibus adc fail\n", __func__);
			goto loop_cont;
		}
		/* if ibus adc value < 100mA), toggle cfo */
		if (ibus < 100000) {
			dev_dbg(cdata->dev, "%s: enter toggle cfo\n", __func__);
			ret = mt6362_toggle_cfo(cdata);
			if (ret < 0)
				dev_err(cdata->dev,
					"%s: toggle cfo fail\n", __func__);
		}
loop_cont:
		pm_relax(cdata->dev);
		atomic_set(&cdata->mivr_cnt, 0);
		mt6362_chg_irq_enable("chg_mivr_evt", 1);
		msleep(200);
	}
	dev_info(cdata->dev, "%s --\n", __func__);
	return 0;
}

static int __mt6362_enable_otg_parameter(struct mt6362_chg_data *data, bool en)
{
	int ret;

	/* Set switch frequency to 1.5/1 MHz for otg transient */
	ret = regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_TOP2,
				 MT6362_MASK_SEL_CLK_FREQ,
				 en ? MT6362_CLK_FREQ_1500K :
				      MT6362_CLK_FREQ_1000K <<
						MT6362_SHFT_SEL_CLK_FREQ);
	if (ret < 0) {
		dev_err(data->dev,
			"%s: fail to set switch freq 1.5/1 MHz\n", __func__);
		return ret;
	}
	ret = mt6362_enable_hidden_mode(data, true);
	if (ret < 0)
		goto out;
	/* Set comp diode to 2/1 for otg transient */
	ret = regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_DUMMY0,
				 MT6362_MASK_COMP_CLAMP,
				 en ? 0x3 : 0 << MT6362_SHFT_COMP_CLAMP);
	if (ret < 0) {
		dev_err(data->dev,
			"%s: fail to set comp diode 2/1\n", __func__);
		goto out;
	}
	/* Set buck_ramp offset 390/330 mV */
	/* for decrease psm->pwm trnsition current when otg */
	/* and not to enter psk mode when charging in hv and low loading */
	ret = regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_HD_BUCK5,
				 MT6362_MASK_BUCK_RAMPOFT,
				 en ? 0x3 : 0x1 << MT6362_SHFT_BUCK_RAMPOFT);
	if (ret < 0) {
		dev_err(data->dev,
			"%s: fail to set buck ramp offset 390/330 mV\n",
			__func__);
		goto out;
	}
	/* Workaround for otg pwm stop switch when enable otg */
	ret = regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_HD_DRV3,
				 MT6362_MASK_DISDRV_UG2LG,
				 en ? 0xff : 0);
	if (ret < 0)
		dev_err(data->dev,
			"%s: fail to disdrive UG to LG\n", __func__);
out:
	mt6362_enable_hidden_mode(data, false);
	return ret;
}

static int mt6362_enable_otg_parameter(struct mt6362_chg_data *data, bool en)
{
	int ret = 0;

	mt_dbg(data->dev, "%s: en = %d\n", __func__, en);
	mutex_lock(&data->otg_lock);
	if (en) {
		if (data->otg_mode_cnt == 0) {
			ret = __mt6362_enable_otg_parameter(data, en);
			if (ret < 0)
				goto err;
		}
		data->otg_mode_cnt++;
	} else {
		if (data->otg_mode_cnt == 1) {
			ret = __mt6362_enable_otg_parameter(data, en);
			if (ret < 0)
				goto err;
		}
		data->otg_mode_cnt--;
	}
	goto out;
err:
	dev_err(data->dev, "%s: fail\n", __func__);
out:
	mutex_unlock(&data->otg_lock);
	return ret;
}

/*
 * ======================
 * power supply props ops
 * ======================
 */

static int mt6362_charger_get_online(struct mt6362_chg_data *data,
				     union power_supply_propval *val)
{
	int ret;
	bool pwr_rdy;

	if (IS_ENABLED(CONFIG_TCPC_CLASS)) {
		mutex_lock(&data->bc12_lock);
		pwr_rdy = data->attach;
		mutex_unlock(&data->bc12_lock);
	} else {
		ret = mt6362_get_pwr_rdy_stat(data, &pwr_rdy);
		if (ret < 0)
			return ret;
	}
	val->intval = pwr_rdy;
	return 0;
}

static int mt6362_charger_get_status(struct mt6362_chg_data *data,
				     union power_supply_propval *val)
{
	enum mt6362_ic_stat ic_stat;
	int ret, status;
	bool pwr_rdy;

	ret = mt6362_get_pwr_rdy_stat(data, &pwr_rdy);
	if (ret < 0)
		return ret;
	if (!pwr_rdy) {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		goto out;
	}

	ret = mt6362_get_charging_status(data, &ic_stat);
	if (ret < 0)
		return ret;

	switch (ic_stat) {
	case MT6362_STAT_HZ:
	case MT6362_STAT_READY:
	case MT6362_STAT_OTG:
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case MT6362_STAT_TRI_CHG ... MT6362_STAT_BACKGND_CHG:
		status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case MT6362_STAT_CHG_FAULT:
		status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case MT6362_STAT_CHG_DONE:
		status = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		ret = -EIO;
	}
out:
	if (!ret)
		val->intval = status;
	return ret;
}

static int mt6362_charger_get_ocv(struct mt6362_chg_data *data,
				  union power_supply_propval *val)
{
	val->intval = data->zcv;
	return 0;
}

static int mt6362_charger_get_charge_type(struct mt6362_chg_data *data,
					  union power_supply_propval *val)
{
	enum mt6362_ic_stat ic_stat;
	int ret, type = 0;

	ret = mt6362_get_charging_status(data, &ic_stat);
	if (ret < 0)
		return ret;
	switch (ic_stat) {
	case MT6362_STAT_READY: /* Not Charging */
		type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case MT6362_STAT_TRI_CHG: /* Trickle Charge */
		type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case MT6362_STAT_PRE_CHG ... MT6362_STAT_BACKGND_CHG:
		type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case MT6362_STAT_CHG_DONE: /* Charge Done */
		type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case MT6362_STAT_CHG_FAULT: /* Charge Fault */
		type = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	default:
		break;
	}
	val->intval = type;
	return 0;
}

static int mt6362_charger_get_ichg(struct mt6362_chg_data *data,
				   union power_supply_propval *val)
{
	int ret;
	u32 regval = 0;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_ICHG, &regval);
	if (ret < 0)
		return ret;
	regval = (regval & MT6362_MASK_CC) >> MT6362_SHFT_CC;
	val->intval = mt6362_map_real_val(regval, MT6362_CC_MIN, MT6362_CC_MAX,
					  MT6362_CC_STEP);
	return 0;
}

static int mt6362_charger_get_max_ichg(struct mt6362_chg_data *data,
				   union power_supply_propval *val)
{
	val->intval = MT6362_CC_MAX;
	return 0;
}

static int mt6362_charger_get_cv(struct mt6362_chg_data *data,
				 union power_supply_propval *val)
{
	int ret;
	u32 regval = 0;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_VCHG, &regval);
	if (ret < 0)
		return ret;
	regval = (regval & MT6362_MASK_CV) >> MT6362_SHFT_CV;
	val->intval = mt6362_map_real_val(regval, MT6362_CV_MIN, MT6362_CV_MAX,
					  MT6362_CV_STEP);
	return 0;
}

static int mt6362_charger_get_max_cv(struct mt6362_chg_data *data,
				     union power_supply_propval *val)
{
	val->intval = MT6362_CV_MAX;
	return 0;
}

static int mt6362_charger_get_aicr(struct mt6362_chg_data *data,
				   union power_supply_propval *val)
{
	int ret;
	u32 regval = 0;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_AICR, &regval);
	if (ret < 0)
		return ret;
	regval = (regval & MT6362_MASK_AICR) >> MT6362_SHFT_AICR;
	val->intval = mt6362_map_real_val(regval,
					  MT6362_AICR_MIN,
					  MT6362_AICR_MAX,
					  MT6362_AICR_STEP);
	return 0;
}

static int mt6362_charger_get_iprechg(struct mt6362_chg_data *data,
				      union power_supply_propval *val)
{
	int ret;
	u32 regval;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_PREC, &regval);
	if (ret < 0)
		return ret;
	regval = (regval & MT6362_MASK_IPREC) >> MT6362_SHFT_IPREC;
	val->intval = mt6362_map_real_val(regval,
					  MT6362_IPREC_MIN,
					  MT6362_IPREC_MAX,
					  MT6362_IPREC_STEP);
	return 0;
}

static int mt6362_charger_get_ieoc(struct mt6362_chg_data *data,
				   union power_supply_propval *val)
{
	int ret;
	u32 regval = 0;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_EOC, &regval);
	if (ret < 0)
		return ret;
	regval = (regval & MT6362_MASK_IEOC) >> MT6362_SHFT_IEOC;
	val->intval = mt6362_map_real_val(regval,
					  MT6362_IEOC_MIN,
					  MT6362_IEOC_MAX,
					  MT6362_IEOC_STEP);
	return 0;
}

static int mt6362_charger_set_online(struct mt6362_chg_data *data,
				     const union power_supply_propval *val)
{
	dev_info(data->dev, "%s: en = %d\n", __func__, val->intval);
	mt6362_run_bc12_thread(data, MT6362_BC12_TRIG_TYPEC, val->intval);
	return 0;
}

static int mt6362_charger_set_ichg(struct mt6362_chg_data *data,
				   const union power_supply_propval *val)
{
	int ret;

	mutex_lock(&data->ichg_lock);
	ret = __mt6362_set_ichg(data, val->intval);
	mutex_unlock(&data->ichg_lock);
	return ret;
}

static int mt6362_charger_set_cv(struct mt6362_chg_data *data,
				 const union power_supply_propval *val)
{
	u8 sel;

	sel = mt6362_map_reg_sel(val->intval, MT6362_CV_MIN, MT6362_CV_MAX,
				 MT6362_CV_STEP);
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_VCHG,
				  MT6362_MASK_CV,
				  sel << MT6362_SHFT_CV);
}

static int mt6362_charger_set_aicr(struct mt6362_chg_data *data,
				   const union power_supply_propval *val)
{
	u8 sel;

	sel = mt6362_map_reg_sel(val->intval, MT6362_AICR_MIN, MT6362_AICR_MAX,
				 MT6362_AICR_STEP);
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_AICR,
				  MT6362_MASK_AICR,
				  sel << MT6362_SHFT_AICR);
}

static int mt6362_charger_set_iprechg(struct mt6362_chg_data *data,
				      const union power_supply_propval *val)
{
	u8 sel;

	sel = mt6362_map_reg_sel(val->intval,
				 MT6362_IPREC_MIN,
				 MT6362_IPREC_MAX,
				 MT6362_IPREC_STEP);
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_PREC,
				  MT6362_MASK_IPREC,
				  sel << MT6362_SHFT_IPREC);
}

static int mt6362_charger_set_ieoc(struct mt6362_chg_data *data,
				   const union power_supply_propval *val)
{
	u8 sel;

	sel = mt6362_map_reg_sel(val->intval, MT6362_IEOC_MIN, MT6362_IEOC_MAX,
				 MT6362_IEOC_STEP);
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_EOC,
				  MT6362_MASK_IEOC,
				  sel << MT6362_SHFT_IEOC);
}

static int mt6362_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mt6362_chg_data *data = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = mt6362_charger_get_online(data, val);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = data->psy_desc.type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = data->psy_usb_type;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = mt6362_charger_get_status(data, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = mt6362_charger_get_ocv(data, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = mt6362_charger_get_charge_type(data, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6362_charger_get_ichg(data, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = mt6362_charger_get_max_ichg(data, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mt6362_charger_get_cv(data, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = mt6362_charger_get_max_cv(data, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6362_charger_get_aicr(data, val);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = mt6362_charger_get_iprechg(data, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = mt6362_charger_get_ieoc(data, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (data->psy_desc.type == POWER_SUPPLY_TYPE_USB)
			val->intval = 500000;
		break;
	default:
		ret = -ENODATA;
	}
	mt_dbg(data->dev, "%s: prop = %d, val = %d\n", __func__,
	       psp, val->intval);
	return ret;
}

static int mt6362_charger_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct mt6362_chg_data *data = power_supply_get_drvdata(psy);
	int ret;

	mt_dbg(data->dev, "%s: prop = %d, val = %d\n", __func__,
	       psp, val->intval);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = mt6362_charger_set_online(data, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6362_charger_set_ichg(data, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mt6362_charger_set_cv(data, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6362_charger_set_aicr(data, val);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = mt6362_charger_set_iprechg(data, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = mt6362_charger_set_ieoc(data, val);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int mt6362_charger_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property mt6362_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static enum power_supply_usb_type mt6362_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static const struct power_supply_desc mt6362_charger_desc = {
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= mt6362_charger_properties,
	.num_properties		= ARRAY_SIZE(mt6362_charger_properties),
	.get_property		= mt6362_charger_get_property,
	.set_property		= mt6362_charger_set_property,
	.property_is_writeable	= mt6362_charger_property_is_writeable,
	.usb_types		= mt6362_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(mt6362_charger_usb_types),
};

static char *mt6362_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

/*
 * =================
 * charger class ops
 * =================
 */

static int mt6362_enable_charging(struct charger_device *chg_dev, bool en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;
	u32 ichg_ramp_t = 0;

	mt_dbg(data->dev, "%s: en = %d\n", __func__, en);

	/* Workaround for vsys overshoot */
	mutex_lock(&data->ichg_lock);
	if (data->ichg < 500000) {
		dev_info(data->dev,
			 "%s: ichg < 500mA, bypass vsys wkard\n", __func__);
		goto out;
	}
	if (!en) {
		data->ichg_dis_chg = data->ichg;
		ichg_ramp_t = (data->ichg - MT6362_CC_MIN) / MT6362_CC_STEP * 2;
		/* Set ichg to 500mA */
		ret = regmap_update_bits(data->regmap,
					 MT6362_REG_CHG_ICHG,
					 MT6362_MASK_CC,
					 0x0A << MT6362_SHFT_CC);
		if (ret < 0) {
			dev_err(data->dev,
				"%s: set ichg 500mA fail\n", __func__);
			goto vsys_wkard_fail;
		}
		mdelay(ichg_ramp_t);
	} else {
		if (data->ichg == data->ichg_dis_chg) {
			ret = __mt6362_set_ichg(data, data->ichg);
			if (ret < 0)
				dev_err(data->dev,
					"%s: recover ichg fail\n", __func__);
		}
	}
out:
	ret = regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_TOP1,
				 MT6362_MASK_CHG_EN,
				 en ? 0xff : 0);
vsys_wkard_fail:
	mutex_unlock(&data->ichg_lock);
	return ret;
}

static int mt6362_set_ichg(struct charger_device *chg_dev, u32 uA)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;

	mt_dbg(data->dev, "%s: ichg = %d\n", __func__, uA);
	mutex_lock(&data->ichg_lock);
	ret = __mt6362_set_ichg(data, uA);
	mutex_unlock(&data->ichg_lock);
	return ret;
}

static int mt6362_get_ichg(struct charger_device *chg_dev, u32 *uA)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	int ret;

	ret = mt6362_charger_get_ichg(data, &val);
	if (ret < 0)
		return ret;
	*uA = val.intval;
	mt_dbg(data->dev, "%s: ichg = %d\n", __func__, *uA);
	return 0;
}

static int mt6362_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 300000;
	return 0;
}

static int mt6362_set_cv(struct charger_device *chg_dev, u32 uV)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;

	mt_dbg(data->dev, "%s: cv = %d\n", __func__, uV);
	val.intval = uV;
	return mt6362_charger_set_cv(data, &val);
}

static int mt6362_get_cv(struct charger_device *chg_dev, u32 *uV)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	int ret;

	ret = mt6362_charger_get_cv(data, &val);
	if (ret < 0)
		return ret;
	*uV = val.intval;
	mt_dbg(data->dev, "%s: cv = %d\n", __func__, *uV);
	return 0;
}

static int mt6362_set_aicr(struct charger_device *chg_dev, u32 uA)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;

	mt_dbg(data->dev, "%s: aicr = %d\n", __func__, uA);
	val.intval = uA;
	return mt6362_charger_set_aicr(data, &val);
}

static int mt6362_get_aicr(struct charger_device *chg_dev, u32 *uA)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;
	int ret;

	ret = mt6362_charger_get_aicr(data, &val);
	if (ret < 0)
		return ret;
	*uA = val.intval;
	mt_dbg(data->dev, "%s: aicr = %d\n", __func__, *uA);
	return 0;
}

static int mt6362_get_min_aicr(struct charger_device *chg_dev, u32 *uA)
{
	*uA = MT6362_AICR_MIN;
	return 0;
}

static int mt6362_get_ieoc(struct mt6362_chg_data *data, u32 *uA)
{
	int ret;
	u32 regval = 0;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_EOC, &regval);
	if (ret < 0)
		return ret;
	regval = (regval & MT6362_MASK_IEOC) >> MT6362_SHFT_IEOC;
	*uA = mt6362_map_real_val(regval, MT6362_IEOC_MIN, MT6362_IEOC_MAX,
				  MT6362_IEOC_STEP);
	mt_dbg(data->dev, "%s: ieoc = %d\n", __func__, *uA);
	return 0;
}

static int mt6362_set_ieoc(struct charger_device *chg_dev, u32 uA)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	union power_supply_propval val;

	mt_dbg(data->dev, "%s: ieoc = %d\n", __func__, uA);
	val.intval = uA;
	return mt6362_charger_set_ieoc(data, &val);
}

static int __mt6362_set_mivr(struct mt6362_chg_data *data, u32 uV)
{
	u8 sel;

	mt_dbg(data->dev, "%s: mivr = %d\n", __func__, uV);
	sel = mt6362_map_reg_sel(uV, MT6362_MIVR_MIN, MT6362_MIVR_MAX,
				 MT6362_MIVR_STEP);
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_MIVR,
				  MT6362_MASK_MIVR,
				  sel << MT6362_SHFT_MIVR);
}

static int mt6362_set_mivr(struct charger_device *chg_dev, u32 uV)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;

	mutex_lock(&data->bd_lock);
	if (data->bd_flag) {
		dev_info(data->dev,
			 "%s: ignore until disable flash\n", __func__);
		data->bd_mivr = uV;
		mutex_unlock(&data->bd_lock);
		return 0;
	}
	ret = __mt6362_set_mivr(data, uV);
	mutex_unlock(&data->bd_lock);
	return ret;
}

static inline int mt6362_get_mivr(struct charger_device *chg_dev, u32 *uV)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;
	u32 regval = 0;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_MIVR, &regval);
	if (ret < 0)
		return ret;
	regval = (regval & MT6362_MASK_MIVR) >> MT6362_SHFT_MIVR;
	*uV = mt6362_map_real_val(regval, MT6362_MIVR_MIN, MT6362_MIVR_MAX,
				  MT6362_MIVR_STEP);
	mt_dbg(data->dev, "%s: mivr = %d\n", __func__, *uV);
	return 0;
}

static int mt6362_get_mivr_state(struct charger_device *chg_dev, bool *in_loop)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;
	u32 regval = 0;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_STAT1, &regval);
	if (ret < 0)
		return ret;
	*in_loop = (regval & MT6362_MASK_MIVR_STAT) ? true : false;
	mt_dbg(data->dev, "%s: in_loop = %d\n", __func__, *in_loop);
	return 0;
}

static int mt6362_enable_te(struct charger_device *chg_dev, bool en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	struct mt6362_chg_platform_data *pdata = dev_get_platdata(data->dev);

	mt_dbg(data->dev, "%s: en = %d\n", __func__, en);
	if (!pdata->en_te)
		return 0;
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_EOC,
				  MT6362_MASK_TE,
				  en ? 0xff : 0);
}

static int mt6362_run_pump_express(struct mt6362_chg_data *data,
				   enum pe_sel pe_sel)
{
	long timeout, pe_timeout = pe_sel ? 1400 : 2800;
	int ret;

	dev_info(data->dev, "%s\n", __func__);
	ret = mt6362_set_aicr(data->chg_dev, 800000);
	if (ret < 0)
		return ret;
	ret = mt6362_set_ichg(data->chg_dev, 2000000);
	if (ret < 0)
		return ret;
	ret = mt6362_enable_charging(data->chg_dev, true);
	if (ret < 0)
		return ret;
	/* switch pe10/pe20 select */
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_PUMPX,
				 MT6362_MASK_PE_SEL, pe_sel ? 0xff : 0);
	if (ret < 0)
		return ret;
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_PUMPX,
				 MT6362_MASK_PE_EN, 0x00);
	if (ret < 0)
		return ret;
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_PUMPX,
				 MT6362_MASK_PE_EN, 0xff);
	if (ret < 0)
		return ret;
	reinit_completion(&data->pe_done);
	timeout = wait_for_completion_interruptible_timeout(
			       &data->pe_done, msecs_to_jiffies(pe_timeout));
	if (timeout == 0)
		ret = -ETIMEDOUT;
	else if (timeout < 0)
		ret = -EINTR;
	else
		ret = 0;
	if (ret < 0)
		dev_err(data->dev,
			"%s: wait pumpx timeout, ret = %d\n", __func__, ret);
	return ret;
}

static int mt6362_set_pep_current_pattern(struct charger_device *chg_dev,
					  bool is_inc)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;

	dev_dbg(data->dev, "%s: pe1.0 pump up = %d\n", __func__, is_inc);
	mutex_lock(&data->pe_lock);
	/* Set Pump Up/Down */
	ret = regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_PUMPX,
				 MT6362_MASK_PE10_INC,
				 is_inc ? 0xff : 0);
	if (ret < 0) {
		dev_err(data->dev, "%s: set pe10 up/down fail\n", __func__);
		goto out;
	}
	ret = mt6362_run_pump_express(data, MT6362_PE_SEL_10);
	if (ret < 0)
		dev_err(data->dev, "%s: run pump express fail\n", __func__);
out:
	mutex_unlock(&data->pe_lock);
	return ret;
}

static int mt6362_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
	return 0;
}

static int mt6362_set_pep20_current_pattern(struct charger_device *chg_dev,
					    u32 uV)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;
	u8 sel;

	dev_dbg(data->dev, "%s: pep2.0 = %d\n", __func__, uV);
	mutex_lock(&data->pe_lock);
	sel = mt6362_map_reg_sel(uV, MT6362_PE20_MIN, MT6362_PE20_MAX,
				 MT6362_PE20_STEP);
	/* Set Voltage */
	ret = regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_PUMPX,
				 MT6362_MASK_PE20_CODE,
				 sel << MT6362_SHFT_PE20_CODE);
	if (ret < 0) {
		dev_err(data->dev, "%s: set pumpx voltage fail\n", __func__);
		goto out;
	}
	ret = mt6362_run_pump_express(data, MT6362_PE_SEL_20);
	if (ret < 0)
		dev_err(data->dev, "%s: run pump express fail\n", __func__);
out:
	mutex_unlock(&data->pe_lock);
	return ret;
}

static int mt6362_reset_ta(struct charger_device *chg_dev)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;

	dev_dbg(data->dev, "%s\n", __func__);
	ret = mt6362_set_mivr(chg_dev, 4600000);
	if (ret < 0)
		return ret;
	ret = mt6362_set_aicr(chg_dev, 100000);
	if (ret < 0)
		return ret;
	msleep(250);
	return mt6362_set_aicr(chg_dev, 500000);
}

static int mt6362_enable_cable_drop_comp(struct charger_device *chg_dev,
					 bool en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	mutex_lock(&data->pe_lock);
	if (en)
		return 0;
	/* Set disable cable drop compensation */
	ret = regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_PUMPX,
				 MT6362_MASK_PE20_CODE,
				 0x1F << MT6362_SHFT_PE20_CODE);
	if (ret < 0) {
		dev_err(data->dev,
			"%s: set dis cable drop comp fail\n", __func__);
		goto out;
	}
	ret = mt6362_run_pump_express(data, MT6362_PE_SEL_20);
	if (ret < 0)
		dev_err(data->dev, "%s: run pump express fail\n", __func__);
out:
	mutex_unlock(&data->pe_lock);
	return ret;
}

static int mt6362_get_aicc(struct mt6362_chg_data *data, u32 *aicc_val)
{
	int ret;
	u32 regval = 0;
	u8 aicc_sel;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_AICC2, &regval);
	if (ret < 0)
		return ret;
	aicc_sel = (regval & MT6362_MASK_AICC_RPT) >> MT6362_SHFT_AICC_RPT;
	*aicc_val = mt6362_map_real_val(aicc_sel,
					MT6362_AICC_MIN,
					MT6362_AICC_MAX,
					MT6362_CC_STEP);
	return 0;
}

static inline int mt6362_post_aicc_measure(struct charger_device *chg_dev,
					   u32 start, u32 stop, u32 step,
					   u32 *measure)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int cur, ret;
	bool mivr_loop;

	mt_dbg(data->dev,
		"%s: post_aicc = (%d, %d, %d)\n", __func__, start, stop, step);
	for (cur = start; cur < (stop + step); cur += step) {
		/* set_aicr to cur */
		ret = mt6362_set_aicr(chg_dev, cur + step);
		if (ret < 0)
			return ret;
		usleep_range(150, 200);
		ret = mt6362_get_mivr_state(chg_dev, &mivr_loop);
		if (ret < 0)
			return ret;
		/* read mivr stat */
		if (mivr_loop)
			break;
	}
	*measure = cur;
	return 0;
}

static int mt6362_run_aicc(struct charger_device *chg_dev, u32 *uA)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	struct mt6362_chg_platform_data *pdata = dev_get_platdata(data->dev);
	int ret;
	u32 aicc_val, aicr_val;
	bool mivr_loop;
	long timeout;

	/* check MIVR stat is act */
	ret = mt6362_get_mivr_state(chg_dev, &mivr_loop);
	if (ret < 0)
		return ret;
	if (!mivr_loop) {
		mt_dbg(data->dev, "%s: mivr loop not act\n", __func__);
		return ret;
	}

	mt_dbg(data->dev,
		 "%s: aicc_oneshot = %d\n", __func__, pdata->aicc_oneshot);
	/* Auto run AICC */
	if (!pdata->aicc_oneshot) {
		if (!try_wait_for_completion(&data->aicc_done)) {
			dev_info(data->dev, "%s: aicc is not act\n", __func__);
			return 0;
		}

		/* get aicc result */
		ret = mt6362_get_aicc(data, &aicc_val);
		if (ret < 0) {
			dev_err(data->dev,
				"%s: get aicc fail\n", __func__);
			return ret;
		}
		*uA = aicc_val;
		reinit_completion(&data->aicc_done);
		return ret;
	}

	/* Run AICC measure oneshot */
	mutex_lock(&data->pe_lock);
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_AICC1,
				 MT6362_MASK_AICC_EN, 0xff);
	if (ret < 0)
		goto out;
	/* Clear AICC measurement IRQ */
	reinit_completion(&data->aicc_done);
	timeout = wait_for_completion_interruptible_timeout(
				   &data->aicc_done, msecs_to_jiffies(9000));
	if (timeout == 0)
		ret = -ETIMEDOUT;
	else if (timeout < 0)
		ret = -EINTR;
	else
		ret = 0;
	if (ret < 0) {
		dev_err(data->dev,
			"%s: wait AICC time out(%d)\n", __func__, ret);
		goto out;
	}
	/* get aicc_result */
	ret = mt6362_get_aicc(data, &aicc_val);
	if (ret < 0) {
		dev_err(data->dev, "%s: get aicc result fail\n", __func__);
		goto out;
	}

	/* post aicc */
	if (!pdata->post_aicc)
		goto skip_post_aicc;

	dev_info(data->dev, "%s: aicc pre val = %d\n", __func__, aicc_val);
	ret = mt6362_get_aicr(chg_dev, &aicr_val);
	if (ret < 0) {
		dev_err(data->dev, "%s: get aicr fail\n", __func__);
		goto out;
	}
	ret = mt6362_set_aicr(chg_dev, aicc_val);
	if (ret < 0) {
		dev_err(data->dev, "%s: set aicr fail\n", __func__);
		goto out;
	}
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_AICC1,
				 MT6362_MASK_AICC_EN, 0);
	if (ret < 0)
		goto out;
	/* always start/end aicc_val/aicc_val+post_aicc_thr */
	ret = mt6362_post_aicc_measure(chg_dev, aicc_val,
				       aicc_val + pdata->post_aicc_thr,
				       MT6362_AICR_STEP, &aicc_val);
	if (ret < 0)
		goto out;

	ret = mt6362_set_aicr(chg_dev, aicc_val);
	if (ret < 0) {
		dev_err(data->dev, "%s: set aicr fail\n", __func__);
		goto out;
	}
	dev_info(data->dev, "%s: aicc post val = %d\n", __func__, aicc_val);
skip_post_aicc:
	*uA = aicc_val;
out:
	/* Clear EN_AICC */
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_AICC1,
				 MT6362_MASK_AICC_EN, 0);
	mutex_unlock(&data->pe_lock);
	return ret;
}

static int mt6362_enable_power_path(struct charger_device *chg_dev, bool en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	return regmap_update_bits(data->regmap,
				 MT6362_REG_CHG_TOP1,
				 MT6362_MASK_CHG_BUCK_EN,
				 en ? 0xff : 0);
}

static int mt6362_is_power_path_enabled(struct charger_device *chg_dev,
					bool *en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;
	u32 regval;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_TOP1, &regval);
	if (ret < 0)
		return ret;
	*en = (regval & MT6362_MASK_CHG_BUCK_EN) ? true : false;
	dev_info(data->dev, "%s: en = %d\n", __func__, *en);
	return 0;
}

static int mt6362_enable_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_TMR,
				  MT6362_MASK_CHG_TMR_EN,
				  en ? 0xff : 0);
}

static int mt6362_is_safety_timer_enabled(struct charger_device *chg_dev,
					  bool *en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;
	u32 regval;

	ret = regmap_read(data->regmap, MT6362_REG_CHG_TMR, &regval);
	if (ret < 0)
		return ret;
	*en = (regval & MT6362_MASK_CHG_TMR_EN) ? true : false;
	dev_info(data->dev, "%s: en = %d\n", __func__, *en);
	return 0;
}

static int mt6362_set_otg_current_limit(struct charger_device *chg_dev, u32 uA)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int i;

	/* Set higher OC threshold protect */
	for (i = 0; i < ARRAY_SIZE(otg_cc_table); i++) {
		if (uA <= otg_cc_table[i])
			break;
	}
	if (i == ARRAY_SIZE(otg_cc_table))
		i = ARRAY_SIZE(otg_cc_table) - 1;
	dev_info(data->dev,
		"%s: select otg_cc = %d\n", __func__, otg_cc_table[i]);
	return regmap_update_bits(data->regmap,
				  MT6362_REG_OTG_C,
				  MT6362_MASK_OTG_CC,
				  i << MT6362_SHFT_OTG_CC);
}

static int mt6362_enable_otg(struct charger_device *chg_dev, bool en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	ret = mt6362_enable_otg_parameter(data, en);
	if (ret < 0)
		return ret;
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_TOP1,
				  MT6362_MASK_OTG_EN,
				  en ? 0xff : 0);
}

static int mt6362_enable_discharge(struct charger_device *chg_dev, bool en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	const int dischg_retry_cnt = 3;
	bool is_dischg = true;
	int i, ret = 0;
	u32 regval;

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	ret = mt6362_enable_hidden_mode(data, true);
	if (ret < 0)
		return ret;
	/* Set bit6 of reg[0x3B] to 1/0 to enable/disable discharging */
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_HD_TOP1,
				 MT6362_MASK_DISCHG, en ? 0xff : 0);
	if (ret < 0) {
		dev_err(data->dev, "%s: fail, en = %d\n", __func__, en);
		goto out;
	}

	if (!en) {
		for (i = 0; i < dischg_retry_cnt; i++) {
			ret = regmap_read(data->regmap,
					  MT6362_REG_CHG_HD_TOP1, &regval);
			is_dischg = (ret & MT6362_MASK_DISCHG) ? true : false;
			if (!is_dischg)
				break;
			ret = regmap_update_bits(data->regmap,
						 MT6362_REG_CHG_HD_TOP1,
						 MT6362_MASK_DISCHG, 0);
			if (ret < 0)
				dev_err(data->dev,
					"%s: disable dischg failed\n",
					__func__);
		}
		if (i == dischg_retry_cnt)
			dev_err(data->dev, "%s: dischg failed\n", __func__);
	}
out:
	mt6362_enable_hidden_mode(data, false);
	return ret;
}


static int mt6362_enable_chg_type_det(struct charger_device *chg_dev, bool en)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	mt6362_run_bc12_thread(data, MT6362_BC12_TRIG_TYPEC, en);
	return 0;
}

static int mt6362_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret, channel;

	switch (chan) {
	case ADC_CHANNEL_VBUS:
		channel = MT6362_CHG_ADCCH_CHGVINDIV5;
		break;
	case ADC_CHANNEL_VSYS:
		channel = MT6362_CHG_ADCCH_VSYS;
		break;
	case ADC_CHANNEL_VBAT:
		channel = MT6362_CHG_ADCCH_VBAT;
		break;
	case ADC_CHANNEL_IBUS:
		channel = MT6362_CHG_ADCCH_IBUS;
		break;
	case ADC_CHANNEL_IBAT:
		channel = MT6362_CHG_ADCCH_IBAT;
		break;
	case ADC_CHANNEL_TEMP_JC:
		channel = MT6362_CHG_ADCCH_TEMP_JC;
		break;
	default:
		return -EINVAL;
	}
	mt_dbg(data->dev, "%s: read channel(%d)\n", __func__, channel);
	ret = iio_read_channel_processed(&data->iio_ch[channel], min);
	if (ret < 0) {
		dev_info(data->dev, "%s: fail(%d)\n", __func__, ret);
		return ret;
	}
	*max = *min;
	mt_dbg(data->dev, "%s: chan[%s] = %d\n",
		 __func__, mt6362_adcch_list[channel], *min);
	return 0;
}

static int mt6362_get_vbus(struct charger_device *chg_dev, u32 *vbus)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	mt_dbg(data->dev, "%s\n", __func__);
	return mt6362_get_adc(chg_dev, ADC_CHANNEL_VBUS, vbus, vbus);
}

static int mt6362_get_ibus(struct charger_device *chg_dev, u32 *ibus)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	mt_dbg(data->dev, "%s\n", __func__);
	return mt6362_get_adc(chg_dev, ADC_CHANNEL_IBUS, ibus, ibus);
}

static int mt6362_get_ibat(struct charger_device *chg_dev, u32 *ibat)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	mt_dbg(data->dev, "%s\n", __func__);
	return mt6362_get_adc(chg_dev, ADC_CHANNEL_IBAT, ibat, ibat);
}

static int mt6362_get_tchg(struct charger_device *chg_dev,
			   int *tchg_min, int *tchg_max)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int temp_jc = 0, ret = 0, retry_cnt = 3;

	mt_dbg(data->dev, "%s\n", __func__);
	/* temp abnormal Workaround */
	do {
		ret = mt6362_get_adc(chg_dev, ADC_CHANNEL_TEMP_JC,
				     &temp_jc, &temp_jc);
		if (ret < 0) {
			dev_err(data->dev,
				"%s: failed, ret = %d\n", __func__, ret);
			return ret;
		}
	} while (temp_jc >= 120 && (retry_cnt--) > 0);
	mutex_lock(&data->tchg_lock);
	if (temp_jc >= 120)
		temp_jc = data->tchg;
	else
		data->tchg = temp_jc;
	mutex_unlock(&data->tchg_lock);
	*tchg_min = *tchg_max = temp_jc;
	dev_info(data->dev, "%s: tchg = %d\n", __func__, temp_jc);
	return 0;
}

static int mt6362_kick_wdt(struct charger_device *chg_dev)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	mt_dbg(data->dev, "%s\n", __func__);
	return regmap_update_bits(data->regmap, MT6362_REG_CHG_WDT,
				  MT6362_MASK_WDT_CNT_RST, 0xff);
}

static int mt6362_safety_check(struct charger_device *chg_dev, u32 polling_ieoc)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret, ibat = 0;
	static int eoc_cnt;

	mt_dbg(data->dev, "%s\n", __func__);
	ret = mt6362_get_ibat(chg_dev, &ibat);
	if (ret < 0) {
		dev_err(data->dev, "%s: failed(%d)\n", __func__, ret);
		return ret;
	}

	if (ibat <= polling_ieoc)
		eoc_cnt++;
	else
		eoc_cnt = 0;
	/* If ibat is less than polling_ieoc for 3 times, trigger EOC event */
	if (eoc_cnt == 3) {
		dev_info(data->dev, "%s: polling_ieoc = %d, ibat = %d\n",
			 __func__, polling_ieoc, ibat);
		charger_dev_notify(data->chg_dev, CHARGER_DEV_NOTIFY_EOC);
		eoc_cnt = 0;
	}
	return ret;
}

static int mt6362_reset_eoc_state(struct charger_device *chg_dev)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	dev_info(data->dev, "%s\n", __func__);
	return regmap_update_bits(data->regmap,
				  MT6362_REG_CHG_EOC,
				  MT6362_MASK_EOC_RST,
				  0xff);
}

static int mt6362_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	enum mt6362_ic_stat ic_stat;
	int ret;

	ret = mt6362_get_charging_status(data, &ic_stat);
	if (ret < 0)
		return ret;
	*done = (ic_stat == MT6362_STAT_CHG_DONE) ? true : false;
	mt_dbg(data->dev, "%s: done = %d\n", __func__, *done);
	return 0;
}

static int mt6362_get_zcv(struct charger_device *chg_dev, u32 *uV)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	dev_info(data->dev, "%s: zcv = %dmV\n", __func__, data->zcv / 1000);
	*uV = data->zcv;
	return 0;
}

static int mt6362_dump_registers(struct charger_device *chg_dev)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int i, ret;
	u32 ichg = 0, aicr = 0, mivr = 0, cv = 0, ieoc = 0;
	enum mt6362_ic_stat ic_stat = MT6362_STAT_HZ;
	bool chg_en = false;
	u32 adc_vals[5];
	u8 chg_stat[2], chg_top[2];
	u32 chg_eoc = 0;

	ret = mt6362_kick_wdt(chg_dev);
	if (ret < 0) {
		dev_notice(data->dev, "%s: kick wdt fail\n", __func__);
		return ret;
	}
	ret = mt6362_get_ichg(chg_dev, &ichg);
	ret |= mt6362_get_aicr(chg_dev, &aicr);
	ret |= mt6362_get_mivr(chg_dev, &mivr);
	ret |= mt6362_get_cv(chg_dev, &cv);
	ret |= mt6362_get_ieoc(data, &ieoc);
	ret |= mt6362_get_charging_status(data, &ic_stat);
	ret |= mt6362_is_charger_enabled(data, &chg_en);
	if (ret < 0) {
		dev_err(data->dev, "%s: get chg setting fail\n", __func__);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(adc_vals); i++) {
		ret = iio_read_channel_processed(&data->iio_ch[i],
						 &adc_vals[i]);
		if (ret < 0) {
			dev_err(data->dev,
				"%s: read [%s] adc fail(%d)\n",
				__func__, mt6362_adcch_list[i], ret);
			return ret;
		}
	}

	ret = regmap_bulk_read(data->regmap, MT6362_REG_CHG_STAT0, chg_stat, 2);
	if (ret < 0)
		return ret;
	ret = regmap_bulk_read(data->regmap, MT6362_REG_CHG_TOP1, chg_top, 2);
	if (ret < 0)
		return ret;
	ret = regmap_read(data->regmap, MT6362_REG_CHG_EOC, &chg_eoc);
	if (ret < 0)
		return ret;

	dev_info(data->dev,
		 "%s: ICHG = %dmA, AICR = %dmA, MIVR = %dmV, IEOC = %dmA, CV = %dmV\n",
		 __func__, ichg / 1000, aicr / 1000, mivr / 1000, ieoc / 1000,
		 cv / 1000);
	dev_info(data->dev,
		 "%s: VBUS = %dmV, IBUS = %dmA, VSYS = %dmV, VBAT = %dmV, IBAT = %dmA\n",
		 __func__,
		 adc_vals[MT6362_ADCCH_CHGVINDIV5] / 1000,
		 adc_vals[MT6362_ADCCH_IBUS] / 1000,
		 adc_vals[MT6362_ADCCH_VSYS] / 1000,
		 adc_vals[MT6362_ADCCH_VBAT] / 1000,
		 adc_vals[MT6362_ADCCH_IBAT] / 1000);
	dev_info(data->dev, "%s: CHG_EN = %d, CHG_STATUS = %s, CHG_STAT0 = 0x%02X, CHG_STAT1 = 0x%02X\n",
		 __func__, chg_en, mt6362_ic_stat_list[ic_stat],
		 chg_stat[0], chg_stat[1]);
	dev_info(data->dev, "%s: CHG_TOP1 = 0x%02X, CHG_TOP2 = 0x%02X, CHG_EOC = 0x%02X\n",
		 __func__, chg_top[0], chg_top[1], chg_eoc);
	return 0;
}

static int mt6362_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	dev_info(data->dev, "%s\n", __func__);
	switch (event) {
	case EVENT_FULL:
	case EVENT_RECHARGE:
	case EVENT_DISCHARGE:
		power_supply_changed(data->psy);
		break;
	default:
		break;
	}
	return 0;
}

static int mt6362_handle_bleed_discharge(struct mt6362_chg_data *data)
{
	int ret;

	/* Decrease UUG leakage to vbus */
	if (data->bd_flag) {
		ret = mt6362_enable_discharge(data->chg_dev, true);
		if (ret < 0)
			return ret;
		return regmap_update_bits(data->regmap,
					  MT6362_PD_I2C_TO_RST_CTRL,
					  MT6362_MASK_BLEEDDIS_EN, 0xff);
	} else {
		ret = mt6362_enable_discharge(data->chg_dev, false);
		if (ret < 0)
			return ret;
		return regmap_update_bits(data->regmap,
					  MT6362_PD_I2C_TO_RST_CTRL,
					  MT6362_MASK_BLEEDDIS_EN,
					  data->pwr_rdy ? 0 : 0xff);
	}
}

static int mt6362_enable_bleed_discharge(struct mt6362_chg_data *data,
					 bool en)
{
	int ret = 0;

	dev_info(data->dev, "%s: en = %d\n", __func__, en);
	mutex_lock(&data->bd_lock);
	if (en == data->bd_flag)
		goto out;
	if (en) {
		ret = mt6362_get_mivr(data->chg_dev, &data->bd_mivr);
		if (ret < 0)
			goto out;
		ret = __mt6362_set_mivr(data, MT6362_MIVR_MAX);
		if (ret < 0)
			goto out;
	}

	ret = mt6362_enable_otg_parameter(data, en);
	if (ret < 0)
		goto out;
	ret = mt6362_handle_bleed_discharge(data);
	if (ret < 0)
		goto out;
	if (!en) {
		ret = __mt6362_set_mivr(data, data->bd_mivr);
		if (ret < 0)
			goto out;
	}
	data->bd_flag = en;
out:
	mutex_unlock(&data->bd_lock);
	return ret;
}

static int mt6362_set_property(struct charger_device *chg_dev,
			       enum charger_property prop,
			       union charger_propval *val)
{
	int ret;
	struct mt6362_chg_data *data = charger_get_data(chg_dev);

	switch (prop) {
	case CHARGER_PROP_BLEED_DISCHARGE:
		ret = mt6362_enable_bleed_discharge(data,
						    val->intval ? true : false);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int mt6362_plug_in(struct charger_device *chg_dev)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;

	dev_info(data->dev, "%s\n", __func__);
	ret = mt6362_enable_wdt(data, true);
	if (ret < 0) {
		dev_err(data->dev, "%s: en wdt failed\n", __func__);
		return ret;
	}
	ret = mt6362_enable_te(chg_dev, true);
	if (ret < 0)
		dev_err(data->dev, "%s: en te failed\n", __func__);
	return 0;
}

static int mt6362_plug_out(struct charger_device *chg_dev)
{
	struct mt6362_chg_data *data = charger_get_data(chg_dev);
	int ret;

	dev_info(data->dev, "%s\n", __func__);
	ret = mt6362_enable_wdt(data, false);
	if (ret < 0) {
		dev_err(data->dev, "%s: disable wdt failed\n", __func__);
		return ret;
	}
	ret = mt6362_enable_te(chg_dev, false);
	if (ret < 0)
		dev_err(data->dev, "%s: disable te failed\n", __func__);
	return 0;
}

static const struct charger_ops mt6362_chg_ops = {
	/* cable plug in/out */
	.plug_in = mt6362_plug_in,
	.plug_out = mt6362_plug_out,
	/* enable */
	.enable = mt6362_enable_charging,
	/* charging current */
	.set_charging_current = mt6362_set_ichg,
	.get_charging_current = mt6362_get_ichg,
	.get_min_charging_current = mt6362_get_min_ichg,
	/* charging voltage */
	.set_constant_voltage = mt6362_set_cv,
	.get_constant_voltage = mt6362_get_cv,
	/* charging input current */
	.set_input_current = mt6362_set_aicr,
	.get_input_current = mt6362_get_aicr,
	.get_min_input_current = mt6362_get_min_aicr,
	/* set termination current */
	.set_eoc_current = mt6362_set_ieoc,
	/* charging mivr */
	.set_mivr = mt6362_set_mivr,
	.get_mivr = mt6362_get_mivr,
	.get_mivr_state = mt6362_get_mivr_state,
	/* charing termination */
	.enable_termination = mt6362_enable_te,
	/* PE+/PE+20 */
	.send_ta_current_pattern = mt6362_set_pep_current_pattern,
	.set_pe20_efficiency_table = mt6362_set_pep20_efficiency_table,
	.send_ta20_current_pattern = mt6362_set_pep20_current_pattern,
	.reset_ta = mt6362_reset_ta,
	.enable_cable_drop_comp = mt6362_enable_cable_drop_comp,
	.run_aicl = mt6362_run_aicc,
	/* Power path */
	.enable_powerpath = mt6362_enable_power_path,
	.is_powerpath_enabled = mt6362_is_power_path_enabled,
	/* safety timer */
	.enable_safety_timer = mt6362_enable_safety_timer,
	.is_safety_timer_enabled = mt6362_is_safety_timer_enabled,
	/* OTG */
	.set_boost_current_limit = mt6362_set_otg_current_limit,
	.enable_otg = mt6362_enable_otg,
	.enable_discharge = mt6362_enable_discharge,
	/* Charger type detection */
	.enable_chg_type_det = mt6362_enable_chg_type_det,
	/* ADC */
	.get_adc = mt6362_get_adc,
	.get_vbus_adc = mt6362_get_vbus,
	.get_ibus_adc = mt6362_get_ibus,
	.get_ibat_adc = mt6362_get_ibat,
	.get_tchg_adc = mt6362_get_tchg,
	/* kick wdt */
	.kick_wdt = mt6362_kick_wdt,
	/* misc */
	.safety_check = mt6362_safety_check,
	.reset_eoc_state = mt6362_reset_eoc_state,
	.is_charging_done = mt6362_is_charging_done,
	.get_zcv = mt6362_get_zcv,
	.dump_registers = mt6362_dump_registers,
	/* event */
	.event = mt6362_do_event,
	/* Workaround */
	.set_property = mt6362_set_property,
};

static inline int mt6362_set_shipping_mode(struct mt6362_chg_data *data)
{
	int ret;

	ret = regmap_update_bits(data->regmap, MT6362_REG_CORE_CTRL2,
				 MT6362_MASK_SHIP_RST_DIS, 0xff);
	if (ret < 0) {
		dev_notice(data->dev, "%s: ship rst disable fail\n", __func__);
		return ret;
	}
	return regmap_write(data->regmap, MT6362_REG_CHG_TOP1,
			    MT6362_MASK_BATFET_DIS);
}

static ssize_t shipping_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mt6362_chg_data *data = dev_get_drvdata(dev);
	int32_t tmp = 0;
	int ret = 0;

	if (kstrtoint(buf, 10, &tmp) < 0) {
		dev_notice(dev, "parsing number fail\n");
		return -EINVAL;
	}
	if (tmp != 5526789)
		return -EINVAL;
	ret = mt6362_set_shipping_mode(data);
	if (ret < 0)
		return ret;
	return count;
}
static const DEVICE_ATTR_WO(shipping_mode);

static void mt6362_get_boot_mode(struct mt6362_chg_data *data)
{
	struct device_node *boot_node = NULL;
	struct tag_bootmode *tag = NULL;

	boot_node = of_parse_phandle(data->dev->of_node, "bootmode", 0);
	if (!boot_node)
		dev_info(data->dev, "%s: failed to get boot mode phandle\n",
			 __func__);
	else {
		tag = (struct tag_bootmode *)of_get_property(boot_node,
							     "atag,boot", NULL);
		if (!tag)
			dev_info(data->dev, "%s: failed to get atag,boot\n",
				 __func__);
		else {
			dev_info(data->dev,
			"%s: size:0x%x tag:0x%x bootmode:0x%x boottype:0x%x\n",
				__func__, tag->size, tag->tag,
				tag->bootmode, tag->boottype);
			data->bootmode = tag->bootmode;
			data->boottype = tag->boottype;
		}
	}
}

static int mt6362_chg_init_setting(struct mt6362_chg_data *data)
{
	u32 regval = 0;
	int ret;

	/* get boot mode */
	mt6362_get_boot_mode(data);

	/* disable ilim en need delay 5ms */
	usleep_range(5000, 6000);
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_AICR,
				 MT6362_MASK_ILIM_EN, 0);
	if (ret < 0) {
		dev_err(data->dev, "%s: disable ilim_en failed\n", __func__);
		return ret;
	}
	if (data->bootmode == 1 || data->bootmode == 5) {
		/* 1:META_BOOT 5:ADVMETA_BOOT */
		ret = regmap_update_bits(data->regmap,
						 MT6362_REG_CHG_AICR,
						 MT6362_MASK_AICR,
						 0x06 << MT6362_SHFT_AICR);
		dev_info(data->dev, "%s: set aicr to 200mA in meta mode\n",
			__func__);
	}
	/* disable wdt reduce 1mA power consumption */
	ret = mt6362_enable_wdt(data, false);
	if (ret < 0) {
		dev_err(data->dev, "%s: disable wdt failed\n", __func__);
		return ret;
	}
	/* Disable USB charger type detect, no matter use it or not */
	ret = regmap_update_bits(data->regmap, MT6362_REG_BC12_FUNC,
				 MT6362_MASK_BC12_EN, 0);
	if (ret < 0) {
		dev_err(data->dev, "%s: disable chg type detect fail\n",
			__func__);
		return ret;
	}
	/* Disable TE, set TE when plug in/out */
	ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_EOC,
				 MT6362_MASK_TE, 0);
	if (ret < 0) {
		dev_err(data->dev, "%s: disable te fail\n", __func__);
		return ret;
	}
	/* Check BATSYSUV occurred last time boot-on */
	ret = regmap_read(data->regmap, MT6362_REG_CHG_TOP1, &regval);
	if (ret < 0)
		return ret;
	if (!(regval & MT6362_MASK_PP_PG_FLAG)) {
		dev_warn(data->dev, "%s: BATSYSUV occurred\n", __func__);
		ret = regmap_update_bits(data->regmap, MT6362_REG_CHG_TOP1,
					 MT6362_MASK_PP_PG_FLAG, 0xff);
		if (ret < 0)
			dev_err(data->dev,
				"%s: set BATSYSUV flag fail\n", __func__);
	}
	return ret;
}

static const struct charger_properties mt6362_chg_props = {
	.alias_name = "mt6362_chg",
};

struct mt6362_charger_irqt {
	const char *name;
	irq_handler_t	irqh;
	int irq;
};

#define MT6362_IRQ_DECLARE(_name) \
{\
	.name = #_name,\
	.irqh = mt6362_##_name##_evt_handler,\
	.irq = -1,\
}

static irqreturn_t mt6362_fl_pwr_rdy_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_info(cdata->dev, "%s\n", __func__);
	mutex_lock(&cdata->bd_lock);
	cdata->pwr_rdy = true;
	mt6362_handle_bleed_discharge(cdata);
	mutex_unlock(&cdata->bd_lock);
	mt6362_run_bc12_thread(cdata, MT6362_BC12_TRIG_MICROB, true);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_detach_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_info(cdata->dev, "%s\n", __func__);
	mutex_lock(&cdata->bd_lock);
	cdata->pwr_rdy = false;
	mt6362_handle_bleed_discharge(cdata);
	mutex_unlock(&cdata->bd_lock);
	mt6362_run_bc12_thread(cdata, MT6362_BC12_TRIG_MICROB, false);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_vbus_ov_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_notice(cdata->dev, "%s\n", __func__);
	charger_dev_notify(cdata->chg_dev, CHARGER_DEV_NOTIFY_VBUS_OVP);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_chg_batov_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_notice(cdata->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_chg_sysov_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_notice(cdata->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_chg_tout_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_notice(cdata->dev, "%s\n", __func__);
	charger_dev_notify(cdata->chg_dev, CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_chg_threg_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_notice(cdata->dev, "%s: thermal regulation\n", __func__);
	return IRQ_HANDLED;
}

static void mt6362_chg_irq_enable(const char *name, int en);
static irqreturn_t mt6362_fl_chg_mivr_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_notice(cdata->dev, "%s\n", __func__);
	mt6362_chg_irq_enable("chg_mivr_evt", 0);
	atomic_inc(&cdata->mivr_cnt);
	wake_up(&cdata->waitq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_aicc_done_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_info(cdata->dev, "%s\n", __func__);
	complete(&cdata->aicc_done);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_pe_done_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;

	dev_info(cdata->dev, "%s\n", __func__);
	complete(&cdata->pe_done);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_wdt_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;
	int ret;

	dev_info(cdata->dev, "%s\n", __func__);
	ret = mt6362_kick_wdt(cdata->chg_dev);
	if (ret < 0)
		dev_err(cdata->dev, "%s: kick wdt failed\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6362_fl_bc12_dn_evt_handler(int irq, void *data)
{
	struct mt6362_chg_data *cdata = data;
	int ret;
	u32 regval = 0;
	enum mt6362_chg_type port_stat;

	dev_info(cdata->dev, "%s\n", __func__);
	mutex_lock(&cdata->bc12_lock);
	if (!cdata->bc12_update) {
		dev_info(cdata->dev, "%s: no need update bc12\n", __func__);
		mutex_unlock(&cdata->bc12_lock);
		return IRQ_HANDLED;
	}
	cdata->bc12_update = false;
	mutex_unlock(&cdata->bc12_lock);

	ret = regmap_read(cdata->regmap, MT6362_REG_BC12_STAT, &regval);
	if (ret < 0)
		return ret;
	port_stat = (regval & MT6362_MASK_PORT_STAT) >> MT6362_SHFT_PORT_STAT;
	switch (port_stat) {
	case MT6362_CHG_TYPE_NO_INFO:
		dev_info(cdata->dev, "%s: no information\n", __func__);
		return IRQ_HANDLED;
	case MT6362_CHG_TYPE_UNKNOWN_TA:
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		cdata->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	case MT6362_CHG_TYPE_SDP:
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		cdata->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case MT6362_CHG_TYPE_CDP:
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		cdata->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case MT6362_CHG_TYPE_APPLE_10W:
	case MT6362_CHG_TYPE_SAMSUNG_10W:
	case MT6362_CHG_TYPE_APPLE_5W:
	case MT6362_CHG_TYPE_APPLE_12W:
	case MT6362_CHG_TYPE_DCP:
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		cdata->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	default:
		return IRQ_HANDLED;
	}
	if (cdata->psy_desc.type != POWER_SUPPLY_TYPE_USB_DCP)
		__mt6362_enable_bc12(cdata, false);
	power_supply_changed(cdata->psy);
	if (ret < 0)
		dev_err(cdata->dev, "%s: report psy online fail\n", __func__);
	return IRQ_HANDLED;
}

static struct mt6362_charger_irqt irqts[] = {
	MT6362_IRQ_DECLARE(fl_pwr_rdy),
	MT6362_IRQ_DECLARE(fl_detach),
	MT6362_IRQ_DECLARE(fl_vbus_ov),
	MT6362_IRQ_DECLARE(fl_chg_batov),
	MT6362_IRQ_DECLARE(fl_chg_sysov),
	MT6362_IRQ_DECLARE(fl_chg_tout),
	MT6362_IRQ_DECLARE(fl_chg_threg),
	MT6362_IRQ_DECLARE(fl_chg_mivr),
	MT6362_IRQ_DECLARE(fl_aicc_done),
	MT6362_IRQ_DECLARE(fl_pe_done),
	MT6362_IRQ_DECLARE(fl_wdt),
	MT6362_IRQ_DECLARE(fl_bc12_dn),
};

static int mt6362_charger_irq_register(struct platform_device *pdev)
{
	int i, rv;

	for (i = 0; i < ARRAY_SIZE(irqts); i++) {
		rv = platform_get_irq_byname(pdev, irqts[i].name);
		if (rv <= 0)
			continue;
		irqts[i].irq = rv;

		rv = devm_request_threaded_irq(&pdev->dev, rv, NULL,
					       irqts[i].irqh, 0, NULL,
					       platform_get_drvdata(pdev));
		if (rv)
			return rv;
	}
	return 0;
}

static void mt6362_chg_irq_enable(const char *name, int en)
{
	struct mt6362_charger_irqt *irqt;
	int i = 0;

	if (unlikely(!name))
		return;
	for (i = 0; i < ARRAY_SIZE(irqts); i++) {
		irqt = irqts + i;
		if (unlikely(!irqt->name))
			continue;
		if (!strcmp(irqt->name, name)) {
			if (en)
				enable_irq(irqt->irq);
			else
				disable_irq_nosync(irqt->irq);
			break;
		}
	}
}

static int mt6362_chg_parse_dt_data(struct device *dev,
				    struct mt6362_chg_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct device_node *node;
	int i, ret;

	const struct {
		const char *name;
		u32 *val_ptr;
	} u32_opts[] = {
		{ "ichg", &pdata->ichg },
		{ "aicr", &pdata->aicr },
		{ "mivr", &pdata->mivr },
		{ "cv", &pdata->cv },
		{ "ieoc", &pdata->ieoc },
		{ "safety_timer", &pdata->safety_timer },
		{ "ircmp_resistor", &pdata->ircmp_resistor },
		{ "ircmp_vclamp", &pdata->ircmp_vclamp },
		{ "vbusov_sel", &pdata->vbusov_sel },
		{ "dcdt_sel", &pdata->dcdt_sel },
		{ "specta_det", &pdata->specta_det },
		{ "en_te", &pdata->en_te },
		{ "en_wdt", &pdata->en_wdt },
		{ "aicc_oneshot", &pdata->aicc_oneshot },
		{ "shipping_dly_en", &pdata->shipping_dly_en },
		{ "batoc_notify", &pdata->batoc_notify },
	};

	dev_info(dev, "%s: ++\n", __func__);
	memcpy(pdata, &def_platform_data, sizeof(*pdata));

	for (i = 0; i < ARRAY_SIZE(u32_opts); i++)
		if (of_property_read_u32(np, u32_opts[i].name,
					 u32_opts[i].val_ptr))
			dev_err(dev, "error reading '%s'\n", u32_opts[i].name);

	ret = of_property_read_string(np, "chg_name", &pdata->chg_name);
	if (ret)
		dev_err(dev, "error reading 'chg_name'(%d)\n", ret);
	node = of_parse_phandle(np, "bc12_sel", 0);
	if (!node) {
		dev_err(dev, "can't parse phandle bc12_sel\n");
		return -EINVAL;
	}
	ret = of_property_read_u32(node, "bc12_sel", &pdata->bc12_sel);
	if (ret)
		dev_err(dev, "can't parse bc12_sel\n");
	return ret;
}

static int mt6362_chg_apply_pdata(struct mt6362_chg_data *data,
				  struct mt6362_chg_platform_data *pdata)
{
	int i, ret = 0;
	u8 sel;

	const struct {
		u32 *val_ptr;
		u32 prop_min;
		u32 prop_max;
		u32 prop_step;
		u8 reg;
		u8 mask;
		u8 shift;
	} u32_props[] = {
		{
			&pdata->ichg,
			MT6362_CC_MIN, MT6362_CC_MAX, MT6362_CC_STEP,
			MT6362_REG_CHG_ICHG, MT6362_MASK_CC, MT6362_SHFT_CC,
		},
		{
			&pdata->aicr,
			MT6362_AICR_MIN, MT6362_AICR_MAX, MT6362_AICR_STEP,
			MT6362_REG_CHG_AICR, MT6362_MASK_AICR, MT6362_SHFT_AICR,
		},
		{
			&pdata->mivr,
			MT6362_MIVR_MIN, MT6362_MIVR_MAX, MT6362_MIVR_STEP,
			MT6362_REG_CHG_MIVR, MT6362_MASK_MIVR, MT6362_SHFT_MIVR,
		},
		{
			&pdata->cv,
			MT6362_CV_MIN, MT6362_CV_MAX, MT6362_CV_STEP,
			MT6362_REG_CHG_VCHG, MT6362_MASK_CV, MT6362_SHFT_CV,
		},
		{
			&pdata->ieoc,
			MT6362_IEOC_MIN, MT6362_IEOC_MAX, MT6362_IEOC_STEP,
			MT6362_REG_CHG_EOC, MT6362_MASK_IEOC, MT6362_SHFT_IEOC,
		},
		{
			&pdata->safety_timer,
			MT6362_TMR_MIN, MT6362_TMR_MAX, MT6362_TMR_STEP,
			MT6362_REG_CHG_TMR, MT6362_MASK_CHG_TMR_TIME,
			MT6362_SHFT_CHG_TMR_TIME,
		},
		{
			&pdata->ircmp_resistor,
			MT6362_IR_R_MIN, MT6362_IR_R_MAX, MT6362_IR_R_STEP,
			MT6362_REG_BAT_COMP, MT6362_MASK_BAT_IRCOMP_R,
			MT6362_SHFT_BAT_IRCOMP_R,
		},
		{
			&pdata->ircmp_vclamp,
			MT6362_IR_V_MIN, MT6362_IR_V_MAX, MT6362_IR_V_STEP,
			MT6362_REG_BAT_COMP, MT6362_MASK_BAT_IRCOMP_V,
			MT6362_SHFT_BAT_IRCOMP_V,
		},
	};
	const struct {
		u32 *val_ptr;
		u8 reg;
		u8 mask;
		u8 shift;
	} sel_props[] = {
		{
			&pdata->aicc_oneshot, MT6362_REG_CHG_AICC2,
			MT6362_MASK_AICC_ONESHOT, MT6362_SHFT_AICC_ONESHOT,
		},
		{
			&pdata->shipping_dly_en, MT6362_REG_CHG_TOP1,
			MT6362_MASK_BATFET_DIS_DLY, MT6362_SHFT_BATFET_DIS_DLY,
		},
		{
			&pdata->dcdt_sel,
			MT6362_REG_BC12_FUNC, MT6362_MASK_DCDT_SEL,
			MT6362_SHFT_DCDT_SEL,
		},
		{
			&pdata->specta_det,
			MT6362_REG_BC12_FUNC, MT6362_MASK_SPECTA_EN,
			MT6362_SHFT_SPECTA_EN,
		},
		{
			&pdata->vbusov_sel,
			MT6362_REG_CHG_TOP2, MT6362_MASK_VBUS_OV,
			MT6362_SHFT_VBUS_OV,
		},
	};

	for (i = 0; i < ARRAY_SIZE(u32_props); i++) {
		sel = mt6362_map_reg_sel(*u32_props[i].val_ptr,
					 u32_props[i].prop_min,
					 u32_props[i].prop_max,
					 u32_props[i].prop_step);
		ret |= regmap_update_bits(data->regmap,
					  u32_props[i].reg,
					  u32_props[i].mask,
					  sel << u32_props[i].shift);
	}
	for (i = 0; i < ARRAY_SIZE(sel_props); i++)
		ret |= regmap_update_bits(data->regmap,
					  sel_props[i].reg,
					  sel_props[i].mask,
					  *sel_props[i].val_ptr <<
						sel_props[i].shift);
	return ret;
}

static int mt6362_chg_probe(struct platform_device *pdev)
{
	struct mt6362_chg_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct mt6362_chg_data *data;
	struct power_supply_config charger_cfg = {};
	struct regulator_config config = {};
	bool use_dt = pdev->dev.of_node;
	int rc;

	if (use_dt) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		rc = mt6362_chg_parse_dt_data(&pdev->dev, pdata);
		if (rc < 0) {
			dev_err(&pdev->dev, "parse dt fail\n");
			return rc;
		}
		pdev->dev.platform_data = pdata;
	}
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data specified\n");
		return -EINVAL;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!data->regmap) {
		dev_err(&pdev->dev, "failed to allocate regmap\n");
		return -ENODEV;
	}

	data->dev = &pdev->dev;
	init_completion(&data->aicc_done);
	init_completion(&data->pe_done);
	init_completion(&data->bc12_start);
	mutex_init(&data->hidden_mode_lock);
	mutex_init(&data->ichg_lock);
	mutex_init(&data->tchg_lock);
	mutex_init(&data->pe_lock);
	mutex_init(&data->bd_lock);
	mutex_init(&data->bc12_lock);
	mutex_init(&data->otg_lock);
	data->bd_flag = false;
	data->pwr_rdy = false;
	data->hidden_mode_cnt = 0;
	data->otg_mode_cnt = 0;
	data->tchg = 0;
	data->zcv = 0;
	data->ichg = 2000000;
	data->ichg_dis_chg = 2000000;
	data->bd_mivr = 4400000;
	data->bc12_update = false;
	data->attach = false;
	atomic_set(&data->mivr_cnt, 0);
	init_waitqueue_head(&data->waitq);
	platform_set_drvdata(pdev, data);

	rc = mt6362_chg_apply_pdata(data, pdata);
	if (rc < 0) {
		dev_err(&pdev->dev, "apply pdata fail\n");
		goto out;
	}

	rc = mt6362_chg_init_setting(data);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: init setting fail\n", __func__);
		goto out;
	}

	/* otg regulator */
	config.dev = &pdev->dev;
	config.regmap = data->regmap;
	data->otg_rdev = devm_regulator_register(&pdev->dev, &mt6362_otg_rdesc,
						&config);
	if (IS_ERR(data->otg_rdev)) {
		rc = PTR_ERR(data->otg_rdev);
		goto out;
	}

	/* power supply register */
	memcpy(&data->psy_desc, &mt6362_charger_desc, sizeof(data->psy_desc));
	data->psy_desc.name = dev_name(&pdev->dev);

	charger_cfg.drv_data = data;
	charger_cfg.of_node = pdev->dev.of_node;
	charger_cfg.supplied_to = mt6362_charger_supplied_to;
	charger_cfg.num_supplicants = ARRAY_SIZE(mt6362_charger_supplied_to);
	data->psy = devm_power_supply_register(&pdev->dev,
					      &data->psy_desc, &charger_cfg);
	if (IS_ERR(data->psy)) {
		dev_err(&pdev->dev, "Fail to register power supply dev\n");
		rc = PTR_ERR(data->psy);
		goto out;
	}

	rc = device_create_file(data->dev, &dev_attr_shipping_mode);
	if (rc < 0) {
		dev_notice(&pdev->dev, "create shipping attr fail\n");
		goto out;
	}

	/* mivr task */
	data->mivr_task = kthread_run(mt6362_chg_mivr_task_threadfn, data,
				      devm_kasprintf(data->dev, GFP_KERNEL,
				      "mivr_thread.%s", dev_name(data->dev)));
	rc = PTR_ERR_OR_ZERO(data->mivr_task);
	if (rc < 0) {
		dev_err(data->dev, "create mivr handling thread fail\n");
		goto out_devfs;
	}

	data->iio_ch = devm_iio_channel_get_all(&pdev->dev);
	if (IS_ERR(data->iio_ch)) {
		rc = PTR_ERR(data->iio_ch);
		goto out_devfs;
	}

	/* Read ZCV */
	rc = mt6362_read_zcv(data);
	if (rc < 0) {
		dev_err(data->dev, "%s: read zcv fail\n", __func__);
		goto out_devfs;
	}

	/* charger class register */
	data->chg_dev = charger_device_register(pdata->chg_name, data->dev,
						data, &mt6362_chg_ops,
						&mt6362_chg_props);
	if (IS_ERR(data->chg_dev)) {
		dev_err(data->dev, "charger device register fail\n");
		rc = PTR_ERR(data->chg_dev);
		goto out_devfs;
	}

	rc = mt6362_charger_irq_register(pdev);
	if (rc) {
		dev_err(&pdev->dev,
			"failed to register charger irq\n");
		goto out_chgdev;
	}

	mt6362_get_pwr_rdy_stat(data, &data->pwr_rdy);
	rc = mt6362_handle_bleed_discharge(data);
	if (rc < 0) {
		dev_notice(&pdev->dev, "failed to handle bleed discharge\n");
		goto out_chgdev;
	}

	data->bc12_task = kthread_run(mt6362_bc12_thread, data, "bc12_thread");
	if (IS_ERR(data->bc12_task)) {
		rc = PTR_ERR(data->bc12_task);
		goto out_chgdev;
	}
	mt6362_run_bc12_thread(data, MT6362_BC12_TRIG_MICROB, data->pwr_rdy);

	dev_info(&pdev->dev, "%s: successful probe\n", __func__);
	return 0;
out_chgdev:
	charger_device_unregister(data->chg_dev);
out_devfs:
	device_remove_file(data->dev, &dev_attr_shipping_mode);
out:
	mutex_destroy(&data->hidden_mode_lock);
	mutex_destroy(&data->ichg_lock);
	mutex_destroy(&data->tchg_lock);
	mutex_destroy(&data->pe_lock);
	mutex_destroy(&data->bd_lock);
	return rc;
}

static const struct of_device_id __maybe_unused mt6362_chg_ofid_tbls[] = {
	{ .compatible = "mediatek,mt6362-chg", },
	{ },
};
MODULE_DEVICE_TABLE(of, mt6362_chg_ofid_tbls);

static struct platform_driver mt6362_chg_driver = {
	.driver = {
		.name = "mt6362-chg",
		.of_match_table = of_match_ptr(mt6362_chg_ofid_tbls),
	},
	.probe = mt6362_chg_probe,
};
module_platform_driver(mt6362_chg_driver);

MODULE_AUTHOR("ChiYuan Huang <cy_huang@richtek.com>");
MODULE_DESCRIPTION("MT6362 SPMI CHG Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(MT6362_CHG_DRV_VERSION);

/*
 * Version Note
 * 1.0.2
 * (1) adapt new bc12 architecture
 *
 * 1.0.1
 * (1) fix bleed discharge workaround lock
 * (2) fix otg parameter setting both in otg and flash mode
 *
 * 1.0.0
 * Initial Release
 */
