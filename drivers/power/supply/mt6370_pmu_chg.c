// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/delay.h>
//#include <linux/jiffies.h>
#include <uapi/linux/time.h>
#include <linux/power_supply.h>
#include <linux/iio/consumer.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/regulator/driver.h>
#include <linux/phy/phy.h>
#include <linux/kconfig.h>
#include <linux/reboot.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/math64.h>

#include <linux/mfd/mt6370/mt6370.h>
#include <linux/mfd/mt6370/mt6370-private.h>
#if IS_ENABLED(CONFIG_MTK_CHARGER)
#include "charger_class.h"
#include "mtk_charger.h"
#endif
#include <tcpm.h>

#include "mt6370_pmu_charger.h"

#define MT6370_PMU_CHARGER_DRV_VERSION	"1.0.7_MTK"

#define PHY_MODE_BC11_SET 1
#define PHY_MODE_BC11_CLR 2

bool dbg_log_en = true; /* module param to enable/disable debug log */
module_param(dbg_log_en, bool, 0644);
#define mt_dbg(dev, fmt, ...) \
	do { \
		if (dbg_log_en) \
			dev_dbg(dev, fmt, ##__VA_ARGS__); \
	} while (0)

void __attribute__ ((weak)) Charger_Detect_Init(void)
{
}

void __attribute__ ((weak)) Charger_Detect_Release(void)
{
}

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

/* ======================= */
/* MT6370 Charger Variable */
/* ======================= */

enum mt6370_pmu_charger_irqidx {
	MT6370_CHG_IRQIDX_CHGIRQ1 = 0,
	MT6370_CHG_IRQIDX_CHGIRQ2,
	MT6370_CHG_IRQIDX_CHGIRQ3,
	MT6370_CHG_IRQIDX_CHGIRQ4,
	MT6370_CHG_IRQIDX_CHGIRQ5,
	MT6370_CHG_IRQIDX_CHGIRQ6,
	MT6370_CHG_IRQIDX_QCIRQ,
	MT6370_CHG_IRQIDX_DICHGIRQ7,
	MT6370_CHG_IRQIDX_OVPCTRLIRQ,
	MT6370_CHG_IRQIDX_MAX,
};

enum mt6370_pmu_chg_type {
	MT6370_CHG_TYPE_NOVBUS = 0,
	MT6370_CHG_TYPE_UNDER_GOING,
	MT6370_CHG_TYPE_SDP,
	MT6370_CHG_TYPE_SDPNSTD,
	MT6370_CHG_TYPE_DCP,
	MT6370_CHG_TYPE_CDP,
	MT6370_CHG_TYPE_MAX,
};

static enum power_supply_usb_type mt6370_pmu_chg_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

enum mt6370_usbsw_state {
	MT6370_USBSW_CHG = 0,
	MT6370_USBSW_USB,
};

struct mt6370_pmu_charger_desc {
	u32 ichg;
	u32 aicr;
	u32 mivr;
	u32 cv;
	u32 ieoc;
	u32 safety_timer;
	u32 ircmp_resistor;
	u32 ircmp_vclamp;
	u32 dc_wdt;
	u32 lbp_hys_sel;
	u32 lbp_dt;
	u32 bc12_sel;
	bool en_te;
	bool en_wdt;
	bool en_otg_wdt;
	bool en_polling;
	bool disable_vlgc;
	bool fast_unknown_ta_dect;
	bool post_aicl;
	const char *chg_dev_name;
	const char *ls_dev_name;
};

struct mt6370_pmu_charger_data {
	struct regmap *regmap;
	struct mt6370_pmu_charger_desc *chg_desc;
	struct mt6370_pmu_data *mpd;
	struct charger_device *chg_dev;
	struct charger_device *ls_dev;
	struct charger_properties chg_props;
	struct charger_properties ls_props;
	struct mutex adc_access_lock;
	struct mutex irq_access_lock;
	struct mutex aicr_access_lock;
	struct mutex ichg_access_lock;
	struct mutex pe_access_lock;
	struct mutex bc12_access_lock;
	struct mutex hidden_mode_lock;
	struct mutex ieoc_lock;
	struct mutex tchg_lock;
	struct mutex pp_lock;
	struct device *dev;
	wait_queue_head_t wait_queue;
	bool pwr_rdy;
	u8 irq_flag[MT6370_CHG_IRQIDX_MAX];
	int aicr_limit;
	u32 zcv;
	bool adc_hang;
	bool bc12_en;
	int psy_usb_type;
	u32 hidden_mode_cnt;
	u32 ieoc;
	u32 ichg;
	u32 ichg_dis_chg;
	u32 mivr;
	bool ieoc_wkard;
	bool dcd_timeout;
	atomic_t bc12_cnt;
	atomic_t bc12_wkard;
	int tchg;

	u32 bootmode;

#if IS_ENABLED(CONFIG_TCPC_CLASS)
	atomic_t tcpc_usb_connected;

	struct power_supply *chg_psy;
	struct tcpc_device *tcpc;
	struct notifier_block pd_nb;
	struct completion chrdet_start;
	struct task_struct *attach_task;
	struct mutex attach_lock;
	bool typec_attach;
	bool ignore_usb;
	bool bypass_chgdet;
#else
	struct work_struct chgdet_work;
#endif /* CONFIG_TCPC_CLASS */
	struct delayed_work mivr_dwork;
	struct power_supply_desc psy_desc;
	struct power_supply *psy;
	struct regulator_dev *otg_rdev;

	bool pp_en;
	union power_supply_propval *old_propval;
};

/* These default values will be used if there's no property in dts */
static struct mt6370_pmu_charger_desc mt6370_default_chg_desc = {
	.ichg = 2000000,		/* uA */
	.aicr = 500000,			/* uA */
	.mivr = 4400000,		/* uV */
	.cv = 4350000,			/* uA */
	.ieoc = 250000,			/* uA */
	.safety_timer = 12,		/* hour */
#if IS_ENABLED(CONFIG_MTK_BIF_SUPPORT)
	.ircmp_resistor = 0,		/* uohm */
	.ircmp_vclamp = 0,		/* uV */
#else
	.ircmp_resistor = 25000,	/* uohm */
	.ircmp_vclamp = 32000,		/* uV */
#endif
	.dc_wdt = 4000000,		/* us */
	.en_te = true,
	.en_wdt = true,
	.en_polling = false,
	.post_aicl = true,
	.chg_dev_name = "primary_chg",
	.ls_dev_name = "primary_load_switch",
};


static const u32 mt6370_otg_oc_threshold[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000, 3000000,
}; /* uA */

static const u32 mt6370_dc_wdt[] = {
	0, 125000, 250000, 500000, 1000000, 2000000, 4000000, 8000000,
}; /* us */

enum mt6370_charging_status {
	MT6370_CHG_STATUS_READY = 0,
	MT6370_CHG_STATUS_PROGRESS,
	MT6370_CHG_STATUS_DONE,
	MT6370_CHG_STATUS_FAULT,
	MT6370_CHG_STATUS_MAX,
};

/* Charging status name */
static const char *mt6370_chg_status_name[MT6370_CHG_STATUS_MAX] = {
	"ready", "progress", "done", "fault",
};

static const unsigned char mt6370_reg_en_hidden_mode[] = {
	MT6370_PMU_REG_HIDDENPASCODE1,
	MT6370_PMU_REG_HIDDENPASCODE2,
	MT6370_PMU_REG_HIDDENPASCODE3,
	MT6370_PMU_REG_HIDDENPASCODE4,
};

static const unsigned char mt6370_val_en_hidden_mode[] = {
	0x96, 0x69, 0xC3, 0x3C,
};

enum mt6370_iin_limit_sel {
	MT6370_IINLMTSEL_AICR_3250 = 0,
	MT6370_IINLMTSEL_CHG_TYPE,
	MT6370_IINLMTSEL_AICR,
	MT6370_IINLMTSEL_LOWER_LEVEL, /* lower of above three */
};

enum mt6370_adc_sel {
	MT6370_ADC_VBUS_DIV5 = 1,
	MT6370_ADC_VBUS_DIV2,
	MT6370_ADC_VSYS,
	MT6370_ADC_VBAT,
	MT6370_ADC_TS_BAT = 6,
	MT6370_ADC_IBUS = 8,
	MT6370_ADC_IBAT,
	MT6370_ADC_CHG_VDDP = 11,
	MT6370_ADC_TEMP_JC,
	MT6370_ADC_MAX,
};

/* Unit for each ADC parameter
 * 0 stands for reserved
 * For TS_BAT/TS_BUS, the real unit is 0.25.
 * Here we use 25, please remember to divide 100 while showing the value
 */
static const int mt6370_adc_unit[MT6370_ADC_MAX] = {
	0,
	MT6370_ADC_UNIT_VBUS_DIV5,
	MT6370_ADC_UNIT_VBUS_DIV2,
	MT6370_ADC_UNIT_VSYS,
	MT6370_ADC_UNIT_VBAT,
	0,
	MT6370_ADC_UNIT_TS_BAT,
	0,
	MT6370_ADC_UNIT_IBUS,
	MT6370_ADC_UNIT_IBAT,
	0,
	MT6370_ADC_UNIT_CHG_VDDP,
	MT6370_ADC_UNIT_TEMP_JC,
};

static const int mt6370_adc_offset[MT6370_ADC_MAX] = {
	0,
	MT6370_ADC_OFFSET_VBUS_DIV5,
	MT6370_ADC_OFFSET_VBUS_DIV2,
	MT6370_ADC_OFFSET_VSYS,
	MT6370_ADC_OFFSET_VBAT,
	0,
	MT6370_ADC_OFFSET_TS_BAT,
	0,
	MT6370_ADC_OFFSET_IBUS,
	MT6370_ADC_OFFSET_IBAT,
	0,
	MT6370_ADC_OFFSET_CHG_VDDP,
	MT6370_ADC_OFFSET_TEMP_JC,
};


/* =============================== */
/* mt6370 Charger Register Address */
/* =============================== */

static const unsigned char mt6370_chg_reg_addr[] = {
	MT6370_PMU_REG_CHGCTRL1,
	MT6370_PMU_REG_CHGCTRL2,
	MT6370_PMU_REG_CHGCTRL3,
	MT6370_PMU_REG_CHGCTRL4,
	MT6370_PMU_REG_CHGCTRL5,
	MT6370_PMU_REG_CHGCTRL6,
	MT6370_PMU_REG_CHGCTRL7,
	MT6370_PMU_REG_CHGCTRL8,
	MT6370_PMU_REG_CHGCTRL9,
	MT6370_PMU_REG_CHGCTRL10,
	MT6370_PMU_REG_CHGCTRL11,
	MT6370_PMU_REG_CHGCTRL12,
	MT6370_PMU_REG_CHGCTRL13,
	MT6370_PMU_REG_CHGCTRL14,
	MT6370_PMU_REG_CHGCTRL15,
	MT6370_PMU_REG_CHGCTRL16,
	MT6370_PMU_REG_CHGADC,
	MT6370_PMU_REG_DEVICETYPE,
	MT6370_PMU_REG_QCCTRL1,
	MT6370_PMU_REG_QCCTRL2,
	MT6370_PMU_REG_QC3P0CTRL1,
	MT6370_PMU_REG_QC3P0CTRL2,
	MT6370_PMU_REG_USBSTATUS1,
	MT6370_PMU_REG_QCSTATUS1,
	MT6370_PMU_REG_QCSTATUS2,
	MT6370_PMU_REG_CHGPUMP,
	MT6370_PMU_REG_CHGCTRL17,
	MT6370_PMU_REG_CHGCTRL18,
	MT6370_PMU_REG_CHGDIRCHG1,
	MT6370_PMU_REG_CHGDIRCHG2,
	MT6370_PMU_REG_CHGDIRCHG3,
	MT6370_PMU_REG_CHGSTAT,
	MT6370_PMU_REG_CHGNTC,
	MT6370_PMU_REG_ADCDATAH,
	MT6370_PMU_REG_ADCDATAL,
	MT6370_PMU_REG_CHGCTRL19,
	MT6370_PMU_REG_CHGSTAT1,
	MT6370_PMU_REG_CHGSTAT2,
	MT6370_PMU_REG_CHGSTAT3,
	MT6370_PMU_REG_CHGSTAT4,
	MT6370_PMU_REG_CHGSTAT5,
	MT6370_PMU_REG_CHGSTAT6,
	MT6370_PMU_REG_QCSTAT,
	MT6370_PMU_REG_DICHGSTAT,
	MT6370_PMU_REG_OVPCTRLSTAT,
};

/* ===================================================================== */
/* Internal Functions                                                    */
/* ===================================================================== */
static int mt6370_set_aicr(struct charger_device *chg_dev, u32 uA);
static int mt6370_get_aicr(struct charger_device *chg_dev, u32 *uA);
static int mt6370_set_ichg(struct charger_device *chg_dev, u32 uA);
static int mt6370_get_ichg(struct charger_device *chg_dev, u32 *uA);
static int mt6370_enable_charging(struct charger_device *chg_dev, bool en);

static inline void mt6370_chg_irq_set_flag(
	struct mt6370_pmu_charger_data *chg_data, u8 *irq, u8 mask)
{
	mutex_lock(&chg_data->irq_access_lock);
	*irq |= mask;
	mutex_unlock(&chg_data->irq_access_lock);
}

static inline void mt6370_chg_irq_clr_flag(
	struct mt6370_pmu_charger_data *chg_data, u8 *irq, u8 mask)
{
	mutex_lock(&chg_data->irq_access_lock);
	*irq &= ~mask;
	mutex_unlock(&chg_data->irq_access_lock);
}

static inline int mt6370_pmu_reg_test_bit(
	struct mt6370_pmu_charger_data *chg_data, u8 cmd, u8 shift, bool *is_one)
{
	int ret = 0;
	u8 data = 0;
	unsigned int regval = 0;

	ret = regmap_read(chg_data->regmap, cmd, &regval);
	if (ret < 0) {
		*is_one = false;
		return ret;
	}

	data = regval & (1 << shift);
	*is_one = (data == 0 ? false : true);

	return ret;
}

static u8 mt6370_find_closest_reg_value(u32 min, u32 max, u32 step, u32 num,
	u32 target)
{
	u32 i = 0, cur_val = 0, next_val = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target < min)
		return 0;

	for (i = 0; i < num - 1; i++) {
		cur_val = min + i * step;
		next_val = cur_val + step;

		if (cur_val > max)
			cur_val = max;

		if (next_val > max)
			next_val = max;

		if (target >= cur_val && target < next_val)
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return num - 1;
}

static u8 mt6370_find_closest_reg_value_via_table(const u32 *value_table,
	u32 table_size, u32 target_value)
{
	u32 i = 0;

	/* Smaller than minimum supported value, use minimum one */
	if (target_value < value_table[0])
		return 0;

	for (i = 0; i < table_size - 1; i++) {
		if (target_value >= value_table[i] &&
		    target_value < value_table[i + 1])
			return i;
	}

	/* Greater than maximum supported value, use maximum one */
	return table_size - 1;
}

static u32 mt6370_find_closest_real_value(u32 min, u32 max, u32 step,
	u8 reg_val)
{
	u32 ret_val = 0;

	ret_val = min + reg_val * step;
	if (ret_val > max)
		ret_val = max;

	return ret_val;
}

static inline void mt6370_enable_irq(struct mt6370_pmu_charger_data *chg_data,
	const char *name, bool en)
{
	struct resource *res = NULL;
	struct platform_device *pdev = to_platform_device(chg_data->dev);

	dev_info(chg_data->dev, "%s: (%s) en = %d", __func__, name, en);

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, name);
	if (res)
		(en ? enable_irq : disable_irq_nosync)(res->start);
	else
		dev_err(chg_data->dev, "%s: get plat res fail\n", __func__);
}

static int mt6370_set_fast_charge_timer(
	struct mt6370_pmu_charger_data *chg_data, u32 hour)
{
	int ret = 0;
	u8 reg_fct = 0;

	reg_fct = mt6370_find_closest_reg_value(
		MT6370_WT_FC_MIN,
		MT6370_WT_FC_MAX,
		MT6370_WT_FC_STEP,
		MT6370_WT_FC_NUM,
		hour
	);

	dev_info(chg_data->dev, "%s: timer = %d (0x%02X)\n", __func__, hour,
		reg_fct);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL12,
		MT6370_MASK_WT_FC,
		reg_fct << MT6370_SHIFT_WT_FC
	);

	return ret;
}

static int mt6370_enable_hidden_mode(struct mt6370_pmu_charger_data *chg_data,
	bool en)
{
	int ret = 0;

	mutex_lock(&chg_data->hidden_mode_lock);

	if (en) {
		if (chg_data->hidden_mode_cnt == 0) {
			ret = regmap_bulk_write(chg_data->regmap,
				mt6370_reg_en_hidden_mode[0],
				mt6370_val_en_hidden_mode,
				ARRAY_SIZE(mt6370_val_en_hidden_mode));
			if (ret < 0)
				goto err;
		}
		chg_data->hidden_mode_cnt++;
	} else {
		if (chg_data->hidden_mode_cnt == 1) /* last one */
			ret = regmap_write(chg_data->regmap,
				mt6370_reg_en_hidden_mode[0], 0x00);
		chg_data->hidden_mode_cnt--;
		if (ret < 0)
			goto err;
	}
	mt_dbg(chg_data->dev, "%s: en = %d\n", __func__, en);
	goto out;

err:
	dev_err(chg_data->dev, "%s: en = %d fail(%d)\n", __func__, en, ret);
out:
	mutex_unlock(&chg_data->hidden_mode_lock);
	return ret;
}

static int mt6370_get_adc(struct mt6370_pmu_charger_data *chg_data,
	enum mt6370_adc_sel adc_sel, int *adc_val)
{
	int ret = 0, i = 0;
	u8 adc_data[6] = {0};
	bool adc_start = false;
	u32 aicr = 0, ichg = 0;
	s64 adc_result = 0;
	const int max_wait_times = 6;

	unsigned int regval = 0;


	mutex_lock(&chg_data->adc_access_lock);

	mt6370_enable_hidden_mode(chg_data, true);

	/* Select ADC to desired channel */
	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGADC,
		MT6370_MASK_ADC_IN_SEL,
		adc_sel << MT6370_SHIFT_ADC_IN_SEL
	);


	if (ret < 0) {
		dev_err(chg_data->dev, "%s: select ch to %d failed, ret = %d\n",
			__func__, adc_sel, ret);
		goto out;
	}

	/* Workaround for IBUS & IBAT */
	if (adc_sel == MT6370_ADC_IBUS) {
		mutex_lock(&chg_data->aicr_access_lock);
		ret = mt6370_get_aicr(chg_data->chg_dev, &aicr);
		if (ret < 0) {
			dev_err(chg_data->dev, "%s: get aicr failed\n",
				__func__);
			goto out_unlock_all;
		}
	} else if (adc_sel == MT6370_ADC_IBAT) {
		mutex_lock(&chg_data->ichg_access_lock);
		ret = mt6370_get_ichg(chg_data->chg_dev, &ichg);
		if (ret < 0) {
			dev_err(chg_data->dev, "%s: get ichg failed\n",
				__func__);
			goto out_unlock_all;
		}
	}

	/* Start ADC conversation */
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGADC,
		MT6370_MASK_ADC_START, 0xff);
	if (ret < 0) {
		dev_err(chg_data->dev,
			"%s: start conversation failed, sel = %d, ret = %d\n",
			__func__, adc_sel, ret);
		goto out_unlock_all;
	}


	for (i = 0; i < max_wait_times; i++) {
		msleep(35);
		ret = mt6370_pmu_reg_test_bit(chg_data,
			MT6370_PMU_REG_CHGADC, MT6370_SHIFT_ADC_START,
			&adc_start);
		if (!adc_start && ret >= 0)
			break;
	}


	if (i == max_wait_times) {
		dev_err(chg_data->dev,
			"%s: wait conversation failed, sel = %d, ret = %d\n",
			__func__, adc_sel, ret);

		if (!chg_data->adc_hang) {
			for (i = 0; i < ARRAY_SIZE(mt6370_chg_reg_addr); i++) {
				ret = regmap_read(chg_data->regmap,
						mt6370_chg_reg_addr[i], &regval);

				dev_err(chg_data->dev,
					"%s: reg[0x%02X] = 0x%02X\n",
					__func__, mt6370_chg_reg_addr[i], regval);
			}

			chg_data->adc_hang = true;
		}

		/* Add for debug */
		/* ZCV, reg0x10 */
		ret = regmap_read(chg_data->regmap,
					MT6370_PMU_REG_OSCCTRL, &regval);
		if (ret < 0)
			dev_err(chg_data->dev, "%s: read reg0x10 failed\n",
				__func__);
		else
			dev_err(chg_data->dev, "%s: reg0x10 = 0x%02X\n",
				__func__, regval);

		/* TS auto sensing */
		ret = regmap_read(chg_data->regmap,
					MT6370_PMU_REG_CHGHIDDENCTRL15, &regval);
		if (ret < 0)
			dev_err(chg_data->dev, "%s: read reg0x3E failed\n",
				__func__);
		else
			dev_err(chg_data->dev, "%s: reg0x3E = 0x%02X\n",
				__func__, regval);

	}

	mdelay(1);


	/* Read ADC data */
	ret = regmap_bulk_read(chg_data->regmap, MT6370_PMU_REG_ADCDATAH,
				adc_data, 6);
	if (ret < 0) {
		dev_err(chg_data->dev,
			"%s: read ADC data failed, ret = %d\n", __func__, ret);
		goto out_unlock_all;
	}


	mt_dbg(chg_data->dev,
		"%s: adc_sel = %d, adc_h = 0x%02X, adc_l = 0x%02X\n",
		__func__, adc_sel, adc_data[0], adc_data[1]);

	mt_dbg(chg_data->dev,
		"%s: 0x4E~51 = (0x%02X, 0x%02X, 0x%02X, 0x%02X)\n", __func__,
		adc_data[2], adc_data[3], adc_data[4], adc_data[5]);

	/* Calculate ADC value */
	adc_result = ((s64)adc_data[0] * 256
		+ adc_data[1]) * mt6370_adc_unit[adc_sel]
		+ mt6370_adc_offset[adc_sel];

out_unlock_all:
	/* Coefficient of IBUS & IBAT */
#if defined(__LP64__) || defined(_LP64)
	if (adc_sel == MT6370_ADC_IBUS) {
		if (aicr < 400000) /* 400mA */
			adc_result = adc_result * 67 / 100;
		mutex_unlock(&chg_data->aicr_access_lock);
	} else if (adc_sel == MT6370_ADC_IBAT) {
		if (ichg >= 100000 && ichg <= 450000) /* 100~450mA */
			adc_result = adc_result * 475 / 1000;
		else if (ichg >= 500000 && ichg <= 850000) /* 500~850mA */
			adc_result = adc_result * 536 / 1000;
		mutex_unlock(&chg_data->ichg_access_lock);
	}
#else
	if (adc_sel == MT6370_ADC_IBUS) {
		if (aicr < 400000) /* 400mA */
			adc_result = div_s64(adc_result * 67, 100);
		mutex_unlock(&chg_data->aicr_access_lock);
	} else if (adc_sel == MT6370_ADC_IBAT) {
		if (ichg >= 100000 && ichg <= 450000) /* 100~450mA */
			adc_result = div_s64(adc_result * 475, 1000);
		else if (ichg >= 500000 && ichg <= 850000) /* 500~850mA */
			adc_result = div_s64(adc_result * 536, 1000);
		mutex_unlock(&chg_data->ichg_access_lock);
	}
#endif
out:
	*adc_val = adc_result;
	mt6370_enable_hidden_mode(chg_data, false);
	mutex_unlock(&chg_data->adc_access_lock);


	return ret;
}

#if !IS_ENABLED(CONFIG_MT6370_DCDTOUT_SUPPORT)
static int __maybe_unused mt6370_enable_dcd_tout(
			      struct mt6370_pmu_charger_data *chg_data, bool en)
{
	dev_info(chg_data->dev, "%s en = %d\n", __func__, en);

	//return (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_DEVICETYPE,
	//	 MT6370_MASK_DCDTOUTEN);

	return regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_DEVICETYPE,
				MT6370_MASK_DCDTOUTEN, en ? 0xff : 0);
}

static int __maybe_unused mt6370_is_dcd_tout_enable(
			     struct mt6370_pmu_charger_data *chg_data, bool *en)
{
	int ret;
	unsigned int regval = 0;

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_DEVICETYPE, &regval);
	if (ret < 0) {
		*en = false;
		return ret;
	}
	*en = (regval & MT6370_MASK_DCDTOUTEN ? true : false);
	return 0;
}
#endif

static inline bool mt6370_is_meta_mode(
			struct mt6370_pmu_charger_data *chg_data);

	//static void power_supply_changed(
	//		struct mt6370_pmu_charger_data *chg_data);

static int DPDM_Switch_TO_CHG_upstream(struct mt6370_pmu_charger_data *chg_data,
						bool switch_to_chg)
{
	struct phy *phy;
	int mode = 0;
	int ret;

	mode = switch_to_chg ? PHY_MODE_BC11_SET : PHY_MODE_BC11_CLR;
	phy = phy_get(chg_data->dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_info(chg_data->dev, "phy_get fail\n");
		return -EINVAL;
	}

	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, mode);
	if (ret)
		dev_info(chg_data->dev, "phy_set_mode_ext fail\n");

	phy_put(chg_data->dev, phy);

	return 0;
}

static int mt6370_set_usbsw_state(struct mt6370_pmu_charger_data *chg_data,
	int state)
{
	dev_info(chg_data->dev, "%s: state = %d\n", __func__, state);

	if (state == MT6370_USBSW_CHG)
		DPDM_Switch_TO_CHG_upstream(chg_data, true);
	else
		DPDM_Switch_TO_CHG_upstream(chg_data, false);

	return 0;
}

static int __maybe_unused __mt6370_enable_chgdet_flow(
			      struct mt6370_pmu_charger_data *chg_data, bool en)
{
	int ret = 0;
	enum mt6370_usbsw_state usbsw =
		en ? MT6370_USBSW_CHG : MT6370_USBSW_USB;

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);
	mt6370_set_usbsw_state(chg_data, usbsw);
	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_DEVICETYPE,
	//	MT6370_MASK_USBCHGEN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_DEVICETYPE,
		MT6370_MASK_USBCHGEN, en ? 0xff : 0);
	if (ret >= 0)
		chg_data->bc12_en = en;
	return ret;
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

static int __maybe_unused mt6370_enable_chgdet_flow(
			      struct mt6370_pmu_charger_data *chg_data, bool en)
{
	int ret = 0;
#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
	int i;
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	int vbus = 0;
#endif /* !CONFIG_TCPC_CLASS */
	const int max_wait_cnt = 200;
#endif /* CONFIG_USB_MTK_HDRC */
#if !IS_ENABLED(CONFIG_MT6370_DCDTOUT_SUPPORT)
	bool dcd_en = false;
#endif /* CONFIG_MT6370_DCDTOUT_SUPPORT */

	if (en && mt6370_is_meta_mode(chg_data)) {
		/* Skip charger type detection to speed up meta boot.*/
		dev_notice(chg_data->dev, "force Standard USB Host in meta\n");
		chg_data->pwr_rdy = true;
		chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		power_supply_changed(chg_data->psy);
		return 0;
	}

	if (en) {
#if !IS_ENABLED(CONFIG_MT6370_DCDTOUT_SUPPORT)
		ret = mt6370_is_dcd_tout_enable(chg_data, &dcd_en);
		if (!dcd_en)
			msleep(180);
#endif /* CONFIG_MT6370_DCDTOUT_SUPPORT */
#if IS_ENABLED(CONFIG_USB_MTK_HDRC)
		/* Workaround for CDP port */
		for (i = 0; i < max_wait_cnt; i++) {
			if (is_usb_rdy(chg_data->dev))
				break;
			dev_info(chg_data->dev, "%s: CDP block\n", __func__);
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
			ret = mt6370_get_adc(chg_data, MT6370_ADC_VBUS_DIV5,
				&vbus);
			if (ret >= 0 && vbus < 4300000) {
				dev_info(chg_data->dev,
					"%s: plug out, vbus = %dmV\n",
					__func__, vbus / 1000);
				return 0;
			}
#else
			if (!atomic_read(&chg_data->tcpc_usb_connected)) {
				dev_info(chg_data->dev,
					 "%s: plug out\n", __func__);
				return 0;
			}
#endif /* !CONFIG_TCPC_CLASS */
			msleep(100);
		}
		if (i == max_wait_cnt)
			dev_err(chg_data->dev, "%s: CDP timeout\n", __func__);
		else
			dev_info(chg_data->dev, "%s: CDP free\n", __func__);
#endif /* CONFIG_USB_MTK_HDRC */
	}

	mutex_lock(&chg_data->bc12_access_lock);
	ret = __mt6370_enable_chgdet_flow(chg_data, en);
	mutex_unlock(&chg_data->bc12_access_lock);
	return ret;
}

static inline int mt6370_toggle_chgdet_flow(
	struct mt6370_pmu_charger_data *chg_data)
{
	struct mt6370_pmu_data *mpd = chg_data->mpd;
	int ret = 0;
	u8 data = 0;
	/* read data */
	ret = i2c_smbus_read_i2c_block_data(mpd->i2c[MT6370_SLAVE_PMU],
		MT6370_PMU_REG_DEVICETYPE, 1, &data);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: read usbd fail\n", __func__);
		goto out;
	}

	/* usbd off */
	data &= ~MT6370_MASK_USBCHGEN;
	ret = i2c_smbus_write_i2c_block_data(mpd->i2c[MT6370_SLAVE_PMU],
		MT6370_PMU_REG_DEVICETYPE, 1, &data);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: usbd off fail\n", __func__);
		goto out;
	}

	udelay(40);

	/* usbd on */
	data |= MT6370_MASK_USBCHGEN;
	ret = i2c_smbus_write_i2c_block_data(mpd->i2c[MT6370_SLAVE_PMU],
		MT6370_PMU_REG_DEVICETYPE, 1, &data);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: usbd on fail\n", __func__);
out:

	return ret;
}

static int mt6370_bc12_workaround(struct mt6370_pmu_charger_data *chg_data)
{
	int ret = 0;

	dev_info(chg_data->dev, "%s\n", __func__);

	mutex_lock(&chg_data->mpd->io_lock);

	ret = mt6370_toggle_chgdet_flow(chg_data);
	if (ret < 0)
		goto err;

	mdelay(10);

	ret = mt6370_toggle_chgdet_flow(chg_data);
	if (ret < 0)
		goto err;

	goto out;
err:
	dev_err(chg_data->dev, "%s: fail\n", __func__);
out:
	mutex_unlock(&chg_data->mpd->io_lock);
	return ret;
}

static int mt6370_enable_power_path(struct charger_device *chg_dev, bool en);
static int __mt6370_chgdet_handler(struct mt6370_pmu_charger_data *chg_data)
{
	int ret = 0;
	bool pwr_rdy = false;
	u8 usb_status = 0, chip_vid = chg_data->mpd->chip_vid;
	unsigned int regval = 0;

	dev_info(chg_data->dev, "%s\n", __func__);

#if IS_ENABLED(CONFIG_TCPC_CLASS)
	pwr_rdy = atomic_read(&chg_data->tcpc_usb_connected);
#else
	/* Check UVP_D_STAT & OTG mode */
	ret = mt6370_pmu_reg_test_bit(chg_data,
		MT6370_PMU_REG_OVPCTRLSTAT,
		MT6370_SHIFT_OVPCTRL_UVP_D_STAT, &pwr_rdy);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: read uvp_d_stat fail\n", __func__);
		return ret;
	}
	pwr_rdy = !pwr_rdy;
#endif
	if (chg_data->pwr_rdy == pwr_rdy &&
		atomic_read(&chg_data->bc12_wkard) == 0) {
		dev_info(chg_data->dev, "%s: pwr rdy(%d) is the same\n",
			__func__, pwr_rdy);
		if (!pwr_rdy)
			goto out;
		return 0;
	}
	chg_data->pwr_rdy = pwr_rdy;

	/* plug out */
	if (!pwr_rdy) {
		chg_data->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		atomic_set(&chg_data->bc12_cnt, 0);
		goto out;
	}
	atomic_inc(&chg_data->bc12_cnt);

	/* plug in */
	if (chg_data->dcd_timeout) {
		chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		chg_data->dcd_timeout = false;
		goto dcd_timeout;
	}

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_USBSTATUS1, &regval);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: read chg type fail\n", __func__);
		return ret;
	}
	usb_status = (regval & MT6370_MASK_USB_STATUS) >> MT6370_SHIFT_USB_STATUS;

	switch (usb_status) {
	case MT6370_CHG_TYPE_UNDER_GOING:
		dev_info(chg_data->dev, "%s: under going...\n", __func__);
		return ret;
	case MT6370_CHG_TYPE_SDP:
		chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case MT6370_CHG_TYPE_SDPNSTD:
		chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	case MT6370_CHG_TYPE_CDP:
		chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case MT6370_CHG_TYPE_DCP:
		chg_data->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	default:
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		chg_data->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}

	dev_info(chg_data->dev, "%s chg_type:%d, pwr_rdy:%d\n",
		__func__, chg_data->psy_usb_type, chg_data->pwr_rdy);

	/* BC12 workaround (NONSTD -> STD) */
	if (atomic_read(&chg_data->bc12_cnt) < 3 &&
		chg_data->psy_usb_type == POWER_SUPPLY_USB_TYPE_SDP &&
		(chip_vid == RT5081_VENDOR_ID ||
		 chip_vid == MT6370_VENDOR_ID)) {
		ret = mt6370_bc12_workaround(chg_data);
		/* Workaround success, wait for next event */
		if (ret >= 0) {
			atomic_set(&chg_data->bc12_wkard, 1);
			return ret;
		}
		goto out;
	}

#if IS_ENABLED(MT6370_APPLE_SAMSUNG_TA_SUPPORT)
	ret = mt6370_detect_apple_samsung_ta(chg_data);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: detect apple/samsung ta fail(%d)\n",
			__func__, ret);
#endif

out:
	atomic_set(&chg_data->bc12_wkard, 0);

dcd_timeout:
	mt6370_enable_power_path(chg_data->chg_dev,
		chg_data->psy_usb_type != POWER_SUPPLY_USB_TYPE_UNKNOWN);
	/* Turn off USB charger detection */
	if (chg_data->psy_usb_type != POWER_SUPPLY_USB_TYPE_DCP) {
		ret = __mt6370_enable_chgdet_flow(chg_data, false);
		if (ret < 0)
			dev_notice(chg_data->dev, "%s: disable chgdet fail\n",
				   __func__);
	}

	power_supply_changed(chg_data->psy);
	return ret;
}

static int mt6370_chgdet_handler(struct mt6370_pmu_charger_data *chg_data)
{
	int ret = 0;

	mutex_lock(&chg_data->bc12_access_lock);
	ret = __mt6370_chgdet_handler(chg_data);
	mutex_unlock(&chg_data->bc12_access_lock);
	return ret;
}

/* Select IINLMTSEL */
static int mt6370_select_input_current_limit(
	struct mt6370_pmu_charger_data *chg_data, enum mt6370_iin_limit_sel sel)
{
	int ret = 0;

	dev_info(chg_data->dev, "%s: select input current limit = %d\n",
		__func__, sel);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL2,
		MT6370_MASK_IINLMTSEL,
		sel << MT6370_SHIFT_IINLMTSEL
	);

	return ret;
}

/* Hardware pin current limit */
static int mt6370_enable_ilim(struct mt6370_pmu_charger_data *chg_data, bool en)
{
	int ret = 0;

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);

	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGCTRL3, MT6370_MASK_ILIM_EN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL3,
				MT6370_MASK_ILIM_EN, en ? 0xff : 0);

	return ret;
}

static int mt6370_chg_sw_workaround(struct mt6370_pmu_charger_data *chg_data)
{
	int ret = 0;
	u8 zcv_data[2] = {0};

	dev_info(chg_data->dev, "%s\n", __func__);

	mt6370_enable_hidden_mode(chg_data, true);

	/* Read ZCV data */
	ret = regmap_bulk_read(chg_data->regmap,
		MT6370_PMU_REG_ADCBATDATAH, zcv_data, 2);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: read zcv data failed\n", __func__);
	else {
		chg_data->zcv = 5000 * (zcv_data[0] * 256 + zcv_data[1]);

		dev_info(chg_data->dev, "%s: zcv = (0x%02X, 0x%02X, %dmV)\n",
			__func__, zcv_data[0], zcv_data[1],
			chg_data->zcv / 1000);
	}

	/* Trigger any ADC before disabling ZCV */
	ret = regmap_write(chg_data->regmap, MT6370_PMU_REG_CHGADC,
		0x11);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: trigger ADC failed\n", __func__);

	/* Disable ZCV */
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_OSCCTRL,
		0x04, 0);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable ZCV failed\n", __func__);

	/* Disable TS auto sensing */
	ret = regmap_update_bits(chg_data->regmap,
		MT6370_PMU_REG_CHGHIDDENCTRL15, 0x01, 0);

	/* Disable SEN_DCP for charging mode */
	ret = regmap_update_bits(chg_data->regmap,
		MT6370_PMU_REG_QCCTRL2, MT6370_MASK_EN_DCP, 0);

	mt6370_enable_hidden_mode(chg_data, false);

	return ret;
}

static int mt6370_enable_wdt(struct mt6370_pmu_charger_data *chg_data,
	bool en)
{
	int ret = 0;

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);
	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGCTRL13, MT6370_MASK_WDT_EN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL13,
				MT6370_MASK_WDT_EN, en ? 0xff : 0);

	return ret;
}

static int mt6370_is_charging_enable(struct mt6370_pmu_charger_data *chg_data,
	bool *en)
{
	int ret = 0;

	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_CHGCTRL2,
		MT6370_SHIFT_CHG_EN, en);

	return ret;
}

static int __mt6370_enable_te(struct mt6370_pmu_charger_data *chg_data, bool en)
{
	int ret = 0;

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);
	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGCTRL2, MT6370_MASK_TE_EN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL2,
				MT6370_MASK_TE_EN, en ? 0xff : 0);

	return ret;
}

static int mt6370_enable_pump_express(struct mt6370_pmu_charger_data *chg_data,
	bool en)
{
	int ret = 0, i = 0;
	const int max_wait_times = 5;
	bool pumpx_en = false;

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);

	ret = mt6370_set_aicr(chg_data->chg_dev, 800000);
	if (ret < 0)
		return ret;

	ret = mt6370_set_ichg(chg_data->chg_dev, 2000000);
	if (ret < 0)
		return ret;

	ret = mt6370_enable_charging(chg_data->chg_dev, true);
	if (ret < 0)
		return ret;

	mt6370_enable_hidden_mode(chg_data, true);

	ret = regmap_update_bits(chg_data->regmap,
		MT6370_PMU_REG_CHGHIDDENCTRL9, 0x80, 0xff);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable psk mode fail\n", __func__);

	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//(chg_data->mpd, MT6370_PMU_REG_CHGCTRL17, MT6370_MASK_PUMPX_EN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL17,
				MT6370_MASK_PUMPX_EN, en ? 0xff : 0);
	if (ret < 0)
		goto out;

	for (i = 0; i < max_wait_times; i++) {
		msleep(2500);
		ret = mt6370_pmu_reg_test_bit(chg_data,
			MT6370_PMU_REG_CHGCTRL17, MT6370_SHIFT_PUMPX_EN,
			&pumpx_en);
		if (!pumpx_en && ret >= 0)
			break;
	}
	if (i == max_wait_times) {
		dev_err(chg_data->dev, "%s: wait failed, ret = %d\n", __func__,
			ret);
		ret = -EIO;
		goto out;
	}
	ret = 0;
out:
	regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGHIDDENCTRL9,
		0x80, 0);
	mt6370_enable_hidden_mode(chg_data, false);
	return ret;
}

static int mt6370_get_ieoc(struct mt6370_pmu_charger_data *chg_data, u32 *ieoc)
{
	int ret = 0;
	u8 reg_ieoc = 0;
	unsigned int regval = 0;

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_CHGCTRL9, &regval);
	if (ret < 0)
		return ret;

	reg_ieoc = (regval & MT6370_MASK_IEOC) >> MT6370_SHIFT_IEOC;
	*ieoc = mt6370_find_closest_real_value(
		MT6370_IEOC_MIN,
		MT6370_IEOC_MAX,
		MT6370_IEOC_STEP,
		reg_ieoc
	);

	return ret;
}

static int __mt6370_get_mivr(struct mt6370_pmu_charger_data *chg_data,
	u32 *mivr)
{
	int ret = 0;
	u8 reg_mivr = 0;
	unsigned int regval = 0;

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_CHGCTRL6, &regval);
	if (ret < 0)
		return ret;

	reg_mivr = (regval & MT6370_MASK_MIVR) >> MT6370_SHIFT_MIVR;
	*mivr = mt6370_find_closest_real_value(
		MT6370_MIVR_MIN,
		MT6370_MIVR_MAX,
		MT6370_MIVR_STEP,
		reg_mivr
	);

	return ret;
}

static int __mt6370_set_ieoc(struct mt6370_pmu_charger_data *chg_data, u32 ieoc)
{
	int ret = 0;
	u8 reg_ieoc = 0;

	/* IEOC workaround */
	if (chg_data->ieoc_wkard)
		ieoc += 100000; /* 100mA */

	/* Find corresponding reg value */
	reg_ieoc = mt6370_find_closest_reg_value(
		MT6370_IEOC_MIN,
		MT6370_IEOC_MAX,
		MT6370_IEOC_STEP,
		MT6370_IEOC_NUM,
		ieoc
	);

	dev_info(chg_data->dev, "%s: ieoc = %d (0x%02X)\n", __func__, ieoc,
		reg_ieoc);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL9,
		MT6370_MASK_IEOC,
		reg_ieoc << MT6370_SHIFT_IEOC
	);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set ieoc fail\n", __func__);

	/* Store IEOC */
	ret = mt6370_get_ieoc(chg_data, &chg_data->ieoc);

	return ret;
}

static int mt6370_get_charging_status(struct mt6370_pmu_charger_data *chg_data,
	enum mt6370_charging_status *chg_stat)
{
	int ret = 0;
	unsigned int regval = 0;

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_CHGSTAT, &regval);
	if (ret < 0)
		return ret;

	*chg_stat = (regval & MT6370_MASK_CHG_STAT) >> MT6370_SHIFT_CHG_STAT;

	return ret;
}

static int mt6370_set_dc_wdt(struct mt6370_pmu_charger_data *chg_data, u32 us)
{
	int ret = 0;
	u8 reg_wdt = 0;

	reg_wdt = mt6370_find_closest_reg_value_via_table(
		mt6370_dc_wdt,
		ARRAY_SIZE(mt6370_dc_wdt),
		us
	);

	dev_info(chg_data->dev, "%s: wdt = %dms(0x%02X)\n", __func__, us / 1000,
		reg_wdt);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGDIRCHG2,
		MT6370_MASK_DC_WDT,
		reg_wdt << MT6370_SHIFT_DC_WDT
	);

	return ret;
}

static int mt6370_enable_jeita(struct mt6370_pmu_charger_data *chg_data,
	bool en)
{
	int ret = 0;

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);

	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//(chg_data->mpd, MT6370_PMU_REG_CHGCTRL16, MT6370_MASK_JEITA_EN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL16,
				MT6370_MASK_JEITA_EN, en ? 0xff : 0);

	return ret;
}

static int mt6370_set_aicl_vth(struct mt6370_pmu_charger_data *chg_data,
	u32 aicl_vth)
{
	int ret = 0;
	u8 reg_aicl_vth = 0;

	reg_aicl_vth = mt6370_find_closest_reg_value(
		MT6370_AICL_VTH_MIN,
		MT6370_AICL_VTH_MAX,
		MT6370_AICL_VTH_STEP,
		MT6370_AICL_VTH_NUM,
		aicl_vth
	);

	dev_info(chg_data->dev, "%s: vth = %d (0x%02X)\n", __func__, aicl_vth,
		reg_aicl_vth);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL14,
		MT6370_MASK_AICL_VTH,
		reg_aicl_vth << MT6370_SHIFT_AICL_VTH
	);

	if (ret < 0)
		dev_err(chg_data->dev, "%s: set aicl vth failed, ret = %d\n",
			__func__, ret);

	return ret;
}

static int __mt6370_set_mivr(struct mt6370_pmu_charger_data *chg_data, u32 uV)
{
	int ret = 0;
	u8 reg_mivr = 0;

	/* Find corresponding reg value */
	reg_mivr = mt6370_find_closest_reg_value(
		MT6370_MIVR_MIN,
		MT6370_MIVR_MAX,
		MT6370_MIVR_STEP,
		MT6370_MIVR_NUM,
		uV
	);

	dev_info(chg_data->dev, "%s: mivr = %d (0x%02X)\n", __func__, uV,
		reg_mivr);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL6,
		MT6370_MASK_MIVR,
		reg_mivr << MT6370_SHIFT_MIVR
	);

	return ret;
}

static int __mt6370_set_aicr(struct mt6370_pmu_charger_data *chg_data, u32 uA)
{
	int ret = 0;
	u8 reg_aicr = 0;

	/* Find corresponding reg value */
	reg_aicr = mt6370_find_closest_reg_value(
		MT6370_AICR_MIN,
		MT6370_AICR_MAX,
		MT6370_AICR_STEP,
		MT6370_AICR_NUM,
		uA
	);

	mt_dbg(chg_data->dev, "%s: aicr = %d (0x%02X)\n", __func__, uA,
		reg_aicr);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL3,
		MT6370_MASK_AICR,
		reg_aicr << MT6370_SHIFT_AICR
	);

	return ret;
}

static inline int mt6370_post_aicl_measure(struct charger_device *chg_dev,
					   u32 start, u32 stop, u32 step,
					   u32 *measure)
{
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);
	int cur, ret;
	unsigned int regval = 0;

	mt_dbg(chg_data->dev,
	       "%s: post_aicc = (%d, %d, %d)\n", __func__, start, stop, step);
	for (cur = start; cur < stop; cur += step) {
		/* set_aicr to cur */
		ret = __mt6370_set_aicr(chg_data, cur + step);
		if (ret < 0)
			return ret;
		usleep_range(150, 200);
		ret = regmap_read(chg_data->regmap,
					  MT6370_PMU_REG_CHGSTAT1, &regval);
		if (ret < 0)
			return ret;
		/* read mivr stat */
		if (regval & MT6370_MASK_CHG_MIVR)
			break;
	}
	if (cur > stop)
		cur = stop;
	*measure = cur;
	return 0;
}

static int __mt6370_run_aicl(struct mt6370_pmu_charger_data *chg_data)
{
	int ret = 0;
	u32 mivr = 0, aicl_vth = 0, aicr = 0;
	bool mivr_stat = false;

	mt_dbg(chg_data->dev, "%s\n", __func__);

	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_CHGSTAT1,
		MT6370_SHIFT_MIVR_STAT, &mivr_stat);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: read mivr stat failed\n", __func__);
		goto out;
	}

	if (!mivr_stat) {
		mt_dbg(chg_data->dev, "%s: mivr stat not act\n", __func__);
		goto out;
	}

	ret = __mt6370_get_mivr(chg_data, &mivr);
	if (ret < 0)
		goto out;

	/* Check if there's a suitable AICL_VTH */
	aicl_vth = mivr + 200000;
	if (aicl_vth > MT6370_AICL_VTH_MAX) {
		dev_info(chg_data->dev, "%s: no suitable VTH, vth = %d\n",
			__func__, aicl_vth);
		ret = -EINVAL;
		goto out;
	}

	ret = mt6370_set_aicl_vth(chg_data, aicl_vth);
	if (ret < 0)
		goto out;

	/* Clear AICL measurement IRQ */
	mt6370_chg_irq_clr_flag(chg_data,
		&chg_data->irq_flag[MT6370_CHG_IRQIDX_CHGIRQ5],
		MT6370_MASK_CHG_AICLMEASI);

	mutex_lock(&chg_data->pe_access_lock);
	mutex_lock(&chg_data->aicr_access_lock);

	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL14,
		MT6370_MASK_AICL_MEAS, 0xff);
	if (ret < 0)
		goto unlock_out;

	ret = wait_event_interruptible_timeout(chg_data->wait_queue,
		chg_data->irq_flag[MT6370_CHG_IRQIDX_CHGIRQ5] &
		MT6370_MASK_CHG_AICLMEASI,
		msecs_to_jiffies(2500));
	if (ret <= 0) {
		dev_err(chg_data->dev, "%s: wait AICL time out, ret = %d\n",
			__func__, ret);
		ret = -EIO;
		goto unlock_out;
	}

	ret = mt6370_get_aicr(chg_data->chg_dev, &aicr);
	if (ret < 0)
		goto unlock_out;

	if (chg_data->chg_desc->post_aicl == false)
		goto skip_post_aicl;

	dev_info(chg_data->dev, "%s: aicc pre val = %d\n", __func__, aicr);
	/* always start/end aicc_val/aicc_val+200mA */
	ret = mt6370_post_aicl_measure(chg_data->chg_dev, aicr,
				       aicr + 200000, 50000, &aicr);
	if (ret < 0)
		goto out;
	dev_info(chg_data->dev, "%s: aicc post val = %d\n", __func__, aicr);

skip_post_aicl:
	chg_data->aicr_limit = aicr;
	dev_info(chg_data->dev, "%s: OK, aicr upper bound = %dmA\n", __func__,
		aicr / 1000);

unlock_out:
	mutex_unlock(&chg_data->aicr_access_lock);
	mutex_unlock(&chg_data->pe_access_lock);
out:
	return ret;
}

#if !IS_ENABLED(CONFIG_TCPC_CLASS)
static void mt6370_chgdet_work_handler(struct work_struct *work)
{
	int ret = 0;
	bool uvp_d = false, otg_mode = false;
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)container_of(work,
		struct mt6370_pmu_charger_data, chgdet_work);

	/* Check UVP_D_STAT & OTG mode */
	ret = mt6370_pmu_reg_test_bit(chg_data,
		MT6370_PMU_REG_OVPCTRLSTAT, MT6370_SHIFT_OVPCTRL_UVP_D_STAT,
		&uvp_d);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: read uvp_d_stat fail\n", __func__);
		return;
	}

	/* power not good */
	if (uvp_d)
		return;

	/* power good */
	ret = mt6370_pmu_reg_test_bit(chg_data,
		MT6370_PMU_REG_CHGCTRL1, MT6370_SHIFT_OPA_MODE,
		&otg_mode);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: read otg mode fail\n", __func__);
		return;
	}

	/* In OTG mode skip this event */
	if (otg_mode) {
		dev_info(chg_data->dev, "%s: triggered by OTG\n", __func__);
		return;
	}

	/* Turn on USB charger detection */
	ret = mt6370_enable_chgdet_flow(chg_data, true);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: en bc12 fail\n", __func__);
}
#endif /* !CONFIG_TCPC_CLASS */

static int __mt6370_get_ichg(struct mt6370_pmu_charger_data *chg_data,
	u32 *ichg)
{
	int ret = 0;
	u8 reg_ichg = 0;
	unsigned int regval = 0;

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_CHGCTRL7, &regval);
	if (ret < 0)
		return ret;

	reg_ichg = (regval & MT6370_MASK_ICHG) >> MT6370_SHIFT_ICHG;
	*ichg = mt6370_find_closest_real_value(MT6370_ICHG_MIN, MT6370_ICHG_MAX,
		MT6370_ICHG_STEP, reg_ichg);

	return ret;
}

static inline int mt6370_ichg_workaround(
	struct mt6370_pmu_charger_data *chg_data, u32 uA)
{
	int ret = 0;

	/* Vsys short protection */
	mt6370_enable_hidden_mode(chg_data, true);

	if (chg_data->ichg >= 900000 && uA < 900000)
		ret = regmap_update_bits(chg_data->regmap,
			MT6370_PMU_REG_CHGHIDDENCTRL7, 0x60, 0x00);
	else if (uA >= 900000 && chg_data->ichg < 900000)
		ret = regmap_update_bits(chg_data->regmap,
			MT6370_PMU_REG_CHGHIDDENCTRL7, 0x60, 0x40);

	mt6370_enable_hidden_mode(chg_data, false);
	return ret;
}

static int __mt6370_set_ichg(struct mt6370_pmu_charger_data *chg_data, u32 uA)
{
	int ret = 0;
	u8 reg_ichg = 0;
	u8 chip_vid = chg_data->mpd->chip_vid;

	uA = (uA < 500000) ? 500000 : uA;

	if (chip_vid == RT5081_VENDOR_ID || chip_vid == MT6370_VENDOR_ID) {
		ret = mt6370_ichg_workaround(chg_data, uA);
		if (ret < 0)
			dev_info(chg_data->dev, "%s: workaround fail\n",
				 __func__);
	}

	/* Find corresponding reg value */
	reg_ichg = mt6370_find_closest_reg_value(
		MT6370_ICHG_MIN,
		MT6370_ICHG_MAX,
		MT6370_ICHG_STEP,
		MT6370_ICHG_NUM,
		uA
	);

	mt_dbg(chg_data->dev, "%s: ichg = %d (0x%02X)\n", __func__, uA,
		reg_ichg);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL7,
		MT6370_MASK_ICHG,
		reg_ichg << MT6370_SHIFT_ICHG
	);
	if (ret < 0)
		return ret;

	/* Store Ichg setting */
	__mt6370_get_ichg(chg_data, &chg_data->ichg);

	if (chip_vid != RT5081_VENDOR_ID && chip_vid != MT6370_VENDOR_ID)
		goto bypass_ieoc_workaround;
	/* Workaround to make IEOC accurate */
	if (uA < 900000 && !chg_data->ieoc_wkard) { /* 900mA */
		ret = __mt6370_set_ieoc(chg_data, chg_data->ieoc + 100000);
		chg_data->ieoc_wkard = true;
	} else if (uA >= 900000 && chg_data->ieoc_wkard) {
		chg_data->ieoc_wkard = false;
		ret = __mt6370_set_ieoc(chg_data, chg_data->ieoc - 100000);
	}

bypass_ieoc_workaround:
	return ret;
}

static int __mt6370_set_cv(struct mt6370_pmu_charger_data *chg_data, u32 uV)
{
	int ret = 0;
	u8 reg_cv = 0;

	reg_cv = mt6370_find_closest_reg_value(
		MT6370_BAT_VOREG_MIN,
		MT6370_BAT_VOREG_MAX,
		MT6370_BAT_VOREG_STEP,
		MT6370_BAT_VOREG_NUM,
		uV
	);

	dev_info(chg_data->dev, "%s: bat voreg = %d (0x%02X)\n", __func__, uV,
		reg_cv);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL4,
		MT6370_MASK_BAT_VOREG,
		reg_cv << MT6370_SHIFT_BAT_VOREG
	);

	return ret;
}

static int __mt6370_enable_safety_timer(
	struct mt6370_pmu_charger_data *chg_data,
	bool en)
{
	int ret = 0;

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);
	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGCTRL12, MT6370_MASK_TMR_EN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL12,
				MT6370_MASK_TMR_EN, en ? 0xff : 0);

	return ret;
}

static int mt6370_enable_hz(struct mt6370_pmu_charger_data *chg_data, bool en)
{
	int ret = 0;

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);
	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGCTRL1, MT6370_MASK_HZ_EN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL1,
				MT6370_MASK_HZ_EN, en ? 0xff : 0);

	return ret;
}

static int mt6370_set_ircmp_resistor(struct mt6370_pmu_charger_data *chg_data,
	u32 uohm)
{
	int ret = 0;
	u8 reg_resistor = 0;

	reg_resistor = mt6370_find_closest_reg_value(
		MT6370_IRCMP_RES_MIN,
		MT6370_IRCMP_RES_MAX,
		MT6370_IRCMP_RES_STEP,
		MT6370_IRCMP_RES_NUM,
		uohm
	);

	dev_info(chg_data->dev, "%s: resistor = %d (0x%02X)\n", __func__, uohm,
		reg_resistor);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL18,
		MT6370_MASK_IRCMP_RES,
		reg_resistor << MT6370_SHIFT_IRCMP_RES
	);

	return ret;
}

static int mt6370_set_ircmp_vclamp(struct mt6370_pmu_charger_data *chg_data,
	u32 uV)
{
	int ret = 0;
	u8 reg_vclamp = 0;

	reg_vclamp = mt6370_find_closest_reg_value(
		MT6370_IRCMP_VCLAMP_MIN,
		MT6370_IRCMP_VCLAMP_MAX,
		MT6370_IRCMP_VCLAMP_STEP,
		MT6370_IRCMP_VCLAMP_NUM,
		uV
	);

	dev_info(chg_data->dev, "%s: vclamp = %d (0x%02X)\n", __func__, uV,
		reg_vclamp);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL18,
		MT6370_MASK_IRCMP_VCLAMP,
		reg_vclamp << MT6370_SHIFT_IRCMP_VCLAMP
	);

	return ret;
}



/* =================== */
/* Released interfaces */
/* =================== */

static int mt6370_enable_charging(struct charger_device *chg_dev, bool en)
{
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u32 ichg_ramp_t = 0;

	mt_dbg(chg_data->dev, "%s: en = %d\n", __func__, en);

	/* Workaround for avoiding vsys overshoot when charge disable */
	mutex_lock(&chg_data->ichg_access_lock);
	if (!en) {
		if (chg_data->ichg <= 500000)
			goto out;
		chg_data->ichg_dis_chg = chg_data->ichg;
		ichg_ramp_t = (chg_data->ichg - 500000) / 50000 * 2;
		ret = regmap_update_bits(chg_data->regmap,
						 MT6370_PMU_REG_CHGCTRL7,
						 MT6370_MASK_ICHG,
						 0x04 << MT6370_SHIFT_ICHG);
		if (ret < 0) {
			dev_notice(chg_data->dev,
				   "%s: set ichg fail\n", __func__);
			goto out;
		}
		mdelay(ichg_ramp_t);
	} else {
		if (chg_data->ichg == chg_data->ichg_dis_chg) {
			ret = __mt6370_set_ichg(chg_data, chg_data->ichg);
			if (ret < 0)
				dev_notice(chg_data->dev,
					   "%s: set ichg fail\n", __func__);
		}
	}
out:
	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGCTRL2, MT6370_MASK_CHG_EN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL2,
				MT6370_MASK_CHG_EN, en ? 0xff : 0);
	if (ret < 0)
		dev_notice(chg_data->dev, "%s: fail, en = %d\n", __func__, en);
	mutex_unlock(&chg_data->ichg_access_lock);
	return ret;
}

static int mt6370_enable_safety_timer(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = __mt6370_enable_safety_timer(chg_data, en);

	return ret;
}

static int mt6370_enable_te(struct charger_device *chg_dev, bool en)
{
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);
	return __mt6370_enable_te(chg_data, en);
}

static int mt6370_reset_eoc_state(struct charger_device *chg_dev)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s\n", __func__);

	mt6370_enable_hidden_mode(chg_data, true);

	ret = regmap_update_bits(chg_data->regmap,
			MT6370_PMU_REG_CHGHIDDENCTRL0, 0x80, 0xff);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: set failed, ret = %d\n",
			__func__, ret);
		goto err;
	}

	udelay(100);
	ret = regmap_update_bits(chg_data->regmap,
			MT6370_PMU_REG_CHGHIDDENCTRL0, 0x80, 0);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: clear failed, ret = %d\n",
			__func__, ret);
		goto err;
	}

err:
	mt6370_enable_hidden_mode(chg_data, false);

	return ret;
}

static int mt6370_safety_check(struct charger_device *chg_dev, u32 polling_ieoc)
{
	int ret = 0;
	int adc_ibat = 0;
	static int counter;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = mt6370_get_adc(chg_data, MT6370_ADC_IBAT, &adc_ibat);
	if (ret < 0) {
		dev_info(chg_data->dev, "%s: get adc failed\n", __func__);
		return ret;
	}

	if (adc_ibat <= polling_ieoc)
		counter++;
	else
		counter = 0;

	/* If IBAT is less than polling_ieoc for 3 times, trigger EOC event */
	if (counter == 3) {
		dev_info(chg_data->dev, "%s: polling_ieoc = %d, ibat = %d\n",
			__func__, polling_ieoc, adc_ibat);
		charger_dev_notify(chg_data->chg_dev, CHARGER_DEV_NOTIFY_EOC);
		counter = 0;
	}

	return ret;
}

static int mt6370_is_safety_timer_enable(struct charger_device *chg_dev,
	bool *en)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_CHGCTRL12,
		MT6370_SHIFT_TMR_EN, en);

	return ret;
}

static int mt6370_enable_power_path(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&chg_data->pp_lock);

	dev_info(chg_data->dev, "%s: en = %d, pp_en = %d\n",
				__func__, en, chg_data->pp_en);
	if (en == chg_data->pp_en)
		goto out;

	//ret = (en ? mt6370_pmu_reg_clr_bit : mt6370_pmu_reg_set_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGCTRL1,
	//	 MT6370_MASK_FORCE_SLEEP);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL1,
				MT6370_MASK_FORCE_SLEEP, en ? 0 : 0xff);
	/*
	 * enable power path -> unmask mivr irq
	 * mask mivr irq -> disable power path
	 */
	if (!en)
		mt6370_enable_irq(chg_data, "chg_mivr", false);
	ret = __mt6370_set_mivr(chg_data, en ? chg_data->mivr :
					       MT6370_MIVR_MAX);
	if (en)
		mt6370_enable_irq(chg_data, "chg_mivr", true);
	chg_data->pp_en = en;
out:
	mutex_unlock(&chg_data->pp_lock);
	return ret;
}

static int mt6370_is_power_path_enable(struct charger_device *chg_dev, bool *en)
{
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&chg_data->pp_lock);
	*en = chg_data->pp_en;
	mutex_unlock(&chg_data->pp_lock);

	return 0;
}

static int mt6370_get_ichg(struct charger_device *chg_dev, u32 *ichg)
{
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	return __mt6370_get_ichg(chg_data, ichg);
}

static int mt6370_set_ichg(struct charger_device *chg_dev, u32 uA)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&chg_data->ichg_access_lock);
	mutex_lock(&chg_data->ieoc_lock);
	ret = __mt6370_set_ichg(chg_data, uA);
	mutex_unlock(&chg_data->ieoc_lock);
	mutex_unlock(&chg_data->ichg_access_lock);

	return ret;
}

static int mt6370_set_ieoc(struct charger_device *chg_dev, u32 uA)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&chg_data->ichg_access_lock);
	mutex_lock(&chg_data->ieoc_lock);
	ret = __mt6370_set_ieoc(chg_data, uA);
	mutex_unlock(&chg_data->ieoc_lock);
	mutex_unlock(&chg_data->ichg_access_lock);

	return ret;
}

static int mt6370_get_aicr(struct charger_device *chg_dev, u32 *aicr)
{
	int ret = 0;
	u8 reg_aicr = 0;
	unsigned int regval = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_CHGCTRL3, &regval);
	if (ret < 0)
		return ret;

	reg_aicr = (regval & MT6370_MASK_AICR) >> MT6370_SHIFT_AICR;
	*aicr = mt6370_find_closest_real_value(MT6370_AICR_MIN, MT6370_AICR_MAX,
		MT6370_AICR_STEP, reg_aicr);

	return ret;
}

static int mt6370_set_aicr(struct charger_device *chg_dev, u32 uA)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&chg_data->aicr_access_lock);
	ret = __mt6370_set_aicr(chg_data, uA);
	mutex_unlock(&chg_data->aicr_access_lock);

	return ret;
}

static int mt6370_get_mivr_state(struct charger_device *chg_dev, bool *in_loop)
{
	int ret = 0;
	unsigned int regval = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_CHGSTAT1, &regval);
	if (ret < 0)
		return ret;
	*in_loop = (regval & MT6370_MASK_MIVR_STAT) >> MT6370_SHIFT_MIVR_STAT;
	return 0;
}

static int mt6370_get_mivr(struct charger_device *chg_dev, u32 *mivr)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = __mt6370_get_mivr(chg_data, mivr);

	return ret;
}

static int mt6370_set_mivr(struct charger_device *chg_dev, u32 uV)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&chg_data->pp_lock);

	if (!chg_data->pp_en) {
		dev_err(chg_data->dev, "%s: power path is disabled\n",
			__func__);
		goto out;
	}

	ret = __mt6370_set_mivr(chg_data, uV);
out:
	if (ret >= 0)
		chg_data->mivr = uV;
	mutex_unlock(&chg_data->pp_lock);
	return ret;
}

static int mt6370_get_cv(struct charger_device *chg_dev, u32 *cv)
{
	int ret = 0;
	u8 reg_cv = 0;
	unsigned int regval = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_CHGCTRL4, &regval);
	if (ret < 0)
		return ret;

	reg_cv = (regval & MT6370_MASK_BAT_VOREG) >> MT6370_SHIFT_BAT_VOREG;

	*cv = mt6370_find_closest_real_value(
		MT6370_BAT_VOREG_MIN,
		MT6370_BAT_VOREG_MAX,
		MT6370_BAT_VOREG_STEP,
		reg_cv
	);

	return ret;
}

static int mt6370_set_cv(struct charger_device *chg_dev, u32 uV)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = __mt6370_set_cv(chg_data, uV);

	return ret;
}

static int mt6370_set_otg_current_limit(struct charger_device *chg_dev, u32 uA)
{
	int ret = 0;
	u8 reg_ilimit = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	/* Set higher OC threshold */
	for (reg_ilimit = 0;
	     reg_ilimit < ARRAY_SIZE(mt6370_otg_oc_threshold) - 1; reg_ilimit++)
		if (uA <= mt6370_otg_oc_threshold[reg_ilimit])
			break;

	dev_info(chg_data->dev, "%s: ilimit = %d (0x%02X)\n", __func__, uA,
		reg_ilimit);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL10,
		MT6370_MASK_BOOST_OC,
		reg_ilimit << MT6370_SHIFT_BOOST_OC
	);

	return ret;
}

static int mt6370_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	bool en_otg = false;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);
	u8 hidden_val = en ? 0x00 : 0x0F;
	u8 lg_slew_rate = en ? 0x7C : 0x73;
	unsigned int regval = 0;

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);

	mt6370_enable_hidden_mode(chg_data, true);

	/*
	 * Woraround :
	 * slow Low side mos Gate driver slew rate for decline VBUS noise
	 * reg[0x33] = 0x7C after entering OTG mode
	 * reg[0x33] = 0x73 after leaving OTG mode
	 */
	ret = regmap_write(chg_data->regmap, MT6370_PMU_REG_LG_CONTROL,
		lg_slew_rate);
	if (ret < 0) {
		dev_err(chg_data->dev,
			"%s: recover Low side mos Gate drive speed fail(%d)\n",
			__func__, ret);
		goto out;
	}

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_LG_CONTROL, &regval);
	if (ret < 0)
		dev_info(chg_data->dev, "%s: read reg0x33 failed\n", __func__);
	else
		dev_info(chg_data->dev, "%s: reg0x33 = 0x%02X\n", __func__,
			regval);

	/* Turn off USB charger detection/Enable WDT */
	if (en) {
		if (chg_data->chg_desc->en_otg_wdt) {
			ret = mt6370_enable_wdt(chg_data, true);
			if (ret < 0)
				dev_err(chg_data->dev, "%s: en wdt fail\n",
					__func__);
		}
	}

	/* Switch OPA mode to boost mode */
	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGCTRL1, MT6370_MASK_OPA_MODE);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL1,
				MT6370_MASK_OPA_MODE, en ? 0xff : 0);

	msleep(20);

	if (en) {
		ret = mt6370_pmu_reg_test_bit(chg_data,
			MT6370_PMU_REG_CHGCTRL1,
			MT6370_SHIFT_OPA_MODE, &en_otg);
		if (ret < 0 || !en_otg) {
			dev_err(chg_data->dev, "%s: fail(%d)\n", __func__, ret);
			goto err_en_otg;
		}
	}

	/*
	 * Woraround reg[0x35] = 0x00 after entering OTG mode
	 * reg[0x35] = 0x0F after leaving OTG mode
	 */
	ret = regmap_write(chg_data->regmap,
		MT6370_PMU_REG_CHGHIDDENCTRL6, hidden_val);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: workaroud failed, ret = %d\n",
			__func__, ret);

	/* Disable WDT */
	if (!en) {
		ret = mt6370_enable_wdt(chg_data, false);
		if (ret < 0)
			dev_err(chg_data->dev, "%s: disable wdt failed\n",
				__func__);
	}
	goto out;

err_en_otg:
	/* Disable OTG */
	regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL1,
		MT6370_MASK_OPA_MODE, 0);

	/* Disable WDT */
	ret = mt6370_enable_wdt(chg_data, false);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable wdt failed\n", __func__);

	/* Recover Low side mos Gate slew rate */
	ret = regmap_write(chg_data->regmap,
			MT6370_PMU_REG_LG_CONTROL, 0x73);
	if (ret < 0)
		dev_err(chg_data->dev,
			"%s: recover Low side mos Gate drive speed fail(%d)\n",
			__func__, ret);
	ret = -EIO;
out:
	mt6370_enable_hidden_mode(chg_data, false);
	return ret;
}

static int mt6370_enable_discharge(struct charger_device *chg_dev, bool en)
{
	int ret = 0, i = 0;
	const u32 check_dischg_max = 3;
	bool is_dischg = true;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);

	ret = mt6370_enable_hidden_mode(chg_data, true);
	if (ret < 0)
		goto out;

	/* Set bit2 of reg[0x31] to 1/0 to enable/disable discharging */
	//ret = (en ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGHIDDENCTRL1, 0x04);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGHIDDENCTRL1,
				0x04, en ? 0xff : 0);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: en = %d failed, ret = %d\n",
			__func__, en, ret);
		return ret;
	}

	if (!en) {
		for (i = 0; i < check_dischg_max; i++) {
			ret = mt6370_pmu_reg_test_bit(chg_data,
				MT6370_PMU_REG_CHGHIDDENCTRL1, 2, &is_dischg);
			if (!is_dischg)
				break;
			ret = regmap_update_bits(chg_data->regmap,
				MT6370_PMU_REG_CHGHIDDENCTRL1, 0x04, 0);
		}
		if (i == check_dischg_max)
			dev_err(chg_data->dev,
				"%s: disable discharg failed, ret = %d\n",
				__func__, ret);
	}

out:
	mt6370_enable_hidden_mode(chg_data, false);
	return ret;
}

static int mt6370_set_pep_current_pattern(struct charger_device *chg_dev,
	bool is_increase)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s: pe1.0 pump_up = %d\n", __func__,
		is_increase);

	mutex_lock(&chg_data->pe_access_lock);

	/* Set to PE1.0 */
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL17,
		MT6370_MASK_PUMPX_20_10, 0xff);

	/* Set Pump Up/Down */
	//ret = (is_increase ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_CHGCTRL17,
	//	MT6370_MASK_PUMPX_UP_DN);
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL17,
				MT6370_MASK_PUMPX_UP_DN, is_increase ? 0xff : 0);
	if (ret < 0)
		goto out;

	/* Enable PumpX */
	ret = mt6370_enable_pump_express(chg_data, true);

out:
	mutex_unlock(&chg_data->pe_access_lock);
	return ret;
}

static int mt6370_set_pep20_reset(struct charger_device *chg_dev)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	mutex_lock(&chg_data->pe_access_lock);
	/* disable skip mode */
	mt6370_enable_hidden_mode(chg_data, true);

	ret = regmap_update_bits(chg_data->regmap,
		MT6370_PMU_REG_CHGHIDDENCTRL9, 0x80, 0);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable psk mode fail\n", __func__);

	/* Select IINLMTSEL to use AICR */
	ret = mt6370_select_input_current_limit(chg_data,
		MT6370_IINLMTSEL_AICR);
	if (ret < 0)
		goto out;

	ret = mt6370_set_aicr(chg_dev, 100000);
	if (ret < 0)
		goto out;

	msleep(250);

	ret = mt6370_set_aicr(chg_dev, 700000);

out:
	regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGHIDDENCTRL9,
		0x80, 0xff);
	mt6370_enable_hidden_mode(chg_data, false);
	mutex_unlock(&chg_data->pe_access_lock);
	return ret;
}

static int mt6370_set_pep20_current_pattern(struct charger_device *chg_dev,
	u32 uV)
{
	int ret = 0;
	u8 reg_volt = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s: pep2.0  = %d\n", __func__, uV);

	mutex_lock(&chg_data->pe_access_lock);
	/* Set to PEP2.0 */
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL17,
		MT6370_MASK_PUMPX_20_10, 0xff);
	if (ret < 0)
		goto out;

	/* Find register value of target voltage */
	reg_volt = mt6370_find_closest_reg_value(
		MT6370_PEP20_VOLT_MIN,
		MT6370_PEP20_VOLT_MAX,
		MT6370_PEP20_VOLT_STEP,
		MT6370_PEP20_VOLT_NUM,
		uV
	);

	/* Set Voltage */
	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL17,
		MT6370_MASK_PUMPX_DEC,
		reg_volt << MT6370_SHIFT_PUMPX_DEC
	);
	if (ret < 0)
		goto out;

	/* Enable PumpX */
	ret = mt6370_enable_pump_express(chg_data, true);
	ret = (ret >= 0) ? 0 : ret;

out:
	mutex_unlock(&chg_data->pe_access_lock);
	return ret;
}

static int mt6370_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
	return 0;
}

static int mt6370_enable_cable_drop_comp(struct charger_device *chg_dev,
	bool en)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);

	mutex_lock(&chg_data->pe_access_lock);
	/* Set to PEP2.0 */
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL17,
		MT6370_MASK_PUMPX_20_10, 0xff);
	if (ret < 0)
		goto out;

	/* Set Voltage */
	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGCTRL17,
		MT6370_MASK_PUMPX_DEC,
		0x1F << MT6370_SHIFT_PUMPX_DEC
	);
	if (ret < 0)
		goto out;

	/* Enable PumpX */
	ret = mt6370_enable_pump_express(chg_data, true);

out:
	mutex_unlock(&chg_data->pe_access_lock);
	return ret;
}

static int mt6370_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	int ret = 0;
	unsigned int chg_stat = MT6370_CHG_STATUS_READY;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = mt6370_get_charging_status(chg_data, &chg_stat);
	if (ret < 0)
		return ret;

	/* Return is charging done or not */
	switch (chg_stat) {
	case MT6370_CHG_STATUS_READY:
	case MT6370_CHG_STATUS_PROGRESS:
	case MT6370_CHG_STATUS_FAULT:
		*done = false;
		break;
	case MT6370_CHG_STATUS_DONE:
		*done = true;
		break;
	default:
		*done = false;
		break;
	}

	return 0;
}

static int mt6370_kick_wdt(struct charger_device *chg_dev)
{
	/* Any I2C communication can kick watchdog timer */
	int ret = 0;
	enum mt6370_charging_status chg_status;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = mt6370_get_charging_status(chg_data, &chg_status);

	return ret;
}

static int mt6370_enable_direct_charge(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);

	if (en) {
		mt6370_enable_irq(chg_data, "chg_mivr", false);

		/* Enable bypass mode */
		ret = regmap_update_bits(chg_data->regmap,
			MT6370_PMU_REG_CHGCTRL2, MT6370_MASK_BYPASS_MODE, 0xff);
		if (ret < 0) {
			dev_err(chg_data->dev, "%s: en bypass mode failed\n",
				__func__);
			goto out;
		}

		/* VG_EN = 1 */
		ret = regmap_update_bits(chg_data->regmap,
			MT6370_PMU_REG_CHGPUMP, MT6370_MASK_VG_EN, 0xff);
		if (ret < 0) {
			dev_err(chg_data->dev, "%s: en VG_EN failed\n",
				__func__);
			goto disable_bypass;
		}

		return ret;
	}

	/* Disable direct charge */
	/* VG_EN = 0 */
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGPUMP,
		MT6370_MASK_VG_EN, 0);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable VG_EN failed\n", __func__);

disable_bypass:
	/* Disable bypass mode */
	ret = regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_CHGCTRL2,
		MT6370_MASK_BYPASS_MODE, 0);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable bypass mode failed\n",
			__func__);

out:
	mt6370_enable_irq(chg_data, "chg_mivr", true);
	return ret;
}

static int mt6370_set_dc_vbusov(struct charger_device *chg_dev, u32 uV)
{
	int ret = 0;
	u8 reg_vbusov = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	reg_vbusov = mt6370_find_closest_reg_value(
		MT6370_DC_VBUSOV_LVL_MIN,
		MT6370_DC_VBUSOV_LVL_MAX,
		MT6370_DC_VBUSOV_LVL_STEP,
		MT6370_DC_VBUSOV_LVL_NUM,
		uV
	);

	dev_info(chg_data->dev, "%s: vbusov = %d (0x%02X)\n", __func__, uV,
		reg_vbusov);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGDIRCHG3,
		MT6370_MASK_DC_VBUSOV_LVL,
		reg_vbusov << MT6370_SHIFT_DC_VBUSOV_LVL
	);

	return ret;
}

static int mt6370_set_dc_ibusoc(struct charger_device *chg_dev, u32 uA)
{
	int ret = 0;
	u8 reg_ibusoc = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	reg_ibusoc = mt6370_find_closest_reg_value(
		MT6370_DC_IBUSOC_LVL_MIN,
		MT6370_DC_IBUSOC_LVL_MAX,
		MT6370_DC_IBUSOC_LVL_STEP,
		MT6370_DC_IBUSOC_LVL_NUM,
		uA
	);

	dev_info(chg_data->dev, "%s: ibusoc = %d (0x%02X)\n", __func__, uA,
		reg_ibusoc);

	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_CHGDIRCHG1,
		MT6370_MASK_DC_IBUSOC_LVL,
		reg_ibusoc << MT6370_SHIFT_DC_IBUSOC_LVL
	);

	return ret;
}

static int mt6370_kick_dc_wdt(struct charger_device *chg_dev)
{
	/* Any I2C communication can reset watchdog timer */
	int ret = 0;
	enum mt6370_charging_status chg_status;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = mt6370_get_charging_status(chg_data, &chg_status);

	return ret;
}

static int mt6370_get_tchg(struct charger_device *chg_dev, int *tchg_min,
	int *tchg_max)
{
	int ret = 0, adc_temp = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);
	u32 retry_cnt = 3;

	/* Get value from ADC */
	ret = mt6370_get_adc(chg_data, MT6370_ADC_TEMP_JC, &adc_temp);
	if (ret < 0)
		return ret;

	/* Check unusual temperature */
	while (adc_temp >= 120 && retry_cnt > 0) {
		dev_err(chg_data->dev, "%s: [WARNING] t = %d\n",
			__func__, adc_temp);
		mt6370_get_adc(chg_data, MT6370_ADC_VBAT, &adc_temp);
		ret = mt6370_get_adc(chg_data, MT6370_ADC_TEMP_JC, &adc_temp);
		retry_cnt--;
	}
	if (ret < 0)
		return ret;

	mutex_lock(&chg_data->tchg_lock);
	if (adc_temp >= 120)
		adc_temp = chg_data->tchg;
	else
		chg_data->tchg = adc_temp;
	mutex_unlock(&chg_data->tchg_lock);

	*tchg_min = adc_temp;
	*tchg_max = adc_temp;

	dev_info(chg_data->dev, "%s: tchg = %d\n", __func__, adc_temp);

	return ret;
}

static int mt6370_get_ibus(struct charger_device *chg_dev, u32 *ibus)
{
	int ret = 0, adc_ibus = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	/* Get value from ADC */
	ret = mt6370_get_adc(chg_data, MT6370_ADC_IBUS, &adc_ibus);
	if (ret < 0)
		return ret;

	*ibus = adc_ibus;

	dev_info(chg_data->dev, "%s: ibus = %dmA\n", __func__, adc_ibus / 1000);
	return ret;
}

static int mt6370_get_vbus(struct charger_device *chg_dev, u32 *vbus)
{
	int ret = 0, adc_vbus = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	/* Get value from ADC */
	ret = mt6370_get_adc(chg_data, MT6370_ADC_VBUS_DIV5, &adc_vbus);
	if (ret < 0)
		return ret;

	*vbus = adc_vbus;

	dev_info(chg_data->dev, "%s: vbus = %dmA\n", __func__, adc_vbus / 1000);
	return ret;
}

static int mt6370_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s\n", __func__);

	/* Reset AICR limit */
	chg_data->aicr_limit = -1;

	/* Enable charger */
	ret = mt6370_enable_charging(chg_dev, true);
	if (ret < 0) {
		dev_notice(chg_data->dev, "%s: en chg failed\n", __func__);
		return ret;
	}

	/* Disable WDT */
	ret = mt6370_enable_wdt(chg_data, false);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable wdt failed\n", __func__);


	return ret;
}

static int mt6370_plug_in(struct charger_device *chg_dev)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s\n", __func__);

	/* Enable WDT */
	if (chg_data->chg_desc->en_wdt) {
		ret = mt6370_enable_wdt(chg_data, true);
		if (ret < 0)
			dev_err(chg_data->dev, "%s: en wdt failed\n", __func__);
	}

	/* Enable charger */
	ret = mt6370_enable_charging(chg_dev, true);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: en chg failed\n", __func__);
		return ret;
	}

	return ret;
}

static int mt6370_run_aicl(struct charger_device *chg_dev, u32 *uA)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = __mt6370_run_aicl(chg_data);
	if (ret >= 0)
		*uA = chg_data->aicr_limit;

	return ret;
}

static int mt6370_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 500000;
	return 0;
}

static int mt6370_get_min_aicr(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 100000;
	return 0;
}

static int mt6370_dump_register(struct charger_device *chg_dev)
{
	int i = 0, ret = 0;
	u32 ichg = 0, aicr = 0, mivr = 0, ieoc = 0, cv = 0;
	bool chg_en = 0;
	int adc_vsys = 0, adc_vbat = 0, adc_ibat = 0, adc_ibus = 0;
	int adc_vbus = 0;
	unsigned int regval = 0;
	unsigned int chg_stat = 0;
	enum mt6370_charging_status chg_status = MT6370_CHG_STATUS_READY;
	u8 chg_ctrl[2] = {0};
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	ret = mt6370_get_ichg(chg_dev, &ichg);
	ret = mt6370_get_aicr(chg_dev, &aicr);
	ret = mt6370_get_charging_status(chg_data, &chg_status);
	ret = mt6370_get_ieoc(chg_data, &ieoc);
	ret = mt6370_get_mivr(chg_dev, &mivr);
	ret = mt6370_get_cv(chg_dev, &cv);
	ret = mt6370_is_charging_enable(chg_data, &chg_en);
	ret = mt6370_get_adc(chg_data, MT6370_ADC_VSYS, &adc_vsys);
	ret = mt6370_get_adc(chg_data, MT6370_ADC_VBAT, &adc_vbat);
	ret = mt6370_get_adc(chg_data, MT6370_ADC_IBAT, &adc_ibat);
	ret = mt6370_get_adc(chg_data, MT6370_ADC_IBUS, &adc_ibus);
	ret = mt6370_get_adc(chg_data, MT6370_ADC_VBUS_DIV5, &adc_vbus);

	ret = regmap_read(chg_data->regmap, MT6370_PMU_REG_CHGSTAT1, &chg_stat);
	ret = regmap_bulk_read(chg_data->regmap, MT6370_PMU_REG_CHGCTRL1,
		chg_ctrl, 2);

	if (chg_status == MT6370_CHG_STATUS_FAULT) {
		for (i = 0; i < ARRAY_SIZE(mt6370_chg_reg_addr); i++) {
			ret = regmap_read(chg_data->regmap,
				mt6370_chg_reg_addr[i], &regval);
			if (ret < 0)
				return ret;

			dev_dbg(chg_data->dev, "%s: reg[0x%02X] = 0x%02X\n",
				__func__, mt6370_chg_reg_addr[i], regval);
		}
	}

	dev_info(chg_data->dev,
		"%s: ICHG = %dmA, AICR = %dmA, MIVR = %dmV, IEOC = %dmA, CV = %dmV\n",
		__func__, ichg / 1000, aicr / 1000, mivr / 1000,
		ieoc / 1000, cv / 1000);

	dev_info(chg_data->dev,
		"%s: VSYS = %dmV, VBAT = %dmV, IBAT = %dmA, IBUS = %dmA, VBUS = %dmV\n",
		__func__, adc_vsys / 1000, adc_vbat / 1000,
		adc_ibat / 1000, adc_ibus / 1000, adc_vbus / 1000);

	dev_info(chg_data->dev, "%s: CHG_EN = %d, CHG_STATUS = %s, CHG_STAT = 0x%02X\n",
		__func__, chg_en, mt6370_chg_status_name[chg_status], chg_stat);

	dev_info(chg_data->dev, "%s: CHG_CTRL1 = 0x%02X, CHG_CTRL2 = 0x%02X\n",
		__func__, chg_ctrl[0], chg_ctrl[1]);

	ret = 0;
	return ret;
}

static int mt6370_enable_chg_type_det(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s: en = %d\n", __func__, en);

	if (chg_data->chg_desc->bc12_sel != 0)
		return ret;

	atomic_set(&chg_data->tcpc_usb_connected, en);

	/* TypeC detach */
	if (!en) {
		ret = mt6370_chgdet_handler(chg_data);
		return ret;
	}

	/* TypeC attach */
	ret = mt6370_enable_chgdet_flow(chg_data, true);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: en bc12 fail(%d)\n", __func__, ret);
#endif /* CONFIG_TCPC_CLASS */
	return ret;
}

static int mt6370_get_zcv(struct charger_device *chg_dev, u32 *uV)
{
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	dev_info(chg_data->dev, "%s: zcv = %dmV\n", __func__,
		chg_data->zcv / 1000);
	*uV = chg_data->zcv;

	return 0;
}

static int mt6370_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	struct mt6370_pmu_charger_data *chg_data =
		dev_get_drvdata(&chg_dev->dev);

	if (!chg_data->psy) {
		dev_notice(chg_data->dev, "%s: cannot get psy\n", __func__);
		return -ENODEV;
	}

	switch (event) {
	case EVENT_FULL:
	case EVENT_RECHARGE:
	case EVENT_DISCHARGE:
		power_supply_changed(chg_data->psy);
		break;
	default:
		break;
	}
	return 0;
}

#if IS_ENABLED(MT6370_APPLE_SAMSUNG_TA_SUPPORT)
static int mt6370_detect_apple_samsung_ta(
	struct mt6370_pmu_charger_data *chg_data)
{
	int ret = 0;
	bool dcd_timeout = false;
	bool dp_0_9v = false, dp_1_5v = false, dp_2_3v = false, dm_2_3v = false;

	/* Only SDP/CDP/DCP could possibly be Apple/Samsung TA */
	if (chg_data->psy_usb_type != POWER_SUPPLY_USB_TYPE_SDP &&
	    chg_data->psy_usb_type != POWER_SUPPLY_USB_TYPE_CDP &&
	    chg_data->psy_usb_type != POWER_SUPPLY_USB_TYPE_DCP)
		return -EINVAL;

	if (chg_data->psy_usb_type == POWER_SUPPLY_USB_TYPE_SDP ||
	    chg_data->psy_usb_type == POWER_SUPPLY_USB_TYPE_CDP) {
		ret = mt6370_pmu_reg_test_bit(chg_data,
			MT6370_PMU_REG_QCSTAT, MT6370_SHIFT_DCDTI_STAT,
			&dcd_timeout);
		if (ret < 0) {
			dev_err(chg_data->dev, "%s: read dcd timeout failed\n",
				__func__);
			return ret;
		}

		if (!dcd_timeout) {
			dev_info(chg_data->dev, "%s: dcd is not timeout\n",
				__func__);
			return 0;
		}
	}

	/* Check DP > 0.9V */
	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_QCSTATUS2,
		0x0F,
		0x03
	);

	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_QCSTATUS2,
		4, &dp_0_9v);
	if (ret < 0)
		return ret;

	if (!dp_0_9v) {
		dev_info(chg_data->dev, "%s: DP < 0.9V\n", __func__);
		return ret;
	}

	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_QCSTATUS2,
		5, &dp_1_5v);
	if (ret < 0)
		return ret;

	/* Samsung charger */
	if (!dp_1_5v) {
		dev_info(chg_data->dev, "%s: 0.9V < DP < 1.5V\n", __func__);
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		return ret;
	}

	/* Check DP > 2.3 V */
	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_QCSTATUS2,
		0x0F,
		0x0B
	);
	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_QCSTATUS2,
		5, &dp_2_3v);
	if (ret < 0)
		return ret;

	/* Check DM > 2.3V */
	ret = regmap_update_bits(
		chg_data->regmap,
		MT6370_PMU_REG_QCSTATUS2,
		0x0F,
		0x0F
	);
	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_QCSTATUS2,
		5, &dm_2_3v);
	if (ret < 0)
		return ret;

	/* Apple charger */
	if (!dp_2_3v && !dm_2_3v) {
		dev_info(chg_data->dev, "%s: 1.5V < DP < 2.3V && DM < 2.3V\n",
			__func__);
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
	} else if (!dp_2_3v && dm_2_3v) {
		dev_info(chg_data->dev, "%s: 1.5V < DP < 2.3V && 2.3V < DM\n",
			__func__);
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
	} else if (dp_2_3v && !dm_2_3v) {
		dev_info(chg_data->dev, "%s: 2.3V < DP && DM < 2.3V\n",
			__func__);
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
	} else {
		dev_info(chg_data->dev, "%s: 2.3V < DP && 2.3V < DM\n",
			__func__);
		chg_data->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
	}

	return 0;
}
#endif

static int mt6370_toggle_cfo(struct mt6370_pmu_charger_data *chg_data)
{
	struct mt6370_pmu_data *mpd = chg_data->mpd;
	int ret = 0;
	u8 data = 0;

	mutex_lock(&chg_data->mpd->io_lock);

	/* check if strobe mode */
	ret = i2c_smbus_read_i2c_block_data(mpd->i2c[MT6370_SLAVE_PMU],
		MT6370_PMU_REG_FLEDEN, 1, &data);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: check strobe fail\n", __func__);
		goto out;
	}
	if (data/* & MT6370_STROBE_EN_MASK*/) {
		dev_err(chg_data->dev, "%s: in strobe mode\n", __func__);
		goto out;
	}

	/* read data */
	ret = i2c_smbus_read_i2c_block_data(mpd->i2c[MT6370_SLAVE_PMU],
		MT6370_PMU_REG_CHGCTRL2, 1, &data);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: read cfo fail\n", __func__);
		goto out;
	}

	/* cfo off */
	data &= ~MT6370_MASK_CFO_EN;
	ret = i2c_smbus_write_i2c_block_data(mpd->i2c[MT6370_SLAVE_PMU],
		MT6370_PMU_REG_CHGCTRL2, 1, &data);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: cfo off fail\n", __func__);
		goto out;
	}

	/* cfo on */
	data |= MT6370_MASK_CFO_EN;
	ret = i2c_smbus_write_i2c_block_data(mpd->i2c[MT6370_SLAVE_PMU],
		MT6370_PMU_REG_CHGCTRL2, 1, &data);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: cfo on fail\n", __func__);

out:
	mutex_unlock(&chg_data->mpd->io_lock);
	return ret;
}

static irqreturn_t mt6370_pmu_chg_treg_irq_handler(int irq, void *data)
{
	int ret = 0;
	bool treg_stat = false;
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_err(chg_data->dev, "%s\n", __func__);

	/* Read treg status */
	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_CHGSTAT1,
		MT6370_SHIFT_CHG_TREG, &treg_stat);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: read treg stat failed\n", __func__);
	else
		dev_err(chg_data->dev, "%s: treg stat = %d\n", __func__,
			treg_stat);

	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_aicr_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static void mt6370_pmu_chg_mivr_dwork_handler(struct work_struct *work)
{
	struct mt6370_pmu_charger_data *chg_data = container_of(work,
		struct mt6370_pmu_charger_data, mivr_dwork.work);

	mt6370_enable_irq(chg_data, "chg_mivr", true);
}

static irqreturn_t mt6370_pmu_chg_mivr_irq_handler(int irq, void *data)
{
	int ret = 0, ibus = 0;
	bool mivr_stat = false;
	struct mt6370_pmu_charger_data *chg_data = data;

	mt_dbg(chg_data->dev, "%s\n", __func__);
	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_CHGSTAT1,
		MT6370_SHIFT_MIVR_STAT, &mivr_stat);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: read mivr stat failed\n", __func__);
		goto out;
	}

	if (!mivr_stat) {
		mt_dbg(chg_data->dev, "%s: mivr stat not act\n", __func__);
		goto out;
	}

	ret = mt6370_get_adc(chg_data, MT6370_ADC_IBUS, &ibus);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: get ibus fail\n", __func__);
		goto out;
	}

	if (ibus < 100000) { /* 100mA */
		ret = mt6370_toggle_cfo(chg_data);
		if (ret < 0)
			dev_err(chg_data->dev, "%s: toggle cfo fail\n",
				__func__);
		goto out;
	}

out:
	mt6370_enable_irq(chg_data, "chg_mivr", false);
	schedule_delayed_work(&chg_data->mivr_dwork, msecs_to_jiffies(500));
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_pwr_rdy_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_vinovp_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_vsysuv_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_vsysov_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_vbatov_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_vbusov_irq_handler(int irq, void *data)
{
	int ret = 0;
	bool vbusov_stat = false;
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;
	struct chgdev_notify *noti = &(chg_data->chg_dev->noti);

	dev_err(chg_data->dev, "%s\n", __func__);
	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_CHGSTAT2,
		MT6370_SHIFT_CHG_VBUSOV_STAT, &vbusov_stat);
	if (ret < 0)
		return IRQ_HANDLED;

	noti->vbusov_stat = vbusov_stat;
	dev_info(chg_data->dev, "%s: stat = %d\n", __func__, vbusov_stat);

	charger_dev_notify(chg_data->chg_dev, CHARGER_DEV_NOTIFY_VBUS_OVP);

	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ts_bat_cold_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ts_bat_cool_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ts_bat_warm_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ts_bat_hot_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_tmri_irq_handler(int irq, void *data)
{
	int ret = 0;
	bool tmr_stat = false;
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_err(chg_data->dev, "%s\n", __func__);
	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_CHGSTAT4,
		MT6370_SHIFT_CHG_TMRI_STAT, &tmr_stat);
	if (ret < 0)
		return IRQ_HANDLED;

	dev_info(chg_data->dev, "%s: stat = %d\n", __func__, tmr_stat);
	if (!tmr_stat)
		return IRQ_HANDLED;

	charger_dev_notify(chg_data->chg_dev,
		CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT);

	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_batabsi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_adpbadi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_rvpi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_otpi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_aiclmeasi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	mt6370_chg_irq_set_flag(chg_data,
		&chg_data->irq_flag[MT6370_CHG_IRQIDX_CHGIRQ5],
		MT6370_MASK_CHG_AICLMEASI);

	wake_up_interruptible(&chg_data->wait_queue);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_ichgmeasi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chgdet_donei_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_wdtmri_irq_handler(int irq, void *data)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	ret = mt6370_kick_wdt(chg_data->chg_dev);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: kick wdt failed\n", __func__);

	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ssfinishi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);

	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_rechgi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	charger_dev_notify(chg_data->chg_dev, CHARGER_DEV_NOTIFY_RECHG);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_termi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chg_ieoci_irq_handler(int irq, void *data)
{
	int ret = 0;
	bool ieoc_stat = false;
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	ret = mt6370_pmu_reg_test_bit(chg_data, MT6370_PMU_REG_CHGSTAT5,
		MT6370_SHIFT_CHG_IEOCI_STAT, &ieoc_stat);
	if (ret < 0)
		return IRQ_HANDLED;

	dev_info(chg_data->dev, "%s: stat = %d\n", __func__, ieoc_stat);
	if (!ieoc_stat)
		return IRQ_HANDLED;

	charger_dev_notify(chg_data->chg_dev, CHARGER_DEV_NOTIFY_EOC);

	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_adc_donei_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_pumpx_donei_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_bst_batuvi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_bst_vbusovi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_bst_olpi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_attachi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);

	if (chg_data->chg_desc->bc12_sel != 0)
		return IRQ_HANDLED;

	/* Check bc12 enable flag */
	mutex_lock(&chg_data->bc12_access_lock);
	if (!chg_data->bc12_en) {
		dev_err(chg_data->dev, "%s: bc12 disabled, ignore irq\n",
			__func__);
		goto out;
	}
	__mt6370_chgdet_handler(chg_data);
out:
	mutex_unlock(&chg_data->bc12_access_lock);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_detachi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_qc30stpdone_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_qc_vbusdet_done_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_hvdcp_det_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_chgdeti_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_dcdti_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;
	int ret = 0;
	bool dcdt = false;

	dev_info(chg_data->dev, "%s\n", __func__);
	if (chg_data->chg_desc->fast_unknown_ta_dect &&
	    chg_data->chg_desc->bc12_sel == 0) {
		ret = mt6370_pmu_reg_test_bit(chg_data,
				MT6370_PMU_REG_USBSTATUS1,
				MT6370_SHIFT_DCDT, &dcdt);
		if (ret < 0 || !dcdt)
			return IRQ_HANDLED;
		dev_info(chg_data->dev, "%s: unknown TA Detected\n", __func__);
		mutex_lock(&chg_data->bc12_access_lock);
		chg_data->dcd_timeout = true;
		__mt6370_chgdet_handler(chg_data);
		mutex_unlock(&chg_data->bc12_access_lock);
	}
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_dirchg_vgoki_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_info(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_dirchg_wdtmri_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_dirchg_uci_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_dirchg_oci_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_dirchg_ovi_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ovpctrl_swon_evt_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ovpctrl_uvp_d_evt_irq_handler(int irq, void *data)
{
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	int ret = 0;
	bool uvp_d = false, otg_mode = false;
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_err(chg_data->dev, "%s\n", __func__);

	if (chg_data->chg_desc->bc12_sel != 0)
		goto out;

	/* Check UVP_D_STAT & OTG mode */
	ret = mt6370_pmu_reg_test_bit(chg_data,
		MT6370_PMU_REG_OVPCTRLSTAT, MT6370_SHIFT_OVPCTRL_UVP_D_STAT,
		&uvp_d);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: read uvp_d_stat fail\n", __func__);
		goto out;
	}

	/* power good */
	if (!uvp_d) {
		ret = mt6370_pmu_reg_test_bit(chg_data,
			MT6370_PMU_REG_CHGCTRL1, MT6370_SHIFT_OPA_MODE,
			&otg_mode);
		if (ret < 0) {
			dev_err(chg_data->dev, "%s: read otg mode fail\n",
				__func__);
			goto out;
		}

		/* In OTG mode skip this event */
		if (otg_mode) {
			dev_info(chg_data->dev, "%s: triggered by OTG\n",
				__func__);
			goto out;
		}

		/* Turn on USB charger detection */
		ret = mt6370_enable_chgdet_flow(chg_data, true);
		if (ret < 0)
			dev_err(chg_data->dev, "%s: en bc12 fail\n", __func__);

		goto out;
	}

	/* not power good */
	ret = mt6370_chgdet_handler(chg_data);

out:
#endif /* !CONFIG_TCPC_CLASS */

	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ovpctrl_uvp_evt_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ovpctrl_ovp_d_evt_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6370_pmu_ovpctrl_ovp_evt_irq_handler(int irq, void *data)
{
	struct mt6370_pmu_charger_data *chg_data =
		(struct mt6370_pmu_charger_data *)data;

	dev_notice(chg_data->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static struct mt6370_pmu_irq_desc mt6370_chg_irq_desc[] = {
	{ "chg_treg", mt6370_pmu_chg_treg_irq_handler },
	{ "chg_aicr", mt6370_pmu_chg_aicr_irq_handler },
	{ "chg_mivr", mt6370_pmu_chg_mivr_irq_handler },
	{ "pwr_rdy", mt6370_pmu_pwr_rdy_irq_handler },
	{ "chg_vinovp", mt6370_pmu_chg_vinovp_irq_handler },
	{ "chg_vsysuv", mt6370_pmu_chg_vsysuv_irq_handler },
	{ "chg_vsysov", mt6370_pmu_chg_vsysov_irq_handler },
	{ "chg_vbatov", mt6370_pmu_chg_vbatov_irq_handler },
	{ "chg_vbusov", mt6370_pmu_chg_vbusov_irq_handler },
	{ "ts_bat_cold", mt6370_pmu_ts_bat_cold_irq_handler },
	{ "ts_bat_cool", mt6370_pmu_ts_bat_cool_irq_handler },
	{ "ts_bat_warm", mt6370_pmu_ts_bat_warm_irq_handler },
	{ "ts_bat_hot", mt6370_pmu_ts_bat_hot_irq_handler },
	{ "chg_tmri", mt6370_pmu_chg_tmri_irq_handler },
	{ "chg_batabsi", mt6370_pmu_chg_batabsi_irq_handler },
	{ "chg_adpbadi", mt6370_pmu_chg_adpbadi_irq_handler },
	{ "chg_rvpi", mt6370_pmu_chg_rvpi_irq_handler },
	{ "otpi", mt6370_pmu_otpi_irq_handler },
	{ "chg_aiclmeasi", mt6370_pmu_chg_aiclmeasi_irq_handler },
	{ "chg_ichgmeasi", mt6370_pmu_chg_ichgmeasi_irq_handler },
	{ "chgdet_donei", mt6370_pmu_chgdet_donei_irq_handler },
	{ "chg_wdtmri", mt6370_pmu_chg_wdtmri_irq_handler },
	{ "ssfinishi", mt6370_pmu_ssfinishi_irq_handler },
	{ "chg_rechgi", mt6370_pmu_chg_rechgi_irq_handler },
	{ "chg_termi", mt6370_pmu_chg_termi_irq_handler },
	{ "chg_ieoci", mt6370_pmu_chg_ieoci_irq_handler },
	{ "adc_donei", mt6370_pmu_adc_donei_irq_handler },
	{ "pumpx_donei", mt6370_pmu_pumpx_donei_irq_handler },
	{ "bst_batuvi", mt6370_pmu_bst_batuvi_irq_handler },
	{ "bst_vbusovi", mt6370_pmu_bst_vbusovi_irq_handler },
	{ "bst_olpi", mt6370_pmu_bst_olpi_irq_handler },
	{ "attachi", mt6370_pmu_attachi_irq_handler },
	{ "detachi", mt6370_pmu_detachi_irq_handler },
	{ "qc30stpdone", mt6370_pmu_qc30stpdone_irq_handler },
	{ "qc_vbusdet_done", mt6370_pmu_qc_vbusdet_done_irq_handler },
	{ "hvdcp_det", mt6370_pmu_hvdcp_det_irq_handler },
	{ "chgdeti", mt6370_pmu_chgdeti_irq_handler },
	{ "dcdti", mt6370_pmu_dcdti_irq_handler },
	{ "dirchg_vgoki", mt6370_pmu_dirchg_vgoki_irq_handler },
	{ "dirchg_wdtmri", mt6370_pmu_dirchg_wdtmri_irq_handler },
	{ "dirchg_uci", mt6370_pmu_dirchg_uci_irq_handler },
	{ "dirchg_oci", mt6370_pmu_dirchg_oci_irq_handler },
	{ "dirchg_ovi", mt6370_pmu_dirchg_ovi_irq_handler },
	{ "ovpctrl_swon_evt", mt6370_pmu_ovpctrl_swon_evt_irq_handler },
	{ "ovpctrl_uvp_d_evt", mt6370_pmu_ovpctrl_uvp_d_evt_irq_handler },
	{ "ovpctrl_uvp_evt", mt6370_pmu_ovpctrl_uvp_evt_irq_handler },
	{ "ovpctrl_ovp_d_evt", mt6370_pmu_ovpctrl_ovp_d_evt_irq_handler },
	{ "ovpctrl_ovp_evt", mt6370_pmu_ovpctrl_ovp_evt_irq_handler },
};

static void mt6370_pmu_charger_irq_register(struct platform_device *pdev)
{
	struct resource *res;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(mt6370_chg_irq_desc); i++) {
		if (!mt6370_chg_irq_desc[i].name)
			continue;
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					mt6370_chg_irq_desc[i].name);
		if (!res)
			continue;
		ret = devm_request_threaded_irq(&pdev->dev, res->start, NULL,
					mt6370_chg_irq_desc[i].irq_handler,
					IRQF_TRIGGER_FALLING,
					mt6370_chg_irq_desc[i].name,
					platform_get_drvdata(pdev));
		if (ret < 0) {
			dev_err(&pdev->dev, "request %s irq fail\n", res->name);
			continue;
		}
		mt6370_chg_irq_desc[i].irq = res->start;
	}
}

static inline int mt_parse_dt(struct device *dev,
	struct mt6370_pmu_charger_data *chg_data)
{
	struct mt6370_pmu_charger_desc *chg_desc = NULL;
	struct device_node *np = dev->of_node;
	struct i2c_client *i2c = chg_data->mpd->i2c[MT6370_SLAVE_PMU];

	dev_info(chg_data->dev, "%s\n", __func__);
	chg_data->chg_desc = &mt6370_default_chg_desc;

	chg_desc = devm_kzalloc(&i2c->dev,
		sizeof(struct mt6370_pmu_charger_desc), GFP_KERNEL);
	if (!chg_desc)
		return -ENOMEM;
	memcpy(chg_desc, &mt6370_default_chg_desc,
		sizeof(struct mt6370_pmu_charger_desc));

	/* Alias name is in charger properties but not in desc */
	if (of_property_read_string(np, "chg_alias_name",
		&(chg_data->chg_props.alias_name)) < 0) {
		dev_err(chg_data->dev, "%s: no chg alias name\n", __func__);
		chg_data->chg_props.alias_name = "mt6370_chg";
	}

	if (of_property_read_string(np, "ls_alias_name",
		&(chg_data->ls_props.alias_name)) < 0) {
		dev_err(chg_data->dev, "%s: no ls alias name\n", __func__);
		chg_data->ls_props.alias_name = "mt6370_ls";
	}

	if (of_property_read_string(np, "charger_name",
		&(chg_desc->chg_dev_name)) < 0)
		dev_err(chg_data->dev, "%s: no charger name\n", __func__);

	if (of_property_read_string(np, "load_switch_name",
		&(chg_desc->ls_dev_name)) < 0)
		dev_err(chg_data->dev, "%s: no load switch name\n", __func__);

	if (of_property_read_u32(np, "ichg", &chg_desc->ichg) < 0)
		dev_err(chg_data->dev, "%s: no ichg\n", __func__);

	if (of_property_read_u32(np, "aicr", &chg_desc->aicr) < 0)
		dev_err(chg_data->dev, "%s: no aicr\n", __func__);

	if (of_property_read_u32(np, "mivr", &chg_desc->mivr) < 0)
		dev_err(chg_data->dev, "%s: no mivr\n", __func__);

	if (of_property_read_u32(np, "cv", &chg_desc->cv) < 0)
		dev_err(chg_data->dev, "%s: no cv\n", __func__);

	if (of_property_read_u32(np, "ieoc", &chg_desc->ieoc) < 0)
		dev_err(chg_data->dev, "%s: no ieoc\n", __func__);

	if (of_property_read_u32(np, "safety_timer",
		&chg_desc->safety_timer) < 0)
		dev_err(chg_data->dev, "%s: no safety timer\n", __func__);

	if (of_property_read_u32(np, "dc_wdt", &chg_desc->dc_wdt) < 0)
		dev_err(chg_data->dev, "%s: no dc wdt\n", __func__);

	if (of_property_read_u32(np, "ircmp_resistor",
		&chg_desc->ircmp_resistor) < 0)
		dev_err(chg_data->dev, "%s: no ircmp resistor\n", __func__);

	if (of_property_read_u32(np, "ircmp_vclamp",
		&chg_desc->ircmp_vclamp) < 0)
		dev_err(chg_data->dev, "%s: no ircmp vclamp\n", __func__);

	if (of_property_read_u32(np, "lbp_hys_sel", &chg_desc->lbp_hys_sel) < 0)
		dev_err(chg_data->dev, "%s: no lbp_hys_sel\n", __func__);

	if (of_property_read_u32(np, "lbp_dt", &chg_desc->lbp_dt) < 0)
		dev_err(chg_data->dev, "%s: no lbp_dt\n", __func__);

	if (of_property_read_u32(np, "bc12_sel", &chg_desc->bc12_sel) < 0)
		dev_notice(chg_data->dev, "%s: no bc12_sel\n", __func__);

	chg_desc->en_te = of_property_read_bool(np, "enable_te");
	chg_desc->en_wdt = of_property_read_bool(np, "enable_wdt");
	chg_desc->en_otg_wdt = of_property_read_bool(np, "enable_otg_wdt");
	chg_desc->en_polling = of_property_read_bool(np, "enable_polling");
	chg_desc->disable_vlgc = of_property_read_bool(np, "disable_vlgc");
	chg_desc->fast_unknown_ta_dect =
		of_property_read_bool(np, "fast_unknown_ta_dect");
	chg_desc->post_aicl = of_property_read_bool(np, "post_aicl");

	chg_data->chg_desc = chg_desc;

	return 0;
}

static int mt6370_set_otglbp(
	struct mt6370_pmu_charger_data *chg_data, u32 lbp_hys_sel, u32 lbp_dt)
{
	u8 reg_data = (lbp_hys_sel << MT6370_SHIFT_LBPHYS_SEL)
		| (lbp_dt << MT6370_SHIFT_LBP_DT);

	dev_info(chg_data->dev, "%s: otglbp(%d), dt(%d)\n",
		__func__, lbp_hys_sel, lbp_dt);

	return regmap_update_bits(
		chg_data->regmap, MT6370_PMU_REG_VDDASUPPLY,
		MT6370_MASK_LBP, reg_data);
}

static int mt6370_disable_vlgc(
		struct mt6370_pmu_charger_data *chg_data, bool dis)
{
	//return (dis ? mt6370_pmu_reg_set_bit : mt6370_pmu_reg_clr_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_QCSTATUS1,
	//	 MT6370_MASK_VLGC_DISABLE);
	return regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_QCSTATUS1,
				MT6370_MASK_VLGC_DISABLE, dis ? 0xff : 0);
}

static int mt6370_enable_fast_unknown_ta_dect(
		struct mt6370_pmu_charger_data *chg_data, bool en)
{
	//return (en ? mt6370_pmu_reg_clr_bit : mt6370_pmu_reg_set_bit)
	//	(chg_data->mpd, MT6370_PMU_REG_USBSTATUS1,
	//	 MT6370_MASK_FAST_UNKNOWN_TA_DECT);
	return regmap_update_bits(chg_data->regmap, MT6370_PMU_REG_USBSTATUS1,
				MT6370_MASK_FAST_UNKNOWN_TA_DECT, en ? 0xff : 0);
}

static inline bool mt6370_is_meta_mode(struct mt6370_pmu_charger_data *chg_data)
{
	return (chg_data->bootmode == 1 ||
		chg_data->bootmode == 5);
/*1:META_BOOT 5:ADVMETA_BOOT*/
}

static void mt6370_get_bootmode(struct mt6370_pmu_charger_data *chg_data)
{
	struct device_node *boot_node = NULL;
	const struct tag_bootmode *tag = NULL;

	boot_node = of_parse_phandle(chg_data->dev->of_node, "bootmode", 0);
	if (!boot_node) {
		dev_notice(chg_data->dev, "%s: get bootmode phandle fail\n",
					  __func__);
		return;
	}

	tag = of_get_property(boot_node, "atag,boot", NULL);
	if (!tag) {
		dev_notice(chg_data->dev, "%s: get property atag,boot fail\n",
					  __func__);
		return;
	}

	dev_info(chg_data->dev, "%s: size:%d tag:0x%X mode:%d type:%d\n",
				__func__, tag->size, tag->tag,
				tag->bootmode, tag->boottype);
	chg_data->bootmode = tag->bootmode;
}

static int mt6370_chg_init_setting(struct mt6370_pmu_charger_data *chg_data)
{
	int ret = 0;
	struct mt6370_pmu_charger_desc *chg_desc = chg_data->chg_desc;

	dev_info(chg_data->dev, "%s\n", __func__);

	mt6370_get_bootmode(chg_data);

	/* Select IINLMTSEL to use AICR */
	ret = mt6370_select_input_current_limit(chg_data,
		MT6370_IINLMTSEL_AICR);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: select iinlmtsel failed\n",
			__func__);

	mdelay(5);

	/* Disable hardware ILIM */
	ret = mt6370_enable_ilim(chg_data, false);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable ilim failed\n", __func__);

	ret = __mt6370_set_ichg(chg_data, chg_desc->ichg);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set ichg failed\n", __func__);

	if (mt6370_is_meta_mode(chg_data)) {
		ret = __mt6370_set_aicr(chg_data, 200000);
		dev_info(chg_data->dev, "%s: set aicr to 200mA in meta mode\n",
			__func__);
	} else
		ret = __mt6370_set_aicr(chg_data, chg_desc->aicr);

	if (ret < 0)
		dev_err(chg_data->dev, "%s: set aicr failed\n", __func__);

	ret = __mt6370_set_mivr(chg_data, chg_desc->mivr);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set mivr failed\n", __func__);
	chg_data->mivr = chg_desc->mivr;

	ret = __mt6370_set_cv(chg_data, chg_desc->cv);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set voreg failed\n", __func__);

	ret = __mt6370_set_ieoc(chg_data, chg_desc->ieoc);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set ieoc failed\n", __func__);

	ret = __mt6370_enable_te(chg_data, chg_desc->en_te);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set te failed\n", __func__);

	ret = mt6370_set_fast_charge_timer(chg_data, chg_desc->safety_timer);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set fast timer failed\n", __func__);

	ret = mt6370_set_dc_wdt(chg_data, chg_desc->dc_wdt);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set dc watch dog timer failed\n",
			__func__);

	ret = __mt6370_enable_safety_timer(chg_data, true);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: enable charger timer failed\n",
			__func__);

	/* Initially disable WDT to prevent 1mA power consumption */
	ret = mt6370_enable_wdt(chg_data, false);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable watchdog timer failed\n",
			__func__);

	/* Disable JEITA */
	ret = mt6370_enable_jeita(chg_data, false);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable jeita failed\n", __func__);

	/* Disable HZ */
	ret = mt6370_enable_hz(chg_data, false);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: disable hz failed\n", __func__);

	ret = mt6370_set_ircmp_resistor(chg_data, chg_desc->ircmp_resistor);
	if (ret < 0)
		dev_err(chg_data->dev,
			"%s: set IR compensation resistor failed\n", __func__);

	ret = mt6370_set_ircmp_vclamp(chg_data, chg_desc->ircmp_vclamp);
	if (ret < 0)
		dev_err(chg_data->dev,
			"%s: set IR compensation vclamp failed\n", __func__);

	/* Disable USB charger type detection first, no matter use it or not */
	ret = __mt6370_enable_chgdet_flow(chg_data, false);
	if (ret < 0)
		dev_err(chg_data->dev,
			"%s: disable usb chrdet failed\n", __func__);

	ret = mt6370_set_otglbp(
		chg_data, chg_desc->lbp_hys_sel, chg_desc->lbp_dt);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set otg lbp fail\n", __func__);

	ret = mt6370_disable_vlgc(chg_data, chg_desc->disable_vlgc);
	if (ret < 0)
		dev_err(chg_data->dev, "%s: set vlgc fail\n", __func__);

	ret = mt6370_enable_fast_unknown_ta_dect(
		chg_data, chg_desc->fast_unknown_ta_dect);
	if (ret < 0) {
		dev_err(chg_data->dev,
			"%s: set fast unknown ta dect fail\n", __func__);
	}

#if !IS_ENABLED(CONFIG_MT6370_DCDTOUT_SUPPORT)
	/* Disable DCD */
	ret = mt6370_enable_dcd_tout(chg_data, false);
	if (ret < 0)
		dev_notice(chg_data->dev, "%s disable dcd fail\n", __func__);
#endif

	return ret;
}


static struct charger_ops mt6370_chg_ops = {
	/* Normal charging */
	.plug_out = mt6370_plug_out,
	.plug_in = mt6370_plug_in,
	.dump_registers = mt6370_dump_register,
	.enable = mt6370_enable_charging,
	.get_charging_current = mt6370_get_ichg,
	.set_charging_current = mt6370_set_ichg,
	.get_input_current = mt6370_get_aicr,
	.set_input_current = mt6370_set_aicr,
	.get_constant_voltage = mt6370_get_cv,
	.set_constant_voltage = mt6370_set_cv,
	.kick_wdt = mt6370_kick_wdt,
	.set_mivr = mt6370_set_mivr,
	.get_mivr = mt6370_get_mivr,
	.get_mivr_state = mt6370_get_mivr_state,
	.is_charging_done = mt6370_is_charging_done,
	.get_zcv = mt6370_get_zcv,
	.run_aicl = mt6370_run_aicl,
	.set_eoc_current = mt6370_set_ieoc,
	.enable_termination = mt6370_enable_te,
	.reset_eoc_state = mt6370_reset_eoc_state,
	.safety_check = mt6370_safety_check,
	.get_min_charging_current = mt6370_get_min_ichg,
	.get_min_input_current = mt6370_get_min_aicr,

	/* Safety timer */
	.enable_safety_timer = mt6370_enable_safety_timer,
	.is_safety_timer_enabled = mt6370_is_safety_timer_enable,

	/* Power path */
	.enable_powerpath = mt6370_enable_power_path,
	.is_powerpath_enabled = mt6370_is_power_path_enable,

	/* Charger type detection */
	.enable_chg_type_det = mt6370_enable_chg_type_det,

	/* OTG */
	.enable_otg = mt6370_enable_otg,
	.set_boost_current_limit = mt6370_set_otg_current_limit,
	.enable_discharge = mt6370_enable_discharge,

	/* PE+/PE+20 */
	.send_ta_current_pattern = mt6370_set_pep_current_pattern,
	.set_pe20_efficiency_table = mt6370_set_pep20_efficiency_table,
	.send_ta20_current_pattern = mt6370_set_pep20_current_pattern,
	.reset_ta = mt6370_set_pep20_reset,
	.enable_cable_drop_comp = mt6370_enable_cable_drop_comp,

	/* ADC */
	.get_tchg_adc = mt6370_get_tchg,
	.get_ibus_adc = mt6370_get_ibus,
	.get_vbus_adc = mt6370_get_vbus,

	/* Event */
	.event = mt6370_do_event,
};


static struct charger_ops mt6370_ls_ops = {
	/* Direct charging */
	.enable_direct_charging = mt6370_enable_direct_charge,
	.set_direct_charging_vbusov = mt6370_set_dc_vbusov,
	.set_direct_charging_ibusoc = mt6370_set_dc_ibusoc,
	.kick_direct_charging_wdt = mt6370_kick_dc_wdt,
	.get_tchg_adc = mt6370_get_tchg,
	.get_ibus_adc = mt6370_get_ibus,
};

static ssize_t shipping_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mt6370_pmu_charger_data *chg_data = dev_get_drvdata(dev);
	int32_t tmp = 0;
	int ret = 0;

	if (kstrtoint(buf, 10, &tmp) < 0) {
		dev_notice(dev, "parsing number fail\n");
		return -EINVAL;
	}
	if (tmp != 5526789)
		return -EINVAL;
	mutex_lock(&chg_data->adc_access_lock);
	ret = regmap_write(chg_data->regmap,
				   MT6370_PMU_REG_RSTPASCODE1, 0xA9);
	if (ret < 0) {
		dev_notice(dev, "set passcode1 fail\n");
		return ret;
	}
	ret = regmap_write(chg_data->regmap,
				   MT6370_PMU_REG_RSTPASCODE2, 0x96);
	if (ret < 0) {
		dev_notice(dev, "set passcode2 fail\n");
		return ret;
	}
	/* reset all chg/fled/ldo/rgb/bl/db reg and logic */
	ret = regmap_write(chg_data->regmap,
				     MT6370_PMU_REG_CORECTRL2, 0x7F);
	if (ret < 0) {
		dev_notice(dev, "set reset bits fail\n");
		return ret;
	}
	/* disable chg auto sensing */
	mt6370_enable_hidden_mode(chg_data, true);
	ret = regmap_update_bits(chg_data->regmap,
		MT6370_PMU_REG_CHGHIDDENCTRL15, 0x01, 0);
	if (ret < 0) {
		dev_notice(dev, "set auto sensing disable\n");
		return ret;
	}
	mt6370_enable_hidden_mode(chg_data, false);
	mdelay(50);
	/* enter shipping mode */
	ret = regmap_update_bits(chg_data->regmap,
				     MT6370_PMU_REG_CHGCTRL2, 0x80, 0xff);
	if (ret < 0) {
		dev_notice(dev, "enter shipping mode\n");
		return ret;
	}
	return count;
}

static const DEVICE_ATTR_WO(shipping_mode);

/* ======================= */
/* MT6370 Power Supply Ops */
/* ======================= */

static int mt6370_pmu_chg_get_online(struct mt6370_pmu_charger_data *chg_data,
				     union power_supply_propval *val)
{
	bool online = false;

	int ret = 0;

	ret = mt6370_pmu_reg_test_bit(chg_data,
		MT6370_PMU_REG_OVPCTRLSTAT,
		MT6370_SHIFT_OVPCTRL_UVP_D_STAT, &online);
	if (ret < 0) {
		dev_notice(chg_data->dev,
			"%s: read uvp_d_stat fail\n", __func__);
		return ret;
	}
	online = !online;
	dev_info(chg_data->dev, "%s: online = %d\n", __func__, online);
	val->intval = online;

	return 0;
}

static int mt6370_pmu_chg_set_online(struct mt6370_pmu_charger_data *chg_data,
				     const union power_supply_propval *val)
{
	return mt6370_enable_chg_type_det(chg_data->chg_dev, val->intval);
}

static int mt6370_pmu_chg_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mt6370_pmu_charger_data *chg_data =
						  power_supply_get_drvdata(psy);
	enum mt6370_charging_status chg_stat = MT6370_CHG_STATUS_READY;
	int ret = 0;

	val->intval = 0;

	dev_dbg(chg_data->dev, "%s: prop = %d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = mt6370_pmu_chg_get_online(chg_data, val);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = chg_data->psy_desc.type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = chg_data->psy_usb_type;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = mt6370_pmu_chg_get_online(chg_data, val);
		if (!val->intval) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		ret = mt6370_get_charging_status(chg_data, &chg_stat);
		switch (chg_stat) {
		case MT6370_CHG_STATUS_READY:
		case MT6370_CHG_STATUS_FAULT:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case MT6370_CHG_STATUS_PROGRESS:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case MT6370_CHG_STATUS_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (chg_data->psy_desc.type == POWER_SUPPLY_TYPE_USB)
			val->intval = 500000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (chg_data->psy_desc.type == POWER_SUPPLY_TYPE_USB)
			val->intval = 5000000;
		break;
	default:
		ret = -ENODATA;
		break;
	}

	return ret;
}

static int mt6370_pmu_chg_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct mt6370_pmu_charger_data *chg_data =
						  power_supply_get_drvdata(psy);
	int ret = 0;

	dev_dbg(chg_data->dev, "%s: prop = %d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = mt6370_pmu_chg_set_online(chg_data, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mt6370_pmu_chg_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property mt6370_pmu_chg_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_AUTHENTIC,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static const struct power_supply_desc mt6370_pmu_chg_desc = {
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= mt6370_pmu_chg_properties,
	.num_properties		= ARRAY_SIZE(mt6370_pmu_chg_properties),
	.get_property		= mt6370_pmu_chg_get_property,
	.set_property		= mt6370_pmu_chg_set_property,
	.property_is_writeable	= mt6370_pmu_chg_property_is_writeable,
	.usb_types		= mt6370_pmu_chg_usb_types,
	.num_usb_types		= ARRAY_SIZE(mt6370_pmu_chg_usb_types),
};

static char *mt6370_pmu_chg_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};

//static void power_supply_changed(
//			struct mt6370_pmu_charger_data *chg_data)
//{
//	int ret = 0, i = 0;
//	union power_supply_propval propval[mt6370_pmu_chg_desc.num_properties];

//	memset(propval, 0, sizeof(propval));

//	for (i = 0; i < ARRAY_SIZE(propval); i++) {
//		ret = mt6370_pmu_chg_get_property(chg_data->psy,
//						  mt6370_pmu_chg_properties[i],
//						  &propval[i]);
//		if (ret < 0)
//			dev_notice(chg_data->dev,
//				   "%s: get prop fail(%d), i = %d\n",
//				   __func__, ret, i);
//	}

//	if (memcmp(chg_data->old_propval, propval, sizeof(propval))) {
//		memcpy(chg_data->old_propval, propval, sizeof(propval));
//		power_supply_changed(chg_data->psy);
//	}
//}

static int mt6370_boost_enable(struct regulator_dev *rdev)
{
	struct mt6370_pmu_charger_data *chg_data = rdev_get_drvdata(rdev);

	return mt6370_enable_otg(chg_data->chg_dev, true);
}

static int mt6370_boost_disable(struct regulator_dev *rdev)
{
	struct mt6370_pmu_charger_data *chg_data = rdev_get_drvdata(rdev);

	return mt6370_enable_otg(chg_data->chg_dev, false);
}

static int mt6370_boost_is_enabled(struct regulator_dev *rdev)
{
	struct mt6370_pmu_charger_data *chg_data = rdev_get_drvdata(rdev);
	const struct regulator_desc *desc = rdev->desc;
	int ret = 0;
	unsigned int regval = 0;

	ret = regmap_read(chg_data->regmap, desc->enable_reg, &regval);
	if (ret < 0)
		return ret;
	return regval & desc->enable_mask ? true : false;
}

static int mt6370_boost_set_voltage_sel(struct regulator_dev *rdev,
					unsigned int sel)
{
	struct mt6370_pmu_charger_data *chg_data = rdev_get_drvdata(rdev);
	const struct regulator_desc *desc = rdev->desc;
	int shift = ffs(desc->vsel_mask) - 1;

	return regmap_update_bits(chg_data->regmap, desc->vsel_reg,
					  desc->vsel_mask, sel << shift);
}

static int mt6370_boost_get_voltage_sel(struct regulator_dev *rdev)
{
	struct mt6370_pmu_charger_data *chg_data = rdev_get_drvdata(rdev);
	const struct regulator_desc *desc = rdev->desc;
	int shift = ffs(desc->vsel_mask) - 1, ret;
	unsigned int regval = 0;

	ret = regmap_read(chg_data->regmap, desc->vsel_reg, &regval);
	if (ret < 0)
		return ret;
	return (regval & desc->vsel_mask) >> shift;
}

static int mt6370_boost_set_current_limit(struct regulator_dev *rdev,
					  int min_uA, int max_uA)
{
	struct mt6370_pmu_charger_data *chg_data = rdev_get_drvdata(rdev);
	const struct regulator_desc *desc = rdev->desc;
	int i, shift = ffs(desc->csel_mask) - 1;

	for (i = 0; i < ARRAY_SIZE(mt6370_otg_oc_threshold); i++) {
		if (min_uA <= mt6370_otg_oc_threshold[i])
			break;
	}
	if (i == ARRAY_SIZE(mt6370_otg_oc_threshold) ||
		mt6370_otg_oc_threshold[i] > max_uA) {
		dev_notice(chg_data->dev,
			"%s: out of current range\n", __func__);
		return -EINVAL;
	}
	dev_info(chg_data->dev, "%s: select otg_oc = %d\n",
		 __func__, mt6370_otg_oc_threshold[i]);
	return regmap_update_bits(chg_data->regmap,
					  desc->csel_reg,
					  desc->csel_mask,
					  i << shift);
}

static int mt6370_boost_get_current_limit(struct regulator_dev *rdev)
{
	struct mt6370_pmu_charger_data *chg_data = rdev_get_drvdata(rdev);
	const struct regulator_desc *desc = rdev->desc;
	int shift = ffs(desc->csel_mask) - 1, ret;
	unsigned int regval = 0;

	ret = regmap_read(chg_data->regmap, desc->csel_reg, &regval);
	if (ret < 0)
		return ret;
	ret = (regval & desc->csel_mask) >> shift;
	if (ret >= ARRAY_SIZE(mt6370_otg_oc_threshold))
		return -EINVAL;
	return mt6370_otg_oc_threshold[ret];
}

static const struct regulator_ops mt6370_chg_otg_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = mt6370_boost_enable,
	.disable = mt6370_boost_disable,
	.is_enabled = mt6370_boost_is_enabled,
	.set_voltage_sel = mt6370_boost_set_voltage_sel,
	.get_voltage_sel = mt6370_boost_get_voltage_sel,
	.set_current_limit = mt6370_boost_set_current_limit,
	.get_current_limit = mt6370_boost_get_current_limit,
};

static const struct regulator_desc mt6370_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &mt6370_chg_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.min_uV = 4425000,
	.uV_step = 25000, /* 25mV per step */
	.n_voltages = 57, /* 4425mV to 5825mV */
	.vsel_reg = MT6370_PMU_REG_CHGCTRL5,
	.vsel_mask = MT6370_MASK_BOOST_VOREG,
	.enable_reg = MT6370_PMU_REG_CHGCTRL1,
	.enable_mask = MT6370_MASK_OPA_MODE,
	.csel_reg = MT6370_PMU_REG_CHGCTRL10,
	.csel_mask = MT6370_MASK_BOOST_OC,
};

static int mt6370_pmu_charger_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mt6370_pmu_charger_data *chg_data;
	bool use_dt = pdev->dev.of_node;
	struct power_supply_config charger_cfg = {};
	struct regulator_config config = { };

	pr_info("%s: (%s)\n", __func__, MT6370_PMU_CHARGER_DRV_VERSION);

	chg_data = devm_kzalloc(&pdev->dev, sizeof(*chg_data), GFP_KERNEL);
	if (!chg_data)
		return -ENOMEM;

	mutex_init(&chg_data->adc_access_lock);
	mutex_init(&chg_data->irq_access_lock);
	mutex_init(&chg_data->aicr_access_lock);
	mutex_init(&chg_data->ichg_access_lock);
	mutex_init(&chg_data->pe_access_lock);
	mutex_init(&chg_data->bc12_access_lock);
	mutex_init(&chg_data->hidden_mode_lock);
	mutex_init(&chg_data->ieoc_lock);
	mutex_init(&chg_data->tchg_lock);
	mutex_init(&chg_data->pp_lock);
	chg_data->mpd = dev_get_drvdata(pdev->dev.parent);
	chg_data->dev = &pdev->dev;
	chg_data->aicr_limit = -1;
	chg_data->adc_hang = false;
	chg_data->bc12_en = true;
	chg_data->hidden_mode_cnt = 0;
	chg_data->ieoc_wkard = false;
	chg_data->ieoc = 250000; /* register default value 250mA */
	chg_data->ichg = 2000000;
	chg_data->ichg_dis_chg = 2000000;
	atomic_set(&chg_data->bc12_cnt, 0);
	atomic_set(&chg_data->bc12_wkard, 0);
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	atomic_set(&chg_data->tcpc_usb_connected, 0);
	init_completion(&chg_data->chrdet_start);
	mutex_init(&chg_data->attach_lock);
#endif
	chg_data->pp_en = true;
	chg_data->old_propval =
		devm_kcalloc(chg_data->dev, mt6370_pmu_chg_desc.num_properties,
			     sizeof(*chg_data->old_propval), GFP_KERNEL);
	if (!chg_data->old_propval)
		return -ENOMEM;

	if (use_dt) {
		ret = mt_parse_dt(&pdev->dev, chg_data);
		if (ret < 0)
			dev_err(chg_data->dev, "%s: parse dts failed\n",
				__func__);
	}
	platform_set_drvdata(pdev, chg_data);

	/* get parent regmap*/
	chg_data->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chg_data->regmap) {
		dev_err(chg_data->dev, "Fail to get parent regmap\n");
		ret = -ENODEV;
		goto err_chg_init_setting;
	}

	/* Init wait queue head */
	init_waitqueue_head(&chg_data->wait_queue);

#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	if (chg_data->chg_desc->bc12_sel == 0)
		INIT_WORK(&chg_data->chgdet_work, mt6370_chgdet_work_handler);
#endif /* !CONFIG_TCPC_CLASS */
	INIT_DELAYED_WORK(&chg_data->mivr_dwork,
			  mt6370_pmu_chg_mivr_dwork_handler);

	/* Do initial setting */
	ret = mt6370_chg_init_setting(chg_data);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: sw init failed\n", __func__);
		goto err_chg_init_setting;
	}

	/* SW workaround */
	ret = mt6370_chg_sw_workaround(chg_data);
	if (ret < 0) {
		dev_err(chg_data->dev, "%s: software workaround failed\n",
			__func__);
		goto err_chg_sw_workaround;
	}

	/* Register charger device */
	chg_data->chg_dev = charger_device_register(
		chg_data->chg_desc->chg_dev_name,
		chg_data->dev, chg_data, &mt6370_chg_ops, &chg_data->chg_props);
	if (IS_ERR_OR_NULL(chg_data->chg_dev)) {
		ret = PTR_ERR(chg_data->chg_dev);
		goto err_register_chg_dev;
	}
	chg_data->chg_dev->is_polling_mode = chg_data->chg_desc->en_polling;

	/* Register load switch charger device */
	chg_data->ls_dev = charger_device_register(
		chg_data->chg_desc->ls_dev_name,
		chg_data->dev, chg_data, &mt6370_ls_ops, &chg_data->ls_props);
	if (IS_ERR_OR_NULL(chg_data->ls_dev)) {
		ret = PTR_ERR(chg_data->ls_dev);
		goto err_register_ls_dev;
	}
	chg_data->ls_dev->is_polling_mode = chg_data->chg_desc->en_polling;

	mt6370_pmu_charger_irq_register(pdev);

	ret = device_create_file(chg_data->dev, &dev_attr_shipping_mode);
	if (ret < 0) {
		dev_notice(&pdev->dev, "create shipping attr fail\n");
		goto err_devfs;
	}

	/* power supply register */
	memcpy(&chg_data->psy_desc,
		&mt6370_pmu_chg_desc, sizeof(chg_data->psy_desc));
	chg_data->psy_desc.name = dev_name(&pdev->dev);

	charger_cfg.drv_data = chg_data;
	charger_cfg.of_node = pdev->dev.of_node;
	charger_cfg.supplied_to = mt6370_pmu_chg_supplied_to;
	charger_cfg.num_supplicants = ARRAY_SIZE(mt6370_pmu_chg_supplied_to);
	chg_data->psy = devm_power_supply_register(&pdev->dev,
					&chg_data->psy_desc, &charger_cfg);
	if (IS_ERR(chg_data->psy)) {
		dev_notice(&pdev->dev, "Fail to register power supply dev\n");
		ret = PTR_ERR(chg_data->psy);
		goto err_register_psy;
	}

	/* otg regulator */
	config.dev = &pdev->dev;
	config.driver_data = chg_data;
	chg_data->otg_rdev = devm_regulator_register(&pdev->dev,
						&mt6370_otg_rdesc, &config);
	if (IS_ERR(chg_data->otg_rdev)) {
		ret = PTR_ERR(chg_data->otg_rdev);
		goto err_register_otg;
	}


	/* Schedule work for microB's BC1.2 */
#if !IS_ENABLED(CONFIG_TCPC_CLASS)
	if (chg_data->chg_desc->bc12_sel == 0)
		schedule_work(&chg_data->chgdet_work);
#endif /* !CONFIG_TCPC_CLASS */

	dev_info(&pdev->dev, "%s successfully\n", __func__);
	return 0;

err_register_otg:
err_register_psy:
	device_remove_file(chg_data->dev, &dev_attr_shipping_mode);
err_devfs:
	charger_device_unregister(chg_data->ls_dev);
err_register_ls_dev:
	charger_device_unregister(chg_data->chg_dev);
err_register_chg_dev:
err_chg_sw_workaround:
err_chg_init_setting:
	mutex_destroy(&chg_data->ichg_access_lock);
	mutex_destroy(&chg_data->adc_access_lock);
	mutex_destroy(&chg_data->irq_access_lock);
	mutex_destroy(&chg_data->aicr_access_lock);
	mutex_destroy(&chg_data->pe_access_lock);
	mutex_destroy(&chg_data->bc12_access_lock);
	mutex_destroy(&chg_data->hidden_mode_lock);
	mutex_destroy(&chg_data->ieoc_lock);
	mutex_destroy(&chg_data->tchg_lock);
	mutex_destroy(&chg_data->pp_lock);
	return ret;
}

static int mt6370_pmu_charger_remove(struct platform_device *pdev)
{
	struct mt6370_pmu_charger_data *chg_data = platform_get_drvdata(pdev);

	if (chg_data) {
		device_remove_file(chg_data->dev, &dev_attr_shipping_mode);
		charger_device_unregister(chg_data->ls_dev);
		charger_device_unregister(chg_data->chg_dev);
		mutex_destroy(&chg_data->ichg_access_lock);
		mutex_destroy(&chg_data->adc_access_lock);
		mutex_destroy(&chg_data->irq_access_lock);
		mutex_destroy(&chg_data->aicr_access_lock);
		mutex_destroy(&chg_data->pe_access_lock);
		mutex_destroy(&chg_data->bc12_access_lock);
		mutex_destroy(&chg_data->hidden_mode_lock);
		mutex_destroy(&chg_data->ieoc_lock);
		mutex_destroy(&chg_data->tchg_lock);
		mutex_destroy(&chg_data->pp_lock);
		dev_info(chg_data->dev, "%s successfully\n", __func__);
	}

	return 0;
}

static const struct of_device_id mt_ofid_table[] = {
	{ .compatible = "mediatek,mt6370_pmu_charger", },
	{ },
};
MODULE_DEVICE_TABLE(of, mt_ofid_table);

static const struct platform_device_id mt_id_table[] = {
	{ "mt6370_pmu_charger", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, mt_id_table);

static struct platform_driver mt6370_pmu_charger = {
	.driver = {
		.name = "mt6370_pmu_charger",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mt_ofid_table),
	},
	.probe = mt6370_pmu_charger_probe,
	.remove = mt6370_pmu_charger_remove,
	.id_table = mt_id_table,
};
static int __init mt6370_pmu_charger_init(void)
{
	return platform_driver_register(&mt6370_pmu_charger);
}

static void __exit mt6370_pmu_charger_exit(void)
{
	platform_driver_unregister(&mt6370_pmu_charger);
}
device_initcall_sync(mt6370_pmu_charger_init);
module_exit(mt6370_pmu_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek MT6370 PMU Charger");
MODULE_VERSION(MT6370_PMU_CHARGER_DRV_VERSION);

/*
 * Release Note
 * 1.1.30_MTK
 * (1) Fix no charging icon when power role swap to sink on kernel-4.19
 * (2) Reduce IBUS Iq when pp is off for MT6371 and MT6372
 *
 * 1.1.29_MTK
 * (1) Masks mivr irq for 500ms after mivr irq gets handled
 *
 * 1.1.28_MTK
 * (1) mutex_unlock() once in mt6370_pmu_attachi_irq_handler()
 * (2) Do not do the workaround for VSYS overshoot if ichg <= 500mA
 *
 * 1.1.27_MTK
 * (1) Only RT5081/MT6370 need BC12 workaround
 * (2) Fix mt6370_post_aicl_measure() to return measure between start and stop
 *
 * 1.1.26_MTK
 * (1) Add support for MT6372
 * (2) Add workaround for VSYS overshoot
 *
 * 1.1.25_MTK
 * (1) Keep mivr via chg_ops
 *
 * 1.1.24_MTK
 * (1) Add debug information for ADC
 *
 * 1.1.23_MTK
 * (1) Use bc12_access_lock instead of chgdet_lock
 * (2) Add junction ADC workaround
 *
 * 1.1.22_MTK
 * (1) If ichg < 900mA -> disable vsys sp
 *        ichg >= 900mA -> enable vsys sp
 *     Always keep ichg >= 500mA
 *
 * 1.1.21_MTK
 * (1) Remove keeping ichg >= 900mA
 * (2) Add BC12 SDP workaround
 *
 * 1.1.20_MTK
 * (1) Always keep setting of ichg >= 900mA
 * (2) Remove setting ichg to 512mA in pep20_reset
 *
 * 1.1.19_MTK
 * (1) Always disable charger detection first no matter use it or not
 *
 * 1.1.18_MTK
 * (1) Move AICL work from MIVR IRQ handler to charger thread
 * (2) Keep IEOC setting which is modified in set_ichg
 * (3) Add enable_te & set_ieoc ops
 * (4) Add enable_irq function to enable/disable irq usage
 * (5) Use atomic to represent tcpc usb connected
 * (6) Move definition of mt6370 charger type detect to Kconfig
 *
 * 1.1.17_MTK
 * (1) Remove delay after Disable TS auto sensing for speed up
 * (2) Remove dump register in probe for speed up
 *
 * 1.1.16_MTK
 * (1) Modify microB BC1.2 flow to fix AP usb switch issue
 * (2) Remove unlock battery protection workaround
 * (3) Add enable/disable psk mode for pep20_reset
 *
 * 1.1.15_MTK
 * (1) Prevent backboost
 * (2) After PE pattern -> Enable psk mode
 *     Disable psk mode -> Start PE pattern
 *
 * 1.1.14_MTK
 * (1) Add workaround to adjust slow rate for OTG
 *
 * 1.1.13_MTK
 * (1) Add polling mode for EOC/Rechg
 * (2) Use charger_dev_notify instead of srcu_notifier_call_chain
 *
 * 1.1.12_MTK
 * (1) Enable charger before sending PE+/PE+20 pattern
 * (2) Select to use reg AICR as input limit -> disable HW limit
 *
 * 1.1.11_MTK
 * (1) Fix get_adc lock unbalance issue
 * (2) Add pe_access_lock
 * (3) Add a flag to check enable status of bc12
 *
 * 1.1.10_MTK
 * (1) Add ext usb switch control
 * (2) Add MT6370_APPLE_SAMSUNG_TA_SUPPORT config
 * (3) Remove some debug log and unncessary code
 *
 * 1.1.9_MTK
 * (1) Use polling mode for ADC done and PE+
 * (2) Return immediately if usb is plugged out while waiting CDP
 *
 * 1.1.8_MTK
 * (1) Read CHG_STAT in dump register
 * (2) Read ZCV before disabling it and add ZCV ops
 * (3) Add AEE log for ADC hang issue
 *
 * 1.1.7_MTK
 * (1) Add a adc flag for adc_done IRQ
 * (2) Not waitting ssfinish IRQ after enabling OTG
 * (3) Add debug information for ADC
 * (4) For degug, read ADC data even if conversation failed
 * (5) Trigger any ADC before disabling ZCV
 *
 * 1.1.6_MTK
 * (1) Read USB STATUS(0x27) instead of device type(0x22)
 *     to check charger type
 * (2) Show CV value in dump_register
 * (3) If PE20/30 is connected, do not run AICL
 * (4) Disable MIVR IRQ -> enable direct charge
 *     Enable MIVR IRQ -> disable direct charge
 *
 * 1.1.5_MTK
 * (1) Modify probe sequence
 * (2) Change ADC log from ratelimited to normal one for debug
 * (3) Polling ADC_START after receiving ADC_DONE irq
 * (4) Disable ZCV in probe function
 * (5) Plug out -> enable ILIM_EN
 *     Plug in -> sleep 1200ms -> disable ILIM_EN
 *
 * 1.1.4_MTK
 * (1) Add IEOC workaround
 *
 * 1.1.3_MTK
 * (1) Enable safety timer, rechg, vbusov, eoc IRQs
 *     and notify charger manager if event happens
 * (2) Modify IBAT/IBUS ADC's coefficient
 * (3) Change dev_dbg to dev_dbg_ratelimited
 *
 * 1.1.2_MTK
 * (1) Enable charger in plug in callback
 * (2) Enable power path -> Enable MIVR IRQ,
 *     Disable MIVR IRQ -> Disable power path
 * (3) Change "PrimarySWCHG" to "primary_chg"
 *     Change "load_switch" to "primary_load_switch"
 * (4) Check ADC_START bit if ADC_DONE IRQ timeout
 *
 * 1.1.1_MTK
 * (1) Shorten enable to en
 * (2) Enable WDT for charging/OTG mode, disable WDT when no cable plugged in
 * (3) Use dev_ series instead pr_ series to dump log
 * (4) Do AICL in a workqueue after receiving MIVR IRQ
 *
 * 1.1.0_MTK
 * (1) Add a load switch device to support PPS
 *
 * 1.0.9_MTK
 * (1) Report charger online to charger type detection
 * (2) Use MIVR to enable/disable power path
 * (3) Release PE+20 efficiency table interface
 * (4) For ovpctrl_uvp, read uvp status to decide it is plug-in or plug-out
 *
 * 1.0.8_MTK
 * (1) Release mt6370_enable_discharge interface
 *     discharge/OTG is controlled by PD
 *
 * 1.0.7_MTK
 * (1) Adapt to GM30
 *
 * 1.0.6_MTK
 * (1) Use wait queue instead of mdelay for waiting interrupt event
 *
 * 1.0.5_MTK
 * (1) Modify USB charger type detecion flow
 * Follow notifier from TypeC if CONFIG_TCPC_CLASS is defined
 * (2) Add WDT timeout interrupt and kick watchdog in irq handler
 *
 * 1.0.4_MTK
 * (1) Add USB charger type detection
 *
 * 1.0.3_MTK
 * (1) Remove reset chip operation in probe function
 *
 * 1.0.2_MTK
 * (1) For normal boot, set IPREC to 850mA to prevent Isys drop
 * (2) Modify naming rules of functions
 *
 * 1.0.0_MTK
 * (1) Initial Release
 */
