// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/reboot.h>
#include <linux/module.h>
#include <linux/sched/clock.h>
#include <soc/oplus/system/oplus_project.h>

#ifdef CONFIG_OPLUS_CHARGER_MTK

#include <mtk_boot_common.h>
//for dx1 bringup
//#include <mt-plat/mtk_boot.h>
#include <linux/gpio.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
#include <uapi/linux/sched/types.h>
#endif
#else /* CONFIG_OPLUS_CHARGER_MTK */
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/of.h>

#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/leds.h>
#include <linux/rtc.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
#include <linux/qpnp/qpnp-adc.h>
#else
#include <uapi/linux/sched/types.h>
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
#include <linux/msm_bcl.h>
#endif
#include <linux/ktime.h>
#include <linux/kernel.h>
#endif

#include "oplus_charger.h"
#include "oplus_gauge.h"
#include "oplus_vooc.h"
#include "oplus_short.h"
#include "oplus_adapter.h"
#include "charger_ic/oplus_short_ic.h"
#include "oplus_debug_info.h"
//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
#ifndef WPC_NEW_INTERFACE
#include "oplus_wireless.h"
#include "wireless_ic/oplus_chargepump.h"	//for WPC
#else
#include "oplus_wireless.h"
#endif
//#endif
#include "oplus_chg_ops_manager.h"
#include "oplus_chg_comm.h"
#include "oplus_pps.h"
#include "oplus_chg_wls.h"

static struct oplus_chg_chip *g_charger_chip = NULL;

#define MAX_UI_DECIMAL_TIME 24
#define UPDATE_TIME 1

#define OPLUS_CHG_UPDATE_INTERVAL_SEC 		5
#define OPLUS_CHG_UPDATE_NO_CHARGE_INTERVAL_SEC        10

/* first run after init 10s */
#define OPLUS_CHG_UPDATE_INIT_DELAY	round_jiffies_relative(msecs_to_jiffies(500))
/* update cycle 5s */
#define OPLUS_CHG_UPDATE_INTERVAL(time)        round_jiffies_relative(msecs_to_jiffies(time*1000))


#define OPLUS_CHG_DEFAULT_CHARGING_CURRENT	512

int enable_charger_log = 2;
int charger_abnormal_log = 0;
int tbatt_pwroff_enable = 1;
extern bool oplus_is_power_off_charging(struct oplus_vooc_chip *chip);

#define charger_xlog_printk(num, fmt, ...) \
		do { \
			if (enable_charger_log >= (int)num) { \
				printk(KERN_NOTICE pr_fmt("[OPLUS_CHG][%s]"fmt), __func__, ##__VA_ARGS__); \
			} \
		} while (0)

int get_boot_reason(void)
{
	return 0;
}

void oplus_chg_turn_off_charging(struct oplus_chg_chip *chip);
void oplus_chg_turn_on_charging(struct oplus_chg_chip *chip);

static void oplus_chg_smooth_to_soc(struct oplus_chg_chip *chip);
static void oplus_chg_variables_init(struct oplus_chg_chip *chip);
static void oplus_chg_update_work(struct work_struct *work);
static void oplus_chg_reset_adapter_work(struct work_struct *work);
static void oplus_chg_protection_check(struct oplus_chg_chip *chip);
static void oplus_chg_get_battery_data(struct oplus_chg_chip *chip);
static void oplus_chg_check_tbatt_status(struct oplus_chg_chip *chip);
static void oplus_chg_check_tbatt_normal_status(struct oplus_chg_chip *chip);
static void oplus_chg_get_chargerid_voltage(struct oplus_chg_chip *chip);
void oplus_chg_set_input_current_limit(struct oplus_chg_chip *chip);
static void oplus_chg_set_charging_current(struct oplus_chg_chip *chip);
static void oplus_chg_battery_update_status(struct oplus_chg_chip *chip);
static void oplus_chg_pdqc_to_normal(struct oplus_chg_chip *chip);
static void oplus_get_smooth_soc_switch(struct oplus_chg_chip *chip);
static void oplus_chg_pd_config(struct oplus_chg_chip *chip);
static void oplus_chg_qc_config(struct oplus_chg_chip *chip);
extern void oplus_start_svooc_reset(void);
extern void oplus_start_pps_reset(void);
#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *nb, unsigned long event, void *data);
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
static int chg_mtk_drm_notifier_callback(struct notifier_block *nb, unsigned long event, void *data);
#endif
void oplus_chg_ui_soc_decimal_init(void);
void oplus_chg_ui_soc_decimal_deinit(void);

static void oplus_chg_show_ui_soc_decimal(struct work_struct *work);


static int chgr_dbg_vchg = 0;
module_param(chgr_dbg_vchg, int, 0644);
MODULE_PARM_DESC(chgr_dbg_vchg, "debug charger voltage");

static int chgr_dbg_total_time = 0;
module_param(chgr_dbg_total_time, int, 0644);
MODULE_PARM_DESC(chgr_dbg_total_time, "debug charger total time");

/****************************************/
static int reset_mcu_delay = 0;
static bool mcu_update = false;
static bool suspend_charger = false;
static bool vbatt_higherthan_4180mv = false;
static bool vbatt_lowerthan_3300mv = false;

__maybe_unused static bool is_wls_ocm_available(struct oplus_chg_chip *chip)
{
	if (!chip->wls_ocm)
		chip->wls_ocm = oplus_chg_mod_get_by_name("wireless");
	return !!chip->wls_ocm;
}

__maybe_unused static bool is_comm_ocm_available(struct oplus_chg_chip *chip)
{
	if (!chip->comm_ocm)
		chip->comm_ocm = oplus_chg_mod_get_by_name("common");
	return !!chip->comm_ocm;
}

bool oplus_chg_is_wls_present(void)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	union oplus_chg_mod_propval temp_val = {0};
	bool present = false;
	int rc;

	if (!chip) {
		return false;
	}

	if (!is_wls_ocm_available(chip))
		return false;

	rc = oplus_chg_mod_get_property(chip->wls_ocm,
					OPLUS_CHG_PROP_ONLINE_KEEP, &temp_val);
	if (rc < 0)
		present = false;
	else
		present = temp_val.intval;
	rc = oplus_chg_mod_get_property(chip->wls_ocm, OPLUS_CHG_PROP_PRESENT,
					&temp_val);
	if (rc >= 0)
		present = present || (!!temp_val.intval);

	return present;
}

bool oplus_chg_is_wls_fastchg_started(void)
{
	int rc = 0;
	union oplus_chg_mod_propval pval = {0, };
	struct oplus_chg_chip *chip = g_charger_chip;

	if (!chip) {
		return false;
	}

	if (!is_wls_ocm_available(chip)) {
		/*pr_err("wls ocm not found\n");*/
		return false;
	}
	rc = oplus_chg_mod_get_property(chip->wls_ocm, OPLUS_CHG_PROP_FASTCHG_STATUS, &pval);
	if (rc < 0)
		return false;

	return !!pval.intval;
}

bool oplus_chg_is_wls_ffc(void)
{
	bool norchg_ffc = false;
	bool fastchg_started = false;
	struct oplus_chg_chip *chip = g_charger_chip;

	if (!chip) {
		return false;
	}

	if (!is_comm_ocm_available(chip)) {
		/*pr_err("comm ocm not found\n");*/
		return false;
	}

	fastchg_started = oplus_chg_is_wls_fastchg_started();
	norchg_ffc = oplus_chg_comm_get_ffc_status(chip->comm_ocm) != FFC_DEFAULT ? true : false;

	return (fastchg_started | norchg_ffc);
}

int oplus_chg_get_wls_type(void)
{
        int rc = 0;
        union oplus_chg_mod_propval pval = {0, };
        struct oplus_chg_chip *chip = g_charger_chip;

        if (!chip) {
                return OPLUS_CHG_WLS_UNKNOWN;
        }

        if (!is_wls_ocm_available(chip)) {
                /*pr_err("wls ocm not found\n");*/
                return OPLUS_CHG_WLS_UNKNOWN;
        }
        rc = oplus_chg_mod_get_property(chip->wls_ocm, OPLUS_CHG_PROP_WLS_TYPE, &pval);
        if (rc < 0)
                return OPLUS_CHG_WLS_UNKNOWN;

        return pval.intval;
}

bool oplus_chg_is_wls_fw_upgrading(void)
{
	int rc = 0;
	union oplus_chg_mod_propval pval = {0, };
	struct oplus_chg_chip *chip = g_charger_chip;

	if (!chip) {
		return false;
	}

	if (!is_wls_ocm_available(chip)) {
		return false;
	}

	rc = oplus_chg_mod_get_property(chip->wls_ocm, OPLUS_CHG_PROP_FW_UPGRADING, &pval);
	if (rc < 0)
		return false;

	return !!pval.intval;
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
int oplus_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
		{
	int ret = 0;
	struct oplus_chg_chip *chip = g_charger_chip;

	if (chip->charger_exist) {
		if ((chip->charger_type == POWER_SUPPLY_TYPE_USB
				|| chip->charger_type == POWER_SUPPLY_TYPE_USB_CDP)
				&& chip->stop_chg == 1) {
			chip->usb_online = true;
			chip->usb_psd.type = POWER_SUPPLY_TYPE_USB;
		}
	} else {
		chip->usb_online = false;
	}

	switch (psp) {
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = 500000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = 5000000;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = chip->usb_online;
			break;
		default:
			pr_err("get prop %d is not supported in usb\n", psp);
			ret = -EINVAL;
			break;
	}
	return ret;
}
EXPORT_SYMBOL(oplus_usb_get_property);

static void usb_update(struct oplus_chg_chip *chip)
{
	if (chip->charger_exist) {
		/*if (chip->charger_type==STANDARD_HOST || chip->charger_type==CHARGING_HOST) {*/
		if (chip->charger_type == POWER_SUPPLY_TYPE_USB
				|| chip->charger_type == POWER_SUPPLY_TYPE_USB_CDP) {
			chip->usb_online = true;
			chip->usb_psd.type = POWER_SUPPLY_TYPE_USB;
		}
	} else {
		chip->usb_online = false;
	}
	power_supply_changed(chip->usb_psy);
}
#endif

enum power_supply_property oplus_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

int oplus_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	//struct oplus_chg_chip *chip = container_of(psy->desc, struct oplus_chg_chip, ac_psd);
	struct oplus_chg_chip *chip = g_charger_chip;

	if (chip->charger_exist) {
		if ((chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP || suspend_charger)
				|| (oplus_vooc_get_fastchg_started() == true)
				|| (oplus_vooc_get_fastchg_to_normal() == true)
				|| (oplus_vooc_get_fastchg_to_warm() == true)
				|| (oplus_vooc_get_fastchg_dummy_started() == true)
				|| (oplus_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE)
				|| (oplus_vooc_get_btb_temp_over() == true)) {
			chip->ac_online = true;
		} else {
			chip->ac_online = false;
		}
	} else {
		if ((oplus_vooc_get_fastchg_started() == true)
				|| (oplus_vooc_get_fastchg_to_normal() == true)
				|| (oplus_vooc_get_fastchg_to_warm() == true)
				|| (oplus_vooc_get_fastchg_dummy_started() == true)
				|| (oplus_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE)
				|| (oplus_vooc_get_btb_temp_over() == true)
				|| chip->mmi_fastchg == 0) {
			chip->ac_online = true;
		} else {
			chip->ac_online = false;

		}
	}
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = chip->ac_online;
			break;
		default:
			pr_err("get prop %d is not supported in ac\n", psp);
			ret = -EINVAL;
			break;
	}
	if (chip->ac_online) {
		charger_xlog_printk(CHG_LOG_CRTI, "chg_exist:%d, ac_online:%d\n",
				chip->charger_exist, chip->ac_online);
	}
	return ret;
}
EXPORT_SYMBOL(oplus_ac_get_property);

int oplus_battery_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret = 0;
	struct oplus_chg_chip *chip = g_charger_chip;

	if (!chip) {
		return ret;
	}

	switch (psp) {
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if (chip && chip->smart_charging_screenoff) {
				ret = 1;
			} else {
				ret = 0;
			}
			break;
#endif
		default:
			ret = 0;
			break;
	}
	return ret;
}
EXPORT_SYMBOL(oplus_battery_property_is_writeable);

int oplus_battery_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	int ret = 0;
	struct oplus_chg_chip *chip = g_charger_chip;

	if (!chip) {
		return ret;
	}

	switch (psp) {
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if (chip->smart_charging_screenoff) {
				oplus_smart_charge_by_shell_temp(chip, val->intval);
				break;
			} else {
				ret = -EINVAL;
				break;
			}
#endif
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}
EXPORT_SYMBOL(oplus_battery_set_property);

#define OPLUS_MIDAS_CHG_DEBUG 1
#ifdef OPLUS_MIDAS_CHG_DEBUG
#define	midas_debug(fmt, args...)	\
	pr_notice("[OPLUS_MIDAS_CHG_DEBUG]" fmt, ##args)
#else
#define	midas_debug(fmt, args...)
#endif /* OPLUS_MIDAS_CHG_DEBUG */

static struct oplus_midas_chg {
	int cali_passed_chg;
	int passed_chg;
	int accu_delta;

	int prev_chg_stat;	/* 1--charger-on, 0 -- otherwise */

	unsigned int reset_counts;
} midas_chg;

static void oplus_midas_chg_info(const char *name)
{
	midas_debug("%s: passedchg=%d, realpassedchg=%d,"
		"accu_delta=%d, prev_chg_stat=%d, reset_counts=%u\n",
		name, midas_chg.cali_passed_chg, midas_chg.passed_chg,
		midas_chg.accu_delta, midas_chg.prev_chg_stat, midas_chg.reset_counts);
}

/* TODO: how to determine passedchg is reset precisely ? */
#define	__abs(a, b) ((a > b) ? (a - b) : (b - a))
#define ZEROTH	5
#define COMSUMETH 10
static bool oplus_midas_passedchg_reset(int prev, int val)
{
	if(__abs(val, prev) > COMSUMETH) {
		return true;
	} else if (__abs(val, 0) > ZEROTH) {
		return false;
	}

	if(prev < 0 && val - prev >= ZEROTH) {
			return true;
	}
	if(prev >= 0 && val < prev) {
			return true;
	}
	return false;
}

static void oplus_midas_chg_data_init(void)
{
	int val, ret;
	midas_chg.accu_delta = 0;
	midas_chg.reset_counts = 0;
	if(oplus_vooc_get_allow_reading() == true) {
		ret = oplus_gauge_get_passedchg(&val);
		if (ret) {
			pr_err("%s: get passedchg error %d\n", __FUNCTION__, val);
			midas_chg.cali_passed_chg = midas_chg.passed_chg = 0;
		} else {
			midas_chg.cali_passed_chg = midas_chg.passed_chg = val;
		}
	} else {
		pr_err("%s: not allow reading", __FUNCTION__);
	}
}

static void oplus_midas_chg_processing(struct oplus_chg_chip *chip)
{
	static int inited = 0;
	int val, ret = 0;

	if (!inited) {
		oplus_midas_chg_data_init();
		inited = 1;
		return;
	}

	if (chip->charger_exist) {
		midas_chg.prev_chg_stat = 1;
		oplus_midas_chg_info(__FUNCTION__);
		return;
	}

	if(oplus_vooc_get_allow_reading() == true) {
		ret = oplus_gauge_get_passedchg(&val);
		if (ret) {
			pr_err("%s: get passedchg error %d\n", __FUNCTION__, val);
			return;
		}
	} else {
		pr_err("%s: not allow reading", __FUNCTION__);
	}
	if (midas_chg.prev_chg_stat) {
		/* re-init passedchg after charge */
		midas_chg.cali_passed_chg = midas_chg.passed_chg = val;
		midas_chg.accu_delta = 0;
		midas_chg.prev_chg_stat = 0;
	} else {
		if (oplus_midas_passedchg_reset(midas_chg.passed_chg, val)) {
			/* handling passedchg reset ... */
			midas_chg.cali_passed_chg += midas_chg.accu_delta;
			midas_chg.reset_counts++;
		} else {
			/* accumulate normally */
			midas_chg.accu_delta = val - midas_chg.passed_chg;
			midas_chg.cali_passed_chg += midas_chg.accu_delta;
		}
		midas_chg.passed_chg = val;
	}

	oplus_midas_chg_info(__FUNCTION__);
}

int oplus_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	//struct oplus_chg_chip *chip = container_of(psy->desc, struct oplus_chg_chip, battery_psd);
	struct oplus_chg_chip *chip = g_charger_chip;
	union oplus_chg_mod_propval temp_val = {0, };
	int batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	bool wls_online = false;
	static int pre_batt_status;

	if(chip == NULL) {
		pr_err("get prop %d is not ready in batt\n", psp);
		return -EINVAL;
	}

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			if (chip->wls_status_keep) {
				val->intval = pre_batt_status;
			} else {
				if (oplus_chg_show_vooc_logo_ornot() == 1) {
					if ((chip->tbatt_status == BATTERY_STATUS__COLD_TEMP
						|| chip->tbatt_status == BATTERY_STATUS__WARM_TEMP)
						&& chip->batt_full)
						val->intval = POWER_SUPPLY_STATUS_CHARGING;
					else
						val->intval = chip->prop_status;
				} else if (!chip->authenticate) {
					val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
				} else {
					val->intval = chip->prop_status;
				}
				if (oplus_chg_is_wls_present())
					pre_batt_status = val->intval;
			}
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			if (is_wls_ocm_available(chip)) {
				ret = oplus_chg_mod_get_property(chip->wls_ocm, OPLUS_CHG_PROP_PRESENT, &temp_val);
				if (ret == 0)
					wls_online = !!temp_val.intval;
				else
					ret = 0;
			}

			if (is_comm_ocm_available(chip) && wls_online)
				batt_health = oplus_chg_comm_get_batt_health(chip->comm_ocm);

			if ((batt_health == POWER_SUPPLY_HEALTH_UNKNOWN) ||
		    			(batt_health == POWER_SUPPLY_HEALTH_GOOD)) {
				val->intval = oplus_chg_get_prop_batt_health(chip);
			} else {
				val->intval = batt_health;
			}
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = chip->batt_exist;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			if(chip->vooc_show_ui_soc_decimal == true && chip->decimal_control) {
				val->intval = (chip->ui_soc_integer + chip->ui_soc_decimal)/1000;
			} else {
				val->intval = chip->ui_soc;
			}
			if(val->intval > 100) {
				val->intval = 100;
			}

			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef CONFIG_OPLUS_CHARGER_MTK
			val->intval = chip->batt_volt;
#else
			val->intval = chip->batt_volt * 1000;
#endif
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN:
#ifdef CONFIG_OPLUS_CHARGER_MTK
			val->intval = chip->batt_volt_min;
#else
			val->intval = chip->batt_volt_min * 1000;
#endif
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if (oplus_vooc_get_fastchg_started() == true) {
				chip->icharging = oplus_gauge_get_prev_batt_current();
			} else {
				chip->icharging = oplus_gauge_get_batt_current();
			}
			val->intval = chip->icharging;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			if (oplus_vooc_get_fastchg_started() == true) {
				val->intval = chip->tbatt_temp - chip->offset_temp;
			} else {
				val->intval = chip->tbatt_temp - chip->offset_temp;
			}
			break;
		case POWER_SUPPLY_PROP_CHARGE_NOW:
			if (oplus_vooc_get_fastchg_started() == true && (chip->vbatt_num == 2)
			&& oplus_vooc_get_fast_chg_type() != CHARGER_SUBTYPE_FASTCHG_VOOC) {
				val->intval = 10000;
			} else {
				val->intval = chip->charger_volt;
			}

			if (is_wls_ocm_available(chip)) {
				oplus_chg_mod_get_property(chip->wls_ocm,
					OPLUS_CHG_PROP_TRX_ONLINE, &temp_val);
				if (!!temp_val.intval) {
					ret = oplus_chg_mod_get_property(chip->wls_ocm,
						OPLUS_CHG_PROP_TRX_VOLTAGE_NOW, &temp_val);
					val->intval = temp_val.intval / 1000;
				}
			}
			break;
#ifdef CONFIG_OPLUS_CHARGER_MTK
		case POWER_SUPPLY_PROP_CHARGE_COUNTER:
			val->intval = chip->ui_soc * chip->batt_capacity_mah * 1000 / 100;
			break;
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = 2000;
			break;
#endif
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			val->intval = chip->batt_fcc * 1000;
			break;
		default:
			pr_err("get prop %d is not supported in batt\n", psp);
			ret = -EINVAL;
			break;
	}
	return ret;
}
EXPORT_SYMBOL(oplus_battery_get_property);

static ssize_t proc_batt_param_noplug_write(struct file *filp,
		const char __user *buf, size_t len, loff_t *data)
{
	return len;
}

static int noplug_temperature = 0;
static int noplug_batt_volt_max = 0;
static int noplug_batt_volt_min = 0;
static ssize_t proc_batt_param_noplug_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off) {
	char page[256] = {0};
	char read_data[128] = {0};
	int len = 0;

	sprintf(read_data, "%d %d %d", noplug_temperature,
		noplug_batt_volt_max, noplug_batt_volt_min);
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

#if 0
static const struct file_operations batt_param_noplug_proc_fops = {
	.write = proc_batt_param_noplug_write,
	.read = proc_batt_param_noplug_read,
};
#else
static const struct proc_ops batt_param_noplug_proc_fops = {
	.proc_write = proc_batt_param_noplug_write,
	.proc_read = proc_batt_param_noplug_read,
};
#endif

static int init_proc_batt_param_noplug(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("batt_param_noplug", 0664, NULL, &batt_param_noplug_proc_fops);
	if (!p) {
		chg_err("proc_create  fail!\n");
	}
	return 0;
}

static ssize_t proc_tbatt_pwroff_write(struct file *filp,
		const char __user *buf, size_t len, loff_t *data)
{
	char buffer[2] = {0};

	if (len > 2) {
		return -EFAULT;
	}
	if (copy_from_user(buffer, buf, 2)) {
		chg_err("%s:  error.\n", __func__);
		return -EFAULT;
	}
	if (buffer[0] == '0') {
		tbatt_pwroff_enable = 0;
	} else {
		tbatt_pwroff_enable = 1;
		oplus_tbatt_power_off_task_wakeup();
	}
	chg_err("%s:tbatt_pwroff_enable = %d.\n", __func__, tbatt_pwroff_enable);
	return len;
}

static ssize_t proc_tbatt_pwroff_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[3] = {0};
	int len = 0;

	if (tbatt_pwroff_enable == 1) {
		read_data[0] = '1';
	} else {
		read_data[0] = '0';
	}
	read_data[1] = '\0';
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

//for dx1 bringup
#if 0
static const struct file_operations tbatt_pwroff_proc_fops = {
	.write = proc_tbatt_pwroff_write,
	.read = proc_tbatt_pwroff_read,
};
#else
static const struct proc_ops tbatt_pwroff_proc_fops = {
	.proc_write = proc_tbatt_pwroff_write,
	.proc_read = proc_tbatt_pwroff_read,
};
#endif

static int init_proc_tbatt_pwroff(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("tbatt_pwroff", 0664, NULL, &tbatt_pwroff_proc_fops);
	if (!p) {
		chg_err("proc_create  fail!\n");
	}
	return 0;
}

static ssize_t chg_log_write(struct file *filp,
		const char __user *buff, size_t len, loff_t *data)
{
	char write_data[32] = {0};

	if (len > sizeof(write_data)) {
		chg_err("bat_log_write error.\n");
		return -EFAULT;
	}
	if (copy_from_user(&write_data, buff, len)) {
		chg_err("bat_log_write error.\n");
		return -EFAULT;
	}
	if (write_data[0] == '1') {
		charger_xlog_printk(CHG_LOG_CRTI, "enable battery driver log system\n");
		enable_charger_log = 1;
	} else if ((write_data[0] >= '2') &&(write_data[0] <= '9')) {
		charger_xlog_printk(CHG_LOG_CRTI, "enable battery driver log system:2\n");
		enable_charger_log = 2;
	} else {
		charger_xlog_printk(CHG_LOG_CRTI, "Disable battery driver log system\n");
		enable_charger_log = 0;
	}
	return len;
}

static ssize_t chg_log_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[32] = {0};
	int len = 0;

	if (enable_charger_log == 1) {
		read_data[0] = '1';
	} else if (enable_charger_log == 2) {
		read_data[0] = '2';
	} else {
		read_data[0] = '0';
	}
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

//for dx1 bringup
#if 0
static const struct file_operations chg_log_proc_fops = {
	.write = chg_log_write,
	.read = chg_log_read,
};
#else
static const struct proc_ops chg_log_proc_fops = {
	.proc_write = chg_log_write,
	.proc_read = chg_log_read,
};
#endif

static int init_proc_chg_log(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("charger_log", 0664, NULL, &chg_log_proc_fops);
	if (!p) {
		chg_err("proc_create chg_log_proc_fops fail!\n");
	}
	return 0;
}

static void oplus_chg_set_awake(struct oplus_chg_chip *chip, bool awake);

static ssize_t chg_cycle_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	char proc_chg_cycle_data[16];

	if(count >= 16) {
		count = 16;
	}
	if (copy_from_user(&proc_chg_cycle_data, buff, count)) {
		chg_err("chg_cycle_write error.\n");
		return -EFAULT;
	}
	if (strncmp(proc_chg_cycle_data, "en808", 5) == 0) {
		if(g_charger_chip->unwakelock_chg == 1) {
			charger_xlog_printk(CHG_LOG_CRTI, "unwakelock testing , this test not allowed.\n");
			return -EPERM;
		}
		charger_xlog_printk(CHG_LOG_CRTI, "allow charging.\n");
		g_charger_chip->chg_ops->charger_unsuspend();
		g_charger_chip->chg_ops->charging_enable();
		g_charger_chip->mmi_chg = 1;
		g_charger_chip->stop_chg = 1;
		if (g_charger_chip->dual_charger_support) {
			g_charger_chip->slave_charger_enable = false;
			oplus_chg_set_charging_current(g_charger_chip);
		}
		oplus_chg_set_input_current_limit(g_charger_chip);
	} else if (strncmp(proc_chg_cycle_data, "dis808", 6) == 0) {
		if(g_charger_chip->unwakelock_chg == 1) {
			charger_xlog_printk(CHG_LOG_CRTI, "unwakelock testing , this test not allowed.\n");
			return -EPERM;
		}
		charger_xlog_printk(CHG_LOG_CRTI, "not allow charging.\n");
		g_charger_chip->chg_ops->charging_disable();
		g_charger_chip->chg_ops->charger_suspend();
		g_charger_chip->mmi_chg = 0;
		g_charger_chip->stop_chg = 0;
		g_charger_chip->batt_full = false;
		g_charger_chip->charging_state = CHARGING_STATUS_CCCV;

		if (oplus_vooc_get_fastchg_started() == true) {
			g_charger_chip->mmi_fastchg = 0;
		}

		if(g_charger_chip->chg_ops->get_chargerid_switch_val
			&& (g_charger_chip->chg_ops->get_chargerid_switch_val() == 1
			|| oplus_vooc_get_fastchg_to_normal() == true
			|| oplus_vooc_get_fastchg_to_warm() == true)) {
			oplus_vooc_turn_off_fastchg();
		}
	} else if (strncmp(proc_chg_cycle_data, "wakelock", 8) == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, "set wakelock.\n");
		g_charger_chip->unwakelock_chg = 0;
		oplus_chg_set_awake(g_charger_chip, true);
		g_charger_chip->chg_ops->charger_unsuspend();
		g_charger_chip->chg_ops->charging_enable();
		g_charger_chip->mmi_chg = 1;
		g_charger_chip->stop_chg = 1;
		g_charger_chip->chg_powersave = false;
		if (g_charger_chip->chg_ops->oplus_chg_wdt_enable)
			g_charger_chip->chg_ops->oplus_chg_wdt_enable(true);
		if (g_charger_chip->mmi_fastchg == 0) {
			oplus_chg_clear_chargerid_info();
		}
		g_charger_chip->mmi_fastchg = 1;
	} else if (strncmp(proc_chg_cycle_data, "unwakelock", 10) == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, "set unwakelock.\n");
		g_charger_chip->chg_ops->charging_disable();
		//g_charger_chip->chg_ops->charger_suspend();
		g_charger_chip->mmi_chg = 0;
		g_charger_chip->stop_chg = 0;
		g_charger_chip->unwakelock_chg = 1;
		g_charger_chip->chg_powersave = true;
		if (oplus_vooc_get_fastchg_started() == true) {
			oplus_vooc_turn_off_fastchg();
			g_charger_chip->mmi_fastchg = 0;
		}
		oplus_chg_set_awake(g_charger_chip, false);
		if (g_charger_chip->chg_ops->oplus_chg_wdt_enable)
			g_charger_chip->chg_ops->oplus_chg_wdt_enable(false);
	} else if (strncmp(proc_chg_cycle_data, "powersave", 9) == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, "powersave: stop usbtemp monitor, etc.\n");
		g_charger_chip->chg_powersave = true;
	} else if (strncmp(proc_chg_cycle_data, "unpowersave", 11) == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, "unpowersave: start usbtemp monitor, etc.\n");
		g_charger_chip->chg_powersave = false;
	} else {
		return -EFAULT;
	}
	return count;
}

//for dx1 bringup
#if 0
static const struct file_operations chg_cycle_proc_fops = {
	.write = chg_cycle_write,
	.llseek = noop_llseek,
};
#else
static const struct proc_ops chg_cycle_proc_fops = {
	.proc_write = chg_cycle_write,
	.proc_lseek = noop_llseek,
};
#endif

static void init_proc_chg_cycle(void)
{
	if (!proc_create("charger_cycle",
			S_IWUSR | S_IWGRP | S_IWOTH,
			NULL, &chg_cycle_proc_fops)) {
		chg_err("proc_create chg_cycle_proc_fops fail!\n");
	}
}
static ssize_t critical_log_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[32] = {0};
	int len = 0;
	//	itoa(charger_abnormal_log, read_data, 10);
	//	sprintf(read_data,"%s",charger_abnormal_log);
	if (charger_abnormal_log >= 10) {
		charger_abnormal_log = 10;
	}
	read_data[0] = '0' + charger_abnormal_log % 10;
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static ssize_t critical_log_write(struct file *filp,
		const char __user *buff, size_t len, loff_t *data)
{
	char write_data[32] = {0};
	int critical_log = 0;

	if (len > sizeof(write_data) || len <= 0) {
		return -EINVAL;
	}
	if (copy_from_user(&write_data, buff, len)) {
		pr_err("bat_log_write error.\n");
		return -EFAULT;
	}
	/*	critical_log = atoi(write_data);*/
	/*	sprintf(critical_log,"%d",(void *)write_data);*/
	write_data[len] = '\0';
	if (write_data[len - 1] == '\n') {
		write_data[len - 1] = '\0';
	}
	critical_log = (int)simple_strtoul(write_data, NULL, 10);
	/*	pr_err("%s:data=%s,critical_log=%d\n",__func__,write_data,critical_log);*/
	if (critical_log > 256) {
		critical_log = 256;
	}
	charger_abnormal_log = critical_log;
	return len;
}

//for dx1 bringup
#if 0
static const struct file_operations chg_critical_log_proc_fops = {
	.write = critical_log_write,
	.read = critical_log_read,
};
#else
static const struct proc_ops chg_critical_log_proc_fops = {
	.proc_write = critical_log_write,
	.proc_read = critical_log_read,
};
#endif

static void init_proc_critical_log(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("charger_critical_log", 0664, NULL,
		&chg_critical_log_proc_fops);
	if (!p) {
		pr_err("proc_create chg_critical_log_proc_fops fail!\n");
	}
}

#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
/* wenbin.liu add for det rtc reset */
static ssize_t rtc_reset_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[32] = {0};
	int len = 0;
	int rc = 0;

	if (!g_charger_chip) {
		return -EFAULT;
	} else {
		rc = g_charger_chip->chg_ops->check_rtc_reset();
	}
	if (rc < 0 || rc >1) {
		rc = 0;
	}
	read_data[0] = '0' + rc % 10;
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

//for dx1 bringup
#if 0
static const struct file_operations rtc_reset_det_fops = {
	.read = rtc_reset_read,
};
#else
static const struct proc_ops rtc_reset_det_fops = {
	.proc_read = rtc_reset_read,
};
#endif

static void init_proc_rtc_det(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("rtc_reset", 0664, NULL, &rtc_reset_det_fops);
	if (!p) {
		pr_err("proc_create rtc_reset_det_fops fail!\n");
	}
}
static ssize_t vbat_low_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[32] = {0};
	int len = 0;
	int rc = 0;

	if (!g_charger_chip)
		return -EFAULT;
	if (vbatt_lowerthan_3300mv) {
		rc = 1;
	}
	read_data[0] = '0' + rc % 10;
	len = sprintf(page,"%s",read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff,page,(len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

//for dx1 bringup
#if 0
static const struct file_operations vbat_low_det_fops = {
	.read = vbat_low_read,
};
#else
static const struct proc_ops vbat_low_det_fops = {
	.proc_read = vbat_low_read,
};
#endif

static void init_proc_vbat_low_det(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("vbat_low", 0664, NULL, &vbat_low_det_fops);
	if (!p) {
		pr_err("proc_create rtc_reset_det_fops fail!\n");
	}
}

#endif /* CONFIG_OPLUS_RTC_DET_SUPPORT */

static int oplus_chg_update_slow(struct oplus_chg_chip *chip)
{
	int ibat_sum = 0;
	int ibat_average;
	bool slow = 0;
	int i;

	for (i = 0; i < (ARRAY_SIZE(chip->ibat_save) - 1); i++) {
		chip->ibat_save[i] = chip->ibat_save[i + 1];
		ibat_sum += chip->ibat_save[i];
	}

	chip->ibat_save[i] = chip->icharging;

	ibat_sum += chip->ibat_save[i];
	ibat_average = ibat_sum / ARRAY_SIZE(chip->ibat_save);

	slow = !((chip->wireless_support && oplus_wpc_get_wireless_charge_start() == true) || chip->charger_exist ||
		(ibat_average >= 500) || (chip->soc <= 15) ||
		(chip->tbatt_temp >= 500) || (chip->soc >= 90));

	pr_debug("ibat_average = %d, slow = %d\n", ibat_average, slow);

	return slow? OPLUS_CHG_UPDATE_NO_CHARGE_INTERVAL_SEC : OPLUS_CHG_UPDATE_INTERVAL_SEC;
}

 int oplus_get_vbatt_pdqc_to_9v_thr(void)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	int rc, ret;

	if (chip->dev->of_node) {
		rc = of_property_read_u32(chip->dev->of_node, "qcom,vbatt_pdqc_to_9v_thr",
			&ret);
		if (rc < 0) {
			ret = 4000;
		}
	}else{
		ret = 4000;
	}
	return ret;
}
static ssize_t proc_charger_factorymode_test_write
				(struct file *file, const char __user *buf,
					size_t count, loff_t *lo)
{
	char buffer[2] = { 0 };
	struct oplus_chg_chip *chip = g_charger_chip;

	if (chip == NULL) {
		chg_err("%s: g_charger_chip driver is not ready\n", __func__);
		return -1;
	}

	if (count > 2) {
		return -1;
	}
	if (copy_from_user(buffer, buf, 1)) {
		chg_err("%s: error.\n", __func__);
		return -1;
	}

	if(buffer[0] == '1'){
		chip->limits.vbatt_pdqc_to_9v_thr = 4100;
		chg_err("vbatt_pdqc_to_9v_thr=%d\n", chip->limits.vbatt_pdqc_to_9v_thr);
		oplus_chg_pd_config(chip);
		oplus_chg_qc_config(chip);
	}
	if(buffer[0] == '0'){
		chip->limits.vbatt_pdqc_to_9v_thr = oplus_get_vbatt_pdqc_to_9v_thr();
		chg_err("vbatt_pdqc_to_9v_thr=%d\n", chip->limits.vbatt_pdqc_to_9v_thr);
	}

	return count;
}

//for dx1 bringup
#if 0
static const struct file_operations proc_charger_factorymode_test_ops =
{
    .write  = proc_charger_factorymode_test_write,
    .open  = simple_open,
    .owner = THIS_MODULE,
};
#else
static const struct proc_ops proc_charger_factorymode_test_ops =
{
    .proc_write  = proc_charger_factorymode_test_write,
    .proc_open  = simple_open,
};
#endif

static ssize_t proc_hmac_write(struct file *filp,
		const char __user *buf, size_t len, loff_t *data)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	char buffer[2] = {0};

	if (NULL == chip)
		return  -EFAULT;

	if (len > 2) {
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, 2)) {
		chg_err("%s:  error.\n", __func__);
		return -EFAULT;
	}
	if (buffer[0] == '0') {
		chip->hmac = false;
	} else {
		chip->hmac = true;
	}
	return len;
}

static ssize_t proc_hmac_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	char page[256] = {0};
	char read_data[3] = {0};
	int len = 0;

	if (NULL == chip)
		return  -EFAULT;

	if (true == chip->hmac) {
		read_data[0] = '1';
	} else {
		read_data[0] = '0';
	}
	read_data[1] = '\0';
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		chg_err("%s: copy_to_user error hmac = %d.\n", __func__, chip->hmac);
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

//for dx1 bringup
#if 0
static const struct file_operations hmac_proc_fops = {
	.write = proc_hmac_write,
	.read = proc_hmac_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops hmac_proc_fops = {
	.proc_write = proc_hmac_write,
	.proc_read = proc_hmac_read,
};
#endif
static ssize_t proc_charger_input_current_now_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[128] = {0};
	int len = 0;

	struct oplus_chg_chip *chip = g_charger_chip;
	if(!chip) {
		return 0;
		}
	sprintf(read_data, "%d", chip->ibus / 1000);
	len = sprintf(page, "%s", read_data);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static ssize_t proc_charger_passedchg_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[128] = {0};
	int len = 0;

	struct oplus_chg_chip *chip = g_charger_chip;
	if(!chip) {
		return 0;
	}

	sprintf(read_data, "%d", midas_chg.cali_passed_chg);
	len = sprintf(page, "%s", read_data);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static ssize_t proc_charger_passedchg_reset_count_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[128] = {0};
	int len = 0;

	struct oplus_chg_chip *chip = g_charger_chip;
	if(!chip) {
		return 0;
		}
	sprintf(read_data, "%d", midas_chg.reset_counts);
	len = sprintf(page, "%s", read_data);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

//for dx1 bringup
#if 0
static const struct file_operations proc_charger_input_current_now_ops =
{
	.read  = proc_charger_input_current_now_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops proc_charger_input_current_now_ops =
{
	.proc_read  = proc_charger_input_current_now_read,
};
#endif

//for dx1 bringup
#if 0
static const struct file_operations proc_charger_passedchg_ops =
{
	.read  = proc_charger_passedchg_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops proc_charger_passedchg_ops =
{
	.proc_read  = proc_charger_passedchg_read,
};
#endif

//for dx1 bringup
#if 0
static const struct file_operations proc_charger_passedchg_reset_count_ops =
{
	.read  = proc_charger_passedchg_reset_count_read,
	.owner = THIS_MODULE,
};
#else
static const struct proc_ops proc_charger_passedchg_reset_count_ops =
{
	.proc_read  = proc_charger_passedchg_reset_count_read,
};
#endif

static int init_charger_proc(struct oplus_chg_chip *chip)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_da = NULL;
	struct proc_dir_entry *prEntry_tmp = NULL;

	prEntry_da = proc_mkdir("charger", NULL);
	if (prEntry_da == NULL) {
		ret = -1;
		chg_debug("%s: Couldn't create charger proc entry\n",
			  __func__);
	}

	prEntry_tmp = proc_create_data("charger_factorymode_test", 0666, prEntry_da,
				       &proc_charger_factorymode_test_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -1;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("hmac", 0666, prEntry_da,
				       &hmac_proc_fops, chip);
	if (prEntry_tmp == NULL) {
		ret = -1;
		chg_debug("%s: Couldn't create hmac proc entry, %d\n", __func__,
			  __LINE__);
	}
	prEntry_tmp = proc_create_data("input_current_now", 0666, prEntry_da,
				       &proc_charger_input_current_now_ops, chip);
	prEntry_tmp = proc_create_data("passedchg", 0666, prEntry_da,
						   &proc_charger_passedchg_ops, chip);
	prEntry_tmp = proc_create_data("passedchg_reset_count", 0666, prEntry_da,
					   &proc_charger_passedchg_reset_count_ops, chip);
	return 0;
}

static ssize_t proc_ui_soc_decimal_write(struct file *filp,
               const char __user *buf, size_t len, loff_t *data)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	char buffer[2] = {0};

	if (NULL == chip)
		  return  -EFAULT;

	if (len > 2) {
		  return -EFAULT;
	}

	if (copy_from_user(buffer, buf, 2)) {
		  chg_err("%s:  error.\n", __func__);
		  return -EFAULT;
	}
	if (buffer[0] == '0') {
		  chip->boot_completed = false;
	} else {
		  chip->boot_completed = true;
	}
	pr_err("proc_ui_soc_decimal_write write");
	return len;
}
static ssize_t proc_ui_soc_decimal_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[128] = {0};
	int len = 0;
	int schedule_work = 0;
	int val;
	bool svooc_is_control_by_vooc;
	bool control_by_wireless = false;
	struct oplus_chg_chip *chip = g_charger_chip;

	if(!chip) {
		return 0;
	}

	if (chip->vooc_show_ui_soc_decimal) {
		svooc_is_control_by_vooc = (chip->vbatt_num == 2 && oplus_vooc_get_fast_chg_type() == CHARGER_SUBTYPE_FASTCHG_VOOC);
		control_by_wireless = chip->wireless_support &&
				(oplus_wpc_get_wireless_charge_start() ||
				oplus_chg_is_wls_present());
		if(svooc_is_control_by_vooc != true
				&& chip->boot_completed != false
				&& chip->calculate_decimal_time == 0
				&& control_by_wireless == false
				&& oplus_chg_show_vooc_logo_ornot() == true) {
			cancel_delayed_work_sync(&chip->ui_soc_decimal_work);
			oplus_chg_ui_soc_decimal_init();
			schedule_work = mod_delayed_work(system_wq, &chip->ui_soc_decimal_work, 0);
		}

		val = (chip->ui_soc_integer + chip->ui_soc_decimal) / 10;
		if(chip->decimal_control == false) {
			val = 0;
		}
	} else {
		val = 0;
	}

	sprintf(read_data, "%d, %d", chip->init_decimal_ui_soc / 10, val);
	pr_err("APK successful, %d,%d", chip->init_decimal_ui_soc / 10, val);
	len = sprintf(page, "%s", read_data);

	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

//for dx1 bringup
#if 0
static const struct file_operations ui_soc_decimal_ops =
{
    .write  = proc_ui_soc_decimal_write,
    .read = proc_ui_soc_decimal_read,
    .owner = THIS_MODULE,
};
#else
static const struct proc_ops ui_soc_decimal_ops =
{
    .proc_write  = proc_ui_soc_decimal_write,
    .proc_read = proc_ui_soc_decimal_read,
};
#endif

static int init_ui_soc_decimal_proc(struct oplus_chg_chip *chip)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_tmp = NULL;

	prEntry_tmp = proc_create_data("ui_soc_decimal", 0666, NULL,
				       &ui_soc_decimal_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -1;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}
	return 0;
}

void oplus_chg_ui_soc_decimal_init(void)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	if (oplus_vooc_get_fastchg_started() == true) {
		chip->batt_rm =  oplus_gauge_get_prev_remaining_capacity() * chip->vbatt_num;
		chip->batt_fcc =  oplus_gauge_get_prev_batt_fcc() * chip->vbatt_num;
	} else {
		chip->batt_rm =  oplus_gauge_get_remaining_capacity() * chip->vbatt_num;
		chip->batt_fcc =  oplus_gauge_get_batt_fcc() * chip->vbatt_num;
	}
	pr_err("[oplus_chg_ui_soc_decimal_init]!!!soc:%d", (int)((chip->batt_rm * 10000) / chip->batt_fcc));

	if(chip->ui_soc == 100) {
		chip->ui_soc_integer =  chip->ui_soc  *1000;
		chip->ui_soc_decimal = 0;
	} else {
		chip->ui_soc_integer =  chip->ui_soc  *1000;
		chip->ui_soc_decimal = chip->batt_rm * 100000 / chip->batt_fcc - (chip->batt_rm *100 / chip->batt_fcc) * 1000;
		if((chip->ui_soc_integer + chip->ui_soc_decimal) > chip->last_decimal_ui_soc && chip->last_decimal_ui_soc != 0) {
			chip->ui_soc_decimal = ((chip->last_decimal_ui_soc % 1000 - 50) > 0) ? (chip->last_decimal_ui_soc % 1000 - 50) : 0;
		}
	}
	chip->init_decimal_ui_soc = chip->ui_soc_integer + chip->ui_soc_decimal;
	if(chip->init_decimal_ui_soc > 100000) {
		chip->init_decimal_ui_soc = 100000;
		chip->ui_soc_integer = 100000;
		chip->ui_soc_decimal = 0;
	}
	chip->decimal_control = true;
	pr_err("[oplus_chg_ui_soc_decimal_init]!!! 2VBUS ui_soc_decimal:%d", chip->ui_soc_integer + chip->ui_soc_decimal);

	chip->calculate_decimal_time = 1;
}
void oplus_chg_ui_soc_decimal_deinit(void)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	chip->ui_soc_integer =(chip->ui_soc_integer + chip->ui_soc_decimal)/1000;
	if(chip->ui_soc_integer != 0) {
		chip->ui_soc = chip->ui_soc_integer;
	}
	chip->decimal_control = false;
	pr_err("[oplus_chg_ui_soc_decimal_deinit] ui_soc:%d", chip->ui_soc);
	chip->ui_soc_integer = 0;
	chip->ui_soc_decimal = 0;
	chip->init_decimal_ui_soc = 0;
}
#define MIN_DECIMAL_CURRENT 2000
static void oplus_chg_show_ui_soc_decimal(struct work_struct *work)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	int speed, icharging;
	int ratio = 1;
	/*update the battery data*/
	if (oplus_vooc_get_fastchg_started() == true) {
		chip->batt_rm =  oplus_gauge_get_prev_remaining_capacity() * chip->vbatt_num;
		chip->batt_fcc =  oplus_gauge_get_prev_batt_fcc() * chip->vbatt_num;
		chip->icharging = oplus_gauge_get_batt_current();
	} else {
		chip->batt_rm =  oplus_gauge_get_remaining_capacity() * chip->vbatt_num;
		chip->batt_fcc =  oplus_gauge_get_batt_fcc() * chip->vbatt_num;
		chip->icharging = oplus_gauge_get_batt_current();
	}
	icharging = chip->icharging * (-1);

	/*calculate the speed*/
	if(chip->ui_soc - chip->soc > 5) {
		ratio = 2;
	} else {
		ratio = 1;
	}
	if(icharging > 0) {
		speed = 100000 * icharging * UPDATE_TIME * chip->vbatt_num / (chip->batt_fcc * 3600) / ratio;
		pr_err("[oplus_chg_show_ui_soc_decimal] icharging = %d, batt_fcc :%d", chip->icharging, chip->batt_fcc);
	} else {
		/*speed = chip->ui_soc_decimal_speedmin / ratio;*/
		speed = 0;
		if(chip->batt_full) {
			speed = chip->ui_soc_decimal_speedmin;
		}
	}
	if (speed > 500) {
		speed = 500;
	}
	chip->ui_soc_decimal += speed;
	pr_err("[oplus_chg_ui_soc_decimal]chip->ui_soc_decimal+chip_ui_soc: %d , speed: %d, soc :%d\n ",
		(chip->ui_soc_decimal + chip->ui_soc_integer), speed , ((chip->batt_rm * 10000) / chip->batt_fcc));
	if(chip->ui_soc_integer + chip->ui_soc_decimal >= 100000) {
		chip->ui_soc_integer = 100000;
		chip->ui_soc_decimal = 0;
	}

	if(chip->calculate_decimal_time<= MAX_UI_DECIMAL_TIME) {
		chip->calculate_decimal_time++;
	   	schedule_delayed_work(&chip->ui_soc_decimal_work, msecs_to_jiffies(UPDATE_TIME * 1000));
	} else {
		oplus_chg_ui_soc_decimal_deinit();
	}
}

static int charging_limit_time_show(struct seq_file *seq_filp, void *v)
{
	seq_printf(seq_filp, "%d\n", g_charger_chip->limits.max_chg_time_sec);
	return 0;
}
static int charging_limit_time_open(struct inode *inode, struct file *file)
{
	int ret;
	ret = single_open(file, charging_limit_time_show, NULL);
	return ret;
}

static ssize_t charging_limit_time_write(struct file *filp,
		const char __user *buff, size_t len, loff_t *data)
{
	int limit_time;
	char temp[16];

	if (len > sizeof(temp)) {
		return -EINVAL;
	}
	if (copy_from_user(temp, buff, len)) {
		pr_err("charging_limit_time_write error.\n");
		return -EFAULT;
	}
	sscanf(temp, "%d", &limit_time);
	if (g_charger_chip) {
		g_charger_chip->limits.max_chg_time_sec = limit_time;
		printk(KERN_EMERG"charging_feature:max_chg_time_sec = %d\n",
			g_charger_chip->limits.max_chg_time_sec);
	}
	return len;
}

//for dx1 bringup
#if 0
static const struct file_operations charging_limit_time_fops = {
	.open = charging_limit_time_open,
	.write = charging_limit_time_write,
	.read = seq_read,
};
#else
static const struct proc_ops charging_limit_time_fops = {
	.proc_open = charging_limit_time_open,
	.proc_write = charging_limit_time_write,
	.proc_read = seq_read,
};
#endif
static int charging_limit_current_show(struct seq_file *seq_filp, void *v)
{
	seq_printf(seq_filp, "%d\n", g_charger_chip->limits.input_current_led_ma_high);
	seq_printf(seq_filp, "%d\n", g_charger_chip->limits.input_current_led_ma_warm);
	seq_printf(seq_filp, "%d\n", g_charger_chip->limits.input_current_led_ma_normal);
	return 0;
}
static int charging_limit_current_open(struct inode *inode, struct file *file)
{
	int ret;
	ret = single_open(file, charging_limit_current_show, NULL);
	return ret;
}

static ssize_t charging_limit_current_write(struct file *filp,
		const char __user *buff, size_t len, loff_t *data)
{
	int limit_current;
	char temp[16];

	if (len > sizeof(temp)) {
		return -EINVAL;
	}
	if (copy_from_user(temp, buff, len)) {
		pr_err("charging_limit_current_write error.\n");
		return -EFAULT;
	}
	sscanf(temp, "%d", &limit_current);
	if (g_charger_chip) {
		g_charger_chip->limits.input_current_led_ma_high = limit_current;
		g_charger_chip->limits.input_current_led_ma_warm = limit_current;
		g_charger_chip->limits.input_current_led_ma_normal = limit_current;
		printk(KERN_EMERG"charging_feature:limit_current = %d\n",limit_current);
	}
	return len;
}
//for dx1 bringup
#if 0
static const struct file_operations charging_limit_current_fops = {
	.open = charging_limit_current_open,
	.write = charging_limit_current_write,
	.read = seq_read,
};
#else
static const struct proc_ops charging_limit_current_fops = {
	.proc_open = charging_limit_current_open,
	.proc_write = charging_limit_current_write,
	.proc_read = seq_read,
};
#endif

static void init_proc_charging_feature(void)
{
	struct proc_dir_entry *p_time = NULL;
	struct proc_dir_entry *p_current = NULL;

	p_time = proc_create("charging_limit_time", 0664, NULL,
			&charging_limit_time_fops);
	if (!p_time) {
		pr_err("proc_create charging_feature_fops fail!\n");
	}
	p_current = proc_create("charging_limit_current", 0664, NULL,
			&charging_limit_current_fops);
	if (!p_current) {
		pr_err("proc_create charging_feature_fops fail!\n");
	}
}

/*ye.zhang add end*/
static void mmi_adapter_in_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_chip *chip
		= container_of(dwork, struct oplus_chg_chip, mmi_adapter_in_work);
	chip->mmi_fastchg = 1;
	charger_xlog_printk(CHG_LOG_CRTI, "  mmi_fastchg\n");
}

static void oplus_mmi_fastchg_in(struct oplus_chg_chip *chip)
{
	charger_xlog_printk(CHG_LOG_CRTI, "  call\n");
	schedule_delayed_work(&chip->mmi_adapter_in_work,
	round_jiffies_relative(msecs_to_jiffies(2000)));
}

static void oplus_chg_awake_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		return;
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	wake_lock_init(&chip->suspend_lock, WAKE_LOCK_SUSPEND, "battery suspend wakelock");

#else
	chip->suspend_ws = wakeup_source_register(NULL, "battery suspend wakelock");
#endif
}

static void oplus_chg_set_awake(struct oplus_chg_chip *chip, bool awake)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	if (chip->unwakelock_chg == 1 && awake == true) {
		charger_xlog_printk(CHG_LOG_CRTI,
			"error, unwakelock testing, can not set wakelock.\n");
		return;
	}

	if (awake){
		wake_lock(&chip->suspend_lock);
	} else {
		wake_unlock(&chip->suspend_lock);
	}
#else
	static bool pm_flag = false;

	if (chip->unwakelock_chg == 1 && awake == true) {
		charger_xlog_printk(CHG_LOG_CRTI,
			"error, unwakelock testing, can not set wakelock.\n");
		return;
	}

	if (!chip || !chip->suspend_ws)
		return;

	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->suspend_ws);
	} else if (!awake && pm_flag) {
		__pm_relax(chip->suspend_ws);
		pm_flag = false;
	}
#endif
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
static int __ref shortc_thread_main(void *data)
#else
static int shortc_thread_main(void *data)
#endif
{
	struct oplus_chg_chip *chip = data;
	int rc = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	struct cred *new;

	new = prepare_creds();
	if (!new) {
		chg_err("init err\n");
		rc = -1;
		return rc;
	}
	new->fsuid = new->euid = KUIDT_INIT(1000);
	commit_creds(new);
#endif
	while (!kthread_should_stop()) {
		set_current_state(TASK_RUNNING);
		oplus_chg_short_c_battery_check(chip);
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule();
	}
	return rc;
}

int oplus_chg_init(struct oplus_chg_chip *chip)
{
	int rc = 0;
	char *thread_name = "shortc_thread";

	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	struct power_supply *ac_psy;

	if (!chip->chg_ops) {
		dev_err(chip->dev, "charger operations cannot be NULL\n");
		return -1;
	}
	oplus_chg_variables_init(chip);
	oplus_get_smooth_soc_switch(chip);
	oplus_chg_get_battery_data(chip);
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_err(chip->dev, "USB psy not found; deferring probe\n");
		/*return -EPROBE_DEFER;*/
		goto power_psy_reg_failed;
	}

	chip->usb_psy = usb_psy;

	ac_psy = power_supply_get_by_name("ac");
	if (!ac_psy) {
		dev_err(chip->dev, "ac psy not found; deferring probe\n");
		goto power_psy_reg_failed;
	}
	chip->ac_psy = ac_psy;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		dev_err(chip->dev, "battery psy not found; deferring probe\n");
		goto power_psy_reg_failed;
	}
	chip->batt_psy = batt_psy;


#ifndef CONFIG_OPLUS_CHARGER_MTK
	chip->pmic_spmi.psy_registered = true;
#endif

	g_charger_chip = chip;
	oplus_chg_awake_init(chip);
	INIT_DELAYED_WORK(&chip->update_work, oplus_chg_update_work);
	INIT_DELAYED_WORK(&chip->ui_soc_decimal_work, oplus_chg_show_ui_soc_decimal);
	INIT_DELAYED_WORK(&chip->reset_adapter_work, oplus_chg_reset_adapter_work);
	chip->shortc_thread = kthread_create(shortc_thread_main, (void *)chip, thread_name);
	if (!chip->shortc_thread) {
		chg_err("Can't create shortc_thread\n");
		rc = -EPROBE_DEFER;
		goto power_psy_reg_failed;
	}

	oplus_pps_init(chip);

#ifdef CONFIG_FB
	chip->chg_fb_notify.notifier_call = fb_notifier_callback;
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
	rc = msm_drm_register_client(&chip->chg_fb_notify);
#else
	rc = fb_register_client(&chip->chg_fb_notify);
#endif /*CONFIG_DRM_MSM*/
	if (rc) {
		pr_err("Unable to register chg_fb_notify: %d\n", rc);
	}
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
	chip->chg_fb_notify.notifier_call = chg_mtk_drm_notifier_callback;
	if (mtk_disp_notifier_register("Oplus_Chg_v2", &chip->chg_fb_notify)) {
		chg_err("Failed to register disp notifier client!!\n");
	}
#endif

	oplus_chg_debug_info_init(chip);
	init_proc_chg_log();
	init_proc_chg_cycle();
	init_proc_critical_log();
	init_proc_tbatt_pwroff();
	init_proc_batt_param_noplug();

#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	init_proc_rtc_det();
	init_proc_vbat_low_det();
#endif

	init_proc_charging_feature();
	rc = init_ui_soc_decimal_proc(chip);
	rc = init_charger_proc(chip);
	/*ye.zhang add end*/
	schedule_delayed_work(&chip->update_work, OPLUS_CHG_UPDATE_INIT_DELAY);
	INIT_DELAYED_WORK(&chip->mmi_adapter_in_work, mmi_adapter_in_work_func);
	chip->shell_themal = thermal_zone_get_zone_by_name("shell_back");
	if (IS_ERR(chip->shell_themal)) {
		chg_err("Can't get shell_back\n");
	}
	charger_xlog_printk(CHG_LOG_CRTI, " end\n");
	return 0;

power_psy_reg_failed:
if (chip->ac_psy)
		power_supply_unregister(chip->ac_psy);
if (chip->usb_psy)
		power_supply_unregister(chip->usb_psy);
if (chip->batt_psy)
		power_supply_unregister(chip->batt_psy);
	charger_xlog_printk(CHG_LOG_CRTI, " Failed, rc = %d\n", rc);
	return rc;
}
EXPORT_SYMBOL(oplus_chg_init);

/*--------------------------------------------------------*/
int oplus_chg_parse_svooc_dt(struct oplus_chg_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,vbatt_num", &chip->vbatt_num);
	if (rc) {
		chip->vbatt_num = 1;
	}
	rc = of_property_read_u32(node, "qcom,vooc_project", &chip->vooc_project);
	if (rc < 0) {
		chip->vooc_project = 0;
	}
	chip->platform_fg_flag = of_property_read_bool(node, "qcom,platform_fg_flag");
	chg_err("oplus_parse_svooc_dt, chip->vbatt_num = %d,chip->vooc_project = %d.\n",
			chip->vbatt_num,chip->vooc_project);

	oplus_get_chg_ops_name_from_dt(node);

	return 0;
}
EXPORT_SYMBOL(oplus_chg_parse_svooc_dt);

int oplus_chg_parse_charger_dt(struct oplus_chg_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;
	int batt_cold_degree_negative;
	int batt_removed_degree_negative;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,input_current_charger_ma",
			&chip->limits.input_current_charger_ma);
	if (rc) {
		chip->limits.input_current_charger_ma
			= OPCHG_INPUT_CURRENT_LIMIT_CHARGER_MA;
	}
	rc = of_property_read_u32(node, "qcom,pd_input_current_charger_ma",
			&chip->limits.pd_input_current_charger_ma);
	if (rc) {
		chip->limits.pd_input_current_charger_ma
			= OPCHG_INPUT_CURRENT_LIMIT_CHARGER_MA;
	}
	rc = of_property_read_u32(node, "qcom,qc_input_current_charger_ma",
			&chip->limits.qc_input_current_charger_ma);
	if (rc) {
		chip->limits.qc_input_current_charger_ma
			= OPCHG_INPUT_CURRENT_LIMIT_CHARGER_MA;
	}

	rc = of_property_read_u32(node, "qcom,input_current_usb_ma",
			&chip->limits.input_current_usb_ma);
	if (rc) {
		chip->limits.input_current_usb_ma = OPCHG_INPUT_CURRENT_LIMIT_USB_MA;
	}

	rc = of_property_read_u32(node, "qcom,input_current_cdp_ma",
			&chip->limits.input_current_cdp_ma);
	if (rc) {
		chip->limits.input_current_cdp_ma = OPCHG_INPUT_CURRENT_LIMIT_USB_MA;
	}


	if (get_eng_version() == HIGH_TEMP_AGING) {
		chip->limits.input_current_led_ma = OPCHG_INPUT_CURRENT_LIMIT_CHARGER_MA;
		chg_err(" CONFIG_HIGH_TEMP_VERSION enable here,led on current 2A \n");
	} else {
		rc = of_property_read_u32(node, "qcom,input_current_led_ma",
			&chip->limits.input_current_led_ma);
		if (rc) {
			chip->limits.input_current_led_ma = OPCHG_INPUT_CURRENT_LIMIT_LED_MA;
		}
	}

	if (get_eng_version() == HIGH_TEMP_AGING) {
		chip->limits.input_current_led_ma_high = OPCHG_INPUT_CURRENT_LIMIT_CHARGER_MA;
		chg_err(" CONFIG_HIGH_TEMP_VERSION enable here, led_ma_high on current 2A \n");
	} else {
		rc = of_property_read_u32(node, "qcom,input_current_led_ma_high",
			&chip->limits.input_current_led_ma_high);
		if (rc) {
			chip->limits.input_current_led_ma_high = chip->limits.input_current_led_ma;
		}
	}

	rc = of_property_read_u32(node, "qcom,led_high_bat_decidegc",
			&chip->limits.led_high_bat_decidegc);
	if (rc) {
		chip->limits.led_high_bat_decidegc = 370;
	}

	if (get_eng_version() == HIGH_TEMP_AGING) {
		chip->limits.input_current_led_ma_warm = OPCHG_INPUT_CURRENT_LIMIT_CHARGER_MA;
		chg_err(" CONFIG_HIGH_TEMP_VERSION enable here, led_ma_warm on current 2A \n");
	} else {
		rc = of_property_read_u32(node, "qcom,input_current_led_ma_warm",
			&chip->limits.input_current_led_ma_warm);
		if (rc) {
			chip->limits.input_current_led_ma_warm = chip->limits.input_current_led_ma;
		}
	}

	rc = of_property_read_u32(node, "qcom,led_warm_bat_decidegc",
			&chip->limits.led_warm_bat_decidegc);
	if (rc) {
		chip->limits.led_warm_bat_decidegc = 350;
	}

	if (get_eng_version() == HIGH_TEMP_AGING) {
		chip->limits.input_current_led_ma_normal = OPCHG_INPUT_CURRENT_LIMIT_CHARGER_MA;
		chg_err(" CONFIG_HIGH_TEMP_VERSION enable here, led_ma_normal on current 2A \n");
	} else {
		rc = of_property_read_u32(node, "qcom,input_current_led_ma_normal",
			&chip->limits.input_current_led_ma_normal);
		if (rc) {
			chip->limits.input_current_led_ma_normal = chip->limits.input_current_led_ma;
		}
	}

	rc = of_property_read_u32(node, "qcom,input_current_camera_ma",
			&chip->limits.input_current_camera_ma);
	if (rc) {
		chip->limits.input_current_camera_ma = OPCHG_INPUT_CURRENT_LIMIT_CAMERA_MA;
	}
	chip->limits.iterm_disabled = of_property_read_bool(node, "qcom,iterm_disabled");
	rc = of_property_read_u32(node, "qcom,iterm_ma", &chip->limits.iterm_ma);
	if (rc < 0) {
		chip->limits.iterm_ma = -EINVAL;
	}
	chip->smart_charging_screenoff = of_property_read_bool(node, "qcom,smart_charging_screenoff");
	rc = of_property_read_u32(node, "qcom,input_current_calling_ma",
			&chip->limits.input_current_calling_ma);
	if (rc) {
		chip->limits.input_current_calling_ma = OPCHG_INPUT_CURRENT_LIMIT_CALLING_MA;
	}
	rc = of_property_read_u32(node, "qcom,recharge-mv",
			&chip->limits.recharge_mv);
	if (rc < 0) {
		chip->limits.recharge_mv = -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,usb_high_than_bat_decidegc",
			&chip->limits.usb_high_than_bat_decidegc);
	if (rc < 0) {
		chip->limits.usb_high_than_bat_decidegc = 100;
	}
	chg_err("usb_high_than_bat_decidegc:%d\n", chip->limits.usb_high_than_bat_decidegc);

	/*-19C*/
	rc = of_property_read_u32(node, "qcom,removed_bat_decidegc",
			&batt_removed_degree_negative);
	if (rc < 0) {
		chip->limits.removed_bat_decidegc = -19;
	} else {
		chip->limits.removed_bat_decidegc = -batt_removed_degree_negative;
	}
	/*-3~0 C*/

	if (get_eng_version() == HIGH_TEMP_AGING) {
		chg_err(" CONFIG_HIGH_TEMP_VERSION enable here,disable low tbat chg \n");
		batt_cold_degree_negative = 170;
		chip->limits.cold_bat_decidegc = -batt_cold_degree_negative;
	} else {
		chg_err(" CONFIG_HIGH_TEMP_VERSION disabled\n");
		rc = of_property_read_u32(node, "qcom,cold_bat_decidegc", &batt_cold_degree_negative);
		if (rc < 0) {
			chip->limits.cold_bat_decidegc = -EINVAL;
		} else {
			chip->limits.cold_bat_decidegc = -batt_cold_degree_negative;
		}
	}

	rc = of_property_read_u32(node, "qcom,temp_cold_vfloat_mv",
			&chip->limits.temp_cold_vfloat_mv);
	if (rc < 0) {
		chg_err(" temp_cold_vfloat_mv fail\n");
	}
	rc = of_property_read_u32(node, "qcom,temp_cold_fastchg_current_ma",
			&chip->limits.temp_cold_fastchg_current_ma);
	if (rc < 0) {
		chg_err(" temp_cold_fastchg_current_ma fail\n");
	}
	/*0~5 C*/
	rc = of_property_read_u32(node, "qcom,little_cold_bat_decidegc",
			&chip->limits.little_cold_bat_decidegc);
	if (rc < 0) {
		chip->limits.little_cold_bat_decidegc = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_little_cold_vfloat_mv",
			&chip->limits.temp_little_cold_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_little_cold_vfloat_mv = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_little_cold_fastchg_current_ma",
			&chip->limits.temp_little_cold_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.temp_little_cold_fastchg_current_ma = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_little_cold_fastchg_current_ma_high",
			&chip->limits.temp_little_cold_fastchg_current_ma_high);
	if (rc < 0) {
		chip->limits.temp_little_cold_fastchg_current_ma_high
			= chip->limits.temp_little_cold_fastchg_current_ma;
	}
	rc = of_property_read_u32(node, "qcom,temp_little_cold_fastchg_current_ma_low",
			&chip->limits.temp_little_cold_fastchg_current_ma_low);
	if (rc < 0) {
		chip->limits.temp_little_cold_fastchg_current_ma_low
			= chip->limits.temp_little_cold_fastchg_current_ma;
	}
	rc = of_property_read_u32(node, "qcom,pd_temp_little_cold_fastchg_current_ma_high",
			&chip->limits.pd_temp_little_cold_fastchg_current_ma_high);
	if (rc < 0) {
		chip->limits.pd_temp_little_cold_fastchg_current_ma_high
			= chip->limits.temp_little_cold_fastchg_current_ma_high;
	}
	rc = of_property_read_u32(node, "qcom,pd_temp_little_cold_fastchg_current_ma_low",
			&chip->limits.pd_temp_little_cold_fastchg_current_ma_low);
	if (rc < 0) {
		chip->limits.pd_temp_little_cold_fastchg_current_ma_low
			= chip->limits.temp_little_cold_fastchg_current_ma_low;
	}
	rc = of_property_read_u32(node, "qcom,qc_temp_little_cold_fastchg_current_ma_high",
			&chip->limits.qc_temp_little_cold_fastchg_current_ma_high);
	if (rc < 0) {
		chip->limits.qc_temp_little_cold_fastchg_current_ma_high
			= chip->limits.temp_little_cold_fastchg_current_ma_high;
	}
	rc = of_property_read_u32(node, "qcom,qc_temp_little_cold_fastchg_current_ma_low",
			&chip->limits.qc_temp_little_cold_fastchg_current_ma_low);
	if (rc < 0) {
		chip->limits.qc_temp_little_cold_fastchg_current_ma_low
			= chip->limits.temp_little_cold_fastchg_current_ma_low;
	}

	/*5~12 C*/
	rc = of_property_read_u32(node, "qcom,cool_bat_decidegc",
			&chip->limits.cool_bat_decidegc);
	if (rc < 0) {
		chip->limits.cool_bat_decidegc = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_cool_vfloat_mv",
			&chip->limits.temp_cool_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_cool_vfloat_mv = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_cool_fastchg_current_ma_high",
			&chip->limits.temp_cool_fastchg_current_ma_high);
	if (rc < 0) {
		chip->limits.temp_cool_fastchg_current_ma_high = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_cool_fastchg_current_ma_low",
			&chip->limits.temp_cool_fastchg_current_ma_low);
	if (rc < 0) {
		chip->limits.temp_cool_fastchg_current_ma_low = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,pd_temp_cool_fastchg_current_ma_high",
			&chip->limits.pd_temp_cool_fastchg_current_ma_high);
	if (rc < 0) {
		chip->limits.pd_temp_cool_fastchg_current_ma_high
			= chip->limits.temp_cool_fastchg_current_ma_high;
	}
	rc = of_property_read_u32(node, "qcom,pd_temp_cool_fastchg_current_ma_low",
			&chip->limits.pd_temp_cool_fastchg_current_ma_low);
	if (rc < 0) {
		chip->limits.pd_temp_cool_fastchg_current_ma_low
			= chip->limits.temp_cool_fastchg_current_ma_low;
	}
	rc = of_property_read_u32(node, "qcom,qc_temp_cool_fastchg_current_ma_high",
			&chip->limits.qc_temp_cool_fastchg_current_ma_high);
	if (rc < 0) {
		chip->limits.qc_temp_cool_fastchg_current_ma_high
			= chip->limits.temp_cool_fastchg_current_ma_high;
	}
	rc = of_property_read_u32(node, "qcom,qc_temp_cool_fastchg_current_ma_low",
			&chip->limits.qc_temp_cool_fastchg_current_ma_low);
	if (rc < 0) {
		chip->limits.qc_temp_cool_fastchg_current_ma_low
			= chip->limits.temp_cool_fastchg_current_ma_low;
	}

	/*12~16 C*/
	rc = of_property_read_u32(node, "qcom,little_cool_bat_decidegc",
			&chip->limits.little_cool_bat_decidegc);
	if (rc < 0) {
		chip->limits.little_cool_bat_decidegc = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_little_cool_vfloat_mv",
			&chip->limits.temp_little_cool_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_little_cool_vfloat_mv = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_little_cool_fastchg_current_ma",
			&chip->limits.temp_little_cool_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.temp_little_cool_fastchg_current_ma = -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,temp_little_cool_fastchg_current_ma_high",
			&chip->limits.temp_little_cool_fastchg_current_ma_high);
	if (rc < 0) {
		chip->limits.temp_little_cool_fastchg_current_ma_high = chip->limits.temp_little_cool_fastchg_current_ma;
	}

	rc = of_property_read_u32(node, "qcom,temp_little_cool_fastchg_current_ma_low",
			&chip->limits.temp_little_cool_fastchg_current_ma_low);
	if (rc < 0) {
		chip->limits.temp_little_cool_fastchg_current_ma_low = chip->limits.temp_little_cool_fastchg_current_ma;
	}

	rc = of_property_read_u32(node, "qcom,pd_temp_little_cool_fastchg_current_ma",
			&chip->limits.pd_temp_little_cool_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.pd_temp_little_cool_fastchg_current_ma
			= chip->limits.temp_little_cool_fastchg_current_ma;
	}
	rc = of_property_read_u32(node, "qcom,qc_temp_little_cool_fastchg_current_ma",
			&chip->limits.qc_temp_little_cool_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.qc_temp_little_cool_fastchg_current_ma
			= chip->limits.temp_little_cool_fastchg_current_ma;
	}

	/*16~45 C*/
	rc = of_property_read_u32(node, "qcom,normal_bat_decidegc",
			&chip->limits.normal_bat_decidegc);
	if (rc < 0) {
		chg_err(" normal_bat_decidegc fail\n");
	}
	rc = of_property_read_u32(node, "qcom,temp_normal_fastchg_current_ma",
			&chip->limits.temp_normal_fastchg_current_ma);
	if (rc) {
		chip->limits.temp_normal_fastchg_current_ma = OPCHG_FAST_CHG_MAX_MA;
	}
	rc = of_property_read_u32(node, "qcom,temp_normal_vfloat_mv",
			&chip->limits.temp_normal_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_normal_vfloat_mv = 4320;
	}
	rc = of_property_read_u32(node, "qcom,pd_temp_normal_fastchg_current_ma",
			&chip->limits.pd_temp_normal_fastchg_current_ma);
	if (rc) {
		chip->limits.pd_temp_normal_fastchg_current_ma = OPCHG_FAST_CHG_MAX_MA;
	}
	rc = of_property_read_u32(node, "qcom,qc_temp_normal_fastchg_current_ma",
			&chip->limits.qc_temp_normal_fastchg_current_ma);
	if (rc) {
		chip->limits.qc_temp_normal_fastchg_current_ma = OPCHG_FAST_CHG_MAX_MA;
	}

	/* 16C ~ 22C */
	rc = of_property_read_u32(node, "qcom,normal_phase1_bat_decidegc",
			&chip->limits.normal_phase1_bat_decidegc);
	if (rc < 0) {
		chg_err(" normal_phase1_bat_decidegc fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase1_vfloat_mv",
			&chip->limits.temp_normal_phase1_vfloat_mv);
	if (rc < 0) {
		chg_err(" temp_normal_phase1_vfloat_mv fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase1_fastchg_current_ma",
			&chip->limits.temp_normal_phase1_fastchg_current_ma);
	if (rc < 0) {
		chg_err(" temp_normal_phase1_fastchg_current_ma fail\n");
	}

	rc = of_property_read_u32(node, "qcom,normal_phase2_bat_decidegc",
			&chip->limits.normal_phase2_bat_decidegc);
	if (rc < 0) {
		chg_err(" normal_phase2_bat_decidegc fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase2_vfloat_mv",
			&chip->limits.temp_normal_phase2_vfloat_mv);
	if (rc < 0) {
		chg_err(" temp_normal_phase2_vfloat_mv fail\n");
	}

        rc = of_property_read_u32(node, "qcom,temp_normal_phase2_fastchg_current_ma_high",
			&chip->limits.temp_normal_phase2_fastchg_current_ma_high);
	if (rc < 0) {
		chg_err(" temp_normal_phase2_fastchg_current_ma_high fail\n");
	}


	rc = of_property_read_u32(node, "qcom,temp_normal_phase2_fastchg_current_ma_low",
			&chip->limits.temp_normal_phase2_fastchg_current_ma_low);
	if (rc < 0) {
		chg_err(" temp_normal_phase2_fastchg_current_ma_low fail\n");
	}

	rc = of_property_read_u32(node, "qcom,normal_phase3_bat_decidegc",
			&chip->limits.normal_phase3_bat_decidegc);
	if (rc < 0) {
		chg_err(" normal_phase3_bat_decidegc fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase3_vfloat_mv",
			&chip->limits.temp_normal_phase3_vfloat_mv);
	if (rc < 0) {
		chg_err(" temp_normal_phase3_vfloat_mv fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase3_fastchg_current_ma_high",
			&chip->limits.temp_normal_phase3_fastchg_current_ma_high);
	if (rc < 0) {
		chg_err(" temp_normal_phase3_fastchg_current_ma_high fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase3_fastchg_current_ma_low",
			&chip->limits.temp_normal_phase3_fastchg_current_ma_low);
	if (rc < 0) {
		chg_err(" temp_normal_phase3_fastchg_current_ma_low fail\n");
	}

	rc = of_property_read_u32(node, "qcom,normal_phase4_bat_decidegc",
			&chip->limits.normal_phase4_bat_decidegc);
	if (rc < 0) {
		chg_err(" normal_phase4_bat_decidegc fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase4_vfloat_mv",
			&chip->limits.temp_normal_phase4_vfloat_mv);
	if (rc < 0) {
		chg_err(" temp_normal_phase4_vfloat_mv fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase4_fastchg_current_ma_high",
			&chip->limits.temp_normal_phase4_fastchg_current_ma_high);
	if (rc < 0) {
		chg_err(" temp_normal_phase4_fastchg_current_ma_high fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase4_fastchg_current_ma_low",
			&chip->limits.temp_normal_phase4_fastchg_current_ma_low);
	if (rc < 0) {
		chg_err(" temp_normal_phase4_fastchg_current_ma_low fail\n");
	}

	rc = of_property_read_u32(node, "qcom,normal_phase5_bat_decidegc",
			&chip->limits.normal_phase5_bat_decidegc);
	if (rc < 0) {
		chg_err(" normal_phase5_bat_decidegc fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase5_vfloat_mv",
			&chip->limits.temp_normal_phase5_vfloat_mv);
	if (rc < 0) {
		chg_err(" temp_normal_phase5_vfloat_mv fail\n");
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase5_fastchg_current_ma",
			&chip->limits.temp_normal_phase5_fastchg_current_ma);
	if (rc < 0) {
		chg_err(" temp_normal_phase5_fastchg_current_ma fail\n");
	}

	rc = of_property_read_u32(node, "qcom,normal_phase6_bat_decidegc",
			&chip->limits.normal_phase6_bat_decidegc);
	if (rc < 0) {
		chip->limits.normal_phase6_bat_decidegc = 420;
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase6_vfloat_mv",
			&chip->limits.temp_normal_phase6_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_normal_phase6_vfloat_mv = chip->limits.temp_normal_phase5_vfloat_mv;
	}

	rc = of_property_read_u32(node, "qcom,temp_normal_phase6_fastchg_current_ma",
			&chip->limits.temp_normal_phase6_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.temp_normal_phase6_fastchg_current_ma = chip->limits.temp_normal_phase5_fastchg_current_ma;
	}

	charger_xlog_printk(CHG_LOG_CRTI,
			"normal_phase1_bat_decidegc = %d, \
			temp_normal_phase1_vfloat_mv = %d, \
			temp_normal_phase1_fastchg_current_ma = %d, \
			normal_phase2_bat_decidegc = %d, \
			temp_normal_phase2_vfloat_mv = %d, \
			temp_normal_phase2_fastchg_current_ma_high = %d, \
			temp_normal_phase2_fastchg_current_ma_low = %d, \
			normal_phase3_bat_decidegc = %d, \
			temp_normal_phase3_vfloat_mv = %d, \
			temp_normal_phase3_fastchg_current_ma_high = %d, \
			temp_normal_phase3_fastchg_current_ma_low = %d, \
			normal_phase4_bat_decidegc = %d, \
			temp_normal_phase4_vfloat_mv = %d, \
			temp_normal_phase4_fastchg_current_ma_high = %d, \
			temp_normal_phase4_fastchg_current_ma_low = %d, \
			normal_phase5_bat_decidegc = %d, \
			temp_normal_phase5_vfloat_mv = %d, \
			temp_normal_phase5_fastchg_current_ma = %d, \
			normal_phase6_bat_decidegc = %d, \
			temp_normal_phase6_vfloat_mv = %d, \
			temp_normal_phase6_fastchg_current_ma = %d\n",
			chip->limits.normal_phase1_bat_decidegc,
			chip->limits.temp_normal_phase1_vfloat_mv,
			chip->limits.temp_normal_phase1_fastchg_current_ma,
			chip->limits.normal_phase2_bat_decidegc,
			chip->limits.temp_normal_phase2_vfloat_mv,
			chip->limits.temp_normal_phase2_fastchg_current_ma_high,
			chip->limits.temp_normal_phase2_fastchg_current_ma_low,
			chip->limits.normal_phase3_bat_decidegc,
			chip->limits.temp_normal_phase3_vfloat_mv,
			chip->limits.temp_normal_phase3_fastchg_current_ma_high,
			chip->limits.temp_normal_phase3_fastchg_current_ma_low,
			chip->limits.normal_phase4_bat_decidegc,
			chip->limits.temp_normal_phase4_vfloat_mv,
			chip->limits.temp_normal_phase4_fastchg_current_ma_high,
			chip->limits.temp_normal_phase4_fastchg_current_ma_low,
			chip->limits.normal_phase5_bat_decidegc,
			chip->limits.temp_normal_phase5_vfloat_mv,
			chip->limits.temp_normal_phase5_fastchg_current_ma,
			chip->limits.normal_phase6_bat_decidegc,
			chip->limits.temp_normal_phase6_vfloat_mv,
			chip->limits.temp_normal_phase6_fastchg_current_ma);

	/*45~55 C*/
	rc = of_property_read_u32(node, "qcom,warm_bat_decidegc",
			&chip->limits.warm_bat_decidegc);
	if (rc < 0) {
		chip->limits.warm_bat_decidegc = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_warm_vfloat_mv",
			&chip->limits.temp_warm_vfloat_mv);
	if (rc < 0) {
		chip->limits.temp_warm_vfloat_mv = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,temp_warm_fastchg_current_ma",
			&chip->limits.temp_warm_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.temp_warm_fastchg_current_ma = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,pd_temp_warm_fastchg_current_ma",
			&chip->limits.pd_temp_warm_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.pd_temp_warm_fastchg_current_ma
			= chip->limits.temp_warm_fastchg_current_ma;
	}
	rc = of_property_read_u32(node, "qcom,qc_temp_warm_fastchg_current_ma",
			&chip->limits.qc_temp_warm_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.qc_temp_warm_fastchg_current_ma
			= chip->limits.temp_warm_fastchg_current_ma;
	}
	rc = of_property_read_u32(node, "qcom,temp_warm_fastchg_current_ma_led_on",
			&chip->limits.temp_warm_fastchg_current_ma_led_on);
	if (rc < 0) {
		chip->limits.temp_warm_fastchg_current_ma_led_on
			= chip->limits.temp_warm_fastchg_current_ma;
	}

	/*>55 C*/
	rc = of_property_read_u32(node, "qcom,hot_bat_decidegc",
			&chip->limits.hot_bat_decidegc);
	if (rc < 0) {
		chip->limits.hot_bat_decidegc = -EINVAL;
	}
	/*offset temperature, only for userspace, default 0*/
	rc = of_property_read_u32(node, "qcom,offset_temp", &chip->offset_temp);
	if (rc < 0) {
		chip->offset_temp = 0;
	}
	/*non standard battery*/
	rc = of_property_read_u32(node, "qcom,non_standard_vfloat_mv",
			&chip->limits.non_standard_vfloat_mv);
	if (rc < 0) {
		chip->limits.non_standard_vfloat_mv = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,non_standard_fastchg_current_ma",
			&chip->limits.non_standard_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.non_standard_fastchg_current_ma = -EINVAL;
	}
	/*short circuit battery*/
	rc = of_property_read_u32(node, "qcom,short_c_bat_cv_mv",
			&chip->short_c_batt.short_c_bat_cv_mv);
	if (rc < 0) {
		chip->short_c_batt.short_c_bat_cv_mv = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,short_c_bat_vfloat_mv",
			&chip->limits.short_c_bat_vfloat_mv);
	if (rc < 0) {
		chip->limits.short_c_bat_vfloat_mv = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,short_c_bat_fastchg_current_ma",
			&chip->limits.short_c_bat_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.short_c_bat_fastchg_current_ma = -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,short_c_bat_vfloat_sw_limit",
			&chip->limits.short_c_bat_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.short_c_bat_vfloat_sw_limit = -EINVAL;
	}

	/*vfloat_sw_limit*/
	rc = of_property_read_u32(node, "qcom,non_standard_vfloat_sw_limit",
			&chip->limits.non_standard_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.non_standard_vfloat_sw_limit = 3960;
	}
	rc = of_property_read_u32(node, "qcom,cold_vfloat_sw_limit",
			&chip->limits.cold_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.cold_vfloat_sw_limit = 3960;
	}
	rc = of_property_read_u32(node, "qcom,little_cold_vfloat_sw_limit",
			&chip->limits.little_cold_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.little_cold_vfloat_sw_limit = 4330;
	}
	rc = of_property_read_u32(node, "qcom,cool_vfloat_sw_limit",
			&chip->limits.cool_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.cool_vfloat_sw_limit = 4330;
	}
	rc = of_property_read_u32(node, "qcom,little_cool_vfloat_sw_limit",
			&chip->limits.little_cool_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.little_cool_vfloat_sw_limit = 4330;
	}
	rc = of_property_read_u32(node, "qcom,normal_vfloat_sw_limit",
			&chip->limits.normal_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.normal_vfloat_sw_limit = 4330;
	}
	rc = of_property_read_u32(node, "qcom,warm_vfloat_sw_limit",
			&chip->limits.warm_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.warm_vfloat_sw_limit = 4060;
	}

	/*vfloat_over_sw_limit*/
	chip->limits.sw_vfloat_over_protect_enable = of_property_read_bool(node,
			"qcom,sw_vfloat_over_protect_enable");
	rc = of_property_read_u32(node, "qcom,non_standard_vfloat_over_sw_limit",
			&chip->limits.non_standard_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.non_standard_vfloat_over_sw_limit = 3980;
	}
	rc = of_property_read_u32(node, "qcom,cold_vfloat_over_sw_limit",
			&chip->limits.cold_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.cold_vfloat_over_sw_limit = 3980;
	}
	rc = of_property_read_u32(node, "qcom,little_cold_vfloat_over_sw_limit",
			&chip->limits.little_cold_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.little_cold_vfloat_over_sw_limit = 4390;
	}
	rc = of_property_read_u32(node, "qcom,cool_vfloat_over_sw_limit",
			&chip->limits.cool_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.cool_vfloat_over_sw_limit = 4390;
	}
	rc = of_property_read_u32(node, "qcom,little_cool_vfloat_over_sw_limit",
			&chip->limits.little_cool_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.little_cool_vfloat_over_sw_limit = 4390;
	}
	rc = of_property_read_u32(node, "qcom,normal_vfloat_over_sw_limit",
			&chip->limits.normal_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.normal_vfloat_over_sw_limit = 4390;
	}
	rc = of_property_read_u32(node, "qcom,warm_vfloat_over_sw_limit",
			&chip->limits.warm_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.warm_vfloat_over_sw_limit = 4080;
		}
	rc = of_property_read_u32(node, "qcom,charger_hv_thr",
			&chip->limits.charger_hv_thr);
	if (rc < 0) {
		chip->limits.charger_hv_thr = 5800;
	}
	rc = of_property_read_u32(node, "qcom,charger_recv_thr",
			&chip->limits.charger_recv_thr);
	if (rc < 0) {
		chip->limits.charger_recv_thr = 5800;
	}
	rc = of_property_read_u32(node, "qcom,charger_lv_thr",
			&chip->limits.charger_lv_thr);
	if (rc < 0) {
		chip->limits.charger_lv_thr = 3400;
	}
	rc = of_property_read_u32(node, "qcom,vbatt_full_thr",
			&chip->limits.vbatt_full_thr);
	if (rc < 0) {
		chip->limits.vbatt_full_thr = 4400;
	}
	rc = of_property_read_u32(node, "qcom,vbatt_hv_thr",
			&chip->limits.vbatt_hv_thr);
	if (rc < 0) {
		chip->limits.vbatt_hv_thr = 4500;
	}
	rc = of_property_read_u32(node, "qcom,vfloat_step_mv",
			&chip->limits.vfloat_step_mv);
	if (rc < 0) {
		chip->limits.vfloat_step_mv = 16;
	}
	rc = of_property_read_u32(node, "qcom,vbatt_power_off",
			&chip->vbatt_power_off);
	if (rc < 0) {
		chip->vbatt_power_off = 3300;
	}
	rc = of_property_read_u32(node, "qcom,vbatt_soc_1",
			&chip->vbatt_soc_1);
	if (rc < 0) {
		chip->vbatt_soc_1 = 3410;
	}
	rc = of_property_read_u32(node, "qcom,soc_to_0_withchg",
			&chip->soc_to_0_withchg);
	if (rc < 0) {
		chip->soc_to_0_withchg = 3200;
	}
	rc = of_property_read_u32(node, "qcom,soc_to_0_withoutchg",
			&chip->soc_to_0_withoutchg);
	if (rc < 0) {
		chip->soc_to_0_withoutchg = 3300;
	}
	rc = of_property_read_u32(node, "qcom,normal_vterm_hw_inc",
			&chip->limits.normal_vterm_hw_inc);
	if (rc < 0) {
		chip->limits.normal_vterm_hw_inc = 18;
	}
	rc = of_property_read_u32(node, "qcom,non_normal_vterm_hw_inc",
			&chip->limits.non_normal_vterm_hw_inc);
	if (rc < 0) {
		chip->limits.non_normal_vterm_hw_inc = 18;
	}
	rc = of_property_read_u32(node, "qcom,vbatt_pdqc_to_5v_thr",
			&chip->limits.vbatt_pdqc_to_5v_thr);
	if (rc < 0) {
		chip->limits.vbatt_pdqc_to_5v_thr = -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,vbatt_pdqc_to_9v_thr",
			&chip->limits.vbatt_pdqc_to_9v_thr);
	if (rc < 0) {
		chip->limits.vbatt_pdqc_to_9v_thr = 4000;
	}

	charger_xlog_printk(CHG_LOG_CRTI,
			"vbatt_power_off = %d, \
			vbatt_soc_1 = %d, \
			normal_vterm_hw_inc = %d, \
			, \
			non_normal_vterm_hw_inc = %d, \
			vbatt_pdqc_to_9v_thr = %d, \
			vbatt_pdqc_to_5v_thr = %d\n",
			chip->vbatt_power_off,
			chip->vbatt_soc_1,
			chip->limits.normal_vterm_hw_inc,
			chip->limits.non_normal_vterm_hw_inc,
			chip->limits.vbatt_pdqc_to_9v_thr,
			chip->limits.vbatt_pdqc_to_5v_thr);

	rc = of_property_read_u32(node, "qcom,ff1_normal_fastchg_ma",
			&chip->limits.ff1_normal_fastchg_ma);
	if (rc) {
		chip->limits.ff1_normal_fastchg_ma = 1000;
	}
	rc = of_property_read_u32(node, "qcom,ff1_warm_fastchg_ma",
			&chip->limits.ff1_warm_fastchg_ma);
	if (rc) {
		chip->limits.ff1_warm_fastchg_ma = chip->limits.ff1_normal_fastchg_ma;
	}
	rc = of_property_read_u32(node, "qcom,ffc2_temp_warm_decidegc",
			&chip->limits.ffc2_temp_warm_decidegc);
	if (rc) {
		chip->limits.ffc2_temp_warm_decidegc = 350;
	}
	rc = of_property_read_u32(node, "qcom,ffc2_temp_high_decidegc",
			&chip->limits.ffc2_temp_high_decidegc);
	if (rc) {
		chip->limits.ffc2_temp_high_decidegc = 400;
	}
	rc = of_property_read_u32(node, "qcom,ffc2_temp_low_decidegc",
			&chip->limits.ffc2_temp_low_decidegc);
	if (rc) {
		chip->limits.ffc2_temp_low_decidegc = 160;
	}
		rc = of_property_read_u32(node, "qcom,ffc2_normal_fastchg_ma",
			&chip->limits.ffc2_normal_fastchg_ma);
	if (rc < 0) {
		chip->limits.ffc2_normal_fastchg_ma = 700;
	}
		rc = of_property_read_u32(node, "qcom,ffc2_warm_fastchg_ma",
			&chip->limits.ffc2_warm_fastchg_ma);
	if (rc < 0) {
		chip->limits.ffc2_warm_fastchg_ma = 750;
	}
		rc = of_property_read_u32(node, "qcom,ffc2_exit_step_ma",
			&chip->limits.ffc2_exit_step_ma);
	if (rc < 0) {
		chip->limits.ffc2_exit_step_ma = 100;
	}
		rc = of_property_read_u32(node, "qcom,ffc2_warm_exit_step_ma",
			&chip->limits.ffc2_warm_exit_step_ma);
	if (rc < 0) {
		chip->limits.ffc2_warm_exit_step_ma = chip->limits.ffc2_exit_step_ma;
	}
	rc = of_property_read_u32(node, "qcom,ff1_exit_step_ma",
			&chip->limits.ff1_exit_step_ma);
	if (rc < 0) {
		chip->limits.ff1_exit_step_ma = 400;
	}
	rc = of_property_read_u32(node, "qcom,ff1_warm_exit_step_ma",
			&chip->limits.ff1_warm_exit_step_ma);
	if (rc < 0) {
		chip->limits.ff1_warm_exit_step_ma = 350;
	}
	rc = of_property_read_u32(node, "qcom,ffc_normal_vfloat_sw_limit",
			&chip->limits.ffc1_normal_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.ffc1_normal_vfloat_sw_limit = 4450;
	}
	rc = of_property_read_u32(node, "qcom,ffc_warm_vfloat_sw_limit",
			&chip->limits.ffc1_warm_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.ffc1_warm_vfloat_sw_limit
			= chip->limits.ffc1_normal_vfloat_sw_limit;
	}
	rc = of_property_read_u32(node, "qcom,ffc2_normal_vfloat_sw_limit",
			&chip->limits.ffc2_normal_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.ffc2_normal_vfloat_sw_limit
			= chip->limits.ffc1_normal_vfloat_sw_limit;
	}
	rc = of_property_read_u32(node, "qcom,ffc2_warm_vfloat_sw_limit",
			&chip->limits.ffc2_warm_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.ffc2_warm_vfloat_sw_limit
			= chip->limits.ffc2_normal_vfloat_sw_limit;
	}
	rc = of_property_read_u32(node, "qcom,ffc_temp_normal_vfloat_mv",
			&chip->limits.ffc_temp_normal_vfloat_mv);
	if (rc < 0) {
		chip->limits.ffc_temp_normal_vfloat_mv = 4500;
	}
	rc = of_property_read_u32(node, "qcom,ffc1_temp_normal_vfloat_mv",
			&chip->limits.ffc1_temp_normal_vfloat_mv);
	if (rc < 0) {
		chip->limits.ffc1_temp_normal_vfloat_mv
			= chip->limits.ffc_temp_normal_vfloat_mv;
	}
	rc = of_property_read_u32(node, "qcom,ffc2_temp_normal_vfloat_mv",
			&chip->limits.ffc2_temp_normal_vfloat_mv);
	if (rc < 0) {
		chip->limits.ffc2_temp_normal_vfloat_mv
			= chip->limits.ffc_temp_normal_vfloat_mv;
	}
	rc = of_property_read_u32(node, "qcom,ffc_normal_vfloat_over_sw_limit",
			&chip->limits.ffc_normal_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.ffc_normal_vfloat_over_sw_limit = 4500;
	}
	rc = of_property_read_u32(node, "qcom,ffc1_normal_vfloat_over_sw_limit",
			&chip->limits.ffc1_normal_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.ffc1_normal_vfloat_over_sw_limit
			= chip->limits.ffc_normal_vfloat_over_sw_limit;
	}
	rc = of_property_read_u32(node, "qcom,ffc2_normal_vfloat_over_sw_limit",
			&chip->limits.ffc2_normal_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.ffc2_normal_vfloat_over_sw_limit
			= chip->limits.ffc_normal_vfloat_over_sw_limit;
	}

	charger_xlog_printk(CHG_LOG_CRTI,
			"ff1_normal_fastchg_ma = %d, \
			ffc2_temp_warm_decidegc = %d, \
			ffc2_temp_high_decidegc = %d, \
			ffc2_normal_fastchg_ma = %d, \
			chip->limits.ffc2_warm_fastchg_ma = %d, \
			ffc2_exit_step_ma = %d, \
			ffc_normal_vfloat_sw_limit = %d, \
			ffc_warm_vfloat_sw_limit = %d, \
			ffc2_normal_vfloat_sw_limit = %d, \
			ffc2_warm_vfloat_sw_limit = %d, \
			ffc1_temp_normal_vfloat_mv = %d, \
			ffc2_temp_normal_vfloat_mv = %d, \
			ffc_normal_vfloat_over_sw_limit = %d \
			ffc2_temp_low_decidegc = %d \
			limits.ff1_exit_step_ma = %d \
			limits.ff1_warm_exit_step_ma = %d \
			pd_input_current_charger_ma = %d \
			qc_input_current_charger_ma = %d\n",
			chip->limits.ff1_normal_fastchg_ma,
			chip->limits.ffc2_temp_warm_decidegc,
			chip->limits.ffc2_temp_high_decidegc,
			chip->limits.ffc2_normal_fastchg_ma,
			chip->limits.ffc2_warm_fastchg_ma,
			chip->limits.ffc2_exit_step_ma,
			chip->limits.ffc1_normal_vfloat_sw_limit,
			chip->limits.ffc1_warm_vfloat_sw_limit,
			chip->limits.ffc2_normal_vfloat_sw_limit,
			chip->limits.ffc2_warm_vfloat_sw_limit,
			chip->limits.ffc1_temp_normal_vfloat_mv,
			chip->limits.ffc2_temp_normal_vfloat_mv,
			chip->limits.ffc_normal_vfloat_over_sw_limit,
			chip->limits.ffc2_temp_low_decidegc,
			chip->limits.ff1_exit_step_ma,
			chip->limits.ff1_warm_exit_step_ma,
			chip->limits.pd_input_current_charger_ma,
			chip->limits.qc_input_current_charger_ma);

	rc = of_property_read_u32(node, "qcom,default_iterm_ma",
			&chip->limits.default_iterm_ma);
	if (rc < 0) {
		chip->limits.default_iterm_ma = 100;
	}
	rc = of_property_read_u32(node, "qcom,default_temp_normal_fastchg_current_ma",
			&chip->limits.default_temp_normal_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.default_temp_normal_fastchg_current_ma = OPCHG_FAST_CHG_MAX_MA;
	}
	rc = of_property_read_u32(node, "qcom,default_normal_vfloat_sw_limit",
			&chip->limits.default_normal_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.default_normal_vfloat_sw_limit = 4350;
	}
	rc = of_property_read_u32(node, "qcom,default_temp_normal_vfloat_mv",
			&chip->limits.default_temp_normal_vfloat_mv);
	if (rc < 0) {
		chip->limits.default_temp_normal_vfloat_mv = 4370;
	}
	rc = of_property_read_u32(node, "qcom,default_normal_vfloat_over_sw_limit",
			&chip->limits.default_normal_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.default_normal_vfloat_over_sw_limit = 4373;
	}

	charger_xlog_printk(CHG_LOG_CRTI,
			"default_iterm_ma = %d, \
			default_temp_normal_fastchg_current_ma = %d, \
			default_normal_vfloat_sw_limit = %d, \
			default_temp_normal_vfloat_mv = %d, \
			default_normal_vfloat_over_sw_limit = %d\n",
			chip->limits.default_iterm_ma,
			chip->limits.default_temp_normal_fastchg_current_ma,
			chip->limits.default_normal_vfloat_sw_limit,
			chip->limits.default_temp_normal_vfloat_mv,
			chip->limits.default_normal_vfloat_over_sw_limit);

	rc = of_property_read_u32(node, "qcom,default_temp_little_cool_fastchg_current_ma",
			&chip->limits.default_temp_little_cool_fastchg_current_ma);
	if (rc < 0) {
		chip->limits.default_temp_little_cool_fastchg_current_ma
			= chip->limits.temp_little_cool_fastchg_current_ma;
	}
	rc = of_property_read_u32(node, "qcom,default_little_cool_vfloat_sw_limit",
			&chip->limits.default_little_cool_vfloat_sw_limit);
	if (rc < 0) {
		chip->limits.default_little_cool_vfloat_sw_limit
			= chip->limits.little_cool_vfloat_sw_limit;
	}
	rc = of_property_read_u32(node, "qcom,default_temp_little_cool_vfloat_mv",
			&chip->limits.default_temp_little_cool_vfloat_mv);
	if (rc < 0) {
		chip->limits.default_temp_little_cool_vfloat_mv
			= chip->limits.temp_little_cool_vfloat_mv;
	}
	rc = of_property_read_u32(node, "qcom,default_little_cool_vfloat_over_sw_limit",
			&chip->limits.default_little_cool_vfloat_over_sw_limit);
	if (rc < 0) {
		chip->limits.default_little_cool_vfloat_over_sw_limit
			= chip->limits.little_cool_vfloat_over_sw_limit;
	}

	charger_xlog_printk(CHG_LOG_CRTI,
			"default_temp_little_cool_fastchg_current_ma = %d, \
			default_little_cool_vfloat_sw_limit = %d, \
			default_temp_little_cool_vfloat_mv = %d, \
			default_little_cool_vfloat_over_sw_limit = %d\n",
			chip->limits.default_temp_little_cool_fastchg_current_ma,
			chip->limits.default_little_cool_vfloat_sw_limit,
			chip->limits.default_temp_little_cool_vfloat_mv,
			chip->limits.default_little_cool_vfloat_over_sw_limit);

	chip->limits.default_temp_little_cold_fastchg_current_ma_high
			= chip->limits.temp_little_cold_fastchg_current_ma_high;
	chip->limits.default_temp_little_cold_fastchg_current_ma_low
			= chip->limits.temp_little_cold_fastchg_current_ma_low;
	chip->limits.default_temp_cool_fastchg_current_ma_high
			= chip->limits.temp_cool_fastchg_current_ma_high;
	chip->limits.default_temp_cool_fastchg_current_ma_low
			= chip->limits.temp_cool_fastchg_current_ma_low;
	chip->limits.default_temp_little_cool_fastchg_current_ma_high
			= chip->limits.temp_little_cool_fastchg_current_ma_high;
	chip->limits.default_temp_little_cool_fastchg_current_ma_low
			= chip->limits.temp_little_cool_fastchg_current_ma_low;
	chip->limits.default_temp_warm_fastchg_current_ma
			= chip->limits.temp_warm_fastchg_current_ma;
	chip->limits.default_input_current_charger_ma
			= chip->limits.input_current_charger_ma;
	rc = of_property_read_u32(node, "qcom,batt_capacity_mah",
			&chip->batt_capacity_mah);
	if (rc < 0) {
		chip->batt_capacity_mah = 2000;
	}

	chip->chg_ctrl_by_vooc = of_property_read_bool(node, "qcom,chg_ctrl_by_vooc");
	chip->chg_ctrl_by_vooc_default = of_property_read_bool(node, "qcom,chg_ctrl_by_vooc");

	rc = of_property_read_u32(node, "qcom,input_current_vooc_ma_normal",
			&chip->limits.input_current_vooc_ma_normal);
	if (rc) {
		chip->limits.input_current_vooc_ma_normal = 3600;
	}
	rc = of_property_read_u32(node, "qcom,input_current_vooc_led_ma_high",
			&chip->limits.input_current_vooc_led_ma_high);
	if (rc) {
		chip->limits.input_current_vooc_led_ma_high = 1800;
	}
	rc = of_property_read_u32(node, "qcom,input_current_vooc_led_ma_warm",
			&chip->limits.input_current_vooc_led_ma_warm);
	if (rc) {
		chip->limits.input_current_vooc_led_ma_warm = 1800;
	}
	rc = of_property_read_u32(node, "qcom,input_current_vooc_led_ma_normal",
			&chip->limits.input_current_vooc_led_ma_normal);
	if (rc) {
		chip->limits.input_current_vooc_led_ma_normal = 1800;
	}
	rc = of_property_read_u32(node, "qcom,vooc_temp_bat_normal_decidegc",
		&chip->limits.vooc_normal_bat_decidegc);
	if (rc) {
		chip->limits.vooc_normal_bat_decidegc = 340;
	}
	rc = of_property_read_u32(node, "qcom,input_current_vooc_ma_warm",
			&chip->limits.input_current_vooc_ma_warm);
	if (rc) {
		chip->limits.input_current_vooc_ma_warm = 3000;
	}
	rc = of_property_read_u32(node, "qcom,vooc_temp_bat_warm_decidegc",
			&chip->limits.vooc_warm_bat_decidegc);
	if (rc) {
		chip->limits.vooc_warm_bat_decidegc = 380;
	}
	rc = of_property_read_u32(node, "qcom,input_current_vooc_ma_high",
			&chip->limits.input_current_vooc_ma_high);
	if (rc) {
		chip->limits.input_current_vooc_ma_high = 2600;
	}
	rc = of_property_read_u32(node, "qcom,vooc_temp_bat_hot_decidegc",
			&chip->limits.vooc_high_bat_decidegc);
	if (rc) {
		chip->limits.vooc_high_bat_decidegc = 450;
	}
	rc = of_property_read_u32(node, "qcom,charger_current_vooc_ma_normal",
			&chip->limits.charger_current_vooc_ma_normal);
	if (rc) {
		chip->limits.charger_current_vooc_ma_normal = 1000;
	}

	chip->limits.default_input_current_vooc_ma_high
			= chip->limits.input_current_vooc_ma_high;
	chip->limits.default_input_current_vooc_ma_warm
			= chip->limits.input_current_vooc_ma_warm;
	chip->limits.default_input_current_vooc_ma_normal
			= chip->limits.input_current_vooc_ma_normal;
	chip->limits.default_pd_input_current_charger_ma
			= chip->limits.pd_input_current_charger_ma;
	chip->limits.default_qc_input_current_charger_ma
			= chip->limits.qc_input_current_charger_ma;
	chip->suspend_after_full = of_property_read_bool(node, "qcom,suspend_after_full");
	chip->check_batt_full_by_sw = of_property_read_bool(node, "qcom,check_batt_full_by_sw");
	chip->external_gauge = of_property_read_bool(node, "qcom,external_gauge");
	chip->fg_bcl_poll = of_property_read_bool(node, "qcom,fg_bcl_poll_enable");

	chip->wireless_support = of_property_read_bool(node, "qcom,wireless_support");
	chip->wpc_no_chargerpump = of_property_read_bool(node, "qcom,wpc_no_chargerpump");
	chip->chg_ctrl_by_lcd = of_property_read_bool(node, "qcom,chg_ctrl_by_lcd");
	chip->chg_ctrl_by_lcd_default = of_property_read_bool(node, "qcom,chg_ctrl_by_lcd");
	chip->chg_ctrl_by_camera = of_property_read_bool(node, "qcom,chg_ctrl_by_camera");
	chip->bq25890h_flag = of_property_read_bool(node,"qcom,bq25890_flag");
	chip->chg_ctrl_by_calling = of_property_read_bool(node, "qcom,chg_ctrl_by_calling");
	chip->ffc_support = of_property_read_bool(node, "qcom,ffc_support");
	chip->dual_ffc = of_property_read_bool(node, "qcom,dual_ffc");
	chip->new_ui_warning_support = of_property_read_bool(node, "qcom,new_ui_warning_support");
	chip->recharge_after_full = of_property_read_bool(node, "recharge_after_full");
	chip->smooth_switch = of_property_read_bool(node, "qcom,smooth_switch");
	chip->limits.max_chg_time_sec = chip->batt_capacity_mah / 250 * 3600;
	rc = of_property_read_u32(node, "qcom,voocphy_support", &chip->voocphy_support);
	if (rc) {
		chip->voocphy_support = NO_VOOCPHY;
	}

	charger_xlog_printk(CHG_LOG_CRTI,
			"input_current_charger_ma = %d, \
			input_current_usb_ma = %d, \
			input_current_led_ma = %d, \
			input_current_led_ma_normal = %d, \
			input_current_led_ma_warm = %d, \
			input_current_led_ma_high = %d, \
			temp_normal_fastchg_current_ma = %d, \
			temp_normal_vfloat_mv = %d, \
			iterm_ma = %d, \
			recharge_mv = %d, \
			cold_bat_decidegc = %d, \
			temp_cold_vfloat_mv = %d, \
			temp_cold_fastchg_current_ma = %d, \
			little_cold_bat_decidegc = %d, \
			temp_little_cold_vfloat_mv = %d, \
			temp_little_cold_fastchg_current_ma = %d, \
			cool_bat_decidegc = %d, \
			temp_cool_vfloat_mv = %d, \
			temp_cool_fastchg_current_ma_high = %d, \
			temp_cool_fastchg_current_ma_low = %d, \
			little_cool_bat_decidegc = %d, \
			temp_little_cool_vfloat_mv = %d, \
			temp_little_cool_fastchg_current_ma = %d, \
			normal_bat_decidegc = %d, \
			warm_bat_decidegc = %d, \
			temp_warm_vfloat_mv = %d, \
			temp_warm_fastchg_current_ma = %d, \
			hot_bat_decidegc = %d, \
			non_standard_vfloat_mv = %d, \
			non_standard_fastchg_current_ma = %d, \
			max_chg_time_sec = %d, \
			charger_hv_thr = %d, \
			charger_lv_thr = %d, \
			vbatt_full_thr = %d, \
			vbatt_hv_thr = %d, \
			vfloat_step_mv = %d, \
			vooc_project = %d, \
			suspend_after_full = %d, \
			ext_gauge = %d, \
			sw_vfloat_enable = %d, \
			chip->limits.temp_little_cold_fastchg_current_ma_low = %d, \
			chip->limits.temp_little_cold_fastchg_current_ma_high = %d, \
			chip->limits.charger_current_vooc_ma_normal = %d, \
			chip->ffc_support = %d\
			chip->dual_ffc = %d\
			chip->new_ui_warning_support = %d\
			chip->smooth_switch = %d\n",
			chip->limits.input_current_charger_ma,
			chip->limits.input_current_usb_ma,
			chip->limits.input_current_led_ma,
			chip->limits.input_current_led_ma_normal,
			chip->limits.input_current_led_ma_warm,
			chip->limits.input_current_led_ma_high,
			chip->limits.temp_normal_fastchg_current_ma,
			chip->limits.temp_normal_vfloat_mv,
			chip->limits.iterm_ma,
			chip->limits.recharge_mv,
			chip->limits.cold_bat_decidegc,
			chip->limits.temp_cold_vfloat_mv,
			chip->limits.temp_cold_fastchg_current_ma,
			chip->limits.little_cold_bat_decidegc,
			chip->limits.temp_little_cold_vfloat_mv,
			chip->limits.temp_little_cold_fastchg_current_ma,
			chip->limits.cool_bat_decidegc,
			chip->limits.temp_cool_vfloat_mv,
			chip->limits.temp_cool_fastchg_current_ma_high,
			chip->limits.temp_cool_fastchg_current_ma_low,
			chip->limits.little_cool_bat_decidegc,
			chip->limits.temp_little_cool_vfloat_mv,
			chip->limits.temp_little_cool_fastchg_current_ma,
			chip->limits.normal_bat_decidegc,
			chip->limits.warm_bat_decidegc,
			chip->limits.temp_warm_vfloat_mv,
			chip->limits.temp_warm_fastchg_current_ma,
			chip->limits.hot_bat_decidegc,
			chip->limits.non_standard_vfloat_mv,
			chip->limits.non_standard_fastchg_current_ma,
			chip->limits.max_chg_time_sec,
			chip->limits.charger_hv_thr,
			chip->limits.charger_lv_thr,
			chip->limits.vbatt_full_thr,
			chip->limits.vbatt_hv_thr,
			chip->limits.vfloat_step_mv,
			chip->vooc_project,
			chip->suspend_after_full,
			chip->external_gauge,
			chip->limits.sw_vfloat_over_protect_enable,
			chip->limits.temp_little_cold_fastchg_current_ma_low,
			chip->limits.temp_little_cold_fastchg_current_ma_high,
			chip->limits.charger_current_vooc_ma_normal,
			chip->ffc_support,
			chip->dual_ffc,
			chip->new_ui_warning_support,
			chip->smooth_switch);

	chip->dual_charger_support = of_property_read_bool(node, "qcom,dual_charger_support");

	rc = of_property_read_u32(node, "qcom,slave_pct", &chip->slave_pct);
	if (rc) {
		chip->slave_pct = 50;
	}

	rc = of_property_read_u32(node, "qcom,slave_chg_enable_ma", &chip->slave_chg_enable_ma);
	if (rc) {
		chip->slave_chg_enable_ma = 2100;
	}

	rc = of_property_read_u32(node, "qcom,slave_chg_disable_ma", &chip->slave_chg_disable_ma);
	if (rc) {
		chip->slave_chg_disable_ma = 1700;
	}

	rc = of_property_read_u32(node, "qcom,usbtemp_batttemp_gap", &chip->usbtemp_batttemp_gap);
	if (rc) {
		chip->usbtemp_batttemp_gap = 17;
	}

	chip->vooc_show_ui_soc_decimal = of_property_read_bool(node, "qcom,vooc_show_ui_soc_decimal");

	rc = of_property_read_u32(node, "qcom,ui_soc_decimal_speedmin", &chip->ui_soc_decimal_speedmin);
	if (rc) {
		chip->ui_soc_decimal_speedmin = 2;
	}

	charger_xlog_printk(CHG_LOG_CRTI,"dual_charger_support=%d, slave_pct=%d, slave_chg_enable_ma=%d, slave_chg_disable_ma=%d\n",
			chip->dual_charger_support, chip->slave_pct, chip->slave_chg_enable_ma, chip->slave_chg_disable_ma);

	return 0;
}
EXPORT_SYMBOL(oplus_chg_parse_charger_dt);

int oplus_chg_get_tbatt_normal_charging_current(struct oplus_chg_chip *chip)
{
	int charging_current = OPLUS_CHG_DEFAULT_CHARGING_CURRENT;

	switch (chip->tbatt_normal_status) {
		case  BATTERY_STATUS__NORMAL_PHASE1:
			charging_current = chip->limits.temp_normal_phase1_fastchg_current_ma;
			break;
		case BATTERY_STATUS__NORMAL_PHASE2:
			if (vbatt_higherthan_4180mv) {
				charging_current = chip->limits.temp_normal_phase2_fastchg_current_ma_low;
			} else {
				charging_current = chip->limits.temp_normal_phase2_fastchg_current_ma_high;
			}
			break;
		case BATTERY_STATUS__NORMAL_PHASE3:
			if (vbatt_higherthan_4180mv) {
				charging_current = chip->limits.temp_normal_phase3_fastchg_current_ma_low;
			} else {
				charging_current = chip->limits.temp_normal_phase3_fastchg_current_ma_high;
			}
			break;
		case BATTERY_STATUS__NORMAL_PHASE4:
			if (vbatt_higherthan_4180mv) {
				charging_current = chip->limits.temp_normal_phase4_fastchg_current_ma_low;
			} else {
				charging_current = chip->limits.temp_normal_phase4_fastchg_current_ma_high;
			}
			break;
		case BATTERY_STATUS__NORMAL_PHASE5:
			charging_current = chip->limits.temp_normal_phase5_fastchg_current_ma;
			break;
		case BATTERY_STATUS__NORMAL_PHASE6:
			charging_current = chip->limits.temp_normal_phase6_fastchg_current_ma;
			break;
		default:
			break;
	}
	return charging_current;
}
static void oplus_chg_set_charging_current(struct oplus_chg_chip *chip)
{
	int charging_current = OPLUS_CHG_DEFAULT_CHARGING_CURRENT;
//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
		if (chip->wireless_support && (oplus_wpc_get_wireless_charge_start() == true
				|| oplus_chg_is_wls_present())) {
			chg_err(" test do not set ichging , wireless charge start \n");
			return;
		}
//#endif
	switch (chip->tbatt_status) {
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:
		case BATTERY_STATUS__HIGH_TEMP:
			return;
		case BATTERY_STATUS__COLD_TEMP:
			charging_current = chip->limits.temp_cold_fastchg_current_ma;
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:
			//charging_current = chip->limits.temp_little_cold_fastchg_current_ma;
			if (vbatt_higherthan_4180mv) {
				charging_current = chip->limits.temp_little_cold_fastchg_current_ma_low;
			} else {
				charging_current = chip->limits.temp_little_cold_fastchg_current_ma_high;
			}
			charger_xlog_printk(CHG_LOG_CRTI,
					"vbatt_higherthan_4180mv [%d], charging_current[%d]\n",
					vbatt_higherthan_4180mv, charging_current);
			break;
		case BATTERY_STATUS__COOL_TEMP:
			if (vbatt_higherthan_4180mv) {
				charging_current = chip->limits.temp_cool_fastchg_current_ma_low;
			} else {
				charging_current = chip->limits.temp_cool_fastchg_current_ma_high;
			}
			charger_xlog_printk(CHG_LOG_CRTI,
					"vbatt_higherthan_4180mv [%d], charging_current[%d]\n",
					vbatt_higherthan_4180mv, charging_current);
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
			if (chip->dual_charger_support) {
				if (vbatt_higherthan_4180mv) {
					charging_current = chip->limits.temp_little_cool_fastchg_current_ma_low;
				} else {
					charging_current = chip->limits.temp_little_cool_fastchg_current_ma_high;
				}
				charger_xlog_printk(CHG_LOG_CRTI,
						"vbatt_higherthan_4180mv [%d], charging_current[%d]\n",
						vbatt_higherthan_4180mv, charging_current);
			} else {
				charging_current = chip->limits.temp_little_cool_fastchg_current_ma;
			}
			break;
		case BATTERY_STATUS__NORMAL:
			if (chip->dual_charger_support) {
				charging_current = oplus_chg_get_tbatt_normal_charging_current(chip);
			}
			else
				charging_current = chip->limits.temp_normal_fastchg_current_ma;
			break;
		case BATTERY_STATUS__WARM_TEMP:
			charging_current = chip->limits.temp_warm_fastchg_current_ma;
			break;
		default:
			break;
	}
	if (((!chip->authenticate) || (!chip->hmac))
				&& (charging_current > chip->limits.non_standard_fastchg_current_ma)) {
		charging_current = chip->limits.non_standard_fastchg_current_ma;
		charger_xlog_printk(CHG_LOG_CRTI,
			"no high battery, set charging current = %d\n",
			chip->limits.non_standard_fastchg_current_ma);
	}
	if (oplus_short_c_batt_is_prohibit_chg(chip)) {
		if (charging_current > chip->limits.short_c_bat_fastchg_current_ma) {
			charging_current = chip->limits.short_c_bat_fastchg_current_ma;
			charger_xlog_printk(CHG_LOG_CRTI,
				"short circuit battery, set charging current = %d\n",
				chip->limits.short_c_bat_fastchg_current_ma);
		}
	}
	if ((chip->chg_ctrl_by_lcd) && (chip->led_on) &&
				(charging_current > chip->limits.temp_warm_fastchg_current_ma_led_on)) {
		if (chip->tbatt_status == BATTERY_STATUS__WARM_TEMP){
			charging_current = chip->limits.temp_warm_fastchg_current_ma_led_on;
		}
		if (chip->dual_charger_support && chip->tbatt_normal_status == BATTERY_STATUS__NORMAL_PHASE6) {
			charging_current = chip->limits.temp_warm_fastchg_current_ma_led_on;
		}
		charger_xlog_printk(CHG_LOG_CRTI,
				"[BATTERY]LED ON, charging current: %d\n", charging_current);
	}
	if (charging_current == 0) {
		return;
	}
	chip->chg_ops->charging_current_write_fast(charging_current);
}

void oplus_chg_set_input_current_limit(struct oplus_chg_chip *chip)
{
	int current_limit = 0;
	bool is_mcu_fastchg = false;
	is_mcu_fastchg = (oplus_vooc_get_fastchg_started()
					&&(chip->vbatt_num != 2 || oplus_vooc_get_fast_chg_type() != CHARGER_SUBTYPE_FASTCHG_VOOC));

	if(is_mcu_fastchg) {
		chg_err("MCU_READING_iic,return");
		return;
	}
//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
		if (chip->wireless_support && (oplus_wpc_get_wireless_charge_start() == true
				|| oplus_chg_is_wls_present())) {
			chg_err(" test do not set ichging , wireless charge start \n");
			return;
		}
//#endif
	switch (chip->charger_type) {
		case POWER_SUPPLY_TYPE_UNKNOWN:
			return;
		case POWER_SUPPLY_TYPE_USB:
			current_limit = chip->limits.input_current_usb_ma;
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
			current_limit = chip->limits.input_current_charger_ma;
			break;
		case POWER_SUPPLY_TYPE_USB_CDP:
			current_limit = chip->limits.input_current_cdp_ma;
			break;
		default:
			return;
	}

	if ((chip->chg_ctrl_by_lcd) && (chip->led_on)) {
		if (!chip->dual_charger_support || (chip->dual_charger_support && chip->charger_volt > 7500)) {
			if (chip->led_temp_status == LED_TEMP_STATUS__HIGH) {
				if (current_limit > chip->limits.input_current_led_ma_high){
					current_limit = chip->limits.input_current_led_ma_high;
				}
			} else if (chip->led_temp_status == LED_TEMP_STATUS__WARM) {
				if (current_limit > chip->limits.input_current_led_ma_warm){
					current_limit = chip->limits.input_current_led_ma_warm;
				}
			} else {
				if (current_limit > chip->limits.input_current_led_ma_normal){
					current_limit = chip->limits.input_current_led_ma_normal;
				}
			}
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]LED STATUS CHANGED, IS ON\n");
		}
		if ((chip->chg_ctrl_by_camera) && (chip->camera_on)
				&& (current_limit > chip->limits.input_current_camera_ma)) {
			current_limit = chip->limits.input_current_camera_ma;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]CAMERA STATUS CHANGED, IS ON\n");
		}
	} else if ((chip->chg_ctrl_by_camera) && (chip->camera_on)
			&&(current_limit > chip->limits.input_current_camera_ma)) {
		current_limit = chip->limits.input_current_camera_ma;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]CAMERA STATUS CHANGED, IS ON\n");
	}
	if ((chip->chg_ctrl_by_calling) && (chip->calling_on)
			&& (current_limit > chip->limits.input_current_calling_ma)) {
		current_limit = chip->limits.input_current_calling_ma;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY]calling STATUS CHANGED, IS ON\n");
	}
	if (chip->chg_ctrl_by_vooc && chip->vbatt_num == 2
				&& oplus_vooc_get_fast_chg_type() == CHARGER_SUBTYPE_FASTCHG_VOOC
				&& oplus_vooc_get_fastchg_started() == true) {
		if(chip->led_on) {
			if(chip->vooc_temp_status == VOOC_TEMP_STATUS__HIGH) {
				current_limit = chip->limits.input_current_vooc_led_ma_high;
			} else if(chip->vooc_temp_status == VOOC_TEMP_STATUS__WARM) {
				current_limit = chip->limits.input_current_vooc_led_ma_warm;
			} else if(chip->vooc_temp_status == VOOC_TEMP_STATUS__NORMAL) {
				current_limit = chip->limits.input_current_vooc_led_ma_normal;
			}
		} else if (!(chip->chg_ctrl_by_calling && chip->calling_on)) {
			if(chip->vooc_temp_status == VOOC_TEMP_STATUS__HIGH) {
				current_limit = chip->limits.input_current_vooc_ma_high;
			} else if(chip->vooc_temp_status == VOOC_TEMP_STATUS__WARM) {
				current_limit = chip->limits.input_current_vooc_ma_warm;
			} else if(chip->vooc_temp_status == VOOC_TEMP_STATUS__NORMAL) {
				current_limit = chip->limits.input_current_vooc_ma_normal;
			}
		}
		chg_err("chg_ctrl_by_vooc,  \
				led_on = %d,\
				calling_on = %d,\
				current_limit[%d], \
				chip->vooc_temp_status[%d]\n",
				chip->led_on,
				chip->calling_on,
				current_limit,
				chip->vooc_temp_status);
		if ( chip->chg_ops->input_current_ctrl_by_vooc_write) {
			chip->chg_ops->input_current_ctrl_by_vooc_write(current_limit);
			return;
		}
	}
	if (chip->chg_ctrl_by_cool_down && (current_limit > chip->limits.input_current_cool_down_ma)) {
		current_limit = chip->limits.input_current_cool_down_ma;
	}
	charger_xlog_printk(CHG_LOG_CRTI,
		" led_on = %d, \
		current_limit = %d, \
		led_temp_status = %d\n",
		chip->led_on,
		current_limit,
		chip->led_temp_status);
	chip->chg_ops->input_current_write(current_limit);
}

static int oplus_chg_get_float_voltage(struct oplus_chg_chip *chip)
{
	int flv = chip->limits.temp_normal_vfloat_mv;

	switch (chip->tbatt_status) {
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:
		case BATTERY_STATUS__HIGH_TEMP:
			return flv;
		case BATTERY_STATUS__COLD_TEMP:
			flv = chip->limits.temp_cold_vfloat_mv;
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:
			flv = chip->limits.temp_little_cold_vfloat_mv;
			break;
		case BATTERY_STATUS__COOL_TEMP:
			flv = chip->limits.temp_cool_vfloat_mv;
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
			flv = chip->limits.temp_little_cool_vfloat_mv;
		break;
		case BATTERY_STATUS__NORMAL:
			flv = chip->limits.temp_normal_vfloat_mv;
			break;
		case BATTERY_STATUS__WARM_TEMP:
			flv = chip->limits.temp_warm_vfloat_mv;
			break;
		default:
			break;
	}
	if (oplus_short_c_batt_is_prohibit_chg(chip)
			&& flv > chip->limits.short_c_bat_vfloat_mv) {
		flv = chip->limits.short_c_bat_vfloat_mv;
	}
	return flv;
}

static void oplus_chg_set_float_voltage(struct oplus_chg_chip *chip)
{
	int flv = oplus_chg_get_float_voltage(chip);

	if (((!chip->authenticate) ||(!chip->hmac)) && (flv > chip->limits.non_standard_vfloat_mv)) {
		flv = chip->limits.non_standard_vfloat_mv;
		charger_xlog_printk(CHG_LOG_CRTI,
			"no authenticate or no hmac battery, set float voltage = %d\n",
			chip->limits.non_standard_vfloat_mv);
	}
	chip->chg_ops->float_voltage_write(flv * chip->vbatt_num);
	chip->limits.vfloat_sw_set = flv;
}

#define VFLOAT_OVER_NUM		2
static void oplus_chg_vfloat_over_check(struct oplus_chg_chip *chip)
{
	if (!chip->mmi_chg) {
		return;
	}
	if (chip->charging_state == CHARGING_STATUS_FULL) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == false) {
		return;
	}
	if (chip->check_batt_full_by_sw
			&& (chip->limits.sw_vfloat_over_protect_enable == false)) {
		return;
	}
	if (oplus_vooc_get_fastchg_ing() == true) {
		return;
	}
//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (chip->wireless_support && (oplus_wpc_get_ffc_charging() == true || oplus_chg_is_wls_ffc())) {
		return;
	}
//#endif
	//if (!((oplus_vooc_get_fastchg_to_normal()== true) || (oplus_vooc_get_fastchg_to_warm() == true))) {
	if(chip->limits.sw_vfloat_over_protect_enable) {
		if ((chip->batt_volt >= chip->limits.cold_vfloat_over_sw_limit
					&& chip->tbatt_status == BATTERY_STATUS__COLD_TEMP)
				||(chip->batt_volt >= chip->limits.little_cold_vfloat_over_sw_limit
					&& chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP)
				||(chip->batt_volt >= chip->limits.cool_vfloat_over_sw_limit
					&& chip->tbatt_status == BATTERY_STATUS__COOL_TEMP)
				||(chip->batt_volt >= chip->limits.little_cool_vfloat_over_sw_limit
					&& chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP)
				||(chip->batt_volt >= chip->limits.normal_vfloat_over_sw_limit
					&& chip->tbatt_status == BATTERY_STATUS__NORMAL)
				||(chip->batt_volt >= chip->limits.warm_vfloat_over_sw_limit
					&& chip->tbatt_status == BATTERY_STATUS__WARM_TEMP)
				||(((!chip->authenticate) ||(!chip->hmac))
					&& (chip->batt_volt >= chip->limits.non_standard_vfloat_over_sw_limit))) {
			chip->limits.vfloat_over_counts++;
			if (chip->limits.vfloat_over_counts > VFLOAT_OVER_NUM) {
				chip->limits.vfloat_over_counts = 0;
				chip->limits.vfloat_sw_set -= chip->limits.vfloat_step_mv;
				chip->chg_ops->float_voltage_write(chip->limits.vfloat_sw_set * chip->vbatt_num);
				charger_xlog_printk(CHG_LOG_CRTI,
					"bat_volt:%d, \
					tbatt:%d, \
					sw_vfloat_set:%d\n",
					chip->batt_volt,
					chip->tbatt_status,
					chip->limits.vfloat_sw_set);
			}
		} else {
			chip->limits.vfloat_over_counts = 0;
		}
			return;
	}
}

static void oplus_chg_check_vbatt_higher_than_4180mv(struct oplus_chg_chip *chip)
{
	static bool vol_high_pre = false;
	static int lower_count = 0, higher_count = 0;
	static int tbatt_status_pre = BATTERY_STATUS__NORMAL;

	if (!chip->mmi_chg) {
		vbatt_higherthan_4180mv = false;
		vol_high_pre = false;
		lower_count = 0;
		higher_count = 0;
		return;
	}
	if (oplus_vooc_get_fastchg_started() == true) {
		return;
	}
	if (tbatt_status_pre != chip->tbatt_status) {
		tbatt_status_pre = chip->tbatt_status;
		vbatt_higherthan_4180mv = false;
		vol_high_pre = false;
		lower_count = 0;
		higher_count = 0;
	}
	//if (chip->batt_volt >(chip->vbatt_num * 4180)) {
	if (chip->batt_volt > 4180) {
		higher_count++;
		if (higher_count > 2) {
			lower_count = 0;
			higher_count = 3;
			vbatt_higherthan_4180mv = true;
		}
	} else if (vbatt_higherthan_4180mv) {
		//if (chip->batt_volt >(chip->vbatt_num * 4000)) {
		if (chip->batt_volt < 4000) {
			lower_count++;
			if (lower_count > 2) {
				lower_count = 3;
				higher_count = 0;
				vbatt_higherthan_4180mv = false;
			}
		}
	}
	/*chg_err(" tbatt_status:%d,batt_volt:%d,vol_high_pre:%d,vbatt_higherthan_4180mv:%d\n",*/
	/*chip->tbatt_status,chip->batt_volt,vol_high_pre,vbatt_higherthan_4180mv);*/
	if (vol_high_pre != vbatt_higherthan_4180mv) {
		vol_high_pre = vbatt_higherthan_4180mv;
		oplus_chg_set_charging_current(chip);
	}
}

#define TEMP_FFC_COUNTS		2
static void oplus_chg_check_ffc_temp_status(struct oplus_chg_chip *chip)
{
	int batt_temp = chip->temperature;
	OPLUS_CHG_FFC_TEMP_STATUS temp_status = chip->ffc_temp_status;
	static int high_counts = 0, warm_counts = 0, normal_counts = 0;
	static int low_counts = 0;

	if (batt_temp >= chip->limits.ffc2_temp_high_decidegc) {			/*>=40C*/
		//tled_status = FFC_TEMP_STATUS__HIGH;
		high_counts ++;
		if (high_counts >= TEMP_FFC_COUNTS) {
			low_counts = 0;
			high_counts = 0;
			warm_counts = 0;
			normal_counts = 0;
			temp_status = FFC_TEMP_STATUS__HIGH;
		}
	}else if (batt_temp >= chip->limits.ffc2_temp_warm_decidegc) {		/*>=35C && < 40*/
		//temp_status = FFC_TEMP_STATUS__WARM;
		warm_counts ++;
		if (warm_counts >= TEMP_FFC_COUNTS) {
				low_counts = 0;
				high_counts = 0;
						warm_counts = 0;
				normal_counts = 0;
				temp_status = FFC_TEMP_STATUS__WARM;
		}
	} else if (batt_temp >= chip->limits.ffc2_temp_low_decidegc) {		/*>=16C&&<35C*/
		//temp_status = FFC_TEMP_STATUS__NORMAL;
		normal_counts ++;
		if (normal_counts >= TEMP_FFC_COUNTS) {
			low_counts = 0;
			high_counts = 0;
			warm_counts = 0;
			normal_counts = 0;
			temp_status = FFC_TEMP_STATUS__NORMAL;
		}
	} else {															/*<16C*/
		low_counts++;
		if (low_counts >= TEMP_FFC_COUNTS) {
			low_counts = 0;
			high_counts = 0;
			warm_counts = 0;
			normal_counts = 0;
			temp_status = FFC_TEMP_STATUS__LOW;
		}
	}
	if (temp_status != chip->ffc_temp_status) {
	chip->ffc_temp_status = temp_status;
	}
}

void oplus_chg_turn_on_ffc1(struct oplus_chg_chip *chip)
{
	if ((!chip->authenticate) ||(!chip->hmac)) {
		return;
	}
	if (!chip->mmi_chg) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == false) {
		return;
	}
	chip->chg_ops->hardware_init();
	if (chip->stop_chg == 0
			&& (chip->charger_type == POWER_SUPPLY_TYPE_USB
			|| chip->charger_type == POWER_SUPPLY_TYPE_USB_CDP)) {
		chip->chg_ops->charger_suspend();
	}
	if (chip->check_batt_full_by_sw) {
		chip->chg_ops->set_charging_term_disable();
	}

	pr_err("oplus_chg_turn_on_ffc1--------\r\n");
	chip->chg_ctrl_by_lcd = false;
	chip->fastchg_to_ffc = true;
	chip->fastchg_ffc_status = 1;
	chip->chg_ctrl_by_vooc = false;
	chip->recharge_after_ffc = true;

	if (chip->temperature >= chip->limits.ffc2_temp_warm_decidegc) {
		chip->limits.temp_normal_fastchg_current_ma
			= chip->limits.ff1_warm_fastchg_ma;
		chip->limits.temp_little_cool_fastchg_current_ma
			= chip->limits.ff1_warm_fastchg_ma;
	} else {
		chip->limits.temp_normal_fastchg_current_ma
			= chip->limits.ff1_normal_fastchg_ma;
		chip->limits.temp_little_cool_fastchg_current_ma
			= chip->limits.ff1_normal_fastchg_ma;
	}
	chip->limits.normal_vfloat_sw_limit
		= chip->limits.ffc1_normal_vfloat_sw_limit;
	chip->limits.temp_normal_vfloat_mv
		= chip->limits.ffc1_temp_normal_vfloat_mv;
	chip->limits.normal_vfloat_over_sw_limit
		= chip->limits.ffc1_normal_vfloat_over_sw_limit;
	chip->limits.little_cool_vfloat_sw_limit
		= chip->limits.ffc1_normal_vfloat_sw_limit;
	chip->limits.temp_little_cool_vfloat_mv
		= chip->limits.ffc1_temp_normal_vfloat_mv;
	chip->limits.little_cool_vfloat_over_sw_limit
		= chip->limits.ffc1_normal_vfloat_over_sw_limit;
	oplus_chg_check_tbatt_status(chip);
	oplus_chg_set_float_voltage(chip);
	oplus_chg_set_charging_current(chip);
	oplus_chg_set_input_current_limit(chip);
	chip->chg_ops->term_current_set(chip->limits.iterm_ma);
}

void oplus_chg_turn_on_ffc2(struct oplus_chg_chip *chip)
{
	if ((!chip->authenticate) ||(!chip->hmac)) {
		return;
	}
	if (!chip->mmi_chg) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == false) {
		return;
	}
	chip->chg_ops->hardware_init();

	if (chip->check_batt_full_by_sw) {
		chip->chg_ops->set_charging_term_disable();
	}

	pr_err("oplus_chg_turn_on_ffc2--------\r\n");
	chip->chg_ctrl_by_lcd = false;
	chip->fastchg_to_ffc = true;
	chip->fastchg_ffc_status = 2;
	chip->chg_ctrl_by_vooc = false;
	chip->recharge_after_ffc = true;

	if (chip->temperature >= chip->limits.ffc2_temp_warm_decidegc) {
		chip->limits.temp_normal_fastchg_current_ma
			= chip->limits.ffc2_warm_fastchg_ma;
		chip->limits.temp_little_cool_fastchg_current_ma
			= chip->limits.ffc2_warm_fastchg_ma;
	} else {
		chip->limits.temp_normal_fastchg_current_ma
			= chip->limits.ffc2_normal_fastchg_ma;
		chip->limits.temp_little_cool_fastchg_current_ma
			= chip->limits.ffc2_normal_fastchg_ma;
	}

	chip->limits.normal_vfloat_sw_limit
		= chip->limits.ffc2_normal_vfloat_sw_limit;
	chip->limits.temp_normal_vfloat_mv
		= chip->limits.ffc2_temp_normal_vfloat_mv;
	chip->limits.normal_vfloat_over_sw_limit
		= chip->limits.ffc2_normal_vfloat_over_sw_limit;

	chip->limits.little_cool_vfloat_sw_limit
		= chip->limits.ffc2_normal_vfloat_sw_limit;
	chip->limits.temp_little_cool_vfloat_mv
		= chip->limits.ffc2_temp_normal_vfloat_mv;
	chip->limits.little_cool_vfloat_over_sw_limit
		= chip->limits.ffc2_normal_vfloat_over_sw_limit;

	oplus_chg_check_tbatt_status(chip);
	oplus_chg_set_float_voltage(chip);
	oplus_chg_set_charging_current(chip);
	oplus_chg_set_input_current_limit(chip);
	chip->chg_ops->term_current_set(chip->limits.iterm_ma);
}

void oplus_chg_turn_on_charging(struct oplus_chg_chip *chip)
{
	if (!chip->authenticate) {
		return;
	}
	if (!chip->mmi_chg) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == false) {
		return;
	}

	chip->chg_ops->hardware_init();
	if (chip->stop_chg == 0
			&& (chip->charger_type == POWER_SUPPLY_TYPE_USB
				|| chip->charger_type == POWER_SUPPLY_TYPE_USB_CDP)) {
		chip->chg_ops->charger_suspend();
	}
	if (chip->check_batt_full_by_sw) {
		chip->chg_ops->set_charging_term_disable();
	}

	oplus_chg_check_tbatt_status(chip);
	oplus_chg_set_float_voltage(chip);
	oplus_chg_set_charging_current(chip);
	oplus_chg_set_input_current_limit(chip);

	chip->chg_ops->term_current_set(chip->limits.iterm_ma);
}

void oplus_chg_turn_off_charging(struct oplus_chg_chip *chip)
{
	if (oplus_vooc_get_allow_reading() == false) {
		return;
	}

	switch (chip->tbatt_status) {
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:
			break;
		case BATTERY_STATUS__HIGH_TEMP:
			break;
		case BATTERY_STATUS__COLD_TEMP:
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:
		case BATTERY_STATUS__COOL_TEMP:
			chip->chg_ops->charging_current_write_fast(chip->limits.temp_cold_fastchg_current_ma);
			msleep(50);
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
		case BATTERY_STATUS__NORMAL:
			chip->chg_ops->charging_current_write_fast(chip->limits.temp_cool_fastchg_current_ma_high);
			msleep(50);
			chip->chg_ops->charging_current_write_fast(chip->limits.temp_cold_fastchg_current_ma);
			msleep(50);
			break;
		case BATTERY_STATUS__WARM_TEMP:
			chip->chg_ops->charging_current_write_fast(chip->limits.temp_cold_fastchg_current_ma);
			msleep(50);
			break;
		default:
			break;
	}
	chip->chg_ops->charging_disable();
	/*charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] oplus_chg_turn_off_charging !!\n");*/
}
/*
static int oplus_chg_check_suspend_or_disable(struct oplus_chg_chip *chip)
{
	if(chip->suspend_after_full) {
		if(chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP
			|| chip->tbatt_status == BATTERY_STATUS__COOL_TEMP
			|| chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP
			|| chip->tbatt_status == BATTERY_STATUS__NORMAL) {
			return CHG_SUSPEND;
		} else if(chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {
			return CHG_DISABLE;
		} else {
			return CHG_NO_SUSPEND_NO_DISABLE;
		}
	} else {
		if(chip->tbatt_status == BATTERY_STATUS__COLD_TEMP
			|| chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {
			return CHG_DISABLE;
		} else {
			return CHG_NO_SUSPEND_NO_DISABLE;
		}
	}
}
*/

static int oplus_chg_check_suspend_or_disable(struct oplus_chg_chip *chip)
{
	if (chip->suspend_after_full) {
		if ((chip->tbatt_status == BATTERY_STATUS__HIGH_TEMP
			|| chip->tbatt_status == BATTERY_STATUS__WARM_TEMP) && (chip->batt_volt < 4250)) {
			return CHG_DISABLE;
		} else {
			return CHG_SUSPEND;
		}
	} else {
		return CHG_DISABLE;
	}
}


static void oplus_chg_voter_charging_start(struct oplus_chg_chip *chip,
					OPLUS_CHG_STOP_VOTER voter)
{
	if (!chip->authenticate) {
		return;
	}
	if (!chip->mmi_chg) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == false) {
		return;
	}

	chip->chging_on = true;
	chip->stop_voter &= ~(int)voter;
	oplus_chg_turn_on_charging(chip);

	switch (voter) {
		case CHG_STOP_VOTER__FULL:
			chip->charging_state = CHARGING_STATUS_CCCV;
			if (oplus_vooc_get_allow_reading() == true) {
				chip->chg_ops->charger_unsuspend();
				chip->chg_ops->charging_enable();
			}
			break;
		case CHG_STOP_VOTER__VCHG_ABNORMAL:
			chip->charging_state = CHARGING_STATUS_CCCV;
			if (oplus_vooc_get_allow_reading() == true) {
				chip->chg_ops->charger_unsuspend();
			}
			break;
		case CHG_STOP_VOTER__BATTTEMP_ABNORMAL:
		case CHG_STOP_VOTER__VBAT_TOO_HIGH:
		case CHG_STOP_VOTER__MAX_CHGING_TIME:
			chip->charging_state = CHARGING_STATUS_CCCV;
			break;
		default:
			break;
	}
}

void oplus_chg_voter_charging_stop(struct oplus_chg_chip *chip,
					OPLUS_CHG_STOP_VOTER voter)
{
	chip->chging_on = false;
	chip->stop_voter |= (int)voter;

	switch (voter) {
		case CHG_STOP_VOTER__FULL:
			chip->charging_state = CHARGING_STATUS_FULL;
			if (oplus_vooc_get_allow_reading() == true) {
				if (oplus_chg_check_suspend_or_disable(chip) == CHG_SUSPEND) {
					chip->chg_ops->charger_suspend();
				} else {
					oplus_chg_turn_off_charging(chip);
				}
			}
			break;
		case CHG_STOP_VOTER__VCHG_ABNORMAL:
			chip->charging_state = CHARGING_STATUS_FAIL;
			chip->total_time = 0;
			if (oplus_vooc_get_allow_reading() == true) {
				chip->chg_ops->charger_suspend();
			}
			oplus_chg_turn_off_charging(chip);
			break;
		case CHG_STOP_VOTER__BATTTEMP_ABNORMAL:
		case CHG_STOP_VOTER__VBAT_TOO_HIGH:
			chip->charging_state = CHARGING_STATUS_FAIL;
			chip->total_time = 0;
			oplus_chg_turn_off_charging(chip);
			break;
		case CHG_STOP_VOTER__MAX_CHGING_TIME:
			chip->charging_state = CHARGING_STATUS_FAIL;
			oplus_chg_turn_off_charging(chip);
			break;
		default:
			break;
		}
}

#define HYSTERISIS_DECIDEGC			20
#define HYSTERISIS_DECIDEGC_0C		5
#define TBATT_PRE_SHAKE_INVALID		999
static void battery_temp_anti_shake_handle(struct oplus_chg_chip *chip)
{
	int tbatt_cur_shake = chip->temperature, low_shake = 0, high_shake = 0;
	int low_shake_0c = 0, high_shake_0c = 0;

	if (tbatt_cur_shake > chip->tbatt_pre_shake) {			/*get warmer*/
		low_shake = -HYSTERISIS_DECIDEGC;
		high_shake = 0;
		low_shake_0c = -HYSTERISIS_DECIDEGC_0C;
		high_shake_0c = 0;
	} else if (tbatt_cur_shake < chip->tbatt_pre_shake) {	/*get cooler*/
		low_shake = 0;
		high_shake = HYSTERISIS_DECIDEGC;
		low_shake_0c = 0;
		high_shake_0c = HYSTERISIS_DECIDEGC_0C;
	}
	if (chip->tbatt_status == BATTERY_STATUS__HIGH_TEMP) {										/*>53C*/
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;						/*-3C*/
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;		/*0C*/
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;						/*5C*/
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;		/*12C*/
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;					/*16C*/
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;						/*45C*/
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound + low_shake;			/*53C*/
	} else if (chip->tbatt_status == BATTERY_STATUS__LOW_TEMP) {								/*<-3C*/
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound + high_shake;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	} else if (chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {								/*-3C~0C*/
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound + low_shake;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound + high_shake_0c;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {						/*0C-5C*/
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound + low_shake_0c;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound + high_shake;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	} else if (chip->tbatt_status == BATTERY_STATUS__COOL_TEMP) {								/*5C~12C*/
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound + low_shake;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound + high_shake;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP) {						/*12C~16C*/
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound + low_shake;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound + high_shake;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	} else if (chip->tbatt_status == BATTERY_STATUS__NORMAL) {									/*16C~45C*/
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	} else if (chip->tbatt_status == BATTERY_STATUS__WARM_TEMP) {								/*45C~53C*/
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound + low_shake;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound + high_shake;
	} else {	/*BATTERY_STATUS__REMOVED								<-19C*/
		chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
		chip->limits.little_cold_bat_decidegc = chip->anti_shake_bound.little_cold_bound;
		chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
		chip->limits.little_cool_bat_decidegc = chip->anti_shake_bound.little_cool_bound;
		chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
		chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
		chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;
	}
	chip->tbatt_pre_shake = tbatt_cur_shake;
}


#define TEMP_CNT	1
static bool oplus_chg_check_tbatt_is_good(struct oplus_chg_chip *chip)
{
	static bool ret = true;
	static int temp_counts = 0;
	int batt_temp = chip->temperature;
	OPLUS_CHG_TBATT_STATUS tbatt_status = chip->tbatt_status;

	if (batt_temp > chip->limits.hot_bat_decidegc || batt_temp < chip->limits.cold_bat_decidegc) {
		temp_counts++;
		if (temp_counts >= TEMP_CNT) {
			temp_counts = 0;
			ret = false;
			if (batt_temp <= chip->limits.removed_bat_decidegc) {
				tbatt_status = BATTERY_STATUS__REMOVED;
			} else if (batt_temp > chip->limits.hot_bat_decidegc) {
				tbatt_status = BATTERY_STATUS__HIGH_TEMP;
			} else {
				tbatt_status = BATTERY_STATUS__LOW_TEMP;
			}
		}
	} else {
		temp_counts = 0;
		ret = true;
		if (batt_temp >= chip->limits.warm_bat_decidegc) {						/*45C*/
			tbatt_status = BATTERY_STATUS__WARM_TEMP;
		} else if (batt_temp >= chip->limits.normal_bat_decidegc) {				/*16C*/
			tbatt_status = BATTERY_STATUS__NORMAL;
		} else if (batt_temp >= chip->limits.little_cool_bat_decidegc) {		/*12C*/
			tbatt_status = BATTERY_STATUS__LITTLE_COOL_TEMP;
		} else if (batt_temp >= chip->limits.cool_bat_decidegc) {				/*5C*/
			tbatt_status = BATTERY_STATUS__COOL_TEMP;
		} else if (batt_temp >= chip->limits.little_cold_bat_decidegc) {		/*0C*/
			tbatt_status = BATTERY_STATUS__LITTLE_COLD_TEMP;
		} else if (batt_temp >= chip->limits.cold_bat_decidegc) {				/*-3C*/
			tbatt_status = BATTERY_STATUS__COLD_TEMP;
		} else {
			tbatt_status = BATTERY_STATUS__COLD_TEMP;
		}
	}
	if (tbatt_status == BATTERY_STATUS__REMOVED) {
		chip->batt_exist = false;
	} else {
		chip->batt_exist = true;
	}
	if (chip->tbatt_pre_shake == TBATT_PRE_SHAKE_INVALID) {
		chip->tbatt_pre_shake = batt_temp;
	}
	if (tbatt_status != chip->tbatt_status) {
		if (chip->tbatt_status == BATTERY_STATUS__WARM_TEMP
					|| chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {
			if (chip->soc != 100 && chip->batt_full == true
					&& chip->charging_state == CHARGING_STATUS_FULL) {
				chip->batt_full = false;
				chip->tbatt_when_full = 200;
				oplus_chg_voter_charging_start(chip, CHG_STOP_VOTER__FULL);
			}
		}
		chip->tbatt_status = tbatt_status;
		vbatt_higherthan_4180mv = false;
		if (oplus_vooc_get_allow_reading() == true) {
			oplus_chg_set_float_voltage(chip);
			oplus_chg_set_charging_current(chip);
		}
		battery_temp_anti_shake_handle(chip);
	}
	return ret;
}

static void oplus_chg_check_tbatt_status(struct oplus_chg_chip *chip)
{
	int batt_temp = chip->temperature;
	OPLUS_CHG_TBATT_STATUS tbatt_status = chip->tbatt_status;

	if (batt_temp > chip->limits.hot_bat_decidegc) {					/*53C*/
		tbatt_status = BATTERY_STATUS__HIGH_TEMP;
	} else if (batt_temp >= chip->limits.warm_bat_decidegc) {			/*45C*/
		tbatt_status = BATTERY_STATUS__WARM_TEMP;
	} else if (batt_temp >= chip->limits.normal_bat_decidegc) {			/*16C*/
		tbatt_status = BATTERY_STATUS__NORMAL;
	} else if (batt_temp >= chip->limits.little_cool_bat_decidegc) {	/*12C*/
		tbatt_status = BATTERY_STATUS__LITTLE_COOL_TEMP;
	} else if (batt_temp >= chip->limits.cool_bat_decidegc) {			/*5C*/
		tbatt_status = BATTERY_STATUS__COOL_TEMP;
	} else if (batt_temp >= chip->limits.little_cold_bat_decidegc) {	/*0C*/
		tbatt_status = BATTERY_STATUS__LITTLE_COLD_TEMP;
	} else if (batt_temp >= chip->limits.cold_bat_decidegc) {			/*-3C*/
		tbatt_status = BATTERY_STATUS__COLD_TEMP;
	} else if (batt_temp > chip->limits.removed_bat_decidegc) {			/*-20C*/
		tbatt_status = BATTERY_STATUS__LOW_TEMP;
	} else {
		tbatt_status = BATTERY_STATUS__REMOVED;
	}
	if (tbatt_status == BATTERY_STATUS__REMOVED) {
		chip->batt_exist = false;
	} else {
		chip->batt_exist = true;
	}
	chip->tbatt_status = tbatt_status;
}

static void battery_temp_normal_anti_shake_handle(struct oplus_chg_chip *chip)
{
        int tbatt_normal_cur_shake = chip->temperature, low_shake = 0, high_shake = 0;

        if (tbatt_normal_cur_shake > chip->tbatt_normal_pre_shake) {                  /*get warmer*/
                low_shake = -HYSTERISIS_DECIDEGC;
                high_shake = 0;
        } else if (tbatt_normal_cur_shake < chip->tbatt_normal_pre_shake) {   /*get cooler*/
                low_shake = 0;
                high_shake = HYSTERISIS_DECIDEGC;
        }

	if (chip->tbatt_normal_status == BATTERY_STATUS__NORMAL_PHASE1) {
		chip->limits.normal_phase1_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase1_bound;
		chip->limits.normal_phase2_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase2_bound + high_shake;
		chip->limits.normal_phase3_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase3_bound;
		chip->limits.normal_phase4_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase4_bound;
		chip->limits.normal_phase5_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase5_bound;
		chip->limits.normal_phase6_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase6_bound;
	} else if (chip->tbatt_normal_status == BATTERY_STATUS__NORMAL_PHASE2) {
		chip->limits.normal_phase1_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase1_bound;
		chip->limits.normal_phase2_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase2_bound + low_shake;
		chip->limits.normal_phase3_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase3_bound + high_shake;
		chip->limits.normal_phase4_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase4_bound;
		chip->limits.normal_phase5_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase5_bound;
		chip->limits.normal_phase6_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase6_bound;
	} else if (chip->tbatt_normal_status == BATTERY_STATUS__NORMAL_PHASE3) {
		chip->limits.normal_phase1_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase1_bound;
		chip->limits.normal_phase2_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase2_bound;
		chip->limits.normal_phase3_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase3_bound + low_shake;
		chip->limits.normal_phase4_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase4_bound + high_shake;
		chip->limits.normal_phase5_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase5_bound;
		chip->limits.normal_phase6_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase6_bound;
	} else if (chip->tbatt_normal_status == BATTERY_STATUS__NORMAL_PHASE4) {
		chip->limits.normal_phase1_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase1_bound;
		chip->limits.normal_phase2_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase2_bound;
		chip->limits.normal_phase3_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase3_bound;
		chip->limits.normal_phase4_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase4_bound + low_shake;
		chip->limits.normal_phase5_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase5_bound + high_shake;
		chip->limits.normal_phase6_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase6_bound;
	} else if (chip->tbatt_normal_status == BATTERY_STATUS__NORMAL_PHASE5) {
		chip->limits.normal_phase1_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase1_bound;
		chip->limits.normal_phase2_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase2_bound;
		chip->limits.normal_phase3_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase3_bound;
		chip->limits.normal_phase4_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase4_bound;
		chip->limits.normal_phase5_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase5_bound + low_shake;
		chip->limits.normal_phase6_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase6_bound + high_shake;
	} else if (chip->tbatt_normal_status == BATTERY_STATUS__NORMAL_PHASE6) {
		chip->limits.normal_phase1_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase1_bound;
		chip->limits.normal_phase2_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase2_bound;
		chip->limits.normal_phase3_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase3_bound;
		chip->limits.normal_phase4_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase4_bound;
		chip->limits.normal_phase5_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase5_bound;
		chip->limits.normal_phase6_bat_decidegc = chip->tbatt_normal_anti_shake_bound.phase6_bound + low_shake;
	} else {
		/*do nothing*/
	}

	chip->tbatt_normal_pre_shake = tbatt_normal_cur_shake;
}

static void oplus_chg_check_tbatt_normal_status(struct oplus_chg_chip *chip)
{
        int batt_temp = chip->temperature;
        OPLUS_CHG_TBATT_NORMAL_STATUS tbatt_normal_status = chip->tbatt_normal_status;

	if (chip->tbatt_status == BATTERY_STATUS__NORMAL) {
		if (batt_temp >= chip->limits.normal_phase6_bat_decidegc) {
			tbatt_normal_status = BATTERY_STATUS__NORMAL_PHASE6;
		} else if (batt_temp >= chip->limits.normal_phase5_bat_decidegc) {
			tbatt_normal_status = BATTERY_STATUS__NORMAL_PHASE5;
		} else if (batt_temp >= chip->limits.normal_phase4_bat_decidegc) {
			tbatt_normal_status = BATTERY_STATUS__NORMAL_PHASE4;
		} else if (batt_temp >= chip->limits.normal_phase3_bat_decidegc) {
			tbatt_normal_status = BATTERY_STATUS__NORMAL_PHASE3;
		} else if (batt_temp >= chip->limits.normal_phase2_bat_decidegc) {
			tbatt_normal_status = BATTERY_STATUS__NORMAL_PHASE2;
		} else if (batt_temp >= chip->limits.normal_phase1_bat_decidegc) {
			tbatt_normal_status = BATTERY_STATUS__NORMAL_PHASE1;
                } else {
			/*do nothing*/
		}
	}

	if (chip->tbatt_normal_pre_shake == TBATT_PRE_SHAKE_INVALID) {
		chip->tbatt_pre_shake = batt_temp;
	}

	if (chip->tbatt_normal_status != tbatt_normal_status) {
		chip->tbatt_normal_status = tbatt_normal_status;
		if (oplus_vooc_get_allow_reading() == true) {
			oplus_chg_set_float_voltage(chip);
			oplus_chg_set_charging_current(chip);
		}
		battery_temp_normal_anti_shake_handle(chip);
		chg_debug("tbatt_normal_status status change, [%d %d %d %d %d %d %d]\n", chip->tbatt_normal_status,
					chip->limits.normal_phase1_bat_decidegc, chip->limits.normal_phase2_bat_decidegc,
					chip->limits.normal_phase3_bat_decidegc, chip->limits.normal_phase4_bat_decidegc,
					chip->limits.normal_phase5_bat_decidegc, chip->limits.normal_phase6_bat_decidegc);
	}
}

#define VCHG_CNT	2
static bool oplus_chg_check_vchg_is_good(struct oplus_chg_chip *chip)
{
	static bool ret = true;
	static int vchg_counts = 0;
	int chg_volt = chip->charger_volt;
	OPLUS_CHG_VCHG_STATUS vchg_status = chip->vchg_status;
//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (chip->wireless_support && (oplus_wpc_get_wireless_charge_start() == true
			|| oplus_chg_is_wls_present())) {
		chg_err(" test wireless fastchg charge start\n");
		return true;
	}
//#endif
	if (oplus_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE) {
		return true;
	}
	if (oplus_vooc_get_fastchg_started() == true) {
		return true;
	}
	if (chg_volt > chip->limits.charger_hv_thr) {
		vchg_counts++;
		if (vchg_counts >= VCHG_CNT) {
			vchg_counts = 0;
			ret = false;
			vchg_status = CHARGER_STATUS__VOL_HIGH;
		}
	} else  if (chg_volt <= chip->limits.charger_recv_thr) {
		vchg_counts = 0;
		ret = true;
		vchg_status = CHARGER_STATUS__GOOD;
	}
	if (vchg_status != chip->vchg_status) {
		chip->vchg_status = vchg_status;
	}
	return ret;
}

#define VBAT_CNT	1

static bool oplus_chg_check_vbatt_is_good(struct oplus_chg_chip *chip)
{
	static bool ret = true;
	static int vbat_counts = 0;
	int batt_volt = chip->batt_volt;

	if (batt_volt >= chip->limits.vbatt_hv_thr) {
		vbat_counts++;
		if (vbat_counts >= VBAT_CNT) {
			vbat_counts = 0;
			ret = false;
			chip->vbatt_over = true;
		}
	} else {
		vbat_counts = 0;
		ret = true;
		chip->vbatt_over = false;
	}
	return ret;
}

static bool oplus_chg_check_time_is_good(struct oplus_chg_chip *chip)
{
#ifdef SELL_MODE
	chip->chging_over_time = false;
	printk("oplus_chg_check_time_is_good_sell_mode\n");
	return true;
#endif //SELL_MODE

	if (chip->limits.max_chg_time_sec < 0) {
		chip->chging_over_time = false;
		return true;
	}
	if (chip->total_time >= chip->limits.max_chg_time_sec) {
		chip->total_time = chip->limits.max_chg_time_sec;
		chip->chging_over_time = true;
		return false;
	} else {
		chip->chging_over_time = false;
		return true;
	}
}

#ifdef CONFIG_FB
#if IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
static int fb_notifier_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	int blank;
	struct msm_drm_notifier *evdata = data;

	if (!g_charger_chip) {
		return 0;
	}
	if (!evdata || (evdata->id != 0)){
		return 0;
	}

	if (event == MSM_DRM_EARLY_EVENT_BLANK) {
		blank = *(int *)(evdata->data);
		if (blank == MSM_DRM_BLANK_UNBLANK) {
			g_charger_chip->led_on = true;
			g_charger_chip->led_on_change = true;
		} else if (blank == MSM_DRM_BLANK_POWERDOWN) {
			g_charger_chip->led_on = false;
			g_charger_chip->led_on_change = true;
		} else {
			pr_err("%s: receives wrong data EARLY_BLANK:%d\n", __func__, blank);
		}
	}
	return 0;
}
#else
static int fb_notifier_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	int blank;
	struct fb_event *evdata = data;

	if (!g_charger_chip) {
		return 0;
	}
	if (evdata && evdata->data) {
		if (event == FB_EVENT_BLANK) {
			blank = *(int *)evdata->data;
			if (blank == FB_BLANK_UNBLANK) {
				g_charger_chip->led_on = true;
				g_charger_chip->led_on_change = true;
			} else if (blank == FB_BLANK_POWERDOWN) {
				g_charger_chip->led_on = false;
				g_charger_chip->led_on_change = true;
			}
		}
	}
	return 0;
}
#endif /* CONFIG_DRM_MSM */

void oplus_chg_set_allow_switch_to_fastchg(bool allow)
{
	charger_xlog_printk(CHG_LOG_CRTI, " allow = %d\n", allow);
	if (!g_charger_chip) {
		return;
	} else {
		g_charger_chip->allow_swtich_to_fastchg = allow;
	}
}

void oplus_chg_set_led_status(bool val)
{
	/*Do nothing*/
}
EXPORT_SYMBOL(oplus_chg_set_led_status);
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY_CHG)
static int chg_mtk_drm_notifier_callback(struct notifier_block *nb,
			unsigned long event, void *data)
{
	int *blank = (int *)data;

	if (!blank) {
		chg_err("get disp statu err, blank is NULL!");
		g_charger_chip->led_on = true;
		g_charger_chip->led_on_change = true;
		return 0;
	}

	if (!event) {
		chg_err("Invalid event\n");
		return 0;
	}

	if (!g_charger_chip) {
		return 0;
	}

	switch (event) {
	case MTK_DISP_EARLY_EVENT_BLANK:
		if (*blank == MTK_DISP_BLANK_UNBLANK) {
			g_charger_chip->led_on = true;
			g_charger_chip->led_on_change = true;
		} else if (*blank == MTK_DISP_BLANK_POWERDOWN) {
			g_charger_chip->led_on = false;
			g_charger_chip->led_on_change = true;
		} else {
			pr_err("%s: receives wrong data EARLY_BLANK:%d\n", __func__, blank);
		}
	case MTK_DISP_EVENT_BLANK:
		if (*blank == MTK_DISP_BLANK_UNBLANK) {
			g_charger_chip->led_on = true;
			g_charger_chip->led_on_change = true;
		} else if (*blank == MTK_DISP_BLANK_POWERDOWN) {
			g_charger_chip->led_on = false;
			g_charger_chip->led_on_change = true;
		} else {
			pr_err("%s: receives wrong data BLANK:%d\n", __func__, blank);
		}
	}
	return 0;
}
#else
void oplus_chg_set_led_status(bool val)
{
	charger_xlog_printk(CHG_LOG_CRTI, " val = %d\n", val);
	if (!g_charger_chip) {
		return;
	} else {
		g_charger_chip->led_on = val;
		g_charger_chip->led_on_change = true;
	}
}
EXPORT_SYMBOL(oplus_chg_set_led_status);
#endif

void oplus_chg_set_camera_status(bool val)
{
	if (!g_charger_chip) {
		return;
	} else {
		g_charger_chip->camera_on = val;
	}
}
EXPORT_SYMBOL(oplus_chg_set_camera_status);

#define TLED_CHANGE_COUNTS	4
#define TLED_HYSTERISIS_DECIDEGC	10
static void oplus_chg_check_tled_status(struct oplus_chg_chip *chip)
{
	OPLUS_CHG_TLED_STATUS tled_status = chip->led_temp_status;
	static int high_counts = 0, warm_counts = 0, normal_counts = 0;

	if (chip->temperature > chip->limits.led_high_bat_decidegc_antishake) {		/* >37C */
		high_counts ++;
		if (high_counts >= TLED_CHANGE_COUNTS) {
			high_counts = 0;
			warm_counts = 0;
			normal_counts = 0;
			tled_status = LED_TEMP_STATUS__HIGH;
		}
	} else if (chip->temperature > chip->limits.led_warm_bat_decidegc_antishake) {	/* >35C && <= 37 */
		warm_counts ++;
		if (warm_counts >= TLED_CHANGE_COUNTS) {
			high_counts = 0;
			warm_counts = 0;
			normal_counts = 0;
			tled_status = LED_TEMP_STATUS__WARM;
		}
	} else {
		normal_counts ++;
		if (normal_counts >= TLED_CHANGE_COUNTS) {
			high_counts = 0;
			warm_counts = 0;
			normal_counts = 0;
			tled_status = LED_TEMP_STATUS__NORMAL;
		}
	}
	if (tled_status != chip->led_temp_status) {
		chip->limits.led_warm_bat_decidegc_antishake
				= chip->limits.led_warm_bat_decidegc;
		chip->limits.led_high_bat_decidegc_antishake
				= chip->limits.led_high_bat_decidegc;
		if (tled_status > chip->led_temp_status
				&& tled_status == LED_TEMP_STATUS__WARM){
			chip->limits.led_warm_bat_decidegc_antishake
				= chip->limits.led_warm_bat_decidegc - TLED_HYSTERISIS_DECIDEGC;
		} else if (tled_status > chip->led_temp_status
				&& tled_status == LED_TEMP_STATUS__HIGH){
			chip->limits.led_high_bat_decidegc_antishake
				= chip->limits.led_high_bat_decidegc - TLED_HYSTERISIS_DECIDEGC;
		} else if (tled_status < chip->led_temp_status
				&& tled_status == LED_TEMP_STATUS__NORMAL){
			chip->limits.led_warm_bat_decidegc_antishake
				= chip->limits.led_warm_bat_decidegc + TLED_HYSTERISIS_DECIDEGC;
		} else if (tled_status < chip->led_temp_status
				&& tled_status == LED_TEMP_STATUS__WARM){
			chip->limits.led_high_bat_decidegc_antishake
				= chip->limits.led_high_bat_decidegc + TLED_HYSTERISIS_DECIDEGC;
		}
		chg_debug("tled status change, [%d %d %d %d]\n",
				tled_status,
				chip->led_temp_status,
				chip->limits.led_warm_bat_decidegc_antishake,
				chip->limits.led_high_bat_decidegc_antishake);
		chip->led_temp_change = true;
		chip->led_temp_status = tled_status;
	}
}

static void oplus_chg_check_led_on_ichging(struct oplus_chg_chip *chip)
{
	if (chip->led_on_change || (chip->led_on && chip->led_temp_change)) {
		chip->led_on_change = false;
		chip->led_temp_change = false;
		if (chip->chg_ctrl_by_vooc && chip->vbatt_num == 2) {
			if (oplus_vooc_get_fastchg_started() == true
					&& oplus_vooc_get_fast_chg_type()
					!= CHARGER_SUBTYPE_FASTCHG_VOOC) {
				return;
			}
			if (oplus_vooc_get_allow_reading() == true
					|| oplus_vooc_get_fast_chg_type()
					== CHARGER_SUBTYPE_FASTCHG_VOOC) {
				oplus_chg_set_charging_current(chip);
				oplus_chg_set_input_current_limit(chip);
			}
		} else {
			if (oplus_vooc_get_fastchg_started() == true) {
				return;
			}
			if (oplus_vooc_get_allow_reading() == true) {
				if (chip->dual_charger_support) {
					chip->slave_charger_enable = false;
				}
				oplus_chg_set_charging_current(chip);
				oplus_chg_set_input_current_limit(chip);
			}
		}
	}
}

#define TVOOC_COUNTS	2
#define TVOOC_HYSTERISIS_DECIDEGC	10

static void oplus_chg_check_vooc_temp_status(struct oplus_chg_chip *chip)
{
	int batt_temp = chip->temperature;
	OPLUS_CHG_TBAT_VOOC_STATUS tbat_vooc_status = chip->vooc_temp_status;
	static int high_counts = 0, warm_counts = 0, normal_counts = 0;
	static bool vooc_first_set_input_current_flag = false;

	if (chip->vbatt_num != 2
			|| oplus_vooc_get_fast_chg_type() != CHARGER_SUBTYPE_FASTCHG_VOOC
			|| oplus_vooc_get_fastchg_started() == false) {
		vooc_first_set_input_current_flag = false;
		return;
	}
	if (batt_temp > chip->limits.vooc_high_bat_decidegc) {			/*>45C*/
		if (oplus_vooc_get_fastchg_started() == true) {
				chg_err("tbatt > 45, quick out vooc");
				//oplus_chg_set_chargerid_switch_val(0);
				//oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
		}
	} else if (batt_temp > chip->limits.vooc_warm_bat_decidegc_antishake) {	/*>38C && <= 45*/
		high_counts ++;
		if(high_counts >= TVOOC_COUNTS)
		{
			high_counts = 0;
			warm_counts = 0;
			normal_counts = 0;
			tbat_vooc_status = VOOC_TEMP_STATUS__HIGH;
		}
	 } else if (batt_temp >= chip->limits.vooc_normal_bat_decidegc_antishake) { /*>34C && <= 38*/
		warm_counts ++;
		if (warm_counts >= TVOOC_COUNTS)
		{
			high_counts = 0;
			warm_counts = 0;
			normal_counts = 0;
			tbat_vooc_status = VOOC_TEMP_STATUS__WARM;
		}
	} else {								// < 34
		normal_counts ++;
		if (normal_counts >= TVOOC_COUNTS)
		{
			high_counts = 0;
			warm_counts = 0;
			normal_counts = 0;
			tbat_vooc_status = VOOC_TEMP_STATUS__NORMAL;
		}
	}
	chg_err("tbat_vooc_status[%d],chip->vooc_temp_status[%d] ",
		tbat_vooc_status, chip->vooc_temp_status);
	if (vooc_first_set_input_current_flag == false) {
		chip->limits.temp_little_cool_fastchg_current_ma
			= chip->limits.charger_current_vooc_ma_normal;
		chip->limits.temp_normal_fastchg_current_ma
			= chip->limits.charger_current_vooc_ma_normal;
		oplus_chg_set_charging_current(chip);
		chg_err("set charger current ctrl by vooc[%d]\n",
			chip->limits.temp_little_cool_fastchg_current_ma);
	}
	if (tbat_vooc_status != chip->vooc_temp_status
			|| vooc_first_set_input_current_flag == false) {
		chip->limits.vooc_warm_bat_decidegc_antishake
			= chip->limits.vooc_warm_bat_decidegc;
		chip->limits.vooc_normal_bat_decidegc_antishake
			= chip->limits.vooc_normal_bat_decidegc;
		if (tbat_vooc_status > chip->vooc_temp_status
				&& tbat_vooc_status == VOOC_TEMP_STATUS__WARM) {
			chip->limits.vooc_normal_bat_decidegc_antishake
				= chip->limits.vooc_normal_bat_decidegc - TVOOC_HYSTERISIS_DECIDEGC;
		} else if (tbat_vooc_status > chip->vooc_temp_status
				&& tbat_vooc_status == VOOC_TEMP_STATUS__HIGH) {
			chip->limits.vooc_warm_bat_decidegc_antishake
				= chip->limits.vooc_warm_bat_decidegc - TVOOC_HYSTERISIS_DECIDEGC;
		} else if (tbat_vooc_status < chip->vooc_temp_status
				&& tbat_vooc_status == VOOC_TEMP_STATUS__NORMAL) {
			chip->limits.vooc_normal_bat_decidegc_antishake
				= chip->limits.vooc_normal_bat_decidegc + TVOOC_HYSTERISIS_DECIDEGC;
		} else if (tbat_vooc_status < chip->vooc_temp_status
				&& tbat_vooc_status == VOOC_TEMP_STATUS__WARM) {
			chip->limits.vooc_warm_bat_decidegc_antishake
				= chip->limits.vooc_warm_bat_decidegc + TVOOC_HYSTERISIS_DECIDEGC;
		}
		chg_debug("tled status change, [%d %d %d %d]\n",
			tbat_vooc_status, chip->vooc_temp_status,
		chip->limits.vooc_warm_bat_decidegc_antishake,
			chip->limits.vooc_normal_bat_decidegc_antishake);
		vooc_first_set_input_current_flag = true;
		chip->vooc_temp_change = true;
		chip->vooc_temp_status = tbat_vooc_status;
	}

}

static void oplus_chg_check_vooc_ichging(struct oplus_chg_chip *chip)
{
	if (chip->vbatt_num == 2 && chip->vooc_temp_change) {
		chip->vooc_temp_change = false;
		if (oplus_vooc_get_fast_chg_type() == CHARGER_SUBTYPE_FASTCHG_VOOC) {
			oplus_chg_set_input_current_limit(chip);
		}
	}
}

static void oplus_chg_check_cool_down_ichging(struct oplus_chg_chip *chip)
{
	chip->cool_down_done = false;
	if (oplus_vooc_get_allow_reading() == true) {
		if (chip->dual_charger_support) {
			chip->slave_charger_enable = false;
			oplus_chg_set_charging_current(chip);
		}
		oplus_chg_set_input_current_limit(chip);
	}
}

static void oplus_chg_check_camera_on_ichging(struct oplus_chg_chip *chip)
{
	static bool camera_pre = false;

	if (chip->camera_on != camera_pre) {
		camera_pre = chip->camera_on;
		if (oplus_vooc_get_fastchg_started() == true) {
			return;
		}
		if (oplus_vooc_get_allow_reading() == true) {
			if (chip->dual_charger_support) {
				chip->slave_charger_enable = false;
				oplus_chg_set_charging_current(chip);
			}
			oplus_chg_set_input_current_limit(chip);
		}
	}
}

static void oplus_chg_check_calling_on_ichging(struct oplus_chg_chip *chip)
{
	static bool calling_pre = false;

	if (chip->calling_on != calling_pre) {
		calling_pre = chip->calling_on;
		if (oplus_vooc_get_fastchg_started() == true) {
			return;
		}
		if (oplus_vooc_get_allow_reading() == true) {
			if (chip->dual_charger_support) {
				chip->slave_charger_enable = false;
				oplus_chg_set_charging_current(chip);
			}
			oplus_chg_set_input_current_limit(chip);
		}
	}
}

static void oplus_chg_battery_authenticate_check(struct oplus_chg_chip *chip)
{
	static bool charger_exist_pre = false;

	if (charger_exist_pre ^ chip->charger_exist) {
		charger_exist_pre = chip->charger_exist;
		if (chip->charger_exist && !chip->authenticate) {
			chip->authenticate = oplus_gauge_get_batt_authenticate();
		}
	}
}

void oplus_chg_variables_reset(struct oplus_chg_chip *chip, bool in)
{
	int i;
	if (in) {
		chip->charger_exist = true;
		chip->chging_on = true;
		chip->slave_charger_enable = false;
	} else {
		if(!oplus_chg_show_vooc_logo_ornot()) {
			if(chip->decimal_control) {
				cancel_delayed_work_sync(&g_charger_chip->ui_soc_decimal_work);
				chip->last_decimal_ui_soc = (chip->ui_soc_integer + chip->ui_soc_decimal);
				oplus_chg_ui_soc_decimal_deinit();
				pr_err("[oplus_chg_variables_reset]cancel last_decimal_ui_soc:%d", chip->last_decimal_ui_soc);
			}
			chip->calculate_decimal_time = 0;
		}
		oplus_pps_variables_reset();
		chip->allow_swtich_to_fastchg = 1;
		chip->charger_exist = false;
		chip->chging_on = false;
		chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		chip->real_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		vbatt_higherthan_4180mv = false;
		mcu_update = false;
		suspend_charger = false;
		chip->pd_svooc = false;
	}
	chip->limits.vbatt_pdqc_to_9v_thr = oplus_get_vbatt_pdqc_to_9v_thr();
	chip->sw_full_count = 0;
	chip->sw_full = false;
	chip->hw_full_by_sw = false;
	/*chip->charger_volt = 5000;*/
	chip->vchg_status = CHARGER_STATUS__GOOD;
	chip->batt_full = false;
	chip->tbatt_when_full = 200;
	chip->recharge_after_ffc = false;
	chip->tbatt_status = BATTERY_STATUS__NORMAL;
	chip->tbatt_pre_shake = TBATT_PRE_SHAKE_INVALID;
	chip->tbatt_normal_status = BATTERY_STATUS__NORMAL_PHASE1;
	chip->tbatt_normal_pre_shake = TBATT_PRE_SHAKE_INVALID;
	chip->vbatt_over = 0;
	chip->total_time = 0;
	chip->chging_over_time = 0;
	chip->in_rechging = 0;
	/*chip->batt_volt = 0;*/
	/*chip->temperature = 0;*/
	chip->stop_voter = 0x00;
	chip->charging_state = CHARGING_STATUS_CCCV;
	chip->oplus_pps_enter_count = 0;
#ifndef SELL_MODE
	if(chip->mmi_fastchg == 0){
		chip->mmi_chg = 0;
	} else {
		chip->mmi_chg = 1;
	}
#endif //SELL_MODE
	chip->unwakelock_chg = 0;
	chip->notify_code = 0;
	chip->notify_flag = 0;
	if (!((chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP)
				|| (oplus_vooc_get_fastchg_started() == true)
				|| (oplus_vooc_get_fastchg_to_warm() == true)
				|| (oplus_vooc_get_fastchg_dummy_started() == true)
				|| (oplus_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE)
				|| (oplus_vooc_get_btb_temp_over() == true))) {
		chip->cool_down = 0;
		chip->cool_down_done = false;
	}
	chip->cool_down_force_5v = false;
	chip->limits.cold_bat_decidegc = chip->anti_shake_bound.cold_bound;
	chip->limits.little_cold_bat_decidegc
		= chip->anti_shake_bound.little_cold_bound;
	chip->limits.cool_bat_decidegc = chip->anti_shake_bound.cool_bound;
	chip->limits.little_cool_bat_decidegc
		= chip->anti_shake_bound.little_cool_bound;
	chip->limits.normal_bat_decidegc = chip->anti_shake_bound.normal_bound;
	chip->limits.warm_bat_decidegc = chip->anti_shake_bound.warm_bound;
	chip->limits.hot_bat_decidegc = chip->anti_shake_bound.hot_bound;

	chip->limits.normal_phase1_bat_decidegc
		= chip->tbatt_normal_anti_shake_bound.phase1_bound;
	chip->limits.normal_phase2_bat_decidegc
                = chip->tbatt_normal_anti_shake_bound.phase2_bound;
	chip->limits.normal_phase3_bat_decidegc
                = chip->tbatt_normal_anti_shake_bound.phase3_bound;
	chip->limits.normal_phase4_bat_decidegc
                = chip->tbatt_normal_anti_shake_bound.phase4_bound;
	chip->limits.normal_phase5_bat_decidegc
                = chip->tbatt_normal_anti_shake_bound.phase5_bound;
	chip->limits.normal_phase6_bat_decidegc
                = chip->tbatt_normal_anti_shake_bound.phase6_bound;

	chip->limits.vfloat_over_counts = 0;
	chip->limits.led_warm_bat_decidegc_antishake
		= chip->limits.led_warm_bat_decidegc;
	chip->limits.led_high_bat_decidegc_antishake
		= chip->limits.led_high_bat_decidegc;
	chip->led_temp_change = false;
	chip->limits.vooc_warm_bat_decidegc_antishake
		= chip->limits.vooc_warm_bat_decidegc;
	chip->limits.vooc_normal_bat_decidegc_antishake
		= chip->limits.vooc_normal_bat_decidegc;
	chip->vooc_temp_change = false;
	if (chip->temperature > chip->limits.led_high_bat_decidegc) {
		chip->led_temp_status = LED_TEMP_STATUS__HIGH;
	}else if (chip->temperature > chip->limits.led_warm_bat_decidegc) {
		chip->led_temp_status = LED_TEMP_STATUS__WARM;
	}else{
		chip->led_temp_status = LED_TEMP_STATUS__NORMAL;
	}
	chip->dod0_counts = 0;
	chip->fastchg_to_ffc = false;
	chip->fastchg_ffc_status = 0;
	chip->chg_ctrl_by_lcd = chip->chg_ctrl_by_lcd_default;
	chip->chg_ctrl_by_vooc = chip->chg_ctrl_by_vooc_default;
	chip->ffc_temp_status = FFC_TEMP_STATUS__NORMAL;
	chip->vooc_temp_status = VOOC_TEMP_STATUS__NORMAL;
	chip->limits.iterm_ma = chip->limits.default_iterm_ma;
	chip->limits.temp_normal_fastchg_current_ma
		= chip->limits.default_temp_normal_fastchg_current_ma;
	chip->limits.normal_vfloat_sw_limit
		= chip->limits.default_normal_vfloat_sw_limit;
	chip->limits.temp_normal_vfloat_mv
		= chip->limits.default_temp_normal_vfloat_mv;
	chip->limits.normal_vfloat_over_sw_limit
		= chip->limits.default_normal_vfloat_over_sw_limit;
	chip->limits.temp_little_cool_fastchg_current_ma
		= chip->limits.default_temp_little_cool_fastchg_current_ma;
	chip->limits.little_cool_vfloat_sw_limit
		= chip->limits.default_little_cool_vfloat_sw_limit;
	chip->limits.temp_little_cool_vfloat_mv
		= chip->limits.default_temp_little_cool_vfloat_mv;
	chip->limits.little_cool_vfloat_over_sw_limit
		= chip->limits.default_little_cool_vfloat_over_sw_limit;
	chip->limits.temp_little_cool_fastchg_current_ma_high
		= chip->limits.default_temp_little_cool_fastchg_current_ma_high;
	chip->limits.temp_little_cool_fastchg_current_ma_low
		= chip->limits.default_temp_little_cool_fastchg_current_ma_low;
	chip->limits.temp_little_cold_fastchg_current_ma_high
		= chip->limits.default_temp_little_cold_fastchg_current_ma_high;
	chip->limits.temp_little_cold_fastchg_current_ma_low
		= chip->limits.default_temp_little_cold_fastchg_current_ma_low;
	chip->limits.temp_cool_fastchg_current_ma_high
		= chip->limits.default_temp_cool_fastchg_current_ma_high;
	chip->limits.temp_cool_fastchg_current_ma_low
		= chip->limits.default_temp_cool_fastchg_current_ma_low;
	chip->limits.temp_warm_fastchg_current_ma
		= chip->limits.default_temp_warm_fastchg_current_ma;
	chip->limits.input_current_charger_ma
		= chip->limits.default_input_current_charger_ma;
	chip->limits.input_current_vooc_ma_high
		= chip->limits.default_input_current_vooc_ma_high;
	chip->limits.input_current_vooc_ma_warm
		= chip->limits.default_input_current_vooc_ma_warm;
	chip->limits.input_current_vooc_ma_normal
		= chip->limits.default_input_current_vooc_ma_normal;
	chip->limits.pd_input_current_charger_ma
		= chip->limits.default_pd_input_current_charger_ma;
	chip->limits.qc_input_current_charger_ma
		= chip->limits.default_qc_input_current_charger_ma;
	reset_mcu_delay = 0;
	//if(oplus_pps_get_chg_status() != PPS_NOT_SUPPORT)
	//	oplus_start_svooc_reset();
#ifndef CONFIG_OPLUS_CHARGER_MTK
	chip->pmic_spmi.aicl_suspend = false;
#endif
	oplus_chg_battery_authenticate_check(chip);
	chip->chargerid_volt = 0;
	chip->chargerid_volt_got = false;
	chip->short_c_batt.in_idle = true;//defualt in idle for userspace
	chip->short_c_batt.cv_satus = false;//defualt not in cv chg
	chip->short_c_batt.disable_rechg = false;
	chip->short_c_batt.limit_chg = false;
	chip->short_c_batt.limit_rechg = false;
	chip->chg_ctrl_by_cool_down = false;
	chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
	chip->usbtemp_cool_down = 0;
	chip->pd_chging = false;
	chip->bcc_cool_down = 0;
	chip->bcc_curr_done = BCC_CURR_DONE_UNKNOW;
	for (i = 0; i < ARRAY_SIZE(chip->ibat_save); i++)
		chip->ibat_save[i] = 500;
}

static void oplus_chg_variables_init(struct oplus_chg_chip *chip)
{
	chip->charger_exist = false;
	chip->chging_on = false;
	chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->real_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->charger_volt = 0;
	chip->vchg_status = CHARGER_STATUS__GOOD;
	chip->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	chip->sw_full_count = 0;
	chip->sw_full = false;
	chip->hw_full_by_sw = false;
	chip->batt_exist = true;
	chip->batt_full = false;
	chip->tbatt_when_full = 200;
	chip->recharge_after_ffc = false;
	chip->tbatt_status = BATTERY_STATUS__NORMAL;
	chip->tbatt_normal_status = BATTERY_STATUS__NORMAL_PHASE1;
	chip->vbatt_over = 0;
	chip->total_time = 0;
	chip->chging_over_time = 0;
	chip->in_rechging = 0;
	//chip->batt_volt = 3800 * chip->vbatt_num;
	chip->batt_volt = 3800;
	chip->icharging = 0;
	chip->temperature = 250;
	chip->shell_temp = TEMPERATURE_INVALID;
	chip->soc = 0;
	chip->ui_soc = 50;
	chip->notify_code = 0;
	chip->notify_flag = 0;
	chip->cool_down = 0;
	chip->tbatt_pre_shake = TBATT_PRE_SHAKE_INVALID;
	chip->tbatt_normal_pre_shake = TBATT_PRE_SHAKE_INVALID;
	chip->led_on = true;
	chip->camera_on = 0;
	chip->stop_voter = 0x00;
	chip->charging_state = CHARGING_STATUS_CCCV;
	chip->mmi_chg = 1;
	chip->unwakelock_chg = 0;
	chip->chg_powersave = false;
	chip->allow_swtich_to_fastchg = 1;
	chip->stop_chg= 1;
	chip->mmi_fastchg = 1;
	chip->cool_down_done = false;
	chip->healthd_ready = false;
	chip->dischg_flag = false;
	chip->usb_status = 0;
	init_waitqueue_head( &chip->oplus_usbtemp_wq);
	chip->smooth_to_soc_gap = 5;
	chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
	chip->usbtemp_cool_down = 0;
#ifdef CONFIG_OPLUS_CHARGER_MTK
	chip->usb_online = false;
	chip->otg_online = false;
#else
/*	chip->pmic_spmi.usb_online = false;
		IC have init already	*/
#endif
	if(chip->external_gauge) {
		chg_debug("use oplus_gauge_get_batt_authenticate\n");
		chip->authenticate = oplus_gauge_get_batt_authenticate();
		chip->hmac = oplus_gauge_get_batt_hmac();
	} else {
		chg_debug("use get_oplus_high_battery_status\n");
		//chip->authenticate = get_oplus_high_battery_status();
		chip->authenticate = oplus_gauge_get_batt_authenticate();
		chip->hmac = true;
	}
	if (!chip->authenticate) {
		//chip->chg_ops->charger_suspend();
		chip->chg_ops->charging_disable();
	}
	chip->otg_switch = false;
	chip->ui_otg_switch = false;
	chip->boot_mode = chip->chg_ops->get_boot_mode();
	chip->boot_reason = chip->chg_ops->get_boot_reason();
	chip->anti_shake_bound.cold_bound = chip->limits.cold_bat_decidegc;
	chip->anti_shake_bound.little_cold_bound
		= chip->limits.little_cold_bat_decidegc;
	chip->anti_shake_bound.cool_bound = chip->limits.cool_bat_decidegc;
	chip->anti_shake_bound.little_cool_bound
		= chip->limits.little_cool_bat_decidegc;
	chip->anti_shake_bound.normal_bound = chip->limits.normal_bat_decidegc;
	chip->anti_shake_bound.warm_bound = chip->limits.warm_bat_decidegc;
	chip->anti_shake_bound.hot_bound = chip->limits.hot_bat_decidegc;

	chip->tbatt_normal_anti_shake_bound.phase1_bound
		= chip->limits.normal_phase1_bat_decidegc;
	chip->tbatt_normal_anti_shake_bound.phase2_bound
                = chip->limits.normal_phase2_bat_decidegc;
	chip->tbatt_normal_anti_shake_bound.phase3_bound
                = chip->limits.normal_phase3_bat_decidegc;
	chip->tbatt_normal_anti_shake_bound.phase4_bound
                = chip->limits.normal_phase4_bat_decidegc;
	chip->tbatt_normal_anti_shake_bound.phase5_bound
                = chip->limits.normal_phase5_bat_decidegc;
	chip->tbatt_normal_anti_shake_bound.phase6_bound
                = chip->limits.normal_phase6_bat_decidegc;

	chip->limits.led_warm_bat_decidegc_antishake
		= chip->limits.led_warm_bat_decidegc;
	chip->limits.led_high_bat_decidegc_antishake
		= chip->limits.led_high_bat_decidegc;
	chip->led_temp_change = false;
	chip->limits.vooc_warm_bat_decidegc_antishake
		= chip->limits.vooc_warm_bat_decidegc;
	chip->limits.vooc_normal_bat_decidegc_antishake
		= chip->limits.vooc_normal_bat_decidegc;
	chip->vooc_temp_change = false;

	if (chip->temperature > chip->limits.led_high_bat_decidegc)
		chip->led_temp_status = LED_TEMP_STATUS__HIGH;
	else if (chip->temperature > chip->limits.led_warm_bat_decidegc)
		chip->led_temp_status = LED_TEMP_STATUS__WARM;
	else
		chip->led_temp_status = LED_TEMP_STATUS__NORMAL;
//	chip->anti_shake_bound.overtemp_bound = chip->limits.overtemp_bat_decidegc;
	chip->limits.vfloat_over_counts = 0;
	chip->chargerid_volt = 0;
	chip->chargerid_volt_got = false;
	chip->enable_shipmode = 0;
	chip->dod0_counts = 0;
	chip->fastchg_to_ffc = false;
	chip->fastchg_ffc_status = 0;
	chip->ffc_temp_status = FFC_TEMP_STATUS__NORMAL;
	chip->vooc_temp_status = VOOC_TEMP_STATUS__NORMAL;
	chip->short_c_batt.err_code = oplus_short_c_batt_err_code_init();
	chip->short_c_batt.is_switch_on = oplus_short_c_batt_chg_switch_init();
	chip->short_c_batt.is_feature_sw_on
		= oplus_short_c_batt_feature_sw_status_init();
	chip->short_c_batt.is_feature_hw_on
		= oplus_short_c_batt_feature_hw_status_init();
	chip->short_c_batt.shortc_gpio_status = 1;
	chip->short_c_batt.disable_rechg = false;
	chip->short_c_batt.limit_chg = false;
	chip->short_c_batt.limit_rechg = false;
	chip->slave_charger_enable = false;
	chip->cool_down_force_5v = false;
	chip->chg_ctrl_by_cool_down = false;

	chip->ui_soc_decimal = 0;
	chip->ui_soc_integer = 0;
	chip->last_decimal_ui_soc = 0;
	chip->decimal_control = false;
	chip->calculate_decimal_time = 0;
	chip->boot_completed = false;
	chip->pd_chging = false;
	chip->pd_svooc = false;
	chip->usbtemp_check = false;
	chip->em_mode = false;
	chip->oplus_pps_enter_count = 0;
	chip->bcc_cool_down = 0;
	mutex_init(&chip->bcc_curr_done_mutex);
	chip->bcc_curr_done = BCC_CURR_DONE_UNKNOW;
}

static void oplus_chg_fail_action(struct oplus_chg_chip *chip)
{
	chg_err("[BATTERY] BAD Battery status... Charging Stop !!\n");
	chip->charging_state = CHARGING_STATUS_FAIL;
	chip->chging_on = false;
	chip->batt_full = false;
	chip->tbatt_when_full = 200;
	chip->in_rechging = 0;
}

#define D_RECHGING_CNT	5
static void oplus_chg_check_rechg_status(struct oplus_chg_chip *chip)
{
	int recharging_vol;
	int nbat_vol = chip->batt_volt;
	static int rechging_cnt = 0;

	if (chip->mmi_chg == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, " mmi_chg,return\n");
		return;
	}

	if (chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {				//4.0
		recharging_vol = oplus_chg_get_float_voltage(chip) - 300;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {//4.4
		recharging_vol = oplus_chg_get_float_voltage(chip) - 200;
	} else {
		recharging_vol = oplus_chg_get_float_voltage(chip);//warm 4.1
		if (recharging_vol > chip->limits.temp_normal_vfloat_mv) {
			recharging_vol = chip->limits.temp_normal_vfloat_mv;
		}
		recharging_vol = recharging_vol - chip->limits.recharge_mv;
	}
	if ((!chip->authenticate) ||(!chip->hmac)) {
		recharging_vol = chip->limits.non_standard_vfloat_mv - 400;//3.80
	}
	if (nbat_vol <= recharging_vol) {
		rechging_cnt++;
	} else {
		rechging_cnt = 0;
	}

	/*don't rechg here unless prohibit rechg is false*/
	if (oplus_short_c_batt_is_disable_rechg(chip)) {
		if (rechging_cnt >= D_RECHGING_CNT) {
			charger_xlog_printk(CHG_LOG_CRTI,
				"[Battery] disable rechg! batt_volt = %d, nReChgingVol = %d\r\n",
				nbat_vol, recharging_vol);
			rechging_cnt = D_RECHGING_CNT;
		}
	}
	if (rechging_cnt > D_RECHGING_CNT) {
		charger_xlog_printk(CHG_LOG_CRTI,
			"[Battery] Battery rechg begin! batt_volt = %d, recharging_vol = %d\n",
			nbat_vol, recharging_vol);
		rechging_cnt = 0;
		chip->in_rechging = true;
		oplus_chg_voter_charging_start(chip, CHG_STOP_VOTER__FULL);/*now rechging!*/
	}
}

static void oplus_chg_full_action(struct oplus_chg_chip *chip)
{
	charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] Battery full !!\n");
	oplus_chg_voter_charging_stop(chip, CHG_STOP_VOTER__FULL);
	/*chip->charging_state = CHARGING_STATUS_FULL;*/
	if(chip->batt_full == false) {
		chip->tbatt_when_full = chip->temperature;
	}
	chip->batt_full = true;
	chip->total_time = 0;
	chip->in_rechging = false;
	chip->limits.vfloat_over_counts = 0;
	oplus_chg_check_rechg_status(chip);
}
//#ifndef CONFIG_OPLUS_CHARGER_MTK
extern bool oplus_wpc_get_fw_updating(void);
//#endif
extern int mp2650_wireless_get_mps_otg_en_val(void);
void oplus_charger_detect_check(struct oplus_chg_chip *chip)
{
	static bool charger_resumed = true;
#ifdef CONFIG_OPLUS_CHARGER_MTK
	static int charger_flag = 0;
#endif

	if (chip->chg_ops->check_chrdet_status()) {
		oplus_chg_set_awake(chip, true);
//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
		if (chip->wireless_support && (oplus_wpc_get_fw_updating()
			|| oplus_chg_is_wls_fw_upgrading())) {
			return;
		}
		//if (chip->wireless_support && oplus_wpc_get_fw_updating() == true) {
		//	return;
		//}
//#endif
		if (chip->charger_type == POWER_SUPPLY_TYPE_UNKNOWN) {
			noplug_temperature = chip->tbatt_temp;
			noplug_batt_volt_max = chip->batt_volt_max;
			noplug_batt_volt_min = chip->batt_volt_min;
			oplus_chg_variables_reset(chip, true);
#ifdef CONFIG_OPLUS_CHARGER_MTK
			if(is_meta_mode() == true){
				chip->charger_type = POWER_SUPPLY_TYPE_USB;
				chip->real_charger_type = POWER_SUPPLY_TYPE_USB;
			} else {
				chip->charger_type = chip->chg_ops->get_charger_type();
				if(chip->chg_ops->get_real_charger_type) {
					chip->real_charger_type = chip->chg_ops->get_real_charger_type();
				}
			}
			if((chip->chg_ops->usb_connect)
					&& (chip->charger_type == POWER_SUPPLY_TYPE_USB
					|| chip->charger_type == POWER_SUPPLY_TYPE_USB_CDP)) {
				chip->chg_ops->usb_connect();
				charger_flag = 1;
			}
#else
			chip->charger_type = chip->chg_ops->get_charger_type();
			if(chip->chg_ops->get_real_charger_type) {
				chip->real_charger_type = chip->chg_ops->get_real_charger_type();
			}
#endif
			charger_xlog_printk(CHG_LOG_CRTI, "Charger in 1 charger_type=%d\n",
				chip->charger_type);
			if (oplus_vooc_get_fastchg_to_normal() == true
					|| oplus_vooc_get_fastchg_to_warm() == true) {
				charger_xlog_printk(CHG_LOG_CRTI,
					"fast_to_normal or to_warm 1,don't turn on charge here\n");
				if (oplus_vooc_get_reset_adapter_st()) {
					oplus_chg_unsuspend_charger();
				}
			}
//for dx1 bringup
#if 1//ndef CONFIG_OPLUS_CHARGER_MTK
			else if (!oplus_chg_is_wls_present()) {
				charger_xlog_printk(CHG_LOG_CRTI, "oplus chg wls not present\n");
				charger_resumed = chip->chg_ops->check_charger_resume();
				oplus_chg_turn_on_charging(chip);
			}
#else
			else {
				chg_err("enable charging\n");
				charger_resumed = chip->chg_ops->check_charger_resume();
				oplus_chg_turn_on_charging(chip);
			}
#endif
			/*chg_err("Charger in, charger_type=%d\n", chip->charger_type);*/
		} else {
			if (oplus_vooc_get_fastchg_to_normal() == true
					|| oplus_vooc_get_fastchg_to_warm() == true) {
				if (oplus_vooc_get_reset_adapter_st()) {
					oplus_chg_unsuspend_charger();
				}
				charger_xlog_printk(CHG_LOG_CRTI,
					"fast_to_normal or to_warm 2,don't turn on charge here\n");
				if (oplus_vooc_get_reset_adapter_st()) {
					oplus_chg_unsuspend_charger();
				}
			} else if (oplus_vooc_get_fastchg_started() == false
					&& charger_resumed == false) {
				charger_resumed = chip->chg_ops->check_charger_resume();
				oplus_chg_turn_on_charging(chip);
			}
		}
	} else {
		oplus_chg_variables_reset(chip, false);
		if (!chip->mmi_fastchg) {
			oplus_mmi_fastchg_in(chip);
		}
		oplus_gauge_set_batt_full(false);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		if (chip->chg_ops->usb_disconnect && charger_flag == 1) {
			chip->chg_ops->usb_disconnect();
			charger_flag = 0;
		}
#endif
		if (chip->chg_ops->get_charging_enable() == true) {
			oplus_chg_turn_off_charging(chip);
		}
#ifdef CONFIG_OPLUS_CHARGER_MTK
		if (chip->boot_mode != KERNEL_POWER_OFF_CHARGING_BOOT) {
			oplus_chg_set_awake(chip, false);
		}
#else
		oplus_chg_set_awake(chip, false);
#endif
	}
}
static void oplus_get_smooth_soc_switch(struct oplus_chg_chip *chip)
{
/*
	char *substr;
	static int save_num = -1;

	if (save_num == -1) {
		substr = strstr(saved_command_line, "smooth_soc_switch=");
		if(NULL == substr) {
			save_num = 0;
		} else {
			substr += strlen("smooth_soc_switch=");
			if (strncmp(substr, "1", 1) == 0) {
				save_num = 1;
			} else
				save_num = 0;
		}
	}

	chip->smooth_switch = save_num;
	if(chip->vbatt_num == 1)
		chip->smooth_switch = 1;
	else
		chip->smooth_switch = 0;*/
	chg_debug("smooth_switch = %d\n", chip->smooth_switch);
}

#define RETRY_COUNTS	24
static void oplus_chg_get_battery_data(struct oplus_chg_chip *chip)
{
	static int ui_soc_cp_flag = 0;
	static int soc_load = 0;
	int remain_100_thresh = 97;
	static int retry_counts = 0;

	if (oplus_vooc_get_fastchg_started() == true) {
		chip->batt_volt = oplus_gauge_get_prev_batt_mvolts();
		chip->batt_volt_max = oplus_gauge_get_prev_batt_mvolts_2cell_max();
		chip->batt_volt_min = oplus_gauge_get_prev_batt_mvolts_2cell_min();
		chip->icharging = oplus_gauge_get_prev_batt_current();
		chip->temperature = oplus_chg_match_temp_for_chging();
		chip->soc = oplus_gauge_get_prev_batt_soc();
		chip->batt_rm = oplus_gauge_get_prev_remaining_capacity() * chip->vbatt_num;
	} else {
		chip->batt_volt = oplus_gauge_get_batt_mvolts();
		chip->batt_volt_max = oplus_gauge_get_batt_mvolts_2cell_max();
		chip->batt_volt_min = oplus_gauge_get_batt_mvolts_2cell_min();
		chip->icharging = oplus_gauge_get_batt_current();
		chip->temperature = oplus_chg_match_temp_for_chging();
		chip->soc = oplus_gauge_get_batt_soc();
		chip->batt_fcc = oplus_gauge_get_batt_fcc() * chip->vbatt_num;
		chip->batt_cc = oplus_gauge_get_batt_cc() * chip->vbatt_num;
		chip->batt_soh = oplus_gauge_get_batt_soh();
		chip->batt_rm = oplus_gauge_get_remaining_capacity() * chip->vbatt_num;
		oplus_midas_chg_processing(chip);
	}
	if (chgr_dbg_vchg != 0) {
		chip->charger_volt = chgr_dbg_vchg;
	} else {
		if (oplus_vooc_get_allow_reading() == true) {
			chip->charger_volt = chip->chg_ops->get_charger_volt();
#ifndef CONFIG_OPLUS_CHARGER_MTK
			if (chip->charger_volt > 3000 && chip->charger_exist) {
				chip->usbtemp_check = true;
				wake_up_interruptible(&chip->oplus_usbtemp_wq);
			} else {
				chip->usbtemp_check = false;
			}
#endif
		}
	}
	if ((chip->chg_ops->get_charger_current && oplus_vooc_get_allow_reading() == true)
                && (chip->chg_ops->get_charging_enable && (chip->chg_ops->get_charging_enable() == true))) {
		chip->ibus = chip->chg_ops->get_charger_current();
	} else {
		chip->ibus = -1;
	}
	if(chip->smooth_switch ){
		oplus_chg_smooth_to_soc(chip);
	} else {
		chip->smooth_soc = chip->soc;
	}
	if(!chip->healthd_ready && chip->smooth_switch == 1 && retry_counts < RETRY_COUNTS) {
		chg_err(" test gauge soc[%d] \n", chip->soc);
		chip->soc = -1;
	}
	if (ui_soc_cp_flag == 0) {
		if ((chip->soc < 0 || chip->soc > 100) && retry_counts < RETRY_COUNTS) {
			charger_xlog_printk(CHG_LOG_CRTI,
				"[Battery]oplus_chg_get_battery_data,\
				chip->soc[%d],retry_counts[%d]\n",
				chip->soc, retry_counts);
			retry_counts++;
			chip->soc = 50;
			goto next;
		}
		ui_soc_cp_flag = 1;
		if( chip->chg_ops->get_rtc_soc() > 100 || (chip->chg_ops->get_rtc_soc() <= 0)){
			soc_load = chip->soc;
		} else {
			soc_load = chip->chg_ops->get_rtc_soc();
		}
		chip->soc_load = soc_load;
		if(chip->smooth_switch)
			chip->smooth_soc = soc_load;
		if ((chip->soc < 0 || chip->soc > 100) && soc_load > 0 && soc_load <= 100) {
			chip->soc = soc_load;
		}
		if ((soc_load != 0) && ((abs(soc_load-chip->soc)) <= 20)) {
			if (chip->suspend_after_full && chip->external_gauge) {
				remain_100_thresh = 95;
			} else if (chip->suspend_after_full && !chip->external_gauge) {
				remain_100_thresh = 94;
			} else if (!chip->suspend_after_full && chip->external_gauge) {
				remain_100_thresh = 97;
			} else if (!chip->suspend_after_full && !chip->external_gauge) {
				remain_100_thresh = 95;
			} else {
				remain_100_thresh = 97;
			}
			if (chip->soc < soc_load &&	chip->smooth_switch == 1) {
				if (soc_load == 100 && chip->soc > remain_100_thresh) {
					chip->ui_soc = soc_load;
				} else {
					chip->ui_soc = soc_load - 1;
				}
			} else {
				chip->ui_soc = soc_load;
			}
		} else {
			chip->ui_soc = chip->soc;
			if (!chip->external_gauge && soc_load == 0 && chip->soc < 5) {
				chip->ui_soc = 0;
			}
		}
		chg_err("[soc ui_soc soc_load smooth_soc smooth_switch] = [%d %d %d %d %d]\n", chip->soc,
			chip->ui_soc, chip->soc_load, chip->smooth_soc, chip->smooth_switch);

	}
	next:
	return;
}

/*need to extend it*/
static void oplus_chg_set_aicl_point(struct oplus_chg_chip *chip)
{
	if (oplus_vooc_get_allow_reading() == true) {
		chip->chg_ops->set_aicl_point(chip->batt_volt);
	}
}

#define AICL_DELAY_15MIN	180
static void oplus_chg_check_aicl_input_limit(struct oplus_chg_chip *chip)
{
	static int aicl_delay_count = 0;
#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (chip->charging_state == CHARGING_STATUS_FAIL || chip->batt_full == true
			|| ((chip->tbatt_status != BATTERY_STATUS__NORMAL)
			&& (chip->tbatt_status != BATTERY_STATUS__LITTLE_COOL_TEMP))
			|| chip->ui_soc > 85 || oplus_vooc_get_fastchg_started() == true) {
		aicl_delay_count = 0;
		return;
	}
	if (aicl_delay_count > AICL_DELAY_15MIN) {
		aicl_delay_count = 0;
		if (oplus_vooc_get_allow_reading() == true) {
			if (chip->dual_charger_support) {
				chip->slave_charger_enable = false;
				oplus_chg_set_charging_current(chip);
			}
			oplus_chg_set_input_current_limit(chip);
		}
	} else {
		aicl_delay_count++;
	}
#else
	if (chip->charging_state == CHARGING_STATUS_FAIL || chip->batt_full == true
			|| ((chip->tbatt_status != BATTERY_STATUS__NORMAL)
			&& (chip->tbatt_status != BATTERY_STATUS__LITTLE_COOL_TEMP))
			|| ((chip->ui_soc > 85) && (chip->pmic_spmi.aicl_suspend == false))
			|| oplus_vooc_get_fastchg_started() == true) {
		aicl_delay_count = 0;
		return;
	}
	if (aicl_delay_count > AICL_DELAY_15MIN) {
		aicl_delay_count = 0;
		if (oplus_vooc_get_allow_reading() == true) {
			oplus_chg_set_input_current_limit(chip);
		}
	} else if (chip->pmic_spmi.aicl_suspend == true
			&& chip->charger_volt > 4450
			&& chip->charger_volt < 5800) {
		aicl_delay_count = 0;
		if (oplus_vooc_get_allow_reading() == true) {
			chip->chg_ops->rerun_aicl();
			oplus_chg_set_input_current_limit(chip);
		}
		charger_xlog_printk(CHG_LOG_CRTI, "chip->charger_volt=%d\n", chip->charger_volt);
	} else {
		aicl_delay_count++;
	}
	if (chip->charger_type == POWER_SUPPLY_TYPE_USB
			|| chip->charger_type == POWER_SUPPLY_TYPE_USB_CDP) {
		chip->pmic_spmi.usb_hc_count++;
		if (chip->pmic_spmi.usb_hc_count >= 3) {
			chip->pmic_spmi.usb_hc_mode = true;
			chip->pmic_spmi.usb_hc_count = 3;
		}
	}
	if (oplus_vooc_get_allow_reading() == true
			&& chip->pmic_spmi.usb_hc_mode && !chip->pmic_spmi.hc_mode_flag) {
		oplus_chg_set_input_current_limit(chip);
		chip->pmic_spmi.hc_mode_flag = true;
	}
#endif
}

static void oplus_chg_aicl_check(struct oplus_chg_chip *chip)
{
	if (oplus_vooc_get_fastchg_started() == false) {
		oplus_chg_set_aicl_point(chip);
		oplus_chg_check_aicl_input_limit(chip);
	}
}

static void oplus_chg_protection_check(struct oplus_chg_chip *chip)
{
	if (false == oplus_chg_check_tbatt_is_good(chip)) {
		chg_err("oplus_chg_check_tbatt_is_good func ,false!\n");
		oplus_chg_voter_charging_stop(chip, CHG_STOP_VOTER__BATTTEMP_ABNORMAL);
	} else {
		if ((chip->stop_voter & CHG_STOP_VOTER__BATTTEMP_ABNORMAL)
				== CHG_STOP_VOTER__BATTTEMP_ABNORMAL) {
			charger_xlog_printk(CHG_LOG_CRTI,
				"oplus_chg_check_tbatt_is_good func ,true! To Normal\n");
			oplus_chg_voter_charging_start(chip, CHG_STOP_VOTER__BATTTEMP_ABNORMAL);
		}
	}
	if (false == oplus_chg_check_vchg_is_good(chip)) {
		chg_err("oplus_chg_check_vchg_is_good func ,false!\n");
		oplus_chg_voter_charging_stop(chip, CHG_STOP_VOTER__VCHG_ABNORMAL);
	} else {
		if ((chip->stop_voter & CHG_STOP_VOTER__VCHG_ABNORMAL)
				== CHG_STOP_VOTER__VCHG_ABNORMAL) {
			charger_xlog_printk(CHG_LOG_CRTI,
				"oplus_chg_check_vchg_is_good func ,true! To Normal\n");
			oplus_chg_voter_charging_start(chip, CHG_STOP_VOTER__VCHG_ABNORMAL);
		}
	}
#ifdef FEATURE_VBAT_PROTECT
	if (false == oplus_chg_check_vbatt_is_good(chip)) {
		chg_err("oplus_chg_check_vbatt_is_good func ,false!\n");
		oplus_chg_voter_charging_stop(chip, CHG_STOP_VOTER__VBAT_TOO_HIGH);
	}
#endif
	if (false == oplus_chg_check_time_is_good(chip)) {
		chg_err("oplus_chg_check_time_is_good func ,false!\n");
		oplus_chg_voter_charging_stop(chip, CHG_STOP_VOTER__MAX_CHGING_TIME);
	}
	oplus_chg_check_vbatt_higher_than_4180mv(chip);
	oplus_chg_vfloat_over_check(chip);
	oplus_chg_check_ffc_temp_status(chip);
	if (chip->chg_ctrl_by_lcd) {
		oplus_chg_check_tled_status(chip);
		oplus_chg_check_led_on_ichging(chip);
	}
	if (chip->chg_ctrl_by_camera) {
		oplus_chg_check_camera_on_ichging(chip);
	}
	if (chip->chg_ctrl_by_calling) {
		oplus_chg_check_calling_on_ichging(chip);
	}
	if (chip->chg_ctrl_by_vooc) {
		oplus_chg_check_vooc_temp_status(chip);
		oplus_chg_check_vooc_ichging(chip);
	}
	if (chip->cool_down_done) {
		oplus_chg_check_cool_down_ichging(chip);
	}
}


static void battery_notify_tbat_check(struct oplus_chg_chip *chip)
{
	static int count_removed = 0;
	static int count_high = 0;
	if (BATTERY_STATUS__HIGH_TEMP == chip->tbatt_status) {
		count_high++;
		charger_xlog_printk(CHG_LOG_CRTI,
				"[BATTERY] bat_temp(%d), BATTERY_STATUS__HIGH_TEMP count[%d]\n",
				chip->temperature, count_high);
		if (chip->charger_exist && count_high > 10) {
			count_high = 11;
			chip->notify_code |= 1 << NOTIFY_BAT_OVER_TEMP;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] bat_temp(%d) > 55'C\n",
				chip->temperature);
		}
	} else {
		count_high = 0;
	}
	if (BATTERY_STATUS__LOW_TEMP == chip->tbatt_status) {
		if (chip->charger_exist) {
			chip->notify_code |= 1 << NOTIFY_BAT_LOW_TEMP;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] bat_temp(%d) < -10'C\n",
				chip->temperature);
		}
	}
	if (BATTERY_STATUS__REMOVED == chip->tbatt_status) {
		count_removed ++;
		charger_xlog_printk(CHG_LOG_CRTI,
				"[BATTERY] bat_temp(%d), BATTERY_STATUS__REMOVED count[%d]\n",
				chip->temperature, count_removed);
		if (count_removed > 10) {
			count_removed = 11;
			chip->notify_code |= 1 << NOTIFY_BAT_NOT_CONNECT;
			charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] bat_temp(%d) < -19'C\n",
				chip->temperature);
		}
	} else {
		count_removed = 0;
	}
}

static void battery_notify_authenticate_check(struct oplus_chg_chip *chip)
{
	if (!chip->authenticate) {
		chip->notify_code |= 1 << NOTIFY_BAT_NOT_CONNECT;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] bat_authenticate is false!\n");
	}
}
static void battery_notify_hmac_check(struct oplus_chg_chip *chip)
{
	if (!chip->hmac) {
		chip->notify_code |= 1 << NOTIFY_BAT_FULL_THIRD_BATTERY;
		charger_xlog_printk(CHG_LOG_CRTI, "[BATTERY] bat_hmac is false!\n");
	}
}

static void battery_notify_vcharger_check(struct oplus_chg_chip *chip)
{
	if (CHARGER_STATUS__VOL_HIGH == chip->vchg_status) {
		chip->notify_code |= 1 << NOTIFY_CHARGER_OVER_VOL;
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] check_charger_off_vol(%d) > 5800mV\n", chip->charger_volt);
	}

	if (CHARGER_STATUS__VOL_LOW == chip->vchg_status) {
		chip->notify_code |= 1 << NOTIFY_CHARGER_LOW_VOL;
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] check_charger_off_vol(%d) < 3400mV\n", chip->charger_volt);
	}
}

static void battery_notify_vbat_check(struct oplus_chg_chip *chip)
{
	static int count = 0;

	if (true == chip->vbatt_over) {
		count++;
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] Battery is over VOL, count[%d] \n", count);
		if (count > 10) {
			count = 11;
			chip->notify_code |= 1 << NOTIFY_BAT_OVER_VOL;
			charger_xlog_printk(CHG_LOG_CRTI,
				"[BATTERY] Battery is over VOL! Notify \n");
		}
	} else {
		count = 0;
		if ((chip->batt_full) && (chip->charger_exist)) {
			if (chip->tbatt_status == BATTERY_STATUS__WARM_TEMP
					&& chip->ui_soc != 100) {
				chip->notify_code |=  1 << NOTIFY_BAT_FULL_PRE_HIGH_TEMP;
			} else if ((chip->tbatt_status == BATTERY_STATUS__COLD_TEMP)
					&& (chip->ui_soc != 100)) {
				chip->notify_code |=  1 << NOTIFY_BAT_FULL_PRE_LOW_TEMP;
			} else if (!chip->authenticate) {
				chip->notify_code |=  1 << NOTIFY_BAT_NOT_CONNECT;
			} else if (!chip->hmac) {
				chip->notify_code |=  1 << NOTIFY_BAT_FULL_THIRD_BATTERY;
			} else {
				if (chip->ui_soc == 100) {
					chip->notify_code |=  1 << NOTIFY_BAT_FULL;
				}
			}
			charger_xlog_printk(CHG_LOG_CRTI,
					"[BATTERY] FULL,tbatt_status:%d,notify_code:%d\n",
				chip->tbatt_status, chip->notify_code);
		}
	}
}

static void battery_notify_max_charging_time_check(struct oplus_chg_chip *chip)
{
	if (true == chip->chging_over_time) {
		chip->notify_code |= 1 << NOTIFY_CHGING_OVERTIME;
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] Charging is OverTime!Notify \n");
	}
}

static void battery_notify_short_c_battery_check(struct oplus_chg_chip *chip)
{
	if (chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__CV_ERR_CODE1) {
		chip->notify_code |= 1 << NOTIFY_SHORT_C_BAT_CV_ERR_CODE1;
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] battery is short circuit! err_code1!\n");
	}

	if (chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__FULL_ERR_CODE2) {
		chip->notify_code |= 1 << NOTIFY_SHORT_C_BAT_FULL_ERR_CODE2;
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] battery is short circuit! err_code2!\n");
		}

	if (chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__FULL_ERR_CODE3) {
		chip->notify_code |= 1 << NOTIFY_SHORT_C_BAT_FULL_ERR_CODE3;
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] battery is short circuit! err_code3!\n");
	}

	if (chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE4) {
		chip->notify_code |= 1 << NOTIFY_SHORT_C_BAT_DYNAMIC_ERR_CODE4;
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] battery is short circuit! err_code4!\n");
	}

	if (chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE5) {
		chip->notify_code |= 1 << NOTIFY_SHORT_C_BAT_DYNAMIC_ERR_CODE5;
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] battery is short circuit! err_code5!\n");
	}
}

static void battery_notify_flag_check(struct oplus_chg_chip *chip)
{
	if (chip->notify_code & (1 << NOTIFY_CHGING_OVERTIME)) {
		chip->notify_flag = NOTIFY_CHGING_OVERTIME;
	} else if (chip->notify_code & (1 << NOTIFY_CHARGER_OVER_VOL)) {
		chip->notify_flag = NOTIFY_CHARGER_OVER_VOL;
	} else if (chip->notify_code & (1 << NOTIFY_CHARGER_LOW_VOL)) {
		chip->notify_flag = NOTIFY_CHARGER_LOW_VOL;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_OVER_TEMP)) {
		chip->notify_flag = NOTIFY_BAT_OVER_TEMP;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_LOW_TEMP)) {
		chip->notify_flag = NOTIFY_BAT_LOW_TEMP;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_NOT_CONNECT)) {
		chip->notify_flag = NOTIFY_BAT_NOT_CONNECT;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_FULL_THIRD_BATTERY)) {
		chip->notify_flag = NOTIFY_BAT_FULL_THIRD_BATTERY;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_OVER_VOL)) {
		chip->notify_flag = NOTIFY_BAT_OVER_VOL;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_FULL_PRE_HIGH_TEMP)) {
		chip->notify_flag = NOTIFY_BAT_FULL_PRE_HIGH_TEMP;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_FULL_PRE_LOW_TEMP)) {
		chip->notify_flag = NOTIFY_BAT_FULL_PRE_LOW_TEMP;
	} else if (chip->notify_code & (1 << NOTIFY_BAT_FULL)) {
		chip->notify_flag = NOTIFY_BAT_FULL;
	} else {
		chip->notify_flag = 0;
	}
}

static void battery_notify_charge_terminal_check(struct oplus_chg_chip *chip)
{
	if (chip->batt_full == true && chip->charger_exist == true){
		chip->notify_code |= 1 << NOTIFY_CHARGER_TERMINAL;
	}
}

static void battery_notify_gauge_i2c_err_check(struct oplus_chg_chip *chip)
{
	if (oplus_gauge_get_i2c_err() > 0) {
		chip->notify_code |= 1 << NOTIFY_GAUGE_I2C_ERR;
	}

	oplus_gauge_clear_i2c_err();
}

static void oplus_chg_battery_notify_check(struct oplus_chg_chip *chip)
{
	chip->notify_code = 0x0000;
	battery_notify_tbat_check(chip);
	battery_notify_authenticate_check(chip);
	battery_notify_hmac_check(chip);
	battery_notify_vcharger_check(chip);
	battery_notify_vbat_check(chip);
	battery_notify_max_charging_time_check(chip);
	battery_notify_short_c_battery_check(chip);
	battery_notify_charge_terminal_check(chip);
	battery_notify_gauge_i2c_err_check(chip);
	battery_notify_flag_check(chip);
}

int oplus_chg_get_prop_batt_health(struct oplus_chg_chip *chip)
{
	int bat_health = POWER_SUPPLY_HEALTH_GOOD;
	bool vbatt_over = chip->vbatt_over;
	OPLUS_CHG_TBATT_STATUS tbatt_status = chip->tbatt_status;

	if (vbatt_over == true) {
		bat_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	} else if (tbatt_status == BATTERY_STATUS__REMOVED) {
		bat_health = POWER_SUPPLY_HEALTH_DEAD;
	} else if (tbatt_status == BATTERY_STATUS__HIGH_TEMP) {
		bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (tbatt_status == BATTERY_STATUS__LOW_TEMP) {
		bat_health = POWER_SUPPLY_HEALTH_COLD;
	} else {
		bat_health = POWER_SUPPLY_HEALTH_GOOD;
	}
	return bat_health;
}

static bool oplus_chg_soc_reduce_slow_when_1(struct oplus_chg_chip *chip)
{
	static int reduce_count = 0;
	int reduce_count_limit = 0;

	if (chip->batt_exist == false) {
		return false;
	}
	if (chip->charger_exist) {
		reduce_count_limit = 12;
	} else {
		reduce_count_limit = 4;
	}
	if (chip->batt_volt_min < chip->vbatt_soc_1) {
		reduce_count++;
	} else {
		reduce_count = 0;
	}
	charger_xlog_printk(CHG_LOG_CRTI,
			"batt_vol:%d, batt_volt_min:%d, reduce_count:%d\n",
			chip->batt_volt, chip->batt_volt_min, reduce_count);
	if (reduce_count > reduce_count_limit) {
		reduce_count = reduce_count_limit + 1;
		return true;
	} else {
		return false;
	}
}

static bool oplus_chg_soc_reduce_to_shutdown(struct oplus_chg_chip *chip)
{
	static int reduce_count = 0;
	static int last_chg_exit = 0;
	int reduce_count_limit = 7;

	if (chip->batt_exist == false) {
		return false;
	}

	if (chip->charger_exist != last_chg_exit)
		reduce_count = 0;

	if (chip->charger_exist) {
		last_chg_exit = 1;
		if (chip->batt_volt_min < chip->soc_to_0_withchg) {
			reduce_count++;
		} else {
			reduce_count = 0;
		}
	} else {
		last_chg_exit = 0;
		if (chip->batt_volt_min < chip->soc_to_0_withoutchg) {
			reduce_count++;
		} else {
			reduce_count = 0;
		}
	}

	charger_xlog_printk(CHG_LOG_CRTI,
			"batt_vol:%d, batt_volt_min:%d, reduce_count:%d, last_chg_exit:%d\n",
			chip->batt_volt, chip->batt_volt_min, reduce_count, last_chg_exit);
	if (reduce_count > reduce_count_limit) {
		reduce_count = reduce_count_limit + 1;
		return true;
	} else {
		return false;
	}
}

#define SOC_SYNC_UP_RATE_10S			2
#define SOC_SYNC_UP_RATE_60S			12
#define SOC_SYNC_DOWN_RATE_300S			60
#define SOC_SYNC_DOWN_RATE_150S			30
#define SOC_SYNC_DOWN_RATE_90S			18
#define SOC_SYNC_DOWN_RATE_60S			12
#define SOC_SYNC_DOWN_RATE_45S			9
#define SOC_SYNC_DOWN_RATE_40S			8
#define SOC_SYNC_DOWN_RATE_30S			6
#define SOC_SYNC_DOWN_RATE_15S			3
#define TEN_MINUTES				600
#define CHARGING_STATUS  1
#define DISCHARGING_STATUS  0

static void oplus_chg_smooth_to_soc(struct oplus_chg_chip *chip)
{
	static int time = 0;
	static int capacity = -1;
	static int smooth_diff =-1;
	static int soc_pre = -1;
	static int status = DISCHARGING_STATUS;
	pr_err("[oplus_chg_smooth_to_soc] enter the func");
	if(chip->charger_exist && chip->batt_exist
			&& (CHARGING_STATUS_FAIL != chip->charging_state)
			&& chip->mmi_chg && (chip->stop_chg == 1 || chip->charger_type == 5) && status == DISCHARGING_STATUS){
		status = CHARGING_STATUS;
		time = 0;
		capacity = chip->batt_rm;
		soc_pre = chip->soc;
		smooth_diff = capacity/(2 * chip->soc - chip->smooth_soc);
	} else if(!(chip->charger_exist && chip->batt_exist
			&& chip->mmi_chg && (chip->batt_full || CHARGING_STATUS_FAIL != chip->charging_state)
			&& (chip->stop_chg == 1 || chip->charger_type == 5)) && status == CHARGING_STATUS){
		status = DISCHARGING_STATUS;
		time = 0;
		capacity = -1;
		soc_pre = -1;
		smooth_diff = -1;
	}

	if(status == DISCHARGING_STATUS){
		if(chip->soc >= 96 && chip->soc <= 100 && chip->ui_soc == 100){
				chip->smooth_soc = 100;
			} else{
				if(chip->smooth_soc > chip->soc){
					if(capacity == -1){
						time = 0;
						capacity = chip->batt_rm;
						soc_pre = chip->soc;
						smooth_diff = capacity/chip->smooth_soc;
					}
					if((capacity - chip->batt_rm) >= smooth_diff || ((chip->smooth_soc - chip->soc) > chip->smooth_to_soc_gap)){
							chip->smooth_soc--;
							capacity = chip->batt_rm;
							smooth_diff = capacity/chip->smooth_soc;
					}
					if(chip->soc == 0 ){
						time++;
						if(time >= SOC_SYNC_DOWN_RATE_15S){
							chip->smooth_soc--;
							time = 0;
						}
					}
					pr_err("[oplus_chg_smooth_to_soc] smooth_soc[%d],capacity[%d],smooth_diff[%d]",chip->smooth_soc,capacity,smooth_diff);
				} else {
					chip->smooth_soc =chip->soc;
				}
			}
	} else{
		if(chip->smooth_soc > chip->soc){
			if(soc_pre < chip->soc &&(chip->batt_rm - capacity) >= smooth_diff && chip->smooth_soc < 100 ){
					chip->smooth_soc++;
					capacity = chip->batt_rm;
					soc_pre = chip->soc;
					smooth_diff = capacity/(2 * chip->soc - chip->smooth_soc);
			}
			if(chip->soc < soc_pre){
				chip->smooth_soc--;
				soc_pre = chip->soc;
			}
			pr_err("[oplus_chg_smooth_to_soc]charging smooth_soc[%d],capacity[%d],smooth_diff[%d],soc_pre[%d]",chip->smooth_soc,capacity,smooth_diff,soc_pre);
		} else {
			chip->smooth_soc = chip->soc;
		}
	}
}
static void oplus_chg_update_ui_soc(struct oplus_chg_chip *chip)
{
	static int soc_down_count = 0;
	static int soc_up_count = 0;
	static int ui_soc_pre = 50;
	static int cnt = 0;
	int soc_down_limit = 0;
	int soc_up_limit = 0;
	unsigned long sleep_tm = 0;
	unsigned long soc_reduce_margin = 0;
	bool vbatt_too_low = false;
	vbatt_lowerthan_3300mv = false;

	if (chip->ui_soc == 100) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_300S;
	} else if (chip->ui_soc >= 95) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_150S;
	} else if (chip->ui_soc >= 60) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_60S;
	} else if (chip->charger_exist && chip->ui_soc == 1) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_90S;
	} else {
		soc_down_limit = SOC_SYNC_DOWN_RATE_40S;
	}
	if (chip->batt_exist &&
			(chip->batt_volt_min < chip->vbatt_power_off)
			&& (chip->batt_volt_min > 2500)) {
		soc_down_limit = SOC_SYNC_DOWN_RATE_15S;
		vbatt_too_low = true;
		vbatt_lowerthan_3300mv = true;
		charger_xlog_printk(CHG_LOG_CRTI,
			"batt_volt:%d, batt_volt_min:%d, vbatt_too_low:%d\n",
			chip->batt_volt, chip->batt_volt_min, vbatt_too_low);
	}
	if (chip->batt_full) {
		soc_up_limit = SOC_SYNC_UP_RATE_10S;
	} else {
		soc_up_limit = SOC_SYNC_UP_RATE_10S;
	}
//for dx1 bringup
#if 1//ndef CONFIG_OPLUS_CHARGER_MTK
#ifndef WPC_NEW_INTERFACE
	if ((chip->charger_exist || oplus_wpc_get_wireless_charge_start()) &&
			chip->batt_exist && chip->batt_full && chip->mmi_chg &&
			(chip->stop_chg == 1 || chip->charger_type == 5 ||
			chip->charger_type == POWER_SUPPLY_TYPE_WIRELESS)) {
#else
	if ((chip->charger_exist || oplus_wpc_get_status()) &&
			chip->batt_exist && chip->batt_full && chip->mmi_chg &&
			(chip->stop_chg == 1 || chip->charger_type == 5||
			chip->charger_type == POWER_SUPPLY_TYPE_WIRELESS)) {
#endif
#else
	if (chip->charger_exist && chip->batt_exist && chip->batt_full && chip->mmi_chg &&
			(chip->stop_chg == 1 || chip->charger_type == 5 ||
			chip->charger_type == POWER_SUPPLY_TYPE_WIRELESS)) {
#endif
		chip->sleep_tm_sec = 0;
		if (oplus_short_c_batt_is_prohibit_chg(chip)) {
			chip->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else if ((chip->hmac) &&((chip->tbatt_status == BATTERY_STATUS__NORMAL)
				|| (chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP)
				|| (chip->tbatt_status == BATTERY_STATUS__COOL_TEMP)
				|| (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP))) {
			soc_down_count = 0;
			soc_up_count++;
			if (soc_up_count >= soc_up_limit) {
				soc_up_count = 0;
				chip->ui_soc++;
			}
			if (chip->ui_soc >= 100) {
				chip->ui_soc = 100;
				chip->prop_status = POWER_SUPPLY_STATUS_FULL;
			} else {
				chip->prop_status = POWER_SUPPLY_STATUS_CHARGING;
			}
		} else {
			chip->prop_status = POWER_SUPPLY_STATUS_FULL;
		}
		if (chip->ui_soc != ui_soc_pre) {
			chg_debug("full [soc ui_soc smooth_soc up_limit] = [%d %d %d %d]\n",
				chip->soc, chip->ui_soc, chip->smooth_soc, soc_up_limit);

		}
//for dx1 bringup
#if 1 //ndef CONFIG_OPLUS_CHARGER_MTK
#ifndef WPC_NEW_INTERFACE
	} else if ((chip->charger_exist || oplus_wpc_get_wireless_charge_start()) &&
			chip->batt_exist && (CHARGING_STATUS_FAIL != chip->charging_state) &&
			chip->mmi_chg && (chip->stop_chg == 1 || chip->charger_type == 5 ||
			chip->charger_type == POWER_SUPPLY_TYPE_WIRELESS)) {
#else
	} else if ((chip->charger_exist || oplus_wpc_get_status()) && chip->batt_exist &&
			(CHARGING_STATUS_FAIL != chip->charging_state) && chip->mmi_chg &&
			(chip->stop_chg == 1 || chip->charger_type == 5 ||
			chip->charger_type == POWER_SUPPLY_TYPE_WIRELESS)) {
#endif
#else
	} else if (chip->charger_exist && chip->batt_exist &&
			(CHARGING_STATUS_FAIL != chip->charging_state) && chip->mmi_chg &&
			(chip->stop_chg == 1 || chip->charger_type == 5 ||
			chip->charger_type == POWER_SUPPLY_TYPE_WIRELESS)) {
#endif
		chip->sleep_tm_sec = 0;
		chip->prop_status = POWER_SUPPLY_STATUS_CHARGING;
		if (chip->smooth_soc == chip->ui_soc) {
			soc_down_count = 0;
			soc_up_count = 0;
		} else if (chip->smooth_soc > chip->ui_soc) {
			soc_down_count = 0;
			soc_up_count++;
			if (soc_up_count >= soc_up_limit) {
				soc_up_count = 0;
				chip->ui_soc++;
			}
		} else if (chip->smooth_soc < chip->ui_soc) {
			soc_up_count = 0;
			soc_down_count++;
			if (soc_down_count >= soc_down_limit) {
				soc_down_count = 0;
				if(oplus_chg_show_vooc_logo_ornot() == false
					|| (chip->vooc_show_ui_soc_decimal == false || !chip->decimal_control)) {
					chip->ui_soc--;
				}
			}
		}
		if (chip->ui_soc != ui_soc_pre) {
			chg_debug("full [soc ui_soc smooth_soc down_limit up_limit] = [%d %d %d %d %d]\n", chip->soc, chip->ui_soc, chip->smooth_soc, soc_down_limit, soc_up_limit);

		}
		charger_xlog_printk(CHG_LOG_CRTI, "ui_soc:%d,waiting_for_ffc:%d,fastchg_to_ffc:%d,fastchg_start:%d,chg_type=0x%x\n",
		chip->ui_soc, chip->waiting_for_ffc == false, chip->fastchg_to_ffc == false,
		oplus_vooc_get_fastchg_started(), oplus_vooc_get_fast_chg_type());
		if (chip->ui_soc == 100
			&& chip->fastchg_to_ffc == false
			&& !oplus_chg_is_wls_ffc()
			&& (oplus_vooc_get_fastchg_started() == false
			|| oplus_vooc_get_fast_chg_type() == CHARGER_SUBTYPE_FASTCHG_VOOC)) {
			if (++cnt >= 12) {
				chip->batt_full = true;

				chip->in_rechging = false;
				chip->limits.vfloat_over_counts = 0;
				oplus_chg_check_rechg_status(chip);
			}
		} else {
			cnt = 0;
		}
	} else {
		cnt = 0;
		chip->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		soc_up_count = 0;
		if (chip->smooth_soc <= chip->ui_soc || vbatt_too_low) {
			if (soc_down_count > soc_down_limit) {
				soc_down_count = soc_down_limit + 1;
			} else {
				soc_down_count++;
			}
			sleep_tm = chip->sleep_tm_sec;
			if (chip->sleep_tm_sec > 0) {
				soc_reduce_margin = chip->sleep_tm_sec / TEN_MINUTES;
				if (soc_reduce_margin == 0) {
					if ((chip->ui_soc - chip->smooth_soc) > 2) {
						chip->ui_soc--;
						soc_down_count = 0;
						chip->sleep_tm_sec = 0;
					}
				} else if (soc_reduce_margin < (chip->ui_soc - chip->smooth_soc)) {
					chip->ui_soc -= soc_reduce_margin;
					soc_down_count = 0;
					chip->sleep_tm_sec = 0;
				} else if (soc_reduce_margin >= (chip->ui_soc - chip->smooth_soc)) {
					chip->ui_soc = chip->smooth_soc;
					soc_down_count = 0;
					chip->sleep_tm_sec = 0;
				}
			}
			if (soc_down_count >= soc_down_limit && (chip->smooth_soc < chip->ui_soc || vbatt_too_low)) {
				chip->sleep_tm_sec = 0;
				soc_down_count = 0;
				chip->ui_soc--;
			}
		}
	}
	if (chip->ui_soc < 2) {
		cnt = 0;
		if (oplus_chg_soc_reduce_slow_when_1(chip) == true || oplus_chg_soc_reduce_to_shutdown(chip) == true) {
			chip->ui_soc = 0;
		} else {
			chip->ui_soc = 1;
		}
	}
	if (chip->ui_soc != ui_soc_pre) {
		ui_soc_pre = chip->ui_soc;
		chip->chg_ops->set_rtc_soc(chip->ui_soc);
		if (chip->chg_ops->get_rtc_soc() != chip->ui_soc) {
			/*charger_xlog_printk(CHG_LOG_CRTI, "set soc fail:[%d, %d], try again...\n", chip->ui_soc, chip->chg_ops->get_rtc_soc());*/
			chip->chg_ops->set_rtc_soc(chip->ui_soc);
		}
	}
	if(chip->decimal_control) {
		soc_down_count = 0;
		soc_up_count = 0;
	}
}

static void fg_update(struct oplus_chg_chip *chip)
{
	static int ui_soc_pre_fg = 50;
	static struct power_supply *bms_psy = NULL;
	if (!bms_psy) {
		bms_psy = power_supply_get_by_name("bms");
		charger_xlog_printk(CHG_LOG_CRTI, "bms_psy null\n");
	}
	if (bms_psy) {
		if (chip->ui_soc != ui_soc_pre_fg) {
			power_supply_changed(bms_psy);
			charger_xlog_printk(CHG_LOG_CRTI,
				"ui_soc:%d, soc:%d, ui_soc_pre:%d \n",
				chip->ui_soc, chip->soc, ui_soc_pre_fg);
		}
		if (chip->ui_soc != ui_soc_pre_fg) {
			ui_soc_pre_fg = chip->ui_soc;
		}
	}
}

#define BATTERY_UPDATE_THD 4
static void battery_update(struct oplus_chg_chip *chip)
{
	static int pre_ui_soc = -1;
	static int batt_update_count = 0;
	static int pre_charger_exist = 0;

	oplus_chg_update_ui_soc(chip);
	if (chip->fg_bcl_poll) {
		fg_update(chip);
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
	batt_update_count++;
	if (chip->charger_exist || batt_update_count == BATTERY_UPDATE_THD
		|| pre_charger_exist != chip->charger_exist
		|| pre_ui_soc != chip->ui_soc) {
		pre_ui_soc = chip->ui_soc;
		pre_charger_exist = chip->charger_exist;
		batt_update_count = 0;
		power_supply_changed(chip->batt_psy);
		charger_xlog_printk(CHG_LOG_CRTI, "battery_update\n");
	}
#else
	power_supply_changed(&chip->batt_psy);
#endif
}

static void oplus_chg_battery_update_status(struct oplus_chg_chip *chip)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	usb_update(chip);
#endif
	battery_update(chip);
}

#define RESET_MCU_DELAY_30S		6

static void oplus_chg_get_chargerid_voltage(struct oplus_chg_chip *chip)
{
	if (chip->chg_ops->set_chargerid_switch_val == NULL
			|| chip->chg_ops->get_chargerid_switch_val == NULL
			|| chip->chg_ops->get_chargerid_volt == NULL) {
		return;
	} else if (chip->charger_type != POWER_SUPPLY_TYPE_USB_DCP) {
		return;
	}
	if(chip->chg_ops->check_pdphy_ready && chip->chg_ops->check_pdphy_ready() == false){
		chg_err("OPLUS CHG PD_PHY NOT READY");
		return;
	}
	if (reset_mcu_delay > RESET_MCU_DELAY_30S){
		return;
	}
	if (oplus_vooc_get_vooc_switch_val() == 1) {
		if (chip->chargerid_volt_got == false) {
			chip->chg_ops->set_chargerid_switch_val(1);
#ifdef CONFIG_OPLUS_CHARGER_MTK
			if (oplus_vooc_get_fastchg_started() == false){
				oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
			}
			usleep_range(100000, 110000);
#else
			usleep_range(20000, 22000);
#endif /* CONFIG_OPLUS_CHARGER_MTK */
			chip->chargerid_volt = chip->chg_ops->get_chargerid_volt();
			chip->chargerid_volt_got = true;
		} else {
			if (chip->chg_ops->get_chargerid_switch_val() == 0) {
				chip->chg_ops->set_chargerid_switch_val(1);
			} else {
				/* do nothing*/
			}
		}
	} else if (oplus_vooc_get_vooc_switch_val() == 0) {
		if (chip->chargerid_volt_got == false) {
			chip->chg_ops->set_chargerid_switch_val(1);
			oplus_vooc_set_vooc_chargerid_switch_val(1);
#ifdef CONFIG_OPLUS_CHARGER_MTK
			usleep_range(100000, 110000);
#else
			usleep_range(20000, 22000);
#endif /* CONFIG_OPLUS_CHARGER_MTK */
			chip->chargerid_volt = chip->chg_ops->get_chargerid_volt();
			chip->chargerid_volt_got = true;
			oplus_vooc_set_vooc_chargerid_switch_val(0);
			if (chip->vooc_project == false) {
				chip->chg_ops->set_chargerid_switch_val(0);
			}
		} else {
			if (chip->chg_ops->get_chargerid_switch_val() == 1) {
				chip->chg_ops->set_chargerid_switch_val(0);
			} else {
				/* do nothing*/
			}
		}
	} else {
		charger_xlog_printk(CHG_LOG_CRTI, "do nothing\n");
	}
}

static void oplus_chg_chargerid_switch_check(struct oplus_chg_chip *chip)
{
	return oplus_chg_get_chargerid_voltage(chip);
}

#define RESET_MCU_DELAY_15S		3

static void oplus_chg_qc_config(struct oplus_chg_chip *chip);
static void oplus_chg_fast_switch_check(struct oplus_chg_chip *chip)
{
	if (oplus_short_c_batt_is_prohibit_chg(chip)) {
		charger_xlog_printk(CHG_LOG_CRTI, " short_c_battery, return\n");
		return;
	}
	if (chip->mmi_chg == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, " mmi_chg,return\n");
		return;
	}
	if (chip->allow_swtich_to_fastchg == false) {
		charger_xlog_printk(CHG_LOG_CRTI, " allow_swtich_to_fastchg == 0,return\n");
		return;
	}
	if ((!chip->authenticate) ||(!chip->hmac)) {
		charger_xlog_printk(CHG_LOG_CRTI, "non authenticate or hmac,switch return\n");
		return;
	}
	if (chip->notify_flag == NOTIFY_BAT_OVER_VOL) {
		charger_xlog_printk(CHG_LOG_CRTI, " battery over voltage,return\n");
		return;
	}
//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
#ifdef SUPPORT_WPC
	if (chip->wireless_support && (oplus_wpc_get_wireless_charge_start() == true
			|| oplus_chg_is_wls_present())) {
		charger_xlog_printk(CHG_LOG_CRTI, "is in WPC, switch return\n");
		return;
	}
#endif
//#endif
	if(chip->chg_ops->check_pdphy_ready && chip->chg_ops->check_pdphy_ready() == false){
		chg_err("OPLUS CHG PD_PHY NOT READY");
#ifdef CONFIG_OPLUS_CHARGER_MTK
/*10 seconds after typec init, PD starts to communicate, so set true force reset adapter.only for MTK*/
		if (oplus_is_power_off_charging(NULL)) {
			mcu_update = true;
		}
#endif
		return;
	}

	if (oplus_pps_get_chg_status() != PPS_NOT_SUPPORT
			&& chip->pd_svooc == false
			&& chip->chg_ops->oplus_chg_get_pd_type() == PD_PPS_ACTIVE) {
		return;
	}

	if (chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
		if (true == opchg_get_mcu_update_state()) {
			reset_mcu_delay = 0;
			mcu_update = true;
			return;
		}
		if (oplus_vooc_get_fastchg_started() == false
				&& reset_mcu_delay < RESET_MCU_DELAY_30S) {
			if (mcu_update == true) {
				oplus_chg_suspend_charger();
				msleep(2000);
				oplus_chg_unsuspend_charger();
				mcu_update = false;
				chg_debug("mcu_update, reset\n");
				msleep(2000);
			}
			oplus_vooc_switch_fast_chg();
		}
		if (!oplus_vooc_get_fastchg_started()
				&& !oplus_vooc_get_fastchg_dummy_started()
				&& !oplus_vooc_get_fastchg_to_normal()
				&& !oplus_vooc_get_fastchg_to_warm()) {
			if (suspend_charger) {
				reset_mcu_delay = RESET_MCU_DELAY_15S;
				suspend_charger = false;
				oplus_vooc_set_ap_clk_high();
				oplus_vooc_reset_mcu();
			}
			reset_mcu_delay++;
			if (reset_mcu_delay == RESET_MCU_DELAY_15S) {
				charger_xlog_printk(CHG_LOG_CRTI, "  reset mcu again,suspend here\n");
				if (suspend_charger == false) {
					oplus_chg_reset_adapter();
					suspend_charger = true;
					chg_debug(" fastchg start failed, reset adapter\n");
				} else {
					oplus_vooc_set_ap_clk_high();
					oplus_vooc_reset_mcu();
				}
			} else if (reset_mcu_delay == RESET_MCU_DELAY_30S
						&& chip->vbatt_num == 2) {
				suspend_charger = false;
				reset_mcu_delay = RESET_MCU_DELAY_30S + 1;
				charger_xlog_printk(CHG_LOG_CRTI, "  RESET_MCU_DELAY_30S\n");
				if (chip->charger_volt <= 7500
						//for dx1 bringup
						&& chip->chg_ops->oplus_chg_get_pd_type() != PD_PPS_ACTIVE) {
					oplus_vooc_reset_fastchg_after_usbout();
					chip->chg_ops->set_chargerid_switch_val(0);
					if (chip->chg_ops->enable_qc_detect){
						chip->chg_ops->enable_qc_detect();
					}
				}
			}
		} else {
			suspend_charger = false;
			mcu_update = false;
		}
		if(reset_mcu_delay > RESET_MCU_DELAY_30S){
			chip->pd_svooc = false;
		}
	} else {
		mcu_update = false;
	}
}

#define FULL_COUNTS_SW		5
#define FULL_COUNTS_HW		4

static int oplus_chg_check_sw_full(struct oplus_chg_chip *chip)
{
	int vbatt_full_vol_sw = 0;

	if (!chip->charger_exist) {
		chip->sw_full_count = 0;
		chip->sw_full = false;
		return false;
	}

	if (chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {
		vbatt_full_vol_sw = chip->limits.cold_vfloat_sw_limit;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {
		vbatt_full_vol_sw = chip->limits.little_cold_vfloat_sw_limit;
	} else if (chip->tbatt_status == BATTERY_STATUS__COOL_TEMP) {
		vbatt_full_vol_sw = chip->limits.cool_vfloat_sw_limit;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP) {
		vbatt_full_vol_sw = chip->limits.little_cool_vfloat_sw_limit;
	} else if (chip->tbatt_status == BATTERY_STATUS__NORMAL) {
		vbatt_full_vol_sw = chip->limits.normal_vfloat_sw_limit;
	} else if (chip->tbatt_status == BATTERY_STATUS__WARM_TEMP) {
		vbatt_full_vol_sw = chip->limits.warm_vfloat_sw_limit;
	} else {
		chip->sw_full_count = 0;
		chip->sw_full = 0;
		return false;
	}
	if ((!chip->authenticate) || (!chip->hmac)) {
		vbatt_full_vol_sw = chip->limits.non_standard_vfloat_sw_limit;
	}
	if (oplus_short_c_batt_is_prohibit_chg(chip)) {
		vbatt_full_vol_sw = chip->limits.short_c_bat_vfloat_sw_limit;
	}
	/* use SW Vfloat to check */
	if (chip->batt_volt > vbatt_full_vol_sw) {
		if (chip->icharging < 0 && (chip->icharging * -1) <= chip->limits.iterm_ma) {
			chip->sw_full_count++;
			if (chip->sw_full_count > FULL_COUNTS_SW) {
				chip->sw_full_count = 0;
				chip->sw_full = true;
			}
		} else if (chip->icharging >= 0) {
			chip->sw_full_count++;
			if (chip->sw_full_count > FULL_COUNTS_SW * 2) {
				chip->sw_full_count = 0;
				chip->sw_full = true;
				charger_xlog_printk(CHG_LOG_CRTI,
					"[BATTERY] Battery full by sw when icharging>=0!!\n");
			}
		} else {
			chip->sw_full_count = 0;
			chip->sw_full = false;
		}
	} else {
		chip->sw_full_count = 0;
		chip->sw_full = false;
	}
	return chip->sw_full;
}

static int oplus_chg_check_hw_full(struct oplus_chg_chip *chip)
{
	int vbatt_full_vol_hw = 0;
	static int vbat_counts_hw = 0;
	static bool ret_hw = false;

	if (!chip->charger_exist) {
		vbat_counts_hw = 0;
		ret_hw = false;
		chip->hw_full_by_sw = false;
		return false;
	}
	vbatt_full_vol_hw = oplus_chg_get_float_voltage(chip);
	if (chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {
		vbatt_full_vol_hw = chip->limits.temp_cold_vfloat_mv
			+ chip->limits.non_normal_vterm_hw_inc;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {
		vbatt_full_vol_hw = chip->limits.temp_little_cold_vfloat_mv
			+ chip->limits.non_normal_vterm_hw_inc;
	} else if (chip->tbatt_status == BATTERY_STATUS__COOL_TEMP) {
		vbatt_full_vol_hw = chip->limits.temp_cool_vfloat_mv
			+ chip->limits.non_normal_vterm_hw_inc;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP) {
		vbatt_full_vol_hw = chip->limits.temp_little_cool_vfloat_mv
			+ chip->limits.non_normal_vterm_hw_inc;
	} else if (chip->tbatt_status == BATTERY_STATUS__NORMAL) {
		vbatt_full_vol_hw = chip->limits.temp_normal_vfloat_mv
			+ chip->limits.normal_vterm_hw_inc;
	} else if (chip->tbatt_status == BATTERY_STATUS__WARM_TEMP) {
		vbatt_full_vol_hw = chip->limits.temp_warm_vfloat_mv
			+ chip->limits.non_normal_vterm_hw_inc;
	} else {
		vbat_counts_hw = 0;
		ret_hw = 0;
		chip->hw_full_by_sw = false;
		return false;
	}
	if ((!chip->authenticate) || (!chip->hmac)) {
		vbatt_full_vol_hw = chip->limits.non_standard_vfloat_mv
			+ chip->limits.non_normal_vterm_hw_inc;
	}
	if (oplus_short_c_batt_is_prohibit_chg(chip)) {
		vbatt_full_vol_hw = chip->limits.short_c_bat_vfloat_mv
			+ chip->limits.non_normal_vterm_hw_inc;
	}
	/* use HW Vfloat to check */
	if (chip->batt_volt >= vbatt_full_vol_hw) {
		vbat_counts_hw++;
		if (vbat_counts_hw >= FULL_COUNTS_HW) {
			vbat_counts_hw = 0;
			ret_hw = true;
		}
	} else {
		vbat_counts_hw = 0;
		ret_hw = false;
	}

	chip->hw_full_by_sw = ret_hw;
	return ret_hw;
}


#define FFC_VOLT_COUNTS		4
#define FFC_CURRENT_COUNTS	2

static void oplus_chg_ffc_variable_reset(struct oplus_chg_chip *chip)
{
	chip->fastchg_to_ffc = false;
	chip->fastchg_ffc_status = 0;
	chip->chg_ctrl_by_lcd = chip->chg_ctrl_by_lcd_default;
	chip->chg_ctrl_by_vooc = chip->chg_ctrl_by_vooc_default;
	chip->limits.iterm_ma = chip->limits.default_iterm_ma;
	chip->limits.normal_vfloat_sw_limit = chip->limits.default_normal_vfloat_sw_limit;
	chip->limits.temp_normal_vfloat_mv = chip->limits.default_temp_normal_vfloat_mv;
	chip->limits.normal_vfloat_over_sw_limit = chip->limits.default_normal_vfloat_over_sw_limit;
	chip->limits.temp_normal_fastchg_current_ma = chip->limits.default_temp_normal_fastchg_current_ma;
	chip->limits.temp_little_cool_fastchg_current_ma = chip->limits.default_temp_little_cool_fastchg_current_ma;
	chip->limits.little_cool_vfloat_sw_limit = chip->limits.default_little_cool_vfloat_sw_limit;
	chip->limits.temp_little_cool_vfloat_mv = chip->limits.default_temp_little_cool_vfloat_mv;
	chip->limits.little_cool_vfloat_over_sw_limit = chip->limits.default_little_cool_vfloat_over_sw_limit;
}


static bool oplus_chg_check_ffc_status(struct oplus_chg_chip *chip)
{
	static int vffc1_counts = 0;
	static int vffc2_counts = 0;
	static int warm_counts = 0;
	static int normal_counts = 0;
	static int ffc_vfloat_sw_limit = 4450;

	if (chip->fastchg_to_ffc == true) {
		if (chip->fastchg_ffc_status == 1) {
			if (chip->batt_volt >= chip->limits.ffc1_normal_vfloat_sw_limit) {
				vffc1_counts ++;
				if (vffc1_counts >= FFC_VOLT_COUNTS) {
					oplus_chg_turn_on_ffc2(chip);
					return false;
				}
			}
			if (chip->ffc_temp_status == FFC_TEMP_STATUS__NORMAL) {
				if ((chip->icharging * -1) < (chip->limits.ff1_normal_fastchg_ma
						- chip->limits.ff1_exit_step_ma)) {
					normal_counts ++;
					if (normal_counts >= FFC_CURRENT_COUNTS) {
						oplus_chg_ffc_variable_reset(chip);
						oplus_chg_turn_on_charging(chip);
						return true;
					}
				} else {
					normal_counts = 0;
				}
			} else if (chip->ffc_temp_status == FFC_TEMP_STATUS__WARM) {
				if ((chip->icharging * -1) < (chip->limits.ff1_warm_fastchg_ma
						- chip->limits.ff1_warm_exit_step_ma)) {
					warm_counts ++;
					if (warm_counts >= FFC_CURRENT_COUNTS) {
						oplus_chg_ffc_variable_reset(chip);
						oplus_chg_turn_on_charging(chip);
						return true;
					}
				} else {
					warm_counts = 0;
				}
			} else {
				warm_counts = normal_counts = 0;
				oplus_chg_ffc_variable_reset(chip);
				oplus_chg_turn_on_charging(chip);
				return true;
			}
			return false;
		}
		if (chip->fastchg_ffc_status == 2) {
			if (chip->ffc_temp_status == FFC_TEMP_STATUS__WARM)
				ffc_vfloat_sw_limit = chip->limits.ffc2_warm_vfloat_sw_limit;
			else
				ffc_vfloat_sw_limit = chip->limits.ffc2_normal_vfloat_sw_limit;
			if (chip->batt_volt >= ffc_vfloat_sw_limit) {
				vffc2_counts ++;
				if (vffc2_counts >= FFC_VOLT_COUNTS) {
					oplus_chg_ffc_variable_reset(chip);
					oplus_chg_turn_on_charging(chip);
					return true;
				}
			}
			if (chip->ffc_temp_status == FFC_TEMP_STATUS__NORMAL) {
				if ((chip->icharging * -1) < (chip->limits.ffc2_normal_fastchg_ma
						- chip->limits.ffc2_exit_step_ma)) {
					normal_counts ++;
					if (normal_counts >= FFC_CURRENT_COUNTS) {
						oplus_chg_ffc_variable_reset(chip);
						oplus_chg_turn_on_charging(chip);
						return true;
					}
				} else {
					normal_counts = 0;
				}
			} else if (chip->ffc_temp_status == FFC_TEMP_STATUS__WARM) {
				if ((chip->icharging * -1) < (chip->limits.ffc2_warm_fastchg_ma
						- chip->limits.ffc2_warm_exit_step_ma)) {
					warm_counts ++;
					if (warm_counts >= FFC_CURRENT_COUNTS) {
						oplus_chg_ffc_variable_reset(chip);
						oplus_chg_turn_on_charging(chip);
						return true;
					}
				} else {
					warm_counts = 0;
				}
			} else {
				warm_counts = normal_counts = 0;
				oplus_chg_ffc_variable_reset(chip);
				oplus_chg_turn_on_charging(chip);
				return true;
			}
		}
		return false;
	}
	vffc1_counts = 0;
	vffc2_counts = 0;
	warm_counts = 0;
	normal_counts = 0;
	return true;
}

static bool oplus_chg_check_vbatt_is_full_by_sw(struct oplus_chg_chip *chip)
{
	bool ret_sw = false;
	bool ret_hw = false;

	if (!chip->check_batt_full_by_sw) {
		return false;
	}

	ret_sw = oplus_chg_check_sw_full(chip);
	ret_hw = oplus_chg_check_hw_full(chip);
	if (ret_sw == true || ret_hw == true) {
		charger_xlog_printk(CHG_LOG_CRTI,
			"[BATTERY] Battery full by sw[%s] !!\n",
			(ret_sw == true) ? "S" : "H");
		return true;
	} else {
		return false;
	}
}

#define FULL_DELAY_COUNTS		4
#define DOD0_COUNTS		(8 * 60 / 5)

static void oplus_chg_check_status_full(struct oplus_chg_chip *chip)
{
	int is_batt_full = 0;
	static int fastchg_present_wait_count = 0;

	if (chip->chg_ctrl_by_vooc) {
		if (oplus_vooc_get_fastchg_ing() == true
				&& oplus_vooc_get_fast_chg_type() != CHARGER_SUBTYPE_FASTCHG_VOOC)
			return;
	} else {
		if (oplus_vooc_get_fastchg_ing() == true)
			return;
	}

	if (chip->wireless_support
			&& (oplus_wpc_get_wireless_charge_start() == true
			|| oplus_chg_is_wls_present())) {
		if (oplus_chg_comm_get_batt_status(chip->comm_ocm) == POWER_SUPPLY_STATUS_FULL) {
			chip->batt_full = true;
		}
		charger_xlog_printk(CHG_LOG_CRTI, "in wireless charging\n");
		return;
	}

	if (oplus_pps_get_chg_status() == PPS_CHARGERING){
		return;
	}

	if (oplus_vooc_get_allow_reading() == false) {
		is_batt_full = 0;
		fastchg_present_wait_count = 0;
	} else {
		if (((oplus_vooc_get_fastchg_to_normal()== true)
				|| (oplus_vooc_get_fastchg_to_warm() == true))
				&& (fastchg_present_wait_count <= FULL_DELAY_COUNTS)) {
			is_batt_full = 0;
			fastchg_present_wait_count++;
			if (fastchg_present_wait_count == FULL_DELAY_COUNTS
					&& chip->chg_ops->get_charging_enable() == false
					&& chip->charging_state != CHARGING_STATUS_FULL
					&& chip->charging_state != CHARGING_STATUS_FAIL) {
				if (chip->ffc_support && chip->ffc_temp_status != FFC_TEMP_STATUS__HIGH
						&& chip->ffc_temp_status != FFC_TEMP_STATUS__LOW) {
					if (chip->vbatt_num == 2 && chip->dual_ffc == false) {
						oplus_chg_turn_on_ffc2(chip);
					} else {
						oplus_chg_turn_on_ffc1(chip);
					}
				} else {
					oplus_chg_turn_on_charging(chip);
				}
			}
		} else {
			is_batt_full = chip->chg_ops->read_full();
			chip->hw_full = is_batt_full;
			fastchg_present_wait_count = 0;
		}
	}
	if (oplus_chg_check_ffc_status(chip) == false) {
		return;
	}

	if ((is_batt_full == 1) || (chip->charging_state == CHARGING_STATUS_FULL)
			|| oplus_chg_check_vbatt_is_full_by_sw(chip)) {
		charger_xlog_printk(CHG_LOG_CRTI, "is_batt_full : %d,  chip->charging_state= %d\n", is_batt_full, chip->charging_state);
		oplus_chg_full_action(chip);
		if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP
				|| chip->tbatt_status == BATTERY_STATUS__COOL_TEMP
				|| chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP
				|| chip->tbatt_status == BATTERY_STATUS__NORMAL) {
			oplus_gauge_set_batt_full(true);
		}

		if ((chip->recharge_after_full == true || chip->vbatt_num == 2)
			&& (chip->tbatt_when_full <= 450 && chip->tbatt_when_full >= 0)
			&& (chip->temperature <= 350 && chip->temperature >= 0)) {
			if (chip->recharge_after_full) {
				chip->dod0_counts++;
			}
			if (chip->dod0_counts == DOD0_COUNTS) {
				if (chip->vbatt_num == 2){
					oplus_gauge_update_battery_dod0();
					//chip->in_rechging = true;
					//oplus_chg_voter_charging_start(chip, CHG_STOP_VOTER__FULL);/*now rechging!*/
					charger_xlog_printk(CHG_LOG_CRTI, "oplus_chg_check_status_full,dod0_counts = %d\n", chip->dod0_counts);
				}
				if (chip->recharge_after_full == true && chip->recharge_after_ffc == true) {
					chip->in_rechging = true;
					chip->sw_full_count = 0;
					chip->sw_full = false;
					chip->recharge_after_ffc = false;
					oplus_chg_voter_charging_start(chip, CHG_STOP_VOTER__FULL);
					charger_xlog_printk(CHG_LOG_CRTI, "recharge after full, dod0_counts = %d\n", chip->dod0_counts);
				}
				chip->dod0_counts = DOD0_COUNTS + 1;
			}
		}
	} else if (chip->charging_state == CHARGING_STATUS_FAIL) {
		oplus_chg_fail_action(chip);
	} else {
		chip->charging_state = CHARGING_STATUS_CCCV;
	}
}

static void oplus_chg_kpoc_power_off_check(struct oplus_chg_chip *chip)
{
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (chip->boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
			|| chip->boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {		/*vbus < 2.5V*/
		if ((chip->chg_ops->check_chrdet_status() == false)
				&& (chip->charger_volt < 2500)) {
			msleep(1000);
			charger_xlog_printk(CHG_LOG_CRTI,"pmic_thread_kthread]Unplug Charger/USB double check!\n");
			if ((chip->chg_ops->check_chrdet_status() == false)
					&& (chip->chg_ops->get_charger_volt() < 2500)) {
				if ((oplus_vooc_get_fastchg_to_normal() == false)
						&& (oplus_vooc_get_fastchg_to_warm() == false)
						&& (oplus_vooc_get_adapter_update_status() != ADAPTER_FW_NEED_UPDATE)
						&& (oplus_vooc_get_btb_temp_over() == false)) {
					charger_xlog_printk(CHG_LOG_CRTI,
						"[pmic_thread_kthread]Unplug Charger/USB \
						In Kernel Power Off Charging Mode Shutdown OS!\n");
					chip->chg_ops->set_power_off();
				}
			}
		}
	}
#endif
}

static void oplus_chg_print_log(struct oplus_chg_chip *chip)
{
	if(chip->vbatt_num == 1){
		charger_xlog_printk(CHG_LOG_CRTI,
			"CHGR[ %d / %d / %d / %d / %d ], "
			"BAT[ %d / %d / %d / %d / %d / %d ], "
			"GAUGE[ %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d / %d /], "
			"STATUS[ 0x%x / %d / %d / %d / %d / 0x%x ], OTHER[ %d / %d / %d / %d / %d/ %d / %d]\n",
			chip->charger_exist, chip->charger_type, chip->charger_volt,
			chip->prop_status, chip->boot_mode,
			chip->batt_exist, chip->batt_full, chip->chging_on, chip->in_rechging,
			chip->charging_state, chip->total_time,
			chip->temperature, chip->tbatt_temp, chip->batt_volt, chip->batt_volt_min, chip->icharging,
			chip->ibus, chip->soc, chip->ui_soc, chip->soc_load, chip->batt_rm,
			oplus_gauge_get_batt_fc(),oplus_gauge_get_batt_qm(),
			oplus_gauge_get_batt_pd(),oplus_gauge_get_batt_rcu(),
			oplus_gauge_get_batt_rcf(),oplus_gauge_get_batt_fcu(),
			oplus_gauge_get_batt_fcf(),oplus_gauge_get_batt_sou(),
			oplus_gauge_get_batt_do0(),oplus_gauge_get_batt_doe(),
			oplus_gauge_get_batt_trm(),oplus_gauge_get_batt_pc(),
			oplus_gauge_get_batt_qs(),
			chip->vbatt_over, chip->chging_over_time, chip->vchg_status,
			chip->tbatt_status, chip->stop_voter, chip->notify_code,
			chip->otg_switch, chip->mmi_chg, chip->boot_reason, chip->boot_mode,
			chip->chargerid_volt, chip->chargerid_volt_got, chip->shell_temp);
	}else{
		if (chip->prop_status != POWER_SUPPLY_STATUS_CHARGING) {
			oplus_gauge_dump_register();
		}
		charger_xlog_printk(CHG_LOG_CRTI,
			"CHGR[ %d / %d / %d / %d / %d ], "
			"BAT[ %d / %d / %d / %d / %d / %d ], "
			"GAUGE[ %d / %d / %d / %d / %d / %d / %d / %d / %d / %d ], "
			"STATUS[ 0x%x / %d / %d / %d / %d / 0x%x ], "
			"OTHER[ %d / %d / %d / %d / %d/ %d / %d]\n",
			chip->charger_exist, chip->charger_type, chip->charger_volt,
			chip->prop_status, chip->boot_mode,
			chip->batt_exist, chip->batt_full, chip->chging_on, chip->in_rechging,
			chip->charging_state, chip->total_time,
			chip->temperature, chip->tbatt_temp, chip->batt_volt, chip->batt_volt_min, chip->icharging,
			chip->ibus, chip->soc, chip->ui_soc, chip->soc_load, chip->batt_rm,
			chip->vbatt_over, chip->chging_over_time, chip->vchg_status,
			chip->tbatt_status, chip->stop_voter, chip->notify_code,
			chip->otg_switch, chip->mmi_chg, chip->boot_reason, chip->boot_mode,
			chip->chargerid_volt, chip->chargerid_volt_got, chip->shell_temp);
	}

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
		oplus_vooc_print_log();
	}
#endif
}

#define CHARGER_ABNORMAL_DETECT_TIME	24

static void oplus_chg_critical_log(struct oplus_chg_chip *chip)
{
	static int chg_abnormal_count = 0;

	if (chip->charger_exist) {
		if (chip->stop_voter == 0
				&& chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP
				&& chip->soc <= 75 && chip->icharging >= -20) {
			chg_abnormal_count++;
			if (chg_abnormal_count >= CHARGER_ABNORMAL_DETECT_TIME) {
				chg_abnormal_count = CHARGER_ABNORMAL_DETECT_TIME;
				charger_abnormal_log = CRITICAL_LOG_UNABLE_CHARGING;
			}
			charger_xlog_printk(CHG_LOG_CRTI, " unable charging, count=%d, charger_abnormal_log=%d\n", chg_abnormal_count, charger_abnormal_log);
		} else {
				chg_abnormal_count = 0;
		}
		if ((chip->stop_voter & CHG_STOP_VOTER__BATTTEMP_ABNORMAL)
				== CHG_STOP_VOTER__BATTTEMP_ABNORMAL) {
			charger_abnormal_log = CRITICAL_LOG_BATTTEMP_ABNORMAL;
		} else if ((chip->stop_voter & CHG_STOP_VOTER__VCHG_ABNORMAL)
				== CHG_STOP_VOTER__VCHG_ABNORMAL) {
			charger_abnormal_log = CRITICAL_LOG_VCHG_ABNORMAL;
		} else if ((chip->stop_voter & CHG_STOP_VOTER__VBAT_TOO_HIGH)
				== CHG_STOP_VOTER__VBAT_TOO_HIGH) {
			charger_abnormal_log = CRITICAL_LOG_VBAT_TOO_HIGH;
		} else if ((chip->stop_voter & CHG_STOP_VOTER__MAX_CHGING_TIME)
				== CHG_STOP_VOTER__MAX_CHGING_TIME) {
			charger_abnormal_log = CRITICAL_LOG_CHARGING_OVER_TIME;
		} else {
			/*do nothing*/
		}
	} else if (oplus_vooc_get_btb_temp_over() == true
			|| oplus_vooc_get_fastchg_to_normal() == true) {
		/*Do not clear 0x5d and 0x59*/
		charger_xlog_printk(CHG_LOG_CRTI, " btb_temp_over or fastchg_to_normal, charger_abnormal_log=%d\n", charger_abnormal_log);
	} else {
		charger_abnormal_log = 0;
	}
}

int oplus_chg_get_curr_time_ms(unsigned long *time_ms)
{
	u64 ts_nsec;

	ts_nsec = local_clock();
	*time_ms = ts_nsec / 1000000;

	return *time_ms;
}

static void oplus_chg_other_thing(struct oplus_chg_chip *chip)
{
	static unsigned long start_chg_time = 0;
	unsigned long cur_chg_time = 0;
	if (oplus_vooc_get_fastchg_started() == false) {
		chip->chg_ops->kick_wdt();
		chip->chg_ops->dump_registers();
	}
	if (chip->charger_exist) {
		if (chgr_dbg_total_time != 0) {
			chip->total_time = chgr_dbg_total_time;
		} else {
			if (!chip->total_time) {
				oplus_chg_get_curr_time_ms(&start_chg_time);
				start_chg_time = start_chg_time/1000;
				chip->total_time += OPLUS_CHG_UPDATE_INTERVAL_SEC;
			} else {
				oplus_chg_get_curr_time_ms(&cur_chg_time);
				cur_chg_time = cur_chg_time/1000;
				chip->total_time = OPLUS_CHG_UPDATE_INTERVAL_SEC + cur_chg_time - start_chg_time;
			}
		}
	}
	oplus_chg_debug_chg_monitor(chip);
	oplus_chg_print_log(chip);
	oplus_chg_critical_log(chip);
//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (chip->wireless_support) {
        oplus_wpc_print_log();
        // chargepump_print_log();
	}
//#endif
}

#define IBATT_COUNT	10

static void oplus_chg_ibatt_check_and_set(struct oplus_chg_chip *chip)
{
	static int average_current = 0;
	static int ibatt_count = 0;
	static int current_adapt = 0;
	static int pre_tbatt_status = BATTERY_STATUS__INVALID;
	static int fail_count = 0;
	bool set_current_flag = false;
	int recharge_volt = 0;
	int current_limit = 0;
	int current_init = 0;
	int threshold = 0;
	int current_step = 0;

	if ((chip->chg_ops->need_to_check_ibatt
			&& chip->chg_ops->need_to_check_ibatt() == false)
			|| !chip->chg_ops->need_to_check_ibatt) {
		return;
	}
	if (!chip->charger_exist || (oplus_vooc_get_fastchg_started() == true)) {
		current_adapt = 0;
		ibatt_count = 0;
		average_current = 0;
		fail_count = 0;
		return;
	}
	if (oplus_short_c_batt_is_prohibit_chg(chip)) {
		return;
	}
	if (current_adapt == 0) {
		if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {
			current_adapt = chip->limits.temp_little_cold_fastchg_current_ma;
			pre_tbatt_status = BATTERY_STATUS__LITTLE_COLD_TEMP;
		} else if (chip->tbatt_status == BATTERY_STATUS__WARM_TEMP) {
			current_adapt = chip->limits.temp_warm_fastchg_current_ma;
			pre_tbatt_status = BATTERY_STATUS__WARM_TEMP;
		} else if (chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {
			current_adapt = chip->limits.temp_cold_fastchg_current_ma;
			pre_tbatt_status = BATTERY_STATUS__COLD_TEMP;
		} else if (chip->tbatt_status == BATTERY_STATUS__NORMAL) {
			current_adapt = chip->limits.temp_normal_fastchg_current_ma;
			pre_tbatt_status = BATTERY_STATUS__NORMAL;
		} else if (chip->tbatt_status == BATTERY_STATUS__COOL_TEMP) {
			//if ( chip->batt_volt > (chip->vbatt_num * 4180)) {
			if ( chip->batt_volt > 4180) {
				current_adapt = chip->limits.temp_cool_fastchg_current_ma_low;
			} else {
				current_adapt = chip->limits.temp_cool_fastchg_current_ma_high;
			}
			pre_tbatt_status = BATTERY_STATUS__COOL_TEMP;
		} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP) {
			current_adapt = chip->limits.temp_little_cool_fastchg_current_ma;
			pre_tbatt_status = BATTERY_STATUS__LITTLE_COOL_TEMP;
		}
	}
	if (chip->tbatt_status != pre_tbatt_status) {
		current_adapt = 0;
		ibatt_count = 0;
		average_current = 0;
		fail_count = 0;
		return;
	}
	if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP) {
		recharge_volt = chip->limits.temp_little_cold_vfloat_mv - chip->limits.recharge_mv;
		current_init = chip->limits.temp_little_cold_fastchg_current_ma;
		current_limit = chip->batt_capacity_mah * 15 / 100;
		threshold = 50;
	} else if (chip->tbatt_status == BATTERY_STATUS__WARM_TEMP) {
		recharge_volt = chip->limits.temp_warm_vfloat_mv - chip->limits.recharge_mv;
		current_init = chip->limits.temp_warm_fastchg_current_ma;
		current_limit = chip->batt_capacity_mah * 25 / 100;
		threshold = 50;
	} else if (chip->tbatt_status == BATTERY_STATUS__COLD_TEMP) {
		recharge_volt = chip->limits.temp_cold_vfloat_mv - chip->limits.recharge_mv;
		current_init = chip->limits.temp_cold_fastchg_current_ma;
		current_limit = 350;
		threshold = 50;
	} else if (chip->tbatt_status == BATTERY_STATUS__NORMAL) {
		recharge_volt = chip->limits.temp_normal_vfloat_mv- chip->limits.recharge_mv;
		current_init = chip->limits.temp_normal_fastchg_current_ma;
		if (chip->vooc_project) {
			current_limit = 2000;
		} else {
			current_limit = chip->batt_capacity_mah * 65 / 100;
		}
		threshold = 70;
	} else if (chip->tbatt_status == BATTERY_STATUS__COOL_TEMP) {
		recharge_volt = chip->limits.temp_cool_vfloat_mv - chip->limits.recharge_mv;
		if (vbatt_higherthan_4180mv) {
				current_init = chip->limits.temp_cool_fastchg_current_ma_low;
				current_limit = chip->batt_capacity_mah * 15 / 100;
		} else {
				current_init = chip->limits.temp_cool_fastchg_current_ma_high;
				current_limit = chip->batt_capacity_mah * 25 / 100;
		}
		threshold = 50;
	} else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COOL_TEMP) {
		recharge_volt = chip->limits.temp_little_cool_vfloat_mv - chip->limits.recharge_mv;
		current_init = chip->limits.temp_little_cool_fastchg_current_ma;
		current_limit = chip->batt_capacity_mah * 45 / 100;
		threshold = 70;
	}
	if (chip->batt_volt > recharge_volt || chip->led_on) {
		ibatt_count = 0;
		average_current = 0;
		fail_count = 0;
		return;
	}
	if (oplus_vooc_get_allow_reading() == true) {
		current_step = chip->chg_ops->get_chg_current_step();
	} else {
		current_adapt = 0;
		ibatt_count = 0;
		average_current = 0;
		fail_count = 0;
		return;
	}
	if (chip->icharging < 0) {
		ibatt_count++;
		average_current = average_current + chip->icharging;
	}
	/*charge current larger than limit*/
	if ((-1 * chip->icharging) > current_limit) {
		if (current_adapt > current_init) {
			current_adapt = current_init;
		} else {
			current_adapt -= 2 * current_step;
		}
		set_current_flag = true;
		fail_count++;
	} else if (ibatt_count == IBATT_COUNT) {
		average_current = -1 * average_current / ibatt_count;
		threshold += fail_count * current_step;
		if (average_current < current_limit - threshold) {
			current_adapt += current_step;
			set_current_flag = true;
		} else {
			ibatt_count = 0;
			average_current = 0;
		}
	}
	if (set_current_flag == true) {
		if (current_adapt > (current_limit + 100)) {
			current_adapt = current_limit + 100;
		} else if (current_adapt < 103) {/*(512*20%)*/
			current_adapt = 103;
		}
		charger_xlog_printk(CHG_LOG_CRTI,
			"charging_current_write_fast[%d] step[%d]\n",
			current_adapt, current_step);
		chip->chg_ops->charging_current_write_fast(current_adapt);
		ibatt_count = 0;
		average_current = 0;
	}
}

static void oplus_chg_pd_config(struct oplus_chg_chip *chip)
{
	int pd_type = 0;

	if (chip->pd_svooc == true) {
		return;
	}

	if (chip->charger_type != POWER_SUPPLY_TYPE_USB_DCP) {
		chip->pd_chging = false;
		return;
	}
	if (!chip->chg_ops->oplus_chg_pd_setup
			|| !chip->chg_ops->oplus_chg_get_pd_type) {
		return;
	}

	if (chip->dual_charger_support) {
		if (chip->charger_volt > 7500) {
			chip->pd_chging = true;
		} else {
			chip->pd_chging = false;
		}
	}

	if (chip->chg_ops->oplus_chg_get_pd_type() > 0 && oplus_chg_show_vooc_logo_ornot() == false) {
		pd_type = chip->chg_ops->oplus_chg_get_pd_type();

		switch (pd_type) {
		case PD_PPS_ACTIVE:
			if ((chip->pd_svooc == false) && oplus_pps_get_chg_status() != PPS_NOT_SUPPORT
							&& chip->oplus_pps_enter_count == 2) {
				chip->oplus_pps_enter_count = 0;
				chip->pd_chging = false;
				if (oplus_pps_get_chg_status() == PPS_CHECKING) {
					oplus_start_pps_reset();
					oplus_pps_start();
				}
				if (oplus_pps_get_chg_status() == PPS_CHARGE_END) {
					if(chip->pd_chging == true)
						return;
					chip->pd_chging = true;
					chip->chg_ops->oplus_chg_pd_setup();
					chg_err("oppo_chg_pd_config:pps status:PPS_CHARGE_END \n");
				} else {
					return;
				}
			} else if (chip->oplus_pps_enter_count < 2) {
				if(chip->pd_chging == false) {
					chip->pd_chging = true;
					chip->chg_ops->oplus_chg_pd_setup();
				}
				chip->oplus_pps_enter_count = chip->oplus_pps_enter_count + 1;
			}
			break;
		case PD_ACTIVE:
			if(chip->pd_chging == true)
				return;
			chip->pd_chging = true;
			chg_err("POWER_SUPPLY_PD_ACTIVE\n");
			chip->chg_ops->oplus_chg_pd_setup();
			break;
		default:
			chg_err("oppo_chg_pd_config:unhandled type \n");
			break;
		}

		if (oplus_pps_get_chg_status() != PPS_CHARGERING) {
			chip->limits.temp_little_cool_fastchg_current_ma
				= chip->limits.pd_temp_little_cool_fastchg_current_ma;
			chip->limits.temp_normal_fastchg_current_ma
				= chip->limits.pd_temp_normal_fastchg_current_ma;
			chip->limits.temp_little_cold_fastchg_current_ma_high
				= chip->limits.pd_temp_little_cold_fastchg_current_ma_high;
			chip->limits.temp_little_cold_fastchg_current_ma_low
				= chip->limits.pd_temp_little_cold_fastchg_current_ma_low;
			chip->limits.temp_cool_fastchg_current_ma_high
				= chip->limits.pd_temp_cool_fastchg_current_ma_high;
			chip->limits.temp_cool_fastchg_current_ma_low
				= chip->limits.pd_temp_cool_fastchg_current_ma_low;
			chip->limits.temp_warm_fastchg_current_ma
				= chip->limits.pd_temp_warm_fastchg_current_ma;
			chip->limits.input_current_charger_ma
				= chip->limits.pd_input_current_charger_ma;
			oplus_chg_set_charging_current(chip);
			oplus_chg_set_input_current_limit(chip);
			oplus_chg_get_charger_voltage();
		}
		oplus_chg_enable_burst_mode(true);
	}
}

static void oplus_chg_pdqc_to_normal(struct oplus_chg_chip *chip)
{
	int ret = 0;
	static int pdqc_9v = false;

	if (!chip->chg_ops->get_charger_subtype) {
		return;
	}
	if (!chip->chg_ops->oplus_chg_pd_setup || !chip->chg_ops->set_qc_config) {
		return;
	}
	if (oplus_pps_get_chg_status() == PPS_CHARGERING) {
			return;
	}
	if (chip->limits.vbatt_pdqc_to_5v_thr < 0) {
		return;
	}
	if (oplus_chg_show_vooc_logo_ornot() == true) {
		return;
	}
	if (chip->charger_volt > 7500) {
		pdqc_9v = true;
	} else {
		pdqc_9v = false;
	}
	if (chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_PD) {
		if (chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr && pdqc_9v == true) {
			ret = chip->chg_ops->oplus_chg_pd_setup();
			if (ret >= 0) {
				pdqc_9v = false;
				chip->limits.temp_normal_fastchg_current_ma
					= chip->limits.default_temp_normal_fastchg_current_ma;
				chip->limits.temp_little_cool_fastchg_current_ma
					= chip->limits.default_temp_little_cool_fastchg_current_ma;
				chip->limits.temp_little_cool_fastchg_current_ma_high
					= chip->limits.default_temp_little_cool_fastchg_current_ma_high;
				chip->limits.temp_little_cool_fastchg_current_ma_low
					= chip->limits.default_temp_little_cool_fastchg_current_ma_low;
				chip->limits.temp_little_cold_fastchg_current_ma_high
					= chip->limits.default_temp_little_cold_fastchg_current_ma_high;
				chip->limits.temp_little_cold_fastchg_current_ma_low
					= chip->limits.default_temp_little_cold_fastchg_current_ma_low;
				chip->limits.temp_cool_fastchg_current_ma_high
					= chip->limits.default_temp_cool_fastchg_current_ma_high;
				chip->limits.temp_cool_fastchg_current_ma_low
					= chip->limits.default_temp_cool_fastchg_current_ma_low;
				chip->limits.temp_warm_fastchg_current_ma
					= chip->limits.default_temp_warm_fastchg_current_ma;
				chip->limits.input_current_charger_ma
					= chip->limits.default_input_current_charger_ma;
				oplus_chg_set_charging_current(chip);
				oplus_chg_set_input_current_limit(chip);
			}
			oplus_chg_enable_burst_mode(true);
		}
	} else if (chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_QC) {
		if (chip->batt_volt > chip->limits.vbatt_pdqc_to_5v_thr && pdqc_9v == true) {
			ret = chip->chg_ops->set_qc_config();
			if (ret >= 0) {
				pdqc_9v = false;
				chip->limits.temp_normal_fastchg_current_ma
					= chip->limits.default_temp_normal_fastchg_current_ma;
				chip->limits.temp_little_cool_fastchg_current_ma
					= chip->limits.default_temp_little_cool_fastchg_current_ma;
				chip->limits.temp_little_cool_fastchg_current_ma_high
					= chip->limits.default_temp_little_cool_fastchg_current_ma_high;
				chip->limits.temp_little_cool_fastchg_current_ma_low
					= chip->limits.default_temp_little_cool_fastchg_current_ma_low;
				chip->limits.temp_little_cold_fastchg_current_ma_high
					= chip->limits.default_temp_little_cold_fastchg_current_ma_high;
				chip->limits.temp_little_cold_fastchg_current_ma_low
					= chip->limits.default_temp_little_cold_fastchg_current_ma_low;
				chip->limits.temp_cool_fastchg_current_ma_high
					= chip->limits.default_temp_cool_fastchg_current_ma_high;
				chip->limits.temp_cool_fastchg_current_ma_low
					= chip->limits.default_temp_cool_fastchg_current_ma_low;
				chip->limits.temp_warm_fastchg_current_ma
					= chip->limits.default_temp_warm_fastchg_current_ma;
				chip->limits.input_current_charger_ma
					= chip->limits.default_input_current_charger_ma;
				oplus_chg_set_charging_current(chip);
				oplus_chg_set_input_current_limit(chip);
			}
			oplus_chg_enable_burst_mode(true);
		}
	}
}

static void oplus_chg_qc_config(struct oplus_chg_chip *chip)
{
	static int qc_chging = false;
	int ret = 0;

	if (chip->charger_type != POWER_SUPPLY_TYPE_USB_DCP) {
		qc_chging = false;
		return;
	}
	if (!chip->chg_ops->set_qc_config || !chip->chg_ops->get_charger_subtype)
		return;
	chg_err("chip->charger_type[%d], subtype[%d]\n",
		chip->charger_type, chip->chg_ops->get_charger_subtype());

	if (chip->dual_charger_support) {
		if (chip->charger_volt > 7500) {
			qc_chging = true;
		} else {
			qc_chging = false;
		}
	}

	if (qc_chging == false
			&& chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_QC
			&& oplus_chg_show_vooc_logo_ornot() == false) {
		ret = chip->chg_ops->set_qc_config();
		if (ret >= 0) {
			qc_chging = true;
			chg_err("QC  config success");
			chip->limits.temp_little_cool_fastchg_current_ma_high
				= chip->limits.qc_temp_little_cool_fastchg_current_ma;
			chip->limits.temp_little_cool_fastchg_current_ma_low
				= chip->limits.qc_temp_little_cool_fastchg_current_ma;
			chip->limits.temp_normal_fastchg_current_ma
				= chip->limits.qc_temp_normal_fastchg_current_ma;
			chip->limits.temp_little_cold_fastchg_current_ma_high
				= chip->limits.qc_temp_little_cold_fastchg_current_ma_high;
			chip->limits.temp_little_cold_fastchg_current_ma_low
				= chip->limits.qc_temp_little_cold_fastchg_current_ma_low;
			chip->limits.temp_cool_fastchg_current_ma_high
				= chip->limits.qc_temp_cool_fastchg_current_ma_high;
			chip->limits.temp_cool_fastchg_current_ma_low
				= chip->limits.qc_temp_cool_fastchg_current_ma_low;
			chip->limits.temp_warm_fastchg_current_ma
				= chip->limits.qc_temp_warm_fastchg_current_ma;
			chip->limits.input_current_charger_ma
				= chip->limits.qc_input_current_charger_ma;
			oplus_chg_set_input_current_limit(chip);
			oplus_chg_set_charging_current(chip);
			oplus_chg_get_charger_voltage();
		}
		oplus_chg_enable_burst_mode(true);
	}
}

static void oplus_chg_dual_charger_config(struct oplus_chg_chip *chip)
{
	static int enable_slave_cnt = 0;
	static int disable_slave_cnt = 0;

	if (!chip->dual_charger_support) {
		return;
	}

        if (chip->charger_type != POWER_SUPPLY_TYPE_USB_DCP) {
                return;
        }

	if (chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_DEFAULT) {
		return;
	}


	if (chip->slave_charger_enable == false) {
		if (chip->icharging < 0 && (chip->icharging * -1) > chip->slave_chg_enable_ma) {
			enable_slave_cnt++;
		} else {
			enable_slave_cnt = 0;
		}

		if (enable_slave_cnt >= 3) {
			chg_err("Enable slave charger!!\n");
			chip->slave_charger_enable = true;
			oplus_chg_set_input_current_limit(chip);
			oplus_chg_set_charging_current(chip);
			enable_slave_cnt = 0;
		}
	} else {
		if (chip->slave_charger_enable ==  true) {
			if (chip->icharging < 0 && (chip->icharging * -1) < chip->slave_chg_disable_ma) {
				disable_slave_cnt++;
			} else {
				disable_slave_cnt = 0;
			}

			if (disable_slave_cnt >= 3) {
				chg_err("Disable slave charger!!\n");
				chip->slave_charger_enable = false;
				oplus_chg_set_input_current_limit(chip);
				oplus_chg_set_charging_current(chip);
				disable_slave_cnt = 0;
			}
		}
	}
}

static void oplus_chg_reset_adapter_work(struct work_struct *work) {
	oplus_chg_suspend_charger();
	msleep(1000);
	if (g_charger_chip->mmi_chg) {
		oplus_chg_unsuspend_charger();
		oplus_vooc_set_ap_clk_high();
		oplus_vooc_reset_mcu();
	}
}

static void oplus_chg_update_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_chg_chip *chip = container_of(dwork, struct oplus_chg_chip, update_work);

	oplus_charger_detect_check(chip);
	oplus_chg_get_battery_data(chip);
	if (chip->charger_exist) {
		oplus_chg_aicl_check(chip);
		oplus_chg_protection_check(chip);
		oplus_chg_check_tbatt_normal_status(chip);
		oplus_chg_check_status_full(chip);
		oplus_chg_battery_notify_check(chip);
		oplus_chg_pd_config(chip);
		oplus_chg_get_chargerid_voltage(chip);
		oplus_chg_fast_switch_check(chip);
		oplus_chg_chargerid_switch_check(chip);
	}
	oplus_chg_dual_charger_config(chip);
	oplus_chg_qc_config(chip);
	oplus_chg_pdqc_to_normal(chip);
	oplus_chg_ibatt_check_and_set(chip);
	/* oplus_chg_short_c_battery_check(chip); */
	if (chip->shortc_thread)
		wake_up_process(chip->shortc_thread);
	oplus_chg_battery_update_status(chip);
	oplus_chg_kpoc_power_off_check(chip);
	oplus_chg_other_thing(chip);
	/* run again after interval */
	schedule_delayed_work(&chip->update_work, OPLUS_CHG_UPDATE_INTERVAL(oplus_chg_update_slow(chip)));
}
void oplus_chg_cancel_update_work_sync(void)
{
	if (!g_charger_chip) {
		return;
	}

	cancel_delayed_work_sync(&g_charger_chip->update_work);
}

void oplus_chg_restart_update_work(void)
{
	if (!g_charger_chip) {
		return;
	}

	schedule_delayed_work(&g_charger_chip->update_work, 0);
}
bool oplus_chg_wake_update_work(void)
{
	int shedule_work = 0;

	if (!g_charger_chip) {
		chg_err(" g_charger_chip NULL,return\n");
		return true;
	}
	shedule_work = mod_delayed_work(system_wq, &g_charger_chip->update_work, 0);
	return true;
}
EXPORT_SYMBOL(oplus_chg_wake_update_work);

void oplus_chg_reset_adapter(void)
{
	if (!g_charger_chip) {
		return;
	}
	schedule_delayed_work(&g_charger_chip->reset_adapter_work, 0);
}

void oplus_chg_kick_wdt(void)
{
	if (!g_charger_chip) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == true) {
		g_charger_chip->chg_ops->kick_wdt();
	}
}

void oplus_chg_disable_charge(void)
{
	if (!g_charger_chip) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == true) {
		g_charger_chip->chg_ops->charging_disable();
	}
}

void oplus_chg_enable_charge(void)
{
	if (!g_charger_chip) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == true) {
		g_charger_chip->chg_ops->charging_enable();
	}
}

void oplus_chg_suspend_charger(void)
{
	if (!g_charger_chip) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == true) {
		g_charger_chip->chg_ops->charger_suspend();
	}
}

bool oplus_chg_check_disable_charger(void)
{
	bool ret = false;

	if (!g_charger_chip) {
		return false;
	}
	if (oplus_vooc_get_allow_reading() == true) {
		ret = g_charger_chip->chg_ops->get_charging_enable();
	}

	return ret;
}

void oplus_chg_unsuspend_charger(void)
{
	if (!g_charger_chip) {
		return;
	}
	if (oplus_vooc_get_allow_reading() == true) {
		g_charger_chip->chg_ops->charger_unsuspend();
	}
}

int oplus_chg_get_batt_volt(void)
{
	if (!g_charger_chip) {
		return 4000;
	} else {
		return g_charger_chip->batt_volt;
	}
}

int oplus_chg_get_cool_bat_decidegc(void)
{
	if (!g_charger_chip) {
		return -EINVAL;
	} else {
		return g_charger_chip->limits.cool_bat_decidegc;
	}
}

int oplus_chg_get_little_cool_bat_decidegc(void)
{
	if (!g_charger_chip) {
		return -EINVAL;
	} else {
		return g_charger_chip->limits.little_cool_bat_decidegc;
	}
}

int oplus_chg_get_normal_bat_decidegc(void)
{
	if (!g_charger_chip) {
		return -EINVAL;
	} else {
		return g_charger_chip->limits.normal_bat_decidegc;
	}
}

int oplus_chg_get_icharging(void)
{
	if (!g_charger_chip) {
		return 4000;
	} else {
		return g_charger_chip->icharging;
	}
}

int oplus_chg_get_voocphy_support(void)
{
	if (!g_charger_chip) {
		return false;
	} else {
		return g_charger_chip->voocphy_support;
	}
}
EXPORT_SYMBOL(oplus_chg_get_voocphy_support);

int oplus_chg_get_ui_soc(void)
{
	if (!g_charger_chip) {
		return 50;
	} else {
		return g_charger_chip->ui_soc;
	}
}
EXPORT_SYMBOL(oplus_chg_get_ui_soc);

int oplus_chg_get_soc(void)
{
	if (!g_charger_chip) {
		return 50;
	} else {
		return g_charger_chip->soc;
	}
}
EXPORT_SYMBOL(oplus_chg_get_notify_flag);

void oplus_chg_soc_update_when_resume(unsigned long sleep_tm_sec)
{
	int new_soc;
	if (!g_charger_chip) {
		return;
	}
	g_charger_chip->sleep_tm_sec = sleep_tm_sec;
	new_soc = oplus_gauge_get_batt_soc();
	if(new_soc != g_charger_chip->soc){
		g_charger_chip->smooth_soc -= (g_charger_chip->soc - new_soc);
	}
	g_charger_chip->soc = new_soc;
	if(g_charger_chip->smooth_switch){
		oplus_chg_smooth_to_soc(g_charger_chip);
	}
	oplus_chg_update_ui_soc(g_charger_chip);
}

void oplus_chg_soc_update(void)
{
	if (!g_charger_chip) {
		return;
	}
	oplus_chg_update_ui_soc(g_charger_chip);
	oplus_chg_debug_set_soc_info(g_charger_chip);
}

int oplus_chg_get_chg_type(void)
{
	if (!g_charger_chip) {
		return POWER_SUPPLY_TYPE_UNKNOWN;
	} else {
		return g_charger_chip->charger_type;
	}
}

int oplus_chg_get_chg_temperature(void)
{
	if (!g_charger_chip) {
		return 250;
	} else {
		return g_charger_chip->temperature;
	}
}

int oplus_chg_get_notify_flag(void)
{
	if (!g_charger_chip) {
		return 0;
	} else {
		return g_charger_chip->notify_flag;
	}
}

int oplus_is_vooc_project(void)
{
	if (!g_charger_chip) {
		return 0;
	} else {
		return g_charger_chip->vooc_project;
	}
}
EXPORT_SYMBOL(oplus_is_vooc_project);

int oplus_chg_show_vooc_logo_ornot(void)
{
	if (!g_charger_chip) {
		return 0;
	}

	if (oplus_chg_get_wls_type() == OPLUS_CHG_WLS_SVOOC
			|| oplus_chg_get_wls_type() == OPLUS_CHG_WLS_PD_65W) {
		return 1;
	}

	if (g_charger_chip->chg_ctrl_by_vooc) {
		if (oplus_vooc_get_fastchg_started() == true
				&& oplus_vooc_get_fast_chg_type() == CHARGER_SUBTYPE_FASTCHG_VOOC) {
			//chg_err("show_vooc_logo_ornot by vooc\n");
			if (g_charger_chip->prop_status != POWER_SUPPLY_STATUS_FULL
					&& (g_charger_chip->stop_voter == CHG_STOP_VOTER__FULL
					|| g_charger_chip->stop_voter == CHG_STOP_VOTER_NONE)) {
				return 1;
			} else {
				return 0;
			}
		}
	}
	if (oplus_vooc_get_fastchg_started()) {
		return 1;
	} else if (oplus_vooc_get_fastchg_to_normal() == true
			|| oplus_vooc_get_fastchg_to_warm() == true
			|| oplus_vooc_get_fastchg_dummy_started() == true
			|| oplus_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE) {
		if (g_charger_chip->prop_status != POWER_SUPPLY_STATUS_FULL
				&&(g_charger_chip->stop_voter == CHG_STOP_VOTER__FULL
				|| g_charger_chip->stop_voter == CHG_STOP_VOTER_NONE)) {
			return 1;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(oplus_chg_show_vooc_logo_ornot);

bool get_otg_switch(void)
{
	if (!g_charger_chip) {
		return 0;
	} else {
		return g_charger_chip->otg_switch;
	}
}

bool oplus_chg_get_otg_online(void)
{
	if (!g_charger_chip) {
		return 0;
	} else {
		return g_charger_chip->otg_online;
	}
}

void oplus_chg_set_otg_online(bool online)
{
	if (g_charger_chip) {
		g_charger_chip->otg_online = online;
	}
}

bool oplus_chg_get_batt_full(void)
{
	if (!g_charger_chip) {
		return 0;
	} else {
		return g_charger_chip->batt_full;
	}
}

bool oplus_chg_get_rechging_status(void)
{
	if (!g_charger_chip) {
		return 0;
	} else {
		return g_charger_chip->in_rechging;
	}
}

bool oplus_chg_get_chging_status(void)
{
	if (!g_charger_chip) {
		return false;
	} else {
		return g_charger_chip->chging_on;
	}
}

bool oplus_chg_check_chip_is_null(void)
{
	if (!g_charger_chip) {
		return true;
	} else {
		return false;
	}
}
EXPORT_SYMBOL(oplus_chg_check_chip_is_null);

void oplus_chg_set_charger_type_unknown(void)
{
	if (g_charger_chip) {
		g_charger_chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		g_charger_chip->real_charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}
}

int oplus_chg_get_charger_voltage(void)
{
	if (!g_charger_chip) {
		return -500;
	} else {
		g_charger_chip->charger_volt = g_charger_chip->chg_ops->get_charger_volt();
	}
	return g_charger_chip->charger_volt;
}

void oplus_chg_set_chargerid_switch_val(int value)
{
	if (g_charger_chip && g_charger_chip->chg_ops->set_chargerid_switch_val) {
		g_charger_chip->chg_ops->set_chargerid_switch_val(value);
	}
}

void oplus_chg_clear_chargerid_info(void)
{
	if (g_charger_chip && g_charger_chip->chg_ops->set_chargerid_switch_val) {
		g_charger_chip->chargerid_volt = 0;
		g_charger_chip->chargerid_volt_got = false;
	}
}

int oplus_chg_get_cool_down_status(void)
{
	int ret = 0;
	struct oplus_chg_chip *chip = g_charger_chip;

	if (!chip) {
		return 0;
	}

	if (chip->smart_charge_user == SMART_CHARGE_USER_USBTEMP) {
		if (chip->smart_charging_screenoff) {
			chip->usbtemp_cool_down = oplus_convert_current_to_level(chip, USBTEMP_CHARGING_CURRENT_LIMIT);
		} else {
			chip->usbtemp_cool_down = 3;
		}
		if (chip->cool_down) {
			ret = (chip->usbtemp_cool_down < chip->cool_down) ? chip->usbtemp_cool_down : chip->cool_down;
		} else {
			ret = chip->usbtemp_cool_down;
		}
		charger_xlog_printk(CHG_LOG_CRTI, "cool_down=%d,usbtemp_cool_down=%d\n", chip->cool_down, chip->usbtemp_cool_down);
	} else {
		ret = chip->cool_down;
	}

	if (oplus_chg_get_voocphy_support() == NO_VOOCPHY) {
		if (chip->bcc_cool_down) {
			if (ret == 0) {
				ret = chip->bcc_cool_down;
			} else {
				ret = (chip->bcc_cool_down < ret) ? chip->bcc_cool_down : ret;
			}
		}
	}

	charger_xlog_printk(CHG_LOG_CRTI, "ret = 0x%x, cool_down=%d,usbtemp_cool_down=%d, bcc_cool_down=%d\n",
		ret, chip->cool_down, chip->usbtemp_cool_down, chip->bcc_cool_down);

	return ret;
}

struct current_level {
	int level;
	int icharging;
};

struct current_level SVOOC_2_0[] = {
	{1, 1000}, {2, 1500}, {3, 2000}, {4, 2500}, {5, 3000},
	{6, 3500}, {7, 4000}, {8, 4500}, {9, 5000}, {10, 5500},
	{11, 6000}, {12, 6300}, {13, 6500}, {14, 7000}, {15, 7300},
	{15, 7500}, {16, 8000}, {17, 8500}, {18, 9000}, {19, 9500},
	{20, 10000}, {21, 10500}, {22, 11000}, {23, 11500}, {24, 12000}, {25, 12500},
};

struct current_level SVOOC_1_0[] = {
	{1, 2000}, {2, 3000}, {3, 4000}, {4, 5000}, {5, 6000},
	{6, 7000}, {7, 8000}, {8, 9000}, {9, 10000}, {10, 1200},
	{11, 12000}, {12, 12600}
};

int find_level_to_current(int val, struct current_level *table, int len)
{
	int i = 0;
	bool find_out_flag = false;

	for(i = 0; i < len; i++) {
		if(table[i].icharging > val) {
			find_out_flag = true;
			break;
		}
		find_out_flag = false;
	}
	if(find_out_flag == true) {
		return table[i-1].level;
	} else {
		return 0;
	}
}

int oplus_convert_current_to_level(struct oplus_chg_chip *chip, int val)
{
	int level = 0;
	int reply_bits = 0;
	reply_bits = oplus_vooc_get_reply_bits();
	if(!val || !reply_bits) {
		return level;
	}
	if(reply_bits == 7) {
		if(chip->vbatt_num == 2) {
			level = find_level_to_current(val, SVOOC_2_0, ARRAY_SIZE(SVOOC_2_0));
		} else {
			level = find_level_to_current(val, SVOOC_1_0, ARRAY_SIZE(SVOOC_1_0));
		}
	} else {
		level = val / 1000;
	}
	return level;
}

#define BATT_NTC_CTRL_THRESHOLD_LOW -100
#define BATT_NTC_CTRL_THRESHOLD_HIGH 600
int oplus_chg_override_by_shell_temp(int temp)
{
	if (!g_charger_chip) {
		return 0;
	}
	if (oplus_is_power_off_charging(NULL)) {
		return 0;
	}
//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
#ifdef SUPPORT_WPC
	if (g_charger_chip->wireless_support && (oplus_wpc_get_wireless_charge_start() == true
			|| oplus_chg_is_wls_present())) {
		return 0;
	}
#endif
//#endif
	if ((g_charger_chip->charger_exist) && g_charger_chip->smart_charging_screenoff &&
			(temp > BATT_NTC_CTRL_THRESHOLD_LOW) && (temp < BATT_NTC_CTRL_THRESHOLD_HIGH)) {
		charger_xlog_printk(CHG_LOG_CRTI, "charging override by shell temperature\n");
		return 1;
	}
	return 0;
}

int oplus_chg_get_shell_temp(void) {
	int temp_val = 0, rc = -EINVAL;

	if (!g_charger_chip) {
		return TEMPERATURE_INVALID;
	}

	rc = thermal_zone_get_temp(g_charger_chip->shell_themal, &temp_val);
	if (rc) {
		g_charger_chip->shell_themal = thermal_zone_get_zone_by_name("shell_back");
		rc = thermal_zone_get_temp(g_charger_chip->shell_themal, &temp_val);
		if (rc) {
			chg_err("thermal_zone_get_temp get error");
			return g_charger_chip->shell_temp;
		}
	}
	g_charger_chip->shell_temp = temp_val / 100;
	return g_charger_chip->shell_temp;
}

int oplus_smart_charge_by_bcc(struct oplus_chg_chip *chip, int val)
{
	int subtype = 0;
	int ret = 0;
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	union power_supply_propval pval = {0, };
	ret = -EINVAL;
	#endif

	/*if (!chip->smart_chg_bcc_support) {
		charger_xlog_printk(CHG_LOG_CRTI, "not support smart chg bcc mode\n");
		return 0;
	}*/

	chip->bcc_current = val * 100;
	if (val == 0) {
		charger_xlog_printk(CHG_LOG_CRTI, "val is invalid\n");
		return 0;
	}

	#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	ret = power_supply_get_property(chip->usb_psy, POWER_SUPPLY_PROP_FAST_CHG_TYPE, &pval);
	if (ret < 0) {
		pr_err("Failed to read charging subtype\n");
		return -1
	}
	subtype = pval.intval;
	#else
	subtype = oplus_get_fast_chg_type();
	#endif

	switch(subtype) {
	case CHARGER_SUBTYPE_DEFAULT:
	case CHARGER_SUBTYPE_FASTCHG_VOOC:
	case CHARGER_SUBTYPE_PD:
	case CHARGER_SUBTYPE_QC:
		break;
	case CHARGER_SUBTYPE_FASTCHG_SVOOC:
		if (chip->chg_ops
			&& chip->chg_ops->set_bcc_curr_to_voocphy)
			ret = chip->chg_ops->set_bcc_curr_to_voocphy(val);
		chip->bcc_cool_down = oplus_convert_current_to_level(chip, chip->bcc_current);
		break;
	default:
		if (subtype > 10) {
			if (chip->chg_ops
				&& chip->chg_ops->set_bcc_curr_to_voocphy)
				ret = chip->chg_ops->set_bcc_curr_to_voocphy(val);
		}
		chip->bcc_cool_down = oplus_convert_current_to_level(chip, chip->bcc_current);
		break;
	}

	charger_xlog_printk(CHG_LOG_CRTI, "bcc_current[%d], bcc_cool_down[%d]\n", chip->bcc_current, chip->bcc_cool_down);

	return ret;
}

void oplus_smart_charge_by_shell_temp(struct oplus_chg_chip *chip, int val) {
	int subtype = 0, rc = -EINVAL;

	if (!chip) {
		return;
	}

	if (chip->shell_themal) {
		rc = thermal_zone_get_temp(chip->shell_themal, &chip->shell_temp);
		if (rc) {
			g_charger_chip->shell_themal = thermal_zone_get_zone_by_name("shell_back");
			rc = thermal_zone_get_temp(g_charger_chip->shell_themal, &chip->shell_temp);
			if (rc) {
				chip->shell_temp = (val >> 16) & 0XFFFF;
				chg_err("thermal_zone_get_temp get error");
			} else {
				chip->shell_temp = chip->shell_temp / 100;
			}
		} else {
			chip->shell_temp = chip->shell_temp / 100;
		}
	} else {
		chip->shell_temp = (val >> 16) & 0XFFFF;
	}
	val = val & 0XFFFF;

	charger_xlog_printk(CHG_LOG_CRTI, "val->intval = [%04x], shell_temp = [%04x], set shell_temp_down = [%d].\n", val, chip->shell_temp, chip->cool_down);
	if (chip->led_on) {
		return;
	}
	if (!val) {
		if(chip->cool_down != 0){
			chip->cool_down = 0;
			chip->limits.input_current_charger_ma = chip->limits.default_input_current_charger_ma;
			chip->limits.input_current_vooc_ma_high = chip->limits.default_input_current_vooc_ma_high;
			chip->limits.input_current_vooc_ma_warm = chip->limits.default_input_current_vooc_ma_warm;
			chip->limits.input_current_vooc_ma_normal = chip->limits.default_input_current_vooc_ma_normal;
			chip->limits.pd_input_current_charger_ma = chip->limits.default_pd_input_current_charger_ma;
			chip->limits.qc_input_current_charger_ma = chip->limits.default_qc_input_current_charger_ma;
			chip->cool_down_done = true;
			chip->cool_down_force_5v = false;
			chip->chg_ctrl_by_cool_down = false;
		}
		return;
	}

	subtype = oplus_chg_get_fast_chg_type();
	charger_xlog_printk(CHG_LOG_CRTI, "get subtype = [%d]\n", subtype);
	switch(subtype) {
	case CHARGER_SUBTYPE_DEFAULT:
		break;
	case CHARGER_SUBTYPE_FASTCHG_VOOC:
		chip->limits.input_current_vooc_ma_high = val;
		chip->limits.input_current_vooc_ma_warm = val;
		chip->limits.input_current_vooc_ma_normal = val;
		chip->limits.input_current_cool_down_ma = val;
		chip->cool_down_done = true;
		chip->cool_down_force_5v = true;
		chip->chg_ctrl_by_cool_down = true;
		break;
	case CHARGER_SUBTYPE_FASTCHG_SVOOC:
		/*chip->cool_down = val / 1000;*/
		/*for 7bits 25 current steps*/
		chip->cool_down = oplus_convert_current_to_level(chip, val);
		break;
	case CHARGER_SUBTYPE_PD:
		chip->limits.input_current_cool_down_ma = val;
		chip->limits.pd_input_current_charger_ma = val;
		chip->limits.qc_input_current_charger_ma = val;
		chip->cool_down_done = true;
		chip->cool_down_force_5v = false;
		chip->chg_ctrl_by_cool_down = true;
		break;
	case CHARGER_SUBTYPE_QC:
		chip->limits.input_current_cool_down_ma = val;
		chip->limits.pd_input_current_charger_ma = val;
		chip->limits.qc_input_current_charger_ma = val;
		chip->cool_down_done = true;
		chip->cool_down_force_5v = false;
		chip->chg_ctrl_by_cool_down = true;
		break;
	default:
		if (subtype > 10) {
			chip->cool_down = oplus_convert_current_to_level(chip, val);
		}
		break;
	}
}

static const int cool_down_current_limit_normal[6] = {1200, 1500, 2000, 2000, 2000, 2000};
static const int cool_down_current_limit_vooc[6] = {1000, 1500, 2000, 2500, 3000, 3500};
void oplus_smart_charge_by_cool_down(struct oplus_chg_chip *chip, int val)
{
	OPLUS_CHARGER_SUBTYPE esubtype = CHARGER_SUBTYPE_DEFAULT;

	if (!chip) {
		return;
	}

	if (val == chip->cool_down)
		return;

	if (!val) {
		chip->cool_down = 0;
		esubtype = chip->chg_ops->get_charger_subtype();
		if (CHARGER_SUBTYPE_PD == esubtype)
			chip->limits.input_current_charger_ma = chip->limits.default_pd_input_current_charger_ma;
		else if (CHARGER_SUBTYPE_QC == esubtype)
			chip->limits.input_current_charger_ma = chip->limits.default_qc_input_current_charger_ma;
		else
			chip->limits.input_current_charger_ma = chip->limits.default_input_current_charger_ma;
		chip->limits.input_current_vooc_ma_high = chip->limits.default_input_current_vooc_ma_high;
		chip->limits.input_current_vooc_ma_warm = chip->limits.default_input_current_vooc_ma_warm;
		chip->limits.input_current_vooc_ma_normal = chip->limits.default_input_current_vooc_ma_normal;
		chip->limits.pd_input_current_charger_ma = chip->limits.default_pd_input_current_charger_ma;
		chip->limits.qc_input_current_charger_ma = chip->limits.default_qc_input_current_charger_ma;
		chip->cool_down_done = true;
		charger_xlog_printk(CHG_LOG_CRTI, "val->intval = [%04x], set cool_down = [%d].\n", val, chip->cool_down);
		return;
	}
	chip->cool_down = val;
	if (val > 6) {
		chip->limits.input_current_charger_ma = cool_down_current_limit_normal[6 - 1];
		chip->limits.pd_input_current_charger_ma = cool_down_current_limit_normal[6 - 1];
		chip->limits.qc_input_current_charger_ma = cool_down_current_limit_normal[6 - 1];
		chip->limits.input_current_vooc_ma_high = cool_down_current_limit_vooc[6 - 1];
		chip->limits.input_current_vooc_ma_warm = cool_down_current_limit_vooc[6 - 1];
		chip->limits.input_current_vooc_ma_normal = cool_down_current_limit_vooc[6 - 1];
		chip->cool_down_done = true;
	} else {
		chip->limits.input_current_charger_ma = cool_down_current_limit_normal[val - 1];
		chip->limits.pd_input_current_charger_ma = cool_down_current_limit_normal[val - 1];
		chip->limits.qc_input_current_charger_ma = cool_down_current_limit_normal[val - 1];
		chip->limits.input_current_vooc_ma_high = cool_down_current_limit_vooc[val - 1];
		chip->limits.input_current_vooc_ma_warm = cool_down_current_limit_vooc[val - 1];
		chip->limits.input_current_vooc_ma_normal = cool_down_current_limit_vooc[val - 1];
		chip->cool_down_done = true;
	}

	charger_xlog_printk(CHG_LOG_CRTI, "val->intval = [%04x], set cool_down = [%d].\n", val, chip->cool_down);
}

int oplus_is_rf_ftm_mode(void)
{
	int boot_mode = get_boot_mode();
#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (boot_mode == META_BOOT || boot_mode == FACTORY_BOOT
			|| boot_mode == ADVMETA_BOOT || boot_mode == ATE_FACTORY_BOOT) {
		chg_debug(" boot_mode:%d, return\n",boot_mode);
		return true;
	} else {
		chg_debug(" boot_mode:%d, return false\n",boot_mode);
		return false;
	}
#else
	if(boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN
			|| boot_mode == MSM_BOOT_MODE__FACTORY){
		chg_debug(" boot_mode:%d, return\n",boot_mode);
		return true;
	} else {
		chg_debug(" boot_mode:%d, return false\n",boot_mode);
		return false;
	}
#endif
}

#ifdef CONFIG_OPLUS_CHARGER_MTK
/*
int get_oplus_short_check_fast_to_normal(void)
{
	if (!g_charger_chip) {
		return 0;
	} else {
		return g_charger_chip->short_check_fast_to_normal_flag;
	}

}
*/
int oplus_get_prop_status(void)
{
	if (!g_charger_chip) {
		return 0;
	} else {
		return g_charger_chip->prop_status;
	}
}
EXPORT_SYMBOL(oplus_get_prop_status);
#endif

#define OPLUS_TBATT_HIGH_PWROFF_COUNT	(18)
#define OPLUS_TBATT_EMERGENCY_PWROFF_COUNT	(6)

DECLARE_WAIT_QUEUE_HEAD(oplus_tbatt_pwroff_wq);

static int oplus_tbatt_power_off_kthread(void *arg)
{
	int over_temp_count = 0, emergency_count = 0;
	int batt_temp = 0;
	//struct oplus_chg_chip *chip = (struct oplus_chg_chip *)arg;
	struct sched_param param = {.sched_priority = MAX_RT_PRIO-1};

	sched_setscheduler(current, SCHED_FIFO, &param);
	tbatt_pwroff_enable = 1;

	while (!kthread_should_stop()) {
		schedule_timeout_interruptible(round_jiffies_relative(msecs_to_jiffies(5*1000)));
		//chg_err(" tbatt_pwroff_enable:%d over_temp_count[%d] start\n", tbatt_pwroff_enable, over_temp_count);
		if (!tbatt_pwroff_enable) {
			emergency_count = 0;
			over_temp_count = 0;
			wait_event_interruptible(oplus_tbatt_pwroff_wq, tbatt_pwroff_enable == 1);
		}
		if (oplus_vooc_get_fastchg_started() == true) {
			batt_temp = oplus_gauge_get_prev_batt_temperature();
		} else {
			batt_temp = oplus_gauge_get_batt_temperature();
		}
		if (batt_temp > OPCHG_PWROFF_EMERGENCY_BATT_TEMP) {
			emergency_count++;
			chg_err(" emergency_count:%d \n", emergency_count);
		} else {
			emergency_count = 0;
		}
		if (batt_temp > OPCHG_PWROFF_HIGH_BATT_TEMP) {
			over_temp_count++;
			chg_err("over_temp_count[%d] \n", over_temp_count);
		} else {
			over_temp_count = 0;
		}
		//chg_err("over_temp_count[%d], chip->temperature[%d]\n", over_temp_count, batt_temp);
		if (get_eng_version() != HIGH_TEMP_AGING) {
			if (over_temp_count >= OPLUS_TBATT_HIGH_PWROFF_COUNT
				|| emergency_count >= OPLUS_TBATT_EMERGENCY_PWROFF_COUNT) {
				chg_err("ERROR: battery temperature is too high, goto power off\n");
				/*msleep(1000);*/
				//for dx1 bringup
				//machine_power_off();
				kernel_power_off();
			}
		}
	}
	return 0;
}

int oplus_tbatt_power_off_task_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		return -1;
	}
	chip->tbatt_pwroff_task
		= kthread_create(oplus_tbatt_power_off_kthread, chip, "tbatt_pwroff");
	if(chip->tbatt_pwroff_task){
		wake_up_process(chip->tbatt_pwroff_task);
	}else{
		chg_err("ERROR: chip->tbatt_pwroff_task creat fail\n");
		return -1;
	}
	return 0;
}

void oplus_tbatt_power_off_task_wakeup(void)
{
	wake_up_interruptible(&oplus_tbatt_pwroff_wq);
	return;
}

bool oplus_get_chg_powersave(void)
{
	if (!g_charger_chip) {
		return false;
	} else {
		return g_charger_chip->chg_powersave;
	}
}

int oplus_get_chg_unwakelock(void)
{
	if (!g_charger_chip) {
                return 0;
	} else {
		return g_charger_chip->unwakelock_chg;
	}
}

bool oplus_get_vbatt_higherthan_xmv(void)
{
        if (!g_charger_chip) {
                return false;
        } else {
                return vbatt_higherthan_4180mv;
        }
}

void oplus_chg_set_input_current_without_aicl(int current_ma)
{
    if (!g_charger_chip) {
        return ;
    } else {
        if(g_charger_chip->chg_ops->input_current_write_without_aicl && oplus_vooc_get_allow_reading() == true) {
            g_charger_chip->chg_ops->input_current_write_without_aicl(current_ma);
            chg_err("current_ma[%d]\n", current_ma);
        }
    }
}

void oplus_chg_config_charger_vsys_threshold(int val)
{
    if (!g_charger_chip) {
        return ;
    } else {
        if(g_charger_chip->chg_ops->set_charger_vsys_threshold && oplus_vooc_get_allow_reading() == true) {
            g_charger_chip->chg_ops->set_charger_vsys_threshold(val);
            chg_err("set val[%d]\n", val);
        }
    }
}

void oplus_chg_enable_burst_mode(bool enable)
{
    if (!g_charger_chip) {
        return ;
    } else {
        if(g_charger_chip->chg_ops->enable_burst_mode && oplus_vooc_get_allow_reading() == true) {
            g_charger_chip->chg_ops->enable_burst_mode(enable);
            chg_err("set val[%d]\n", enable);
        }
    }
}

int oplus_chg_get_tbatt_status(void)
{
    if (!g_charger_chip) {
        return 0;
    } else {
		return g_charger_chip->tbatt_status;
    }
}

int oplus_chg_match_temp_for_chging(void)
{
	int batt_temp = 0;
	int shell_temp = 0;
	int diff = 0;
	int chging_temp = 0;

	if (!g_charger_chip) {
		return chging_temp;
	}
	if (oplus_vooc_get_fastchg_started() == true) {
		batt_temp = oplus_gauge_get_prev_batt_temperature();
	} else {
		batt_temp = oplus_gauge_get_batt_temperature();
	}
	if (oplus_chg_override_by_shell_temp(batt_temp)) {
		shell_temp = oplus_chg_get_shell_temp();
		diff = shell_temp - batt_temp;
		if(diff < 150 && diff > -150  && batt_temp >= 320
						&& batt_temp < g_charger_chip->limits.hot_bat_decidegc) {
			chging_temp = shell_temp;
		} else {
			chging_temp = batt_temp;
		}
	} else {
		chging_temp = batt_temp;
	}
	g_charger_chip->tbatt_temp = batt_temp;

	return chging_temp;
}

int oplus_chg_get_fast_chg_type(void)
{
	int type = 0;

	if (!g_charger_chip) {
		return 0;
	}

	type = oplus_vooc_get_fast_chg_type();
	if (type == 0 && g_charger_chip->chg_ops->get_charger_subtype) {
		type = g_charger_chip->chg_ops->get_charger_subtype();
	}

//for dx1 bringup
//#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (type == 0) {
		if (oplus_wpc_get_adapter_type() == CHARGER_SUBTYPE_FASTCHG_VOOC
				|| oplus_wpc_get_adapter_type() == CHARGER_SUBTYPE_FASTCHG_SVOOC)
			type = oplus_wpc_get_adapter_type();
	}
//#endif

	return type;
}

struct oplus_chg_chip *oplus_chg_get_chg_struct(void)
{
	return g_charger_chip;
}

int oplus_chg_get_pps_type(void)
{
	if (!g_charger_chip) {
		return PD_INACTIVE;
	}

	if (!g_charger_chip->chg_ops || !g_charger_chip->chg_ops->oplus_chg_get_pd_type) {
		return PD_INACTIVE;
	}

	return g_charger_chip->chg_ops->oplus_chg_get_pd_type();
}

void oplus_chg_check_bcc_curr_done(void)
{
	struct oplus_chg_chip *chip = g_charger_chip;

	if (!chip) {
		return;
	}

	mutex_lock(&chip->bcc_curr_done_mutex);
	if (chip->bcc_curr_done == BCC_CURR_DONE_REQUEST) {
		chip->bcc_curr_done = BCC_CURR_DONE_ACK;
		chg_err("bcc_curr_done:%d\n", chip->bcc_curr_done);
	}
	mutex_unlock(&chip->bcc_curr_done_mutex);
}

int oplus_chg_get_bcc_curr_done_status(void)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	int ret = 0;

	if (!chip) {
		return 0;
	}

	mutex_lock(&chip->bcc_curr_done_mutex);
	ret = chip->bcc_curr_done;
	mutex_unlock(&chip->bcc_curr_done_mutex);

	chg_err("bcc_curr_done:%d\n", ret);
	return ret;
}

