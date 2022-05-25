// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#include <linux/configfs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/nls.h>
#include <linux/kdev_t.h>
#include <soc/oplus/system/oplus_chg.h>
#include "oplus_charger.h"
#include "oplus_gauge.h"
#include "oplus_vooc.h"
#include "oplus_short.h"
#include "oplus_adapter.h"
#include "oplus_wireless.h"
#include "charger_ic/oplus_short_ic.h"
#include "oplus_chg_wls.h"
#include "oplus_debug_info.h"
#include "oplus_pps.h"
//#include "wireless_ic/oplus_p922x.h"


static struct class *oplus_chg_class;
static struct device *oplus_ac_dir;
static struct device *oplus_usb_dir;
static struct device *oplus_battery_dir;
static struct device *oplus_wireless_dir;
static struct device *oplus_common_dir;

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


/**********************************************************************
* ac device nodes
**********************************************************************/
static ssize_t ac_online_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_ac_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->charger_exist) {
		if ((chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP)
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

	if (chip->ac_online) {
		chg_err("chg_exist:%d, ac_online:%d\n", chip->charger_exist, chip->ac_online);
	}

	return sprintf(buf, "%d\n", chip->ac_online);
}
static DEVICE_ATTR(online, S_IRUGO, ac_online_show, NULL);

static ssize_t ac_type_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%s\n", "Mains");
}
static DEVICE_ATTR(type, S_IRUGO, ac_type_show, NULL);

static struct device_attribute *oplus_ac_attributes[] = {
	&dev_attr_online,
	&dev_attr_type,
	NULL
};


/**********************************************************************
* usb device nodes
**********************************************************************/
int __attribute__((weak)) oplus_get_fast_chg_type(void)
{
	return 0;
}

static ssize_t fast_chg_type_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int fast_chg_type = 0;

	fast_chg_type = oplus_get_fast_chg_type();
	return sprintf(buf, "%d\n", fast_chg_type);
}
static DEVICE_ATTR_RO(fast_chg_type);

int __attribute__((weak)) oplus_get_otg_online_status(void)
{
	return 0;
}

static ssize_t otg_online_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;
	int otg_online = 0;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_usb_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	otg_online = oplus_get_otg_online_status();

	return sprintf(buf, "%d\n", otg_online);
}
static DEVICE_ATTR_RO(otg_online);

static ssize_t otg_switch_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_usb_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->otg_switch);
}

void __attribute__((weak)) oplus_set_otg_switch_status(bool value)
{
	return;
}

static ssize_t otg_switch_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_usb_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	if (val == 1) {
		chip->otg_switch = true;
		oplus_set_otg_switch_status(true);
	} else {
		chip->otg_switch = false;
		chip->otg_online = false;
		oplus_set_otg_switch_status(false);
	}

	return count;
}
static DEVICE_ATTR_RW(otg_switch);

int  __attribute__((weak)) oplus_get_usb_status(void)
{
	return 0;
}

static ssize_t usb_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int status = 0;

	status = oplus_get_usb_status();
	return sprintf(buf, "%d\n", status);
}
static DEVICE_ATTR_RO(usb_status);

int __attribute__((weak)) oplus_get_typec_cc_orientation(void)
{
	return 0;
}

static ssize_t typec_cc_orientation_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        int cc_orientation = 0;

        cc_orientation = oplus_get_typec_cc_orientation();

        return sprintf(buf, "%d\n", cc_orientation);
}
static DEVICE_ATTR_RO(typec_cc_orientation);

static struct device_attribute *oplus_usb_attributes[] = {
	&dev_attr_otg_online,
	&dev_attr_otg_switch,
	&dev_attr_usb_status,
	&dev_attr_typec_cc_orientation,
	&dev_attr_fast_chg_type,
	NULL
};


/**********************************************************************
* battery device nodes
**********************************************************************/
static ssize_t authenticate_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->authenticate);
}
static DEVICE_ATTR_RO(authenticate);

static ssize_t battery_cc_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->batt_cc);
}
static DEVICE_ATTR_RO(battery_cc);

static ssize_t battery_fcc_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->batt_fcc);
}
static DEVICE_ATTR_RO(battery_fcc);

static ssize_t battery_rm_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->batt_rm);
}
static DEVICE_ATTR_RO(battery_rm);

static ssize_t battery_soh_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->batt_soh);
}
static DEVICE_ATTR_RO(battery_soh);

static ssize_t soh_report_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", oplus_chg_get_soh_report());
}
static DEVICE_ATTR_RO(soh_report);

static ssize_t cc_report_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", oplus_chg_get_cc_report());
}
static DEVICE_ATTR_RO(cc_report);

#ifdef CONFIG_OPLUS_CALL_MODE_SUPPORT
static ssize_t call_mode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->calling_on);
}

static ssize_t call_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}
	chip->calling_on = val;
	if (is_wls_ocm_available(chip))
		oplus_chg_anon_mod_event(chip->wls_ocm, val ? OPLUS_CHG_EVENT_CALL_ON : OPLUS_CHG_EVENT_CALL_OFF);

	return count;
}
static DEVICE_ATTR_RW(call_mode);
#endif /*CONFIG_OPLUS_CALL_MODE_SUPPORT*/

static ssize_t charge_technology_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->vooc_project);
}
static DEVICE_ATTR_RO(charge_technology);

#ifdef CONFIG_OPLUS_CHIP_SOC_NODE
static ssize_t chip_soc_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->soc);
}
static DEVICE_ATTR_RO(chip_soc);
#endif /*CONFIG_OPLUS_CHIP_SOC_NODE*/

#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
static ssize_t cool_down_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->cool_down);
}

static ssize_t cool_down_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;
	union oplus_chg_mod_propval pval;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}
	oplus_smart_charge_by_cool_down(chip, val);
	if (is_wls_ocm_available(chip)) {
		pval.intval = val;
		oplus_chg_mod_set_property(chip->wls_ocm,
			OPLUS_CHG_PROP_COOL_DOWN, &pval);
	}

	return count;
}
static DEVICE_ATTR_RW(cool_down);
#endif /*CONFIG_OPLUS_SMART_CHARGER_SUPPORT*/

static ssize_t em_mode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->em_mode);
}

static ssize_t em_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
        }

	if (val == 0) {
		chip->em_mode = false;
	} else {
		chip->em_mode = true;
#ifndef CONFIG_OPLUS_CHARGER_MTK
		if (chip->chg_ops && chip->chg_ops->subcharger_force_enable)
			chip->chg_ops->subcharger_force_enable();
#endif
	}

        return count;
}
static DEVICE_ATTR_RW(em_mode);

static ssize_t fast_charge_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}
	val = oplus_chg_show_vooc_logo_ornot();

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(fast_charge);

static ssize_t mmi_charging_enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->mmi_chg);
}

static ssize_t mmi_charging_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	int ret = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chg_err("set mmi_chg = [%d].\n", val);
	if (val == 0) {
		if (chip->unwakelock_chg == 1) {
			ret = -EINVAL;
			chg_err("unwakelock testing , this test not allowed.\n");
		} else {
			chip->mmi_chg = 0;
			oplus_chg_turn_off_charging(chip);
			if (oplus_vooc_get_fastchg_started() == true) {
				oplus_chg_set_chargerid_switch_val(0);
				oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
				chip->mmi_fastchg = 0;
			}
		}
	} else {
		if (chip->unwakelock_chg == 1) {
			ret = -EINVAL;
			chg_err("unwakelock testing , this test not allowed.\n");
		} else {
			chip->mmi_chg = 1;
			if (chip->mmi_fastchg == 0) {
				oplus_chg_clear_chargerid_info();
			}
			chip->mmi_fastchg = 1;
			oplus_chg_turn_on_charging(chip);
		}
	}

	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_RW(mmi_charging_enable);

#ifdef CONFIG_OPLUS_CHARGER_MTK
static ssize_t stop_charging_enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->stop_chg);
}

static ssize_t stop_charging_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
        }

	chg_err("set stop_chg = [%d].\n", val);

	if (val == 0) {
		chip->stop_chg = false;
	} else {
		chip->stop_chg = true;
	}

        return count;
}
static DEVICE_ATTR_RW(stop_charging_enable);

int __attribute__((weak)) oplus_get_water_detect(void)
{
	return 0;
}

void __attribute__((weak)) oplus_set_water_detect(bool enable)
{
	return;
}

static ssize_t water_detect_feature_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", oplus_get_water_detect());
}

static ssize_t water_detect_feature_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chg_err("set water_detect_feature = [%d].\n", val);

	if (val == 0) {
		oplus_set_water_detect(false);
	} else {
		oplus_set_water_detect(true);
	}

	return count;
}
static DEVICE_ATTR_RW(water_detect_feature);
#endif

static ssize_t battery_notify_code_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->notify_code);
}
static DEVICE_ATTR_RO(battery_notify_code);

int __attribute__((weak)) oplus_chg_get_subcurrent(void)
{
        return 0;
}

static ssize_t sub_current_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
	struct oplus_chg_chip *chip = NULL;
	int sub_current = 0;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (chip->dual_charger_support)
		sub_current = oplus_chg_get_subcurrent();

	return sprintf(buf, "%d\n", sub_current);
}
static DEVICE_ATTR_RO(sub_current);

static ssize_t charge_timeout_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->chging_over_time);
}
static DEVICE_ATTR_RO(charge_timeout);

static ssize_t adapter_fw_update_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", oplus_vooc_get_adapter_update_status());
}
static DEVICE_ATTR_RO(adapter_fw_update);

static ssize_t batt_cb_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", oplus_gauge_get_battery_cb_status());
}
static DEVICE_ATTR_RO(batt_cb_status);

int __attribute__((weak)) oplus_get_chg_i2c_err(void)
{
	return 0;
}

void __attribute__((weak)) oplus_clear_chg_i2c_err(void)
{
	return;
}

static ssize_t chg_i2c_err_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", oplus_get_chg_i2c_err());
}

static ssize_t chg_i2c_err_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	oplus_clear_chg_i2c_err();

	return count;
}
static DEVICE_ATTR_RW(chg_i2c_err);

static ssize_t usbtemp_volt_l_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

        return sprintf(buf, "%d\n", chip->usbtemp_volt_l);
}
static DEVICE_ATTR_RO(usbtemp_volt_l);

static ssize_t usbtemp_volt_r_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->usbtemp_volt_r);
}
static DEVICE_ATTR_RO(usbtemp_volt_r);

#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
static ssize_t ship_mode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->enable_shipmode);
}

static ssize_t ship_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}
	chip->enable_shipmode = val;
	oplus_gauge_update_soc_smooth_parameter();

	return count;
}
static DEVICE_ATTR_RW(ship_mode);
#endif /*CONFIG_OPLUS_SHIP_MODE_SUPPORT*/

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
static ssize_t short_c_limit_chg_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", (int)chip->short_c_batt.limit_chg);
}

static ssize_t short_c_limit_chg_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	printk(KERN_ERR "[OPLUS_CHG] [short_c_bat] set limit chg[%d]\n", !!val);
	chip->short_c_batt.limit_chg = !!val;
	//for userspace logic
	if (!!val == 0) {
		chip->short_c_batt.is_switch_on = 0;
	}

	return count;
}
static DEVICE_ATTR_RW(short_c_limit_chg);

static ssize_t short_c_limit_rechg_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", (int)chip->short_c_batt.limit_rechg);
}

static ssize_t short_c_limit_rechg_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	printk(KERN_ERR "[OPLUS_CHG] [short_c_bat] set limit rechg[%d]\n", !!val);
	chip->short_c_batt.limit_rechg = !!val;

	return count;
}
static DEVICE_ATTR_RW(short_c_limit_rechg);

static ssize_t charge_term_current_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->limits.iterm_ma);
}
static DEVICE_ATTR_RO(charge_term_current);

static ssize_t input_current_settled_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	val = 2000;
	if (chip && chip->chg_ops->get_dyna_aicl_result) {
		val = chip->chg_ops->get_dyna_aicl_result();
	}

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(input_current_settled);
#endif /*CONFIG_OPLUS_SHORT_USERSPACE*/
#endif /*CONFIG_OPLUS_SHORT_C_BATT_CHECK*/

#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
static ssize_t short_c_hw_feature_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->short_c_batt.is_feature_hw_on);
}

static ssize_t short_c_hw_feature_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	printk(KERN_ERR "[OPLUS_CHG] [short_c_hw_check]: set is_feature_hw_on [%d]\n", val);
	chip->short_c_batt.is_feature_hw_on = val;

	return count;
}
static DEVICE_ATTR_RW(short_c_hw_feature);

static ssize_t short_c_hw_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->short_c_batt.shortc_gpio_status);
}
static DEVICE_ATTR_RO(short_c_hw_status);
#endif /*CONFIG_OPLUS_SHORT_HW_CHECK*/

#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
static ssize_t short_ic_otp_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->short_c_batt.ic_short_otp_st);
}
static DEVICE_ATTR_RO(short_ic_otp_status);

static ssize_t short_ic_volt_thresh_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->short_c_batt.ic_volt_threshold);
}

static ssize_t short_ic_volt_thresh_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chip->short_c_batt.ic_volt_threshold = val;
	oplus_short_ic_set_volt_threshold(chip);

	return count;
}
static DEVICE_ATTR_RW(short_ic_volt_thresh);

static ssize_t short_ic_otp_value_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", oplus_short_ic_get_otp_error_value(chip));
}
static DEVICE_ATTR_RO(short_ic_otp_value);
#endif /*CONFIG_OPLUS_SHORT_IC_CHECK*/

static ssize_t voocchg_ing_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;
	union oplus_chg_mod_propval pval = {0, };

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	val = oplus_vooc_get_fastchg_ing();
	if (is_wls_ocm_available(chip) && !val) {
		oplus_chg_mod_get_property(chip->wls_ocm, OPLUS_CHG_PROP_FASTCHG_STATUS, &pval);
		val = pval.intval;
	}

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(voocchg_ing);

static ssize_t ppschg_ing_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	val = oplus_is_pps_charging();

	return sprintf(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(ppschg_ing);

static ssize_t bcc_exception_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	chg_err("%s\n", buf);
	oplus_chg_bcc_err(buf);

	return count;
}
static DEVICE_ATTR_WO(bcc_exception);

int __attribute__((weak)) oplus_gauge_get_bcc_parameters(char *buf)
{
	return 0;
}

int __attribute__((weak)) oplus_gauge_get_prev_bcc_parameters(char *buf)
{
	return 0;
}

int __attribute__((weak)) oplus_gauge_set_bcc_parameters(const char *buf)
{
	return 0;
}

static ssize_t bcc_parms_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int val = 0;
	ssize_t len = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (oplus_vooc_get_reply_bits() == 7 && oplus_vooc_get_fastchg_started()) {
		val = oplus_gauge_get_prev_bcc_parameters(buf);
	} else {
		val = oplus_gauge_get_bcc_parameters(buf);
	}
	len = strlen(buf);
	chg_err("len: %d\n", len);

	return len;
}

static ssize_t bcc_parms_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	ret = oplus_gauge_set_bcc_parameters(buf);
	if (ret < 0) {
		chg_err("error\n");
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(bcc_parms);

static ssize_t bcc_current_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->bcc_current);
}

static ssize_t bcc_current_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0, ret = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_battery_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	ret = oplus_smart_charge_by_bcc(chip, val);
	if (ret < 0) {
		chg_err("error\n");
		return -EINVAL;
	}

	mutex_lock(&chip->bcc_curr_done_mutex);
	chip->bcc_curr_done = BCC_CURR_DONE_REQUEST;
	chg_err("bcc_curr_done:%d\n", chip->bcc_curr_done);
	mutex_unlock(&chip->bcc_curr_done_mutex);

	if (oplus_chg_get_voocphy_support() != NO_VOOCPHY) {
		oplus_chg_check_bcc_curr_done();
	}

	return count;
}
static DEVICE_ATTR_RW(bcc_current);

static struct device_attribute *oplus_battery_attributes[] = {
	&dev_attr_authenticate,
	&dev_attr_battery_cc,
	&dev_attr_battery_fcc,
	&dev_attr_battery_rm,
	&dev_attr_battery_soh,
	&dev_attr_soh_report,
	&dev_attr_cc_report,
#ifdef CONFIG_OPLUS_CALL_MODE_SUPPORT
	&dev_attr_call_mode,
#endif
	&dev_attr_charge_technology,
#ifdef CONFIG_OPLUS_CHIP_SOC_NODE
	&dev_attr_chip_soc,
#endif
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
	&dev_attr_cool_down,
#endif
	&dev_attr_em_mode,
	&dev_attr_fast_charge,
	&dev_attr_mmi_charging_enable,
#ifdef CONFIG_OPLUS_CHARGER_MTK
	&dev_attr_stop_charging_enable,
	&dev_attr_water_detect_feature,
#endif
	&dev_attr_battery_notify_code,
	&dev_attr_sub_current,
	&dev_attr_charge_timeout,
	&dev_attr_adapter_fw_update,
	&dev_attr_batt_cb_status,
	&dev_attr_chg_i2c_err,
	&dev_attr_usbtemp_volt_l,
	&dev_attr_usbtemp_volt_r,
#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
	&dev_attr_ship_mode,
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
	&dev_attr_short_c_limit_chg,
	&dev_attr_short_c_limit_rechg,
	&dev_attr_charge_term_current,
	&dev_attr_input_current_settled,
#endif
#endif
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
	&dev_attr_short_c_hw_feature,
	&dev_attr_short_c_hw_status,
#endif
#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
	&dev_attr_short_ic_otp_status,
	&dev_attr_short_ic_volt_thresh,
	&dev_attr_short_ic_otp_value,
#endif
	&dev_attr_voocchg_ing,
	&dev_attr_ppschg_ing,
	&dev_attr_bcc_parms,
	&dev_attr_bcc_current,
	&dev_attr_bcc_exception,
	NULL
};


/**********************************************************************
* wireless device nodes
**********************************************************************/
static ssize_t tx_voltage_now_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(tx_voltage_now);

static ssize_t tx_current_now_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(tx_current_now);

static ssize_t cp_voltage_now_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(cp_voltage_now);

static ssize_t cp_current_now_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(cp_current_now);

static ssize_t wireless_mode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(wireless_mode);

static ssize_t wireless_type_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(wireless_type);

static ssize_t cep_info_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}
	return sprintf(buf, "%d\n", 0);
}
static DEVICE_ATTR_RO(cep_info);

int  __attribute__((weak)) oplus_wpc_get_real_type(void)
{
	return 0;
}
static ssize_t real_type_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int real_type = 0;
	struct oplus_chg_chip *chip = NULL;
	union oplus_chg_mod_propval pval = {0, };

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return 0;
	}

	real_type = oplus_wpc_get_real_type();

	if (is_wls_ocm_available(chip)) {
		oplus_chg_mod_get_property(chip->wls_ocm, OPLUS_CHG_PROP_REAL_TYPE, &pval);
		real_type = pval.intval;
	}

	return sprintf(buf, "%d\n", real_type);
}
static DEVICE_ATTR_RO(real_type);

ssize_t  __attribute__((weak)) oplus_chg_wls_upgrade_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}
ssize_t  __attribute__((weak)) oplus_chg_wls_upgrade_fw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static ssize_t upgrade_firmware_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (is_wls_ocm_available(chip))
		return oplus_chg_wls_upgrade_fw_show(&chip->wls_ocm->dev, attr, buf);
	return 0;
}

static ssize_t upgrade_firmware_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (is_wls_ocm_available(chip))
		count = oplus_chg_wls_upgrade_fw_store(&chip->wls_ocm->dev, attr, buf, count);

	return count;
}
static DEVICE_ATTR_RW(upgrade_firmware);

static ssize_t status_keep_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", chip->wls_status_keep);
}

static ssize_t status_keep_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (kstrtos32(buf, 0, &val)) {
		chg_err("buf error\n");
		return -EINVAL;
	}

	chg_err("set wls_status_keep=%d\n", val);
	if (val == WLS_SK_BY_HAL && chip->wls_status_keep == WLS_SK_NULL) {
		val = WLS_SK_NULL;
		chg_err("force to set wls_status_keep=%d\n", val);
	}
	WRITE_ONCE(chip->wls_status_keep, val);
	if (chip->wls_status_keep == 0)
		power_supply_changed(chip->batt_psy);

	return count;
}
static DEVICE_ATTR_RW(status_keep);

int  __attribute__((weak)) oplus_wpc_get_max_wireless_power(void)
{
	return 0;
}

int  __attribute__((weak)) oplus_chg_wls_get_max_wireless_power(struct device *dev)
{
	return 0;
}

static ssize_t max_w_power_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;
	int max_wls_power = 0;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	max_wls_power = oplus_wpc_get_max_wireless_power();

	if (is_wls_ocm_available(chip))
		max_wls_power = oplus_chg_wls_get_max_wireless_power(&chip->wls_ocm->dev);

	return sprintf(buf, "%d\n", max_wls_power);
}
static DEVICE_ATTR_RO(max_w_power);

static struct device_attribute *oplus_wireless_attributes[] = {
	&dev_attr_tx_voltage_now,
	&dev_attr_tx_current_now,
	&dev_attr_cp_voltage_now,
	&dev_attr_cp_current_now,
	&dev_attr_wireless_mode,
	&dev_attr_wireless_type,
	&dev_attr_cep_info,
	&dev_attr_real_type,
	&dev_attr_upgrade_firmware,
	&dev_attr_status_keep,
	&dev_attr_max_w_power,
	NULL
};


/**********************************************************************
* common device nodes
**********************************************************************/
/*#ifdef OPLUS_CHG_ADB_ROOT_ENABLE*/
ssize_t  __attribute__((weak)) oplus_chg_comm_charge_parameter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}
ssize_t  __attribute__((weak)) oplus_chg_comm_charge_parameter_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static ssize_t charge_parameter_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_common_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (is_comm_ocm_available(chip))
		return oplus_chg_comm_charge_parameter_show(&chip->comm_ocm->dev, attr, buf);
	return 0;
}

static ssize_t charge_parameter_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_common_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	if (is_comm_ocm_available(chip))
		count = oplus_chg_comm_charge_parameter_store(&chip->comm_ocm->dev, attr, buf, count);

	return count;
}
static DEVICE_ATTR_RW(charge_parameter);
/*#endif OPLUS_CHG_ADB_ROOT_ENABLE*/

static struct device_attribute *oplus_common_attributes[] = {
/*#ifdef OPLUS_CHG_ADB_ROOT_ENABLE*/
	&dev_attr_charge_parameter,
/*#endif*/
	NULL
};

/**********************************************************************
* ac/usb/battery/wireless/common directory nodes create
**********************************************************************/
static int oplus_ac_dir_create(struct oplus_chg_chip *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "ac");
	if (status < 0) {
		chg_err("alloc_chrdev_region ac fail!\n");
		return -ENOMEM;
	}
	oplus_ac_dir = device_create(oplus_chg_class, NULL, devt, NULL, "%s", "ac");
	oplus_ac_dir->devt = devt;
	dev_set_drvdata(oplus_ac_dir, chip);

	attrs = oplus_ac_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_ac_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_ac_dir->class, oplus_ac_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_ac_dir_destroy(void)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_ac_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_ac_dir, attr);
	device_destroy(oplus_ac_dir->class, oplus_ac_dir->devt);
	unregister_chrdev_region(oplus_ac_dir->devt, 1);
}

static int oplus_usb_dir_create(struct oplus_chg_chip *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "usb");
	if (status < 0) {
		chg_err("alloc_chrdev_region usb fail!\n");
		return -ENOMEM;
	}
	oplus_usb_dir = device_create(oplus_chg_class, NULL, devt, NULL, "%s", "usb");
	oplus_usb_dir->devt = devt;
	dev_set_drvdata(oplus_usb_dir, chip);

	attrs = oplus_usb_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_usb_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_usb_dir->class, oplus_usb_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_usb_dir_destroy(void)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_usb_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_usb_dir, attr);
	device_destroy(oplus_usb_dir->class, oplus_usb_dir->devt);
	unregister_chrdev_region(oplus_usb_dir->devt, 1);
}

static int oplus_battery_dir_create(struct oplus_chg_chip *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "battery");
	if (status < 0) {
		chg_err("alloc_chrdev_region battery fail!\n");
		return -ENOMEM;
	}

	oplus_battery_dir = device_create(oplus_chg_class, NULL,
			devt, NULL, "%s", "battery");
	oplus_battery_dir->devt = devt;
	dev_set_drvdata(oplus_battery_dir, chip);

	attrs = oplus_battery_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_battery_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_battery_dir->class, oplus_battery_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_battery_dir_destroy(void)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_battery_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_battery_dir, attr);
	device_destroy(oplus_battery_dir->class, oplus_battery_dir->devt);
	unregister_chrdev_region(oplus_battery_dir->devt, 1);
}

static int oplus_wireless_dir_create(struct oplus_chg_chip *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "wireless");
	if (status < 0) {
		chg_err("alloc_chrdev_region wireless fail!\n");
		return -ENOMEM;
	}

	oplus_wireless_dir = device_create(oplus_chg_class, NULL,
			devt, NULL, "%s", "wireless");
	oplus_wireless_dir->devt = devt;
	dev_set_drvdata(oplus_wireless_dir, chip);

	attrs = oplus_wireless_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_wireless_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_wireless_dir->class, oplus_wireless_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_wireless_dir_destroy(void)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_wireless_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_wireless_dir, attr);
	device_destroy(oplus_wireless_dir->class, oplus_wireless_dir->devt);
	unregister_chrdev_region(oplus_wireless_dir->devt, 1);
}

static int oplus_common_dir_create(struct oplus_chg_chip *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "common");
	if (status < 0) {
		chg_err("alloc_chrdev_region common fail!\n");
		return -ENOMEM;
	}

	oplus_common_dir = device_create(oplus_chg_class, NULL,
			devt, NULL, "%s", "common");
	oplus_common_dir->devt = devt;
	dev_set_drvdata(oplus_common_dir, chip);

	attrs = oplus_common_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_common_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_common_dir->class, oplus_common_dir->devt);
			return err;
		}
	}

	return 0;
}

static void oplus_common_dir_destroy(void)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_common_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_common_dir, attr);
	device_destroy(oplus_common_dir->class, oplus_common_dir->devt);
	unregister_chrdev_region(oplus_common_dir->devt, 1);
}


/**********************************************************************
* configfs init APIs
**********************************************************************/
int oplus_ac_node_add(struct device_attribute **ac_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = ac_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_ac_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_ac_dir->class, oplus_ac_dir->devt);
			return err;
		}
	}

	return 0;
}

void oplus_ac_node_delete(struct device_attribute **ac_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = ac_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_ac_dir, attr);
}

int oplus_usb_node_add(struct device_attribute **usb_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = usb_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_usb_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_usb_dir->class, oplus_usb_dir->devt);
			return err;
		}
	}

	return 0;
}

void oplus_usb_node_delete(struct device_attribute **usb_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = usb_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_usb_dir, attr);
}

int oplus_battery_node_add(struct device_attribute **battery_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = battery_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_battery_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_battery_dir->class, oplus_battery_dir->devt);
			return err;
		}
	}

	return 0;
}

void oplus_battery_node_delete(struct device_attribute **battery_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = battery_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_battery_dir, attr);
}

int oplus_wireless_node_add(struct device_attribute **wireless_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = wireless_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_wireless_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_wireless_dir->class, oplus_wireless_dir->devt);
			return err;
		}
	}

	return 0;
}

void oplus_wireless_node_delete(struct device_attribute **wireless_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = wireless_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_wireless_dir, attr);
}

int oplus_common_node_add(struct device_attribute **common_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = common_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_common_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_common_dir->class, oplus_common_dir->devt);
			return err;
		}
	}

	return 0;
}

void oplus_common_node_delete(struct device_attribute **common_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = common_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_common_dir, attr);
}

int oplus_chg_configfs_init(struct oplus_chg_chip *chip)
{
	int status = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	oplus_chg_class = class_create(THIS_MODULE, "oplus_chg");
	if (IS_ERR(oplus_chg_class)) {
		chg_err("oplus_chg_configfs_init fail!\n");
		return PTR_ERR(oplus_chg_class);
	}

	status = oplus_ac_dir_create(chip);
	if (status < 0)
		chg_err("oplus_ac_dir_create fail!\n");

	status = oplus_usb_dir_create(chip);
	if (status < 0)
		chg_err("oplus_usb_dir_create fail!\n");

	status = oplus_battery_dir_create(chip);
	if (status < 0)
		chg_err("oplus_battery_dir_create fail!\n");

	status = oplus_wireless_dir_create(chip);
	if (status < 0)
		chg_err("oplus_wireless_dir_create fail!\n");

	status = oplus_common_dir_create(chip);
	if (status < 0)
		chg_err("oplus_common_dir_create fail!\n");

	return 0;
}
EXPORT_SYMBOL(oplus_chg_configfs_init);

int oplus_chg_configfs_exit(void)
{
	oplus_common_dir_destroy();
	oplus_wireless_dir_destroy();
	oplus_battery_dir_destroy();
	oplus_usb_dir_destroy();
	oplus_ac_dir_destroy();

	if (!IS_ERR(oplus_chg_class))
		class_destroy(oplus_chg_class);
	return 0;
}
EXPORT_SYMBOL(oplus_chg_configfs_exit);

