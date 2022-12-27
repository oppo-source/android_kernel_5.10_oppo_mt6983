// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author Wy Chuang<wy.chuang@mediatek.com>
 */

#include <linux/cdev.h>		/* cdev */
#include <linux/err.h>	/* IS_ERR, PTR_ERR */
#include <linux/init.h>		/* For init/exit macros */
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>	/*irq_to_desc*/
#include <linux/kernel.h>
#include <linux/kthread.h>	/* For Kthread_run */
#include <linux/math64.h>
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/netlink.h>	/* netlink */
#include <linux/of_fdt.h>	/*of_dt API*/
#include <linux/of.h>
#include <linux/platform_device.h>	/* platform device */
#include <linux/proc_fs.h>
#include <linux/reboot.h>	/*kernel_power_off*/
#include <linux/sched.h>	/* For wait queue*/
#include <linux/skbuff.h>	/* netlink */
#include <linux/socket.h>	/* netlink */
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>		/* For wait queue*/
#include <net/sock.h>		/* netlink */
#include <linux/suspend.h>
#include "mtk_battery.h"
#include "mtk_battery_table.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#ifdef OPLUS_FEATURE_CHG_BASIC
#include "../oplus/oplus_gauge.h"
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
#define LOWEST_TEMP_FOR_NTC_DISCONNECT 10000000
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/iio/consumer.h>
extern void fg_int_event(struct mtk_battery *gm, enum gauge_event evt);
extern struct iio_channel *iio_channel_get(struct device *dev,const char *channel_name);
extern void oplus_gauge_init(struct oplus_gauge_chip *chip);
extern int oplus_chg_get_voocphy_support(void);
extern int oplus_force_get_subboard_temp(void);


struct mtk_battery *oplus_gm = NULL;
struct iio_channel	*batt_id = NULL;
int fuelgauge_apply = 0;
int batt_id_fast_chcek = 0;
int g_ntc_switch_not_use = 0;
static int g_switch_ntc = 0;
int is_subboard_temp_support = 0;
int enable_is_force_full;
bool last_full = false;

#define BAT_ATL_BATT_ID 2
#define BAT_XINWANG_BATT_ID 3
#define BAT_TYPE__XINWANG_4450mV_NTC_MIN 50
#define BAT_TYPE__XINWANG_4450mV_NTC_MAX 200
#define BAT_TYPE__ATL_4450mV_NTC_MIN 550
#define BAT_TYPE__ATL_4450mV_NTC_MAX 790

enum {
	BAT_TYPE__UNKNOWN,
	BAT_TYPE__SDI_4350mV, //50mV~290mV
	BAT_TYPE__SDI_4400mV, //300mV~520mV
	BAT_TYPE__LG_4350mV, //NO use
	BAT_TYPE__LG_4400mV, //530mV~780mV
	BAT_TYPE__ATL_4350mV, //1110mV~1450mV
	BAT_TYPE__ATL_4400mV, //790mV~1100mV
	BAT_TYPE__TWS_4400mV,
	BAT_TYPE__ATL_4450mV, //550mV~790mV
	BAT_TYPE__XWD_4450mV, //50mv~200mv

};

bool is_fuelgauge_apply(void)
{
	return fuelgauge_apply;
}
//EXPORT_SYMBOL(is_fuelgauge_apply);

bool is_batt_id_check(void)
{
	return batt_id_fast_chcek;
}

bool prj_is_subboard_temp_support(void)
{
	return is_subboard_temp_support;
}

int battery_type_check(int *battery_type)
{
	int value = 0;
	int ret = 0;
	int ret_value = 0;
	int battery_id = 0;

	if (batt_id == NULL) {
		bm_debug("[battery_type_check]: batt_id is null [%d]\n");
		*battery_type = BAT_TYPE__ATL_4400mV;
		battery_id = 0;
		return battery_id;
	}
	
	if (is_batt_id_check()) {
		if (gpio_is_valid(g_switch_ntc)) {
			gpio_direction_output(g_switch_ntc, 1);
			gpio_set_value(g_switch_ntc, 0);
			msleep(10);
		}
		ret = iio_read_channel_processed(batt_id, &ret_value);
		if (ret < 0) {
			bm_debug( "[battery_type_check] read channel err = %d,\n", ret);
		}
		if (!g_ntc_switch_not_use)
			gpio_set_value(g_switch_ntc, 1);

		bm_debug( "[battery_type_check] g_switch_ntc = %d,ret = %d,ret_value[%d]\n", gpio_get_value(g_switch_ntc),ret, ret_value);
		value = ret_value;

		if(is_fuelgauge_apply() == true){
			if (value >= BAT_TYPE__XINWANG_4450mV_NTC_MIN && value <= BAT_TYPE__XINWANG_4450mV_NTC_MAX) {
				*battery_type = BAT_TYPE__XWD_4450mV;
				battery_id = BAT_XINWANG_BATT_ID;
			} else if (value >= BAT_TYPE__ATL_4450mV_NTC_MIN && value < BAT_TYPE__ATL_4450mV_NTC_MAX) {
				*battery_type = BAT_TYPE__ATL_4450mV;
				battery_id = BAT_ATL_BATT_ID;
			} else {
				*battery_type = BAT_TYPE__UNKNOWN;
				battery_id = 0;
			}
		} else {
			*battery_type = BAT_TYPE__UNKNOWN;
			battery_id = 0;
		}
	} else {
		ret = iio_read_channel_processed(batt_id, &ret_value);
		if (ret < 0) {
			bm_debug( "[battery_type_check] read channel err = %d,\n", ret);
		}
		bm_debug( "[battery_type_check]: ret = %d,ret_value[%d]\n", ret, ret_value);
		value = ret_value;
		bm_debug("[battery_value= %d\n", value);

		if(is_fuelgauge_apply() == true){
			if (value >= 790 && value <= 1100) {
				*battery_type = BAT_TYPE__ATL_4400mV;
				battery_id = 0;
			} else if (value >= 300 && value <= 520) {
				*battery_type = BAT_TYPE__SDI_4400mV;
				battery_id = 1;
			} else if (value >= 550 && value < 790) {
				*battery_type = BAT_TYPE__ATL_4450mV;
				battery_id = 2;
			} else {
				*battery_type = BAT_TYPE__UNKNOWN;
				battery_id = 0;
			}
		} else {
			*battery_type = BAT_TYPE__UNKNOWN;
			battery_id = 0;
		}
	}

	printk(KERN_ERR "[battery_type_check]: adc_value[%d], battery_type[%d], g_fg_battery_id[%d]\n", value, *battery_type, battery_id);

	return battery_id;
}

#endif

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

int __attribute__ ((weak))
	mtk_battery_daemon_init(struct platform_device *pdev)
{
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;

	gauge = dev_get_drvdata(&pdev->dev);
	gm = gauge->gm;

	gm->algo.active = true;
	bm_err("[%s]: weak function,kernel algo=%d\n", __func__,
		gm->algo.active);
	return -EIO;
}

int __attribute__ ((weak))
	wakeup_fg_daemon(unsigned int flow_state, int cmd, int para1)
{
	return 0;
}

void __attribute__ ((weak))
	fg_sw_bat_cycle_accu(struct mtk_battery *gm)
{
}

void __attribute__ ((weak))
	notify_fg_chr_full(struct mtk_battery *gm)
{
}

void __attribute__ ((weak))
	fg_drv_update_daemon(struct mtk_battery *gm)
{
}

void enable_gauge_irq(struct mtk_gauge *gauge,
	enum gauge_irq irq)
{
	struct irq_desc *desc;

	if (irq >= GAUGE_IRQ_MAX)
		return;

	desc = irq_to_desc(gauge->irq_no[irq]);
	bm_debug("%s irq_no:%d:%d depth:%d\n",
		__func__, irq, gauge->irq_no[irq],
		desc->depth);
	if (desc->depth == 1)
		enable_irq(gauge->irq_no[irq]);
}

void disable_gauge_irq(struct mtk_gauge *gauge,
	enum gauge_irq irq)
{
	struct irq_desc *desc;

	if (irq >= GAUGE_IRQ_MAX)
		return;

	if (gauge->irq_no[irq] == 0)
		return;

	desc = irq_to_desc(gauge->irq_no[irq]);
	bm_debug("%s irq_no:%d:%d depth:%d\n",
		__func__, irq, gauge->irq_no[irq],
		desc->depth);
	if (desc->depth == 0)
		disable_irq_nosync(gauge->irq_no[irq]);
}

struct mtk_battery *get_mtk_battery(void)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;

	psy = power_supply_get_by_name("mtk-gauge");
	if (psy == NULL) {
		bm_err("[%s]psy is not rdy\n", __func__);
		return NULL;
	}

	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
	if (gauge == NULL) {
		bm_err("[%s]mtk_gauge is not rdy\n", __func__);
		return NULL;
	}
	return gauge->gm;
}

int bat_get_debug_level(void)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;
	static struct mtk_battery *gm;

	if (gm == NULL) {
		psy = power_supply_get_by_name("mtk-gauge");
		if (psy == NULL)
			return BMLOG_DEBUG_LEVEL;
		gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
		if (gauge == NULL || gauge->gm == NULL)
			return BMLOG_DEBUG_LEVEL;
		gm = gauge->gm;
	}
	return gm->log_level;
}

bool is_algo_active(struct mtk_battery *gm)
{
	return gm->algo.active;
}

#ifndef CONFIG_OPLUS_CHARGER_MTK6789S
int fgauge_get_profile_id(void)
{
	return 0;
}
#else
int fgauge_get_profile_id(void)
{
	int battery_id = 0;
	int battery_type = BAT_TYPE__UNKNOWN;

	battery_id = battery_type_check(&battery_type);

	return battery_id;
}
#endif


int wakeup_fg_algo_cmd(
	struct mtk_battery *gm, unsigned int flow_state, int cmd, int para1)
{

	bm_debug("[%s] 0x%x %d %d\n", __func__, flow_state, cmd, para1);
	if (gm->disableGM30) {
		bm_err("FG daemon is disabled\n");
		return -1;
	}
	if (is_algo_active(gm) == true)
		do_fg_algo(gm, flow_state);
	else
		wakeup_fg_daemon(flow_state, cmd, para1);

	return 0;
}

int wakeup_fg_algo(struct mtk_battery *gm, unsigned int flow_state)
{
	return wakeup_fg_algo_cmd(gm, flow_state, 0, 0);
}

bool is_recovery_mode(void)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_debug("%s, bootmdoe = %d\n", __func__, gm->bootmode);

	/* RECOVERY_BOOT */
	if (gm->bootmode == 2)
		return true;

	return false;
}

/* select gm->charge_power_sel to CHARGE_NORMAL ,CHARGE_R1,CHARGE_R2 */
/* example: gm->charge_power_sel = CHARGE_NORMAL */
bool set_charge_power_sel(enum charge_sel select)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	gm->charge_power_sel = select;

	wakeup_fg_algo_cmd(gm, FG_INTR_KERNEL_CMD,
		FG_KERNEL_CMD_FORCE_BAT_TEMP, select);

	return 0;
}

int dump_pseudo100(enum charge_sel select)
{
	int i = 0;
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_err("%s:select=%d\n", __func__, select);

	if (select > MAX_CHARGE_RDC || select < 0)
		return 0;

	for (i = 0; i < MAX_TABLE; i++) {
		bm_err("%6d\n",
			gm->fg_table_cust_data.fg_profile[
				i].r_pseudo100.pseudo[select]);
	}

	return 0;
}

bool is_kernel_power_off_charging(void)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_debug("%s, bootmdoe = %d\n", gm->bootmode);

	/* KERNEL_POWER_OFF_CHARGING_BOOT */
	if (gm->bootmode == 8)
		return true;

	return false;
}

/* ============================================================ */
/* power supply: battery */
/* ============================================================ */
int check_cap_level(int uisoc)
{
	if (uisoc >= 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (uisoc >= 80 && uisoc < 100)
		return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else if (uisoc >= 20 && uisoc < 80)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	else if (uisoc > 0 && uisoc < 20)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (uisoc == 0)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
}

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

static int battery_psy_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	int curr_now = 0, curr_avg = 0;
	struct mtk_battery *gm;
	struct battery_data *bs_data;

	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);
	if (gm == NULL)
		return -ENODEV;
	bs_data = &gm->bs_data;

	if (gm->algo.active == true)
		bs_data->bat_capacity = gm->ui_soc;

	/* gauge_get_property should check return value */
	/* to avoid i2c suspend but query by other module */

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bs_data->bat_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bs_data->bat_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:

		ret = gauge_get_property(GAUGE_PROP_BATTERY_EXIST,
			&bs_data->bat_present);

		if (ret == -EHOSTDOWN)
			val->intval = gm->present;
		else {
			val->intval = bs_data->bat_present;
			gm->present = bs_data->bat_present;
		}
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = bs_data->bat_technology;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* 1 = META_BOOT, 4 = FACTORY_BOOT 5=ADVMETA_BOOT */
		/* 6= ATE_factory_boot */
		if (gm->bootmode == 1 || gm->bootmode == 4
			|| gm->bootmode == 5 || gm->bootmode == 6) {
			val->intval = 75;
			break;
		}
		if (gm->fixed_uisoc != 0xffff)
			val->intval = gm->fixed_uisoc;
		else
			val->intval = bs_data->bat_capacity;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = gauge_get_property(GAUGE_PROP_BATTERY_CURRENT,
			&curr_now);

		if (ret == -EHOSTDOWN)
			val->intval = gm->ibat * 100;
		else {
			val->intval = curr_now * 100;
			gm->ibat = curr_now;
		}

		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = gauge_get_property(GAUGE_PROP_AVERAGE_CURRENT,
			&curr_avg);

		if (ret == -EHOSTDOWN)
			val->intval = gm->ibat * 100;
		else
			val->intval = curr_avg * 100;

		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval =
			gm->fg_table_cust_data.fg_profile[
				gm->battery_id].q_max * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = gm->ui_soc *
			gm->fg_table_cust_data.fg_profile[
				gm->battery_id].q_max * 1000 / 100;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* 1 = META_BOOT, 4 = FACTORY_BOOT 5=ADVMETA_BOOT */
		/* 6= ATE_factory_boot */
		if (gm->bootmode == 1 || gm->bootmode == 4
			|| gm->bootmode == 5 || gm->bootmode == 6) {
			val->intval = 4000000;
			break;
		}

		if (gm->disableGM30)
			bs_data->bat_batt_vol = 4000;
		else
			ret = gauge_get_property(GAUGE_PROP_BATTERY_VOLTAGE,
				&bs_data->bat_batt_vol);

		if (ret == -EHOSTDOWN)
			val->intval = gm->vbat;
		else {
			gm->vbat = bs_data->bat_batt_vol;
			val->intval = bs_data->bat_batt_vol * 1000;
		}
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = force_get_tbat(gm, true) * 10;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = check_cap_level(bs_data->bat_capacity);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		/* full or unknown must return 0 */
		ret = check_cap_level(bs_data->bat_capacity);
		if ((ret == POWER_SUPPLY_CAPACITY_LEVEL_FULL) ||
			(ret == POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN))
			val->intval = 0;
		else {
			int q_max_now = gm->fg_table_cust_data.fg_profile[
						gm->battery_id].q_max;
			int remain_ui = 100 - bs_data->bat_capacity;
			int remain_mah = remain_ui * q_max_now / 10;
			int current_now = 0;
			int time_to_full = 0;

			ret = gauge_get_property(GAUGE_PROP_BATTERY_CURRENT,
				&current_now);

			if (ret == -EHOSTDOWN)
				current_now = gm->ibat;

			if (current_now != 0)
				time_to_full = remain_mah * 3600 / current_now;

				bm_debug("time_to_full:%d, remain:ui:%d mah:%d, current_now:%d, qmax:%d\n",
					time_to_full, remain_ui, remain_mah,
					current_now, q_max_now);
			val->intval = abs(time_to_full);
		}
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (check_cap_level(bs_data->bat_capacity) ==
			POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN)
			val->intval = 0;
		else {
			int q_max_mah = 0;
			int q_max_uah = 0;

			q_max_mah =
				gm->fg_table_cust_data.fg_profile[
				gm->battery_id].q_max / 10;

			q_max_uah = q_max_mah * 1000;
			if (q_max_uah <= 100000) {
				bm_debug("%s q_max_mah:%d q_max_uah:%d\n",
					__func__, q_max_mah, q_max_uah);
				q_max_uah = 100001;
			}
			val->intval = q_max_uah;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		bs_data = &gm->bs_data;
		if (IS_ERR_OR_NULL(bs_data->chg_psy)) {
			bs_data->chg_psy = devm_power_supply_get_by_phandle(
				&gm->gauge->pdev->dev, "charger");
			bm_err("%s retry to get chg_psy\n", __func__);
		}
		if (IS_ERR_OR_NULL(bs_data->chg_psy)) {
			bm_err("%s Couldn't get chg_psy\n", __func__);
			ret = 4350;
		} else {
			ret = power_supply_get_property(bs_data->chg_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, val);
			if (ret < 0)
				bm_err("get CV property fail\n");
		}
		break;


	default:
		ret = -EINVAL;
		break;
		}

	bm_debug("%s psp:%d ret:%d val:%d",
		__func__, psp, ret, val->intval);

	return ret;
}

static int battery_psy_set_property(struct power_supply *psy,
	enum power_supply_property psp,
	const union power_supply_propval *val)
{
	int ret = 0;
	struct mtk_battery *gm;

	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);

	if (gm == NULL)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		if (val->intval > 0) {
			wakeup_fg_algo_cmd(gm, FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_GET_DYNAMIC_CV, (val->intval / 100));
			bm_err("[%s], dynamic_cv: %d\n",  __func__, val->intval);
		}
		break;
	default:
		ret = -EINVAL;
		break;
		}

	bm_debug("%s psp:%d ret:%d val:%d",
		__func__, psp, ret, val->intval);

	return ret;
}

static void mtk_battery_external_power_changed(struct power_supply *psy)
{
	struct mtk_battery *gm;
	struct battery_data *bs_data;
	union power_supply_propval online, status, vbat0;
	union power_supply_propval prop_type;
	int cur_chr_type = 0, old_vbat0 = 0;

	struct power_supply *chg_psy = NULL;
	struct power_supply *dv2_chg_psy = NULL;
	int ret;

	gm = psy->drv_data;
	bs_data = &gm->bs_data;
	chg_psy = bs_data->chg_psy;

	if (gm->is_probe_done == false) {
		bm_err("[%s]battery probe is not rdy:%d\n",
			__func__, gm->is_probe_done);
		return;
	}

	if (IS_ERR_OR_NULL(chg_psy)) {
		chg_psy = devm_power_supply_get_by_phandle(&gm->gauge->pdev->dev,
							   "charger");
		bm_err("%s retry to get chg_psy\n", __func__);
		bs_data->chg_psy = chg_psy;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &online);

		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_STATUS, &status);

		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ENERGY_EMPTY, &vbat0);

		if (!online.intval) {
			bs_data->bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			if (status.intval == POWER_SUPPLY_STATUS_NOT_CHARGING) {
				bs_data->bat_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;

				dv2_chg_psy = power_supply_get_by_name("mtk-mst-div-chg");
				if (!IS_ERR_OR_NULL(dv2_chg_psy)) {
					ret = power_supply_get_property(dv2_chg_psy,
						POWER_SUPPLY_PROP_ONLINE, &online);
					if (online.intval) {
						bs_data->bat_status =
							POWER_SUPPLY_STATUS_CHARGING;
						status.intval =
							POWER_SUPPLY_STATUS_CHARGING;
					}
				}
			} else {
				bs_data->bat_status =
					POWER_SUPPLY_STATUS_CHARGING;
			}

			fg_sw_bat_cycle_accu(gm);
		}

		if (status.intval == POWER_SUPPLY_STATUS_FULL
			&& gm->b_EOC != true) {
			bm_err("POWER_SUPPLY_STATUS_FULL, EOC\n");
			gauge_get_int_property(GAUGE_PROP_BAT_EOC);
			bm_err("GAUGE_PROP_BAT_EOC done\n");
			gm->b_EOC = true;
			notify_fg_chr_full(gm);
		} else
			gm->b_EOC = false;

		battery_update(gm);

		/* check charger type */
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop_type);

		/* plug in out */
		cur_chr_type = prop_type.intval;

		if (cur_chr_type == POWER_SUPPLY_TYPE_UNKNOWN) {
			if (gm->chr_type != POWER_SUPPLY_TYPE_UNKNOWN)
				bm_err("%s chr plug out\n");
		} else {
			if (gm->chr_type == POWER_SUPPLY_TYPE_UNKNOWN)
				wakeup_fg_algo(gm, FG_INTR_CHARGER_IN);
		}

		if (gm->vbat0_flag != vbat0.intval) {
			old_vbat0 = gm->vbat0_flag;
			gm->vbat0_flag = vbat0.intval;

			bm_err("fuelgauge NAFG for calibration,vbat0[o:%d n:%d]\n",
				old_vbat0, vbat0.intval);
			wakeup_fg_algo(gm, FG_INTR_NAG_C_DLTV);
		}
	}

	bm_err("%s event, name:%s online:%d, status:%d, EOC:%d, cur_chr_type:%d old:%d, vbat0:[o:%d n:%d]\n",
		__func__, psy->desc->name, online.intval, status.intval,
		gm->b_EOC, cur_chr_type, gm->chr_type,
		old_vbat0, vbat0.intval);

	gm->chr_type = cur_chr_type;

}
void battery_service_data_init(struct mtk_battery *gm)
{
	struct battery_data *bs_data;

	bs_data = &gm->bs_data;
#ifndef OPLUS_FEATURE_CHG_BASIC
	bs_data->psd.name = "battery",
#else
	bs_data->psd.name = "mtk-battery",
#endif
	bs_data->psd.type = POWER_SUPPLY_TYPE_BATTERY;
	bs_data->psd.properties = battery_props;
	bs_data->psd.num_properties = ARRAY_SIZE(battery_props);
	bs_data->psd.get_property = battery_psy_get_property;
	bs_data->psd.set_property = battery_psy_set_property;
	bs_data->psd.external_power_changed =
		mtk_battery_external_power_changed;
	bs_data->psy_cfg.drv_data = gm;

	bs_data->bat_status = POWER_SUPPLY_STATUS_DISCHARGING,
	bs_data->bat_health = POWER_SUPPLY_HEALTH_GOOD,
	bs_data->bat_present = 1,
	bs_data->bat_technology = POWER_SUPPLY_TECHNOLOGY_LION,
	bs_data->bat_capacity = -1,
	bs_data->bat_batt_vol = 0,
	bs_data->bat_batt_temp = 0,

	gm->fixed_uisoc = 0xffff;
}

/* ============================================================ */
/* voltage to battery temperature */
/* ============================================================ */
#ifdef OPLUS_FEATURE_CHG_BASIC
#define OPLUS_ADC_BATT_TBAT_VAL -2000
#endif
int adc_battemp(struct mtk_battery *gm, int res)
{
	int i = 0;
	int res1 = 0, res2 = 0;
#ifdef OPLUS_FEATURE_CHG_BASIC
	int tbatt_value = OPLUS_ADC_BATT_TBAT_VAL, tmp1 = 0, tmp2 = 0;
#else
	int tbatt_value = -200, tmp1 = 0, tmp2 = 0;
#endif
	struct fg_temp *ptable;

	ptable = gm->tmp_table;
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (res >= ptable[0].TemperatureR) {
		tbatt_value = BAT_NTC_TEMP_MIN;
	} else if (res <= ptable[BAT_NTC_10_TABLE_NUM - 1].TemperatureR) {
		tbatt_value = BAT_NTC_TEMP_MAX;
	} else {
		res1 = ptable[0].TemperatureR;
		tmp1 = ptable[0].BatteryTemp;

		for (i = 0; i <= (BAT_NTC_10_TABLE_NUM - 1); i++) {
			if (res >= ptable[i].TemperatureR) {
				res2 = ptable[i].TemperatureR;
				tmp2 = ptable[i].BatteryTemp;
				break;
			}
			{	/* hidden else */
				res1 = ptable[i].TemperatureR;
				tmp1 = ptable[i].BatteryTemp;
			}
		}

		tbatt_value = (((res - res2) * tmp1) +
			((res1 - res) * tmp2)) / (res1 - res2);
	}
#else
	if (res >= ptable[0].TemperatureR) {
		tbatt_value = -40;
	} else if (res <= ptable[28].TemperatureR) {
		tbatt_value = 100;
	} else {
		res1 = ptable[0].TemperatureR;
		tmp1 = ptable[0].BatteryTemp;

		for (i = 0; i <= 28; i++) {
			if (res >= ptable[i].TemperatureR) {
				res2 = ptable[i].TemperatureR;
				tmp2 = ptable[i].BatteryTemp;
				break;
			}
			{	/* hidden else */
				res1 = ptable[i].TemperatureR;
				tmp1 = ptable[i].BatteryTemp;
			}
		}

		tbatt_value = (((res - res2) * tmp1) +
			((res1 - res) * tmp2)) / (res1 - res2);
	}
#endif
	bm_debug("[%s] %d %d %d %d %d %d\n",
		__func__,
		res1, res2, res, tmp1,
		tmp2, tbatt_value);

	return tbatt_value;
}

int volttotemp(struct mtk_battery *gm, int dwVolt, int volt_cali)
{
	long long tres_temp;
	long long tres;
	int sbattmp = -100;
	int vbif28 = gm->rbat.rbat_pull_up_volt;
	int delta_v;
	int vbif28_raw;
	int ret;

	tres_temp = (gm->rbat.rbat_pull_up_r * (long long) dwVolt);
	ret = gauge_get_property(GAUGE_PROP_BIF_VOLTAGE,
		&vbif28_raw);

	if (ret != -ENOTSUPP) {
		vbif28 = vbif28_raw + volt_cali;
		delta_v = abs(vbif28 - dwVolt);
		if (delta_v == 0)
			delta_v = 1;
		tres_temp = div_s64(tres_temp, delta_v);
		if (vbif28 > 3000 || vbif28 < 1700)
			bm_debug("[RBAT_PULL_UP_VOLT_BY_BIF] vbif28:%d\n",
				vbif28_raw);
	} else {
		delta_v = abs(gm->rbat.rbat_pull_up_volt - dwVolt);
		if (delta_v == 0)
			delta_v = 1;
		tres_temp = div_s64(tres_temp, delta_v);
	}

#ifdef RBAT_PULL_DOWN_R
	tres = (tres_temp * RBAT_PULL_DOWN_R);
	tres = div_s64(tres, abs(RBAT_PULL_DOWN_R - tres_temp));
#ifdef OPLUS_FEATURE_CHG_BASIC
	if(abs(RBAT_PULL_DOWN_R - tres_temp) == 0)
		tres = LOWEST_TEMP_FOR_NTC_DISCONNECT;
#endif

#else
	tres = tres_temp;
#endif
	sbattmp = adc_battemp(gm, (int)tres);

	bm_debug("[%s] %d %d %d %d\n",
		__func__,
		dwVolt, gm->rbat.rbat_pull_up_r,
		vbif28, volt_cali);
	return sbattmp;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
#define OPLUS_BATON_NTC_01 10
#endif
int force_get_tbat_internal(struct mtk_battery *gm, bool update)
{
	int bat_temperature_volt = 2;
	int bat_temperature_val = 0;
	static int pre_bat_temperature_val = -1;
	int fg_r_value = 0;
	int fg_meter_res_value = 0;
	int fg_current_temp = 0;
	bool fg_current_state = false;
	int bat_temperature_volt_temp = 0;
	int vol_cali = 0;
	static int pre_bat_temperature_volt_temp, pre_bat_temperature_volt;
	static int pre_fg_current_temp;
	static int pre_fg_current_state;
	static int pre_fg_r_value;
	static int pre_bat_temperature_val2;
	ktime_t ctime = 0, dtime = 0, pre_time = 0;
	struct timespec64 tmp_time;
	int ret = 0;

//	return 25;

	if (update == true || pre_bat_temperature_val == -1) {
		/* Get V_BAT_Temperature */
		ret = gauge_get_property(GAUGE_PROP_BATTERY_TEMPERATURE_ADC,
			&bat_temperature_volt);

		if (ret == -EHOSTDOWN)
			return ret;

		gm->baton = bat_temperature_volt;

		if (bat_temperature_volt != 0) {
			fg_r_value = gm->fg_cust_data.com_r_fg_value;
			if (gm->no_bat_temp_compensate == 0)
				fg_meter_res_value =
				gm->fg_cust_data.com_fg_meter_resistance;
			else
				fg_meter_res_value = 0;

			gauge_get_property(GAUGE_PROP_BATTERY_CURRENT,
				&fg_current_temp);

			gm->ibat = fg_current_temp;

			if (fg_current_temp > 0)
				fg_current_state = true;

			fg_current_temp = abs(fg_current_temp) / 10;

			if (fg_current_state == true) {
				bat_temperature_volt_temp =
					bat_temperature_volt;
				bat_temperature_volt =
				bat_temperature_volt -
				((fg_current_temp *
					(fg_meter_res_value + fg_r_value))
						/ 10000);
				vol_cali =
					-((fg_current_temp *
					(fg_meter_res_value + fg_r_value))
						/ 10000);
			} else {
				bat_temperature_volt_temp =
					bat_temperature_volt;
				bat_temperature_volt =
				bat_temperature_volt +
				((fg_current_temp *
				(fg_meter_res_value + fg_r_value)) / 10000);
				vol_cali =
					((fg_current_temp *
					(fg_meter_res_value + fg_r_value))
					/ 10000);
			}

			bat_temperature_val =
				volttotemp(gm,
				bat_temperature_volt,
				vol_cali);
#ifdef OPLUS_FEATURE_CHG_BASIC
			gm->tbat_precise = bat_temperature_val;
			bat_temperature_val /= OPLUS_BATON_NTC_01;
#endif
		}

		bm_notice("[%s] %d,%d,%d,%d,%d,%d r:%d %d %d\n",
			__func__,
			bat_temperature_volt_temp, bat_temperature_volt,
			fg_current_state, fg_current_temp,
			fg_r_value, bat_temperature_val,
			fg_meter_res_value, fg_r_value,
			gm->no_bat_temp_compensate);

		if (pre_bat_temperature_val2 == 0) {
			pre_bat_temperature_volt_temp =
				bat_temperature_volt_temp;
			pre_bat_temperature_volt = bat_temperature_volt;
			pre_fg_current_temp = fg_current_temp;
			pre_fg_current_state = fg_current_state;
			pre_fg_r_value = fg_r_value;
			pre_bat_temperature_val2 = bat_temperature_val;
			pre_time = ktime_get_boottime();
		} else {
			ctime = ktime_get_boottime();
			dtime = ktime_sub(ctime, pre_time);
			tmp_time = ktime_to_timespec64(dtime);

			if ((tmp_time.tv_sec <= 20) &&
				(abs(pre_bat_temperature_val2 -
				bat_temperature_val) >= 5)) {
				bm_err("[%s][err] current:%d,%d,%d,%d,%d,%d pre:%d,%d,%d,%d,%d,%d\n",
					__func__,
					bat_temperature_volt_temp,
					bat_temperature_volt,
					fg_current_state,
					fg_current_temp,
					fg_r_value,
					bat_temperature_val,
					pre_bat_temperature_volt_temp,
					pre_bat_temperature_volt,
					pre_fg_current_state,
					pre_fg_current_temp,
					pre_fg_r_value,
					pre_bat_temperature_val2);
				/*pmic_auxadc_debug(1);*/
				WARN_ON(1);
			}

			pre_bat_temperature_volt_temp =
				bat_temperature_volt_temp;
			pre_bat_temperature_volt = bat_temperature_volt;
			pre_fg_current_temp = fg_current_temp;
			pre_fg_current_state = fg_current_state;
			pre_fg_r_value = fg_r_value;
			pre_bat_temperature_val2 = bat_temperature_val;
			pre_time = ctime;

			tmp_time = ktime_to_timespec64(dtime);

			bm_trace("[%s] current:%d,%d,%d,%d,%d,%d pre:%d,%d,%d,%d,%d,%d time:%d\n",
				__func__,
				bat_temperature_volt_temp, bat_temperature_volt,
				fg_current_state, fg_current_temp,
				fg_r_value, bat_temperature_val,
				pre_bat_temperature_volt_temp,
				pre_bat_temperature_volt,
				pre_fg_current_state, pre_fg_current_temp,
				pre_fg_r_value,
				pre_bat_temperature_val2, tmp_time.tv_sec);
		}
	} else {
		bat_temperature_val = pre_bat_temperature_val;
	}

	return bat_temperature_val;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
#define OPLUS_TEMP_25 250
#endif
int force_get_tbat(struct mtk_battery *gm, bool update)
{
	int bat_temperature_val = 0;

#ifdef OPLUS_FEATURE_CHG_BASIC
	int borad_temp = 0;

	if (prj_is_subboard_temp_support()){
		if (oplus_chg_get_voocphy_support()) {
			borad_temp = oplus_force_get_subboard_temp()/10;
		}
		return borad_temp;
	}
#endif

	if (gm->is_probe_done == false) {
#ifdef OPLUS_FEATURE_CHG_BASIC
		gm->tbat_precise = OPLUS_TEMP_25;
#endif
		gm->cur_bat_temp = 25;
		return 25;
	}

	if (gm->fixed_bat_tmp != 0xffff) {
		gm->cur_bat_temp = gm->fixed_bat_tmp;
#ifdef OPLUS_FEATURE_CHG_BASIC
		gm->tbat_precise = gm->fixed_bat_tmp * OPLUS_BATON_NTC_01;
#endif
		return gm->fixed_bat_tmp;
	}

	bat_temperature_val = force_get_tbat_internal(gm, true);

	if (bat_temperature_val == -EHOSTDOWN) {
#ifdef OPLUS_FEATURE_CHG_BASIC
		gm->tbat_precise = gm->cur_bat_temp * OPLUS_BATON_NTC_01;
#endif
		return gm->cur_bat_temp;
	}

	gm->cur_bat_temp = bat_temperature_val;

	return bat_temperature_val;
}

/* ============================================================ */
/* gaugel hal interface */
/* ============================================================ */
int gauge_get_property(enum gauge_property gp,
	int *val)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;
	struct mtk_gauge_sysfs_field_info *attr;
	static struct mtk_battery *gm;
	int ret = 0;

	psy = power_supply_get_by_name("mtk-gauge");
	if (psy == NULL)
		return -ENODEV;

	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
	gm = gauge->gm;
	if (gm != NULL && gm->disableGM30) {
		bm_err("%s disable GM30", __func__);
		return -EOPNOTSUPP;
	}
	bm_debug("%s enable GM30", __func__);

	attr = gauge->attr;
	if (attr == NULL) {
		bm_err("%s attr =NULL\n", __func__);
		return -ENODEV;
	}
	if (attr[gp].prop == gp) {
		mutex_lock(&gauge->ops_lock);
		ret = attr[gp].get(gauge, &attr[gp], val);

		mutex_unlock(&gauge->ops_lock);
	} else {
		bm_err("%s gp:%d idx error\n", __func__, gp);
		return -ENOTSUPP;
	}

	return ret;
}

int gauge_get_int_property(enum gauge_property gp)
{
	int val;

	gauge_get_property(gp, &val);
	return val;
}

int gauge_set_property(enum gauge_property gp,
	int val)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;
	struct mtk_gauge_sysfs_field_info *attr;

	psy = power_supply_get_by_name("mtk-gauge");
	if (psy == NULL)
		return -ENODEV;

	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
	attr = gauge->attr;

	if (attr == NULL) {
		bm_err("%s attr =NULL\n", __func__);
		return -ENODEV;
	}
	if (attr[gp].prop == gp) {
		mutex_lock(&gauge->ops_lock);
		attr[gp].set(gauge, &attr[gp], val);
		mutex_unlock(&gauge->ops_lock);
	} else {
		bm_err("%s gp:%d idx error\n", __func__, gp);
		return -ENOTSUPP;
	}

	return 0;
}

/* ============================================================ */
/* load .h/dtsi */
/* ============================================================ */

void fg_custom_init_from_header(struct mtk_battery *gm)
{
	int i, j;
	struct fuel_gauge_custom_data *fg_cust_data;
	struct fuel_gauge_table_custom_data *fg_table_cust_data;
	int version = 0;

	fg_cust_data = &gm->fg_cust_data;
	fg_table_cust_data = &gm->fg_table_cust_data;

	fgauge_get_profile_id();

	fg_cust_data->versionID1 = FG_DAEMON_CMD_FROM_USER_NUMBER;
	fg_cust_data->versionID2 = sizeof(gm->fg_cust_data);
	fg_cust_data->versionID3 = FG_KERNEL_CMD_FROM_USER_NUMBER;

	if (gm->gauge != NULL) {
		gauge_get_property(GAUGE_PROP_HW_VERSION, &version);
		fg_cust_data->hardwareVersion = version;
		fg_cust_data->pl_charger_status =
			gm->gauge->hw_status.pl_charger_status;
	}

	fg_cust_data->q_max_L_current = Q_MAX_L_CURRENT;
	fg_cust_data->q_max_H_current = Q_MAX_H_CURRENT;
	fg_cust_data->q_max_sys_voltage =
		UNIT_TRANS_10 * g_Q_MAX_SYS_VOLTAGE[gm->battery_id];

	fg_cust_data->pseudo1_en = PSEUDO1_EN;
	fg_cust_data->pseudo100_en = PSEUDO100_EN;
	fg_cust_data->pseudo100_en_dis = PSEUDO100_EN_DIS;
	fg_cust_data->pseudo1_iq_offset = UNIT_TRANS_100 *
		g_FG_PSEUDO1_OFFSET[gm->battery_id];

	/* iboot related */
	fg_cust_data->qmax_sel = QMAX_SEL;
	fg_cust_data->iboot_sel = IBOOT_SEL;
	fg_cust_data->shutdown_system_iboot = SHUTDOWN_SYSTEM_IBOOT;

	/* multi-temp gague 0% related */
	fg_cust_data->multi_temp_gauge0 = MULTI_TEMP_GAUGE0;

	/*hw related */
	fg_cust_data->car_tune_value = UNIT_TRANS_10 * CAR_TUNE_VALUE;
	fg_cust_data->fg_meter_resistance = FG_METER_RESISTANCE;
	fg_cust_data->com_fg_meter_resistance = FG_METER_RESISTANCE;
	fg_cust_data->r_fg_value = UNIT_TRANS_10 * R_FG_VALUE;
	fg_cust_data->com_r_fg_value = UNIT_TRANS_10 * R_FG_VALUE;
	fg_cust_data->unit_multiple = UNIT_MULTIPLE;

	/* Dynamic CV */
	fg_cust_data->dynamic_cv_factor = DYNAMIC_CV_FACTOR;
	fg_cust_data->charger_ieoc = CHARGER_IEOC;

	/* Aging Compensation */
	fg_cust_data->aging_one_en = AGING_ONE_EN;
	fg_cust_data->aging1_update_soc = UNIT_TRANS_100 * AGING1_UPDATE_SOC;
	fg_cust_data->aging1_load_soc = UNIT_TRANS_100 * AGING1_LOAD_SOC;
	fg_cust_data->aging4_update_soc = UNIT_TRANS_100 * AGING4_UPDATE_SOC;
	fg_cust_data->aging4_load_soc = UNIT_TRANS_100 * AGING4_LOAD_SOC;
	fg_cust_data->aging5_update_soc = UNIT_TRANS_100 * AGING5_UPDATE_SOC;
	fg_cust_data->aging5_load_soc = UNIT_TRANS_100 * AGING5_LOAD_SOC;
	fg_cust_data->aging6_update_soc = UNIT_TRANS_100 * AGING6_UPDATE_SOC;
	fg_cust_data->aging6_load_soc = UNIT_TRANS_100 * AGING6_LOAD_SOC;
	fg_cust_data->aging_temp_diff = AGING_TEMP_DIFF;
	fg_cust_data->aging_temp_low_limit = AGING_TEMP_LOW_LIMIT;
	fg_cust_data->aging_temp_high_limit = AGING_TEMP_HIGH_LIMIT;
	fg_cust_data->aging_100_en = AGING_100_EN;
	fg_cust_data->difference_voltage_update = DIFFERENCE_VOLTAGE_UPDATE;
	fg_cust_data->aging_factor_min = UNIT_TRANS_100 * AGING_FACTOR_MIN;
	fg_cust_data->aging_factor_diff = UNIT_TRANS_100 * AGING_FACTOR_DIFF;
	/* Aging Compensation 2*/
	fg_cust_data->aging_two_en = AGING_TWO_EN;
	/* Aging Compensation 3*/
	fg_cust_data->aging_third_en = AGING_THIRD_EN;
	fg_cust_data->aging_4_en = AGING_4_EN;
	fg_cust_data->aging_5_en = AGING_5_EN;
	fg_cust_data->aging_6_en = AGING_6_EN;

	/* ui_soc related */
	fg_cust_data->diff_soc_setting = DIFF_SOC_SETTING;
	fg_cust_data->keep_100_percent = UNIT_TRANS_100 * KEEP_100_PERCENT;
	fg_cust_data->difference_full_cv = DIFFERENCE_FULL_CV;
	fg_cust_data->diff_bat_temp_setting = DIFF_BAT_TEMP_SETTING;
	fg_cust_data->diff_bat_temp_setting_c = DIFF_BAT_TEMP_SETTING_C;
	fg_cust_data->discharge_tracking_time = DISCHARGE_TRACKING_TIME;
	fg_cust_data->charge_tracking_time = CHARGE_TRACKING_TIME;
	fg_cust_data->difference_fullocv_vth = DIFFERENCE_FULLOCV_VTH;
	fg_cust_data->difference_fullocv_ith =
		UNIT_TRANS_10 * DIFFERENCE_FULLOCV_ITH;
	fg_cust_data->charge_pseudo_full_level = CHARGE_PSEUDO_FULL_LEVEL;
	fg_cust_data->over_discharge_level = OVER_DISCHARGE_LEVEL;
	fg_cust_data->full_tracking_bat_int2_multiply =
		FULL_TRACKING_BAT_INT2_MULTIPLY;

	/* pre tracking */
	fg_cust_data->fg_pre_tracking_en = FG_PRE_TRACKING_EN;
	fg_cust_data->vbat2_det_time = VBAT2_DET_TIME;
	fg_cust_data->vbat2_det_counter = VBAT2_DET_COUNTER;
	fg_cust_data->vbat2_det_voltage1 = VBAT2_DET_VOLTAGE1;
	fg_cust_data->vbat2_det_voltage2 = VBAT2_DET_VOLTAGE2;
	fg_cust_data->vbat2_det_voltage3 = VBAT2_DET_VOLTAGE3;

	/* sw fg */
	fg_cust_data->difference_fgc_fgv_th1 = DIFFERENCE_FGC_FGV_TH1;
	fg_cust_data->difference_fgc_fgv_th2 = DIFFERENCE_FGC_FGV_TH2;
	fg_cust_data->difference_fgc_fgv_th3 = DIFFERENCE_FGC_FGV_TH3;
	fg_cust_data->difference_fgc_fgv_th_soc1 = DIFFERENCE_FGC_FGV_TH_SOC1;
	fg_cust_data->difference_fgc_fgv_th_soc2 = DIFFERENCE_FGC_FGV_TH_SOC2;
	fg_cust_data->nafg_time_setting = NAFG_TIME_SETTING;
	fg_cust_data->nafg_ratio = NAFG_RATIO;
	fg_cust_data->nafg_ratio_en = NAFG_RATIO_EN;
	fg_cust_data->nafg_ratio_tmp_thr = NAFG_RATIO_TMP_THR;
	fg_cust_data->nafg_resistance = NAFG_RESISTANCE;

	/* ADC resistor  */
	fg_cust_data->r_charger_1 = R_CHARGER_1;
	fg_cust_data->r_charger_2 = R_CHARGER_2;

	/* mode select */
	fg_cust_data->pmic_shutdown_current = PMIC_SHUTDOWN_CURRENT;
	fg_cust_data->pmic_shutdown_sw_en = PMIC_SHUTDOWN_SW_EN;
	fg_cust_data->force_vc_mode = FORCE_VC_MODE;
	fg_cust_data->embedded_sel = EMBEDDED_SEL;
	fg_cust_data->loading_1_en = LOADING_1_EN;
	fg_cust_data->loading_2_en = LOADING_2_EN;
	fg_cust_data->diff_iavg_th = DIFF_IAVG_TH;

	fg_cust_data->shutdown_gauge0 = SHUTDOWN_GAUGE0;
	fg_cust_data->shutdown_1_time = SHUTDOWN_1_TIME;
	fg_cust_data->shutdown_gauge1_xmins = SHUTDOWN_GAUGE1_XMINS;
	fg_cust_data->shutdown_gauge0_voltage = SHUTDOWN_GAUGE0_VOLTAGE;
	fg_cust_data->shutdown_gauge1_vbat_en = SHUTDOWN_GAUGE1_VBAT_EN;
	fg_cust_data->shutdown_gauge1_vbat = SHUTDOWN_GAUGE1_VBAT;
	fg_cust_data->power_on_car_chr = POWER_ON_CAR_CHR;
	fg_cust_data->power_on_car_nochr = POWER_ON_CAR_NOCHR;
	fg_cust_data->shutdown_car_ratio = SHUTDOWN_CAR_RATIO;

	/* log level*/
	fg_cust_data->daemon_log_level = BMLOG_TRACE_LEVEL;

	/* ZCV update */
	fg_cust_data->zcv_suspend_time = ZCV_SUSPEND_TIME;
	fg_cust_data->sleep_current_avg = SLEEP_CURRENT_AVG;
	fg_cust_data->zcv_com_vol_limit = ZCV_COM_VOL_LIMIT;
	fg_cust_data->zcv_car_gap_percentage = ZCV_CAR_GAP_PERCENTAGE;

	/* dod_init */
	fg_cust_data->hwocv_oldocv_diff = HWOCV_OLDOCV_DIFF;
	fg_cust_data->hwocv_oldocv_diff_chr = HWOCV_OLDOCV_DIFF_CHR;
	fg_cust_data->hwocv_swocv_diff = HWOCV_SWOCV_DIFF;
	fg_cust_data->hwocv_swocv_diff_lt = HWOCV_SWOCV_DIFF_LT;
	fg_cust_data->hwocv_swocv_diff_lt_temp = HWOCV_SWOCV_DIFF_LT_TEMP;
	fg_cust_data->swocv_oldocv_diff = SWOCV_OLDOCV_DIFF;
	fg_cust_data->swocv_oldocv_diff_chr = SWOCV_OLDOCV_DIFF_CHR;
	fg_cust_data->vbat_oldocv_diff = VBAT_OLDOCV_DIFF;
	fg_cust_data->swocv_oldocv_diff_emb = SWOCV_OLDOCV_DIFF_EMB;
	fg_cust_data->vir_oldocv_diff_emb = VIR_OLDOCV_DIFF_EMB;
	fg_cust_data->vir_oldocv_diff_emb_lt = VIR_OLDOCV_DIFF_EMB_LT;
	fg_cust_data->vir_oldocv_diff_emb_tmp = VIR_OLDOCV_DIFF_EMB_TMP;

	fg_cust_data->pmic_shutdown_time = UNIT_TRANS_60 * PMIC_SHUTDOWN_TIME;
	fg_cust_data->tnew_told_pon_diff = TNEW_TOLD_PON_DIFF;
	fg_cust_data->tnew_told_pon_diff2 = TNEW_TOLD_PON_DIFF2;
	gm->ext_hwocv_swocv = EXT_HWOCV_SWOCV;
	gm->ext_hwocv_swocv_lt = EXT_HWOCV_SWOCV_LT;
	gm->ext_hwocv_swocv_lt_temp = EXT_HWOCV_SWOCV_LT_TEMP;

	fg_cust_data->dc_ratio_sel = DC_RATIO_SEL;
	fg_cust_data->dc_r_cnt = DC_R_CNT;

	fg_cust_data->pseudo1_sel = PSEUDO1_SEL;

	fg_cust_data->d0_sel = D0_SEL;
	fg_cust_data->dlpt_ui_remap_en = DLPT_UI_REMAP_EN;

	fg_cust_data->aging_sel = AGING_SEL;
	fg_cust_data->bat_par_i = BAT_PAR_I;

	fg_cust_data->fg_tracking_current = FG_TRACKING_CURRENT;
	fg_cust_data->fg_tracking_current_iboot_en =
		FG_TRACKING_CURRENT_IBOOT_EN;
	fg_cust_data->ui_fast_tracking_en = UI_FAST_TRACKING_EN;
	fg_cust_data->ui_fast_tracking_gap = UI_FAST_TRACKING_GAP;

	fg_cust_data->bat_plug_out_time = BAT_PLUG_OUT_TIME;
	fg_cust_data->keep_100_percent_minsoc = KEEP_100_PERCENT_MINSOC;

	fg_cust_data->uisoc_update_type = UISOC_UPDATE_TYPE;

	fg_cust_data->battery_tmp_to_disable_gm30 = BATTERY_TMP_TO_DISABLE_GM30;
	fg_cust_data->battery_tmp_to_disable_nafg = BATTERY_TMP_TO_DISABLE_NAFG;
	fg_cust_data->battery_tmp_to_enable_nafg = BATTERY_TMP_TO_ENABLE_NAFG;

	fg_cust_data->low_temp_mode = LOW_TEMP_MODE;
	fg_cust_data->low_temp_mode_temp = LOW_TEMP_MODE_TEMP;

	/* current limit for uisoc 100% */
	fg_cust_data->ui_full_limit_en = UI_FULL_LIMIT_EN;
	fg_cust_data->ui_full_limit_soc0 = UI_FULL_LIMIT_SOC0;
	fg_cust_data->ui_full_limit_ith0 = UI_FULL_LIMIT_ITH0;
	fg_cust_data->ui_full_limit_soc1 = UI_FULL_LIMIT_SOC1;
	fg_cust_data->ui_full_limit_ith1 = UI_FULL_LIMIT_ITH1;
	fg_cust_data->ui_full_limit_soc2 = UI_FULL_LIMIT_SOC2;
	fg_cust_data->ui_full_limit_ith2 = UI_FULL_LIMIT_ITH2;
	fg_cust_data->ui_full_limit_soc3 = UI_FULL_LIMIT_SOC3;
	fg_cust_data->ui_full_limit_ith3 = UI_FULL_LIMIT_ITH3;
	fg_cust_data->ui_full_limit_soc4 = UI_FULL_LIMIT_SOC4;
	fg_cust_data->ui_full_limit_ith4 = UI_FULL_LIMIT_ITH4;
	fg_cust_data->ui_full_limit_time = UI_FULL_LIMIT_TIME;

	fg_cust_data->ui_full_limit_fc_soc0 = UI_FULL_LIMIT_FC_SOC0;
	fg_cust_data->ui_full_limit_fc_ith0 = UI_FULL_LIMIT_FC_ITH0;
	fg_cust_data->ui_full_limit_fc_soc1 = UI_FULL_LIMIT_FC_SOC1;
	fg_cust_data->ui_full_limit_fc_ith1 = UI_FULL_LIMIT_FC_ITH1;
	fg_cust_data->ui_full_limit_fc_soc2 = UI_FULL_LIMIT_FC_SOC2;
	fg_cust_data->ui_full_limit_fc_ith2 = UI_FULL_LIMIT_FC_ITH2;
	fg_cust_data->ui_full_limit_fc_soc3 = UI_FULL_LIMIT_FC_SOC3;
	fg_cust_data->ui_full_limit_fc_ith3 = UI_FULL_LIMIT_FC_ITH3;
	fg_cust_data->ui_full_limit_fc_soc4 = UI_FULL_LIMIT_FC_SOC4;
	fg_cust_data->ui_full_limit_fc_ith4 = UI_FULL_LIMIT_FC_ITH4;

	/* voltage limit for uisoc 1% */
	fg_cust_data->ui_low_limit_en = UI_LOW_LIMIT_EN;
	fg_cust_data->ui_low_limit_soc0 = UI_LOW_LIMIT_SOC0;
	fg_cust_data->ui_low_limit_vth0 = UI_LOW_LIMIT_VTH0;
	fg_cust_data->ui_low_limit_soc1 = UI_LOW_LIMIT_SOC1;
	fg_cust_data->ui_low_limit_vth1 = UI_LOW_LIMIT_VTH1;
	fg_cust_data->ui_low_limit_soc2 = UI_LOW_LIMIT_SOC2;
	fg_cust_data->ui_low_limit_vth2 = UI_LOW_LIMIT_VTH2;
	fg_cust_data->ui_low_limit_soc3 = UI_LOW_LIMIT_SOC3;
	fg_cust_data->ui_low_limit_vth3 = UI_LOW_LIMIT_VTH3;
	fg_cust_data->ui_low_limit_soc4 = UI_LOW_LIMIT_SOC4;
	fg_cust_data->ui_low_limit_vth4 = UI_LOW_LIMIT_VTH4;
	fg_cust_data->ui_low_limit_time = UI_LOW_LIMIT_TIME;

	fg_cust_data->moving_battemp_en = MOVING_BATTEMP_EN;
	fg_cust_data->moving_battemp_thr = MOVING_BATTEMP_THR;

	if (version == GAUGE_HW_V2001) {
		bm_debug("GAUGE_HW_V2001 disable nafg\n");
		fg_cust_data->disable_nafg = 1;
	}

	fg_table_cust_data->active_table_number = ACTIVE_TABLE;

	if (fg_table_cust_data->active_table_number == 0)
		fg_table_cust_data->active_table_number = 5;

	bm_debug("fg active table:%d\n",
		fg_table_cust_data->active_table_number);

	fg_table_cust_data->temperature_tb0 = TEMPERATURE_TB0;
	fg_table_cust_data->temperature_tb1 = TEMPERATURE_TB1;

	fg_table_cust_data->fg_profile[0].size =
		sizeof(fg_profile_t0[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[0].fg_profile,
			&fg_profile_t0[gm->battery_id],
			sizeof(fg_profile_t0[gm->battery_id]));

	fg_table_cust_data->fg_profile[1].size =
		sizeof(fg_profile_t1[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[1].fg_profile,
			&fg_profile_t1[gm->battery_id],
			sizeof(fg_profile_t1[gm->battery_id]));

	fg_table_cust_data->fg_profile[2].size =
		sizeof(fg_profile_t2[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[2].fg_profile,
			&fg_profile_t2[gm->battery_id],
			sizeof(fg_profile_t2[gm->battery_id]));

	fg_table_cust_data->fg_profile[3].size =
		sizeof(fg_profile_t3[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[3].fg_profile,
			&fg_profile_t3[gm->battery_id],
			sizeof(fg_profile_t3[gm->battery_id]));

	fg_table_cust_data->fg_profile[4].size =
		sizeof(fg_profile_t4[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[4].fg_profile,
			&fg_profile_t4[gm->battery_id],
			sizeof(fg_profile_t4[gm->battery_id]));

	fg_table_cust_data->fg_profile[5].size =
		sizeof(fg_profile_t5[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[5].fg_profile,
			&fg_profile_t5[gm->battery_id],
			sizeof(fg_profile_t5[gm->battery_id]));

	fg_table_cust_data->fg_profile[6].size =
		sizeof(fg_profile_t6[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[6].fg_profile,
			&fg_profile_t6[gm->battery_id],
			sizeof(fg_profile_t6[gm->battery_id]));

	fg_table_cust_data->fg_profile[7].size =
		sizeof(fg_profile_t7[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[7].fg_profile,
			&fg_profile_t7[gm->battery_id],
			sizeof(fg_profile_t7[gm->battery_id]));

	fg_table_cust_data->fg_profile[8].size =
		sizeof(fg_profile_t8[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[8].fg_profile,
			&fg_profile_t8[gm->battery_id],
			sizeof(fg_profile_t8[gm->battery_id]));

	fg_table_cust_data->fg_profile[9].size =
		sizeof(fg_profile_t9[gm->battery_id]) /
		sizeof(struct fuelgauge_profile_struct);

	memcpy(&fg_table_cust_data->fg_profile[9].fg_profile,
			&fg_profile_t9[gm->battery_id],
			sizeof(fg_profile_t9[gm->battery_id]));

	for (i = 0; i < MAX_TABLE; i++) {
		struct fuelgauge_profile_struct *p;

		p = &fg_table_cust_data->fg_profile[i].fg_profile[0];
		fg_table_cust_data->fg_profile[i].temperature =
			g_temperature[i];
		fg_table_cust_data->fg_profile[i].q_max =
			g_Q_MAX[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].q_max_h_current =
			g_Q_MAX_H_CURRENT[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].pseudo1 =
			UNIT_TRANS_100 * g_FG_PSEUDO1[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].pseudo100 =
			UNIT_TRANS_100 * g_FG_PSEUDO100[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].pmic_min_vol =
			g_PMIC_MIN_VOL[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].pon_iboot =
			g_PON_SYS_IBOOT[i][gm->battery_id];
		fg_table_cust_data->fg_profile[i].qmax_sys_vol =
			g_QMAX_SYS_VOL[i][gm->battery_id];
		/* shutdown_hl_zcv */
		fg_table_cust_data->fg_profile[i].shutdown_hl_zcv =
			g_SHUTDOWN_HL_ZCV[i][gm->battery_id];

		for (j = 0; j < 100; j++)
			if (p[j].charge_r.rdc[0] == 0)
				p[j].charge_r.rdc[0] = p[j].resistance;
	}

	/* init battery temperature table */
	gm->rbat.type = 10;
	gm->rbat.rbat_pull_up_r = RBAT_PULL_UP_R;
	gm->rbat.rbat_pull_up_volt = RBAT_PULL_UP_VOLT;
	gm->rbat.bif_ntc_r = BIF_NTC_R;

	if (IS_ENABLED(BAT_NTC_47)) {
		gm->rbat.type = 47;
		gm->rbat.rbat_pull_up_r = RBAT_PULL_UP_R;
	}
}

#if IS_ENABLED(CONFIG_OF)
static int fg_read_dts_val(const struct device_node *np,
		const char *node_srting,
		int *param, int unit)
{
	static unsigned int val;

	if (!of_property_read_u32(np, node_srting, &val)) {
		*param = (int)val * unit;
		bm_debug("Get %s: %d\n",
			 node_srting, *param);
	} else {
		bm_debug("Get %s no data\n", node_srting);
		return -1;
	}
	return 0;
}

static int fg_read_dts_val_by_idx(const struct device_node *np,
		const char *node_srting,
		int idx, int *param, int unit)
{
	unsigned int val;

	if (!of_property_read_u32_index(np, node_srting, idx, &val)) {
		*param = (int)val * unit;
		bm_debug("Get %s %d: %d\n",
			 node_srting, idx, *param);
	} else {
		bm_debug("Get %s no data, idx %d\n", node_srting, idx);
		return -1;
	}
	return 0;
}

static void fg_custom_parse_table(struct mtk_battery *gm,
		const struct device_node *np,
		const char *node_srting,
		struct fuelgauge_profile_struct *profile_struct, int column)
{
	int mah, voltage, resistance, idx, saddles;
	int i = 0, charge_rdc[MAX_CHARGE_RDC];
	struct fuelgauge_profile_struct *profile_p;

	profile_p = profile_struct;

	saddles = gm->fg_table_cust_data.fg_profile[0].size;
	idx = 0;

/*	bm_debug("%s: %s, %d, column:%d\n",
		__func__,
		node_srting, saddles, column);*/

	while (!of_property_read_u32_index(np, node_srting, idx, &mah)) {
		idx++;
		if (!of_property_read_u32_index(
			np, node_srting, idx, &voltage)) {
		}
		idx++;
		if (!of_property_read_u32_index(
				np, node_srting, idx, &resistance)) {
		}
		idx++;

		if (column == 3) {
			for (i = 0; i < MAX_CHARGE_RDC; i++)
				charge_rdc[i] = resistance;
		} else if (column >= 4) {
			if (!of_property_read_u32_index(
				np, node_srting, idx, &charge_rdc[0]))
				idx++;
		}

		/* read more for column >4 case */
		if (column > 4) {
			for (i = 1; i <= column - 4; i++) {
				if (!of_property_read_u32_index(
					np, node_srting, idx, &charge_rdc[i]))
					idx++;
			}
		}

/*		bm_debug("%s: mah: %d, voltage: %d, resistance: %d, rdc0:%d rdc:%d %d %d %d\n",
			__func__, mah, voltage, resistance, charge_rdc[0],
			charge_rdc[1], charge_rdc[2], charge_rdc[3],
			charge_rdc[4]);*/

		profile_p->mah = mah;
		profile_p->voltage = voltage;
		profile_p->resistance = resistance;

		for (i = 0; i < MAX_CHARGE_RDC; i++)
			profile_p->charge_r.rdc[i] = charge_rdc[i];

		profile_p++;

		if (idx >= (saddles * column))
			break;
	}

	if (idx == 0) {
		bm_err("[%s] cannot find %s in dts\n",
			__func__, node_srting);
		return;
	}

	profile_p--;

	while (idx < (100 * column)) {
		profile_p++;
		profile_p->mah = mah;
		profile_p->voltage = voltage;
		profile_p->resistance = resistance;

		for (i = 0; i < MAX_CHARGE_RDC; i++)
			profile_p->charge_r.rdc[i] = charge_rdc[i];

		idx = idx + column;
	}
}


void fg_custom_init_from_dts(struct platform_device *dev,
	struct mtk_battery *gm)
{
	struct device_node *np = dev->dev.of_node;
	unsigned int val;
	int bat_id, multi_battery, active_table, i, j, ret, column;
	int r_pseudo100_raw = 0, r_pseudo100_col = 0;
	int lk_v, lk_i, shuttime;
#ifdef OPLUS_FEATURE_CHG_BASIC
	int dim2_table_distinguish = 0;
#endif
	char node_name[128];
	struct fuel_gauge_custom_data *fg_cust_data;
	struct fuel_gauge_table_custom_data *fg_table_cust_data;

	gm->battery_id = fgauge_get_profile_id();
	bat_id = gm->battery_id;
	fg_cust_data = &gm->fg_cust_data;
	fg_table_cust_data = &gm->fg_table_cust_data;

	bm_err("%s\n", __func__);

	if (gm->ptim_lk_v == 0) {
		fg_read_dts_val(np, "fg_swocv_v", &(lk_v), 1);
		gm->ptim_lk_v = lk_v;
	}

	if (gm->ptim_lk_i == 0) {
		fg_read_dts_val(np, "fg_swocv_i", &(lk_i), 1);
		gm->ptim_lk_i = lk_i;
	}

	if (gm->pl_shutdown_time == 0) {
		fg_read_dts_val(np, "shutdown_time", &(shuttime), 1);
		gm->pl_shutdown_time = shuttime;
	}

	bm_err("%s swocv_v:%d swocv_i:%d shutdown_time:%d\n",
		__func__, gm->ptim_lk_v, gm->ptim_lk_i, gm->pl_shutdown_time);

	fg_cust_data->disable_nafg =
		of_property_read_bool(np, "DISABLE_NAFG");
	bm_err("disable_nafg:%d\n",
		fg_cust_data->disable_nafg);

	fg_read_dts_val(np, "MULTI_BATTERY", &(multi_battery), 1);
	fg_read_dts_val(np, "ACTIVE_TABLE", &(active_table), 1);

	fg_read_dts_val(np, "Q_MAX_L_CURRENT", &(fg_cust_data->q_max_L_current),
		1);
	fg_read_dts_val(np, "Q_MAX_H_CURRENT", &(fg_cust_data->q_max_H_current),
		1);
	fg_read_dts_val_by_idx(np, "g_Q_MAX_SYS_VOLTAGE", gm->battery_id,
		&(fg_cust_data->q_max_sys_voltage), UNIT_TRANS_10);

	fg_read_dts_val(np, "PSEUDO1_EN", &(fg_cust_data->pseudo1_en), 1);
	fg_read_dts_val(np, "PSEUDO100_EN", &(fg_cust_data->pseudo100_en), 1);
	fg_read_dts_val(np, "PSEUDO100_EN_DIS",
		&(fg_cust_data->pseudo100_en_dis), 1);
	fg_read_dts_val_by_idx(np, "g_FG_PSEUDO1_OFFSET", gm->battery_id,
		&(fg_cust_data->pseudo1_iq_offset), UNIT_TRANS_100);

	/* iboot related */
	fg_read_dts_val(np, "QMAX_SEL", &(fg_cust_data->qmax_sel), 1);
	fg_read_dts_val(np, "IBOOT_SEL", &(fg_cust_data->iboot_sel), 1);
	fg_read_dts_val(np, "SHUTDOWN_SYSTEM_IBOOT",
		&(fg_cust_data->shutdown_system_iboot), 1);

	/*hw related */
	fg_read_dts_val(np, "CAR_TUNE_VALUE", &(fg_cust_data->car_tune_value),
		UNIT_TRANS_10);
	gm->gauge->hw_status.car_tune_value =
		fg_cust_data->car_tune_value;

	fg_read_dts_val(np, "FG_METER_RESISTANCE",
		&(fg_cust_data->fg_meter_resistance), 1);
	ret = fg_read_dts_val(np, "COM_FG_METER_RESISTANCE",
		&(fg_cust_data->com_fg_meter_resistance), 1);
	if (ret == -1)
		fg_cust_data->com_fg_meter_resistance =
			fg_cust_data->fg_meter_resistance;

	fg_read_dts_val(np, "NO_BAT_TEMP_COMPENSATE",
		&(gm->no_bat_temp_compensate), 1);

	fg_read_dts_val(np, "CURR_MEASURE_20A", &(fg_cust_data->curr_measure_20a), 1);

	fg_read_dts_val(np, "UNIT_MULTIPLE", &(fg_cust_data->unit_multiple), 1);

	fg_read_dts_val(np, "R_FG_VALUE", &(fg_cust_data->r_fg_value),
		UNIT_TRANS_10);

	fg_read_dts_val(np, "CURR_MEASURE_20A",
		&(fg_cust_data->curr_measure_20a), 1);
	fg_read_dts_val(np, "UNIT_MULTIPLE",
		&(fg_cust_data->unit_multiple), 1);

	gm->gauge->hw_status.r_fg_value =
		fg_cust_data->r_fg_value;

	ret = fg_read_dts_val(np, "COM_R_FG_VALUE",
		&(fg_cust_data->com_r_fg_value), UNIT_TRANS_10);
	if (ret == -1)
		fg_cust_data->com_r_fg_value = fg_cust_data->r_fg_value;

	fg_read_dts_val(np, "FULL_TRACKING_BAT_INT2_MULTIPLY",
		&(fg_cust_data->full_tracking_bat_int2_multiply), 1);
	fg_read_dts_val(np, "enable_tmp_intr_suspend",
		&(gm->enable_tmp_intr_suspend), 1);

	/* Aging Compensation */
	fg_read_dts_val(np, "AGING_ONE_EN", &(fg_cust_data->aging_one_en), 1);
	fg_read_dts_val(np, "AGING1_UPDATE_SOC",
		&(fg_cust_data->aging1_update_soc), UNIT_TRANS_100);
	fg_read_dts_val(np, "AGING1_LOAD_SOC",
		&(fg_cust_data->aging1_load_soc), UNIT_TRANS_100);
	fg_read_dts_val(np, "AGING_TEMP_DIFF",
		&(fg_cust_data->aging_temp_diff), 1);
	fg_read_dts_val(np, "AGING_100_EN", &(fg_cust_data->aging_100_en), 1);
	fg_read_dts_val(np, "DIFFERENCE_VOLTAGE_UPDATE",
		&(fg_cust_data->difference_voltage_update), 1);
	fg_read_dts_val(np, "AGING_FACTOR_MIN",
		&(fg_cust_data->aging_factor_min), UNIT_TRANS_100);
	fg_read_dts_val(np, "AGING_FACTOR_DIFF",
		&(fg_cust_data->aging_factor_diff), UNIT_TRANS_100);
	/* Aging Compensation 2*/
	fg_read_dts_val(np, "AGING_TWO_EN", &(fg_cust_data->aging_two_en), 1);
	/* Aging Compensation 3*/
	fg_read_dts_val(np, "AGING_THIRD_EN", &(fg_cust_data->aging_third_en),
		1);

	/* ui_soc related */
	fg_read_dts_val(np, "DIFF_SOC_SETTING",
		&(fg_cust_data->diff_soc_setting), 1);
	fg_read_dts_val(np, "KEEP_100_PERCENT",
		&(fg_cust_data->keep_100_percent), UNIT_TRANS_100);
	fg_read_dts_val(np, "DIFFERENCE_FULL_CV",
		&(fg_cust_data->difference_full_cv), 1);
	fg_read_dts_val(np, "DIFF_BAT_TEMP_SETTING",
		&(fg_cust_data->diff_bat_temp_setting), 1);
	fg_read_dts_val(np, "DIFF_BAT_TEMP_SETTING_C",
		&(fg_cust_data->diff_bat_temp_setting_c), 1);
	fg_read_dts_val(np, "DISCHARGE_TRACKING_TIME",
		&(fg_cust_data->discharge_tracking_time), 1);
	fg_read_dts_val(np, "CHARGE_TRACKING_TIME",
		&(fg_cust_data->charge_tracking_time), 1);
	fg_read_dts_val(np, "DIFFERENCE_FULLOCV_VTH",
		&(fg_cust_data->difference_fullocv_vth), 1);
	fg_read_dts_val(np, "DIFFERENCE_FULLOCV_ITH",
		&(fg_cust_data->difference_fullocv_ith), UNIT_TRANS_10);
	fg_read_dts_val(np, "CHARGE_PSEUDO_FULL_LEVEL",
		&(fg_cust_data->charge_pseudo_full_level), 1);
	fg_read_dts_val(np, "OVER_DISCHARGE_LEVEL",
		&(fg_cust_data->over_discharge_level), 1);

	/* pre tracking */
	fg_read_dts_val(np, "FG_PRE_TRACKING_EN",
		&(fg_cust_data->fg_pre_tracking_en), 1);
	fg_read_dts_val(np, "VBAT2_DET_TIME",
		&(fg_cust_data->vbat2_det_time), 1);
	fg_read_dts_val(np, "VBAT2_DET_COUNTER",
		&(fg_cust_data->vbat2_det_counter), 1);
	fg_read_dts_val(np, "VBAT2_DET_VOLTAGE1",
		&(fg_cust_data->vbat2_det_voltage1), 1);
	fg_read_dts_val(np, "VBAT2_DET_VOLTAGE2",
		&(fg_cust_data->vbat2_det_voltage2), 1);
	fg_read_dts_val(np, "VBAT2_DET_VOLTAGE3",
		&(fg_cust_data->vbat2_det_voltage3), 1);

	/* sw fg */
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH1",
		&(fg_cust_data->difference_fgc_fgv_th1), 1);
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH2",
		&(fg_cust_data->difference_fgc_fgv_th2), 1);
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH3",
		&(fg_cust_data->difference_fgc_fgv_th3), 1);
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH_SOC1",
		&(fg_cust_data->difference_fgc_fgv_th_soc1), 1);
	fg_read_dts_val(np, "DIFFERENCE_FGC_FGV_TH_SOC2",
		&(fg_cust_data->difference_fgc_fgv_th_soc2), 1);
	fg_read_dts_val(np, "NAFG_TIME_SETTING",
		&(fg_cust_data->nafg_time_setting), 1);
	fg_read_dts_val(np, "NAFG_RATIO", &(fg_cust_data->nafg_ratio), 1);
	fg_read_dts_val(np, "NAFG_RATIO_EN", &(fg_cust_data->nafg_ratio_en), 1);
	fg_read_dts_val(np, "NAFG_RATIO_TMP_THR",
		&(fg_cust_data->nafg_ratio_tmp_thr), 1);
	fg_read_dts_val(np, "NAFG_RESISTANCE", &(fg_cust_data->nafg_resistance),
		1);

	/* mode select */
	fg_read_dts_val(np, "PMIC_SHUTDOWN_CURRENT",
		&(fg_cust_data->pmic_shutdown_current), 1);
	fg_read_dts_val(np, "PMIC_SHUTDOWN_SW_EN",
		&(fg_cust_data->pmic_shutdown_sw_en), 1);
	fg_read_dts_val(np, "FORCE_VC_MODE", &(fg_cust_data->force_vc_mode), 1);
	fg_read_dts_val(np, "EMBEDDED_SEL", &(fg_cust_data->embedded_sel), 1);
	fg_read_dts_val(np, "LOADING_1_EN", &(fg_cust_data->loading_1_en), 1);
	fg_read_dts_val(np, "LOADING_2_EN", &(fg_cust_data->loading_2_en), 1);
	fg_read_dts_val(np, "DIFF_IAVG_TH", &(fg_cust_data->diff_iavg_th), 1);

	fg_read_dts_val(np, "SHUTDOWN_GAUGE0", &(fg_cust_data->shutdown_gauge0),
		1);
	fg_read_dts_val(np, "SHUTDOWN_1_TIME", &(fg_cust_data->shutdown_1_time),
		1);
	fg_read_dts_val(np, "SHUTDOWN_GAUGE1_XMINS",
		&(fg_cust_data->shutdown_gauge1_xmins), 1);
	fg_read_dts_val(np, "SHUTDOWN_GAUGE0_VOLTAGE",
		&(fg_cust_data->shutdown_gauge0_voltage), 1);
	fg_read_dts_val(np, "SHUTDOWN_GAUGE1_VBAT_EN",
		&(fg_cust_data->shutdown_gauge1_vbat_en), 1);
	fg_read_dts_val(np, "SHUTDOWN_GAUGE1_VBAT",
		&(fg_cust_data->shutdown_gauge1_vbat), 1);

	/* ZCV update */
	fg_read_dts_val(np, "ZCV_SUSPEND_TIME",
		&(fg_cust_data->zcv_suspend_time), 1);
	fg_read_dts_val(np, "SLEEP_CURRENT_AVG",
		&(fg_cust_data->sleep_current_avg), 1);
	fg_read_dts_val(np, "ZCV_COM_VOL_LIMIT",
		&(fg_cust_data->zcv_com_vol_limit), 1);
	fg_read_dts_val(np, "ZCV_CAR_GAP_PERCENTAGE",
		&(fg_cust_data->zcv_car_gap_percentage), 1);

	/* dod_init */
	fg_read_dts_val(np, "HWOCV_OLDOCV_DIFF",
		&(fg_cust_data->hwocv_oldocv_diff), 1);
	fg_read_dts_val(np, "HWOCV_OLDOCV_DIFF_CHR",
		&(fg_cust_data->hwocv_oldocv_diff_chr), 1);
	fg_read_dts_val(np, "HWOCV_SWOCV_DIFF",
		&(fg_cust_data->hwocv_swocv_diff), 1);
	fg_read_dts_val(np, "HWOCV_SWOCV_DIFF_LT",
		&(fg_cust_data->hwocv_swocv_diff_lt), 1);
	fg_read_dts_val(np, "HWOCV_SWOCV_DIFF_LT_TEMP",
		&(fg_cust_data->hwocv_swocv_diff_lt_temp), 1);
	fg_read_dts_val(np, "SWOCV_OLDOCV_DIFF",
		&(fg_cust_data->swocv_oldocv_diff), 1);
	fg_read_dts_val(np, "SWOCV_OLDOCV_DIFF_CHR",
		&(fg_cust_data->swocv_oldocv_diff_chr), 1);
	fg_read_dts_val(np, "VBAT_OLDOCV_DIFF",
		&(fg_cust_data->vbat_oldocv_diff), 1);
	fg_read_dts_val(np, "SWOCV_OLDOCV_DIFF_EMB",
		&(fg_cust_data->swocv_oldocv_diff_emb), 1);

	fg_read_dts_val(np, "PMIC_SHUTDOWN_TIME",
		&(fg_cust_data->pmic_shutdown_time), UNIT_TRANS_60);
	fg_read_dts_val(np, "TNEW_TOLD_PON_DIFF",
		&(fg_cust_data->tnew_told_pon_diff), 1);
	fg_read_dts_val(np, "TNEW_TOLD_PON_DIFF2",
		&(fg_cust_data->tnew_told_pon_diff2), 1);
	fg_read_dts_val(np, "EXT_HWOCV_SWOCV",
		&(gm->ext_hwocv_swocv), 1);
	fg_read_dts_val(np, "EXT_HWOCV_SWOCV_LT",
		&(gm->ext_hwocv_swocv_lt), 1);
	fg_read_dts_val(np, "EXT_HWOCV_SWOCV_LT_TEMP",
		&(gm->ext_hwocv_swocv_lt_temp), 1);

	fg_read_dts_val(np, "DC_RATIO_SEL", &(fg_cust_data->dc_ratio_sel), 1);
	fg_read_dts_val(np, "DC_R_CNT", &(fg_cust_data->dc_r_cnt), 1);

	fg_read_dts_val(np, "PSEUDO1_SEL", &(fg_cust_data->pseudo1_sel), 1);

	fg_read_dts_val(np, "D0_SEL", &(fg_cust_data->d0_sel), 1);
	fg_read_dts_val(np, "AGING_SEL", &(fg_cust_data->aging_sel), 1);
	fg_read_dts_val(np, "BAT_PAR_I", &(fg_cust_data->bat_par_i), 1);
	fg_read_dts_val(np, "RECORD_LOG", &(fg_cust_data->record_log), 1);


	fg_read_dts_val(np, "FG_TRACKING_CURRENT",
		&(fg_cust_data->fg_tracking_current), 1);
	fg_read_dts_val(np, "FG_TRACKING_CURRENT_IBOOT_EN",
		&(fg_cust_data->fg_tracking_current_iboot_en), 1);
	fg_read_dts_val(np, "UI_FAST_TRACKING_EN",
		&(fg_cust_data->ui_fast_tracking_en), 1);
	fg_read_dts_val(np, "UI_FAST_TRACKING_GAP",
		&(fg_cust_data->ui_fast_tracking_gap), 1);

	fg_read_dts_val(np, "BAT_PLUG_OUT_TIME",
		&(fg_cust_data->bat_plug_out_time), 1);
	fg_read_dts_val(np, "KEEP_100_PERCENT_MINSOC",
		&(fg_cust_data->keep_100_percent_minsoc), 1);

	fg_read_dts_val(np, "UISOC_UPDATE_TYPE",
		&(fg_cust_data->uisoc_update_type), 1);

	fg_read_dts_val(np, "BATTERY_TMP_TO_DISABLE_GM30",
		&(fg_cust_data->battery_tmp_to_disable_gm30), 1);
	fg_read_dts_val(np, "BATTERY_TMP_TO_DISABLE_NAFG",
		&(fg_cust_data->battery_tmp_to_disable_nafg), 1);
	fg_read_dts_val(np, "BATTERY_TMP_TO_ENABLE_NAFG",
		&(fg_cust_data->battery_tmp_to_enable_nafg), 1);

	fg_read_dts_val(np, "LOW_TEMP_MODE", &(fg_cust_data->low_temp_mode), 1);
	fg_read_dts_val(np, "LOW_TEMP_MODE_TEMP",
		&(fg_cust_data->low_temp_mode_temp), 1);

	/* current limit for uisoc 100% */
	fg_read_dts_val(np, "UI_FULL_LIMIT_EN",
		&(fg_cust_data->ui_full_limit_en), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC0",
		&(fg_cust_data->ui_full_limit_soc0), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH0",
		&(fg_cust_data->ui_full_limit_ith0), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC1",
		&(fg_cust_data->ui_full_limit_soc1), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH1",
		&(fg_cust_data->ui_full_limit_ith1), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC2",
		&(fg_cust_data->ui_full_limit_soc2), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH2",
		&(fg_cust_data->ui_full_limit_ith2), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC3",
		&(fg_cust_data->ui_full_limit_soc3), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH3",
		&(fg_cust_data->ui_full_limit_ith3), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_SOC4",
		&(fg_cust_data->ui_full_limit_soc4), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_ITH4",
		&(fg_cust_data->ui_full_limit_ith4), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_TIME",
		&(fg_cust_data->ui_full_limit_time), 1);

	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_SOC0",
		&(fg_cust_data->ui_full_limit_fc_soc0), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_ITH0",
		&(fg_cust_data->ui_full_limit_fc_ith0), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_SOC1",
		&(fg_cust_data->ui_full_limit_fc_soc1), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_ITH1",
		&(fg_cust_data->ui_full_limit_fc_ith1), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_SOC2",
		&(fg_cust_data->ui_full_limit_fc_soc2), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_ITH2",
		&(fg_cust_data->ui_full_limit_fc_ith2), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_SOC3",
		&(fg_cust_data->ui_full_limit_fc_soc3), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_ITH3",
		&(fg_cust_data->ui_full_limit_fc_ith3), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_SOC4",
		&(fg_cust_data->ui_full_limit_fc_soc4), 1);
	fg_read_dts_val(np, "UI_FULL_LIMIT_FC_ITH4",
		&(fg_cust_data->ui_full_limit_fc_ith4), 1);

	/* voltage limit for uisoc 1% */
	fg_read_dts_val(np, "UI_LOW_LIMIT_EN", &(fg_cust_data->ui_low_limit_en),
		1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC0",
		&(fg_cust_data->ui_low_limit_soc0), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH0",
		&(fg_cust_data->ui_low_limit_vth0), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC1",
		&(fg_cust_data->ui_low_limit_soc1), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH1",
		&(fg_cust_data->ui_low_limit_vth1), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC2",
		&(fg_cust_data->ui_low_limit_soc2), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH2",
		&(fg_cust_data->ui_low_limit_vth2), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC3",
		&(fg_cust_data->ui_low_limit_soc3), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH3",
		&(fg_cust_data->ui_low_limit_vth3), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_SOC4",
		&(fg_cust_data->ui_low_limit_soc4), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_VTH4",
		&(fg_cust_data->ui_low_limit_vth4), 1);
	fg_read_dts_val(np, "UI_LOW_LIMIT_TIME",
		&(fg_cust_data->ui_low_limit_time), 1);

	/* average battemp */
	fg_read_dts_val(np, "MOVING_BATTEMP_EN",
		&(fg_cust_data->moving_battemp_en), 1);
	fg_read_dts_val(np, "MOVING_BATTEMP_THR",
		&(fg_cust_data->moving_battemp_thr), 1);

	gm->disableGM30 = of_property_read_bool(
		np, "DISABLE_MTKBATTERY");
	fg_read_dts_val(np, "MULTI_TEMP_GAUGE0",
		&(fg_cust_data->multi_temp_gauge0), 1);
	fg_read_dts_val(np, "FGC_FGV_TH1",
		&(fg_cust_data->difference_fgc_fgv_th1), 1);
	fg_read_dts_val(np, "FGC_FGV_TH2",
		&(fg_cust_data->difference_fgc_fgv_th2), 1);
	fg_read_dts_val(np, "FGC_FGV_TH3",
		&(fg_cust_data->difference_fgc_fgv_th3), 1);
	fg_read_dts_val(np, "UISOC_UPDATE_T",
		&(fg_cust_data->uisoc_update_type), 1);
	fg_read_dts_val(np, "UIFULLLIMIT_EN",
		&(fg_cust_data->ui_full_limit_en), 1);
	fg_read_dts_val(np, "MTK_CHR_EXIST", &(fg_cust_data->mtk_chr_exist), 1);

	fg_read_dts_val(np, "GM30_DISABLE_NAFG", &(fg_cust_data->disable_nafg),
		1);
	fg_read_dts_val(np, "FIXED_BATTERY_TEMPERATURE", &(gm->fixed_bat_tmp),
		1);

	fg_read_dts_val(np, "ACTIVE_TABLE",
		&(fg_table_cust_data->active_table_number), 1);

#if IS_ENABLED(CONFIG_MTK_ADDITIONAL_BATTERY_TABLE)
	if (fg_table_cust_data->active_table_number == 0)
		fg_table_cust_data->active_table_number = 5;
#else
	if (fg_table_cust_data->active_table_number == 0)
		fg_table_cust_data->active_table_number = 4;
#endif

	bm_err("fg active table:%d\n",
		fg_table_cust_data->active_table_number);

	/* battery temperature  related*/
	fg_read_dts_val(np, "RBAT_PULL_UP_R", &(gm->rbat.rbat_pull_up_r), 1);
	fg_read_dts_val(np, "RBAT_PULL_UP_VOLT",
		&(gm->rbat.rbat_pull_up_volt), 1);

	/* battery temperature, TEMPERATURE_T0 ~ T9 */
	for (i = 0; i < fg_table_cust_data->active_table_number; i++) {
		sprintf(node_name, "TEMPERATURE_T%d", i);
		fg_read_dts_val(np, node_name,
			&(fg_table_cust_data->fg_profile[i].temperature), 1);
		}

	fg_read_dts_val(np, "TEMPERATURE_TB0",
		&(fg_table_cust_data->temperature_tb0), 1);
	fg_read_dts_val(np, "TEMPERATURE_TB1",
		&(fg_table_cust_data->temperature_tb1), 1);

	for (i = 0; i < MAX_TABLE; i++) {
		struct fuelgauge_profile_struct *p;

		p = &fg_table_cust_data->fg_profile[i].fg_profile[0];
		fg_read_dts_val_by_idx(np, "g_temperature", i,
			&(fg_table_cust_data->fg_profile[i].temperature), 1);
		fg_read_dts_val_by_idx(np, "g_Q_MAX",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].q_max), 1);
		fg_read_dts_val_by_idx(np, "g_Q_MAX_H_CURRENT",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].q_max_h_current),
			1);
		fg_read_dts_val_by_idx(np, "g_FG_PSEUDO1",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].pseudo1),
			UNIT_TRANS_100);
		fg_read_dts_val_by_idx(np, "g_FG_PSEUDO100",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val_by_idx(np, "g_PMIC_MIN_VOL",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].pmic_min_vol), 1);
		fg_read_dts_val_by_idx(np, "g_PON_SYS_IBOOT",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].pon_iboot), 1);
		fg_read_dts_val_by_idx(np, "g_QMAX_SYS_VOL",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].qmax_sys_vol), 1);
		fg_read_dts_val_by_idx(np, "g_SHUTDOWN_HL_ZCV",
			i*TOTAL_BATTERY_NUMBER + gm->battery_id,
			&(fg_table_cust_data->fg_profile[i].shutdown_hl_zcv),
			1);
		for (j = 0; j < 100; j++) {
			if (p[j].charge_r.rdc[0] == 0)
				p[j].charge_r.rdc[0] = p[j].resistance;
	}
	}

	if (bat_id >= 0 && bat_id < TOTAL_BATTERY_NUMBER) {
		sprintf(node_name, "Q_MAX_SYS_VOLTAGE_BAT%d", bat_id);
		fg_read_dts_val(np, node_name,
			&(fg_cust_data->q_max_sys_voltage), UNIT_TRANS_10);
		sprintf(node_name, "PSEUDO1_IQ_OFFSET_BAT%d", bat_id);
		fg_read_dts_val(np, node_name,
			&(fg_cust_data->pseudo1_iq_offset), UNIT_TRANS_100);
	} else
		bm_err(
		"get Q_MAX_SYS_VOLTAGE_BAT, PSEUDO1_IQ_OFFSET_BAT %d no data\n",
		bat_id);

	if (fg_cust_data->multi_temp_gauge0 == 0) {
		int i = 0;
		int min_vol;

		min_vol = fg_table_cust_data->fg_profile[0].pmic_min_vol;
		if (!of_property_read_u32(np, "PMIC_MIN_VOL", &val)) {
			for (i = 0; i < MAX_TABLE; i++)
				fg_table_cust_data->fg_profile[i].pmic_min_vol =
				(int)val;
				bm_debug("Get PMIC_MIN_VOL: %d\n",
					min_vol);
		} else {
			bm_err("Get PMIC_MIN_VOL no data\n");
		}

		if (!of_property_read_u32(np, "POWERON_SYSTEM_IBOOT", &val)) {
			for (i = 0; i < MAX_TABLE; i++)
				fg_table_cust_data->fg_profile[i].pon_iboot =
				(int)val * UNIT_TRANS_10;

			bm_debug("Get POWERON_SYSTEM_IBOOT: %d\n",
				fg_table_cust_data->fg_profile[0].pon_iboot);
		} else {
			bm_err("Get POWERON_SYSTEM_IBOOT no data\n");
		}
	}

	if (active_table == 0 && multi_battery == 0) {
		fg_read_dts_val(np, "g_FG_PSEUDO100_T0",
			&(fg_table_cust_data->fg_profile[0].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO100_T1",
			&(fg_table_cust_data->fg_profile[1].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO100_T2",
			&(fg_table_cust_data->fg_profile[2].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO100_T3",
			&(fg_table_cust_data->fg_profile[3].pseudo100),
			UNIT_TRANS_100);
		fg_read_dts_val(np, "g_FG_PSEUDO100_T4",
			&(fg_table_cust_data->fg_profile[4].pseudo100),
			UNIT_TRANS_100);
	}

	/* compatiable with old dtsi*/
	if (active_table == 0) {
		fg_read_dts_val(np, "TEMPERATURE_T0",
			&(fg_table_cust_data->fg_profile[0].temperature), 1);
		fg_read_dts_val(np, "TEMPERATURE_T1",
			&(fg_table_cust_data->fg_profile[1].temperature), 1);
		fg_read_dts_val(np, "TEMPERATURE_T2",
			&(fg_table_cust_data->fg_profile[2].temperature), 1);
		fg_read_dts_val(np, "TEMPERATURE_T3",
			&(fg_table_cust_data->fg_profile[3].temperature), 1);
		fg_read_dts_val(np, "TEMPERATURE_T4",
			&(fg_table_cust_data->fg_profile[4].temperature), 1);
	}

	fg_read_dts_val(np, "g_FG_charge_PSEUDO100_row",
		&(r_pseudo100_raw), 1);
	fg_read_dts_val(np, "g_FG_charge_PSEUDO100_col",
		&(r_pseudo100_col), 1);

	/* init for pseudo100 */
	for (i = 0; i < MAX_TABLE; i++) {
		for (j = 0; j < MAX_CHARGE_RDC; j++)
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[j]
				= fg_table_cust_data->fg_profile[i].pseudo100;
	}

	for (i = 0; i < MAX_TABLE; i++) {
		bm_err("%6d %6d %6d %6d %6d\n",
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[0],
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[1],
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[2],
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[3],
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[4]
			);
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	fg_read_dts_val(np, "DIM2_TABLE_DISTINGUISH",
		&(dim2_table_distinguish), 1);
		bm_err("dim2_table_distinguish:%d\n", dim2_table_distinguish);
	if (dim2_table_distinguish) {
		sprintf(node_name, "battery%d_g_FG_charge_PSEUDO100", bat_id);
		for (i = 0; i < MAX_TABLE; i++) {
			for (j = 0; j < r_pseudo100_raw; j++) {
				fg_read_dts_val_by_idx(np, node_name,
					i*r_pseudo100_raw+j,
					&(fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[j+1]),
						UNIT_TRANS_100);
			}
		}

		bm_err("battery%d_g_FG_charge_PSEUDO100 g_FG_charge_PSEUDO100_row:%d g_FG_charge_PSEUDO100_col:%d\n", bat_id, r_pseudo100_raw, r_pseudo100_col);
	} else {
		for (i = 0; i < MAX_TABLE; i++) {
			for (j = 0; j < r_pseudo100_raw; j++) {
				fg_read_dts_val_by_idx(np, "g_FG_charge_PSEUDO100",
					i*r_pseudo100_raw+j,
					&(fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[j+1]),
						UNIT_TRANS_100);
			}
		}

		bm_err("battery_g_FG_charge_PSEUDO100_row:%d battery_g_FG_charge_PSEUDO100_col:%d\n",
			r_pseudo100_raw, r_pseudo100_col);
	}
#else
	/* read dtsi from pseudo100 */
	for (i = 0; i < MAX_TABLE; i++) {
		for (j = 0; j < r_pseudo100_raw; j++) {
			fg_read_dts_val_by_idx(np, "g_FG_charge_PSEUDO100",
				i*r_pseudo100_raw+j,
				&(fg_table_cust_data->fg_profile[
					i].r_pseudo100.pseudo[j+1]),
					UNIT_TRANS_100);
		}
	}


	bm_err("g_FG_charge_PSEUDO100_row:%d g_FG_charge_PSEUDO100_col:%d\n",
		r_pseudo100_raw, r_pseudo100_col);
#endif

	for (i = 0; i < MAX_TABLE; i++) {
		bm_err("%6d %6d %6d %6d %6d\n",
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[0],
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[1],
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[2],
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[3],
			fg_table_cust_data->fg_profile[i].r_pseudo100.pseudo[4]
			);
	}

	// END of pseudo100


	for (i = 0; i < fg_table_cust_data->active_table_number; i++) {
		sprintf(node_name, "battery%d_profile_t%d_num", bat_id, i);
		fg_read_dts_val(np, node_name,
			&(fg_table_cust_data->fg_profile[i].size), 1);

		/* compatiable with old dtsi table*/
		sprintf(node_name, "battery%d_profile_t%d_col", bat_id, i);
		ret = fg_read_dts_val(np, node_name, &(column), 1);
		if (ret == -1)
			column = 3;

		if (column < 3 || column > 8) {
			bm_err("%s, %s,column:%d ERROR!",
				__func__, node_name, column);
			/* correction */
			column = 3;
		}

		sprintf(node_name, "battery%d_profile_t%d", bat_id, i);
		fg_custom_parse_table(gm, np, node_name,
			fg_table_cust_data->fg_profile[i].fg_profile, column);
	}
}

#endif	/* end of CONFIG_OF */

/* ============================================================ */
/* power supply battery */
/* ============================================================ */
void battery_update_psd(struct mtk_battery *gm)
{
	struct battery_data *bat_data = &gm->bs_data;

	if (gm->disableGM30)
		bat_data->bat_batt_vol = 4000;
	else
		gauge_get_property(GAUGE_PROP_BATTERY_VOLTAGE,
			&bat_data->bat_batt_vol);

	bat_data->bat_batt_temp = force_get_tbat(gm, true);
}
void battery_update(struct mtk_battery *gm)
{
	struct battery_data *bat_data = &gm->bs_data;
	struct power_supply *bat_psy = bat_data->psy;

	if (gm->is_probe_done == false || bat_psy == NULL) {
		bm_err("[%s]battery is not rdy:probe:%d\n",
			__func__, gm->is_probe_done);
		return;
	}

	battery_update_psd(gm);
	bat_data->bat_technology = POWER_SUPPLY_TECHNOLOGY_LION;
	bat_data->bat_health = POWER_SUPPLY_HEALTH_GOOD;
	bat_data->bat_present =
		gauge_get_int_property(GAUGE_PROP_BATTERY_EXIST);

	if (battery_get_int_property(BAT_PROP_DISABLE))
		bat_data->bat_capacity = 50;

	if (gm->algo.active == true)
		bat_data->bat_capacity = gm->ui_soc;

	power_supply_changed(bat_psy);

}

/* ============================================================ */
/* interrupt handler */
/* ============================================================ */
void disable_fg(struct mtk_battery *gm)
{
	gm->disableGM30 = true;
	gm->ui_soc = 50;
	gm->bs_data.bat_capacity = 50;

	disable_all_irq(gm);
}

bool fg_interrupt_check(struct mtk_battery *gm)
{
	if (gm->disableGM30) {
		disable_fg(gm);
		return false;
	}

	return true;
}

int fg_coulomb_int_h_handler(struct mtk_battery *gm,
	struct gauge_consumer *consumer)
{
	int fg_coulomb = 0;

	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);

	gm->coulomb_int_ht = fg_coulomb + gm->coulomb_int_gap;
	gm->coulomb_int_lt = fg_coulomb - gm->coulomb_int_gap;

	gauge_coulomb_start(gm, &gm->coulomb_plus, gm->coulomb_int_gap);
	gauge_coulomb_start(gm, &gm->coulomb_minus, -gm->coulomb_int_gap);

	bm_err("[%s] car:%d ht:%d lt:%d gap:%d\n",
		__func__,
		fg_coulomb, gm->coulomb_int_ht,
		gm->coulomb_int_lt, gm->coulomb_int_gap);

	wakeup_fg_algo(gm, FG_INTR_BAT_INT1_HT);

	return 0;
}

int fg_coulomb_int_l_handler(struct mtk_battery *gm,
	struct gauge_consumer *consumer)
{
	int fg_coulomb = 0;

	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);

	fg_sw_bat_cycle_accu(gm);
	gm->coulomb_int_ht = fg_coulomb + gm->coulomb_int_gap;
	gm->coulomb_int_lt = fg_coulomb - gm->coulomb_int_gap;

	gauge_coulomb_start(gm, &gm->coulomb_plus, gm->coulomb_int_gap);
	gauge_coulomb_start(gm, &gm->coulomb_minus, -gm->coulomb_int_gap);

	bm_err("[%s] car:%d ht:%d lt:%d gap:%d\n",
		__func__,
		fg_coulomb, gm->coulomb_int_ht,
		gm->coulomb_int_lt, gm->coulomb_int_gap);
	wakeup_fg_algo(gm, FG_INTR_BAT_INT1_LT);

	return 0;
}

int fg_bat_int2_h_handler(struct mtk_battery *gm,
	struct gauge_consumer *consumer)
{
	int fg_coulomb = 0;

	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
	bm_debug("[%s] car:%d ht:%d\n",
		__func__,
		fg_coulomb, gm->uisoc_int_ht_en);
	fg_sw_bat_cycle_accu(gm);
	wakeup_fg_algo(gm, FG_INTR_BAT_INT2_HT);
	return 0;
}

int fg_bat_int2_l_handler(struct mtk_battery *gm,
	struct gauge_consumer *consumer)
{
	int fg_coulomb = 0;

	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
	bm_debug("[%s] car:%d ht:%d\n",
		__func__,
		fg_coulomb, gm->uisoc_int_lt_gap);
	fg_sw_bat_cycle_accu(gm);
	wakeup_fg_algo(gm, FG_INTR_BAT_INT2_LT);
	return 0;
}

/* ============================================================ */
/* sysfs */
/* ============================================================ */
static int temperature_get(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int *val)
{
	gm->bs_data.bat_batt_temp = force_get_tbat(gm, true);
	*val = gm->bs_data.bat_batt_temp;
	bm_debug("%s %d\n", __func__, *val);
	return 0;
}

static int temperature_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->fixed_bat_tmp = val;
	bm_debug("%s %d\n", __func__, val);
	return 0;
}

static int log_level_get(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int *val)
{
	*val = gm->log_level;
	return 0;
}

static int log_level_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->log_level = val;
	return 0;
}

static int coulomb_int_gap_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	int fg_coulomb = 0;

	gauge_get_property(GAUGE_PROP_COULOMB, &fg_coulomb);
	gm->coulomb_int_gap = val;

	gm->coulomb_int_ht = fg_coulomb + gm->coulomb_int_gap;
	gm->coulomb_int_lt = fg_coulomb - gm->coulomb_int_gap;
	gauge_coulomb_start(gm, &gm->coulomb_plus, gm->coulomb_int_gap);
	gauge_coulomb_start(gm, &gm->coulomb_minus, -gm->coulomb_int_gap);

	bm_debug("[%s]BAT_PROP_COULOMB_INT_GAP = %d car:%d\n",
		__func__,
		gm->coulomb_int_gap, fg_coulomb);
	return 0;
}

static int uisoc_ht_int_gap_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->uisoc_int_ht_gap = val;
	gauge_coulomb_start(gm, &gm->uisoc_plus, gm->uisoc_int_ht_gap);
	bm_debug("[%s]BATTERY_UISOC_INT_HT_GAP = %d\n",
		__func__,
		gm->uisoc_int_ht_gap);
	return 0;
}

static int uisoc_lt_int_gap_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->uisoc_int_lt_gap = val;
	gauge_coulomb_start(gm, &gm->uisoc_minus, -gm->uisoc_int_lt_gap);
	bm_debug("[%s]BATTERY_UISOC_INT_LT_GAP = %d\n",
		__func__,
		gm->uisoc_int_lt_gap);
	return 0;
}

static int en_uisoc_ht_int_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->uisoc_int_ht_en = val;
	if (gm->uisoc_int_ht_en == 0)
		gauge_coulomb_stop(gm, &gm->uisoc_plus);
	bm_debug("[%s][fg_bat_int2] FG_DAEMON_CMD_ENABLE_FG_BAT_INT2_HT = %d\n",
		__func__,
		gm->uisoc_int_ht_en);

	return 0;
}

static int en_uisoc_lt_int_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->uisoc_int_lt_en = val;
	if (gm->uisoc_int_lt_en == 0)
		gauge_coulomb_stop(gm, &gm->uisoc_minus);
	bm_debug("[%s][fg_bat_int2] FG_DAEMON_CMD_ENABLE_FG_BAT_INT2_HT = %d\n",
		__func__,
		gm->uisoc_int_lt_en);

	return 0;
}

static int uisoc_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	int daemon_ui_soc;
	int old_uisoc;
	ktime_t now_time, diff;
	struct timespec64 tmp_time;
	struct mtk_battery_algo *algo;
	struct fuel_gauge_table_custom_data *ptable;
	struct fuel_gauge_custom_data *pdata;

	algo = &gm->algo;
	ptable = &gm->fg_table_cust_data;
	pdata = &gm->fg_cust_data;
	daemon_ui_soc = val;

	if (daemon_ui_soc < 0) {
		bm_debug("[%s] error,daemon_ui_soc:%d\n",
			__func__,
			daemon_ui_soc);
		daemon_ui_soc = 0;
	}

	pdata->ui_old_soc = daemon_ui_soc;
	old_uisoc = gm->ui_soc;

	if (gm->disableGM30 == true)
		gm->ui_soc = 50;
	else
		gm->ui_soc = (daemon_ui_soc + 50) / 100;

	/* when UISOC changes, check the diff time for smooth */
	if (old_uisoc != gm->ui_soc) {
		now_time = ktime_get_boottime();
		diff = ktime_sub(now_time, gm->uisoc_oldtime);

		tmp_time = ktime_to_timespec64(diff);

		bm_debug("[%s] FG_DAEMON_CMD_SET_KERNEL_UISOC = %d %d GM3:%d old:%d diff=%ld\n",
			__func__,
			daemon_ui_soc, gm->ui_soc,
			gm->disableGM30, old_uisoc, tmp_time.tv_sec);
		gm->uisoc_oldtime = now_time;

		gm->bs_data.bat_capacity = gm->ui_soc;
		battery_update(gm);
	} else {
		bm_debug("[%s] FG_DAEMON_CMD_SET_KERNEL_UISOC = %d %d GM3:%d\n",
			__func__,
			daemon_ui_soc, gm->ui_soc, gm->disableGM30);
		/* ac_update(&ac_main); */
		gm->bs_data.bat_capacity = gm->ui_soc;
		battery_update(gm);
	}
	return 0;
}

static int disable_get(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int *val)
{
	*val = gm->disableGM30;
	return 0;
}

static int disable_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->disableGM30 = val;
	if (gm->disableGM30 == true)
		battery_update(gm);
	return 0;
}

static int init_done_get(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int *val)
{
	*val = gm->init_flag;
	return 0;
}

static int init_done_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	gm->init_flag = val;

	bm_debug("[%s] init_flag = %d\n",
		__func__,
		gm->init_flag);

	return 0;
}

static int reset_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	int car;

	if (gm->disableGM30)
		return 0;

	/* must handle sw_ncar before reset car */
	fg_sw_bat_cycle_accu(gm);
	gm->bat_cycle_car = 0;
	car = gauge_get_int_property(GAUGE_PROP_COULOMB);
	gm->log.car_diff += car;

	bm_err("%s car:%d\n",
		__func__, car);

	gauge_coulomb_before_reset(gm);
	gauge_set_property(GAUGE_PROP_RESET, 0);
	gauge_coulomb_after_reset(gm);

	gm->sw_iavg_time = ktime_get_boottime();
	gm->sw_iavg_car = gauge_get_int_property(GAUGE_PROP_COULOMB);
	gm->bat_cycle_car = 0;

	return 0;
}

static int temp_th_set(struct mtk_battery *gm,
	struct mtk_battery_sysfs_field_info *attr,
	int val)
{
	int gap = val;
	int tmp = force_get_tbat(gm, true);

	gm->bat_tmp_c_ht = tmp + gap;
	gm->bat_tmp_c_lt = tmp - gap;

	bm_debug(
		"[%s]FG_DAEMON_CMD_SET_FG_BAT_TMP_C_GAP=%d ht:%d lt:%d\n",
		__func__, gap,
		gm->bat_tmp_c_ht,
		gm->bat_tmp_c_lt);

	return 0;
}

static ssize_t bat_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy;
	struct mtk_battery *gm;
	struct mtk_battery_sysfs_field_info *battery_attr;
	int val;
	ssize_t ret;

	ret = kstrtos32(buf, 0, &val);
	if (ret < 0)
		return ret;

	psy = dev_get_drvdata(dev);
	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);

	if (gm == NULL)
		return -ENODEV;

	battery_attr = container_of(attr,
		struct mtk_battery_sysfs_field_info, attr);
	if (battery_attr->set != NULL)
		battery_attr->set(gm, battery_attr, val);

	return count;
}

static ssize_t bat_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct power_supply *psy;
	struct mtk_battery *gm;
	struct mtk_battery_sysfs_field_info *battery_attr;
	int val = 0;
	ssize_t count;

	psy = dev_get_drvdata(dev);
	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);
	if (gm == NULL)
		return -ENODEV;

	battery_attr = container_of(attr,
		struct mtk_battery_sysfs_field_info, attr);
	if (battery_attr->get != NULL)
		battery_attr->get(gm, battery_attr, &val);

	count = scnprintf(buf, PAGE_SIZE, "%d\n", val);
	return count;
}

/* Must be in the same order as BAT_PROP_* */
static struct mtk_battery_sysfs_field_info battery_sysfs_field_tbl[] = {
	BAT_SYSFS_FIELD_RW(temperature, BAT_PROP_TEMPERATURE),
	BAT_SYSFS_FIELD_WO(coulomb_int_gap, BAT_PROP_COULOMB_INT_GAP),
	BAT_SYSFS_FIELD_WO(uisoc_ht_int_gap, BAT_PROP_UISOC_HT_INT_GAP),
	BAT_SYSFS_FIELD_WO(uisoc_lt_int_gap, BAT_PROP_UISOC_LT_INT_GAP),
	BAT_SYSFS_FIELD_WO(en_uisoc_ht_int, BAT_PROP_ENABLE_UISOC_HT_INT),
	BAT_SYSFS_FIELD_WO(en_uisoc_lt_int, BAT_PROP_ENABLE_UISOC_LT_INT),
	BAT_SYSFS_FIELD_WO(uisoc, BAT_PROP_UISOC),
	BAT_SYSFS_FIELD_RW(disable, BAT_PROP_DISABLE),
	BAT_SYSFS_FIELD_RW(init_done, BAT_PROP_INIT_DONE),
	BAT_SYSFS_FIELD_WO(reset, BAT_PROP_FG_RESET),
	BAT_SYSFS_FIELD_RW(log_level, BAT_PROP_LOG_LEVEL),
	BAT_SYSFS_FIELD_WO(temp_th, BAT_PROP_TEMP_TH_GAP),
};

int battery_get_property(enum battery_property bp,
			    int *val)
{
	struct mtk_battery *gm;
	struct power_supply *psy;

#ifndef OPLUS_FEATURE_CHG_BASIC
	psy = power_supply_get_by_name("battery");
#else
	psy = power_supply_get_by_name("mtk-battery");
#endif

	if (psy == NULL)
		return -ENODEV;

	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);
	if (gm == NULL)
		return -ENODEV;
	if (battery_sysfs_field_tbl[bp].prop == bp)
		battery_sysfs_field_tbl[bp].get(gm,
			&battery_sysfs_field_tbl[bp], val);
	else {
		bm_err("%s bp:%d idx error\n", __func__, bp);
		return -ENOTSUPP;
	}

	return 0;
}

int battery_get_int_property(enum battery_property bp)
{
	int val;

	battery_get_property(bp, &val);
	return val;
}

int battery_set_property(enum battery_property bp,
			    int val)
{
	struct mtk_battery *gm;
	struct power_supply *psy;

#ifndef OPLUS_FEATURE_CHG_BASIC
	psy = power_supply_get_by_name("battery");
#else
	psy = power_supply_get_by_name("mtk-battery");
#endif

	if (psy == NULL)
		return -ENODEV;

	gm = (struct mtk_battery *)power_supply_get_drvdata(psy);

	if (gm == NULL)
		return -ENODEV;

	if (battery_sysfs_field_tbl[bp].prop == bp)
		battery_sysfs_field_tbl[bp].set(gm,
			&battery_sysfs_field_tbl[bp], val);
	else {
		bm_err("%s bp:%d idx error\n", __func__, bp);
		return -ENOTSUPP;
	}
	return 0;
}

static struct attribute *
	battery_sysfs_attrs[ARRAY_SIZE(battery_sysfs_field_tbl) + 1];

static const struct attribute_group battery_sysfs_attr_group = {
	.attrs = battery_sysfs_attrs,
};

static void battery_sysfs_init_attrs(void)
{
	int i, limit = ARRAY_SIZE(battery_sysfs_field_tbl);

	for (i = 0; i < limit; i++)
		battery_sysfs_attrs[i] = &battery_sysfs_field_tbl[i].attr.attr;

	battery_sysfs_attrs[limit] = NULL; /* Has additional entry for this */
}

static int battery_sysfs_create_group(struct power_supply *psy)
{
	battery_sysfs_init_attrs();

	return sysfs_create_group(&psy->dev.kobj,
			&battery_sysfs_attr_group);
}

/* ============================================================ */
/* nafg monitor */
/* ============================================================ */
void fg_nafg_monitor(struct mtk_battery *gm)
{
	int nafg_cnt = 0;
	ktime_t now_time = 0, dtime = 0;
	struct timespec64 tmp_dtime, tmp_now_time, tmp_last_time;

	if (gm->disableGM30 || gm->cmd_disable_nafg || gm->ntc_disable_nafg)
		return;

	tmp_now_time.tv_sec = 0;
	tmp_now_time.tv_nsec = 0;
	tmp_dtime.tv_sec = 0;
	tmp_dtime.tv_nsec = 0;

	nafg_cnt = gauge_get_int_property(GAUGE_PROP_NAFG_CNT);

	if (gm->last_nafg_cnt != nafg_cnt) {
		gm->last_nafg_cnt = nafg_cnt;
		gm->last_nafg_update_time = ktime_get_boottime();
	} else {
		now_time = ktime_get_boottime();
		dtime = ktime_sub(now_time, gm->last_nafg_update_time);
		tmp_dtime = ktime_to_timespec64(dtime);

		if (tmp_dtime.tv_sec >= 600) {
			gm->is_nafg_broken = true;
			wakeup_fg_algo_cmd(
				gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_DISABLE_NAFG,
				true);
		}
	}

	tmp_now_time = ktime_to_timespec64(now_time);
	tmp_last_time = ktime_to_timespec64(gm->last_nafg_update_time);

	bm_debug("[%s]diff_time:%d nafg_cnt:%d, now:%d, last_t:%d\n",
		__func__,
		(int)tmp_dtime.tv_sec,
		gm->last_nafg_cnt,
		(int)tmp_now_time.tv_sec,
		(int)tmp_last_time.tv_sec);

}

/* ============================================================ */
/* periodic timer */
/* ============================================================ */
static void fg_drv_update_hw_status(struct mtk_battery *gm)
{
	ktime_t ktime;

	gm->tbat = force_get_tbat_internal(gm, true);

	bm_err("car[%d,%ld,%ld,%ld,%ld] tmp:%d soc:%d uisoc:%d vbat:%d ibat:%d baton:%d algo:%d gm3:%d %d %d %d %d,boot:%d\n",
		gauge_get_int_property(GAUGE_PROP_COULOMB),
		gm->coulomb_plus.end, gm->coulomb_minus.end,
		gm->uisoc_plus.end, gm->uisoc_minus.end,
		gm->tbat,
		gm->soc, gm->ui_soc,
		gm->vbat,
		gm->ibat,
		gm->baton,
		gm->algo.active,
		gm->disableGM30, gm->fg_cust_data.disable_nafg,
		gm->ntc_disable_nafg, gm->cmd_disable_nafg, gm->vbat0_flag,
		gm->bootmode);

	fg_drv_update_daemon(gm);

	/* kernel mode need regular update info */
	if (gm->algo.active == true)
		battery_update(gm);

	if (bat_get_debug_level() >= BMLOG_DEBUG_LEVEL)
		ktime = ktime_set(10, 0);
	else
		ktime = ktime_set(60, 0);

	hrtimer_start(&gm->fg_hrtimer, ktime, HRTIMER_MODE_REL);
}

int battery_update_routine(void *arg)
{
	struct mtk_battery *gm = (struct mtk_battery *)arg;
	int ret = 0;

	battery_update_psd(gm);
	while (1) {
		bm_err("%s\n", __func__);
		ret = wait_event_interruptible(gm->wait_que,
			(gm->fg_update_flag > 0) && !gm->in_sleep);
		mutex_lock(&gm->fg_update_lock);
		if (gm->in_sleep)
			goto in_sleep;
		gm->fg_update_flag = 0;
		fg_drv_update_hw_status(gm);
in_sleep:
		mutex_unlock(&gm->fg_update_lock);
	}
}

#ifdef CONFIG_PM
static int system_pm_notify(struct notifier_block *nb,
			    unsigned long mode, void *_unused)
{
	struct mtk_battery *gm =
			container_of(nb, struct mtk_battery, pm_nb);
	struct battery_data *bat_data = &gm->bs_data;
	struct power_supply *bat_psy = bat_data->psy;

	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
	case PM_SUSPEND_PREPARE:
		if (bat_psy->changed)
			return NOTIFY_BAD;
		if (!mutex_trylock(&gm->fg_update_lock))
			return NOTIFY_BAD;
		gm->in_sleep = true;
		mutex_unlock(&gm->fg_update_lock);
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		mutex_lock(&gm->fg_update_lock);
		gm->in_sleep = false;
		mutex_unlock(&gm->fg_update_lock);
		wake_up(&gm->wait_que);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}
#endif /* CONFIG_PM */

void fg_update_routine_wakeup(struct mtk_battery *gm)
{
	bm_debug("%s\n", __func__);
	gm->fg_update_flag = 1;
	wake_up(&gm->wait_que);
}

enum hrtimer_restart fg_drv_thread_hrtimer_func(struct hrtimer *timer)
{
	struct mtk_battery *gm;

	bm_debug("%s\n", __func__);
	gm = container_of(timer,
		struct mtk_battery, fg_hrtimer);
	fg_update_routine_wakeup(gm);
	return HRTIMER_NORESTART;
}

void fg_drv_thread_hrtimer_init(struct mtk_battery *gm)
{
	ktime_t ktime;

	ktime = ktime_set(10, 0);
	hrtimer_init(&gm->fg_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	gm->fg_hrtimer.function = fg_drv_thread_hrtimer_func;
	hrtimer_start(&gm->fg_hrtimer, ktime, HRTIMER_MODE_REL);
}

/* ============================================================ */
/* alarm timer handler */
/* ============================================================ */
static void tracking_timer_work_handler(struct work_struct *data)
{
	struct mtk_battery *gm;

	gm = container_of(data,
		struct mtk_battery, tracking_timer_work);
	bm_debug("[%s]\n", __func__);
	wakeup_fg_algo(gm, FG_INTR_FG_TIME);
}

static enum alarmtimer_restart tracking_timer_callback(
	struct alarm *alarm, ktime_t now)
{
	struct mtk_battery *gm;

	gm = container_of(alarm,
		struct mtk_battery, tracking_timer);
	bm_debug("[%s]\n", __func__);
	schedule_work(&gm->tracking_timer_work);
	return ALARMTIMER_NORESTART;
}

static void one_percent_timer_work_handler(struct work_struct *data)
{
	struct mtk_battery *gm;

	gm = container_of(data,
		struct mtk_battery, one_percent_timer_work);
	bm_debug("[%s]\n", __func__);
	wakeup_fg_algo_cmd(gm, FG_INTR_FG_TIME, 0, 1);
}

static enum alarmtimer_restart one_percent_timer_callback(
	struct alarm *alarm, ktime_t now)
{
	struct mtk_battery *gm;

	gm = container_of(alarm,
		struct mtk_battery, one_percent_timer);
	bm_debug("[%s]\n", __func__);
	schedule_work(&gm->one_percent_timer_work);
	return ALARMTIMER_NORESTART;
}

static void sw_uisoc_timer_work_handler(struct work_struct *data)
{
	struct mtk_battery *gm;

	gm = container_of(data,
		struct mtk_battery, one_percent_timer_work);
	bm_debug("[%s] %d %d\n", __func__,
		gm->soc, gm->ui_soc);
	if (gm->soc > gm->ui_soc)
		wakeup_fg_algo(gm, FG_INTR_BAT_INT2_HT);
	else if (gm->soc < gm->ui_soc)
		wakeup_fg_algo(gm, FG_INTR_BAT_INT2_LT);
}

static enum alarmtimer_restart sw_uisoc_timer_callback(
	struct alarm *alarm, ktime_t now)
{
	struct mtk_battery *gm;

	gm = container_of(alarm,
		struct mtk_battery, sw_uisoc_timer);
	bm_debug("[%s]\n", __func__);
	schedule_work(&gm->sw_uisoc_timer_work);
	return ALARMTIMER_NORESTART;
}

/* ============================================================ */
/* power misc */
/* ============================================================ */
static void wake_up_power_misc(struct shutdown_controller *sdd)
{
	sdd->timeout = true;
	wake_up(&sdd->wait_que);
}

#ifndef OPLUS_FEATURE_CHG_BASIC
static void wake_up_overheat(struct shutdown_controller *sdd)
{
	sdd->overheat = true;
	wake_up(&sdd->wait_que);
}
#endif

void set_shutdown_vbat_lt(struct mtk_battery *gm, int vbat_lt, int vbat_lt_lv1)
{
	gm->sdc.vbat_lt = vbat_lt;
	gm->sdc.vbat_lt_lv1 = vbat_lt_lv1;
}

int get_shutdown_cond(struct mtk_battery *gm)
{
	int ret = 0;
	int vbat = 0;
	struct shutdown_controller *sdc;

	if (gm->disableGM30)
		vbat = 4000;
	else
		vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);

	if (gm->disableGM30)
		vbat = 4000;
	else
		vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);

	sdc = &gm->sdc;
	if (sdc->shutdown_status.is_soc_zero_percent)
		ret |= 1;
	if (sdc->shutdown_status.is_uisoc_one_percent)
		ret |= 1;
	if (sdc->lowbatteryshutdown)
		ret |= 1;
	bm_debug("%s ret:%d %d %d %d vbat:%d\n",
		__func__,
	ret, sdc->shutdown_status.is_soc_zero_percent,
	sdc->shutdown_status.is_uisoc_one_percent,
	sdc->lowbatteryshutdown, vbat);

	return ret;
}

void set_shutdown_cond_flag(struct mtk_battery *gm, int val)
{
	gm->sdc.shutdown_cond_flag = val;
}

int get_shutdown_cond_flag(struct mtk_battery *gm)
{
	return gm->sdc.shutdown_cond_flag;
}

int disable_shutdown_cond(struct mtk_battery *gm, int shutdown_cond)
{
	int now_current;
	int now_is_charging = 0;
	int now_is_kpoc = 0;
	struct shutdown_controller *sdc;
	int vbat = 0;

	sdc = &gm->sdc;
	now_current = gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT);
	now_is_kpoc = is_kernel_power_off_charging();

	if (gm->disableGM30)
		vbat = 4000;
	else
		vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);

	bm_debug("%s %d, is kpoc %d curr %d is_charging %d flag:%d lb:%d\n",
		__func__,
		shutdown_cond, now_is_kpoc, now_current, now_is_charging,
		sdc->shutdown_cond_flag,
		vbat);

	switch (shutdown_cond) {
#ifdef SHUTDOWN_CONDITION_LOW_BAT_VOLT
	case LOW_BAT_VOLT:
		sdc->shutdown_status.is_under_shutdown_voltage = false;
		sdc->lowbatteryshutdown = false;
		bm_debug("disable LOW_BAT_VOLT avgvbat %d ,threshold:%d %d %d\n",
		sdc->avgvbat,
		BAT_VOLTAGE_HIGH_BOUND,
		sdc->vbat_lt,
		sdc->vbat_lt_lv1);
		break;
#endif
	default:
		break;
	}
	return 0;
}

int set_shutdown_cond(struct mtk_battery *gm, int shutdown_cond)
{
	int now_current;
	int now_is_charging = 0;
	int now_is_kpoc = 0;
	int vbat = 0;
	struct shutdown_controller *sdc;
	struct shutdown_condition *sds;
	int enable_lbat_shutdown;

#ifdef SHUTDOWN_CONDITION_LOW_BAT_VOLT
	enable_lbat_shutdown = 1;
#else
	enable_lbat_shutdown = 0;
#endif

	now_current = gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT);
	now_is_kpoc = is_kernel_power_off_charging();

	if (gm->disableGM30)
		vbat = 4000;
	else
		vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);

	sdc = &gm->sdc;
	sds = &gm->sdc.shutdown_status;

	if (now_current >= 0)
		now_is_charging = 1;

	bm_debug("%s %d %d kpoc %d curr %d is_charging %d flag:%d lb:%d\n",
		__func__,
		shutdown_cond, enable_lbat_shutdown,
		now_is_kpoc, now_current, now_is_charging,
		sdc->shutdown_cond_flag, vbat);

	if (sdc->shutdown_cond_flag == 1)
		return 0;

	if (sdc->shutdown_cond_flag == 2 && shutdown_cond != LOW_BAT_VOLT)
		return 0;

	if (sdc->shutdown_cond_flag == 3 && shutdown_cond != DLPT_SHUTDOWN)
		return 0;

	switch (shutdown_cond) {
	case OVERHEAT:
		mutex_lock(&sdc->lock);
		sdc->shutdown_status.is_overheat = true;
		mutex_unlock(&sdc->lock);
		bm_debug("[%s]OVERHEAT shutdown!\n", __func__);
		kernel_power_off();
		break;
	case SOC_ZERO_PERCENT:
		if (sdc->shutdown_status.is_soc_zero_percent != true) {
			mutex_lock(&sdc->lock);
			if (now_is_kpoc != 1) {
				if (now_is_charging != 1) {
					sds->is_soc_zero_percent =
						true;

					sdc->pre_time[SOC_ZERO_PERCENT] =
						ktime_get_boottime();
					bm_debug("[%s]soc_zero_percent shutdown\n",
						__func__);
					wakeup_fg_algo(gm, FG_INTR_SHUTDOWN);
				}
			}
			mutex_unlock(&sdc->lock);
		}
		break;
	case UISOC_ONE_PERCENT:
		if (sdc->shutdown_status.is_uisoc_one_percent != true) {
			mutex_lock(&sdc->lock);
			if (now_is_kpoc != 1) {
				if (now_is_charging != 1) {
					sds->is_uisoc_one_percent =
						true;

					sdc->pre_time[UISOC_ONE_PERCENT] =
						ktime_get_boottime();

					bm_debug("[%s]uisoc 1 percent shutdown\n",
						__func__);
					wakeup_fg_algo(gm, FG_INTR_SHUTDOWN);
				}
			}
			mutex_unlock(&sdc->lock);
		}
		break;
#ifdef SHUTDOWN_CONDITION_LOW_BAT_VOLT
	case LOW_BAT_VOLT:
		if (sdc->shutdown_status.is_under_shutdown_voltage != true) {
			int i;

			mutex_lock(&sdc->lock);
			if (now_is_kpoc != 1) {
				sds->is_under_shutdown_voltage = true;
				for (i = 0; i < AVGVBAT_ARRAY_SIZE; i++)
					sdc->batdata[i] =
						VBAT2_DET_VOLTAGE1 / 10;
				sdc->batidx = 0;
			}
			bm_debug("LOW_BAT_VOLT:vbat %d %d",
				vbat, VBAT2_DET_VOLTAGE1 / 10);
			mutex_unlock(&sdc->lock);
		}
		break;
#endif
	case DLPT_SHUTDOWN:
		if (sdc->shutdown_status.is_dlpt_shutdown != true) {
			mutex_lock(&sdc->lock);
			sdc->shutdown_status.is_dlpt_shutdown = true;
			sdc->pre_time[DLPT_SHUTDOWN] = ktime_get_boottime();
			wakeup_fg_algo(gm, FG_INTR_DLPT_SD);
			mutex_unlock(&sdc->lock);
		}
		break;

	default:
		break;
	}

	wake_up_power_misc(sdc);

	return 0;
}

int next_waketime(int polling)
{
	if (polling <= 0)
		return 0;
	else
		return 10;
}

static int shutdown_event_handler(struct mtk_battery *gm)
{
	ktime_t now, duraction;
	struct timespec64 tmp_duraction;
	int polling = 0;
	static int ui_zero_time_flag;
	static int down_to_low_bat;
	int now_current = 0;
	int current_ui_soc = gm->ui_soc;
	int current_soc = gm->soc;
	int vbat = 0;
	int tmp = 25;
	struct shutdown_controller *sdd = &gm->sdc;

	tmp_duraction.tv_sec = 0;
	tmp_duraction.tv_nsec = 0;

	now = ktime_get_boottime();

	bm_debug("%s:soc_zero:%d,ui 1percent:%d,dlpt_shut:%d,under_shutdown_volt:%d\n",
		__func__,
		sdd->shutdown_status.is_soc_zero_percent,
		sdd->shutdown_status.is_uisoc_one_percent,
		sdd->shutdown_status.is_dlpt_shutdown,
		sdd->shutdown_status.is_under_shutdown_voltage);

	if (sdd->shutdown_status.is_soc_zero_percent) {
		if (current_ui_soc == 0) {
			duraction = ktime_sub(
				now, sdd->pre_time[SOC_ZERO_PERCENT]);

			tmp_duraction = ktime_to_timespec64(duraction);
			polling++;
			if (tmp_duraction.tv_sec >= SHUTDOWN_TIME) {
				bm_debug("soc zero shutdown\n");
				kernel_power_off();
				return next_waketime(polling);
			}
		} else if (current_soc > 0) {
			sdd->shutdown_status.is_soc_zero_percent = false;
		} else {
			/* ui_soc is not zero, check it after 10s */
			polling++;
		}
	}

	if (sdd->shutdown_status.is_uisoc_one_percent) {
		now_current = gauge_get_int_property(
			GAUGE_PROP_BATTERY_CURRENT);

		if (current_ui_soc == 0) {
			duraction =
				ktime_sub(
				now, sdd->pre_time[UISOC_ONE_PERCENT]);

			tmp_duraction = ktime_to_timespec64(duraction);
			if (tmp_duraction.tv_sec >= SHUTDOWN_TIME) {
				bm_debug("uisoc one percent shutdown\n");
				kernel_power_off();
				return next_waketime(polling);
			}
		} else if (now_current > 0 && current_soc > 0) {
			polling = 0;
			sdd->shutdown_status.is_uisoc_one_percent = 0;
			bm_debug("disable uisoc_one_percent shutdown cur:%d soc:%d\n",
				now_current, current_soc);
			return next_waketime(polling);
		}
		/* ui_soc is not zero, check it after 10s */
		polling++;

	}

	if (sdd->shutdown_status.is_dlpt_shutdown) {
		duraction = ktime_sub(now, sdd->pre_time[DLPT_SHUTDOWN]);
		tmp_duraction = ktime_to_timespec64(duraction);
		polling++;
		if (tmp_duraction.tv_sec >= SHUTDOWN_TIME) {
			bm_debug("dlpt shutdown count, %d\n",
				(int)tmp_duraction.tv_sec);
			return next_waketime(polling);
		}
	}

	if (sdd->shutdown_status.is_under_shutdown_voltage) {

		int vbatcnt = 0, i;

		if (gm->disableGM30)
			vbat = 4000;
		else
			vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);

		sdd->batdata[sdd->batidx] = vbat;

		for (i = 0; i < AVGVBAT_ARRAY_SIZE; i++)
			vbatcnt += sdd->batdata[i];
		sdd->avgvbat = vbatcnt / AVGVBAT_ARRAY_SIZE;
		tmp = force_get_tbat(gm, true);

		bm_debug("lbatcheck vbat:%d avgvbat:%d %d,%d tmp:%d,bound:%d,th:%d %d,en:%d\n",
			vbat,
			sdd->avgvbat,
			sdd->vbat_lt,
			sdd->vbat_lt_lv1,
			tmp,
			BAT_VOLTAGE_LOW_BOUND,
			LOW_TEMP_THRESHOLD,
			LOW_TMP_BAT_VOLTAGE_LOW_BOUND,
			LOW_TEMP_DISABLE_LOW_BAT_SHUTDOWN);

		if (sdd->avgvbat < BAT_VOLTAGE_LOW_BOUND) {
			/* avg vbat less than 3.4v */
			sdd->lowbatteryshutdown = true;
			polling++;

			if (down_to_low_bat == 0) {
				if (IS_ENABLED(
					LOW_TEMP_DISABLE_LOW_BAT_SHUTDOWN)) {
					if (tmp >= LOW_TEMP_THRESHOLD) {
						down_to_low_bat = 1;
						bm_debug("normal tmp, battery voltage is low shutdown\n");
						wakeup_fg_algo(gm,
							FG_INTR_SHUTDOWN);
					} else if (sdd->avgvbat <=
						LOW_TMP_BAT_VOLTAGE_LOW_BOUND) {
						down_to_low_bat = 1;
						bm_debug("cold tmp, battery voltage is low shutdown\n");
						wakeup_fg_algo(gm,
							FG_INTR_SHUTDOWN);
					} else
						bm_debug("low temp disable low battery sd\n");
				} else {
					down_to_low_bat = 1;
					bm_debug("[%s]avg vbat is low to shutdown\n",
						__func__);
					wakeup_fg_algo(gm, FG_INTR_SHUTDOWN);
				}
			}

			if ((current_ui_soc == 0) && (ui_zero_time_flag == 0)) {
				sdd->pre_time[LOW_BAT_VOLT] =
					ktime_get_boottime();
				ui_zero_time_flag = 1;
			}

			if (current_ui_soc == 0) {
				duraction = ktime_sub(
					now, sdd->pre_time[LOW_BAT_VOLT]);

				tmp_duraction  = ktime_to_timespec64(duraction);
				ui_zero_time_flag = 1;
				if (tmp_duraction.tv_sec >= SHUTDOWN_TIME) {
					bm_debug("low bat shutdown, over %d second\n",
						SHUTDOWN_TIME);
					kernel_power_off();
					return next_waketime(polling);
				}
			}
		} else {
			/* greater than 3.4v, clear status */
			down_to_low_bat = 0;
			ui_zero_time_flag = 0;
			sdd->pre_time[LOW_BAT_VOLT] = 0;
			sdd->lowbatteryshutdown = false;
			polling++;
		}

		polling++;
			bm_debug("[%s][UT] V %d ui_soc %d dur %d [%d:%d:%d:%d] batdata[%d] %d\n",
				__func__,
			sdd->avgvbat, current_ui_soc,
			(int)tmp_duraction.tv_sec,
			down_to_low_bat, ui_zero_time_flag,
			(int)sdd->pre_time[LOW_BAT_VOLT],
			sdd->lowbatteryshutdown,
			sdd->batidx, sdd->batdata[sdd->batidx]);

		sdd->batidx++;
		if (sdd->batidx >= AVGVBAT_ARRAY_SIZE)
			sdd->batidx = 0;
	}

	bm_debug(
		"%s %d avgvbat:%d sec:%d lowst:%d\n",
		__func__,
		polling, sdd->avgvbat,
		(int)tmp_duraction.tv_sec, sdd->lowbatteryshutdown);

	return next_waketime(polling);

}

static enum alarmtimer_restart power_misc_kthread_fgtimer_func(
	struct alarm *alarm, ktime_t now)
{
	struct shutdown_controller *info =
		container_of(
			alarm, struct shutdown_controller, kthread_fgtimer);

	wake_up_power_misc(info);
	return ALARMTIMER_NORESTART;
}

static void power_misc_handler(void *arg)
{
	struct mtk_battery *gm = arg;
	struct shutdown_controller *sdd = &gm->sdc;
	struct timespec64 end_time, tmp_time_now;
	ktime_t ktime, time_now;
	int secs = 0;

	secs = shutdown_event_handler(gm);
	if (secs != 0 && gm->disableGM30 == false) {
		time_now  = ktime_get_boottime();
		tmp_time_now  = ktime_to_timespec64(time_now);
		end_time.tv_sec = tmp_time_now.tv_sec + secs;
		ktime = ktime_set(end_time.tv_sec, end_time.tv_nsec);

		alarm_start(&sdd->kthread_fgtimer, ktime);
		bm_debug("%s:set new alarm timer:%ds\n",
			__func__, secs);
	}
}

static int power_misc_routine_thread(void *arg)
{
	struct mtk_battery *gm = arg;
	struct shutdown_controller *sdd = &gm->sdc;
	int ret = 0;

	while (1) {
		ret = wait_event_interruptible(sdd->wait_que, (sdd->timeout == true)
			|| (sdd->overheat == true));
		if (sdd->timeout == true) {
			sdd->timeout = false;
			power_misc_handler(gm);
		}
		if (sdd->overheat == true) {
			sdd->overheat = false;
			bm_err("%s battery overheat~ power off, ret = %d\n",
				__func__, ret);
			kernel_power_off();
			return 1;
		}
	}

	return 0;
}

#ifndef OPLUS_FEATURE_CHG_BASIC
static int mtk_power_misc_psy_event(
	struct notifier_block *nb, unsigned long event, void *v)
{
	struct power_supply *psy = v;
	struct shutdown_controller *sdc;
	struct mtk_battery *gm;
	int tmp = 0;

	gm = get_mtk_battery();

	if (strcmp(psy->desc->name, "battery") == 0) {
		if (gm != NULL) {
			sdc = container_of(
				nb, struct shutdown_controller, psy_nb);

			if (gm->cur_bat_temp >= BATTERY_SHUTDOWN_TEMPERATURE) {
				bm_debug(
					"%d battery temperature >= %d,shutdown",
					gm->cur_bat_temp, tmp);
				wake_up_overheat(sdc);
			}
		}
	}

	return NOTIFY_DONE;
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

void mtk_power_misc_init(struct mtk_battery *gm)
{
	mutex_init(&gm->sdc.lock);
	alarm_init(&gm->sdc.kthread_fgtimer, ALARM_BOOTTIME,
		power_misc_kthread_fgtimer_func);
	init_waitqueue_head(&gm->sdc.wait_que);

	if (!gm->disableGM30)
		kthread_run(power_misc_routine_thread, gm, "power_misc_thread");

#ifndef OPLUS_FEATURE_CHG_BASIC
	gm->sdc.psy_nb.notifier_call = mtk_power_misc_psy_event;
	power_supply_reg_notifier(&gm->sdc.psy_nb);
#endif
}

int battery_psy_init(struct platform_device *pdev)
{
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;
	int ret;

	bm_err("[%s]\n", __func__);
	gm = devm_kzalloc(&pdev->dev, sizeof(*gm), GFP_KERNEL);
	if (!gm)
		return -ENOMEM;

	gauge = dev_get_drvdata(&pdev->dev);
	gauge->gm = gm;
	gm->gauge = gauge;
	mutex_init(&gm->ops_lock);

	gm->bs_data.chg_psy = devm_power_supply_get_by_phandle(&pdev->dev,
							 "charger");
	if (IS_ERR_OR_NULL(gm->bs_data.chg_psy))
		bm_err("[BAT_probe] %s: fail to get chg_psy !!\n", __func__);

	battery_service_data_init(gm);
	gm->bs_data.psy =
		power_supply_register(
			&(pdev->dev), &gm->bs_data.psd, &gm->bs_data.psy_cfg);
	if (IS_ERR(gm->bs_data.psy)) {
		bm_err("[BAT_probe] power_supply_register Battery Fail !!\n");
		ret = PTR_ERR(gm->bs_data.psy);
		return ret;
	}
	bm_err("[BAT_probe] power_supply_register Battery Success !!\n");
	return 0;
}

void fg_check_bootmode(struct device *dev,
	struct mtk_battery *gm)
{
	struct device_node *boot_node = NULL;
	struct tag_bootmode *tag = NULL;

	boot_node = of_parse_phandle(dev->of_node, "bootmode", 0);
	if (!boot_node)
		bm_err("%s: failed to get boot mode phandle\n", __func__);
	else {
		tag = (struct tag_bootmode *)of_get_property(boot_node,
							"atag,boot", NULL);
		if (!tag)
			bm_err("%s: failed to get atag,boot\n", __func__);
		else {
			bm_err("%s: size:0x%x tag:0x%x bootmode:0x%x boottype:0x%x\n",
				__func__, tag->size, tag->tag,
				tag->bootmode, tag->boottype);
			gm->bootmode = tag->bootmode;
			gm->boottype = tag->boottype;
		}
	}
}

void fg_check_lk_swocv(struct device *dev,
	struct mtk_battery *gm)
{
	struct device_node *boot_node = NULL;
	int len = 0;
	char temp[10];
	int *prop;

	boot_node = of_parse_phandle(dev->of_node, "bootmode", 0);
	if (!boot_node)
		bm_err("%s: failed to get boot mode phandle\n", __func__);
	else {
		prop = (void *)of_get_property(
			boot_node, "atag,fg_swocv_v", &len);

		if (prop == NULL) {
			bm_err("fg_swocv_v prop == NULL, len=%d\n", len);
		} else {
			snprintf(temp, (len + 1), "%s", prop);
			kstrtoint(temp, 10, &gm->ptim_lk_v);
			bm_err("temp %s gm->ptim_lk_v=%d\n",
				temp, gm->ptim_lk_v);
		}

		prop = (void *)of_get_property(
			boot_node, "atag,fg_swocv_i", &len);

		if (prop == NULL) {
			bm_err("fg_swocv_i prop == NULL, len=%d\n", len);
		} else {
			snprintf(temp, (len + 1), "%s", prop);
			kstrtoint(temp, 10, &gm->ptim_lk_i);
			bm_err("temp %s gm->ptim_lk_i=%d\n",
				temp, gm->ptim_lk_i);
		}
		prop = (void *)of_get_property(
			boot_node, "atag,shutdown_time", &len);

		if (prop == NULL) {
			bm_err("shutdown_time prop == NULL, len=%d\n", len);
		} else {
			snprintf(temp, (len + 1), "%s", prop);
			kstrtoint(temp, 10, &gm->pl_shutdown_time);
			bm_err("temp %s gm->pl_shutdown_time=%d\n",
				temp, gm->pl_shutdown_time);
		}
	}

	bm_err("%s swocv_v:%d swocv_i:%d shutdown_time:%d\n",
		__func__, gm->ptim_lk_v, gm->ptim_lk_i, gm->pl_shutdown_time);
}

#ifdef OPLUS_FEATURE_CHG_BASIC

static bool battery_type_is_4450mv(void)
{
	int battery_type = BAT_TYPE__UNKNOWN;
	int retry_flag = 0;

try_again:
	battery_type_check(&battery_type);
	if (battery_type == BAT_TYPE__ATL_4450mV || battery_type == BAT_TYPE__XWD_4450mV) {
		return true;
	} else {
		if (retry_flag == 0) {
			retry_flag = 1;
			goto try_again;
		}
		//if (is_meta_mode() == true) {
		//	return false;
		//} else {
			return false;
		//}
	}
}

static bool battery_type_is_4400mv(void)
{
	int battery_type = BAT_TYPE__UNKNOWN;
	int retry_flag = 0;

try_again:
	battery_type_check(&battery_type);
	if (battery_type == BAT_TYPE__SDI_4400mV || battery_type == BAT_TYPE__ATL_4400mV) {
		return true;
	} else {
		if (retry_flag == 0) {
			retry_flag = 1;
			goto try_again;
		}
		//if (is_meta_mode() == true) {
		//	return false;
		//} else {
			return false;
		//}
	}
}
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
bool is_battery_init_done(void)
{
	return oplus_gm->is_probe_done;
}

int oplus_battery_get_bat_temperature(void)
{
	if (is_battery_init_done()) {
#if (defined (CONFIG_OPLUS_CHARGER_MTK6789S))
		force_get_tbat(oplus_gm, true);
		return oplus_gm->tbat_precise;
#else
		return (force_get_tbat(oplus_gm, true) * 10);
#endif
	} else {
		return -1270;
	}
}

static int meter_fg_30_get_battery_mvolts(void)
{
	int bat_volt = 0;

	bat_volt = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);
	bat_volt -= 10;
	return bat_volt;
}

static int meter_fg_30_get_battery_temperature(void)
{
	int bat_temperature = 0;
	if(prj_is_subboard_temp_support()) {
		if (oplus_chg_get_voocphy_support()) {
			bat_temperature = oplus_force_get_subboard_temp();
		}
	} else {
		bat_temperature = oplus_battery_get_bat_temperature();
	}
	
	return bat_temperature;
}

static int meter_fg_30_get_batt_remaining_capacity(void)
{
	return oplus_gm->prev_batt_remaining_capacity;
}

static int meter_fg_30_get_battery_soc(void)
{
	if(oplus_gm->init_flag == 1) {
		return oplus_gm->ui_soc;
	} else {
		return -1;
	}
}

static int meter_fg_30_get_average_current(void)
{
	int bat_current = 0;
	
	bat_current= gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT);
	bat_current = 0 - bat_current / 10;

	return bat_current;
}

static int meter_fg_30_get_prev_battery_fcc(void)
{
		return (oplus_gm->prev_batt_fcc)/10;
}

static int meter_fg_30_get_battery_fcc(void)
{
	return (oplus_gm->prev_batt_fcc)/10;
}

static int meter_fg_30_get_battery_cc(void)
{
	return -1;
}

static int meter_fg_30_get_battery_soh(void)
{
	return -1;
}

static int meter_fg_30_get_prev_batt_remaining_capacity(void)
{
	return oplus_gm->prev_batt_remaining_capacity;
}

static int meter_fg_30_modify_dod0(void)
{
	return -1;
}

static int meter_fg_30_update_soc_smooth_parameter(void)
{
	return -1;
}

static void meter_fg_30_set_battery_full(bool full)
{
	printk("last full = %d, full = %d\n", last_full, full);
	if(last_full != full) {
		if (full) {
			if(enable_is_force_full == 1)
				oplus_gm->is_force_full = true;
			wakeup_fg_algo(oplus_gm, FG_INTR_CHR_FULL);
			fg_int_event(oplus_gm, EVT_INT_CHR_FULL);
		}
		last_full = full;
	}
}

bool meter_fg_30_get_battery_authenticate(void)
{
	int battery_id = 0;
#ifndef CONFIG_OPLUS_CHARGER_MTK6789S
	return true;//debug temp
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
	battery_id = fgauge_get_profile_id();

    if(battery_id == 2 || battery_id == BAT_XINWANG_BATT_ID) {
		return battery_type_is_4450mv();
    }
	else
	return battery_type_is_4400mv();
#endif
}

#ifdef OPLUS_FEATURE_CHG_BASIC
/* BSP.CHG.basic, 2022/02/28, Add for charger */
static bool meter_set_gauge_power_sel(int sel)
{
        enum charge_sel chgsel = CHARGE_NORMAL;

        if ((sel >= CHARGE_NORMAL) && (sel <= CHARGE_R4))
                chgsel = (enum charge_sel)sel;
        return set_charge_power_sel(chgsel);
}
#endif

static struct oplus_gauge_operations oplus_battery_gauge = {
	.get_battery_mvolts 		= meter_fg_30_get_battery_mvolts,
	.get_battery_temperature		= meter_fg_30_get_battery_temperature,
	.get_batt_remaining_capacity	= meter_fg_30_get_batt_remaining_capacity,
	.get_battery_soc				= meter_fg_30_get_battery_soc,
	.get_average_current			= meter_fg_30_get_average_current,
	.get_battery_fcc				= meter_fg_30_get_battery_fcc,
	.get_battery_cc 			= meter_fg_30_get_battery_cc,
	.get_battery_soh				= meter_fg_30_get_battery_soh,
	.get_battery_authenticate		= meter_fg_30_get_battery_authenticate,
	.set_battery_full				= meter_fg_30_set_battery_full,
	.get_prev_battery_mvolts		= meter_fg_30_get_battery_mvolts,
	.get_prev_battery_temperature	= meter_fg_30_get_battery_temperature,
	.get_prev_battery_soc			= meter_fg_30_get_battery_soc,
	.get_prev_average_current		= meter_fg_30_get_average_current,
	.get_prev_batt_remaining_capacity	= meter_fg_30_get_prev_batt_remaining_capacity,
	.get_battery_mvolts_2cell_max		= meter_fg_30_get_battery_mvolts,
	.get_battery_mvolts_2cell_min		= meter_fg_30_get_battery_mvolts,
	.get_prev_battery_mvolts_2cell_max	= meter_fg_30_get_battery_mvolts,
	.get_prev_battery_mvolts_2cell_min	= meter_fg_30_get_battery_mvolts,
	.get_prev_batt_fcc				= meter_fg_30_get_prev_battery_fcc,
	.update_battery_dod0				= meter_fg_30_modify_dod0,
	.update_soc_smooth_parameter		= meter_fg_30_update_soc_smooth_parameter,
	.set_gauge_power_sel				= meter_set_gauge_power_sel,
};
#endif

int battery_init(struct platform_device *pdev)
{
	int ret = 0;
	bool b_recovery_mode = 0;
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct oplus_gauge_chip *chip = NULL;
#endif
	gauge = dev_get_drvdata(&pdev->dev);
	gm = gauge->gm;
	gm->fixed_bat_tmp = 0xffff;
	gm->tmp_table = fg_temp_table;
	gm->log_level = BMLOG_ERROR_LEVEL;
	gm->sw_iavg_gap = 3000;
	gm->in_sleep = false;
#ifdef OPLUS_FEATURE_CHG_BASIC
	fg_read_dts_val(pdev->dev.of_node, "FUELGAGUE_APPLY", &(fuelgauge_apply), 1);
	bm_err("%s, fuelgauge_apply:%d\n", __func__, fuelgauge_apply);

	fg_read_dts_val(pdev->dev.of_node, "IS_SUBBOARD_TEMP_SUPPORT", &(is_subboard_temp_support), 1);
	bm_err("%s, is_subboard_temp_support:%d\n", __func__, is_subboard_temp_support);

	fg_read_dts_val(pdev->dev.of_node, "Enable_Is_Force_Full", &(enable_is_force_full), 1);
	bm_err("%s, enable_is_force_full:%d\n", __func__, enable_is_force_full);

	fg_read_dts_val(pdev->dev.of_node, "BATT_ID_FAST_CHECK", &(batt_id_fast_chcek), 1);
	bm_err("%s, batt_id_fast_chcek:%d\n", __func__, batt_id_fast_chcek);

	fg_read_dts_val(pdev->dev.of_node, "NTC_SWITCH_NOT_USE", &(g_ntc_switch_not_use), 1);
	bm_err("%s, batt_id_fast_chcek:%d\n", __func__, g_ntc_switch_not_use);

	g_switch_ntc = of_get_named_gpio(pdev->dev.of_node, "ntc_switch_gpio", 0);
	if (g_switch_ntc < 0) {
		pr_err("ntc_switch_gpio < 0 !!!\r\n");
		g_switch_ntc = 0;
	}

	if(gpio_request(g_switch_ntc, "NTC_SWITCH_GPIO") < 0) {
		pr_err("ntc_switch_gpio gpio_request fail\r\n");
	}

	if(is_fuelgauge_apply() == true) {
#ifdef CONFIG_OPLUS_CHARGER_MTK6789S
		batt_id = devm_iio_channel_get(&pdev->dev, "auxadc4-batt_id_v");
#else
		batt_id = devm_iio_channel_get(&pdev->dev, "auxadc6-batt_id_v");
#endif
		if (IS_ERR(batt_id)){
			bm_err("battery ID CHANNEL ERR \n");
			batt_id = NULL;
		}
	}

#endif
	mutex_init(&gm->fg_update_lock);

	init_waitqueue_head(&gm->wait_que);

	fg_check_bootmode(&pdev->dev, gm);
	fg_check_lk_swocv(&pdev->dev, gm);
	fg_custom_init_from_header(gm);
	fg_custom_init_from_dts(pdev, gm);
	gauge_coulomb_service_init(gm);
	gm->coulomb_plus.callback = fg_coulomb_int_h_handler;
	gauge_coulomb_consumer_init(&gm->coulomb_plus, &pdev->dev, "car+1%");
	gm->coulomb_minus.callback = fg_coulomb_int_l_handler;
	gauge_coulomb_consumer_init(&gm->coulomb_minus, &pdev->dev, "car-1%");

	gauge_coulomb_consumer_init(&gm->uisoc_plus, &pdev->dev, "uisoc+1%");
	gm->uisoc_plus.callback = fg_bat_int2_h_handler;
	gauge_coulomb_consumer_init(&gm->uisoc_minus, &pdev->dev, "uisoc-1%");
	gm->uisoc_minus.callback = fg_bat_int2_l_handler;



	alarm_init(&gm->tracking_timer, ALARM_BOOTTIME,
		tracking_timer_callback);
	INIT_WORK(&gm->tracking_timer_work, tracking_timer_work_handler);
	alarm_init(&gm->one_percent_timer, ALARM_BOOTTIME,
		one_percent_timer_callback);
	INIT_WORK(&gm->one_percent_timer_work, one_percent_timer_work_handler);

	alarm_init(&gm->sw_uisoc_timer, ALARM_BOOTTIME,
		sw_uisoc_timer_callback);
	INIT_WORK(&gm->sw_uisoc_timer_work, sw_uisoc_timer_work_handler);

	if (!gm->disableGM30)
		kthread_run(battery_update_routine, gm, "battery_thread");

#ifdef CONFIG_PM
	gm->pm_nb.notifier_call = system_pm_notify;
	ret = register_pm_notifier(&gm->pm_nb);
	if (ret) {
		bm_err("%s failed to register system pm notify\n", __func__);
		unregister_pm_notifier(&gm->pm_nb);
	}
#endif /* CONFIG_PM */

	fg_drv_thread_hrtimer_init(gm);
	battery_sysfs_create_group(gm->bs_data.psy);

	/* for gauge hal hw ocv */
	gm->bs_data.bat_batt_temp = force_get_tbat(gm, true);
	mtk_power_misc_init(gm);

	ret = mtk_battery_daemon_init(pdev);
	b_recovery_mode = is_recovery_mode();
	gm->is_probe_done = true;

#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_gm = gm;
	printk(KERN_ERR "!!!!! oplus_gm is ready\n");
#endif

	if (ret == 0 && b_recovery_mode == 0)
		bm_err("[%s]: daemon mode DONE\n", __func__);
	else {
		gm->algo.active = true;
		battery_algo_init(gm);
		bm_err("[%s]: enable Kernel mode Gauge\n", __func__);
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	if(is_fuelgauge_apply() == true) {
		chip = (struct oplus_gauge_chip*) kzalloc(sizeof(struct oplus_gauge_chip),
					GFP_KERNEL);
		if (!chip) {
			pr_err("oplus_gauge_chip devm_kzalloc failed.\n");
			return -ENOMEM;
		}
	}
	if(is_fuelgauge_apply() == true) {
		chip->gauge_ops = &oplus_battery_gauge;
		oplus_gauge_init(chip);
		chip->gauge_ops->get_battery_authenticate();
	}
#endif

	return 0;
}

