// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author Wy Chuang<wy.chuang@mediatek.com>
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/mfd/mt6359p/registers.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/socket.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <net/sock.h>
#include "mtk_battery.h"
#include "mtk_gauge.h"

/* ============================================================ */
/* pmic control start*/
/* ============================================================ */

/* mt6359 610.352 uA */
#define UNIT_FGCURRENT			(610352)
/* CHARGE_LSB 0.085 uAh*/
#define UNIT_CHARGE			(85)

/* AUXADC */
#define R_VAL_TEMP_2			(25)
#define R_VAL_TEMP_3			(35)

#define UNIT_TIME			(50)
#define UNIT_FG_IAVG			(305176)
/* IAVG LSB: 305.176 uA */
#define DEFAULT_R_FG			(50)
/* 5mm ohm */
#define UNIT_FGCAR_ZCV			(85)
/* CHARGE_LSB = 0.085 uAh */

#define VOLTAGE_FULL_RANGES		1800
#define ADC_PRECISE				32768	/* 15 bits */

#define CAR_TO_REG_SHIFT		(5)
/*coulomb interrupt lsb might be different with coulomb lsb */
#define CAR_TO_REG_FACTOR		(0x2E14)
/* 1000 * 1000 / CHARGE_LSB */
#define UNIT_FGCAR				(174080)
/* CHARGE_LSB 0.085 * 2^11 */


/************ bat_cali *******************/
#define BAT_CALI_DEVNAME "MT_pmic_adc_cali"
/* add for meta tool----------------------------------------- */
#define Get_META_BAT_VOL _IOW('k', 10, int)
#define Get_META_BAT_SOC _IOW('k', 11, int)
#define Get_META_BAT_CAR_TUNE_VALUE _IOW('k', 12, int)
#define Set_META_BAT_CAR_TUNE_VALUE _IOW('k', 13, int)
#define Set_BAT_DISABLE_NAFG _IOW('k', 14, int)
#define Set_CARTUNE_TO_KERNEL _IOW('k', 15, int)

#define MT6359_AUXADC_BAT_TEMP_1	0x1228
#define PMIC_AUXADC_BAT_TEMP_FROZE_EN_ADDR	\
	MT6359_AUXADC_BAT_TEMP_1
#define PMIC_AUXADC_BAT_TEMP_FROZE_EN_MASK	0x1
#define PMIC_AUXADC_BAT_TEMP_FROZE_EN_SHIFT	0

static struct class *bat_cali_class;
static int bat_cali_major;
static dev_t bat_cali_devno;
static struct cdev *bat_cali_cdev;



void __attribute__ ((weak))
	mtk_battery_netlink_handler(struct sk_buff *skb)
{
}

static signed int reg_to_mv_value(signed int _reg)
{
	long long _reg64 = _reg;
	int ret;

#if defined(__LP64__) || defined(_LP64)
	_reg64 = (_reg64 * VOLTAGE_FULL_RANGES
		* R_VAL_TEMP_3) / ADC_PRECISE;
#else
	_reg64 = div_s64(_reg64 * VOLTAGE_FULL_RANGES
		* R_VAL_TEMP_3, ADC_PRECISE);
#endif
	ret = _reg64;
	bm_debug("[%s] %lld => %d\n",
		__func__, _reg64, ret);
	return ret;
}

static signed int mv_to_reg_value(signed int _mv)
{
	int ret;
	long long _reg64 = _mv;
#if defined(__LP64__) || defined(_LP64)
	_reg64 = (_reg64 * ADC_PRECISE) / (VOLTAGE_FULL_RANGES
		* R_VAL_TEMP_3);
#else
	_reg64 = div_s64((_reg64 * ADC_PRECISE), (VOLTAGE_FULL_RANGES
		* R_VAL_TEMP_3));
#endif
	ret = _reg64;

	if (ret <= 0) {
		bm_err(
			"[fg_bat_nafg][%s] mv=%d,%lld => %d,\n",
			__func__, _mv, _reg64, ret);
		return ret;
	}

	bm_debug("[%s] mv=%d,%lld => %d,\n", __func__, _mv, _reg64, ret);
	return ret;
}

static int mv_to_reg_12_temp_value(signed int _reg)
{
	int ret = (_reg * 4096) / (VOLTAGE_FULL_RANGES * R_VAL_TEMP_2);

	bm_debug("[%s] %d => %d\n", __func__, _reg, ret);
	return ret;
}

static void pre_gauge_update(struct mtk_gauge *gauge)
{
	int m = 0;
	unsigned int reg_val = 0;

	regmap_update_bits(gauge->regmap,
		PMIC_FG_SW_READ_PRE_ADDR,
		PMIC_FG_SW_READ_PRE_MASK << PMIC_FG_SW_READ_PRE_SHIFT,
		1 << PMIC_FG_SW_READ_PRE_SHIFT);
	do {
		m++;
		if (m > 1000) {
			bm_err("[%s] gauge_update_polling timeout 1!\r\n",
				__func__);
			break;
		}
		regmap_read(gauge->regmap, PMIC_FG_LATCHDATA_ST_ADDR, &reg_val);
		reg_val =
			(reg_val & (PMIC_FG_LATCHDATA_ST_MASK
			<< PMIC_FG_LATCHDATA_ST_SHIFT))
			>> PMIC_FG_LATCHDATA_ST_SHIFT;
	} while (reg_val == 0);
}

void disable_all_irq(struct mtk_battery *gm)
{
	disable_gauge_irq(gm->gauge, COULOMB_H_IRQ);
	disable_gauge_irq(gm->gauge, COULOMB_L_IRQ);
	disable_gauge_irq(gm->gauge, VBAT_H_IRQ);
	disable_gauge_irq(gm->gauge, VBAT_L_IRQ);
	disable_gauge_irq(gm->gauge, NAFG_IRQ);
	disable_gauge_irq(gm->gauge, BAT_PLUGOUT_IRQ);
	disable_gauge_irq(gm->gauge, ZCV_IRQ);
	disable_gauge_irq(gm->gauge, FG_N_CHARGE_L_IRQ);
	disable_gauge_irq(gm->gauge, FG_IAVG_H_IRQ);
	disable_gauge_irq(gm->gauge, FG_IAVG_L_IRQ);
	disable_gauge_irq(gm->gauge, BAT_TMP_H_IRQ);
	disable_gauge_irq(gm->gauge, BAT_TMP_L_IRQ);
}

static void post_gauge_update(struct mtk_gauge *gauge)
{
	int m = 0;
	unsigned int reg_val;

	regmap_update_bits(gauge->regmap,
		PMIC_FG_SW_CLEAR_ADDR,
		PMIC_FG_SW_CLEAR_MASK << PMIC_FG_SW_CLEAR_SHIFT,
		1 << PMIC_FG_SW_CLEAR_SHIFT);
	regmap_update_bits(gauge->regmap,
		PMIC_FG_SW_READ_PRE_ADDR,
		PMIC_FG_SW_READ_PRE_MASK << PMIC_FG_SW_READ_PRE_SHIFT,
		0 << PMIC_FG_SW_READ_PRE_SHIFT);

	do {
		m++;
		if (m > 1000) {
			bm_err("[%s] gauge_update_polling timeout 2!\r\n",
				__func__);
			break;
		}
		regmap_read(gauge->regmap, PMIC_FG_LATCHDATA_ST_ADDR, &reg_val);
		reg_val =
			(reg_val & (PMIC_FG_LATCHDATA_ST_MASK
			<< PMIC_FG_LATCHDATA_ST_SHIFT))
			>> PMIC_FG_LATCHDATA_ST_SHIFT;
	} while (reg_val != 0);

	regmap_update_bits(gauge->regmap,
		PMIC_FG_SW_CLEAR_ADDR,
		PMIC_FG_SW_CLEAR_MASK << PMIC_FG_SW_CLEAR_SHIFT,
		0 << PMIC_FG_SW_CLEAR_SHIFT);
}

static int mv_to_reg_12_value(struct mtk_gauge *gauge,
	signed int _reg)
{
	int ret = (_reg * 4096) / (VOLTAGE_FULL_RANGES * R_VAL_TEMP_3);

	bm_debug("[%s] %d => %d\n", __func__, _reg, ret);
	return ret;
}

static int reg_to_current(struct mtk_gauge *gauge,
	unsigned int regval)
{
	unsigned short uvalue16 = 0;
	int dvalue, retval;
	long long temp_value = 0;
	bool is_charging = true;

	uvalue16 = (unsigned short) regval;

	dvalue = (unsigned int) uvalue16;
	if (dvalue == 0) {
		temp_value = (long long) dvalue;
		is_charging = false;
	} else if (dvalue > 32767) {
		/* > 0x8000 */
		temp_value = (long long) (dvalue - 65535);
		temp_value = temp_value - (temp_value * 2);
		is_charging = false;
	} else {
		temp_value = (long long) dvalue;
	}

	temp_value = temp_value * UNIT_FGCURRENT;
#if defined(__LP64__) || defined(_LP64)
	do_div(temp_value, 100000);
#else
	temp_value = div_s64(temp_value, 100000);
#endif
	retval = (unsigned int) temp_value;

	bm_debug("[%s] 0x%x 0x%x 0x%x 0x%x 0x%x %d\n",
		__func__,
		regval,
		uvalue16,
		dvalue,
		(int)temp_value,
		retval,
		is_charging);

	if (is_charging == false)
		return -retval;

	return retval;
}

/* ============================================================ */
/* pmic control end*/
/* ============================================================ */
u8 get_rtc_spare0_fg_value(struct mtk_gauge *gauge)
{
	struct nvmem_cell *cell;
	u8 *buf, data;

	cell = nvmem_cell_get(&gauge->pdev->dev, "initialization");
	if (IS_ERR(cell)) {
		bm_err("[%s]get rtc cell fail\n", __func__);
		return 0;
	}

	buf = nvmem_cell_read(cell, NULL);
	nvmem_cell_put(cell);
	if (IS_ERR(buf)) {
		bm_err("[%s]read rtc cell fail\n", __func__);
		return 0;
	}
	bm_debug("[%s] val=0x%x, %d\n", __func__, *buf, *buf);

	data = *buf;
	kfree(buf);

	return data;
}

void set_rtc_spare0_fg_value(struct mtk_gauge *gauge, u8 val)
{
	struct nvmem_cell *cell;
	u32 length = 1;
	int ret;

	cell = nvmem_cell_get(&gauge->pdev->dev, "initialization");
	if (IS_ERR(cell)) {
		bm_err("[%s]get rtc cell fail\n", __func__);
		return;
	}

	ret = nvmem_cell_write(cell, &val, length);
	nvmem_cell_put(cell);
	if (ret != length)
		bm_err("[%s] write rtc cell fail\n", __func__);
}

u8 get_rtc_spare_fg_value(struct mtk_gauge *gauge)
{
	struct nvmem_cell *cell;
	u8 *buf, data;

	cell = nvmem_cell_get(&gauge->pdev->dev, "state-of-charge");
	if (IS_ERR(cell)) {
		bm_err("[%s]get rtc cell fail\n", __func__);
		return 0;
	}

	buf = nvmem_cell_read(cell, NULL);
	nvmem_cell_put(cell);

	if (IS_ERR(buf)) {
		bm_err("[%s]read rtc cell fail\n", __func__);
		return 0;
	}

	bm_debug("[%s] val=%d\n", __func__, *buf);
	data = *buf;
	kfree(buf);

	return data;
}

void set_rtc_spare_fg_value(struct mtk_gauge *gauge, u8 val)
{
	struct nvmem_cell *cell;
	u32 length = 1;
	int ret;

	cell = nvmem_cell_get(&gauge->pdev->dev, "state-of-charge");
	if (IS_ERR(cell)) {
		bm_err("[%s]get rtc cell fail\n", __func__);
		return;
	}

	ret = nvmem_cell_write(cell, &val, length);
	nvmem_cell_put(cell);

	if (ret != length)
		bm_err("[%s] write rtc cell fail\n", __func__);

	bm_debug("[%s] val=%d\n", __func__, val);
}

static int fgauge_set_info(struct mtk_gauge *gauge,
	enum gauge_property ginfo, unsigned int value)
{

	bm_debug("[%s]info:%d v:%d\n", __func__, ginfo, value);

	if (ginfo == GAUGE_PROP_2SEC_REBOOT)
		regmap_update_bits(gauge->regmap,
		PMIC_RG_SYSTEM_INFO_CON0_ADDR,
		0x0001,
		value);
	else if (ginfo == GAUGE_PROP_PL_CHARGING_STATUS)
		regmap_update_bits(gauge->regmap,
		PMIC_RG_SYSTEM_INFO_CON0_ADDR,
		0x0001 << 0x1,
		value << 0x1);
	else if (ginfo == GAUGE_PROP_MONITER_PLCHG_STATUS)
		regmap_update_bits(gauge->regmap,
		PMIC_RG_SYSTEM_INFO_CON0_ADDR,
		0x0001 << 0x2,
		value << 0x2);
	else if (ginfo == GAUGE_PROP_BAT_PLUG_STATUS)
		regmap_update_bits(gauge->regmap,
		PMIC_RG_SYSTEM_INFO_CON0_ADDR,
		0x0001 << 0x3,
		value << 0x3);
	else if (ginfo == GAUGE_PROP_IS_NVRAM_FAIL_MODE)
		regmap_update_bits(gauge->regmap,
		PMIC_RG_SYSTEM_INFO_CON0_ADDR,
		0x0001 << 0x4,
		value << 0x4);
	else if (ginfo == GAUGE_PROP_MONITOR_SOFF_VALIDTIME)
		regmap_update_bits(gauge->regmap,
		PMIC_RG_SYSTEM_INFO_CON0_ADDR,
		0x0001 << 0x5,
		value << 0x5);
	else if (ginfo == GAUGE_PROP_CON0_SOC) {
		value = value / 100;
		regmap_update_bits(gauge->regmap,
		PMIC_RG_SYSTEM_INFO_CON0_ADDR,
		0x007f << 0x9,
		value << 0x9);
	}
	return 0;
}

static int fgauge_get_info(struct mtk_gauge *gauge,
	enum gauge_property ginfo, int *value)
{
	int reg_val = 0;

	regmap_read(gauge->regmap, PMIC_RG_SYSTEM_INFO_CON0_ADDR, &reg_val);

	if (ginfo == GAUGE_PROP_2SEC_REBOOT)
		*value = reg_val & 0x0001;
	else if (ginfo == GAUGE_PROP_PL_CHARGING_STATUS)
		*value =
		(reg_val & (0x0001 << 0x1))	>> 0x1;
	else if (ginfo == GAUGE_PROP_MONITER_PLCHG_STATUS)
		*value =
		(reg_val & (0x0001 << 0x2))	>> 0x2;
	else if (ginfo == GAUGE_PROP_BAT_PLUG_STATUS)
		*value =
		(reg_val & (0x0001 << 0x3))	>> 0x3;
	else if (ginfo == GAUGE_PROP_IS_NVRAM_FAIL_MODE)
		*value =
		(reg_val & (0x0001 << 0x4))	>> 0x4;
	else if (ginfo == GAUGE_PROP_MONITOR_SOFF_VALIDTIME)
		*value =
		(reg_val & (0x0001 << 0x5))	>> 0x5;
	else if (ginfo == GAUGE_PROP_CON0_SOC)
		*value =
		(reg_val & (0x007F << 0x9))	>> 0x9;

	bm_debug("[%s]info:%d v:%d\n", __func__, ginfo, *value);

	return 0;
}

static void switch_nafg_period(int _prd, int *value)
{
	if (_prd >= 1 && _prd < 5)
		*value = 0;
	else if (_prd >= 5 && _prd < 10)
		*value = 1;
	else if (_prd >= 10 && _prd < 20)
		*value = 2;
	else if (_prd >= 20)
		*value = 3;
}

static void fgauge_set_nafg_intr_internal(struct mtk_gauge *gauge,
	int _prd, int _zcv_mv, int _thr_mv)
{
	int NAG_C_DLTV_Threashold_26_16;
	int NAG_C_DLTV_Threashold_15_0;
	int period = 0;

	gauge->zcv_reg = mv_to_reg_value(_zcv_mv);
	gauge->thr_reg = mv_to_reg_value(_thr_mv);

	if (gauge->thr_reg >= 32768) {
		bm_err("[%s]nag_c_dltv_thr mv=%d ,thr_reg=%d,limit thr_reg to 32767\n",
			__func__, _thr_mv, gauge->thr_reg);
		gauge->thr_reg = 32767;
	}

	NAG_C_DLTV_Threashold_26_16 = (gauge->thr_reg & 0xffff0000) >> 16;
	NAG_C_DLTV_Threashold_15_0 = (gauge->thr_reg & 0x0000ffff);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_NAG_ZCV_ADDR,
		PMIC_AUXADC_NAG_ZCV_MASK << PMIC_AUXADC_NAG_ZCV_SHIFT,
		gauge->zcv_reg << PMIC_AUXADC_NAG_ZCV_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_NAG_C_DLTV_TH_26_16_ADDR,
		PMIC_AUXADC_NAG_C_DLTV_TH_26_16_MASK <<
		PMIC_AUXADC_NAG_C_DLTV_TH_26_16_SHIFT,
		NAG_C_DLTV_Threashold_26_16 <<
		PMIC_AUXADC_NAG_C_DLTV_TH_26_16_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_NAG_C_DLTV_TH_15_0_ADDR,
		PMIC_AUXADC_NAG_C_DLTV_TH_15_0_MASK <<
		PMIC_AUXADC_NAG_C_DLTV_TH_15_0_SHIFT,
		NAG_C_DLTV_Threashold_15_0 <<
		PMIC_AUXADC_NAG_C_DLTV_TH_15_0_SHIFT);

	/* AUXADC_NAG_PRD_SEL  change to 0x10 means 10s detect*/
	switch_nafg_period(_prd, &period);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_NAG_PRD_SEL_ADDR,
		PMIC_AUXADC_NAG_PRD_SEL_MASK <<
		PMIC_AUXADC_NAG_PRD_SEL_SHIFT,
		period <<
		PMIC_AUXADC_NAG_PRD_SEL_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_NAG_VBAT1_SEL_ADDR,
		PMIC_AUXADC_NAG_VBAT1_SEL_MASK <<
		PMIC_AUXADC_NAG_VBAT1_SEL_SHIFT,
		0 <<
		PMIC_AUXADC_NAG_VBAT1_SEL_SHIFT);

	bm_err("[fg_bat_nafg][fgauge_set_nafg_interrupt_internal] time[%d] zcv[%d:%d] thr[%d:%d] 26_16[0x%x] 15_00[0x%x]\n",
		_prd, _zcv_mv, gauge->zcv_reg, _thr_mv, gauge->thr_reg,
		NAG_C_DLTV_Threashold_26_16, NAG_C_DLTV_Threashold_15_0);

}

int nafg_zcv_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int zcv)
{
	gauge->nafg_zcv_mv = zcv;	/* 0.1 mv*/
	return 0;
}

int zcv_current_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *zcv_current)
{
	unsigned int uvalue16 = 0;
	signed int dvalue = 0;
	long long Temp_Value = 0;

	regmap_read(gauge->regmap, PMIC_FG_ZCV_CURR_ADDR,
		&uvalue16);
	uvalue16 =
		(uvalue16 & (PMIC_FG_ZCV_CURR_MASK
		<< PMIC_FG_ZCV_CURR_SHIFT))
		>> PMIC_FG_ZCV_CURR_SHIFT;

	dvalue = uvalue16;
		if (dvalue == 0) {
			Temp_Value = (long long) dvalue;
		} else if (dvalue > 32767) {
			/* > 0x8000 */
			Temp_Value = (long long) (dvalue - 65535);
			Temp_Value = Temp_Value - (Temp_Value * 2);
		} else {
			Temp_Value = (long long) dvalue;
		}

	Temp_Value = Temp_Value * UNIT_FGCURRENT;

#if defined(__LP64__) || defined(_LP64)
	do_div(Temp_Value, 100000);
#else
	Temp_Value = div_s64(Temp_Value, 100000);
#endif
	dvalue = (unsigned int) Temp_Value;

	/* Auto adjust value */
	if (gauge->gm->fg_cust_data.r_fg_value != DEFAULT_R_FG) {
		bm_debug(
		"[fgauge_read_current] Auto adjust value due to the Rfg is %d\n Ori curr=%d",
		gauge->gm->fg_cust_data.r_fg_value, dvalue);

		dvalue = (dvalue * DEFAULT_R_FG) /
		gauge->gm->fg_cust_data.r_fg_value;

		bm_debug("[fgauge_read_current] new current=%d\n", dvalue);
	}

	bm_debug("[fgauge_read_current] ori current=%d\n", dvalue);
	dvalue = ((dvalue * gauge->gm->fg_cust_data.car_tune_value) / 1000);
	bm_debug("[fgauge_read_current] final current=%d (ratio=%d)\n",
		 dvalue, gauge->gm->fg_cust_data.car_tune_value);
	*zcv_current = dvalue;

	return 0;
}

int nafg_c_dltv_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int c_dltv_mv)
{
	gauge->nafg_c_dltv_mv = c_dltv_mv;	/* 0.1 mv*/
	fgauge_set_nafg_intr_internal(
	gauge,
	gauge->gm->fg_cust_data.nafg_time_setting,
	gauge->nafg_zcv_mv, gauge->nafg_c_dltv_mv);
	return 0;
}

static int get_nafg_vbat(struct mtk_gauge *gauge)
{
	unsigned int nag_vbat_reg, vbat_val;
	int nag_vbat_mv, i = 0;

	do {
		regmap_read(gauge->regmap, PMIC_AUXADC_ADC_OUT_NAG_ADDR,
			&nag_vbat_reg);

		nag_vbat_reg =
			(nag_vbat_reg & (PMIC_AUXADC_ADC_OUT_NAG_MASK <<
			PMIC_AUXADC_ADC_OUT_NAG_SHIFT))
			>> PMIC_AUXADC_ADC_OUT_NAG_SHIFT;

		if ((nag_vbat_reg & 0x8000) != 0)
			break;
		msleep(30);
		i++;
	} while (i <= 5);

	vbat_val = nag_vbat_reg & 0x7fff;
	nag_vbat_mv = reg_to_mv_value(vbat_val);
	return nag_vbat_mv;
}

int nafg_vbat_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *vbat)
{
	*vbat = get_nafg_vbat(gauge);
	return 0;
}

int bat_plugout_en_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	if (val != 0) {
		val = 1;
		enable_gauge_irq(gauge, BAT_PLUGOUT_IRQ);
	} else
		disable_gauge_irq(gauge, BAT_PLUGOUT_IRQ);

	return 0;
}

static void fgauge_set_zcv_intr_internal(
	struct mtk_gauge *gauge_dev,
	int fg_zcv_det_time,
	int fg_zcv_car_th)
{
	int fg_zcv_car_thr_h_reg, fg_zcv_car_thr_l_reg;
	long long fg_zcv_car_th_reg = fg_zcv_car_th;

	fg_zcv_car_th_reg = (fg_zcv_car_th_reg * 100 * 1000);

#if defined(__LP64__) || defined(_LP64)
	do_div(fg_zcv_car_th_reg, UNIT_FGCAR_ZCV);
#else
	fg_zcv_car_th_reg = div_s64(fg_zcv_car_th_reg, UNIT_FGCAR_ZCV);
#endif

	if (gauge_dev->hw_status.r_fg_value != DEFAULT_R_FG)
#if defined(__LP64__) || defined(_LP64)
		fg_zcv_car_th_reg = (fg_zcv_car_th_reg *
				gauge_dev->hw_status.r_fg_value) /
				DEFAULT_R_FG;
#else
		fg_zcv_car_th_reg = div_s64(fg_zcv_car_th_reg *
				gauge_dev->hw_status.r_fg_value,
				DEFAULT_R_FG);
#endif

#if defined(__LP64__) || defined(_LP64)
	fg_zcv_car_th_reg = ((fg_zcv_car_th_reg * 1000) /
			gauge_dev->gm->fg_cust_data.car_tune_value);
#else
	fg_zcv_car_th_reg = div_s64((fg_zcv_car_th_reg * 1000),
			gauge_dev->gm->fg_cust_data.car_tune_value);
#endif

	fg_zcv_car_thr_h_reg = (fg_zcv_car_th_reg & 0xffff0000) >> 16;
	fg_zcv_car_thr_l_reg = fg_zcv_car_th_reg & 0x0000ffff;

	regmap_update_bits(gauge_dev->regmap,
		PMIC_FG_ZCV_DET_IV_ADDR,
		PMIC_FG_ZCV_DET_IV_MASK <<
		PMIC_FG_ZCV_DET_IV_SHIFT,
		fg_zcv_det_time <<
		PMIC_FG_ZCV_DET_IV_SHIFT);

	regmap_update_bits(gauge_dev->regmap,
		PMIC_FG_ZCV_CAR_TH_15_00_ADDR,
		PMIC_FG_ZCV_CAR_TH_15_00_MASK <<
		PMIC_FG_ZCV_CAR_TH_15_00_SHIFT,
		fg_zcv_car_thr_l_reg <<
		PMIC_FG_ZCV_CAR_TH_15_00_SHIFT);

	regmap_update_bits(gauge_dev->regmap,
		PMIC_FG_ZCV_CAR_TH_30_16_ADDR,
		PMIC_FG_ZCV_CAR_TH_30_16_MASK <<
		PMIC_FG_ZCV_CAR_TH_30_16_SHIFT,
		fg_zcv_car_thr_h_reg <<
		PMIC_FG_ZCV_CAR_TH_30_16_SHIFT);

	bm_debug("[FG_ZCV_INT][%s] det_time %d mv %d reg %lld 30_16 0x%x 15_00 0x%x\n",
		__func__, fg_zcv_det_time, fg_zcv_car_th, fg_zcv_car_th_reg,
		fg_zcv_car_thr_h_reg, fg_zcv_car_thr_l_reg);
}

int zcv_intr_threshold_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int zcv_avg_current)
{
	int fg_zcv_det_time;
	int fg_zcv_car_th = 0;

	fg_zcv_det_time = gauge->gm->fg_cust_data.zcv_suspend_time;
	fg_zcv_car_th = (fg_zcv_det_time + 1) * 4 * zcv_avg_current / 60;

	bm_debug("[%s] current:%d, fg_zcv_det_time:%d, fg_zcv_car_th:%d\n",
		__func__, zcv_avg_current, fg_zcv_det_time, fg_zcv_car_th);

	fgauge_set_zcv_intr_internal(
		gauge, fg_zcv_det_time, fg_zcv_car_th);

	return 0;
}

int zcv_intr_en_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int en)
{
	static int cnt;

	bm_debug("%s %d %d\n", __func__,
		cnt, en);
	if (en != 0)
		cnt++;
	else
		cnt--;

	if (en == 0) {
		disable_gauge_irq(gauge, ZCV_IRQ);
		regmap_update_bits(gauge->regmap,
			PMIC_FG_ZCV_DET_EN_ADDR,
			PMIC_FG_ZCV_DET_EN_MASK <<
			PMIC_FG_ZCV_DET_EN_SHIFT,
			en <<
			PMIC_FG_ZCV_DET_EN_SHIFT);
		mdelay(1);
	}
	if (en == 1) {
		enable_gauge_irq(gauge, ZCV_IRQ);
		regmap_update_bits(gauge->regmap,
			PMIC_FG_ZCV_DET_EN_ADDR,
			PMIC_FG_ZCV_DET_EN_MASK <<
			PMIC_FG_ZCV_DET_EN_SHIFT,
			en <<
			PMIC_FG_ZCV_DET_EN_SHIFT);
	}

	bm_debug("[FG_ZCV_INT][fg_set_zcv_intr_en] En %d\n", en);

	return 0;
}

int soff_reset_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int en)
{
	return 0;
}

int ncar_reset_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	regmap_update_bits(gauge->regmap,
		PMIC_FG_N_CHARGE_RST_ADDR,
		PMIC_FG_N_CHARGE_RST_MASK
		<< PMIC_FG_N_CHARGE_RST_SHIFT,
		1 << PMIC_FG_N_CHARGE_RST_SHIFT);
	udelay(200);
	regmap_update_bits(gauge->regmap,
		PMIC_FG_N_CHARGE_RST_ADDR,
		PMIC_FG_N_CHARGE_RST_MASK
		<< PMIC_FG_N_CHARGE_RST_SHIFT,
		0 << PMIC_FG_N_CHARGE_RST_SHIFT);

	return 0;
}

int nafg_check_corner(struct mtk_gauge *gauge)
{
	int nag_vbat = 0;
	int setto_cdltv_thr_mv = 0;
	int get_c_dltv_mv = 0;
	int diff = 0;
	signed int nag_c_dltv_value;
	signed int nag_c_dltv_value_h;
	signed int nag_c_dltv_reg_value;
	bool bcheckbit10;
	int nag_zcv = gauge->nafg_zcv_mv;

	setto_cdltv_thr_mv = gauge->nafg_c_dltv_mv;

	/*AUXADC_NAG_7*/
	regmap_read(gauge->regmap, PMIC_AUXADC_NAG_C_DLTV_15_0_ADDR,
		&nag_c_dltv_value);

	nag_c_dltv_value =
		(nag_c_dltv_value & (PMIC_AUXADC_NAG_C_DLTV_15_0_MASK <<
		PMIC_AUXADC_NAG_C_DLTV_15_0_SHIFT))
		>> PMIC_AUXADC_NAG_C_DLTV_15_0_SHIFT;

	/*AUXADC_NAG_8*/
	regmap_read(gauge->regmap, PMIC_AUXADC_NAG_C_DLTV_26_16_ADDR,
		&nag_c_dltv_value_h);

	nag_c_dltv_value_h =
		(nag_c_dltv_value_h & (PMIC_AUXADC_NAG_C_DLTV_26_16_MASK <<
		PMIC_AUXADC_NAG_C_DLTV_26_16_SHIFT))
		>> PMIC_AUXADC_NAG_C_DLTV_26_16_SHIFT;

	bcheckbit10 = nag_c_dltv_value_h & 0x0400;

	if (bcheckbit10 == 0)
		nag_c_dltv_reg_value = (nag_c_dltv_value & 0xffff) +
				((nag_c_dltv_value_h & 0x07ff) << 16);
	else
		nag_c_dltv_reg_value = (nag_c_dltv_value & 0xffff) +
			(((nag_c_dltv_value_h | 0xf800) & 0xffff) << 16);

	get_c_dltv_mv = reg_to_mv_value(nag_c_dltv_reg_value);
	nag_vbat = get_nafg_vbat(gauge);

	if (nag_vbat < 31500 && nag_zcv > 31500)
		gauge->nafg_corner = 1;
	else if (nag_zcv < 31500 && nag_vbat > 31500)
		gauge->nafg_corner = 2;
	else
		gauge->nafg_corner = 0;

	bm_debug("%s:corner:%d nag_vbat:%d nag_zcv:%d get_c_dltv_mv:%d setto_cdltv_thr_mv:%d, diff:%d, RG[0x%x,0x%x]\n",
		__func__, gauge->nafg_corner, nag_vbat, nag_zcv, get_c_dltv_mv,
		setto_cdltv_thr_mv, diff,
		nag_c_dltv_value_h, nag_c_dltv_value);

	return 0;
}

int event_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int event)
{
	if (event == EVT_INT_NAFG_CHECK)
		nafg_check_corner(gauge);

	return 0;
}

int bat_tmp_ht_threshold_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int threshold)
{
	int tmp_int_lt = mv_to_reg_12_temp_value(threshold);

	/* min is high temp */
	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_BAT_TEMP_VOLT_MIN_ADDR,
		PMIC_AUXADC_BAT_TEMP_VOLT_MIN_MASK
		<< PMIC_AUXADC_BAT_TEMP_VOLT_MIN_SHIFT,
		tmp_int_lt << PMIC_AUXADC_BAT_TEMP_VOLT_MIN_SHIFT);

	bm_debug("[%s]mv:%d reg:%d\n",
		__func__, threshold, tmp_int_lt);
	return 0;
}

int en_bat_tmp_ht_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int en)
{
	if (en == 0) {
		disable_gauge_irq(gauge, BAT_TMP_H_IRQ);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_IRQ_EN_MIN_ADDR,
			PMIC_AUXADC_BAT_TEMP_IRQ_EN_MIN_MASK
			<< PMIC_AUXADC_BAT_TEMP_IRQ_EN_MIN_SHIFT,
			0 << PMIC_AUXADC_BAT_TEMP_IRQ_EN_MIN_SHIFT);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_EN_ADDR,
			PMIC_AUXADC_BAT_TEMP_EN_MASK
			<< PMIC_AUXADC_BAT_TEMP_EN_SHIFT,
			0 << PMIC_AUXADC_BAT_TEMP_EN_SHIFT);
	} else {
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_DET_PRD_SEL_ADDR,
			PMIC_AUXADC_BAT_TEMP_DET_PRD_SEL_MASK
			<< PMIC_AUXADC_BAT_TEMP_DET_PRD_SEL_SHIFT,
			2 << PMIC_AUXADC_BAT_TEMP_DET_PRD_SEL_SHIFT);
		/* unit: 0x10 = 2, means 5 second */

		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_DEBT_MIN_SEL_ADDR,
			PMIC_AUXADC_BAT_TEMP_DEBT_MIN_SEL_MASK
			<< PMIC_AUXADC_BAT_TEMP_DEBT_MIN_SEL_SHIFT,
			2 << PMIC_AUXADC_BAT_TEMP_DEBT_MIN_SEL_SHIFT);
		/* debounce 0x10 = 2 , means 4 times*/
		/* 5s * 4 times = 20s to issue bat_temp interrupt */

		regmap_update_bits(gauge->regmap,
			PMIC_RG_INT_MASK_BAT_TEMP_H_ADDR,
			PMIC_RG_INT_MASK_BAT_TEMP_H_MASK
			<< PMIC_RG_INT_MASK_BAT_TEMP_H_SHIFT,
			0 << PMIC_RG_INT_MASK_BAT_TEMP_H_SHIFT);

		enable_gauge_irq(gauge, BAT_TMP_H_IRQ);

		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_IRQ_EN_MIN_ADDR,
			PMIC_AUXADC_BAT_TEMP_IRQ_EN_MIN_MASK
			<< PMIC_AUXADC_BAT_TEMP_IRQ_EN_MIN_SHIFT,
			1 << PMIC_AUXADC_BAT_TEMP_IRQ_EN_MIN_SHIFT);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_DET_MIN_ADDR,
			PMIC_AUXADC_BAT_TEMP_DET_MIN_MASK
			<< PMIC_AUXADC_BAT_TEMP_DET_MIN_SHIFT,
			1 << PMIC_AUXADC_BAT_TEMP_DET_MIN_SHIFT);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_EN_ADDR,
			PMIC_AUXADC_BAT_TEMP_EN_MASK
			<< PMIC_AUXADC_BAT_TEMP_EN_SHIFT,
			1 << PMIC_AUXADC_BAT_TEMP_EN_SHIFT);
	}

	bm_debug("[%s]en:%d\n",
			__func__, en);

	return 0;
}

int bat_tmp_lt_threshold_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int threshold)
{
	int tmp_int_lt = mv_to_reg_12_temp_value(threshold);

	/* max is low temp */
	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_BAT_TEMP_VOLT_MAX_ADDR,
		PMIC_AUXADC_BAT_TEMP_VOLT_MAX_MASK
		<< PMIC_AUXADC_BAT_TEMP_VOLT_MAX_SHIFT,
		tmp_int_lt << PMIC_AUXADC_BAT_TEMP_VOLT_MAX_SHIFT);

	bm_debug("[%s]mv:%d reg:%d\n",
		__func__, threshold, tmp_int_lt);
	return 0;
}

int en_bat_tmp_lt_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int en)
{
	if (en == 0) {
		disable_gauge_irq(gauge, BAT_TMP_L_IRQ);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_IRQ_EN_MAX_ADDR,
			PMIC_AUXADC_BAT_TEMP_IRQ_EN_MAX_MASK
			<< PMIC_AUXADC_BAT_TEMP_IRQ_EN_MAX_SHIFT,
			0 << PMIC_AUXADC_BAT_TEMP_IRQ_EN_MAX_SHIFT);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_EN_ADDR,
			PMIC_AUXADC_BAT_TEMP_EN_MASK
			<< PMIC_AUXADC_BAT_TEMP_EN_SHIFT,
			0 << PMIC_AUXADC_BAT_TEMP_EN_SHIFT);
	} else {

		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_DET_PRD_SEL_ADDR,
			PMIC_AUXADC_BAT_TEMP_DET_PRD_SEL_MASK
			<< PMIC_AUXADC_BAT_TEMP_DET_PRD_SEL_SHIFT,
			2 << PMIC_AUXADC_BAT_TEMP_DET_PRD_SEL_SHIFT);
		/* unit: 0x10 = 2, means 5 second */

		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_DEBT_MAX_SEL_ADDR,
			PMIC_AUXADC_BAT_TEMP_DEBT_MAX_SEL_MASK
			<< PMIC_AUXADC_BAT_TEMP_DEBT_MAX_SEL_SHIFT,
			2 << PMIC_AUXADC_BAT_TEMP_DEBT_MAX_SEL_SHIFT);
		/* debounce 0x10 = 2 , means 4 times*/
		/* 5s * 4 times = 20s to issue bat_temp interrupt */

		regmap_update_bits(gauge->regmap,
			PMIC_RG_INT_MASK_BAT_TEMP_L_ADDR,
			PMIC_RG_INT_MASK_BAT_TEMP_L_MASK
			<< PMIC_RG_INT_MASK_BAT_TEMP_L_SHIFT,
			0 << PMIC_RG_INT_MASK_BAT_TEMP_L_SHIFT);
		enable_gauge_irq(gauge, BAT_TMP_L_IRQ);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_IRQ_EN_MAX_ADDR,
			PMIC_AUXADC_BAT_TEMP_IRQ_EN_MAX_MASK
			<< PMIC_AUXADC_BAT_TEMP_IRQ_EN_MAX_SHIFT,
			1 << PMIC_AUXADC_BAT_TEMP_IRQ_EN_MAX_SHIFT);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_DET_MAX_ADDR,
			PMIC_AUXADC_BAT_TEMP_DET_MAX_MASK
			<< PMIC_AUXADC_BAT_TEMP_DET_MAX_SHIFT,
			1 << PMIC_AUXADC_BAT_TEMP_DET_MAX_SHIFT);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_BAT_TEMP_EN_ADDR,
			PMIC_AUXADC_BAT_TEMP_EN_MASK
			<< PMIC_AUXADC_BAT_TEMP_EN_SHIFT,
			1 << PMIC_AUXADC_BAT_TEMP_EN_SHIFT);
	}

	bm_debug("[%s]en:%d\n",
			__func__, en);

	return 0;
}

int bat_cycle_intr_threshold_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int threshold)
{
	long long car = threshold;
	long long carReg;

	disable_gauge_irq(gauge, FG_N_CHARGE_L_IRQ);

#if defined(__LP64__) || defined(_LP64)
	car = car * 100000 / UNIT_CHARGE;
	/* 1000 * 100 */
#else
	car = div_s64(car * 100000, UNIT_CHARGE);
#endif

	if (gauge->hw_status.r_fg_value != DEFAULT_R_FG) {
		car = (car * gauge->hw_status.r_fg_value);
#if defined(__LP64__) || defined(_LP64)
		do_div(car, DEFAULT_R_FG);
#else
		car = div_s64(car, DEFAULT_R_FG);
#endif
	}

	car = car * 1000;
#if defined(__LP64__) || defined(_LP64)
	do_div(car, gauge->gm->fg_cust_data.car_tune_value);
#else
	car = div_s64(car, gauge->gm->fg_cust_data.car_tune_value);
#endif

	carReg = car;
	carReg = 0 - carReg;

	regmap_update_bits(gauge->regmap,
		PMIC_FG_N_CHARGE_LTH_15_00_ADDR,
		PMIC_FG_N_CHARGE_LTH_15_00_MASK
		<< PMIC_FG_N_CHARGE_LTH_15_00_SHIFT,
		(carReg & 0xffff) <<
		PMIC_FG_N_CHARGE_LTH_15_00_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_FG_N_CHARGE_LTH_31_16_ADDR,
		PMIC_FG_N_CHARGE_LTH_31_16_MASK
		<< PMIC_FG_N_CHARGE_LTH_31_16_SHIFT,
		((carReg & 0xffff0000) >> 16) <<
		PMIC_FG_N_CHARGE_LTH_31_16_SHIFT);

	bm_err("car:%d carR:%lld r:%lld\n",
		threshold, car, carReg);

	enable_gauge_irq(gauge, FG_N_CHARGE_L_IRQ);

	return 0;

}

int fgauge_get_time(struct mtk_gauge *gauge_dev, unsigned int *ptime)
{
	unsigned int time_29_16, time_15_00, ret_time;
	long long time = 0;

	pre_gauge_update(gauge_dev);

	regmap_read(gauge_dev->regmap, PMIC_FG_NTER_15_00_ADDR,
		&time_15_00);
	time_15_00 =
		(time_15_00 & (PMIC_FG_NTER_15_00_MASK <<
		PMIC_FG_NTER_15_00_SHIFT))
		>> PMIC_FG_NTER_15_00_SHIFT;

	regmap_read(gauge_dev->regmap, PMIC_FG_NTER_29_16_ADDR,
		&time_29_16);
	time_29_16 =
		(time_29_16 & (PMIC_FG_NTER_29_16_MASK <<
		PMIC_FG_NTER_29_16_SHIFT))
		>> PMIC_FG_NTER_29_16_SHIFT;

	time = time_15_00;
	time |= time_29_16 << 16;
#if defined(__LP64__) || defined(_LP64)
	time = time * UNIT_TIME / 100;
#else
	time = div_s64(time * UNIT_TIME, 100);
#endif
	ret_time = time;

	bm_debug(
		 "[%s] low:0x%x high:0x%x rtime:0x%llx 0x%x!\r\n",
		 __func__, time_15_00, time_29_16, time, ret_time);

	post_gauge_update(gauge_dev);

	*ptime = ret_time;

	return 0;
}

static unsigned int instant_current_for_car_tune(struct mtk_gauge *gauge)
{
	unsigned int reg_value = 0;

	pre_gauge_update(gauge);

	regmap_read(gauge->regmap, PMIC_FG_CURRENT_OUT_ADDR, &reg_value);
	reg_value = (reg_value &
		(PMIC_FG_CURRENT_OUT_MASK << PMIC_FG_CURRENT_OUT_SHIFT))
		>> PMIC_FG_CURRENT_OUT_SHIFT;

	post_gauge_update(gauge);

	bm_err("%s, reg_value=%d\n", __func__, reg_value);

	return reg_value;
}

static int instant_current(struct mtk_gauge *gauge)
{
	unsigned int reg_value;
	int dvalue;
	int r_fg_value;
	int car_tune_value;

	r_fg_value = gauge->hw_status.r_fg_value;
	car_tune_value = gauge->gm->fg_cust_data.car_tune_value;
	pre_gauge_update(gauge);

	regmap_read(gauge->regmap, PMIC_FG_CURRENT_OUT_ADDR, &reg_value);
	reg_value = (reg_value &
		(PMIC_FG_CURRENT_OUT_MASK << PMIC_FG_CURRENT_OUT_SHIFT))
		>> PMIC_FG_CURRENT_OUT_SHIFT;

	post_gauge_update(gauge);
	dvalue = reg_to_current(gauge, reg_value);

	/* Auto adjust value */
	if (r_fg_value != DEFAULT_R_FG) {

		dvalue = (dvalue * DEFAULT_R_FG) /
			r_fg_value;
	}

	dvalue =
	((dvalue * car_tune_value) / 1000);

	return dvalue;
}

void read_fg_hw_info_current_1(struct mtk_gauge *gauge_dev)
{
	gauge_dev->fg_hw_info.current_1 =
		instant_current(gauge_dev);
}

void read_fg_hw_info_current_2(struct mtk_gauge *gauge_dev)
{
	long long fg_current_2_reg;
	int cic2_reg;
	signed int dvalue;
	long long Temp_Value;
	int sign_bit = 0;

	regmap_read(gauge_dev->regmap, PMIC_MT6359_FG_CIC2_ADDR,
		&cic2_reg);
	cic2_reg =
		(cic2_reg & (PMIC_MT6359_FG_CIC2_MASK <<
		PMIC_MT6359_FG_CIC2_SHIFT))
		>> PMIC_MT6359_FG_CIC2_SHIFT;
	fg_current_2_reg = cic2_reg;

	/*calculate the real world data    */
	dvalue = (unsigned int) fg_current_2_reg;
	if (dvalue == 0) {
		Temp_Value = (long long) dvalue;
		sign_bit = 0;
	} else if (dvalue > 32767) {
		/* > 0x8000 */
		Temp_Value = (long long) (dvalue - 65535);
		Temp_Value = Temp_Value - (Temp_Value * 2);
		sign_bit = 1;
	} else {
		Temp_Value = (long long) dvalue;
		sign_bit = 0;
	}

	Temp_Value = Temp_Value * UNIT_FGCURRENT;
#if defined(__LP64__) || defined(_LP64)
	do_div(Temp_Value, 100000);
#else
	Temp_Value = div_s64(Temp_Value, 100000);
#endif
	dvalue = (unsigned int) Temp_Value;


	if (gauge_dev->hw_status.r_fg_value != DEFAULT_R_FG)
		dvalue = (dvalue * DEFAULT_R_FG) /
			gauge_dev->hw_status.r_fg_value;

	if (sign_bit == 1)
		dvalue = dvalue - (dvalue * 2);

	gauge_dev->fg_hw_info.current_2 =
		((dvalue * gauge_dev->gm->fg_cust_data.car_tune_value) / 1000);

}

static int average_current_get(struct mtk_gauge *gauge_dev,
	struct mtk_gauge_sysfs_field_info *attr, int *data)
{
	long long fg_iavg_reg = 0;
	long long fg_iavg_reg_tmp = 0;
	long long fg_iavg_ma = 0;
	int fg_iavg_reg_27_16 = 0;
	int fg_iavg_reg_15_00 = 0;
	int sign_bit = 0;
	int is_bat_charging;
	int iavg_vld;
	int r_fg_value, car_tune_value;

	r_fg_value = gauge_dev->hw_status.r_fg_value;
	car_tune_value = gauge_dev->gm->fg_cust_data.car_tune_value;

	pre_gauge_update(gauge_dev);

	regmap_read(gauge_dev->regmap, PMIC_FG_IAVG_VLD_ADDR, &iavg_vld);
	iavg_vld =
		(iavg_vld & (PMIC_FG_IAVG_VLD_MASK
		<< PMIC_FG_IAVG_VLD_SHIFT))
		>> PMIC_FG_IAVG_VLD_SHIFT;

	if (iavg_vld == 1) {
		regmap_read(gauge_dev->regmap, PMIC_FG_IAVG_27_16_ADDR,
			&fg_iavg_reg_27_16);
		fg_iavg_reg_27_16 =
			(fg_iavg_reg_27_16 & (PMIC_FG_IAVG_27_16_MASK
			<< PMIC_FG_IAVG_27_16_SHIFT))
			>> PMIC_FG_IAVG_27_16_SHIFT;

		regmap_read(gauge_dev->regmap, PMIC_FG_IAVG_15_00_ADDR,
			&fg_iavg_reg_15_00);
		fg_iavg_reg_15_00 =
			(fg_iavg_reg_15_00 & (PMIC_FG_IAVG_15_00_MASK
			<< PMIC_FG_IAVG_15_00_SHIFT))
			>> PMIC_FG_IAVG_15_00_SHIFT;

		fg_iavg_reg = fg_iavg_reg_27_16;
		fg_iavg_reg =
		((long long)fg_iavg_reg << 16) + fg_iavg_reg_15_00;

		sign_bit = (fg_iavg_reg_27_16 & 0x800) >> 11;

		if (sign_bit) {
			fg_iavg_reg_tmp = fg_iavg_reg;
			/*fg_iavg_reg = fg_iavg_reg_tmp - 0xfffffff - 1;*/
			fg_iavg_reg = 0xfffffff - fg_iavg_reg_tmp + 1;
		}

		if (sign_bit == 1)
			is_bat_charging = 0;	/* discharge */
		else
			is_bat_charging = 1;	/* charge */

		fg_iavg_ma = fg_iavg_reg * UNIT_FG_IAVG *
			car_tune_value;

		bm_debug(
			"[fg_get_current_iavg] fg_iavg_ma %lld fg_iavg_reg %lld fg_iavg_reg_tmp %lld\n",
			fg_iavg_ma, fg_iavg_reg, fg_iavg_reg_tmp);

#if defined(__LP64__) || defined(_LP64)
		do_div(fg_iavg_ma, 1000000);
#else
		fg_iavg_ma = div_s64(fg_iavg_ma, 1000000);
#endif


		if (r_fg_value != DEFAULT_R_FG) {
#if defined(__LP64__) || defined(_LP64)
			fg_iavg_ma = (fg_iavg_ma * DEFAULT_R_FG /
				r_fg_value);
#else
			fg_iavg_ma = div_s64(fg_iavg_ma * DEFAULT_R_FG,
				r_fg_value);
#endif
		}

#if defined(__LP64__) || defined(_LP64)
		do_div(fg_iavg_ma, 100);
#else
		fg_iavg_ma = div_s64(fg_iavg_ma, 100);
#endif

		bm_debug("[fg_get_current_iavg] fg_iavg_ma %lld\n",
			fg_iavg_ma);


		if (sign_bit == 1)
			fg_iavg_ma = 0 - fg_iavg_ma;

		bm_debug(
			"[fg_get_current_iavg] fg_iavg_ma %lld fg_iavg_reg %lld r_fg_value %d 27_16 0x%x 15_00 0x%x\n",
			fg_iavg_ma, fg_iavg_reg,
			r_fg_value,
			fg_iavg_reg_27_16, fg_iavg_reg_15_00);

		gauge_dev->fg_hw_info.current_avg = fg_iavg_ma;
		gauge_dev->fg_hw_info.current_avg_sign = sign_bit;
		bm_debug("[fg_get_current_iavg] PMIC_FG_IAVG_VLD == 1\n");
	} else {
		read_fg_hw_info_current_1(gauge_dev);
		gauge_dev->fg_hw_info.current_avg =
			gauge_dev->fg_hw_info.current_1;

		if (gauge_dev->fg_hw_info.current_1 < 0)
			gauge_dev->fg_hw_info.current_avg_sign = 1;

		bm_debug("[fg_get_current_iavg] PMIC_FG_IAVG_VLD != 1, avg %d, current_1 %d\n",
			gauge_dev->fg_hw_info.current_avg,
			gauge_dev->fg_hw_info.current_1);
	}

	post_gauge_update(gauge_dev);
	*data = gauge_dev->fg_hw_info.current_avg;

	gauge_dev->fg_hw_info.current_avg_valid = iavg_vld;
	bm_debug("[fg_get_current_iavg] %d %d\n", *data, iavg_vld);

	return 0;
}

static signed int fg_set_iavg_intr(struct mtk_gauge *gauge_dev, void *data)
{
	int iavg_gap = *(unsigned int *) (data);
	int iavg;
	long long iavg_ht, iavg_lt;
	long long fg_iavg_reg_ht, fg_iavg_reg_lt;
	int fg_iavg_lth_28_16, fg_iavg_lth_15_00;
	int fg_iavg_hth_28_16, fg_iavg_hth_15_00;


	average_current_get(gauge_dev, NULL, &iavg);

	iavg_ht = abs(iavg) + iavg_gap;
	iavg_lt = abs(iavg) - iavg_gap;

	if (iavg_lt <= 0)
		iavg_lt = 0;

	gauge_dev->hw_status.iavg_ht = iavg_ht;
	gauge_dev->hw_status.iavg_lt = iavg_lt;

/* reverse for IAVG */
/* fg_iavg_ma * 100 * fg_cust_data.r_fg_value / DEFAULT_RFG * 1000 * 1000 */
/* / fg_cust_data.car_tune_value / UNIT_FG_IAVG  = fg_iavg_reg  */

	fg_iavg_reg_ht = iavg_ht * 100;
	if (gauge_dev->hw_status.r_fg_value != DEFAULT_R_FG) {
		fg_iavg_reg_ht = fg_iavg_reg_ht *
			gauge_dev->hw_status.r_fg_value;
#if defined(__LP64__) || defined(_LP64)
		do_div(fg_iavg_reg_ht, DEFAULT_R_FG);
#else
		fg_iavg_reg_ht = div_s64(fg_iavg_reg_ht, DEFAULT_R_FG);
#endif
	}

	fg_iavg_reg_ht = fg_iavg_reg_ht * 1000000;

#if defined(__LP64__) || defined(_LP64)
	do_div(fg_iavg_reg_ht, UNIT_FG_IAVG);
	do_div(fg_iavg_reg_ht, gauge_dev->gm->fg_cust_data.car_tune_value);
#else
	fg_iavg_reg_ht = div_s64(fg_iavg_reg_ht, UNIT_FG_IAVG);
	fg_iavg_reg_ht = div_s64(fg_iavg_reg_ht,
				gauge_dev->gm->fg_cust_data.car_tune_value);
#endif


	fg_iavg_reg_lt = iavg_lt * 100;

	if (gauge_dev->hw_status.r_fg_value != DEFAULT_R_FG) {
		fg_iavg_reg_lt = fg_iavg_reg_lt *
			gauge_dev->hw_status.r_fg_value;
#if defined(__LP64__) || defined(_LP64)
		do_div(fg_iavg_reg_lt, DEFAULT_R_FG);
#else
		fg_iavg_reg_lt = div_s64(fg_iavg_reg_lt, DEFAULT_R_FG);
#endif
	}

	fg_iavg_reg_lt = fg_iavg_reg_lt * 1000000;

#if defined(__LP64__) || defined(_LP64)
	do_div(fg_iavg_reg_lt, UNIT_FG_IAVG);
	do_div(fg_iavg_reg_lt, gauge_dev->gm->fg_cust_data.car_tune_value);
#else
	fg_iavg_reg_lt = div_s64(fg_iavg_reg_lt, UNIT_FG_IAVG);
	fg_iavg_reg_lt = div_s64(fg_iavg_reg_lt,
				gauge_dev->gm->fg_cust_data.car_tune_value);
#endif

	fg_iavg_lth_28_16 = (fg_iavg_reg_lt & 0x1fff0000) >> 16;
	fg_iavg_lth_15_00 = fg_iavg_reg_lt & 0xffff;
	fg_iavg_hth_28_16 = (fg_iavg_reg_ht & 0x1fff0000) >> 16;
	fg_iavg_hth_15_00 = fg_iavg_reg_ht & 0xffff;

	disable_gauge_irq(gauge_dev, FG_IAVG_H_IRQ);
	disable_gauge_irq(gauge_dev, FG_IAVG_L_IRQ);

	regmap_update_bits(gauge_dev->regmap,
		PMIC_FG_IAVG_LTH_28_16_ADDR,
		PMIC_FG_IAVG_LTH_28_16_MASK
		<< PMIC_FG_IAVG_LTH_28_16_SHIFT,
		fg_iavg_lth_28_16 << PMIC_FG_IAVG_LTH_28_16_SHIFT);

	regmap_update_bits(gauge_dev->regmap,
		PMIC_FG_IAVG_LTH_15_00_ADDR,
		PMIC_FG_IAVG_LTH_15_00_MASK
		<< PMIC_FG_IAVG_LTH_15_00_SHIFT,
		fg_iavg_lth_15_00 << PMIC_FG_IAVG_LTH_15_00_SHIFT);

	regmap_update_bits(gauge_dev->regmap,
		PMIC_FG_IAVG_HTH_28_16_ADDR,
		PMIC_FG_IAVG_HTH_28_16_MASK
		<< PMIC_FG_IAVG_HTH_28_16_SHIFT,
		fg_iavg_hth_28_16 << PMIC_FG_IAVG_HTH_28_16_SHIFT);

	regmap_update_bits(gauge_dev->regmap,
		PMIC_FG_IAVG_HTH_15_00_ADDR,
		PMIC_FG_IAVG_HTH_15_00_MASK
		<< PMIC_FG_IAVG_HTH_15_00_SHIFT,
		fg_iavg_hth_15_00 << PMIC_FG_IAVG_HTH_15_00_SHIFT);

	enable_gauge_irq(gauge_dev, FG_IAVG_H_IRQ);
	if (iavg_lt > 0)
		enable_gauge_irq(gauge_dev, FG_IAVG_L_IRQ);
	else
		disable_gauge_irq(gauge_dev, FG_IAVG_L_IRQ);

	bm_debug("[FG_IAVG_INT][%s] iavg %d iavg_gap %d iavg_ht %lld iavg_lt %lld fg_iavg_reg_ht %lld fg_iavg_reg_lt %lld\n",
			__func__, iavg, iavg_gap, iavg_ht, iavg_lt,
			fg_iavg_reg_ht, fg_iavg_reg_lt);

	bm_debug("[FG_IAVG_INT][%s] lt_28_16 0x%x lt_15_00 0x%x ht_28_16 0x%x ht_15_00 0x%x\n",
			__func__, fg_iavg_lth_28_16, fg_iavg_lth_15_00,
			fg_iavg_hth_28_16, fg_iavg_hth_15_00);

	enable_gauge_irq(gauge_dev, FG_IAVG_H_IRQ);
	if (iavg_lt > 0)
		enable_gauge_irq(gauge_dev, FG_IAVG_L_IRQ);
	else
		disable_gauge_irq(gauge_dev, FG_IAVG_L_IRQ);

	return 0;
}

void read_fg_hw_info_ncar(struct mtk_gauge *gauge_dev)
{
	unsigned int uvalue32_NCAR = 0;
	unsigned int uvalue32_NCAR_MSB = 0;
	unsigned int temp_NCAR_15_0 = 0;
	unsigned int temp_NCAR_31_16 = 0;
	signed int dvalue_NCAR = 0;
	long long Temp_Value = 0;

	regmap_read(gauge_dev->regmap, PMIC_FG_NCAR_15_00_ADDR,
		&temp_NCAR_15_0);
	temp_NCAR_15_0 =
		(temp_NCAR_15_0 & (PMIC_FG_NCAR_15_00_MASK
		<< PMIC_FG_NCAR_15_00_SHIFT))
		>> PMIC_FG_NCAR_15_00_SHIFT;

	regmap_read(gauge_dev->regmap, PMIC_FG_NCAR_31_16_ADDR,
		&temp_NCAR_31_16);
	temp_NCAR_31_16 =
		(temp_NCAR_31_16 & (PMIC_FG_NCAR_31_16_MASK
		<< PMIC_FG_NCAR_31_16_SHIFT))
		>> PMIC_FG_NCAR_31_16_SHIFT;

	uvalue32_NCAR = temp_NCAR_15_0 & 0xffff;
	uvalue32_NCAR |= (temp_NCAR_31_16 & 0x7fff) << 16;

	uvalue32_NCAR_MSB = (temp_NCAR_31_16 & 0x8000) >> 15;

	/*calculate the real world data    */
	dvalue_NCAR = (signed int) uvalue32_NCAR;

	if (uvalue32_NCAR == 0) {
		Temp_Value = 0;
	} else if (uvalue32_NCAR_MSB == 0x1) {
		/* dis-charging */
		Temp_Value = (long long) (dvalue_NCAR - 0x7fffffff);
		/* keep negative value */
		Temp_Value = Temp_Value - (Temp_Value * 2);
	} else {
		/*charging */
		Temp_Value = (long long) dvalue_NCAR;
	}


	/* 0.1 mAh */
#if defined(__LP64__) || defined(_LP64)
	Temp_Value = Temp_Value * UNIT_CHARGE / 1000;
#else
	Temp_Value = div_s64(Temp_Value * UNIT_CHARGE, 1000);
#endif

#if defined(__LP64__) || defined(_LP64)
	do_div(Temp_Value, 10);
	Temp_Value = Temp_Value + 5;
	do_div(Temp_Value, 10);
#else
	Temp_Value = div_s64(Temp_Value, 10);
	Temp_Value = Temp_Value + 5;
	Temp_Value = div_s64(Temp_Value, 10);
#endif

	if (uvalue32_NCAR_MSB == 0x1)
		dvalue_NCAR = (signed int) (Temp_Value - (Temp_Value * 2));
	else
		dvalue_NCAR = (signed int) Temp_Value;

	/*Auto adjust value*/
	if (gauge_dev->hw_status.r_fg_value != DEFAULT_R_FG)
		dvalue_NCAR = (dvalue_NCAR * DEFAULT_R_FG) /
			gauge_dev->hw_status.r_fg_value;

	gauge_dev->fg_hw_info.ncar =
		((dvalue_NCAR * gauge_dev->gm->fg_cust_data.car_tune_value)
		/ 1000);

}

static int coulomb_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	unsigned int uvalue32_car = 0;
	unsigned int uvalue32_car_msb = 0;
	unsigned int temp_car_15_0 = 0;
	unsigned int temp_car_31_16 = 0;
	signed int dvalue_CAR = 0;
	long long temp_value = 0;
	int r_fg_value;
	int car_tune_value;

	r_fg_value = gauge->hw_status.r_fg_value;
	car_tune_value = gauge->gm->fg_cust_data.car_tune_value;
	pre_gauge_update(gauge);

	regmap_read(gauge->regmap, PMIC_FG_CAR_15_00_ADDR, &temp_car_15_0);
	temp_car_15_0 =	(temp_car_15_0 &
		(PMIC_FG_CAR_15_00_MASK << PMIC_FG_CAR_15_00_SHIFT))
		>> PMIC_FG_CAR_15_00_SHIFT;

	regmap_read(gauge->regmap, PMIC_FG_CAR_31_16_ADDR, &temp_car_31_16);
	temp_car_31_16 = (temp_car_31_16 &
		(PMIC_FG_CAR_31_16_MASK << PMIC_FG_CAR_31_16_SHIFT))
		>> PMIC_FG_CAR_31_16_SHIFT;

	post_gauge_update(gauge);

	uvalue32_car = temp_car_15_0 & 0xffff;
	uvalue32_car |= (temp_car_31_16 & 0x7fff) << 16;

	uvalue32_car_msb = (temp_car_31_16 & 0x8000) >> 15;

	/*calculate the real world data    */
	dvalue_CAR = (signed int) uvalue32_car;

	if (uvalue32_car == 0) {
		temp_value = 0;
	} else if (uvalue32_car_msb == 0x1) {
		/* dis-charging */
		temp_value = (long long) (dvalue_CAR - 0x7fffffff);
		/* keep negative value */
		temp_value = temp_value - (temp_value * 2);
	} else {
		/*charging */
		temp_value = (long long) dvalue_CAR;
	}

#if defined(__LP64__) || defined(_LP64)
	temp_value = temp_value * UNIT_CHARGE / 1000;
#else
	temp_value = div_s64(temp_value * UNIT_CHARGE, 1000);
#endif


#if defined(__LP64__) || defined(_LP64)
	do_div(temp_value, 10);
	temp_value = temp_value + 5;
	do_div(temp_value, 10);
#else
	temp_value = div_s64(temp_value, 10);
	temp_value = temp_value + 5;
	temp_value = div_s64(temp_value, 10);
#endif


	if (uvalue32_car_msb == 0x1)
		dvalue_CAR = (signed int) (temp_value - (temp_value * 2));
		/* keep negative value */
	else
		dvalue_CAR = (signed int) temp_value;


	bm_debug("[%s]l:0x%x h:0x%x val:%d msb:%d car:%d\n",
		__func__,
		temp_car_15_0, temp_car_31_16,
		uvalue32_car, uvalue32_car_msb,
		dvalue_CAR);

/*Auto adjust value*/
	if (r_fg_value != DEFAULT_R_FG) {
		bm_debug("[%s] Auto adjust value deu to the Rfg is %d\n Ori CAR=%d",
			 __func__,
			 r_fg_value, dvalue_CAR);

		dvalue_CAR = (dvalue_CAR * DEFAULT_R_FG) /
			r_fg_value;

		bm_debug("[%s] new CAR=%d\n",
			__func__,
			dvalue_CAR);
	}

	dvalue_CAR = ((dvalue_CAR *
		car_tune_value) / 1000);

	bm_debug("[%s] CAR=%d r_fg_value=%d car_tune_value=%d\n",
		__func__,
		dvalue_CAR, r_fg_value,
		car_tune_value);

	*val = dvalue_CAR;

	return 0;
}

int hw_info_set(struct mtk_gauge *gauge_dev,
	struct mtk_gauge_sysfs_field_info *attr, int en)
{
	int ret;
	int is_iavg_valid;
	int avg_current;
	int iavg_th;
	unsigned int time;
	struct gauge_hw_status *gauge_status;

	gauge_status = &gauge_dev->hw_status;
	/* Set Read Latchdata */
	post_gauge_update(gauge_dev);

	/* Current_1 */
	read_fg_hw_info_current_1(gauge_dev);

	/* Current_2 */
	read_fg_hw_info_current_2(gauge_dev);

	/* curr_out = pmic_get_register_value(PMIC_FG_CURRENT_OUT); */
	/* fg_offset = pmic_get_register_value(PMIC_FG_OFFSET); */

	/* Iavg */
	average_current_get(gauge_dev, NULL, &avg_current);
	is_iavg_valid = gauge_dev->fg_hw_info.current_avg_valid;
	if ((is_iavg_valid == 1) && (gauge_status->iavg_intr_flag == 0)) {
		bm_debug("[read_fg_hw_info]set first fg_set_iavg_intr %d %d\n",
			is_iavg_valid, gauge_status->iavg_intr_flag);
		gauge_status->iavg_intr_flag = 1;
		iavg_th = gauge_dev->gm->fg_cust_data.diff_iavg_th;
		ret = fg_set_iavg_intr(gauge_dev, &iavg_th);
	} else if (is_iavg_valid == 0) {
		gauge_status->iavg_intr_flag = 0;
		disable_gauge_irq(gauge_dev, FG_IAVG_H_IRQ);
		disable_gauge_irq(gauge_dev, FG_IAVG_L_IRQ);
		bm_debug(
			"[read_fg_hw_info] doublecheck first fg_set_iavg_intr %d %d\n",
			is_iavg_valid, gauge_status->iavg_intr_flag);
	}
	bm_debug("[read_fg_hw_info] thirdcheck first fg_set_iavg_intr %d %d\n",
		is_iavg_valid, gauge_status->iavg_intr_flag);

	/* Ncar */
	read_fg_hw_info_ncar(gauge_dev);

	/* recover read */
	post_gauge_update(gauge_dev);

	coulomb_get(gauge_dev, NULL, &gauge_dev->fg_hw_info.car);
	fgauge_get_time(gauge_dev, &time);
	gauge_dev->fg_hw_info.time = time;

	bm_debug("[FGADC_intr_end][read_fg_hw_info] curr_1 %d curr_2 %d Iavg %d sign %d car %d ncar %d time %d\n",
		gauge_dev->fg_hw_info.current_1,
		gauge_dev->fg_hw_info.current_2,
		gauge_dev->fg_hw_info.current_avg,
		gauge_dev->fg_hw_info.current_avg_sign,
		gauge_dev->fg_hw_info.car,
		gauge_dev->fg_hw_info.ncar, gauge_dev->fg_hw_info.time);

	return 0;
}

int nafg_en_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	static int cnt;

	bm_debug("%s %d %d\n", __func__,
		cnt, val);
	if (val != 0)
		cnt++;
	else
		cnt--;

	if (val != 0) {
		val = 1;
		enable_gauge_irq(gauge, NAFG_IRQ);
		bm_debug("[%s]enable:%d\n", __func__, val);
	} else {
		disable_gauge_irq(gauge, NAFG_IRQ);
		bm_debug("[%s]disable:%d\n", __func__, val);
	}
	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_NAG_IRQ_EN_ADDR,
		PMIC_AUXADC_NAG_IRQ_EN_MASK
		<< PMIC_AUXADC_NAG_IRQ_EN_SHIFT,
		val << PMIC_AUXADC_NAG_IRQ_EN_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_NAG_EN_ADDR,
		PMIC_AUXADC_NAG_EN_MASK
		<< PMIC_AUXADC_NAG_EN_SHIFT,
		val << PMIC_AUXADC_NAG_EN_SHIFT);

	return 0;
}

static int calculate_car_tune(struct mtk_gauge *gauge)
{
	int cali_car_tune;
	long long sum_all = 0;
	unsigned long long temp_sum = 0;
	int avg_cnt = 0;
	int i;
	unsigned int uvalue32 = 0;
	signed int dvalue = 0;
	long long Temp_Value1 = 0;
	unsigned long long Temp_Value2 = 0;
	long long current_from_ADC = 0;

	bm_err("%s, meta_current=%d,\n", __func__,
		gauge->hw_status.meta_current);
	if (gauge->hw_status.meta_current != 0) {
		for (i = 0; i < CALI_CAR_TUNE_AVG_NUM; i++) {
			uvalue32 = instant_current_for_car_tune(gauge);
			if (uvalue32 != 0) {
				if (uvalue32 <= 0x8000) {
					Temp_Value1 = (long long)uvalue32;
					bm_err("[111]uvalue32 %d Temp_Value1 %lld\n",
						uvalue32,
						Temp_Value1);
				} else if (uvalue32 > 0x8000) {

					Temp_Value1 =
					(long long) (65535 - uvalue32);
					bm_err("[222]uvalue32 %d Temp_Value1 %lld\n",
						uvalue32,
						Temp_Value1);
				}
				sum_all += Temp_Value1;
				avg_cnt++;
				/*****************/
				bm_err("[333]uvalue32 %d Temp_Value1 %lld sum_all %lld\n",
						uvalue32,
						Temp_Value1, sum_all);
				/*****************/
			}
			mdelay(30);
		}
		/*calculate the real world data    */
		/*current_from_ADC = sum_all / avg_cnt;*/
		temp_sum = sum_all;
		bm_err("[444]sum_all %lld temp_sum %lld avg_cnt %d current_from_ADC %lld\n",
			sum_all, temp_sum, avg_cnt, current_from_ADC);

		if (avg_cnt != 0)
			do_div(temp_sum, avg_cnt);
		current_from_ADC = temp_sum;

		bm_err("[555]sum_all %lld temp_sum %lld avg_cnt %d current_from_ADC %lld\n",
			sum_all, temp_sum, avg_cnt, current_from_ADC);

		Temp_Value2 = current_from_ADC * UNIT_FGCURRENT;

		bm_err("[555]Temp_Value2 %lld current_from_ADC %lld UNIT_FGCURRENT %d\n",
			Temp_Value2, current_from_ADC, UNIT_FGCURRENT);

		/* Move 100 from denominator to cali_car_tune's numerator */
		/*do_div(Temp_Value2, 1000000);*/
		do_div(Temp_Value2, 10000);

		bm_err("[666]Temp_Value2 %lld current_from_ADC %lld UNIT_FGCURRENT %d\n",
			Temp_Value2, current_from_ADC, UNIT_FGCURRENT);

		dvalue = (unsigned int) Temp_Value2;

		/* Auto adjust value */
		if (gauge->hw_status.r_fg_value != 100)
			dvalue = (dvalue * 100) /
			gauge->hw_status.r_fg_value;

		bm_err("[666]dvalue %d fg_cust_data.r_fg_value %d\n",
			dvalue, gauge->hw_status.r_fg_value);

		/* Move 100 from denominator to cali_car_tune's numerator */
		/*cali_car_tune = meta_input_cali_current * 1000 / dvalue;*/

		if (dvalue != 0) {
			cali_car_tune =
				gauge->hw_status.meta_current *
				1000 * 100 / dvalue;

			bm_err("[777]dvalue %d fg_cust_data.r_fg_value %d cali_car_tune %d\n",
				dvalue,
				gauge->hw_status.r_fg_value,
				cali_car_tune);
			gauge->hw_status.tmp_car_tune = cali_car_tune;

			bm_err(
				"[fgauge_meta_cali_car_tune_value][%d] meta:%d, adc:%lld, UNI_FGCUR:%d, r_fg_value:%d\n",
				cali_car_tune, gauge->hw_status.meta_current,
				current_from_ADC, UNIT_FGCURRENT,
				gauge->hw_status.r_fg_value);
		}

		return 0;
	}

	return 0;
}

int info_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	int ret = 0;

	if (attr->prop == GAUGE_PROP_CAR_TUNE_VALUE &&
		(val > 500 && val < 1500)) {
		/* send external_current for calculate_car_tune */
		gauge->hw_status.meta_current = val;
		calculate_car_tune(gauge);
	} else if (attr->prop == GAUGE_PROP_R_FG_VALUE &&
		val != 0)
		gauge->hw_status.r_fg_value = val;
	else if (attr->prop == GAUGE_PROP_VBAT2_DETECT_TIME)
		gauge->hw_status.vbat2_det_time = val;
	else if (attr->prop == GAUGE_PROP_VBAT2_DETECT_COUNTER)
		gauge->hw_status.vbat2_det_counter = val;
	else
		ret = fgauge_set_info(gauge, attr->prop, val);

	return ret;
}

int info_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	int ret = 0;

	if (attr->prop == GAUGE_PROP_CAR_TUNE_VALUE)
		*val = gauge->hw_status.tmp_car_tune;
	else if (attr->prop == GAUGE_PROP_R_FG_VALUE)
		*val = gauge->hw_status.r_fg_value;
	else if (attr->prop == GAUGE_PROP_VBAT2_DETECT_TIME)
		*val = gauge->hw_status.vbat2_det_time;
	else if (attr->prop == GAUGE_PROP_VBAT2_DETECT_COUNTER)
		*val = gauge->hw_status.vbat2_det_counter;
	else
		ret = fgauge_get_info(gauge, attr->prop, val);

	return ret;
}

static int get_ptim_current(struct mtk_gauge *gauge)
{
	unsigned int reg_value;
	int dvalue;
	int r_fg_value;
	int car_tune_value;

	r_fg_value = gauge->hw_status.r_fg_value;
	car_tune_value = gauge->gm->fg_cust_data.car_tune_value;
	regmap_read(gauge->regmap, PMIC_FG_R_CURR_ADDR, &reg_value);
	reg_value =
		(reg_value & (PMIC_FG_R_CURR_MASK << PMIC_FG_R_CURR_SHIFT))
		>> PMIC_FG_R_CURR_SHIFT;
	dvalue = reg_to_current(gauge, reg_value);

	/* Auto adjust value */
	if (r_fg_value != DEFAULT_R_FG)
		dvalue = (dvalue * DEFAULT_R_FG) / r_fg_value;

	dvalue =
	((dvalue * car_tune_value) / 1000);

	/* ptim current >0 means discharge, different to bat_current */
	dvalue = dvalue * -1;
	bm_err("[%s]ptim current:%d\n", __func__, dvalue);

	return dvalue;
}

static enum power_supply_property gauge_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,

};

static int psy_gauge_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_gauge *gauge;
	struct mtk_battery *gm;

	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		/* store disableGM30 status to mtk-gauge psy for DLPT */
		if (gauge == NULL || gauge->gm == NULL)
			val->intval = 0;
		else
			val->intval = gauge->gm->disableGM30;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (gauge == NULL || gauge->gm == NULL)
			val->intval = 0;
		else
			val->intval = gauge->gm->disableGM30;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_ptim_current(gauge);
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		gm = gauge->gm;
		if (gm != NULL)
			val->intval = gm->sdc.shutdown_status.is_dlpt_shutdown;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int psy_gauge_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	int ret = 0;
	struct mtk_gauge *gauge;
	struct mtk_battery *gm;

	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		pr_notice("%s: %d %d\n", __func__, psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		gm = gauge->gm;
		if (gm != NULL && val->intval == 1)
			set_shutdown_cond(gm, DLPT_SHUTDOWN);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;

}

static void fgauge_read_RTC_boot_status(struct mtk_gauge *gauge)
{
	unsigned int hw_id;
	u8 spare0_reg = 0;
	unsigned int spare0_reg_b13 = 0;
	u8 spare3_reg = 0;
	int spare3_reg_valid = 0;

	regmap_read(gauge->regmap, PMIC_HWCID_ADDR, &hw_id);
	hw_id =	(hw_id & (PMIC_HWCID_MASK << PMIC_HWCID_SHIFT))
		>> PMIC_HWCID_SHIFT;
	spare0_reg = get_rtc_spare0_fg_value(gauge);
	spare3_reg = get_rtc_spare_fg_value(gauge);
	gauge->hw_status.gspare0_reg = spare0_reg;
	gauge->hw_status.gspare3_reg = spare3_reg;
	spare3_reg_valid = (spare3_reg & 0x80) >> 7;

	if (spare3_reg_valid == 0)
		gauge->hw_status.rtc_invalid = 1;
	else
		gauge->hw_status.rtc_invalid = 0;

	if (gauge->hw_status.rtc_invalid == 0) {
		spare0_reg_b13 = (spare0_reg & 0x20) >> 5;
		if ((hw_id & 0xff00) == 0x3500)
			gauge->hw_status.is_bat_plugout = spare0_reg_b13;
		else
			gauge->hw_status.is_bat_plugout = !spare0_reg_b13;

		gauge->hw_status.bat_plug_out_time = spare0_reg & 0x1f;
	} else {
		gauge->hw_status.is_bat_plugout = 1;
		gauge->hw_status.bat_plug_out_time = 31;
	}

	bm_err("[%s]rtc_invalid %d plugout %d plugout_time %d spare3 0x%x spare0 0x%x hw_id 0x%x\n",
			__func__,
			gauge->hw_status.rtc_invalid,
			gauge->hw_status.is_bat_plugout,
			gauge->hw_status.bat_plug_out_time,
			spare3_reg, spare0_reg, hw_id);
}

static int reset_fg_rtc_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	int hw_id;
	int temp_value;
	u8 spare0_reg, after_rst_spare0_reg;
	u8 spare3_reg, after_rst_spare3_reg;

	regmap_read(gauge->regmap, PMIC_HWCID_ADDR, &hw_id);
	hw_id =	(hw_id & (PMIC_HWCID_MASK << PMIC_HWCID_SHIFT))
		>> PMIC_HWCID_SHIFT;

	fgauge_read_RTC_boot_status(gauge);

	/* read spare0 */
	spare0_reg = get_rtc_spare0_fg_value(gauge);

	/* raise 15b to reset */
	if ((hw_id & 0xff00) == 0x3500) {
		temp_value = 0x80;
		set_rtc_spare0_fg_value(gauge, temp_value);
		mdelay(1);
		temp_value = 0x00;
		set_rtc_spare0_fg_value(gauge, temp_value);
	} else {
		temp_value = 0x80;
		set_rtc_spare0_fg_value(gauge, temp_value);
		mdelay(1);
		temp_value = 0x20;
		set_rtc_spare0_fg_value(gauge, temp_value);
	}

	/* read spare0 again */
	after_rst_spare0_reg = get_rtc_spare0_fg_value(gauge);

	/* read spare3 */
	spare3_reg = get_rtc_spare_fg_value(gauge);

	/* set spare3 0x7f */
	set_rtc_spare_fg_value(gauge, spare3_reg | 0x80);

	/* read spare3 again */
	after_rst_spare3_reg = get_rtc_spare_fg_value(gauge);

	bm_err("[fgauge_read_RTC_boot_status] spare0 0x%x 0x%x, spare3 0x%x 0x%x\n",
		spare0_reg, after_rst_spare0_reg, spare3_reg,
		after_rst_spare3_reg);

	return 0;
}

static int read_hw_ocv_6359_plug_in(struct mtk_gauge *gauge)
{
	signed int adc_rdy = 0;
	signed int adc_result_reg = 0;
	signed int adc_result = 0;
	int sel;

/* 6359 no need to switch SWCHR_POWER_PATH, only 56 57 */
	regmap_read(gauge->regmap, PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_PCHR_ADDR,
		&adc_rdy);
	adc_rdy = (adc_rdy & (PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_PCHR_MASK
		<< PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_PCHR_SHIFT))
		>> PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_PCHR_SHIFT;

	regmap_read(gauge->regmap, PMIC_AUXADC_ADC_OUT_BAT_PLUGIN_PCHR_ADDR,
		&adc_result_reg);
	adc_result_reg = (adc_result_reg &
		(PMIC_AUXADC_ADC_OUT_BAT_PLUGIN_PCHR_MASK
		<< PMIC_AUXADC_ADC_OUT_BAT_PLUGIN_PCHR_SHIFT))
		>> PMIC_AUXADC_ADC_OUT_BAT_PLUGIN_PCHR_SHIFT;

	regmap_read(gauge->regmap, PMIC_RG_HK_STRUP_AUXADC_START_SEL_ADDR,
		&sel);
	sel = (sel & (PMIC_RG_HK_STRUP_AUXADC_START_SEL_MASK
		<< PMIC_RG_HK_STRUP_AUXADC_START_SEL_SHIFT))
		>> PMIC_RG_HK_STRUP_AUXADC_START_SEL_SHIFT;

	adc_result = reg_to_mv_value(adc_result_reg);
	bm_err("[oam] %s (pchr): adc_result_reg=%d, adc_result=%d, start_sel=%d, rdy=%d\n",
		__func__, adc_result_reg, adc_result,
		sel,
		adc_rdy);

	if (adc_rdy == 1) {
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_CLR_ADDR,
			PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_CLR_MASK <<
			PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_CLR_SHIFT,
			1 << PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_CLR_SHIFT);
		mdelay(1);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_CLR_ADDR,
			PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_CLR_MASK <<
			PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_CLR_SHIFT,
			0 << PMIC_AUXADC_ADC_RDY_BAT_PLUGIN_CLR_SHIFT);
	}

	return adc_result;
}

static int read_hw_ocv_6359_power_on(struct mtk_gauge *gauge)
{
	signed int adc_result_rdy = 0;
	signed int adc_result_reg = 0;
	signed int adc_result = 0;
	int sel;

	regmap_read(gauge->regmap, PMIC_AUXADC_ADC_RDY_PWRON_PCHR_ADDR,
		&adc_result_rdy);
	adc_result_rdy = (adc_result_rdy & (PMIC_AUXADC_ADC_RDY_PWRON_PCHR_MASK
		<< PMIC_AUXADC_ADC_RDY_PWRON_PCHR_SHIFT))
		>> PMIC_AUXADC_ADC_RDY_PWRON_PCHR_SHIFT;

	regmap_read(gauge->regmap, PMIC_AUXADC_ADC_OUT_PWRON_PCHR_ADDR,
		&adc_result_reg);
	adc_result_reg = (adc_result_reg & (PMIC_AUXADC_ADC_OUT_PWRON_PCHR_MASK
		<< PMIC_AUXADC_ADC_OUT_PWRON_PCHR_SHIFT))
		>> PMIC_AUXADC_ADC_OUT_PWRON_PCHR_SHIFT;

	regmap_read(gauge->regmap, PMIC_RG_HK_STRUP_AUXADC_START_SEL_ADDR,
		&sel);
	sel = (sel & (PMIC_RG_HK_STRUP_AUXADC_START_SEL_MASK
		<< PMIC_RG_HK_STRUP_AUXADC_START_SEL_SHIFT))
		>> PMIC_RG_HK_STRUP_AUXADC_START_SEL_SHIFT;

	adc_result = reg_to_mv_value(adc_result_reg);
	bm_err("[oam] %s (pchr) : adc_result_reg=%d, adc_result=%d, start_sel=%d, rdy=%d\n",
		__func__, adc_result_reg, adc_result,
		sel, adc_result_rdy);

	if (adc_result_rdy == 1) {
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_ADDR,
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_MASK <<
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_SHIFT,
			1 << PMIC_AUXADC_ADC_RDY_PWRON_CLR_SHIFT);
		mdelay(1);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_ADDR,
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_MASK <<
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_SHIFT,
			0 << PMIC_AUXADC_ADC_RDY_PWRON_CLR_SHIFT);
	}
	return adc_result;
}

static int read_hw_ocv_6359_power_on_rdy(struct mtk_gauge *gauge)
{
	int pon_rdy = 0;

	regmap_read(gauge->regmap, PMIC_AUXADC_ADC_RDY_PWRON_PCHR_ADDR,
		&pon_rdy);
	pon_rdy = (pon_rdy & (PMIC_AUXADC_ADC_RDY_PWRON_PCHR_MASK
		<< PMIC_AUXADC_ADC_RDY_PWRON_PCHR_SHIFT))
		>> PMIC_AUXADC_ADC_RDY_PWRON_PCHR_SHIFT;

	bm_err("[%s] pwron_PCHR_rdy %d\n", __func__, pon_rdy);

	return pon_rdy;
}

static int nafg_cnt_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *nag_cnt)
{
	signed int NAG_C_DLTV_CNT;
	signed int NAG_C_DLTV_CNT_H;

	/*AUXADC_NAG_4*/
	regmap_read(gauge->regmap,
		PMIC_AUXADC_NAG_CNT_15_0_ADDR,
		&NAG_C_DLTV_CNT);

	/*AUXADC_NAG_5*/
	regmap_read(gauge->regmap,
		PMIC_AUXADC_NAG_CNT_25_16_ADDR,
		&NAG_C_DLTV_CNT_H);
	*nag_cnt = (NAG_C_DLTV_CNT & PMIC_AUXADC_NAG_CNT_15_0_MASK) +
		((NAG_C_DLTV_CNT_H & PMIC_AUXADC_NAG_CNT_25_16_MASK) << 16);
	bm_debug("[fg_bat_nafg][%s] %d [25_16 %d 15_0 %d]\n",
			__func__, *nag_cnt, NAG_C_DLTV_CNT_H, NAG_C_DLTV_CNT);

	return 0;
}

static int battery_voltage_cali(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	return 0;
}

static int nafg_dltv_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *nag_dltv)
{
	signed int nag_dltv_reg_value;
	signed int nag_dltv_mv_value;
	short reg_value;

	/*AUXADC_NAG_4*/
	regmap_read(gauge->regmap,
		PMIC_AUXADC_NAG_DLTV_ADDR,
		&nag_dltv_reg_value);

	reg_value = nag_dltv_reg_value & 0xffff;

	nag_dltv_mv_value = reg_to_mv_value(nag_dltv_reg_value);
	*nag_dltv = nag_dltv_mv_value;

	bm_debug("[fg_bat_nafg][%s] mV:Reg [%d:%d] [%d:%d]\n",
		__func__, nag_dltv_mv_value, nag_dltv_reg_value,
		reg_to_mv_value(reg_value),
		reg_value);

	return 0;
}

static int nafg_c_dltv_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *nafg_c_dltv)
{
	signed int nag_c_dltv_value;
	signed int nag_c_dltv_value_h;
	signed int nag_c_dltv_reg_value;
	signed int nag_c_dltv_mv_value;
	bool bcheckbit10;

	/*AUXADC_NAG_7*/
	regmap_read(gauge->regmap, PMIC_AUXADC_NAG_C_DLTV_15_0_ADDR,
		&nag_c_dltv_value);

	/*AUXADC_NAG_8*/
	regmap_read(gauge->regmap, PMIC_AUXADC_NAG_C_DLTV_26_16_ADDR,
		&nag_c_dltv_value_h);
	nag_c_dltv_value_h = (nag_c_dltv_value_h &
		PMIC_AUXADC_NAG_C_DLTV_26_16_MASK);

	bcheckbit10 = nag_c_dltv_value_h & 0x0400;

	if (gauge->nafg_corner == 1) {
		nag_c_dltv_reg_value = (nag_c_dltv_value & 0x7fff);
		nag_c_dltv_mv_value = reg_to_mv_value(nag_c_dltv_reg_value);
		*nafg_c_dltv = nag_c_dltv_mv_value;

		bm_debug("[fg_bat_nafg][%s] mV:Reg[%d:%d] [b10:%d][26_16(0x%04x) 15_00(0x%04x)] corner:%d\n",
			__func__, nag_c_dltv_mv_value, nag_c_dltv_reg_value,
			bcheckbit10, nag_c_dltv_value_h, nag_c_dltv_value,
			gauge->nafg_corner);
		return 0;
	} else if (gauge->nafg_corner == 2) {
		nag_c_dltv_reg_value = (nag_c_dltv_value - 32768);
		nag_c_dltv_mv_value =
			reg_to_mv_value(nag_c_dltv_reg_value);
		*nafg_c_dltv = nag_c_dltv_mv_value;

		bm_debug("[fg_bat_nafg][%s] mV:Reg[%d:%d] [b10:%d][26_16(0x%04x) 15_00(0x%04x)] corner:%d\n",
			__func__, nag_c_dltv_mv_value, nag_c_dltv_reg_value,
			bcheckbit10, nag_c_dltv_value_h, nag_c_dltv_value,
			gauge->nafg_corner);
		return 0;
	}

	if (bcheckbit10 == 0)
		nag_c_dltv_reg_value = (nag_c_dltv_value & 0xffff) +
				((nag_c_dltv_value_h & 0x07ff) << 16);
	else
		nag_c_dltv_reg_value = (nag_c_dltv_value & 0xffff) +
			(((nag_c_dltv_value_h | 0xf800) & 0xffff) << 16);

	nag_c_dltv_mv_value = reg_to_mv_value(nag_c_dltv_reg_value);
	*nafg_c_dltv = nag_c_dltv_mv_value;

	bm_debug("[fg_bat_nafg][%s] mV:Reg[%d:%d] [b10:%d][26_16(0x%04x) 15_00(0x%04x)] corner:%d\n",
		__func__, nag_c_dltv_mv_value, nag_c_dltv_reg_value,
		bcheckbit10, nag_c_dltv_value_h, nag_c_dltv_value,
		gauge->nafg_corner);

	return 0;
}

static int zcv_get(struct mtk_gauge *gauge_dev,
	struct mtk_gauge_sysfs_field_info *attr, int *zcv)
{
	signed int adc_result_reg = 0;
	signed int adc_result = 0;

	regmap_read(gauge_dev->regmap,
		PMIC_AUXADC_ADC_OUT_FGADC_PCHR_ADDR,
		&adc_result_reg);
	adc_result_reg =
		(adc_result_reg & (PMIC_AUXADC_ADC_OUT_FGADC_PCHR_MASK
		<< PMIC_AUXADC_ADC_OUT_FGADC_PCHR_SHIFT))
		>> PMIC_AUXADC_ADC_OUT_FGADC_PCHR_SHIFT;

	adc_result = reg_to_mv_value(adc_result_reg);
	bm_err("[oam] %s BATSNS  (pchr):adc_result_reg=%d, adc_result=%d\n",
		 __func__, adc_result_reg, adc_result);

	*zcv = adc_result;
	return 0;
}

static int get_charger_zcv(struct mtk_gauge *gauge_dev)
{
	struct power_supply *chg_psy;
	union power_supply_propval val;
	int ret = 0;

	chg_psy = power_supply_get_by_name("mtk-master-charger");

	if (chg_psy == NULL) {
		bm_err("[%s] can get charger psy\n", __func__);
		return -ENODEV;
	}

	ret = power_supply_get_property(chg_psy,
		POWER_SUPPLY_PROP_VOLTAGE_BOOT, &val);

	bm_err("[%s]_hw_ocv_chgin=%d, ret=%d\n", __func__, val.intval, ret);

	return val.intval;
}

static int boot_zcv_get(struct mtk_gauge *gauge_dev,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	int _hw_ocv, _sw_ocv;
	int _hw_ocv_src;
	int _prev_hw_ocv, _prev_hw_ocv_src;
	int _hw_ocv_rdy;
	int _flag_unreliable;
	int _hw_ocv_59_pon;
	int _hw_ocv_59_plugin;
	int _hw_ocv_59_pon_rdy;
	int _hw_ocv_chgin;
	int _hw_ocv_chgin_rdy;
	int now_temp;
	int now_thr;
	int tmp_hwocv_chgin = 0;
	bool fg_is_charger_exist;
	struct mtk_battery *gm;
	struct zcv_data *zcvinfo;
	struct gauge_hw_status *p;

	gm = gauge_dev->gm;
	p = &gauge_dev->hw_status;
	zcvinfo = &gauge_dev->zcv_info;
	_hw_ocv_59_pon_rdy = read_hw_ocv_6359_power_on_rdy(gauge_dev);
	_hw_ocv_59_pon = read_hw_ocv_6359_power_on(gauge_dev);
	_hw_ocv_59_plugin = read_hw_ocv_6359_plug_in(gauge_dev);

	tmp_hwocv_chgin = get_charger_zcv(gauge_dev);
	if (tmp_hwocv_chgin != -ENODEV)
		_hw_ocv_chgin = tmp_hwocv_chgin / 100;
	else
		_hw_ocv_chgin = 0;

	now_temp = gm->bs_data.bat_batt_temp;

	if (gm == NULL)
		now_thr = 300;
	else {
		if (now_temp > gm->ext_hwocv_swocv_lt_temp)
			now_thr = gm->ext_hwocv_swocv;
		else
			now_thr = gm->ext_hwocv_swocv_lt;
	}

	if (_hw_ocv_chgin < 25000)
		_hw_ocv_chgin_rdy = 0;
	else
		_hw_ocv_chgin_rdy = 1;

	/* if preloader records charge in, need to using subpmic as hwocv */
	fgauge_get_info(
		gauge_dev, GAUGE_PROP_PL_CHARGING_STATUS,
		&zcvinfo->pl_charging_status);
	fgauge_set_info(
		gauge_dev, GAUGE_PROP_PL_CHARGING_STATUS, 0);
	fgauge_get_info(
		gauge_dev, GAUGE_PROP_MONITER_PLCHG_STATUS,
		&zcvinfo->moniter_plchg_bit);
	fgauge_set_info(
		gauge_dev, GAUGE_PROP_MONITER_PLCHG_STATUS, 0);

	if (zcvinfo->pl_charging_status == 1)
		fg_is_charger_exist = 1;
	else
		fg_is_charger_exist = 0;

	_hw_ocv = _hw_ocv_59_pon;
	_sw_ocv = gauge_dev->hw_status.sw_ocv;
	/* _sw_ocv = get_sw_ocv();*/
	_hw_ocv_src = FROM_PMIC_PON_ON;
	_prev_hw_ocv = _hw_ocv;
	_prev_hw_ocv_src = FROM_PMIC_PON_ON;
	_flag_unreliable = 0;

	if (fg_is_charger_exist) {
		_hw_ocv_rdy = _hw_ocv_59_pon_rdy;
		if (_hw_ocv_rdy == 1) {
			if (_hw_ocv_chgin_rdy == 1) {
				_hw_ocv = _hw_ocv_chgin;
				_hw_ocv_src = FROM_CHR_IN;
			} else {
				_hw_ocv = _hw_ocv_59_pon;
				_hw_ocv_src = FROM_PMIC_PON_ON;
			}

			if (abs(_hw_ocv - _sw_ocv) > now_thr) {
				_prev_hw_ocv = _hw_ocv;
				_prev_hw_ocv_src = _hw_ocv_src;
				_hw_ocv = _sw_ocv;
				_hw_ocv_src = FROM_SW_OCV;
				p->flag_hw_ocv_unreliable = true;
				_flag_unreliable = 1;
			}
		} else {
			/* fixme: swocv is workaround */
			/* plug charger poweron but charger not ready */
			/* should use swocv to workaround */
			_hw_ocv = _sw_ocv;
			_hw_ocv_src = FROM_SW_OCV;
			if (_hw_ocv_chgin_rdy != 1) {
				if (abs(_hw_ocv - _sw_ocv) > now_thr) {
					_prev_hw_ocv = _hw_ocv;
					_prev_hw_ocv_src = _hw_ocv_src;
					_hw_ocv = _sw_ocv;
					_hw_ocv_src = FROM_SW_OCV;
					p->flag_hw_ocv_unreliable = true;
					_flag_unreliable = 1;
				}
			}
		}
	} else {
		if (_hw_ocv_59_pon_rdy == 0) {
			_hw_ocv = _sw_ocv;
			_hw_ocv_src = FROM_SW_OCV;
		}
	}

	/* final chance to check hwocv */
	if (gm != NULL)
		if (_hw_ocv < 28000 && (gm->disableGM30 == 0)) {
			bm_err("[%s] ERROR, _hw_ocv=%d  src:%d, force use swocv\n",
			__func__, _hw_ocv, _hw_ocv_src);
			_hw_ocv = _sw_ocv;
			_hw_ocv_src = FROM_SW_OCV;
		}

	*val = _hw_ocv;

	zcvinfo->charger_zcv = _hw_ocv_chgin;
	zcvinfo->pmic_rdy = _hw_ocv_59_pon_rdy;
	zcvinfo->pmic_zcv = _hw_ocv_59_pon;
	zcvinfo->pmic_in_zcv = _hw_ocv_59_plugin;
	zcvinfo->swocv = _sw_ocv;
	zcvinfo->zcv_from = _hw_ocv_src;
	zcvinfo->zcv_tmp = now_temp;

	if (zcvinfo->zcv_1st_read == false) {
		zcvinfo->charger_zcv_1st = zcvinfo->charger_zcv;
		zcvinfo->pmic_rdy_1st = zcvinfo->pmic_rdy;
		zcvinfo->pmic_zcv_1st = zcvinfo->pmic_zcv;
		zcvinfo->pmic_in_zcv_1st = zcvinfo->pmic_in_zcv;
		zcvinfo->swocv_1st = zcvinfo->swocv;
		zcvinfo->zcv_from_1st = zcvinfo->zcv_from;
		zcvinfo->zcv_tmp_1st = zcvinfo->zcv_tmp;
		zcvinfo->zcv_1st_read = true;
	}

	gauge_dev->fg_hw_info.pmic_zcv = _hw_ocv_59_pon;
	gauge_dev->fg_hw_info.pmic_zcv_rdy = _hw_ocv_59_pon_rdy;
	gauge_dev->fg_hw_info.charger_zcv = _hw_ocv_chgin;
	gauge_dev->fg_hw_info.hw_zcv = _hw_ocv;

	bm_err("[%s] g_fg_is_charger_exist %d _hw_ocv_chgin_rdy %d pl:%d %d\n",
		__func__, fg_is_charger_exist, _hw_ocv_chgin_rdy,
		zcvinfo->pl_charging_status, zcvinfo->moniter_plchg_bit);
	bm_err("[%s] _hw_ocv %d _sw_ocv %d now_thr %d\n",
		__func__, _prev_hw_ocv, _sw_ocv, now_thr);
	bm_err("[%s] _hw_ocv %d _hw_ocv_src %d _prev_hw_ocv %d _prev_hw_ocv_src %d _flag_unreliable %d\n",
		__func__, _hw_ocv, _hw_ocv_src, _prev_hw_ocv,
		_prev_hw_ocv_src, _flag_unreliable);
	bm_err("[%s] _hw_ocv_59_pon_rdy %d _hw_ocv_59_pon %d _hw_ocv_59_plugin %d _hw_ocv_chgin %d _sw_ocv %d now_temp %d now_thr %d\n",
		__func__, _hw_ocv_59_pon_rdy, _hw_ocv_59_pon,
		_hw_ocv_59_plugin, _hw_ocv_chgin, _sw_ocv,
		now_temp, now_thr);

	return 0;
}

static int initial_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	int bat_flag = 0;
	int is_charger_exist;
	int rev_val = 0;

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_NAG_PRD_SEL_ADDR,
		PMIC_AUXADC_NAG_PRD_SEL_MASK << PMIC_AUXADC_NAG_PRD_SEL_SHIFT,
		2 << PMIC_AUXADC_NAG_PRD_SEL_SHIFT);

	fgauge_get_info(gauge,
		GAUGE_PROP_BAT_PLUG_STATUS, &bat_flag);
	fgauge_get_info(gauge,
		GAUGE_PROP_PL_CHARGING_STATUS, &is_charger_exist);

	regmap_read(gauge->regmap, PMIC_RG_SYSTEM_INFO_CON0_ADDR, &rev_val);
	bm_err("bat_plug:%d chr:%d info:0x%x\n",
		bat_flag, is_charger_exist, rev_val);

	gauge->hw_status.pl_charger_status = is_charger_exist;

	if (is_charger_exist == 1) {
		gauge->hw_status.is_bat_plugout = 1;
		fgauge_set_info(gauge, GAUGE_PROP_2SEC_REBOOT, 0);
	} else {
		if (bat_flag == 0)
			gauge->hw_status.is_bat_plugout = 1;
		else
			gauge->hw_status.is_bat_plugout = 0;
	}

	fgauge_set_info(gauge, GAUGE_PROP_BAT_PLUG_STATUS, 1);
	/*[12:8], 5 bits*/
	gauge->hw_status.bat_plug_out_time = 31;

	fgauge_read_RTC_boot_status(gauge);

	return 1;
}

static int battery_current_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	*val = instant_current(gauge);

	return 0;
}

static int hw_version_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{

	*val = GAUGE_HW_V2000;

	return 0;
}

static int rtc_ui_soc_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	u8 rtc_value;
	int rtc_ui_soc = 0;

	rtc_value = get_rtc_spare_fg_value(gauge);
	rtc_ui_soc = (rtc_value & 0x7f);

	*val = rtc_ui_soc;

	if (rtc_ui_soc > 100 || rtc_ui_soc < 0)
		bm_err("[%s]ERR!rtc=0x%x,ui_soc=%d\n", rtc_value, rtc_ui_soc);
	else
		bm_debug("[%s]rtc=0x%x,ui_soc=%d\n", rtc_value, rtc_ui_soc);

	return 0;
}

static int rtc_ui_soc_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	u8 spare3_reg = get_rtc_spare_fg_value(gauge);
	int spare3_reg_valid = 0;
	int new_spare3_reg = 0;

	spare3_reg_valid = (spare3_reg & 0x80);
	new_spare3_reg = spare3_reg_valid + val;

	set_rtc_spare_fg_value(gauge, new_spare3_reg);

	bm_debug("[%s] ui_soc=%d, spare3_reg=0x%x, valid:%d, new_spare3_reg:0x%x\n",
		__func__, val, spare3_reg,
		spare3_reg_valid, new_spare3_reg);
	return 1;
}

static int gauge_initialized_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	int fg_reset_status;

	regmap_read(gauge->regmap, PMIC_FG_RSTB_STATUS_ADDR, &fg_reset_status);
	*val =
		(fg_reset_status & (PMIC_FG_RSTB_STATUS_MASK
		<< PMIC_FG_RSTB_STATUS_SHIFT))
		>> PMIC_FG_RSTB_STATUS_SHIFT;

	return 0;
}

static int gauge_initialized_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	regmap_update_bits(gauge->regmap,
		PMIC_FG_RSTB_STATUS_ADDR,
		PMIC_FG_RSTB_STATUS_MASK
		<< PMIC_FG_RSTB_STATUS_SHIFT,
		val << PMIC_FG_RSTB_STATUS_SHIFT);

	return 0;
}

static int battery_exist_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	unsigned int regval;

#if defined(CONFIG_FPGA_EARLY_PORTING)
	*val = 0;
	return 0;
#endif

	regmap_read(gauge->regmap, PMIC_AD_BATON_UNDET_ADDR, &regval);
	regval =
		(regval & (PMIC_AD_BATON_UNDET_MASK
		<< PMIC_AD_BATON_UNDET_SHIFT))
		>> PMIC_AD_BATON_UNDET_SHIFT;

	if (regval == 0)
		*val = 1;
	else {
		*val = 0;
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_ADDR,
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_MASK
			<< PMIC_AUXADC_ADC_RDY_PWRON_CLR_SHIFT,
			1 << PMIC_AUXADC_ADC_RDY_PWRON_CLR_SHIFT);
		mdelay(1);
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_ADDR,
			PMIC_AUXADC_ADC_RDY_PWRON_CLR_MASK
			<< PMIC_AUXADC_ADC_RDY_PWRON_CLR_SHIFT,
			0 << PMIC_AUXADC_ADC_RDY_PWRON_CLR_SHIFT);
	}

	return 0;
}

static int bat_vol_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	int ret;

	if (!IS_ERR(gauge->chan_bat_voltage)) {
		ret = iio_read_channel_processed(gauge->chan_bat_voltage, val);
		if (ret < 0)
			bm_err("[%s]read fail,ret=%d\n", __func__, ret);
	} else {
		bm_err("[%s]chan error\n", __func__);
		ret = -ENOTSUPP;
	}

	return ret;
}

static int battery_temperature_adc_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	int ret;

	if (!IS_ERR(gauge->chan_bat_temp)) {
		ret = iio_read_channel_processed(gauge->chan_bat_temp, val);
		if (ret < 0)
			bm_err("[%s]read fail,ret=%d\n", __func__, ret);
	} else {
		bm_err("[%s]chan error\n", __func__);
		ret = -ENOTSUPP;
	}

	return ret;
}

static int bif_voltage_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	int ret;

	if (!IS_ERR(gauge->chan_bif)) {
		ret = iio_read_channel_processed(gauge->chan_bif, val);
		if (ret < 0)
			bm_err("[%s]read fail,ret=%d\n", __func__, ret);
	} else {
		bm_err("[%s]chan error\n", __func__);
		ret = -ENOTSUPP;
	}

	return ret;
}

static int ptim_battery_voltage_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	int ret;

	if (!IS_ERR(gauge->chan_ptim_bat_voltage)) {
		ret = iio_read_channel_processed(
			gauge->chan_ptim_bat_voltage, val);
		if (ret < 0)
			bm_err("[%s]read fail,ret=%d\n", __func__, ret);
	} else {
		bm_err("[%s]chan error\n", __func__);
		ret = -ENOTSUPP;
	}

	return ret;
}

static int ptim_resist_get(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int *val)
{
	int ret;

	if (!IS_ERR(gauge->chan_ptim_r)) {
		ret = iio_read_channel_processed(
			gauge->chan_ptim_r, val);
		if (ret < 0)
			bm_err("[%s]read fail,ret=%d\n", __func__, ret);
	} else {
		bm_err("[%s]chan error\n", __func__);
		ret = -ENOTSUPP;
	}

	return ret;
}

static int bat_temp_froze_en_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	if (val != 0)
		val = 1;
	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_BAT_TEMP_FROZE_EN_ADDR,
		PMIC_AUXADC_BAT_TEMP_FROZE_EN_MASK
		<< PMIC_AUXADC_BAT_TEMP_FROZE_EN_SHIFT,
		val << PMIC_AUXADC_BAT_TEMP_FROZE_EN_SHIFT);
	return 0;
}

static int coulomb_interrupt_ht_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	unsigned int temp_car_15_0 = 0;
	unsigned int temp_car_31_16 = 0;
	unsigned int uvalue32_car_msb = 0;
	signed int upperbound = 0;
	signed int upperbound_31_16 = 0, upperbound_15_00 = 0;
	signed int value32_car;
	long long car = val;
	int r_fg_value;
	int car_tune_value;

	r_fg_value = gauge->hw_status.r_fg_value;
	car_tune_value = gauge->gm->fg_cust_data.car_tune_value;
	bm_debug("%s car=%d\n", __func__, val);
	if (car == 0) {
		disable_gauge_irq(gauge, COULOMB_H_IRQ);
		return 0;
	}

	pre_gauge_update(gauge);

	regmap_read(gauge->regmap, PMIC_FG_CAR_15_00_ADDR, &temp_car_15_0);
	temp_car_15_0 =
		(temp_car_15_0 &
		(PMIC_FG_CAR_15_00_MASK << PMIC_FG_CAR_15_00_SHIFT))
		>> PMIC_FG_CAR_15_00_SHIFT;

	regmap_read(gauge->regmap, PMIC_FG_CAR_31_16_ADDR, &temp_car_31_16);
	temp_car_31_16 =
		(temp_car_31_16 &
		(PMIC_FG_CAR_31_16_MASK << PMIC_FG_CAR_31_16_SHIFT))
		>> PMIC_FG_CAR_31_16_SHIFT;

	post_gauge_update(gauge);

	uvalue32_car_msb = (temp_car_31_16 & 0x8000) >> 15;
	value32_car = temp_car_15_0 & 0xffff;
	value32_car |= (temp_car_31_16 & 0xffff) << 16;

	bm_debug("[%s] FG_CAR = 0x%x:%d uvalue32_car_msb:0x%x 0x%x 0x%x\r\n",
		__func__, value32_car, value32_car, uvalue32_car_msb,
		temp_car_15_0,
		temp_car_31_16);

#if defined(__LP64__) || defined(_LP64)
	car = car * 100000 / UNIT_CHARGE;
#else
	car = div_s64(car * 100000, UNIT_CHARGE);
#endif

	if (r_fg_value != DEFAULT_R_FG)
#if defined(__LP64__) || defined(_LP64)
		car = (car * r_fg_value) /
			DEFAULT_R_FG;
#else
		car = div_s64(car * r_fg_value,
			DEFAULT_R_FG);
#endif

#if defined(__LP64__) || defined(_LP64)
	car = ((car * 1000) / car_tune_value);
#else
	car = div_s64((car * 1000), car_tune_value);
#endif

	upperbound = value32_car;

	bm_debug("[%s] upper = 0x%x:%d diff_car=0x%llx:%lld\r\n",
		 __func__, upperbound, upperbound, car, car);

	upperbound = upperbound + car;

	upperbound_31_16 = (upperbound & 0xffff0000) >> 16;
	upperbound_15_00 = (upperbound & 0xffff);

	bm_debug("[%s] final upper = 0x%x:%d car=0x%llx:%lld\r\n",
		 __func__, upperbound, upperbound, car, car);

	bm_debug("[%s] final upper 0x%x 0x%x 0x%x car=0x%llx\n",
		 __func__,
		upperbound, upperbound_31_16, upperbound_15_00, car);

	disable_gauge_irq(gauge, COULOMB_H_IRQ);

	regmap_update_bits(gauge->regmap,
		PMIC_FG_BAT_HTH_15_00_ADDR,
		PMIC_FG_BAT_HTH_15_00_MASK << PMIC_FG_BAT_HTH_15_00_SHIFT,
		upperbound_15_00 << PMIC_FG_BAT_HTH_15_00_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_FG_BAT_HTH_31_16_ADDR,
		PMIC_FG_BAT_HTH_31_16_MASK << PMIC_FG_BAT_HTH_31_16_SHIFT,
		upperbound_31_16 << PMIC_FG_BAT_HTH_31_16_SHIFT);
	mdelay(1);

	enable_gauge_irq(gauge, COULOMB_H_IRQ);

	bm_debug("[%s] high:0x%x 0x%x car_value:%d car:%d irq:%d\r\n",
		__func__,
		upperbound_15_00,
		upperbound_31_16,
		val, value32_car,
		gauge->irq_no[COULOMB_H_IRQ]);

	return 0;
}

static int coulomb_interrupt_lt_set(struct mtk_gauge *gauge,
	 struct mtk_gauge_sysfs_field_info *attr, int val)
{
	unsigned int temp_car_15_0 = 0;
	unsigned int temp_car_31_16 = 0;
	unsigned int uvalue32_car_msb = 0;
	signed int lowbound = 0;
	signed int lowbound_31_16 = 0, lowbound_15_00 = 0;
	signed int value32_car;
	long long car = val;
	int r_fg_value;
	int car_tune_value;

	r_fg_value = gauge->hw_status.r_fg_value;
	car_tune_value = gauge->gm->fg_cust_data.car_tune_value;
	bm_debug("%s car=%d\n", __func__, val);
	if (car == 0) {
		disable_gauge_irq(gauge, COULOMB_L_IRQ);
		return 0;
	}

	pre_gauge_update(gauge);

	regmap_read(gauge->regmap, PMIC_FG_CAR_15_00_ADDR, &temp_car_15_0);
	temp_car_15_0 =
		(temp_car_15_0 &
		(PMIC_FG_CAR_15_00_MASK << PMIC_FG_CAR_15_00_SHIFT))
		>> PMIC_FG_CAR_15_00_SHIFT;

	regmap_read(gauge->regmap, PMIC_FG_CAR_31_16_ADDR, &temp_car_31_16);
	temp_car_31_16 =
		(temp_car_31_16 &
		(PMIC_FG_CAR_31_16_MASK << PMIC_FG_CAR_31_16_SHIFT))
		>> PMIC_FG_CAR_31_16_SHIFT;

	post_gauge_update(gauge);

	uvalue32_car_msb =
		(temp_car_31_16 & 0x8000) >> 15;
	value32_car = temp_car_15_0 & 0xffff;
	value32_car |= (temp_car_31_16 & 0xffff) << 16;

	bm_debug("[%s] FG_CAR = 0x%x:%d uvalue32_car_msb:0x%x 0x%x 0x%x\r\n",
		__func__,
		value32_car, value32_car, uvalue32_car_msb,
		temp_car_15_0,
		temp_car_31_16);

	/* gap to register-base */
#if defined(__LP64__) || defined(_LP64)
	car = car * 100000 / UNIT_CHARGE;
	/* car * 1000 * 100 */
#else
	car = div_s64(car * 100000, UNIT_CHARGE);
#endif

	if (r_fg_value != DEFAULT_R_FG)
#if defined(__LP64__) || defined(_LP64)
		car = (car * r_fg_value) /
			DEFAULT_R_FG;
#else
		car = div_s64(car * r_fg_value,
			DEFAULT_R_FG);
#endif

#if defined(__LP64__) || defined(_LP64)
	car = ((car * 1000) / car_tune_value);
#else
	car = div_s64((car * 1000), car_tune_value);
#endif

	lowbound = value32_car;

	bm_debug("[%s]low=0x%x:%d diff_car=0x%llx:%lld\r\n",
		 __func__, lowbound, lowbound, car, car);

	lowbound = lowbound - car;

	lowbound_31_16 = (lowbound & 0xffff0000) >> 16;
	lowbound_15_00 = (lowbound & 0xffff);

	bm_debug("[%s]final low=0x%x:%d car=0x%llx:%lld\r\n",
		 __func__, lowbound, lowbound, car, car);

	bm_debug("[%s] final low 0x%x 0x%x 0x%x car=0x%llx\n",
		 __func__, lowbound, lowbound_31_16, lowbound_15_00, car);

	disable_gauge_irq(gauge, COULOMB_L_IRQ);
	regmap_update_bits(gauge->regmap,
		PMIC_FG_BAT_LTH_15_00_ADDR,
		PMIC_FG_BAT_LTH_15_00_MASK << PMIC_FG_BAT_LTH_15_00_SHIFT,
		lowbound_15_00 << PMIC_FG_BAT_LTH_15_00_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_FG_BAT_LTH_31_16_ADDR,
		PMIC_FG_BAT_LTH_31_16_MASK << PMIC_FG_BAT_LTH_31_16_SHIFT,
		lowbound_31_16 << PMIC_FG_BAT_LTH_31_16_SHIFT);
	mdelay(1);
	enable_gauge_irq(gauge, COULOMB_L_IRQ);

	bm_debug("[%s] low:0x%x 0x%x car_value:%d car:%d irq:%d\r\n",
		__func__, lowbound_15_00,
		lowbound_31_16,
		val, value32_car,
		gauge->irq_no[COULOMB_L_IRQ]);

	return 0;
}

static void enable_lbat2_en(struct mtk_gauge *gauge)
{
	if (gauge->vbat_l_en == true || gauge->vbat_h_en == true)
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_LBAT2_EN_ADDR,
			PMIC_AUXADC_LBAT2_EN_MASK << PMIC_AUXADC_LBAT2_EN_SHIFT,
			1 << PMIC_AUXADC_LBAT2_EN_SHIFT);

	if (gauge->vbat_l_en == false && gauge->vbat_h_en == false)
		regmap_update_bits(gauge->regmap,
			PMIC_AUXADC_LBAT2_EN_ADDR,
			PMIC_AUXADC_LBAT2_EN_MASK << PMIC_AUXADC_LBAT2_EN_SHIFT,
			0 << PMIC_AUXADC_LBAT2_EN_SHIFT);
}

static int en_h_vbat_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	if (val != 0) {
		val = 1;
		enable_gauge_irq(gauge, VBAT_H_IRQ);
	} else
		disable_gauge_irq(gauge, VBAT_H_IRQ);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_IRQ_EN_MAX_ADDR,
		PMIC_AUXADC_LBAT2_IRQ_EN_MAX_MASK
		<< PMIC_AUXADC_LBAT2_IRQ_EN_MAX_SHIFT,
		val << PMIC_AUXADC_LBAT2_IRQ_EN_MAX_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_DET_MAX_ADDR,
		PMIC_AUXADC_LBAT2_DET_MAX_MASK
		<< PMIC_AUXADC_LBAT2_DET_MAX_SHIFT,
		val << PMIC_AUXADC_LBAT2_DET_MAX_SHIFT);

	gauge->vbat_h_en = val;
	enable_lbat2_en(gauge);

	return 0;
}

static int en_l_vbat_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int val)
{
	static int cnt;

	bm_debug("%s %d %d\n", __func__,
		cnt, val);
	if (val != 0)
		cnt++;
	else
		cnt--;

	if (val != 0) {
		val = 1;
		enable_gauge_irq(gauge, VBAT_L_IRQ);
	} else
		disable_gauge_irq(gauge, VBAT_L_IRQ);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_IRQ_EN_MIN_ADDR,
		PMIC_AUXADC_LBAT2_IRQ_EN_MIN_MASK
		<< PMIC_AUXADC_LBAT2_IRQ_EN_MIN_SHIFT,
		val << PMIC_AUXADC_LBAT2_IRQ_EN_MIN_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_DET_MIN_ADDR,
		PMIC_AUXADC_LBAT2_DET_MIN_MASK
		<< PMIC_AUXADC_LBAT2_IRQ_EN_MIN_SHIFT,
		val << PMIC_AUXADC_LBAT2_IRQ_EN_MIN_SHIFT);

	gauge->vbat_l_en = val;
	enable_lbat2_en(gauge);

	return 0;
}

static void switch_vbat2_det_time(int _prd, int *value)
{
	if (_prd >= 1 && _prd < 3)
		*value = 0;
	else if (_prd >= 3 && _prd < 5)
		*value = 1;
	else if (_prd >= 5 && _prd < 10)
		*value = 2;
	else if (_prd >= 10)
		*value = 3;
}

static void switch_vbat2_debt_counter(int _prd, int *value)
{
	if (_prd >= 1 && _prd < 2)
		*value = 0;
	else if (_prd >= 2 && _prd < 4)
		*value = 1;
	else if (_prd >= 4 && _prd < 8)
		*value = 2;
	else if (_prd >= 8)
		*value = 3;
}

static int vbat_ht_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int threshold)
{
	int vbat2_h_th_mv =  threshold;
	int vbat2_h_th_reg = mv_to_reg_12_value(gauge, vbat2_h_th_mv);
	int vbat2_det_counter = 0;
	int vbat2_det_time = 0;

	switch_vbat2_det_time(
		gauge->hw_status.vbat2_det_time,
		&vbat2_det_time);

	switch_vbat2_debt_counter(
		gauge->hw_status.vbat2_det_counter,
		&vbat2_det_counter);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_VOLT_MAX_ADDR,
		PMIC_AUXADC_LBAT2_VOLT_MAX_MASK
		<< PMIC_AUXADC_LBAT2_VOLT_MAX_SHIFT,
		vbat2_h_th_reg << PMIC_AUXADC_LBAT2_VOLT_MAX_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_DET_PRD_SEL_ADDR,
		PMIC_AUXADC_LBAT2_DET_PRD_SEL_MASK
		<< PMIC_AUXADC_LBAT2_DET_PRD_SEL_SHIFT,
		vbat2_det_time << PMIC_AUXADC_LBAT2_DET_PRD_SEL_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_DEBT_MAX_SEL_ADDR,
		PMIC_AUXADC_LBAT2_DEBT_MAX_SEL_MASK
		<< PMIC_AUXADC_LBAT2_DEBT_MAX_SEL_SHIFT,
		vbat2_det_counter << PMIC_AUXADC_LBAT2_DEBT_MAX_SEL_SHIFT);

	bm_debug("[fg_set_vbat2_h_th] thr:%d [0x%x %d 0x%x %d 0x%x]\n",
		threshold, vbat2_h_th_reg,
		gauge->hw_status.vbat2_det_time, vbat2_det_time,
		gauge->hw_status.vbat2_det_counter, vbat2_det_counter);

	return 0;
}

static int reset_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int threshold)
{
	unsigned int ret = 0;

	bm_err("[fgauge_hw_reset]\n");
	regmap_update_bits(gauge->regmap,
		MT6359P_FGADC_CON1,
		0x0F00, 0x0630);
	bm_err("[fgauge_hw_reset] reset fgadc car ret =%d\n", ret);
	mdelay(1);
	regmap_update_bits(gauge->regmap,
		MT6359P_FGADC_CON1,
		0x0F00, 0x0030);
	return 0;
}

static int vbat_lt_set(struct mtk_gauge *gauge,
	struct mtk_gauge_sysfs_field_info *attr, int threshold)
{
	int vbat2_l_th_mv =  threshold;
	int vbat2_l_th_reg = mv_to_reg_12_value(gauge, vbat2_l_th_mv);
	int vbat2_det_counter = 0;
	int vbat2_det_time = 0;

	switch_vbat2_det_time(
		gauge->hw_status.vbat2_det_time,
		&vbat2_det_time);

	switch_vbat2_debt_counter(
		gauge->hw_status.vbat2_det_counter,
		&vbat2_det_counter);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_VOLT_MIN_ADDR,
		PMIC_AUXADC_LBAT2_VOLT_MIN_MASK
		<< PMIC_AUXADC_LBAT2_VOLT_MIN_SHIFT,
		vbat2_l_th_reg << PMIC_AUXADC_LBAT2_VOLT_MIN_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_DET_PRD_SEL_ADDR,
		PMIC_AUXADC_LBAT2_DET_PRD_SEL_MASK
		<< PMIC_AUXADC_LBAT2_DET_PRD_SEL_SHIFT,
		vbat2_det_time << PMIC_AUXADC_LBAT2_DET_PRD_SEL_SHIFT);

	regmap_update_bits(gauge->regmap,
		PMIC_AUXADC_LBAT2_DEBT_MIN_SEL_ADDR,
		PMIC_AUXADC_LBAT2_DEBT_MIN_SEL_MASK
		<< PMIC_AUXADC_LBAT2_DEBT_MIN_SEL_SHIFT,
		vbat2_det_counter << PMIC_AUXADC_LBAT2_DEBT_MIN_SEL_SHIFT);

	bm_debug("[fg_set_vbat2_l_th] thr:%d [0x%x %d 0x%x %d 0x%x]\n",
		threshold,
		vbat2_l_th_reg,
		gauge->hw_status.vbat2_det_time, vbat2_det_time,
		gauge->hw_status.vbat2_det_counter, vbat2_det_counter);

	return 0;
}

void dump_nag(struct mtk_gauge *gauge)
{
	int nag[12];

	regmap_read(gauge->regmap, 0x11be, &nag[0]);
	regmap_read(gauge->regmap, 0x11c0, &nag[1]);
	regmap_read(gauge->regmap, 0x11c2, &nag[2]);
	regmap_read(gauge->regmap, 0x11c4, &nag[3]);
	regmap_read(gauge->regmap, 0x11c6, &nag[4]);
	regmap_read(gauge->regmap, 0x11c8, &nag[5]);
	regmap_read(gauge->regmap, 0x11ca, &nag[6]);
	regmap_read(gauge->regmap, 0x11cc, &nag[7]);
	regmap_read(gauge->regmap, 0x11ce, &nag[8]);
	regmap_read(gauge->regmap, 0x11d0, &nag[9]);
	regmap_read(gauge->regmap, 0x11d2, &nag[10]);
	regmap_read(gauge->regmap, 0x11d4, &nag[11]);

	bm_err("nag %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
		nag[0], nag[1], nag[2], nag[3], nag[4], nag[5],
		nag[6], nag[7], nag[8], nag[9], nag[10], nag[11],
		reg_to_mv_value(nag[1]),
		reg_to_mv_value(nag[2]),
		reg_to_mv_value(nag[6])
		);
}

static ssize_t gauge_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy;
	struct mtk_gauge *gauge;
	struct mtk_gauge_sysfs_field_info *gauge_attr;
	int val;
	ssize_t ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	psy = dev_get_drvdata(dev);
	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);

	gauge_attr = container_of(attr,
		struct mtk_gauge_sysfs_field_info, attr);
	if (gauge_attr->set != NULL) {
		mutex_lock(&gauge->ops_lock);
		gauge_attr->set(gauge, gauge_attr, val);
		mutex_unlock(&gauge->ops_lock);
	}

	return count;
}

static ssize_t gauge_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct power_supply *psy;
	struct mtk_gauge *gauge;
	struct mtk_gauge_sysfs_field_info *gauge_attr;
	int val = 0;
	ssize_t count;

	psy = dev_get_drvdata(dev);
	gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);

	gauge_attr = container_of(attr,
		struct mtk_gauge_sysfs_field_info, attr);
	if (gauge_attr->get != NULL) {
		mutex_lock(&gauge->ops_lock);
		gauge_attr->get(gauge, gauge_attr, &val);
		mutex_unlock(&gauge->ops_lock);
	}

	count = scnprintf(buf, PAGE_SIZE, "%d\n", val);
	return count;
}

/* Must be in the same order as GAUGE_PROP_* */
static struct mtk_gauge_sysfs_field_info mt6359_sysfs_field_tbl[] = {
	GAUGE_SYSFS_FIELD_WO(initial_set,
			GAUGE_PROP_INITIAL),
	GAUGE_SYSFS_FIELD_RO(battery_current_get,
		GAUGE_PROP_BATTERY_CURRENT),
	GAUGE_SYSFS_FIELD_RO(coulomb_get,
		GAUGE_PROP_COULOMB),
	GAUGE_SYSFS_FIELD_WO(coulomb_interrupt_ht_set,
		GAUGE_PROP_COULOMB_HT_INTERRUPT),
	GAUGE_SYSFS_FIELD_WO(coulomb_interrupt_lt_set,
		GAUGE_PROP_COULOMB_LT_INTERRUPT),
	GAUGE_SYSFS_FIELD_RO(battery_exist_get,
		GAUGE_PROP_BATTERY_EXIST),
	GAUGE_SYSFS_FIELD_RO(hw_version_get,
		GAUGE_PROP_HW_VERSION),
	GAUGE_SYSFS_FIELD_RO(bat_vol_get,
		GAUGE_PROP_BATTERY_VOLTAGE),
	GAUGE_SYSFS_FIELD_RO(battery_temperature_adc_get,
		GAUGE_PROP_BATTERY_TEMPERATURE_ADC),
	GAUGE_SYSFS_FIELD_RO(bif_voltage_get,
		GAUGE_PROP_BIF_VOLTAGE),
	GAUGE_SYSFS_FIELD_WO(en_h_vbat_set,
		GAUGE_PROP_EN_HIGH_VBAT_INTERRUPT),
	GAUGE_SYSFS_FIELD_WO(en_l_vbat_set,
		GAUGE_PROP_EN_LOW_VBAT_INTERRUPT),
	GAUGE_SYSFS_FIELD_WO(vbat_ht_set,
		GAUGE_PROP_VBAT_HT_INTR_THRESHOLD),
	GAUGE_SYSFS_FIELD_WO(vbat_lt_set,
		GAUGE_PROP_VBAT_LT_INTR_THRESHOLD),
	GAUGE_SYSFS_FIELD_RW(rtc_ui_soc, rtc_ui_soc_set, rtc_ui_soc_get,
		GAUGE_PROP_RTC_UI_SOC),
	GAUGE_SYSFS_FIELD_RO(ptim_battery_voltage_get,
		GAUGE_PROP_PTIM_BATTERY_VOLTAGE),
	GAUGE_SYSFS_FIELD_RO(ptim_resist_get,
		GAUGE_PROP_PTIM_RESIST),
	GAUGE_SYSFS_FIELD_WO(reset_set,
		GAUGE_PROP_RESET),
	GAUGE_SYSFS_FIELD_RO(boot_zcv_get,
		GAUGE_PROP_BOOT_ZCV),
	GAUGE_SYSFS_FIELD_RO(zcv_get,
		GAUGE_PROP_ZCV),
	GAUGE_SYSFS_FIELD_RO(zcv_current_get,
		GAUGE_PROP_ZCV_CURRENT),
	GAUGE_SYSFS_FIELD_RO(nafg_cnt_get,
		GAUGE_PROP_NAFG_CNT),
	GAUGE_SYSFS_FIELD_RO(nafg_dltv_get,
		GAUGE_PROP_NAFG_DLTV),
	GAUGE_SYSFS_FIELD_RW(nafg_c_dltv, nafg_c_dltv_set, nafg_c_dltv_get,
		GAUGE_PROP_NAFG_C_DLTV),
	GAUGE_SYSFS_FIELD_WO(nafg_en_set,
		GAUGE_PROP_NAFG_EN),
	GAUGE_SYSFS_FIELD_WO(nafg_zcv_set,
		GAUGE_PROP_NAFG_ZCV),
	GAUGE_SYSFS_FIELD_RO(nafg_vbat_get,
		GAUGE_PROP_NAFG_VBAT),
	GAUGE_SYSFS_FIELD_WO(reset_fg_rtc_set,
		GAUGE_PROP_RESET_FG_RTC),
	GAUGE_SYSFS_FIELD_RW(gauge_initialized, gauge_initialized_set, gauge_initialized_get,
		GAUGE_PROP_GAUGE_INITIALIZED),
	GAUGE_SYSFS_FIELD_RO(average_current_get,
		GAUGE_PROP_AVERAGE_CURRENT),
	GAUGE_SYSFS_FIELD_WO(bat_plugout_en_set,
		GAUGE_PROP_BAT_PLUGOUT_EN),
	GAUGE_SYSFS_FIELD_WO(zcv_intr_threshold_set,
		GAUGE_PROP_ZCV_INTR_THRESHOLD),
	GAUGE_SYSFS_FIELD_WO(zcv_intr_en_set,
		GAUGE_PROP_ZCV_INTR_EN),
	GAUGE_SYSFS_FIELD_WO(soff_reset_set,
		GAUGE_PROP_SOFF_RESET),
	GAUGE_SYSFS_FIELD_WO(ncar_reset_set,
		GAUGE_PROP_NCAR_RESET),
	GAUGE_SYSFS_FIELD_WO(bat_cycle_intr_threshold_set,
		GAUGE_PROP_BAT_CYCLE_INTR_THRESHOLD),
	GAUGE_SYSFS_FIELD_WO(hw_info_set,
		GAUGE_PROP_HW_INFO),
	GAUGE_SYSFS_FIELD_WO(event_set,
		GAUGE_PROP_EVENT),
	GAUGE_SYSFS_FIELD_WO(en_bat_tmp_ht_set,
		GAUGE_PROP_EN_BAT_TMP_HT),
	GAUGE_SYSFS_FIELD_WO(en_bat_tmp_lt_set,
		GAUGE_PROP_EN_BAT_TMP_LT),
	GAUGE_SYSFS_FIELD_WO(bat_tmp_ht_threshold_set,
		GAUGE_PROP_BAT_TMP_HT_THRESHOLD),
	GAUGE_SYSFS_FIELD_WO(bat_tmp_lt_threshold_set,
		GAUGE_PROP_BAT_TMP_LT_THRESHOLD),
	GAUGE_SYSFS_INFO_FIELD_RW(
		info_2sec_reboot,
		GAUGE_PROP_2SEC_REBOOT),
	GAUGE_SYSFS_INFO_FIELD_RW(
		info_pl_charging_status,
		GAUGE_PROP_PL_CHARGING_STATUS),
	GAUGE_SYSFS_INFO_FIELD_RW(
		info_monitor_plchg_status,
		GAUGE_PROP_MONITER_PLCHG_STATUS),
	GAUGE_SYSFS_INFO_FIELD_RW(
		info_bat_plug_status,
		GAUGE_PROP_BAT_PLUG_STATUS),
	GAUGE_SYSFS_INFO_FIELD_RW(
		info_is_nvram_fail_mode,
		GAUGE_PROP_IS_NVRAM_FAIL_MODE),
	GAUGE_SYSFS_INFO_FIELD_RW(
		info_monitor_soff_validtime,
		GAUGE_PROP_MONITOR_SOFF_VALIDTIME),
	GAUGE_SYSFS_INFO_FIELD_RW(
		info_con0_soc, GAUGE_PROP_CON0_SOC),
	GAUGE_SYSFS_INFO_FIELD_RW(
		info_shutdown_car, GAUGE_PROP_SHUTDOWN_CAR),
	GAUGE_SYSFS_INFO_FIELD_RW(
		car_tune_value, GAUGE_PROP_CAR_TUNE_VALUE),
	GAUGE_SYSFS_INFO_FIELD_RW(
		r_fg_value, GAUGE_PROP_R_FG_VALUE),
	GAUGE_SYSFS_INFO_FIELD_RW(
		vbat2_detect_time, GAUGE_PROP_VBAT2_DETECT_TIME),
	GAUGE_SYSFS_INFO_FIELD_RW(
		vbat2_detect_counter, GAUGE_PROP_VBAT2_DETECT_COUNTER),
	GAUGE_SYSFS_FIELD_WO(
		bat_temp_froze_en_set, GAUGE_PROP_BAT_TEMP_FROZE_EN),
	GAUGE_SYSFS_FIELD_RO(battery_voltage_cali, GAUGE_PROP_BAT_EOC),
};

static struct attribute *
	mt6359_sysfs_attrs[GAUGE_PROP_MAX + 1];

static const struct attribute_group mt6359_sysfs_attr_group = {
	.attrs = mt6359_sysfs_attrs,
};

static void mt6359_sysfs_init_attrs(void)
{
	int i, limit = ARRAY_SIZE(mt6359_sysfs_field_tbl);

	for (i = 0; i < limit; i++)
		mt6359_sysfs_attrs[i] = &mt6359_sysfs_field_tbl[i].attr.attr;

	mt6359_sysfs_attrs[limit] = NULL; /* Has additional entry for this */
}

static int mt6359_sysfs_create_group(struct mtk_gauge *gauge)
{
	mt6359_sysfs_init_attrs();

	return sysfs_create_group(&gauge->psy->dev.kobj,
			&mt6359_sysfs_attr_group);
}

static void mt6359_gauge_shutdown(struct platform_device *pdev)
{
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;

	gauge = dev_get_drvdata(&pdev->dev);
	gm = gauge->gm;

	if (gm->shutdown != NULL)
		gm->shutdown(gm);
}

static int mt6359_gauge_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;

	gauge = dev_get_drvdata(&pdev->dev);
	gm = gauge->gm;

	if (gm->suspend != NULL)
		gm->suspend(gm, state);

	return 0;
}

static int mt6359_gauge_resume(struct platform_device *pdev)
{
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;

	gauge = dev_get_drvdata(&pdev->dev);
	gm = gauge->gm;

	if (gm->resume != NULL)
		gm->resume(gm);

	return 0;
}


signed int battery_meter_meta_tool_cali_car_tune(struct mtk_battery *gm,
	int meta_current)
{
	int cali_car_tune = 0;

	if (meta_current == 0)
		return gm->fg_cust_data.car_tune_value * 10;

	gm->gauge->hw_status.meta_current = meta_current;
	bm_err("%s meta_current=%d\n", __func__, meta_current);

	calculate_car_tune(gm->gauge);
	cali_car_tune = gm->gauge->hw_status.tmp_car_tune;

	bm_err("%s cali_car_tune=%d\n", __func__, cali_car_tune);

	return cali_car_tune;		/* 1000 base */
}

#if IS_ENABLED(CONFIG_COMPAT)
static long compat_adc_cali_ioctl(
struct file *filp, unsigned int cmd, unsigned long arg)
{
	int adc_out_datas[2] = { 1, 1 };

	bm_notice("%s 32bit IOCTL, cmd=0x%08x\n",
		__func__, cmd);
	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		bm_err("%s file has no f_op or no f_op->unlocked_ioctl.\n",
			__func__);
		return -ENOTTY;
	}

	if (sizeof(arg) != sizeof(adc_out_datas))
		return -EFAULT;

	switch (cmd) {
	case Get_META_BAT_VOL:
	case Get_META_BAT_SOC:
	case Get_META_BAT_CAR_TUNE_VALUE:
	case Set_META_BAT_CAR_TUNE_VALUE:
	case Set_BAT_DISABLE_NAFG:
	case Set_CARTUNE_TO_KERNEL: {
		bm_notice(
			"%s send to unlocked_ioctl cmd=0x%08x\n",
			__func__,
			cmd);
		return filp->f_op->unlocked_ioctl(
			filp, cmd,
			(unsigned long)compat_ptr(arg));
	}
		break;
	default:
		bm_err("%s unknown IOCTL: 0x%08x, %d\n",
			__func__, cmd, adc_out_datas[0]);
		break;
	}

	return 0;
}
#endif



static long adc_cali_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	int *user_data_addr;
	int ret = 0;
	int adc_in_data[2] = { 1, 1 };
	int adc_out_data[2] = { 1, 1 };
	int temp_car_tune;
	int isdisNAFG = 0;
	struct mtk_battery *gm;

	bm_notice("%s enter\n", __func__);

	gm = get_mtk_battery();
	mutex_lock(&gm->gauge->fg_mutex);
	user_data_addr = (int *)arg;
	ret = copy_from_user(adc_in_data, user_data_addr, sizeof(adc_in_data));
	if (adc_in_data[1] < 0) {
		bm_err("%s unknown data: %d\n", __func__, adc_in_data[1]);
		mutex_unlock(&gm->gauge->fg_mutex);
		return -EFAULT;
	}

	switch (cmd) {
		/* add for meta tool------------------------------- */

	case Get_META_BAT_VOL:
		adc_out_data[0] =
			gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);
		if (copy_to_user(user_data_addr, adc_out_data,
			sizeof(adc_out_data))) {
			mutex_unlock(&gm->gauge->fg_mutex);
			return -EFAULT;
		}

		bm_notice("**** unlocked_ioctl :Get_META_BAT_VOL Done!\n");
		break;
	case Get_META_BAT_SOC:
		adc_out_data[0] = gm->ui_soc;

		if (copy_to_user(user_data_addr, adc_out_data,
			sizeof(adc_out_data))) {
			mutex_unlock(&gm->gauge->fg_mutex);
			return -EFAULT;
		}

		bm_notice("**** unlocked_ioctl :Get_META_BAT_SOC Done!\n");
		break;

	case Get_META_BAT_CAR_TUNE_VALUE:
		adc_out_data[0] = gm->fg_cust_data.car_tune_value;
		bm_err("Get_BAT_CAR_TUNE_VALUE, res=%d\n", adc_out_data[0]);

		if (copy_to_user(user_data_addr, adc_out_data,
			sizeof(adc_out_data))) {
			mutex_unlock(&gm->gauge->fg_mutex);
			return -EFAULT;
		}

		bm_notice("**** unlocked_ioctl :Get_META_BAT_CAR_TUNE_VALUE Done!\n");
		break;
	case Set_META_BAT_CAR_TUNE_VALUE:
		/* meta tool input: adc_in_data[1] (mA)*/
		/* Send cali_current to hal to calculate car_tune_value*/
		temp_car_tune =
			battery_meter_meta_tool_cali_car_tune(gm, adc_in_data[1]);

		/* return car_tune_value to meta tool in adc_out_data[0] */
		if (temp_car_tune >= 900 && temp_car_tune <= 1100)
			gm->fg_cust_data.car_tune_value = temp_car_tune;
		else
			bm_err("car_tune_value invalid:%d\n",
			temp_car_tune);

		adc_out_data[0] = temp_car_tune;

		if (copy_to_user(user_data_addr, adc_out_data,
			sizeof(adc_out_data))) {
			mutex_unlock(&gm->gauge->fg_mutex);
			return -EFAULT;
		}

		bm_err("**** unlocked_ioctl Set_BAT_CAR_TUNE_VALUE[%d], tmp_car_tune=%d result=%d, ret=%d\n",
			adc_in_data[1], adc_out_data[0], temp_car_tune,
			ret);

		break;

	case Set_BAT_DISABLE_NAFG:
		isdisNAFG = adc_in_data[1];

		if (isdisNAFG == 1) {
			gm->cmd_disable_nafg = true;
			wakeup_fg_algo_cmd(
				gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_DISABLE_NAFG, 1);
		} else if (isdisNAFG == 0) {
			gm->cmd_disable_nafg = false;
			wakeup_fg_algo_cmd(
				gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_DISABLE_NAFG, 0);
		}
		bm_debug("unlocked_ioctl Set_BAT_DISABLE_NAFG,isdisNAFG=%d [%d]\n",
			isdisNAFG, adc_in_data[1]);
		break;

		/* add bing meta tool------------------------------- */
	case Set_CARTUNE_TO_KERNEL:
		temp_car_tune = adc_in_data[1];
		if (temp_car_tune > 500 && temp_car_tune < 1500)
			gm->fg_cust_data.car_tune_value = temp_car_tune;

		bm_err("**** unlocked_ioctl Set_CARTUNE_TO_KERNEL[%d,%d], ret=%d\n",
			adc_in_data[0], adc_in_data[1], ret);
		break;
	default:
		bm_err("**** unlocked_ioctl unknown IOCTL: 0x%08x\n", cmd);
		mutex_unlock(&gm->gauge->fg_mutex);
		return -EINVAL;
	}

	mutex_unlock(&gm->gauge->fg_mutex);

	return 0;
}

static int adc_cali_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int adc_cali_release(struct inode *inode, struct file *file)
{
	return 0;
}



static const struct file_operations adc_cali_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = adc_cali_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_adc_cali_ioctl,
#endif
	.open = adc_cali_open,
	.release = adc_cali_release,
};


static int adc_cali_cdev_init(struct platform_device *pdev)
{
	int ret = 0;
	struct class_device *class_dev = NULL;
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	if (gm != NULL)
		mutex_init(&gm->gauge->fg_mutex);

	ret = alloc_chrdev_region(&bat_cali_devno, 0, 1, BAT_CALI_DEVNAME);
	if (ret)
		bm_err("Error: Can't Get Major number for adc_cali\n");

	bat_cali_cdev = cdev_alloc();
	bat_cali_cdev->owner = THIS_MODULE;
	bat_cali_cdev->ops = &adc_cali_fops;
	ret = cdev_add(bat_cali_cdev, bat_cali_devno, 1);
	if (ret)
		bm_err("adc_cali Error: cdev_add\n");

	bat_cali_major = MAJOR(bat_cali_devno);
	bat_cali_class = class_create(THIS_MODULE, BAT_CALI_DEVNAME);
	class_dev = (struct class_device *)device_create(bat_cali_class,
		NULL,
		bat_cali_devno,
		NULL, BAT_CALI_DEVNAME);

	return 0;
}

static void mtk_gauge_netlink_handler(struct sk_buff *skb)
{
	mtk_battery_netlink_handler(skb);
}

int bat_create_netlink(struct platform_device *pdev)
{
	struct mtk_gauge *gauge;
	struct netlink_kernel_cfg cfg = {
		.input = mtk_gauge_netlink_handler,
	};

	gauge = dev_get_drvdata(&pdev->dev);
	gauge->gm->mtk_battery_sk =
		netlink_kernel_create(&init_net, NETLINK_FGD, &cfg);

	if (gauge->gm->mtk_battery_sk == NULL) {
		bm_err("netlink_kernel_create error\n");
		return -EIO;
	}

	bm_err("[%s]netlink_kernel_create protol= %d\n",
		__func__, NETLINK_FGD);

	return 0;
}

static int mt6359_gauge_probe(struct platform_device *pdev)
{
	struct mtk_gauge *gauge;
	int ret;
	struct iio_channel *chan_bat_temp;

	bm_err("%s: starts\n", __func__);

	chan_bat_temp = devm_iio_channel_get(
		&pdev->dev, "pmic_battery_temp");
	if (IS_ERR(chan_bat_temp)) {
		bm_err("mt6359 requests probe deferral\n");
		return -EPROBE_DEFER;
	}

	gauge = devm_kzalloc(&pdev->dev, sizeof(*gauge), GFP_KERNEL);
	if (!gauge)
		return -ENOMEM;

	gauge->chip = (struct mt6397_chip *)dev_get_drvdata(
		pdev->dev.parent);
	gauge->regmap = gauge->chip->regmap;
	dev_set_drvdata(&pdev->dev, gauge);
	gauge->pdev = pdev;
	mutex_init(&gauge->ops_lock);

	gauge->irq_no[COULOMB_H_IRQ] =
		platform_get_irq_byname(pdev, "COULOMB_H");
	gauge->irq_no[COULOMB_L_IRQ] =
		platform_get_irq_byname(pdev, "COULOMB_L");
	gauge->irq_no[VBAT_H_IRQ] = platform_get_irq_byname(pdev, "VBAT_H");
	gauge->irq_no[VBAT_L_IRQ] = platform_get_irq_byname(pdev, "VBAT_L");
	gauge->irq_no[NAFG_IRQ] = platform_get_irq_byname(pdev, "NAFG");
	gauge->irq_no[BAT_PLUGOUT_IRQ] =
		platform_get_irq_byname(pdev, "BAT_OUT");
	gauge->irq_no[ZCV_IRQ] = platform_get_irq_byname(pdev, "ZCV");
	gauge->irq_no[FG_N_CHARGE_L_IRQ] = platform_get_irq_byname(pdev,
		"FG_N_CHARGE_L");
	gauge->irq_no[FG_IAVG_H_IRQ] =
		platform_get_irq_byname(pdev, "FG_IAVG_H");
	gauge->irq_no[FG_IAVG_L_IRQ] =
		platform_get_irq_byname(pdev, "FG_IAVG_L");
	gauge->irq_no[BAT_TMP_H_IRQ] =
		platform_get_irq_byname(pdev, "BAT_TMP_H");
	gauge->irq_no[BAT_TMP_L_IRQ] =
		platform_get_irq_byname(pdev, "BAT_TMP_L");

	gauge->chan_bat_temp = devm_iio_channel_get(
		&pdev->dev, "pmic_battery_temp");
	if (IS_ERR(gauge->chan_bat_temp)) {
		ret = PTR_ERR(gauge->chan_bat_temp);
		bm_err("pmic_battery_temp auxadc get fail, ret=%d\n", ret);
	}

	gauge->chan_bat_voltage = devm_iio_channel_get(
		&pdev->dev, "pmic_battery_voltage");
	if (IS_ERR(gauge->chan_bat_voltage)) {
		ret = PTR_ERR(gauge->chan_bat_voltage);
		bm_err("chan_bat_voltage auxadc get fail, ret=%d\n", ret);
	}

	gauge->chan_bif = devm_iio_channel_get(
		&pdev->dev, "pmic_bif_voltage");
	if (IS_ERR(gauge->chan_bif)) {
		ret = PTR_ERR(gauge->chan_bif);
		bm_err("pmic_bif_voltage auxadc get fail, ret=%d\n", ret);
	}

	gauge->chan_ptim_bat_voltage = devm_iio_channel_get(
		&pdev->dev, "pmic_ptim_voltage");
	if (IS_ERR(gauge->chan_ptim_bat_voltage)) {
		ret = PTR_ERR(gauge->chan_ptim_bat_voltage);
		bm_err("chan_ptim_bat_voltage auxadc get fail, ret=%d\n",
			ret);
	}

	gauge->chan_ptim_r = devm_iio_channel_get(
		&pdev->dev, "pmic_ptim_r");
	if (IS_ERR(gauge->chan_ptim_r)) {
		ret = PTR_ERR(gauge->chan_ptim_r);
		bm_err("chan_ptim_r auxadc get fail, ret=%d\n",
			ret);
	}

	gauge->hw_status.car_tune_value = 1000;
	gauge->hw_status.r_fg_value = 50;
	gauge->attr = mt6359_sysfs_field_tbl;

	if (battery_psy_init(pdev))
		return -ENOMEM;

	gauge->psy_desc.name = "mtk-gauge";
	gauge->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	gauge->psy_desc.properties = gauge_properties;
	gauge->psy_desc.num_properties = ARRAY_SIZE(gauge_properties);
	gauge->psy_desc.get_property = psy_gauge_get_property;
	gauge->psy_desc.set_property = psy_gauge_set_property;
	gauge->psy_cfg.drv_data = gauge;
	gauge->psy = power_supply_register(&pdev->dev, &gauge->psy_desc,
			&gauge->psy_cfg);
	mt6359_sysfs_create_group(gauge);
	initial_set(gauge, 0, 0);
	bat_create_netlink(pdev);
	battery_init(pdev);
	adc_cali_cdev_init(pdev);

	bm_err("%s: done\n", __func__);

	return 0;
}

static const struct of_device_id mt6359_gauge_of_match[] = {
	{.compatible = "mediatek,mt6359p-gauge",},
	{},
};

static int mt6359_gauge_remove(struct platform_device *pdev)
{
	struct mtk_gauge *gauge = platform_get_drvdata(pdev);

	if (gauge)
		devm_kfree(&pdev->dev, gauge);
	return 0;
}

MODULE_DEVICE_TABLE(of, mt6359_gauge_of_match);

static struct platform_driver mt6359_gauge_driver = {
	.probe = mt6359_gauge_probe,
	.remove = mt6359_gauge_remove,
	.shutdown = mt6359_gauge_shutdown,
	.suspend = mt6359_gauge_suspend,
	.resume = mt6359_gauge_resume,
	.driver = {
		.name = "mt6359p_gauge",
		.of_match_table = mt6359_gauge_of_match,
		},
};

static int __init mt6359_gauge_init(void)
{
	return platform_driver_register(&mt6359_gauge_driver);
}
module_init(mt6359_gauge_init);

static void __exit mt6359_gauge_exit(void)
{
	platform_driver_unregister(&mt6359_gauge_driver);
}
module_exit(mt6359_gauge_exit);

MODULE_AUTHOR("wy.chuang <wy.chuang@mediatek.com>");
MODULE_DESCRIPTION("MTK Gauge Device Driver");
MODULE_LICENSE("GPL");

