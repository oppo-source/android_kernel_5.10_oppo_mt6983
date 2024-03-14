// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2018-2020 Oplus. All rights reserved.
*/
/*
 * SG Micro sgm7220 Type-C Port Control Driver
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <uapi/linux/sched/types.h>
#include <linux/pm_wakeup.h>
#include <linux/sched/clock.h>
#include <linux/sched/types.h>

#include "inc/tcpci.h"

#include <linux/sched/rt.h>

#define SGM7220_DRV_VERSION "1.0"

#define sgm7220_IRQ_WAKE_TIME (500) /* ms */

#define SGM7220_WAIT_SYS_WAKEUP_BY_USBTEMP 10

#define RETRY_CNT 3

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_BASE 0x00
/* 0x00 - 0x07 reserved for ID */
#define REG_MOD 0x08
#define REG_INT 0x09
#define REG_SET 0x0A
#define REG_CTL 0X45
#define REG_REVISION 0XA0

/******************************************************************************
* Register bits
******************************************************************************/
/* REG_ID (0x00) */
#define ID_REG_LEN 0X08

/* REG_MOD (0x08) */
#define MOD_ACTIVE_CABLE_DETECTION 0X01 /*RU*/
#define MOD_ACCESSORY_CONNECTED_SHIFT 1
#define MOD_ACCESSORY_CONNECTED (0x07 << MOD_ACCESSORY_CONNECTED_SHIFT) /*RU*/
#define MOD_CURRENT_MODE_DETECT_SHIFT 4
#define MOD_CURRENT_MODE_DETECT (0x03 << MOD_CURRENT_MODE_DETECT_SHIFT) /*RU*/
#define MOD_CURRENT_MODE_ADVERTISE_SHIFT 6
#define MOD_CURRENT_MODE_ADVERTISE (0x03 << MOD_CURRENT_MODE_ADVERTISE_SHIFT) /*RW*/

/* REG_INT (0x09) */
#define SET_DISABLE_UFP_ACCESS 0x01
#define INT_DRP_DUTY_CYCLE_SHIFT 1
#define INT_DRP_DUTY_CYCLE (0x03 << INT_DRP_DUTY_CYCLE_SHIFT) /*RW*/
#define INT_INTERRUPT_STATUS_SHIFT 4
#define INT_INTERRUPT_STATUS (0x01 << INT_INTERRUPT_STATUS_SHIFT) /*RCU*/
#define INT_CABLE_DIR_SHIFT 5
#define INT_CABLE_DIR (0x01 << INT_CABLE_DIR_SHIFT) /*RU*/
#define INT_ATTACHED_STATE_SHIFT 6
#define INT_ATTACHED_STATE (0x03 << INT_ATTACHED_STATE_SHIFT) /*RU*/

/* REG_SET (0x0A) */
#define SET_I2C_ENABLE_TERM 0x00
#define SET_I2C_DISABLE_TERM 0x01 /*RW*/
#define SET_I2C_SOURCE_PREF_SHIFT 1
#define SET_I2C_SOURCE_PREF (0x03 << SET_I2C_SOURCE_PREF_SHIFT) /*RW*/
#define SET_I2C_SOFT_RESET_SHIFT 3
#define SET_I2C_SOFT_RESET (0x01 << SET_I2C_SOFT_RESET_SHIFT) /*RSU*/
#define SET_MODE_SELECT_SHIFT 4
#define SET_MODE_SELECT (0x03 << SET_MODE_SELECT_SHIFT) /*RW*/
#define SET_DEBOUNCE_SHIFT 6
#define SET_DEBOUNCE (0x03 << SET_DEBOUNCE_SHIFT) /*RW*/

/* REG_CTR (0x45) */
#define CTR_DISABLE_RD_RP_SHIFT 2
#define CTR_DISABLE_RD_RP (0x01 << CTR_DISABLE_RD_RP_SHIFT) /*RW*/

/******************************************************************************
 * Register values
 ******************************************************************************/
/* SET_MODE_SELECT */
#define SET_MODE_SELECT_DEFAULT 0x00
#define SET_MODE_SELECT_SNK 0x01
#define SET_MODE_SELECT_SRC 0x02
#define SET_MODE_SELECT_DRP 0x03
/* MOD_CURRENT_MODE_ADVERTISE */
#define MOD_CURRENT_MODE_ADVERTISE_DEFAULT 0x00
#define MOD_CURRENT_MODE_ADVERTISE_MID 0x01
#define MOD_CURRENT_MODE_ADVERTISE_HIGH 0x02
/* MOD_CURRENT_MODE_DETECT */
#define MOD_CURRENT_MODE_DETECT_DEFAULT 0x00
#define MOD_CURRENT_MODE_DETECT_MID 0x01
#define MOD_CURRENT_MODE_DETECT_ACCESSARY 0x02
#define MOD_CURRENT_MODE_DETECT_HIGH 0x03
/* CTR_DISABLE_RD_RP */
#define CTR_DISABLE_RD_RP_DEFAULT      0x00
#define CTR_DISABLE_RD_RP_DISABLE      0x01


/******************************************************************************
 * Constants
 ******************************************************************************/
enum drp_toggle_type {
	TOGGLE_DFP_DRP_30 = 0,
	TOGGLE_DFP_DRP_40,
	TOGGLE_DFP_DRP_50,
	TOGGLE_DFP_DRP_60
};

enum current_adv_type {
	HOST_CUR_USB = 0, /*default 500mA or 900mA*/
	HOST_CUR_1P5,	  /*1.5A*/
	HOST_CUR_3A		  /*3A*/
};

enum current_det_type {
	DET_CUR_USB = 0, /*default 500mA or 900mA*/
	DET_CUR_1P5,
	DET_CUR_ACCESSORY, /*charg through accessory 500mA*/
	DET_CUR_3A
};

enum accessory_attach_type {
	ACCESSORY_NOT_ATTACHED = 0,
	ACCESSORY_AUDIO = 4,
	ACCESSORY_CHG_THRU_AUDIO = 5,
	ACCESSORY_DEBUG = 6
};

enum cable_attach_type {
	CABLE_NOT_ATTACHED = 0,
	CABLE_ATTACHED
};

enum cable_state_type {
	CABLE_STATE_NOT_ATTACHED = 0,
	CABLE_STATE_AS_DFP,
	CABLE_STATE_AS_UFP,
	CABLE_STATE_TO_ACCESSORY
};

enum cable_dir_type {
	ORIENT_CC2 = 0,
	ORIENT_CC1
};

enum cc_modes_type {
	MODE_DEFAULT = 0,
	MODE_UFP,
	MODE_DFP,
	MODE_DRP
};
static int i_sgm7220 = 0;
static int is_probe_sucess = 0;

/* Type-C Attrs */
struct type_c_parameters {
	enum current_det_type current_det;			 /*charging current on UFP*/
	enum accessory_attach_type accessory_attach; /*if an accessory is attached*/
	enum cable_attach_type active_cable_attach;	 /*if an active_cable is attached*/
	enum cable_state_type attach_state;			 /*DFP->UFP or UFP->DFP*/
	enum cable_dir_type cable_dir;				 /*cc1 or cc2*/
};

struct sgm7220_chip {
	struct i2c_client *client;
	struct device *dev;
	struct semaphore io_lock;
	struct semaphore suspend_lock;
	struct tcpc_desc *tcpc_desc;
	struct tcpc_device *tcpc;
	struct kthread_worker irq_worker;
	struct kthread_work irq_work;
	struct task_struct *irq_worker_task;
	struct wakeup_source *irq_wake_lock;
	struct mutex mutex;
	struct type_c_parameters type_c_param;
	struct type_c_parameters type_c_param_old;
	int irq_gpio;
	int irq;
	int chip_id;
};

/* i2c operate interfaces */
static int sgm7220_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct sgm7220_chip *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int sgm7220_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct sgm7220_chip *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&info->mutex);
	if (ret < 0)
		pr_err("%s: (0x%x) error, ret(%d)\n", __func__, reg, ret);

	return ret;
}

static int sgm7220_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct sgm7220_chip *info = i2c_get_clientdata(i2c);
	int ret;
	u8 old_val, new_val;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);

	if (ret >= 0) {
		old_val = ret & 0xff;
		new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&info->mutex);
	return ret;
}

static int sgm7220_init_alert_mask(struct tcpc_device *tcpc)
{
	return 0;
}

static int sgm7220_init_power_status_mask(struct tcpc_device *tcpc)
{
	return 0;
}

static int sgm7220_init_fault_mask(struct tcpc_device *tcpc)
{
	return 0;
}

static int sgm7220_init_rt_mask(struct tcpc_device *tcpc)
{
	return 0;
}
/*
static inline void sgm7220_poll_ctrl(struct sgm7220_chip *chip)
{
	cancel_delayed_work_sync(&chip->poll_work);

	if (atomic_read(&chip->poll_count) == 0) {
		atomic_inc(&chip->poll_count);
		cpu_idle_poll_ctrl(true);
	}

	schedule_delayed_work(
		&chip->poll_work, msecs_to_jiffies(40));
}*/

/***********************************************************
 * read registers in irq process
 ***********************************************************/
static void process_mode_register(struct sgm7220_chip *info)
{
	u8 val, tmp, reg_val;
	int ret;

	ret = sgm7220_read_reg(info->client, REG_MOD, &reg_val);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		return;
	}
	tmp = reg_val;

	/* check current_detect */
	val = ((tmp & MOD_CURRENT_MODE_DETECT) >> MOD_CURRENT_MODE_DETECT_SHIFT);
	info->type_c_param.current_det = val;

	/* check accessory attch */
	tmp = reg_val;
	val = ((tmp & MOD_ACCESSORY_CONNECTED) >> MOD_ACCESSORY_CONNECTED_SHIFT);
	info->type_c_param.accessory_attach = val;

	/* check cable attach */
	tmp = reg_val;
	val = (tmp & MOD_ACTIVE_CABLE_DETECTION);
	info->type_c_param.active_cable_attach = val;
}

static void process_interrupt_register(struct sgm7220_chip *info)
{
	u8 val, tmp, reg_val;
	int ret;

	ret = sgm7220_read_reg(info->client, REG_INT, &reg_val);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		return;
	}
	tmp = reg_val;

	/* check attach state */
	val = ((tmp & INT_ATTACHED_STATE) >> INT_ATTACHED_STATE_SHIFT);
	info->type_c_param.attach_state = val;

	/* update current adv when act as DFP */
	if (info->type_c_param.attach_state == CABLE_STATE_AS_DFP ||
		info->type_c_param.attach_state == CABLE_STATE_TO_ACCESSORY) {
		val = (HOST_CUR_USB << MOD_CURRENT_MODE_ADVERTISE_SHIFT);
	} else {
		val = (HOST_CUR_3A << MOD_CURRENT_MODE_ADVERTISE_SHIFT);
	}
	sgm7220_update_reg(info->client, REG_MOD, val, MOD_CURRENT_MODE_ADVERTISE);

	/* check cable dir */
	tmp = reg_val;
	val = ((tmp & INT_CABLE_DIR) >> INT_CABLE_DIR_SHIFT);
	info->type_c_param.cable_dir = val;
}

static inline void sgm7220_unattached_cc_entry(struct tcpc_device *tcpc_dev)
{
	tcpc_dev->typec_role = tcpc_dev->typec_role_new;
	switch (tcpc_dev->typec_role) {
	case TYPEC_ATTACHED_NORP_SRC:
	case TYPEC_ROLE_SNK:
		pr_info("%s set_cc sink\n", __func__);
		tcpci_set_cc(tcpc_dev, TYPEC_CC_RD);
		break;
	case TYPEC_ROLE_DRP:
		pr_info("%s set_cc drp\n", __func__);
		tcpci_set_cc(tcpc_dev, TYPEC_CC_DRP);
		break;
	}
}

static void sgm7220_process_tcpci_attach_state(struct sgm7220_chip *chip)
{
	struct tcpc_device *tcpc_dev = chip->tcpc;
	int rv;
	uint16_t power_status = 0;

	rv = tcpci_get_power_status(tcpc_dev, &power_status);
	if (rv < 0)
		return;

	pr_info("%s power_status = 0x%2x, vbus_level = %d\n", __func__, power_status, tcpc_dev->vbus_level);
	pr_info("%s typec_attach_old = %d tcpc_dev->typec_attach_new = %d type_c_param.attach_stat = %d old.attach_state = %d \n", __func__
				, tcpc_dev->typec_attach_old, tcpc_dev->typec_attach_new, chip->type_c_param.attach_state, chip->type_c_param_old.attach_state);

	if ((chip->type_c_param.attach_state == CABLE_STATE_NOT_ATTACHED) &&
		(chip->type_c_param_old.attach_state != CABLE_STATE_NOT_ATTACHED)) {
		tcpc_dev->typec_attach_old = tcpc_dev->typec_attach_new;
		tcpc_dev->typec_attach_new = TYPEC_UNATTACHED;
		tcpci_notify_typec_state(tcpc_dev);
		tcpci_source_vbus(tcpc_dev,
						  TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_0V, 0);
	}

	if ((chip->type_c_param.attach_state == CABLE_STATE_AS_UFP) &&
		(chip->type_c_param_old.attach_state != CABLE_STATE_AS_UFP)) {
		tcpc_dev->typec_attach_old = tcpc_dev->typec_attach_new;
		tcpc_dev->typec_attach_new = TYPEC_ATTACHED_SNK;
		tcpci_notify_typec_state(tcpc_dev);
		tcpci_source_vbus(tcpc_dev,
						  TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_0V, 0);
	}

	if ((chip->type_c_param.attach_state == CABLE_STATE_AS_DFP) &&
		(chip->type_c_param_old.attach_state != CABLE_STATE_AS_DFP)) {
		tcpc_dev->typec_attach_old = tcpc_dev->typec_attach_new;
		tcpc_dev->typec_attach_new = TYPEC_ATTACHED_SRC;
		tcpci_notify_typec_state(tcpc_dev);
		tcpci_source_vbus(tcpc_dev,
						  TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, -1);
	}

	if ((chip->type_c_param.attach_state == CABLE_STATE_TO_ACCESSORY) &&
		(chip->type_c_param_old.attach_state != CABLE_STATE_TO_ACCESSORY)) {
		tcpc_dev->typec_attach_old = tcpc_dev->typec_attach_new;
		tcpc_dev->typec_attach_new = TYPEC_ATTACHED_AUDIO;
		tcpci_notify_typec_state(tcpc_dev);
	}

	if (tcpc_dev->typec_attach_new == TYPEC_UNATTACHED)
		sgm7220_unattached_cc_entry(tcpc_dev);

	chip->type_c_param_old.attach_state = chip->type_c_param.attach_state;
	tcpc_dev->typec_polarity = chip->type_c_param.cable_dir;

	pr_info("%s typec_attach_old = %d tcpc_dev->typec_attach_new = %d type_c_param.attach_stat = %d old.attach_state = %d \n", __func__
				, tcpc_dev->typec_attach_old, tcpc_dev->typec_attach_new, chip->type_c_param.attach_state, chip->type_c_param_old.attach_state);
}

static void sgm7220_irq_work_handler(struct kthread_work *work)
{
	struct sgm7220_chip *chip =
		container_of(work, struct sgm7220_chip, irq_work);

	int ret = 0;

	pr_info("%s enter!\n", __func__);

	down(&chip->suspend_lock);
	tcpci_lock_typec(chip->tcpc);

	process_mode_register(chip);
	process_interrupt_register(chip);

	sgm7220_process_tcpci_attach_state(chip);

	ret = sgm7220_update_reg(chip->client,
							 REG_INT, (0x1 << INT_INTERRUPT_STATUS_SHIFT), INT_INTERRUPT_STATUS);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}

	tcpci_unlock_typec(chip->tcpc);
	up(&chip->suspend_lock);
}
/*
static void sgm7220_poll_work(struct work_struct *work)
{
	struct sgm7220_chip *chip = container_of(
		work, struct sgm7220_chip, poll_work.work);

	if (atomic_dec_and_test(&chip->poll_count))
		cpu_idle_poll_ctrl(false);
}
*/
static irqreturn_t sgm7220_intr_handler(int irq, void *data)
{
	struct sgm7220_chip *chip = data;

	__pm_wakeup_event(chip->irq_wake_lock, sgm7220_IRQ_WAKE_TIME);

	kthread_queue_work(&chip->irq_worker, &chip->irq_work);
	return IRQ_HANDLED;
}

static int sgm7220_init_alert(struct tcpc_device *tcpc)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	struct sched_param param = {.sched_priority = MAX_RT_PRIO - 1};
	int ret;
	char *name;
	int len;

	len = strlen(chip->tcpc_desc->name);
	name = devm_kzalloc(chip->dev, len + 5, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	snprintf(name, PAGE_SIZE, "%s-IRQ", chip->tcpc_desc->name);

	pr_info("%s name = %s, gpio = %d\n", __func__,
			chip->tcpc_desc->name, chip->irq_gpio);

	ret = devm_gpio_request(chip->dev, chip->irq_gpio, name);

	if (ret < 0) {
		pr_err("Error: failed to request GPIO%d (ret = %d)\n",
			   chip->irq_gpio, ret);
		goto init_alert_err;
	}

	ret = gpio_direction_input(chip->irq_gpio);
	if (ret < 0) {
		pr_err("Error: failed to set GPIO%d as input pin(ret = %d)\n",
			   chip->irq_gpio, ret);
		goto init_alert_err;
	}

	chip->irq = gpio_to_irq(chip->irq_gpio);
	if (chip->irq <= 0) {
		pr_err("%s gpio to irq fail, chip->irq(%d)\n",
			   __func__, chip->irq);
		goto init_alert_err;
	}

	pr_info("%s : IRQ number = %d\n", __func__, chip->irq);

	kthread_init_worker(&chip->irq_worker);
	chip->irq_worker_task = kthread_run(kthread_worker_fn,
										&chip->irq_worker, "%s", chip->tcpc_desc->name);
	if (IS_ERR(chip->irq_worker_task)) {
		pr_err("Error: Could not create tcpc task\n");
		goto init_alert_err;
	}

	sched_setscheduler(chip->irq_worker_task, SCHED_FIFO, &param);
	kthread_init_work(&chip->irq_work, sgm7220_irq_work_handler);

	pr_info("IRQF_NO_THREAD Test\r\n");
	ret = request_irq(chip->irq, sgm7220_intr_handler,
					  IRQF_TRIGGER_FALLING | IRQF_NO_THREAD |
						  IRQF_NO_SUSPEND,
					  name, chip);
	if (ret < 0) {
		pr_err("Error: failed to request irq%d (gpio = %d, ret = %d)\n",
			   chip->irq, chip->irq_gpio, ret);
		goto init_alert_err;
	}

	enable_irq_wake(chip->irq);
	return 0;
init_alert_err:
	return -EINVAL;
}

int sgm7220_alert_status_clear(struct tcpc_device *tcpc, uint32_t mask)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	int ret = 0;

	ret = sgm7220_update_reg(chip->client,
							 REG_INT, (0x1 << INT_INTERRUPT_STATUS_SHIFT), INT_INTERRUPT_STATUS);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}

	return 0;
}

static int sgm7220_initialization(struct tcpc_device *tcpc)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	int ret = 0;
	u8 reg_val;

	/* do initialization here, before enable irq,
	 * clear irq,
	 * config DRP/UFP/DFP mode,
	 * and etc..
	 */
	pr_debug("%s enter\n", __func__);

	/* 1 Disable term(default 0) [bit 0]
	 * 2 Source pref(DRP performs try.SNK 01) [bit 2:1]
	 * 3 Soft reset(default 0) [bit 3]
	 * 4 Mode slect(DRP start from Try.SNK 00) [bit 5:4]
	 * 5 Debounce time of voltage change on CC pin(default 0) [bit 7:6]
	 */
	reg_val = 0x02;
	ret = sgm7220_write_reg(chip->client, REG_SET, reg_val);
	if (ret < 0) {
		pr_err("%s: init REG_SET fail!\n", __func__);
		return ret;
	}
	ret = sgm7220_update_reg(chip->client, REG_INT,
							 0x01, SET_DISABLE_UFP_ACCESS);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}
	/* CURRENT MODE ADVERTISE 3A [bit 7:6] */
	reg_val = (HOST_CUR_3A << MOD_CURRENT_MODE_ADVERTISE_SHIFT);
	ret = sgm7220_update_reg(chip->client, REG_MOD, reg_val, MOD_CURRENT_MODE_ADVERTISE);
	if (ret < 0) {
		pr_err("%s: init REG_MOD fail!\n", __func__);
		return ret;
	}

	return ret;
}

static inline int sgm7220_software_reset(struct tcpc_device *tcpc)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	u8 reg_val;
	int ret;

	reg_val = (0x01 << SET_I2C_SOFT_RESET_SHIFT);
	ret = sgm7220_update_reg(chip->client, REG_SET, reg_val, SET_I2C_SOFT_RESET);
	if (ret < 0) {
		pr_err("%s: init REG_MOD fail!\n", __func__);
		return ret;
	}
	return 0;
}

static inline int sgm7220_i2c_reset(struct tcpc_device *tcpc)
{
	return 0;
}

static int sgm7220_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
	int ret;

	pr_info("%s\n", __func__);

	if (sw_reset) {
		ret = sgm7220_software_reset(tcpc);
		if (ret < 0)
			return ret;
	}

#if IS_ENABLED(CONFIG_TCPC_I2CRST_EN)
	sgm7220_i2c_reset(tcpc);
#endif

	ret = sgm7220_initialization(tcpc);
	if (ret < 0) {
		pr_err("%s: fails to initialize %d\n", __func__, ret);
		return ret;
	}

	tcpci_alert_status_clear(tcpc, 0xffffffff);

	sgm7220_init_power_status_mask(tcpc);
	sgm7220_init_alert_mask(tcpc);
	sgm7220_init_fault_mask(tcpc);
	sgm7220_init_rt_mask(tcpc);

	return 0;
}

int sgm7220_fault_status_clear(struct tcpc_device *tcpc, uint8_t status)
{
	return 0;
}

int sgm7220_get_alert_mask(struct tcpc_device *tcpc, uint32_t *mask)
{
	*mask = 0;
	*mask |= (TCPC_REG_ALERT_CC_STATUS |
			  TCPC_REG_ALERT_POWER_STATUS |
			  TCPC_REG_ALERT_EXT_RA_DETACH);
	return 0;
}

/*****************************************************************
 * sgm7220 does not have alert register, compare type_c_param_old
 * with type_c_param for figuring out what has been changed.
 ****************************************************************/
int sgm7220_get_alert_status(struct tcpc_device *tcpc, uint32_t *alert)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	*alert = 0;

	/* cc status change */
	if ((chip->type_c_param.current_det != chip->type_c_param_old.current_det) ||
		(chip->type_c_param.active_cable_attach != chip->type_c_param_old.active_cable_attach) ||
		(chip->type_c_param.attach_state != chip->type_c_param_old.attach_state) ||
		(chip->type_c_param.cable_dir != chip->type_c_param_old.cable_dir)) {
		*alert |= TCPC_REG_ALERT_CC_STATUS;
	}

	/* UFP <-> Non-UFP = Power present change */
	if (((chip->type_c_param.attach_state == CABLE_STATE_AS_UFP) &&
		 (chip->type_c_param_old.attach_state != CABLE_STATE_AS_UFP)) ||
		((chip->type_c_param.attach_state == CABLE_STATE_AS_DFP) &&
		 (chip->type_c_param_old.attach_state != CABLE_STATE_AS_DFP))) {
		*alert |= TCPC_REG_ALERT_POWER_STATUS;
	}

	/* accessory attach change */
	if (chip->type_c_param.accessory_attach != chip->type_c_param_old.accessory_attach) {
		*alert |= TCPC_REG_ALERT_EXT_RA_DETACH;
	}
	/* Add more alert bits here if need */

	pr_info("%s, %d->%d, %d->%d, %d->%d, %d->%d, %d->%d\n", __func__,
			chip->type_c_param_old.current_det, chip->type_c_param.current_det,
			chip->type_c_param_old.active_cable_attach, chip->type_c_param.active_cable_attach,
			chip->type_c_param_old.attach_state, chip->type_c_param.attach_state,
			chip->type_c_param_old.cable_dir, chip->type_c_param.cable_dir,
			chip->type_c_param_old.accessory_attach, chip->type_c_param.accessory_attach);

	/* sync status to type_c_param_old */
	chip->type_c_param_old.current_det = chip->type_c_param.current_det;
	chip->type_c_param_old.active_cable_attach = chip->type_c_param.active_cable_attach;
	chip->type_c_param_old.attach_state = chip->type_c_param.attach_state;
	chip->type_c_param_old.cable_dir = chip->type_c_param.cable_dir;
	chip->type_c_param_old.accessory_attach = chip->type_c_param.accessory_attach;

	return 0;
}

static int sgm7220_get_power_status(
	struct tcpc_device *tcpc, uint16_t *pwr_status)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	u8 reg_val;
	u8 val;
	int ret;

	pr_info("%s enter\n", __func__);

	ret = sgm7220_read_reg(chip->client, REG_INT, &reg_val);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		return ret;
	}

	val = ((reg_val & INT_ATTACHED_STATE) >> INT_ATTACHED_STATE_SHIFT);
	*pwr_status = 0;

	if (val & CABLE_STATE_AS_UFP) {
		/* sgm7220 does not supprt vbus detection,
		assume vbus is detected if act as UFP */
		*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES;
	}
	/*
	if (val & CABLE_STATE_AS_DFP) {
		if (*pwr_status & TCPC_REG_POWER_STATUS_EXT_VSAFE0V) {
			*pwr_status |= TCPC_REG_ALERT_POWER_STATUS;
		} else {
			*pwr_status |= TCPC_REG_POWER_STATUS_EXT_VSAFE0V;
		}
	}
	*/
	return 0;
}

int sgm7220_get_fault_status(struct tcpc_device *tcpc, uint8_t *status)
{
	return 0;
}

/*******************************************************************
 * Translate sgm7220 cc register's value to tcpc_cc_voltage_status.
 *******************************************************************/
static int sgm7220_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	int *active_cc, *secondary_cc;
	bool act_as_sink;

	pr_info("%s enter\n", __func__);

	*cc1 = TYPEC_CC_VOLT_OPEN;
	*cc2 = TYPEC_CC_VOLT_OPEN;

	if (chip->type_c_param.attach_state == CABLE_STATE_NOT_ATTACHED) {
		pr_debug("cc not attached\n");
		return 0;
	}

	active_cc = (chip->type_c_param.cable_dir) ? cc1 : cc2;
	secondary_cc = (chip->type_c_param.cable_dir) ? cc2 : cc1;

	if (chip->type_c_param.attach_state == CABLE_STATE_AS_UFP) {
		switch (chip->type_c_param.current_det) {
		case 0: /* RP Default */
			*active_cc |= TYPEC_CC_VOLT_SNK_DFT;
			break;
		case 1: /* RP 1.5V */
			*active_cc |= TYPEC_CC_VOLT_SNK_1_5;
			break;
		case 3: /* RP 3.0V */
			*active_cc |= TYPEC_CC_VOLT_SNK_3_0;
			break;
		default:
			*active_cc |= TYPEC_CC_VOLT_SNK_DFT;
			break;
		}
	} else if (chip->type_c_param.attach_state == CABLE_STATE_TO_ACCESSORY) {
		switch (chip->type_c_param.accessory_attach) {
		case 4: /* audio accessory */
			*active_cc |= TYPEC_CC_VOLT_RA;
			*secondary_cc |= TYPEC_CC_VOLT_RA;
			break;
		case 5: /* charged thru audio accessory */
			*active_cc |= TYPEC_CC_VOLT_RA;
			*secondary_cc |= TYPEC_CC_VOLT_RA;
			break;
		case 6: /* DFP, debug accessory */
			*active_cc |= TYPEC_CC_VOLT_RD;
			*secondary_cc |= TYPEC_CC_VOLT_RD;
			break;
		default:
			break;
		}
	} else if (chip->type_c_param.attach_state == CABLE_STATE_AS_DFP) {
		if (chip->type_c_param.active_cable_attach) {
			*active_cc |= TYPEC_CC_VOLT_RA;
		} else {
			*active_cc |= TYPEC_CC_VOLT_RD;
		}
	}

	act_as_sink = (chip->type_c_param.attach_state == CABLE_STATE_AS_UFP) ? true : false;

	if (*cc1 != TYPEC_CC_VOLT_OPEN)
		*cc1 |= (act_as_sink << 2);

	if (*cc2 != TYPEC_CC_VOLT_OPEN)
		*cc2 |= (act_as_sink << 2);

	return 0;
}

static int sgm7220_set_cc(struct tcpc_device *tcpc, int pull)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	int ret;
	uint8_t value = 0;

	pr_info("%s enter pull = %d\n", __func__, pull);

	value = 0x01;
	ret = sgm7220_update_reg(chip->client, REG_SET, value, SET_I2C_DISABLE_TERM);
	if (ret < 0) {
		pr_err("%s: init REG_SET fail!\n", __func__);
		return ret;
	}

	pull = TYPEC_CC_PULL_GET_RES(pull);
	if (pull == TYPEC_CC_RP)
		value = SET_MODE_SELECT_SRC;
	else if (pull == TYPEC_CC_RD)
		value = SET_MODE_SELECT_SNK;
	else if (pull == TYPEC_CC_DRP)
		value = SET_MODE_SELECT_DRP;
	else
		value = SET_MODE_SELECT_DRP;

	value = value << SET_MODE_SELECT_SHIFT;
	ret = sgm7220_update_reg(chip->client, REG_SET, value, SET_MODE_SELECT);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}

	ret = sgm7220_read_reg(chip->client, REG_SET, &value);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		return ret;
	}

	value = 0x00;
	ret = sgm7220_update_reg(chip->client, REG_SET, value, SET_I2C_DISABLE_TERM);
	if (ret < 0) {
		pr_err("%s: init REG_SET fail!\n", __func__);
		return ret;
	}

	pr_info("%s end reg_val = %x\n", __func__, value);

	return 0;
}

void sgm7220_set_sinkonly(struct tcpc_device *tcpc)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	int ret;
	int retry = 0;
	uint8_t value = 0;

	if (is_probe_sucess == 0) {
		pr_err("%s: not for this chip!\n", __func__);
		return;
	}
	pr_info("%s: enter!\n", __func__);

	value = 0x01;
	while (retry < RETRY_CNT) {
		ret = sgm7220_update_reg(chip->client, REG_SET, value, SET_I2C_DISABLE_TERM);
		if (ret < 0) {
			retry++;
			msleep(SGM7220_WAIT_SYS_WAKEUP_BY_USBTEMP);
			pr_err("%s: init REG_SET fail, retry = %d!\n", __func__, retry);
		}
		else {
			break;
		}
	}
	retry = 0;

	value = CTR_DISABLE_RD_RP_DEFAULT << CTR_DISABLE_RD_RP_SHIFT;
	ret = sgm7220_update_reg(chip->client, REG_CTL, value, CTR_DISABLE_RD_RP);
	if (ret < 0) {
		pr_err("%s: CTR_ENABLE_RD_RP update reg fail!\n", __func__);
		return;
	}
	pr_info("%s: CTR_ENABLE_RD_RP update reg success!\n", __func__);

	value = SET_MODE_SELECT_SNK;
	value = value << SET_MODE_SELECT_SHIFT;
	ret = sgm7220_update_reg(chip->client, REG_SET, value, SET_MODE_SELECT);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
		return;
	}
	pr_info("%s: set sink success, value = %x!\n", __func__, value);

	value = 0x00;
	ret = sgm7220_update_reg(chip->client, REG_SET, value, SET_I2C_DISABLE_TERM);
	if (ret < 0) {
		pr_err("%s: init REG_SET fail!\n", __func__);
		return;
	}
}
EXPORT_SYMBOL(sgm7220_set_sinkonly);

void sgm7220_set_cc_open(struct tcpc_device *tcpc)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	int ret;
	uint8_t value = 0;

	if (is_probe_sucess == 0) {
		pr_err("%s: not for this chip!\n", __func__);
		return;
	}
	pr_info("%s: enter!\n", __func__);

	value = 0x01;
	ret = sgm7220_update_reg(chip->client, REG_SET, value, SET_I2C_DISABLE_TERM);
	if (ret < 0) {
		pr_err("%s: init REG_SET fail!\n", __func__);
		return;
	}

	value = CTR_DISABLE_RD_RP_DISABLE << CTR_DISABLE_RD_RP_SHIFT;
	ret = sgm7220_update_reg(chip->client, REG_CTL, value, CTR_DISABLE_RD_RP);
	if (ret < 0) {
		pr_err("%s: CTR_DISABLE_RD_RP update reg fail!\n", __func__);
		return;
	}
	pr_info("%s: CTR_DISABLE_RD_RP update reg success!\n", __func__);

	value = 0x00;
	ret = sgm7220_update_reg(chip->client, REG_SET, value, SET_I2C_DISABLE_TERM);
	if (ret < 0) {
		pr_err("%s: init REG_SET fail!\n", __func__);
		return;
	}
}
EXPORT_SYMBOL(sgm7220_set_cc_open);


static int sgm7220_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	/* sgm7220 handles polarity automatcally */
	return 0;
}

static int sgm7220_set_low_rp_duty(struct tcpc_device *tcpc, bool low_rp)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	int ret;
	u8 val = low_rp ? 0x00 : 0x10;

	ret = sgm7220_update_reg(chip->client, REG_INT,
							 val << INT_DRP_DUTY_CYCLE_SHIFT, INT_DRP_DUTY_CYCLE);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}

	return 0;
}

static int sgm7220_set_vconn(struct tcpc_device *tcpc, int enable)
{
	/* sgm7220 handles vconn automatically */
	return 0;
}

static int sgm7220_tcpc_deinit(struct tcpc_device *tcpc)
{
	struct sgm7220_chip *chip = tcpc_get_dev_data(tcpc);
	int ret;

#if IS_ENABLED(CONFIG_TCPC_SHUTDOWN_CC_DETACH)
	sgm7220_set_cc(tcpc, TYPEC_CC_RD);

	ret = sgm7220_update_reg(chip->client, REG_CTL, 1 << CTR_DISABLE_RD_RP, CTR_DISABLE_RD_RP);
	if (ret < 0) {
		pr_err("%s: update reg fail!\n", __func__);
	}
#endif

	return 0;
}

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
static int sgm7220_set_msg_header(struct tcpc_device *tcpc,
								  uint8_t power_role, uint8_t data_role)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int sgm7220_set_rx_enable(struct tcpc_device *tcpc, uint8_t enable)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int sgm7220_protocol_reset(struct tcpc_device *tcpc_dev)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int sgm7220_get_message(struct tcpc_device *tcpc, uint32_t *payload,
							   uint16_t *msg_head, enum tcpm_transmit_type *frame_type)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int sgm7220_transmit(struct tcpc_device *tcpc,
							enum tcpm_transmit_type type, uint16_t header, const uint32_t *data)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int sgm7220_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int sgm7220_set_bist_carrier_mode(
	struct tcpc_device *tcpc, uint8_t pattern)
{
	pr_err("%s\n", __func__);
	return 0;
}
#endif

/************************************************************************
 *
 *       fregdump_show
 *
 *  Description :
 *  -------------
 *  Dump registers to user space. there is side-effects for Read/Clear
 *  registers. For example interrupt status.
 *
 ************************************************************************/
static ssize_t sgm7220_regdump_show(struct device *dev,
									struct device_attribute *attr,
									char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sgm7220_chip *chip = i2c_get_clientdata(client);
	int ret = 0;
	int i, rc;

	pr_err("sgm7220_regdump_show\n");

	tcpci_lock_typec(chip->tcpc);
	for (i = REG_BASE; i <= REG_SET; i++) {
		rc = i2c_smbus_read_byte_data(chip->client, (uint8_t)i);
		if (rc < 0) {
			pr_err("cannot read 0x%02x\n", i);
			rc = 0;
		}
		ret += snprintf(buf + ret, 1024 - ret, "from 0x%02x read 0x%02x\n", (uint8_t)i, rc);
	}
	tcpci_unlock_typec(chip->tcpc);

	return ret;
}

static ssize_t sgm7220_regdump_store(struct device *dev,
									 struct device_attribute *attr,
									 const char *buf, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sgm7220_chip *chip = i2c_get_clientdata(client);
	int ret = 0;
	u8 value;
	unsigned int reg = 0;

	if (buf != NULL && size != 0) {
		ret = kstrtouint(buf, 10, &reg);
		if (ret < 0) {
			pr_err("%s: failed, ret = %d\n", __func__, ret);
			return ret;
		}
	}

	value = (u8)reg;
	pr_err("%s reg = 0x%x\n", __func__, value);
	tcpci_lock_typec(chip->tcpc);
	ret = sgm7220_write_reg(chip->client, REG_SET, value);
	if (ret < 0) {
		pr_err("%s err\n", __func__);
		return ret;
	}
	tcpci_unlock_typec(chip->tcpc);

	return size;
}
DEVICE_ATTR(sgm7220_regdump, 0664, sgm7220_regdump_show, sgm7220_regdump_store);

static struct tcpc_ops sgm7220_tcpc_ops = {
	.init = sgm7220_tcpc_init,
	.alert_status_clear = sgm7220_alert_status_clear,
	.fault_status_clear = sgm7220_fault_status_clear,
	.get_alert_mask = sgm7220_get_alert_mask,
	.get_alert_status = sgm7220_get_alert_status,
	.get_power_status = sgm7220_get_power_status,
	.get_fault_status = sgm7220_get_fault_status,
	.get_cc = sgm7220_get_cc,
	.set_cc = sgm7220_set_cc,
	.set_polarity = sgm7220_set_polarity,
	.set_low_rp_duty = sgm7220_set_low_rp_duty,
	.set_vconn = sgm7220_set_vconn,
	.deinit = sgm7220_tcpc_deinit,

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
	.set_msg_header = sgm7220_set_msg_header,
	.set_rx_enable = sgm7220_set_rx_enable,
	.protocol_reset = sgm7220_protocol_reset,
	.get_message = sgm7220_get_message,
	.transmit = sgm7220_transmit,
	.set_bist_test_mode = sgm7220_set_bist_test_mode,
	.set_bist_carrier_mode = sgm7220_set_bist_carrier_mode,
#endif /* CONFIG_USB_POWER_DELIVERY */
};

static int sgm7220_parse_dt(struct sgm7220_chip *chip, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	if (!np)
		return -EINVAL;

	pr_info("%s\n", __func__);

	np = of_find_node_by_name(NULL, "usb_type_c_sgm7220");
	if (!np) {
		pr_err("%s find node usb_type_c_sgm7220 fail\n", __func__);
		return -ENODEV;
	}

#if (!IS_ENABLED(CONFIG_MTK_GPIO) || IS_ENABLED(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "sgm7220,intr_gpio", 0);
	if (ret < 0) {
		pr_err("%s no sgm7220,intr_gpio info\n", __func__);
		return ret;
	}
	chip->irq_gpio = ret;
#else
	ret = of_property_read_u32(np, "sgm7220,intr_gpio_num", &chip->irq_gpio);
	if (ret < 0)
		pr_err("%s no sgm7220,intr_gpio_num info\n", __func__);
#endif
	return ret;
}

static int sgm7220_tcpcdev_init(struct sgm7220_chip *chip, struct device *dev)
{
	struct tcpc_desc *desc;
	struct device_node *np = dev->of_node;
	u32 val, len;
	const char *name = "default";

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	if (of_property_read_u32(np, "sgm7220-tcpc,role_def", &val) >= 0) {
		if (val >= TYPEC_ROLE_NR)
			desc->role_def = TYPEC_ROLE_DRP;
		else
			desc->role_def = val;
	} else {
		dev_info(dev, "use default Role DRP\n");
		desc->role_def = TYPEC_ROLE_DRP;
	}

	if (of_property_read_u32(
			np, "sgm7220-tcpc,notifier_supply_num", &val) >= 0) {
		if (val < 0)
			desc->notifier_supply_num = 0;
		else
			desc->notifier_supply_num = val;
	} else
		desc->notifier_supply_num = 0;

	if (of_property_read_u32(np, "sgm7220-tcpc,rp_level", &val) >= 0) {
		switch (val) {
		case 0: /* RP Default */
			desc->rp_lvl = TYPEC_CC_RP_DFT;
			break;
		case 1: /* RP 1.5V */
			desc->rp_lvl = TYPEC_CC_RP_1_5;
			break;
		case 2: /* RP 3.0V */
			desc->rp_lvl = TYPEC_CC_RP_3_0;
			break;
		default:
			break;
		}
	}

#if IS_ENABLED(CONFIG_TCPC_VCONN_SUPPLY_MODE)
	if (of_property_read_u32(np, "sgm7220-tcpc,vconn_supply", &val) >= 0) {
		if (val >= TCPC_VCONN_SUPPLY_NR)
			desc->vconn_supply = TCPC_VCONN_SUPPLY_ALWAYS;
		else
			desc->vconn_supply = val;
	} else {
		dev_info(dev, "use default Vconn Supply\n");
		desc->vconn_supply = TCPC_VCONN_SUPPLY_ALWAYS;
	}
#endif /* CONFIG_TCPC_VCONN_SUPPLY_MODE */

	of_property_read_string(np, "sgm7220-tcpc,name", (char const **)&name);
	len = strlen(name);
	desc->name = kzalloc(len + 1, GFP_KERNEL);
	if (!desc->name)
		return -ENOMEM;
	strncpy((char *)desc->name, name, strlen(name) + 1);

	chip->tcpc_desc = desc;
	chip->tcpc = tcpc_device_register(dev, desc, &sgm7220_tcpc_ops, chip);
	if (IS_ERR(chip->tcpc))
		return -EINVAL;

	chip->tcpc->tcpc_flags = TCPC_FLAGS_LPM_WAKEUP_WATCHDOG;
	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
	return 0;
}

static int sgm7220_check_revision(struct i2c_client *client)
{
	int ret;
	u8 data;

	ret = sgm7220_read_reg(client, REG_REVISION, &data);
	if (ret < 0) {
		dev_err(&client->dev, "read chip ID fail\n");
		return -EIO;
	}

	return ret;
}

static int sgm7220_i2c_probe(struct i2c_client *client,
							 const struct i2c_device_id *id)
{
	struct sgm7220_chip *chip;
	int ret = 0, chip_id;
	bool use_dt = client->dev.of_node;

	if (i_sgm7220 > 0) {
		pr_info("second probe_sgm7220 run...\n");
		return 0;
	}
	i_sgm7220++;
	pr_info("%s\n", __func__);

	if (i2c_check_functionality(client->adapter,
								I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_info("I2C functionality : OK...\n");
	} else {
		pr_info("I2C functionality check : failuare...\n");
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (use_dt) {
		sgm7220_parse_dt(chip, &client->dev);
	} else {
		dev_err(&client->dev, "no dts node\n");
		return -ENODEV;
	}
	chip->dev = &client->dev;
	chip->client = client;
	sema_init(&chip->io_lock, 1);
	sema_init(&chip->suspend_lock, 1);
	mutex_init(&chip->mutex);
	i2c_set_clientdata(client, chip);

	chip_id = sgm7220_check_revision(client);
	if (chip_id < 0)
		return chip_id;

	ret = device_create_file(&client->dev, &dev_attr_sgm7220_regdump);
	if (ret < 0) {
		dev_err(&client->dev, "failed to create dev_attr_fregdump\n");
		ret = -ENODEV;
		goto err_create_fregdump_file;
	}

	chip->irq_wake_lock =
		wakeup_source_register(chip->dev, "sgm7220_irq_wakelock");

	chip->chip_id = chip_id;
	pr_info("sgm7220_chipID = 0x%0x\n", chip_id);

	ret = sgm7220_tcpcdev_init(chip, &client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "sgm7110 tcpc dev init fail\n");
		return -EINVAL;
	}

	ret = sgm7220_init_alert(chip->tcpc);
	if (ret < 0) {
		pr_err("sgm7110 init alert fail\n");
		goto err_irq_init;
	}

	tcpc_schedule_init_work(chip->tcpc);

	is_probe_sucess = 1;
	sgm7220_set_sinkonly(chip->tcpc);

	pr_info("%s probe OK!\n", __func__);
	return 0;

err_irq_init:
	tcpc_device_unregister(chip->dev, chip->tcpc);
	mutex_destroy(&chip->mutex);
err_create_fregdump_file:
	device_remove_file(&client->dev, &dev_attr_sgm7220_regdump);

	return ret;
}

static int sgm7220_i2c_remove(struct i2c_client *client)
{
	struct sgm7220_chip *chip = i2c_get_clientdata(client);

	if (chip) {
		tcpc_device_unregister(chip->dev, chip->tcpc);
	}
	mutex_destroy(&chip->mutex);
	return 0;
}

#if CONFIG_PM
static int sgm7220_i2c_suspend(struct device *dev)
{
	struct sgm7220_chip *chip;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip)
			down(&chip->suspend_lock);
	}

	return 0;
}

static int sgm7220_i2c_resume(struct device *dev)
{
	struct sgm7220_chip *chip;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip)
			up(&chip->suspend_lock);
	}

	return 0;
}

static void sgm7220_shutdown(struct i2c_client *client)
{
	struct sgm7220_chip *chip = i2c_get_clientdata(client);

	/* Please reset IC here */
	if (chip != NULL) {
		if (chip->irq)
			disable_irq(chip->irq);
		tcpm_shutdown(chip->tcpc);
	}
}

#if IS_ENABLED(CONFIG_PM_RUNTIME)
static int sgm7220_pm_suspend_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: suspending...\n");
	return 0;
}

static int sgm7220_pm_resume_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: resuming...\n");
	return 0;
}
#endif /* #ifdef CONFIG_PM_RUNTIME */

static const struct dev_pm_ops sgm7220_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
		sgm7220_i2c_suspend,
		sgm7220_i2c_resume)
#if IS_ENABLED(CONFIG_PM_RUNTIME)
		SET_RUNTIME_PM_OPS(
			sgm7220_pm_suspend_runtime,
			sgm7220_pm_resume_runtime,
			NULL)
#endif
};
#define SGM7220_PM_OPS (&sgm7220_pm_ops)
#else
#define SGM7220_PM_OPS (NULL)
#endif /* CONFIG_PM */

static const struct i2c_device_id sgm7220_id_table[] = {
	{"sgm7220", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm7220_id_table);

static const struct of_device_id sgm_match_table[] = {
	{ .compatible = "sgm,usb_type_c", },
	{},
};

static struct i2c_driver sgm7220_driver = {
	.driver = {
		.name = "usb_type_c_sgm7220",
		.owner = THIS_MODULE,
		.of_match_table = sgm_match_table,
		.pm = SGM7220_PM_OPS,
	},
	.probe = sgm7220_i2c_probe,
	.remove = sgm7220_i2c_remove,
	.shutdown = sgm7220_shutdown,
	.id_table = sgm7220_id_table,
};

static int __init sgm7220_init(void)
{
	struct device_node *np;

	pr_info("sgm7220_init (%s): initializing...\n", SGM7220_DRV_VERSION);
	np = of_find_node_by_name(NULL, "usb_type_c_sgm7220");
	if (np != NULL)
		pr_info("usb_type_c node found...\n");
	else
		pr_info("usb_type_c node not found...\n");

	return i2c_add_driver(&sgm7220_driver);
}
subsys_initcall(sgm7220_init);

static void __exit sgm7220_exit(void)
{
	i2c_del_driver(&sgm7220_driver);
}
module_exit(sgm7220_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NULL");
MODULE_DESCRIPTION("SGM7220 TCPC Driver");
MODULE_VERSION(SGM7220_DRV_VERSION);
