// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2018-2020 Oplus. All rights reserved.
*/
/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * Mediatek wusb3801 Type-C Port Control Driver
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
#include "inc/pd_dbg_info.h"
#include "inc/tcpci.h"
#include "inc/wusb3801.h"
#include <linux/compiler.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/sched/clock.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <uapi/linux/sched/types.h>
#include <linux/atomic.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/sched/rt.h>
#include "inc/tcpci_timer.h"
#include "inc/tcpci_typec.h"
#include <linux/sched/types.h>

#define __TEST_CC_PATCH__

/* #define DEBUG_GPIO	66 */

#define WUSB3801_DRV_VERSION "2.0.1_MTK"

#define WUSB3801_IRQ_WAKE_TIME (1000) /* ms */

#define WUSB3801_WAIT_SYS_WAKEUP_BY_USBTEMP 10

#define RETRY_CNT 3

static int is_probe_sucess = 0;
static u32 is_audio_support = 0;

struct wusb3801_chip {
	struct i2c_client *client;
	struct device *dev;
	struct semaphore io_lock;
	struct semaphore suspend_lock;
	struct tcpc_desc *tcpc_desc;
	struct tcpc_device *tcpc;
	struct kthread_worker irq_worker;
	struct kthread_work irq_work;
	struct task_struct *irq_worker_task;
	struct wakeup_source *for_irq_wake_lock;
	struct wakeup_source *for_i2c_wake_lock;

	atomic_t poll_count;
	struct delayed_work poll_work;
	struct delayed_work first_check_typec_work;

	int irq_gpio;
#ifdef __TEST_CC_PATCH__
	uint8_t cc_test_flag;
	uint8_t cc_sts;
#endif /* __TEST_CC_PATCH__ */
	uint8_t dev_id;
	uint8_t dev_sub_id;
	int irq;
	int chip_id;
};

#ifdef __TEST_CC_PATCH__
uint8_t typec_cc_orientation;
#endif /* __TEST_CC_PATCH__ */
static struct i2c_client *w_client;

enum wusb3801_mode {
	REVERSE_CHG_DRP,
	REVERSE_CHG_SINK,
	REVERSE_CHG_SOURCE,
};

static int wusb3801_read_device(void *client, u32 reg, int len, void *dst)
{
	struct i2c_client *i2c = (struct i2c_client *)client;
	struct wusb3801_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0, count = 5;

	__pm_stay_awake(chip->for_i2c_wake_lock);
	down(&chip->suspend_lock);
	while (count) {
		if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(i2c, reg, len, dst);
			if (ret < 0)
				count--;
			else
				goto out;
		} else {
			ret = i2c_smbus_read_byte_data(i2c, reg);
			if (ret < 0)
				count--;
			else {
				*(u8 *)dst = (u8)ret;
				goto out;
			}
		}
		udelay(100);
	}
out:
	up(&chip->suspend_lock);
	__pm_relax(chip->for_i2c_wake_lock);
	return ret;
}

static int wusb3801_write_device(void *client, u32 reg, int len, const void *src)
{
	const u8 *data;
	struct i2c_client *i2c = (struct i2c_client *)client;
	struct wusb3801_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0, count = 5;

	__pm_stay_awake(chip->for_i2c_wake_lock);
	down(&chip->suspend_lock);
	while (count) {
		if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(i2c,
												 reg, len, src);
			if (ret < 0)
				count--;
			else
				goto out;
		} else {
			data = src;
			ret = i2c_smbus_write_byte_data(i2c, reg, *data);
			if (ret < 0)
				count--;
			else
				goto out;
		}
		udelay(100);
	}
out:
	up(&chip->suspend_lock);
	__pm_relax(chip->for_i2c_wake_lock);
	return ret;
}

static int wusb3801_reg_read(struct i2c_client *i2c, u8 reg)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(i2c);
	u8 val = 0;
	int ret = 0;

	ret = wusb3801_read_device(chip->client, reg, 1, &val);
	if (ret < 0) {
		dev_err(chip->dev, "wusb3801 reg read fail\n");
		return ret;
	}
	return val;
}

static int wusb3801_reg_write(struct i2c_client *i2c, u8 reg, const u8 data)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;

	ret = wusb3801_write_device(chip->client, reg, 1, &data);
	if (ret < 0)
		dev_err(chip->dev, "wusb3801 reg write fail\n");
	return ret;
}

static inline int wusb3801_i2c_write8(
	struct tcpc_device *tcpc, u8 reg, const u8 data)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);

	return wusb3801_reg_write(chip->client, reg, data);
}

static inline int wusb3801_i2c_read8(struct tcpc_device *tcpc, u8 reg)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);

	return wusb3801_reg_read(chip->client, reg);
}

#ifdef __TEST_CC_PATCH__
static int test_cc_patch(struct wusb3801_chip *chip)
{
	int rc;
	int ret;
	struct device *cdev = &chip->client->dev;
	dev_err(cdev, "%s \n", __func__);

	wusb3801_i2c_write8(chip->tcpc,
						WUSB3801_REG_TEST_02, 0x82);
	msleep(100);
	wusb3801_i2c_write8(chip->tcpc,
						WUSB3801_REG_TEST_09, 0xC0);
	msleep(100);
	rc = wusb3801_i2c_read8(chip->tcpc, WUSB3801_REG_TEST0);
	msleep(10);
	wusb3801_i2c_write8(chip->tcpc,
						WUSB3801_REG_TEST_09, 0x00);
	msleep(10);
	wusb3801_i2c_write8(chip->tcpc,
						WUSB3801_REG_TEST_02, 0x80);
	msleep(10);
	wusb3801_i2c_write8(chip->tcpc,
						WUSB3801_REG_TEST_02, 0x00);
	pr_err("dhx---add msleep 200\n");
	dev_err(cdev, "%s rc = [0x%02x] \n", __func__, rc);
	ret = wusb3801_i2c_read8(chip->tcpc, WUSB3801_REG_TEST_02);
	if (ret & WUSB3801_FORCE_ERR_RCY_MASK) {
		pr_err("wusb3801 [%s]enter error recovery :0x%x\n", __func__, ret);
		wusb3801_i2c_write8(chip->tcpc, WUSB3801_REG_TEST_02, 0x00);
	}
	return BITS_GET(rc, 0x40);
}
#endif /* __TEST_CC_PATCH__ */

static inline void wusb3801_unattached_cc_entry(struct tcpc_device *tcpc_dev)
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

static int first_check_flag;
static void wusb3801_irq_work_handler(struct kthread_work *work)
{
	struct wusb3801_chip *chip =
		container_of(work, struct wusb3801_chip, irq_work);
	int rc;
	int sts;
	uint8_t status, type;
	struct tcpc_device *tcpc;
	int i = 0;

	if (0 == first_check_flag)
		return;

	tcpc = chip->tcpc;

	tcpci_lock_typec(tcpc);
	/* get interrupt */
	rc = wusb3801_i2c_read8(chip->tcpc, WUSB3801_REG_INTERRUPT);
	if (rc < 0) {
		pr_err("%s: failed to read interrupt\n", __func__);
		return;
	}
	if (rc == 0) {
		for (i = WUSB3801_REG_VERSION_ID; i <= WUSB3801_REG_TEST_12; i++) {
			rc = i2c_smbus_read_byte_data(chip->client, (uint8_t)i);
			if (rc < 0) {
				pr_err("cannot read 0x%02x\n", i);
				rc = 0;
			}
			pr_err("%s from 0x%02x read 0x%02x\n", __func__, (uint8_t)i, rc);
		}

		tcpci_unlock_typec(chip->tcpc);
	}
	sts = rc & WUSB3801_INT_STS_MASK;

	rc = wusb3801_i2c_read8(chip->tcpc, WUSB3801_REG_STATUS);
	if (rc < 0) {
		pr_err("%s: failed to read reg status\n", __func__);
		return;
	}
	pr_info("%s WUSB3801_REG_STATUS : 0x%02x\n", __func__, rc);

	pr_info("%s: sts[0x%02x]\n", __func__, sts);
	status = (rc & WUSB3801_ATTACH) ? true : false;
	type = status ? rc & WUSB3801_TYPE_MASK : WUSB3801_TYPE_INVALID;
	pr_info("sts[0x%02x], type[0x%02x]\n", status, type);
	if (sts & WUSB3801_INT_DETACH) {
#ifdef __TEST_CC_PATCH__
		if (chip->cc_test_flag == 1) {
			pr_err("%s: test_cc_patch not used int and return \n", __func__);
			tcpci_unlock_typec(tcpc);
			return;
		}
#endif /* __TEST_CC_PATCH__ */
		typec_cc_orientation = 0x0;
		tcpc->typec_attach_old = tcpc->typec_attach_new;

		tcpc->typec_attach_new = TYPEC_UNATTACHED;

		tcpci_notify_typec_state(tcpc);

		tcpci_source_vbus(tcpc, TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_0V, 0);
	}
	if (sts & WUSB3801_INT_ATTACH) {
#ifdef __TEST_CC_PATCH__
		if (chip->dev_sub_id != 0xA0) {
			if (chip->cc_test_flag == 0 && BITS_GET(rc, WUSB3801_CC_STS_MASK) == 0) {
				chip->cc_sts = test_cc_patch(chip);
				chip->cc_test_flag = 1;
				pr_err("%s: cc_sts[0x%02x]\n", __func__, chip->cc_sts);
				tcpci_unlock_typec(tcpc);
				return;
			}
			if (chip->cc_test_flag == 1) {
				chip->cc_test_flag = 0;
				if (BITS_GET(rc, WUSB3801_CC_STS_MASK) == 0) {
					if (chip->cc_sts == WUSB3801_CC2_CONNECTED)
						rc = rc | 0x02;
					else if (chip->cc_sts == WUSB3801_CC1_CONNECTED)
						rc = rc | 0x01;
					pr_err("%s: cc_test_patch rc[0x%02x]\n",
						   __func__, rc);
				} else
					pr_err("%s:rc[0x%x},cc is not null\n",
						   __func__, rc);
			}
		}
		typec_cc_orientation = BITS_GET(rc, WUSB3801_CC_STS_MASK);
		if(typec_cc_orientation == WUSB3801_CC1) {
			tcpc->typec_polarity = WUSB3801_CC1_CONNECTED;
		} else if (typec_cc_orientation == WUSB3801_CC2) {
			tcpc->typec_polarity = WUSB3801_CC2_CONNECTED;
		}
		pr_err("%s:typec_cc_orientation=0x%x,tcpc->typec_polarity=%d\n", __func__, typec_cc_orientation, tcpc->typec_polarity);
#endif /* __TEST_CC_PATCH__ */
		switch (type) {
		case WUSB3801_TYPE_SNK:
			/*if ( tcpc->typec_role != TYPEC_ROLE_SRC) {
				tcpc->typec_role = TYPEC_ROLE_SRC;
				tcpci_notify_role_swap(tcpc, TCP_NOTIFY_DR_SWAP, PD_ROLE_DFP);
		}*/
			if (tcpc->typec_attach_new != TYPEC_ATTACHED_SRC && tcpc->typec_attach_new != TYPEC_ATTACHED_AUDIO) {
				tcpc->typec_attach_old = tcpc->typec_attach_new;
				tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
				tcpci_source_vbus(tcpc, TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, 0);
				tcpci_notify_typec_state(tcpc);
			}
			break;
		case WUSB3801_TYPE_SRC:
			/*if ( tcpc->typec_role != TYPEC_ROLE_SNK) {
				tcpc->typec_role = TYPEC_ROLE_SNK;
				tcpci_notify_role_swap(tcpc, TCP_NOTIFY_DR_SWAP, PD_ROLE_UFP);
		 }*/
			if (tcpc->typec_attach_new != TYPEC_ATTACHED_SNK && tcpc->typec_attach_new != TYPEC_ATTACHED_AUDIO) {
				tcpc->typec_attach_old = tcpc->typec_attach_new;
				tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
				tcpci_notify_typec_state(tcpc);
			}
			break;
		case WUSB3801_TYPE_AUD_ACC:

			if (tcpc->typec_attach_new != TYPEC_ATTACHED_SNK  && tcpc->typec_attach_new != TYPEC_ATTACHED_SRC) {
				if (is_audio_support <= 0) {
					pr_err("no support typec-audio out\n");
					break;
				}
				pr_err("%s:%d: TYPEC_ATTACHED_AUDIO  return\n", __func__ , tcpc->typec_attach_new);
				tcpc->typec_attach_old = tcpc->typec_attach_new;
				tcpc->typec_attach_new = TYPEC_ATTACHED_AUDIO;
				pr_err("%s: new :%u  old = %d \n", __func__ , tcpc->typec_attach_new, tcpc->typec_attach_old);
				tcpci_notify_typec_state(tcpc);
			}
			break;
		default:
			pr_err("%s: Unknwon type[0x%02x]\n", __func__, type);
			break;
		}
	}

	if (tcpc->typec_attach_new == TYPEC_UNATTACHED)
		wusb3801_unattached_cc_entry(tcpc);

	tcpci_unlock_typec(tcpc);
}

static irqreturn_t wusb3801_intr_handler(int irq, void *data)
{
	struct wusb3801_chip *chip = data;

	__pm_wakeup_event(chip->for_irq_wake_lock, WUSB3801_IRQ_WAKE_TIME);

	kthread_queue_work(&chip->irq_worker, &chip->irq_work);
	return IRQ_HANDLED;
}

static int wusb3801_init_alert(struct tcpc_device *tcpc)
{
	struct wusb3801_chip *chip = tcpc_get_dev_data(tcpc);
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
										&chip->irq_worker, "chip->tcpc_desc->name");
	if (IS_ERR(chip->irq_worker_task)) {
		pr_err("Error: Could not create tcpc task\n");
		goto init_alert_err;
	}

	sched_setscheduler(chip->irq_worker_task, SCHED_FIFO, &param);
	kthread_init_work(&chip->irq_work, wusb3801_irq_work_handler);

	pr_info("IRQF_NO_THREAD Test\r\n");
	i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	ret = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_TEST_02);
	if (ret & WUSB3801_FORCE_ERR_RCY_MASK) {
		pr_err("wusb3801 [%s]enter error recovery :0x%x\n", __func__, ret);
		wusb3801_i2c_write8(chip->tcpc, WUSB3801_REG_TEST_02, 0x00);
	}
	ret = request_irq(chip->irq, wusb3801_intr_handler,
		IRQF_TRIGGER_FALLING | IRQF_NO_THREAD |
		IRQF_NO_SUSPEND, name, chip);
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

int wusb3801_alert_status_clear(struct tcpc_device *tcpc, uint32_t mask)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

static int wusb3801_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
	int rc;
	pr_info("%s enter \n", __func__);

	rc = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_CONTROL0);
	if (rc < 0) {
		pr_err("%s: fail to read mode\n", __func__);
		return rc;
	}

	rc |= 0x80;
	pr_info("%s %d 0x%x\n", __func__, __LINE__, rc);
	rc = i2c_smbus_write_byte_data(w_client,
								   WUSB3801_REG_CONTROL0, rc);

	if (rc < 0) {
		pr_err("failed to write mode(%d)\n", rc);
		return rc;
	}

	return 0;
}

int wusb3801_fault_status_clear(struct tcpc_device *tcpc, uint8_t status)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

int wusb3801_get_alert_mask(struct tcpc_device *tcpc, uint32_t *mask)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

int wusb3801_get_alert_status(struct tcpc_device *tcpc, uint32_t *alert)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

static int wusb3801_get_power_status(
	struct tcpc_device *tcpc, uint16_t *pwr_status)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

int wusb3801_get_fault_status(struct tcpc_device *tcpc, uint8_t *status)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

static int wusb3801_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

int wusb3801_typec_cc_orientation(void)
{
	if (typec_cc_orientation != 0) {
		return typec_cc_orientation;
	} else {
		return -2;
	}
}

static int wusb3801_set_role(struct tcpc_device *tcpc, int mode);
static int wusb3801_set_cc(struct tcpc_device *tcpc, int pull)
{
	int ret;
	uint8_t mode = 0;

	pr_info("%s enter\n", __func__);

	pull = TYPEC_CC_PULL_GET_RES(pull);
	if (pull == TYPEC_CC_RP)
		mode = REVERSE_CHG_SOURCE;
	else if (pull == TYPEC_CC_RD)
		mode = REVERSE_CHG_SINK;
	else if (pull == TYPEC_CC_DRP)
		mode = REVERSE_CHG_DRP;

	ret = wusb3801_set_role(tcpc, mode);
	if (ret < 0) {
		pr_err("%s: set mode fail!\n", __func__);
	}

	return 0;
}

void wusb3801_set_sinkonly(struct tcpc_device *tcpc)
{
	int rc = 0;
	int ret = 0;
	int retry = 0;
	uint8_t mode = 0;

	if (is_probe_sucess == 0) {
		pr_err("%s: not for this chip!\n", __func__);
		return;
	}
	pr_info("%s: enter!\n", __func__);

	while (retry < RETRY_CNT) {
		rc = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_TEST_02);
		if (rc < 0) {
			retry++;
			msleep(WUSB3801_WAIT_SYS_WAKEUP_BY_USBTEMP);
			pr_err("%s: fail to read WUSB3801_REG_TEST_02, retry = %d!\n", __func__, retry);
		}
		else {
			break;
		}
	}
	retry = 0;
	pr_info("%s: original: reg:0x%02x, value:0x%02x\n", __func__, WUSB3801_REG_TEST_02, rc);

	rc &= ~(TEST02_DIS_RD_RP << TEST02_RD_RP_DIS_SHIFT);
	rc = i2c_smbus_write_byte_data(w_client,
								   WUSB3801_REG_TEST_02, rc);
	if (rc < 0) {
		pr_err("%s: fail to write WUSB3801_REG_TEST_02, rc = %d\n", __func__, rc);
		return;
	}

	rc = 0x00;
	rc = i2c_smbus_write_byte_data(w_client,
								   WUSB3801_REG_TEST_04, rc);
	if (rc < 0) {
		pr_err("%s: fail to write WUSB3801_REG_TEST_04, rc = %d\n", __func__, rc);
		return;
	}

	rc = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_TEST_02);
	if (rc < 0) {
	  pr_err("%s: fail to read WUSB3801_REG_TEST_02\n", __func__);
	  return;
	}
	pr_info("%s: cc default: reg:0x%02x, value:0x%02x\n", __func__, WUSB3801_REG_TEST_02, rc);

	mode = REVERSE_CHG_SINK;
	ret = wusb3801_set_role(tcpc, mode);
	if (ret < 0) {
		pr_err("%s: set mode fail!\n", __func__);
		return;
	}
	pr_info("%s: set sink success!\n", __func__);

	return;
}
EXPORT_SYMBOL(wusb3801_set_sinkonly);

void wusb3801_set_cc_open(struct tcpc_device *tcpc)
{
	int rc = 0;

	if (is_probe_sucess == 0) {
		pr_err("%s: not for this chip!\n", __func__);
		return;
	}
	pr_info("%s: enter!\n", __func__);

	rc = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_TEST_02);
	if (rc < 0) {
		pr_err("%s: fail to read WUSB3801_REG_TEST_02\n", __func__);
		return;
	}
	pr_info("%s: original: reg:0x%02x, value:0x%02x\n", __func__, WUSB3801_REG_TEST_02, rc);

	rc |= (TEST02_DIS_RD_RP << TEST02_RD_RP_DIS_SHIFT);
	rc = i2c_smbus_write_byte_data(w_client,
								   WUSB3801_REG_TEST_02, rc);
	if (rc < 0) {
		pr_err("%s: fail to write WUSB3801_REG_TEST_02, rc = %d\n", __func__, rc);
		return;
	}

	rc = 0x55;
	rc = i2c_smbus_write_byte_data(w_client,
								   WUSB3801_REG_TEST_04, rc);
	if (rc < 0) {
		pr_err("%s: fail to write WUSB3801_REG_TEST_04, rc = %d\n", __func__, rc);
		return;
	}

	rc = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_TEST_02);
	if (rc < 0) {
	   pr_err("%s: fail to read WUSB3801_REG_TEST_02\n", __func__);
	   return;
	}
	pr_info("%s: cc open: reg:0x%02x, value:0x%02x\n", __func__, WUSB3801_REG_TEST_02, rc);

	return;
}
EXPORT_SYMBOL(wusb3801_set_cc_open);

void wusb3801_set_cc_open_recover(struct tcpc_device *tcpc)
{
	int rc = 0;

	if (is_probe_sucess == 0) {
		pr_err("%s: not for this chip!\n", __func__);
		return;
	}
	pr_info("%s: enter!\n", __func__);

	rc = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_TEST_02);
	if (rc < 0) {
		pr_err("%s: fail to read WUSB3801_REG_TEST_02\n", __func__);
		return;
	}
	pr_info("%s: original: reg:0x%02x, value:0x%02x\n", __func__, WUSB3801_REG_TEST_02, rc);

	rc = i2c_smbus_write_byte_data(w_client,
								   WUSB3801_REG_TEST_02, 0x00);
	if (rc < 0) {
		pr_err("%s: fail to write WUSB3801_REG_TEST_02, rc = %d\n", __func__, rc);
		return;
	}

	rc = i2c_smbus_write_byte_data(w_client,
								   WUSB3801_REG_TEST_04, 0x00);
	if (rc < 0) {
		pr_err("%s: fail to write WUSB3801_REG_TEST_04, rc = %d\n", __func__, rc);
		return;
	}

	rc = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_TEST_02);
	if (rc < 0) {
	   pr_err("%s: fail to read WUSB3801_REG_TEST_02\n", __func__);
	   return;
	}
	pr_info("%s: open recover: reg:0x%02x, value:0x%02x\n", __func__, WUSB3801_REG_TEST_02, rc);

	return;
}
EXPORT_SYMBOL(wusb3801_set_cc_open_recover);

static int wusb3801_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

static int wusb3801_set_low_rp_duty(struct tcpc_device *tcpc, bool low_rp)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

static int wusb3801_set_vconn(struct tcpc_device *tcpc, int enable)
{
	pr_info("%s enter \n", __func__);
	return 0;
}

static int wusb3801_tcpc_deinit(struct tcpc_device *tcpc_dev)
{
	pr_info("%s enter \n", __func__);

#if IS_ENABLED(CONFIG_TCPC_SHUTDOWN_CC_DETACH)
	wusb3801_set_cc(tcpc_dev, TYPEC_CC_RD);
#endif

	return 0;
}

static int wusb3801_set_role(struct tcpc_device *tcpc, int mode)
{
	int rc = 0;

	rc = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_CONTROL0);
	if (rc < 0) {
		pr_err("%s: fail to read mode\n", __func__);
		return rc;
	}
	pr_err("dhx--set role %d\n", mode);
	rc &= ~WUSB3801_MODE_MASK;
	rc &= ~WUSB3801_INT_MASK;
	if (mode == REVERSE_CHG_SOURCE) {
		rc |= 0x02;
	} else if (mode == REVERSE_CHG_SINK) {
		rc |= 0x80;
	} else if (mode == REVERSE_CHG_DRP) {
		rc |= 0x04;
	} else {
		return 0;
	}

	rc = i2c_smbus_write_byte_data(w_client,
								   WUSB3801_REG_CONTROL0, rc);

	if (rc < 0) {
		pr_err("failed to write mode(%d)\n", rc);
		return rc;
	}

	rc = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_CONTROL0);
	if (rc < 0) {
		pr_err("%s: fail to clear chip interrupt\n", __func__);
		return rc;
	}

	pr_err("dhx--set role  end %x\n", rc);

	return rc;
}

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
static int wusb3801_set_msg_header(struct tcpc_device *tcpc,
								   uint8_t power_role, uint8_t data_role)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int wusb3801_set_rx_enable(struct tcpc_device *tcpc, uint8_t enable)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int wusb3801_protocol_reset(struct tcpc_device *tcpc_dev)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int wusb3801_get_message(struct tcpc_device *tcpc, uint32_t *payload,
								uint16_t *msg_head, enum tcpm_transmit_type *frame_type)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int wusb3801_transmit(struct tcpc_device *tcpc,
							 enum tcpm_transmit_type type, uint16_t header, const uint32_t *data)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int wusb3801_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int wusb3801_set_bist_carrier_mode(
	struct tcpc_device *tcpc, uint8_t pattern)
{
	pr_err("%s\n", __func__);
	return 0;
}
#endif

static struct tcpc_ops wusb3801_tcpc_ops = {
	.init = wusb3801_tcpc_init,
	.alert_status_clear = wusb3801_alert_status_clear,
	.fault_status_clear = wusb3801_fault_status_clear,
	.get_alert_mask = wusb3801_get_alert_mask,
	.get_alert_status = wusb3801_get_alert_status,
	.get_power_status = wusb3801_get_power_status,
	.get_fault_status = wusb3801_get_fault_status,
	.get_cc = wusb3801_get_cc,
	.set_cc = wusb3801_set_cc,
	/* .set_role = wusb3801_set_role, */
	/* .get_mode = wusb3801_tcpc_get_mode, */
	.set_polarity = wusb3801_set_polarity,
	.set_low_rp_duty = wusb3801_set_low_rp_duty,
	.set_vconn = wusb3801_set_vconn,
	.deinit = wusb3801_tcpc_deinit,

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
	.set_msg_header = wusb3801_set_msg_header,
	.set_rx_enable = wusb3801_set_rx_enable,
	.protocol_reset = wusb3801_protocol_reset,
	.get_message = wusb3801_get_message,
	.transmit = wusb3801_transmit,
	.set_bist_test_mode = wusb3801_set_bist_test_mode,
	.set_bist_carrier_mode = wusb3801_set_bist_carrier_mode,
#endif /* CONFIG_USB_POWER_DELIVERY */
};

static int mt_parse_dt(struct wusb3801_chip *chip, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	if (!np)
		return -EINVAL;

	pr_info("%s\n", __func__);

	np = of_find_node_by_name(NULL, "usb_type_c_wusb3801x");
	if (!np) {
		pr_err("%s find node type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	ret = of_property_read_u32(
		np, "audio,support", &is_audio_support);
	if (ret <= 0)
		pr_err("%s no support typec-audio\n", __func__);
#if (!IS_ENABLED(CONFIG_MTK_GPIO) || IS_ENABLED(CONFIG_MTK_GPIOLIB_STAND))
	ret = of_get_named_gpio(np, "wusb3801,irq-gpio", 0);
	if (ret < 0) {
		pr_err("%s no intr_gpio info\n", __func__);
		return ret;
	}
	chip->irq_gpio = ret;
#else
	ret = of_property_read_u32(
		np, "wusb3801,irq-gpio_num", &chip->irq_gpio);
	if (ret < 0)
		pr_err("%s no intr_gpio info\n", __func__);
#endif
	return ret;
}

static void wusb3801_first_check_typec_work(struct work_struct *work)
{
	struct wusb3801_chip *chip = container_of(work, struct wusb3801_chip, first_check_typec_work.work);
	int status, type, rc, in_sts;

	tcpci_lock_typec(chip->tcpc);
	if (chip->cc_test_flag == 1 || typec_cc_orientation != 0) {
		pr_err("%s: first enter interrupt and return \n", __func__);
		tcpci_unlock_typec(chip->tcpc);
		return;
	}
	/* get interrupt */
	rc = wusb3801_i2c_read8(chip->tcpc, WUSB3801_REG_INTERRUPT);
	if (rc < 0) {
		pr_err("%s: failed to read interrupt\n", __func__);
		return;
	}
	in_sts = rc & WUSB3801_INT_STS_MASK;

	first_check_flag = 1;
	rc = wusb3801_i2c_read8(chip->tcpc, WUSB3801_REG_STATUS);
	if (rc < 0) {
		pr_err("%s: failed to read reg status\n", __func__);
		return;
	}
	pr_info("%s WUSB3801_REG_STATUS : 0x%02x\n", __func__, rc);

	pr_info("%s: in_sts[0x%02x]\n", __func__, in_sts);
	status = (rc & WUSB3801_ATTACH) ? true : false;
	type = status ? rc & WUSB3801_TYPE_MASK : WUSB3801_TYPE_INVALID;
	pr_info("sts[0x%02x], type[0x%02x]\n", status, type);
	if (status) {
#ifdef __TEST_CC_PATCH__
		if (chip->dev_sub_id != 0xA0) {
			if (chip->cc_test_flag == 0 && BITS_GET(rc, WUSB3801_CC_STS_MASK) == 0 && type == WUSB3801_TYPE_SRC) {
				chip->cc_sts = test_cc_patch(chip);
				chip->cc_test_flag = 1;
				pr_err("%s: cc_sts[0x%02x]\n", __func__, chip->cc_sts);
				tcpci_unlock_typec(chip->tcpc);
				return;
			}
			if (chip->cc_test_flag == 1) {
				chip->cc_test_flag = 0;
				if (chip->cc_sts == WUSB3801_CC2_CONNECTED) {
					rc = rc | 0x02;
				} else if (chip->cc_sts == WUSB3801_CC1_CONNECTED) {
					rc = rc | 0x01;
				}
				pr_err("%s: cc_test_patch rc[0x%02x]\n", __func__, rc);
			}
		}
		typec_cc_orientation = BITS_GET(rc, WUSB3801_CC_STS_MASK);
#endif /* __TEST_CC_PATCH__ */
		switch (type) {
		case WUSB3801_TYPE_SNK:
			chip->tcpc->typec_attach_old = chip->tcpc->typec_attach_new;
			chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SRC;
			tcpci_source_vbus(chip->tcpc, TCP_VBUS_CTRL_TYPEC, TCPC_VBUS_SOURCE_5V, 0);
			tcpci_notify_typec_state(chip->tcpc);
			break;
		case WUSB3801_TYPE_SRC:
			chip->tcpc->typec_attach_old = chip->tcpc->typec_attach_new;
			chip->tcpc->typec_attach_new = TYPEC_ATTACHED_SNK;
			tcpci_notify_typec_state(chip->tcpc);
			break;
		case WUSB3801_TYPE_AUD_ACC:
			if (is_audio_support <= 0) {
				pr_err("no support typec-audio out\n");
				break;
			}
			chip->tcpc->typec_attach_old = chip->tcpc->typec_attach_new;
			chip->tcpc->typec_attach_new = TYPEC_ATTACHED_AUDIO;
			pr_err("%s: new :%u  old = %d \n", __func__ , chip->tcpc->typec_attach_old, chip->tcpc->typec_attach_new);
			tcpci_notify_typec_state(chip->tcpc);
			break;
		default:
			pr_err("%s: Unknwon type[0x%02x]\n", __func__, type);
			break;
		}
	}
	tcpci_unlock_typec(chip->tcpc);
}
static int wusb3801_tcpcdev_init(struct wusb3801_chip *chip, struct device *dev)
{
	struct tcpc_desc *desc;
	struct device_node *np;
	u32 val, len;

	const char *name = "default";

	np = of_find_node_by_name(NULL, "usb_type_c_wusb3801x");
	if (!np) {
		pr_err("%s find node mt6370 fail\n", __func__);
		return -ENODEV;
	}

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	if (of_property_read_u32(np, "wusb3801-tcpc,role_def", &val) >= 0) {
		if (val >= TYPEC_ROLE_NR)
			desc->role_def = TYPEC_ROLE_DRP;
		else
			desc->role_def = val;
	} else {
		dev_info(dev, "use default Role DRP\n");
		desc->role_def = TYPEC_ROLE_DRP;
	}

	if (of_property_read_u32(
			np, "wusb3801-tcpc,notifier_supply_num", &val) >= 0) {
		if (val < 0)
			desc->notifier_supply_num = 0;
		else
			desc->notifier_supply_num = val;
	} else
		desc->notifier_supply_num = 0;

	if (of_property_read_u32(np, "wusb3801,rp_level", &val) >= 0) {
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
	desc->rp_lvl = TYPEC_CC_RP_1_5;

	of_property_read_string(np, "wusb3801-tcpc,name", (char const **)&name);

	len = strlen(name);
	desc->name = kzalloc(len + 1, GFP_KERNEL);
	if (!desc->name)
		return -ENOMEM;

	strlcpy((char *)desc->name, name, len + 1);

	chip->tcpc_desc = desc;

	chip->tcpc = tcpc_device_register(dev,
									  desc, &wusb3801_tcpc_ops, chip);
	if (IS_ERR(chip->tcpc))
		return -EINVAL;

	chip->tcpc->typec_attach_old = TYPEC_UNATTACHED;
	chip->tcpc->typec_attach_new = TYPEC_UNATTACHED;
	schedule_delayed_work(
		&chip->first_check_typec_work, msecs_to_jiffies(3000));
	return 0;
}

static uint8_t dev_sub_id;
static inline int wusb3801_check_revision(struct i2c_client *client)
{
	int rc;
	rc = i2c_smbus_read_byte_data(client, WUSB3801_REG_VERSION_ID);
	if (rc < 0)
		return rc;

	pr_info("VendorID register: 0x%02x\n", rc);
	if ((rc & WUSB3801_VENDOR_ID_MASK) != WUSB3801_VENDOR_ID) {
		return -EINVAL;
	}
	pr_info("Vendor id: 0x%02x, Version id: 0x%02x\n", rc & WUSB3801_VENDOR_ID_MASK,
			(rc & WUSB3801_VERSION_ID_MASK) >> 3);

	rc = i2c_smbus_read_byte_data(client, WUSB3801_REG_TEST_01);
	if (rc > 0)
		dev_sub_id = rc & WUSB3801_VENDOR_SUB_ID_MASK;
	pr_info("VendorSUBID register: 0x%02x\n", rc & WUSB3801_VENDOR_SUB_ID_MASK);

	return WUSB3801_VENDOR_ID;
}
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
static ssize_t wusb3801_regdump_show(struct device *dev,
									 struct device_attribute *attr,
									 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int i, rc, ret = 0;

	tcpci_lock_typec(chip->tcpc);
	for (i = WUSB3801_REG_VERSION_ID; i <= WUSB3801_REG_TEST_12; i++) {
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

DEVICE_ATTR(wusb3801_regdump, S_IRUGO, wusb3801_regdump_show, NULL);

#ifdef __TEST_CC_PATCH__
/************************************************************************
 *
 *       fcc_status_show
 *
 *  Description :
 *  -------------
 *  show cc_status
 *
 ************************************************************************/
static ssize_t typec_cc_orientation_show(struct device *dev,
										 struct device_attribute *attr,
										 char *buf)
{
	int ret;
	ret = snprintf(buf, PAGE_SIZE, "cc_sts (%d)\n", typec_cc_orientation);
	return ret;
}

DEVICE_ATTR(typec_cc_orientation, S_IRUGO, typec_cc_orientation_show, NULL);
#endif /*  __TEST_CC_PATCH__	 */

static int wusb3801_i2c_probe(struct i2c_client *client,
							  const struct i2c_device_id *id)
{
	struct wusb3801_chip *chip;
	int ret = 0, chip_id;
	int i, rc;
	bool use_dt = client->dev.of_node;

	pr_info("%s\n", __func__);
	if (i2c_check_functionality(client->adapter,
								I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_SMBUS_BYTE_DATA))
		pr_info("I2C functionality : OK...\n");
	else
		pr_info("I2C functionality check : failuare...\n");

	chip_id = wusb3801_check_revision(client);
	if (chip_id < 0) {
		chip_id = wusb3801_check_revision(client);
		if (chip_id < 0)
			return chip_id;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	if (use_dt)
		mt_parse_dt(chip, &client->dev);
	else {
		dev_err(&client->dev, "no dts node\n");
		return -ENODEV;
	}
	chip->dev = &client->dev;
	chip->client = client;
#ifdef __TEST_CC_PATCH__
	chip->cc_sts = 0xFF;
	chip->cc_test_flag = 0;
	chip->dev_sub_id = dev_sub_id;
	typec_cc_orientation = 0;
#endif /* __TEST_CC_PATCH__ */
	sema_init(&chip->io_lock, 1);
	sema_init(&chip->suspend_lock, 1);
	i2c_set_clientdata(client, chip);
	w_client = chip->client;
	INIT_DELAYED_WORK(&chip->first_check_typec_work, wusb3801_first_check_typec_work);
	chip->for_irq_wake_lock =
		wakeup_source_register(chip->dev, "wusb3801_irq_wakelock");
	chip->for_i2c_wake_lock =
		wakeup_source_register(chip->dev, "wusb3801_i2c_wakelock");

	chip->chip_id = chip_id;
	pr_info("wusb3801_chipID = 0x%0x\n", chip_id);

	ret = wusb3801_tcpcdev_init(chip, &client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "wusb3801 tcpc dev init fail\n");
		goto err_tcpc_reg;
	}

	ret = wusb3801_init_alert(chip->tcpc);
	if (ret < 0) {
		pr_err("wusb3801 init alert fail\n");
		goto err_irq_init;
	}

	ret = device_create_file(&client->dev, &dev_attr_wusb3801_regdump);
	if (ret < 0) {
		dev_err(&client->dev, "failed to create dev_attr_fregdump\n");
		ret = -ENODEV;
		goto err_create_fregdump_file;
	}
#ifdef __TEST_CC_PATCH__
	ret = device_create_file(&client->dev, &dev_attr_typec_cc_orientation);
	if (ret < 0) {
		dev_err(&client->dev, "failed to create dev_attr_typec_cc_orientation\n");
		ret = -ENODEV;
		goto err_create_file;
	}
#endif /* __TEST_CC_PATCH__ */

	ret = i2c_smbus_read_byte_data(w_client, WUSB3801_REG_CONTROL0);
	if (ret < 0) {
		pr_err("%s: fail to read mode\n", __func__);
		return ret;
	}

	ret |= 0x80;
	pr_info("%s %d 0x%x\n", __func__, __LINE__, ret);
	ret = i2c_smbus_write_byte_data(w_client,
									WUSB3801_REG_CONTROL0, ret);

	if (ret < 0) {
		pr_err("failed to write mode(%d)\n", ret);
		return ret;
	}

	for (i = WUSB3801_REG_VERSION_ID; i <= WUSB3801_REG_TEST_12; i++) {
		rc = i2c_smbus_read_byte_data(chip->client, (uint8_t)i);
		if (rc < 0) {
			pr_err("cannot read 0x%02x\n", i);
			rc = 0;
		}
		pr_err("from 0x%02x read 0x%02x\n", (uint8_t)i, rc);
	}

	is_probe_sucess = 1;
	wusb3801_set_sinkonly(chip->tcpc);

	pr_info("%s probe OK!\n", __func__);
	return 0;

#ifdef __TEST_CC_PATCH__
err_create_file:
	device_remove_file(&client->dev, &dev_attr_typec_cc_orientation);
#endif /* __TEST_CC_PATCH__ */
err_create_fregdump_file:
	device_remove_file(&client->dev, &dev_attr_wusb3801_regdump);
err_irq_init:
	tcpc_device_unregister(chip->dev, chip->tcpc);
err_tcpc_reg:
	wakeup_source_unregister(chip->for_i2c_wake_lock);
	wakeup_source_unregister(chip->for_irq_wake_lock);
	return ret;
}

static int wusb3801_i2c_remove(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);

	if (chip) {
		cancel_delayed_work_sync(&chip->first_check_typec_work);
		tcpc_device_unregister(chip->dev, chip->tcpc);
#ifdef __TEST_CC_PATCH__
		device_remove_file(&client->dev, &dev_attr_typec_cc_orientation);
#endif /* __TEST_CC_PATCH__ */
		device_remove_file(&client->dev, &dev_attr_wusb3801_regdump);
	}

	return 0;
}

#if CONFIG_PM
static int wusb3801_i2c_suspend(struct device *dev)
{
	struct wusb3801_chip *chip;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip) {
			down(&chip->suspend_lock);
		}
	}

	return 0;
}

static int wusb3801_i2c_resume(struct device *dev)
{
	struct wusb3801_chip *chip;
	struct i2c_client *client = to_i2c_client(dev);

	if (client) {
		chip = i2c_get_clientdata(client);
		if (chip)
			up(&chip->suspend_lock);
	}

	return 0;
}

static void wusb3801_shutdown(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);

	/* Please reset IC here */
	if (chip != NULL) {
		tcpm_shutdown(chip->tcpc);
		if (chip->irq)
			disable_irq(chip->irq);
	}
}

#if IS_ENABLED(CONFIG_PM_RUNTIME)
static int wusb3801_pm_suspend_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: suspending...\n");
	return 0;
}

static int wusb3801_pm_resume_runtime(struct device *device)
{
	dev_dbg(device, "pm_runtime: resuming...\n");
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops wusb3801_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
		wusb3801_i2c_suspend,
		wusb3801_i2c_resume)
#if IS_ENABLED(CONFIG_PM_RUNTIME)
		SET_RUNTIME_PM_OPS(
			wusb3801_pm_suspend_runtime,
			wusb3801_pm_resume_runtime,
			NULL)
#endif /* CONFIG_PM_RUNTIME */
};
#define wusb3801_PM_OPS (&wusb3801_pm_ops)
#else
#define wusb3801_PM_OPS (NULL)
#endif /* CONFIG_PM */

static const struct i2c_device_id wusb3801_id_table[] = {
	{"wusb3801", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, wusb3801_id_table);

static const struct of_device_id rt_match_table[] = {
	{ .compatible = "mediatek,usb_type_c_wusb3801", },
	{},
};

static struct i2c_driver wusb3801_driver = {
	.driver = {
		.name = "usb_type_c_wusb3801",
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
		.pm = wusb3801_PM_OPS,
	},
	.probe = wusb3801_i2c_probe,
	.remove = wusb3801_i2c_remove,
	.shutdown = wusb3801_shutdown,
	.id_table = wusb3801_id_table,
};

static int __init wusb3801_init(void)
{
	struct device_node *np;

	pr_info("%s (%s): initializing...\n", __func__, WUSB3801_DRV_VERSION);
	np = of_find_node_by_name(NULL, "usb_type_c_wusb3801x");
	if (np != NULL)
		pr_info("usb_type_c node found...\n");
	else
		pr_info("usb_type_c node not found...\n");

	return i2c_add_driver(&wusb3801_driver);
}
subsys_initcall(wusb3801_init);

static void __exit wusb3801_exit(void)
{
	i2c_del_driver(&wusb3801_driver);
}
module_exit(wusb3801_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NULL");
MODULE_DESCRIPTION("wusb3801 TCPC Driver");
MODULE_VERSION(WUSB3801_DRV_VERSION);
