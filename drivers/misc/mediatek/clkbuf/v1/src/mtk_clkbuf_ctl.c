// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: ren-ting.wang <ren-ting.wang@mediatek.com>
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "mtk_clkbuf_ctl.h"
#include "mtk-clkbuf-dcxo.h"
#include "mtk-clkbuf-pmif.h"

#include "mtk-srclken-rc-hw.h"

DEFINE_MUTEX(clk_buf_lock);

static struct clkbuf_misc clkbuf_ctl;

int clk_buf_ctrl(const char *xo_name, bool onoff)
{
	int id;

	id = clkbuf_dcxo_get_xo_id_by_name(xo_name);
	if (id < 0) {
		pr_notice("xo name: %s not found, err: %d\n", xo_name, id);
		return id;
	}

	return clkbuf_dcxo_set_xo_sw_en(id, onoff);
}
EXPORT_SYMBOL(clk_buf_ctrl);

int clk_buf_hw_ctrl(const char *xo_name, bool onoff)
{
	int id;
	struct xo_buf_ctl_cmd_t ctl_cmd = {
		.hw_id = CLKBUF_DCXO,
		.cmd = onoff,
	};

	id = clkbuf_dcxo_get_xo_id_by_name(xo_name);
	if (id < 0) {
		pr_notice("xo name: %s not found, err: %d\n", xo_name, id);
		return id;
	}

	if (!clkbuf_dcxo_get_xo_controllable(id)) {
		pr_notice("xo name: %s not support sw control\n", xo_name);
		return -EHW_NOT_SUPPORT;
	}

	return clkbuf_dcxo_notify(id, &ctl_cmd);
}
EXPORT_SYMBOL(clk_buf_hw_ctrl);


int clk_buf_set_voter_by_name(const char *xo_name, const char *voter)
{
	int id, ret = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	id = clkbuf_dcxo_get_xo_id_by_name(xo_name);
	if (id < 0) {
		pr_notice("xo name: %s not found, err: %d\n", xo_name, id);
		return id;
	}

	if (id == 0) {
		pr_notice("xo %s is invalid for control!!\n", xo_name);
		return id;
	}

	ret = clkbuf_dcxo_pmic_store("DCXO", xo_name, "EN_BB");

	ret |= clkbuf_dcxo_pmic_store("XO_VOTER", xo_name, voter);

	return ret;
}
EXPORT_SYMBOL(clk_buf_set_voter_by_name);

static ssize_t __clk_buf_dump_xo_en_sta(char *buf)
{
	u32 mode = 0;
	u32 val = 0;
	int len = 0;
	int i;

	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++) {
		if (!clkbuf_dcxo_is_xo_in_use(i))
			continue;

		if (clkbuf_dcxo_get_xo_en(i, &val)) {
			pr_notice("get xo_buf%u en failed\n", i);
			return len;
		}

		if (clkbuf_dcxo_get_xo_mode(i, &mode)) {
			pr_notice("get xo_buf%u mode failed\n", i);
			return len;
		}

		len += snprintf(buf + len, PAGE_SIZE - len,
			"%s   SW(1)/HW(2) CTL: %d, Dis(0)/En(1): %d, RS: %u\n",
			clkbuf_dcxo_get_xo_name(i),
			mode,
			clkbuf_dcxo_get_xo_sw_en(i),
			val);
	}

	return len;
}

static ssize_t __clk_buf_dump_pmif_log(char *buf)
{
	u32 clr_cmd = 0;
	u32 set_cmd = 0;
	u32 clr_addr = 0;
	u32 set_addr = 0;
	u32 inf_en = 0;
	u32 mode_ctl = 0;
	u32 sleep_ctl = 0;
	u32 i = 0;
	int len = 0;
	int ret = 0;

	ret = clkbuf_pmif_get_inf_data(PMIF_CONN_INF,
			&clr_addr, &set_addr, &clr_cmd, &set_cmd);
	if (!ret) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"DCXO_CONN_ADDR0/WDATA0/ADDR1/WDATA1=0x%x 0x%x 0x%x 0x%x\n",
			clr_addr, clr_cmd, set_addr, set_cmd);
	}

	ret = clkbuf_pmif_get_inf_data(PMIF_NFC_INF,
			&clr_addr, &set_addr, &clr_cmd, &set_cmd);
	if (!ret) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"DCXO_NFC_ADDR0/WDATA0/ADDR1/WDATA1=0x%x 0x%x 0x%x 0x%x\n",
			clr_addr, clr_cmd, set_addr, set_cmd);
	}

	ret = clkbuf_pmif_get_inf_en(PMIF_CONN_INF, &inf_en);
	if (!ret) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"CONN_INF_EN: 0x%x, ", inf_en);
	}

	ret = clkbuf_pmif_get_inf_en(PMIF_NFC_INF, &inf_en);
	if (!ret) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"NFC_INF_EN: 0x%x, ", inf_en);
	}

	ret = clkbuf_pmif_get_inf_en(PMIF_RC_INF, &inf_en);
	if (!ret) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"RC_INF_EN: 0x%x\n", inf_en);
	}

	for (i = 0; i < clkbuf_pmif_get_pmif_cnt(); i++) {
		ret = clkbuf_pmif_get_misc_reg(&mode_ctl, &sleep_ctl, i);
		if (ret)
			return len;
		len += snprintf(buf + len, PAGE_SIZE - len,
			"pmif%u mode_ctrl: 0x%x, sleep_ctrl: 0x%x\n",
			i, mode_ctl, sleep_ctl);
	}

	return len;
}

static ssize_t __clk_buf_dump_bblpm_info(char *buf)
{
	u32 val = 0;
	int len = 0;

	if (clkbuf_dcxo_get_bblpm_en(&val))
		return len;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"bblpm_state: %u ", val);

	return len;
}

static ssize_t __clk_buf_show_status_info(char *buf)
{
	int len = 0;

	len +=  __clk_buf_dump_xo_en_sta(buf + len);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"********** clock buffer debug info **********\n");

	len += clkbuf_dcxo_dump_rc_voter_log(buf + len);

	if (clkbuf_ctl.reg_debug)
		len += clkbuf_dcxo_dump_reg_log(buf + len);

	if (clkbuf_ctl.misc_debug)
		len += clkbuf_dcxo_dump_misc_log(buf + len);

	if (clkbuf_ctl.dws_debug)
		len += clkbuf_dcxo_dump_dws(buf + len);

	if (clkbuf_ctl.pmrc_en_debug)
		len += clkbuf_dcxo_dump_pmrc_en(buf + len);

	len += __clk_buf_dump_pmif_log(buf + len);

	len += __clk_buf_dump_bblpm_info(buf + len);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"********** clock buffer command help **********\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
			"PMIC switch on/off: echo DCXO pmic en1 en2 en3 en4 ");
	len += snprintf(buf + len, PAGE_SIZE - len,
			"en5 en6 en7 > /sys/kernel/clk_buf/clk_buf_pmic\n");

	return len;
}

int clk_buf_dump_log(void)
{
	char *buf = NULL;
	int len = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf not init yet\n");
		return -ENODEV;
	}

	buf = vmalloc(CLKBUF_STATUS_INFO_SIZE);
	if (!buf)
		return -ENOMEM;

	len = __clk_buf_show_status_info(buf);
	if (len <= 0) {
		vfree(buf);
		return -EAGAIN;
	}

	pr_notice("%s\n", buf);

	vfree(buf);

	return 0;
}
EXPORT_SYMBOL(clk_buf_dump_log);

int clk_buf_get_xo_en_sta(const char *xo_name)
{
	int ret = 0;
	u32 en = 0;
	u8 i;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf not init yet\n");
		return -ENODEV;
	}

	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++)
		if (!strcmp(xo_name, clkbuf_dcxo_get_xo_name(i)))
			break;

	if (i >= clkbuf_dcxo_get_xo_num()) {
		pr_notice("xo name: %s not found\n", xo_name);
		return -EINVAL;
	}

	ret = clkbuf_dcxo_get_xo_en(i, &en);
	if (ret)
		return ret;

	return en;
}
EXPORT_SYMBOL(clk_buf_get_xo_en_sta);

u8 clk_buf_get_xo_num(void)
{
	return clkbuf_dcxo_get_xo_num();
}
EXPORT_SYMBOL(clk_buf_get_xo_num);

const char *clk_buf_get_xo_name(u8 idx)
{
	return clkbuf_dcxo_get_xo_name(idx);
}
EXPORT_SYMBOL(clk_buf_get_xo_name);

#if IS_ENABLED(CONFIG_PM)
static int __clk_buf_pmic_ctrl_all(const char *arg1)
{
	struct xo_buf_ctl_cmd_t xo_cmd;
	u32 *en = NULL;
	u32 val = 0;
	int i;

	if (kstrtouint(arg1, 16, &val))
		return -EPERM;

	en = kcalloc(clkbuf_dcxo_get_xo_num(), sizeof(u32), GFP_KERNEL);
	if (!en)
		return -ENOMEM;

	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++) {
		en[i] = val & 0x1;
		val >>= 0x1;
	}

	xo_cmd.hw_id = CLKBUF_DCXO;
	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++) {
		xo_cmd.cmd = (en[i] ? CLKBUF_CMD_ON : CLKBUF_CMD_OFF);
		clkbuf_dcxo_notify(i, &xo_cmd);
	}

	kfree(en);

	return 0;
}

static ssize_t clk_buf_pmic_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int len = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	len += snprintf(buf + len, PAGE_SIZE - len,
		"usage: echo DCXO pmic (hex value) > clk_buf_pmic\n");

	len += snprintf(buf + len, PAGE_SIZE - len,
		"usage: echo DCXO (XO_NAME) (CONTROL) > clk_buf_pmic\n");

	len += snprintf(buf + len, PAGE_SIZE - len,
		"usage: echo misc (de-sense/drv-curr/impedance) > clk_buf_pmic\n");

	return len;
}

static ssize_t clk_buf_pmic_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	char cmd[11] = {0};
	char arg1[11] = {0};
	char arg2[11] = {0};
	int ret = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	if (sscanf(buf, "%10s %10s %10s", cmd, arg1, arg2) != 3)
		return -EPERM;

	if ((!strcmp(cmd, "DCXO")) && (!strcmp(arg1, "pmic"))) {
		ret = __clk_buf_pmic_ctrl_all(arg2);
		if (ret) {
			pr_notice("control all xo failed: %d\n", ret);
			return ret;
		}
		goto PMIC_STORE_DONE;
	} else if (!clkbuf_dcxo_pmic_store(cmd, arg1, arg2)) {
		goto PMIC_STORE_DONE;
	}

	pr_notice("unknown command: %s, arg1: %s, arg2: %s\n", cmd, arg1, arg2);

	return -EPERM;

PMIC_STORE_DONE:
	return count;
}

static ssize_t clk_buf_pmif_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int len = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	len += snprintf(buf + len, PAGE_SIZE - len,
		"usage: echo [ON/OFF] [XO_NAME] > clk_buf_pmif\n");

	return len;
}

static ssize_t clk_buf_pmif_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct xo_buf_ctl_cmd_t xo_cmd = { .hw_id = CLKBUF_PMIF };
	char cmd[11] = {0};
	char target[11] = {0};
	int ret = 0;
	u8 i;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	if (sscanf(buf, "%10s %10s", cmd, target) != 2)
		return -EPERM;

	if (!strcmp(target, "RC")) {
		__clk_buf_pmif_rc_inf_store(cmd);
		goto PMIF_STORE_DONE;
	}

	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++)
		if (!strcmp(target, clkbuf_dcxo_get_xo_name(i)))
			break;

	if (i >= clkbuf_dcxo_get_xo_num()) {
		pr_notice("no xo_buf named: %s\n", target);
		return count;
	}

	if (!strcmp(cmd, "ON"))
		xo_cmd.cmd = CLKBUF_CMD_ON;
	else if (!strcmp(cmd, "OFF"))
		xo_cmd.cmd = CLKBUF_CMD_OFF;
	else if (!strcmp(cmd, "INIT"))
		xo_cmd.cmd = CLKBUF_CMD_INIT;
	else {
		pr_notice("unknown command: %s, target: %s\n", cmd, target);
		goto PMIF_STORE_DONE;
	}

	ret = clkbuf_dcxo_notify(i, &xo_cmd);
	if (ret)
		pr_notice("clkbuf pmif cmd failed: %d\n", ret);

PMIF_STORE_DONE:
	return count;
}

static ssize_t clk_buf_ctrl_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int len = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	len += __clk_buf_show_status_info(buf + len);

	return len;
}

static ssize_t clk_buf_debug_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	char cmd[21] = {0};
	u32 val = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	if (sscanf(buf, "%20s %u", cmd, &val) != 2)
		return -EPERM;

	if (!strcmp(cmd, "DEBUG")) {
		clkbuf_ctl.debug = (val ? true : false);
		goto DEBUG_STORE_DONE;
	} else if (!strcmp(cmd, "MISC_DEBUG")) {
		clkbuf_ctl.misc_debug = (val ? true : false);
		goto DEBUG_STORE_DONE;
	} else if (!strcmp(cmd, "DWS_DEBUG")) {
		clkbuf_ctl.dws_debug = (val ? true : false);
		goto DEBUG_STORE_DONE;
	} else if (!strcmp(cmd, "REG_DEBUG")) {
		clkbuf_ctl.reg_debug = (val ? true : false);
		goto DEBUG_STORE_DONE;
	} else if (!strcmp(cmd, "PMRC_EN_DEBUG")) {
		clkbuf_ctl.pmrc_en_debug = (val ? true : false);
		goto DEBUG_STORE_DONE;
	}

	pr_notice("unknown cmd: %s, arg: %u\n", cmd, val);
	return -EPERM;

DEBUG_STORE_DONE:
	return count;
}

static ssize_t clk_buf_debug_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int len = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "clkbuf_debug=%d\n",
			clkbuf_ctl.debug);

	len += snprintf(buf + len, PAGE_SIZE - len, "clkbuf_misc_debug: %d\n",
			clkbuf_ctl.misc_debug);

	len += snprintf(buf + len, PAGE_SIZE - len, "clkbuf_dws_debug: %d\n",
			clkbuf_ctl.dws_debug);

	len += snprintf(buf + len, PAGE_SIZE - len, "clkbuf_reg_debug: %d\n",
			clkbuf_ctl.reg_debug);

	len += snprintf(buf + len,
			PAGE_SIZE - len,
			"clkbuf_pmrc_en_debug: %d\n",
			clkbuf_ctl.pmrc_en_debug);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"available control: DEBUG, MISC_DEBUG, DWS_DEBUG, REG_DEBUG, PMRC_EN_DEBUG\n");

	return len;
}

static ssize_t clk_buf_bblpm_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	u32 bblpm_stat = 0;
	u32 xo_stat = 0;
	u32 hwbblpm_sel = 0;
	int len = 0;
	int ret = 0;
	int i;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++) {
		if (!clkbuf_dcxo_is_xo_in_use(i))
			continue;
		ret = clkbuf_dcxo_get_xo_en(i, &xo_stat);
		if (ret) {
			pr_notice("get xo en stat failed for xo_buf%d\n", i);
			continue;
		}

		len += snprintf(buf + len, PAGE_SIZE - len, "%s enable: %d, ",
				clkbuf_dcxo_get_xo_name(i),
				xo_stat);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	ret = clkbuf_dcxo_get_hwbblpm_sel(&hwbblpm_sel);
	if (!ret) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"hwbblpm sel: %x\n",
			hwbblpm_sel);
	}

	ret = clkbuf_dcxo_get_bblpm_en(&bblpm_stat);
	if (!ret) {
		len += snprintf(buf + len, PAGE_SIZE - len, "bblpm en: %d\n",
			bblpm_stat);
	}

	return len;
}

static ssize_t clk_buf_capid_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	uint32_t capid, capid_old, capid_new;
	int ret;
	const char *capid_buf;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	if (buf != NULL && count != 0) {
		if (!strncmp(buf, "cmd1#", 5)) {
			clkbuf_dcxo_set_capid_pre();
		} else if (!strncmp(buf, "cmd2#", 5)) {
			capid_buf = &buf[5];
			ret = kstrtouint(capid_buf, 0, &capid);

			if (ret) {
				pr_info("wrong format!\n");
				return ret;
			}

			if (capid > 0xFF) {
				pr_info("offset should be within(%x) %x!\n",
					0xFF, capid);
				return -EINVAL;
			}

			clkbuf_dcxo_get_capid(&capid_old);
			pr_info("original cap code: 0x%x\n", capid_old);

			clkbuf_dcxo_set_capid(capid);

			mdelay(1);

			clkbuf_dcxo_get_capid(&capid_new);
			pr_info("write capid 0x%x done. current capid: 0x%x\n",
				capid, capid_new);
		} else {
			ret = kstrtouint(buf, 0, &capid);

			if (ret) {
				pr_info("wrong format!\n");
				return ret;
			}

			if (capid > 0xFF) {
				pr_info("offset should be within(%x) %x!\n",
					0xFF, capid);
				return -EINVAL;
			}

			clkbuf_dcxo_get_capid(&capid_old);
			pr_info("original cap code: 0x%x\n", capid_old);

			clkbuf_dcxo_set_capid_pre();
			clkbuf_dcxo_set_capid(capid);
			mdelay(1);

			clkbuf_dcxo_get_capid(&capid_new);
			pr_info("write capid 0x%x done. current capid: 0x%x\n",
				capid, capid_new);
		}
	} else {
		pr_info("invalid parameter!\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t clk_buf_capid_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	uint32_t capid;
	int len = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	clkbuf_dcxo_get_capid(&capid);
	len += snprintf(buf+len, PAGE_SIZE-len, "dcxo capid: 0x%x\n",
		capid);

	return len;
}

static ssize_t clk_buf_heater_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	bool on;
	const char *capid_buf;
	int ret = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	capid_buf = &buf[0];
	ret = kstrtobool(capid_buf, &on);

	if (ret) {
		pr_info("wrong format!\n");
		return ret;
	}

	if (on)
		clkbuf_dcxo_set_heater(true);
	else
		clkbuf_dcxo_set_heater(false);
	return count;
}

static ssize_t clk_buf_heater_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	bool on;
	int len = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	clkbuf_dcxo_get_heater(&on);
	len += snprintf(buf+len, PAGE_SIZE-len, "dcxo heater: 0x%x\n",
		on);

	return len;
}

static ssize_t clk_buf_all_ctrl_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct xo_buf_ctl_cmd_t xo_cmd = {
		.hw_id = CLKBUF_HW_ALL,
		.cmd = CLKBUF_CMD_SHOW
	};
	char xo_name[16] = {0};
	char cmd[11] = {0};
	int ret = 0;
	u8 i;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf HW not init yet\n");
		return -ENODEV;
	}

	if (sscanf(buf, "%10s %15s", cmd, xo_name) != 2)
		return -EPERM;

	if (!strcmp(cmd, "ON")) {
		xo_cmd.cmd = CLKBUF_CMD_ON;
	} else if (!strcmp(cmd, "OFF")) {
		xo_cmd.cmd = CLKBUF_CMD_OFF;
	} else if (!strcmp(cmd, "INIT")) {
		xo_cmd.cmd = CLKBUF_CMD_INIT;
	} else {
		pr_notice("unknown cmd: %s, xo_name: %s\n", cmd, xo_name);
		return -EPERM;
	}

	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++)
		if (!strcmp(xo_name, clkbuf_dcxo_get_xo_name(i)))
			break;

	if (i >= clkbuf_dcxo_get_xo_num()) {
		pr_notice("no xo_buf named: %s\n", xo_name);
		return count;
	}

	ret = clkbuf_dcxo_notify(i, &xo_cmd);
	if (ret) {
		pr_notice("clkbuf cmd failed: %d\n", ret);
		return ret;
	}

	return count;
}

static ssize_t clk_buf_all_ctrl_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct xo_buf_ctl_cmd_t xo_cmd = {
		.hw_id = CLKBUF_HW_ALL,
		.cmd = CLKBUF_CMD_SHOW,
		};
	int ret = 0;
	u32 val = 0;
	int len = 0;
	u8 i = 0;

	if (!clkbuf_ctl.init_done) {
		pr_notice("clkbuf not init yet\n");
		return -ENODEV;
	}

	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++) {
		if (!clkbuf_dcxo_is_xo_in_use(i))
			continue;

		if (clkbuf_dcxo_get_xo_en(i, &val)) {
			pr_notice("get xo_buf%u en failed\n", i);
			continue;
		}

		xo_cmd.buf = buf + len;

		ret = clkbuf_dcxo_notify(i, &xo_cmd);
		len += strlen(xo_cmd.buf);

		if (ret)
			pr_notice("get xo_buf%u srlkcen_rc status failed\n", i);
	}

	return len;
}

DEFINE_ATTR_RW(clk_buf_all_ctrl);
DEFINE_ATTR_RO(clk_buf_ctrl);
DEFINE_ATTR_RW(clk_buf_pmic);
DEFINE_ATTR_RW(clk_buf_pmif);
DEFINE_ATTR_RW(clk_buf_debug);
DEFINE_ATTR_RO(clk_buf_bblpm);
DEFINE_ATTR_RW(clk_buf_capid);
DEFINE_ATTR_RW(clk_buf_heater);
DEFINE_ATTR_RO(rc_cfg_ctl);
DEFINE_ATTR_RW(rc_sta_reg);
DEFINE_ATTR_RW(rc_trace_ctl);
DEFINE_ATTR_RW(rc_subsys_ctl);
DEFINE_ATTR_RW(rc_subsys_sta);

static struct attribute *clk_buf_attrs[] = {
	/* for clock buffer and RC control */
	__ATTR_OF(clk_buf_all_ctrl),
	__ATTR_OF(clk_buf_ctrl),
	__ATTR_OF(clk_buf_pmic),
	__ATTR_OF(clk_buf_pmif),
	__ATTR_OF(clk_buf_debug),
	__ATTR_OF(clk_buf_bblpm),
	__ATTR_OF(clk_buf_capid),
	__ATTR_OF(clk_buf_heater),
	__ATTR_OF(rc_cfg_ctl),
	__ATTR_OF(rc_sta_reg),
	__ATTR_OF(rc_trace_ctl),
	__ATTR_OF(rc_subsys_ctl),
	__ATTR_OF(rc_subsys_sta),
	/* must */
	NULL,
};

static struct attribute_group clk_buf_attr_group = {
	.name = "clk_buf",
	.attrs = clk_buf_attrs,
};

static int clkbuf_sysfs_init(void)
{
	int ret = 0;

	/* create /sys/kernel/clk_buf/xxx */
	ret = sysfs_create_group(kernel_kobj, &clk_buf_attr_group);
	if (ret)
		pr_notice("FAILED TO CREATE /sys/kernel/clk_buf (%d)\n", ret);

	return ret;
}
#endif /* IS_ENABLED(CONFIG_PM) */

int clk_buf_register_xo_ctl_op(const char *xo_name,
		struct xo_buf_ctl_t *xo_buf_ctl)
{
	int i;

	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++)
		if (!strcmp(xo_name, clkbuf_dcxo_get_xo_name(i)))
			return clkbuf_dcxo_register_op(i, xo_buf_ctl);

	pr_notice("xo_name: %s not found\n", xo_name);
	return -EINVAL;
}

static int clkbuf_dts_init(struct platform_device *pdev)
{
	struct device_node *node;

	node = of_parse_phandle(pdev->dev.of_node, CLKBUF_CTL_PHANDLE_NAME, 0);
	if (!node) {
		pr_notice("find clock_buffer_ctrl node failed\n");
		return -EFIND_DTS_ERR;
	}

	if (!of_property_read_bool(node, ENABLE_PROP_NAME)) {
		clkbuf_ctl.enable = false;
		pr_notice("clkbuf_ctl hw does not enabled\n");
		return -EHW_NOT_SUPPORT;
	}

	clkbuf_ctl.enable = true;

	of_node_put(node);

	return 0;
}

static int clkbuf_post_init(void)
{
	int ret = 0;

	ret = clkbuf_dcxo_post_init();
	if (ret) {
		pr_notice("dcxo post init failed\n");
		return ret;
	}

	ret = clkbuf_pmif_post_init();
	if (ret) {
		pr_notice("clkbuf pmif init failed: %d\n", ret);
		return ret;
	}

#if IS_ENABLED(CONFIG_PM)
	ret = clkbuf_sysfs_init();
	if (ret) {
		pr_notice("clkbuf sysfs init failed\n");
		return ret;
	}
#endif /* IS_ENABLED(CONFIG_PM) */

	clkbuf_ctl.init_done = true;

	return ret;
}

static int clkbuf_init(struct platform_device *pdev)
{
	int ret = 0;

	if (clkbuf_ctl.init_done) {
		pr_notice("clkbuf_ctl hw already init\n");
		return -EHW_ALREADY_INIT;
	}

	ret = clkbuf_dts_init(pdev);
	if (ret) {
		pr_notice("clkbuf_ctl dts init failed with err: %d\n", ret);
		return ret;
	}

	ret = clkbuf_dcxo_init(pdev);
	if (ret) {
		pr_notice("clkbuf dcxo init failed with err: %d\n", ret);
		return ret;
	}

	/* Register RC driver */
	ret = srclken_rc_init();
	if (ret) {
		pr_notice("srclken_rc init failed\n");
		return ret;
	}

	ret = clkbuf_pmif_hw_init(pdev);
	if (ret) {
		pr_notice("clkbuf pmif init failed: %d\n", ret);
		return ret;
	}

	return ret;
}

static int __clk_buf_dev_pm_dump(void)
{
	char *buf = NULL;
	int ret = 0;
	int len = 0;
	u32 val = 0;
	u32 en = 0;
	u8 i;

	buf = vmalloc(CLKBUF_STATUS_INFO_SIZE);
	if (!buf)
		return -ENOMEM;

	for (i = 0; i < clkbuf_dcxo_get_xo_num(); i++) {
		if (!clkbuf_dcxo_is_xo_in_use(i))
			continue;
		ret = clkbuf_dcxo_get_xo_en(i, &en);
		if (ret) {
			pr_notice("get xo en failed: %d\n", ret);
			continue;
		}

		if (en)
			len += snprintf(buf + len, CLKBUF_STATUS_INFO_SIZE - len,
					", %s en: %u", clkbuf_dcxo_get_xo_name(i), en);
		val |= (en << i);
	}
	pr_debug("%s\n", buf+2);
	len = 0;

	len += snprintf(buf + len, CLKBUF_STATUS_INFO_SIZE - len,
		"xo_buf_en: 0x%x ", val);

	if (is_srclken_rc_init_done())
		for (i = 0; i < 8; i++)
			len += srclken_rc_dump_trace(i, buf + len,
					CLKBUF_STATUS_INFO_SIZE - len);

	strreplace(buf, '\n', ' ');

	pr_debug("%s\n", buf);

	vfree(buf);

	return 0;
}

static int clk_buf_dev_pm_suspend(struct device *dev)
{
	return __clk_buf_dev_pm_dump();
}

static int clk_buf_dev_pm_resume(struct device *dev)
{
	return __clk_buf_dev_pm_dump();
}

static const struct dev_pm_ops clk_buf_suspend_ops = {
	.suspend_noirq = clk_buf_dev_pm_suspend,
	.resume_noirq = clk_buf_dev_pm_resume,
};

static int mtk_clkbuf_ctl_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = clkbuf_init(pdev);
	if (ret) {
		pr_notice("clkbuf hw init failed with err: %d\n", ret);
		return ret;
	}

	ret = clkbuf_post_init();
	if (ret)
		pr_notice("clkbuf post init failed: %d\n", ret);

	pr_notice("clkbuf probe done\n");

	return ret;
}

static const struct of_device_id mtk_clkbuf_of_match[] = {
	{
		.compatible = "mediatek,clock_buffer",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mtk_clkbuf_of_match);

static struct platform_driver mtk_clkbuf_driver = {
	.driver = {
		.name = "mtk-clock-buffer",
		.of_match_table = of_match_ptr(mtk_clkbuf_of_match),
		.pm = &clk_buf_suspend_ops,
	},
	.probe = mtk_clkbuf_ctl_probe,
};

static int __init mtk_clkbuf_init(void)
{
	pr_notice("clkbuf init start\n");
	return platform_driver_register(&mtk_clkbuf_driver);
}

static void __exit mtk_clkbuf_exit(void)
{
	/* Unregister RC driver */
	srclken_rc_exit();

	platform_driver_unregister(&mtk_clkbuf_driver);
}

module_init(mtk_clkbuf_init);
module_exit(mtk_clkbuf_exit);
MODULE_AUTHOR("Ren-Ting Wang <ren-ting.wang@mediatek.com");
MODULE_DESCRIPTION("SOC Driver for MediaTek Clock Buffer");
MODULE_LICENSE("GPL v2");
