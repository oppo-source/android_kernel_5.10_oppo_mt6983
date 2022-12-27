// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/syscalls.h>
#include "mt-plat/mtk_thermal_monitor.h"
#include "mt-plat/mtk_ccci_common.h"
#include <linux/uidgid.h>
#include <mtk_cooler_setting.h>
#include <linux/debugfs.h>

/* extern unsigned long ccci_get_md_boot_count(int md_id); */

#if !defined(FEATURE_MUTT_INTERFACE_VER)

#define MUTT_ACTIVATED_OFFSET		8
#define MUTT_SUSPEND_OFFSET		16

#define MUTT_ACTIVATED_FILTER		0x0000FF00
#define MUTT_SUSPEND_FILTER		0x00FF0000


#if FEATURE_MUTT_V2
/*
 * No UL data(except IMS): active = 1; suspend = 255; bit0 in reserved =0;
 * No UL data(no IMS): active = 1; suspend = 255; bit0 in reserved =1;
 */
#define BIT_MD_CL_NO_IMS	0x01000000
#define MD_CL_NO_UL_DATA	0x00FF0101
#endif




#define MUTT_ENABLE_IMS_ENABLE		0x00000001

#else /*FEATURE_MUTT_INTERFACE_VER*/

/*enum mapping must be align with MD site*/
enum tmc_ctrl_cmd_enum {
	TMC_CTRL_CMD_THROTTLING = 0,
	TMC_CTRL_CMD_CA_CTRL,
	TMC_CTRL_CMD_PA_CTRL,
	TMC_CTRL_CMD_COOLER_LV,
};

enum tmc_throt_ctrl_enum {
	TMC_THROT_ENABLE_IMS_ENABLE = 0,
	TMC_THROT_ENABLE_IMS_DISABLE,
	TMC_THROT_DISABLE,
};

enum tmc_ca_ctrl_enum {
	TMC_CA_ON = 0, /* leave thermal control*/
	TMC_CA_OFF,
};

enum tmc_pa_ctrl_enum {
	TMC_PA_ALL_ON = 0, /* leave thermal control*/
	TMC_PA_OFF_1PA,
};

enum tmc_cooler_lv_ctrl_enum {
	TMC_COOLER_LV_ENABLE = 0,
	TMC_COOLER_LV_DISABLE
};


enum tmc_cooler_lv_enum {
	TMC_COOLER_LV0 = 0,
	TMC_COOLER_LV1,
	TMC_COOLER_LV2,
	TMC_COOLER_LV3,
	TMC_COOLER_LV4,
	TMC_COOLER_LV5,
	TMC_COOLER_LV6,
	TMC_COOLER_LV7,
	TMC_COOLER_LV8,
};


#define MUTT_ACTIVATED_OFFSET		16
#define MUTT_SUSPEND_OFFSET		24

#define MUTT_ACTIVATED_FILTER		0x00FF0000
#define MUTT_SUSPEND_FILTER		0xFF000000



#define MUTT_ENABLE_IMS_ENABLE	\
	(TMC_CTRL_CMD_THROTTLING | TMC_THROT_ENABLE_IMS_ENABLE << 8)
#define MUTT_ENABLE_IMS_DISABLE	\
	(TMC_CTRL_CMD_THROTTLING | TMC_THROT_ENABLE_IMS_DISABLE << 8)
#define TMC_CA_CTRL_CA_ON \
	(TMC_CTRL_CMD_CA_CTRL | TMC_CA_ON << 8)
#define TMC_CA_CTRL_CA_OFF \
	(TMC_CTRL_CMD_CA_CTRL | TMC_CA_OFF << 8)
#define TMC_PA_CTRL_PA_ALL_ON \
	(TMC_CTRL_CMD_PA_CTRL | TMC_PA_ALL_ON << 8)
#define TMC_PA_CTRL_PA_OFF_1PA \
	(TMC_CTRL_CMD_PA_CTRL | TMC_PA_OFF_1PA << 8)
#define TMC_THROTTLING_THROT_DISABLE \
	(TMC_CTRL_CMD_THROTTLING | TMC_THROT_DISABLE << 8)
#define TMC_LEVEL_CTRL_COOLER_LV_ENABLE \
	(TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV_ENABLE << 8)
#define TMC_LEVEL_CTRL_COOLER_LV_DISABLE \
	(TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV_DISABLE << 8)


#define TMC_COOLER_LV_CTRL00 (TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV0 << 16)
#define TMC_COOLER_LV_CTRL01 (TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV1 << 16)
#define TMC_COOLER_LV_CTRL02 (TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV2 << 16)
#define TMC_COOLER_LV_CTRL03 (TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV3 << 16)
#define TMC_COOLER_LV_CTRL04 (TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV4 << 16)
#define TMC_COOLER_LV_CTRL05 (TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV5 << 16)
#define TMC_COOLER_LV_CTRL06 (TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV6 << 16)
#define TMC_COOLER_LV_CTRL07 (TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV7 << 16)
#define TMC_COOLER_LV_CTRL08 (TMC_CTRL_CMD_COOLER_LV | TMC_COOLER_LV8 << 16)

#if FEATURE_MUTT_V2
/*
 * No UL data(except IMS): active = 1; suspend = 255; bit0 in reserved =0;
 * No UL data(no IMS): active = 1; suspend = 255; bit0 in reserved =1;
 */
#define BIT_MD_CL_NO_IMS MUTT_ENABLE_IMS_DISABLE /*IMS disable*/
#define MD_CL_NO_UL_DATA (0xFF010000 | MUTT_ENABLE_IMS_ENABLE) /*IMS only*/
#endif
#endif/*FEATURE_MUTT_INTERFACE_VER*/



#if FEATURE_THERMAL_DIAG
/* signal */
/* #define MAX_LEN	256 */
static unsigned int tmd_pid;
static unsigned int tmd_input_pid;
static struct task_struct *ptmd_task;
#endif

#if FEATURE_MUTT_V2
/* signal */
#define MAX_LEN	128
static unsigned int tm_pid;
static unsigned int tm_input_pid;
static struct task_struct *pg_task;


/* mdoff cooler */
static struct thermal_cooling_device *cl_dev_mdoff;
static unsigned int cl_dev_mdoff_state;

/* noIMS cooler */
static struct thermal_cooling_device *cl_dev_noIMS;
static unsigned int cl_dev_noIMS_state;
#endif

#if FEATURE_ADAPTIVE_MUTT
/* adp mutt cooler */
static struct thermal_cooling_device *cl_dev_adp_mutt;
static unsigned int cl_dev_adp_mutt_state;
static unsigned int cl_dev_adp_mutt_limit;

#if defined(FEATURE_MUTT_INTERFACE_VER)
static unsigned int cl_dev_adp_mutt_pa_limit;
static unsigned int cl_dev_adp_mutt_ca_limit;
#endif

static int curr_adp_mutt_level = -1;

#define init_MD_tput_limit 0

/* Interal usage */
/* parameter from adb shell */
/* this is in milli degree C */
static int MD_target_t = 58000;
static int MD_TARGET_T_HIGH;
static int MD_TARGET_T_LOW;
static int t_stable_range = 1000;

/* initial value: assume 1 degreeC for temp. <=> 20% for MD_tput_limit(0~100) */
static int tt_MD_high = 50;
static int tt_MD_low = 50;
static int triggered;
#endif

unsigned long __attribute__ ((weak))
ccci_get_md_boot_count(int md_id)
{
	pr_notice("E_WF: %s doesn't exist\n", __func__);
	return 0;
}

int __attribute__ ((weak))
exec_ccci_kern_func_by_md_id(
int md_id, unsigned int id, char *buf, unsigned int len)
{
	pr_notice("E_WF: %s doesn't exist\n", __func__);
	return -316;
}

#define mtk_cooler_mutt_dprintk_always(fmt, args...) \
pr_debug("[Thermal/TC/mutt]" fmt, ##args)

#define mtk_cooler_mutt_dprintk(fmt, args...) \
do { \
	if (cl_mutt_klog_on == 1) \
		pr_debug("[Thermal/TC/mutt]" fmt, ##args); \
} while (0)

/* State of "MD off & noIMS" are not included. */
#define MAX_NUM_INSTANCE_MTK_COOLER_MUTT  4

#define MTK_CL_MUTT_GET_LIMIT(limit, state) \
{ (limit) = (short) (((unsigned long) (state))>>16); }

#define MTK_CL_MUTT_SET_LIMIT(limit, state) \
{ state = ((((unsigned long) (state))&0xFFFF) | ((short) limit<<16)); }

#define MTK_CL_MUTT_GET_CURR_STATE(curr_state, state) \
{ curr_state = (((unsigned long) (state))&0xFFFF); }

#define MTK_CL_MUTT_SET_CURR_STATE(curr_state, state) \
do { \
	if (curr_state == 0) \
		state &= ~0x1; \
	else \
		state |= 0x1; \
} while (0)


static kuid_t uid = KUIDT_INIT(0);
static kgid_t gid = KGIDT_INIT(1000);

static int cl_mutt_klog_on;
static struct thermal_cooling_device
			*cl_mutt_dev[MAX_NUM_INSTANCE_MTK_COOLER_MUTT] = { 0 };

static unsigned int cl_mutt_param[MAX_NUM_INSTANCE_MTK_COOLER_MUTT] = { 0 };

#if defined(FEATURE_MUTT_INTERFACE_VER)
/*default setting*/
static unsigned int cl_mutt_ca_param[MAX_NUM_INSTANCE_MTK_COOLER_MUTT] = {
					TMC_CA_CTRL_CA_ON,
					TMC_CA_CTRL_CA_ON,
					TMC_CA_CTRL_CA_ON,
					TMC_CA_CTRL_CA_OFF};

static unsigned int cl_mutt_pa_param[MAX_NUM_INSTANCE_MTK_COOLER_MUTT] = {
					TMC_PA_CTRL_PA_ALL_ON,
					TMC_PA_CTRL_PA_ALL_ON,
					TMC_PA_CTRL_PA_ALL_ON,
					TMC_PA_CTRL_PA_OFF_1PA};

/*No IMS cooler, parameter order in array: [CA PA]*/
static unsigned int cl_mutt_No_IMS_ca_pa_param[2] = {
					TMC_CA_CTRL_CA_OFF,
					TMC_PA_CTRL_PA_OFF_1PA};


static unsigned int cl_mutt_cur_pa_limit;
static unsigned int cl_mutt_cur_ca_limit;
#endif

static unsigned long cl_mutt_state[MAX_NUM_INSTANCE_MTK_COOLER_MUTT] = { 0 };

static unsigned int cl_mutt_cur_limit;

static unsigned long last_md_boot_cnt;

#if FEATURE_THERMAL_DIAG
/*
 * use "si_code" for Action identify
 * for tmd_pid (/system/bin/thermald)
 */
enum {
/*	TMD_Alert_ShutDown = 1, */
	TMD_Alert_ULdataBack = 2,
	TMD_Alert_NOULdata = 3
};

static int clmutt_send_tmd_signal(int level)
{
	int ret = 0;
	static int warning_state = TMD_Alert_ULdataBack;

	if (warning_state == level)
		return ret;

	if (tmd_input_pid == 0) {
		mtk_cooler_mutt_dprintk("%s pid is empty\n", __func__);
		ret = -1;
	}

	mtk_cooler_mutt_dprintk_always(" %s pid is %d, %d; MD_Alert: %d\n",
				__func__, tmd_pid, tmd_input_pid, level);

	if (ret == 0 && tmd_input_pid != tmd_pid) {
		tmd_pid = tmd_input_pid;

		if (ptmd_task != NULL)
			put_task_struct(ptmd_task);
		ptmd_task = get_pid_task(find_vpid(tmd_pid), PIDTYPE_PID);
	}

	if (ret == 0 && ptmd_task) {
		struct kernel_siginfo info;

		clear_siginfo(&info);
		info.si_signo = SIGIO;
		info.si_errno = 0;
		info.si_code = level;
		info.si_addr = NULL;
		ret = send_sig_info(SIGIO, &info, ptmd_task);
	}

	if (ret != 0)
		mtk_cooler_mutt_dprintk_always(" %s ret=%d\n", __func__, ret);
	else {
		if (TMD_Alert_ULdataBack == level)
			warning_state = TMD_Alert_ULdataBack;
		else if (TMD_Alert_NOULdata == level)
			warning_state = TMD_Alert_NOULdata;
	}

	return ret;
}

static ssize_t clmutt_tmd_pid_write(
struct file *filp, const char __user *buf, size_t count, loff_t *data)
{
	int ret = 0;
	char tmp[MAX_LEN] = { 0 };
	int len = 0;

	len = (count < (MAX_LEN - 1)) ? count : (MAX_LEN - 1);
	/* write data to the buffer */
	if (copy_from_user(tmp, buf, len))
		return -EFAULT;

	ret = kstrtouint(tmp, 10, &tmd_input_pid);
	if (ret)
		WARN_ON_ONCE(1);

	mtk_cooler_mutt_dprintk("%s %s = %d\n", __func__, tmp, tmd_input_pid);

	return len;
}

static int clmutt_tmd_pid_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", tmd_input_pid);
	mtk_cooler_mutt_dprintk("%s %d\n", __func__, tmd_input_pid);

	return 0;
}

static int clmutt_tmd_pid_open(struct inode *inode, struct file *file)
{
	return single_open(file, clmutt_tmd_pid_read, PDE_DATA(inode));
}

static const struct proc_ops clmutt_tmd_pid_fops = {
	.proc_open = clmutt_tmd_pid_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_write = clmutt_tmd_pid_write,
	.proc_release = single_release,
};
#endif

#if FEATURE_MUTT_V2
/*
 * use "si_errno" for client identify
 * for tm_pid (/system/bin/thermal)
 */
enum {
	/*	TM_CLIENT_clwmt = 0,
	 *	TM_CLIENT_mdulthro =1,
	 *	TM_CLIENT_mddlthro =2,
	 */
	TM_CLIENT_clmutt = 3
};
static int clmutt_send_tm_signal(int level)
{
	int ret = 0, j = 0;

	if (cl_dev_mdoff_state == level)
		return ret;

	if (tm_input_pid == 0) {
		mtk_cooler_mutt_dprintk("[%s] pid is empty\n", __func__);
		ret = -1;
	}

	mtk_cooler_mutt_dprintk_always("[%s] pid is %d, %d; MD off: %d\n",
					__func__, tm_pid, tm_input_pid, level);

	if (ret == 0 && tm_input_pid != tm_pid) {
		tm_pid = tm_input_pid;

		if (pg_task != NULL)
			put_task_struct(pg_task);
		pg_task = get_pid_task(find_vpid(tm_pid), PIDTYPE_PID);
	}

	if (ret == 0 && pg_task) {
		struct kernel_siginfo info;

		clear_siginfo(&info);
		info.si_signo = SIGIO;
		info.si_errno = TM_CLIENT_clmutt;
		info.si_code = level; /* Toggle MD ON: 0 OFF: 1*/
		info.si_addr = NULL;
		ret = send_sig_info(SIGIO, &info, pg_task);
	}

	if (ret != 0)
		mtk_cooler_mutt_dprintk_always("[%s] ret=%d\n", __func__, ret);
	else {
		if (level == 1) {
			cl_dev_mdoff_state = level;
			cl_dev_noIMS_state = 0;
			cl_mutt_cur_limit = 0;

			for (; j < MAX_NUM_INSTANCE_MTK_COOLER_MUTT; j++)
				MTK_CL_MUTT_SET_CURR_STATE(0, cl_mutt_state[j]);
		} else
			cl_dev_mdoff_state = 0;
	}
	return ret;
}

static ssize_t clmutt_tm_pid_write(
struct file *filp, const char __user *buf, size_t count, loff_t *data)
{
	int ret = 0;
	char tmp[MAX_LEN] = {0};
	int len = 0;

	len = (count < (MAX_LEN - 1)) ? count : (MAX_LEN - 1);
	/* write data to the buffer */
	if (copy_from_user(tmp, buf, len))
		return -EFAULT;

	ret = kstrtouint(tmp, 10, &tm_input_pid);
	if (ret)
		WARN_ON_ONCE(1);

	mtk_cooler_mutt_dprintk("[%s] %s = %d\n", __func__, tmp, tm_input_pid);

	return len;
}

static int clmutt_tm_pid_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", tm_input_pid);

	mtk_cooler_mutt_dprintk("[%s] %d\n", __func__, tm_input_pid);

	return 0;
}

static int clmutt_tm_pid_open(struct inode *inode, struct file *file)
{
	return single_open(file, clmutt_tm_pid_read, PDE_DATA(inode));
}

static const struct proc_ops clmutt_tm_pid_fops = {
	.proc_open = clmutt_tm_pid_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_write = clmutt_tm_pid_write,
	.proc_release = single_release,
};

/*
 * cooling device callback functions (mtk_cl_mdoff_ops)
 * 1 : True and 0 : False
 */
static int mtk_cl_mdoff_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	*state = 1;
	mtk_cooler_mutt_dprintk("%s() %s %lu\n", __func__,
							cdev->type, *state);

	return 0;
}

static int mtk_cl_mdoff_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	*state = cl_dev_mdoff_state;
	mtk_cooler_mutt_dprintk(
			"[%s] %s %lu (0: md on;  1: md off)\n", __func__,
			cdev->type, *state);
	return 0;
}

static int mtk_cl_mdoff_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
	if ((state >= 0) && (state <= 1))
		mtk_cooler_mutt_dprintk(
			"[%s] %s %lu (0: md on;  1: md off)\n", __func__,
			cdev->type, state);
	else {
		mtk_cooler_mutt_dprintk(
		"[%s]: Invalid input (0:md on;	 1: md off)\n", __func__);

		return 0;
	}

	clmutt_send_tm_signal(state);

	return 0;
}

static struct thermal_cooling_device_ops mtk_cl_mdoff_ops = {
	.get_max_state = mtk_cl_mdoff_get_max_state,
	.get_cur_state = mtk_cl_mdoff_get_cur_state,
	.set_cur_state = mtk_cl_mdoff_set_cur_state,
};

static void mtk_cl_mutt_set_onIMS(int level)
{
	int ret = 0;
	unsigned int cl_mutt_param_noIMS = 0;
#if defined(FEATURE_MUTT_INTERFACE_VER)
	unsigned int cl_mutt_param_noIMS_pa = 0;
	unsigned int cl_mutt_param_noIMS_ca = 0;
	int ret_pa = 0, ret_ca = 0;
#endif


	if (cl_dev_noIMS_state == level)
		return;

	if (level) {
		/*level=1 => No IMS*/
		cl_mutt_param_noIMS = (MD_CL_NO_UL_DATA | BIT_MD_CL_NO_IMS);
#if defined(FEATURE_MUTT_INTERFACE_VER)
		cl_mutt_param_noIMS_pa = cl_mutt_No_IMS_ca_pa_param[1];
		cl_mutt_param_noIMS_ca = cl_mutt_No_IMS_ca_pa_param[0];
#endif
	} else {
		/*level=0 => IMS Only*/
		cl_mutt_param_noIMS = MD_CL_NO_UL_DATA;
#if defined(FEATURE_MUTT_INTERFACE_VER)
		/*Restore to last PA status*/
		cl_mutt_param_noIMS_pa =
			cl_mutt_pa_param[MAX_NUM_INSTANCE_MTK_COOLER_MUTT - 1];
		cl_mutt_param_noIMS_ca =
			cl_mutt_ca_param[MAX_NUM_INSTANCE_MTK_COOLER_MUTT - 1];
#endif
	}
	if (cl_mutt_param_noIMS != cl_mutt_cur_limit) {
		cl_mutt_cur_limit = cl_mutt_param_noIMS;
#if defined(FEATURE_MUTT_INTERFACE_VER)
		cl_mutt_cur_ca_limit = cl_mutt_param_noIMS_ca;
		cl_mutt_cur_pa_limit = cl_mutt_param_noIMS_pa;
#endif
		last_md_boot_cnt = ccci_get_md_boot_count(MD_SYS1);
		ret = exec_ccci_kern_func_by_md_id(MD_SYS1,
			ID_THROTTLING_CFG,
			(char *) &cl_mutt_cur_limit, 4);
#if defined(FEATURE_MUTT_INTERFACE_VER)
		ret_pa = exec_ccci_kern_func_by_md_id(MD_SYS1,
			ID_THROTTLING_CFG,
			(char *) &cl_mutt_cur_pa_limit, 4);
		ret_ca = exec_ccci_kern_func_by_md_id(MD_SYS1,
			ID_THROTTLING_CFG,
			(char *) &cl_mutt_cur_ca_limit, 4);
#endif
#if defined(FEATURE_MUTT_INTERFACE_VER)
		mtk_cooler_mutt_dprintk_always(
		"[%s]r %d, rpa %d, rca %d,p %x, p_pa %x, p_ca %x, bcnt %lul\n",
		__func__, ret, ret_pa, ret_ca,
		cl_mutt_cur_limit,
		cl_mutt_cur_pa_limit,
		cl_mutt_cur_ca_limit,
		last_md_boot_cnt);
#else
		mtk_cooler_mutt_dprintk_always(
			"[%s] ret %d param %x bcnt %lul\n",
			__func__, ret, cl_mutt_cur_limit,
			last_md_boot_cnt);
#endif
	} else if (cl_mutt_param_noIMS != 0) {
		unsigned long cur_md_bcnt = ccci_get_md_boot_count(MD_SYS1);

		if (last_md_boot_cnt != cur_md_bcnt) {
			last_md_boot_cnt = cur_md_bcnt;
			ret = exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *) &cl_mutt_cur_limit, 4);
#if defined(FEATURE_MUTT_INTERFACE_VER)
			ret_pa = exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *) &cl_mutt_cur_pa_limit, 4);
			ret_ca = exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *) &cl_mutt_cur_ca_limit, 4);
#endif
#if defined(FEATURE_MUTT_INTERFACE_VER)
		mtk_cooler_mutt_dprintk_always(
		"[%s]mdrb r %d,rpa %d,rca %d,p %x,p_pa %x,p_ca %x, bcnt %lul\n",
		__func__, ret, ret_pa, ret_ca,
		cl_mutt_cur_limit,
		cl_mutt_cur_pa_limit,
		cl_mutt_cur_ca_limit,
		last_md_boot_cnt);
#else
		mtk_cooler_mutt_dprintk_always(
				"[%s] mdrb ret %d param %x bcnt %lul\n",
				__func__, ret, cl_mutt_cur_limit,
				last_md_boot_cnt);
#endif
		}
	}
#if defined(FEATURE_MUTT_INTERFACE_VER)
	if ((ret != 0) || (ret_pa != 0) || (ret_ca != 0))
		mtk_cooler_mutt_dprintk_always(
			"[%s] ret=%d ret_pa=%d ret_ca=%d\n", __func__,
			ret, ret_pa, ret_ca);
#else
	if (ret != 0)
		mtk_cooler_mutt_dprintk_always("[%s] ret=%d\n", __func__, ret);
#endif
	else {
		if (level == 1)
			cl_dev_noIMS_state = level;
		else
			cl_dev_noIMS_state = 0;
	}

}

/*
 * cooling device callback functions (mtk_cl_noIMS_ops)
 * 1 : True and 0 : False
 */
static int mtk_cl_noIMS_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	*state = 1;
	mtk_cooler_mutt_dprintk("%s() %s %lu\n", __func__,
							cdev->type, *state);

	return 0;
}

static int mtk_cl_noIMS_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	*state = cl_dev_noIMS_state;
	mtk_cooler_mutt_dprintk(
			"%s() %s %lu (0: md IMS OK;  1: md no IMS)\n", __func__,
			cdev->type, *state);

	return 0;
}

static int mtk_cl_noIMS_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{

	if (cl_dev_mdoff_state == 1) {
		mtk_cooler_mutt_dprintk(
			"%s():  MD STILL OFF!!\n", __func__);

		return 0;
	}

	if ((state >= 0) && (state <= 1))
		mtk_cooler_mutt_dprintk(
			"%s() %s %lu (0: md IMS OK;	1: md no IMS)\n",
			__func__,
			cdev->type, state);
	else {
		mtk_cooler_mutt_dprintk(
			"%s(): Invalid input(0: md IMS OK; 1: md no IMS)\n",
			__func__);
		return 0;
	}

	mtk_cl_mutt_set_onIMS(state);

	return 0;
}

static struct thermal_cooling_device_ops mtk_cl_noIMS_ops = {
	.get_max_state = mtk_cl_noIMS_get_max_state,
	.get_cur_state = mtk_cl_noIMS_get_cur_state,
	.set_cur_state = mtk_cl_noIMS_set_cur_state,
};
#endif


static void mtk_cl_mutt_set_mutt_limit(void)
{
	/* TODO: optimize */
	int i = 0, j = 0, ret = 0;
	int min_limit = 255;
	unsigned int min_param = 0;
#if defined(FEATURE_MUTT_INTERFACE_VER)
	unsigned int min_pa_param = 0, min_ca_param = 0;
	int ret_ca = 0, ret_pa = 0;
	/*initial value for unlimit*/
	min_param = TMC_THROTTLING_THROT_DISABLE;
	min_pa_param = TMC_PA_CTRL_PA_ALL_ON;
	min_ca_param = TMC_CA_CTRL_CA_ON;
#endif

	for (; i < MAX_NUM_INSTANCE_MTK_COOLER_MUTT; i++) {
		unsigned long curr_state;

		MTK_CL_MUTT_GET_CURR_STATE(curr_state, cl_mutt_state[i]);

		if (curr_state == 1) {
			unsigned int active;
			unsigned int suspend;
			int limit = 0;

			active =
				(cl_mutt_param[i] & MUTT_ACTIVATED_FILTER) >>
					MUTT_ACTIVATED_OFFSET;
			suspend =
				(cl_mutt_param[i] & MUTT_SUSPEND_FILTER) >>
					MUTT_SUSPEND_OFFSET;

			mtk_cooler_mutt_dprintk_always(
			"[%s]active %d, suspend %d\n", __func__,
			active, suspend);

			/* a cooler with 0 active or 0 suspend is not allowed */
			if (active == 0 || suspend == 0)
				goto err_unreg;

			/* compare the active/suspend ratio */
			if (active >= suspend)
				limit = active / suspend;
			else
				limit = (0 - suspend) / active;

			if (limit <= min_limit) {
				min_limit = limit;
				min_param = cl_mutt_param[i];
			}
		}
	}

#if FEATURE_ADAPTIVE_MUTT
		if (cl_dev_adp_mutt_limit > min_param) {
			min_param = cl_dev_adp_mutt_limit;
#if defined(FEATURE_MUTT_INTERFACE_VER)
			min_pa_param = cl_dev_adp_mutt_pa_limit;
			min_ca_param = cl_dev_adp_mutt_ca_limit;
#endif
		}
#endif


	if (min_param != cl_mutt_cur_limit) {
		cl_mutt_cur_limit = min_param;
#if defined(FEATURE_MUTT_INTERFACE_VER)
		cl_mutt_cur_pa_limit = min_pa_param;
		cl_mutt_cur_ca_limit = min_ca_param;
#endif


		last_md_boot_cnt = ccci_get_md_boot_count(MD_SYS1);

		ret = exec_ccci_kern_func_by_md_id(MD_SYS1,
			ID_THROTTLING_CFG,
			(char *)&cl_mutt_cur_limit, 4);

#if defined(FEATURE_MUTT_INTERFACE_VER)
		ret_pa = exec_ccci_kern_func_by_md_id(MD_SYS1,
			ID_THROTTLING_CFG,
			(char *)&cl_mutt_cur_pa_limit, 4);

		ret_ca = exec_ccci_kern_func_by_md_id(MD_SYS1,
			ID_THROTTLING_CFG,
			(char *)&cl_mutt_cur_ca_limit, 4);
#endif

		mtk_cooler_mutt_dprintk_always(
			"[%s] ret %d param 0x%08x bcnt %lul\n", __func__,
			ret, cl_mutt_cur_limit, last_md_boot_cnt);
#if defined(FEATURE_MUTT_INTERFACE_VER)
		mtk_cooler_mutt_dprintk_always(
			"[%s] ret_pa %d param 0x%08x bcnt %lul\n", __func__,
			ret_pa, cl_mutt_cur_pa_limit, last_md_boot_cnt);
		mtk_cooler_mutt_dprintk_always(
			"[%s] ret_ca %d param 0x%08x bcnt %lul\n", __func__,
			ret_ca, cl_mutt_cur_ca_limit, last_md_boot_cnt);
#endif

#if defined(FEATURE_MUTT_INTERFACE_VER)
	} else if (min_param != TMC_THROTTLING_THROT_DISABLE) {
#else
	} else if (min_param != 0) {
#endif
		unsigned long cur_md_bcnt = ccci_get_md_boot_count(MD_SYS1);

		if (last_md_boot_cnt != cur_md_bcnt) {
			last_md_boot_cnt = cur_md_bcnt;
			ret =
			exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *)&cl_mutt_cur_limit, 4);
#if defined(FEATURE_MUTT_INTERFACE_VER)
			ret_pa =
			exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *)&cl_mutt_cur_pa_limit, 4);
			ret_ca =
			exec_ccci_kern_func_by_md_id(MD_SYS1,
			ID_THROTTLING_CFG,
			(char *)&cl_mutt_cur_ca_limit, 4);
#endif
			mtk_cooler_mutt_dprintk_always(
				"[%s] mdrb ret %d param 0x%08x bcnt %lul\n",
				__func__, ret, cl_mutt_cur_limit,
				last_md_boot_cnt);
#if defined(FEATURE_MUTT_INTERFACE_VER)
			mtk_cooler_mutt_dprintk_always(
				"[%s] mdrb ret_pa %d param 0x%08x bcnt %lul\n",
				__func__, ret_pa, cl_mutt_cur_pa_limit,
				last_md_boot_cnt);
			mtk_cooler_mutt_dprintk_always(
				"[%s] mdrb ret_ca %d param 0x%08x bcnt %lul\n",
				__func__, ret_ca, cl_mutt_cur_ca_limit,
				last_md_boot_cnt);
#endif
		}
	} else
		return;
#if defined(FEATURE_MUTT_INTERFACE_VER)
	if ((ret != 0) || (ret_pa != 0) || (ret_ca != 0)) {
		cl_mutt_cur_limit = TMC_THROTTLING_THROT_DISABLE;
#else
	if (ret != 0) {
		cl_mutt_cur_limit = 0;
#endif
		for (; j < MAX_NUM_INSTANCE_MTK_COOLER_MUTT; j++) {
			MTK_CL_MUTT_SET_CURR_STATE(0, cl_mutt_state[j]);
			mtk_cooler_mutt_dprintk_always(
			"[%s]cl_mutt_state[%d] %ld\n",
			__func__, j, cl_mutt_state[j]);

		}
		mtk_cooler_mutt_dprintk_always("[%s] ret=%d\n", __func__, ret);
	}
#if FEATURE_THERMAL_DIAG
	else {/*IMS only*/
		if (cl_mutt_cur_limit == MD_CL_NO_UL_DATA)
			clmutt_send_tmd_signal(TMD_Alert_NOULdata);
		else
			clmutt_send_tmd_signal(TMD_Alert_ULdataBack);
	}
#endif

err_unreg:
	return;

}

static int mtk_cl_mutt_get_max_state(
struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = 1;
	mtk_cooler_mutt_dprintk("%s() %s %lu\n", __func__,
							cdev->type, *state);

	return 0;
}

static int mtk_cl_mutt_get_cur_state(
struct thermal_cooling_device *cdev, unsigned long *state)
{
	MTK_CL_MUTT_GET_CURR_STATE(*state, *((unsigned long *)cdev->devdata));
	mtk_cooler_mutt_dprintk("%s() %s %lu\n", __func__,
							cdev->type, *state);
	return 0;
}

static int mtk_cl_mutt_set_cur_state(
struct thermal_cooling_device *cdev, unsigned long state)
{
#if FEATURE_MUTT_V2

	if ((cl_dev_mdoff_state == 1) || (cl_dev_noIMS_state == 1)) {
		mtk_cooler_mutt_dprintk(
		"[%s]: MD OFF or noIMS or 1PA OFF!!\n", __func__);
		return 0;
	}
#endif

	mtk_cooler_mutt_dprintk("[%s] %s %lu\n", __func__, cdev->type, state);

	MTK_CL_MUTT_SET_CURR_STATE(state, *((unsigned long *)cdev->devdata));
	mtk_cl_mutt_set_mutt_limit();

	return 0;
}

/* bind fan callbacks to fan device */
static struct thermal_cooling_device_ops mtk_cl_mutt_ops = {
	.get_max_state = mtk_cl_mutt_get_max_state,
	.get_cur_state = mtk_cl_mutt_get_cur_state,
	.set_cur_state = mtk_cl_mutt_set_cur_state,
};

#if FEATURE_ADAPTIVE_MUTT
/* decrease by one level */
static void decrease_mutt_limit(void)
{
	if (curr_adp_mutt_level >= 0)
		curr_adp_mutt_level--;

	if (curr_adp_mutt_level < 0) {
		cl_dev_adp_mutt_limit = 0;
#if defined(FEATURE_MUTT_INTERFACE_VER)
		cl_dev_adp_mutt_pa_limit = TMC_PA_CTRL_PA_ALL_ON;
		cl_dev_adp_mutt_ca_limit = TMC_CA_CTRL_CA_ON;
#endif
	} else if (cl_mutt_param[curr_adp_mutt_level] != 0) {
		cl_dev_adp_mutt_limit = cl_mutt_param[curr_adp_mutt_level];
#if defined(FEATURE_MUTT_INTERFACE_VER)
		cl_dev_adp_mutt_pa_limit =
			cl_mutt_pa_param[curr_adp_mutt_level];
		cl_dev_adp_mutt_ca_limit =
			cl_mutt_ca_param[curr_adp_mutt_level];
#endif
	}
#if defined(FEATURE_MUTT_INTERFACE_VER)
	if (curr_adp_mutt_level >= 0) {
		mtk_cooler_mutt_dprintk("%s : cl_mutt_param[%d]= 0x%x,",
			__func__, curr_adp_mutt_level,
			cl_mutt_param[curr_adp_mutt_level]);
		mtk_cooler_mutt_dprintk("cl_mutt_pa_param[%d]= 0x%x,",
			curr_adp_mutt_level,
			cl_mutt_pa_param[curr_adp_mutt_level]);
		mtk_cooler_mutt_dprintk("cl_mutt_ca_param[%d]= 0x%x\n",
			curr_adp_mutt_level,
			cl_mutt_ca_param[curr_adp_mutt_level]);
	}
#else
	if (curr_adp_mutt_level >= 0) {
		mtk_cooler_mutt_dprintk("%s : cl_mutt_param[%d]= 0x%x\n",
			__func__, curr_adp_mutt_level,
			cl_mutt_param[curr_adp_mutt_level]);
	}
#endif
}

/* increase by one level */
static void increase_mutt_limit(void)
{
	if (curr_adp_mutt_level < (MAX_NUM_INSTANCE_MTK_COOLER_MUTT - 1))
		curr_adp_mutt_level++;

	if (cl_mutt_param[curr_adp_mutt_level] != 0) {
		cl_dev_adp_mutt_limit = cl_mutt_param[curr_adp_mutt_level];
#if defined(FEATURE_MUTT_INTERFACE_VER)
		cl_dev_adp_mutt_pa_limit =
			cl_mutt_pa_param[curr_adp_mutt_level];
		cl_dev_adp_mutt_ca_limit =
			cl_mutt_ca_param[curr_adp_mutt_level];
#endif
	}
#if defined(FEATURE_MUTT_INTERFACE_VER)
	mtk_cooler_mutt_dprintk("%s : cl_mutt_param[%d]= 0x%x\n", __func__,
		curr_adp_mutt_level, cl_mutt_param[curr_adp_mutt_level]);
	mtk_cooler_mutt_dprintk("cl_mutt_pa_param[%d]= 0x%x\n",
		curr_adp_mutt_level, cl_mutt_pa_param[curr_adp_mutt_level]);
	mtk_cooler_mutt_dprintk("cl_mutt_ca_param[%d]= 0x%x\n",
		curr_adp_mutt_level, cl_mutt_ca_param[curr_adp_mutt_level]);
#else
	mtk_cooler_mutt_dprintk("%s : cl_mutt_param[%d]= 0x%x\n", __func__,
		curr_adp_mutt_level, cl_mutt_param[curr_adp_mutt_level]);
#endif
}

static void unlimit_mutt_limit(void)
{
	curr_adp_mutt_level = -1;
	cl_dev_adp_mutt_limit = 0;
#if defined(FEATURE_MUTT_INTERFACE_VER)
	cl_dev_adp_mutt_pa_limit = TMC_PA_CTRL_PA_ALL_ON;
	cl_dev_adp_mutt_ca_limit = TMC_CA_CTRL_CA_ON;
#endif
	mtk_cooler_mutt_dprintk("%s\n", __func__);
}

static int adaptive_tput_limit(long curr_temp)
{
	static int MD_tput_limit = init_MD_tput_limit;

	MD_TARGET_T_HIGH = MD_target_t + t_stable_range;
	MD_TARGET_T_LOW = MD_target_t - t_stable_range;

	/* mtk_cooler_mutt_dprintk(
	 *	"%s : active= %d tirgger= %d curr_temp= %ld\n", __func__,
	 *	cl_dev_adp_mutt_state, triggered, curr_temp);
	 */

	if (cl_dev_adp_mutt_state == 1) {
		int tt_MD = MD_target_t - curr_temp;	/* unit: mC */

		/* Check if it is triggered */
		if (!triggered) {
			if (curr_temp < MD_target_t)
				return 0;

			triggered = 1;
		}

		/* Adjust total power budget if necessary */
		if (curr_temp >= MD_TARGET_T_HIGH)
			MD_tput_limit += (tt_MD / tt_MD_high);
		else if (curr_temp <= MD_TARGET_T_LOW)
			MD_tput_limit += (tt_MD / tt_MD_low);

		/* mtk_cooler_mutt_dprintk(
		 * "%s MD T %d Tc %ld, MD_tput_limit %d\n",
		 *	       __func__, MD_target_t, curr_temp, MD_tput_limit);
		 */

		/* Adjust MUTT level  */
		{
			if (MD_tput_limit >= 100) {
				decrease_mutt_limit();
				MD_tput_limit = 0;
			} else if (MD_tput_limit <= -100) {
				increase_mutt_limit();
				MD_tput_limit = 0;
			}
		}
	} else {
		if (triggered) {
			triggered = 0;
			MD_tput_limit = 0;
			unlimit_mutt_limit();
		}
	}

	return 0;
}

/*
 * cooling device callback functions (mtk_cl_adp_mutt_ops)
 * 1 : True and 0 : False
 */
static int mtk_cl_adp_mutt_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	*state = 1;
	mtk_cooler_mutt_dprintk("[%s] %s %lu\n", __func__, cdev->type, *state);
	return 0;
}

static int mtk_cl_adp_mutt_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	*state = cl_dev_adp_mutt_state;
	mtk_cooler_mutt_dprintk("[%s] %s %lu (0:adp mutt off; 1:adp mutt on)\n",
						__func__, cdev->type, *state);
	return 0;
}

static int mtk_cl_adp_mutt_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
#if FEATURE_MUTT_V2
		if ((cl_dev_mdoff_state == 1) || (cl_dev_noIMS_state == 1)) {
			mtk_cooler_mutt_dprintk("[%s]  MD OFF or noIMS!!\n",
								__func__);

			return 0;
		}
#endif

	if ((state != 0) && (state != 1)) {
		mtk_cooler_mutt_dprintk(
			"[%s] Invalid input(0:adp mutt off; 1:adp mutt on)\n",
			__func__);

		return 0;
	}

	mtk_cooler_mutt_dprintk("[%s] %s %lu (0:adp mutt off; 1:adp mutt on)\n",
						__func__, cdev->type, state);
	cl_dev_adp_mutt_state = state;

	adaptive_tput_limit(mtk_thermal_get_temp(MTK_THERMAL_SENSOR_MD_PA));

	mtk_cl_mutt_set_mutt_limit();

	return 0;
}

static struct thermal_cooling_device_ops mtk_cl_adp_mutt_ops = {
	.get_max_state = mtk_cl_adp_mutt_get_max_state,
	.get_cur_state = mtk_cl_adp_mutt_get_cur_state,
	.set_cur_state = mtk_cl_adp_mutt_set_cur_state,
};

/* =======================
 *#define debugfs_entry(name) \
 *do { \
 *	dentry_f = debugfs_create_u32(#name, S_IWUSR | S_IRUGO, _d, &name); \
 *	if (IS_ERR_OR_NULL(dentry_f)) {	\
 *		pr_notice("Unable to create debugfsfile: " #name "\n"); \
 *		return; \
 *	} \
 *} while (0)
 *
 *static void create_debugfs_entries(void)
 *{
 *	struct dentry *dentry_f;
 *	struct dentry *_d;
 *
 *	_d = debugfs_create_dir("cl_adp_mutt", NULL);
 *	if (IS_ERR_OR_NULL(_d)) {
 *		pr_info("unable to create debugfs directory\n");
 *		return;
 *	}
 *
 *	debugfs_entry(MD_target_t);
 *	debugfs_entry(t_stable_range);
 *	debugfs_entry(tt_MD_high);
 *	debugfs_entry(tt_MD_low);
 *}
 *
 *#undef debugfs_entry
 *==========================
 */
#endif

static int mtk_cooler_mutt_register_ltf(void)
{
	int i;

	mtk_cooler_mutt_dprintk("register ltf\n");

	for (i = MAX_NUM_INSTANCE_MTK_COOLER_MUTT; i-- > 0;) {
		char temp[20] = { 0 };

		sprintf(temp, "mtk-cl-mutt%02d", i);
		/* put mutt state to cooler devdata */
		cl_mutt_dev[i] =
			mtk_thermal_cooling_device_register(temp,
						(void *)&cl_mutt_state[i],
						&mtk_cl_mutt_ops);
	}

#if FEATURE_MUTT_V2
	cl_dev_noIMS = mtk_thermal_cooling_device_register("mtk-cl-noIMS", NULL,
							&mtk_cl_noIMS_ops);

	cl_dev_mdoff = mtk_thermal_cooling_device_register("mtk-cl-mdoff", NULL,
							&mtk_cl_mdoff_ops);
#endif


#if FEATURE_ADAPTIVE_MUTT
	cl_dev_adp_mutt =
		mtk_thermal_cooling_device_register("mtk-cl-adp-mutt",
						NULL, &mtk_cl_adp_mutt_ops);
#endif

	return 0;
}

static void mtk_cooler_mutt_unregister_ltf(void)
{
	int i;

	mtk_cooler_mutt_dprintk("unregister ltf\n");

	for (i = MAX_NUM_INSTANCE_MTK_COOLER_MUTT; i-- > 0;) {
		if (cl_mutt_dev[i]) {
			mtk_thermal_cooling_device_unregister(cl_mutt_dev[i]);
			cl_mutt_dev[i] = NULL;
			cl_mutt_state[i] = 0;
		}
	}
#if FEATURE_MUTT_V2
	if (cl_dev_noIMS) {
		mtk_thermal_cooling_device_unregister(cl_dev_noIMS);
		cl_dev_noIMS = NULL;
	}

	if (cl_dev_mdoff) {
		mtk_thermal_cooling_device_unregister(cl_dev_mdoff);
		cl_dev_mdoff = NULL;
	}
#endif


#if FEATURE_ADAPTIVE_MUTT
	if (cl_dev_adp_mutt) {
		mtk_thermal_cooling_device_unregister(cl_dev_adp_mutt);
		cl_dev_adp_mutt = NULL;
	}
#endif

}

static int _mtk_cl_mutt_proc_read(struct seq_file *m, void *v)
{
    /**
     * The format to print out:
     *  kernel_log <0 or 1>
     *  <mtk-cl-mutt<ID>> <active (ms)> <suspend (ms)> <param> <state>
     *  ..
     */
	int i = 0;

	seq_printf(m, "klog %d\n", cl_mutt_klog_on);
#if FEATURE_MUTT_V2
#if defined(FEATURE_MUTT_INTERFACE_VER)
	seq_printf(m, "curr_limit:mutt 0x%08x, pa 0x%08x, ca 0x%08x, noIMS: %d, mdoff: %d\n",
			cl_mutt_cur_limit,
			cl_mutt_cur_pa_limit,
			cl_mutt_cur_ca_limit,
			cl_dev_noIMS_state,
			cl_dev_mdoff_state);
#else
	seq_printf(m, "curr_limit %x, noIMS: %d, mdoff: %d\n",
			cl_mutt_cur_limit,
			cl_dev_noIMS_state,
			cl_dev_mdoff_state);
#endif
#else
		seq_printf(m, "curr_limit %x\n", cl_mutt_cur_limit);
#endif

		for (; i < MAX_NUM_INSTANCE_MTK_COOLER_MUTT; i++) {
			unsigned int active;
			unsigned int suspend;
			unsigned long curr_state;

		active = (cl_mutt_param[i] & MUTT_ACTIVATED_FILTER) >>
				MUTT_ACTIVATED_OFFSET;
		suspend = (cl_mutt_param[i] & MUTT_SUSPEND_FILTER) >>
				MUTT_SUSPEND_OFFSET;

			MTK_CL_MUTT_GET_CURR_STATE(curr_state,
						cl_mutt_state[i]);
#if defined(FEATURE_MUTT_INTERFACE_VER)
		seq_printf(m, "mtk-cl-mutt%02d %03u : %03u => 0x%08x,pa:0x%08x, ca:0x%08x state %lu\n",
					i, active, suspend,
					cl_mutt_param[i], cl_mutt_pa_param[i],
					cl_mutt_ca_param[i], curr_state);

#else
		seq_printf(m, "mtk-cl-mutt%02d %u %u %x, state %lu\n",
					i, active, suspend,
					cl_mutt_param[i], curr_state);
#endif
		}

#if FEATURE_ADAPTIVE_MUTT
		seq_printf(m, "amutt_target_temp %d\n", MD_target_t);
#endif


	return 0;
}

static ssize_t _mtk_cl_mutt_proc_write(
struct file *filp, const char __user *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[128];
	int klog_on, mutt0_a, mutt0_s, mutt1_a, mutt1_s, mutt2_a,
		mutt2_s, mutt3_a, mutt3_s, amutt_target_temp;
#if defined(FEATURE_MUTT_INTERFACE_VER)
	int mutt0_off1pa = 0, mutt0_off1ca = 0,
		mutt1_off1pa = 0, mutt1_off1ca = 0,
		mutt2_off1pa = 0, mutt2_off1ca = 0,
		mutt3_off1pa = 0, mutt3_off1ca = 0;
#endif

	int scan_count = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);

	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

    /**
     * sscanf format <klog_on> <mtk-cl-mutt00 active (ms)>
     *<mtk-cl-mutt00 suspended (ms)> <mtk-cl-mutt01 active (ms)>
     *<mtk-cl-mutt01 suspended (ms)> <mtk-cl-mutt02 active (ms)>
     *<mtk-cl-mutt02 suspended (ms)>...
     * <klog_on> can only be 0 or 1
     * <mtk-cl-mutt* active/suspended (ms) > can only be positive
     *integer or 0 to denote no limit
     */

	if (data == NULL) {
		mtk_cooler_mutt_dprintk("[%s] null data\n", __func__);
		return -EINVAL;
	}
	/* WARNING: Modify here if
	 * MTK_THERMAL_MONITOR_COOLER_MAX_EXTRA_CONDITIONS
	 * is changed to other than 4
	 */
#if (MAX_NUM_INSTANCE_MTK_COOLER_MUTT == 4)
	/* cl_mutt_param[0] = 0; */
	/* cl_mutt_param[1] = 0; */
	/* cl_mutt_param[2] = 0; */
#if defined(FEATURE_MUTT_INTERFACE_VER)
	scan_count =
	sscanf(desc, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
			&klog_on, &mutt0_a, &mutt0_s,
			&mutt1_a, &mutt1_s, &mutt2_a, &mutt2_s,
			&mutt3_a, &mutt3_s, &amutt_target_temp,
			&mutt0_off1pa, &mutt0_off1ca,
			&mutt1_off1pa, &mutt1_off1ca,
			&mutt2_off1pa, &mutt2_off1ca,
			&mutt3_off1pa, &mutt3_off1ca);
#else
	scan_count = sscanf(desc, "%d %d %d %d %d %d %d %d %d %d",
					&klog_on, &mutt0_a, &mutt0_s,
					&mutt1_a, &mutt1_s, &mutt2_a, &mutt2_s,
					&mutt3_a, &mutt3_s, &amutt_target_temp);

#endif
	if (scan_count >= 1) {
		if (klog_on == 0 || klog_on == 1)
			cl_mutt_klog_on = klog_on;

		if (mutt0_a == 0)
			cl_mutt_param[0] = 0;
		else if (mutt0_a >= 100 && mutt0_a <= 25500
		&& mutt0_s >= 100 && mutt0_s <= 25500)
			cl_mutt_param[0] =
			((mutt0_s / 100) << MUTT_SUSPEND_OFFSET) |
			((mutt0_a / 100) << MUTT_ACTIVATED_OFFSET) |
			MUTT_ENABLE_IMS_ENABLE;

		if (mutt1_a == 0)
			cl_mutt_param[1] = 0;
		else if (mutt1_a >= 100 && mutt1_a <= 25500
		&& mutt1_s >= 100 && mutt1_s <= 25500)
			cl_mutt_param[1] =
			((mutt1_s / 100) << MUTT_SUSPEND_OFFSET) |
			((mutt1_a / 100) << MUTT_ACTIVATED_OFFSET) |
			MUTT_ENABLE_IMS_ENABLE;

		if (mutt2_a == 0)
			cl_mutt_param[2] = 0;
		else if (mutt2_a >= 100 && mutt2_a <= 25500
		&& mutt2_s >= 100 && mutt2_s <= 25500)
			cl_mutt_param[2] =
			((mutt2_s / 100) << MUTT_SUSPEND_OFFSET) |
			((mutt2_a / 100) << MUTT_ACTIVATED_OFFSET) |
			MUTT_ENABLE_IMS_ENABLE;

		if (mutt3_a == 0)
			cl_mutt_param[3] = 0;
		else if (mutt3_a >= 100 && mutt3_a <= 25500
		&& mutt3_s >= 100 && mutt3_s <= 25500)
			cl_mutt_param[3] =
			((mutt3_s / 100) << MUTT_SUSPEND_OFFSET) |
			((mutt3_a / 100) << MUTT_ACTIVATED_OFFSET) |
			MUTT_ENABLE_IMS_ENABLE;


#if FEATURE_ADAPTIVE_MUTT
		if (scan_count > 1+MAX_NUM_INSTANCE_MTK_COOLER_MUTT*2)
			MD_target_t = amutt_target_temp;
#endif

#if defined(FEATURE_MUTT_INTERFACE_VER)
	if (scan_count > (2 + MAX_NUM_INSTANCE_MTK_COOLER_MUTT * 2)) {
		if (mutt0_off1pa == 0)
			cl_mutt_pa_param[0] = TMC_PA_CTRL_PA_ALL_ON;
		else
			cl_mutt_pa_param[0] = TMC_PA_CTRL_PA_OFF_1PA;

		if (mutt0_off1ca == 0)
			cl_mutt_ca_param[0] = TMC_CA_CTRL_CA_ON;
		else
			cl_mutt_ca_param[0] = TMC_CA_CTRL_CA_OFF;

		if (mutt1_off1pa == 0)
			cl_mutt_pa_param[1] = TMC_PA_CTRL_PA_ALL_ON;
		else
			cl_mutt_pa_param[1] = TMC_PA_CTRL_PA_OFF_1PA;

		if (mutt1_off1ca == 0)
			cl_mutt_ca_param[1] = TMC_CA_CTRL_CA_ON;
		else
			cl_mutt_ca_param[1] = TMC_CA_CTRL_CA_OFF;

		if (mutt2_off1pa == 0)
			cl_mutt_pa_param[2] = TMC_PA_CTRL_PA_ALL_ON;
		else
			cl_mutt_pa_param[2] = TMC_PA_CTRL_PA_OFF_1PA;

		if (mutt2_off1ca == 0)
			cl_mutt_ca_param[2] = TMC_CA_CTRL_CA_ON;
		else
			cl_mutt_ca_param[2] = TMC_CA_CTRL_CA_OFF;

		if (mutt3_off1pa == 0)
			cl_mutt_pa_param[3] = TMC_PA_CTRL_PA_ALL_ON;
		else
			cl_mutt_pa_param[3] = TMC_PA_CTRL_PA_OFF_1PA;

		if (mutt3_off1ca == 0)
			cl_mutt_ca_param[3] = TMC_CA_CTRL_CA_ON;
		else
			cl_mutt_ca_param[3] = TMC_CA_CTRL_CA_OFF;
	}
#endif

		return count;
	}
#else
#error	\
"Change correspondent part when changing MAX_NUM_INSTANCE_MTK_COOLER_MUTT!"
#endif
	mtk_cooler_mutt_dprintk("[%s] bad arg\n", __func__);
	return -EINVAL;
}

static int _mtk_cl_mutt_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, _mtk_cl_mutt_proc_read, NULL);
}

static const struct proc_ops cl_mutt_fops = {
	.proc_open = _mtk_cl_mutt_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_write = _mtk_cl_mutt_proc_write,
	.proc_release = single_release,
};

#if defined(FEATURE_MUTT_INTERFACE_VER)
static unsigned int cl_mutt_tuning_param;
static unsigned int cl_mutt_tuning_param_pa;
static unsigned int cl_mutt_tuning_param_ca;
static unsigned int cl_mutt_tuning_param_lv;
static unsigned long last_md_tuning_boot_cnt;

static int _mtk_cl_mutt_tuning_read(struct seq_file *m, void *v)
{
	seq_printf(m,
	"klog_on, mutt_a, mutt_s, mutt_noIMS, mutt_off1pa, mutt_off1ca\n");

	seq_printf(m,
	"klog %d\n", cl_mutt_klog_on);
	seq_printf(m,
	"curr_limit %x, pa %x, ca %x, boot_cnt: %lul, mdoff: %d, level: %d\n",
		cl_mutt_tuning_param,
		cl_mutt_tuning_param_pa,
		cl_mutt_tuning_param_ca,
		last_md_tuning_boot_cnt,
		cl_mutt_tuning_param_lv,
		cl_dev_mdoff_state);

	return 0;
}

unsigned int level_selection(int lv)
{
	unsigned int ctrl_lv = 0;

	switch (lv) {
	case 0:
		ctrl_lv = TMC_COOLER_LV_CTRL00;
		break;
	case 1:
		ctrl_lv = TMC_COOLER_LV_CTRL01;
		break;
	case 2:
		ctrl_lv = TMC_COOLER_LV_CTRL02;
		break;
	case 3:
		ctrl_lv = TMC_COOLER_LV_CTRL03;
		break;
	case 4:
		ctrl_lv = TMC_COOLER_LV_CTRL04;
		break;
	case 5:
		ctrl_lv = TMC_COOLER_LV_CTRL05;
		break;
	case 6:
		ctrl_lv = TMC_COOLER_LV_CTRL06;
		break;
	case 7:
		ctrl_lv = TMC_COOLER_LV_CTRL07;
		break;
	case 8:
		ctrl_lv = TMC_COOLER_LV_CTRL08;
		break;
	default:
		ctrl_lv = TMC_COOLER_LV_CTRL00;
		break;
	}

	mtk_cooler_mutt_dprintk_always(
		"[%s]lv(%d):ctrl_lv: 0x%08x\n", __func__, lv, ctrl_lv);

	return ctrl_lv;
}

static ssize_t _mtk_cl_mutt_tuning_write(
struct file *filp, const char __user *buffer, size_t count, loff_t *data)
{
	unsigned int len = 0;
	char desc[128];
	int klog_on = 0, mutt_a = 0, mutt_s = 0;
	int mutt_off1pa = 0, mutt_off1ca = 0, mutt_noIMS = 0, mutt_level = 0;
	//int ret_pa = 0;
	int ret = 0, ret_ca = 0, ret_pa = 0, ret_lv = 0;
	int scan_count = 0;
	char arg_name[32] = { 0 };

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);

	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

    /**
     * sscanf format <klog_on> <mtk-cl-mutt00 active (ms)>
     *<mtk-cl-mutt00 suspended (ms)> <mtk-cl-mutt01 active (ms)>
     *<mtk-cl-mutt01 suspended (ms)> <mtk-cl-mutt02 active (ms)>
     *<mtk-cl-mutt02 suspended (ms)>...
     * <klog_on> can only be 0 or 1
     * <mtk-cl-mutt* active/suspended (ms) > can only be positive
     *integer or 0 to denote no limit
     */

	if (data == NULL) {
		mtk_cooler_mutt_dprintk("[%s] null data\n", __func__);
		return -EINVAL;
	}

#if (MAX_NUM_INSTANCE_MTK_COOLER_MUTT == 4)
	/* cl_mutt_param[0] = 0; */
	/* cl_mutt_param[1] = 0; */
	/* cl_mutt_param[2] = 0; */
	/* cl_mutt_param[3] = 0; */

	scan_count =
	sscanf(desc, "%d %31s %d %d %d %d %d %d", &klog_on, arg_name,
		&mutt_level, &mutt_a, &mutt_s, &mutt_noIMS, &mutt_off1pa,
		&mutt_off1ca);

	if (scan_count >= 1) {

		mtk_cooler_mutt_dprintk_always(
			"[%s] log:%d A:%d,S:%d,noIMS:%d,off1pa:%x,off1ca:%x\n",
			__func__, klog_on, mutt_a, mutt_s, mutt_noIMS,
			mutt_off1pa, mutt_off1ca);

		if (klog_on == 0 || klog_on == 1)
			cl_mutt_klog_on = klog_on;


		if ((strncmp(arg_name, "mutt_level", 10) == 0)
				&& (mutt_level >= 0) && (mutt_level <= 8)) {

			last_md_tuning_boot_cnt =
				ccci_get_md_boot_count(MD_SYS1);

			cl_mutt_tuning_param_lv =
				level_selection(mutt_level);
			cl_mutt_tuning_param_lv =
				cl_mutt_tuning_param_lv |
				(TMC_COOLER_LV_ENABLE << 8);
			ret_lv = exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *)&cl_mutt_tuning_param_lv, 4);


			mtk_cooler_mutt_dprintk_always(
			"[%s]mutt_level:%d, lv: %d param: 0x%08x bcnt: %lul\n",
			__func__, mutt_level, ret_lv, cl_mutt_tuning_param_lv,
			last_md_tuning_boot_cnt);

			return len;
		}

		if ((strncmp(arg_name, "disable_level", 13) == 0)) {

			last_md_tuning_boot_cnt =
				ccci_get_md_boot_count(MD_SYS1);

			cl_mutt_tuning_param_lv =
				level_selection(mutt_level);
			cl_mutt_tuning_param_lv =
				TMC_LEVEL_CTRL_COOLER_LV_DISABLE;
			ret_lv = exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *)&cl_mutt_tuning_param_lv, 4);


			mtk_cooler_mutt_dprintk_always(
			"[%s]disable_lv:%d, lv: %d param: 0x%08x bcnt: %lul\n",
			__func__, mutt_level, ret_lv, cl_mutt_tuning_param_lv,
			last_md_tuning_boot_cnt);

			return len;
		}


		if (mutt_a == 0)
			cl_mutt_tuning_param = 0;
		else if (mutt_a >= 100 && mutt_a <= 25500
		&& mutt_s >= 100 && mutt_s <= 25500)
			cl_mutt_tuning_param =
			((mutt_s / 100) << MUTT_SUSPEND_OFFSET) |
			((mutt_a / 100) << MUTT_ACTIVATED_OFFSET);


		if (mutt_noIMS != 0xFF) {
			if (mutt_noIMS == 0)/*IMS enable*/
				cl_mutt_tuning_param = cl_mutt_tuning_param |
					MUTT_ENABLE_IMS_ENABLE;
			else/*IMS disable*/
				cl_mutt_tuning_param = cl_mutt_tuning_param |
					MUTT_ENABLE_IMS_DISABLE;
		}

		if (mutt_off1pa != 0xFF) {
			if (mutt_off1pa == 0)/*enable*/
				cl_mutt_tuning_param_pa =
					TMC_PA_CTRL_PA_ALL_ON;
			else/*disable*/
				cl_mutt_tuning_param_pa =
					TMC_PA_CTRL_PA_OFF_1PA;
		}

		if (mutt_off1ca != 0xFF) {
			if (mutt_off1ca == 0)/*enable*/
				cl_mutt_tuning_param_ca = TMC_CA_CTRL_CA_ON;
			else/*disable*/
				cl_mutt_tuning_param_ca = TMC_CA_CTRL_CA_OFF;
		}



		last_md_tuning_boot_cnt = ccci_get_md_boot_count(MD_SYS1);

		if (mutt_noIMS == 0xFF) {
			cl_mutt_tuning_param = TMC_THROTTLING_THROT_DISABLE;

			ret = exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *)&cl_mutt_tuning_param, 4);
			mtk_cooler_mutt_dprintk_always(
				"[%s]2 ret %d param %x bcnt %lul\n",
				__func__, ret, cl_mutt_tuning_param,
				last_md_tuning_boot_cnt);
		} else {/*Throttle disable*/
			ret = exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *)&cl_mutt_tuning_param, 4);
			mtk_cooler_mutt_dprintk_always(
				"[%s]Throttle disable: ret %d param %x bcnt %lul\n",
				__func__, ret, cl_mutt_tuning_param,
				last_md_tuning_boot_cnt);
		}


		if (mutt_off1pa != 0xFF) {
			ret_pa = exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *)&cl_mutt_tuning_param_pa, 4);

			mtk_cooler_mutt_dprintk_always(
				"[%s]3 ret_pa %d param %x bcnt %lul\n",
				__func__, ret_pa, cl_mutt_tuning_param_pa,
				last_md_tuning_boot_cnt);
		}


		if (mutt_off1ca != 0xFF) {
			ret_pa = exec_ccci_kern_func_by_md_id(MD_SYS1,
				ID_THROTTLING_CFG,
				(char *)&cl_mutt_tuning_param_ca, 4);
			mtk_cooler_mutt_dprintk_always(
			"[%s]4 ret_ca %d param %x bcnt %lul\n",
			__func__, ret_ca, cl_mutt_tuning_param_ca,
			last_md_tuning_boot_cnt);
		}

		return len;
	} /* scan_count >= 1 */
#else
#error	\
"Change correspondent part when changing MAX_NUM_INSTANCE_MTK_COOLER_MUTT!"
#endif

	mtk_cooler_mutt_dprintk("[%s] bad arg\n", __func__);
	return -EINVAL;
}

static int _mtk_cl_mutt_tuning_open(struct inode *inode, struct file *file)
{
	return single_open(file, _mtk_cl_mutt_tuning_read, NULL);
}

static const struct proc_ops clmutt_tuning_fops = {
	.proc_open = _mtk_cl_mutt_tuning_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_write = _mtk_cl_mutt_tuning_write,
	.proc_release = single_release,
};
#endif

int mtk_cooler_mutt_init(void)
{
	int err = 0;
	int i;

	for (i = MAX_NUM_INSTANCE_MTK_COOLER_MUTT; i-- > 0;) {
		cl_mutt_dev[i] = NULL;
		cl_mutt_state[i] = 0;
	}

	mtk_cooler_mutt_dprintk("init\n");

#if defined(FEATURE_MUTT_INTERFACE_VER)
	cl_mutt_cur_limit = TMC_THROTTLING_THROT_DISABLE;
#endif
	err = mtk_cooler_mutt_register_ltf();
	if (err)
		goto err_unreg;

	/* create a proc file */
	{
		struct proc_dir_entry *entry = NULL;
		struct proc_dir_entry *dir_entry = NULL;

		dir_entry = mtk_thermal_get_proc_drv_therm_dir_entry();
		if (!dir_entry) {
			mtk_cooler_mutt_dprintk_always(
				"[%s]: mkdir /proc/driver/thermal failed\n",
				__func__);
		} else {
			entry = proc_create("clmutt", 0664, dir_entry,
					&cl_mutt_fops);
			if (entry)
				proc_set_user(entry, uid, gid);
#if FEATURE_MUTT_V2
			entry = proc_create("clmutt_tm_pid", 0664, dir_entry,
							&clmutt_tm_pid_fops);
			if (entry)
				proc_set_user(entry, uid, gid);
#endif
#if FEATURE_THERMAL_DIAG
			entry = proc_create("clmutt_tmd_pid", 0664, dir_entry,
							&clmutt_tmd_pid_fops);
			if (entry)
				proc_set_user(entry, uid, gid);
#endif
#if defined(FEATURE_MUTT_INTERFACE_VER)
			entry = proc_create("clmutt_tuning", 0664, dir_entry,
							&clmutt_tuning_fops);
			if (entry)
				proc_set_user(entry, uid, gid);
#endif
		}
	}

#if FEATURE_ADAPTIVE_MUTT
	/* create_debugfs_entries(); */
#endif

	return 0;

err_unreg:
	mtk_cooler_mutt_unregister_ltf();
	return err;
}

void  mtk_cooler_mutt_exit(void)
{
	mtk_cooler_mutt_dprintk("exit\n");

	/* remove the proc file */
	remove_proc_entry("clmutt", NULL);

	mtk_cooler_mutt_unregister_ltf();
}
//module_init(mtk_cooler_mutt_init);
//module_exit(mtk_cooler_mutt_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("MediaTek Inc.");
