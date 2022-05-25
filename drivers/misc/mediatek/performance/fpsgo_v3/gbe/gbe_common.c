// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#include <linux/sched/clock.h>
#include <linux/sched/mm.h>
#include <linux/sched/numa_balancing.h>
#include <linux/sched/task_stack.h>
#include <linux/sched/task.h>
#include <linux/sched/cputime.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/kallsyms.h>
#include <linux/trace_events.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>   /* for misc_register, and SYNTH_MINOR */
#include <linux/proc_fs.h>


#include <linux/workqueue.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/pm_qos.h>
#include "gbe_common.h"
#include "gbe1.h"
#include "gbe2.h"
#include "gbe_sysfs.h"

static DEFINE_MUTEX(gbe_lock);
static int boost_set[KIR_NUM];
static unsigned long policy_mask;

enum GBE_BOOST_DEVICE {
	GBE_BOOST_UNKNOWN = -1,
	GBE_BOOST_CPU = 0,
	GBE_BOOST_EAS = 1,
	GBE_BOOST_VCORE = 2,
	GBE_BOOST_IO = 3,
	GBE_BOOST_HE = 4,
	GBE_BOOST_GPU = 5,
	GBE_BOOST_LLF = 6,
	GBE_BOOST_NUM = 7,
};

void gbe_trace_printk(int pid, char *module, char *string)
{
	int len;
	char buf2[256];

	len = snprintf(buf2, sizeof(buf2), "%d [%s] %s\n",
			pid, module, string);
	if (unlikely(len < 0))
		return;
	else if (unlikely(len == 256))
		buf2[255] = '\0';

	trace_printk(buf2);
}

struct k_list {
	struct list_head queue_list;
	int gbe2pwr_cmd;
	int gbe2pwr_value1;
	int gbe2pwr_value2;
};
static LIST_HEAD(head);
static int condition_get_cmd;
static DEFINE_MUTEX(gbe2pwr_lock);
static DECLARE_WAIT_QUEUE_HEAD(pwr_queue);
void gbe_sentcmd(int cmd, int value1, int value2)
{
	static struct k_list *node;

	mutex_lock(&gbe2pwr_lock);
	node = kmalloc(sizeof(*node), GFP_KERNEL);
	if (node == NULL)
		goto out;
	node->gbe2pwr_cmd = cmd;
	node->gbe2pwr_value1 = value1;
	node->gbe2pwr_value2 = value2;
	list_add_tail(&node->queue_list, &head);
	condition_get_cmd = 1;
out:
	mutex_unlock(&gbe2pwr_lock);
	wake_up_interruptible(&pwr_queue);
}

void gbe_ctrl2base_get_pwr_cmd(int *cmd, int *value1, int *value2)
{
	static struct k_list *node;

	wait_event_interruptible(pwr_queue, condition_get_cmd);
	mutex_lock(&gbe2pwr_lock);
	if (!list_empty(&head)) {
		node = list_first_entry(&head,struct k_list, queue_list);
		*cmd = node->gbe2pwr_cmd;
		*value1 = node->gbe2pwr_value1;
		*value2 = node->gbe2pwr_value2;
		list_del(&node->queue_list);
		kfree(node);
	}
	if (list_empty(&head))
		condition_get_cmd = 0;
	mutex_unlock(&gbe2pwr_lock);
}

void gbe_boost(enum GBE_KICKER kicker, int boost)
{
	int i;
	int boost_final = 0;
	int cpu_boost = 0, eas_boost = -1, vcore_boost = -1,
			io_boost = 0, he_boost = 0, gpu_boost = 0,
			llf_boost = 0;

	mutex_lock(&gbe_lock);

	if (boost_set[kicker] == !!boost)
		goto out;

	boost_set[kicker] = !!boost;

	for (i = 0; i < KIR_NUM; i++)
		if (boost_set[i] == 1) {
			boost_final = 1;
			break;
		}

	if (boost_final) {
		cpu_boost = 1;
		eas_boost = 100;
		vcore_boost = 0;
		io_boost = 1;
		he_boost = 1;
		gpu_boost = 1;
		llf_boost = 1;
	} 

	if (test_bit(GBE_BOOST_CPU, &policy_mask))
		gbe_sentcmd(GBE_BOOST_CPU, cpu_boost, -1);

	if (test_bit(GBE_BOOST_EAS, &policy_mask))
		gbe_sentcmd(GBE_BOOST_EAS, eas_boost, -1);

	if (test_bit(GBE_BOOST_VCORE, &policy_mask))
		gbe_sentcmd(GBE_BOOST_VCORE, vcore_boost, -1);

	if (test_bit(GBE_BOOST_IO, &policy_mask))
		gbe_sentcmd(GBE_BOOST_IO, io_boost, -1);

	if (test_bit(GBE_BOOST_HE, &policy_mask))
		gbe_sentcmd(GBE_BOOST_HE, he_boost, -1);

	if (test_bit(GBE_BOOST_GPU, &policy_mask))
		gbe_sentcmd(GBE_BOOST_GPU, gpu_boost, -1);

	if (test_bit(GBE_BOOST_LLF, &policy_mask))
		gbe_sentcmd(GBE_BOOST_LLF, llf_boost, -1);

out:
	mutex_unlock(&gbe_lock);

}

static ssize_t gbe_policy_mask_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", policy_mask);
}

static ssize_t gbe_policy_mask_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int val = 0;
	char acBuffer[GBE_SYSFS_MAX_BUFF_SIZE];
	int arg;
	int cpu_boost = 0, eas_boost = -1, vcore_boost = -1,
			io_boost = 0, he_boost = 0, gpu_boost = 0,
			llf_boost = 0;

	if ((count > 0) && (count < GBE_SYSFS_MAX_BUFF_SIZE)) {
		if (scnprintf(acBuffer, GBE_SYSFS_MAX_BUFF_SIZE, "%s", buf)) {
			if (kstrtoint(acBuffer, 0, &arg) == 0)
				val = arg;
			else
				return count;
		}
	}

	if (val > 1 << GBE_BOOST_NUM || val < 0)
		return count;

	mutex_lock(&gbe_lock);
	policy_mask = val;

	gbe_sentcmd(GBE_BOOST_CPU, cpu_boost, -1);
	gbe_sentcmd(GBE_BOOST_EAS, eas_boost, -1);
	gbe_sentcmd(GBE_BOOST_VCORE, vcore_boost, -1);
	gbe_sentcmd(GBE_BOOST_IO, io_boost, -1);
	gbe_sentcmd(GBE_BOOST_HE, he_boost, -1);
	gbe_sentcmd(GBE_BOOST_GPU, gpu_boost, -1);
	gbe_sentcmd(GBE_BOOST_LLF, llf_boost, -1);
	mutex_unlock(&gbe_lock);

	return count;
}

static KOBJ_ATTR_RW(gbe_policy_mask);

void exit_gbe_common(void)
{
	gbe_sysfs_remove_file(&kobj_attr_gbe_policy_mask);
	gbe_sysfs_exit();
	gbe1_exit();
	gbe2_exit();
}

struct dentry *gbe_debugfs_dir;
int init_gbe_common(void)
{
	gbe_get_cmd_fp = gbe_ctrl2base_get_pwr_cmd;

	gbe_sysfs_init();
	gbe1_init();
	gbe2_init();

	gbe_sysfs_create_file(&kobj_attr_gbe_policy_mask);


	set_bit(GBE_BOOST_CPU, &policy_mask);
	set_bit(GBE_BOOST_EAS, &policy_mask);
	set_bit(GBE_BOOST_VCORE, &policy_mask);
	set_bit(GBE_BOOST_HE, &policy_mask);

	return 0;
}

