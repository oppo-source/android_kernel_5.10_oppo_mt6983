/*
 * Copyright (C) 2015 MediaTek Inc.
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

#define pr_fmt(fmt) "["KBUILD_MODNAME"] " fmt
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/kfifo.h>

#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#ifdef CONFIG_OF
#include <linux/of_fdt.h>
#endif
#include <linux/atomic.h>
#include <mt-plat/mtk_boot_common.h>


enum {
	BM_UNINIT = 0,
	BM_INITIALIZING = 1,
	BM_INITIALIZED = 2,
} BM_INIT_STATE;

enum boot_mode_t g_boot_mode = UNKNOWN_BOOT;
static int g_boot_type = 0xFF;
static atomic_t g_boot_init = ATOMIC_INIT(BM_UNINIT);
static atomic_t g_boot_errcnt = ATOMIC_INIT(0);
static atomic_t g_boot_status = ATOMIC_INIT(0);

#ifdef CONFIG_OF
struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};
static int __init dt_get_boot_common(void)
{
	struct tag_bootmode *tags = NULL;
	struct device_node *node;

	node = of_find_node_by_path("/chosen");
	if (!node) {
		node = of_find_node_by_path("/chosen@0");
		if (!node) {
			pr_err("chosen node is not found!!\n");
			return 0;
		}
	}

	tags = (struct tag_bootmode *)of_get_property(node,
		"atag,boot", NULL);

	if (tags) {
		g_boot_mode = tags->bootmode;
		g_boot_type = tags->boottype;
		/* Add default value if no lk pass storage type */
		if ((g_boot_type > 2) || (g_boot_type < 0))
			g_boot_type = BOOTDEV_SDMMC;
		atomic_set(&g_boot_status, 1);
	} else {
		pr_warn("'atag,boot' is not found\n");
	}

	/* break now */
	return 1;
}
#endif


static void __init init_boot_common(unsigned int line)
{
#ifdef CONFIG_OF
	int rc;

	if (atomic_read(&g_boot_init) == BM_INITIALIZING) {
		pr_notice("%s (%d) state(%d,%d)\n", __func__, line,
			atomic_read(&g_boot_init), g_boot_mode);
		atomic_inc(&g_boot_errcnt);
		return;
	}

	if (atomic_read(&g_boot_init) == BM_UNINIT)
		atomic_set(&g_boot_init, BM_INITIALIZING);
	else
		return;

	if (g_boot_mode != UNKNOWN_BOOT) {
		atomic_set(&g_boot_init, BM_INITIALIZED);
		pr_notice("%s (%d) boot_mode = %d\n",
			__func__, line, g_boot_mode);
		return;
	}

	pr_debug("%s %d %d %d\n", __func__, line, g_boot_mode,
		atomic_read(&g_boot_init));
	rc = dt_get_boot_common();
	if (rc != 0)
		atomic_set(&g_boot_init, BM_INITIALIZED);
	else
		pr_warn("fail, of_scan_flat_dt() = %d", rc);
	pr_err("Roland:%s %d %d %d\n", __func__, line, g_boot_mode,
		atomic_read(&g_boot_init));
#endif
}

/* return boot mode */
/* #ifndef FEATURE_BUG_STABILITY */
/* modify for conlict with oplus/kernel/system/boot_mode.h build error */
/* unsigned int get_boot_mode(void) */
/* #else */
int get_boot_mode(void)
/* #endif */
{
	if (atomic_read(&g_boot_init) != BM_INITIALIZED) {
		pr_warn("fail, %s (%d) state(%d,%d)\n", __func__, __LINE__,
			atomic_read(&g_boot_init), g_boot_mode);
	}
	return g_boot_mode;
}
EXPORT_SYMBOL(get_boot_mode);

unsigned int get_boot_type(void)
{
	if (atomic_read(&g_boot_init) != BM_INITIALIZED) {
		pr_debug("fail, %s (%d) state(%d,%d)\n", __func__, __LINE__,
			atomic_read(&g_boot_init), g_boot_mode);
	}
	return g_boot_type;
}
EXPORT_SYMBOL(get_boot_type);

bool is_advanced_meta_mode(void)
{
	if (atomic_read(&g_boot_init) != BM_INITIALIZED) {
		pr_warn("fail, %s (%d) state(%d,%d)\n", __func__, __LINE__,
			atomic_read(&g_boot_init), g_boot_mode);
	}

	if (g_boot_mode == ADVMETA_BOOT)
		return true;
	else
		return false;
}
EXPORT_SYMBOL(is_advanced_meta_mode);

static ssize_t boot_show(struct kobject *kobj, struct attribute *a, char *buf)
{
	if (!strcmp(a->name, "boot_mode"))
		return sprintf(buf, "%d\n", get_boot_mode());
	else if (!strcmp(a->name, "boot_type"))
		return sprintf(buf, "%u\n", get_boot_type());
	else
		return sprintf(buf, "0\n");
}

static ssize_t boot_store(struct kobject *kobj, struct attribute *a,
	const char *buf, size_t count)
{
	return count;
}

/* boot object */
static struct kobject boot_kobj;
static const struct sysfs_ops boot_sysfs_ops = {
	.show = boot_show,
	.store = boot_store,
};

/* boot attribute */
struct attribute boot_mode_attr = { BOOT_MODE_SYSFS_ATTR, 0644 };
struct attribute boot_type_attr = { BOOT_TYPE_SYSFS_ATTR, 0644 };

static struct attribute *boot_attrs[] = {
	&boot_mode_attr,
	&boot_type_attr,
	NULL
};

/* boot mode and type */
static struct kobj_type boot_ktype = {
	.sysfs_ops = &boot_sysfs_ops,
	.default_attrs = boot_attrs
};

/* boot device node */
static dev_t boot_dev_num;
static struct cdev boot_cdev;
static const struct file_operations boot_fops = {
	.owner = THIS_MODULE,
	.open = NULL,
	.release = NULL,
	.write = NULL,
	.read = NULL,
	.unlocked_ioctl = NULL
};

#ifdef OPLUS_BUG_STABILITY
static struct kobject *systeminfo_kobj;
static ssize_t ftmmode_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	if (oppo_boot_mode == OPPO_SILENCE_BOOT)
		return sprintf(buf, "%d\n", SILENCE_BOOT);
	else if (oppo_boot_mode == OPPO_SAFE_BOOT)
		return sprintf(buf, "%d\n", SAFE_BOOT);
	else if (oppo_boot_mode == OPPO_AGING_BOOT)
		return sprintf(buf, "%d\n", AGING_BOOT);
	else
		return sprintf(buf, "%d\n", get_boot_mode());
}

struct kobj_attribute ftmmode_attr = {
	.attr = {"ftmmode", 0444},

	.show = &ftmmode_show,
};
static struct attribute * g[] = {
	&ftmmode_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};
#endif /* OPLUS_BUG_STABILITY */
/* boot device class */
static struct class *boot_class;
static struct device *boot_device;

static int __init create_sysfs(void)
{
	int ret;

	/* allocate device major number */
	if (alloc_chrdev_region(&boot_dev_num, 0, 1, BOOT_DEV_NAME) < 0) {
		pr_warn("fail to register chrdev\n");
		return -1;
	}

	/* add character driver */
	cdev_init(&boot_cdev, &boot_fops);
	ret = cdev_add(&boot_cdev, boot_dev_num, 1);
	if (ret < 0) {
		pr_warn("fail to add cdev\n");
		return ret;
	}

	/* create class (device model) */
	boot_class = class_create(THIS_MODULE, BOOT_DEV_NAME);
	if (IS_ERR(boot_class)) {
		pr_warn("fail to create class\n");
		return -1;
	}

	boot_device = device_create(boot_class, NULL, boot_dev_num, NULL,
					BOOT_DEV_NAME);
	if (IS_ERR(boot_device)) {
		pr_warn("fail to create device\n");
		return -1;
	}

	/* add kobject */
	ret = kobject_init_and_add(&boot_kobj, &boot_ktype,
			&(boot_device->kobj), BOOT_SYSFS);
	if (ret < 0) {
		pr_warn("fail to add kobject\n");
		return ret;
	}

#ifdef OPLUS_BUG_STABILITY
	systeminfo_kobj = kobject_create_and_add("systeminfo", NULL);
	printk("oppo create systeminto node suscess!\n");
	if (systeminfo_kobj)
		ret = sysfs_create_group(systeminfo_kobj, &attr_group);
#endif /* OPLUS_BUG_STABILITY */
	return 0;
}

/*static void __exit destroy_sysfs(void)
{
	cdev_del(&boot_cdev);
}*/

static int boot_mode_proc_show(struct seq_file *p, void *v)
{
	seq_puts(p, "\n\rMTK BOOT MODE : ");
	switch (g_boot_mode) {
	case NORMAL_BOOT:
		seq_puts(p, "NORMAL BOOT\n");
		break;
	case META_BOOT:
		seq_puts(p, "META BOOT\n");
		break;
	case ADVMETA_BOOT:
		seq_puts(p, "Advanced META BOOT\n");
		break;
	case ATE_FACTORY_BOOT:
		seq_puts(p, "ATE_FACTORY BOOT\n");
		break;
	case ALARM_BOOT:
		seq_puts(p, "ALARM BOOT\n");
		break;
	default:
		seq_puts(p, "UNKNOWN BOOT\n");
		break;
	}

	return 0;
}

#if 0
#ifdef OPLUS_BUG_STABILITY
OPPO_BOOTMODE oppo_boot_mode = OPPO_NORMAL_BOOT;
static int oppo_get_boot_mode(char *oppo_boot_mode_char)
{
	int  boot_mode_temp = 0;
	sscanf(oppo_boot_mode_char, "%d", &boot_mode_temp);
	if(boot_mode_temp == 0)
	{
		oppo_boot_mode = OPPO_NORMAL_BOOT;
	}
	else if (boot_mode_temp == 1)
	{
		oppo_boot_mode = OPPO_SILENCE_BOOT;
	}
	else if (boot_mode_temp == 2)
	{
		oppo_boot_mode = OPPO_SAFE_BOOT;
	}
	else if (boot_mode_temp == 3)
	{
		oppo_boot_mode = OPPO_AGING_BOOT;
	}
	else
	{
		oppo_boot_mode = OPPO_UNKNOWN_BOOT;
	}
    pr_err("oppo_boot_mode: %d ",oppo_boot_mode);

     return 1;
}
__setup("oplus_boot_mode=", oppo_get_boot_mode);
#endif /* OPLUS_BUG_STABILITY */
#endif

static int boot_mode_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, boot_mode_proc_show, NULL);
}

static const struct proc_ops boot_mode_proc_fops = {
	.proc_open = boot_mode_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int __init boot_common_core(void)
{
	init_boot_common(__LINE__);
	/* create proc entry at /proc/boot_mode */
	if (proc_create_data("boot_mode", 0444, NULL,
		&boot_mode_proc_fops, NULL) == NULL)
		pr_warn("create procfs fail");

	/* create sysfs entry at /sys/class/BOOT/BOOT/boot */
	create_sysfs();
	return 0;
}

arch_initcall(boot_common_core);
MODULE_DESCRIPTION("MTK Boot Information Common Driver");
MODULE_LICENSE("GPL");
