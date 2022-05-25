// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/arm-smccc.h> /* for Kernel Native SMC API */
#include <linux/module.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>       /* min() */
#include <linux/uaccess.h>      /* copy_to_user() */
#include <linux/sched/clock.h> /* local_clock() */
#include <linux/sched/signal.h> /* TASK_INTERRUPTIBLE/signal_pending/schedule */
#include <linux/poll.h>
#include <linux/io.h>           /* ioremap() */
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <asm/setup.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/atomic.h>
#include <linux/irq.h>
#include <linux/syscore_ops.h>
#include <linux/soc/mediatek/mtk_sip_svc.h> /* for SMC ID table */
#if CONFIG_OF_RESERVED_MEM
#include <linux/of_reserved_mem.h>
#define ATF_LOG_RESERVED_MEMORY_KEY "mediatek,atf-log-reserved"
#endif
#include "tfa_debug.h"

/* #define ATF_LOGGER_DEBUG */
#define ATF_LOG_CTRL_BUF_SIZE 512
#define ATF_CRASH_MAGIC_NO	0xdead1abf
#define ATF_LAST_MAGIC_NO	0x41544641

static struct mutex atf_mutex; /* shared between the threads */
static struct mutex atf_raw_buf_mutex;
static u32 atf_raw_read_offset;
static wait_queue_head_t    atf_log_wq;

#if CONFIG_OF
static const struct of_device_id atf_logger_of_ids[] = {
	{ .compatible = "mediatek,atf_logger", },
	{},
};
#endif

union atf_log_ctrl_t {
	struct {
		u64 atf_log_addr;          /*  0x00 */
		u64 atf_log_size;
		u32 atf_write_offset;
		u32 atf_read_offset;

		/* must keep value after reboot */
		u64 atf_crash_log_addr;
		u64 atf_crash_log_size;
		u32 atf_total_write_count;
		u32 atf_crash_flag;

		/* Remove arrays used CONFIG_NR_CPUS */
		/* Please add new field before these arrays */
	} info;
	unsigned char data[ATF_LOG_CTRL_BUF_SIZE];
};

static union atf_log_ctrl_t *atf_buf_vir_ctrl;
static phys_addr_t atf_buf_phy_ctrl;
static phys_addr_t atf_buf_len;
static unsigned char *atf_buf_vir_addr;
static unsigned char *atf_log_vir_addr;
static unsigned int atf_log_len;


#if CONFIG_OF
static int dt_parse_atf_logger_buf(void)
{
	struct device_node *atf_logger_rmem_np;
	struct reserved_mem *rmem;

	atf_logger_rmem_np = of_find_compatible_node(NULL, NULL,
			ATF_LOG_RESERVED_MEMORY_KEY);

	if (!atf_logger_rmem_np) {
		pr_info("No atf_logger reserved memory\n");
		return -EINVAL;
	}
	rmem = of_reserved_mem_lookup(atf_logger_rmem_np);
	if (!rmem) {
		pr_info("No rmem\n");
		return -EINVAL;
	}

	atf_buf_phy_ctrl = (phys_addr_t) rmem->base;
	atf_buf_len = (phys_addr_t) rmem->size;

#ifdef ATF_LOGGER_DEBUG
	pr_notice("%s, start: %pa, size: %pa\n",
		ATF_LOG_RESERVED_MEMORY_KEY,
		&atf_buf_phy_ctrl, &atf_buf_len);
#endif
	return 0;
}
#endif

static ssize_t atf_log_write(struct file *file,
	const char __user *buf, size_t count, loff_t *pos)
{
	unsigned long ret = -1;
	unsigned long param = -1;
	struct arm_smccc_res res;

	if (count < 12) {
	/* for coverity check */
		/* use kstrtoul_from_user() instead of */
		/* copy_from_user() and kstrtoul() */
		ret = kstrtoul_from_user(buf, count, 16, &param);
	}

#ifdef ATF_LOGGER_DEBUG
	pr_notice("[%s]param:0x%lx, count:%zu, ret:%ld\n",
		__func__, param, count, ret);
#endif

	if (!ret) {
		arm_smccc_smc(MTK_SIP_KERNEL_ATF_DEBUG,
			param, 0, 0, 0, 0, 0, 0, &res);
	} else {
		wake_up_interruptible(&atf_log_wq);
	}
	*pos += count;

	return count;
}

static ssize_t do_read_log_to_usr(char __user *buf, size_t count)
{
	size_t copy_len;
	size_t right;
	unsigned int local_write_index =
		atf_buf_vir_ctrl->info.atf_write_offset;
	unsigned int local_read_index =
		atf_buf_vir_ctrl->info.atf_read_offset;

	/* check copy length */
	copy_len = (local_write_index +
		atf_log_len - local_read_index) % atf_log_len;

	/* if copy length < count, just copy the "copy length" */
	if (count > copy_len)
		count = copy_len;

	if (local_write_index > local_read_index) {
		/* write (right) - read (left) */
		/* --------R-------W-----------*/
		if (copy_to_user(buf,
			atf_log_vir_addr + local_read_index, count))
			return -EFAULT;
	} else {
		/* turn around to the head */
		/* --------W-------R-----------*/
		right = atf_log_len - local_read_index;

		/* check buf space is enough to copy */
		if (count > right) {
			/* if space is enough to copy */
			if (copy_to_user(buf,
				atf_log_vir_addr + local_read_index, right))
				return -EFAULT;
			if (copy_to_user((buf + right),
				atf_log_vir_addr, count - right))
				return -EFAULT;
		} else {
			/* if count is only enough to copy right or count,
			 * just copy right or count.
			 */
			if (copy_to_user(buf,
				atf_log_vir_addr + local_read_index, count))
				return -EFAULT;
		}
	}
	/* update the read pos */
	local_read_index = (local_read_index + count) % atf_log_len;
	atf_buf_vir_ctrl->info.atf_read_offset = local_read_index;

	return count;
}

static int atf_log_open(struct inode *inode, struct file *file)
{
	int ret;

	ret = nonseekable_open(inode, file);
	if (unlikely(ret))
		return ret;
	file->private_data = NULL;

	return 0;
}

static int atf_log_release(struct inode *ignored, struct file *file)
{
	return 0;
}

static ssize_t atf_log_read(struct file *file,
	char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t readback_bytes = 0;
	int rc = 0;
	unsigned int write_pos;
	unsigned int read_pos;

	/* inside a thread */
	mutex_lock(&atf_mutex);
start:
	while (1) {
		/* pr_notice("atf_log_read: wait in wq\n"); */
		wait_event_timeout(atf_log_wq,
			(atf_buf_vir_ctrl->info.atf_write_offset !=
			atf_buf_vir_ctrl->info.atf_read_offset), HZ);

		write_pos = atf_buf_vir_ctrl->info.atf_write_offset;
		read_pos = atf_buf_vir_ctrl->info.atf_read_offset;

		if (write_pos != read_pos) {
			break;
		} else if (file->f_flags & O_NONBLOCK) {
			rc = -EAGAIN;
			break;
		}
	}

	if (rc) {
		pr_notice("%s: rc=%d\n", __func__, rc);
		/* do the work with the data you're protecting */
		mutex_unlock(&atf_mutex);
		return rc;
	}

	if (unlikely(write_pos == read_pos))
		goto start;

	readback_bytes = do_read_log_to_usr(buf, count);
	/* update the file pos */
	*f_pos += readback_bytes;

	/* do the work with the data you're protecting */
	mutex_unlock(&atf_mutex);

	return readback_bytes;
}

static unsigned int atf_log_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;

	if (!(file->f_mode & FMODE_READ))
		return ret;

	poll_wait(file, &atf_log_wq, wait);

	if (atf_buf_vir_ctrl->info.atf_write_offset !=
		atf_buf_vir_ctrl->info.atf_read_offset)
		ret = POLLIN | POLLRDNORM;

	return ret;
}

static long atf_log_ioctl(struct file *flip,
	unsigned int cmd, unsigned long arg)
{
	return 0;
}

#ifdef ATF_LOGGER_DEBUG
void show_atf_log_ctl(void)
{
	pr_notice("ATF reserved memory, start: %pa, size: %pa\n",
				&atf_buf_phy_ctrl, &atf_buf_len);
	pr_notice("atf_buf_phy_ctrl: %pa\n", &atf_buf_phy_ctrl);
	pr_notice("atf_buf_len: %pa\n", &atf_buf_len);
	pr_notice("atf_buf_vir_ctrl: %pa\n", &atf_buf_vir_ctrl);
	pr_notice("atf_log_vir_addr: %pa\n", &atf_log_vir_addr);
	pr_notice("atf_log_len: %u\n", atf_log_len);
	pr_notice("atf_log_addr(%pa) = 0x%x\n",
			&(atf_buf_vir_ctrl->info.atf_log_addr),
			atf_buf_vir_ctrl->info.atf_log_addr);
	pr_notice("atf_log_size(%pa) = 0x%x\n",
			&(atf_buf_vir_ctrl->info.atf_log_size),
			atf_buf_vir_ctrl->info.atf_log_size);
	pr_notice("atf_write_offset(%pa) = %u\n",
			&(atf_buf_vir_ctrl->info.atf_write_offset),
			atf_buf_vir_ctrl->info.atf_write_offset);
	pr_notice("atf_read_offset(%pa) = %u\n",
			&(atf_buf_vir_ctrl->info.atf_read_offset),
			atf_buf_vir_ctrl->info.atf_read_offset);
}

static void show_data(unsigned long addr,
	uint64_t nbytes, const char *name)
{
	uint64_t i, j;
	uint64_t nlines;
	uint32_t *p = 0;

	pr_notice("\n%s: %#lx:\n", name, addr);

	/*
	 * round address down to a 32 bit boundary
	 * and always dump a multiple of 32 bytes
	 */
	p = (uint32_t *)(addr & ~(sizeof(uint32_t) - 1));
	nbytes += (uint64_t) (addr & (sizeof(uint32_t) - 1));
	nlines = (nbytes + 31) / 32;

	for (i = 0; i < nlines; i++) {
		/*
		 * just display low 16 bits of address to keep
		 * each line of the dump < 80 characters
		 */
		pr_notice("%#lx ", ((uintptr_t)p) & 0xffff);
		for (j = 0; j < 8; j++) {
			u32	data;

			if (probe_kernel_address((void *)p, data))
				pr_notice(" ********");
			else
				pr_notice(" %08x", data);
			++p;
		}
		pr_debug("\n");
	}
}
#endif


static int atf_time_sync_resume(struct device *dev)
{
	/* Get local_clock and sync to TF-A */
	u64 time_to_sync = local_clock();
	struct arm_smccc_res res;

#ifdef ATF_LOGGER_DEBUG
	/* make sure atf logger do the time sync with TF-A */
	pr_notice("atf time sync when resume\n");
#endif
	/* always separate time_to_sync into two 32 bits args */
	arm_smccc_smc(MTK_SIP_KERNEL_TIME_SYNC,
		(u32)time_to_sync, (u32)(time_to_sync >> 32), 0, 0,
		0, 0, 0, &res);

	return 0;
}

static const struct proc_ops atf_log_proc_fops = {
	.proc_ioctl = atf_log_ioctl,
	.proc_compat_ioctl = atf_log_ioctl,
	.proc_poll       = atf_log_poll,
	.proc_open       = atf_log_open,
	.proc_release    = atf_log_release,
	.proc_read       = atf_log_read,
	.proc_write      = atf_log_write,
};

static const struct file_operations atf_log_fops = {
	.owner      = THIS_MODULE,
	.unlocked_ioctl = atf_log_ioctl,
	.compat_ioctl = atf_log_ioctl,
	.poll       = atf_log_poll,
	.open       = atf_log_open,
	.release    = atf_log_release,
	.read       = atf_log_read,
	.write      = atf_log_write,
};
static struct miscdevice atf_log_dev = {
	.minor      = MISC_DYNAMIC_MINOR,
	.name       = "atf_log",
	.fops       = &atf_log_fops,
	.mode       = 0640,
};

static ssize_t do_read_raw_buf_to_usr(char __user *buf, size_t count)
{
	size_t copy_len;

	/* check copy length */
	copy_len = atf_buf_len - atf_raw_read_offset;

	/* if copy length < count, just copy the "copy length" */
	if (count > copy_len)
		count = copy_len;

	if (copy_to_user(buf,
		(uint8_t *)(atf_buf_vir_addr + atf_raw_read_offset), count))
		return -EFAULT;

	/* update the read pos */
	atf_raw_read_offset += count;

	return count;
}

static ssize_t atf_raw_buf_read(struct file *file,
	char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t readback_bytes = 0;

	/* inside a thread */
	mutex_lock(&atf_raw_buf_mutex);

	readback_bytes = do_read_raw_buf_to_usr(buf, count);
	/* update the file pos */
	*f_pos += readback_bytes;

	/* do the work with the data you're protecting */
	mutex_unlock(&atf_raw_buf_mutex);

	return readback_bytes;
}

static int atf_raw_buf_open(struct inode *inode, struct file *file)
{
	int ret;

	ret = nonseekable_open(inode, file);
	if (unlikely(ret))
		return ret;
	file->private_data = NULL;

	/* clear raw buffer read offset */
	mutex_lock(&atf_raw_buf_mutex);
	atf_raw_read_offset = 0;
	mutex_unlock(&atf_raw_buf_mutex);

	return 0;
}

static unsigned int atf_raw_buf_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;

	if (!(file->f_mode & FMODE_READ))
		return ret;

	poll_wait(file, &atf_log_wq, wait);

	/* always reply readable */
	if (atf_raw_read_offset < atf_buf_len)
		ret = POLLIN | POLLRDNORM;

	return ret;
}

static const struct proc_ops atf_raw_buf_proc_fops = {
	.proc_poll       = atf_raw_buf_poll,
	.proc_open       = atf_raw_buf_open,
	.proc_release    = atf_log_release,
	.proc_read       = atf_raw_buf_read,
};

static const struct file_operations atf_raw_buf_fops = {
	.owner      = THIS_MODULE,
	.poll       = atf_raw_buf_poll,
	.open       = atf_raw_buf_open,
	.release    = atf_log_release,
	.read       = atf_raw_buf_read,
};

static struct miscdevice atf_raw_buf_dev = {
	.minor      = MISC_DYNAMIC_MINOR,
	.name       = "atf_raw_buf",
	.fops       = &atf_raw_buf_fops,
	.mode       = 0440,
};

static int __init atf_logger_probe(struct platform_device *pdev)
{
	/* register module driver */
	int err;
	struct proc_dir_entry *atf_log_proc_dir = NULL;
	struct proc_dir_entry *atf_log_proc_file = NULL;
	struct proc_dir_entry *atf_raw_buf_proc_file = NULL;

	pr_notice("%s\n", __func__);
	err = dt_parse_atf_logger_buf();
	if (unlikely(err)) {
		pr_info("No atf_log_buffer\n");
		return -1;
	}

	err = misc_register(&atf_log_dev);
	if (unlikely(err)) {
		pr_info("atf_log: failed to register device");
		return -1;
	}

	err = misc_register(&atf_raw_buf_dev);
	if (unlikely(err)) {
		pr_info("atf_raw_buf: failed to register device");
		return -1;
	}

	/* map whole buffer for raw dump */
	atf_buf_vir_addr = ioremap_wc(atf_buf_phy_ctrl, atf_buf_len);
	atf_buf_vir_ctrl = (union atf_log_ctrl_t *) atf_buf_vir_addr;
	atf_log_len = atf_buf_vir_ctrl->info.atf_log_size;
	atf_log_vir_addr = (unsigned char *)atf_buf_vir_ctrl +
					ATF_LOG_CTRL_BUF_SIZE;

#ifdef ATF_LOGGER_DEBUG
	show_atf_log_ctl();
	show_data((unsigned long)atf_buf_vir_ctrl, 512, "atf_buf");
#endif

	/* Synchronize timestamp in Kernel and ATF */
	atf_time_sync_resume(NULL);

	/* initial wait queue */
	init_waitqueue_head(&atf_log_wq);

	/* create /proc/atf_log */
	atf_log_proc_dir = proc_mkdir("atf_log", NULL);
	if (atf_log_proc_dir == NULL) {
		pr_info("atf_log proc_mkdir failed\n");
		return -ENOMEM;
	}

	/* create /proc/atf_log/atf_log */
	atf_log_proc_file = proc_create("atf_log", 0440,
		atf_log_proc_dir, &atf_log_proc_fops);
	if (atf_log_proc_file == NULL) {
		pr_info("atf_log proc_create failed at atf_log\n");
		return -ENOMEM;
	}

	/* create /proc/atf_log/atf_raw_buf */
	atf_raw_buf_proc_file = proc_create("atf_raw_buf", 0440,
		atf_log_proc_dir, &atf_raw_buf_proc_fops);
	if (atf_raw_buf_proc_file == NULL) {
		pr_info("atf_raw_buf proc_create failed at atf_log\n");
		return -ENOMEM;
	}

	return 0;
}

static int atf_logger_remove(struct platform_device *dev)
{
	misc_deregister(&atf_log_dev);
	return 0;
}

static const struct dev_pm_ops atf_pm_ops = {
	.resume_noirq = atf_time_sync_resume,
};

static struct platform_driver atf_logger_driver_probe = {
	.probe = atf_logger_probe,
	.remove = atf_logger_remove,
	.driver = {
		.name = "atf_log",
		.owner = THIS_MODULE,
#if CONFIG_OF
		.of_match_table = atf_logger_of_ids,
#endif
		.pm = &atf_pm_ops,
	},
};

static int __init atf_log_init(void)
{
	int ret = 0;

	pr_notice("%s\n", __func__);
	mutex_init(&atf_mutex); /* called only ONCE */
	mutex_init(&atf_raw_buf_mutex);

	pr_notice("atf_logger driver register\n");
	ret = platform_driver_register(&atf_logger_driver_probe);
	if (ret)
		pr_info("atf logger init FAIL, ret 0x%x\n", ret);
	tfa_debug_drv_register();
	return ret;
}

static void __exit atf_log_exit(void)
{
	tfa_debug_drv_unregister();
	platform_driver_unregister(&atf_logger_driver_probe);
	pr_notice("atf_log: exited");
}

module_init(atf_log_init);
module_exit(atf_log_exit);

MODULE_DESCRIPTION("MEDIATEK Module ATF Logging Driver");
MODULE_AUTHOR("Chun Fan<chun.fan@mediatek.com>");
MODULE_LICENSE("GPL");

