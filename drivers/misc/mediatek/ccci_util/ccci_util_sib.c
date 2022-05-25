// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <asm/cacheflush.h>

#include <mt-plat/mtk_ccci_common.h>
#include "ccci_util_sib.h"
#include "ccci_util_log.h"

static struct ccci_sib_region ccci_sib;

static ssize_t ccci_sib_read(struct file *file, char __user *buf,
	size_t size, loff_t *ppos)
{
	unsigned int *read_ptr;
	unsigned int read_pos, read_len, available;

	read_ptr = file->private_data;
	read_pos = *read_ptr;
	available = ccci_sib.size - read_pos;
	read_len = size < available ? size : available;
	if (read_len == 0)
		return 0;
#ifndef mtk09077
	/* #ifdef CONFIG_ARCH_HAS_PMEM_API */
	//arch_invalidate_pmem(ccci_sib.base_ap_view_vir + read_pos, read_len);
#else
	__inval_dcache_area(ccci_sib.base_ap_view_vir + read_pos, read_len);
#endif
	if (copy_to_user(buf,
			ccci_sib.base_ap_view_vir + read_pos,
			read_len)) {
		pr_notice("[ccci0/util]ccci_sib: copy_to_user fail\n");
		return -EFAULT;
	}
	*read_ptr = read_pos + read_len;

	return read_len;
}

unsigned int ccci_sib_poll(struct file *fp, struct poll_table_struct *poll)
{
	unsigned int *read_ptr;
	unsigned int read_pos;

	read_ptr = fp->private_data;
	read_pos = *read_ptr;

	if (ccci_sib.size == read_pos)
		return 0;

	return POLLIN | POLLRDNORM;
}

static int ccci_sib_open(struct inode *inode, struct file *file)
{
	unsigned int *read_ptr;

	read_ptr = kzalloc(sizeof(unsigned int), GFP_KERNEL);
	if (read_ptr == NULL)
		return -1;

	file->private_data = read_ptr;

	return 0;
}

static int ccci_sib_close(struct inode *inode, struct file *file)
{
	unsigned int *read_ptr;

	read_ptr = file->private_data;
	kfree(read_ptr);

	return 0;
}

static const struct proc_ops ccci_sib_fops = {
	.proc_read = ccci_sib_read,
	.proc_open = ccci_sib_open,
	.proc_release = ccci_sib_close,
	.proc_poll = ccci_sib_poll,
};

static void ccci_sib_smem_remap(void)
{
	phys_addr_t md_sib_mem_addr;
	unsigned int md_sib_mem_size;

	get_md_sib_mem_info(&md_sib_mem_addr, &md_sib_mem_size);
	ccci_sib.base_ap_view_phy = md_sib_mem_addr;
	ccci_sib.size = md_sib_mem_size;
	ccci_sib.base_ap_view_vir =
		ccci_map_phy_addr(md_sib_mem_addr, md_sib_mem_size);
	ccci_sib.base_md_view_phy = 0;
	pr_notice(
		"[ccci0/util]md sib mem info: (0x%llx 0x%llx %p %d)\n",
		(unsigned long long)ccci_sib.base_ap_view_phy,
		(unsigned long long)ccci_sib.base_md_view_phy,
		ccci_sib.base_ap_view_vir,
		ccci_sib.size);
}

void ccci_sib_init(void)
{
	struct proc_dir_entry *ccci_sib_proc;

	ccci_sib_proc = proc_create("ccci_sib", 0444, NULL, &ccci_sib_fops);
	if (ccci_sib_proc == NULL) {
		pr_notice("[ccci0/util]fail to create proc entry for sib dump\n");
		return;
	}
	ccci_sib_smem_remap();
}
