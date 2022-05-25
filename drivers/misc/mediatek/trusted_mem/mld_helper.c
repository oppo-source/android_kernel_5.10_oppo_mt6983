// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PR_FMT_HEADER_MUST_BE_INCLUDED_BEFORE_ALL_HDRS
#include "private/tmem_pr_fmt.h" PR_FMT_HEADER_MUST_BE_INCLUDED_BEFORE_ALL_HDRS

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include "private/mld_helper.h"
#include "private/tmem_utils.h"

struct tcore_mem_list_item {
	void *mem_ptr;
	size_t mem_size;
	struct list_head list;
};

struct tcore_mem_list_context {
	size_t malloc_total_size;
	struct tcore_mem_list_item tcore_list;
	struct mutex lock;
};

struct tcore_mem_list_context g_mld_context;

#define MLD_LOCK() mutex_lock(&g_mld_context.lock)
#define MLD_UNLOCK() mutex_unlock(&g_mld_context.lock)

void mld_init(void)
{
	pr_info("TMEM_MLD_DETECTION_ENABLED\n");

	INIT_LIST_HEAD(&g_mld_context.tcore_list.list);
	mutex_init(&g_mld_context.lock);
	g_mld_context.malloc_total_size = 0;
}

static void mld_create(size_t size, void *mem_ptr)
{
	struct tcore_mem_list_item *mem_item;

	mem_item = kmalloc(sizeof(struct tcore_mem_list_item), GFP_KERNEL);
	if (INVALID(mem_item)) {
		pr_err("%s:%d out of memory!\n", __func__, __LINE__);
		return;
	}

	mem_item->mem_size = size;
	mem_item->mem_ptr = mem_ptr;

	MLD_LOCK();

	list_add_tail(&(mem_item->list), &g_mld_context.tcore_list.list);
	g_mld_context.malloc_total_size += size;

	MLD_UNLOCK();
}

static void mld_destroy(const void *mem_ptr)
{
	struct tcore_mem_list_item *mem_item, *tmp;

	MLD_LOCK();

	/* clang-format off */
	list_for_each_entry_safe(mem_item, tmp,
				 &g_mld_context.tcore_list.list, list) {
		/* clang-format on */
		if (mem_item->mem_ptr == mem_ptr) {
			list_del(&mem_item->list);
			if (g_mld_context.malloc_total_size
			    < mem_item->mem_size)
				pr_err("%s:%d system error! (%zx < %zx)\n",
				       __func__, __LINE__,
				       g_mld_context.malloc_total_size,
				       mem_item->mem_size);
			else
				g_mld_context.malloc_total_size -=
					mem_item->mem_size;
			kfree(mem_item);
		}
	}

	MLD_UNLOCK();
}

void *mld_kmalloc(size_t size, gfp_t flags)
{
	void *mem_ptr;

	mem_ptr = kmalloc(size, flags);
	if (VALID(mem_ptr))
		mld_create(size, mem_ptr);

	return mem_ptr;
}

void mld_kfree(const void *mem_ptr)
{
	if (VALID(mem_ptr))
		mld_destroy(mem_ptr);

	kfree(mem_ptr);
}

size_t mld_stamp(void)
{
	size_t current_size;

	MLD_LOCK();
	current_size = g_mld_context.malloc_total_size;
	MLD_UNLOCK();

	return current_size;
}

enum MLD_CHECK_STATUS mld_stamp_check(size_t previous_stamped_size)
{
	size_t current_size;

	MLD_LOCK();
	current_size = g_mld_context.malloc_total_size;
	MLD_UNLOCK();

	if (previous_stamped_size == current_size) {
		pr_debug("[MLD_CHECK] pass: 0x%zx\n", current_size);
		return MLD_CHECK_PASS;
	}

	pr_err("[MLD_CHECK] previous: 0x%zx\n", previous_stamped_size);
	pr_err("[MLD_CHECK] current: 0x%zx\n", current_size);
	pr_err("[MLD_CHECK] diff: 0x%zx\n",
	       (current_size - previous_stamped_size));
	pr_err("[MLD_CHECK] memory leak is detected!\n");

	return MLD_CHECK_FAIL;
}
