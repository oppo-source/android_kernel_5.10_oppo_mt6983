// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sync_file.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/kallsyms.h>
//#include "sync_debug.h"

#define CREATE_TRACE_POINTS
//#include "sync_trace.h"

/*
 * MDP SYNC validation framework
 *
 * A sync object driver that uses a 32bit counter to coordinate
 * synchronization.  Useful when there is no hardware primitive backing
 * the synchronization.
 *
 * To start the framework just open:
 *
 * dev/mdp_sync
 *
 * That will create a sync timeline, all fences created under this timeline
 * file descriptor will belong to the this timeline.
 *
 * The 'mdp_sync' file can be opened many times as to create different
 * timelines.
 *
 * Fences can be created with MDP_SYNC_IOC_CREATE_FENCE ioctl with struct
 * mdp_sync_ioctl_create_fence as parameter.
 *
 * To increment the timeline counter, MDP_SYNC_IOC_INC ioctl should be used
 * with the increment as u32. This will update the last signaled value
 * from the timeline and signal any fence that has a seqno smaller or equal
 * to it.
 *
 * struct mdp_sync_ioctl_create_fence
 * @value:	the seqno to initialise the fence with
 * @name:	the name of the new sync point
 * @fence:	return the fd of the new sync_file with the created fence
 */
struct mdp_sync_create_fence_data {
	__u32	value;
	char	name[32];
	__s32	dma_fence; /* fd of new fence */
};

#define MDP_SYNC_IOC_MAGIC	'W'

#define MDP_SYNC_IOC_CREATE_FENCE	_IOWR(MDP_SYNC_IOC_MAGIC, 0,\
		struct mdp_sync_create_fence_data)

#define MDP_SYNC_IOC_INC	_IOW(MDP_SYNC_IOC_MAGIC, 1, __u32)

/**
 * struct sync_timeline - sync object
 * @kref:		reference count on fence.
 * @name:		name of the sync_timeline. Useful for debugging
 * @lock:		lock protecting @pt_list and @value
 * @pt_tree:		rbtree of active (unsignaled/errored) sync_pts
 * @pt_list:		list of active (unsignaled/errored) sync_pts
 * @sync_timeline_list:	membership in global sync_timeline_list
 */
struct sync_timeline {
	struct kref		kref;
	char			name[32];

	/* protected by lock */
	u64			context;
	int			value;

	struct rb_root		pt_tree;
	struct list_head	pt_list;
	spinlock_t		lock;

	struct list_head	sync_timeline_list;
};

static inline struct sync_timeline *fence_parent(struct dma_fence *dma_fence)
{
	return container_of(dma_fence->lock, struct sync_timeline, lock);
}

/**
 * struct sync_pt - sync_pt object
 * @base: base dma_fence object
 * @link: link on the sync timeline's list
 * @node: node in the sync timeline's tree
 */
struct sync_pt {
	struct dma_fence base;
	struct list_head link;
	struct rb_node node;
};

static const struct dma_fence_ops timeline_dma_fence_ops;

static inline struct sync_pt *fence_to_sync_pt(struct dma_fence *dma_fence)
{
	if (dma_fence->ops != &timeline_dma_fence_ops)
		return NULL;
	return container_of(dma_fence, struct sync_pt, base);
}

/**
 * mdp_timeline_create() - creates a sync object
 * @name:	sync_timeline name
 *
 * Creates a new sync_timeline. Returns the sync_timeline object or NULL in
 * case of error.
 */
struct sync_timeline *mdp_timeline_create(const char *name)
{
	struct sync_timeline *obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return NULL;

	kref_init(&obj->kref);
	obj->context = dma_fence_context_alloc(1);
	strlcpy(obj->name, name, sizeof(obj->name));

	obj->pt_tree = RB_ROOT;
	INIT_LIST_HEAD(&obj->pt_list);
	spin_lock_init(&obj->lock);

	//sync_timeline_debug_add(obj);

	return obj;
}

static void sync_timeline_free(struct kref *kref)
{
	struct sync_timeline *obj =
		container_of(kref, struct sync_timeline, kref);
	//sync_timeline_debug_remove(obj);

	kfree(obj);
}

static void sync_timeline_get(struct sync_timeline *obj)
{
	kref_get(&obj->kref);
}

static void sync_timeline_put(struct sync_timeline *obj)
{
	kref_put(&obj->kref, sync_timeline_free);
}

static const char *timeline_fence_get_driver_name(struct dma_fence *dma_fence)
{
	return "mdp_sync";
}

static const char *timeline_fence_get_timeline_name(struct dma_fence *dma_fence)
{
	struct sync_timeline *parent = fence_parent(dma_fence);

	return parent->name;
}

static void timeline_fence_release(struct dma_fence *dma_fence)
{
	struct sync_pt *pt = fence_to_sync_pt(dma_fence);
	struct sync_timeline *parent = fence_parent(dma_fence);
	unsigned long flags;

	spin_lock_irqsave(dma_fence->lock, flags);
	if (pt && !list_empty(&pt->link)) {
		list_del(&pt->link);
		rb_erase(&pt->node, &parent->pt_tree);
	}
	spin_unlock_irqrestore(dma_fence->lock, flags);

	sync_timeline_put(parent);
	dma_fence_free(dma_fence);
}

static bool timeline_fence_signaled(struct dma_fence *dma_fence)
{
	struct sync_timeline *parent = fence_parent(dma_fence);

	return !__dma_fence_is_later(dma_fence->seqno,
		parent->value, dma_fence->ops);
}

static bool timeline_fence_enable_signaling(struct dma_fence *dma_fence)
{
	return true;
}

static void timeline_fence_value_str(struct dma_fence *dma_fence,
				    char *str, int size)
{
	snprintf(str, size, "%llu", dma_fence->seqno);
}

static void timeline_fence_timeline_value_str(struct dma_fence *dma_fence,
					     char *str, int size)
{
	struct sync_timeline *parent = fence_parent(dma_fence);

	snprintf(str, size, "%d", parent->value);
}

static const struct dma_fence_ops timeline_dma_fence_ops = {
	.get_driver_name = timeline_fence_get_driver_name,
	.get_timeline_name = timeline_fence_get_timeline_name,
	.enable_signaling = timeline_fence_enable_signaling,
	.signaled = timeline_fence_signaled,
	.wait = dma_fence_default_wait,
	.release = timeline_fence_release,
	.fence_value_str = timeline_fence_value_str,
	.timeline_value_str = timeline_fence_timeline_value_str,
};

/**
 * sync_timeline_signal() - signal a status change on a sync_timeline
 * @obj:	sync_timeline to signal
 * @inc:	num to increment on timeline->value
 *
 * A sync implementation should call this any time one of it's fences
 * has signaled or has an error condition.
 */
static void sync_timeline_signal(struct sync_timeline *obj, unsigned int inc)
{
	struct sync_pt *pt, *next;

	//trace_sync_timeline(obj);

	spin_lock_irq(&obj->lock);

	obj->value += inc;

	list_for_each_entry_safe(pt, next, &obj->pt_list, link) {
		if (!timeline_fence_signaled(&pt->base))
			break;

		list_del_init(&pt->link);
		rb_erase(&pt->node, &obj->pt_tree);

	/*
	 * A signal callback may release the last reference to this
	 * dma_fence, causing it to be freed. That operation has to be
	 * last to avoid a use after free inside this loop, and must
	 * be after we remove the dma_fence from the timeline in order to
	 * prevent deadlocking on timeline->lock inside
	 * timeline_fence_release().
	 */
		dma_fence_signal_locked(&pt->base);
	}

	spin_unlock_irq(&obj->lock);
}

/**
 * sync_pt_create() - creates a sync pt
 * @parent:	dma_fence's parent sync_timeline
 * @inc:	value of the dma_fence
 *
 * Creates a new sync_pt as a child of @parent.  @size bytes will be
 * allocated allowing for implementation specific data to be kept after
 * the generic sync_timeline struct. Returns the sync_pt object or
 * NULL in case of error.
 */
static struct sync_pt *sync_pt_create(struct sync_timeline *obj,
					  unsigned int value)
{
	struct sync_pt *pt;

	pt = kzalloc(sizeof(*pt), GFP_KERNEL);
	if (!pt)
		return NULL;

	sync_timeline_get(obj);
	dma_fence_init(&pt->base, &timeline_dma_fence_ops, &obj->lock,
		   obj->context, value);
	INIT_LIST_HEAD(&pt->link);

	spin_lock_irq(&obj->lock);
	if (!dma_fence_is_signaled_locked(&pt->base)) {
		struct rb_node **p = &obj->pt_tree.rb_node;
		struct rb_node *parent = NULL;

		while (*p) {
			struct sync_pt *other;
			int cmp;

			parent = *p;
			other = rb_entry(parent, typeof(*pt), node);
			cmp = value - other->base.seqno;
			if (cmp > 0) {
				p = &parent->rb_right;
			} else if (cmp < 0) {
				p = &parent->rb_left;
			} else {
				if (dma_fence_get_rcu(&other->base)) {
					sync_timeline_put(obj);
					kfree(pt);
					pt = other;
					goto unlock;
				}
				p = &parent->rb_left;
			}
		}
		rb_link_node(&pt->node, parent, p);
		rb_insert_color(&pt->node, &obj->pt_tree);

		parent = rb_next(&pt->node);
		list_add_tail(&pt->link,
			      parent ? &rb_entry(parent, typeof(*pt),
				node)->link : &obj->pt_list);
	}
unlock:
	spin_unlock_irq(&obj->lock);

	return pt;
}

/*
 * *WARNING*
 *
 * improper use of this can result in deadlocking kernel drivers from userspace.
 */

/* opening mdp_sync create a new sync obj */
static int mdp_sync_open(struct inode *inode, struct file *file)
{
	struct sync_timeline *obj;
	char task_comm[TASK_COMM_LEN];

	get_task_comm(task_comm, current);

	obj = mdp_timeline_create(task_comm);
	if (!obj)
		return -ENOMEM;

	file->private_data = obj;

	return 0;
}

static int mdp_sync_release(struct inode *inode, struct file *file)
{
	struct sync_timeline *obj = file->private_data;
	struct sync_pt *pt, *next;

	spin_lock_irq(&obj->lock);

	list_for_each_entry_safe(pt, next, &obj->pt_list, link) {
		dma_fence_set_error(&pt->base, -ENOENT);
		dma_fence_signal_locked(&pt->base);
	}

	spin_unlock_irq(&obj->lock);

	//smp_wmb();/*memory barrier*/

	sync_timeline_put(obj);
	return 0;
}

static long mdp_sync_ioctl_create_fence(struct sync_timeline *obj,
				       unsigned long arg)
{
	int fd = get_unused_fd_flags(O_CLOEXEC);
	int err;
	struct sync_pt *pt;
	struct sync_file *sync_file;
	struct mdp_sync_create_fence_data data;

	if (fd < 0)
		return fd;

	if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
		err = -EFAULT;
		goto err;
	}

	pt = sync_pt_create(obj, data.value);
	if (!pt) {
		err = -ENOMEM;
		goto err;
	}

	sync_file = sync_file_create(&pt->base);
	dma_fence_put(&pt->base);
	if (!sync_file) {
		err = -ENOMEM;
		goto err;
	}

	data.dma_fence = fd;
	if (copy_to_user((void __user *)arg, &data, sizeof(data))) {
		fput(sync_file->file);
		err = -EFAULT;
		goto err;
	}

	fd_install(fd, sync_file->file);
	return 0;

err:
	put_unused_fd(fd);
	return err;
}

static long mdp_sync_ioctl_inc(struct sync_timeline *obj, unsigned long arg)
{
	u32 value;

	if (copy_from_user(&value, (void __user *)arg, sizeof(value)))
		return -EFAULT;

	while (value > INT_MAX)  {
		sync_timeline_signal(obj, INT_MAX);
		value -= INT_MAX;
	}

	sync_timeline_signal(obj, value);
	return 0;
}

static long mdp_sync_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct sync_timeline *obj = file->private_data;

	switch (cmd) {
	case MDP_SYNC_IOC_CREATE_FENCE:
		return mdp_sync_ioctl_create_fence(obj, arg);

	case MDP_SYNC_IOC_INC:
		return mdp_sync_ioctl_inc(obj, arg);

	default:
		return -ENOTTY;
	}
}

const struct file_operations mdp_sync_fops = {
	.open           = mdp_sync_open,
	.release        = mdp_sync_release,
	.unlocked_ioctl = mdp_sync_ioctl,
	.compat_ioctl	= mdp_sync_ioctl,
};


static struct miscdevice mdp_sync_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mdp_sync",
	.fops	= &mdp_sync_fops,
};

int mdp_sync_device_init(void)
{
	return misc_register(&mdp_sync_dev);
}

MODULE_LICENSE("GPL");
