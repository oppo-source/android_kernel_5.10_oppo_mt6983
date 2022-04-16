// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#include <linux/seq_file.h>
#include <drivers/android/binder_internal.h>

#include "sched_assist.h"
#include "sa_common.h"
#include "sa_binder.h"


static inline void binder_set_inherit_ux(struct task_struct *thread_task, struct task_struct *from_task, bool sync)
{
	int from_depth = oplus_get_ux_depth(from_task);
	int from_state = oplus_get_ux_state(from_task);

	if (from_task && test_set_inherit_ux(from_task)) {
		if (!test_task_ux(thread_task))
			set_inherit_ux(thread_task, INHERIT_UX_BINDER, from_depth, from_state);
		else
			reset_inherit_ux(thread_task, from_task, INHERIT_UX_BINDER);
	}  else if (from_task && test_task_is_rt(from_task)) { /* rt trans can be set as ux if binder thread is cfs class */
		if (!test_task_ux(thread_task)) {
			int ux_value = SA_TYPE_LIGHT;
			set_inherit_ux(thread_task, INHERIT_UX_BINDER, from_depth, ux_value);
		}
	}
}

static inline void binder_unset_inherit_ux(struct task_struct *thread_task)
{
	if (test_inherit_ux(thread_task, INHERIT_UX_BINDER)) {
		unset_inherit_ux(thread_task, INHERIT_UX_BINDER);
	}
}

/* implement vender hook in driver/android/binder.c */
void android_vh_binder_wakeup_ilocked_handler(void *unused, struct task_struct *task, bool sync, struct binder_proc *proc)
{
}

void android_vh_binder_restore_priority_handler(void *unused, struct binder_transaction *t, struct task_struct *task)
{
	if (unlikely(!global_sched_assist_enabled))
		return;

	if (t != NULL) {
		binder_unset_inherit_ux(task);
	}
}

void android_vh_binder_wait_for_work_handler(void *unused,
			bool do_proc_work, struct binder_thread *tsk, struct binder_proc *proc)
{
	if (unlikely(!global_sched_assist_enabled))
		return;

	if (do_proc_work) {
		binder_unset_inherit_ux(tsk->task);
	}
}

void android_vh_sync_txn_recvd_handler(void *unused, struct task_struct *tsk, struct task_struct *from)
{
	if (unlikely(!global_sched_assist_enabled))
		return;

	binder_set_inherit_ux(tsk, from, false);
}

#ifdef CONFIG_OPLUS_BINDER_PRIO_SKIP
void android_vh_binder_priority_skip_handler(void *unused, struct task_struct *task, bool *skip)
{
	if (task->prio < MAX_RT_PRIO)
		*skip = true;
}
#endif

void android_vh_binder_proc_transaction_end_handler(void *unused, struct task_struct *caller_task, struct task_struct *binder_proc_task,
		struct task_struct *binder_th_task, unsigned int code,
		bool pending_async, bool sync)
{
	bool set_ux = sync;
	struct task_struct *grp_leader = NULL;

	if (unlikely(!global_sched_assist_enabled))
		return;

	if (!binder_th_task)
		return;

	grp_leader = binder_th_task->group_leader;
	if (grp_leader) {
		struct oplus_task_struct *ots = get_oplus_task_struct(current);
		if ((ots->im_flag == IM_FLAG_SURFACEFLINGER) && !sync && test_task_ux(grp_leader)) {
			set_ux = true;
		}
	}

	if (set_ux) {
		binder_set_inherit_ux(binder_th_task, current, sync);
	}

	if (unlikely(global_debug_enabled & DEBUG_FTRACE)) {
		trace_printk("caller_task(comm=%-12s pid=%d tgid=%d) binder_proc_task(comm=%-12s pid=%d tgid=%d) binder_th_task(comm=%-12s pid=%d tgid=%d) code=%d sync=%d\n",
			caller_task->comm, caller_task->pid, caller_task->tgid,
			binder_proc_task->comm, binder_proc_task->pid, binder_proc_task->tgid,
			binder_th_task->comm, binder_th_task->pid, binder_th_task->tgid,
			code, sync);
	}
}
