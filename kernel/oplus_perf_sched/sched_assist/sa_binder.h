/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SA_BINDER_H_
#define _OPLUS_SA_BINDER_H_


extern void android_vh_binder_wakeup_ilocked_handler(void *unused, struct task_struct *task, bool sync, struct binder_proc *proc);
extern void android_vh_binder_restore_priority_handler(void *unused, struct binder_transaction *t, struct task_struct *task);
extern void android_vh_binder_wait_for_work_handler(void *unused,
			bool do_proc_work, struct binder_thread *tsk, struct binder_proc *proc);
extern void android_vh_sync_txn_recvd_handler(void *unused, struct task_struct *tsk, struct task_struct *from);
#ifdef CONFIG_OPLUS_BINDER_PRIO_SKIP
extern void android_vh_binder_priority_skip_handler(void *unused, struct task_struct *task, bool *skip);
#endif
extern void android_vh_binder_proc_transaction_end_handler(void *unused, struct task_struct *caller_task, struct task_struct *binder_proc_task,
		struct task_struct *binder_th_task, unsigned int code,
		bool pending_async, bool sync);
#endif /* _OPLUS_SA_BINDER_H_ */
