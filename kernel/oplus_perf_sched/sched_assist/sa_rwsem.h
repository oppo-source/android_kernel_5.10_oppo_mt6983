/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SA_RWSEM_H_
#define _OPLUS_SA_RWSEM_H_

extern bool oplus_rwsem_list_add(struct task_struct *tsk, struct list_head *entry, struct list_head *head);
extern bool rwsem_list_add_skip_ux(struct task_struct *in_tsk, struct task_struct *tsk);

/* register vender hook in kernel/locking/rwsem.c */
extern void android_vh_alter_rwsem_list_add_handler(void *unused, struct rwsem_waiter *waiter, struct rw_semaphore *sem, bool *already_on_list);
extern void android_vh_rwsem_wake_handler(void *unused, struct rw_semaphore *sem);
extern void android_vh_rwsem_wake_finish_handler(void *unused, struct rw_semaphore *sem);

#endif /* _OPLUS_SA_RWSEM_H_ */
