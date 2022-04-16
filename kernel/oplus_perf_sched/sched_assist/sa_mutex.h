/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SA_MUTEX_H_
#define _OPLUS_SA_MUTEX_H_

extern void android_vh_alter_mutex_list_add_handler(void *unused, struct mutex *lock,
	struct mutex_waiter *waiter, struct list_head *list, bool *already_on_list);
extern void android_vh_mutex_wait_start_handler(void *unused, struct mutex *lock);
extern void android_vh_mutex_wait_finish_handler(void *unused, struct mutex *lock);
extern void android_vh_mutex_unlock_slowpath_handler(void *unused, struct mutex *lock);


#endif /* _OPLUS_SA_MUTEX_H_ */
