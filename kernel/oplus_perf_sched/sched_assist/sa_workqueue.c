// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#include "sched_assist.h"
#include "sa_common.h"
#include "sa_workqueue.h"


/* register vender hook in driver/android/wurkqueue.c */
/* struct worker; */
void android_vh_create_worker_handler(void *unused, struct worker *worker, struct workqueue_attrs *attrs)
{
}

