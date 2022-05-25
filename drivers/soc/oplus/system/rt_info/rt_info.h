/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _RT_INFO_H_
#define _RT_INFO_H_

#include <linux/sched.h>
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
#include "../../../../../kernel_platform/common/kernel/sched/sched.h"
#include "../../../../../kernel_platform/common/drivers/android/binder_internal.h"
#include "../../../../../kernel_platform/common/drivers/android/binder_alloc.h"
#else
#include "../../../../../kernel-5.10/kernel/sched/sched.h"
#include "../../../../../kernel-5.10/drivers/android/binder_internal.h"
#include "../../../../../kernel-5.10/drivers/android/binder_alloc.h"
#endif

#define MAX_RT_NUM         8

#ifdef RT_INFO_DEBUG
#define rt_err(fmt, ...) \
            printk_deferred(KERN_ERR "[RT_INFO][%s]"fmt, __func__, ##__VA_ARGS__)
#else
#define rt_err(fmt, ...)
#endif

#endif /*_RT_INFO_H_*/