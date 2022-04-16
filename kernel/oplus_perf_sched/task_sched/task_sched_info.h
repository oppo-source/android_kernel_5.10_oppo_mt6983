/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 Oplus. All rights reserved.
 */
#ifndef _OPLUS_TASK_SCHED_INFO_H
#define _OPLUS_TASK_SCHED_INFO_H

#define sched_err(fmt, ...) \
		printk(KERN_ERR "[SCHED_INFO_ERR][%s]"fmt, __func__, ##__VA_ARGS__)

enum {
	task_sched_info_running = 0,
	task_sched_info_runnable,
	task_sched_info_IO,
	task_sched_info_D,
	task_sched_info_S,
	task_sched_info_freq,
	task_sched_info_freq_limit,
	task_sched_info_isolate,
	task_sched_info_backtrace,
};

enum {
	block_runnable = 1,
	running_runnable,
};

enum {
	cpu_unisolate = 0,
	cpu_isolate,
};

struct task_sched_info {
	u64 sched_info_one;
	u64 sched_info_two;
};

extern int sched_info_init(void);
extern int register_sched_info_vendor_hooks(void);
extern void update_cpu_isolate_info(int cpu, u64 type);
extern void update_cpus_isolate_info(struct cpumask *cpus, u64 type);

#endif /* _OPLUS_TASK_SCHED_INFO_H */

