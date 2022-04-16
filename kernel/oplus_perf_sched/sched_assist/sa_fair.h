/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SA_FAIR_H_
#define _OPLUS_SA_FAIR_H_

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
#define SA_CGROUP_DEFAULT			(1)
#define SA_CGROUP_FOREGROUND		(2)
#define SA_CGROUP_BACKGROUND		(3)
#define SA_CGROUP_TOP_APP			(4)

/* define for load balance task type */
#define SA_HIGH_LOAD 	1
#define SA_LOW_LOAD		0

#define SA_INVALID      (-1)
#define SA_UX			1
#define SA_TOP			2
#define SA_FG			3
#define SA_BG			4

struct task_count_rq{
	int ux_high;
	int ux_low;
	int top_high;
	int top_low;
	int foreground_high;
	int foreground_low;
	int background_high;
	int background_low;
};
DECLARE_PER_CPU(struct task_count_rq, task_lb_count);
extern struct cpumask nr_mask;
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

extern void task_tpd_mask(struct task_struct *tsk, cpumask_t *request);
extern bool task_tpd_check(struct task_struct *tsk, int dst_cpu);

extern bool should_ux_task_skip_cpu(struct task_struct *task, unsigned int dst_cpu);
extern bool should_ux_task_skip_eas(struct task_struct *p);
extern bool set_ux_task_to_prefer_cpu(struct task_struct *task, int *orig_target_cpu);
extern void oplus_replace_next_task_fair(struct rq *rq, struct task_struct **p, struct sched_entity **se, bool *repick, bool simple);
extern void oplus_check_preempt_wakeup(struct rq *rq, struct task_struct *p, bool *preempt, bool *nopreempt);

#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
extern void init_rq_cpu(int cpu);
extern int task_lb_sched_type(struct task_struct *tsk);
extern void dec_task_lb(struct task_struct *tsk, struct rq *rq,
	int high_load, int task_type);
extern void inc_task_lb(struct task_struct *tsk, struct rq *rq,
	int high_load, int task_type);
extern void inc_ld_stats(struct task_struct *tsk, struct rq *rq);
extern void dec_ld_stats(struct task_struct *tsk, struct rq *rq);
extern void update_load_flag(struct task_struct *tsk, struct rq *rq);

extern int task_lb_sched_type(struct task_struct *tsk);
extern void sched_assist_spread_tasks(struct task_struct *p, cpumask_t new_allowed_cpus,
		int order_index, int end_index, int skip_cpu, cpumask_t *cpus, bool strict);
extern bool should_force_spread_tasks(void);
extern void update_rq_nr_imbalance(int cpu);
extern bool is_spread_task_enabled(void);
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */

/* register vender hook in kernel/sched/fair.c */
extern void android_rvh_place_entity_handler(void *unused, struct cfs_rq *cfs_rq, struct sched_entity *se, int initial, u64 vruntime);
extern void android_rvh_check_preempt_tick_handler(void *unused, struct task_struct *p, unsigned long *ideal_runtime);
extern void android_rvh_pick_next_entity_handler(void *unused, struct cfs_rq *cfs_rq, struct sched_entity *curr, struct sched_entity **se);
extern void android_rvh_check_preempt_wakeup_ignore_handler(void *unused, struct task_struct *p, bool *ignore);
extern void android_rvh_check_preempt_wakeup_handler(void *unused, struct rq *rq, struct task_struct *p, bool *preempt, bool *nopreempt,
	int wake_flags, struct sched_entity *se, struct sched_entity *pse, int next_buddy_marked, unsigned int granularity);
extern void android_rvh_replace_next_task_fair_handler(void *unused, struct rq *rq, struct task_struct **p, struct sched_entity **se,
	bool *repick, bool simple, struct task_struct *prev);
extern void android_rvh_can_migrate_task_handler(void *unused, struct task_struct *p, int dst_cpu, int *can_migrate);

#endif /* _OPLUS_SA_FAIR_H_ */
