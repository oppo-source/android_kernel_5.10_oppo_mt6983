/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SA_COMMON_H_
#define _OPLUS_SA_COMMON_H_

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/types.h>
#include <asm/atomic.h>
#include <linux/hashtable.h>
#if IS_ENABLED(CONFIG_SCHED_WALT)
#include <linux/sched/walt.h>
#endif


#define ux_err(fmt, ...) \
		printk(KERN_ERR "[sched_assist_error][%s]"fmt, __func__, ##__VA_ARGS__)
#define ux_warn(fmt, ...) \
		printk(KERN_WARNING "[sched_assist_warn][%s]"fmt, __func__, ##__VA_ARGS__)
#define ux_debug(fmt, ...) \
		printk(KERN_INFO "[sched_assist_info][%s]"fmt, __func__, ##__VA_ARGS__)

#define UX_MSG_LEN					64
#define UX_DEPTH_MAX				5

/* define for debug */
#define DEBUG_SYSTRACE (1 << 0)
#define DEBUG_FTRACE   (1 << 1)

/* define for sched assist feature */
#define FEATURE_COMMON (1 << 0)
#define FEATURE_SPREAD (1 << 1)

#define UX_EXEC_SLICE (4000000U)

/* define for sched assist thread type, keep same as the define in java file */
#define SA_OPT_CLEAR				(0)
#define SA_TYPE_LIGHT				(1 << 0)
#define SA_TYPE_HEAVY				(1 << 1)
#define SA_TYPE_ANIMATOR			(1 << 2)
#define SA_TYPE_LISTPICK 			(1 << 3)
#define SA_TYPE_ONCE				(1 << 4) /* clear ux type when dequeue */
#define SA_OPT_SET				(1 << 7)
#define SA_TYPE_INHERIT				(1 << 8)

#define SCHED_ASSIST_UX_MASK		(0xFF)

/* define for sched assist scene type, keep same as the define in java file */
#define SA_SCENE_OPT_CLEAR			(0)
#define SA_LAUNCH					(1 << 0)
#define SA_SLIDE					(1 << 1)
#define SA_CAMERA					(1 << 2)
#define SA_ANIM_START				(1 << 3) /* we care about both launcher and top app */
#define SA_ANIM						(1 << 4) /* we only care about launcher */
#define SA_INPUT					(1 << 5)
#define SA_LAUNCHER_SI				(1 << 6)
#define SA_SCENE_OPT_SET			(1 << 7)

extern pid_t save_audio_tgid;
extern pid_t save_top_app_tgid;
extern unsigned int top_app_type;

/* define for boost threshold unit */
#define BOOST_THRESHOLD_UNIT (51)

enum UX_STATE_TYPE {
	UX_STATE_INVALID = 0,
	UX_STATE_NONE,
	UX_STATE_SCHED_ASSIST,
	UX_STATE_INHERIT,
	MAX_UX_STATE_TYPE,
};

enum INHERIT_UX_TYPE {
	INHERIT_UX_BINDER = 0,
	INHERIT_UX_RWSEM,
	INHERIT_UX_MUTEX,
	INHERIT_UX_MAX,
};

/* new flag should be add before MAX_IM_FLAG_TYPE, never change the value of those existed flag type. */
enum IM_FLAG_TYPE {
	IM_FLAG_NONE = 0,
	IM_FLAG_SURFACEFLINGER,
	IM_FLAG_HWC,
	IM_FLAG_RENDERENGINE,
	IM_FLAG_WEBVIEW,
	IM_FLAG_CAMERA_HAL,
	MAX_IM_FLAG_TYPE,
};

DECLARE_PER_CPU(struct list_head, ux_thread_list);

struct ux_sched_cluster {
	struct cpumask cpus;
	unsigned long capacity;
};

#define OPLUS_NR_CPUS (8)
struct ux_sched_cputopo {
	int cls_nr;
	struct ux_sched_cluster sched_cls[OPLUS_NR_CPUS];
};

/* Please add your own members of task_struct here :) */
struct oplus_task_struct {
	struct list_head ux_entry;
	atomic64_t inherit_ux;
	u64 enqueue_time; /* remove */
	u64 inherit_ux_start; /* remove */
	u64 sum_exec_baseline;
	u64 total_exec;
	int ux_state;
	int ux_depth;
	int im_flag;
	int tpd; /* task placement decision */
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	int lb_state;
	int ld_flag;
#endif /* CONFIG_OPLUS_FEATURE_SCHED_SPREAD */
	u64 exec_calc_runtime;
	int is_update_runtime;
	int target_process;
	u64 wake_tid;
	u64 running_start_time;
};

struct oplus_rq {
	struct list_head ux_list;
};

extern int global_debug_enabled;
extern int global_sched_assist_enabled;
extern int global_sched_assist_scene;

struct rq;

/* attention: before insert .ko, task's list->prev/next will be init with 0 */
static inline bool oplus_list_empty(struct list_head *list)
{
	return list_empty(list) || (list->prev == 0 && list->next == 0);
}

#define ots_to_ts(ots) ({ \
		void *__mptr = (void *)(ots); \
		((struct task_struct *)(__mptr - \
		offsetof(struct task_struct, android_oem_data1))); })

static inline struct oplus_task_struct *get_oplus_task_struct(struct task_struct *t)
{
	return (struct oplus_task_struct *) t->android_oem_data1;
}

static inline int oplus_get_im_flag(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return ots->im_flag;
}

static inline void oplus_set_im_flag(struct task_struct *t, int im_flag)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	ots->im_flag = im_flag;
}

static inline int oplus_get_ux_state(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return ots->ux_state;
}

static inline void oplus_set_ux_state(struct task_struct *t, int ux_state)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	ots->ux_state = ux_state;
}

static inline s64 oplus_get_inherit_ux(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return atomic64_read(&ots->inherit_ux);
}

static inline void oplus_set_inherit_ux(struct task_struct *t, s64 inherit_ux)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	atomic64_set(&ots->inherit_ux, inherit_ux);
}


static inline struct list_head *oplus_get_ux_entry(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return &ots->ux_entry;
}

static inline int oplus_get_ux_depth(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return ots->ux_depth;
}

static inline void oplus_set_ux_depth(struct task_struct *t, int ux_depth)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	ots->ux_depth = ux_depth;
}

static inline u64 oplus_get_enqueue_time(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return ots->enqueue_time;
}

static inline void oplus_set_enqueue_time(struct task_struct *t, u64 enqueue_time)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	ots->enqueue_time = enqueue_time;
}

static inline void oplus_set_inherit_ux_start(struct task_struct *t, u64 start_time)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	ots->inherit_ux_start = start_time;
}

static inline int oplus_get_tpd(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return (ots->tpd > 0) ? ots->tpd : 0;
}

static inline void oplus_set_tpd(struct task_struct *t, int tpd)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	ots->tpd = tpd;
}

static inline void init_task_ux_info(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	INIT_LIST_HEAD(&ots->ux_entry);
	ots->ux_state = 0;
	atomic64_set(&ots->inherit_ux, 0);
	ots->ux_depth = 0;
	ots->enqueue_time = 0;
	ots->inherit_ux_start = 0;
	ots->tpd = 0;
#ifdef CONFIG_OPLUS_FEATURE_SCHED_SPREAD
	ots->lb_state = 0;
	ots->ld_flag = 0;
#endif
	ots->exec_calc_runtime = 0;
	ots->is_update_runtime = 0;
	ots->target_process = -1;
	ots->wake_tid = 0;
	ots->running_start_time = 0;
}

static inline bool test_sched_assist_ux_type(struct task_struct *task, unsigned int sa_ux_type)
{
	return oplus_get_ux_state(task) & sa_ux_type;
}

static inline bool is_tpd_task(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return (ots->tpd > 0);
}

static inline bool is_heavy_ux_task(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return ots->ux_state & SA_TYPE_HEAVY;
}

static inline bool is_anim_ux_task(struct task_struct *t)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(t);

	return ots->ux_state & SA_TYPE_ANIMATOR;
}

static inline bool sched_assist_scene(unsigned int scene)
{
	if (unlikely(!global_sched_assist_enabled))
		return false;
	/*
	if (scene == SA_SLIDE)
		return sysctl_slide_boost_enabled;
	*/

	return global_sched_assist_scene & scene;
}

static inline unsigned long oplus_task_util(struct task_struct *p)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	struct walt_task_struct *wts = (struct walt_task_struct *) p->android_vendor_data1;

	return wts->demand_scaled;
#else
	return READ_ONCE(p->se.avg.util_avg);
#endif
}

#if IS_ENABLED(CONFIG_SCHED_WALT)
static inline u32 task_wts_sum(struct task_struct *tsk)
{
	struct walt_task_struct *wts = (struct walt_task_struct *) tsk->android_vendor_data1;
	return wts->sum;
}
#endif

extern void sched_assist_init_oplus_rq(void);
extern void queue_ux_thread(struct rq *rq, struct task_struct *p, int enqueue);

extern void inherit_ux_inc(struct task_struct *task, int type);
extern void inherit_ux_sub(struct task_struct *task, int type, int value);
extern void set_inherit_ux(struct task_struct *task, int type, int depth, int inherit_val);
extern void reset_inherit_ux(struct task_struct *inherit_task, struct task_struct *ux_task, int reset_type);
extern void unset_inherit_ux(struct task_struct *task, int type);
extern void unset_inherit_ux_value(struct task_struct *task, int type, int value);
extern void inc_inherit_ux_refs(struct task_struct *task, int type);

extern bool test_task_is_fair(struct task_struct *task);
extern bool test_task_is_rt(struct task_struct *task);

extern bool prio_higher(int a, int b);
extern bool test_task_ux(struct task_struct *task);
extern bool test_task_ux_depth(int ux_depth);
extern bool test_inherit_ux(struct task_struct *task, int type);
extern bool test_set_inherit_ux(struct task_struct *task);
extern bool test_task_identify_ux(struct task_struct *task, int id_type_ux);
extern bool test_list_pick_ux(struct task_struct *task);
extern int get_ux_state_type(struct task_struct *task);
extern void sched_assist_target_comm(struct task_struct *task, const char *buf);

extern void update_ux_sched_cputopo(void);
extern bool is_task_util_over(struct task_struct *tsk, int threshold);
extern bool oplus_task_misfit(struct task_struct *tsk, int cpu);
extern ssize_t oplus_show_cpus(const struct cpumask *mask, char *buf);
extern void adjust_rt_lowest_mask(struct task_struct *p, struct cpumask *local_cpu_mask, int ret, bool force_adjust);

extern void account_ux_runtime(struct rq *rq, struct task_struct *curr);

/* register vender hook in kernel/sched/topology.c */
extern void android_vh_build_sched_domains_handler(void *unused, bool has_asym);

/* register vender hook in kernel/sched/rt.c */
extern void android_rvh_select_task_rq_rt_handler(void *unused, struct task_struct *p, int prev_cpu, int sd_flag, int wake_flags, int *new_cpu);
extern void android_rvh_find_lowest_rq_handler(void *unused, struct task_struct *p, struct cpumask *local_cpu_mask, int ret, int *best_cpu);

/* register vender hook in kernel/sched/core.c */
extern void android_rvh_sched_fork_handler(void *unused, struct task_struct *p);
extern void android_rvh_enqueue_task_handler(void *unused, struct rq *rq, struct task_struct *p, int flags);
extern void android_rvh_dequeue_task_handler(void *unused, struct rq *rq, struct task_struct *p, int flags);
extern void android_rvh_schedule_handler(void *unused, struct task_struct *prev, struct task_struct *next, struct rq *rq);
extern void android_vh_scheduler_tick_handler(void *unused, struct rq *rq);

/* register vendor hook in kernel/cgroup/cgroup-v1.c */
extern void android_vh_cgroup_set_task_handler(void *unused, int ret, struct task_struct *task);
#endif /* _OPLUS_SA_COMMON_H_ */
