// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/clocksource.h>
#include <linux/sched/clock.h>
#include <linux/suspend.h>
#include <linux/timex.h>
#include <linux/types.h>
#include <asm/arch_timer.h>
#include <asm/timex.h>
#include "sspm_define.h"
#include "sspm_ipi.h"
#include "sspm_ipi_mbox.h"
#include "sspm_mbox.h"
#include "sspm_timesync.h"

#define sspm_ts_write(id, val) \
	sspm_mbox_write(SSPM_TS_MBOX, id, (void *)&val, 1)

#define TIMESYNC_TAG	"[SSPM_TS]"

#define TIMESYNC_MAX_VER           (0x7)
#define TIMESYNC_HEADER_FREEZE_OFS (31)
#define TIMESYNC_HEADER_FREEZE     (1 << TIMESYNC_HEADER_FREEZE_OFS)
#define TIMESYNC_HEADER_VER_OFS    (28)
#define TIMESYNC_HEADER_VER_MASK   (TIMESYNC_MAX_VER << TIMESYNC_HEADER_VER_OFS)

#define TIMESYNC_FLAG_SYNC     (1 << 0)
#define TIMESYNC_FLAG_ASYNC    (1 << 1)
#define TIMESYNC_FLAG_FREEZE   (1 << 2)
#define TIMESYNC_FLAG_UNFREEZE (1 << 3)

/* sched_clock wrap time is 4398 seconds for arm arch timer
 * applying a period less than it for tinysys timesync
 */
#define TIMESYNC_WRAP_TIME (4000ULL * NSEC_PER_SEC)

struct timesync_context_t {
	spinlock_t lock;
	struct work_struct work;
	ktime_t wrap_kt;
	u8 enabled;
	u64 base_tick;
	u64 base_ts;
};

static struct workqueue_struct *timesync_workqueue;
static struct timesync_context_t timesync_ctx;
static struct timecounter timesync_counter;
static struct hrtimer timesync_refresh_timer;
static u8 sspm_base_ver;

static void sspm_ts_update(int suspended, u64 tick, u64 ts)
{
	u32 header, val;

	sspm_base_ver = (sspm_base_ver + 1)%(TIMESYNC_MAX_VER+1);

	/* make header: freeze and version */
	header = suspended ? TIMESYNC_HEADER_FREEZE : 0;

	header |= ((sspm_base_ver << TIMESYNC_HEADER_VER_OFS) &
		TIMESYNC_HEADER_VER_MASK);

	/* update tick, h -> l */
	val = (tick >> 32) & 0xFFFFFFFF;
	val |= header;
	sspm_ts_write(SSPM_TS_MBOX_TICK_H, val);

	/* fix update sequence to promise atomicity */
	mb();

	val = tick & 0xFFFFFFFF;
	sspm_ts_write(SSPM_TS_MBOX_TICK_L, val);

	/* fix update sequence to promise atomicity */
	mb();

	/* update ts, l -> h */
	val = ts & 0xFFFFFFFF;
	sspm_ts_write(SSPM_TS_MBOX_TS_L, val);

	/* fix update sequence to promise atomicity */
	mb();

	val = (ts >> 32) & 0xFFFFFFFF;
	val |= header;
	sspm_ts_write(SSPM_TS_MBOX_TS_H, val);

	/* fix update sequence to promise atomicity */
	mb();
}

static u64 timesync_tick_read(const struct cyclecounter *cc)
{
	return arch_timer_read_counter();
}

static struct cyclecounter timesync_cc __ro_after_init = {
	.read	= timesync_tick_read,
	.mask	= CLOCKSOURCE_MASK(56),
};

static void timesync_sync_base_internal(unsigned int flag)
{
	u64 tick, ts;
	unsigned long irq_flags = 0;
	int freeze, unfreeze;

	spin_lock_irqsave(&timesync_ctx.lock, irq_flags);

	ts =  timecounter_read(&timesync_counter);
	tick = timesync_counter.cycle_last;

	timesync_ctx.base_tick = tick;
	timesync_ctx.base_ts = ts;

	freeze = (flag & TIMESYNC_FLAG_FREEZE) ? 1 : 0;
	unfreeze = (flag & TIMESYNC_FLAG_UNFREEZE) ? 1 : 0;

	/* sync with sspm */
	sspm_ts_update(freeze, tick, ts);

	spin_unlock_irqrestore(&timesync_ctx.lock, irq_flags);

	pr_info("%s update base: ts=%llu, tick=0x%llx, fz=%d, ver=%d\n",
		TIMESYNC_TAG, ts, tick, freeze, sspm_base_ver);
}

static void timesync_sync_base(unsigned int flag)
{
	if (!timesync_ctx.enabled)
		return;

	if (flag & TIMESYNC_FLAG_ASYNC)
		queue_work(timesync_workqueue, &(timesync_ctx.work));
	else
		timesync_sync_base_internal(flag);
}

static enum hrtimer_restart timesync_refresh(struct hrtimer *hrt)
{
	hrtimer_forward_now(hrt, timesync_ctx.wrap_kt);
	/* snchronize new sched_clock base to co-processors */
	timesync_sync_base(TIMESYNC_FLAG_ASYNC);

	return HRTIMER_RESTART;
}

static void timesync_ws(struct work_struct *ws)
{
	timesync_sync_base(TIMESYNC_FLAG_SYNC);
}

unsigned int __init sspm_timesync_init(void)
{
	timesync_workqueue = create_workqueue("sspm_ts_wq");
	if (!timesync_workqueue) {
		pr_info("%s workqueue create failed\n", __func__);
		timesync_ctx.enabled = 0;
		return -1;
	}

	INIT_WORK(&(timesync_ctx.work), timesync_ws);

	spin_lock_init(&timesync_ctx.lock);

	/* init cyclecounter mult and shift as sched_clock */
	clocks_calc_mult_shift(&timesync_cc.mult, &timesync_cc.shift,
				arch_timer_get_cntfrq(), NSEC_PER_SEC, 3600);

	timesync_ctx.wrap_kt = ns_to_ktime(TIMESYNC_WRAP_TIME);

	/* Init time counter:
	 * start_time: current sched_clock
	 * read: arch timer counter
	 */
	timecounter_init(&timesync_counter, &timesync_cc, sched_clock());

	hrtimer_init(&timesync_refresh_timer,
				CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timesync_refresh_timer.function = timesync_refresh;
	hrtimer_start(&timesync_refresh_timer,
		timesync_ctx.wrap_kt, HRTIMER_MODE_REL);

	pr_info("%s ts: cycle_last %lld, time_base:%lld, wrap:%lld\n",
		TIMESYNC_TAG, timesync_counter.cycle_last,
		timesync_counter.nsec, timesync_ctx.wrap_kt);

	timesync_ctx.enabled = 1;

	timesync_sync_base(TIMESYNC_FLAG_SYNC);

	return 0;
}

void sspm_timesync_suspend(void)
{
	if (!timesync_ctx.enabled)
		return;

	hrtimer_cancel(&timesync_refresh_timer);

	/* snchronize new sched_clock base to co-processors */
	timesync_sync_base(TIMESYNC_FLAG_SYNC |
		TIMESYNC_FLAG_FREEZE);
}

void sspm_timesync_resume(void)
{
	if (!timesync_ctx.enabled)
		return;

	/* re-init timecounter because sched_clock will be stopped during
	 * suspend but arch timer counter is not, so we need to update
	 *  start time after resume
	 */
	timecounter_init(&timesync_counter, &timesync_cc, sched_clock());

	hrtimer_start(&timesync_refresh_timer,
		timesync_ctx.wrap_kt, HRTIMER_MODE_REL);

	/* snchronize new sched_clock base to co-processors */
	timesync_sync_base(TIMESYNC_FLAG_SYNC |
		TIMESYNC_FLAG_UNFREEZE);
}

