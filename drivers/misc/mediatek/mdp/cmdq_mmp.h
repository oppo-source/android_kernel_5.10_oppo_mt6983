/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef __CMDQ_MMP_H__
#define __CMDQ_MMP_H__

#include "cmdq_helper_ext.h"
#if IS_ENABLED(CONFIG_MMPROFILE)
#include "mmprofile.h"
#endif

struct MDP_MMP_events_t {
	mmp_event CMDQ;
	mmp_event CMDQ_IRQ;
	mmp_event thread_en;
	mmp_event warning;
	mmp_event loopBeat;
	mmp_event autoRelease_add;
	mmp_event autoRelease_done;
	mmp_event consume_add;
	mmp_event consume_done;
	mmp_event alloc_task;
	mmp_event wait_task;
	mmp_event wait_task_done;
	mmp_event task_exec;
	mmp_event wait_thread;
	mmp_event wait_task_clean;
	mmp_event MDP_reset;
	mmp_event MDP_clock_on;
	mmp_event MDP_clock_off;
	mmp_event MDP_clock_smi;
	mmp_event thread_suspend;
	mmp_event thread_resume;
	mmp_event alloc_buffer;
	mmp_event timeout;
	mmp_event read_reg;
};

void mdp_mmp_init(void);
struct MDP_MMP_events_t *mdp_mmp_get_event(void);

#endif				/* __CMDQ_MMP_H__ */
