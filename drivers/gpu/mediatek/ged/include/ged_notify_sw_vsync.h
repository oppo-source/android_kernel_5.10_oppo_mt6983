/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __GED_NOTIFY_SW_VSYNC_H__
#define __GED_NOTIFY_SW_VSYNC_H__

#include "ged_type.h"

extern unsigned int gpu_block;
extern unsigned int gpu_idle;
extern unsigned int gpu_av_loading;

extern unsigned long long g_ns_gpu_on_ts;

GED_ERROR ged_notify_sw_vsync(GED_VSYNC_TYPE eType,
	struct GED_DVFS_UM_QUERY_PACK *psQueryData);

GED_ERROR ged_notify_sw_vsync_system_init(void);

void ged_notify_sw_vsync_system_exit(void);

void ged_set_backup_timer_timeout(u64 time_out);
void ged_cancel_backup_timer(void);

int ged_get_policy_state_pre(void);
int ged_get_policy_state(void);
void ged_set_policy_state(int state);
#if defined(CONFIG_GPU_MT8167) || defined(CONFIG_GPU_MT8173) ||\
defined(CONFIG_GPU_MT6739) || defined(CONFIG_GPU_MT6761) ||\
defined(CONFIG_GPU_MT6765)
extern void MTKFWDump(void);
#endif

#endif
