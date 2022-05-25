/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef FSTB_USEDEXT_H
#define FSTB_USEDEXT_H

#include <mt-plat/fpsgo_common.h>
#include <linux/list.h>
#include <linux/sched.h>

#define DEFAULT_DFPS 60
#define CFG_MAX_FPS_LIMIT	240
#define CFG_MIN_FPS_LIMIT	10
#define FRAME_TIME_BUFFER_SIZE 200
#define MAX_NR_FPS_LEVELS	1
#define MAX_NR_RENDER_FPS_LEVELS	10
#define DISPLAY_FPS_FILTER_NS 100000000ULL
#define DISPLAY_FPS_FILTER_NUM 4
#define ASFC_THRESHOLD_NS 20000000ULL
#define ASFC_THRESHOLD_PERCENTAGE 30
#define VPU_MAX_CAP 100
#define MDLA_MAX_CAP 100
#define RESET_TOLERENCE 3
#define DEFAULT_JUMP_CHECK_NUM 21
#define DEFAULT_JUMP_CHECK_Q_PCT 33
#define JUMP_VOTE_MAX_I 60
#define FSTB_IDLE_DBNC 10

extern int (*fbt_notifier_cpu_frame_time_fps_stabilizer)(
	int pid,
	int frame_type,
	unsigned long long Q2Q_time,
	unsigned long long Runnging_time,
	unsigned int Curr_cap,
	unsigned int Max_cap,
	unsigned int Target_fps);
extern void (*ged_kpi_output_gfx_info2_fp)(long long t_gpu,
	unsigned int cur_freq, unsigned int cur_max_freq, u64 ulID);

struct FSTB_FRAME_INFO {
	struct hlist_node hlist;

	int pid;
	int proc_id;
	char proc_name[16];
	int target_fps;
	int target_fps_v2;
	int target_fps_margin_v2;
	int target_fps_margin;
	int target_fps_margin_gpu;
	int target_fps_margin2;
	int target_fps_margin_dbnc_a;
	int target_fps_margin_dbnc_b;
	int target_fps_margin_gpu_dbnc_a;
	int target_fps_margin_gpu_dbnc_b;
	int queue_fps;
	unsigned long long bufid;
	int in_list;
	int new_info;
	int target_fps_diff;
	int target_fps_notifying;
	int sbe_state; /* -1: no chase, 0: free run, 1: max_fps*/
	/*set ui control: 1, otherswise: 0
	 *  HWUI & 1-> sbe_state: 0,1
	 *  HWUI & 0-> sbe_state: -1
	 *  non-HWUI-> 0
	 */
	int sbe_fpsgo_ctrl;

	long long cpu_time;
	long long gpu_time;
	int gpu_freq;

	unsigned long long queue_time_ts[FRAME_TIME_BUFFER_SIZE]; /*timestamp*/
	int queue_time_begin;
	int queue_time_end;
	int vote_fps[JUMP_VOTE_MAX_I];
	int vote_i;
	unsigned long long weighted_cpu_time[FRAME_TIME_BUFFER_SIZE];
	unsigned long long weighted_cpu_time_ts[FRAME_TIME_BUFFER_SIZE];
	unsigned long long weighted_gpu_time[FRAME_TIME_BUFFER_SIZE];
	unsigned long long weighted_gpu_time_ts[FRAME_TIME_BUFFER_SIZE];
	int weighted_cpu_time_begin;
	int weighted_cpu_time_end;
	int weighted_gpu_time_begin;
	int weighted_gpu_time_end;
	unsigned long long sorted_weighted_cpu_time[FRAME_TIME_BUFFER_SIZE];
	unsigned long long sorted_weighted_gpu_time[FRAME_TIME_BUFFER_SIZE];
	int quantile_cpu_time;
	int quantile_gpu_time;

	unsigned long long gblock_b;
	unsigned long long gblock_time;
	int fps_raise_flag;
	int render_idle_cnt;
	int hwui_flag;
};

struct FSTB_RENDER_TARGET_FPS {
	struct hlist_node hlist;

	char process_name[16];
	int pid;
	int nr_level;
	struct fps_level level[MAX_NR_RENDER_FPS_LEVELS];
};

struct FSTB_POWERFPS_LIST {
	int pid;
	int fps;
};

struct FSTB_NOTIFIER_PUSH_TAG {
	int pid;
	unsigned long long bufid;
	unsigned long long cur_dequeue_start_ts;
	unsigned long long cur_queue_end_ts;

	struct work_struct sWork;
};

#endif
