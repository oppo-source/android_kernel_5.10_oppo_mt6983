// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifdef FS_UT
#include <string.h>
#include <stdlib.h>         /* Needed by memory allocate */
#else
/* INSTEAD of using stdio.h, you have to use the following include */
#include <linux/module.h>   /* Needed by all modules */
#include <linux/kernel.h>   /* Needed for KERN_INFO */
#include <linux/slab.h>     /* Needed by memory allocate */

#include <linux/string.h>
#endif // FS_UT

#include "frame_sync_algo.h"
#include "frame_monitor.h"

#if !defined(FS_UT)
#include "frame_sync_console.h"
#endif // FS_UT


/******************************************************************************/
// Log message
/******************************************************************************/
#include "frame_sync_log.h"

#define REDUCE_FS_ALGO_LOG
#define PFX "FrameSyncAlgo"
/******************************************************************************/


#ifdef SUPPORT_FS_NEW_METHOD
/******************************************************************************/
// Lock
/******************************************************************************/
#ifdef FS_UT
#include <pthread.h>
static pthread_mutex_t fs_algo_sa_proc_mutex_lock = PTHREAD_MUTEX_INITIALIZER;
#else
#include <linux/mutex.h>
static DEFINE_MUTEX(fs_algo_sa_proc_mutex_lock);
#endif // FS_UT
/******************************************************************************/
#endif // SUPPORT_FS_NEW_METHOD





/******************************************************************************/
// Frame Sync Instance Structure (private structure)
/******************************************************************************/
#ifdef SUPPORT_FS_NEW_METHOD
/*
 * structure for FrameSync StandAlone (SA) mode using
 */
struct FrameSyncDynamicPara {
	/* serial number for each dynamic paras */
	unsigned int magic_num;

	/* per-frame status variables */
	int master_idx;
	int ref_m_idx_magic_num;
	unsigned int ask_for_chg;       // if finally ask FS DRV switch to master
	unsigned int chg_master;        // if request to change to master
	long long adj_diff_m;
	long long adj_diff_s;
	unsigned int adj_or_not;

	/* for current pf ctrl, min suitable frame length for stable sync */
	unsigned int pure_min_fl_us;    // max((exp+margin), user-cofig-min_fl)
	unsigned int min_fl_us;         // max((exp+margin), user-cofig-min_fl)+flk
	unsigned int target_min_fl_us;  // FPS sync result
	unsigned int stable_fl_us;      // same as out_fl_us

	/* predicted frame length (0:current / 1:next) */
	unsigned int pred_fl_us[2];

	/* sync target ts bias (for feature that sync to non-LE) */
	unsigned int ts_bias_us;

	/* N:1 sync */
	unsigned int f_tag;
	unsigned int f_cell;
	unsigned int tag_bias_us;

	/* total dalta (without adding timestamp diff) */
	unsigned int delta;

	/* timestamp info */
	unsigned int cur_tick;          // current tick at querying data
	unsigned int last_ts;           // last timestamp at querying data
	unsigned int vsyncs;            // passed vsync counts

	/* fs SA mode cfg */
	struct fs_sa_cfg sa_cfg;
};


struct FrameSyncStandAloneInst {
	/* support: 0:adaptive switch master */
	unsigned int sa_algo;

	FS_Atomic_T master_idx;

	/* serial number for each dynamic paras */
	unsigned int magic_num[SENSOR_MAX_NUM];


	/* all sensor shared dynamic paras for FS SA mode */
	struct FrameSyncDynamicPara dynamic_paras[SENSOR_MAX_NUM];
};
static struct FrameSyncStandAloneInst fs_sa_inst;
#endif // SUPPORT_FS_NEW_METHOD
//----------------------------------------------------------------------------//


#define FS_FL_RECORD_DEPTH (5)
struct fs_fl_rec_st {
	unsigned int ref_magic_num;
	unsigned int target_min_fl_us;
	unsigned int out_fl_us;
};
//----------------------------------------------------------------------------//


struct FrameSyncInst {
	/* register sensor info */
	unsigned int sensor_id;         // imx586 -> 0x0586; s5k3m5sx -> 0x30D5
	unsigned int sensor_idx;        // main1 -> 0; sub1 -> 1;
					// main2 -> 2; sub2 -> 3; main3 -> 4;

//----------------------------------------------------------------------------//

	enum FS_SYNC_TYPE sync_type;
	unsigned int custom_bias_us;    // for sync with diff

//----------------------------------------------------------------------------//

//---------------------------- fs_streaming_st -------------------------------//
	/* know TG when streaming */
	unsigned int tg;                // Not used if ISP7 uses sensor_idx

	unsigned int fl_active_delay;   // SONY/auto_ext:(3, 1); others:(2, 0);
	unsigned int def_min_fl_lc;
	unsigned int max_fl_lc;         // for frame length boundary check

	unsigned int def_shutter_lc;    // default shutter_lc in driver

//----------------------------------------------------------------------------//

	/* IOCTRL CID ANTI_FLICKER */
	unsigned int flicker_en:1;      // move to perframe_st

//---------------------------- fs_perframe_st --------------------------------//
	/* IOCTRL CID SHUTTER_FRAME_LENGTH */
	unsigned int min_fl_lc;         // dynamic FPS using
	unsigned int shutter_lc;

	/* current/previous multi exposure settings */
	/*    because exp is N+1 delay, so just keep previous settings */
	struct fs_hdr_exp_st hdr_exp;
	struct fs_hdr_exp_st prev_hdr_exp;

	/* on-the-fly sensor mode change */
	unsigned int margin_lc;
	unsigned int pclk;
	unsigned int linelength;
	unsigned int lineTimeInNs;      // ~= 10^9 * (linelength/pclk)
	unsigned int readout_time_us;   // current mode read out time.

	/* output frame length */
	unsigned int output_fl_lc;
	unsigned int output_fl_us;      // microsecond for algo using
//----------------------------------------------------------------------------//

//---------------------------- private member --------------------------------//
	/* for STG sensor read offset change, effect valid min fl */
	unsigned int readout_min_fl_lc;
	unsigned int prev_readout_min_fl_lc;


	/* for STG sensor using FDOL mode like DOL mode */
	/* when doing STG seamless switch */
	unsigned int extend_fl_lc;
	unsigned int extend_fl_us;


	/* for different fps sync sensor sync together */
	/* e.g. fps: (60 vs 30) */
	/* => frame_cell_size: 2 */
	/* => frame_tag: 0, 1, 0, 1, 0, ... */
	unsigned int n_1_on_off:1;
	unsigned int frame_cell_size;
	unsigned int frame_tag;


	/* predicted frame length, current:0, next:1 */
	/* must be updated when getting new frame record data / vsync data */
	unsigned int predicted_fl_us[2];
	unsigned int predicted_fl_lc[2];

	unsigned int vsyncs_updated:1;

	/* frame_record_st (record shutter and framelength settings) */
	struct frame_record_st recs[RECORDER_DEPTH];

	struct fs_fl_rec_st fl_rec[FS_FL_RECORD_DEPTH];


	/* frame monitor data */
	unsigned int vsyncs;
	unsigned int last_vts;
	unsigned int timestamps[VSYNCS_MAX];
	unsigned int cur_tick;
	unsigned int vdiff;

	unsigned int is_nonvalid_ts:1;

//----------------------------------------------------------------------------//

	/* debug variables */
	unsigned int sof_cnt;            // from seninf vsync notify
	int req_id;                      // from mtk hdr ae structure
};
static struct FrameSyncInst fs_inst[SENSOR_MAX_NUM];


/* fps sync result */
static unsigned int target_min_fl_us;


/* frame monitor data */
static unsigned int cur_tick;
static unsigned int tick_factor;


/* frame sync flicker table */
#define FLICKER_TABLE_SIZE 8
static unsigned int fs_flicker_table[FLICKER_TABLE_SIZE][2] = {

	/* 14.6 ~ 15.3 */
	{68493, 65359},

	/* 23.6 ~ 24.3 */
	{42372, 41152},

	/* 24.6 ~ 25.3 */
	{40650, 39525},

	/* 29.6 ~ 30.5 */
	{33783, 32786},

	/* 59.2 ~ 60.7 */
	{16891, 16474},

	/* END */
	{0, 0}
};
/******************************************************************************/





/******************************************************************************/
// utility functions
/******************************************************************************/
static inline unsigned int get_anti_flicker_fl(unsigned int framelength)
{
	unsigned int i = 0;


	for (i = 0; i < FLICKER_TABLE_SIZE; ++i) {
		if (fs_flicker_table[i][0] == 0)
			break;

		if ((fs_flicker_table[i][0] > framelength) &&
			(framelength >= fs_flicker_table[i][1])) {

			framelength = fs_flicker_table[i][0];
			break;
		}
	}


	return framelength;
}


static inline unsigned int check_sync_flicker_en_status(unsigned int idx)
{
	unsigned int i = 0;
	unsigned int flk_en_fdelay = (0 - 1);


	/* find out min fdelay of all flk_en */
	for (i = 0; i < SENSOR_MAX_NUM; ++i) {
		if (fs_inst[i].flicker_en > 0) {
			if (fs_inst[i].fl_active_delay < flk_en_fdelay)
				flk_en_fdelay = fs_inst[i].fl_active_delay;
		}
	}


	return (fs_inst[idx].fl_active_delay < flk_en_fdelay) ? 0 : 1;
}


static inline void check_fl_boundary(unsigned int idx)
{
	unsigned int buf_fl_lc = 0, buf_fl_us = 0;


	/* check framelength boundary */
	if (fs_inst[idx].output_fl_lc > fs_inst[idx].max_fl_lc) {
		buf_fl_lc = fs_inst[idx].output_fl_lc;
		buf_fl_us = fs_inst[idx].output_fl_us;


		/* reset fl_lc and fl_us to maximum value */
		fs_inst[idx].output_fl_lc = fs_inst[idx].max_fl_lc;
		fs_inst[idx].output_fl_us = convert2TotalTime(
					fs_inst[idx].lineTimeInNs,
					fs_inst[idx].output_fl_lc);


		/* log warning (this case will break sync) */
		LOG_PR_WARN(
			"WARNING: [%u] ID:%#x(sidx:%u), set fl:%u(%u), but reaches max_fl:%u(%u)\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			buf_fl_us,
			buf_fl_lc,
			fs_inst[idx].output_fl_us,
			fs_inst[idx].output_fl_lc);
	}
}


static inline unsigned int check_fs_inst_vsync_data_valid(
	const unsigned int solveIdxs[], const unsigned int len)
{
	unsigned int i = 0;
	unsigned int ret = 1; // valid -> 1 / non-valid -> 0


	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];


		/* valid -> ret = 1 (not be changed) */
		/* non-valid -> ret = 0, and keep it being 0 */
		if (fs_inst[idx].last_vts == 0) {
			ret = 0;

#if !defined(REDUCE_FS_ALGO_LOG)
			LOG_PR_ERR(
				"ERROR: [%u] ID:%#x(sidx:%u), last vts:%u\n",
				idx,
				fs_inst[idx].sensor_id,
				fs_inst[idx].sensor_idx,
				fs_inst[idx].last_vts);
#endif // REDUCE_FS_ALGO_LOG

		}
	}


	return ret;
}


#if defined(SUPPORT_FS_NEW_METHOD)
/*
 * Be careful: query frame_cell_size behavior must use this API
 *
 * return:
 *     1: when frame_cell_size is 0 or 1
 *     u_int (>1): when frame_cell_size is bigger than 1
 */
static inline unsigned int get_valid_frame_cell_size(unsigned int idx)
{
	return (fs_inst[idx].frame_cell_size > 1)
		? (fs_inst[idx].frame_cell_size) : 1;
}
#endif // SUPPORT_FS_NEW_METHOD


/*
 * all framelength operation must use this API
 */
static inline void set_fl_us(unsigned int idx, unsigned int us)
{
	/* 1. set / update framelength value */
	fs_inst[idx].output_fl_us = us;
	fs_inst[idx].output_fl_lc = convert2LineCount(
					fs_inst[idx].lineTimeInNs,
					fs_inst[idx].output_fl_us);


#ifndef REDUCE_FS_ALGO_LOG
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), set fl:%u(%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].output_fl_us,
		fs_inst[idx].output_fl_lc);
#endif // REDUCE_FS_ALGO_LOG


	/* 2. check framelength boundary */
	check_fl_boundary(idx);


	/* 3. update result to frame recorder for predicted fl used */
	// *fs_inst[idx].recs[0].framelength_lc = fs_inst[idx].output_fl_lc;


#ifdef FS_UT
	/* N. set frame monitor sensor_curr_fl_us for FS_UT timestamp */
	frm_set_sensor_curr_fl_us(idx, us);
#endif // FS_UT
}


/*
 * calculate a appropriate min framelength base on shutter with boundary check
 *
 * input: idx, min_fl_lc
 *
 * reference: shutter, margin, max_fl_lc
 *
 * "min_fl_lc":
 *      could be "def_min_fl_lc" ( like sensor driver write_shutter() function )
 *      or "min_fl_lc" ( for frame sync dynamic FPS ).
 */
static unsigned int calc_min_fl_lc(
	unsigned int idx, unsigned int min_fl_lc)
{
	unsigned int output_fl_lc = 0;
	unsigned int shutter_margin =
			fs_inst[idx].shutter_lc + fs_inst[idx].margin_lc;


	/* calculate appropriate min framelength */
	if (shutter_margin > min_fl_lc)
		output_fl_lc = shutter_margin;
	else
		output_fl_lc = min_fl_lc;


	if (fs_inst[idx].hdr_exp.mode_exp_cnt > 1) {
		/* STG FLL constraints */
		output_fl_lc =
			(output_fl_lc > fs_inst[idx].readout_min_fl_lc)
			? output_fl_lc : fs_inst[idx].readout_min_fl_lc;
	}


	/* check extend frame length had been set */
	if (fs_inst[idx].extend_fl_lc != 0) {
		output_fl_lc += fs_inst[idx].extend_fl_lc;

		LOG_INF(
			"[%u] ID:%#x(sidx:%u), set fl to %u(%u) due to ext_fl:%u(%u))\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			convert2TotalTime(
				fs_inst[idx].lineTimeInNs,
				output_fl_lc),
			output_fl_lc,
			fs_inst[idx].extend_fl_us,
			fs_inst[idx].extend_fl_lc);
	}


	/* framelength boundary check */
	if (output_fl_lc > fs_inst[idx].max_fl_lc) {
		LOG_INF(
			"WARNING: [%u] ID:%#x(sidx:%u), set fl:%u(%u), but reaches max_fl:%u(%u)\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			convert2TotalTime(
				fs_inst[idx].lineTimeInNs,
				output_fl_lc),
			output_fl_lc,
			convert2TotalTime(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].max_fl_lc),
			fs_inst[idx].max_fl_lc);

		output_fl_lc = fs_inst[idx].max_fl_lc;
	}


	return output_fl_lc;
}


#ifdef FS_UT
/*
 * like sensor driver set_max_framerate() function
 */
static inline void
set_max_framerate(unsigned int idx, unsigned int framerate)
{
	fs_inst[idx].output_fl_lc =
		fs_inst[idx].pclk / framerate * 10 / fs_inst[idx].linelength;
}


/* TODO: output value in fs_inst[].output_fl_c or in stack? */
unsigned int fs_alg_write_shutter(unsigned int idx)
{
	unsigned int realtime_fps = 0;
	unsigned int is_adjust_fps = 0;


	/* get appropriate min framelength base on shutter */
	fs_inst[idx].output_fl_lc =
		calc_min_fl_lc(idx, fs_inst[idx].def_min_fl_lc);

	/* shutter boundary check, pass for UT */

	/* for anti flicker */
	realtime_fps = fs_inst[idx].pclk / fs_inst[idx].linelength * 10
					/ fs_inst[idx].output_fl_lc;

	if (fs_inst[idx].flicker_en) {
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(idx, 296);
			realtime_fps = 296;
			is_adjust_fps = 1;
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(idx, 146);
			realtime_fps = 146;
			is_adjust_fps = 1;
		}
	}


	LOG_INF(
		"[%u] ID:%#x(sidx:%u), fl:%u(%u), x10fps(%u):%u, flicker(%u), shutter:%u(%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		convert2TotalTime(fs_inst[idx].lineTimeInNs,
				fs_inst[idx].output_fl_lc),
		fs_inst[idx].output_fl_lc,
		is_adjust_fps,
		realtime_fps,
		fs_inst[idx].flicker_en,
		convert2TotalTime(fs_inst[idx].lineTimeInNs,
				fs_inst[idx].shutter_lc),
		fs_inst[idx].shutter_lc);


	return fs_inst[idx].output_fl_lc;
}
#endif // FS_UT


static inline unsigned int check_valid_for_using_vsyncs_recs_data(
	unsigned int idx)
{
	return (fs_inst[idx].vsyncs_updated) ? 1 : 0;
}


static inline void set_valid_for_using_vsyncs_recs_data(
	unsigned int idx, unsigned int flag)
{
	fs_inst[idx].vsyncs_updated = flag;
}


/*
 * predicted frame length ( current:0, next:1 )
 *
 * must ONLY be updated when
 *    setting new frame recs data and after getting vsyncs data
 *    otherwise the results will be not correct
 */
static void calc_predicted_frame_length(unsigned int idx)
{
	unsigned int i = 0;

	unsigned int fdelay = fs_inst[idx].fl_active_delay;
	unsigned int *p_fl_lc = fs_inst[idx].predicted_fl_lc;
	unsigned int *p_fl_us = fs_inst[idx].predicted_fl_us;


	/* for error handle, check sensor fl_active_delay value */
	if ((fdelay < 2) || (fdelay > 3)) {
		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), frame_time_delay_frame:%u is not valid (must be 2 or 3), plz check sensor driver for getting correct value\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			fs_inst[idx].fl_active_delay);

		return;
	}

	/* calculate "current predicted" and "next predicted" framelength */
	/* P.S. current:0, next:1 */
	for (i = 0; i < 2; ++i) {
		if (fdelay == 3) {
			/* SONY sensor with auto-extend on */
			unsigned int sm =
				*fs_inst[idx].recs[1-i].shutter_lc +
				fs_inst[idx].margin_lc;

			p_fl_lc[i] =
				(*fs_inst[idx].recs[2-i].framelength_lc > sm)
				? *fs_inst[idx].recs[2-i].framelength_lc
				: sm;

			if (fs_inst[idx].hdr_exp.mode_exp_cnt > 1) {
				/* STG FLL constraints */
				if (i == 0) {
					p_fl_lc[i] =
						(p_fl_lc[i] > fs_inst[idx].prev_readout_min_fl_lc)
						? p_fl_lc[i]
						: fs_inst[idx].prev_readout_min_fl_lc;

				} else { // if (i == 1)
					p_fl_lc[i] =
						(p_fl_lc[i] > fs_inst[idx].readout_min_fl_lc)
						? p_fl_lc[i]
						: fs_inst[idx].readout_min_fl_lc;
				}
			}


			p_fl_us[i] = convert2TotalTime(
						fs_inst[idx].lineTimeInNs,
						p_fl_lc[i]);

		} else { // fdelay == 2
			/* non-SONY sensor case */
			p_fl_lc[i] = *fs_inst[idx].recs[1-i].framelength_lc;


			p_fl_us[i] = convert2TotalTime(
						fs_inst[idx].lineTimeInNs,
						p_fl_lc[i]);
		}
	}
}


static inline void calibrate_recs_data_by_vsyncs(unsigned int idx)
{
	if (!check_valid_for_using_vsyncs_recs_data(idx)) {
		LOG_MUST(
			"WARNING: [%u] ID:%#x(sidx:%u), calibrate recs failed, please update vsyncs data first\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx);

		return;
	}


	/* Check how many vsyncs have passed so far, */
	/* for syncing correct frame settings. */
	/* EX: [0, 1, 2] -> [1, 2, temp] or [2, 2, temp] */
	if (fs_inst[idx].vsyncs > 1) {
		*fs_inst[idx].recs[2].framelength_lc =
				*fs_inst[idx].recs[1].framelength_lc;
		*fs_inst[idx].recs[2].shutter_lc =
				*fs_inst[idx].recs[1].shutter_lc;


#ifndef REDUCE_FS_ALGO_LOG
		LOG_INF(
			"[%u] ID:%#x(sidx:%u), passed vsyncs:%u, calibrate frame records data\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			fs_inst[idx].vsyncs);
#endif // REDUCE_FS_ALGO_LOG
	}

	/* in the same frame case */
	/* EX: [0, 1, 2, temp] -> [0, 0, 1, temp] */
	if (fs_inst[idx].vsyncs == 0 && RECORDER_DEPTH == 4) {
		*fs_inst[idx].recs[1].framelength_lc =
				*fs_inst[idx].recs[2].framelength_lc;
		*fs_inst[idx].recs[1].shutter_lc =
				*fs_inst[idx].recs[2].shutter_lc;

		*fs_inst[idx].recs[2].framelength_lc =
				*fs_inst[idx].recs[3].framelength_lc;
		*fs_inst[idx].recs[2].shutter_lc =
				*fs_inst[idx].recs[3].shutter_lc;
	}


	/* update predicted frame length by new frame recorder data */
	calc_predicted_frame_length(idx);


	/* set NOT valid for using vsyncs / recs data for */
	/* preventing calculation error */
	/* (call calibrate_recs_data_by_vsyncs() API again */
	/*  without re-query timestamp data) */
	set_valid_for_using_vsyncs_recs_data(idx, 0);
}


static unsigned int calc_vts_sync_bias(unsigned int idx)
{
	unsigned int vts_exp_bias = 0, vts_signal_bias = 0, total_bias = 0;
	unsigned int margin_lc_per_exp = 0;


	if (fs_inst[idx].hdr_exp.mode_exp_cnt > 1) {
		/* for exp sync type (sync on SE) */
		margin_lc_per_exp =
			fs_inst[idx].margin_lc /
			fs_inst[idx].hdr_exp.mode_exp_cnt;

		if (fs_inst[idx].sync_type & FS_SYNC_TYPE_SE) {
			if (fs_inst[idx].hdr_exp.mode_exp_cnt == 2) {
				/* LE / SE */
				vts_exp_bias =
					margin_lc_per_exp +
					fs_inst[idx].hdr_exp.exp_lc[FS_HDR_SE];

			} else if (fs_inst[idx].hdr_exp.mode_exp_cnt == 3) {
				/* LE / ME / SE */
				vts_exp_bias =
					margin_lc_per_exp +
					fs_inst[idx].hdr_exp.exp_lc[FS_HDR_ME] +
					margin_lc_per_exp +
					fs_inst[idx].hdr_exp.exp_lc[FS_HDR_SE];
			}
		}
	}


	/* for signal sync type (vsync / readout center) */
	if (fs_inst[idx].sync_type & FS_SYNC_TYPE_READOUT_CENTER) {
		// TODO: calculate by add a variable of sensor readout time
		vts_signal_bias = 0;
	} else if (fs_inst[idx].sync_type & FS_SYNC_TYPE_VSYNC)
		vts_signal_bias = 0;


#ifndef REDUCE_FS_ALGO_LOG
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), vts_exp_bias:%u, margin_lc/exp_cnt:%u, vts_signal_bias:%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		vts_exp_bias,
		margin_lc_per_exp,
		vts_signal_bias);
#endif // REDUCE_FS_ALGO_LOG

	total_bias = (vts_exp_bias + vts_signal_bias);

	if (fs_inst[idx].custom_bias_us != 0) {
		total_bias += convert2LineCount(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].custom_bias_us);

		LOG_INF(
			"NOTICE: [%u] ID:%#x(sidx:%u), set custom_bias:%u(%u)\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			fs_inst[idx].custom_bias_us,
			convert2LineCount(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].custom_bias_us));
	}


	return total_bias;
}

static unsigned int calc_seamless_frame_time_us(const unsigned int idx,
	const struct fs_seamless_st *p_seamless_info)
{
	const unsigned int mode_exp_cnt =
		fs_inst[idx].prev_hdr_exp.mode_exp_cnt;
	unsigned int re_exp_us = 0, re_exp_lc = 0;
	unsigned int hw_init_time_us = 0;
	unsigned int readout_start_shift_us = 0;
	unsigned int i = 0;
	unsigned int ret = 0;

	/* error handling (unexpected case) */
	if (unlikely(p_seamless_info == NULL)) {
		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), get p_seamless_info:%p, return 0\n",
			idx, fs_inst[idx].sensor_id, fs_inst[idx].sensor_idx,
			p_seamless_info);
		return 0;
	}

	/* get basic info */
	/* check normal or hdr situation (normal: shutter_lc / hdr: hdr_exp) */
	if (p_seamless_info->seamless_pf_ctrl.shutter_lc != 0)
		re_exp_lc = p_seamless_info->seamless_pf_ctrl.shutter_lc;
	else
		re_exp_lc = p_seamless_info->seamless_pf_ctrl.hdr_exp.exp_lc[0];

	re_exp_us = convert2TotalTime(
			p_seamless_info->seamless_pf_ctrl.lineTimeInNs,
			re_exp_lc);

	/* read back some last pf ctrl settings for calculating */
	if (mode_exp_cnt) {
		for (i = 1; i < mode_exp_cnt; ++i) {
			int hdr_idx = hdr_exp_idx_map[mode_exp_cnt][i];

			/* error case (unexpected) */
			if (unlikely(hdr_idx < 0)) {
				readout_start_shift_us = 0;
				LOG_MUST(
					"ERROR: [%u] ID:%#x(sidx:%u), hdr_exp_idx_map[%u][%u] = %d => return readout_start_shift_us:%u\n",
					idx, fs_inst[idx].sensor_id, fs_inst[idx].sensor_idx,
					mode_exp_cnt,
					i,
					hdr_idx,
					readout_start_shift_us);

				return readout_start_shift_us;
			}

			readout_start_shift_us +=
				(fs_inst[idx].prev_hdr_exp.exp_lc[hdr_idx]
				+ (fs_inst[idx].margin_lc / mode_exp_cnt));
		}

		readout_start_shift_us =
			convert2TotalTime(fs_inst[idx].lineTimeInNs, readout_start_shift_us);
	}

	ret = readout_start_shift_us + p_seamless_info->orig_readout_time_us
		+ hw_init_time_us + re_exp_us;

	LOG_MUST(
		"[%u] ID:%#x(sidx:%u), seamless_frame_time_us:%u (readout_start_shift_us:%u, orig_readout_time_us:%u, hw_init_time_us:%u, re_exp_us:%u, line_time(ns):(%u => %u)\n",
		idx, fs_inst[idx].sensor_id, fs_inst[idx].sensor_idx,
		ret,
		readout_start_shift_us,
		p_seamless_info->orig_readout_time_us,
		hw_init_time_us,
		re_exp_us,
		fs_inst[idx].lineTimeInNs,
		p_seamless_info->seamless_pf_ctrl.lineTimeInNs);

	return ret;
}


/*
 * check pf ctrl trigger in critical section of timestamp.
 * next timestamp of one sensor is coming soon.
 * if it is sync, pred_vdiff almost equal to target_min_fl_us.
 *
 * return:
 *     1: trigger in ts critaical section
 *     0: not trigger in ts critical section
 */
static inline unsigned int check_timing_critical_section(
	const unsigned int pred_vdiff, const unsigned int target_min_fl_us)
{
	unsigned int threshold = 0/*, delta = 0*/;

	// threshold = FS_TOLERANCE / 2;
	// delta = threshold / 10;
	// threshold += delta;
	threshold = FS_TOLERANCE;

	if (pred_vdiff > target_min_fl_us)
		return (((pred_vdiff - target_min_fl_us) < threshold) ? 1 : 0);
	else
		return (((target_min_fl_us - pred_vdiff) < threshold) ? 1 : 0);
}


/*
 * be careful:
 *    In each frame this API should only be called at once,
 *    otherwise will cause wrong frame monitor data.
 *
 *    So calling this API at/before next vsync coming maybe a good choise.
 */
void fs_alg_setup_frame_monitor_fmeas_data(unsigned int idx)
{
	/* 1. update predicted frame length by new frame recorder data */
	calc_predicted_frame_length(idx);

	/* 2. set frame measurement predicted frame length */
	frm_set_frame_measurement(
		idx, fs_inst[idx].vsyncs,
		fs_inst[idx].predicted_fl_us[0],
		fs_inst[idx].predicted_fl_lc[0],
		fs_inst[idx].predicted_fl_us[1],
		fs_inst[idx].predicted_fl_lc[1]);
}
/******************************************************************************/





/******************************************************************************/
// Dump & Debug function
/******************************************************************************/
void fs_alg_get_cur_frec_data(unsigned int idx,
	unsigned int *p_fl_lc, unsigned int *p_shut_lc)
{
	if (p_fl_lc != NULL)
		*p_fl_lc = *fs_inst[idx].recs[0].framelength_lc;

	if (p_shut_lc != NULL)
		*p_shut_lc = *fs_inst[idx].recs[0].shutter_lc;
}


void fs_alg_get_fs_inst_ts_data(unsigned int idx,
	unsigned int *p_tg, unsigned int ts_arr[],
	unsigned int *p_last_vts, unsigned int *p_time_after_sof,
	unsigned int *p_cur_tick, unsigned int *p_vsyncs)
{
	unsigned int i = 0;

	if (p_tg != NULL)
		*p_tg = fs_inst[idx].tg;

	if (p_last_vts != NULL)
		*p_last_vts = fs_inst[idx].last_vts;

	if (p_time_after_sof != NULL) {
		*p_time_after_sof =
			calc_time_after_sof(
				fs_inst[idx].last_vts,
				fs_inst[idx].cur_tick, tick_factor);
	}

	if (p_cur_tick != NULL)
		*p_cur_tick = fs_inst[idx].cur_tick;

	if (p_vsyncs != NULL)
		*p_vsyncs = fs_inst[idx].vsyncs;

	if (ts_arr != NULL) {
		for (i = 0; i < VSYNCS_MAX; ++i)
			ts_arr[i] = fs_inst[idx].timestamps[i];
	}
}


static inline void fs_alg_dump_streaming_data(unsigned int idx)
{
	LOG_MUST(
		"[%u] ID:%#x(sidx:%u), tg:%u, fl_delay:%u, fl_lc(def/max):%u/%u, def_shut_lc:%u, lineTime:%u(linelength:%u/pclk:%u), hdr_exp: c(%u/%u/%u/%u/%u, %u/%u), prev(%u/%u/%u/%u/%u, %u/%u), cnt:(mode/ae)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].tg,
		fs_inst[idx].fl_active_delay,
		fs_inst[idx].def_min_fl_lc,
		fs_inst[idx].max_fl_lc,
		fs_inst[idx].def_shutter_lc,
		fs_inst[idx].lineTimeInNs,
		fs_inst[idx].linelength,
		fs_inst[idx].pclk,
		fs_inst[idx].hdr_exp.exp_lc[0],
		fs_inst[idx].hdr_exp.exp_lc[1],
		fs_inst[idx].hdr_exp.exp_lc[2],
		fs_inst[idx].hdr_exp.exp_lc[3],
		fs_inst[idx].hdr_exp.exp_lc[4],
		fs_inst[idx].hdr_exp.mode_exp_cnt,
		fs_inst[idx].hdr_exp.ae_exp_cnt,
		fs_inst[idx].prev_hdr_exp.exp_lc[0],
		fs_inst[idx].prev_hdr_exp.exp_lc[1],
		fs_inst[idx].prev_hdr_exp.exp_lc[2],
		fs_inst[idx].prev_hdr_exp.exp_lc[3],
		fs_inst[idx].prev_hdr_exp.exp_lc[4],
		fs_inst[idx].prev_hdr_exp.mode_exp_cnt,
		fs_inst[idx].prev_hdr_exp.ae_exp_cnt);
}


static inline void fs_alg_dump_perframe_data(unsigned int idx)
{
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), flk_en:%u, min_fl:%u(%u), shutter:%u(%u), margin:%u(%u), lineTime(ns):%u(%u/%u), hdr_exp: c(%u(%u)/%u(%u)/%u(%u)/%u(%u)/%u(%u), %u/%u), prev(%u(%u)/%u(%u)/%u(%u)/%u(%u)/%u(%u), %u/%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].flicker_en,
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].min_fl_lc),
		fs_inst[idx].min_fl_lc,
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].shutter_lc),
		fs_inst[idx].shutter_lc,
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].margin_lc),
		fs_inst[idx].margin_lc,
		fs_inst[idx].lineTimeInNs,
		fs_inst[idx].linelength,
		fs_inst[idx].pclk,
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].hdr_exp.exp_lc[0]),
		fs_inst[idx].hdr_exp.exp_lc[0],
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].hdr_exp.exp_lc[1]),
		fs_inst[idx].hdr_exp.exp_lc[1],
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].hdr_exp.exp_lc[2]),
		fs_inst[idx].hdr_exp.exp_lc[2],
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].hdr_exp.exp_lc[3]),
		fs_inst[idx].hdr_exp.exp_lc[3],
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].hdr_exp.exp_lc[4]),
		fs_inst[idx].hdr_exp.exp_lc[4],
		fs_inst[idx].hdr_exp.mode_exp_cnt,
		fs_inst[idx].hdr_exp.ae_exp_cnt,
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].prev_hdr_exp.exp_lc[0]),
		fs_inst[idx].prev_hdr_exp.exp_lc[0],
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].prev_hdr_exp.exp_lc[1]),
		fs_inst[idx].prev_hdr_exp.exp_lc[1],
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].prev_hdr_exp.exp_lc[2]),
		fs_inst[idx].prev_hdr_exp.exp_lc[2],
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].prev_hdr_exp.exp_lc[3]),
		fs_inst[idx].prev_hdr_exp.exp_lc[3],
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].prev_hdr_exp.exp_lc[4]),
		fs_inst[idx].prev_hdr_exp.exp_lc[4],
		fs_inst[idx].prev_hdr_exp.mode_exp_cnt,
		fs_inst[idx].prev_hdr_exp.ae_exp_cnt);
}


/* for debug using, dump all data in instance */
void fs_alg_dump_fs_inst_data(const unsigned int idx)
{
	LOG_MUST(
		"[%u] ID:%#x(sidx:%u), (%d/%u), tg:%u, fdelay:%u, fl_lc(def/min/max/out):%u/%u/%u/%u(%u), pred_fl(c:%u(%u)/n:%u(%u)), shut_lc:%u(def:%u), margin_lc:%u, flk_en:%u, lineTime:%u(%u/%u), readout(us):%u, f_cell:%u, f_tag:%u, n_1:%u, hdr_exp(c(%u/%u/%u/%u/%u, %u/%u, %u/%u), prev(%u/%u/%u/%u/%u, %u/%u, %u/%u), cnt:(mode/ae), read(len/margin)), ts(%u/%u/%u/%u, %u/+(%u)/%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].req_id,
		fs_inst[idx].sof_cnt,
		fs_inst[idx].tg,
		fs_inst[idx].fl_active_delay,
		fs_inst[idx].def_min_fl_lc,
		fs_inst[idx].min_fl_lc,
		fs_inst[idx].max_fl_lc,
		fs_inst[idx].output_fl_lc,
		fs_inst[idx].output_fl_us,
		fs_inst[idx].predicted_fl_us[0],
		fs_inst[idx].predicted_fl_lc[0],
		fs_inst[idx].predicted_fl_us[1],
		fs_inst[idx].predicted_fl_lc[1],
		fs_inst[idx].shutter_lc,
		fs_inst[idx].def_shutter_lc,
		fs_inst[idx].margin_lc,
		fs_inst[idx].flicker_en,
		fs_inst[idx].lineTimeInNs,
		fs_inst[idx].linelength,
		fs_inst[idx].pclk,
		fs_inst[idx].readout_time_us,
		fs_inst[idx].frame_tag,
		fs_inst[idx].frame_cell_size,
		fs_inst[idx].n_1_on_off,
		fs_inst[idx].hdr_exp.exp_lc[0],
		fs_inst[idx].hdr_exp.exp_lc[1],
		fs_inst[idx].hdr_exp.exp_lc[2],
		fs_inst[idx].hdr_exp.exp_lc[3],
		fs_inst[idx].hdr_exp.exp_lc[4],
		fs_inst[idx].hdr_exp.mode_exp_cnt,
		fs_inst[idx].hdr_exp.ae_exp_cnt,
		fs_inst[idx].hdr_exp.readout_len_lc,
		fs_inst[idx].hdr_exp.read_margin_lc,
		fs_inst[idx].prev_hdr_exp.exp_lc[0],
		fs_inst[idx].prev_hdr_exp.exp_lc[1],
		fs_inst[idx].prev_hdr_exp.exp_lc[2],
		fs_inst[idx].prev_hdr_exp.exp_lc[3],
		fs_inst[idx].prev_hdr_exp.exp_lc[4],
		fs_inst[idx].prev_hdr_exp.mode_exp_cnt,
		fs_inst[idx].prev_hdr_exp.ae_exp_cnt,
		fs_inst[idx].prev_hdr_exp.readout_len_lc,
		fs_inst[idx].prev_hdr_exp.read_margin_lc,
		fs_inst[idx].timestamps[0],
		fs_inst[idx].timestamps[1],
		fs_inst[idx].timestamps[2],
		fs_inst[idx].timestamps[3],
		fs_inst[idx].last_vts,
		fs_inst[idx].cur_tick,
		fs_inst[idx].vsyncs);
}


/* for debug using, dump all data in all instance */
void fs_alg_dump_all_fs_inst_data(void)
{
	unsigned int i = 0;

	for (i = 0; i < SENSOR_MAX_NUM; ++i)
		fs_alg_dump_fs_inst_data(i);
}


#ifdef SUPPORT_FS_NEW_METHOD
static inline void fs_alg_sa_dump_dynamic_para(unsigned int idx)
{
	unsigned int time_after_sof = 0, time_after_sof_cur = 0;
	unsigned int fmeas_idx = 0, pr_fl_us = 0, pr_fl_lc = 0, act_fl_us = 0;
	unsigned int fmeas_ts[VSYNCS_MAX] = {0};


	time_after_sof =
		calc_time_after_sof(
			fs_sa_inst.dynamic_paras[idx].last_ts,
			fs_sa_inst.dynamic_paras[idx].cur_tick, tick_factor);

	time_after_sof_cur =
		calc_time_after_sof(
			fs_inst[idx].last_vts,
			fs_inst[idx].cur_tick, tick_factor);

	frm_get_curr_frame_mesurement_and_ts_data(idx,
		&fmeas_idx, &pr_fl_us, &pr_fl_lc, &act_fl_us, fmeas_ts);


	LOG_MUST(
		"[%u] ID:%#x(sidx:%u), #%u, (%d/%u), out_fl:%u(%u), (%u/%u/%u/%u(%u/%u), %u, %u(%u)), pr_fl(c:%u(%u)/n:%u(%u)), ts_bias(exp:%u/tag:%u(%u/%u)), readout_time:%u, delta:%u(fdelay:%u), m_idx:%u(ref:%d)/chg:%u(%u), adj_diff(s:%lld(%u)/m:%lld), flk_en:%u, sa_cfg(idx:%u/m_idx:%d/async_m_idx:%d/async_s:%#x/valid_sync:%#x/method:%u), tg:%u, ts(%u/+%u(%u)/%u), [frec(0:%u/%u)(fl_lc/shut_lc), fmeas:%u(pr:%u(%u)/act:%u), fmeas_ts(%u/%u/%u/%u), fs_inst_ts(%u/%u/%u/%u, %u/+%u(%u)/%u)]\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_sa_inst.dynamic_paras[idx].magic_num,
		fs_inst[idx].req_id,
		fs_inst[idx].sof_cnt,
		fs_sa_inst.dynamic_paras[idx].stable_fl_us,
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			fs_sa_inst.dynamic_paras[idx].stable_fl_us),
		fs_inst[idx].shutter_lc,
		fs_inst[idx].margin_lc,
		fs_inst[idx].min_fl_lc,
		fs_inst[idx].readout_min_fl_lc,
		fs_inst[idx].hdr_exp.readout_len_lc,
		fs_inst[idx].hdr_exp.read_margin_lc,
		fs_inst[idx].lineTimeInNs,
		fs_sa_inst.dynamic_paras[idx].target_min_fl_us,
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			fs_sa_inst.dynamic_paras[idx].target_min_fl_us),
		fs_sa_inst.dynamic_paras[idx].pred_fl_us[0],
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			fs_sa_inst.dynamic_paras[idx].pred_fl_us[0]),
		fs_sa_inst.dynamic_paras[idx].pred_fl_us[1],
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			fs_sa_inst.dynamic_paras[idx].pred_fl_us[1]),
		fs_sa_inst.dynamic_paras[idx].ts_bias_us,
		fs_sa_inst.dynamic_paras[idx].tag_bias_us,
		fs_sa_inst.dynamic_paras[idx].f_tag,
		fs_sa_inst.dynamic_paras[idx].f_cell,
		fs_inst[idx].readout_time_us,
		fs_sa_inst.dynamic_paras[idx].delta,
		fs_inst[idx].fl_active_delay,
		fs_sa_inst.dynamic_paras[idx].master_idx,
		fs_sa_inst.dynamic_paras[idx].ref_m_idx_magic_num,
		fs_sa_inst.dynamic_paras[idx].ask_for_chg,
		fs_sa_inst.dynamic_paras[idx].chg_master,
		fs_sa_inst.dynamic_paras[idx].adj_diff_s,
		fs_sa_inst.dynamic_paras[idx].adj_or_not,
		fs_sa_inst.dynamic_paras[idx].adj_diff_m,
		fs_inst[idx].flicker_en,
		fs_sa_inst.dynamic_paras[idx].sa_cfg.idx,
		fs_sa_inst.dynamic_paras[idx].sa_cfg.m_idx,
		fs_sa_inst.dynamic_paras[idx].sa_cfg.async_m_idx,
		fs_sa_inst.dynamic_paras[idx].sa_cfg.async_s_bits,
		fs_sa_inst.dynamic_paras[idx].sa_cfg.valid_sync_bits,
		fs_sa_inst.dynamic_paras[idx].sa_cfg.sa_method,
		fs_inst[idx].tg,
		fs_sa_inst.dynamic_paras[idx].last_ts,
		time_after_sof,
		fs_sa_inst.dynamic_paras[idx].cur_tick,
		fs_sa_inst.dynamic_paras[idx].vsyncs,
		*fs_inst[idx].recs[0].framelength_lc,
		*fs_inst[idx].recs[0].shutter_lc,
		fmeas_idx,
		pr_fl_us,
		pr_fl_lc,
		act_fl_us,
		fmeas_ts[0],
		fmeas_ts[1],
		fmeas_ts[2],
		fmeas_ts[3],
		fs_inst[idx].timestamps[0],
		fs_inst[idx].timestamps[1],
		fs_inst[idx].timestamps[2],
		fs_inst[idx].timestamps[3],
		fs_inst[idx].last_vts,
		time_after_sof_cur,
		fs_inst[idx].cur_tick,
		fs_inst[idx].vsyncs);
}
#endif // SUPPORT_FS_NEW_METHOD
/******************************************************************************/





/******************************************************************************/
// fs frame length record functions
/******************************************************************************/
static void fs_alg_init_fl_rec_st(const unsigned int idx)
{
	const unsigned int f_cell = get_valid_frame_cell_size(idx);
	const unsigned int f_tag = fs_inst[idx].frame_tag;

	/* for m-stream case (only tag 0 need reset) */
	if (f_cell == 2 && f_tag != 0)
		return;

	memset(&fs_inst[idx].fl_rec, 0, sizeof(fs_inst[idx].fl_rec));
}


static void fs_alg_update_fl_rec_st_fl_info(const unsigned int idx,
	const unsigned int ref_magic_num,
	const unsigned int target_min_fl_us, const unsigned int out_fl_us)
{
	const unsigned int f_cell = get_valid_frame_cell_size(idx);
	const unsigned int f_tag = fs_inst[idx].frame_tag;

	if (f_tag >= FS_FL_RECORD_DEPTH) {
		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), f_tag:%u >= FS_FL_RECORD_DEPTH:%u, f_cell:%u, array index overflow, return   [idx:%u, target_min_fl_us:%u, out_fl_us:%u]\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			f_tag,
			FS_FL_RECORD_DEPTH,
			f_cell,
			idx,
			target_min_fl_us,
			out_fl_us);
		return;
	}

	/* update debug info */
	fs_inst[idx].fl_rec[f_tag].ref_magic_num = ref_magic_num;

	/* update/setup frame length info */
	/* target_min_fl_us is equal to (fl_us * f_cell) */
	fs_inst[idx].fl_rec[f_tag].target_min_fl_us =
		(f_cell != 0) ? (target_min_fl_us/f_cell) : target_min_fl_us;
	fs_inst[idx].fl_rec[f_tag].out_fl_us = out_fl_us;

	LOG_INF(
		"[%u] ID:%#x(sidx:%u), fl_rec(0:(#%u, %u/%u), 1:(#%u, %u/%u), 2:(#%u, %u/%u), 3:(#%u, %u/%u), 4:(#%u, %u/%u), (target_min_fl_us/out_fl_us)), f_tag:%u/f_cell:%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].fl_rec[0].ref_magic_num,
		fs_inst[idx].fl_rec[0].target_min_fl_us,
		fs_inst[idx].fl_rec[0].out_fl_us,
		fs_inst[idx].fl_rec[1].ref_magic_num,
		fs_inst[idx].fl_rec[1].target_min_fl_us,
		fs_inst[idx].fl_rec[1].out_fl_us,
		fs_inst[idx].fl_rec[2].ref_magic_num,
		fs_inst[idx].fl_rec[2].target_min_fl_us,
		fs_inst[idx].fl_rec[2].out_fl_us,
		fs_inst[idx].fl_rec[3].ref_magic_num,
		fs_inst[idx].fl_rec[3].target_min_fl_us,
		fs_inst[idx].fl_rec[3].out_fl_us,
		fs_inst[idx].fl_rec[4].ref_magic_num,
		fs_inst[idx].fl_rec[4].target_min_fl_us,
		fs_inst[idx].fl_rec[4].out_fl_us,
		f_tag,
		f_cell);
}


void fs_alg_get_fl_rec_st_info(const unsigned int idx,
	unsigned int *p_target_min_fl_us, unsigned int *p_out_fl_us)
{
	const unsigned int f_cell = get_valid_frame_cell_size(idx);
	const unsigned int f_tag = fs_inst[idx].frame_tag;
	unsigned int i = 0;

	/* error handle (unexpected case) */
	if (p_target_min_fl_us == NULL || p_out_fl_us == NULL) {
		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), get non-valid p_target_min_fl_us:%p or p_out_fl_us:%p, return\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			p_target_min_fl_us,
			p_out_fl_us);
		return;
	}

	/* clear data */
	*p_target_min_fl_us = 0;
	*p_out_fl_us = 0;

	if (f_tag >= FS_FL_RECORD_DEPTH) {
		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), f_tag:%u >= FS_FL_RECORD_DEPTH:%u, f_cell:%u, array index overflow, return\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			f_tag,
			FS_FL_RECORD_DEPTH,
			f_cell);
		return;
	}

	/* copy data */
	for (i = 0; i < f_cell; ++i) {
		*p_target_min_fl_us += fs_inst[idx].fl_rec[i].target_min_fl_us;
		*p_out_fl_us += fs_inst[idx].fl_rec[i].out_fl_us;
	}

	LOG_MUST(
		"[%u] ID:%#x(sidx:%u), target_min_fl_us:%u/out_fl_us:%u, fl_rec(0:(#%u, %u/%u), 1:(#%u, %u/%u), 2:(#%u, %u/%u), 3:(#%u, %u/%u), 4:(#%u, %u/%u), (target_min_fl_us/out_fl_us)), f_cell:%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		*p_target_min_fl_us,
		*p_out_fl_us,
		fs_inst[idx].fl_rec[0].ref_magic_num,
		fs_inst[idx].fl_rec[0].target_min_fl_us,
		fs_inst[idx].fl_rec[0].out_fl_us,
		fs_inst[idx].fl_rec[1].ref_magic_num,
		fs_inst[idx].fl_rec[1].target_min_fl_us,
		fs_inst[idx].fl_rec[1].out_fl_us,
		fs_inst[idx].fl_rec[2].ref_magic_num,
		fs_inst[idx].fl_rec[2].target_min_fl_us,
		fs_inst[idx].fl_rec[2].out_fl_us,
		fs_inst[idx].fl_rec[3].ref_magic_num,
		fs_inst[idx].fl_rec[3].target_min_fl_us,
		fs_inst[idx].fl_rec[3].out_fl_us,
		fs_inst[idx].fl_rec[4].ref_magic_num,
		fs_inst[idx].fl_rec[4].target_min_fl_us,
		fs_inst[idx].fl_rec[4].out_fl_us,
		f_cell);
}



/******************************************************************************/
// fs algo operation functions (set information data)
/******************************************************************************/
#ifdef SUPPORT_FS_NEW_METHOD
static inline void fs_alg_sa_sync_settings_for_dynamic_paras(
	const unsigned int idx, struct FrameSyncDynamicPara *p_para)
{
	p_para->f_tag = fs_inst[idx].frame_tag;
	p_para->f_cell = fs_inst[idx].frame_cell_size;
}


static inline void fs_alg_sa_init_new_ctrl(
	const struct fs_sa_cfg *p_sa_cfg,
	struct FrameSyncDynamicPara *p_para)
{
	const unsigned int idx = p_sa_cfg->idx;
	const int m_idx = p_sa_cfg->m_idx;

	p_para->sa_cfg = *p_sa_cfg;

	/* generate new ctrl serial number */
	p_para->magic_num = ++fs_sa_inst.magic_num[idx];
	p_para->master_idx = m_idx;

	/* sync current settings */
	fs_alg_sa_sync_settings_for_dynamic_paras(idx, p_para);

	/* init frame length record st data (if needed) */
	fs_alg_init_fl_rec_st(idx);
}


static inline void fs_alg_reset_fs_sa_inst(const unsigned int idx)
{
	struct FrameSyncDynamicPara clear_st = {0};

	fs_sa_inst.magic_num[idx] = 0;

	fs_sa_inst.dynamic_paras[idx] = clear_st;


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_INF("clear idx:%u data. (all to zero)\n", idx);
#endif // REDUCE_FS_ALGO_LOG
}


static unsigned int fs_alg_sa_calc_f_tag_diff(
	const unsigned int idx, const unsigned int f_tag)
{
	unsigned int f_cell = 0, fdelay = 0;
	unsigned int fl = 0;


	f_cell = get_valid_frame_cell_size(idx);
	fdelay = fs_inst[idx].fl_active_delay;
	fl = fs_inst[idx].output_fl_us;


	return (f_cell > 1)
		? (fl * ((f_cell - (fdelay-1) + (f_cell - f_tag)) % (f_cell)))
		: 0;
}


/*
 * special API return equivelent prdicted frame length
 * according to fdelay, target (current/next), f_cell size.
 *
 * be careful:
 *     for sensor that N+1 FL activate,
 *     this API only calculate to current predicted frame length.
 *
 * input:
 *     pred_fl_us: predicted frame length
 *     stable_fl_us: if not change register setting, FL is this value for all frame
 *     fdelay: frame length activate delay
 *     target: current/next predicted => 0/1
 *     f_cell: for algorithm, treat f_cell frame as one frame to give predicted FL
 *
 * output:
 *     u_int: according to input return corresponding predicted frame length
 */
static unsigned int fs_alg_sa_calc_target_pred_fl_us(
	unsigned int pred_fl_us[], const unsigned int stable_fl_us,
	const unsigned int fdelay, const unsigned int target,
	unsigned int f_cell)
{
	unsigned int i = 0, cnt = 0, val = 0;
	unsigned int pred_fl = 0;


	/* fdelay must only be 2 or 3 */
	if (!((fdelay == 2) || (fdelay == 3))) {
		LOG_MUST(
			"ERROR: frame_time_delay_frame:%u is not valid (must be 2 or 3), plz check sensor driver for getting correct value\n",
			fdelay
		);
	}

	/* N+1 FL activate, only calculate to current predicted frame length */
	if ((fdelay == 2) && (target == 1))
		return 0;

	/* for the logic of this function, min f_cell is 1, */
	/* through at normal Frame-Sync case f_cell is 0 */
	if (f_cell < 1)
		f_cell = 1;


	i = (target * f_cell);
	cnt = (target * f_cell) + f_cell;

	if ((target == 0) || (target == 1)) {
		/* calculate curr/next predicted frame length */
		for (; i < cnt; ++i) {
			val = (i < 2) ? pred_fl_us[i] : stable_fl_us;

			pred_fl += val;
		}

	} else {
		LOG_INF(
			"ERROR: request to calculate invalid target:%u (0:curr/1:next/unknown)\n",
			target
		);

		return 0;
	}


	return pred_fl;
}


static unsigned int fs_alg_sa_get_flk_diff_and_fl(const unsigned int idx,
	unsigned int *fl_us, const unsigned int sync_flk_en)
{
	unsigned int flk_diff = 0, fl_us_old = 0;

	fl_us_old = *fl_us;

	if (fs_inst[idx].flicker_en || sync_flk_en) {
		*fl_us = get_anti_flicker_fl(fl_us_old);
		flk_diff = *fl_us - fl_us_old;
	}


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), +flk:%u, fl_us:%u   [flk_en:%u/sync_flk_en:%u]\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		flk_diff,
		*fl_us,
		fs_inst[idx].flicker_en,
		sync_flk_en
	);
#endif // REDUCE_FS_ALGO_LOG


	return flk_diff;
}


static void fs_alg_sa_update_pred_fl_and_ts_bias(const unsigned int idx,
	struct FrameSyncDynamicPara *p_para)
{
	unsigned int ts_bias_lc = 0;

	/* calculate and get predicted frame length */
	calc_predicted_frame_length(idx);
	p_para->pred_fl_us[0] =
		fs_inst[idx].predicted_fl_us[0];
	p_para->pred_fl_us[1] =
		fs_inst[idx].predicted_fl_us[1];

	/* calculate and get timestamp bias */
	ts_bias_lc = calc_vts_sync_bias(idx);
	p_para->ts_bias_us =
		convert2TotalTime(fs_inst[idx].lineTimeInNs, ts_bias_lc);


#if defined(FS_UT)
	/* update frame monitor current predicted framelength data */
	frm_update_next_vts_bias_us(idx, p_para->ts_bias_us);
#endif // FS_UT


	/* for N:1 FrameSync case, calculate and get tag bias */
	p_para->tag_bias_us = fs_alg_sa_calc_f_tag_diff(
		idx, fs_inst[idx].frame_tag);


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), #%u, pred_fl(c:%u(%u), n:%u(%u))(%u), bias(exp:%u/tag:%u), fdelay:%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		p_para->magic_num,
		p_para->pred_fl_us[0],
		fs_inst[idx].predicted_fl_lc[0],
		p_para->pred_fl_us[1],
		fs_inst[idx].predicted_fl_lc[1],
		fs_inst[idx].lineTimeInNs,
		p_para->ts_bias_us,
		p_para->tag_bias_us,
		fs_inst[idx].fl_active_delay);
#endif // REDUCE_FS_ALGO_LOG
}


static void fs_alg_sa_prepare_dynamic_para(const unsigned int idx,
	struct FrameSyncDynamicPara *p_para)
{
	unsigned int i = 0;

	/* update predicted frame length and ts_bias */
	fs_alg_sa_update_pred_fl_and_ts_bias(idx, p_para);

	/* calculate predicted total delta (without timestamp diff) */
	p_para->delta = (p_para->ts_bias_us + p_para->tag_bias_us);
	for (i = 0; i < 2; ++i) {
		p_para->delta +=
			fs_alg_sa_calc_target_pred_fl_us(
				p_para->pred_fl_us, p_para->stable_fl_us,
				fs_inst[idx].fl_active_delay, i, 1);
	}
}


static void fs_alg_sa_update_fl_us(const unsigned int idx,
	const unsigned int us, struct FrameSyncDynamicPara *p_para)
{
	set_fl_us(idx, us);

	p_para->stable_fl_us = us;

	/* for correctly showing info */
	/* update fl also update all related variable */
	fs_alg_sa_prepare_dynamic_para(idx, p_para);

	/* update frame length info to user */
	fs_alg_update_fl_rec_st_fl_info(idx,
		p_para->magic_num,
		p_para->target_min_fl_us, p_para->stable_fl_us);
}


static void fs_alg_sa_update_seamless_dynamic_para(const unsigned int idx,
	struct fs_seamless_st *p_seamless_info,
	struct FrameSyncDynamicPara *p_para)
{
	unsigned int seamless_frame_time_us = 0;
	unsigned int ts_bias_lc = 0;
	unsigned int i = 0;

	/* error handling (unexpected case) */
	if (unlikely(p_seamless_info == NULL)) {
		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), get p_seamless_info:%p, return\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			p_seamless_info);
		return;
	}

	/* calculate seamless frame time */
	seamless_frame_time_us = calc_seamless_frame_time_us(idx, p_seamless_info);

	/* using seamless ctrl to update pf ctrl */
	fs_inst[idx].hdr_exp = p_seamless_info->seamless_pf_ctrl.hdr_exp;
	fs_alg_set_perframe_st_data(idx, &p_seamless_info->seamless_pf_ctrl);

	/* calculate and get timestamp bias */
	ts_bias_lc = calc_vts_sync_bias(idx);
	p_para->ts_bias_us =
		convert2TotalTime(fs_inst[idx].lineTimeInNs, ts_bias_lc);

	/* overwrite pred_fl_us/lc[] value to match seamless ctrl */
	fs_inst[idx].predicted_fl_us[0] = seamless_frame_time_us;
	fs_inst[idx].predicted_fl_lc[0] =
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].predicted_fl_us[0]);
	fs_inst[idx].predicted_fl_lc[1] = p_seamless_info->seamless_pf_ctrl.out_fl_lc;
	fs_inst[idx].predicted_fl_us[1] =
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].predicted_fl_lc[1]);
	p_para->pred_fl_us[0] =
		fs_inst[idx].predicted_fl_us[0];
	p_para->pred_fl_us[1] =
		fs_inst[idx].predicted_fl_us[1];

	/* overwrite frame measurement predicted frame length for debugging */
	frm_set_frame_measurement(
		idx, 0,
		fs_inst[idx].predicted_fl_us[0],
		fs_inst[idx].predicted_fl_lc[0],
		fs_inst[idx].predicted_fl_us[1],
		fs_inst[idx].predicted_fl_lc[1]);


#if defined(FS_UT)
	/* update frame monitor current predicted framelength data */
	frm_update_next_vts_bias_us(idx, p_para->ts_bias_us);
#endif // FS_UT


	/* for N:1 FrameSync case, calculate and get tag bias */
	p_para->tag_bias_us = fs_alg_sa_calc_f_tag_diff(
		idx, fs_inst[idx].frame_tag);


	/* sync seamless frame length RG value */
	set_fl_us(idx, p_para->stable_fl_us);


	/* calculate predicted total delta (without timestamp diff) */
	p_para->delta = (p_para->ts_bias_us + p_para->tag_bias_us);
	for (i = 0; i < 2; ++i) {
		p_para->delta +=
			fs_alg_sa_calc_target_pred_fl_us(
				p_para->pred_fl_us, p_para->stable_fl_us,
				fs_inst[idx].fl_active_delay, i, 1);
	}


	FS_MUTEX_LOCK(&fs_algo_sa_proc_mutex_lock);

	fs_sa_inst.dynamic_paras[idx] = *p_para;

	FS_MUTEX_UNLOCK(&fs_algo_sa_proc_mutex_lock);


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), #%u, stable_fl_us:%u, pred_fl(c:%u(%u), n:%u(%u))(%u), bias(exp:%u/tag:%u), delta:%u(fdelay:%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		p_para->magic_num,
		p_para->stable_fl_us,
		p_para->pred_fl_us[0],
		fs_inst[idx].predicted_fl_lc[0],
		p_para->pred_fl_us[1],
		fs_inst[idx].predicted_fl_lc[1],
		fs_inst[idx].lineTimeInNs,
		p_para->ts_bias_us,
		p_para->tag_bias_us,
		p_para->delta,
		fs_inst[idx].fl_active_delay);

	fs_alg_sa_dump_dynamic_para(idx);
#endif // REDUCE_FS_ALGO_LOG


	fs_alg_dump_fs_inst_data(idx);
}


static unsigned int fs_alg_sa_get_timestamp_info(const unsigned int idx,
	struct FrameSyncDynamicPara *p_para)
{
#if !defined(QUERY_CCU_TS_AT_SOF)
	unsigned int query_ts_idx[1] = {0};


	query_ts_idx[0] = idx;
	if (fs_alg_get_vsync_data(query_ts_idx, 1)) {
#else
	if (fs_inst[idx].is_nonvalid_ts) {
#endif // QUERY_CCU_TS_AT_SOF
		LOG_INF(
			"ERROR: [%u] ID:%#x(sidx:%u), get Vsync data ERROR, SA ctrl mag_num:%u\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			p_para->magic_num);

		return 1;
	}


	/* write back newest last_ts and cur_tick data */
	p_para->last_ts = fs_inst[idx].last_vts;
	p_para->cur_tick = fs_inst[idx].cur_tick;
	p_para->vsyncs = fs_inst[idx].vsyncs;


	return 0;
}


static void fs_alg_sa_pre_set_dynamic_paras(const unsigned int idx,
	struct FrameSyncDynamicPara *p_para)
{
	const unsigned int f_cell = get_valid_frame_cell_size(idx);
	unsigned int fl_us = convert2TotalTime(
		fs_inst[idx].lineTimeInNs,
		*fs_inst[idx].recs[0].framelength_lc);

	/* sync current settings */
	fs_alg_sa_sync_settings_for_dynamic_paras(idx, p_para);

	p_para->target_min_fl_us = (fl_us * f_cell);
	fs_alg_sa_update_fl_us(idx, fl_us, p_para);
	fs_alg_sa_get_timestamp_info(idx, p_para);

	LOG_MUST(
		"NOTICE: #%u, out_fl:%u(%u), frec(0:%u/%u), %u, pr_fl(c:%u(%u)/n:%u(%u)), ts_bias(exp:%u/tag:%u(%u/%u)), delta:%u(fdelay:%u), tg:%u, ts(%u/%u/%u)\n",
		p_para->magic_num,
		p_para->stable_fl_us,
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			p_para->stable_fl_us),
		*fs_inst[idx].recs[0].framelength_lc,
		*fs_inst[idx].recs[0].shutter_lc,
		fs_inst[idx].lineTimeInNs,
		p_para->pred_fl_us[0],
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			p_para->pred_fl_us[0]),
		p_para->pred_fl_us[1],
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			p_para->pred_fl_us[1]),
		p_para->ts_bias_us,
		p_para->tag_bias_us,
		p_para->f_tag,
		p_para->f_cell,
		p_para->delta,
		fs_inst[idx].fl_active_delay,
		fs_inst[idx].tg,
		p_para->last_ts,
		p_para->cur_tick,
		p_para->vsyncs);
}


/*
 * return:
 *     0: check passed / non-0: non-valid data is detected
 *     bit 1: last timestamp is zero
 *     bit 2: sensor frame_time_delay_frame value is non-valid
 */
static unsigned int fs_alg_sa_dynamic_paras_checker(
	const unsigned int s_idx, const unsigned int m_idx,
	struct FrameSyncDynamicPara *p_para_s,
	struct FrameSyncDynamicPara *p_para_m)
{
	unsigned int ret = 0;
	unsigned int query_ts_idx[2] = {s_idx, m_idx};
	unsigned int fdelay_s = fs_inst[s_idx].fl_active_delay;
	unsigned int fdelay_m = fs_inst[m_idx].fl_active_delay;


	/* check if last timestamp equal to zero */
	if (check_fs_inst_vsync_data_valid(query_ts_idx, 2) == 0) {
		LOG_MUST(
			"NOTICE: [%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), set shutter before first vsync, latest timestamp is/are ZERO (s:%u/m:%u), fs_inst(s(%u/%u/%u/%u), m(%u/%u/%u/%u)), p_para_ts(s:%u/m:%u)\n",
			s_idx,
			fs_inst[s_idx].sensor_id,
			fs_inst[s_idx].sensor_idx,
			p_para_s->magic_num,
			p_para_m->magic_num,
			m_idx,
			fs_inst[s_idx].last_vts,
			fs_inst[m_idx].last_vts,
			fs_inst[s_idx].timestamps[0],
			fs_inst[s_idx].timestamps[1],
			fs_inst[s_idx].timestamps[2],
			fs_inst[s_idx].timestamps[3],
			fs_inst[m_idx].timestamps[0],
			fs_inst[m_idx].timestamps[1],
			fs_inst[m_idx].timestamps[2],
			fs_inst[m_idx].timestamps[3],
			p_para_s->last_ts,
			p_para_m->last_ts
		);

		ret |= 1U << 1;
	}


	/* check sensor fl_active_delay value */
	/* in this time predicted frame length are equal to zero */
	if ((fdelay_s < 2 || fdelay_s > 3) || (fdelay_m < 2 || fdelay_m > 3)) {
		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), frame_time_delay_frame is/are not valid (must be 2 or 3), s:%u/m:%u\n",
			s_idx,
			fs_inst[s_idx].sensor_id,
			fs_inst[s_idx].sensor_idx,
			fs_inst[s_idx].fl_active_delay,
			fs_inst[m_idx].fl_active_delay
		);

		ret |= 1U << 2;
	}


	/* for first req ctrl, slave get ctrl faster than master, */
	/* so gen a pre set dynamic para data for slave adjust diff */
	if (!ret && (p_para_m->last_ts == 0 && fs_inst[m_idx].last_vts != 0))
		fs_alg_sa_pre_set_dynamic_paras(m_idx, p_para_m);


	return ret;
}


/*
 * input:
 *     p_para_m: a pointer to dynamic para structure of master sensor
 *     p_para_s: a pointer to dynamic para structure of slave sensor
 *
 * output:
 *     p_ts_diff_m: a pointer for ts diff of master sensor
 *     p_ts_diff_s: a pointer for ts diff of slave sensor
 *
 * be careful:
 *     Before do any operation, timestamp should be converted to clock count
 *     Tick is uint_32_t, so for correct calculation
 *         all data type should also uint32_t.
 */
static void fs_alg_sa_calc_m_s_ts_diff(
	const struct FrameSyncDynamicPara *p_para_m,
	const struct FrameSyncDynamicPara *p_para_s,
	long long *p_ts_diff_m, long long *p_ts_diff_s)
{
	unsigned int cur_tick = 0;
	unsigned int ts_diff_m = 0, ts_diff_s = 0;


	if (tick_factor == 0) {
		LOG_INF(
			"ERROR: tick_factor:%u, all ts calculation will be force to zero\n",
			tick_factor
		);
	}


	/* find newest ts info */
	if (check_tick_b_after_a(p_para_m->cur_tick, p_para_s->cur_tick)) {
		/* case - master is before slave */
		cur_tick = p_para_s->cur_tick;
	} else {
		/* case - master is after slave */
		cur_tick = p_para_m->cur_tick;
	}

	/* all operation must be in clock domain */
	ts_diff_m = p_para_m->last_ts * tick_factor;
	ts_diff_s = p_para_s->last_ts * tick_factor;


	/* normalization/shift (oldest ts => 0) */
	if ((cur_tick - ts_diff_m) < (cur_tick - ts_diff_s)) {
		ts_diff_m -= ts_diff_s;
		ts_diff_s = 0;

		if (tick_factor != 0)
			ts_diff_m /= tick_factor;

	} else {
		ts_diff_s -= ts_diff_m;
		ts_diff_m = 0;

		if (tick_factor != 0)
			ts_diff_s /= tick_factor;
	}


	/* sync result out */
	*p_ts_diff_m = (long long)ts_diff_m;
	*p_ts_diff_s = (long long)ts_diff_s;
}


static inline long long fs_alg_sa_calc_adjust_diff_master(
	const unsigned int m_idx, const long long adjust_diff_s,
	const struct FrameSyncDynamicPara *p_para_m)
{
	unsigned int f_cell_m = 1;


	f_cell_m = get_valid_frame_cell_size(m_idx);


	return ((long long)(p_para_m->stable_fl_us) * f_cell_m - adjust_diff_s);
}


static long long fs_alg_sa_calc_adjust_diff_slave(
	const unsigned int m_idx, const unsigned int s_idx,
	const long long ts_diff_m, const long long ts_diff_s,
	const struct FrameSyncDynamicPara *p_para_m,
	const struct FrameSyncDynamicPara *p_para_s)
{
	unsigned int f_cell_m = get_valid_frame_cell_size(m_idx);
	unsigned int f_cell_s = get_valid_frame_cell_size(s_idx);
	long long adjust_diff_s = 0;

	adjust_diff_s =
		(ts_diff_m + p_para_m->delta) -
		(ts_diff_s + p_para_s->delta);


	/* check adjust_diff_s situation (N+2/N+1 mixed) */
	if ((fs_inst[s_idx].fl_active_delay != fs_inst[m_idx].fl_active_delay)
		&& (adjust_diff_s > 0)) {
		/* if there are the pair, N+2 pred_fl will bigger than N+1 sensor */
		adjust_diff_s -= (long long)(p_para_s->stable_fl_us) * f_cell_s;
	}

	/* checking N:1, high fps slave's adjust diff should be normalize */
	if (adjust_diff_s > ((long long)p_para_s->stable_fl_us * f_cell_s))
		adjust_diff_s -= (long long)(p_para_s->stable_fl_us) * f_cell_s;

	/* calculate suitable adjust_diff_s */
	while (adjust_diff_s < 0) {
		/* prevent infinite loop */
		if (p_para_m->stable_fl_us == 0) {
			LOG_INF(
				"NOTICE: [%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), detect master stable_fl_us:%u, for preventing calculation error, abort processing\n",
				s_idx,
				fs_inst[s_idx].sensor_id,
				fs_inst[s_idx].sensor_idx,
				p_para_s->magic_num,
				p_para_m->magic_num,
				m_idx,
				p_para_m->stable_fl_us
			);

			return 0;
		}

		adjust_diff_s += (long long)(p_para_m->stable_fl_us) * f_cell_m;
	}


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), adjust_diff_s:%lld(ts:%lld/%lld(%u/%u), delta:%u/%u), pure_min_fl(%u/%u), stable_fl(%u/%u), f_cell(%u/%u), fdely(%u/%u)\n",
		s_idx,
		fs_inst[s_idx].sensor_id,
		fs_inst[s_idx].sensor_idx,
		p_para_s->magic_num,
		p_para_m->magic_num,
		m_idx,
		adjust_diff_s,
		ts_diff_s,
		ts_diff_m,
		fs_sa_inst.dynamic_paras[s_idx].last_ts,
		fs_sa_inst.dynamic_paras[m_idx].last_ts,
		p_para_s->delta,
		p_para_m->delta,
		p_para_s->pure_min_fl_us,
		p_para_m->pure_min_fl_us,
		p_para_s->stable_fl_us,
		p_para_m->stable_fl_us,
		f_cell_s,
		f_cell_m,
		fs_inst[s_idx].fl_active_delay,
		fs_inst[m_idx].fl_active_delay
	);
#endif // REDUCE_FS_ALGO_LOG


	return adjust_diff_s;
}


static long long fs_alg_sa_calc_adjust_diff_async(
	unsigned int m_idx, unsigned int s_idx,
	long long ts_diff_m, long long ts_diff_s,
	struct FrameSyncDynamicPara *p_para_m,
	struct FrameSyncDynamicPara *p_para_s)
{
	unsigned int f_cell_m = get_valid_frame_cell_size(m_idx);
	unsigned int f_cell_s = get_valid_frame_cell_size(s_idx);
	long long adjust_diff_s = 0;

	/* case handle for FL: N+1 sensor due to NOT doing FPS sync */
	if (fs_inst[s_idx].fl_active_delay == 2)
		p_para_s->delta += p_para_s->stable_fl_us;

	adjust_diff_s =
		(ts_diff_m + p_para_m->delta) -
		(ts_diff_s + p_para_s->delta);


	/* check adjust_diff_s situation (N+2/N+1 mixed) */
	if ((fs_inst[s_idx].fl_active_delay != fs_inst[m_idx].fl_active_delay)
		&& (adjust_diff_s > 0)) {
		/* if there are the pair, N+2 pred_fl will bigger than N+1 sensor */
		adjust_diff_s -= (long long)(p_para_s->pure_min_fl_us) * f_cell_s;
	}

	/* checking N:1, high fps slave's adjust diff should be normalize */
	if (adjust_diff_s > ((long long)p_para_s->stable_fl_us * f_cell_s))
		adjust_diff_s -= (long long)(p_para_s->stable_fl_us) * f_cell_s;


	/* TODO: for low/high FS, add a method for auto calculate complement diff */
	// adjust_diff_s += 433*3;
	// if (fs_inst[m_idx].flicker_en)
	//	adjust_diff_s -= 433*3;


	/* calculate suitable adjust_diff_s */
	while (adjust_diff_s < 0) {
		/* prevent infinite loop */
		if (p_para_m->stable_fl_us == 0) {
			LOG_INF(
				"NOTICE: [%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), detect master stable_fl_us:%u, for preventing calculation error, abort processing\n",
				s_idx,
				fs_inst[s_idx].sensor_id,
				fs_inst[s_idx].sensor_idx,
				p_para_s->magic_num,
				p_para_m->magic_num,
				m_idx,
				p_para_m->stable_fl_us
			);

			return 0;
		}

		adjust_diff_s += (long long)(p_para_m->stable_fl_us) * f_cell_m;
	}


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), adjust_diff_s:%lld(ts:%lld/%lld(%u/%u), delta:%u/%u), pure_min_fl(%u/%u), stable_fl(%u/%u), f_cell(%u/%u), fdely(%u/%u)\n",
		s_idx,
		fs_inst[s_idx].sensor_id,
		fs_inst[s_idx].sensor_idx,
		p_para_s->magic_num,
		p_para_m->magic_num,
		m_idx,
		adjust_diff_s,
		ts_diff_s,
		ts_diff_m,
		fs_sa_inst.dynamic_paras[s_idx].last_ts,
		fs_sa_inst.dynamic_paras[m_idx].last_ts,
		p_para_s->delta,
		p_para_m->delta,
		p_para_s->pure_min_fl_us,
		p_para_m->pure_min_fl_us,
		p_para_s->stable_fl_us,
		p_para_m->stable_fl_us,
		f_cell_s,
		f_cell_m,
		fs_inst[s_idx].fl_active_delay,
		fs_inst[m_idx].fl_active_delay
	);
#endif // REDUCE_FS_ALGO_LOG


	return adjust_diff_s;
}


static inline unsigned int fs_alg_sa_calc_sync_delay(
	const unsigned int idx, const long long adjust_diff,
	const struct FrameSyncDynamicPara *p_para)
{
	unsigned int f_cell = get_valid_frame_cell_size(idx);

	return (fs_inst[idx].fl_active_delay == 3)
		? (p_para->pred_fl_us[1] + (p_para->stable_fl_us * f_cell
			+ adjust_diff))
		: ((p_para->stable_fl_us * f_cell
			+ adjust_diff));
}


static unsigned int fs_alg_sa_adjust_slave_diff_resolver(
	const unsigned int m_idx, const unsigned int s_idx,
	const struct FrameSyncDynamicPara *p_para_m,
	struct FrameSyncDynamicPara *p_para_s)
{
	long long adjust_diff_m = 0, adjust_diff_s = 0;
	long long ts_diff_m = 0, ts_diff_s = 0;
	unsigned int sync_delay_m = 0, sync_delay_s = 0;
	unsigned int request_switch_master = 0;
	unsigned int adjust_or_not = 1;
	unsigned int f_cell_m = get_valid_frame_cell_size(m_idx);
	unsigned int f_cell_s = get_valid_frame_cell_size(s_idx);


	p_para_s->ref_m_idx_magic_num = p_para_m->magic_num;


	/* calculate/get current receive timestamp diff */
	fs_alg_sa_calc_m_s_ts_diff(p_para_m, p_para_s,
		&ts_diff_m, &ts_diff_s);


	/* calculate master/slave adjust_diff */
	adjust_diff_s =
		fs_alg_sa_calc_adjust_diff_slave(
			m_idx, s_idx, ts_diff_m, ts_diff_s,
			p_para_m, p_para_s);

	adjust_diff_m =
		fs_alg_sa_calc_adjust_diff_master(
			m_idx, adjust_diff_s, p_para_m);


	/* calculate master/slave sync_delay */
	sync_delay_m =
		fs_alg_sa_calc_sync_delay(m_idx, adjust_diff_m, p_para_m);

	sync_delay_s =
		fs_alg_sa_calc_sync_delay(s_idx, adjust_diff_s, p_para_s);


	/* check situation for changing master and adjusting this diff or not */
#if !defined(FORCE_ADJUST_SMALLER_DIFF)
	if ((adjust_diff_s > FS_TOLERANCE) && (adjust_diff_m > 0)
		&& (sync_delay_m < sync_delay_s))
		request_switch_master = 1;
#else
	if ((adjust_diff_s > FS_TOLERANCE) && (adjust_diff_m > 0)
		&& (adjust_diff_m < adjust_diff_s))
		request_switch_master = 1;
#endif // FORCE_ADJUST_SMALLER_DIFF

	if ((adjust_diff_s > FS_TOLERANCE)
		&& (adjust_diff_m > 0) && (adjust_diff_m < FS_TOLERANCE))
		request_switch_master = 1;

	if (check_timing_critical_section(
		adjust_diff_s, (p_para_m->stable_fl_us * f_cell_m))) {

		adjust_or_not = 0;
	}


	/* check if adjust diff is reasonable */
	if (adjust_diff_s > (long long)(p_para_m->stable_fl_us) * f_cell_m) {
		adjust_or_not = 0;

		LOG_MUST(
			"NOTICE: [%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), s/m adjust_diff(%lld(%u)/%lld) is not reasonable, do not adjust frame length\n",
			s_idx,
			fs_inst[s_idx].sensor_id,
			fs_inst[s_idx].sensor_idx,
			p_para_s->magic_num,
			p_para_m->magic_num,
			m_idx,
			adjust_diff_s,
			adjust_or_not,
			adjust_diff_m);
	}


	/* update slave all per-frame status variables to dynamic_para struct */
	p_para_s->adj_diff_m = adjust_diff_m;
	p_para_s->adj_diff_s = adjust_diff_s;
	p_para_s->adj_or_not = adjust_or_not;
	p_para_s->chg_master = request_switch_master;
	p_para_s->ask_for_chg = request_switch_master || (!adjust_or_not);


	LOG_INF(
		"[%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), ask_switch(%u), s/m adjust_diff(%lld(%u)/%lld), ts(%lld/%lld), delta(%u/%u), sync_delay(%u/%u) [(c:%u/n:%u/s:%u/e:%u/t:%u) / (c:%u/n:%u/s:%u/e:%u/t:%u)], f_cell(%u/%u), fdelay(%u/%u), ts_abs(%u/%u)\n",
		s_idx,
		fs_inst[s_idx].sensor_id,
		fs_inst[s_idx].sensor_idx,
		p_para_s->magic_num,
		p_para_m->magic_num,
		m_idx,
		request_switch_master,
		adjust_diff_s,
		adjust_or_not,
		adjust_diff_m,
		ts_diff_s,
		ts_diff_m,
		p_para_s->delta,
		p_para_m->delta,
		sync_delay_s,
		sync_delay_m,
		p_para_s->pred_fl_us[0],
		p_para_s->pred_fl_us[1],
		p_para_s->stable_fl_us,
		p_para_s->ts_bias_us,
		p_para_s->tag_bias_us,
		p_para_m->pred_fl_us[0],
		p_para_m->pred_fl_us[1],
		p_para_m->stable_fl_us,
		p_para_m->ts_bias_us,
		p_para_m->tag_bias_us,
		f_cell_s,
		f_cell_m,
		fs_inst[s_idx].fl_active_delay,
		fs_inst[m_idx].fl_active_delay,
		p_para_s->last_ts,
		// fs_sa_inst.dynamic_paras[s_idx].last_ts,
		fs_sa_inst.dynamic_paras[m_idx].last_ts
	);


	return (request_switch_master || (!adjust_or_not))
		? 0 : adjust_diff_s;
}


static inline void fs_alg_sa_get_dynamic_para(
	const unsigned int idx, struct FrameSyncDynamicPara *p_para)
{
	FS_MUTEX_LOCK(&fs_algo_sa_proc_mutex_lock);

	*p_para = fs_sa_inst.dynamic_paras[idx];

	FS_MUTEX_UNLOCK(&fs_algo_sa_proc_mutex_lock);
}


static inline void fs_alg_sa_update_dynamic_para(
	const unsigned int idx, struct FrameSyncDynamicPara *p_para)
{
	FS_MUTEX_LOCK(&fs_algo_sa_proc_mutex_lock);

	fs_alg_sa_update_pred_fl_and_ts_bias(idx, p_para);
	fs_sa_inst.dynamic_paras[idx] = *p_para;

	FS_MUTEX_UNLOCK(&fs_algo_sa_proc_mutex_lock);


#if !defined(TWO_STAGE_FS)
	fs_alg_sa_dump_dynamic_para(idx);
#endif // TWO_STAGE_FS
}


static void fs_alg_sa_query_all_min_fl_us(
	unsigned int min_fl_us[], unsigned int target_min_fl_us[],
	unsigned int magic_num[])
{
	unsigned int i = 0;


	FS_MUTEX_LOCK(&fs_algo_sa_proc_mutex_lock);

	for (i = 0; i < SENSOR_MAX_NUM; ++i) {
		min_fl_us[i] = fs_sa_inst.dynamic_paras[i].min_fl_us;
		target_min_fl_us[i] =
			fs_sa_inst.dynamic_paras[i].target_min_fl_us;

		magic_num[i] = fs_sa_inst.dynamic_paras[i].magic_num;
	}

	FS_MUTEX_UNLOCK(&fs_algo_sa_proc_mutex_lock);
}
#endif // SUPPORT_FS_NEW_METHOD


void fs_alg_set_frame_cell_size(unsigned int idx, unsigned int size)
{
	fs_inst[idx].frame_cell_size = size;


	LOG_INF(
		"[%u] ID:%#x(sidx:%u), set frame_cell_size:%u for doing sync\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].frame_cell_size
	);
}


void fs_alg_set_frame_tag(unsigned int idx, unsigned int count)
{
	fs_inst[idx].frame_tag = count;


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), set frame_tag:%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].frame_tag
	);
#endif // REDUCE_FS_ALGO_LOG
}


void fs_alg_set_n_1_on_off_flag(unsigned int idx, unsigned int flag)
{
	fs_inst[idx].n_1_on_off = flag;


	LOG_INF(
		"[%u] ID:%#x(sidx:%u), at n_1_on_off:%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].n_1_on_off
	);
}


void fs_alg_set_sync_type(unsigned int idx, unsigned int type)
{
	fs_inst[idx].sync_type = type;


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), set sync type:%u (V:%u/C:%u/L:%u/S:%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].sync_type,
		fs_inst[idx].sync_type & FS_SYNC_TYPE_VSYNC,
		fs_inst[idx].sync_type & FS_SYNC_TYPE_READOUT_CENTER,
		fs_inst[idx].sync_type & FS_SYNC_TYPE_LE,
		fs_inst[idx].sync_type & FS_SYNC_TYPE_SE);
#endif // REDUCE_FS_ALGO_LOG


#if defined(SYNC_WITH_CUSTOM_DIFF)
	if (fs_inst[idx].sensor_idx == CUSTOM_DIFF_SENSOR_IDX)
		fs_alg_set_sync_with_diff(idx, CUSTOM_DIFF_US);
#endif // SYNC_WITH_CUSTOM_DIFF
}


void fs_alg_set_anti_flicker(unsigned int idx, unsigned int flag)
{
	fs_inst[idx].flicker_en = flag;


#ifndef REDUCE_FS_ALGO_LOG
	LOG_INF("[%u] ID:%#x(sidx:%u), flk_en:%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].flicker_en);
#endif // REDUCE_FS_ALGO_LOG
}


void fs_alg_set_extend_framelength(unsigned int idx,
	unsigned int ext_fl_lc, unsigned int ext_fl_us)
{
	if (ext_fl_lc == 0 && ext_fl_us == 0) {
		/* clear/exit extend framelength stage */
		fs_inst[idx].extend_fl_lc = ext_fl_lc;
		fs_inst[idx].extend_fl_us = ext_fl_us;

	} else if (ext_fl_lc != 0 && ext_fl_us == 0) {
		fs_inst[idx].extend_fl_lc = ext_fl_lc;
		fs_inst[idx].extend_fl_us =
			convert2TotalTime(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].extend_fl_lc);

	} else if (ext_fl_lc == 0 && ext_fl_us != 0) {
		fs_inst[idx].extend_fl_us = ext_fl_us;
		fs_inst[idx].extend_fl_lc =
			convert2LineCount(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].extend_fl_us);

	} else { // both have non zero value
		unsigned int tmp_ext_fl_lc = 0;

		tmp_ext_fl_lc =
			convert2LineCount(
				fs_inst[idx].lineTimeInNs,
				ext_fl_us);

		fs_inst[idx].extend_fl_lc = (ext_fl_lc > tmp_ext_fl_lc)
			? ext_fl_lc : tmp_ext_fl_lc;

		fs_inst[idx].extend_fl_us =
			convert2TotalTime(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].extend_fl_lc);


		LOG_INF(
			"WARNING: [%u] ID:%#x(sidx:%u), both set value, ext_fl_lc:%u, ext_fl_us:%u\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			ext_fl_lc,
			ext_fl_us);
	}


	LOG_MUST(
		"[%u] ID:%#x(sidx:%u), setup extend_framelength:%u(%u) us(lc)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].extend_fl_us,
		fs_inst[idx].extend_fl_lc);
}


void fs_alg_seamless_switch(const unsigned int idx,
	struct fs_seamless_st *p_seamless_info,
	const struct fs_sa_cfg *p_sa_cfg)
{
	struct FrameSyncDynamicPara para = {0};

	/* error handling (unexpected case) */
	if (unlikely(p_seamless_info == NULL)) {
		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), get p_seamless_info:%p, return\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			p_seamless_info);
		return;
	}

	if (unlikely(p_sa_cfg == NULL)) {
		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), get p_sa_cfg:%p, return\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			p_sa_cfg);
		return;
	}

	/* prepare new dynamic para */
	fs_alg_sa_init_new_ctrl(p_sa_cfg, &para);
	para.stable_fl_us = convert2TotalTime(
		p_seamless_info->seamless_pf_ctrl.lineTimeInNs,
		p_seamless_info->seamless_pf_ctrl.out_fl_lc);

	/* get Vsync data by Frame Monitor */
	fs_alg_sa_get_timestamp_info(idx, &para);

	/* X. update dynamic para for sharing to other sensor */
	fs_alg_sa_update_seamless_dynamic_para(idx, p_seamless_info, &para);

	// fs_alg_sa_dump_dynamic_para(idx);
}


void fs_alg_update_tg(unsigned int idx, unsigned int tg)
{
	fs_inst[idx].tg = tg;


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_INF("[%u] ID:%#x(sidx:%u), updated tg:%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].tg);
#endif // REDUCE_FS_ALGO_LOG
}


void fs_alg_update_min_fl_lc(unsigned int idx, unsigned int min_fl_lc)
{
	if (fs_inst[idx].min_fl_lc != min_fl_lc) {
		/* min_fl_lc was changed after set shutter, so update it */
		fs_inst[idx].min_fl_lc = min_fl_lc;


#if !defined(REDUCE_FS_ALGO_LOG)
		LOG_INF("[%u] ID:%#x(sidx:%u), updated min_fl:%u(%u)\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			convert2TotalTime(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].min_fl_lc),
			fs_inst[idx].min_fl_lc);
#endif // REDUCE_FS_ALGO_LOG
	}
}


static unsigned int fs_alg_get_hdr_equivalent_exp_lc(unsigned int idx)
{
	unsigned int i = 0;

	unsigned int exp_lc = 0;
	unsigned int result_1 = 0, result_2 = 0;
	unsigned int mode_exp_cnt_1 = fs_inst[idx].hdr_exp.mode_exp_cnt;
	unsigned int mode_exp_cnt_2 = fs_inst[idx].prev_hdr_exp.mode_exp_cnt;


	/* calc. method 1. */
	for (i = 0; i < mode_exp_cnt_1; ++i) {
		int hdr_idx = hdr_exp_idx_map[mode_exp_cnt_1][i];

		if (hdr_idx < 0) {
			LOG_INF(
				"ERROR: [%u] ID:%#x(sidx:%u), hdr_exp_idx_map[%u][%u] = %d\n",
				idx,
				fs_inst[idx].sensor_id,
				fs_inst[idx].sensor_idx,
				mode_exp_cnt_1,
				i,
				hdr_idx);

			return 0;
		}

		result_1 += fs_inst[idx].hdr_exp.exp_lc[hdr_idx];
	}


	/* calc. method 2. */
	result_2 += fs_inst[idx].hdr_exp.exp_lc[0];
	for (i = 1; i < mode_exp_cnt_2; ++i) {
		int hdr_idx = hdr_exp_idx_map[mode_exp_cnt_2][i];

		if (hdr_idx < 0) {
			LOG_INF(
				"ERROR: [%u] ID:%#x(sidx:%u), hdr_exp_idx_map[%u][%u] = %d\n",
				idx,
				fs_inst[idx].sensor_id,
				fs_inst[idx].sensor_idx,
				mode_exp_cnt_2,
				i,
				hdr_idx);

			return 0;
		}

		result_2 += fs_inst[idx].prev_hdr_exp.exp_lc[hdr_idx];
	}


	exp_lc = (result_1 > result_2) ? result_1 : result_2;


#ifndef REDUCE_FS_ALGO_LOG
	LOG_INF("[%u] ID:%#x(sidx:%u), equiv_exp_lc:%u(%u/%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		exp_lc,
		result_1,
		result_2);
#endif // REDUCE_FS_ALGO_LOG


	return exp_lc;
}


static void fs_alg_update_hdr_exp_readout_fl_lc(unsigned int idx)
{
	int read_offset_diff = 0;
	unsigned int i = 1;
	unsigned int readout_fl_lc = 0, readout_min_fl_lc = 0;
	unsigned int mode_exp_cnt = fs_inst[idx].hdr_exp.mode_exp_cnt;
	unsigned int readout_len_lc = fs_inst[idx].hdr_exp.readout_len_lc;
	unsigned int read_margin_lc = fs_inst[idx].hdr_exp.read_margin_lc;

	struct fs_hdr_exp_st *p_curr_hdr = &fs_inst[idx].hdr_exp;
	struct fs_hdr_exp_st *p_prev_hdr = &fs_inst[idx].prev_hdr_exp;


	if ((mode_exp_cnt > 1) && (readout_len_lc == 0)) {
		/* multi exp mode but with readout length equal to zero */
		fs_inst[idx].readout_min_fl_lc = 0;

		LOG_INF(
			"WARNING: [%u] ID:%#x(sidx:%u), readout_len_lc:%d (mode_exp_cnt:%u) FL calc. may have error\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			readout_len_lc,
			mode_exp_cnt);

		return;
	}

	/* calc. each exp readout offset change, except LE */
	for (i = 1; i < mode_exp_cnt; ++i) {
		int hdr_idx = hdr_exp_idx_map[mode_exp_cnt][i];

		if (hdr_idx < 0) {
			LOG_INF(
				"ERROR: [%u] ID:%#x(sidx:%u), hdr_exp_idx_map[%u][%u] = %d\n",
				idx,
				fs_inst[idx].sensor_id,
				fs_inst[idx].sensor_idx,
				mode_exp_cnt,
				i,
				hdr_idx);

			return;
		}


		read_offset_diff +=
			p_prev_hdr->exp_lc[hdr_idx] -
			p_curr_hdr->exp_lc[hdr_idx];

		readout_fl_lc = (read_offset_diff > 0)
			? (readout_len_lc + read_margin_lc + read_offset_diff)
			: (readout_len_lc + read_margin_lc);

		if (readout_min_fl_lc < readout_fl_lc)
			readout_min_fl_lc = readout_fl_lc;
	}


	fs_inst[idx].readout_min_fl_lc = readout_min_fl_lc;
}


static void fs_alg_set_hdr_exp_st_data(
	unsigned int idx, unsigned int *shutter_lc,
	struct fs_hdr_exp_st *p_hdr_exp)
{
	unsigned int i = 0;
	unsigned int valid_exp_idx[FS_HDR_MAX] = {0};


	/* boundary ckeck */
	if (p_hdr_exp->ae_exp_cnt == 0)
		return;

	if (p_hdr_exp->ae_exp_cnt > FS_HDR_MAX ||
		p_hdr_exp->mode_exp_cnt > FS_HDR_MAX) {

		LOG_INF(
			"ERROR: [%u] ID:%#x(sidx:%u), hdr_exp: cnt:(mode:%u/ae:%u)), set to max:%u\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			p_hdr_exp->mode_exp_cnt,
			p_hdr_exp->ae_exp_cnt,
			FS_HDR_MAX);

		if (p_hdr_exp->mode_exp_cnt > FS_HDR_MAX)
			p_hdr_exp->mode_exp_cnt = FS_HDR_MAX;

		if (p_hdr_exp->ae_exp_cnt > FS_HDR_MAX)
			p_hdr_exp->ae_exp_cnt = FS_HDR_MAX;
	}


	/* sensor is NOT at STG mode */
	if (p_hdr_exp->mode_exp_cnt == 0) {
		/* NOT STG mode => get first EXP and overwrite shutter_lc data */
		fs_inst[idx].shutter_lc = p_hdr_exp->exp_lc[0];
		*shutter_lc = fs_inst[idx].shutter_lc;

		fs_inst[idx].prev_hdr_exp = fs_inst[idx].hdr_exp;
		memset(&fs_inst[idx].hdr_exp, 0, sizeof(fs_inst[idx].hdr_exp));


		/* NOT STG mode and ae_exp_cnt == 1 => fine, return */
		if (p_hdr_exp->ae_exp_cnt == 1)
			return;

		LOG_INF(
			"WARNING: [%u] ID:%#x(sidx:%u), Not HDR mode, set shutter:%u(%u) (hdr_exp: ctrl(%u/%u/%u/%u/%u, %u/%u) cnt:(mode/ae))\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			convert2TotalTime(
				fs_inst[idx].lineTimeInNs,
				*shutter_lc),
			fs_inst[idx].shutter_lc,
			p_hdr_exp->exp_lc[0],
			p_hdr_exp->exp_lc[1],
			p_hdr_exp->exp_lc[2],
			p_hdr_exp->exp_lc[3],
			p_hdr_exp->exp_lc[4],
			p_hdr_exp->mode_exp_cnt,
			p_hdr_exp->ae_exp_cnt);

		return;
	}


	/* for sensor is at STG mode */
	/* 1.  update from new -> old: p_hdr_exp -> .hdr_exp -> .prev_hdr_exp */
	fs_inst[idx].prev_hdr_exp = fs_inst[idx].hdr_exp;

	/* 1.1 update hdr_exp struct data one by one */
	fs_inst[idx].hdr_exp.mode_exp_cnt = p_hdr_exp->mode_exp_cnt;
	fs_inst[idx].hdr_exp.ae_exp_cnt = p_hdr_exp->ae_exp_cnt;
	fs_inst[idx].hdr_exp.readout_len_lc = p_hdr_exp->readout_len_lc;
	fs_inst[idx].hdr_exp.read_margin_lc = p_hdr_exp->read_margin_lc;

	/* 1.2 update hdr_exp.exp_lc array value */
	for (i = 0; i < p_hdr_exp->ae_exp_cnt; ++i) {
		int hdr_idx = hdr_exp_idx_map[p_hdr_exp->ae_exp_cnt][i];

		if (hdr_idx < 0) {
			LOG_INF(
				"ERROR: [%u] ID:%#x(sidx:%u), hdr_exp_idx_map[%u] = %d\n",
				idx,
				fs_inst[idx].sensor_id,
				fs_inst[idx].sensor_idx,
				i,
				hdr_idx);

			return;
		}

		fs_inst[idx].hdr_exp.exp_lc[hdr_idx] =
						p_hdr_exp->exp_lc[hdr_idx];


#ifndef REDUCE_FS_ALGO_LOG
		LOG_INF("[%u] ID:%#x(sidx:%u), exp_lc[%u] = %u / %u, i = %u\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			hdr_idx,
			fs_inst[idx].hdr_exp.exp_lc[idx],
			p_hdr_exp->exp_lc[idx],
			i);
#endif // REDUCE_FS_ALGO_LOG
	}


	/* 2. clear non exp value in non valid idx */
	/* 2.1 generate valid_exp_idx array for clear data using */
	for (i = 0; i < p_hdr_exp->mode_exp_cnt; ++i)
		valid_exp_idx[hdr_exp_idx_map[p_hdr_exp->mode_exp_cnt][i]] = 1;


#ifndef REDUCE_FS_ALGO_LOG
	LOG_INF("[%u] ID:%#x(sidx:%u), valid_idx:%u/%u/%u/%u/%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		valid_exp_idx[0],
		valid_exp_idx[1],
		valid_exp_idx[2],
		valid_exp_idx[3],
		valid_exp_idx[4]);
#endif // REDUCE_FS_ALGO_LOG


	/* 2.2 clear the data in non valid idx */
	for (i = 0 ; i < FS_HDR_MAX; ++i) {
		if (valid_exp_idx[i] == 0) // 0 => non valid
			fs_inst[idx].hdr_exp.exp_lc[i] = 0;
	}


	/* 3. calc. equivalent exp lc */
	/*    and overwrite shutter_lc data */
	fs_inst[idx].shutter_lc = fs_alg_get_hdr_equivalent_exp_lc(idx);
	*shutter_lc = fs_inst[idx].shutter_lc;


	/* 4. update read offset change (update readout_min_fl_lc) */
	fs_alg_update_hdr_exp_readout_fl_lc(idx);


// #ifndef REDUCE_FS_ALGO_LOG
	LOG_INF(
		"[%u] ID:%#x(sidx:%u), hdr_exp: c(%u/%u/%u/%u/%u, %u/%u, %u/%u), p(%u/%u/%u/%u/%u, %u/%u, %u/%u), ctrl(%u/%u/%u/%u/%u, %u/%u, %u/%u) cnt:(mode/ae) read:(len/margin), readout_min_fl:%u(%u), shutter:%u(%u) (equiv)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].hdr_exp.exp_lc[0],
		fs_inst[idx].hdr_exp.exp_lc[1],
		fs_inst[idx].hdr_exp.exp_lc[2],
		fs_inst[idx].hdr_exp.exp_lc[3],
		fs_inst[idx].hdr_exp.exp_lc[4],
		fs_inst[idx].hdr_exp.mode_exp_cnt,
		fs_inst[idx].hdr_exp.ae_exp_cnt,
		fs_inst[idx].hdr_exp.readout_len_lc,
		fs_inst[idx].hdr_exp.read_margin_lc,
		fs_inst[idx].prev_hdr_exp.exp_lc[0],
		fs_inst[idx].prev_hdr_exp.exp_lc[1],
		fs_inst[idx].prev_hdr_exp.exp_lc[2],
		fs_inst[idx].prev_hdr_exp.exp_lc[3],
		fs_inst[idx].prev_hdr_exp.exp_lc[4],
		fs_inst[idx].prev_hdr_exp.mode_exp_cnt,
		fs_inst[idx].prev_hdr_exp.ae_exp_cnt,
		fs_inst[idx].prev_hdr_exp.readout_len_lc,
		fs_inst[idx].prev_hdr_exp.read_margin_lc,
		p_hdr_exp->exp_lc[0],
		p_hdr_exp->exp_lc[1],
		p_hdr_exp->exp_lc[2],
		p_hdr_exp->exp_lc[3],
		p_hdr_exp->exp_lc[4],
		p_hdr_exp->mode_exp_cnt,
		p_hdr_exp->ae_exp_cnt,
		p_hdr_exp->readout_len_lc,
		p_hdr_exp->read_margin_lc,
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			fs_inst[idx].readout_min_fl_lc),
		fs_inst[idx].readout_min_fl_lc,
		convert2TotalTime(
			fs_inst[idx].lineTimeInNs,
			*shutter_lc),
		*shutter_lc);
// #endif // REDUCE_FS_ALGO_LOG

}


void fs_alg_set_sync_with_diff(unsigned int idx, unsigned int diff_us)
{
	fs_inst[idx].custom_bias_us = diff_us;

	LOG_MUST(
		"NOTICE: [%u] ID:%#x(sidx:%u), set sync with diff:%u (us)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].custom_bias_us);
}


void fs_alg_set_streaming_st_data(
	unsigned int idx, struct fs_streaming_st (*pData))
{
	fs_inst[idx].sensor_id = pData->sensor_id;
	fs_inst[idx].sensor_idx = pData->sensor_idx;
	fs_inst[idx].tg = pData->tg;
	fs_inst[idx].fl_active_delay = pData->fl_active_delay;
	fs_inst[idx].def_min_fl_lc = pData->def_fl_lc;
	fs_inst[idx].max_fl_lc = pData->max_fl_lc;
	fs_inst[idx].def_shutter_lc = pData->def_shutter_lc;
	fs_inst[idx].margin_lc = pData->margin_lc;

	fs_inst[idx].pclk = pData->pclk;
	fs_inst[idx].linelength = pData->linelength;
	fs_inst[idx].lineTimeInNs = pData->lineTimeInNs;


	if (fs_inst[idx].fl_active_delay < 2
		|| fs_inst[idx].fl_active_delay > 3) {

		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), get non valid frame_time_delay_frame:%u (must be 2 or 3), plz check sensor driver for getting correct value\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			fs_inst[idx].fl_active_delay);
	}


	/* for first run, assume the hdr exp not be changed */
	fs_inst[idx].hdr_exp = pData->hdr_exp;

	/* hdr exp settings, overwrite shutter_lc value (equivalent shutter) */
	fs_alg_set_hdr_exp_st_data(idx, &pData->def_shutter_lc, &pData->hdr_exp);

	/* init frame length record st data (if needed) */
	fs_alg_init_fl_rec_st(idx);

	fs_alg_dump_streaming_data(idx);
}


void fs_alg_set_perframe_st_data(
	unsigned int idx, struct fs_perframe_st (*pData))
{
	//fs_inst[idx].sensor_id = pData->sensor_id;
	//fs_inst[idx].sensor_idx = pData->sensor_idx;
	fs_inst[idx].min_fl_lc = pData->min_fl_lc;
	fs_inst[idx].shutter_lc = pData->shutter_lc;
	fs_inst[idx].margin_lc = pData->margin_lc;
	fs_inst[idx].flicker_en = pData->flicker_en;
	fs_inst[idx].pclk = pData->pclk;
	fs_inst[idx].linelength = pData->linelength;
	fs_inst[idx].lineTimeInNs = pData->lineTimeInNs;
	fs_inst[idx].readout_time_us = pData->readout_time_us;

	fs_inst[idx].prev_readout_min_fl_lc = fs_inst[idx].readout_min_fl_lc;
	fs_inst[idx].readout_min_fl_lc = 0;

	fs_inst[idx].req_id = pData->req_id;

	/* hdr exp settings, overwrite shutter_lc value (equivalent shutter) */
	fs_alg_set_hdr_exp_st_data(idx, &pData->shutter_lc, &pData->hdr_exp);


	if (fs_inst[idx].margin_lc == 0) {
		LOG_MUST(
			"WARNING: [%u] ID:%#x(sidx:%u), get non valid margin_lc:%u, plz check sensor driver for getting correct value\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			fs_inst[idx].margin_lc);
	}


#ifndef REDUCE_FS_ALGO_LOG
	fs_alg_dump_perframe_data(idx);
#endif // REDUCE_FS_ALGO_LOG
}

void fs_alg_set_preset_perframe_streaming_st_data(const unsigned int idx,
	struct fs_streaming_st *p_stream_data,
	struct fs_perframe_st *p_pf_ctrl_data)
{
	/* from streaming st */
	fs_inst[idx].sensor_id = p_stream_data->sensor_id;
	fs_inst[idx].sensor_idx = p_stream_data->sensor_idx;
	fs_inst[idx].tg = p_stream_data->tg;
	fs_inst[idx].fl_active_delay = p_stream_data->fl_active_delay;
	fs_inst[idx].def_min_fl_lc = p_stream_data->def_fl_lc;
	fs_inst[idx].max_fl_lc = p_stream_data->max_fl_lc;
	fs_inst[idx].def_shutter_lc = p_stream_data->def_shutter_lc;

	/* from perframe st */
	fs_inst[idx].min_fl_lc = p_pf_ctrl_data->min_fl_lc;
	fs_inst[idx].shutter_lc = p_pf_ctrl_data->shutter_lc;
	fs_inst[idx].margin_lc = p_pf_ctrl_data->margin_lc;
	fs_inst[idx].flicker_en = p_pf_ctrl_data->flicker_en;
	fs_inst[idx].pclk = p_pf_ctrl_data->pclk;
	fs_inst[idx].linelength = p_pf_ctrl_data->linelength;
	fs_inst[idx].lineTimeInNs = p_pf_ctrl_data->lineTimeInNs;

	fs_inst[idx].prev_readout_min_fl_lc = fs_inst[idx].readout_min_fl_lc;
	fs_inst[idx].readout_min_fl_lc = 0;

	fs_inst[idx].req_id = p_pf_ctrl_data->req_id;

	/* for first run, assume the hdr exp not be changed */
	p_stream_data->hdr_exp = p_pf_ctrl_data->hdr_exp;
	fs_inst[idx].hdr_exp = p_pf_ctrl_data->hdr_exp;

	/* hdr exp settings, overwrite shutter_lc value (equivalent shutter) */
	fs_alg_set_hdr_exp_st_data(idx,
		&p_stream_data->def_shutter_lc, &p_stream_data->hdr_exp);


	/* check if get invalid data */
	if (fs_inst[idx].fl_active_delay < 2
		|| fs_inst[idx].fl_active_delay > 3) {

		LOG_MUST(
			"ERROR: [%u] ID:%#x(sidx:%u), get non valid frame_time_delay_frame:%u (must be 2 or 3), plz check sensor driver for getting correct value\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			fs_inst[idx].fl_active_delay);
	}

	if (fs_inst[idx].margin_lc == 0) {
		LOG_MUST(
			"WARNING: [%u] ID:%#x(sidx:%u), get non valid margin_lc:%u, plz check sensor driver for getting correct value\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			fs_inst[idx].margin_lc);
	}


	fs_alg_dump_fs_inst_data(idx);
}


void fs_alg_set_debug_info_sof_cnt(const unsigned int idx,
	const unsigned int sof_cnt)
{
	fs_inst[idx].sof_cnt = sof_cnt;
}


void fs_alg_reset_vsync_data(const unsigned int idx)
{
	unsigned int i = 0;

	fs_inst[idx].vsyncs = 0;
	fs_inst[idx].last_vts = 0;
	fs_inst[idx].cur_tick = 0;

	for (i = 0; i < VSYNCS_MAX; ++i)
		fs_inst[idx].timestamps[i] = 0;
}


void fs_alg_reset_fs_inst(unsigned int idx)
{
	struct FrameSyncInst clear_fs_inst_st = {0};

	fs_inst[idx] = clear_fs_inst_st;


#ifndef REDUCE_FS_ALGO_LOG
	LOG_INF("clear idx:%u data. (all to zero)\n", idx);
#endif // REDUCE_FS_ALGO_LOG


#ifdef SUPPORT_FS_NEW_METHOD
	fs_alg_reset_fs_sa_inst(idx);
#endif // SUPPORT_FS_NEW_METHOD
}


/*
 * receive frame record data from fs_drv
 *
 * fs algo will use these information to predict current and
 *     next framelength when calculating vsync diff.
 */
void fs_alg_set_frame_record_st_data(
	unsigned int idx, struct frame_record_st data[])
{
	unsigned int i = 0;


	/* 1. set/update frame recoder data */
	for (i = 0; i < RECORDER_DEPTH; ++i)
		fs_inst[idx].recs[i] = data[i];


	/* 2. update predicted frame length by new frame recorder data */
	calc_predicted_frame_length(idx);


	/* 3. set frame measurement predicted frame length */
	// frm_set_frame_measurement(
	//	idx, fs_inst[idx].vsyncs,
	//	fs_inst[idx].predicted_fl_us[0],
	//	fs_inst[idx].predicted_fl_lc[0],
	//	fs_inst[idx].predicted_fl_us[1],
	//	fs_inst[idx].predicted_fl_lc[1]);


#if defined(TRACE_FS_FREC_LOG)
	LOG_MUST(
		"[%u] ID:%#x(sidx:%u), tg:%u, frecs: (0:%u/%u), (1:%u/%u), (2:%u/%u), (3:%u/%u) (fl_lc/shut_lc), pred_fl(curr:%u(%u), next:%u(%u))(%u), margin_lc:%u, fdelay:%u\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		fs_inst[idx].tg,
		*fs_inst[idx].recs[0].framelength_lc,
		*fs_inst[idx].recs[0].shutter_lc,
		*fs_inst[idx].recs[1].framelength_lc,
		*fs_inst[idx].recs[1].shutter_lc,
		*fs_inst[idx].recs[2].framelength_lc,
		*fs_inst[idx].recs[2].shutter_lc,
		*fs_inst[idx].recs[3].framelength_lc,
		*fs_inst[idx].recs[3].shutter_lc,
		fs_inst[idx].predicted_fl_us[0],
		fs_inst[idx].predicted_fl_lc[0],
		fs_inst[idx].predicted_fl_us[1],
		fs_inst[idx].predicted_fl_lc[1],
		fs_inst[idx].lineTimeInNs,
		fs_inst[idx].margin_lc,
		fs_inst[idx].fl_active_delay);
#endif
}


void fs_alg_sa_notify_setup_all_frame_info(unsigned int idx)
{
	// fs_alg_sa_dump_dynamic_para(idx);
	fs_alg_setup_frame_monitor_fmeas_data(idx);
}


void fs_alg_sa_notify_vsync(unsigned int idx)
{
	unsigned int query_ts_idx[1] = {idx};


	/* get timestamp info and calibrate frame recorder data */
	fs_inst[idx].is_nonvalid_ts = fs_alg_get_vsync_data(query_ts_idx, 1);

#if !defined(REDUCE_FS_ALGO_LOG)
	if (fs_inst[idx].is_nonvalid_ts) {
		LOG_INF(
			"WARNING: [%u] ID:%#x(sidx:%u), get Vsync data ERROR, SA ctrl\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx);
	}
#endif // REDUCE_FS_ALGO_LOG

	fs_alg_sa_dump_dynamic_para(idx);
}
/******************************************************************************/





/******************************************************************************/
// Frame Sync Algorithm function
/******************************************************************************/

/* return "0" -> done; "non 0" -> error ? */
unsigned int fs_alg_get_vsync_data(unsigned int solveIdxs[], unsigned int len)
{
	unsigned int i = 0, j = 0;

	unsigned int query_tg_ts[TG_MAX_NUM];
	struct vsync_rec vsync_recs = {0};


	/* according to "solve Idx", get correct "TG / sensor_idx" */
	for (i = 0; i < len; ++i)
		query_tg_ts[i] = fs_inst[solveIdxs[i]].tg;


	/* call Frame Monitor API to get vsync data from CCU */
	if (frm_query_vsync_data(query_tg_ts, len, &vsync_recs)) {
		LOG_PR_WARN("ERROR: query vsync data from CCU error\n");
		return 1;
	}


	/* keep cur_tick and tick_factor value */
	cur_tick = vsync_recs.cur_tick;
	tick_factor = vsync_recs.tick_factor;


#ifndef REDUCE_FS_ALGO_LOG
	LOG_INF("cur_tick:%u, tick_factor:%u\n",
		cur_tick,
		tick_factor);
#endif // REDUCE_FS_ALGO_LOG


	/* keep vsync and last_vts data */
	for (i = 0; i < len; ++i) {
		if (fs_inst[solveIdxs[i]].tg != vsync_recs.recs[i].id) {
			LOG_PR_WARN(
				"ERROR: [%u].tg:%u not sync to v_recs[%u].tg:%u\n",
				solveIdxs[i],
				fs_inst[solveIdxs[i]].tg,
				i,
				vsync_recs.recs[i].id);

			return 1;
		}

		fs_inst[solveIdxs[i]].vsyncs =
					vsync_recs.recs[i].vsyncs;
		fs_inst[solveIdxs[i]].last_vts =
					vsync_recs.recs[i].timestamps[0];
		fs_inst[solveIdxs[i]].cur_tick =
					vsync_recs.cur_tick;

		for (j = 0; j < VSYNCS_MAX; ++j) {
			fs_inst[solveIdxs[i]].timestamps[j] =
					vsync_recs.recs[i].timestamps[j];
		}


//#ifndef REDUCE_FS_ALGO_LOG
		LOG_PF_INF(
			"[%u] ID:%#x(sidx:%u), tg:%u, vsyncs:%u, last_vts:%u, cur_tick:%u, ts(%u/%u/%u/%u)\n",
			solveIdxs[i],
			fs_inst[solveIdxs[i]].sensor_id,
			fs_inst[solveIdxs[i]].sensor_idx,
			fs_inst[solveIdxs[i]].tg,
			fs_inst[solveIdxs[i]].vsyncs,
			fs_inst[solveIdxs[i]].last_vts,
			fs_inst[solveIdxs[i]].cur_tick,
			fs_inst[solveIdxs[i]].timestamps[0],
			fs_inst[solveIdxs[i]].timestamps[1],
			fs_inst[solveIdxs[i]].timestamps[2],
			fs_inst[solveIdxs[i]].timestamps[3]);
//#endif // REDUCE_FS_ALGO_LOG


		set_valid_for_using_vsyncs_recs_data(solveIdxs[i], 1);


		/* using frame monitor to get new vsync information */
		/* and make frame recorder data be correct */
		calibrate_recs_data_by_vsyncs(solveIdxs[i]);
	}


	return 0;
}


static void do_fps_sync(unsigned int solveIdxs[], unsigned int len)
{
	unsigned int i = 0;
	unsigned int target_fl_us = 0;

	int ret = 0;
	char *log_buf = NULL;

#ifdef FS_UT
	log_buf = calloc(LOG_BUF_STR_LEN, sizeof(char));
#else
	log_buf = kcalloc(LOG_BUF_STR_LEN, sizeof(char), GFP_KERNEL);
#endif // FS_UT

	if (log_buf == NULL) {
		LOG_PR_ERR("ERROR: log_buf allocate memory failed\n");

		return;
	}

	log_buf[0] = '\0';


	/* 1. calc each fl_us for each sensor, take max fl_us as target_fl_us */
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];
		unsigned int fl_us = 0, fl_lc = 0;

		fl_lc = calc_min_fl_lc(idx, fs_inst[idx].min_fl_lc);
		fl_us = convert2TotalTime(fs_inst[idx].lineTimeInNs, fl_lc);


		ret = snprintf(log_buf + strlen(log_buf),
			LOG_BUF_STR_LEN - strlen(log_buf),
			"[%u] ID:%#x(sidx:%u), fl:%u(%u) (%u/%u/%u/%u/%u, %u); ",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			fl_us,
			fl_lc,
			fs_inst[idx].shutter_lc,
			fs_inst[idx].margin_lc,
			fs_inst[idx].min_fl_lc,
			fs_inst[idx].readout_min_fl_lc,
			*fs_inst[idx].recs[0].framelength_lc,
			fs_inst[idx].lineTimeInNs);

		if (ret < 0)
			LOG_INF("ERROR: LOG encoding error, ret:%d\n", ret);


		if (fl_us > target_fl_us)
			target_fl_us = fl_us;
	}


	/* 2. use target_fl_us as output_fl_us for each sensor */
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];

		/* set to min framelength */
		/* all framelength operation must use this API */
		set_fl_us(idx, target_fl_us);
	}

	target_min_fl_us = target_fl_us;


	ret = snprintf(log_buf + strlen(log_buf),
		LOG_BUF_STR_LEN - strlen(log_buf),
		"FL sync to %u (us)",
		target_fl_us);

	if (ret < 0)
		LOG_INF("ERROR: LOG encoding error, ret:%d\n", ret);


	LOG_INF("%s\n", log_buf);


#ifdef FS_UT
	free(log_buf);
#else
	kfree(log_buf);
#endif // FS_UT
}


static void adjust_vsync_diff(unsigned int solveIdxs[], unsigned int len)
{
	unsigned int i = 0;

	unsigned int min_vtick_diff = (0 - 1);    // 0xffffffff
	unsigned int target_vts = 0;
	unsigned int pf_ctrl_timing_issue[SENSOR_MAX_NUM] = {0};

	unsigned int anti_flicker_en = 0;
	unsigned int flicker_vdiff[SENSOR_MAX_NUM] = {0};
	unsigned int max_flicker_vdiff = 0;


	/* for snprintf and memory allocate */
	int ret = 0;
	char *log_buf[SENSOR_MAX_NUM] = {NULL};

	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];

#ifdef FS_UT
		log_buf[idx] = calloc(LOG_BUF_STR_LEN, sizeof(char));
#else
		log_buf[idx] = kcalloc(LOG_BUF_STR_LEN, sizeof(char),
					GFP_KERNEL);
#endif // FS_UT

		if (log_buf[idx] == NULL) {
			LOG_PR_ERR(
				"ERROR: [%u] log_buf allocate memory failed\n",
				idx);

			/* return, and free alloc memory */
			goto free_alloc_mem;
		} else
			log_buf[idx][0] = '\0';
	}


	/* 0. check vsync timestamp (preventing last vts is "0") */
	if (check_fs_inst_vsync_data_valid(solveIdxs, len) == 0) {
		LOG_PR_WARN(
			"ERROR: Incorrect vsync timestamp detected, not adjust vsync diff\n"
			);

		/* return, and free alloc memory */
		goto free_alloc_mem;
	}


	/* 1. calculate vsync diff for all */
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];


		fs_inst[idx].vdiff =
			fs_inst[idx].last_vts * tick_factor - cur_tick;


		if (fs_inst[idx].vdiff < min_vtick_diff)
			min_vtick_diff = fs_inst[idx].vdiff;
	}
	// LOG_INF("min vtick diff:%u\n", min_vtick_diff);
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];


		fs_inst[idx].vdiff -= min_vtick_diff;


		/* prevent for floating point exception */
		if (tick_factor != 0)
			fs_inst[idx].vdiff /= tick_factor;

		// fs_inst[idx].predicted_vts = fs_inst[idx].vdiff;


		ret = snprintf(log_buf[idx] + strlen(log_buf[idx]),
			LOG_BUF_STR_LEN - strlen(log_buf[idx]),
			"[%u] ID:%#x(sidx:%u), tg:%u, vsyncs:%u, vdiff:%u, ",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			fs_inst[idx].tg,
			fs_inst[idx].vsyncs,
			fs_inst[idx].vdiff);

		if (ret < 0)
			LOG_INF("ERROR: LOG encoding error, ret:%d\n", ret);
	}


	/* 2. predict current and next vsync timestamp */
	/*    and calculate vsync timestamp bias */
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];
		unsigned int *predicted_fl_us = fs_inst[idx].predicted_fl_us;
		unsigned int vts_bias = 0, vts_bias_us = 0;

		calc_predicted_frame_length(idx);

		fs_inst[idx].vdiff += predicted_fl_us[0] + predicted_fl_us[1];


		vts_bias = calc_vts_sync_bias(idx);
		vts_bias_us =
			convert2TotalTime(fs_inst[idx].lineTimeInNs, vts_bias);

		fs_inst[idx].vdiff += vts_bias_us;


		ret = snprintf(log_buf[idx] + strlen(log_buf[idx]),
			LOG_BUF_STR_LEN - strlen(log_buf[idx]),
			"pred_fl(c:%u(%u), n:%u(%u)), fdelay:%u, bias(%u(%u), opt:%u), ",
			predicted_fl_us[0],
			fs_inst[idx].predicted_fl_lc[0],
			predicted_fl_us[1],
			fs_inst[idx].predicted_fl_lc[1],
			fs_inst[idx].fl_active_delay,
			vts_bias_us,
			vts_bias,
			fs_inst[idx].sync_type);

		if (ret < 0)
			LOG_INF("ERROR: LOG encoding error, ret:%d\n", ret);


#ifdef FS_UT
		/* update frame monitor current predicted framelength data */
		frm_update_predicted_curr_fl_us(idx, predicted_fl_us[0]);
		frm_update_next_vts_bias_us(idx, vts_bias_us);
#endif // FS_UT
	}


	/* 3. calculate diff of predicted_vts */
	/* 3.1 find target timestamp to sync */
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];

		if (fs_inst[idx].vdiff > target_vts)
			target_vts = fs_inst[idx].vdiff;
	}
	/* 3.2 extend frame length to align target timestamp */
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];
		unsigned int pred_vdiff = 0;
		unsigned int fl_lc = 0;


		/* detect pf ctrl timing error */
		pred_vdiff = target_vts - fs_inst[idx].vdiff;
		if (check_timing_critical_section(
				pred_vdiff, target_min_fl_us)) {

			/* pf ctrl trigger in critiacal section */
			/* maybe the coming soon timestamp is in sync */
			/* => Do not adjust vdiff, only use fps sync result */

			pf_ctrl_timing_issue[idx] = 1UL << 1;
			pred_vdiff = 0;

#ifndef REDUCE_FS_ALGO_LOG
			LOG_INF(
				"WARNING: [%u] ID:%#x(sidx:%u), in timing_cs, coming Vsync is in sync, set pred_vdiff:%u\n",
				idx,
				fs_inst[idx].sensor_id,
				fs_inst[idx].sensor_idx,
				pred_vdiff);
#endif // REDUCE_FS_ALGO_LOG
		} else {
			if (pred_vdiff >= target_min_fl_us) {
				/* pf ctrl timing error */
				/* prevent framelength getting bigger */
				/* => Do not adjust vdiff, only use fps sync result */

				pf_ctrl_timing_issue[idx] = 1;
				pred_vdiff = 0;
			}
		}


		/* add diff and set framelength */
		/* all framelength operation must use this API */
		fl_lc = fs_inst[idx].output_fl_us + pred_vdiff;

		set_fl_us(idx, fl_lc);


		ret = snprintf(log_buf[idx] + strlen(log_buf[idx]),
			LOG_BUF_STR_LEN - strlen(log_buf[idx]),
			"pred_vdiff:%u(t_issue:%u), flk_en:%u, out_fl:%u(%u)",
			target_vts - fs_inst[idx].vdiff,
			pf_ctrl_timing_issue[idx],
			fs_inst[idx].flicker_en,
			fs_inst[idx].output_fl_us,
			fs_inst[idx].output_fl_lc);

		if (ret < 0)
			LOG_INF("ERROR: LOG encoding error, ret:%d\n", ret);
	}


	/* TODO: add check perframe ctrl timing error */


	/* 4. anti-flicker */

	/* 4.1 check anti-flicker enable, */
	/*     and find out max flicker vdiff simultaneously */
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];


		/* check anti-flicker enable */
		if (fs_inst[idx].flicker_en == 0)
			continue;

		/* for anti-flicker enable case */
		anti_flicker_en |= fs_inst[idx].flicker_en;


		/* calculate anti-flicker vdiff */
		/*      flk vdiff = 0 => not flk fl */
		/*      flk vdiff > 0 => flk fl, adjust fl */
		flicker_vdiff[idx] =
			get_anti_flicker_fl(fs_inst[idx].output_fl_us) -
			fs_inst[idx].output_fl_us;

		if (flicker_vdiff[idx] == 0)
			continue;


		/* get maximum flicker vdiff */
		if (flicker_vdiff[idx] > max_flicker_vdiff)
			max_flicker_vdiff = flicker_vdiff[idx];
	}


	if (anti_flicker_en == 0) {
		for (i = 0; i < len; ++i) {
			unsigned int idx = solveIdxs[i];

			LOG_INF("%s\n", log_buf[idx]);
		}

		/* return, and free alloc memory */
		goto free_alloc_mem;
	}


	/* 4.2 add max anti-flicker vdiff to (all) sony sensor */
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];
		unsigned int fl_lc = 0;


		/* add max flk vdiff and set framelength */
		/* all framelength operation must use this API */
		fl_lc = fs_inst[idx].output_fl_us + max_flicker_vdiff;
		set_fl_us(idx, fl_lc);


		ret = snprintf(log_buf[idx] + strlen(log_buf[idx]),
			LOG_BUF_STR_LEN - strlen(log_buf[idx]),
			", +flk:%u, 10xFPS:%u, out_fl:%u(%u)",
			max_flicker_vdiff,
			(unsigned int)(10000000/fs_inst[idx].output_fl_us),
			fs_inst[idx].output_fl_us,
			fs_inst[idx].output_fl_lc);

		if (ret < 0)
			LOG_INF("ERROR: LOG encoding error, ret:%d\n", ret);
	}


	for (i = 0; i < len; ++i)
		LOG_INF("%s\n", log_buf[solveIdxs[i]]);


free_alloc_mem:
	for (i = 0; i < len; ++i) {
		if (log_buf[solveIdxs[i]] == NULL)
			continue;
#ifdef FS_UT
		free(log_buf[solveIdxs[i]]);
#else
		kfree(log_buf[solveIdxs[i]]);
#endif // FS_UT
	}
}


/* return "0" -> done; "non 0" -> error ? */
unsigned int fs_alg_solve_frame_length(
	unsigned int solveIdxs[],
	unsigned int framelength_lc[], unsigned int len)
{
	unsigned int i = 0;


#ifndef REDUCE_FS_ALGO_LOG
	LOG_INF("%u sensors do frame sync\n", len);
#endif // REDUCE_FS_ALGO_LOG


	/* boundary checking for how many sensor sync */
	// if (len > SENSOR_MAX_NUM)
	if (len > TG_MAX_NUM)
		return 1;


#ifndef REDUCE_FS_ALGO_LOG
	/* dump information */
	for (i = 0; i < len; ++i)
		dump_fs_inst_data(solveIdxs[i]);

	// dump_all_fs_inst_data();
#endif // REDUCE_FS_ALGO_LOG


	/* 1. get Vsync data by Frame Monitor */
	if (fs_alg_get_vsync_data(solveIdxs, len)) {
		LOG_PR_WARN("Get Vsync data ERROR\n");
		return 1;
	}


	/* 2. FPS Sync */
	do_fps_sync(solveIdxs, len);


	/* 3. adjust Vsync diff */
	adjust_vsync_diff(solveIdxs, len);


	/* 4. copy framelength_lc results */
	for (i = 0; i < len; ++i) {
		unsigned int idx = solveIdxs[i];

		framelength_lc[idx] = fs_inst[idx].output_fl_lc;


		/* 5. set NOT valid for using vsyncs / recs data for */
		/*    preventing calculation error */
		// set_valid_for_using_vsyncs_recs_data(idx, 0);
	}


	return 0;
}


#ifdef SUPPORT_FS_NEW_METHOD
static void do_fps_sync_sa(
	unsigned int idx, unsigned int m_idx,
	unsigned int valid_sync_bits, unsigned int async_slave_bits,
	struct FrameSyncDynamicPara *p_para)
{
	unsigned int i = 0, fl_us = 0, fl_lc = 0, flk_diff = 0;
	unsigned int f_cell = 1, sync_flk_en = 0;
	unsigned int min_fl_us = 0, target_min_fl_us = 0, out_fl_us = 0;
	unsigned int min_fl_us_buf[SENSOR_MAX_NUM] = {0};
	unsigned int target_min_fl_us_buf[SENSOR_MAX_NUM] = {0};
	unsigned int magic_num_buf[SENSOR_MAX_NUM] = {0};


	f_cell = get_valid_frame_cell_size(idx);
	sync_flk_en = check_sync_flicker_en_status(idx);


	/* 1. find min_fl that this sensor support */
	/*    and check anti-flicker frame length */
	fl_lc = calc_min_fl_lc(idx, fs_inst[idx].min_fl_lc);
	fl_us = convert2TotalTime(fs_inst[idx].lineTimeInNs, fl_lc);
	p_para->pure_min_fl_us = (fl_us * f_cell);

	flk_diff = fs_alg_sa_get_flk_diff_and_fl(idx, &fl_us, sync_flk_en);
	p_para->min_fl_us = target_min_fl_us = min_fl_us = (fl_us * f_cell);


	/* 2. find max min_fl in all other sensor that doing frame-sync */
	fs_alg_sa_query_all_min_fl_us(
		min_fl_us_buf, target_min_fl_us_buf, magic_num_buf);

	/* select sensor that valid (streaming + set sync) for sync */
	for (i = 0; i < SENSOR_MAX_NUM; ++i) {
		/* TODO: [NEED FIX] bypass idx that set to async_slave */
		/*       ONLY when min_FL of async_slave is not same as async_master */

		/* select sensor that valid (streaming + set sync) for sync */
		if (((valid_sync_bits >> i) & 1UL) == 1
			&& ((async_slave_bits >> i) & 1UL) == 0) {

#if !defined(FORCE_ADJUST_SMALLER_DIFF)
			if (fs_inst[idx].fl_active_delay
				>= fs_inst[i].fl_active_delay) {
#endif // FORCE_ADJUST_SMALLER_DIFF

				/* find maximum min_fl */
				if (min_fl_us_buf[i] > min_fl_us)
					min_fl_us =
						min_fl_us_buf[i];

				/* find maximum target_min_fl */
				if (target_min_fl_us_buf[i] > target_min_fl_us)
					target_min_fl_us =
						target_min_fl_us_buf[i];

#if !defined(FORCE_ADJUST_SMALLER_DIFF)
			}
#endif // FORCE_ADJUST_SMALLER_DIFF

		}
	}

	/* chk for preventing FL not retracting after extending */
	out_fl_us = (min_fl_us < target_min_fl_us)
		? min_fl_us : target_min_fl_us;


	/* 3. check anti-flicker frame length of FPS sync */
	flk_diff = fs_alg_sa_get_flk_diff_and_fl(idx, &out_fl_us, sync_flk_en);

	p_para->target_min_fl_us = out_fl_us;


	/* N. setup FPS sync result */
	/* all SA mode framelength operation must use this API */
	fs_alg_sa_update_fl_us(idx, (out_fl_us/f_cell), p_para);


	LOG_INF(
		"[%u] ID:%#x(sidx:%u), #%u, m_idx:%u, FL sync to %u(%u), fl:%u(%u) (%u/%u/%u/%u/%u, %u), FL((#%u[%u/%u],%u/#%u[%u/%u],%u/#%u[%u/%u],%u/#%u[%u/%u],%u/#%u[%u/%u],%u), valid:%d, [%u/%u]), flk_en:[%u/%u/%u/%u/%u](+%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		p_para->magic_num,
		m_idx,
		out_fl_us,
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			out_fl_us),
		fl_us,
		fl_lc,
		fs_inst[idx].shutter_lc,
		fs_inst[idx].margin_lc,
		fs_inst[idx].min_fl_lc,
		fs_inst[idx].readout_min_fl_lc,
		*fs_inst[idx].recs[0].framelength_lc,
		fs_inst[idx].lineTimeInNs,
		magic_num_buf[0],
		min_fl_us_buf[0],
		target_min_fl_us_buf[0],
		fs_inst[0].fl_active_delay,
		magic_num_buf[1],
		min_fl_us_buf[1],
		target_min_fl_us_buf[1],
		fs_inst[1].fl_active_delay,
		magic_num_buf[2],
		min_fl_us_buf[2],
		target_min_fl_us_buf[2],
		fs_inst[2].fl_active_delay,
		magic_num_buf[3],
		min_fl_us_buf[3],
		target_min_fl_us_buf[3],
		fs_inst[3].fl_active_delay,
		magic_num_buf[4],
		min_fl_us_buf[4],
		target_min_fl_us_buf[4],
		fs_inst[4].fl_active_delay,
		valid_sync_bits,
		min_fl_us,
		target_min_fl_us,
		fs_inst[0].flicker_en,
		fs_inst[1].flicker_en,
		fs_inst[2].flicker_en,
		fs_inst[3].flicker_en,
		fs_inst[4].flicker_en,
		flk_diff);
}


static void adjust_vsync_diff_sa(
	unsigned int idx, unsigned int m_idx, int sa_method,
	struct FrameSyncDynamicPara *p_para)
{
	long long adjust_diff = 0;
	unsigned int out_fl_us = 0;
	struct FrameSyncDynamicPara m_para = {0};
	struct FrameSyncDynamicPara *p_para_m = &m_para;

#if !defined(REDUCE_FS_ALGO_LOG)
	unsigned int flk_diff = 0;
#endif // REDUCE_FS_ALGO_LOG

#if !defined(FS_UT)
	unsigned int listen_vsync_alg = 0, auto_listen_ext_vsync = 0;
#endif // FS_UT


	/* this should be done when fl be updated */
	// fs_alg_sa_prepare_dynamic_para(idx, p_para);


	/* master only do fps sync */
	if (idx == m_idx) {
		/* TODO: if want/need any extra operation on master, do here */
		return;
	}


	/* get master dynamic para */
	fs_alg_sa_get_dynamic_para(m_idx, p_para_m);


	/* check all needed info is valid or not for preventing error */
	if (fs_alg_sa_dynamic_paras_checker(idx, m_idx, p_para, p_para_m)) {
		LOG_INF(
			"NOTICE: [%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), do not adjust vsync diff, out_fl:%u(%u)\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			p_para->magic_num,
			p_para_m->magic_num,
			m_idx,
			fs_inst[idx].output_fl_us,
			convert2LineCount(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].output_fl_us)
		);

		return;
	}


	/* calculate/get suitable slave adjust diff */
	adjust_diff = fs_alg_sa_adjust_slave_diff_resolver(
		m_idx, idx, p_para_m, p_para);

	if (adjust_diff == 0) {
		fs_sa_request_switch_master(idx);

#if !defined(REDUCE_FS_ALGO_LOG)
		LOG_MUST(
			"[%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), request switch to master sensor, out_fl:%u(%u)\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			p_para->magic_num,
			p_para_m->magic_num,
			m_idx,
			fs_inst[idx].output_fl_us,
			convert2LineCount(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].output_fl_us)
		);
#endif // REDUCE_FS_ALGO_LOG

		return;
	}


#if !defined(FS_UT)
	auto_listen_ext_vsync =
		fs_con_get_usr_auto_listen_ext_vsync();

	if (auto_listen_ext_vsync > 0) {
		/* take tolerance/4 as estimated error */
		listen_vsync_alg = (adjust_diff > (FS_TOLERANCE/4)) ? 1 : 0;
		fs_con_set_listen_vsync_alg_cfg(listen_vsync_alg);
	}
#endif // FS_UT


	out_fl_us = fs_inst[idx].output_fl_us + adjust_diff;


#if !defined(REDUCE_FS_ALGO_LOG)
	flk_diff = fs_alg_sa_get_flk_diff_and_fl(idx, &out_fl_us, 0);
#else
	fs_alg_sa_get_flk_diff_and_fl(idx, &out_fl_us, 0);
#endif // REDUCE_FS_ALGO_LOG


#if !defined(REDUCE_FS_ALGO_LOG)
	LOG_MUST(
		"[%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), out_fl:%u(%u), flk_en:%u(+%u), set listen_ext_vsync:%u(auto_listen_ext_vsync:%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		p_para->magic_num,
		p_para_m->magic_num,
		m_idx,
		out_fl_us,
		convert2LineCount(
			fs_inst[idx].lineTimeInNs,
			out_fl_us),
		fs_inst[idx].flicker_en,
		flk_diff,
		listen_vsync_alg,
		auto_listen_ext_vsync);
#endif // REDUCE_FS_ALGO_LOG


	fs_alg_sa_update_fl_us(idx, out_fl_us, p_para);
}


static void adjust_async_vsync_diff_sa(
	unsigned int idx, unsigned int m_idx,
	struct FrameSyncDynamicPara *p_para)
{
	unsigned int fl_lc = 0, fl_us = 0, flk_diff = 0, f_cell = 0, out_fl_us = 0;
	unsigned int f_cell_m = get_valid_frame_cell_size(m_idx);
	long long adjust_diff = 0;
	long long ts_diff_m = 0, ts_diff_s = 0;
	struct FrameSyncDynamicPara m_para = {0};
	struct FrameSyncDynamicPara *p_para_m = &m_para;


	f_cell = get_valid_frame_cell_size(idx);

	/* 1. find min_fl that this sensor support */
	/*    and check anti-flicker frame length */
	fl_lc = calc_min_fl_lc(idx, fs_inst[idx].min_fl_lc);
	fl_us = convert2TotalTime(fs_inst[idx].lineTimeInNs, fl_lc);
	p_para->pure_min_fl_us = (fl_us * f_cell);

	flk_diff = fs_alg_sa_get_flk_diff_and_fl(idx, &fl_us, 0);

	p_para->min_fl_us = (fl_us * f_cell);
	p_para->target_min_fl_us = (fl_us * f_cell);
	out_fl_us = p_para->min_fl_us;

	fs_alg_sa_update_fl_us(idx, (out_fl_us/f_cell), p_para);


	if (idx == m_idx)
		return;


	/* get master dynamic para */
	fs_alg_sa_get_dynamic_para(m_idx, p_para_m);


	/* check all needed info is valid or not for preventing error */
	if (fs_alg_sa_dynamic_paras_checker(idx, m_idx, p_para, p_para_m)) {
		LOG_INF(
			"ERROR: [%u] ID:%#x(sidx:%u), #%u/#%u(m_idx:%u), do not adjust vsync diff, out_fl:%u(%u)\n",
			idx,
			fs_inst[idx].sensor_id,
			fs_inst[idx].sensor_idx,
			p_para->magic_num,
			p_para_m->magic_num,
			m_idx,
			fs_inst[idx].output_fl_us,
			convert2LineCount(
				fs_inst[idx].lineTimeInNs,
				fs_inst[idx].output_fl_us)
		);

		return;
	}


	/* calculate/get current receive timestamp diff */
	fs_alg_sa_calc_m_s_ts_diff(p_para_m, p_para,
		&ts_diff_m, &ts_diff_s);

	/* calculate master/slave adjust_diff */
	adjust_diff =
		fs_alg_sa_calc_adjust_diff_async(
			m_idx, idx, ts_diff_m, ts_diff_s,
			p_para_m, p_para);

	if (check_timing_critical_section(
		adjust_diff, (p_para_m->stable_fl_us * f_cell_m))) {

		adjust_diff = 0;
	}


	/* update slave all per-frame status variables to dynamic_para struct */
	p_para->ref_m_idx_magic_num = p_para_m->magic_num;
	p_para->adj_diff_m = 0;
	p_para->adj_diff_s = adjust_diff;
	p_para->adj_or_not = 1;
	p_para->chg_master = 0;
	p_para->ask_for_chg = 0;


	LOG_MUST(
		"[%u] ID:%#x(sidx:%u), #%u/#%u(async_m_idx:%u), adjust_diff:%lld, flk_en:%u(+%u), ts(%lld/%lld), delta(%u/%u), [(c:%u/n:%u/s:%u/e:%u/t:%u) / (c:%u/n:%u/s:%u/e:%u/t:%u)], f_cell(%u/%u), fdelay(%u/%u), ts_abs(%u/%u)\n",
		idx,
		fs_inst[idx].sensor_id,
		fs_inst[idx].sensor_idx,
		p_para->magic_num,
		p_para_m->magic_num,
		m_idx,
		adjust_diff,
		fs_inst[idx].flicker_en,
		flk_diff,
		ts_diff_s,
		ts_diff_m,
		p_para->delta,
		p_para_m->delta,
		p_para->pred_fl_us[0],
		p_para->pred_fl_us[1],
		p_para->stable_fl_us,
		p_para->ts_bias_us,
		p_para->tag_bias_us,
		p_para_m->pred_fl_us[0],
		p_para_m->pred_fl_us[1],
		p_para_m->stable_fl_us,
		p_para_m->ts_bias_us,
		p_para_m->tag_bias_us,
		f_cell,
		f_cell_m,
		fs_inst[idx].fl_active_delay,
		fs_inst[m_idx].fl_active_delay,
		p_para->last_ts,
		// fs_sa_inst.dynamic_paras[s_idx].last_ts,
		fs_sa_inst.dynamic_paras[m_idx].last_ts
	);


	out_fl_us = fs_inst[idx].output_fl_us + adjust_diff;

#if !defined(REDUCE_FS_ALGO_LOG)
	flk_diff = fs_alg_sa_get_flk_diff_and_fl(idx, &out_fl_us, 0);
#else
	fs_alg_sa_get_flk_diff_and_fl(idx, &out_fl_us, 0);
#endif // REDUCE_FS_ALGO_LOG


	fs_alg_sa_update_fl_us(idx, out_fl_us, p_para);
}


/*
 * Every sensor will call into this function
 *
 * return: (0/1) for (no error/some error happened)
 *
 * input:
 *     struct fs_sa_cfg
 *         idx: standalone instance idx
 *         m_idx: master instance idx
 *         valid_sync_bits: all valid for doing frame-sync instance idxs
 *         sa_method: 0 => adaptive switch master; 1 => fix master
 *
 * output:
 *     *fl_lc: pointer for output frame length
 */
unsigned int fs_alg_solve_frame_length_sa(
	const struct fs_sa_cfg *p_sa_cfg, unsigned int *fl_lc)
{
	struct FrameSyncDynamicPara para = {0};
	unsigned int ret = 0;


	FS_ATOMIC_SET(p_sa_cfg->m_idx, &fs_sa_inst.master_idx);

	/* prepare new dynamic para */
	fs_alg_sa_init_new_ctrl(p_sa_cfg, &para);


	/* 0. get Vsync data by Frame Monitor */
	ret = fs_alg_sa_get_timestamp_info(p_sa_cfg->idx, &para);
	if (ret != 0) {
		/* for set shutter with frame length API, */
		/*     give a min FL for sensor driver auto judgment */
		*fl_lc = fs_inst[p_sa_cfg->idx].min_fl_lc;
		return ret;
	}


	/* check this idx is normal sync or async slave idx */
	if (!((p_sa_cfg->async_s_bits >> p_sa_cfg->idx) & 0x01)) {
		/* 1. FPS sync */
		do_fps_sync_sa(
			p_sa_cfg->idx, p_sa_cfg->m_idx,
			p_sa_cfg->valid_sync_bits, p_sa_cfg->async_s_bits,
			&para);


		/* 2. adjust vsync diff */
		adjust_vsync_diff_sa(
			p_sa_cfg->idx, p_sa_cfg->m_idx,
			p_sa_cfg->sa_method,
			&para);

	} else {
		/* 1. adjust async slave vsync diff */
		adjust_async_vsync_diff_sa(
			p_sa_cfg->idx, p_sa_cfg->async_m_idx,
			&para);
	}


	/* 3. copy fl result out */
	*fl_lc = fs_inst[p_sa_cfg->idx].output_fl_lc;


	/* X. update dynamic para for sharing to other sensor */
	fs_alg_sa_update_dynamic_para(p_sa_cfg->idx, &para);


	return 0;
}
#endif // SUPPORT_FS_NEW_METHOD


/******************************************************************************/
