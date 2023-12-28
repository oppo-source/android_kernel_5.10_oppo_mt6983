/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#ifndef __CCCI_HIF_INTERNAL_H__
#define __CCCI_HIF_INTERNAL_H__

#include "ccci_core.h"
#include "ccci_port.h"
#include "ccci_fsm.h"
#include "ccci_hif.h"
#include "ccci_modem.h"

#define MAX_TXQ_NUM 16
#define MAX_RXQ_NUM 16

#define PACKET_HISTORY_DEPTH 16	/* must be power of 2 */

extern void *ccci_hif[CCCI_HIF_NUM];
extern struct ccci_hif_ops *ccci_hif_op[CCCI_HIF_NUM];
extern void set_ccmni_rps(unsigned long value);

struct ccci_log {
	struct ccci_header msg;
	u64 tv;
	int dropped;
};

struct ccci_hif_traffic {
#if PACKET_HISTORY_DEPTH
		struct ccci_log tx_history[MAX_TXQ_NUM][PACKET_HISTORY_DEPTH];
		struct ccci_log rx_history[MAX_RXQ_NUM][PACKET_HISTORY_DEPTH];
		int tx_history_ptr[MAX_TXQ_NUM];
		int rx_history_ptr[MAX_RXQ_NUM];
#endif
		unsigned long logic_ch_pkt_cnt[CCCI_MAX_CH_NUM];
		unsigned long logic_ch_pkt_pre_cnt[CCCI_MAX_CH_NUM];
		short seq_nums[2][CCCI_MAX_CH_NUM];

		unsigned long long latest_isr_time;
		unsigned long long latest_q_rx_isr_time[MAX_RXQ_NUM];
		unsigned long long latest_q_rx_time[MAX_RXQ_NUM];
#ifdef DPMAIF_DEBUG_LOG
		unsigned long long isr_time_bak;
		unsigned long long rx_done_isr_cnt[MAX_RXQ_NUM];
		unsigned long long rx_other_isr_cnt[MAX_RXQ_NUM];
		unsigned long long rx_full_cnt;
		unsigned long long rx_tasket_cnt;
		unsigned long long tx_done_isr_cnt[MAX_TXQ_NUM];
		unsigned long long tx_other_isr_cnt[MAX_TXQ_NUM];
#endif
		unsigned long long isr_cnt;
#ifdef DEBUG_FOR_CCB
		unsigned long long latest_ccb_isr_time;
		unsigned int last_ccif_r_ch;
#endif
		struct work_struct traffic_work_struct;
};


struct ccci_hif_ops {
	/* must-have */
	int (*send_skb)(unsigned char hif_id, int qno, struct sk_buff *skb,
		int skb_from_pool, int blocking);
	int (*give_more)(unsigned char hif_id, unsigned char qno);
	int (*write_room)(unsigned char hif_id, unsigned char qno);
	int (*start_queue)(unsigned char hif_id, unsigned char qno,
		enum DIRECTION dir);
	int (*stop_queue)(unsigned char hif_id, unsigned char qno,
		enum DIRECTION dir);
	int (*broadcast_state)(unsigned char hif_id, enum MD_STATE state);
	int (*dump_status)(unsigned char hif_id, enum MODEM_DUMP_FLAG dump_flag,
		void *buff, int length);
	int (*suspend)(unsigned char hif_id);
	int (*resume)(unsigned char hif_id);

	int (*init)(unsigned char md_id, unsigned int hif_flag);
	int (*late_init)(unsigned char hif_id);
	int (*start)(unsigned char hif_id);
	int (*pre_stop)(unsigned char hif_id);
	int (*stop)(unsigned char hif_id);
	int (*debug)(unsigned char hif_id, enum ccci_hif_debug_flg debug_id,
		int *paras);
	int (*send_data)(unsigned char hif_id, int channel_id);
	void* (*fill_rt_header)(unsigned char hif_id,
		int packet_size, unsigned int tx_ch, unsigned int txqno);

	int (*stop_for_ee)(unsigned char hif_id);
	int (*all_q_reset)(unsigned char hif_id);
	int (*clear_all_queue)(unsigned char hif_id, enum DIRECTION dir);
	int (*clear)(unsigned char hif_id);
	void (*set_clk_cg)(unsigned char md_id, unsigned int on);
	void (*hw_reset)(unsigned char md_id);
	int (*empty_query)(int qno);
};

enum RX_COLLECT_RESULT {
	ONCE_MORE,
	ALL_CLEAR,
	LOW_MEMORY,
	ERROR_STOP,
};

void ccci_md_dump_log_history(unsigned char md_id,
		struct ccci_hif_traffic *tinfo, int dump_multi_rec,
		int tx_queue_num, int rx_queue_num);
void ccci_md_add_log_history(struct ccci_hif_traffic *tinfo,
		enum DIRECTION dir, int queue_index,
		struct ccci_header *msg, int is_dropped);

#ifndef CCCI_KMODULE_ENABLE
static inline void *ccci_hif_get_by_id(unsigned char hif_id)
{
	if (hif_id >= CCCI_HIF_NUM) {
		CCCI_ERROR_LOG(-1, CORE,
		"%s  hif_id = %u\n", __func__, hif_id);
		return NULL;
	} else
		return ccci_hif[hif_id];
}
#else
extern void *ccci_hif_get_by_id(unsigned char hif_id);
#endif

static inline void ccci_hif_queue_status_notify(int md_id, int hif_id,
	int qno, int dir, int state)
{
	return ccci_port_queue_status_notify(md_id, hif_id, qno,
		dir, state);
}


static inline void ccci_reset_seq_num(struct ccci_hif_traffic *traffic_info)
{
	/* it's redundant to use 2 arrays,
	 * but this makes sequence checking easy
	 */
	memset(traffic_info->seq_nums[OUT], 0,
		sizeof(traffic_info->seq_nums[OUT]));
	memset(traffic_info->seq_nums[IN], -1,
		sizeof(traffic_info->seq_nums[IN]));
	traffic_info->isr_cnt = 0;
}

/*
 * as one channel can only use one hardware queue,
 * so it's safe we call this function in hardware
 * queue's lock protection
 */
static inline void ccci_md_inc_tx_seq_num(unsigned char md_id,
	struct ccci_hif_traffic *traffic_info,
	struct ccci_header *ccci_h)
{
	if (ccci_h->channel >= ARRAY_SIZE(traffic_info->seq_nums[OUT])) {
		CCCI_NORMAL_LOG(md_id, CORE,
			"ignore seq inc on channel %x\n",
			*(((u32 *) ccci_h) + 2));
		return;		/* for force assert channel, etc. */
	}
	ccci_h->seq_num = traffic_info->seq_nums[OUT][ccci_h->channel]++;
	ccci_h->assert_bit = 1;

	/* for rpx channel, can only set assert_bit when
	 * md is in single-task phase.
	 */
	/* when md is in multi-task phase, assert bit should be 0,
	 * since ipc task are preemptible
	 */
	if ((ccci_h->channel == CCCI_RPC_TX
		|| ccci_h->channel == CCCI_FS_TX)
		&& ccci_fsm_get_md_state(md_id) != BOOT_WAITING_FOR_HS2)
		ccci_h->assert_bit = 0;
}

static inline void ccci_channel_update_packet_counter(
	unsigned long *logic_ch_pkt_cnt, struct ccci_header *ccci_h)
{
	if (ccci_h->channel < CCCI_MAX_CH_NUM)
		logic_ch_pkt_cnt[ccci_h->channel]++;
}

static inline void ccci_channel_dump_packet_counter(
	unsigned char md_id, struct ccci_hif_traffic *traffic_info)
{
	CCCI_REPEAT_LOG(md_id, CORE,
	"traffic(ch): tx:[%d]%ld, [%d]%ld, [%d]%ld rx:[%d]%ld, [%d]%ld, [%d]%ld\n",
	CCCI_PCM_TX, traffic_info->logic_ch_pkt_cnt[CCCI_PCM_TX],
	CCCI_UART2_TX, traffic_info->logic_ch_pkt_cnt[CCCI_UART2_TX],
	CCCI_FS_TX, traffic_info->logic_ch_pkt_cnt[CCCI_FS_TX],
	CCCI_PCM_RX, traffic_info->logic_ch_pkt_cnt[CCCI_PCM_RX],
	CCCI_UART2_RX, traffic_info->logic_ch_pkt_cnt[CCCI_UART2_RX],
	CCCI_FS_RX, traffic_info->logic_ch_pkt_cnt[CCCI_FS_RX]);
	CCCI_REPEAT_LOG(md_id, CORE,
	"traffic(net): tx: [%d]%ld %ld, [%d]%ld %ld, [%d]%ld %ld, rx:[%d]%ld, [%d]%ld, [%d]%ld\n",
	CCCI_CCMNI1_TX, traffic_info->logic_ch_pkt_pre_cnt[CCCI_CCMNI1_TX],
	traffic_info->logic_ch_pkt_cnt[CCCI_CCMNI1_TX],
	CCCI_CCMNI2_TX, traffic_info->logic_ch_pkt_pre_cnt[CCCI_CCMNI2_TX],
	traffic_info->logic_ch_pkt_cnt[CCCI_CCMNI2_TX],
	CCCI_CCMNI3_TX, traffic_info->logic_ch_pkt_pre_cnt[CCCI_CCMNI3_TX],
	traffic_info->logic_ch_pkt_cnt[CCCI_CCMNI3_TX],
	CCCI_CCMNI1_RX, traffic_info->logic_ch_pkt_cnt[CCCI_CCMNI1_RX],
	CCCI_CCMNI2_RX, traffic_info->logic_ch_pkt_cnt[CCCI_CCMNI2_RX],
	CCCI_CCMNI3_RX, traffic_info->logic_ch_pkt_cnt[CCCI_CCMNI3_RX]);
}

static inline unsigned int ccci_md_get_seq_num(
	struct ccci_hif_traffic *traffic_info, enum DIRECTION dir,
	enum CCCI_CH ch)
{
	return traffic_info->seq_nums[dir][ch];
}

extern void ccci_hif_register(unsigned char hif_id, void *hif_per_data,
	struct ccci_hif_ops *ops);

#endif
