/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#ifndef __MODEM_DPMA_H__
#define __MODEM_DPMA_H__

#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/dmapool.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/skbuff.h>
#include "mt-plat/mtk_ccci_common.h"
#include "ccci_bm.h"
#include "ccci_hif_internal.h"
#include "dpmaif_debug.h"

/*
 * hardcode, max queue number should be synced with port array in port_cfg.c
 */
#ifndef _E1_SB_SW_WORKAROUND_
#define BAT_CNT_BURST_UPDATE  /* update with pit cnt update */
#endif
#define DPMAIF_RXQ_NUM  1
#define DPMAIF_TXQ_NUM  4

/*Default DPMAIF DL common setting*/
#define DPMAIF_HW_BAT_REMAIN       64
#ifndef DPMAIF_PKT_SIZE
#define DPMAIF_PKT_SIZE      (128*28) /* 3584 ==SKB_4K */
#define DPMAIF_FRG_SIZE      (128) /* ==, no used */
#endif
#define DPMAIF_HW_BAT_PKTBUF      DPMAIF_PKT_SIZE
#define DPMAIF_HW_FRG_PKTBUF      DPMAIF_FRG_SIZE

#define DPMAIF_HW_BAT_RSVLEN      0 /* 88 */
#define DPMAIF_HW_PKT_BIDCNT      1  /* 3-->1 should be 1 in E1 */
#define DPMAIF_HW_PKT_ALIGN       64
#define DPMAIF_HW_MTU_SIZE        (3*1024 + 8)

#define DPMAIF_BUF_PKT_SIZE     DPMAIF_PKT_SIZE
#define DPMAIF_BUF_FRAG_SIZE    DPMAIF_FRG_SIZE

#ifdef MT6297
#define DPMAIF_HW_CHK_BAT_NUM      62
#define DPMAIF_HW_CHK_FRG_NUM      DPMAIF_HW_CHK_BAT_NUM
#define DPMAIF_HW_CHK_PIT_NUM      (DPMAIF_HW_CHK_BAT_NUM*2)
#define DPMAIF_HW_CHK_RB_PIT_NUM   64

#define DPMAIF_DL_BAT_ENTRY_SIZE  8192 /* <- 1024 <- 128 */

#else
#define DPMAIF_HW_CHK_PIT_NUM      6
#define DPMAIF_HW_CHK_BAT_NUM      3
#define DPMAIF_HW_CHK_FRG_NUM      3

#define DPMAIF_DL_BAT_ENTRY_SIZE  1024 /* 128 */
#endif

/* 2048*/ /* 256, 100pkts*2*10ms=2000*12B=>24k */
#define DPMAIF_DL_PIT_ENTRY_SIZE  (DPMAIF_DL_BAT_ENTRY_SIZE * 2)
#define DPMAIF_UL_DRB_ENTRY_SIZE  2048 /* from 512 */

#ifdef MT6297
#define DPMAIF_DL_PIT_BYTE_SIZE   16
#else
#define DPMAIF_DL_PIT_BYTE_SIZE   12
#endif
#define DPMAIF_DL_BAT_BYTE_SIZE   8
#define DPMAIF_UL_DRB_BYTE_SIZE  8

#define DPMAIF_DL_PIT_SIZE (DPMAIF_DL_PIT_ENTRY_SIZE*DPMAIF_DL_PIT_BYTE_SIZE)
#define DPMAIF_DL_BAT_SIZE (DPMAIF_DL_BAT_ENTRY_SIZE*DPMAIF_DL_BAT_BYTE_SIZE)
#define DPMAIF_UL_DRB_SIZE (DPMAIF_UL_DRB_ENTRY_SIZE*DPMAIF_UL_DRB_BYTE_SIZE)

#ifdef _HW_REORDER_SW_WORKAROUND_
#define DPMAIF_DUMMY_PIT_MAX_NUM 0x3fffff
#define DPMAIF_DUMMY_PIT_AIDX    1024
#endif

struct ringbuf_str {
unsigned int rd_idx;
unsigned int wrt_idx;
unsigned int buf_len;
};

/****************************************************************************
 * Structure of DL PIT
 ****************************************************************************/
#if (MD_GENERATION < 6297)
struct dpmaifq_normal_pit {
	unsigned int    packet_type:1; /* 0-payload packet; 1-message packet */
	unsigned int    c_bit:1;/* 1-1/n; 0-the last one */
	unsigned int    buffer_type:1; /* 0-pkt bat buffer; 1-frag bat buffer */
	unsigned int    buffer_id:13; /* BAT index */
	unsigned int    data_len:16;
	unsigned int    p_data_addr;
	unsigned int    data_addr_ext:8;
	unsigned int    reserved:24;
};
#else
struct dpmaifq_normal_pit {
	unsigned int	packet_type:1;
	unsigned int    c_bit:1;
	unsigned int    buffer_type:1;
	unsigned int    buffer_id:13;
	unsigned int    data_len:16;
	unsigned int	p_data_addr;
	unsigned int	data_addr_ext;
	unsigned int	pit_seq:16;
	unsigned int	ig:1;
	unsigned int	reserved2:7;
	unsigned int	ulq_done:6;
	unsigned int	dlq_done:2;
};
#endif
/* packet_type */
#define DES_PT_PD            0x00
#define DES_PT_MSG           0x01
/* c_bit */
#define PKT_LAST_ONE    0x0
/* buffer_type */
#define PKT_BUF_FRAG    0x1

#if (MD_GENERATION < 6297)
struct dpmaifq_msg_pit {
	unsigned int    packet_type:1;
	unsigned int    c_bit:1;
	unsigned int    check_sum:2;
	unsigned int    error_bit:1;
	unsigned int	  reserved:11;
	unsigned int    channel_id:8;
	unsigned int	  network_type:3;
	unsigned int    reserved2:5;
	unsigned int    count_l:16;
	unsigned int    flow:4;
	unsigned int    cmd:3;
	unsigned int    reserved3:9;
	unsigned int    reserved4;
};
#else
struct dpmaifq_msg_pit {
	unsigned int    packet_type:1;
	unsigned int    c_bit:1;
	unsigned int    check_sum:2;
	unsigned int    error_bit:1;
	unsigned int    src_qid:3;
	unsigned int	reserved:8;
	unsigned int    channel_id:8;
	unsigned int	network_type:3;
	unsigned int    reserved2:4;
	unsigned int    dp:1;

	unsigned int    count_l:16;
	unsigned int    flow:5;
	unsigned int    reserved3:3;
	unsigned int    cmd:3;
	unsigned int    reserved4:5;

	unsigned int    reserved5:3;
	unsigned int    vbid:13;
	unsigned int    reserved6:16;

	unsigned int    pit_seq:16;
	unsigned int    ig:1;
	unsigned int    reserved7:7;
	unsigned int    ulq_done:6;
	unsigned int    dlq_done:2;
};
#endif
/****************************************************************************
 * Structure of DL BAT
 ****************************************************************************/
struct dpmaif_bat_t {
	unsigned int p_buffer_addr;
	unsigned int buffer_addr_ext:8;
	unsigned int reserved:24;
};

struct dpmaif_bat_skb_t {
	struct sk_buff *skb;
	dma_addr_t data_phy_addr;
	unsigned int data_len;
};

struct dpmaif_bat_page_t {
	struct page *page;
	dma_addr_t data_phy_addr;
	unsigned long offset;
	unsigned int data_len;
};

#define MAX_BD_NUM (MAX_SKB_FRAGS + 1)
#define DPMAIF_TRAFFIC_MONITOR_INTERVAL 10
#define SKB_RX_LIST_MAX_LEN 0xFFFFFFFF

struct dpmaif_bat_request {
	void *bat_base;
	dma_addr_t bat_phy_addr; /* physical address for DMA */
	unsigned int    bat_size_cnt;
	unsigned short    bat_wr_idx;
	unsigned short    bat_rd_idx;
	void *bat_skb_ptr;/* collect skb linked to bat */
	unsigned int     skb_pkt_cnt;
	unsigned int pkt_buf_sz;

	/* for debug */
	int check_bid_fail_cnt;
};

enum error_num {
	LOW_MEMORY_ERR = -15,
	LOW_MEMORY_PIT,
	LOW_MEMORY_BAT,
	LOW_MEMORY_SKB,
	LOW_MEMORY_DRB,
	LOW_MEMORY_TYPE_MAX, /* -10 */

	DMA_MAPPING_ERR,
	FLOW_CHECK_ERR,
	DATA_CHECK_FAIL,
	HW_REG_CHK_FAIL,
	HW_REG_TIME_OUT,
	ERROR_STOP_MAX, /* -4 */
};

struct dpmaif_rx_queue {
	unsigned char index;
	bool que_started;
	unsigned short budget;

	void *pit_base;
	dma_addr_t pit_phy_addr; /* physical address for DMA */
	unsigned int    pit_size_cnt;

	unsigned short    pit_rd_idx;
	unsigned short    pit_wr_idx;
	unsigned short    pit_rel_rd_idx;
	unsigned int reg_int_mask_bak;
#ifdef _HW_REORDER_SW_WORKAROUND_
	unsigned long     pit_dummy_cnt;
	unsigned long	  pit_dummy_idx;
	unsigned char     pit_reload_en;
#endif

	struct tasklet_struct dpmaif_rxq0_task;
	wait_queue_head_t rx_wq;
	struct task_struct *rx_thread;
	struct workqueue_struct *worker;
	struct work_struct dpmaif_rxq0_work;
	spinlock_t rx_lock;
	atomic_t rx_processing;

	unsigned int  cur_chn_idx:8;
	unsigned int check_sum:2;
	int skb_idx;

	struct ccci_skb_queue skb_list;
	unsigned int pit_dp;
};

/****************************************************************************
 * Structure of UL DRB
 ****************************************************************************
 */
/*
 * UL unit: WORD == 4bytes, && 1drb == 8bytes,
 * so 1 drb = size_cnt * 2, rd_idx_sw = rd_idx_hw/2.
 */
 #define DPMAIF_UL_DRB_ENTRY_WORD  2 /* (sizeof(dpmaif_drb_pd)/4)*/

struct dpmaif_drb_pd {
	unsigned int    dtyp:2;
	unsigned int    c_bit:1;
	unsigned int    reserved:5;
	unsigned int    data_addr_ext:8;
	unsigned int    data_len:16;

	unsigned int    p_data_addr;
};
/* drb->dtype */
#define DES_DTYP_PD           0x00
#define DES_DTYP_MSG          0x01
#define DES_DRB_CBIT          0x01

struct dpmaif_drb_msg {
	unsigned int    dtyp:2;
	unsigned int    c_bit:1;
	unsigned int    reserved:13;
	unsigned int    packet_len:16;
	unsigned int    count_l:16;
	unsigned int    channel_id:8;
	unsigned int    network_type:3;
	unsigned int    r:1;
	unsigned int    ipv4:1; /* enable ul checksum offload for ipv4 header */
	unsigned int    l4:1; /* enable ul checksum offload for tcp/udp */
	unsigned int    rsv:2;
};

struct dpmaif_drb_skb {
	struct sk_buff *skb;
	dma_addr_t phy_addr; /* physical address for DMA */
	unsigned short data_len;

	/* just for debug */
	unsigned short drb_idx:13;
	unsigned short is_msg:1;
	unsigned short is_frag:1;
	unsigned short is_last_one:1;
};

struct dpmaif_tx_queue {
	unsigned char index;
	bool que_started;
	atomic_t tx_budget;

	void    *drb_base;
	dma_addr_t drb_phy_addr;	/* physical address for DMA */
	unsigned int    drb_size_cnt;

	unsigned short    drb_wr_idx;
	unsigned short    drb_rd_idx;
	unsigned short    drb_rel_rd_idx;
	unsigned short    last_ch_id;

	void    *drb_skb_base;
	wait_queue_head_t req_wq;

	/* For Tx done Kernel thread */
	struct hrtimer tx_done_timer;
	atomic_t txq_done;
	wait_queue_head_t tx_done_wait;
	struct task_struct *tx_done_thread;

	spinlock_t tx_lock;
	atomic_t tx_processing;
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
	unsigned int busy_count;
#endif
	atomic_t tx_resume_tx;
	atomic_t tx_resume_done;
};

enum hifdpmaif_state {
	HIFDPMAIF_STATE_MIN  = 0,
	HIFDPMAIF_STATE_PWROFF,
	HIFDPMAIF_STATE_PWRON,
	HIFDPMAIF_STATE_EXCEPTION,
	HIFDPMAIF_STATE_MAX,
};


struct  ccci_hif_dpmaif_val {
	struct regmap *infra_ao_base;
	unsigned int md_gen;
	unsigned long offset_epof_md1;
	void __iomem *md_plat_info;
};

struct hif_dpmaif_ctrl {
	enum hifdpmaif_state dpmaif_state;
	struct dpmaif_tx_queue txq[DPMAIF_TXQ_NUM];
	struct dpmaif_rx_queue rxq[DPMAIF_RXQ_NUM];
	struct dma_pool *tx_drb_dmapool;
	struct dma_pool *rx_pit_dmapool;
	struct dma_pool *rx_bat_dmapool;

	unsigned char md_id;
	unsigned char hif_id;
	struct ccci_hif_traffic traffic_info;
	atomic_t wakeup_src;

	void __iomem *dpmaif_ao_ul_base;
	void __iomem *dpmaif_ao_dl_base;
	void __iomem *dpmaif_pd_ul_base;
	void __iomem *dpmaif_pd_dl_base;
	void __iomem *dpmaif_pd_misc_base;
	void __iomem *dpmaif_pd_md_misc_base;
	void __iomem *dpmaif_pd_sram_base;
	void __iomem *dpmaif_pd_rdma_base;
	void __iomem *dpmaif_pd_wdma_base;
	void __iomem *dpmaif_ao_md_dl_base;

	unsigned int dpmaif_irq_id;
	unsigned long dpmaif_irq_flags;
	atomic_t dpmaif_irq_enabled;

	struct ccci_hif_ops *ops;
#if DPMAIF_TRAFFIC_MONITOR_INTERVAL
	unsigned int tx_traffic_monitor[DPMAIF_TXQ_NUM];
	unsigned int rx_traffic_monitor[DPMAIF_RXQ_NUM];
	unsigned int tx_pre_traffic_monitor[DPMAIF_TXQ_NUM];
	unsigned long long tx_done_last_start_time[DPMAIF_TXQ_NUM];
	unsigned int tx_done_last_count[DPMAIF_TXQ_NUM];

	struct timer_list traffic_monitor;
	char traffic_started;
#endif
	struct platform_device *plat_dev; /* maybe: no need. */
	struct ccci_hif_dpmaif_val plat_val;

	atomic_t suspend_flag;
	struct dpmaif_bat_request *bat_req;
	struct dpmaif_bat_request *bat_frag;
	wait_queue_head_t   bat_alloc_wq;
	struct task_struct *bat_alloc_thread;
	atomic_t bat_need_alloc;
	atomic_t bat_paused_alloc;
	int bat_alloc_running;
};

#ifndef CCCI_KMODULE_ENABLE
static inline int ccci_dpma_hif_send_skb(unsigned char hif_id, int tx_qno,
	struct sk_buff *skb, int from_pool, int blocking)
{
	struct hif_dpmaif_ctrl *hif_ctrl =
		(struct hif_dpmaif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (hif_ctrl)
		return hif_ctrl->ops->send_skb(hif_id, tx_qno, skb, from_pool,
					blocking);
	else
		return -1;
}

static inline int ccci_dpma_hif_write_room(unsigned char hif_id,
	unsigned char qno)
{
	struct hif_dpmaif_ctrl *hif_ctrl =
		(struct hif_dpmaif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (hif_ctrl)
		return hif_ctrl->ops->write_room(hif_id, qno);
	else
		return -1;

}
static inline int ccci_dpma_hif_give_more(unsigned char hif_id, int rx_qno)
{
	struct hif_dpmaif_ctrl *hif_ctrl =
		(struct hif_dpmaif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (hif_ctrl)
		return hif_ctrl->ops->give_more(hif_id, rx_qno);
	else
		return -1;

}

static inline int ccci_dpmaif_hif_dump_status(unsigned char hif_id,
	enum MODEM_DUMP_FLAG dump_flag, void *buff, int length)
{
	struct hif_dpmaif_ctrl *hif_ctrl =
		(struct hif_dpmaif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (hif_ctrl)
		return hif_ctrl->ops->dump_status(hif_id, dump_flag,
			buff, length);
	else
		return -1;

}

static inline int ccci_dpmaif_hif_set_wakeup_src(unsigned char hif_id,
	int value)
{
	struct hif_dpmaif_ctrl *hif_ctrl =
		(struct hif_dpmaif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (hif_ctrl) {
		arch_atomic_set(&hif_ctrl->wakeup_src, value);
		return value;
	} else
		return -1;

}

#else

#define ccci_write32(b, a, v)  \
do { \
	writel(v, (b) + (a)); \
	mb(); /* make sure register access in order */ \
} while (0)


#define ccci_write16(b, a, v)  \
do { \
	writew(v, (b) + (a)); \
	mb(); /* make sure register access in order */ \
} while (0)


#define ccci_write8(b, a, v)  \
do { \
	writeb(v, (b) + (a)); \
	mb(); /* make sure register access in order */ \
} while (0)


#define ccci_read32(b, a)               ioread32((void __iomem *)((b)+(a)))
#define ccci_read16(b, a)               ioread16((void __iomem *)((b)+(a)))
#define ccci_read8(b, a)                ioread8((void __iomem *)((b)+(a)))
#endif

int ccci_dpmaif_hif_init(struct device *dev);
int dpmaif_late_init(unsigned char hif_id);
int dpmaif_start(unsigned char hif_id);
int dpmaif_stop_rx(unsigned char hif_id);
int dpmaif_stop_tx(unsigned char hif_id);
int dpmaif_stop(unsigned char hif_id);
void dpmaif_stop_hw(void);
extern void ccmni_clr_flush_timer(void);
extern struct regmap *syscon_regmap_lookup_by_phandle(struct device_node *np,
	const char *property);
extern int regmap_write(struct regmap *map, unsigned int reg, unsigned int val);
extern int regmap_read(struct regmap *map, unsigned int reg, unsigned int *val);
#ifdef CONFIG_MTK_GIC_V3_EXT
extern void mt_irq_dump_status(int irq);
#endif

#endif				/* __MODEM_DPMA_H__ */
