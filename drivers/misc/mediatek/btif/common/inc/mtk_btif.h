/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MTK_BTIF_H_
#define __MTK_BTIF_H_

#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/time.h>		/* gettimeofday */
#include <asm-generic/bug.h>

#include "btif_pub.h"
#include "btif_dma_pub.h"
#include "mtk_btif_exp.h"

#define BTIF_PORT_NR 1
#define BTIF_USER_NAME_MAX_LEN 32

/*-------------Register Defination Start ---------------*/
#if defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
#define BTIF_RX_BUFFER_SIZE (1024 * 32)
#else
#define BTIF_RX_BUFFER_SIZE (1024 * 64)
#endif
#define BTIF_TX_BUFFER_FIFO_SIZE (1024 * 4)

/*------------Register Defination End ----------------*/

/*------------BTIF Module Clock and Power Control Defination---------------*/
enum _ENUM_BTIF_RX_TYPE_ {
	BTIF_IRQ_CTX = 0,
	BTIF_TASKLET_CTX = BTIF_IRQ_CTX + 1,
	BTIF_THREAD_CTX = BTIF_TASKLET_CTX + 1,
	BTIF_WQ_CTX = BTIF_THREAD_CTX + 1,
	BTIF_RX_TYPE_MAX,
};

enum _ENUM_BTIF_TX_TYPE_ {
	BTIF_TX_USER_CTX = 0,
	BTIF_TX_SINGLE_CTX = BTIF_TX_USER_CTX + 1,
	BTIF_TX_TYPE_MAX,
};

enum _ENUM_BTIF_STATE_ {
	B_S_OFF = 0,
	B_S_SUSPEND = B_S_OFF + 1,
	B_S_DPIDLE = B_S_SUSPEND + 1,
	B_S_ON = B_S_DPIDLE + 1,
	B_S_MAX,
};

#if BTIF_DBG_SUPPORT
enum _ENUM_BTIF_TEST_CASE_ {
	BTIF_TEST_RX_THREAD_BLOCK = 0,
	BTIF_TEST_RX_IRQ_BLOCK = 1,
};
#endif

#define ENABLE_BTIF_RX_DMA 1
#define ENABLE_BTIF_TX_DMA 1

#ifdef ENABLE_BTIF_TX_DMA
#define BTIF_TX_MODE BTIF_MODE_DMA
#else
#define BTIF_TX_MODE BTIF_MODE_PIO
#endif

#ifdef ENABLE_BTIF_RX_DMA
#define BTIF_RX_MODE BTIF_MODE_DMA
#else
#define BTIF_RX_MODE BTIF_MODE_PIO
#endif

#define BTIF_RX_BTM_CTX BTIF_THREAD_CTX/*BTIF_WQ_CTX*//* BTIF_TASKLET_CTX */
/*
 * -- cannot be used because ,
 * mtk_wcn_stp_parser data will call *(stp_if_tx) to send ack,
 * in which context sleepable lock or usleep operation may be used,
 * these operation is not allowed in tasklet, may cause schedule_bug
 */

#define BTIF_TX_CTX BTIF_TX_USER_CTX	/* BTIF_TX_SINGLE_CTX */

#define ENABLE_BTIF_RX_THREAD_RT_SCHED 1
#define MAX_BTIF_RXD_TIME_REC 3

/*Structure Defination*/

/*-----------------BTIF setting--------------*/
struct _mtk_btif_setting_ {
	enum _ENUM_BTIF_MODE_ tx_mode;	/*BTIF Tx Mode Setting */
	enum _ENUM_BTIF_MODE_ rx_mode;	/*BTIF Tx Mode Setting */
	enum _ENUM_BTIF_RX_TYPE_ rx_type;	/*rx handle type */
	enum _ENUM_BTIF_TX_TYPE_ tx_type;	/*tx type */
};
/*---------------------------------------------------------------------------*/

struct _btif_buf_str_ {
	unsigned int size;
	unsigned char *p_buf;
	/*
	 * For Tx: next Tx data pointer to FIFO;
	 * For Rx: next read data pointer from BTIF user
	 */
	unsigned int rd_idx;
	/*
	 * For Tx: next Tx data pointer from BTIF user;
	 * For Rx: next write data(from FIFO) pointer
	 */
	unsigned int wr_idx;
};

/*---------------------------------------------------------------------------*/
struct _mtk_btif_dma_ {
					/*struct _mtk_btif_ **/ void *p_btif;
					/*BTIF pointer to which DMA belongs */

	enum _ENUM_BTIF_DIR_ dir;	/*DMA's direction: */
	bool enable;		/*DMA enable or disable flag */

	struct _MTK_DMA_INFO_STR_ *p_dma_info;	/*DMA's IRQ information */

	spinlock_t iolock;	/*io lock for DMA channel */
	atomic_t entry;		/* entry count */
};

#if defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
#define BTIF_LOG_ENTRY_NUM 10
#else
#define BTIF_LOG_ENTRY_NUM 30
#endif

#define BTIF_LOG_SZ  16

typedef void (*MTK_BTIF_RX_NOTIFY) (void);

struct _btif_log_buf_t_ {
	unsigned int len;
	struct timespec64 timer;
	struct timespec64 ts;
	unsigned char buffer[BTIF_LOG_SZ];
};

struct _btif_log_queue_t_ {
	enum _ENUM_BTIF_DIR_ dir;
	bool enable;
	bool output_flag;
	unsigned int in;
	unsigned int out;
	unsigned int size;
	spinlock_t lock;
	struct _btif_log_buf_t_ *p_queue;
	struct _btif_log_buf_t_ *p_dump_queue;
	struct work_struct dump_work;
	unsigned int dump_size;
};

/*---------------------------------------------------------------------------*/
struct _mtk_btif_ {
	unsigned int open_counter;	/*open counter */
	bool enable;		/*BTIF module enable flag */
	bool lpbk_flag;		/*BTIF module enable flag */

	enum _ENUM_BTIF_STATE_ state;	/*BTIF state mechanism */
	struct mutex state_mtx;	/*lock to BTIF state mechanism's state change */
	struct mutex ops_mtx;	/*lock to BTIF's open and close */

	enum _ENUM_BTIF_MODE_ tx_mode;	/* BTIF Tx channel mode */
	enum _ENUM_BTIF_MODE_ rx_mode;	/* BTIF Rx channel mode */
	struct mutex tx_mtx;	/*lock to BTIF's tx process */
/*rx handling */
	enum _ENUM_BTIF_RX_TYPE_ btm_type;	/*BTIF Rx bottom half context */
/*tx handling*/
	enum _ENUM_BTIF_TX_TYPE_ tx_ctx;	/*BTIF tx context */
/* unsigned char rx_buf[BTIF_RX_BUFFER_SIZE]; */
	struct _btif_buf_str_ btif_buf;
	spinlock_t rx_irq_spinlock;	/*lock for rx irq handling */

/*rx workqueue information*/
	/*lock to BTIF's rx bottom half when kernel thread is used */
	struct mutex rx_mtx;
	struct workqueue_struct *p_rx_wq;
	struct work_struct rx_work;

	struct workqueue_struct *p_tx_wq;
	struct work_struct tx_work;
	struct kfifo *p_tx_fifo;

/*rx tasklet information*/
	struct tasklet_struct rx_tasklet;
	/*lock to BTIF's rx bottom half when tasklet is used */
	spinlock_t rx_tasklet_spinlock;

/*rx thread information*/
	struct mutex rx_thread_mtx;
	struct task_struct *p_task;
	struct completion rx_comp;

	struct _mtk_btif_setting_ *setting;	/*BTIF setting */

	struct _mtk_btif_dma_ *p_tx_dma;	/*BTIF Tx channel DMA */
	struct _mtk_btif_dma_ *p_rx_dma;	/*BTIF Rx channel DMA */
	void __iomem *dma_clk_addr;		/*APDMA clock address */
	void __iomem *dma_idle_en_addr;		/*APDMA idle en address */

	MTK_WCN_BTIF_RX_CB rx_cb;	/*Rx callback function */
	MTK_BTIF_RX_NOTIFY rx_notify;

	struct _MTK_BTIF_INFO_STR_ *p_btif_info;	/*BTIF's information */

/*Log Tx data to buffer*/
	struct _btif_log_queue_t_ tx_log;

/*Log Rx data to buffer*/
	struct _btif_log_queue_t_ rx_log;

/* struct list_head *p_user_list; */
	struct list_head user_list;
/* get btif dev pointer*/
	void *private_data;
#if BTIF_DBG_SUPPORT
/* Test btif thread */
	struct delayed_work btif_rx_test_work;
	enum _ENUM_BTIF_TEST_CASE_ test_case;
	int delay_sched_time;
#endif
};

/*---------------------------------------------------------------------------*/
struct _mtk_btif_user_ {
	bool enable;		/*register its state */
	struct list_head entry;	/*btif_user's bi-direction list table */
	/*BTIF's user, static allocation */
	char u_name[BTIF_USER_NAME_MAX_LEN];
	unsigned long u_id;
	struct _mtk_btif_ *p_btif;
};

/*---------------------------------------------------------------------------*/
#define BBS_PTR(ptr, idx) ((ptr->p_buf) + idx)

#define BBS_SIZE(ptr) ((ptr)->size)
#define BBS_MASK(ptr) (BBS_SIZE(ptr) - 1)
#define BBS_COUNT(ptr) ((ptr)->wr_idx >= (ptr)->rd_idx ? (ptr)->wr_idx - \
(ptr)->rd_idx : BBS_SIZE(ptr) - \
((ptr)->rd_idx - (ptr)->wr_idx))
#define BBS_COUNT_CUR(ptr, wr_idx) (wr_idx >= (ptr)->rd_idx ? wr_idx - \
(ptr)->rd_idx : BBS_SIZE(ptr) - \
((ptr)->rd_idx - wr_idx))

#define BBS_LEFT(ptr) (BBS_SIZE(ptr) - BBS_COUNT(ptr))

#define BBS_AVL_SIZE(ptr) (BBS_SIZE(ptr) - BBS_COUNT(ptr))
#define BBS_FULL(ptr) (BBS_COUNT(ptr) - BBS_SIZE(ptr))
#define BBS_EMPTY(ptr) ((ptr)->wr_idx == (ptr)->rd_idx)
#define BBS_WRITE_MOVE_NEXT(ptr) ((ptr)->wr_idx = \
((ptr)->wr_idx + 1) & BBS_MASK(ptr))
#define BBS_READ_MOVE_NEXT(ptr) ((ptr)->rd_idx = \
((ptr)->rd_idx + 1) & BBS_MASK(ptr))

#define BBS_INIT(ptr) \
{ \
(ptr)->rd_idx = (ptr)->wr_idx = 0; \
(ptr)->size = BTIF_RX_BUFFER_SIZE; \
}


#define BTIF_MUTEX_UNLOCK(x) mutex_unlock(x)

extern struct _mtk_btif_ g_btif[];

int btif_open(struct _mtk_btif_ *p_btif);
int btif_close(struct _mtk_btif_ *p_btif);
int btif_send_data(struct _mtk_btif_ *p_btif,
		   const unsigned char *p_buf, unsigned int buf_len);
int btif_enter_dpidle(struct _mtk_btif_ *p_btif);
int btif_exit_dpidle(struct _mtk_btif_ *p_btif);
int btif_rx_cb_reg(struct _mtk_btif_ *p_btif, MTK_WCN_BTIF_RX_CB rx_cb);

/*for test purpose*/
int _btif_suspend(struct _mtk_btif_ *p_btif);
int _btif_resume(struct _mtk_btif_ *p_btif);
int _btif_restore_noirq(struct _mtk_btif_ *p_btif);

int btif_lpbk_ctrl(struct _mtk_btif_ *p_btif, bool flag);
int btif_log_buf_dmp_in(struct _btif_log_queue_t_ *p_log_que, const char *p_buf,
			int len);
int btif_dump_data(const char *p_buf, int len);
int btif_log_buf_dmp_out(struct _btif_log_queue_t_ *p_log_que);
int btif_log_buf_enable(struct _btif_log_queue_t_ *p_log_que);
int btif_log_buf_disable(struct _btif_log_queue_t_ *p_log_que);
int btif_log_output_enable(struct _btif_log_queue_t_ *p_log_que);
int btif_log_output_disable(struct _btif_log_queue_t_ *p_log_que);
int btif_log_buf_reset(struct _btif_log_queue_t_ *p_log_que);
int btif_log_buf_init(struct _mtk_btif_ *p_btif);
int btif_dump_reg(struct _mtk_btif_ *p_btif, enum _ENUM_BTIF_REG_ID_ flag);
int btif_rx_notify_reg(struct _mtk_btif_ *p_btif, MTK_BTIF_RX_NOTIFY rx_notify);
int btif_raise_wak_signal(struct _mtk_btif_ *p_btif);
int btif_clock_ctrl(struct _mtk_btif_ *p_btif, int en);
bool btif_parser_wmt_evt(struct _mtk_btif_ *p_btif,
				const char *sub_str,
				unsigned int sub_len);
void mtk_btif_read_cpu_sw_rst_debug(void);
int btif_rx_data_path_lock(struct _mtk_btif_ *p_btif);
int btif_rx_data_path_unlock(struct _mtk_btif_ *p_btif);
int btif_rx_buf_has_pending_data(struct _mtk_btif_ *p_btif);
int btif_rx_dma_has_pending_data(struct _mtk_btif_ *p_btif);
int btif_tx_dma_has_pending_data(struct _mtk_btif_ *p_btif);
void btif_dump_dma_vfifo(struct _mtk_btif_ *p_btif);
bool btif_is_tx_complete(struct _mtk_btif_ *p_btif);
bool btif_is_rx_complete(struct _mtk_btif_ *p_btif);
struct task_struct *btif_rx_thread_get(struct _mtk_btif_ *p_btif);
void btif_do_gettimeofday(struct timespec64 *tv);
int btif_dump_array(const char *string, const char *p_buf, int len);
#endif /*__MTK_BTIF_H_*/
