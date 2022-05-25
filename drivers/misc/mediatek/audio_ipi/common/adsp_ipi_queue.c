// SPDX-License-Identifier: GPL-2.0
//
// adsp_ipi_queue.c--  Mediatek ADSP IPI with queue
//
// Copyright (c) 2018 MediaTek Inc.


#include <adsp_ipi_queue.h>

#include <linux/types.h>
#include <linux/errno.h>

#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/spinlock.h>

#include <linux/delay.h>
#include <linux/jiffies.h>

#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include <audio_log.h>
#include <audio_assert.h>
#include <audio_ipi_platform.h>

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
#include <adsp_helper.h>
#endif

#if IS_ENABLED(CONFIG_MTK_AUDIO_CM4_SUPPORT)
#include <scp_ipi.h>
#endif



#include <audio_messenger_ipi.h>



/*
 * =============================================================================
 *                     log
 * =============================================================================
 */

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[IPI][DSP_Q] %s(), " fmt "\n", __func__


/*
 * =============================================================================
 *                     MACRO
 * =============================================================================
 */

#define MAX_DSP_MSG_NUM_IN_QUEUE (64)
#define DSP_MSG_BUFFER_SIZE ((SHARE_BUF_SIZE) - 16)


/*
 * =============================================================================
 *                     struct def
 * =============================================================================
 */

struct dsp_msg_t {
	uint32_t ipi_id;
	uint8_t buf[DSP_MSG_BUFFER_SIZE]; /* TODO: use ring buf? */
	uint32_t len;

	/* recv msg callback */
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len);
};


struct dsp_queue_element_t {
	struct dsp_msg_t msg;

	spinlock_t element_lock;
	wait_queue_head_t element_wq;

	bool wait_in_thread;
	bool signal_arrival;
	int send_retval;
	uint32_t queue_counter;
};


enum { /* dsp_path_t */
	DSP_PATH_A2D = 0, /* AP to DSP */
	DSP_PATH_D2A = 1, /* DSP to AP */
	DSP_NUM_PATH
};


struct dsp_msg_queue_t {
	uint32_t dsp_id;   /* enum dsp_id_t */
	uint32_t dsp_path; /* enum dsp_path_t */

	struct dsp_queue_element_t element[MAX_DSP_MSG_NUM_IN_QUEUE];

	uint32_t size;
	uint32_t idx_r;
	uint32_t idx_w;

	uint32_t queue_counter;

	spinlock_t queue_lock;
	wait_queue_head_t queue_wq;

	/* dsp_send_msg_to_dsp() / dsp_process_msg_from_dsp() */
	int (*dsp_process_msg_func)(
		struct dsp_msg_queue_t *msg_queue,
		struct dsp_msg_t *p_dsp_msg);

	struct task_struct *dsp_thread_task;
	bool thread_enable;

	bool init;
	bool enable;
};



/*
 * =============================================================================
 *                     private global members
 * =============================================================================
 */

/* TODO: CM4 */
static struct dsp_msg_queue_t g_dsp_msg_queue[NUM_OPENDSP_TYPE][DSP_NUM_PATH];


/*
 * =============================================================================
 *                     utilities
 * =============================================================================
 */

inline bool dsp_check_idx_msg_valid(
	const struct dsp_msg_queue_t *msg_queue,
	const uint32_t idx_msg);

inline bool dsp_check_queue_empty(
	const struct dsp_msg_queue_t *msg_queue);

inline bool dsp_check_queue_to_be_full(
	const struct dsp_msg_queue_t *msg_queue);

inline uint32_t dsp_get_num_messages_in_queue(
	const struct dsp_msg_queue_t *msg_queue);


/*
 * =============================================================================
 *                     private function declaration
 * =============================================================================
 */

static int dsp_push_msg(
	struct dsp_msg_queue_t *msg_queue,
	uint32_t ipi_id,
	void *buf,
	uint32_t len,
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len),
	bool wait_in_thread,
	uint32_t *p_idx_msg,
	uint32_t *p_queue_counter);

static int dsp_pop_msg(struct dsp_msg_queue_t *msg_queue);

static int dsp_front_msg(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t **pp_dsp_msg,
	uint32_t *p_idx_msg);


static int dsp_get_queue_element(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t **pp_dsp_msg,
	uint32_t *p_idx_msg);


static int dsp_init_single_msg_queue(
	struct dsp_msg_queue_t *msg_queue,
	const uint32_t dsp_id,
	const uint32_t dsp_path);

static int dsp_process_msg_thread(void *data);


static int dsp_send_msg_to_dsp(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t *p_dsp_msg);

static int dsp_process_msg_from_dsp(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t *p_dsp_msg);



/*
 * =============================================================================
 *                     utilities
 * =============================================================================
 */

inline bool dsp_check_idx_msg_valid(
	const struct dsp_msg_queue_t *msg_queue,
	const uint32_t idx_msg)
{
	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return false;
	}

	return (idx_msg < msg_queue->size) ? true : false;
}


inline bool dsp_check_queue_empty(const struct dsp_msg_queue_t *msg_queue)
{
	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return false;
	}

	return (msg_queue->idx_r == msg_queue->idx_w);
}


inline bool dsp_check_queue_to_be_full(const struct dsp_msg_queue_t *msg_queue)
{
	uint32_t idx_w_to_be = 0;

	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return false;
	}

	idx_w_to_be = msg_queue->idx_w + 1;
	if (idx_w_to_be == msg_queue->size)
		idx_w_to_be = 0;

	return (idx_w_to_be == msg_queue->idx_r) ? true : false;
}


inline uint32_t dsp_get_num_messages_in_queue(
	const struct dsp_msg_queue_t *msg_queue)
{
	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return 0;
	}

	return (msg_queue->idx_w >= msg_queue->idx_r) ?
	       (msg_queue->idx_w - msg_queue->idx_r) :
	       ((msg_queue->size - msg_queue->idx_r) + msg_queue->idx_w);
}


#define DUMP_IPC_MSG(description, p_dsp_msg) \
	do { \
		struct ipi_msg_t *p_ipi_msg = NULL; \
		if (description == NULL || (p_dsp_msg) == NULL) \
			break; \
		if (p_dsp_msg->len >= IPI_MSG_HEADER_SIZE) { \
			p_ipi_msg = (struct ipi_msg_t *)p_dsp_msg->buf; \
			if (p_ipi_msg->magic == IPI_MSG_MAGIC_NUMBER) \
				DUMP_IPI_MSG(description, p_ipi_msg); \
		} else { \
			pr_info("%s, ipi_id: %u, buf %p, len: %u", \
				description, \
				p_dsp_msg->ipi_id, \
				p_dsp_msg->buf, \
				p_dsp_msg->len); \
		} \
	} while (0)



/*
 * =============================================================================
 *                     create/destroy/init/deinit functions
 * =============================================================================
 */


int ipi_queue_init_by_dsp(uint32_t dsp_id)
{
	struct dsp_msg_queue_t *msg_queue = NULL;

	uint32_t dsp_path = 0;

	int ret = 0;

	if (dsp_id >= NUM_OPENDSP_TYPE) {
		pr_info("dsp_id: %u error!!", dsp_id);
		return -EFAULT;
	}


	for (dsp_path = 0; dsp_path < DSP_NUM_PATH; dsp_path++) {
		msg_queue = &g_dsp_msg_queue[dsp_id][dsp_path];
		ret = dsp_init_single_msg_queue(msg_queue, dsp_id, dsp_path);
		if (ret != 0)
			WARN_ON(1);
	}

	return ret;
}

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
int dsp_send_msg_to_queue_wrap(
	uint32_t core_id, /* enum adsp_core_id */
	uint32_t ipi_id,  /* enum adsp_ipi_id */
	void *buf,
	uint32_t len,
	uint32_t wait_ms)
{
	uint32_t dsp_id = adsp_cid_to_ipi_dsp_id(core_id);

	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -1;

	return dsp_send_msg_to_queue(dsp_id, ipi_id, buf, len, wait_ms);
}

int dsp_dispatch_ipi_hanlder_to_queue_wrap(
	uint32_t core_id, /* enum adsp_core_id */
	uint32_t ipi_id,
	void *buf,
	uint32_t len,
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len))
{
	uint32_t dsp_id = adsp_cid_to_ipi_dsp_id(core_id);

	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -1;

	return dsp_dispatch_ipi_hanlder_to_queue(dsp_id, ipi_id, buf, len, ipi_handler);
}
#endif

void ipi_queue_init(void)
{
	uint32_t dsp_id = 0;

	for (dsp_id = 0; dsp_id < NUM_OPENDSP_TYPE; dsp_id++) {
		if (is_audio_dsp_support(dsp_id))
			ipi_queue_init_by_dsp(dsp_id);
	}

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	hook_ipi_queue_send_msg_handler(dsp_send_msg_to_queue_wrap);
	hook_ipi_queue_recv_msg_hanlder(dsp_dispatch_ipi_hanlder_to_queue_wrap);
#endif
}

int dsp_flush_msg_queue(uint32_t dsp_id)
{
	struct dsp_msg_queue_t *msg_queue = NULL;
	unsigned long flags = 0;

	if (dsp_id >= NUM_OPENDSP_TYPE) {
		pr_info("dsp_id: %u error!!", dsp_id);
		return -EOVERFLOW;
	}

	msg_queue = &g_dsp_msg_queue[dsp_id][DSP_PATH_A2D];

	spin_lock_irqsave(&msg_queue->queue_lock, flags);

	while (dsp_check_queue_empty(msg_queue) == false) {
		dsb(SY);
		wake_up_interruptible(
			&msg_queue->element[msg_queue->idx_r].element_wq);
		dsp_pop_msg(msg_queue);
	}

	spin_unlock_irqrestore(&msg_queue->queue_lock, flags);

	return 0;
}


/*
 * =============================================================================
 *                     public functions
 * =============================================================================
 */

int dsp_send_msg_to_queue(
	uint32_t dsp_id, /* enum dsp_id_t */
	uint32_t ipi_id,
	void *buf,
	uint32_t len,
	uint32_t wait_ms)
{
	struct dsp_msg_queue_t *msg_queue = NULL;
	struct dsp_queue_element_t *p_element = NULL;

	uint32_t idx_msg = 0;
	uint32_t queue_counter = 0;

	int retval = 0;
	unsigned long flags = 0;

	uint32_t try_cnt = 0;
	const uint32_t k_max_try_cnt = 300; /* retry 3 sec for -ERESTARTSYS */
	const uint32_t k_restart_sleep_min_us = 10 * 1000; /* 10 ms */
	const uint32_t k_restart_sleep_max_us = (k_restart_sleep_min_us + 200);
	struct ipi_msg_t *p_ipi_msg = NULL;


	ipi_dbg("in, dsp_id: %u, ipi_id: %u, buf: %p, len: %u, wait_ms: %u",
		dsp_id, ipi_id, buf, len, wait_ms);

	if (dsp_id >= NUM_OPENDSP_TYPE) {
		pr_info("dsp_id: %u error!!", dsp_id);
		return -EOVERFLOW;
	}
	if (buf == NULL || len > DSP_MSG_BUFFER_SIZE) {
		pr_info("buf: %p, len: %u!! return", buf, len);
		return -EFAULT;
	}
	if (is_audio_dsp_ready(dsp_id) == false) {
		pr_info("dsp_id: %u not ready!! ipi_id: %u",
			dsp_id, ipi_id);
		return 0;
	}

	msg_queue = &g_dsp_msg_queue[dsp_id][DSP_PATH_A2D];
	if (msg_queue->enable == false) {
		pr_info("queue disabled!! return");
		return -1;
	}


	/* NEVER sleep in ISR */
	if (in_interrupt() && wait_ms != 0) {
		pr_info("in_interrupt()!! wait_ms %u => 0!!", wait_ms);
		wait_ms = 0;
	}

	/* push message to queue */
	spin_lock_irqsave(&msg_queue->queue_lock, flags);
	retval = dsp_push_msg(msg_queue,
			      ipi_id,
			      buf,
			      len,
			      NULL, /* only for recv msg from dsp */
			      (wait_ms != 0),
			      &idx_msg,
			      &queue_counter);
	spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
	if (retval != 0) {
		pr_info("dsp_id: %u, push fail!!", dsp_id);
		return retval;
	}

	/* notify queue thread to process it */
	dsb(SY);
	wake_up_interruptible(&msg_queue->queue_wq);

	/* no need to wait */
	if (wait_ms == 0) {
		ipi_dbg("out, wait_ms == 0, exit");
		return 0;
	}


	/* wait until message processed */
	p_element = &msg_queue->element[idx_msg];
	for (try_cnt = 0; try_cnt < k_max_try_cnt; try_cnt++) {
		retval = wait_event_interruptible_timeout(
				 p_element->element_wq,
				 (p_element->signal_arrival == true ||
				  msg_queue->enable == false),
				 msecs_to_jiffies(wait_ms));

		if (msg_queue->enable == false) {
			pr_info("queue disable");
			retval = 0;
			break;
		}
		if (p_element->signal_arrival == true) {
			retval = 0;
			break;
		}
		if (retval > 0) {
			pr_notice("wait ret %d, sig %d, enable %d",
				  retval,
				  p_element->signal_arrival,
				  msg_queue->enable);
			retval = 0;
			break;
		}
		if (retval == 0) { /* timeout */
			p_ipi_msg = (struct ipi_msg_t *)p_element->msg.buf;
			pr_info("wait %u ms timeout, will still send to dsp later!! msg: 0x%x, task: %d",
				wait_ms, p_ipi_msg->msg_id,
				p_ipi_msg->task_scene);
			break;
		}
		if (retval == -ERESTARTSYS) {
			pr_info("-ERESTARTSYS, #%u, sleep us: %u",
				try_cnt, k_restart_sleep_min_us);
			retval = -EINTR;
			usleep_range(k_restart_sleep_min_us,
				     k_restart_sleep_max_us);
			continue;
		}
		pr_notice("retval: %d not handle!!", retval);
	}

	/* get ipc result */
	spin_lock_irqsave(&p_element->element_lock, flags);
	if (p_element->queue_counter == queue_counter) {
		/* keep retval of dsp_send_msg_to_dsp */
		if (p_element->signal_arrival == true)
			retval = p_element->send_retval;
		p_element->wait_in_thread = false;
	}
	spin_unlock_irqrestore(&p_element->element_lock, flags);



	ipi_dbg("out, dsp_id: %u, ipi_id: %u, buf: %p, len: %u, wait_ms: %u",
		dsp_id, ipi_id, buf, len, wait_ms);

	return retval;
}
EXPORT_SYMBOL(dsp_send_msg_to_queue);

int dsp_dispatch_ipi_hanlder_to_queue(
	uint32_t dsp_id, /* enum dsp_id_t */
	uint32_t ipi_id,
	void *buf,
	uint32_t len,
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len))
{
	struct dsp_msg_queue_t *msg_queue = NULL;

	uint32_t idx_msg = 0;
	uint32_t queue_counter = 0;

	int retval = 0;
	unsigned long flags = 0;

	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -EOVERFLOW;
	if (buf == NULL || len > DSP_MSG_BUFFER_SIZE)
		return -EFAULT;
	if (ipi_handler == NULL)
		return -EFAULT;

	msg_queue = &g_dsp_msg_queue[dsp_id][DSP_PATH_D2A];
	if (msg_queue->enable == false)
		return -1;


	/* push message to queue */
	spin_lock_irqsave(&msg_queue->queue_lock, flags);
	retval = dsp_push_msg(msg_queue,
			      ipi_id,
			      buf,
			      len,
			      ipi_handler,
			      false,
			      &idx_msg,
			      &queue_counter);
	spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
	if (retval != 0)
		return retval;

	/* notify queue thread to process it */
	dsb(SY);
	wake_up_interruptible(&msg_queue->queue_wq);

	return 0;
}
EXPORT_SYMBOL(dsp_dispatch_ipi_hanlder_to_queue);


/*
 * =============================================================================
 *                     private implementation
 * =============================================================================
 */

static int dsp_push_msg(
	struct dsp_msg_queue_t *msg_queue,
	uint32_t ipi_id,
	void *buf,
	uint32_t len,
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len),
	bool wait_in_thread,
	uint32_t *p_idx_msg,
	uint32_t *p_queue_counter)
{
	struct ipi_msg_t *p_ipi_msg = NULL;
	char dump_str[128] = {0};

	struct dsp_msg_t *p_dsp_msg = NULL;
	struct dsp_queue_element_t *p_element = NULL;

	bool need_retry = false;
	unsigned long flags = 0;
	int n = 0;

	if (msg_queue == NULL || buf == NULL ||
	    p_idx_msg == NULL || p_queue_counter == NULL)
		return -EFAULT;

	/* check queue full */
	if (dsp_check_queue_to_be_full(msg_queue) == true) {
		n = snprintf(dump_str, sizeof(dump_str),
			     "dsp_id: %u, ipi_id: %u, queue overflow, idx_r: %u, idx_w: %u, drop it",
			     msg_queue->dsp_id,
			     ipi_id, msg_queue->idx_r,
			     msg_queue->idx_w);
		if (n < 0 || n > sizeof(dump_str))
			pr_info("error to get string dump_str");

		p_ipi_msg = (struct ipi_msg_t *)buf;
		if (len >= IPI_MSG_HEADER_SIZE &&
		    p_ipi_msg->magic == IPI_MSG_MAGIC_NUMBER) {
			if (p_ipi_msg->ack_type == AUDIO_IPI_MSG_NEED_ACK ||
			    p_ipi_msg->ack_type == AUDIO_IPI_MSG_ACK_BACK) {
				DUMP_IPI_MSG(dump_str, p_ipi_msg);
				need_retry = true;
			}
		} else {
			pr_info("%s", dump_str);
			need_retry = true;
		}

		return (need_retry) ? -EOVERFLOW : 0;
	}

	if (dsp_check_idx_msg_valid(msg_queue, msg_queue->idx_w) == false)
		return -1;


	/* push */
	*p_idx_msg = msg_queue->idx_w;
	msg_queue->idx_w++;
	if (msg_queue->idx_w == msg_queue->size)
		msg_queue->idx_w = 0;

	/* copy */
	p_element = &msg_queue->element[*p_idx_msg];
	spin_lock_irqsave(&p_element->element_lock, flags);

	p_dsp_msg = &p_element->msg;
	p_dsp_msg->ipi_id = ipi_id;
	memcpy((void *)p_dsp_msg->buf, buf, len);
	p_dsp_msg->len = len;
	p_dsp_msg->ipi_handler = ipi_handler;

	p_element->wait_in_thread = wait_in_thread;
	p_element->signal_arrival = false;
	p_element->send_retval = 0;
	p_element->queue_counter = msg_queue->queue_counter;

	*p_queue_counter = msg_queue->queue_counter;
	if (msg_queue->queue_counter == 0xFFFFFFFF)
		msg_queue->queue_counter = 0;
	else
		msg_queue->queue_counter++;
	spin_unlock_irqrestore(&p_element->element_lock, flags);

	return 0;
}


static int dsp_pop_msg(struct dsp_msg_queue_t *msg_queue)
{
	struct dsp_msg_t *p_dsp_msg = NULL;

	if (msg_queue == NULL) {
		pr_info("NULL!! msg_queue: %p", msg_queue);
		return -EFAULT;
	}

	/* check queue empty */
	if (dsp_check_queue_empty(msg_queue) == true) {
		pr_info("dsp_id: %u, queue is empty, idx_r: %u, idx_w: %u",
			msg_queue->dsp_id,
			msg_queue->idx_r,
			msg_queue->idx_w);
		return -1;
	}

	/* pop */
	p_dsp_msg = &msg_queue->element[msg_queue->idx_r].msg;
	msg_queue->idx_r++;
	if (msg_queue->idx_r == msg_queue->size)
		msg_queue->idx_r = 0;


	ipi_dbg("dsp_id: %u, dsp_path: %u, ipi_id: %u, idx_r: %u, idx_w: %u, queue(%u/%u)",
		msg_queue->dsp_id,
		msg_queue->dsp_path,
		p_dsp_msg->ipi_id,
		msg_queue->idx_r,
		msg_queue->idx_w,
		dsp_get_num_messages_in_queue(msg_queue),
		msg_queue->size);

	return 0;
}


static int dsp_front_msg(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t **pp_dsp_msg,
	uint32_t *p_idx_msg)
{
	if (msg_queue == NULL || pp_dsp_msg == NULL || p_idx_msg == NULL) {
		pr_info("NULL!! msg_queue: %p, pp_dsp_msg: %p, p_idx_msg: %p",
			msg_queue, pp_dsp_msg, p_idx_msg);
		return -EFAULT;
	}

	*pp_dsp_msg = NULL;
	*p_idx_msg = 0xFFFFFFFF;

	/* check queue empty */
	if (dsp_check_queue_empty(msg_queue) == true) {
		pr_info("dsp_id: %u, queue empty, idx_r: %u, idx_w: %u",
			msg_queue->dsp_id,
			msg_queue->idx_r, msg_queue->idx_w);
		return -ENOMEM;
	}

	/* front */
	if (dsp_check_idx_msg_valid(msg_queue, msg_queue->idx_r) == false) {
		pr_info("idx_r %u is invalid!! return",
			msg_queue->idx_r);
		return -1;
	}
	*p_idx_msg = msg_queue->idx_r;
	*pp_dsp_msg = &msg_queue->element[*p_idx_msg].msg;

	ipi_dbg("%s(), dsp_id: %u, dsp_path: %u, ipi_id: %u, idx_r: %u, idx_w: %u, queue(%u/%u), *p_idx_msg: %u",
		__func__,
		msg_queue->dsp_id,
		msg_queue->dsp_path,
		msg_queue->element[*p_idx_msg].msg.ipi_id,
		msg_queue->idx_r,
		msg_queue->idx_w,
		dsp_get_num_messages_in_queue(msg_queue),
		msg_queue->size,
		*p_idx_msg);

	return 0;
}


static int dsp_get_queue_element(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t **pp_dsp_msg,
	uint32_t *p_idx_msg)
{
	bool is_empty = true;

	unsigned long flags = 0;
	int retval = 0;

	uint32_t try_cnt = 0;
	const uint32_t k_max_try_cnt = 300; /* retry 3 sec for -ERESTARTSYS */
	const uint32_t k_restart_sleep_min_us = 10 * 1000; /* 10 ms */
	const uint32_t k_restart_sleep_max_us = (k_restart_sleep_min_us + 200);

	spin_lock_irqsave(&msg_queue->queue_lock, flags);
	is_empty = dsp_check_queue_empty(msg_queue);
	spin_unlock_irqrestore(&msg_queue->queue_lock, flags);

	/* wait until message is pushed to queue */
	if (is_empty == true) {
		for (try_cnt = 0; try_cnt < k_max_try_cnt; try_cnt++) {
			retval = wait_event_interruptible(
					 msg_queue->queue_wq,
					 (!dsp_check_queue_empty(msg_queue) ||
					  !msg_queue->thread_enable));
			if (msg_queue->thread_enable == false) {
				pr_info("thread disable");
				retval = -1;
				break;
			}
			if (dsp_check_queue_empty(msg_queue) == false) {
				retval = 0;
				break;
			}
			if (retval == 0) { /* got msg in queue */
				pr_notice("wait ret 0, empty %d, enable %d",
					  dsp_check_queue_empty(msg_queue),
					  msg_queue->enable);
				break;
			}
			if (retval == -ERESTARTSYS) {
				pr_info("-ERESTARTSYS, #%u, sleep us: %u",
					try_cnt, k_restart_sleep_min_us);
				retval = -EINTR;
				usleep_range(k_restart_sleep_min_us,
					     k_restart_sleep_max_us);
				continue;
			}
			pr_notice("retval: %d not handle!!", retval);
		}
	}

	if (dsp_check_queue_empty(msg_queue) == false) {
		spin_lock_irqsave(&msg_queue->queue_lock, flags);
		retval = dsp_front_msg(msg_queue, pp_dsp_msg, p_idx_msg);
		spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
	}

	return retval;
}


static int dsp_init_single_msg_queue(
	struct dsp_msg_queue_t *msg_queue,
	const uint32_t dsp_id,
	const uint32_t dsp_path)
{
	struct dsp_queue_element_t *p_element = NULL;
	struct dsp_msg_t *p_dsp_msg = NULL;

	char thread_name[32] = {0};

	int i = 0, n = 0;

	if (msg_queue == NULL) {
		pr_info("NULL!! msg_queue: %p", msg_queue);
		return -EFAULT;
	}
	if (dsp_id >= NUM_OPENDSP_TYPE || dsp_path >= DSP_NUM_PATH) {
		pr_info("dsp_id: %u, dsp_path: %u error!!",
			dsp_id, dsp_path);
		return -EFAULT;
	}


	/* check double init */
	if (msg_queue->init) {
		pr_info("dsp_id: %u already init!!", dsp_id);
		return 0;
	}
	msg_queue->init = true;

	/* init var */
	msg_queue->dsp_id = dsp_id;
	msg_queue->dsp_path = dsp_path;

	for (i = 0; i < MAX_DSP_MSG_NUM_IN_QUEUE; i++) {
		p_element = &msg_queue->element[i];

		p_dsp_msg = &p_element->msg;
		p_dsp_msg->ipi_id = 0;
		memset((void *)p_dsp_msg->buf, 0, DSP_MSG_BUFFER_SIZE);
		p_dsp_msg->len = 0;
		p_dsp_msg->ipi_handler = NULL;

		spin_lock_init(&p_element->element_lock);
		init_waitqueue_head(&p_element->element_wq);

		p_element->wait_in_thread = false;
		p_element->signal_arrival = false;
		p_element->send_retval = 0;
		p_element->queue_counter = 0;
	}

	msg_queue->size = MAX_DSP_MSG_NUM_IN_QUEUE;
	msg_queue->idx_r = 0;
	msg_queue->idx_w = 0;

	msg_queue->queue_counter = 0;

	spin_lock_init(&msg_queue->queue_lock);
	init_waitqueue_head(&msg_queue->queue_wq);


	if (dsp_path == DSP_PATH_A2D) {
		msg_queue->dsp_process_msg_func = dsp_send_msg_to_dsp;
		n = snprintf(thread_name,
			     sizeof(thread_name),
			     "dsp_send_thread_id_%u", dsp_id);
		if (n < 0 || n > sizeof(thread_name))
			pr_info("error to get string thread_name");

	} else if (dsp_path == DSP_PATH_D2A) {
		msg_queue->dsp_process_msg_func = dsp_process_msg_from_dsp;
		n = snprintf(thread_name,
			     sizeof(thread_name),
			     "dsp_recv_thread_id_%u", dsp_id);
		if (n < 0 || n > sizeof(thread_name))
			pr_info("error to get string thread_name");
	} else
		WARN_ON(1);

	/* lunch thread */
	msg_queue->dsp_thread_task = kthread_create(
					     dsp_process_msg_thread,
					     msg_queue,
					     "%s",
					     thread_name);
	if (IS_ERR(msg_queue->dsp_thread_task)) {
		pr_info("can not create %s kthread", thread_name);
		WARN_ON(1);
		msg_queue->thread_enable = false;
	} else {
		msg_queue->thread_enable = true;
		dsb(SY);
		wake_up_process(msg_queue->dsp_thread_task);
	}

	/* enable */
	msg_queue->enable = true;


	return (msg_queue->enable && msg_queue->thread_enable) ? 0 : -EFAULT;
}


static int dsp_process_msg_thread(void *data)
{
	struct dsp_msg_queue_t *msg_queue = (struct dsp_msg_queue_t *)data;
	struct dsp_queue_element_t *p_element = NULL;
	struct dsp_msg_t *p_dsp_msg = NULL;
	uint32_t idx_msg = 0;

	unsigned long flags = 0;
	int retval = 0;

	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return -EFAULT;
	}

	set_user_nice(current, -20); /* normal thread highest priority */

	while (msg_queue->thread_enable && !kthread_should_stop()) {
		/* wait until element pushed */
		retval = dsp_get_queue_element(msg_queue, &p_dsp_msg, &idx_msg);
		if (retval != 0) {
			pr_info("dsp_get_queue_element retval %d", retval);
			continue;
		}
		if (idx_msg >= MAX_DSP_MSG_NUM_IN_QUEUE) {
			pr_info("dsp_get_queue_element idx_msg %u > %u",
				idx_msg, MAX_DSP_MSG_NUM_IN_QUEUE);
			spin_lock_irqsave(&msg_queue->queue_lock, flags);
			dsp_pop_msg(msg_queue);
			spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
			continue;
		}
		p_element = &msg_queue->element[idx_msg];

		/* send to dsp */
		retval = msg_queue->dsp_process_msg_func(msg_queue, p_dsp_msg);

		/* notify element if need */
		spin_lock_irqsave(&p_element->element_lock, flags);
		if (p_element->wait_in_thread == true) {
			p_element->send_retval = retval;
			p_element->signal_arrival = true;
			dsb(SY);
			wake_up_interruptible(&p_element->element_wq);
		}
		spin_unlock_irqrestore(&p_element->element_lock, flags);


		/* pop message from queue */
		spin_lock_irqsave(&msg_queue->queue_lock, flags);
		dsp_pop_msg(msg_queue);
		spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
	}


	pr_info("thread exit, dsp_id %u, dsp_path %u, idx_r %d, idx_w %d",
		msg_queue->dsp_id, msg_queue->dsp_path,
		msg_queue->idx_r, msg_queue->idx_w);

	return 0;
}

static int audio_ipi_send_msg_to_adsp(const struct dsp_msg_t *p_dsp_msg,
				      const uint32_t dsp_id)
{
#if !IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	DUMP_IPC_MSG("DSP not support!!", p_dsp_msg);
	return -ENODEV;
#else
	uint32_t core_id = 0xFFFFFFFF;
	int ret = -1;

	if (p_dsp_msg == NULL)
		return -EFAULT;

	core_id = ipi_dsp_id_to_adsp_cid(dsp_id);
	if (core_id == 0xFFFFFFFF)
		return -ENODEV;

	ret = adsp_send_message(
		      (enum adsp_ipi_id)p_dsp_msg->ipi_id,
		      (void *)p_dsp_msg->buf,
		      p_dsp_msg->len,
		      1, /* wait until sent or timeout */
		      (enum adsp_core_id)core_id);

	if (ret == ADSP_IPI_DONE)
		ret = 0;
	else if (ret == ADSP_IPI_BUSY)
		ret = -EBUSY;
	else if (ret == ADSP_IPI_ERROR)
		ret = -ECOMM;

#ifdef DEBUG_IPI
	if (ret == 0)
		DUMP_IPC_MSG("send pass", p_dsp_msg);
#endif

	return ret;
#endif
}

static int audio_ipi_send_msg_to_scp(struct dsp_msg_t *p_dsp_msg,
				     const uint32_t dsp_id)
{
#if !IS_ENABLED(CONFIG_MTK_AUDIO_CM4_SUPPORT)
	DUMP_IPC_MSG("DSP not support!!", p_dsp_msg);
	return -ENODEV;
#else
	uint32_t core_id = 0xFFFFFFFF;
	int ret = -1;

	if (p_dsp_msg == NULL)
		return -EFAULT;

	core_id = ipi_dsp_id_to_scp_cid(dsp_id);
	if (core_id == 0xFFFFFFFF)
		return -ENODEV;

	ret = scp_ipi_send(
		      (enum ipi_id)p_dsp_msg->ipi_id,
		      (void *)p_dsp_msg->buf,
		      p_dsp_msg->len,
		      0, /* avoid busy waiting */
		      (enum scp_core_id)core_id);

	if (ret == SCP_IPI_DONE)
		ret = 0;
	else if (ret == SCP_IPI_BUSY)
		ret = -EBUSY;
	else if (ret == SCP_IPI_ERROR)
		ret = -ECOMM;

#ifdef DEBUG_IPI
	if (ret == 0)
		DUMP_IPC_MSG("send pass", p_dsp_msg);
#endif

	return ret;
#endif
}

static int dsp_send_msg_to_dsp(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t *p_dsp_msg)
{
	uint32_t dsp_id = 0xFFFFFFFF;
	uint32_t audio_ipi_id = 0xFFFFFFFF;
	struct ipi_msg_t *p_ipi_msg = NULL;

	static bool start_flag[NUM_OPENDSP_TYPE] = {0};
	bool retry_flag = false;
	char dump_str[64] = {0};

	uint32_t try_cnt = 0;
	const uint32_t k_max_try_cnt = 1000 * 20; /* 1 sec for write ipc */
	const uint32_t k_sleep_min_us = 50;
	const uint32_t k_sleep_max_us = (k_sleep_min_us + 10);

	int ret = 0;
	int n = 0;

	if (msg_queue == NULL || p_dsp_msg == NULL) {
		pr_info("NULL!! msg_queue: %p, p_dsp_msg: %p",
			msg_queue, p_dsp_msg);
		return -EFAULT;
	}

	dsp_id = msg_queue->dsp_id;
	if (!is_audio_dsp_support(dsp_id))
		return -ENODEV;

	audio_ipi_id = audio_get_audio_ipi_id_by_dsp(dsp_id);

	/* IPC */
	for (try_cnt = 0; try_cnt < k_max_try_cnt; try_cnt++) {
		if (is_audio_use_scp(dsp_id))
			ret = audio_ipi_send_msg_to_scp(p_dsp_msg, dsp_id);
		else if (is_audio_use_adsp(dsp_id))
			ret = audio_ipi_send_msg_to_adsp(p_dsp_msg, dsp_id);
		else {
			pr_notice("dsp_id %u not support!!", dsp_id);
			ret = -ENODEV;
			break;
		}

		if (ret == 0) { /* pass */
			start_flag[dsp_id] = true;
			if (p_dsp_msg->ipi_id == audio_ipi_id) {
				p_ipi_msg = (struct ipi_msg_t *)p_dsp_msg->buf;
				if (check_print_msg_info(p_ipi_msg) &&
				    !retry_flag)
					DUMP_IPI_MSG("pass", p_ipi_msg);
			}
			ret = 0;
			break;
		}
		if (start_flag[dsp_id] == false) {
			if (try_cnt == 0) /* only dump log once */
				DUMP_IPC_MSG("booting..., retry", p_dsp_msg);
			retry_flag = true;
			ret = -ENODEV;
			usleep_range(k_sleep_min_us, k_sleep_max_us);
			continue;
		}
		if (is_audio_dsp_ready(dsp_id) == false) {
			n = snprintf(dump_str, sizeof(dump_str),
				     "dsp_id %u dead!!", dsp_id);
			if (n < 0 || n > sizeof(dump_str))
				pr_info("error to get string dump_str");
			DUMP_IPC_MSG(dump_str, p_dsp_msg);
			ret = 0;
			break;
		}
		if (ret == -ECOMM) { /* send fail */
			n = snprintf(dump_str, sizeof(dump_str),
				     "dsp_id %u error!!", dsp_id);
			if (n < 0 || n > sizeof(dump_str))
				pr_info("error to get string dump_str");
			DUMP_IPC_MSG(dump_str, p_dsp_msg);
			ret = -1;
			break;
		}
		if (ret == -EBUSY) { /* busy */
			if (try_cnt == 0) /* only dump log once */
				DUMP_IPC_MSG("IPC busy, retry", p_dsp_msg);
			retry_flag = true;
			ret = -ETIMEDOUT;
			usleep_range(k_sleep_min_us, k_sleep_max_us);
			continue;
		}
		pr_notice("ret: %d not handle!!", ret);
	}

	if (retry_flag == true) {
		if (ret == 0) {
			n = snprintf(dump_str, sizeof(dump_str),
				     "dsp_id %u retry %u pass", dsp_id, try_cnt);
			if (n < 0 || n > sizeof(dump_str))
				pr_info("error to get string dump_str");
		} else {
			n = snprintf(dump_str, sizeof(dump_str),
				     "dsp_id %u retry %u err ret %d",
				     dsp_id, try_cnt, ret);
			if (n < 0 || n > sizeof(dump_str))
				pr_info("error to get string dump_str");
		}
		DUMP_IPC_MSG(dump_str, p_dsp_msg);
	}

	return ret;
}


static int dsp_process_msg_from_dsp(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t *p_dsp_msg)
{
	if (msg_queue == NULL || p_dsp_msg == NULL) {
		pr_info("NULL!! msg_queue: %p, p_dsp_msg: %p",
			msg_queue, p_dsp_msg);
		return -EFAULT;
	}

	if (p_dsp_msg->ipi_handler == NULL) {
		pr_info("NULL!! p_dsp_msg->ipi_handler: %p",
			p_dsp_msg->ipi_handler);
		return -EFAULT;
	}

	if (p_dsp_msg->len == 0) {
		pr_info("p_dsp_msg->len: %u", p_dsp_msg->len);
		return -EFAULT;
	}

	p_dsp_msg->ipi_handler(
		p_dsp_msg->ipi_id,
		p_dsp_msg->buf,
		p_dsp_msg->len);

	return 0;
}


