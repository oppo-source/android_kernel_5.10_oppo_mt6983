/***********************************************************
** Copyright (C) 2018-2020 Oplus. All rights reserved.
** File: hans.h
** Description: Add for hans freeze manager
**
** Version: 1.0
** Date : 2019/09/23
** Author:
**
** ------------------ Revision History:------------------------
** <author>      <data>      <version >       <desc>
** Kun Zhou    2019/09/23      1.0       OPLUS_ARCH_EXTENDS
** Kun Zhou    2019/09/23      1.1       OPLUS_FEATURE_HANS_FREEZE
** Qingxin Guo 2021/08/04      1.2       GENERIC NETLINK
****************************************************************/

#ifndef _HANS_H
#define _HANS_H

#include <linux/freezer.h>
#include <linux/cgroup.h>
#include <uapi/linux/android/binder.h>
#include <trace/hooks/binder.h>
#include <trace/hooks/signal.h>
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
#include "../../../../../kernel_platform/common/kernel/sched/sched.h"
#include "../../../../../kernel_platform/common/drivers/android/binder_internal.h"
#include "../../../../../kernel_platform/common/drivers/android/binder_alloc.h"
#else
#include "../../../../../kernel-5.10/kernel/sched/sched.h"
#include "../../../../../kernel-5.10/drivers/android/binder_internal.h"
#include "../../../../../kernel-5.10/drivers/android/binder_alloc.h"
#endif

#define HANS_NOERROR             (0)
#define HANS_ERROR               (-1)
#define MIN_USERAPP_UID (10000)
#define HANS_SYSTEM_UID (1000)
#define INTERFACETOKEN_BUFF_SIZE (140)
#define PARCEL_OFFSET (16) /* sync with the writeInterfaceToken */
#define CPUCTL_VERSION (2)
#define TRANS_BINDER_DEBUG_ON (-10000)
#define TRANS_BINDER_DEBUG_OFF (-9000)

#define HANS_FAMILY_VERSION  1
#define HANS_FAMILY  "oplus_hans"
#define GENL_ID_GENERATE    0
#define NLA_DATA(na) ((char *)((char *)(na) + NLA_HDRLEN))
#define NLA_PAYLOAD(len) (len - NLA_HDRLEN)

/* attribute type */
enum {
	HANS_ATTR_MSG_UNDEFINE = 0,
	HANS_ATTR_MSG_GENL,
	__HANS_ATTR_MSG_MAX
};
#define HANS_ATTR_MSG_MAX (__HANS_ATTR_MSG_MAX - 1)

/* cmd type */
enum {
	HANS_CMD_UNDEFINE = 0,
	HANS_CMD_GENL,
	__HANS_CMD_MAX,
};
#define HANS_CMD_MAX (__HANS_CMD_MAX - 1)

/* hans_message for comunication with HANS native deamon
 * type: async binder/sync binder/signal/pkg/loopback
 *      Only loop back type is duplex (native deamon <---> kernel) for handshake
 * port: native deamon pid
 * caller_pid: binder, caller -> unfreeze (target) UID
 * target_uid: UID want to be unfrozen
 * pkg_cmd: Add/Remove monitored UID
 */
struct hans_message {
	int type;
	int port;  /* pid */

	int caller_uid;  /*caller -> unfreeze UID*/
	int caller_pid;  /*caller -> unfreeze UID*/
	int target_pid;  /*unfreeze UID, pkg add/remove UID*/
	int target_uid;  /*unfreeze UID, pkg add/remove UID*/

	int pkg_cmd;     /*Add/remove monitored uid*/

	int code;
	char rpc_name[INTERFACETOKEN_BUFF_SIZE];
};

/* hans message type definition */
enum message_type {
	/* kernel --> native deamon */
	ASYNC_BINDER,
	SYNC_BINDER,
	FROZEN_TRANS,
	SIGNAL,
	PKG,

	/*For cpuclt solution*/
	SYNC_BINDER_CPUCTL,
	SIGNAL_CPUCTL,
	CPUCTL_TRANS,

	/*kernel <--> native deamon*/
	LOOP_BACK,
	TYPE_MAX
};

/* pkg cmd type */
enum pkg_cmd {
	ADD_ONE_UID,
	DEL_ONE_UID,
	DEL_ALL_UID,

	PKG_CMD_MAX
};

extern bool trans_binder_debug;
/* Check if the thread group is frozen */
static inline bool is_frozen_tg(struct task_struct *task)
{
	return (freezing(task->group_leader) || frozen(task->group_leader));
}

int hans_report(enum message_type type, int caller_pid, int caller_uid,
		int target_pid, int target_uid, const char *rpc_name, int code);
void hans_network_cmd_parse(uid_t uid, enum pkg_cmd cmd);
void hans_check_frozen_transcation(uid_t uid, enum message_type type);
int hans_netfilter_init(void);
void hans_netfilter_deinit(void);
void hans_check_async_binder_buffer(bool is_async, int free_async_space,
				    int size, int binder_buffer_size, int alloc_buffer_size, int pid);
void hans_check_signal(struct task_struct *p, int sig);

void binder_preset_handler(void *data, struct hlist_head *hhead,
			   struct mutex *lock);
void binder_trans_handler(void *data, struct binder_proc *target_proc,
			  struct binder_proc *proc, struct binder_thread *thread,
			  struct binder_transaction_data *tr);
void binder_reply_handler(void *data, struct binder_proc *target_proc,
			  struct binder_proc *proc, struct binder_thread *thread,
			  struct binder_transaction_data *tr);
void binder_alloc_handler(void *data, size_t size, struct binder_alloc *alloc,
			  int is_async);
void send_signal_handler(void *data, int sig, struct task_struct *killer,
			 struct task_struct *dst);


#if defined(CONFIG_CFS_BANDWIDTH)
static inline bool is_belong_cpugrp(struct task_struct *task)
{
	if (task->sched_task_group != NULL) {
		struct cfs_bandwidth cfs_b = task->sched_task_group->cfs_bandwidth;

		if (cfs_b.quota != -1)
			return true;

		else if (cfs_b.quota == -1)
			return false;
	}

	return false;
}
#else
static inline bool is_belong_cpugrp(struct task_struct *task)
{
	return false;
}
#endif  /*CONFIG_CFS_BANDWIDTH*/
#endif  /*_HANS_H*/
