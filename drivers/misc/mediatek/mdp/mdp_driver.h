/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#ifndef __MDP_DRIVER_H__
#define __MDP_DRIVER_H__

#include <linux/kernel.h>
#include "mdp_def.h"
#include "mdp_def_ex.h"

struct cmdqUsageInfoStruct {
	/* [OUT] current engine ref count */
	uint32_t count[CMDQ_MAX_ENGINE_COUNT];
};

struct cmdqJobStruct {
	struct cmdqCommandStruct command;	/* [IN] the job to perform */
	cmdqJobHandle_t hJob;	/* [OUT] handle to resulting job */
};

struct cmdqJobResultStruct {
	/* [IN]  Job handle from CMDQ_IOCTL_ASYNC_JOB_EXEC */
	cmdqJobHandle_t hJob;
	uint64_t engineFlag;	/* [OUT] engine flag passed down originally */

	/* [IN/OUT] read register values, if any.
	 * as input, the "count" field must represent
	 * buffer space pointed by "regValues".
	 * Upon return, CMDQ driver fills "count" with
	 * actual requested register count.
	 * However, if the input "count" is too small,
	 * -ENOMEM is returned, and "count" is filled
	 * with requested register count.
	 */
	struct cmdqRegValueStruct regValue;

	/* [IN/OUT] physical address to read */
	struct cmdqReadAddressStruct readAddress;
};

struct cmdqWriteAddressStruct {
	/* [IN] count of the writable buffer
	 * (unit is # of u32, NOT in byte)
	 */
	uint32_t count;

	/* [OUT] When Alloc, this is the resulting PA.
	 * It is guaranteed to be continuous.
	 * [IN]  When Free, please pass returned address down to ioctl.
	 *
	 * indeed param startPA should be UNSIGNED LONG type for 64 bit kernel
	 * Considering our plartform supports max 4GB RAM
	 * (upper-32bit don't care for SW)
	 * and consistent common code interface, remain u32 type.
	 */
	uint32_t startPA;
};

#define CMDQ_IOCTL_MAGIC_NUMBER 'x'

#define CMDQ_IOCTL_LOCK_MUTEX   _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 1, int)
#define CMDQ_IOCTL_UNLOCK_MUTEX _IOR(CMDQ_IOCTL_MAGIC_NUMBER, 2, int)
#define CMDQ_IOCTL_EXEC_COMMAND _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 3, \
	struct cmdqCommandStruct)
#define CMDQ_IOCTL_QUERY_USAGE  _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 4, \
	struct cmdqUsageInfoStruct)

/*  */
/* Async operations */
/*  */
#define CMDQ_IOCTL_ASYNC_JOB_EXEC _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 5, \
	struct cmdqJobStruct)
#define CMDQ_IOCTL_ASYNC_JOB_WAIT_AND_CLOSE _IOR(CMDQ_IOCTL_MAGIC_NUMBER, 6, \
	struct cmdqJobResultStruct)

#define CMDQ_IOCTL_ALLOC_WRITE_ADDRESS _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 7, \
	struct cmdqWriteAddressStruct)
#define CMDQ_IOCTL_FREE_WRITE_ADDRESS _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 8, \
	struct cmdqWriteAddressStruct)
#define CMDQ_IOCTL_READ_ADDRESS_VALUE _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 9, \
	struct cmdqReadAddressStruct)

/*  */
/* Chip capability query. output parameter is a bit field. */
/* Bit definition is CMDQ_CAP_BITS. */
/*  */
#define CMDQ_IOCTL_QUERY_CAP_BITS _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 10, int)

/*  */
/* HW info. from DTS */
/*  */
#define CMDQ_IOCTL_QUERY_DTS _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 11, \
	struct cmdqDTSDataStruct)

/*  */
/* Notify MDP will use specified engine before really use. */
/* input int is same as EngineFlag. */
/*  */
#define CMDQ_IOCTL_NOTIFY_ENGINE _IOW(CMDQ_IOCTL_MAGIC_NUMBER, 12, uint64_t)

s32 cmdq_driver_ioctl_query_usage(struct file *pf, unsigned long param);
s32 cmdq_driver_ioctl_query_cap_bits(unsigned long param);
s32 cmdq_driver_ioctl_query_dts(unsigned long param);
s32 cmdq_driver_ioctl_notify_engine(unsigned long param);

void cmdq_driver_dump_readback(dma_addr_t *addrs, u32 count, u32 *values);

/* for mdp_fence.c header */
int mdp_sync_device_init(void);

#endif				/* __MDP_DRIVER_H__ */
