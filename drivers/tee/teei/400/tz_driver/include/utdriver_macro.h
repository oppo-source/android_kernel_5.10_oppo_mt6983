/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2015-2019, MICROTRUST Incorporated
 * All Rights Reserved.
 *
 */

#ifndef UTDRIVER_MACRO_H
#define UTDRIVER_MACRO_H

/* define TEEI_MUTIL_TA_DEBUG */

#define START_STATUS			(0)
#define END_STATUS			(1)


#define VFS_SIZE			(0x80000)
#define VDRV_MAX_SIZE			(0x80000)
#define KEYMASTER_BUFF_SIZE		(512 * 1024)

/* Command ID in the notify queue entry */
#define TEEI_CREAT_FDRV			(0x40)
#define TEEI_CREAT_BDRV			(0x41)
#define TEEI_LOAD_TEE			(0x42)
#define NEW_CAPI_CALL			(0x43)
#define TEEI_BDRV_CALL			(0x44)
#define TEEI_FDRV_CALL			(0x45)
#define TEEI_SCHED_CALL			(0x46)
#define TEEI_MODIFY_TEE_CONFIG		(0x47)
/* Command ID in the notify queue entry END */



/* Switch function ID */
#define SMC_CALL_TYPE			(0x10)
#define SWITCH_CORE_TYPE		(0x11)
/* Switch function ID END */

#define VFS_SYS_NO                      0x08
#define REETIME_SYS_NO                  0x07
#define SCHED_SYS_NO			0x09
#define CANCEL_SYS_NO                   (110)


#define MAX_BUFF_SIZE			(0x2000)

#define NQ_SIZE				(0x1000)
#define NQ_BUFF_SIZE			(0x1000)
#define NQ_BLOCK_SIZE			(64)
#define BLOCK_MAX_COUNT			(NQ_BUFF_SIZE / NQ_BLOCK_SIZE - 1)

#define CTL_BUFF_SIZE			(0x1000)

#define MESSAGE_LENGTH			(0x1000)
#define MESSAGE_SIZE			(0x1000)

/* #define DYNAMIC_SET_PRIORITY */

#endif /* end of UTDRIVER_MACRO_H */
