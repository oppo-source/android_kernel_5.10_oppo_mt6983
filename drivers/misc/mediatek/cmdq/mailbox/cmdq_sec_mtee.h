/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __CMDQ_SEC_MTEE_H__
#define __CMDQ_SEC_MTEE_H__

#include <linux/types.h>
#include <linux/delay.h>

#include <linux/limits.h>
#include <kree/system.h>
#include <kree/mem.h>

//#include "tee_client_api.h"
#if IS_ENABLED(CONFIG_MICROTRUST_TEE_SUPPORT)
#include "teei_client_main.h"
#endif
#if IS_ENABLED(CONFIG_TRUSTONIC_TEE_SUPPORT)
#include "mobicore_driver_api.h"
#endif

/* context for tee vendor */
struct cmdq_sec_mtee_context {
	char ta_uuid[NAME_MAX];
	KREE_SESSION_HANDLE pHandle;

	char wsm_uuid[NAME_MAX];
	KREE_SESSION_HANDLE wsm_pHandle;

	KREE_SHAREDMEM_HANDLE wsm_handle;
	KREE_SHAREDMEM_PARAM wsm_param;
	KREE_SHAREDMEM_HANDLE wsm_ex_handle;
	KREE_SHAREDMEM_PARAM wsm_ex_param;
	KREE_SHAREDMEM_HANDLE wsm_ex2_handle;
	KREE_SHAREDMEM_PARAM wsm_ex2_param;
	KREE_SHAREDMEM_HANDLE mem_handle;
	KREE_SHAREDMEM_PARAM mem_param;
#if 0
	/* Universally Unique Identifier of secure tl/dr */
	struct TEEC_UUID uuid;
	struct TEEC_Context gp_context; /* basic context */
	struct TEEC_Session session; /* session handle */
	struct TEEC_SharedMemory wsm_param; /* shared memory */
	struct TEEC_SharedMemory wsm_ex_param; /* shared memory */
#endif
};

#endif	/* __CMDQ_SEC_MTEE_H__ */
