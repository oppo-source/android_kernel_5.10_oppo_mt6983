/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 * GenieZone (hypervisor-based seucrity platform) enables hardware protected
 * and isolated security execution environment, includes
 * 1. GZ hypervisor
 * 2. Hypervisor-TEE OS (built-in Trusty OS)
 * 3. Drivers (ex: debug, communication and interrupt) for GZ and
 *    hypervisor-TEE OS
 * 4. GZ and hypervisor-TEE and GZ framework (supporting multiple TEE
 *    ecosystem, ex: M-TEE, Trusty, GlobalPlatform, ...)
 */


/*
 * Header files for basic KREE functions.
 */

#ifndef __KREE_SYSTEM_H__
#define __KREE_SYSTEM_H__

#if IS_ENABLED(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT)	\
	|| IS_ENABLED(CONFIG_MTK_ENABLE_GENIEZONE)

#include <tz_cross/trustzone.h>
#include <gz-trusty/trusty.h>

#define MAX_UUID_LEN (40)

int gz_get_cpuinfo_thread(void *data);
void set_gz_bind_cpu(int on);
int get_gz_bind_cpu(void);
int ree_dummy_thread(void *data);

struct _cpus_cluster_freq {
	unsigned int max_freq;
	unsigned int min_freq;
};

#include "mem.h"

#if IS_ENABLED(CONFIG_GZ_VPU_WITH_M4U)
int gz_do_m4u_map(KREE_SHAREDMEM_HANDLE handle,
					phys_addr_t pa, uint32_t size,
					uint32_t region_id);
int gz_do_m4u_umap(KREE_SHAREDMEM_HANDLE handle);
#endif


/* Session Management */
/**
 *  Create a new TEE sesssion
 *
 * @param ta_uuid UUID of the TA to connect to.
 * @param pHandle Handle for the new session. Return KREE_SESSION_HANDLE_FAIL if
 * fail.
 * @return return code
 */
TZ_RESULT KREE_CreateSession(const char *ta_uuid, KREE_SESSION_HANDLE *pHandle);

/**
 * Close TEE session
 *
 * @param handle Handle for session to close.
 * @return return code
 */
TZ_RESULT KREE_CloseSession(KREE_SESSION_HANDLE handle);

/**
 * Make a TEE service call and schedule on a CPU mask
 *
 * @param handle      Session handle to make the call
 * @param command     The command to call.
 * @param paramTypes  Types for the parameters, use TZ_ParamTypes() to
 * consturct.
 * @param param       The parameters to pass to TEE. Maximum 4 params.
 * @param cpumask     CPU mask.
 * @return            Return value from TEE service.
 */
TZ_RESULT KREE_TeeServiceCallPlus(KREE_SESSION_HANDLE handle, uint32_t command,
				   uint32_t paramTypes, union MTEEC_PARAM param[4],
				   int32_t cpumask);

/**
 * Make a TEE service call
 *
 * @param handle      Session handle to make the call
 * @param command     The command to call.
 * @param paramTypes  Types for the parameters, use TZ_ParamTypes() to
 * consturct.
 * @param param       The parameters to pass to TEE. Maximum 4 params.
 * @return            Return value from TEE service.
 */
TZ_RESULT KREE_TeeServiceCall(KREE_SESSION_HANDLE handle, uint32_t command,
			      uint32_t paramTypes, union MTEEC_PARAM param[4]);


/* System Hardware Counter */
/**
 * Get system counter value.
 *
 * @return The system counter value
 */
u64 KREE_GetSystemCnt(void);


/**
 * Get system counter frequency.
 *
 * @return The system counter frequency
 */
u32 KREE_GetSystemCntFrq(void);

/**
 * KREE_SessionToTID - get tee id from session. Only works after the session
 * has been completely created.
 *
 * @param session	Session handle
 * @param o_tid		The output tee_id.
 *			This API always sets a tee_id value even when errors
 *			happened, but still needs to check the return value.
 * @return		TZ_RESULT_SUCCESS or errno.
 */
TZ_RESULT KREE_SessionToTID(KREE_SESSION_HANDLE session, enum tee_id_t *o_tid);

#endif /* CONFIG_MTK_IN_HOUSE_TEE_SUPPORT || CONFIG_MTK_ENABLE_GENIEZONE */
#endif /* __KREE_H__ */
