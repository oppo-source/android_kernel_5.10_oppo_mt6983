// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015-2019, MICROTRUST Incorporated
 * All Rights Reserved.
 *
 */

#include "teei_client_transfer_data.h"
#define IMSG_TAG "[tz_driver]"
#include <imsg_log.h>
#include <linux/string.h>
#include <linux/uaccess.h>

int ut_pf_gp_initialize_context(struct TEEC_Context *context)
{
	const char *hostname = "bta_loader";
	TEEC_Result ret  = 0;

	if (context == NULL)
		return -1;

	memset(context, 0, sizeof(struct TEEC_Context));
	ret = TEEC_InitializeContext(hostname, context);
	if (ret != TEEC_SUCCESS)
		IMSG_ERROR("Failed to initialize context,err: %x", ret);

	return ret;
}

int ut_pf_gp_finalize_context(struct TEEC_Context *context)
{
	if (context)
		TEEC_FinalizeContext(context);
	return 0;
}

int ut_pf_gp_transfer_data(struct TEEC_Context *context, struct TEEC_UUID *uuid,
	unsigned int command, void *buffer, unsigned long size)
{
	struct TEEC_Session session;
	struct TEEC_Operation operation;
	struct TEEC_SharedMemory sharedmem;
	TEEC_Result result;
	uint32_t returnOrigin = 0;

	if (NULL == context || NULL == uuid || NULL == buffer || size < 1)
		return -1;

	memset(&session, 0, sizeof(session));
	result = TEEC_OpenSession(context, &session, uuid, TEEC_LOGIN_PUBLIC,
			NULL, NULL, &returnOrigin);
	if (result != TEEC_SUCCESS) {
		IMSG_ERROR("Failed to open session:%x size = %lu\n",
					result, size);
		goto release_1;
	}

	sharedmem.buffer = buffer;
	sharedmem.size = size;
	sharedmem.flags = TEEC_MEM_INPUT | TEEC_MEM_OUTPUT;
	result = TEEC_RegisterSharedMemory(context, &sharedmem);
	if (result != TEEC_SUCCESS) {
		IMSG_ERROR("Failed to register kernel %d shared memory,err: %x",
		(unsigned int)size, result);
		goto release_2;
	}
	memset(&operation, 0x00, sizeof(operation));
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_PARTIAL_INOUT,
				TEEC_NONE, TEEC_NONE, TEEC_NONE);
	operation.started = 1;
	operation.params[0].memref.parent = &sharedmem;
	operation.params[0].memref.offset = 0;
	operation.params[0].memref.size = sharedmem.size;
	result = TEEC_InvokeCommand(&session, command, &operation, NULL);
	if (result != TEEC_SUCCESS) {
		IMSG_ERROR("Failed to invoke command,err: %x", result);
		goto release_3;
	}

release_3:
	TEEC_ReleaseSharedMemory(&sharedmem);
release_2:
	TEEC_CloseSession(&session);
release_1:
	return result;
}


int ut_pf_gp_transfer_user_data(struct TEEC_Context *context,
	struct TEEC_UUID *uuid,
	unsigned int command, void *buffer, unsigned long size)
{
	struct TEEC_Session session;
	struct TEEC_Operation operation;
	struct TEEC_SharedMemory sharedmem;
	TEEC_Result result;
	uint32_t returnOrigin = 0;

	if (NULL == context || NULL == uuid || NULL == buffer || size < 1)
		return -1;

	memset(&session, 0, sizeof(session));
	result = TEEC_OpenSession(context, &session, uuid, TEEC_LOGIN_PUBLIC,
			NULL, NULL, &returnOrigin);
	if (result != TEEC_SUCCESS) {
		IMSG_ERROR("Failed to open session,err: %x", result);
		goto release_1;
	}

	sharedmem.size = size;
	sharedmem.flags = TEEC_MEM_INPUT | TEEC_MEM_OUTPUT;
	result = TEEC_AllocateSharedMemory(context, &sharedmem);
	if (result != TEEC_SUCCESS) {
		IMSG_ERROR("Failed to register user %d shared memory,err: %x",
		(unsigned int)size, result);
		goto release_2;
	}

	if (copy_from_user((void *)sharedmem.buffer, buffer, size)) {
		IMSG_ERROR("Failed to copy_from_user!\n");
		goto release_3;
	}

	memset(&operation, 0x00, sizeof(operation));
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_PARTIAL_INOUT,
				TEEC_NONE, TEEC_NONE, TEEC_NONE);
	operation.started = 1;
	operation.params[0].memref.parent = &sharedmem;
	operation.params[0].memref.offset = 0;
	operation.params[0].memref.size = sharedmem.size;
	result = TEEC_InvokeCommand(&session, command, &operation, NULL);
	if (result != TEEC_SUCCESS) {
		IMSG_ERROR("Failed to invoke command,err: %x", result);
		goto release_3;
	}

	if (copy_to_user(buffer, (void *)sharedmem.buffer, size))
		IMSG_ERROR("Failed to copy_to_user!\n");

release_3:
	TEEC_ReleaseSharedMemory(&sharedmem);
release_2:
	TEEC_CloseSession(&session);
release_1:
	return result;
}
