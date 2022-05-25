/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef TEE_INVOKE_H_
#define TEE_INVOKE_H_

int tee_directly_invoke_cmd(struct trusted_driver_cmd_params *invoke_params);

#if IS_ENABLED(CONFIG_MTK_MTEE_MULTI_CHUNK_SUPPORT)
int secmem_set_mchunks_region(u64 pa, u32 size, int remote_region_type);
#endif

#if IS_ENABLED(CONFIG_MTK_SECURE_MEM_SUPPORT)
int secmem_svp_dump_info(void);
int secmem_dynamic_debug_control(bool enable_dbg);
int secmem_force_hw_protection(void);
#endif

#if IS_ENABLED(CONFIG_TRUSTONIC_TEE_SUPPORT) || \
	IS_ENABLED(CONFIG_MICROTRUST_TEE_SUPPORT)
int secmem_fr_set_svp_region(u64 pa, u32 size, int remote_region_type);
int secmem_fr_set_wfd_region(u64 pa, u32 size, int remote_region_type);
int secmem_fr_set_sapu_data_shm_region(u64 pa, u32 size, int remote_region_type);
int secmem_fr_set_sapu_engine_shm_region(u64 pa, u32 size, int remote_region_type);
int secmem_fr_set_prot_shared_region(u64 pa, u32 size, int remote_region_type);
int secmem_fr_dump_info(void);
#endif

#if IS_ENABLED(CONFIG_MTK_WFD_SMEM_SUPPORT)
int wfd_smem_dump_info(void);
#endif

#endif /* TEE_INVOKE_H_ */
