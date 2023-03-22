/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MTK_SPM_SLEEP_H__
#define __MTK_SPM_SLEEP_H__

#include <linux/kernel.h>
#include <mtk_spm.h>

/*
 * for suspend
 */
extern bool spm_is_md_sleep(void);
extern bool spm_is_md1_sleep(void);
extern bool spm_is_md2_sleep(void);
extern bool spm_is_conn_sleep(void);
extern void spm_ap_mdsrc_req(u8 set);
extern ssize_t get_spm_sleep_count(char *ToUserBuf
			, size_t sz, void *priv);
extern ssize_t get_spm_last_wakeup_src(char *ToUserBuf
			, size_t sz, void *priv);
extern ssize_t get_spm_last_debug_flag(char *ToUserBuf
			, size_t sz, void *priv);
extern void spm_adsp_mem_protect(void);
extern void spm_adsp_mem_unprotect(void);

/* record last wakesta */
extern u32 spm_get_last_wakeup_src(void);
extern u32 spm_get_last_wakeup_misc(void);
#endif /* __MTK_SPM_SLEEP_H__ */
