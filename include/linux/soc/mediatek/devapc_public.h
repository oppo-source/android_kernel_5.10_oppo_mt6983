/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

#ifndef __DEVAPC_PUBLIC_H__
#define __DEVAPC_PUBLIC_H__

#include <linux/types.h>

enum infra_subsys_id {
	INFRA_SUBSYS_MD = 0,
	INFRA_SUBSYS_CONN,
	INFRA_SUBSYS_ADSP,
	INFRA_SUBSYS_GCE,
	INFRA_SUBSYS_APMCU,
	INFRA_SUBSYS_GZ,
	DEVAPC_SUBSYS_CLKMGR,
	DEVAPC_SUBSYS_TEST,
	DEVAPC_SUBSYS_RESERVED,
};

enum devapc_cb_status {
	DEVAPC_OK = 0,
	DEVAPC_NOT_KE,
};

struct devapc_vio_callbacks {
	struct list_head list;
	enum infra_subsys_id id;
	void (*debug_dump)(void);
	enum devapc_cb_status (*debug_dump_adv)(uint32_t vio_addr);
};

uint32_t devapc_vio_check(void);
void dump_dbg_info(void);
void register_devapc_vio_callback(struct devapc_vio_callbacks *viocb);
void devapc_catch_illegal_range(phys_addr_t phys_addr, size_t size);
void clkchk_devapc_dump(void);
void cmdq_util_devapc_dump(void);
int mmup_enable_count(void);

bool is_adsp_feature_in_active(void);

void __attribute__((weak)) register_devapc_vio_callback(struct devapc_vio_callbacks *viocb)
{
}

#endif  /* __DEVAPC_PUBLIC_H__ */

