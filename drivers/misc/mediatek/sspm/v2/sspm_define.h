/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */
#ifndef __SSPM_DEFINE_H__
#define __SSPM_DEFINE_H__

#include <linux/param.h> /* for HZ */
#include <asm/arch_timer.h>

#define SSPM_MBOX_MAX		5
#define SSPM_MBOX_IN_IRQ_OFS	0x0
#define SSPM_MBOX_OUT_IRQ_OFS	0x4
#define SSPM_MBOX_SLOT_SIZE	0x4

#define SSPM_CFG_OFS_SEMA	0x048
#define SSPM_MPU_REGION_ID	5

#define SSPM_SHARE_BUFFER_SUPPORT

#define SSPM_PLT_SERV_SUPPORT       (1)
#define SSPM_LOGGER_SUPPORT         (1)
#define SSPM_TIMESYNC_SUPPORT       (1)
#define SSPM_EMI_PROTECTION_SUPPORT (0)

#define PLT_INIT		0x504C5401
#define PLT_LOG_ENABLE		0x504C5402
/* Legacy magic code define
 * Should avoid to use duplicate definitions
#define PLT_LASTK_READY	0x504C5403
#define PLT_COREDUMP_READY	0x504C5404
#define PLT_TIMESYNC_SYNC	0x504C5405
*/
#define PLT_TIMESYNC_SRAM_TEST	0x504C5406


struct plt_ipi_data_s {
	unsigned int cmd;
	union {
		struct {
			unsigned int phys;
			unsigned int size;
		} ctrl;
		struct {
			unsigned int enable;
		} logger;
	} u;
};

#endif
