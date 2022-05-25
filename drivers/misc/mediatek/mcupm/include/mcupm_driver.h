/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2020 MediaTek Inc.
 */
#ifndef __MCUPM_DEFINE_H__
#define __MCUPM_DEFINE_H__

#include <linux/types.h>

/* MCUPM MBOX */
#define MCUPM_MBOX_SLOT_SIZE		0x4

#define MCUPM_PLT_SERV_SUPPORT		(1)
#define MCUPM_LOGGER_SUPPORT		(1)
#define MCUPM_COREDUMP_SUPPORT		(1)
#define MCUPM_ALIVE_THREAD		(0)

#define MCUPM_MBOX_NO_SUSPEND		4

#define MCUPM_MBOX_OFFSET_PDN		10 /* offset: 40 bytes */
#define MCUPM_MBOX_SIZE_PDN		1  /* slot size: 4 bytes */

#define MCUPM_MBOX_OFFSET_STATE		(MCUPM_MBOX_OFFSET_PDN + \
					MCUPM_MBOX_SIZE_PDN)
#define MCUPM_MBOX_SIZE_STATE		1  /* slot size: 4 bytes */

#define MCUPM_MBOX_OFFSET_TIMESTAMP	(MCUPM_MBOX_OFFSET_STATE + \
					MCUPM_MBOX_SIZE_STATE)

#define MCUPM_PLT_INIT			0x504C5401
#define MCUPM_PLT_LOG_ENABLE		0x504C5402
#define MCUPM_PLT_SERV_READY		0x504C5403
#define MCUPM_POWER_DOWN		0x4D50444E

/* MCUPM RESERVED MEM */
#define MCUPM_RESERVED_DEBUG		(1)
#define MCUPM_PLT_LOGGER_BUF_LEN	0x100000

#if !defined(CONFIG_MTK_GMO_RAM_OPTIMIZE) && !defined(CONFIG_MTK_MET_MEM_ALLOC)
#define MCUPM_MET_LOGGER_BUF_LEN	0x400000
#endif
#define MCUPM_PLT_EEMSN_BUF_LEN		0x1000
#define MCUPM_BRISKET_BUF_LEN		0x1000

enum {
	MCUPM_MEM_ID = 0,
#if !defined(CONFIG_MTK_GMO_RAM_OPTIMIZE) && !defined(CONFIG_MTK_MET_MEM_ALLOC)
	MCUPM_MET_ID,
#endif
	MCUPM_EEMSN_MEM_ID,
	MCUPM_BRISKET_ID,
	NUMS_MCUPM_MEM_ID,
};

struct mcupm_reserve_mblock {
	u32 num;
	u64 start_phys;
	u64 start_virt;
	u64 size;
};

/* MCUPM LOGGER */
#ifdef MCUPM_PLT_LOGGER_BUF_LEN
/* use platform-defined buffer length */
#define BUF_LEN				MCUPM_PLT_LOGGER_BUF_LEN
#else
/* otherwise use default buffer length */
#define BUF_LEN				(1 * 1024 * 1024)
#endif
#define LBUF_LEN			(4 * 1024)
#define MCUPM_TIMER_TIMEOUT		(1 * HZ) /* 1 seconds*/
#define ROUNDUP(a, b)			(((a) + ((b)-1)) & ~((b)-1))
#define ENABLE				0x101
#define DISABLE				0x1

struct log_ctrl_s {
	unsigned int base;
	unsigned int size;
	unsigned int enable;
	unsigned int info_ofs;
	unsigned int buff_ofs;
unsigned int buff_size;
};

struct buffer_info_s {
	unsigned int r_pos;
	unsigned int w_pos;
};

/* MCUPM IPI define */
struct mcupm_ipi_data_s {
	unsigned int cmd;
	union {
		struct {
			unsigned int phys;
			unsigned int size;
		} ctrl;
		struct {
			unsigned int enable;
		} logger;
		struct {
			unsigned int mode;
		} ts;
	} u;
};
extern bool has_reserved_memory;
extern bool skip_logger;

extern ssize_t mcupm_log_read(char __user *data, size_t len);
extern unsigned int mcupm_log_poll(void);
extern int mcupm_mbox_write(unsigned int mbox, unsigned int slot, void *buf,
				unsigned int len);
extern int mcupm_mbox_read(unsigned int mbox, unsigned int slot, void *buf,
			unsigned int len);
extern phys_addr_t mcupm_reserve_mem_get_phys(unsigned int id);
extern phys_addr_t mcupm_reserve_mem_get_virt(unsigned int id);
extern phys_addr_t mcupm_reserve_mem_get_size(unsigned int id);
#endif
