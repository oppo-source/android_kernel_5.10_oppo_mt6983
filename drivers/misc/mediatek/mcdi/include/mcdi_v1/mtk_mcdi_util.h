/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2017 MediaTek Inc.
 */
#ifndef __MTK_MCDI_UTIL_H__
#define __MTK_MCDI_UTIL_H__

#include <mt-plat/sync_write.h>

#define mcdi_read(addr)            __raw_readl((void __force __iomem *)(addr))
#define mcdi_write(addr, val)      mt_reg_sync_writel(val, addr)

#define NF_CMD_BUF          128
#define LOG_BUF_LEN         1024

#define log2buf(p, s, fmt, args...) \
	(p += scnprintf(p, sizeof(s) - strlen(s), fmt, ##args))

#undef mcdi_log
#define mcdi_log(fmt, args...)	log2buf(p, dbg_buf, fmt, ##args)

struct mtk_mcdi_buf {
	char buf[LOG_BUF_LEN];
	char *p_idx;
};

#define reset_mcdi_buf(mcdi) ((mcdi).p_idx = (mcdi).buf)
#define get_mcdi_buf(mcdi)   ((mcdi).buf)
#define mcdi_buf_append(mcdi, fmt, args...)           \
	((mcdi).p_idx += snprintf((mcdi).p_idx,           \
					LOG_BUF_LEN - strlen((mcdi).buf), \
					fmt, ##args))

#define PROC_FOPS_MCDI(name)						\
static int _mcdi_ ## name ## _open(struct seq_file *s, void *data)	\
{									\
	return 0;							\
}									\
static int mcdi_ ## name ## _open(struct inode *inode, struct file *file)\
{									\
	return single_open(file, _mcdi_ ## name ## _open, inode->i_private);\
}									\
static const struct proc_ops mcdi_ ## name ## _fops = {		\
	.proc_open	= mcdi_ ## name ## _open,				\
	.proc_read	= mcdi_ ## name ## _read,				\
	.proc_write	= mcdi_ ## name ## _write,				\
	.proc_lseek	= seq_lseek,						\
	.proc_release = single_release,					\
}
#define PROC_CREATE_MCDI(parent, name)					\
do {									\
	if (!proc_create(#name, 0644, parent, &mcdi_ ## name ## _fops))	\
		pr_notice("%s(), create /proc/mcdi/%s failed\n",	\
			__func__, #name);				\
} while (0)

unsigned int mcdi_mbox_read(int id);
void mcdi_mbox_write(int id, unsigned int val);
int mcdi_fw_is_ready(void);

bool mcdi_is_cpc_mode(void);
unsigned int mcdi_get_cluster_off_cnt(unsigned int cluster);
void mcdi_notify_cluster_off(unsigned int cluster);
unsigned int mcdi_get_raw_pwr_sta(void);

void mcdi_set_cpu_iso_smc(unsigned int iso_mask);
void mcdi_set_cpu_iso_mbox(unsigned int iso_mask);
unsigned long long idle_get_current_time_us(void);

#endif /* __MTK_MCDI_UTIL_H__ */
