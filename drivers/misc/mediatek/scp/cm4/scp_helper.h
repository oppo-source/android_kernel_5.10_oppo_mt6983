/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __SCP_HELPER_H__
#define __SCP_HELPER_H__

#include <linux/notifier.h>
#include <linux/interrupt.h>
#include "scp_reg.h"
#include "scp_feature_define.h"

/* scp config reg. definition*/
#define SCP_TCM_SIZE		(scpreg.total_tcmsize)
#define SCP_A_TCM_SIZE		(scpreg.scp_tcmsize)
#define SCP_TCM			(scpreg.sram)	/* virtual address */
#define SCP_REGION_INFO_OFFSET	0x400
#define SCP_RTOS_START		0x800
#define SCP_A_SHARE_BUFFER	(scpreg.sram + \
				SCP_RTOS_START - SHARE_BUF_SIZE * 2)
#define SCP_TCM_PHY_ADDR        (0x10500000)	/* AP view: physical address */

/* scp dvfs return status flag */
#define SET_PLL_FAIL		(1)
#define SET_PMIC_VOLT_FAIL	(2)

/* This structre need to sync with SCP-side */
struct SCP_IRQ_AST_INFO {
	unsigned int scp_irq_ast_time;
	unsigned int scp_irq_ast_pc_s;
	unsigned int scp_irq_ast_pc_e;
	unsigned int scp_irq_ast_lr_s;
	unsigned int scp_irq_ast_lr_e;
	unsigned int scp_irq_ast_irqd;
};

/* reset ID */
#define SCP_ALL_ENABLE		0x00
#define SCP_ALL_REBOOT		0x01
#define SCP_A_ENABLE		0x10
#define SCP_A_REBOOT		0x11


/* scp reset status */
enum SCP_RESET_STATUS {
	RESET_STATUS_STOP = 0,
	RESET_STATUS_START = 1,
};

/* scp reset status */
enum SCP_RESET_TYPE {
	RESET_TYPE_WDT = 0,
	RESET_TYPE_AWAKE = 1,
	RESET_TYPE_CMD = 2,
};

struct scp_regs {
	void __iomem *scpsys;
	void __iomem *sram;
	void __iomem *cfg;
	void __iomem *clkctrl;
	void __iomem *l1cctrl;
	int irq;
	unsigned int total_tcmsize;
	unsigned int cfgregsize;
	unsigned int scp_tcmsize;
};

/* scp work struct definition*/
struct scp_work_struct {
	struct work_struct work;
	unsigned int flags;
	unsigned int id;
};

struct scp_reserve_mblock {
	enum scp_reserve_mem_id_t num;
	u64 start_phys;
	u64 start_virt;
	u64 size;
};

struct scp_region_info_st {
	uint32_t ap_loader_start;
	uint32_t ap_loader_size;
	uint32_t ap_firmware_start;
	uint32_t ap_firmware_size;
	uint32_t ap_dram_start;
	uint32_t ap_dram_size;
	uint32_t ap_dram_backup_start;
	/*	This is the size of the structure.
	 *	It can act as a version number if entries can only be
	 *	added to (not deleted from) the structure.
	 *	It should be the first entry of the structure, but for
	 *	compatibility reason, it is appended here.
	 */
	uint32_t struct_size;
	uint32_t scp_log_thru_ap_uart;
	uint32_t TaskContext_ptr;
	uint32_t Il1c_con;
	uint32_t Dl1c_con;
	uint32_t scpctl;
	uint32_t ap_params_start;
};

/* scp device attribute */
extern struct device_attribute dev_attr_scp_A_mobile_log_UT;
extern struct device_attribute dev_attr_scp_A_logger_wakeup_AP;
extern const struct file_operations scp_A_log_file_ops;

extern struct scp_regs scpreg;
extern struct device_attribute dev_attr_scp_mobile_log;
extern struct device_attribute dev_attr_scp_A_get_last_log;
extern struct device_attribute dev_attr_scp_A_status;
extern struct bin_attribute bin_attr_scp_dump;

/* scp loggger */
int scp_logger_init(phys_addr_t start, phys_addr_t limit);
void scp_logger_uninit(void);


/* scp exception */
extern int scp_excep_init(void);
extern void scp_ram_dump_init(void);
extern void scp_excep_cleanup(void);
extern void scp_aee_last_reg(void);

/* scp irq */
extern irqreturn_t scp_A_irq_handler(int irq, void *dev_id);
extern void scp_A_irq_init(void);
extern void scp_A_ipi_init(void);

/* scp helper */
extern void scp_schedule_work(struct scp_work_struct *scp_ws);
extern void scp_schedule_logger_work(struct scp_work_struct *scp_ws);

extern void memcpy_to_scp(void __iomem *trg,
		const void *src, int size);
extern void memcpy_from_scp(void *trg, const void __iomem *src,
		int size);
extern int reset_scp(int reset);

extern int scp_check_resource(void);
void set_scp_mpu(void);
extern phys_addr_t scp_mem_base_phys;
extern void __iomem *scp_mem_base_virt;
extern phys_addr_t scp_mem_size;
extern atomic_t scp_reset_status;

/*extern scp notify*/
extern void scp_send_reset_wq(enum SCP_RESET_TYPE type);
extern void scp_extern_notify(enum SCP_NOTIFY_EVENT notify_status);
extern struct completion scp_sys_reset_cp;
extern void scp_status_set(unsigned int value);
extern void scp_logger_init_set(unsigned int value);
extern unsigned int scp_set_reset_status(void);
extern void scp_enable_sram(void);
extern int scp_sys_full_reset(void);
extern void scp_reset_awake_counts(void);
extern void scp_awake_init(void);
#if SCP_RECOVERY_SUPPORT
extern phys_addr_t scp_loader_base_virt;
extern unsigned int scp_reset_by_cmd;
extern struct scp_region_info_st scp_region_info_copy;
extern struct scp_region_info_st *scp_region_info;
extern void __iomem *scp_l1c_start_virt;
#endif

__attribute__((weak))
int sensor_params_to_scp(void *addr_vir, size_t size);

#endif

