/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MDLA_IOCTL__
#define __MDLA_IOCTL__

#include <stdbool.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#ifndef __KERNEL__
#include <stdint.h>
#endif

/* Memory type for mdla_buf_alloc */
enum mem_type {
	MEM_DRAM,
	MEM_IOMMU,
	MEM_GSM
};

enum MDLA_PMU_INTERFACE {
	MDLA_PMU_IF_WDEC0 = 0xe,
	MDLA_PMU_IF_WDEC1 = 0xf,
	MDLA_PMU_IF_CBLD0 = 0x10,
	MDLA_PMU_IF_CBLD1 = 0x11,
	MDLA_PMU_IF_SBLD0 = 0x12,
	MDLA_PMU_IF_SBLD1 = 0x13,
	MDLA_PMU_IF_STE0 = 0x14,
	MDLA_PMU_IF_STE1 = 0x15,
	MDLA_PMU_IF_CMDE = 0x16,
	MDLA_PMU_IF_DDE = 0x17,
	MDLA_PMU_IF_CONV = 0x18,
	MDLA_PMU_IF_RQU = 0x19,
	MDLA_PMU_IF_POOLING = 0x1a,
	MDLA_PMU_IF_EWE = 0x1b,
	MDLA_PMU_IF_CFLD = 0x1c
};

enum MDLA_PMU_DDE_EVENT {
	MDLA_PMU_DDE_WORK_CYC = 0x0,
	MDLA_PMU_DDE_TILE_DONE_CNT,
	MDLA_PMU_DDE_EFF_WORK_CYC,
	MDLA_PMU_DDE_BLOCK_CNT,
	MDLA_PMU_DDE_READ_CB_WT_CNT,
	MDLA_PMU_DDE_READ_CB_ACT_CNT,
	MDLA_PMU_DDE_WAIT_CB_TOKEN_CNT,
	MDLA_PMU_DDE_WAIT_CONV_RDY_CNT,
	MDLA_PMU_DDE_WAIT_CB_FCWT_CNT,
};

enum MDLA_PMU_MODE {
	MDLA_PMU_ACC_MODE = 0x0,
	MDLA_PMU_INTERVAL_MODE = 0x1,
};

#define MDLA_IOC_MAGIC (0x3d1a632fULL)

struct ioctl_malloc {
	uint32_t size;  /* [in] allocate size */
	uint32_t mva;   /* [out] modified virtual address */
	void *pa;       /* [out] physical address */
	void *kva;      /* [out] kernel virtual address */
	uint8_t type;   /* [in] allocate memory type */
	void *data;     /* [out] userspace virtual address */
};

struct ioctl_run_cmd {
	struct {
		uint32_t size;
		uint32_t mva;
		void *pa;
		void *kva;
		uint32_t id;
		uint8_t type;
		void *data;

		int32_t ion_share_fd;
		int32_t ion_handle;   /* user space handle */
		uint64_t ion_khandle; /* kernel space handle */
	} buf;

	uint32_t offset;        /* [in] command byte offset in buf */
	uint32_t count;         /* [in] # of commands */
	uint32_t id;            /* [out] command id */
	uint8_t priority;       /* [in] dvfs priority */
	uint8_t boost_value;    /* [in] dvfs boost value */
};

enum MDLA_CMD_RESULT {
	MDLA_CMD_SUCCESS = 0,
	MDLA_CMD_TIMEOUT = 1,
};

#define MDLA_IOC_SET_ARRAY_CNT(n) \
	((MDLA_IOC_MAGIC << 32) | ((n) & 0xFFFFFFFF))
#define MDLA_IOC_GET_ARRAY_CNT(n) \
	(((n >> 32) == MDLA_IOC_MAGIC) ? ((n) & 0xFFFFFFFF) : 0)
#define MDLA_IOC_SET_ARRAY_PTR(a) \
	((unsigned long)(a))
#define MDLA_IOC_GET_ARRAY_PTR(a) \
	((void *)((unsigned long)(a)))

#define MDLA_WAIT_CMD_ARRAY_SIZE 6

struct ioctl_wait_cmd {
	uint32_t id;           /* [in] command id */
	int32_t result;        /* [out] success(0), timeout(1) */
	uint64_t queue_time;   /* [out] time queued in driver (ns) */
	uint64_t busy_time;    /* [out] mdla execution time (ns) */
	uint32_t bandwidth;    /* [out] mdla bandwidth */
};

struct ioctl_run_cmd_sync {
	struct ioctl_run_cmd req;
	struct ioctl_wait_cmd res;
	uint32_t mdla_id;
};

struct ioctl_perf {
	uint32_t handle;
	uint32_t interface;
	uint32_t event;
	uint32_t counter;
	uint32_t start;
	uint32_t end;
	uint32_t mode;
	uint32_t mdlaid;
};

struct ioctl_ion {
	int32_t fd;        /* [in] user handle, eq. ion_user_handle_t */
	uint64_t mva;      /* [in] phyiscal address */
	uint64_t kva;      /* [in(unmap)/out(map)] kernel virtual address */
	uint64_t khandle;  /* [in(unmap)/out(map)] kernel handle */
	size_t len;        /* [in] memory size */
};

enum MDLA_CONFIG {
	MDLA_CFG_NONE = 0,
	MDLA_CFG_TIMEOUT_GET = 1,
	MDLA_CFG_TIMEOUT_SET = 2,
	MDLA_CFG_FIFO_SZ_GET = 3,
	MDLA_CFG_FIFO_SZ_SET = 4,
	MDLA_CFG_GSM_INFO = 5,
};

struct ioctl_config {
	uint32_t op;
	uint32_t arg_count;
	uint64_t arg[8];
};

struct mdla_power {
	uint8_t boost_value;
	/* align with core index defined in user space header file */
	uint32_t core_id;
};

enum MDLA_OPP_PRIORIYY {
	MDLA_OPP_DEBUG = 0,
	MDLA_OPP_THERMAL = 1,
	MDLA_OPP_POWER_HAL = 2,
	MDLA_OPP_EARA_QOS = 3,
	MDLA_OPP_NORMAL = 4,
	MDLA_OPP_PRIORIYY_NUM
};

struct mdla_lock_power {
	uint32_t core;
	uint8_t max_boost_value;
	uint8_t min_boost_value;
	bool lock;
	enum MDLA_OPP_PRIORIYY priority;
};

#define IOC_MDLA ('\x1d')

#define IOCTL_MALLOC              _IOWR(IOC_MDLA, 0, struct ioctl_malloc)
#define IOCTL_FREE                _IOWR(IOC_MDLA, 1, struct ioctl_malloc)
#define IOCTL_RUN_CMD_SYNC        _IOWR(IOC_MDLA, 2, struct ioctl_run_cmd)
#define IOCTL_RUN_CMD_ASYNC       _IOWR(IOC_MDLA, 3, struct ioctl_run_cmd_sync)
#define IOCTL_WAIT_CMD            _IOWR(IOC_MDLA, 4, struct ioctl_wait_cmd)
#define IOCTL_PERF_SET_EVENT      _IOWR(IOC_MDLA, 5, struct ioctl_perf)
#define IOCTL_PERF_GET_EVENT      _IOWR(IOC_MDLA, 6, struct ioctl_perf)
#define IOCTL_PERF_GET_CNT        _IOWR(IOC_MDLA, 7, struct ioctl_perf)
#define IOCTL_PERF_UNSET_EVENT    _IOWR(IOC_MDLA, 8, struct ioctl_perf)
#define IOCTL_PERF_GET_START      _IOWR(IOC_MDLA, 9, struct ioctl_perf)
#define IOCTL_PERF_GET_END        _IOWR(IOC_MDLA, 10, struct ioctl_perf)
#define IOCTL_PERF_GET_CYCLE      _IOWR(IOC_MDLA, 11, struct ioctl_perf)
#define IOCTL_PERF_RESET_CNT      _IOWR(IOC_MDLA, 12, struct ioctl_perf)
#define IOCTL_PERF_RESET_CYCLE    _IOWR(IOC_MDLA, 13, struct ioctl_perf)
#define IOCTL_PERF_SET_MODE       _IOWR(IOC_MDLA, 14, struct ioctl_perf)
#define IOCTL_ION_KMAP            _IOWR(IOC_MDLA, 15, struct ioctl_ion)
#define IOCTL_ION_KUNMAP          _IOWR(IOC_MDLA, 16, struct ioctl_ion)

/* 17 ~ 63: reserved for DVFS */
#define IOCTL_SET_POWER         _IOW(IOC_MDLA, 17, struct mdla_power)
#define IOCTL_EARA_LOCK_POWER    _IOW(IOC_MDLA, 18, struct mdla_lock_power)
#define IOCTL_POWER_HAL_LOCK_POWER _IOW(IOC_MDLA, 19, struct mdla_lock_power)
#define IOCTL_EARA_UNLOCK_POWER   _IOW(IOC_MDLA, 20, struct mdla_lock_power)
#define IOCTL_POWER_HAL_UNLOCK_POWER _IOW(IOC_MDLA, 21, struct mdla_lock_power)

#define MDLA_DVFS_IOCTL_START IOCTL_SET_POWER
#define MDLA_DVFS_IOCTL_END   IOCTL_POWER_HAL_UNLOCK_POWER

#define IOCTL_CONFIG              _IOWR(IOC_MDLA, 64, struct ioctl_config)

const struct file_operations *mdla_ioctl_get_fops(void);
void mdla_ioctl_register_perf_handle(int (*pmu_ioctl)(struct file *filp,
						unsigned int command,
						unsigned long arg,
						bool need_pwr_on));
void mdla_ioctl_unregister_perf_handle(void);

#endif /* __MDLA_IOCTL__ */

