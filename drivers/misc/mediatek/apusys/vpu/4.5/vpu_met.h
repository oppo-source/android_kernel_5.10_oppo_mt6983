/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#ifndef __VPU_MET_H__
#define __VPU_MET_H__

#include <linux/platform_device.h>

struct vpu_device;

/* met switch at /sys/kernel/debug/vpu/met */
#define VPU_MET_DISABLED 0x0
#define VPU_MET_LEGACY   0x1
#define VPU_MET_COMPACT  0x2

#if IS_ENABLED(CONFIG_MTK_APUSYS_VPU_DEBUG)
int vpu_init_drv_met(void);
int vpu_exit_drv_met(void);
int vpu_init_dev_met(struct platform_device *pdev,
	struct vpu_device *vd);
void vpu_exit_dev_met(struct platform_device *pdev,
	struct vpu_device *vd);
int vpu_met_set_ftrace(struct vpu_device *vd);
void vpu_met_isr(struct vpu_device *vd);
void vpu_met_pm_get(struct vpu_device *vd);
void vpu_met_pm_put(struct vpu_device *vd);
#else
static inline int vpu_init_drv_met(void)
{
	return 0;
}
static inline int vpu_exit_drv_met(void)
{
	return 0;
}
static inline
int vpu_init_dev_met(struct platform_device *pdev,
	struct vpu_device *vd)
{
	return 0;
}
static inline
void vpu_exit_dev_met(struct platform_device *pdev,
	struct vpu_device *vd)
{
}
static inline
int vpu_met_set_ftrace(struct vpu_device *vd)
{
	return 0;
}
static inline
void vpu_met_isr(struct vpu_device *vd)
{
}
static inline
void vpu_met_pm_get(struct vpu_device *vd)
{
}
static inline
void vpu_met_pm_put(struct vpu_device *vd)
{
}
#endif

struct vpu_met_work {
	struct list_head list;
	spinlock_t lock;
	int pid;
	struct work_struct work;
};

#define VPU_MET_PM_MAX 8

struct vpu_met_pm {
	uint32_t val[VPU_MET_PM_MAX];
};

#define PERF_PMG_ENABLE                 0x00000001
/* global enable bit */
#define PERF_PMG_ENABLE_SHIFT           0

#define PERF_PMCTRL_INT_ENABLE          0x00000001
/* assert interrupt on overflow */

#define PERF_PMCTRL_INT_ENABLE_SHIFT    0
#define PERF_PMCTRL_KRNLCNT             0x00000008
/* ignore TRACELEVEL */
#define PERF_PMCTRL_KRNLCNT_SHIFT       3
#define PERF_PMCTRL_TRACELEVEL          0x000000F0
/* count when CINTLEVEL <= TRACELEVEL */
#define PERF_PMCTRL_TRACELEVEL_SHIFT    4
#define PERF_PMCTRL_SELECT              0x00001F00
/* events group selector */
#define PERF_PMCTRL_SELECT_SHIFT        8
#define PERF_PMCTRL_MASK                0xFFFF0000
/* events mask */
#define PERF_PMCTRL_MASK_SHIFT          16

#define PERF_PMSTAT_OVERFLOW            0x00000001
/* counter overflowed */
#define PERF_PMSTAT_OVERFLOW_SHIFT      0
#define PERF_PMSTAT_INT                 0x00000010
/* interrupt asserted */
#define PERF_PMSTAT_INT_SHIFT           4

#define XTPERF_CNT_CYCLES         0
/* Count cycles */
#define XTPERF_CNT_OVERFLOW       1
/* Overflow of counter n-1 (assuming this is counter n) */
#define XTPERF_CNT_INSN           2
/* Successfully completed instructions */
#define XTPERF_CNT_D_STALL        3
/* Data-related GlobalStall cycles */
#define XTPERF_CNT_I_STALL        4
/* Instruction-related and other GlobalStall cycles */
#define XTPERF_CNT_EXR            5
/* Exceptions and pipeline replays */
#define XTPERF_CNT_BUBBLES        6
/* Hold and other bubble cycles */
#define XTPERF_CNT_I_TLB          7
/* Instruction TLB Accesses (per instruction retiring) */
#define XTPERF_CNT_I_MEM          8
/* Instruction memory accesses (per instruction retiring) */
#define XTPERF_CNT_D_TLB          9
/* Data TLB accesses */
#define XTPERF_CNT_D_LOAD_U1      10
/* Data memory load instruction (load-store unit 1) */
#define XTPERF_CNT_D_STORE_U1     11
/* Data memory store instruction (load-store unit 1) */
#define XTPERF_CNT_D_ACCESS_U1    12
/* Data memory accesses (load, store, S32C1I, etc; load-store unit 1) */
#define XTPERF_CNT_D_LOAD_U2      13
/* Data memory load instruction (load-store unit 2) */
#define XTPERF_CNT_D_STORE_U2     14
/* Data memory store instruction (load-store unit 2) */
#define XTPERF_CNT_D_ACCESS_U2    15
/* Data memory accesses (load, store, S32C1I, etc; load-store unit 2) */
#define XTPERF_CNT_D_LOAD_U3      16
/* Data memory load instruction (load-store unit 3) */
#define XTPERF_CNT_D_STORE_U3     17
/* Data memory store instruction (load-store unit 3) */
#define XTPERF_CNT_D_ACCESS_U3    18
/* Data memory accesses (load, store, S32C1I, etc; load-store unit 3) */
#define XTPERF_CNT_MULTIPLE_LS    22
/* Multiple Load/Store */
#define XTPERF_CNT_OUTBOUND_PIF   23
/* Outbound PIF transactions */
#define XTPERF_CNT_INBOUND_PIF    24
/* Inbound PIF transactions */
#define XTPERF_CNT_PREFETCH       26
/* Prefetch events */
#define XTPERF_CNT_IDMA           27
/* iDMA */


/*
 * Masks for each of the selector listed above
 */

/* XTPERF_CNT_COMMITTED_INSN selector mask */

#define XTPERF_MASK_COMMITTED_INSN               0x0001

/* XTPERF_CNT_BRANCH_PENALTY selector mask */

#define XTPERF_MASK_BRANCH_PENALTY               0x0001

/* XTPERF_CNT_PIPELINE_INTERLOCKS selector mask */

#define XTPERF_MASK_PIPELINE_INTERLOCKS          0x0001

/* XTPERF_CNT_ICACHE_MISSES selector mask */

#define XTPERF_MASK_ICACHE_MISSES                0x0001

/* XTPERF_CNT_DCACHE_MISSES selector mask */

#define XTPERF_MASK_DCACHE_MISSES                0x0001

/* XTPERF_CNT_CYCLES selector mask */

#define XTPERF_MASK_CYCLES                       0x0001

/* XTPERF_CNT_OVERFLOW selector mask */

#define XTPERF_MASK_OVERFLOW                     0x0001

/*
 * XTPERF_CNT_INSN selector mask
 */

#define XTPERF_MASK_INSN_ALL                     0x8DFF

#define XTPERF_MASK_INSN_JX                      0x0001
/* JX */
#define XTPERF_MASK_INSN_CALLX                   0x0002
/* CALLXn */
#define XTPERF_MASK_INSN_RET                     0x0004
/* call return i.e. RET, RETW */
#define XTPERF_MASK_INSN_RF                      0x0008
/* supervisor return i.e. RFDE, RFE, RFI, RFWO, RFWU */
#define XTPERF_MASK_INSN_BRANCH_TAKEN            0x0010
/* Conditional branch taken, or loopgtz/loopnez skips loop */
#define XTPERF_MASK_INSN_J                       0x0020
/* J */
#define XTPERF_MASK_INSN_CALL                    0x0040
/* CALLn */
#define XTPERF_MASK_INSN_BRANCH_NOT_TAKEN        0x0080
/* Conditional branch fall through (aka. not-taken branch) */
#define XTPERF_MASK_INSN_LOOP_TAKEN              0x0100
/* Loop instr falls into loop (aka. taken loop) */
#define XTPERF_MASK_INSN_LOOP_BEG                0x0400
/* Loopback taken to LBEG */
#define XTPERF_MASK_INSN_LOOP_END                0x0800
/* Loopback falls through to LEND */
#define XTPERF_MASK_INSN_NON_BRANCH              0x8000
/* Non-branch instruction (aka. non-CTI) */

/*
 * XTPERF_CNT_D_STALL selector mask
 */

#define XTPERF_MASK_D_STALL_ALL                  0x01FE

#define XTPERF_MASK_D_STALL_STORE_BUF_FULL       0x0002
/* Store buffer full stall */
#define XTPERF_MASK_D_STALL_STORE_BUF_CONFLICT   0x0004
/* Store buffer conflict stall */
#define XTPERF_MASK_D_STALL_CACHE_MISS           0x0008
/* DCache-miss stall */
#define XTPERF_MASK_D_STALL_BUSY                 0x0010
/* Data RAM/ROM/XLMI busy stall */
#define XTPERF_MASK_D_STALL_IN_PIF               0x0020
/* Data inbound-PIF request stall (incl s32c1i) */
#define XTPERF_MASK_D_STALL_MHT_LOOKUP           0x0040
/* MHT lookup stall */
#define XTPERF_MASK_D_STALL_UNCACHED_LOAD        0x0080
/* Uncached load stall (included in MHT lookup stall) */
#define XTPERF_MASK_D_STALL_BANK_CONFLICT        0x0100
/* Bank-conflict stall */

/*
 * XTPERF_CNT_I_STALL selector mask
 */

#define XTPERF_MASK_I_STALL_ALL                  0x01FF

#define XTPERF_MASK_I_STALL_CACHE_MISS           0x0001
/* ICache-miss stall */
#define XTPERF_MASK_I_STALL_BUSY                 0x0002
/* Instruction RAM/ROM busy stall */
#define XTPERF_MASK_I_STALL_IN_PIF               0x0004
/* Instruction RAM inbound-PIF request stall */
#define XTPERF_MASK_I_STALL_TIE_PORT             0x0008
/* TIE port stall */
#define XTPERF_MASK_I_STALL_EXTERNAL_SIGNAL      0x0010
/* External RunStall signal status */
#define XTPERF_MASK_I_STALL_UNCACHED_FETCH       0x0020
/* Uncached fetch stall */
#define XTPERF_MASK_I_STALL_FAST_L32R            0x0040
/* FastL32R stall */
#define XTPERF_MASK_I_STALL_ITERATIVE_MUL        0x0080
/* Iterative multiply stall */
#define XTPERF_MASK_I_STALL_ITERATIVE_DIV        0x0100
/* Iterative divide stall */

/*
 * XTPERF_CNT_EXR selector mask
 */

#define XTPERF_MASK_EXR_ALL                      0x01FF

#define XTPERF_MASK_EXR_REPLAYS                  0x0001
/* Other Pipeline Replay (i.e. excludes $ miss etc.) */
#define XTPERF_MASK_EXR_LEVEL1_INT               0x0002
/* Level-1 interrupt */
#define XTPERF_MASK_EXR_LEVELH_INT               0x0004
/* Greater-than-level-1 interrupt */
#define XTPERF_MASK_EXR_DEBUG                    0x0008
/* Debug exception */
#define XTPERF_MASK_EXR_NMI                      0x0010
/* NMI */
#define XTPERF_MASK_EXR_WINDOW                   0x0020
/* Window exception */
#define XTPERF_MASK_EXR_ALLOCA                   0x0040
/* Alloca exception */
#define XTPERF_MASK_EXR_OTHER                    0x0080
/* Other exceptions */
#define XTPERF_MASK_EXR_MEM_ERR                  0x0100
/* HW-corrected memory error */

/*
 * XTPERF_CNT_BUBBLES selector mask
 */

#define XTPERF_MASK_BUBBLES_ALL                   0x01FD

#define XTPERF_MASK_BUBBLES_PSO                   0x0001
/* Processor domain PSO bubble */
#define XTPERF_MASK_BUBBLES_R_HOLD_D_CACHE_MISS   0x0004
/* R hold caused by DCache miss */
#define XTPERF_MASK_BUBBLES_R_HOLD_STORE_RELEASE  0x0008
/* R hold caused by Store release */
#define XTPERF_MASK_BUBBLES_R_HOLD_REG_DEP        0x0010
/* R hold caused by register dependency */
#define XTPERF_MASK_BUBBLES_R_HOLD_WAIT           0x0020
/* R hold caused by MEMW, EXTW or EXCW */
#define XTPERF_MASK_BUBBLES_R_HOLD_HALT           0x0040
/* R hold caused by Halt instruction (TX only) */
#define XTPERF_MASK_BUBBLES_CTI                   0x0080
/* CTI bubble (e.g. branch delay slot) */
#define XTPERF_MASK_BUBBLES_WAITI                 0x0100
/* WAITI bubble */

/*
 * XTPERF_CNT_I_TLB selector mask
 */

#define XTPERF_MASK_I_TLB_ALL                     0x000F

#define XTPERF_MASK_I_TLB_HITS                    0x0001
/* Hit */
#define XTPERF_MASK_I_TLB_REPLAYS                 0x0002
/* Replay of instruction due to ITLB miss */
#define XTPERF_MASK_I_TLB_REFILLS                 0x0004
/* HW-assisted TLB Refill completes */
#define XTPERF_MASK_I_TLB_MISSES                  0x0008
/* ITLB Miss Exception */

/*
 * XTPERF_CNT_I_MEM selector mask
 */

#define XTPERF_MASK_I_MEM_ALL                     0x000F

#define XTPERF_MASK_I_MEM_CACHE_HITS              0x0001
/* ICache Hit */
#define XTPERF_MASK_I_MEM_CACHE_MISSES            0x0002
/* ICache Miss (includes uncached) */
#define XTPERF_MASK_I_MEM_IRAM                    0x0004
/* InstRAM or InstROM */
#define XTPERF_MASK_I_MEM_BYPASS                  0x0008
/* Bypass (i.e. uncached) fetch */

/*
 * XTPERF_CNT_D_TLB selector mask
 */

#define XTPERF_MASK_D_TLB_ALL                     0x000F

#define XTPERF_MASK_D_TLB_HITS                    0x0001
/* Hit */
#define XTPERF_MASK_D_TLB_REPLAYS                 0x0002
/* Replay of instruction due to DTLB miss */
#define XTPERF_MASK_D_TLB_REFILLS                 0x0004
/* HW-assisted TLB Refill completes */
#define XTPERF_MASK_D_TLB_MISSES                  0x0008
/* DTLB Miss Exception */

/*
 * XTPERF_CNT_D_LOAD_U* selector mask
 */

#define XTPERF_MASK_D_LOAD_ALL                    0x000F

#define XTPERF_MASK_D_LOAD_CACHE_HITS             0x0001
/* Cache Hit */
#define XTPERF_MASK_D_LOAD_CACHE_MISSES           0x0002
/* Cache Miss */
#define XTPERF_MASK_D_LOAD_LOCAL_MEM              0x0004
/* Local memory hit */
#define XTPERF_MASK_D_LOAD_BYPASS                 0x0008
/* Bypass (i.e. uncached) load */

/*
 * XTPERF_CNT_D_STORE_U* selector mask
 */

#define XTPERF_MASK_D_STORE_ALL                    0x000F

#define XTPERF_MASK_D_STORE_CACHE_HITS             0x0001
/* DCache Hit */
#define XTPERF_MASK_D_STORE_CACHE_MISSES           0x0002
/* DCache Miss */
#define XTPERF_MASK_D_STORE_LOCAL_MEM              0x0004
/* Local memory hit */
#define XTPERF_MASK_D_STORE_PIF                    0x0008
/* PIF Store */

/*
 * XTPERF_CNT_D_ACCESS_U* selector mask
 */

#define XTPERF_MASK_D_ACCESS_ALL                   0x000F

#define XTPERF_MASK_D_ACCESS_CACHE_MISSES          0x0001
/* DCache Miss */
#define XTPERF_MASK_D_ACCESS_HITS_SHARED           0x0002
/* Hit Shared */
#define XTPERF_MASK_D_ACCESS_HITS_EXCLUSIVE        0x0004
/* Hit Exclusive */
#define XTPERF_MASK_D_ACCESS_HITS_MODIFIED         0x0008
/* Hit Modified */

/*
 * XTPERF_CNT_MULTIPLE_LS selector mask
 */

#define XTPERF_MASK_MULTIPLE_LS_ALL                0x003F

#define XTPERF_MASK_MULTIPLE_LS_0S_0L              0x0001
/* 0 stores and 0 loads */
#define XTPERF_MASK_MULTIPLE_LS_0S_1L              0x0002
/* 0 stores and 1 loads */
#define XTPERF_MASK_MULTIPLE_LS_1S_0L              0x0004
/* 1 stores and 0 loads */
#define XTPERF_MASK_MULTIPLE_LS_1S_1L              0x0008
/* 1 stores and 1 loads */
#define XTPERF_MASK_MULTIPLE_LS_0S_2L              0x0010
/* 0 stores and 2 loads */
#define XTPERF_MASK_MULTIPLE_LS_2S_0L              0x0020
/* 2 stores and 0 loads */

/*
 * XTPERF_CNT_OUTBOUND_PIF selector mask
 */

#define XTPERF_MASK_OUTBOUND_PIF_ALL               0x0003

#define XTPERF_MASK_OUTBOUND_PIF_CASTOUT           0x0001
/* Castout */
#define XTPERF_MASK_OUTBOUND_PIF_PREFETCH          0x0002
/* Prefetch */

/*
 * XTPERF_CNT_INBOUND_PIF selector mask
 */

#define XTPERF_MASK_INBOUND_PIF_ALL                0x0003

#define XTPERF_MASK_INBOUND_PIF_I_DMA              0x0001
/* Instruction DMA */
#define XTPERF_MASK_INBOUND_PIF_D_DMA              0x0002
/* Data DMA */

/*
 * XTPERF_CNT_PREFETCH selector mask
 */

#define XTPERF_MASK_PREFETCH_ALL                   0x002F

#define XTPERF_MASK_PREFETCH_I_HIT                 0x0001
/* I prefetch-buffer-lookup hit */
#define XTPERF_MASK_PREFETCH_D_HIT                 0x0002
/* D prefetch-buffer-lookup hit */
#define XTPERF_MASK_PREFETCH_I_MISS                0x0004
/* I prefetch-buffer-lookup miss */
#define XTPERF_MASK_PREFETCH_D_MISS                0x0008
/* D prefetch-buffer-lookup miss */
#define XTPERF_MASK_PREFETCH_D_L1_FILL             0x0020
/* Fill directly to DCache L1 */

/*
 * XTPERF_CNT_IDMA selector mask
 */

#define XTPERF_MASK_IDMA_ALL                       0x0001

#define XTPERF_MASK_IDMA_ACTIVE_CYCLES             0x0001
/* Active Cycles */

/*
 * XTPERF_CNT_INSN_LENGTH selector mask
 */

#define XTPERF_MASK_INSN_LENGTH_ALL                0x7FFF

#define XTPERF_MASK_INSN_LENGTH_16                 0x0001
/* 16-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_24                 0x0002
/* 24-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_32                 0x0004
/* 32-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_40                 0x0008
/* 40-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_48                 0x0010
/* 48-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_56                 0x0020
/* 56-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_64                 0x0040
/* 64-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_72                 0x0080
/* 72-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_80                 0x0100
/* 80-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_88                 0x0200
/* 88-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_96                 0x0400
/* 96-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_104                0x0800
/* 104-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_112                0x1000
/* 112-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_120                0x2000
/* 120-bit instruction length */
#define XTPERF_MASK_INSN_LENGTH_128                0x4000
/* 128-bit instruction length */

#endif
