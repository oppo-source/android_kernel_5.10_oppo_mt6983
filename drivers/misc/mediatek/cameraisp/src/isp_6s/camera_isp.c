// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

/******************************************************************************
 * camera_isp.c - MT6769 Linux ISP Device Driver
 *
 * DESCRIPTION:
 *     This file provid the other drivers ISP relative functions
 *
 ******************************************************************************/
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>  /*for kernel log reduction */
#include <linux/proc_fs.h> /* proc file use */
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/suspend.h>

/* MET: define to enable MET */
/* GKI: not support MET yet */
// #define ISP_MET_READY

/* GKI: no need to adjust upper bound of kernel log count due to no supported related API. */
#define EP_NO_K_LOG_ADJUST

/* Device link is not ready. So, we should use smi api to get smi. */
#define USE_MTK_SMI_LARB_API
#ifdef USE_MTK_SMI_LARB_API
#include <soc/mediatek/smi.h>
#endif

/* #define EP_STAGE */
#ifdef EP_STAGE
#define EP_MARK_SMI /* disable SMI related for EP */
//#define DUMMY_INT   /* For early if load dont need to use camera */

/* Clkmgr is not ready in early porting, en/disable clock by hardcode */
#define EP_NO_CLKMGR

/* EP no need to adjust upper bound of kernel log count */
//#define EP_NO_K_LOG_ADJUST
#endif
#define ENABLE_TIMESYNC_HANDLE /* able/disable TimeSync related for EP */

#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/compat.h>
#include <linux/fs.h>
#endif

#if IS_ENABLED(CONFIG_PM_WAKELOCKS)
#include <linux/pm_wakeup.h>
#else
#include <linux/wakelock.h>
#endif

#ifdef CONFIG_OF
#include <linux/of_address.h>  /* for device tree */
#include <linux/of_irq.h>      /* for device tree */
#include <linux/of_platform.h> /* for device tree */
#endif

#include <linux/pm_runtime.h> /* for mtcmos */

#if defined(ISP_MET_READY)
#define CREATE_TRACE_POINTS
#include "inc/met_events_camsys.h"
#endif

#ifndef EP_MARK_SMI
// #include <smi_public.h>
/*for SMI BW debug log*/
/* #include <smi_debug.h> */
#endif

#include "inc/cam_qos.h"
#include "inc/camera_isp.h"
#include "inc/cam_common.h"

#ifdef ENABLE_TIMESYNC_HANDLE
#include <archcounter_timesync.h>
#endif

/*  */
#ifndef MTRUE
#define MTRUE 1
#endif
#ifndef MFALSE
#define MFALSE 0
#endif

#define CAMERA_SMI_ENABLE 1
#define CAMERA_SMI_DISABLE 0

#define ISP_DEV_NAME "camera-isp"
#define SMI_LARB_MMU_CTL (0)
/*#define ENABLE_WAITIRQ_LOG*/ /* wait irq debug logs */
/*#define ENABLE_STT_IRQ_LOG*/ /*show STT irq debug logs */

#define Lafi_WAM_CQ_ERR (1)
/* Queue timestamp for deque. Update when non-drop frame @SOF */
#define TIMESTAMP_QUEUE_EN (0)
#if (TIMESTAMP_QUEUE_EN == 1)
#define TSTMP_SUBSAMPLE_INTPL (1)
#else
#define TSTMP_SUBSAMPLE_INTPL (0)
#endif
#define ISP_BOTTOMHALF_WORKQ (1)

#if (ISP_BOTTOMHALF_WORKQ == 1)
#include <linux/workqueue.h>
#endif

/* -------------------------------------------------------------------------- */

#define MyTag "[ISP]"
#define IRQTag "KEEPER"

#define LOG_VRB(format, args...)                                               \
	pr_debug(MyTag "[%s] " format, __func__, ##args)

/* #define ISP_DEBUG */
#ifdef ISP_DEBUG
#define LOG_DBG(format, args...) pr_info(MyTag "[%s] " format, __func__, ##args)
#else
#define LOG_DBG(format, args...)
#endif

#define LOG_INF(format, args...) pr_info(MyTag "[%s] " format, __func__, ##args)

#define LOG_NOTICE(format, args...)                                            \
	pr_notice(MyTag "[%s] " format, __func__, ##args)

/******************************************************************************
 *
 *****************************************************************************/
#define ISP_WR32(addr, data) \
do {    \
	writel((data), (void __force __iomem *)((addr))); \
	mb(); /* memory barrier */ \
} while (0)

/* NEED_TUNING_BY_PROJECT */
#define ISP_RD32(addr) ioread32((void *)addr)

/******************************************************************************
 *
 *****************************************************************************/
/* dynamic log level */
#define ISP_DBG_INT (0x00000001)
#define ISP_DBG_READ_REG (0x00000004)
#define ISP_DBG_WRITE_REG (0x00000008)
#define ISP_DBG_CLK (0x00000010)
#define ISP_DBG_TASKLET (0x00000020)
#define ISP_DBG_SCHEDULE_WORK (0x00000040)
#define ISP_DBG_BUF_WRITE (0x00000080)
#define ISP_DBG_BUF_CTRL (0x00000100)
#define ISP_DBG_REF_CNT_CTRL (0x00000200)
#define ISP_DBG_INT_2 (0x00000400)
#define ISP_DBG_INT_3 (0x00000800)
#define ISP_DBG_HW_DON (0x00001000)
#define ISP_DBG_ION_CTRL (0x00002000)
#define ISP_DBG_CAMSV_LOG (0x00004000)

/******************************************************************************
 *
 *****************************************************************************/
static irqreturn_t ISP_Irq_CAM(enum ISP_IRQ_TYPE_ENUM irq_module,
				enum ISP_DEV_NODE_ENUM cam_idx);
static irqreturn_t ISP_Irq_CAM_A(int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_CAM_B(int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_CAM_C(int Irq, void *DeviceId);

static irqreturn_t ISP_Irq_CAMSV(enum ISP_IRQ_TYPE_ENUM irq_module,
				 enum ISP_DEV_NODE_ENUM cam_idx,
				 const char *str);
static irqreturn_t ISP_Irq_CAMSV_0(int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_CAMSV_1(int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_CAMSV_2(int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_CAMSV_3(int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_CAMSV_4(int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_CAMSV_5(int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_CAMSV_6(int Irq, void *DeviceId);
static irqreturn_t ISP_Irq_CAMSV_7(int Irq, void *DeviceId);

typedef irqreturn_t (*IRQ_CB)(int, void *);

struct ISR_TABLE {
	IRQ_CB isr_fp;
	unsigned int int_number;
	char device_name[16];
};

#ifndef CONFIG_OF
const struct ISR_TABLE IRQ_CB_TBL[ISP_IRQ_TYPE_AMOUNT] = {
	{ISP_Irq_CAM_A, CAM0_IRQ_BIT_ID, "CAM_A"},
	{NULL, 0, "CAM_B"},

	{ISP_Irq_CAMSV_0, CAM_SV0_IRQ_BIT_ID, "CAMSV_0"},
	{ISP_Irq_CAMSV_1, CAM_SV1_IRQ_BIT_ID, "CAMSV_1"},
	{NULL, 0, "UNI"} };

#else
/* int number is got from kernel api */

const struct ISR_TABLE IRQ_CB_TBL[ISP_IRQ_TYPE_AMOUNT] = {
/* Must be the same name with that in device node. */
#ifdef DUMMY_INT
	{ISP_Irq_CAM_A, 0, "cam1-dummy"},
	{ISP_Irq_CAM_B, 0, "cam2-dummy"},
	{ISP_Irq_CAM_C, 0, "cam3-dummy"},

	{ISP_Irq_CAMSV_0, 0, "camsv1-dummy"},
	{ISP_Irq_CAMSV_1, 0, "camsv2-dummy"},
	{ISP_Irq_CAMSV_2, 0, "camsv3-dummy"},
	{ISP_Irq_CAMSV_3, 0, "camsv4-dummy"},
	{ISP_Irq_CAMSV_4, 0, "camsv5-dummy"},
	{ISP_Irq_CAMSV_5, 0, "camsv6-dummy"},
	{ISP_Irq_CAMSV_6, 0, "camsv7-dummy"},
	{ISP_Irq_CAMSV_7, 0, "camsv8-dummy"}
#else
	/* device_name definded in IRQ_CB_TBL must be match with the node name
	 * defined in dts.
	 */
	{ISP_Irq_CAM_A, 0, "cam1_legacy"},
	{ISP_Irq_CAM_B, 0, "cam2_legacy"},
	{ISP_Irq_CAM_C, 0, "cam3_legacy"},
	{ISP_Irq_CAMSV_0, 0, "camsv1_legacy"},
	{ISP_Irq_CAMSV_1, 0, "camsv2_legacy"},
	{ISP_Irq_CAMSV_2, 0, "camsv3_legacy"},
	{ISP_Irq_CAMSV_3, 0, "camsv4_legacy"},
	{ISP_Irq_CAMSV_4, 0, "camsv5_legacy"},
	{ISP_Irq_CAMSV_5, 0, "camsv6_legacy"},
	{ISP_Irq_CAMSV_6, 0, "camsv7_legacy"},
	{ISP_Irq_CAMSV_7, 0, "camsv8_legacy"}
#endif
};

static unsigned int g_platform_id;

#ifdef CAM_ISP_DBGFS
#include <linux/debugfs.h>

struct dentry *camisp_dbg_root;
#endif

/*
 * Note!!! The order and member of .compatible must be the same with that in
 *  dts file.
 */
static const struct of_device_id isp_of_ids[] = {
	{
		.compatible = "mediatek,camisp_legacy",
	},
	{
		.compatible = "mediatek,cam1_legacy",
	},
	{
		.compatible = "mediatek,cam1_inner_legacy",
	},
	{
		.compatible = "mediatek,camsys_rawa_legacy",
	},
	{
		.compatible = "mediatek,cam2_legacy",
	},
	{
		.compatible = "mediatek,cam2_inner_legacy",
	},
	{
		.compatible = "mediatek,camsys_rawb_legacy",
	},
	{
		.compatible = "mediatek,cam3_legacy",
	},
	{
		.compatible = "mediatek,cam3_inner_legacy",
	},
	{
		.compatible = "mediatek,camsys_rawc_legacy",
	},

	{
		.compatible = "mediatek,camsv1_legacy",
	},
	{
		.compatible = "mediatek,camsv2_legacy",
	},
	{
		.compatible = "mediatek,camsv3_legacy",
	},
	{
		.compatible = "mediatek,camsv4_legacy",
	},
	{
		.compatible = "mediatek,camsv5_legacy",
	},
	{
		.compatible = "mediatek,camsv6_legacy",
	},
	{
		.compatible = "mediatek,camsv7_legacy",
	},
	{
		.compatible = "mediatek,camsv8_legacy",
	},
	{} };

#endif
/* ////////////////////////////////////////////////////////////////////////// */
/*  */

#if (ISP_BOTTOMHALF_WORKQ == 1)
struct IspWorkqueTable {
	enum ISP_IRQ_TYPE_ENUM module;
	struct work_struct isp_bh_work;
};

static void ISP_BH_Workqueue(struct work_struct *pWork);

static struct IspWorkqueTable isp_workque[ISP_IRQ_TYPE_AMOUNT] = {
	{ISP_IRQ_TYPE_INT_CAM_A_ST},   {ISP_IRQ_TYPE_INT_CAM_B_ST},
	{ISP_IRQ_TYPE_INT_CAM_C_ST},
	{ISP_IRQ_TYPE_INT_CAMSV_0_ST}, {ISP_IRQ_TYPE_INT_CAMSV_1_ST},
	{ISP_IRQ_TYPE_INT_CAMSV_2_ST}, {ISP_IRQ_TYPE_INT_CAMSV_3_ST},
	{ISP_IRQ_TYPE_INT_CAMSV_4_ST}, {ISP_IRQ_TYPE_INT_CAMSV_5_ST},
	{ISP_IRQ_TYPE_INT_CAMSV_6_ST}, {ISP_IRQ_TYPE_INT_CAMSV_7_ST},
};

/* isr log workqueue. */
struct workqueue_struct *isr_log_wq;
#endif

#ifdef CONFIG_OF

#include <linux/clk.h>
struct ISP_CLK_STRUCT {
	struct clk *ISP_CAM_CAMSYS;
	struct clk *ISP_CAM_CAMTG;
	struct clk *ISP_CAM_CAMSV0;
	struct clk *ISP_CAM_CAMSV1;
	struct clk *ISP_CAM_CAMSV2;

	struct clk *ISP_CAM_CAMSV3;

	struct clk *CAMSYS_SENINF_CGPDN;

	struct clk *CAMSYS_CAM2MM_GALS_CGPDN;

	struct clk *CAMSYS_TOP_MUX_CCU;
	struct clk *CAMSYS_CCU0_CGPDN;
	struct clk *CAMSYS_LARB13_CGPDN;
	struct clk *CAMSYS_LARB14_CGPDN;

	struct clk *CAMSYS_LARB15_CGPDN;

	struct clk *ISP_CAM_LARB16_RAWA;
	struct clk *ISP_CAM_SUBSYS_RAWA;
	struct clk *ISP_CAM_TG_RAWA;
	struct clk *ISP_CAM_LARB17_RAWB;
	struct clk *ISP_CAM_SUBSYS_RAWB;
	struct clk *ISP_CAM_TG_RAWB;

	struct clk *ISP_CAM_LARB18_RAWC;
	struct clk *ISP_CAM_SUBSYS_RAWC;
	struct clk *ISP_CAM_TG_RAWC;

	struct clk *ISP_TOP_MUX_CAMTM;
};
struct ISP_CLK_STRUCT isp_clk;

struct isp_device {
	void __iomem *regs;
	struct device *dev;
	int irq;
#ifdef USE_MTK_SMI_LARB_API
	struct device *larb;
	struct device *larb_2nd;
#endif
};

struct isp_sec_dapc_reg {
	unsigned int CAM_REG_CTL_EN[ISP_DEV_NODE_NUM];
	unsigned int CAM_REG_CTL_EN2[ISP_DEV_NODE_NUM];
	unsigned int CAM_REG_CTL_EN3[ISP_DEV_NODE_NUM];
	//unsigned int CAM_REG_CTL_EN4[ISP_DEV_NODE_NUM];
	unsigned int CAM_REG_CTL_DMA_EN[ISP_DEV_NODE_NUM];
	unsigned int CAM_REG_CTL_DMA2_EN[ISP_DEV_NODE_NUM];
	unsigned int CAM_REG_CTL_SEL[ISP_DEV_NODE_NUM];
	unsigned int CAM_REG_CTL_SEL2[ISP_DEV_NODE_NUM];
};

static struct isp_device isp_devs[ISP_DEV_NODE_NUM];
static unsigned int m_CurrentPPB;
static struct isp_sec_dapc_reg lock_reg;
static unsigned int sec_on;
static int irq3a_wait_cnt = 1;
static int irq3a_print_vf_off[ISP_IRQ_TYPE_AMOUNT] = {-1};
static unsigned int cq_recovery[ISP_IRQ_TYPE_AMOUNT];

#if IS_ENABLED(CONFIG_PM_WAKELOCKS)
struct wakeup_source *isp_wake_lock;
#else
struct wake_lock isp_wake_lock;
#endif
static int g_WaitLockCt;

/*prevent isp race condition in vulunerbility test*/
static struct mutex open_isp_mutex;

/* Get HW modules' base address from device nodes */
#define ISP_CAMSYS_CONFIG_BASE (isp_devs[ISP_CAMSYS_CONFIG_IDX].regs)
#define ISP_CAMSYS_RAWA_CONFIG_BASE (isp_devs[ISP_CAMSYS_RAWA_CONFIG_IDX].regs)
#define ISP_CAMSYS_RAWB_CONFIG_BASE (isp_devs[ISP_CAMSYS_RAWB_CONFIG_IDX].regs)
#define ISP_CAMSYS_RAWC_CONFIG_BASE (isp_devs[ISP_CAMSYS_RAWC_CONFIG_IDX].regs)
#define ISP_CAM_A_BASE (isp_devs[ISP_CAM_A_IDX].regs)
#define ISP_CAM_B_BASE (isp_devs[ISP_CAM_B_IDX].regs)
#define ISP_CAM_C_BASE (isp_devs[ISP_CAM_C_IDX].regs)
#define ISP_CAM_A_INNER_BASE (isp_devs[ISP_CAM_A_INNER_IDX].regs)
#define ISP_CAM_B_INNER_BASE (isp_devs[ISP_CAM_B_INNER_IDX].regs)
#define ISP_CAM_C_INNER_BASE (isp_devs[ISP_CAM_C_INNER_IDX].regs)
#define ISP_CAMSV0_BASE (isp_devs[ISP_CAMSV0_IDX].regs)
#define ISP_CAMSV1_BASE (isp_devs[ISP_CAMSV1_IDX].regs)
#define ISP_CAMSV2_BASE (isp_devs[ISP_CAMSV2_IDX].regs)
#define ISP_CAMSV3_BASE (isp_devs[ISP_CAMSV3_IDX].regs)
#define ISP_CAMSV4_BASE (isp_devs[ISP_CAMSV4_IDX].regs)
#define ISP_CAMSV5_BASE (isp_devs[ISP_CAMSV5_IDX].regs)
#define ISP_CAMSV6_BASE (isp_devs[ISP_CAMSV6_IDX].regs)
#define ISP_CAMSV7_BASE (isp_devs[ISP_CAMSV7_IDX].regs)

#include "inc/cam_regs.h"

#if (SMI_LARB_MMU_CTL == 1)
void __iomem *SMI_LARB_BASE[21];
#endif

#endif

struct S_START_T {
	unsigned int sec;
	unsigned int usec;
};

/* maximum number for supporting user to do interrupt operation */
/* index 0 is for all the user that do not do register irq first */
#define IRQ_USER_NUM_MAX 32
static spinlock_t SpinLock_UserKey;

#if (TIMESTAMP_QUEUE_EN == 1)
static int32_t ISP_PopBufTimestamp(unsigned int module, unsigned int dma_id,
				   struct S_START_T *pTstp);

static int32_t ISP_WaitTimestampReady(unsigned int module, unsigned int dma_id);
#endif
#if ((TIMESTAMP_QUEUE_EN == 1) || (Lafi_WAM_CQ_ERR == 1))
static void ISP_GetDmaPortsStatus(enum ISP_DEV_NODE_ENUM reg_module,
			unsigned int *DmaPortsStats);
#endif

/******************************************************************************
 *
 *****************************************************************************/
/* internal data */
/* pointer to the kmalloc'd area, rounded up to a page boundary */
static int *pTbl_RTBuf[ISP_IRQ_TYPE_AMOUNT];
static int Tbl_RTBuf_MMPSize[ISP_IRQ_TYPE_AMOUNT];

/* original pointer for kmalloc'd area as returned by kmalloc */
static void *pBuf_kmalloc[ISP_IRQ_TYPE_AMOUNT];
/*  */
static struct ISP_RT_BUF_STRUCT *pstRTBuf[ISP_IRQ_TYPE_AMOUNT] = {NULL};

static unsigned int G_u4EnableClockCount[ISP_DEV_NODE_NUM];
static atomic_t G_u4DevNodeCt;

int pr_detect_count;

/* use to prevent CQ recovery when in-complete frame occurs*/
static unsigned int pre_magic_num;

static int RUNTIME_ISP_CAMSV_START_ST = 0;
static int RUNTIME_ISP_CAMSV_START_IDX = 0;

/******************************************************************************
 *
 *****************************************************************************/
struct ISP_USER_INFO_STRUCT {
	pid_t Pid;
	pid_t Tid;
};

/******************************************************************************
 *
 ******************************************************************************/
#define ISP_BUF_SIZE (4096)
#define ISP_BUF_WRITE_AMOUNT 6

enum ISP_BUF_STATUS_ENUM {
	ISP_BUF_STATUS_EMPTY,
	ISP_BUF_STATUS_HOLD,
	ISP_BUF_STATUS_READY
};

struct ISP_BUF_STRUCT {
	enum ISP_BUF_STATUS_ENUM Status;
	unsigned int Size;
	unsigned char *pData;
};

struct ISP_BUF_INFO_STRUCT {
	struct ISP_BUF_STRUCT Read;
	struct ISP_BUF_STRUCT Write[ISP_BUF_WRITE_AMOUNT];
};

/******************************************************************************
 *
 ******************************************************************************/
#define ISP_ISR_MAX_NUM 32
#define INT_ERR_WARN_TIMER_THREAS 1000
#define INT_ERR_WARN_MAX_TIME 1

struct ISP_IRQ_ERR_WAN_CNT_STRUCT {
	/* cnt for each err int # */
	unsigned int m_err_int_cnt[ISP_IRQ_TYPE_AMOUNT][ISP_ISR_MAX_NUM];
	/* cnt for each warning int # */
	unsigned int m_warn_int_cnt[ISP_IRQ_TYPE_AMOUNT][ISP_ISR_MAX_NUM];
	/* mark for err int, where its cnt > threshold */
	unsigned int m_err_int_mark[ISP_IRQ_TYPE_AMOUNT];
	/* mark for warn int, where its cnt > threshold */
	unsigned int m_warn_int_mark[ISP_IRQ_TYPE_AMOUNT];
	unsigned long m_int_usec[ISP_IRQ_TYPE_AMOUNT];
};

static int FirstUnusedIrqUserKey = 1;
#define USERKEY_USERNAME_STR_LEN 128

struct UserKeyInfo {
	/* name for the user that register a userKey */
	char userName[USERKEY_USERNAME_STR_LEN];
	int userKey; /* the user key for that user */
};
/* array for recording the user name for a specific user key */
static struct UserKeyInfo IrqUserKey_UserInfo[IRQ_USER_NUM_MAX];

struct ISP_IRQ_INFO_STRUCT {
	/* Add an extra index for status type in mt6797 -> signal or dma */
	unsigned int Status[ISP_IRQ_TYPE_AMOUNT][ISP_IRQ_ST_AMOUNT]
			   [IRQ_USER_NUM_MAX];

	unsigned int Mask[ISP_IRQ_TYPE_AMOUNT][ISP_IRQ_ST_AMOUNT];
	unsigned int ErrMask[ISP_IRQ_TYPE_AMOUNT][ISP_IRQ_ST_AMOUNT];
	unsigned int WarnMask[ISP_IRQ_TYPE_AMOUNT][ISP_IRQ_ST_AMOUNT];
	unsigned int Warn2Mask[ISP_IRQ_TYPE_AMOUNT][ISP_IRQ_ST_AMOUNT];
	/* flag for indicating that user do mark for a interrupt or not */
	unsigned int MarkedFlag[ISP_IRQ_TYPE_AMOUNT][ISP_IRQ_ST_AMOUNT]
			       [IRQ_USER_NUM_MAX];

	/* time for marking a specific interrupt */
	unsigned int MarkedTime_sec[ISP_IRQ_TYPE_AMOUNT][32][IRQ_USER_NUM_MAX];

	/* time for marking a specific interrupt */
	unsigned int MarkedTime_usec[ISP_IRQ_TYPE_AMOUNT][32][IRQ_USER_NUM_MAX];

	/* number of a specific signal that passed by */
	int PassedBySigCnt[ISP_IRQ_TYPE_AMOUNT][32][IRQ_USER_NUM_MAX];
	/* */
	unsigned int LastestSigTime_sec[ISP_IRQ_TYPE_AMOUNT][32];
	/* latest time for each interrupt */
	unsigned int LastestSigTime_usec[ISP_IRQ_TYPE_AMOUNT][32];
	/* latest time for each interrupt */
};

struct ISP_TIME_LOG_STRUCT {
	unsigned int Vd;
	unsigned int Expdone;
	unsigned int WorkQueueVd;
	unsigned int WorkQueueExpdone;
	unsigned int TaskletVd;
	unsigned int TaskletExpdone;
};

#if (TIMESTAMP_QUEUE_EN == 1)
#define ISP_TIMESTPQ_DEPTH (256)
struct ISP_TIMESTPQ_INFO_STRUCT {
	struct {
		struct S_START_T TimeQue[ISP_TIMESTPQ_DEPTH];
		/* increase when p1done or dmao done */
		unsigned int WrIndex;
		unsigned int RdIndex; /* increase when user deque */
		unsigned long long TotalWrCnt;
		unsigned long long TotalRdCnt;
		/* TSTP_V3 unsigned int     PrevFbcDropCnt; */
		unsigned int PrevFbcWCnt;
	} Dmao[_cam_max_];
	unsigned int DmaEnStatus[_cam_max_];
};
#endif

/**********************************************************************/
#define my_get_pow_idx(value)                                                  \
	({                                                                     \
		int i = 0, cnt = 0;                                            \
		for (i = 0; i < 32; i++) {                                     \
			if ((value >> i) & (0x00000001)) {                     \
				break;                                         \
			} else {                                               \
				cnt++;                                         \
			}                                                      \
		}                                                              \
		cnt;                                                           \
	})

static struct ISP_RAW_INT_STATUS g_ISPIntStatus[ISP_IRQ_TYPE_AMOUNT];
static struct ISP_RAW_INT_STATUS g_ISPIntStatus_SMI[ISP_IRQ_TYPE_AMOUNT];

/* ISP_CAM_A_IDX-ISP_CAM_C_IDX, each has 25 CQ thread. */
static unsigned int g_cqBaseAddr[ISP_CAM_C_IDX-ISP_CAM_A_IDX+1][25] = {{0} };
static unsigned int g_cq0NextBA[ISP_CAM_C_IDX-ISP_CAM_A_IDX+1][1] = {{0} };
#if Lafi_WAM_CQ_ERR
static unsigned int g_cqDoneStatus[ISP_CAM_C_IDX-ISP_CAM_A_IDX+1] = {0};
static union FBC_CTRL_2 g_fbc_ctrl2[ISP_CAM_C_IDX-ISP_CAM_A_IDX+1][_cam_max_];
#endif
static unsigned int g_DmaErr_CAM[ISP_IRQ_TYPE_AMOUNT][_cam_max_] = {{0} };

enum ISP_WAITQ_HEAD_IRQ_ENUM {
	ISP_WAITQ_HEAD_IRQ_SOF = 0,
	ISP_WAITQ_HEAD_IRQ_SW_P1_DONE,
	ISP_WAITQ_HEAD_IRQ_HW_P1_DONE,
	ISP_WAITQ_HEAD_IRQ_AAO_DONE,
	ISP_WAITQ_HEAD_IRQ_AAHO_DONE,
	ISP_WAITQ_HEAD_IRQ_FLKO_DONE,
	ISP_WAITQ_HEAD_IRQ_AFO_DONE,
	ISP_WAITQ_HEAD_IRQ_TSFSO_DONE,
	ISP_WAITQ_HEAD_IRQ_LTMSO_DONE,
	ISP_WAITQ_HEAD_IRQ_PDO_DONE,
	ISP_WAITQ_HEAD_IRQ_AMOUNT
};

enum ISP_WAIT_QUEUE_HEAD_IRQ_SV_ENUM {
	ISP_WAITQ_HEAD_IRQ_SV_SOF = 0,
	ISP_WAITQ_HEAD_IRQ_SV_SW_P1_DONE,
	ISP_WAITQ_HEAD_IRQ_SV_AMOUNT
};

#define CAM_AMOUNT (ISP_IRQ_TYPE_INT_CAM_C_ST - ISP_IRQ_TYPE_INT_CAM_A_ST + 1)
#define CAMSV_AMOUNT                                                           \
	(ISP_IRQ_TYPE_INT_CAMSV_END_ST - ISP_IRQ_TYPE_INT_CAMSV_START_ST + 1)

#define SUPPORT_MAX_IRQ 32
struct ISP_INFO_STRUCT {
	spinlock_t SpinLockIspRef;
	spinlock_t SpinLockIsp;
	spinlock_t SpinLockIrq[ISP_IRQ_TYPE_AMOUNT];
	spinlock_t SpinLockIrqCnt[ISP_IRQ_TYPE_AMOUNT];
	spinlock_t SpinLockRTBC;
	spinlock_t SpinLockClock;
	wait_queue_head_t WaitQueueHead[ISP_IRQ_TYPE_AMOUNT];

	wait_queue_head_t WaitQHeadCam[CAM_AMOUNT][ISP_WAITQ_HEAD_IRQ_AMOUNT];

	wait_queue_head_t WaitQHeadCamsv[CAMSV_AMOUNT]
					[ISP_WAITQ_HEAD_IRQ_SV_AMOUNT];
	unsigned int UserCount;
	unsigned int DebugMask;
	int IrqNum;
	struct ISP_IRQ_INFO_STRUCT IrqInfo;
	struct ISP_IRQ_ERR_WAN_CNT_STRUCT IrqCntInfo;
	struct ISP_BUF_INFO_STRUCT BufInfo;
	struct ISP_TIME_LOG_STRUCT TimeLog;
#if (TIMESTAMP_QUEUE_EN == 1)
	struct ISP_TIMESTPQ_INFO_STRUCT TstpQInfo[ISP_IRQ_TYPE_AMOUNT];
#endif
};

static struct ISP_INFO_STRUCT IspInfo;
static bool SuspnedRecord[ISP_DEV_NODE_NUM] = {0};
//Drop frame check
static unsigned int g_virtual_cq_cnt[ISP_IRQ_TYPE_INT_CAM_C_ST-ISP_IRQ_TYPE_INT_CAM_A_ST+1] = {0};

enum eLOG_TYPE {
	/* currently, only used at ipl_buf_ctrl. to protect critical section */
	_LOG_DBG = 0,
	_LOG_INF = 1,
	_LOG_ERR = 2,
	_LOG_MAX = 3,
};

enum RAW_IDX {
	CAM_A = 0,
	CAM_B,
	CAM_C,
	CAM_MAX,
};
#define P1DONE_STR_LEN (256)
struct RAW_LOG {
	char module;
	char _str[P1DONE_STR_LEN];
};
static struct RAW_LOG gPass1doneLog[ISP_IRQ_TYPE_AMOUNT];
static struct RAW_LOG gLostPass1doneLog[ISP_IRQ_TYPE_AMOUNT];
#define NORMAL_STR_LEN (512)
#define ERR_PAGE 2
#define DBG_PAGE 2
#define INF_PAGE 4
/* #define SV_LOG_STR_LEN NORMAL_STR_LEN */

#define LOG_PPNUM 2
struct SV_LOG_STR {
	unsigned int _cnt[LOG_PPNUM][_LOG_MAX];
	/* char   _str[_LOG_MAX][SV_LOG_STR_LEN]; */
	char *_str[LOG_PPNUM][_LOG_MAX];
	struct S_START_T _lastIrqTime;
};

/* If log buffer size(See avaLen in the macro, "IRQ_LOG_KEEPER") is not
 * enough, dbg msg will be printed out via pr_info/pr_notice. This behavior
 * costs too much for ISR and CPU. If ISR_NO_PRINT is defined as "true", we'll
 * hide the dbg msg, when log buffer is not enough.
 */
#define ISR_NO_PRINT (true)

static void print_isr_log(unsigned int logtype, char *ptr)
{
	int i = 0;
	unsigned int log_page_num = 0;

	if (logtype == _LOG_DBG)
		log_page_num = DBG_PAGE;
	else if (logtype == _LOG_INF)
		log_page_num = INF_PAGE;
	else if (logtype == _LOG_ERR)
		log_page_num = ERR_PAGE;

	for (i = 0; i < log_page_num; i++) {
		if (ptr[NORMAL_STR_LEN * (i + 1) - 1] != '\0') {
			ptr[NORMAL_STR_LEN * (i + 1) - 1] = '\0';

			if ((ISR_NO_PRINT) || (logtype == _LOG_DBG))
				LOG_DBG("%s", &ptr[NORMAL_STR_LEN * i]);
			else if (logtype == _LOG_INF)
				LOG_INF("%s", &ptr[NORMAL_STR_LEN * i]);
			else if (logtype == _LOG_ERR)
				LOG_NOTICE("%s", &ptr[NORMAL_STR_LEN * i]);
			else
				LOG_NOTICE("Not Support logtype(%d)\n", logtype);
		} else {
			if ((ISR_NO_PRINT) || (logtype == _LOG_DBG))
				LOG_DBG("%s", &ptr[NORMAL_STR_LEN * i]);
			else if (logtype == _LOG_INF)
				LOG_INF("%s", &ptr[NORMAL_STR_LEN * i]);
			else if (logtype == _LOG_ERR)
				LOG_NOTICE("%s", &ptr[NORMAL_STR_LEN * i]);
			else
				LOG_NOTICE("Not Support logtype(%d)\n", logtype);
			break;
		}
	}
}

static void *pLog_kmalloc;
static struct SV_LOG_STR gSvLog[ISP_IRQ_TYPE_AMOUNT];
static bool g_is_dumping[ISP_DEV_NODE_NUM] = {0};

#define HW_DON_LOG_ENABLE (0)

#define STR_REG "Addr(0x%x) 0x%8x-0x%8x-0x%8x-0x%8x|0x%8x-0x%8x-0x%8x-0x%8x\n"
/**
 *   for irq used,keep log until IRQ_LOG_PRINTER being involked,
 *   limited:
 *   each log must shorter than 512 bytes
 *   total log length in each irq/logtype can't over 1024 bytes
 */
#define IRQ_LOG_KEEPER_T(sec, usec)                                            \
	{                                                                      \
		ktime_t time;                                                  \
		time = ktime_get();                                            \
		sec = time;                                                    \
		do_div(sec, 1000);                                             \
		usec = do_div(sec, 1000000);                                   \
	}
#define IRQ_LOG_KEEPER(irq, ppb, logT, fmt, ...) do {\
	char *ptr; \
	char *pDes;\
	int avaLen;\
	unsigned int *ptr2 = &gSvLog[irq]._cnt[ppb][logT];\
	unsigned int str_leng;\
	struct SV_LOG_STR *pSrc = &gSvLog[irq];\
	if (logT == _LOG_ERR) {\
		str_leng = NORMAL_STR_LEN*ERR_PAGE;\
	} else if (logT == _LOG_DBG) {\
		str_leng = NORMAL_STR_LEN*DBG_PAGE;\
	} else if (logT == _LOG_INF) {\
		str_leng = NORMAL_STR_LEN*INF_PAGE;\
	} else {\
		str_leng = 0;\
	} \
	ptr = pDes = (char *)&(gSvLog[irq].\
		_str[ppb][logT][gSvLog[irq]._cnt[ppb][logT]]);\
	\
	avaLen = str_leng - 1 - gSvLog[irq]._cnt[ppb][logT];\
	if (avaLen > 1) {\
		if (snprintf((char *)(pDes), avaLen, "[%d.%06d]" fmt,\
			gSvLog[irq]._lastIrqTime.sec,\
			gSvLog[irq]._lastIrqTime.usec,\
			##__VA_ARGS__) < 0) {\
			LOG_NOTICE("[Error] %s: snprintf failed", __func__);\
		} \
		if ('\0' != gSvLog[irq]._str[ppb][logT][str_leng - 1]) {\
			LOG_NOTICE("log str over flow(%d)", irq);\
		} \
		while (*ptr++ != '\0') {\
			(*ptr2)++;\
		} \
	} else {\
		if (ISR_NO_PRINT) \
			LOG_DBG("(%d)(%d)log str avalible=0, print log\n", irq, logT);\
		else \
			LOG_INF("(%d)(%d)log str avalible=0, print log\n", irq, logT);\
		ptr = pSrc->_str[ppb][logT];\
		if (pSrc->_cnt[ppb][logT] != 0) {\
			print_isr_log(logT, ptr); \
			ptr[0] = '\0';\
			pSrc->_cnt[ppb][logT] = 0;\
			avaLen = str_leng - 1;\
			ptr = pDes = (char *)&(pSrc->\
				_str[ppb][logT][pSrc->_cnt[ppb][logT]]);\
			\
			ptr2 = &(pSrc->_cnt[ppb][logT]);\
			if (snprintf((char *)(pDes), avaLen,\
				fmt, ##__VA_ARGS__) < 0) {\
				LOG_NOTICE("[Error] %s: snprintf failed",\
					    __func__);\
			} \
			while (*ptr++ != '\0') {\
				(*ptr2)++;\
			} \
		} \
	} \
} while (0)

#if 1
#define IRQ_LOG_PRINTER(irq, ppb_in, logT_in) do {\
		struct SV_LOG_STR *pSrc = &gSvLog[irq];\
		char *ptr;\
		unsigned int i;\
		unsigned int ppb = 0;\
		unsigned int logT = 0;\
		if (ppb_in > 1) {\
			ppb = 1;\
		} else {\
			ppb = ppb_in;\
		} \
		if (logT_in > _LOG_ERR) {\
			logT = _LOG_ERR;\
		} else {\
			logT = logT_in;\
		} \
		ptr = pSrc->_str[ppb][logT];\
		if (pSrc->_cnt[ppb][logT] != 0) {\
			if (logT == _LOG_DBG) {\
				for (i = 0; i < DBG_PAGE; i++) {\
					if (ptr[NORMAL_STR_LEN*(i+1) - 1] !=\
					    '\0') {\
						ptr[NORMAL_STR_LEN*(i+1) - 1] =\
						'\0';\
						LOG_DBG("%s",\
						    &ptr[NORMAL_STR_LEN*i]);\
					} else {\
						LOG_DBG("%s",\
						    &ptr[NORMAL_STR_LEN*i]);\
						break;\
					} \
				} \
			} \
			else if (logT == _LOG_INF) {\
				for (i = 0; i < INF_PAGE; i++) {\
					if (ptr[NORMAL_STR_LEN*(i+1) - 1] !=\
					    '\0') {\
						ptr[NORMAL_STR_LEN*(i+1) - 1] =\
							'\0';\
						\
						LOG_INF("%s",\
						    &ptr[NORMAL_STR_LEN*i]);\
					} else {\
						LOG_INF("%s",\
						    &ptr[NORMAL_STR_LEN*i]);\
						break;\
					} \
				} \
			} \
			else if (logT == _LOG_ERR) {\
				for (i = 0; i < ERR_PAGE; i++) {\
					if (ptr[NORMAL_STR_LEN*(i+1) - 1] !=\
						'\0') {\
						ptr[NORMAL_STR_LEN*(i+1) - 1] =\
						'\0';\
						LOG_NOTICE("%s",\
						    &ptr[NORMAL_STR_LEN*i]);\
					} else {\
						LOG_NOTICE("%s",\
						    &ptr[NORMAL_STR_LEN*i]);\
						break;\
					} \
				} \
			} \
			else {\
				LOG_NOTICE("N.S.%d", logT);\
			} \
			ptr[0] = '\0';\
			pSrc->_cnt[ppb][logT] = 0;\
		} \
	} while (0)

#else
#define IRQ_LOG_PRINTER(irq, ppb, logT)
#endif

/* //////////////////////////////////////////////////// */

struct _isp_bk_reg_t {
	unsigned int CAM_TG_INTER_ST; /* 453C */
};

static struct _isp_bk_reg_t g_BkReg[ISP_IRQ_TYPE_AMOUNT];

/* CAM_REG_TG_INTER_ST 0x3b3c, CAMSV_REG_TG_INTER_ST 0x016C */
void __iomem *CAMX_REG_TG_INTER_ST(int reg_module)
{
	void __iomem *_regVal;

	switch (reg_module) {
	case ISP_CAM_A_IDX:
	case ISP_CAM_B_IDX:
	case ISP_CAM_C_IDX:
		_regVal = CAM_REG_TG_INTER_ST(reg_module);
		break;
	case ISP_CAMSV0_IDX:
	case ISP_CAMSV1_IDX:
	case ISP_CAMSV2_IDX:
	case ISP_CAMSV3_IDX:
	case ISP_CAMSV4_IDX:
	case ISP_CAMSV5_IDX:
	case ISP_CAMSV6_IDX:
	case ISP_CAMSV7_IDX:
		_regVal = CAMSV_REG_TG_INTER_ST(reg_module);
		break;
	default:
		_regVal = CAM_REG_TG_INTER_ST(reg_module);
	}
	return _regVal;
}

/* CAM_REG_TG_VF_CON 0x3b04, CAMSV_REG_TG_VF_CON 0x0134 */
void __iomem *CAMX_REG_TG_VF_CON(int reg_module)
{
	void __iomem *_regVal;

	switch (reg_module) {
	case ISP_CAM_A_IDX:
	case ISP_CAM_B_IDX:
	case ISP_CAM_C_IDX:
		_regVal = CAM_REG_TG_VF_CON(reg_module);
		break;
	case ISP_CAMSV0_IDX:
	case ISP_CAMSV1_IDX:
	case ISP_CAMSV2_IDX:
	case ISP_CAMSV3_IDX:
	case ISP_CAMSV4_IDX:
	case ISP_CAMSV5_IDX:
	case ISP_CAMSV6_IDX:
	case ISP_CAMSV7_IDX:
		_regVal = CAMSV_REG_TG_VF_CON(reg_module);
		break;
	default:
		_regVal = CAM_REG_TG_VF_CON(reg_module);
	}
	return _regVal;
}

/* CAM_REG_TG_SEN_MODE 0x3b00, CAMSV_REG_TG_SEN_MODE 0x0130 */
void __iomem *CAMX_REG_TG_SEN_MODE(int reg_module)
{
	void __iomem *_regVal;

	switch (reg_module) {
	case ISP_CAM_A_IDX:
	case ISP_CAM_B_IDX:
	case ISP_CAM_C_IDX:
		_regVal = CAM_REG_TG_SEN_MODE(reg_module);
		break;
	case ISP_CAMSV0_IDX:
	case ISP_CAMSV1_IDX:
	case ISP_CAMSV2_IDX:
	case ISP_CAMSV3_IDX:
	case ISP_CAMSV4_IDX:
	case ISP_CAMSV5_IDX:
	case ISP_CAMSV6_IDX:
	case ISP_CAMSV7_IDX:
		_regVal = CAMSV_REG_TG_SEN_MODE(reg_module);
		break;
	default:
		_regVal = CAM_REG_TG_SEN_MODE(reg_module);
	}
	return _regVal;
}

/* if isp has been suspend, frame cnt needs to add previous value*/
/* CAM_REG_TG_INTER_ST 0x3b3c, CAMSV_REG_TG_INTER_ST 0x016C */
unsigned int ISP_RD32_TG_CAMX_FRM_CNT(unsigned int IrqType, int reg_module)
{
	unsigned int _regVal;

	switch (reg_module) {
	case ISP_CAM_A_IDX:
	case ISP_CAM_B_IDX:
	case ISP_CAM_C_IDX:
		_regVal = ISP_RD32(CAM_REG_TG_INTER_ST(reg_module));
		break;
	case ISP_CAMSV0_IDX:
	case ISP_CAMSV1_IDX:
	case ISP_CAMSV2_IDX:
	case ISP_CAMSV3_IDX:
	case ISP_CAMSV4_IDX:
	case ISP_CAMSV5_IDX:
	case ISP_CAMSV6_IDX:
	case ISP_CAMSV7_IDX:
		_regVal = ISP_RD32(CAMSV_REG_TG_INTER_ST(reg_module));
		break;
	default:
		_regVal = ISP_RD32(CAM_REG_TG_INTER_ST(reg_module));
	}
	_regVal = ((_regVal & 0x00FF0000) >> 16) +
		  g_BkReg[IrqType].CAM_TG_INTER_ST;
	if (_regVal > 255)
		_regVal -= 256;
	return _regVal;
}

/*******************************************************************************
 * Add MET ftrace event for power profilling. CAM_enter when SOF and CAM_leave
 * when P1_Done
 ******************************************************************************/
#if defined(ISP_MET_READY)
int MET_Event_Get_BPP(enum _isp_dma_enum_ dmao, unsigned int reg_module)
{
	unsigned int fmt_sel = ISP_RD32(CAM_REG_CTL_FMT_SEL(reg_module));
	int ret = 0;

	if (dmao == _imgo_) {
		switch ((fmt_sel >> 4) & 0x1F) {
		case 0:
		case 12:
		case 13:
		case 14:
		case 15:
			ret = 16;
			break;
		case 8:
			ret = 8;
			break;
		case 9:
		case 16:
			ret = 10;
			break;
		case 10:
			ret = 12;
			break;
		case 11:
			ret = 14;
			break;
		default:
			LOG_NOTICE("get imgo bpp error fmt_sel:0x%x value=%x\n",
				   fmt_sel, (fmt_sel >> 4) & 0x1F);
			break;
		}
	} else if (dmao == _rrzo_) {
		switch ((fmt_sel >> 2) & 0x3) {
		case 0:
			ret = 8;
			break;
		case 1:
			ret = 10;
			break;
		case 2:
			ret = 12;
			break;
		default:
			LOG_NOTICE("get rrzo bpp error fmt_sel:0x%x value=%x\n",
				   fmt_sel, (fmt_sel >> 2) & 0x3);
			break;
		}
	}

	return ret;
}

void CAMSYS_MET_Events_Trace(bool enter, u32 reg_module,
			     enum ISP_IRQ_TYPE_ENUM cam)
{
	if (enter) {
		int imgo_en = 0, rrzo_en = 0, imgo_bpp, rrzo_bpp;
		int imgo_xsize, imgo_ysize;
		int rrzo_xsize, rrzo_ysize, rrz_src_w, rrz_src_h, rrz_dst_w;
		int rrz_dst_h, rrz_hori_step, rrz_vert_step;
		u32 ctl_dma_en, rrz_in, rrz_out;
		u32 ctl_en, ctl_en2;

		if (sec_on) {
			ctl_dma_en = lock_reg.CAM_REG_CTL_DMA_EN[reg_module];
			ctl_en = lock_reg.CAM_REG_CTL_EN[reg_module];
			ctl_en2 = lock_reg.CAM_REG_CTL_EN2[reg_module];
		} else {
			ctl_dma_en = ISP_RD32(CAM_REG_CTL_DMA_EN(reg_module));
			ctl_en = ISP_RD32(CAM_REG_CTL_EN(reg_module));
			ctl_en2 = ISP_RD32(CAM_REG_CTL_EN2(reg_module));
		}
		rrz_in = ISP_RD32(CAM_REG_RRZ_IN_IMG(reg_module));
		rrz_out = ISP_RD32(CAM_REG_RRZ_OUT_IMG(reg_module));
		imgo_en = ctl_dma_en & 0x1;
		rrzo_en = ctl_dma_en & 0x4;
		imgo_bpp = MET_Event_Get_BPP(_imgo_, reg_module);
		rrzo_bpp = MET_Event_Get_BPP(_rrzo_, reg_module);
		imgo_xsize = (int)(ISP_RD32(CAM_REG_IMGO_XSIZE(reg_module)) &
				   0xFFFF);

		imgo_ysize = (int)(ISP_RD32(CAM_REG_IMGO_YSIZE(reg_module)) &
				   0xFFFF);

		rrzo_xsize = (int)(ISP_RD32(CAM_REG_RRZO_XSIZE(reg_module)) &
				   0xFFFF);

		rrzo_ysize = (int)(ISP_RD32(CAM_REG_RRZO_YSIZE(reg_module)) &
				   0xFFFF);

		rrz_src_w = rrz_in & 0xFFFF;
		rrz_src_h = (rrz_in >> 16) & 0xFFFF;
		rrz_dst_w = rrz_out & 0xFFFF;
		rrz_dst_h = (rrz_out >> 16) & 0xFFFF;

		rrz_hori_step =
			(int)(ISP_RD32(CAM_REG_RRZ_HORI_STEP(reg_module)) &
			      0x3FFFF);

		rrz_vert_step =
			(int)(ISP_RD32(CAM_REG_RRZ_VERT_STEP(reg_module)) &
			      0x3FFFF);

		trace_ISP__Pass1_CAM_enter(
			cam, imgo_en, rrzo_en, imgo_bpp, rrzo_bpp, imgo_xsize,
			imgo_ysize, rrzo_xsize, rrzo_ysize, rrz_src_w,
			rrz_src_h, rrz_dst_w, rrz_dst_h, rrz_hori_step,
			rrz_vert_step, ctl_en, ctl_dma_en, ctl_en2);
	} else {
		trace_ISP__Pass1_CAM_leave(cam, 0);
	}
}
#endif

/******************************************************************************
 *
 *****************************************************************************/
static int32_t ISP_CheckUseCamWaitQ(enum ISP_IRQ_TYPE_ENUM type,
				    enum ISP_ST_ENUM st_type,
				    unsigned int status)
{
	if (type >= ISP_IRQ_TYPE_INT_CAM_A_ST &&
	    type <= ISP_IRQ_TYPE_INT_CAM_C_ST) {
		if (st_type == SIGNAL_INT) {
			if (status == SOF_INT_ST || status == SW_PASS1_DON_ST ||
			    status == HW_PASS1_DON_ST)
				return 1;
		} else if (st_type == DMA_INT) {
			if (status == AAO_DONE_ST || status == FLKO_DONE_ST ||
			    status == AFO_DONE_ST || status == PDO_DONE_ST ||
	    status == TSFSO_DONE_ST || status == LTMSO_R1_DONE_ST ||
			    status == AAHO_DONE_ST) {
				return 1;
			}
		}
	}

	return 0;
}

/******************************************************************************
 *
 *****************************************************************************/
static int32_t ISP_CheckUseCamsvWaitQ(enum ISP_IRQ_TYPE_ENUM type,
				      enum ISP_ST_ENUM st_type,
				      unsigned int status)
{
	if (type >= ISP_IRQ_TYPE_INT_CAMSV_START_ST &&
	    type <= ISP_IRQ_TYPE_INT_CAMSV_END_ST) {
		if (st_type == SIGNAL_INT) {
			if (status == SV_SOF_INT_ST ||
			    status == SV_SW_PASS1_DON_ST)
				return 1;
		}
	}

	return 0;
}

/******************************************************************************
 *
 *****************************************************************************/
static int32_t ISP_GetWaitQCamIndex(enum ISP_IRQ_TYPE_ENUM type)
{
	int32_t index = type - ISP_IRQ_TYPE_INT_CAM_A_ST;

	if (index >= CAM_AMOUNT)
		pr_info("waitq cam index out of range:%d", index);

	return index;
}

/******************************************************************************
 *
 *****************************************************************************/
static int32_t ISP_GetWaitQCamsvIndex(enum ISP_IRQ_TYPE_ENUM type)
{
	int32_t index = type - ISP_IRQ_TYPE_INT_CAMSV_START_ST;

	if (index >= CAMSV_AMOUNT)
		pr_info("waitq camsv index out of range:%d", index);

	return index;
}

/******************************************************************************
 *
 *****************************************************************************/
static int32_t ISP_GetWaitQCamIrqIndex(enum ISP_ST_ENUM st_type,
				       unsigned int status)
{
	int32_t index = ISP_WAITQ_HEAD_IRQ_AMOUNT;

	if (st_type == SIGNAL_INT) {
		if (status == SOF_INT_ST)
			index = ISP_WAITQ_HEAD_IRQ_SOF;
		else if (status == SW_PASS1_DON_ST)
			index = ISP_WAITQ_HEAD_IRQ_SW_P1_DONE;
		else if (status == HW_PASS1_DON_ST)
			index = ISP_WAITQ_HEAD_IRQ_HW_P1_DONE;
	} else if (st_type == DMA_INT) {
		if (status == AAO_DONE_ST)
			index = ISP_WAITQ_HEAD_IRQ_AAO_DONE;
		else if (status == AAHO_DONE_ST)
			index = ISP_WAITQ_HEAD_IRQ_AAHO_DONE;
		else if (status == FLKO_DONE_ST)
			index = ISP_WAITQ_HEAD_IRQ_FLKO_DONE;
		else if (status == AFO_DONE_ST)
			index = ISP_WAITQ_HEAD_IRQ_AFO_DONE;
		else if (status == TSFSO_DONE_ST)
			index = ISP_WAITQ_HEAD_IRQ_TSFSO_DONE;
		else if (status == LTMSO_R1_DONE_ST)
			index = ISP_WAITQ_HEAD_IRQ_LTMSO_DONE;
		else if (status == PDO_DONE_ST)
			index = ISP_WAITQ_HEAD_IRQ_PDO_DONE;
	}

	if (index == ISP_WAITQ_HEAD_IRQ_AMOUNT)
		pr_info("waitq cam irq index out of range:%d_%d", st_type,
			status);

	return index;
}

/******************************************************************************
 *
 *****************************************************************************/
static int32_t ISP_GetWaitQCamsvIrqIndex(enum ISP_ST_ENUM st_type,
					 unsigned int status)
{
	int32_t index = ISP_WAITQ_HEAD_IRQ_SV_AMOUNT;

	if (st_type == SIGNAL_INT) {
		if (status == SV_SOF_INT_ST)
			index = ISP_WAITQ_HEAD_IRQ_SV_SOF;
		else if (status == SV_SW_PASS1_DON_ST)
			index = ISP_WAITQ_HEAD_IRQ_SV_SW_P1_DONE;
	}

	if (index == ISP_WAITQ_HEAD_IRQ_SV_AMOUNT)
		pr_info("waitq camsv irq index out of range:%d_%d", st_type,
			status);

	return index;
}

/******************************************************************************
 *
 ******************************************************************************/
static inline unsigned int ISP_MsToJiffies(unsigned int Ms)
{
	return ((Ms * HZ + 512) >> 10);
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline unsigned int ISP_UsToJiffies(unsigned int Us)
{
	return (((Us / 1000) * HZ + 512) >> 10);
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline unsigned int ISP_GetIRQState(unsigned int type,
					   unsigned int stType,
					   unsigned int userNumber,
					   unsigned int stus)
{
	unsigned int ret;
	/* FIX to avoid build warning */
	unsigned long flags; /* old: unsigned int flags; */

	/*  */
	spin_lock_irqsave(&(IspInfo.SpinLockIrq[type]), flags);
	ret = (IspInfo.IrqInfo.Status[type][stType][userNumber] & stus);
	spin_unlock_irqrestore(&(IspInfo.SpinLockIrq[type]), flags);
	/*  */
	return ret;
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline unsigned int ISP_JiffiesToMs(unsigned int Jiffies)
{
	return ((Jiffies * 1000) / HZ);
}

/*******************************************************************************
 *
 ******************************************************************************/
#if (Lafi_WAM_CQ_ERR == 1)
static void ISP_RecordCQAddr(enum ISP_DEV_NODE_ENUM regModule)
{
	unsigned int i = 0, tmp_module = 0, index = 0;
	union CAMCTL_TWIN_STATUS_ twinStatus;
	unsigned int reg_module_array[3];

	unsigned int reg_module_count = 0;
	unsigned int reg_value = 0;
	unsigned int DmaEnStatus[ISP_CAM_C_IDX-ISP_CAM_A_IDX+1][_cam_max_];

	reg_module_array[0] = regModule;
	twinStatus.Raw = ISP_RD32(CAM_REG_CTL_TWIN_STATUS(regModule));

	if (twinStatus.Bits.TWIN_EN == MTRUE) {
		if (((reg_module_array[0] == ISP_CAM_A_IDX) &&
		     (twinStatus.Bits.MASTER_MODULE != CAM_A)) ||
		    ((reg_module_array[0] == ISP_CAM_B_IDX) &&
		     (twinStatus.Bits.MASTER_MODULE != CAM_B)) ||
		    ((reg_module_array[0] == ISP_CAM_C_IDX) &&
		     (twinStatus.Bits.MASTER_MODULE != CAM_C))) {
			LOG_NOTICE(
				"twin module is invalid! recover fail\n");
		}

		if (IS_MT6853(g_platform_id)) {
			if (twinStatus.Bits.TWIN_MODULE == CAM_C)
				LOG_NOTICE("twin module(CAM_C) is invalid! recover fail\n");
		}

		for (i = 0; i < twinStatus.Bits.SLAVE_CAM_NUM; i++) {
			if (i == 0) {
				switch (twinStatus.Bits.TWIN_MODULE) {
				case CAM_A:
				reg_module_array[1] = ISP_CAM_A_IDX;
				break;
				case CAM_B:
				reg_module_array[1] = ISP_CAM_B_IDX;
				break;
				case CAM_C:
				reg_module_array[1] = ISP_CAM_C_IDX;
				break;
				default:
				LOG_NOTICE("twin module is invalid! recover fail\n");
				}
			} else if (i == 1) {
				switch (twinStatus.Bits.TRIPLE_MODULE) {
				case CAM_A:
				reg_module_array[2] = ISP_CAM_A_IDX;
				break;
				case CAM_B:
				reg_module_array[2] = ISP_CAM_B_IDX;
				break;
				case CAM_C:
				reg_module_array[2] = ISP_CAM_C_IDX;
				break;
				default:
				LOG_NOTICE("twin module is invalid! recover fail\n");
				}
			}
		}
		reg_module_count = twinStatus.Bits.SLAVE_CAM_NUM + 1;
	} else {
		reg_module_count = 1;
	}


	for (i = 0; i < reg_module_count; i++) {
		//read inner register
		tmp_module = reg_module_array[i] - ISP_CAMSYS_RAWC_CONFIG_IDX;
		index = tmp_module - ISP_CAM_A_INNER_IDX;

		if (index > (ISP_CAM_C_INNER_IDX - ISP_CAM_A_INNER_IDX)) {
			LOG_NOTICE("index is invalid! recover fail\n");
			return;
		}
		//CQ1
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR1_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][1] =
				ISP_RD32(CAM_REG_CQ_THR1_BASEADDR(tmp_module));
		//CQ9
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR9_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][9] =
				ISP_RD32(CAM_REG_CQ_THR9_BASEADDR(tmp_module));
		//CQ10
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR10_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][10] =
				ISP_RD32(CAM_REG_CQ_THR10_BASEADDR(tmp_module));
		//CQ11
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR11_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][11] =
				ISP_RD32(CAM_REG_CQ_THR11_BASEADDR(tmp_module));
		//CQ12
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR12_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][12] =
				ISP_RD32(CAM_REG_CQ_THR12_BASEADDR(tmp_module));
		//CQ13
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR13_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][13] =
				ISP_RD32(CAM_REG_CQ_THR13_BASEADDR(tmp_module));
		//CQ14
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR14_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][14] =
				ISP_RD32(CAM_REG_CQ_THR14_BASEADDR(tmp_module));
		//CQ16
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR16_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][16] =
				ISP_RD32(CAM_REG_CQ_THR16_BASEADDR(tmp_module));
		//CQ17
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR17_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][17] =
				ISP_RD32(CAM_REG_CQ_THR17_BASEADDR(tmp_module));
		//CQ20
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR20_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][20] =
				ISP_RD32(CAM_REG_CQ_THR20_BASEADDR(tmp_module));
		//CQ24
		reg_value = (unsigned int)ISP_RD32(
					CAM_REG_CQ_THR24_CTL(tmp_module));
		if (reg_value & 0x1)
			g_cqBaseAddr[index][24] =
				ISP_RD32(CAM_REG_CQ_THR24_BASEADDR(tmp_module));
	}

	/* record current FBC value */
	for (i = 0; i < reg_module_count; i++) {
		tmp_module = reg_module_array[i];
		index = tmp_module - ISP_CAM_A_IDX;
		if (index > (ISP_CAM_C_IDX - ISP_CAM_A_IDX)) {
			LOG_NOTICE(
				"index is invalid! recover fail\n");
				return;
		}
		ISP_GetDmaPortsStatus(tmp_module,
			&DmaEnStatus[index][0]);
		//read inner register
		tmp_module = reg_module_array[i] - ISP_CAMSYS_RAWC_CONFIG_IDX;
		if (DmaEnStatus[index][_aao_])
			g_fbc_ctrl2[index][_aao_].Raw =
				ISP_RD32(CAM_REG_FBC_AAO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_aao_].Raw = 0;

		if (DmaEnStatus[index][_aaho_])
			g_fbc_ctrl2[index][_aaho_].Raw =
				ISP_RD32(CAM_REG_FBC_AAHO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_aaho_].Raw = 0;

		if (DmaEnStatus[index][_afo_])
			g_fbc_ctrl2[index][_afo_].Raw =
				ISP_RD32(CAM_REG_FBC_AFO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_afo_].Raw = 0;

		if (DmaEnStatus[index][_flko_])
			g_fbc_ctrl2[index][_flko_].Raw =
				ISP_RD32(CAM_REG_FBC_FLKO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_flko_].Raw = 0;

		if (DmaEnStatus[index][_pdo_])
			g_fbc_ctrl2[index][_pdo_].Raw =
				ISP_RD32(CAM_REG_FBC_PDO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_pdo_].Raw = 0;

		if (DmaEnStatus[index][_imgo_])
			g_fbc_ctrl2[index][_imgo_].Raw =
				ISP_RD32(CAM_REG_FBC_IMGO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_imgo_].Raw = 0;

		if (DmaEnStatus[index][_rrzo_])
			g_fbc_ctrl2[index][_rrzo_].Raw =
				ISP_RD32(CAM_REG_FBC_RRZO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_rrzo_].Raw = 0;

		if (DmaEnStatus[index][_ufeo_])
			g_fbc_ctrl2[index][_ufeo_].Raw =
				ISP_RD32(CAM_REG_FBC_UFEO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_ufeo_].Raw = 0;

		if (DmaEnStatus[index][_ufgo_])
			g_fbc_ctrl2[index][_ufgo_].Raw =
				ISP_RD32(CAM_REG_FBC_UFGO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_ufgo_].Raw = 0;

		if (DmaEnStatus[index][_rsso_])
			g_fbc_ctrl2[index][_rsso_].Raw =
				ISP_RD32(CAM_REG_FBC_RSSO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_rsso_].Raw = 0;

		if (DmaEnStatus[index][_lmvo_])
			g_fbc_ctrl2[index][_lmvo_].Raw =
				ISP_RD32(CAM_REG_FBC_LMVO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_lmvo_].Raw = 0;

		if (DmaEnStatus[index][_lcso_])
			g_fbc_ctrl2[index][_lcso_].Raw =
				ISP_RD32(CAM_REG_FBC_LCESO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_lcso_].Raw = 0;

		if (DmaEnStatus[index][_lcesho_])
			g_fbc_ctrl2[index][_lcesho_].Raw =
				ISP_RD32(CAM_REG_FBC_LCESHO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_lcesho_].Raw = 0;

		if (DmaEnStatus[index][_ltmso_])
			g_fbc_ctrl2[index][_ltmso_].Raw =
				ISP_RD32(CAM_REG_FBC_LTMSO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_ltmso_].Raw = 0;

		if (DmaEnStatus[index][_tsfso_])
			g_fbc_ctrl2[index][_tsfso_].Raw =
				ISP_RD32(CAM_REG_FBC_TSFSO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_tsfso_].Raw = 0;

		if (DmaEnStatus[index][_yuvo_])
			g_fbc_ctrl2[index][_yuvo_].Raw =
				ISP_RD32(CAM_REG_FBC_YUVO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_yuvo_].Raw = 0;

		if (DmaEnStatus[index][_yuvbo_])
			g_fbc_ctrl2[index][_yuvbo_].Raw =
				ISP_RD32(CAM_REG_FBC_YUVBO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_yuvbo_].Raw = 0;

		if (DmaEnStatus[index][_yuvco_])
			g_fbc_ctrl2[index][_yuvco_].Raw =
				ISP_RD32(CAM_REG_FBC_YUVCO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_yuvco_].Raw = 0;

		if (DmaEnStatus[index][_crzo_])
			g_fbc_ctrl2[index][_crzo_].Raw =
				ISP_RD32(CAM_REG_FBC_CRZO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_crzo_].Raw = 0;

		if (DmaEnStatus[index][_crzbo_])
			g_fbc_ctrl2[index][_crzbo_].Raw =
				ISP_RD32(CAM_REG_FBC_CRZBO_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_crzbo_].Raw = 0;

		if (DmaEnStatus[index][_crzo_r2_])
			g_fbc_ctrl2[index][_crzo_r2_].Raw =
				ISP_RD32(CAM_REG_FBC_CRZO_R2_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_crzo_r2_].Raw = 0;

		if (DmaEnStatus[index][_rsso_r2_])
			g_fbc_ctrl2[index][_rsso_r2_].Raw =
				ISP_RD32(CAM_REG_FBC_RSSO_R2_CTL2(tmp_module));
		else
			g_fbc_ctrl2[index][_rsso_r2_].Raw = 0;
	}
}
#endif
/*******************************************************************************
 *
 ******************************************************************************/
static void ISP_DumpDebugData(enum ISP_IRQ_TYPE_ENUM module, unsigned int debug_idx)
{
	unsigned int checksum, data_cnt, lpix_cnt, lpix_cnt_tmp;
	unsigned int smi_dbg_data_local, smi_dbg_data_case0;
	unsigned int smi_dbg_data_local_be, smi_dbg_data_case1;
	unsigned int fifo_dbg_data_case0, fifo_dbg_data_case1;
	unsigned int fifo_dbg_data_case2, fifo_dbg_data_case3;
	unsigned int debug_sel = 0, wdma_smi = 0x300, wdma_fifo = 0x0;

	char cam[10] = {'\0'};
	enum ISP_DEV_NODE_ENUM innerRegModule; /* for read/write register */
	enum ISP_DEV_NODE_ENUM outerRegModule; /* for read/write register */

	switch (module) {
	case ISP_IRQ_TYPE_INT_CAM_A_ST:
		innerRegModule = ISP_CAM_A_INNER_IDX;
		outerRegModule = ISP_CAM_A_IDX;
		strncpy(cam, "CAM_A", sizeof("CAM_A"));
		break;
	case ISP_IRQ_TYPE_INT_CAM_B_ST:
		innerRegModule = ISP_CAM_B_INNER_IDX;
		outerRegModule = ISP_CAM_B_IDX;
		strncpy(cam, "CAM_B", sizeof("CAM_B"));
		break;
	case ISP_IRQ_TYPE_INT_CAM_C_ST:
		innerRegModule = ISP_CAM_C_INNER_IDX;
		outerRegModule = ISP_CAM_C_IDX;
		strncpy(cam, "CAM_C", sizeof("CAM_C"));
		break;
	default:
		LOG_NOTICE("unsupported module:0x%x\n", module);
		return;
	}

	ISP_WR32(CAM_REG_DBG_SET(innerRegModule), 0x2);
	switch (debug_idx) {
	case BPCI_R1_DEBUG:
	case LSCI_R1_DEBUG:
	case RAWI_R2_DEBUG:
	case RAWI_R3_DEBUG:
	case CQI_R1_DEBUG:
	case CQI_R2_DEBUG:
	case BPCI_R2_DEBUG:
	case BPCI_R3_DEBUG:
	case UFDI_R2_DEBUG:
		wdma_smi = 0x0;
		wdma_fifo = 0x0;
		break;
	default:
		wdma_smi = 0x300;
		wdma_fifo = 0x100;
	}

	debug_sel = 0x400 * debug_idx + 0xB;
	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), debug_sel);
	checksum = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));

	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), (debug_sel + 0x100));
	lpix_cnt_tmp = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));

	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), (debug_sel + 0x200));
	lpix_cnt = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));

	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), (debug_sel + 0x300));
	data_cnt = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));

	debug_sel = 0x00000100 + debug_idx + wdma_smi;

	if (debug_idx > PDO_R1_DEBUG)
		debug_sel = debug_sel + 4;

	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), debug_sel + 0x80000);
	smi_dbg_data_local = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));
	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), debug_sel + 0x90000);
	smi_dbg_data_local_be = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));
	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), debug_sel);
	smi_dbg_data_case0 = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));
	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), debug_sel + 0x10000);
	smi_dbg_data_case1 = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));

	debug_sel = 0x00000200 + debug_idx + wdma_fifo;
	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), debug_sel);
	fifo_dbg_data_case0 = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));

	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), debug_sel + 0x10000);
	fifo_dbg_data_case1 = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));

	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), debug_sel + 0x20000);
	fifo_dbg_data_case2 = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));

	ISP_WR32(CAM_REG_DMA_DEBUG_SEL(innerRegModule), debug_sel + 0x30000);
	fifo_dbg_data_case3 = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));

	IRQ_LOG_KEEPER(
		module, m_CurrentPPB, _LOG_INF,
		"debug port = %d checksum=0x%x,lpix_cnt_tmp=0x%x,lpix_cnt=0x%x,data_cnt=0x%x,smi_dbg_data_local=0x%x,smi_dbg_data_local_be=0x%x,smi_dbg_data_case0=0x%x,smi_dbg_data_case1=0x%x,fifo case0=0x%x,case1=0x%x,case2=0x%x,case3=0x%x\n",
		debug_idx, checksum, lpix_cnt_tmp, lpix_cnt, data_cnt,
		smi_dbg_data_local, smi_dbg_data_local_be,
		smi_dbg_data_case0, smi_dbg_data_case0,
		fifo_dbg_data_case0, fifo_dbg_data_case1,
		fifo_dbg_data_case2, fifo_dbg_data_case3);

	/* Reset SNAPSHOT_SEL to dma_error for RAWI debug */
	ISP_WR32(CAM_REG_DBG_SET(innerRegModule), 0x00040000);
}

void dumpAllRegs(enum ISP_DEV_NODE_ENUM module)
{
	unsigned int i = 0;
	unsigned int log_ba = 0;
	enum ISP_IRQ_TYPE_ENUM irq_type = ISP_IRQ_TYPE_AMOUNT;

	if (g_is_dumping[module])
		return;

	switch (module) {
	case ISP_CAM_A_INNER_IDX:
		log_ba = CAM_A_BASE_HW;
		irq_type = ISP_IRQ_TYPE_INT_CAM_A_ST;
		break;
	case ISP_CAM_B_INNER_IDX:
		log_ba = CAM_B_BASE_HW;
		irq_type = ISP_IRQ_TYPE_INT_CAM_B_ST;
		break;
	case ISP_CAM_C_INNER_IDX:
		log_ba = CAM_C_BASE_HW;
		irq_type = ISP_IRQ_TYPE_INT_CAM_C_ST;
		break;
	default:
		break;
	}

	g_is_dumping[module] = MTRUE;
	IRQ_LOG_KEEPER(irq_type,  m_CurrentPPB, _LOG_INF,
		"----%s(module:%d)----\n", __func__, module);

	for (i = 0 ; i < ISP_REG_RANGE; i += 0x0020) {
		IRQ_LOG_KEEPER(irq_type, m_CurrentPPB, _LOG_INF, STR_REG,
			log_ba + i,
			ISP_RD32(isp_devs[module].regs + i),
			ISP_RD32(isp_devs[module].regs + i + 0x0004),
			ISP_RD32(isp_devs[module].regs + i + 0x0008),
			ISP_RD32(isp_devs[module].regs + i + 0x000C),
			ISP_RD32(isp_devs[module].regs + i + 0x0010),
			ISP_RD32(isp_devs[module].regs + i + 0x0014),
			ISP_RD32(isp_devs[module].regs + i + 0x0018),
			ISP_RD32(isp_devs[module].regs + i + 0x001C));
	}
	g_is_dumping[module] = MFALSE;
}

int STT_FBC_Reset(unsigned int reg_module)
{
	unsigned int DmaEnStatus[_cam_max_];
	union FBC_CTRL_2 fbc_ctrl2[_cam_max_ + 1];
	enum ISP_IRQ_TYPE_ENUM irq_type = ISP_IRQ_TYPE_AMOUNT;

	switch (reg_module) {
	case ISP_CAM_A_IDX:
		irq_type = ISP_IRQ_TYPE_INT_CAM_A_ST;
		break;
	case ISP_CAM_B_IDX:
		irq_type = ISP_IRQ_TYPE_INT_CAM_B_ST;
		break;
	case ISP_CAM_C_IDX:
		irq_type = ISP_IRQ_TYPE_INT_CAM_C_ST;
		break;
	default:
		break;
	}

	ISP_GetDmaPortsStatus(reg_module, &DmaEnStatus[0]);
	IRQ_LOG_KEEPER(irq_type,  m_CurrentPPB, _LOG_INF,
		"STT recover start reg_module:%d\n", reg_module);
	if (DmaEnStatus[_aao_]) {
		fbc_ctrl2[_aao_].Raw = ISP_RD32(CAM_REG_FBC_AAO_CTL2(reg_module));
		ISP_WR32(CAM_REG_FBC_AAO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_AAO_CTL1(reg_module))|0x100));
		ISP_WR32(CAM_REG_FBC_AAO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_AAO_CTL1(reg_module)) & (~(0x100))));
		ISP_WR32(CAM_REG_FBC_AAO_CTL2(reg_module), fbc_ctrl2[_aao_].Raw);
	}

	if (DmaEnStatus[_flko_]) {
		fbc_ctrl2[_flko_].Raw = ISP_RD32(CAM_REG_FBC_FLKO_CTL2(reg_module));
		ISP_WR32(CAM_REG_FBC_FLKO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_FLKO_CTL1(reg_module))|0x100));
		ISP_WR32(CAM_REG_FBC_FLKO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_FLKO_CTL1(reg_module)) & (~(0x100))));
		ISP_WR32(CAM_REG_FBC_FLKO_CTL2(reg_module), fbc_ctrl2[_flko_].Raw);
	}

	if (DmaEnStatus[_aaho_]) {
		fbc_ctrl2[_aaho_].Raw = ISP_RD32(CAM_REG_FBC_AAHO_CTL2(reg_module));
		ISP_WR32(CAM_REG_FBC_AAHO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_AAHO_CTL1(reg_module))|0x100));
		ISP_WR32(CAM_REG_FBC_AAHO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_AAHO_CTL1(reg_module)) & (~(0x100))));
		ISP_WR32(CAM_REG_FBC_AAHO_CTL2(reg_module), fbc_ctrl2[_aaho_].Raw);
	}

	if (DmaEnStatus[_afo_]) {
		fbc_ctrl2[_afo_].Raw = ISP_RD32(CAM_REG_FBC_AFO_CTL2(reg_module));
		ISP_WR32(CAM_REG_FBC_AFO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_AFO_CTL1(reg_module))|0x100));
		ISP_WR32(CAM_REG_FBC_AFO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_AFO_CTL1(reg_module)) & (~(0x100))));
		ISP_WR32(CAM_REG_FBC_AFO_CTL2(reg_module), fbc_ctrl2[_afo_].Raw);
	}

	if (DmaEnStatus[_pdo_]) {
		fbc_ctrl2[_pdo_].Raw = ISP_RD32(CAM_REG_FBC_PDO_CTL2(reg_module));
		ISP_WR32(CAM_REG_FBC_PDO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_PDO_CTL1(reg_module))|0x100));
		ISP_WR32(CAM_REG_FBC_PDO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_PDO_CTL1(reg_module)) & (~(0x100))));
		ISP_WR32(CAM_REG_FBC_PDO_CTL2(reg_module), fbc_ctrl2[_pdo_].Raw);
	}

	if (DmaEnStatus[_tsfso_]) {
		fbc_ctrl2[_tsfso_].Raw = ISP_RD32(CAM_REG_FBC_TSFSO_CTL2(reg_module));
		ISP_WR32(CAM_REG_FBC_TSFSO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_TSFSO_CTL1(reg_module))|0x100));
		ISP_WR32(CAM_REG_FBC_TSFSO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_TSFSO_CTL1(reg_module)) & (~(0x100))));
		ISP_WR32(CAM_REG_FBC_TSFSO_CTL2(reg_module), fbc_ctrl2[_tsfso_].Raw);
	}

	if (DmaEnStatus[_ltmso_]) {
		fbc_ctrl2[_ltmso_].Raw = ISP_RD32(CAM_REG_FBC_LTMSO_CTL2(reg_module));
		ISP_WR32(CAM_REG_FBC_LTMSO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_LTMSO_CTL1(reg_module))|0x100));
		ISP_WR32(CAM_REG_FBC_LTMSO_CTL1(reg_module),
			(ISP_RD32(CAM_REG_FBC_LTMSO_CTL1(reg_module)) & (~(0x100))));
		ISP_WR32(CAM_REG_FBC_LTMSO_CTL2(reg_module), fbc_ctrl2[_ltmso_].Raw);
	}
	IRQ_LOG_KEEPER(irq_type,  m_CurrentPPB, _LOG_INF,
		"STT recover done reg_module:%d\n", reg_module);
	return 0;
}

#define Rdy_ReqDump
static void ISP_DumpDmaDeepDbg(enum ISP_IRQ_TYPE_ENUM module, unsigned int ErrStatus)
{
#define NO_DMA_ERROR (0xffff0000)
#ifdef Rdy_ReqDump
#define ISP_MODULE_GROUPS 7
	unsigned int moduleReqStatus[ISP_MODULE_GROUPS];
	unsigned int moduleRdyStatus[ISP_MODULE_GROUPS];
	unsigned int i;
#endif
	unsigned int dmaerr[_cam_max_] = {0};
	char cam[10] = {'\0'};
	enum ISP_DEV_NODE_ENUM regModule; /* for read/write register */
	enum ISP_DEV_NODE_ENUM innerRegModule; /* for read/write register */
	union CAMCTL_TWIN_STATUS_ twin_status;
	unsigned int dump_all = 0;

	switch (module) {
	case ISP_IRQ_TYPE_INT_CAM_A_ST:
		regModule = ISP_CAM_A_IDX;
		innerRegModule = ISP_CAM_A_INNER_IDX;
		strncpy(cam, "CAM_A", sizeof("CAM_A"));
		break;
	case ISP_IRQ_TYPE_INT_CAM_B_ST:
		regModule = ISP_CAM_B_IDX;
		innerRegModule = ISP_CAM_B_INNER_IDX;
		strncpy(cam, "CAM_B", sizeof("CAM_B"));
		break;
	case ISP_IRQ_TYPE_INT_CAM_C_ST:
		regModule = ISP_CAM_C_IDX;
		innerRegModule = ISP_CAM_C_INNER_IDX;
		strncpy(cam, "CAM_C", sizeof("CAM_C"));
		break;
	default:
		LOG_NOTICE("unsupported module:0x%x\n", module);
		return;
	}

	if (ErrStatus & TG_ERR_ST) {
		STT_FBC_Reset(regModule);
		twin_status.Raw =
			ISP_RD32(CAM_REG_CTL_TWIN_STATUS(innerRegModule));
		IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_INF, "twin status en:%d, master:%d\n",
			twin_status.Bits.TWIN_EN,
			twin_status.Bits.MASTER_MODULE);
		if (twin_status.Bits.TWIN_EN == MTRUE) {
			for (i = 0 ; i < twin_status.Bits.SLAVE_CAM_NUM ; i++) {
				switch (i) {
				case 0:
					if (twin_status.Bits.TWIN_MODULE == CAM_B)
						STT_FBC_Reset(ISP_CAM_B_IDX);
					else if (twin_status.Bits.TWIN_MODULE == CAM_C)
						STT_FBC_Reset(ISP_CAM_C_IDX);
					break;
				case 1:
					LOG_INF("2nd slave%d cam:%d\n", i,
						twin_status.Bits.TRIPLE_MODULE);
						STT_FBC_Reset(ISP_CAM_C_IDX);
					break;
				default:
					LOG_INF("unexpected slave cam\n");
					break;
				}
			}
		}
	}

	/* DMAO */
	dmaerr[_imgo_] =
		(unsigned int)ISP_RD32(CAM_REG_IMGO_ERR_STAT(regModule));

	dmaerr[_ltmso_] =
		(unsigned int)ISP_RD32(CAM_REG_LTMSO_ERR_STAT(regModule));

	dmaerr[_rrzo_] =
		(unsigned int)ISP_RD32(CAM_REG_RRZO_ERR_STAT(regModule));

	dmaerr[_lcso_] =
		(unsigned int)ISP_RD32(CAM_REG_LCESO_ERR_STAT(regModule));

	dmaerr[_lcesho_] =
		(unsigned int)ISP_RD32(CAM_REG_LCESHO_ERR_STAT(regModule));

	dmaerr[_aao_] =
		(unsigned int)ISP_RD32(CAM_REG_AAO_ERR_STAT(regModule));

	dmaerr[_aaho_] =
		(unsigned int)ISP_RD32(CAM_REG_AAHO_ERR_STAT(regModule));

	dmaerr[_flko_] =
		(unsigned int)ISP_RD32(CAM_REG_FLKO_ERR_STAT(regModule));

	dmaerr[_ufeo_] =
		(unsigned int)ISP_RD32(CAM_REG_UFEO_ERR_STAT(regModule));

	dmaerr[_afo_] =
		(unsigned int)ISP_RD32(CAM_REG_AFO_ERR_STAT(regModule));

	dmaerr[_ufgo_] =
		(unsigned int)ISP_RD32(CAM_REG_UFGO_ERR_STAT(regModule));

	dmaerr[_rsso_] =
		(unsigned int)ISP_RD32(CAM_REG_RSSO_A_ERR_STAT(regModule));

	dmaerr[_lmvo_] =
		(unsigned int)ISP_RD32(CAM_REG_LMVO_ERR_STAT(regModule));

	dmaerr[_yuvbo_] =
		(unsigned int)ISP_RD32(CAM_REG_YUVBO_ERR_STAT(regModule));

	dmaerr[_tsfso_] =
		(unsigned int)ISP_RD32(CAM_REG_TSFSO_ERR_STAT(regModule));

	dmaerr[_pdo_] =
		(unsigned int)ISP_RD32(CAM_REG_PDO_ERR_STAT(regModule));

	dmaerr[_crzo_] =
		(unsigned int)ISP_RD32(CAM_REG_CRZO_ERR_STAT(regModule));

	dmaerr[_crzbo_] =
		(unsigned int)ISP_RD32(CAM_REG_CRZBO_ERR_STAT(regModule));

	dmaerr[_yuvco_] =
		(unsigned int)ISP_RD32(CAM_REG_YUVCO_ERR_STAT(regModule));

	dmaerr[_crzo_r2_] =
		(unsigned int)ISP_RD32(CAM_REG_CRZO_R2_ERR_STAT(regModule));

	dmaerr[_rsso_r2_] =
		(unsigned int)ISP_RD32(CAM_REG_RSSO_R2_ERR_STAT(regModule));

	dmaerr[_yuvo_] =
		(unsigned int)ISP_RD32(CAM_REG_YUVO_ERR_STAT(regModule));

	/* DMAI */
	dmaerr[_rawi_] =
		(unsigned int)ISP_RD32(CAM_REG_RAWI_R2_ERR_STAT(regModule));

	dmaerr[_rawi_r3_] =
		(unsigned int)ISP_RD32(CAM_REG_RAWI_R3_ERR_STAT(regModule));

	dmaerr[_bpci_] =
		(unsigned int)ISP_RD32(CAM_REG_BPCI_ERR_STAT(regModule));

	dmaerr[_lsci_] =
		(unsigned int)ISP_RD32(CAM_REG_LSCI_ERR_STAT(regModule));

	dmaerr[_bpci_r2_] =
		(unsigned int)ISP_RD32(CAM_REG_BPCI_R2_ERR_STAT(regModule));

	dmaerr[_bpci_r3_] =
		(unsigned int)ISP_RD32(CAM_REG_BPCI_R3_ERR_STAT(regModule));

	dmaerr[_pdi_] =
		(unsigned int)ISP_RD32(CAM_REG_PDI_ERR_STAT(regModule));

	dmaerr[_ufdi_r2_] =
		(unsigned int)ISP_RD32(CAM_REG_UFDI_R2_ERR_STAT(regModule));

	IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR, "camsys:0x%x",
		       ISP_RD32(ISP_CAMSYS_CONFIG_BASE));

	if ((dmaerr[_imgo_] | dmaerr[_ltmso_] | dmaerr[_rrzo_] |
		dmaerr[_lcso_] | dmaerr[_lcesho_] | dmaerr[_aao_]) != NO_DMA_ERROR)
		IRQ_LOG_KEEPER(
			module, m_CurrentPPB, _LOG_ERR,
			"%s:IMGO=0x%x,LTMSO=0x%x,RRZO=0x%x,LCSO=0x%x,LCESHO=0x%x,AAO=0x%x\n",
			cam, dmaerr[_imgo_], dmaerr[_ltmso_], dmaerr[_rrzo_],
			dmaerr[_lcso_], dmaerr[_lcesho_], dmaerr[_aao_]);

	if ((dmaerr[_aaho_] | dmaerr[_flko_] | dmaerr[_ufeo_] |
		dmaerr[_afo_] | dmaerr[_ufgo_] | dmaerr[_rsso_]) != NO_DMA_ERROR)
		IRQ_LOG_KEEPER(
			module, m_CurrentPPB, _LOG_ERR,
			"AAHO=0x%x,FLKO=0x%x,UFEO=0x%x,AFO=0x%x,UFGO=0x%x,RSSO=0x%x\n",
			dmaerr[_aaho_], dmaerr[_flko_], dmaerr[_ufeo_], dmaerr[_afo_],
			dmaerr[_ufgo_], dmaerr[_rsso_]);

	if ((dmaerr[_lmvo_] | dmaerr[_yuvbo_] | dmaerr[_tsfso_] |
		dmaerr[_pdo_] | dmaerr[_crzo_] | dmaerr[_crzbo_]) != NO_DMA_ERROR)
		IRQ_LOG_KEEPER(
			module, m_CurrentPPB, _LOG_ERR,
			"EISO=0x%x,YUVBO=0x%x,TSFSO=0x%x,PDO=0x%x,CRZO=0x%x,CRZBO=0x%x\n",
			dmaerr[_lmvo_], dmaerr[_yuvbo_], dmaerr[_tsfso_], dmaerr[_pdo_],
			dmaerr[_crzo_], dmaerr[_crzbo_]);

	if ((dmaerr[_yuvco_] | dmaerr[_crzo_r2_] |
		dmaerr[_rsso_r2_] | dmaerr[_yuvo_]) != NO_DMA_ERROR)
		IRQ_LOG_KEEPER(
			module, m_CurrentPPB, _LOG_ERR,
			"YUVCO=0x%x,CRZO_R2=0x%x,RSSO_R2=0x%x,YUVO=0x%x\n",
			dmaerr[_yuvco_], dmaerr[_crzo_r2_], dmaerr[_rsso_r2_],
			dmaerr[_yuvo_]);

	if ((dmaerr[_rawi_] | dmaerr[_rawi_r3_]) != NO_DMA_ERROR)
		IRQ_LOG_KEEPER(
			module, m_CurrentPPB, _LOG_ERR,
			"DMA_DBG_SEL=0x%x,TOP_DBG_PORT=0x%x,RAWI_R2=0x%x,RAWI_R3=0x%x\n",
			(unsigned int)ISP_RD32(CAM_REG_DMA_DEBUG_SEL(regModule)), 0,
			dmaerr[_rawi_], dmaerr[_rawi_r3_]);

	if ((dmaerr[_bpci_] | dmaerr[_lsci_] |
		dmaerr[_bpci_r2_] | dmaerr[_pdi_] | dmaerr[_ufdi_r2_]) != NO_DMA_ERROR)
		IRQ_LOG_KEEPER(
			module, m_CurrentPPB, _LOG_ERR,
			"%s:BPCI=0x%x,LSCI=0x%x,BPCI_R2=0x%x,PDI=0x%x,UFDI_R2=0x%x\n",
			cam, dmaerr[_bpci_], dmaerr[_lsci_],
			dmaerr[_bpci_r2_], dmaerr[_pdi_], dmaerr[_ufdi_r2_]);

	/* DMAO */
	g_DmaErr_CAM[module][_imgo_] |= dmaerr[_imgo_];
	g_DmaErr_CAM[module][_ltmso_] |= dmaerr[_ltmso_];
	g_DmaErr_CAM[module][_rrzo_] |= dmaerr[_rrzo_];
	g_DmaErr_CAM[module][_lcso_] |= dmaerr[_lcso_];
	g_DmaErr_CAM[module][_lcesho_] |= dmaerr[_lcesho_];
	g_DmaErr_CAM[module][_aao_] |= dmaerr[_aao_];
	g_DmaErr_CAM[module][_aaho_] |= dmaerr[_aaho_];
	g_DmaErr_CAM[module][_flko_] |= dmaerr[_flko_];
	g_DmaErr_CAM[module][_ufeo_] |= dmaerr[_ufeo_];
	g_DmaErr_CAM[module][_afo_] |= dmaerr[_afo_];
	g_DmaErr_CAM[module][_ufgo_] |= dmaerr[_ufgo_];
	g_DmaErr_CAM[module][_rsso_] |= dmaerr[_rsso_];
	g_DmaErr_CAM[module][_lmvo_] |= dmaerr[_lmvo_];
	g_DmaErr_CAM[module][_yuvbo_] |= dmaerr[_yuvbo_];
	g_DmaErr_CAM[module][_tsfso_] |= dmaerr[_tsfso_];
	g_DmaErr_CAM[module][_pdo_] |= dmaerr[_pdo_];
	g_DmaErr_CAM[module][_crzo_] |= dmaerr[_crzo_];
	g_DmaErr_CAM[module][_crzbo_] |= dmaerr[_crzbo_];
	g_DmaErr_CAM[module][_yuvco_] |= dmaerr[_yuvco_];
	g_DmaErr_CAM[module][_crzo_r2_] |= dmaerr[_crzo_r2_];
	g_DmaErr_CAM[module][_rsso_r2_] |= dmaerr[_rsso_r2_];
	g_DmaErr_CAM[module][_yuvo_] |= dmaerr[_yuvo_];
	/* DMAI */
	g_DmaErr_CAM[module][_rawi_] |= dmaerr[_rawi_];
	g_DmaErr_CAM[module][_bpci_] |= dmaerr[_bpci_];
	g_DmaErr_CAM[module][_lsci_] |= dmaerr[_lsci_];
	g_DmaErr_CAM[module][_bpci_r2_] |= dmaerr[_bpci_r2_];
	g_DmaErr_CAM[module][_bpci_r3_] |= dmaerr[_bpci_r3_];
	g_DmaErr_CAM[module][_pdi_] |= dmaerr[_pdi_];
	g_DmaErr_CAM[module][_ufdi_r2_] |= dmaerr[_ufdi_r2_];

#ifdef Rdy_ReqDump
	/* Module DebugInfo when no p1_done */

	for (i = 0; i < ISP_MODULE_GROUPS; i++) {
		ISP_WR32(CAM_REG_DBG_SET(innerRegModule),
			 (0x00040101 + (i * 0x100)));
		moduleReqStatus[i] = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));
	}

	for (i = 0; i < ISP_MODULE_GROUPS; i++) {
		ISP_WR32(CAM_REG_DBG_SET(innerRegModule),
			 (0x00041101 + (i * 0x100)));
		moduleRdyStatus[i] = ISP_RD32(CAM_REG_DBG_PORT(innerRegModule));
	}

	IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
		"%s Module[%d] Req/Rdy Status: 0=(0x%08x 0x%08x)",
		cam, innerRegModule,
		moduleReqStatus[0], moduleRdyStatus[0]);
	IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
		"1=(0x%08x 0x%08x), 2=(0x%08x 0x%08x)",
		moduleReqStatus[1], moduleRdyStatus[1],
		moduleReqStatus[2], moduleRdyStatus[2]);
	IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
		"3=(0x%08x 0x%08x), 4=(0x%08x 0x%08x)",
		moduleReqStatus[3], moduleRdyStatus[3],
		moduleReqStatus[4], moduleRdyStatus[4]);
	IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
		"5=(0x%08x 0x%08x), 6=(0x%08x 0x%08x)",
		moduleReqStatus[5], moduleRdyStatus[5],
		moduleReqStatus[6], moduleRdyStatus[6]);

	if (dmaerr[_imgo_] != 0xFFFF0000) {
		ISP_DumpDebugData(module, IMGO_R1_DEBUG);
		dump_all = 1;
	}

	if (dmaerr[_ltmso_] != 0xFFFF0000) {
		ISP_DumpDebugData(module, LTMSO_R1_DEBUG);
		dump_all = 1;
	}

	if (dmaerr[_rrzo_] != 0xFFFF0000) {
		ISP_DumpDebugData(module, RRZO_R1_DEBUG);
		dump_all = 1;
	}

	if (dmaerr[_lcesho_] != 0xFFFF0000) {
		ISP_DumpDebugData(module, LCESO_R1_DEBUG);
		dump_all = 1;
	}

	if (dmaerr[_rawi_] != 0xFFFF0000) {
		ISP_DumpDebugData(module, RAWI_R2_DEBUG);
		dump_all = 1;
	}

	if (ErrStatus & TG_ERR_ST) {
		dump_all = 1;
	}

	if (dump_all) {

		/* single/master case*/
		dumpAllRegs(innerRegModule);

		/* twin case*/
		if (twin_status.Bits.TWIN_EN == MTRUE) {
			for (i = 0 ; i < twin_status.Bits.SLAVE_CAM_NUM ; i++) {
				switch (i) {
				case 0:
					IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_INF,
						"1st slave%d cam:%d\n", i,
						twin_status.Bits.TWIN_MODULE);
					if (twin_status.Bits.TWIN_MODULE ==
									CAM_B)
						innerRegModule =
							ISP_CAM_B_INNER_IDX;
					else if (twin_status.Bits.TWIN_MODULE ==
									CAM_C)
						innerRegModule =
							ISP_CAM_C_INNER_IDX;
					dumpAllRegs(innerRegModule);
					break;
				case 1:
					IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_INF,
						"2nd slave%d cam:%d\n", i,
						twin_status.Bits.TRIPLE_MODULE);
					innerRegModule = ISP_CAM_C_INNER_IDX;
					dumpAllRegs(innerRegModule);
					break;
				default:
					LOG_INF("unexpected slave cam\n");
					break;
				}
			}
		}
	}
#undef ISP_MODULE_GROUPS
#endif
}

static inline void Prepare_Enable_ccf_clock(enum ISP_DEV_NODE_ENUM module)
{
	int ret;

	/* must keep this clk open order: */
	/* ISP PM domain -> CAMTG/CAMSV clock */
#ifdef USE_MTK_SMI_LARB_API

	if (module == ISP_CAM_A_IDX) {
		if (isp_devs[ISP_CAM_A_IDX].larb) {
			ret = mtk_smi_larb_get(isp_devs[ISP_CAM_A_IDX].larb);
			if (ret)
				LOG_NOTICE("mtk_smi_larb_get cam a larb fail %d\n", ret);
		} else
			LOG_NOTICE("No larb device for cam a\n");
	}

	if (module == ISP_CAM_B_IDX) {
		if (isp_devs[ISP_CAM_B_IDX].larb) {
			ret = mtk_smi_larb_get(isp_devs[ISP_CAM_B_IDX].larb);
			if (ret)
				LOG_NOTICE("mtk_smi_larb_get cam b larb fail %d\n", ret);
		} else
			LOG_NOTICE("No larb device for cam b\n");
	}

	if (module == ISP_CAM_C_IDX) {
		if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
			if (isp_devs[ISP_CAM_C_IDX].larb) {
				ret = mtk_smi_larb_get(isp_devs[ISP_CAM_C_IDX].larb);
				if (ret)
					LOG_NOTICE("mtk_smi_larb_get cam c larb fail %d\n", ret);
			} else
				LOG_NOTICE("No larb device for cam c\n");
		}
	}

	if (module >= ISP_CAMSV1_IDX) {
		if (isp_devs[ISP_CAMSYS_CONFIG_IDX].larb) {
			ret = mtk_smi_larb_get(isp_devs[ISP_CAMSYS_CONFIG_IDX].larb);
			if (ret)
				LOG_NOTICE("mtk_smi_larb_get CAMSYS larb fail %d\n", ret);
		} else
			LOG_NOTICE("No larb device for camsys\n");
	}

	if (module == ISP_CAMSV0_IDX) {
		if (isp_devs[ISP_CAMSYS_CONFIG_IDX].larb_2nd) {
			ret = mtk_smi_larb_get(isp_devs[ISP_CAMSYS_CONFIG_IDX].larb_2nd);
			if (ret)
				LOG_NOTICE("mtk_smi_larb_get CAMSYS larb_2nd fail %d\n", ret);
		} else
			LOG_NOTICE("No 2nd larb device for camsys\n");
	}
#endif
	ret = pm_runtime_get_sync(isp_devs[ISP_CAMSYS_CONFIG_IDX].dev);
	if (ret < 0)
		LOG_NOTICE("cannot pm runtime get ISP_CAMSYS_CONFIG_IDX mtcmos\n");

	if (module == ISP_CAM_A_IDX) {
		ret = pm_runtime_get_sync(isp_devs[ISP_CAM_A_IDX].dev);
		if (ret < 0)
			LOG_NOTICE("cannot pm runtime get ISP_CAM_A_IDX mtcmos\n");
	}

	if (module == ISP_CAM_B_IDX) {
		ret = pm_runtime_get_sync(isp_devs[ISP_CAM_B_IDX].dev);
		if (ret < 0)
			LOG_NOTICE("cannot pm runtime get ISP_CAM_B_IDX mtcmos\n");
	}

	if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
		if (module == ISP_CAM_C_IDX) {
			ret = pm_runtime_get_sync(isp_devs[ISP_CAM_C_IDX].dev);
			if (ret < 0)
				LOG_NOTICE("cannot pm runtime get ISP_CAM_C_IDX mtcmos\n");
		}
	}

	if (module >= ISP_CAMSV1_IDX) {
		ret = clk_prepare_enable(isp_clk.CAMSYS_LARB13_CGPDN);
		if (ret)
			LOG_NOTICE("cannot pre-en CAMSYS_LARB13_CGPDN clock\n");
	}

	if (module == ISP_CAMSV0_IDX) {
		ret = clk_prepare_enable(isp_clk.CAMSYS_LARB14_CGPDN);
		if (ret)
			LOG_NOTICE("cannot pre-en CAMSYS_LARB14_CGPDN clock\n");
	}

	if (IS_MT6893(g_platform_id)) {
		ret = clk_prepare_enable(isp_clk.CAMSYS_LARB15_CGPDN);
		if (ret)
			LOG_NOTICE("cannot pre-en CAMSYS_LARB15_CGPDN clock\n");
	}

	ret = clk_prepare_enable(isp_clk.CAMSYS_SENINF_CGPDN);
	if (ret)
		LOG_NOTICE("cannot pre-en CAMSYS_SENINF_CGPDN clock\n");

	if (IS_MT6873(g_platform_id) || IS_MT6853(g_platform_id)) {
		ret = clk_prepare_enable(isp_clk.CAMSYS_CAM2MM_GALS_CGPDN);
		if (ret)
			LOG_NOTICE("cannot pre-en CAMSYS_CAM2MM_GALS_CGPDN clock\n");
	}

	ret = clk_prepare_enable(isp_clk.CAMSYS_TOP_MUX_CCU);
	if (ret)
		LOG_NOTICE("cannot pre-en CAMSYS_TOP_MUX_CCU clock\n");

	ret = clk_prepare_enable(isp_clk.CAMSYS_CCU0_CGPDN);
	if (ret)
		LOG_NOTICE("cannot pre-en CAMSYS_CCU0_CGPDN clock\n");

	ret = clk_prepare_enable(isp_clk.ISP_CAM_CAMSYS);
	if (ret)
		LOG_NOTICE("cannot pre-en ISP_CAM_CAMSYS clock\n");

	ret = clk_prepare_enable(isp_clk.ISP_CAM_CAMTG);
	if (ret)
		LOG_NOTICE("cannot pre-en ISP_CAM_CAMTG clock\n");

	if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
		if (module == ISP_CAMSV0_IDX || module == ISP_CAMSV1_IDX) {
			ret = clk_prepare_enable(isp_clk.ISP_CAM_CAMSV0);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_CAMSV0 clock\n");
		}

		if (module == ISP_CAMSV2_IDX || module == ISP_CAMSV3_IDX) {
			ret = clk_prepare_enable(isp_clk.ISP_CAM_CAMSV1);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_CAMSV1 clock\n");
		}

		if (module == ISP_CAMSV4_IDX || module == ISP_CAMSV5_IDX) {
			ret = clk_prepare_enable(isp_clk.ISP_CAM_CAMSV2);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_CAMSV2 clock\n");
		}

		if (module == ISP_CAMSV6_IDX || module == ISP_CAMSV7_IDX) {
			ret = clk_prepare_enable(isp_clk.ISP_CAM_CAMSV3);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_CAMSV3 clock\n");
		}
	} else {
		/* DISABLE_SV_TOP0 */
		if (module == ISP_CAMSV2_IDX || module == ISP_CAMSV3_IDX) {
			ret = clk_prepare_enable(isp_clk.ISP_CAM_CAMSV0);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_CAMSV0 clock\n");
		}

		if (module == ISP_CAMSV4_IDX || module == ISP_CAMSV5_IDX) {
			ret = clk_prepare_enable(isp_clk.ISP_CAM_CAMSV1);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_CAMSV1 clock\n");
		}

		if (module == ISP_CAMSV6_IDX || module == ISP_CAMSV7_IDX) {
			ret = clk_prepare_enable(isp_clk.ISP_CAM_CAMSV2);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_CAMSV2 clock\n");
		}
	}

	if (module == ISP_CAM_A_IDX) {
		ret = clk_prepare_enable(isp_clk.ISP_CAM_LARB16_RAWA);
		if (ret)
			LOG_NOTICE("cannot pre-en ISP_CAM_LARB16_RAWA clock\n");
		ret = clk_prepare_enable(isp_clk.ISP_CAM_SUBSYS_RAWA);
		if (ret)
			LOG_NOTICE("cannot pre-en ISP_CAM_SUBSYS_RAWA clock\n");
		ret = clk_prepare_enable(isp_clk.ISP_CAM_TG_RAWA);
		if (ret)
			LOG_NOTICE("cannot pre-en ISP_CAM_TG_RAWA clock\n");
	}
	if (module == ISP_CAM_B_IDX) {
		ret = clk_prepare_enable(isp_clk.ISP_CAM_LARB17_RAWB);
		if (ret)
			LOG_NOTICE("cannot pre-en ISP_CAM_LARB17_RAWB clock\n");
		ret = clk_prepare_enable(isp_clk.ISP_CAM_SUBSYS_RAWB);
		if (ret)
			LOG_NOTICE("cannot pre-en ISP_CAM_SUBSYS_RAWB clock\n");
		ret = clk_prepare_enable(isp_clk.ISP_CAM_TG_RAWB);
		if (ret)
			LOG_NOTICE("cannot pre-en ISP_CAM_TG_RAWB clock\n");
	}

	if (module == ISP_CAM_C_IDX) {
		if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
			ret = clk_prepare_enable(isp_clk.ISP_CAM_LARB18_RAWC);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_LARB18_RAWC clock\n");
			ret = clk_prepare_enable(isp_clk.ISP_CAM_SUBSYS_RAWC);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_SUBSYS_RAWC clock\n");
			ret = clk_prepare_enable(isp_clk.ISP_CAM_TG_RAWC);
			if (ret)
				LOG_NOTICE("cannot pre-en ISP_CAM_TG_RAWC clock\n");
		}
	}

	ret = clk_prepare_enable(isp_clk.ISP_TOP_MUX_CAMTM);
	if (ret)
		LOG_NOTICE("cannot pre-en ISP_TOP_MUX_CAMTM clock\n");

}

static inline void Disable_Unprepare_ccf_clock(enum ISP_DEV_NODE_ENUM module)
{
	int ret = 0;
	/* must keep this clk close order: */
	/* CAMTG/CAMSV clock -> ISP PM domain */
	clk_disable_unprepare(isp_clk.ISP_TOP_MUX_CAMTM);

	if (module == ISP_CAM_C_IDX) {
		if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
			clk_disable_unprepare(isp_clk.ISP_CAM_TG_RAWC);
			clk_disable_unprepare(isp_clk.ISP_CAM_SUBSYS_RAWC);
			clk_disable_unprepare(isp_clk.ISP_CAM_LARB18_RAWC);
		}
	}

	if (module == ISP_CAM_B_IDX) {
		clk_disable_unprepare(isp_clk.ISP_CAM_TG_RAWB);
		clk_disable_unprepare(isp_clk.ISP_CAM_SUBSYS_RAWB);
		clk_disable_unprepare(isp_clk.ISP_CAM_LARB17_RAWB);
	}

	if (module == ISP_CAM_A_IDX) {
		clk_disable_unprepare(isp_clk.ISP_CAM_TG_RAWA);
		clk_disable_unprepare(isp_clk.ISP_CAM_SUBSYS_RAWA);
		clk_disable_unprepare(isp_clk.ISP_CAM_LARB16_RAWA);
	}

	if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
		if (module == ISP_CAMSV0_IDX || module == ISP_CAMSV1_IDX)
			clk_disable_unprepare(isp_clk.ISP_CAM_CAMSV0);
		if (module == ISP_CAMSV2_IDX || module == ISP_CAMSV3_IDX)
			clk_disable_unprepare(isp_clk.ISP_CAM_CAMSV1);
		if (module == ISP_CAMSV4_IDX || module == ISP_CAMSV5_IDX)
			clk_disable_unprepare(isp_clk.ISP_CAM_CAMSV2);
		if (module == ISP_CAMSV6_IDX || module == ISP_CAMSV7_IDX)
			clk_disable_unprepare(isp_clk.ISP_CAM_CAMSV3);
	} else {
		/* DISABLE_SV_TOP0 */
		if (module == ISP_CAMSV2_IDX || module == ISP_CAMSV3_IDX)
			clk_disable_unprepare(isp_clk.ISP_CAM_CAMSV0);
		if (module == ISP_CAMSV4_IDX || module == ISP_CAMSV5_IDX)
			clk_disable_unprepare(isp_clk.ISP_CAM_CAMSV1);
		if (module == ISP_CAMSV6_IDX || module == ISP_CAMSV7_IDX)
			clk_disable_unprepare(isp_clk.ISP_CAM_CAMSV2);
	}


	clk_disable_unprepare(isp_clk.ISP_CAM_CAMTG);
	clk_disable_unprepare(isp_clk.ISP_CAM_CAMSYS);
	clk_disable_unprepare(isp_clk.CAMSYS_CCU0_CGPDN);
	clk_disable_unprepare(isp_clk.CAMSYS_TOP_MUX_CCU);

	if (IS_MT6873(g_platform_id) || IS_MT6853(g_platform_id))
		clk_disable_unprepare(isp_clk.CAMSYS_CAM2MM_GALS_CGPDN);

	clk_disable_unprepare(isp_clk.CAMSYS_SENINF_CGPDN);

	if (IS_MT6893(g_platform_id))
		clk_disable_unprepare(isp_clk.CAMSYS_LARB15_CGPDN);

	if (module == ISP_CAMSV0_IDX)
		clk_disable_unprepare(isp_clk.CAMSYS_LARB14_CGPDN);
	if (module >= ISP_CAMSV1_IDX)
		clk_disable_unprepare(isp_clk.CAMSYS_LARB13_CGPDN);

	if (module == ISP_CAM_C_IDX) {
		if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
			ret = pm_runtime_put_sync(isp_devs[ISP_CAM_C_IDX].dev);
			if (ret < 0)
				LOG_NOTICE("cannot pm runtime put ISP_CAM_C_IDX mtcmos\n");
		}
	}

	if (module == ISP_CAM_B_IDX) {
		ret = pm_runtime_put_sync(isp_devs[ISP_CAM_B_IDX].dev);
		if (ret < 0)
			LOG_NOTICE("cannot pm runtime put ISP_CAM_B_IDX mtcmos\n");
	}

	if (module == ISP_CAM_A_IDX) {
		ret = pm_runtime_put_sync(isp_devs[ISP_CAM_A_IDX].dev);
		if (ret < 0)
			LOG_NOTICE("cannot pm runtime put ISP_CAM_A_IDX mtcmos\n");
	}

	ret = pm_runtime_put_sync(isp_devs[ISP_CAMSYS_CONFIG_IDX].dev);
	if (ret < 0)
		LOG_NOTICE("cannot pm runtime put ISP_CAMSYS_CONFIG_IDX mtcmos\n");

#ifdef USE_MTK_SMI_LARB_API
	if (module == ISP_CAM_C_IDX) {
		if (isp_devs[ISP_CAM_C_IDX].larb)
			mtk_smi_larb_put(isp_devs[ISP_CAM_C_IDX].larb);
		else
			LOG_NOTICE("isp_devs[ISP_CAM_C_IDX].larb is NULL!\n");
	}
	if (module == ISP_CAM_B_IDX) {
		if (isp_devs[ISP_CAM_B_IDX].larb)
			mtk_smi_larb_put(isp_devs[ISP_CAM_B_IDX].larb);
		else
			LOG_NOTICE("isp_devs[ISP_CAM_B_IDX].larb is NULL!\n");
	}
	if (module == ISP_CAM_A_IDX) {
		if (isp_devs[ISP_CAM_A_IDX].larb)
			mtk_smi_larb_put(isp_devs[ISP_CAM_A_IDX].larb);
		else
			LOG_NOTICE("isp_devs[ISP_CAM_A_IDX].larb is NULL!\n");
	}
	if (module == ISP_CAMSV0_IDX) {
		if (isp_devs[ISP_CAMSYS_CONFIG_IDX].larb_2nd)
			mtk_smi_larb_put(isp_devs[ISP_CAMSYS_CONFIG_IDX].larb_2nd);
		else
			LOG_NOTICE("isp_devs[ISP_CAMSYS_CONFIG_IDX].larb_2nd is NULL!\n");
	}
	if (module >= ISP_CAMSV1_IDX) {
		if (isp_devs[ISP_CAMSYS_CONFIG_IDX].larb)
			mtk_smi_larb_put(isp_devs[ISP_CAMSYS_CONFIG_IDX].larb);
		else
			LOG_NOTICE("isp_devs[ISP_CAMSYS_CONFIG_IDX].larb is NULL!\n");
	}
#endif
}

/*******************************************************************************
 *
 ******************************************************************************/

void ISP_Halt_Mask(unsigned int isphaltMask)
{
	unsigned int setReg;

	setReg = (ISP_RD32(ISP_CAMSYS_CONFIG_BASE + 0x120) &
		  ~((unsigned int)(1 << (isphaltMask))));

	ISP_WR32(ISP_CAMSYS_CONFIG_BASE + 0x120, setReg);

	LOG_INF("ISP halt_en for dvfs:0x%x\n",
		ISP_RD32(ISP_CAMSYS_CONFIG_BASE + 0x120));
}
EXPORT_SYMBOL(ISP_Halt_Mask);

/*******************************************************************************
 *
 ******************************************************************************/
static void ISP_ConfigDMAControl(enum ISP_DEV_NODE_ENUM module)
{
	if (module >= ISP_CAMSV_START_IDX) {
		LOG_DBG("+ unsupport module:%d", module);
		return;
	}

	/* WDMA */
	if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
		ISP_WR32(CAM_REG_IMGO_CON(module), 0x80000800);//IMGO 2048
		ISP_WR32(CAM_REG_IMGO_CON2(module), 0x04000300);//1/2~3/8
		ISP_WR32(CAM_REG_IMGO_CON3(module), 0x02000100);//1/4~1/8
		ISP_WR32(CAM_REG_IMGO_DRS(module), 0x85550455);//2/3~13/24

		ISP_WR32(CAM_REG_RRZO_CON(module), 0x80000500);//RRZO 1280
		ISP_WR32(CAM_REG_RRZO_CON2(module), 0x028001E0);//1/2~3/8
		ISP_WR32(CAM_REG_RRZO_CON3(module), 0x014000A0);//1/4~1/8
		ISP_WR32(CAM_REG_RRZO_DRS(module), 0x835502B5);//2/3~13/24
	} else if (IS_MT6853(g_platform_id)) {
		ISP_WR32(CAM_REG_IMGO_CON(module), 0x80000600);//IMGO 1536
		ISP_WR32(CAM_REG_IMGO_CON2(module), 0x03000300);//1/2~3/8
		ISP_WR32(CAM_REG_IMGO_CON3(module), 0x01800180);//1/4~1/8
		ISP_WR32(CAM_REG_IMGO_DRS(module), 0x84000400);//2/3~13/24

		ISP_WR32(CAM_REG_RRZO_CON(module), 0x80000300);//RRZO 768
		ISP_WR32(CAM_REG_RRZO_CON2(module), 0x01800180);//1/2~3/8
		ISP_WR32(CAM_REG_RRZO_CON3(module), 0x00C000C0);//1/4~1/8
		ISP_WR32(CAM_REG_RRZO_DRS(module), 0x82000200);//2/3~13/24
	}

	ISP_WR32(CAM_REG_PDO_CON(module), 0x80000060);//PDO 96
	ISP_WR32(CAM_REG_PDO_CON2(module), 0x00300024);//1/2~3/8
	ISP_WR32(CAM_REG_PDO_CON3(module), 0x0018000C);//1/4~1/8
	ISP_WR32(CAM_REG_PDO_DRS(module), 0x80400034);//2/3~13/24

	ISP_WR32(CAM_REG_TSFSO_CON(module), 0x80000080);//TSFSO 128
	ISP_WR32(CAM_REG_TSFSO_CON2(module), 0x00400030);//1/2~3/8
	ISP_WR32(CAM_REG_TSFSO_CON3(module), 0x00200010);//1/4~1/8
	ISP_WR32(CAM_REG_TSFSO_DRS(module), 0x80550045);//2/3~13/24

	ISP_WR32(CAM_REG_AAO_CON(module), 0x80000100);//AAO 256
	ISP_WR32(CAM_REG_AAO_CON2(module), 0x00800060);//1/2~3/8
	ISP_WR32(CAM_REG_AAO_CON3(module), 0x00400020);//1/4~1/8
	ISP_WR32(CAM_REG_AAO_DRS(module), 0x80AA008A);//2/3~13/24

	ISP_WR32(CAM_REG_AAHO_CON(module), 0x80000080);//AAHO 128
	ISP_WR32(CAM_REG_AAHO_CON2(module), 0x00000000);
	ISP_WR32(CAM_REG_AAHO_CON3(module), 0x00000000);
	ISP_WR32(CAM_REG_AAHO_DRS(module), 0x80000000);//1/4~1/8

	ISP_WR32(CAM_REG_AFO_CON(module), 0x80000180);//AFO 384
	ISP_WR32(CAM_REG_AFO_CON2(module), 0x00C00090);//1/2~3/8
	ISP_WR32(CAM_REG_AFO_CON3(module), 0x00600030);//1/4~1/8
	ISP_WR32(CAM_REG_AFO_DRS(module), 0x809000D0);//2/3~13/24

	ISP_WR32(CAM_REG_FLKO_CON(module), 0x80000020);//FLKO 32
	ISP_WR32(CAM_REG_FLKO_CON2(module), 0x0010000C);//1/2~3/8
	ISP_WR32(CAM_REG_FLKO_CON3(module), 0x00080004);//1/4~1/8
	ISP_WR32(CAM_REG_FLKO_DRS(module), 0x800C0011);//2/3~13/24

	ISP_WR32(CAM_REG_LTMSO_CON(module), 0x80000080);//LTMSO 128
	ISP_WR32(CAM_REG_LTMSO_CON2(module), 0x00400030);
	ISP_WR32(CAM_REG_LTMSO_CON3(module), 0x00200010);
	ISP_WR32(CAM_REG_LTMSO_DRS(module), 0x80550045);

	ISP_WR32(CAM_REG_LCESO_CON(module), 0x80000040);//LCESO 64
	ISP_WR32(CAM_REG_LCESO_CON2(module), 0x00200018);
	ISP_WR32(CAM_REG_LCESO_CON3(module), 0x00100008);
	ISP_WR32(CAM_REG_LCESO_DRS(module), 0x802A0022);

	ISP_WR32(CAM_REG_LCESHO_CON(module), 0x80000040);//LCESO 64
	ISP_WR32(CAM_REG_LCESHO_CON2(module), 0x00200018);
	ISP_WR32(CAM_REG_LCESHO_CON3(module), 0x00100008);
	ISP_WR32(CAM_REG_LCESHO_DRS(module), 0x802A0022);

	ISP_WR32(CAM_REG_RSSO_CON(module), 0x80000060);//RSSO 96
	ISP_WR32(CAM_REG_RSSO_CON2(module), 0x00300024);
	ISP_WR32(CAM_REG_RSSO_CON3(module), 0x0018000C);
	ISP_WR32(CAM_REG_RSSO_DRS(module), 0x80400034);

	ISP_WR32(CAM_REG_LMVO_CON(module), 0x80000020);//LMVO 32
	ISP_WR32(CAM_REG_LMVO_CON2(module), 0x0010000C);
	ISP_WR32(CAM_REG_LMVO_CON3(module), 0x00080004);
	ISP_WR32(CAM_REG_LMVO_DRS(module), 0x800C0011);

	ISP_WR32(CAM_REG_UFEO_CON(module), 0x80000020);//UFEo 32
	ISP_WR32(CAM_REG_UFEO_CON2(module), 0x0010000C);
	ISP_WR32(CAM_REG_UFEO_CON3(module), 0x00080004);
	ISP_WR32(CAM_REG_UFEO_DRS(module), 0x800C0011);

	ISP_WR32(CAM_REG_UFGO_CON(module), 0x80000020);//UFGO 32
	ISP_WR32(CAM_REG_UFGO_CON2(module), 0x0010000C);
	ISP_WR32(CAM_REG_UFGO_CON3(module), 0x00080004);
	ISP_WR32(CAM_REG_UFGO_DRS(module), 0x800C0011);

	ISP_WR32(CAM_REG_YUVO_CON(module), 0x80000280);//YUVO 640
	ISP_WR32(CAM_REG_YUVO_CON2(module), 0x014000F0);
	ISP_WR32(CAM_REG_YUVO_CON3(module), 0x00A00050);
	ISP_WR32(CAM_REG_YUVO_DRS(module), 0x81AA015A);

	ISP_WR32(CAM_REG_YUVBO_CON(module), 0x80000140);//YUVBO 320
	ISP_WR32(CAM_REG_YUVBO_CON2(module), 0x00A00078);
	ISP_WR32(CAM_REG_YUVBO_CON3(module), 0x00500028);
	ISP_WR32(CAM_REG_YUVBO_DRS(module), 0x80D500AD);

	ISP_WR32(CAM_REG_YUVCO_CON(module), 0x800000A0);//YUVCO 160
	ISP_WR32(CAM_REG_YUVCO_CON2(module), 0x0050003C);//1/2~3/8
	ISP_WR32(CAM_REG_YUVCO_CON3(module), 0x00280014);//1/4~1/8
	ISP_WR32(CAM_REG_YUVCO_DRS(module), 0x806A0056);//2/3~13/24

	ISP_WR32(CAM_REG_CRZO_CON(module), 0x80000040);//CRZO 64
	ISP_WR32(CAM_REG_CRZO_CON2(module), 0x00200018);
	ISP_WR32(CAM_REG_CRZO_CON3(module), 0x00100008);
	ISP_WR32(CAM_REG_CRZO_DRS(module), 0x802A0022);

	ISP_WR32(CAM_REG_CRZBO_CON(module), 0x80000020);//CRZBO 32
	ISP_WR32(CAM_REG_CRZBO_CON2(module), 0x0010000C);
	ISP_WR32(CAM_REG_CRZBO_CON3(module), 0x00080004);
	ISP_WR32(CAM_REG_CRZBO_DRS(module), 0x800C0011);

	ISP_WR32(CAM_REG_CRZO_R2_CON(module), 0x80000080);//CRZO_R2 128
	ISP_WR32(CAM_REG_CRZO_R2_CON2(module), 0x00400030);
	ISP_WR32(CAM_REG_CRZO_R2_CON3(module), 0x00200010);
	ISP_WR32(CAM_REG_CRZO_R2_DRS(module), 0x80550045);

	ISP_WR32(CAM_REG_RSSO_R2_CON(module), 0x80000060);//RSSO_R2 96
	ISP_WR32(CAM_REG_RSSO_R2_CON2(module), 0x00300024);
	ISP_WR32(CAM_REG_RSSO_R2_CON3(module), 0x0018000C);
	ISP_WR32(CAM_REG_RSSO_R2_DRS(module), 0x80400034);

	/* RDMA */
	ISP_WR32(CAM_REG_RAWI_R2_CON(module), 0x80000300);//RAWI_R2 768
	ISP_WR32(CAM_REG_RAWI_R2_CON2(module), 0x01800120);//1/2~3/8
	ISP_WR32(CAM_REG_RAWI_R2_CON3(module), 0x00C00060);//1/4~1/8
	ISP_WR32(CAM_REG_RAWI_R2_DRS(module), 0x820001A0);//2/3~13/24

	ISP_WR32(CAM_REG_RAWI_R3_CON(module), 0x80000300);//RAWI_R3 768
	ISP_WR32(CAM_REG_RAWI_R3_CON2(module), 0x01800120);//1/2~3/8
	ISP_WR32(CAM_REG_RAWI_R3_CON3(module), 0x00C00060);//1/4~1/8
	ISP_WR32(CAM_REG_RAWI_R3_DRS(module), 0x820001A0);//2/3~13/24

	ISP_WR32(CAM_REG_UFDI_R2_CON(module), 0x80000020);//UFDI_R2 32
	ISP_WR32(CAM_REG_UFDI_R2_CON2(module), 0x0010000C);
	ISP_WR32(CAM_REG_UFDI_R2_CON3(module), 0x00080004);
	ISP_WR32(CAM_REG_UFDI_R2_DRS(module), 0x800C0011);

	ISP_WR32(CAM_REG_PDI_CON(module), 0x80000060);//PDI 96
	ISP_WR32(CAM_REG_PDI_CON2(module), 0x00300024);
	ISP_WR32(CAM_REG_PDI_CON3(module), 0x0018000C);
	ISP_WR32(CAM_REG_PDI_DRS(module), 0x80400034);

	ISP_WR32(CAM_REG_BPCI_CON(module), 0x80000060);//BPCI 96
	ISP_WR32(CAM_REG_BPCI_CON2(module), 0x00300024);
	ISP_WR32(CAM_REG_BPCI_CON3(module), 0x0018000C);
	ISP_WR32(CAM_REG_BPCI_DRS(module), 0x80400034);

	ISP_WR32(CAM_REG_BPCI_R2_CON(module), 0x80000060);//BPCI_R2 96
	ISP_WR32(CAM_REG_BPCI_R2_CON2(module), 0x00300024);
	ISP_WR32(CAM_REG_BPCI_R2_CON3(module), 0x0018000C);
	ISP_WR32(CAM_REG_BPCI_R2_DRS(module), 0x80400034);

	ISP_WR32(CAM_REG_BPCI_R3_CON(module), 0x80000060);//BPCI_R3 96
	ISP_WR32(CAM_REG_BPCI_R3_CON2(module), 0x00300024);
	ISP_WR32(CAM_REG_BPCI_R3_CON3(module), 0x0018000C);
	ISP_WR32(CAM_REG_BPCI_R3_DRS(module), 0x80400034);

	ISP_WR32(CAM_REG_LSCI_CON(module), 0x80000080);//LCSI 128
	ISP_WR32(CAM_REG_LSCI_CON2(module), 0x00400030);
	ISP_WR32(CAM_REG_LSCI_CON3(module), 0x00200010);
	ISP_WR32(CAM_REG_LSCI_DRS(module), 0x80550045);

	ISP_WR32(CAM_REG_CQI_R1_CON(module), 0x80000040);//CQI_R1 64
	ISP_WR32(CAM_REG_CQI_R1_CON2(module), 0x00200018);
	ISP_WR32(CAM_REG_CQI_R1_CON3(module), 0x00100008);
	ISP_WR32(CAM_REG_CQI_R1_DRS(module), 0x802A0022);

	ISP_WR32(CAM_REG_CQI_R2_CON(module), 0x80000040);//CQI_R2 96
	ISP_WR32(CAM_REG_CQI_R2_CON2(module), 0x00200018);
	ISP_WR32(CAM_REG_CQI_R2_CON3(module), 0x00100008);
	ISP_WR32(CAM_REG_CQI_R2_DRS(module), 0x802A0022);


	/* Enable urgent FIFO (DRS) setting */
	ISP_WR32(CAMSYS_REG_HALT1_EN, 0x00000001);
	ISP_WR32(CAMSYS_REG_HALT2_EN, 0x00000001);
	ISP_WR32(CAMSYS_REG_HALT3_EN, 0x00000001);
	ISP_WR32(CAMSYS_REG_HALT4_EN, 0x00000001);
	ISP_WR32(CAMSYS_REG_HALT1_SEC_EN, 0x00000001);
	ISP_WR32(CAMSYS_REG_HALT2_SEC_EN, 0x00000001);
	ISP_WR32(CAMSYS_REG_HALT3_SEC_EN, 0x00000001);
	ISP_WR32(CAMSYS_REG_HALT4_SEC_EN, 0x00000001);
}

/*******************************************************************************
 *
 ******************************************************************************/
static void ISP_EnableClock(enum ISP_DEV_NODE_ENUM module, bool En)
{
	if (En) {
#if defined(EP_NO_CLKMGR)
		int cg_con1 = 0, cg_con2 = 0;
		int cg_con3 = 0, cg_con4 = 0;
		int cg_con5 = 0, cg_con6 = 0;
		int cg_con7 = 0, cg_con8 = 0;

		spin_lock(&(IspInfo.SpinLockClock));
		/* LOG_DBG("Camera clock enbled. G_u4EnableClockCount: %d.", */
		/*      G_u4EnableClockCount[module]); */

		if (G_u4EnableClockCount[module] == 0) {
			/* Enable clock by hardcode:
			 * 1. CAMSYS_CG_CLR (0x1A000008) = 0xffffffff;
			 * 2. IMG_CG_CLR (0x15000008) = 0xffffffff;
			 */
			cg_con1 = ISP_RD32(CAMSYS_REG_CG_CON);
			ISP_WR32(CAMSYS_REG_CG_CLR, 0xFFFFFFFF);
			cg_con2 = ISP_RD32(CAMSYS_REG_CG_CON);
			if (module == ISP_CAM_A_IDX) {
				cg_con3 = ISP_RD32(CAMSYS_RAWA_REG_CG_CON);
				ISP_WR32(CAMSYS_RAWA_REG_CG_CLR, 0xFFFFFFFF);
				cg_con4 = ISP_RD32(CAMSYS_RAWA_REG_CG_CON);
			}
			if (module == ISP_CAM_B_IDX) {
				cg_con5 = ISP_RD32(CAMSYS_RAWB_REG_CG_CON);
				ISP_WR32(CAMSYS_RAWB_REG_CG_CLR, 0xFFFFFFFF);
				cg_con6 = ISP_RD32(CAMSYS_RAWB_REG_CG_CON);
			}
			if (module == ISP_CAM_C_IDX) {
				cg_con7 = ISP_RD32(CAMSYS_RAWC_REG_CG_CON);
				ISP_WR32(CAMSYS_RAWC_REG_CG_CLR, 0xFFFFFFFF);
				cg_con8 = ISP_RD32(CAMSYS_RAWC_REG_CG_CON);
			}
		}

		 G_u4EnableClockCount[module]++;
		spin_unlock(&(IspInfo.SpinLockClock));
		LOG_INF("camsyscg org:0x%x,%x,%x,%x new:0x%x,%x,%x,%x cnt:%d\n",
			cg_con1,
			cg_con3, cg_con5, cg_con7, cg_con2,
			cg_con4, cg_con6, cg_con8, G_u4EnableClockCount[module]);

#else /*CCF*/
		/*LOG_INF("CCF:prepare_enable clk"); */
		spin_lock(&(IspInfo.SpinLockClock));
		G_u4EnableClockCount[module]++;
		spin_unlock(&(IspInfo.SpinLockClock));
		Prepare_Enable_ccf_clock(module); /* !!cannot be used in spinlock!! */
#endif
	} else { /* Disable clock. */
#if defined(EP_NO_CLKMGR)
		spin_lock(&(IspInfo.SpinLockClock));
		/* LOG_DBG("Camera clock disabled. */
		/*      G_u4EnableClockCount: %d.", G_u4EnableClockCount[module]); */
		G_u4EnableClockCount[module]--;
		switch (G_u4EnableClockCount[module]) {
		case 0:
			/* Disable clock by hardcode:
			 * 1. CAMSYS_CG_SET (0x1A000004) = 0xffffffff;
			 * 2. IMG_CG_SET (0x15000004) = 0xffffffff;
			 */
			/* No need to set, no mattter ccf is on or off */
			/* ISP_WR32(CAMSYS_REG_CG_SET, 0xFFFFFEBF); */
			break;
		default:
			break;
		}
		spin_unlock(&(IspInfo.SpinLockClock));
#else
		/*LOG_INF("CCF:disable_unprepare clk\n"); */
		spin_lock(&(IspInfo.SpinLockClock));
		if (G_u4EnableClockCount[module] == 0) {
			spin_unlock(&(IspInfo.SpinLockClock));

			LOG_INF(
				"G_u4EnableClockCount aleady be 0, do nothing\n");

			return;
		}

		G_u4EnableClockCount[module]--;
		spin_unlock(&(IspInfo.SpinLockClock));
		/* !!cannot be used in spinlock!! */
		Disable_Unprepare_ccf_clock(module);
#endif
	}
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline void ISP_Reset(int module)
{
	unsigned long long sec = 0, usec = 0, m_sec = 0, m_usec = 0;
	unsigned long long timeoutMs = 5000; /*5ms */
	bool bDumped = MFALSE;

	if (IS_MT6853(g_platform_id)) {
		if (module == ISP_CAM_C_IDX) {
			LOG_NOTICE("Not support reset module: ISP_CAM_C_IDX\n");
			return;
		}
	}

	LOG_DBG(" Reset module(%d)\n", module);

	switch (module) {
	case ISP_CAM_A_IDX:
	case ISP_CAM_B_IDX:
	case ISP_CAM_C_IDX:
	{
		/* Reset CAM flow */
		ISP_WR32(CAM_REG_CTL_SW_CTL(module), 0x0);
		ISP_WR32(CAM_REG_CTL_SW_CTL(module), 0x1);

		/* timer */
		m_sec = ktime_get(); /* ns */
		do_div(m_sec, 1000); /* usec */
		m_usec = do_div(m_sec, 1000000); /* sec and usec */
		while ((ISP_RD32(CAM_REG_CTL_SW_CTL(module)) & 0x2) != 0x2) {
			LOG_DBG("CAM resetting... module:%d,0x%x\n", module,
				(unsigned int)ISP_RD32(
				CAM_REG_CTL_SW_CTL(module)));
			sec = ktime_get(); /* ns */
			do_div(sec, 1000); /* usec */
			usec = do_div(sec, 1000000); /* sec and usec */
			/* wait time>timeoutMs */
			if (((usec - m_usec) > timeoutMs) &&
				(bDumped == MFALSE)) {
				LOG_INF(
				"%d: wait SW idle timeout, reg(0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x)\n",
				module,
				(unsigned int)ISP_RD32(
					CAM_REG_TG_SEN_MODE(module)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_SW_CTL(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_SW_CTL(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_SW_CTL(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(CAMSYS_REG_CG_CON),
				(unsigned int)ISP_RD32(CAMSYS_REG_CG_SET),
				(unsigned int)ISP_RD32(CAMSYS_REG_CG_CLR));
				//dump smi for debugging
#if IS_ENABLED(CONFIG_MTK_SMI_EXT)
				if (smi_debug_bus_hang_detect(
					false, "camera_isp") != 0)
					LOG_INF(
					"ERR:smi_debug_bus_hang_detect");
#endif
				bDumped = MTRUE;
				break;
			}
		}

		ISP_WR32(CAM_REG_CTL_SW_CTL(module), 0x4);
		ISP_WR32(CAM_REG_CTL_SW_CTL(module), 0x0);

		break;
	}
	case ISP_CAMSV0_IDX:
	case ISP_CAMSV1_IDX:
	case ISP_CAMSV2_IDX:
	case ISP_CAMSV3_IDX:
	case ISP_CAMSV4_IDX:
	case ISP_CAMSV5_IDX:
	case ISP_CAMSV6_IDX:
	case ISP_CAMSV7_IDX: {
		/* Reset CAMSV flow */
		ISP_WR32(CAMSV_REG_SW_CTL(module), 0x1); /* IMGO_RST_TRIG: 1 */

		while ((ISP_RD32(CAMSV_REG_SW_CTL(module)) & 0x3) != 0x3) {
			/* camsv_top0 DMA2, camsv_2 need additional polling */
			/* register DMA_SOFT_RSTSTAT with IMGO and FUEO */
			if ((module == ISP_CAMSV1_IDX) &&
			    ((DMA_ST_MASK_CAMSV_IMGO_OR_UFO &
			      ISP_RD32(CAMSV_REG_DMA_SOF_RSTSTAT(module))) ==
			     DMA_ST_MASK_CAMSV_IMGO_OR_UFO))
				break;
			/* Polling IMGO_RST_ST to 1 */
			LOG_DBG("CAMSV resetting... module:%d\n", module);
		}
		ISP_WR32(CAMSV_REG_SW_CTL(module), 0x4); /* SW_RST:1 */
		ISP_WR32(CAMSV_REG_SW_CTL(module), 0x0); /* IMGO_RST_TRIG: 0 */
		break;
	}
	default:
		LOG_NOTICE("Not support reset module:%d\n", module);
		break;
	}
}

/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_ReadReg(struct ISP_REG_IO_STRUCT *pRegIo)
{
	unsigned int i, ispRange = 0;
	int Ret = 0;
	void __iomem *regBase;
	struct ISP_REG_STRUCT *pReg = NULL;
	struct ISP_REG_STRUCT reg;

	if ((pRegIo->pData == NULL) ||
			(pRegIo->Count == 0) ||
			(pRegIo->Count > ISP_REG_RANGE)) {
		LOG_NOTICE(
			"pRegIo->pData is NULL, Count:%d!!\n",
			pRegIo->Count);
		Ret = -EFAULT;
		goto EXIT;
	}

	pReg = kmalloc((pRegIo->Count) *
		sizeof(struct ISP_REG_STRUCT), GFP_ATOMIC);

	if (pReg == NULL) {
		LOG_NOTICE(
			"ERROR: kmalloc failed,(process, pid, tgid)=(%s, %d, %d)\n",
			current->comm,
			current->pid,
			current->tgid);
			Ret = -ENOMEM;
		goto EXIT;
	}

	if (copy_from_user(pReg, (void __user *)pRegIo->pData,
			sizeof(struct ISP_REG_STRUCT) * pRegIo->Count) != 0) {
		LOG_NOTICE("copy_from_user failed\n");
		Ret = -EFAULT;
		goto EXIT;
	}

	switch (pReg->module) {
	case ISP_CAM_A_IDX:
		regBase = ISP_CAM_A_BASE;
		break;
	case ISP_CAM_B_IDX:
		regBase = ISP_CAM_B_BASE;
		break;
	case ISP_CAM_C_IDX:
		regBase = ISP_CAM_C_BASE;
		break;
	case ISP_CAMSV0_IDX:
		regBase = ISP_CAMSV0_BASE;
		break;
	case ISP_CAMSV1_IDX:
		regBase = ISP_CAMSV1_BASE;
		break;
	case ISP_CAMSV2_IDX:
		regBase = ISP_CAMSV2_BASE;
		break;
	case ISP_CAMSV3_IDX:
		regBase = ISP_CAMSV3_BASE;
		break;
	case ISP_CAMSV4_IDX:
		regBase = ISP_CAMSV4_BASE;
		break;
	case ISP_CAMSV5_IDX:
		regBase = ISP_CAMSV5_BASE;
		break;
	case ISP_CAMSV6_IDX:
		regBase = ISP_CAMSV6_BASE;
		break;
	case ISP_CAMSV7_IDX:
		regBase = ISP_CAMSV7_BASE;
		break;
	default:
		LOG_NOTICE("Unsupported module(%x) !!!\n", pReg->module);
		Ret = -EFAULT;
		goto EXIT;
	}

	if (regBase < ISP_CAMSV0_BASE)
		ispRange = ISP_REG_RANGE;
	else
		ispRange = PAGE_SIZE;

	spin_lock(&(IspInfo.SpinLockClock));
	if (G_u4EnableClockCount[pReg->module] == 0) {
		spin_unlock(&(IspInfo.SpinLockClock));
		LOG_NOTICE("hw module is off(%d)\n", pReg->module);
		Ret = -EFAULT;
		goto EXIT;
	}
	spin_unlock(&(IspInfo.SpinLockClock));

	for (i = 0; i < pRegIo->Count; i++) {
		if (get_user(reg.Addr, (unsigned int *)&pReg->Addr) != 0) {
			LOG_NOTICE("get_user failed\n");
			Ret = -EFAULT;
			goto EXIT;
		}

		if ((regBase + reg.Addr) < (regBase + ispRange)) {
			reg.Val = ISP_RD32(regBase + reg.Addr);
		} else {
			LOG_NOTICE("Wrong address(0x%lx)\n",
				   (unsigned long)(regBase + reg.Addr));

			reg.Val = 0;
		}

		if (put_user(reg.Val,
			(unsigned int *)&(pRegIo->pData->Val)) != 0) {
			LOG_NOTICE("put_user failed\n");
			Ret = -EFAULT;
			goto EXIT;
		}
		pReg++;
		/* */
	}
/* */
EXIT:
	if (pReg != NULL) {
		kfree(pReg);
		pReg = NULL;
	}

	return Ret;
}

/*******************************************************************************
 *
 ******************************************************************************/
/* Note: Can write sensor's test model only, */
/* if need write to other modules, need modify current code flow */
static int ISP_WriteRegToHw(struct ISP_REG_STRUCT *pReg, unsigned int Count)
{
	int Ret = 0;
	unsigned int i, ispRange = 0;
	bool dbgWriteReg;
	void __iomem *regBase;

	/* Use local variable to store IspInfo.DebugMask & */
	/* ISP_DBG_WRITE_REG for saving lock time */
	spin_lock(&(IspInfo.SpinLockIsp));
	dbgWriteReg = IspInfo.DebugMask & ISP_DBG_WRITE_REG;
	spin_unlock(&(IspInfo.SpinLockIsp));

	if ((void *)pReg == NULL) {
		LOG_NOTICE("%s pReg is null.\n", __func__);
		return -EFAULT;
	}

	switch (pReg->module) {
	case ISP_CAM_A_IDX:
		regBase = ISP_CAM_A_BASE;
		break;
	case ISP_CAM_B_IDX:
		regBase = ISP_CAM_B_BASE;
		break;
	case ISP_CAM_C_IDX:
		regBase = ISP_CAM_C_BASE;
		break;
	case ISP_CAMSV0_IDX:
		regBase = ISP_CAMSV0_BASE;
		break;
	case ISP_CAMSV1_IDX:
		regBase = ISP_CAMSV1_BASE;
		break;
	case ISP_CAMSV2_IDX:
		regBase = ISP_CAMSV2_BASE;
		break;
	case ISP_CAMSV3_IDX:
		regBase = ISP_CAMSV3_BASE;
		break;
	case ISP_CAMSV4_IDX:
		regBase = ISP_CAMSV4_BASE;
		break;
	case ISP_CAMSV5_IDX:
		regBase = ISP_CAMSV5_BASE;
		break;
	case ISP_CAMSV6_IDX:
		regBase = ISP_CAMSV6_BASE;
		break;
	case ISP_CAMSV7_IDX:
		regBase = ISP_CAMSV7_BASE;
		break;
	default:
		LOG_NOTICE("Unsupported module(%x) !!!\n", pReg->module);
		return -EFAULT;
	}

	spin_lock(&(IspInfo.SpinLockClock));
	if (G_u4EnableClockCount[pReg->module] == 0) {
		spin_unlock(&(IspInfo.SpinLockClock));
		LOG_NOTICE("hw module is off(%d)\n", pReg->module);
		return -EFAULT;
	}
	spin_unlock(&(IspInfo.SpinLockClock));

	/*  */
	if (dbgWriteReg)
		LOG_DBG("- E.\n");

	if (regBase < ISP_CAMSV0_BASE)
		ispRange = ISP_REG_RANGE;
	else
		ispRange = PAGE_SIZE;

	/*  */
	for (i = 0; i < Count; i++) {
		if (dbgWriteReg)
			LOG_DBG(
				"module(%d), base(0x%lx),Addr(0x%lx), Val(0x%x)\n",
				pReg->module, (unsigned long)regBase,
				(unsigned long)(pReg[i].Addr),
				(unsigned int)(pReg[i].Val));

		if (((regBase + pReg[i].Addr) < (regBase + ispRange))) {
			ISP_WR32(regBase + pReg[i].Addr, pReg[i].Val);
		} else {
			LOG_NOTICE("wrong address(0x%lx)\n",
				   (unsigned long)(regBase + pReg[i].Addr));
			Ret = -EFAULT;
		}
	}

	/*  */
	return Ret;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_WriteReg(struct ISP_REG_IO_STRUCT *pRegIo)
{
	int Ret = 0;
	struct ISP_REG_STRUCT *pData = NULL;

	if (((pRegIo->Count * sizeof(struct ISP_REG_STRUCT)) > 0xFFFFF000) ||
		(pRegIo->Count == 0)) {
		LOG_NOTICE("pRegIo->Count error");
		Ret = -EFAULT;
		goto EXIT;
	}
	/*  */
	if (IspInfo.DebugMask & ISP_DBG_WRITE_REG) {
		LOG_DBG("Data(0x%pK), Count(%d)\n", (pRegIo->pData),
			(pRegIo->Count));
	}

	pData = kmalloc((pRegIo->Count) * sizeof(struct ISP_REG_STRUCT),
			GFP_ATOMIC);

	if (pData == NULL) {
		LOG_INF(
			"ERROR: kmalloc failed, (process, pid, tgid)=(%s, %d, %d)\n",
			current->comm, current->pid, current->tgid);

		Ret = -ENOMEM;
		goto EXIT;
	}

	if ((void __user *)(pRegIo->pData) == NULL) {
		LOG_NOTICE("NULL pData");
		Ret = -EFAULT;
		goto EXIT;
	}

	/*  */
	if (copy_from_user(pData, (void __user *)(pRegIo->pData),
			   pRegIo->Count * sizeof(struct ISP_REG_STRUCT)) !=
	    0) {

		LOG_NOTICE("copy_from_user failed\n");
		Ret = -EFAULT;
		goto EXIT;
	}
	/*  */
	Ret = ISP_WriteRegToHw(pData, pRegIo->Count);
/*  */
EXIT:
	if (pData != NULL) {
		kfree(pData);
		pData = NULL;
	}
	return Ret;
}

/*******************************************************************************
 *
 ******************************************************************************/

/* isr dbg log , sw isr response counter , +1 when sw receive 1 sof isr. */
static unsigned int sof_count[ISP_IRQ_TYPE_AMOUNT] = {0};
static int Vsync_cnt[ISP_IRQ_TYPE_AMOUNT] = {0};
static unsigned int HwP1Done_cnt[ISP_IRQ_TYPE_AMOUNT] = {0};

/* keep current frame status */
static enum CAM_FrameST FrameStatus[ISP_IRQ_TYPE_AMOUNT] = {0};

/* current invoked time is at 1st sof or not during */
/* each streaming, reset when streaming off */
static bool g1stSof[ISP_IRQ_TYPE_AMOUNT] = {0};

#if (TSTMP_SUBSAMPLE_INTPL == 1)
static bool g1stSwP1Done[ISP_IRQ_TYPE_AMOUNT] = {0};

static unsigned long long gPrevSofTimestp[ISP_IRQ_TYPE_AMOUNT];
#endif

static struct S_START_T gSTime[ISP_IRQ_TYPE_AMOUNT] = {{0} };

#ifdef _MAGIC_NUM_ERR_HANDLING_
#define _INVALID_FRM_CNT_ 0xFFFF
#endif

static long ISP_Buf_CTRL_FUNC(unsigned long Param)
{
	int Ret = 0;
	enum _isp_dma_enum_ rt_dma;
	unsigned int i = 0;
	struct ISP_BUFFER_CTRL_STRUCT rt_buf_ctrl;

	/*  */
	if ((void __user *)Param == NULL) {
		LOG_NOTICE("[rtbc]NULL Param");
		return -EFAULT;
	}
	/*  */
	if (copy_from_user(&rt_buf_ctrl, (void __user *)Param,
			   sizeof(struct ISP_BUFFER_CTRL_STRUCT)) == 0) {

		if (rt_buf_ctrl.module >= ISP_IRQ_TYPE_AMOUNT ||
		    rt_buf_ctrl.module < 0) {
			LOG_NOTICE("[rtbc]not supported module:0x%x\n",
				   rt_buf_ctrl.module);

			return -EFAULT;
		}

		if (pstRTBuf[rt_buf_ctrl.module] == NULL) {
			LOG_NOTICE("[rtbc]NULL pstRTBuf, module:0x%x\n",
				   rt_buf_ctrl.module);

			return -EFAULT;
		}

		rt_dma = rt_buf_ctrl.buf_id;
		if (rt_dma >= _cam_max_ ||
		    rt_dma < 0) {
			LOG_NOTICE("[rtbc]buf_id error:0x%x\n", rt_dma);
			return -EFAULT;
		}

		/*  */
		switch (rt_buf_ctrl.ctrl) {
		case ISP_RT_BUF_CTRL_CLEAR:
			/*  */
			if (IspInfo.DebugMask & ISP_DBG_BUF_CTRL) {
				LOG_INF("[rtbc][%d][CLEAR]:rt_dma(%d)\n",
					rt_buf_ctrl.module, rt_dma);
			}
			/*  */

			memset((void *)IspInfo.IrqInfo
				       .LastestSigTime_usec[rt_buf_ctrl.module],
			       0, sizeof(unsigned int) * 32);

			memset((void *)IspInfo.IrqInfo
				       .LastestSigTime_sec[rt_buf_ctrl.module],
			       0, sizeof(unsigned int) * 32);

			/* remove, cause clear will be involked only */
			/* when current module r totally stopped */
			/* spin_lock_irqsave( */
			/*      &(IspInfo.SpinLockIrq[irqT_Lock]), flags); */

			/* reset active record */
			pstRTBuf[rt_buf_ctrl.module]->ring_buf[rt_dma].active =
				MFALSE;

			memset((char *)&pstRTBuf[rt_buf_ctrl.module]
				       ->ring_buf[rt_dma],
			       0x00,
			       sizeof(struct ISP_RT_RING_BUF_INFO_STRUCT));

			/* init. frmcnt before vf_en */
			for (i = 0; i < ISP_RT_BUF_SIZE; i++) {
				pstRTBuf[rt_buf_ctrl.module]
					->ring_buf[rt_dma]
					.data[i]
					.image.frm_cnt = _INVALID_FRM_CNT_;
			}
			switch (rt_buf_ctrl.module) {
			case ISP_IRQ_TYPE_INT_CAM_A_ST:
			case ISP_IRQ_TYPE_INT_CAM_B_ST:
			case ISP_IRQ_TYPE_INT_CAM_C_ST:
				if ((pstRTBuf[rt_buf_ctrl.module]
					     ->ring_buf[_imgo_]
					     .active == MFALSE) &&
				    (pstRTBuf[rt_buf_ctrl.module]
					     ->ring_buf[_rrzo_]
					     .active == MFALSE)) {

					sof_count[rt_buf_ctrl.module] = 0;
					g1stSof[rt_buf_ctrl.module] = MTRUE;
					HwP1Done_cnt[rt_buf_ctrl.module] = 0;

#if (TSTMP_SUBSAMPLE_INTPL == 1)

					g1stSwP1Done[rt_buf_ctrl.module] =
						MTRUE;

					gPrevSofTimestp[rt_buf_ctrl.module] = 0;
#endif
					g_ISPIntStatus[rt_buf_ctrl.module]
						.ispIntErr = 0;

					g_ISPIntStatus[rt_buf_ctrl.module]
						.ispInt5Err = 0;

					g_ISPIntStatus_SMI[rt_buf_ctrl.module]
						.ispIntErr = 0;

					g_ISPIntStatus_SMI[rt_buf_ctrl.module]
						.ispInt5Err = 0;

					pstRTBuf[rt_buf_ctrl.module]->dropCnt =
						0;

					pstRTBuf[rt_buf_ctrl.module]->state = 0;
				}

				memset((void *)g_DmaErr_CAM[rt_buf_ctrl.module],
				       0, sizeof(unsigned int) * _cam_max_);

				break;
			case ISP_IRQ_TYPE_INT_CAMSV_0_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_1_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_2_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_3_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_4_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_5_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_6_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_7_ST:
				if (pstRTBuf[rt_buf_ctrl.module]
					    ->ring_buf[_camsv_imgo_]
					    .active == MFALSE) {

					sof_count[rt_buf_ctrl.module] = 0;
					g1stSof[rt_buf_ctrl.module] = MTRUE;
					HwP1Done_cnt[rt_buf_ctrl.module] = 0;
					g_ISPIntStatus[rt_buf_ctrl.module]
						.ispIntErr = 0;

					g_ISPIntStatus[rt_buf_ctrl.module]
						.ispInt5Err = 0;

					g_ISPIntStatus_SMI[rt_buf_ctrl.module]
						.ispIntErr = 0;

					g_ISPIntStatus_SMI[rt_buf_ctrl.module]
						.ispInt5Err = 0;

					pstRTBuf[rt_buf_ctrl.module]->dropCnt =
						0;

					pstRTBuf[rt_buf_ctrl.module]->state = 0;
				}

				break;
			default:
				LOG_NOTICE("unsupported module:0x%x\n",
					   rt_buf_ctrl.module);
				break;
			}

			/* spin_unlock_irqrestore( */
			/*      &(IspInfo.SpinLockIrq[irqT_Lock]), flags); */

			break;
		case ISP_RT_BUF_CTRL_DMA_EN: {
			unsigned char array[_cam_max_];
			unsigned int z;
			unsigned char *pExt;

			if (rt_buf_ctrl.pExtend == NULL) {
				LOG_NOTICE("NULL pExtend");
				Ret = -EFAULT;
				break;
			}

			pExt = (unsigned char *)(rt_buf_ctrl.pExtend);
	for (z = 0; z < _cam_max_; z++) {
		if (get_user(array[z], (unsigned char *)pExt) ==
		0) {

			pstRTBuf[rt_buf_ctrl.module]->ring_buf[z].active =
			array[z];

					if (IspInfo.DebugMask &
					    ISP_DBG_BUF_CTRL) {
						LOG_INF(
							"[rtbc][DMA_EN]:dma_%d:%d",
							z, array[z]);
					}
				} else {
					LOG_NOTICE(
						"[rtbc][DMA_EN]:get_user failed(%d)",
						z);

					Ret = -EFAULT;
				}
				pExt++;
			}
		} break;
		/* Add this to remove build warning. */
		case ISP_RT_BUF_CTRL_MAX:
			/* Do nothing. */
			break;
		}
	} else {
		LOG_NOTICE("[rtbc]copy_from_user failed");
		Ret = -EFAULT;
	}

	return Ret;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_REGISTER_IRQ_USERKEY(char *userName)
{
	int key = -1;
	int i = 0;

	spin_lock((spinlock_t *)(&SpinLock_UserKey));

	/* 1. check the current users is full or not */
	if (FirstUnusedIrqUserKey >= IRQ_USER_NUM_MAX) {
		key = -1;
	} else {
		/* 2. check the user had registered or not */
		for (i = 1; i < FirstUnusedIrqUserKey; i++) {
			/* index 0 is for all the users that do not */
			/* register irq first */
			if (strcmp((void *)IrqUserKey_UserInfo[i].userName,
				   userName) == 0) {
				key = IrqUserKey_UserInfo[i].userKey;
				break;
			}
		}

		/* 3.return new userkey for user */
		/*   if the user had not registered before */
		if (key < 0) {

			if (strcmp((void *)IrqUserKey_UserInfo[i].userName,
				"DefaultUserNametoAllocMem") != 0) {
				LOG_INF("userName was not initialized.\n");
				return key;
			}

			/* IrqUserKey_UserInfo[i].userName=userName; */
			memset((void *)IrqUserKey_UserInfo[i].userName, 0,
			       sizeof(IrqUserKey_UserInfo[i].userName));

			strncpy((void *)IrqUserKey_UserInfo[i].userName,
				userName, USERKEY_USERNAME_STR_LEN - 1);

			IrqUserKey_UserInfo[i].userKey = FirstUnusedIrqUserKey;
			key = FirstUnusedIrqUserKey;
			FirstUnusedIrqUserKey++;
		}
	}

	spin_unlock((spinlock_t *)(&SpinLock_UserKey));
	LOG_DBG("User(%s)key(%d)\n", userName, key);
	return key;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_FLUSH_IRQ(struct ISP_WAIT_IRQ_STRUCT *irqinfo)
{			     /* FIX to avoid build warning */
	unsigned long flags; /* old: unsigned int flags; */

	if (irqinfo->EventInfo.UserKey != 0)
		LOG_INF("type(%d)userKey(%d)St_type(%d)St(0x%x)",
			irqinfo->Type, irqinfo->EventInfo.UserKey,
			irqinfo->EventInfo.St_type, irqinfo->EventInfo.Status);

	if (irqinfo->EventInfo.UserKey == 1)
		LOG_NOTICE("++FLUSH 3a waitirq cnt = %d\n", irq3a_wait_cnt);

	if (irqinfo->Type >= ISP_IRQ_TYPE_AMOUNT ||
	    irqinfo->Type < 0) {
		LOG_NOTICE("FLUSH_IRQ: type error(%d)", irqinfo->Type);
		return -EFAULT;
	}

	if (irqinfo->EventInfo.St_type >= ISP_IRQ_ST_AMOUNT ||
	    irqinfo->EventInfo.St_type < 0) {
		LOG_NOTICE("FLUSH_IRQ: st_type error(%d)",
			   irqinfo->EventInfo.St_type);
		return -EFAULT;
	}

	if (irqinfo->EventInfo.UserKey >= IRQ_USER_NUM_MAX ||
	    irqinfo->EventInfo.UserKey < 0) {
		LOG_NOTICE("FLUSH_IRQ: userkey error(%d)",
			   irqinfo->EventInfo.UserKey);
		return -EFAULT;
	}

	/* 1. enable signal */
	spin_lock_irqsave(&(IspInfo.SpinLockIrq[irqinfo->Type]), flags);
	IspInfo.IrqInfo.Status[irqinfo->Type][irqinfo->EventInfo.St_type]
			      [irqinfo->EventInfo.UserKey] |=
		irqinfo->EventInfo.Status;

	/* 1.2. For camsv FBC issue, enq may wait next SOF during */
	/* camsviopipe stop(another thread) */
	/* append camsv SOF in flush IRQ to wake up camsv enq thread */
	if (irqinfo->Type >= ISP_IRQ_TYPE_INT_CAMSV_START_ST &&
	    irqinfo->Type <= ISP_IRQ_TYPE_INT_CAMSV_END_ST) {
		if (irqinfo->EventInfo.St_type == SIGNAL_INT) {
			if (irqinfo->EventInfo.Status == SV_SW_PASS1_DON_ST)
				IspInfo.IrqInfo
					.Status[irqinfo->Type]
					       [irqinfo->EventInfo.St_type]
					       [irqinfo->EventInfo.UserKey] |=
					SV_SOF_INT_ST;
		}
	}

	spin_unlock_irqrestore(&(IspInfo.SpinLockIrq[irqinfo->Type]), flags);

	/* 2. force to wake up the user that are waiting for that signal */
	if (ISP_CheckUseCamWaitQ(irqinfo->Type, irqinfo->EventInfo.St_type,
				 irqinfo->EventInfo.Status)) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam
				 [ISP_GetWaitQCamIndex(irqinfo->Type)]
				 [ISP_GetWaitQCamIrqIndex(
					 irqinfo->EventInfo.St_type,
					 irqinfo->EventInfo.Status)]);
	} else if (ISP_CheckUseCamsvWaitQ(irqinfo->Type,
					  irqinfo->EventInfo.St_type,
					  irqinfo->EventInfo.Status)) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCamsv
				 [ISP_GetWaitQCamsvIndex(irqinfo->Type)]
				 [ISP_GetWaitQCamsvIrqIndex(
					 irqinfo->EventInfo.St_type,
					 irqinfo->EventInfo.Status)]);

		wake_up_interruptible(
			&IspInfo.WaitQHeadCamsv[ISP_GetWaitQCamsvIndex(
				irqinfo->Type)][ISP_WAITQ_HEAD_IRQ_SV_SOF]);
	} else {
		wake_up_interruptible(&IspInfo.WaitQueueHead[irqinfo->Type]);
	}

	if (irqinfo->EventInfo.UserKey == 1)
		LOG_NOTICE("--FLUSH 3a waitirq cnt = %d\n", irq3a_wait_cnt);

	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_WaitIrq(struct ISP_WAIT_IRQ_STRUCT *WaitIrq)
{
	int Ret = 0, Timeout = WaitIrq->EventInfo.Timeout;
	/* FIX to avoid build warning */
	unsigned long flags; /* old: unsigned int flags; */
	unsigned int irqStatus;

	int idx = my_get_pow_idx(WaitIrq->EventInfo.Status);
	struct timespec64 time_getrequest;
	struct timespec64 time_ready2return;
	bool freeze_passbysigcnt = false;

	ktime_get_ts64(&time_getrequest);

	if (WaitIrq->Type >= ISP_IRQ_TYPE_AMOUNT ||
	    WaitIrq->Type < 0) {
		LOG_NOTICE("WaitIrq: type error(%d)", WaitIrq->Type);
		return -EFAULT;
	}

	if (WaitIrq->EventInfo.St_type >= ISP_IRQ_ST_AMOUNT ||
	    WaitIrq->EventInfo.St_type < 0) {
		LOG_NOTICE("WaitIrq: st_type error(%d)",
			   WaitIrq->EventInfo.St_type);
		return -EFAULT;
	}

	if (WaitIrq->EventInfo.UserKey >= IRQ_USER_NUM_MAX ||
	    WaitIrq->EventInfo.UserKey < 0) {
		LOG_NOTICE("WaitIrq: userkey error(%d)",
			   WaitIrq->EventInfo.UserKey);
		return -EFAULT;
	}
#ifdef ENABLE_WAITIRQ_LOG
	/* Debug interrupt */
	if (IspInfo.DebugMask & ISP_DBG_INT) {
		if (WaitIrq->EventInfo.Status &
		    IspInfo.IrqInfo
			    .Mask[WaitIrq->Type][WaitIrq->EventInfo.St_type]) {

			if (WaitIrq->EventInfo.UserKey > 0) {
				LOG_DBG(
					"+WaitIrq Clear(%d), Type(%d), Status(0x%08X), Timeout(%d/%d),user(%d)\n",
					WaitIrq->EventInfo.Clear,
					WaitIrq->Type,
					WaitIrq->EventInfo.Status,
					Timeout, WaitIrq->EventInfo.Timeout,
					WaitIrq->EventInfo.UserKey);
			}
		}
	}
#endif

	/* 1. wait type update */
	if (WaitIrq->EventInfo.Clear == ISP_IRQ_CLEAR_STATUS) {
		spin_lock_irqsave(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);
		IspInfo.IrqInfo
			.Status[WaitIrq->Type][WaitIrq->EventInfo.St_type]
			       [WaitIrq->EventInfo.UserKey] &=
			(~WaitIrq->EventInfo.Status);

		spin_unlock_irqrestore(&(IspInfo.SpinLockIrq[WaitIrq->Type]),
				       flags);

		return Ret;
	}
	{
		spin_lock_irqsave(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);
		if (WaitIrq->EventInfo.Status &
		    IspInfo.IrqInfo.MarkedFlag[WaitIrq->Type]
					      [WaitIrq->EventInfo.St_type]
					      [WaitIrq->EventInfo.UserKey]) {

			spin_unlock_irqrestore(
				&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

			/* force to be non_clear wait if marked before, */
			/* and check the request wait timing */
			/* if the entry time of wait request after mark */
			/* is before signal, */
			/* we freese the counting for passby signal */
			/*  */
			/* v : kernel receive mark request */
			/* o : kernel receive wait request */
			/* : return to user */
			/*  */
			/* case: freeze is true, and passby signal count = 0 */
			/*  */
			/* |                                              | */
			/* |                                  (wait)    | */
			/* |       v-------------o++++++ | */
			/* |                                              | */
			/* Sig                                            Sig */
			/*  */
			/* case: freeze is false, and passby signal count = 1 */
			/*  */
			/* |                                              | */
			/* |                                              | */
			/* |       v---------------------- |-o  (return) */
			/* |                                              | */
			/* Sig                                            Sig */
			/*  */

			freeze_passbysigcnt = !(ISP_GetIRQState(
				WaitIrq->Type, WaitIrq->EventInfo.St_type,
				WaitIrq->EventInfo.UserKey,
				WaitIrq->EventInfo.Status));
		} else {
			spin_unlock_irqrestore(
				&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

			if (WaitIrq->EventInfo.Clear == ISP_IRQ_CLEAR_WAIT) {
				spin_lock_irqsave(
					&(IspInfo.SpinLockIrq[WaitIrq->Type]),
					flags);

				if (IspInfo.IrqInfo.Status
					    [WaitIrq->Type]
					    [WaitIrq->EventInfo.St_type]
					    [WaitIrq->EventInfo.UserKey] &
				    WaitIrq->EventInfo.Status) {

					IspInfo.IrqInfo.Status
						[WaitIrq->Type]
						[WaitIrq->EventInfo.St_type]
						[WaitIrq->EventInfo.UserKey] &=
						(~WaitIrq->EventInfo.Status);
				}

				spin_unlock_irqrestore(
					&(IspInfo.SpinLockIrq[WaitIrq->Type]),
					flags);

			} else if (WaitIrq->EventInfo.Clear ==
				   ISP_IRQ_CLEAR_ALL) {

				spin_lock_irqsave(
					&(IspInfo.SpinLockIrq[WaitIrq->Type]),
					flags);

				IspInfo.IrqInfo
					.Status[WaitIrq->Type]
					       [WaitIrq->EventInfo.St_type]
					       [WaitIrq->EventInfo.UserKey] = 0;

				spin_unlock_irqrestore(
					&(IspInfo.SpinLockIrq[WaitIrq->Type]),
					flags);
			}
		}
	}

	/* Store irqinfo status in here to redeuce time of spin_lock_irqsave */
	spin_lock_irqsave(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

	irqStatus = IspInfo.IrqInfo
			    .Status[WaitIrq->Type][WaitIrq->EventInfo.St_type]
				   [WaitIrq->EventInfo.UserKey];

	spin_unlock_irqrestore(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

	if (WaitIrq->EventInfo.Clear == ISP_IRQ_CLEAR_NONE) {
		if (IspInfo.IrqInfo
			    .Status[WaitIrq->Type][WaitIrq->EventInfo.St_type]
				   [WaitIrq->EventInfo.UserKey] &
		    WaitIrq->EventInfo.Status) {
#ifdef ENABLE_WAITIRQ_LOG
			LOG_INF("%s,%s",
				"Already have irq!!!: WaitIrq Timeout(%d) Clear(%d), Type(%d), StType(%d)",
				", IrqStatus(0x%08X), WaitStatus(0x%08X), Timeout(%d), userKey(%d)\n",
				WaitIrq->EventInfo.Timeout,
				WaitIrq->EventInfo.Clear, WaitIrq->Type,
				WaitIrq->EventInfo.St_type, irqStatus,
				WaitIrq->EventInfo.Status,
				WaitIrq->EventInfo.Timeout,
				WaitIrq->EventInfo.UserKey);
#endif
			goto NON_CLEAR_WAIT;
		}
	}
#ifdef ENABLE_WAITIRQ_LOG
	LOG_INF(
		"before wait: Clear(%d) Type(%d) StType(%d) Sts(0x%08X) WaitSts(0x%08X) Timeout(%d) userKey(%d)\n",
		WaitIrq->EventInfo.Clear,
		WaitIrq->Type,
		WaitIrq->EventInfo.St_type,
		irqStatus,
		WaitIrq->EventInfo.Status,
		WaitIrq->EventInfo.Timeout,
		WaitIrq->EventInfo.UserKey);
#endif

	if (WaitIrq->EventInfo.UserKey == 1) {
		irq3a_wait_cnt++;
		if (irq3a_print_vf_off[WaitIrq->Type]) {
			LOG_NOTICE(
			"VF off IRQ 3A  irq3a_wait_cnt++ =%d Clear(%d) Type(%d) StType(%d) Status(0x%08X) WaitStatus(0x%08X) Timeout(%d) key(%d)\n",
			irq3a_wait_cnt,
			WaitIrq->EventInfo.Clear, WaitIrq->Type,
			WaitIrq->EventInfo.St_type, irqStatus,
			WaitIrq->EventInfo.Status, WaitIrq->EventInfo.Timeout,
			WaitIrq->EventInfo.UserKey);
		}
	}

	/* 2. start to wait signal */
	if (ISP_CheckUseCamWaitQ(WaitIrq->Type, WaitIrq->EventInfo.St_type,
				 WaitIrq->EventInfo.Status)) {
		Timeout = wait_event_interruptible_timeout(
			IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(
				WaitIrq->Type)]
					    [ISP_GetWaitQCamIrqIndex(
						    WaitIrq->EventInfo.St_type,
						    WaitIrq->EventInfo.Status)],
			ISP_GetIRQState(WaitIrq->Type,
					WaitIrq->EventInfo.St_type,
					WaitIrq->EventInfo.UserKey,
					WaitIrq->EventInfo.Status),
			ISP_MsToJiffies(WaitIrq->EventInfo.Timeout));
	} else if (ISP_CheckUseCamsvWaitQ(WaitIrq->Type,
					  WaitIrq->EventInfo.St_type,
					  WaitIrq->EventInfo.Status)) {
		Timeout = wait_event_interruptible_timeout(
			IspInfo.WaitQHeadCamsv
				[ISP_GetWaitQCamsvIndex(WaitIrq->Type)]
				[ISP_GetWaitQCamsvIrqIndex(
					WaitIrq->EventInfo.St_type,
					WaitIrq->EventInfo.Status)],
			ISP_GetIRQState(WaitIrq->Type,
					WaitIrq->EventInfo.St_type,
					WaitIrq->EventInfo.UserKey,
					WaitIrq->EventInfo.Status),
			ISP_MsToJiffies(WaitIrq->EventInfo.Timeout));
	} else {
		Timeout = wait_event_interruptible_timeout(
			IspInfo.WaitQueueHead[WaitIrq->Type],
			ISP_GetIRQState(WaitIrq->Type,
					WaitIrq->EventInfo.St_type,
					WaitIrq->EventInfo.UserKey,
					WaitIrq->EventInfo.Status),
			ISP_MsToJiffies(WaitIrq->EventInfo.Timeout));
	}

	if (WaitIrq->EventInfo.UserKey == 1) {
		irq3a_wait_cnt--;
		if (irq3a_print_vf_off[WaitIrq->Type]) {
			LOG_NOTICE(
			"VF off IRQ 3A  irq3a_wait_cnt-- =%d Clear(%d) Type(%d) StType(%d) Status(0x%08X) WaitStatus(0x%08X) Timeout(%d) key(%d)\n",
			irq3a_wait_cnt,
			WaitIrq->EventInfo.Clear, WaitIrq->Type,
			WaitIrq->EventInfo.St_type, irqStatus,
			WaitIrq->EventInfo.Status, WaitIrq->EventInfo.Timeout,
			WaitIrq->EventInfo.UserKey);
		}
	}
	/* check if user is interrupted by system signal */
	if ((Timeout != 0) &&
	    (!ISP_GetIRQState(WaitIrq->Type, WaitIrq->EventInfo.St_type,
			      WaitIrq->EventInfo.UserKey,
			      WaitIrq->EventInfo.Status))) {

		LOG_INF(
			"interrupted by system signal,return value(%d),irq Type/User/Sts(0x%x/%d/0x%x)\n",
			Timeout, WaitIrq->Type,
			WaitIrq->EventInfo.UserKey,
			WaitIrq->EventInfo.Status);

		Ret = -ERESTARTSYS; /* actually it should be -ERESTARTSYS */
		goto EXIT;
	}
	/* timeout */
	if (Timeout == 0) {
		/* Store irqinfo status in here to */
		/* redeuce time of spin_lock_irqsave */
		spin_lock_irqsave(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

		irqStatus = IspInfo.IrqInfo.Status[WaitIrq->Type]
						  [WaitIrq->EventInfo.St_type]
						  [WaitIrq->EventInfo.UserKey];

		spin_unlock_irqrestore(&(IspInfo.SpinLockIrq[WaitIrq->Type]),
				       flags);

		LOG_NOTICE(
			"ERRRR WaitIrq Clear(%d) Type(%d) StType(%d) Status(0x%08X) WaitStatus(0x%08X) Timeout(%d) key(%d)\n",
			WaitIrq->EventInfo.Clear, WaitIrq->Type,
			WaitIrq->EventInfo.St_type, irqStatus,
			WaitIrq->EventInfo.Status, WaitIrq->EventInfo.Timeout,
			WaitIrq->EventInfo.UserKey);

		Ret = -EFAULT;
		goto EXIT;
	}
#ifdef ENABLE_WAITIRQ_LOG
	else {
		/* Store irqinfo status in here to   */
		/* redeuce time of spin_lock_irqsave */
		spin_lock_irqsave(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

		irqStatus = IspInfo.IrqInfo.Status[WaitIrq->Type]
						  [WaitIrq->EventInfo.St_type]
						  [WaitIrq->EventInfo.UserKey];

		spin_unlock_irqrestore(&(IspInfo.SpinLockIrq[WaitIrq->Type]),
				       flags);

		LOG_INF(
			"Done WaitIrq Clear(%d) Type(%d) StType(%d) Status(0x%08X) WaitStatus(0x%08X) Timeout(%d) key(%d)\n",
			WaitIrq->EventInfo.Clear,
			WaitIrq->Type,
			WaitIrq->EventInfo.St_type,
			irqStatus,
			WaitIrq->EventInfo.Status,
			WaitIrq->EventInfo.Timeout,
			WaitIrq->EventInfo.UserKey);
	}
#endif

NON_CLEAR_WAIT:
	/* 3. get interrupt and update time related information */
	/*    that would be return to user */
	ktime_get_ts64(&time_ready2return);

	spin_lock_irqsave(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

	/* clear the status if someone get the irq */
	IspInfo.IrqInfo.Status[WaitIrq->Type][WaitIrq->EventInfo.St_type]
			      [WaitIrq->EventInfo.UserKey] &=
		(~WaitIrq->EventInfo.Status);

	spin_unlock_irqrestore(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

EXIT:
	/* 4. clear mark flag / reset marked time /  */
	/*    reset time related infor and passedby signal count */
	spin_lock_irqsave(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

	if (WaitIrq->EventInfo.Status &
	    IspInfo.IrqInfo
		    .MarkedFlag[WaitIrq->Type][WaitIrq->EventInfo.St_type]
			       [WaitIrq->EventInfo.UserKey]) {

		IspInfo.IrqInfo
			.MarkedFlag[WaitIrq->Type][WaitIrq->EventInfo.St_type]
				   [WaitIrq->EventInfo.UserKey] &=
			(~WaitIrq->EventInfo.Status);

		IspInfo.IrqInfo.MarkedTime_usec[WaitIrq->Type][idx]
					       [WaitIrq->EventInfo.UserKey] = 0;

		IspInfo.IrqInfo.MarkedTime_sec[WaitIrq->Type][idx]
					      [WaitIrq->EventInfo.UserKey] = 0;

		IspInfo.IrqInfo.PassedBySigCnt[WaitIrq->Type][idx]
					      [WaitIrq->EventInfo.UserKey] = 0;
	}
	spin_unlock_irqrestore(&(IspInfo.SpinLockIrq[WaitIrq->Type]), flags);

	return Ret;
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline void ISP_StopHW(int module)
{
	unsigned int regTGSt = 0, loopCnt = 3;
	int ret = 0;
	struct ISP_WAIT_IRQ_STRUCT waitirq;
	ktime_t time;
	unsigned long long sec = 0, m_sec = 0;
	unsigned long long timeoutMs = 500000000; /*500ms */
	//3ms * (CAM_A~C + CAMSV0~7) = 33ms(if all timeout)
	unsigned long long timeoutMsRst = 3000000; /*3ms */
	char moduleName[128];

	/* wait TG idle */
	switch (module) {
	case ISP_CAM_A_IDX:
		strncpy(moduleName, "CAMA", 5);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAM_A_ST;
		break;
	case ISP_CAM_B_IDX:
		strncpy(moduleName, "CAMB", 5);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAM_B_ST;
		break;
	case ISP_CAM_C_IDX:
		strncpy(moduleName, "CAMC", 5);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAM_C_ST;
		break;
	default:
		strncpy(moduleName, "CAMC", 5);
		goto RESET;
	}
	waitirq.EventInfo.Clear = ISP_IRQ_CLEAR_WAIT;
	waitirq.EventInfo.Status = VS_INT_ST;
	waitirq.EventInfo.St_type = SIGNAL_INT;
	waitirq.EventInfo.Timeout = 0x100;
	waitirq.EventInfo.UserKey = 0x0;
	waitirq.bDumpReg = 0;

	do {
		regTGSt =
			(ISP_RD32(CAM_REG_TG_INTER_ST(module)) & 0x00003F00) >>
			8;

		//regTGSt should never be 0 except HW issue
		//add "regTGSt == 0" for workaround
		if (regTGSt == 1 || regTGSt == 0)
			break;

		LOG_INF("%s: wait 1VD (%d)\n", moduleName, loopCnt);
		ret = ISP_WaitIrq(&waitirq);
		/* first wait is clear wait, others are non-clear wait */
		waitirq.EventInfo.Clear = ISP_IRQ_CLEAR_NONE;
	} while (--loopCnt);

	if (-ERESTARTSYS == ret) {
		LOG_INF("%s: interrupt by system signal, wait idle\n",
			moduleName);

		/* timer */
		time = ktime_get();
		m_sec = time;

		while (regTGSt != 1) {
			regTGSt = (ISP_RD32(CAM_REG_TG_INTER_ST(module)) &
				   0x00003F00) >>
				  8;

			/*timer */
			time = ktime_get();
			sec = time;
			/* wait time>timeoutMs, break */
			if ((sec - m_sec) > timeoutMs)
				break;
			//add "regTGSt == 0" for workaround
			if (regTGSt == 0)
				break;
		}
		if (regTGSt == 1) {
			LOG_INF("%s: wait idle done\n", moduleName);
		} else if (regTGSt == 0) {
			LOG_INF("%s: plz check regTGSt value\n", moduleName);
		} else {
			LOG_INF("%s: wait idle timeout(%lld)\n", moduleName,
				(sec - m_sec));
		}
	}

RESET:
	LOG_INF("%s: reset\n", moduleName);
	/* timer */
	time = ktime_get();
	m_sec = time;

	/* Reset */
	ISP_WR32(CAM_REG_CTL_SW_CTL(module), 0x0);
	ISP_WR32(CAM_REG_CTL_SW_CTL(module), 0x1);
	while ((ISP_RD32(CAM_REG_CTL_SW_CTL(module)) & 0x2) != 0x2) {
		/*LOG_DBG("%s resetting...\n", moduleName); */
		/*timer */
		time = ktime_get();
		sec = time;
		/* wait time>timeoutMs, break */
		if ((sec - m_sec) > timeoutMsRst) {
			LOG_INF("%s: wait SW idle timeout\n", moduleName);
			if (sec_on) {
				LOG_INF(
				"%d: wait SW idle timeout, reg(0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x)\n",
				module,
				(unsigned int)ISP_RD32(
					CAM_REG_TG_SEN_MODE(module)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_SW_CTL(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_SW_CTL(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_SW_CTL(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(CAMSYS_REG_CG_CON),
				(unsigned int)ISP_RD32(CAMSYS_REG_CG_SET),
				(unsigned int)ISP_RD32(CAMSYS_REG_CG_CLR));
			} else {
				LOG_INF(
				"%d: wait SW idle timeout, reg(0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x)\n",
				module,
				(unsigned int)ISP_RD32(
					CAM_REG_TG_SEN_MODE(module)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_SW_CTL(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_SW_CTL(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_SW_CTL(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(CAMSYS_REG_CG_CON),
				(unsigned int)ISP_RD32(CAMSYS_REG_CG_SET),
				(unsigned int)ISP_RD32(CAMSYS_REG_CG_CLR));
			}
			//dump smi for debugging
#if IS_ENABLED(CONFIG_MTK_SMI_EXT)
			if (smi_debug_bus_hang_detect(false, "camera_isp") != 0)
				LOG_NOTICE("ERR:smi_debug_bus_hang_detect");
#endif
			break;
		}
		//add "regTGSt == 0" for workaround
		if (regTGSt == 0)
			break;
	}

	ISP_WR32(CAM_REG_CTL_SW_CTL(module), 0x4);
	ISP_WR32(CAM_REG_CTL_SW_CTL(module), 0x0);
	regTGSt = (ISP_RD32(CAM_REG_TG_INTER_ST(module)) & 0x00003F00) >> 8;
	LOG_DBG("%s_TG_ST(%d)_SW_ST(0x%x)\n", moduleName, regTGSt,
		ISP_RD32(CAM_REG_CTL_SW_CTL(module)));

	/*disable CMOS */
	ISP_WR32(CAM_REG_TG_SEN_MODE(module),
		 (ISP_RD32(CAM_REG_TG_SEN_MODE(module)) & 0xfffffffe));
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline void ISP_StopSVHW(int module)
{
	unsigned int regTGSt = 0, loopCnt = 3;
	int ret = 0;
	struct ISP_WAIT_IRQ_STRUCT waitirq;
	ktime_t time;
	unsigned long long sec = 0, m_sec = 0;
	unsigned long long timeoutMs = 500000000; /*500ms */
	unsigned long long timeoutMsRst = 3000000; /*3ms */
	char moduleName[128];

	/* wait TG idle */
	switch (module) {
	case ISP_CAMSV0_IDX:
		strncpy(moduleName, "CAMSV0", 7);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAMSV_0_ST;
		break;
	case ISP_CAMSV1_IDX:
		strncpy(moduleName, "CAMSV1", 7);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAMSV_1_ST;
		break;
	case ISP_CAMSV2_IDX:
		strncpy(moduleName, "CAMSV2", 7);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAMSV_2_ST;
		break;
	case ISP_CAMSV3_IDX:
		strncpy(moduleName, "CAMSV3", 7);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAMSV_3_ST;
		break;
	case ISP_CAMSV4_IDX:
		strncpy(moduleName, "CAMSV4", 7);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAMSV_4_ST;
		break;
	case ISP_CAMSV5_IDX:
		strncpy(moduleName, "CAMSV5", 7);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAMSV_5_ST;
		break;
	case ISP_CAMSV6_IDX:
		strncpy(moduleName, "CAMSV6", 7);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAMSV_6_ST;
		break;
	case ISP_CAMSV7_IDX:
		strncpy(moduleName, "CAMSV7", 7);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAMSV_7_ST;
		break;
	default:
		strncpy(moduleName, "CAMSV7", 7);
		waitirq.Type = ISP_IRQ_TYPE_INT_CAMSV_7_ST;
		break;
	}
	waitirq.EventInfo.Clear = ISP_IRQ_CLEAR_WAIT;
	waitirq.EventInfo.Status = VS_INT_ST;
	waitirq.EventInfo.St_type = SIGNAL_INT;
	waitirq.EventInfo.Timeout = 0x100;
	waitirq.EventInfo.UserKey = 0x0;
	waitirq.bDumpReg = 0;

	do {
		regTGSt = (ISP_RD32(CAMSV_REG_TG_INTER_ST(module)) &
			   0x00003F00) >>
			  8;

		//regTGSt should never be 0 except HW issue
		//add "regTGSt == 0" for workaround
		if (regTGSt == 1 || regTGSt == 0)
			break;

		LOG_INF("%s: wait 1VD (%x)(%d)\n", moduleName, regTGSt,
			loopCnt);
		ret = ISP_WaitIrq(&waitirq);
		/* first wait is clear wait, others are non-clear wait */
		waitirq.EventInfo.Clear = ISP_IRQ_CLEAR_NONE;
	} while (--loopCnt);

	if (-ERESTARTSYS == ret) {
		LOG_INF("%s: interrupt by system signal, wait idle\n",
			moduleName);

		/* timer */
		time = ktime_get();
		m_sec = time;

		while (regTGSt != 1) {
			regTGSt = (ISP_RD32(CAMSV_REG_TG_INTER_ST(module)) &
				   0x00003F00) >>
				  8;

			/*timer */
			time = ktime_get();
			sec = time;
			/* wait time>timeoutMs, break */
			if ((sec - m_sec) > timeoutMs)
				break;
			//add "regTGSt == 0" for workaround
			if (regTGSt == 0)
				break;
		}
		if (regTGSt == 1) {
			LOG_INF("%s: wait idle done\n", moduleName);
		} else if (regTGSt == 0) {
			LOG_INF("%s: plz check regTGSt value\n", moduleName);
		} else {
			LOG_INF("%s: wait idle timeout(%lld)\n", moduleName,
				(sec - m_sec));
		}
	}
	if ((module >= ISP_CAMSV_START_IDX) && (module <= ISP_CAMSV_END_IDX))
		LOG_INF("%s: reset\n", moduleName);
	/* timer */
	time = ktime_get();
	m_sec = time;

	/* Reset */
	ISP_WR32(CAMSV_REG_SW_CTL(module), 0x0);
	ISP_WR32(CAMSV_REG_SW_CTL(module), 0x1);
	while ((ISP_RD32(CAMSV_REG_SW_CTL(module)) & 0x3) != 0x3) {
		/* camsv_top0 DMA2, camsv_2 need additional polling */
		/* register DMA_SOFT_RSTSTAT with IMGO and FUEO */
		if ((module == ISP_CAMSV1_IDX) &&
		    ((DMA_ST_MASK_CAMSV_IMGO_OR_UFO &
		      ISP_RD32(CAMSV_REG_DMA_SOF_RSTSTAT(module))) ==
		     DMA_ST_MASK_CAMSV_IMGO_OR_UFO)) {
			break;
		}
		/*LOG_DBG("%s resetting...\n", moduleName); */
		/*timer */
		time = ktime_get();
		sec = time;
		/* wait time>timeoutMs, break */
		if ((sec - m_sec) > timeoutMsRst) {
			LOG_INF("%s: wait SW idle timeout\n", moduleName);
			break;
		}
		//add "regTGSt == 0" for workaround
		if (regTGSt == 0)
			break;
	}
	ISP_WR32(CAMSV_REG_SW_CTL(module), 0x4); /* SW_RST:1 */
	ISP_WR32(CAMSV_REG_SW_CTL(module), 0x0);
	regTGSt = (ISP_RD32(CAMSV_REG_TG_INTER_ST(module)) & 0x00003F00) >> 8;
	LOG_DBG("%s_TG_ST(%d)_SW_ST(0x%x)\n", moduleName, regTGSt,
		ISP_RD32(CAMSV_REG_SW_CTL(module)));

	/*disable CMOS */
	ISP_WR32(CAMSV_REG_TG_SEN_MODE(module),
		 (ISP_RD32(CAMSV_REG_TG_SEN_MODE(module)) & 0xfffffffe));
}

/*******************************************************************************
 *
 ******************************************************************************/
static long ISP_ioctl(struct file *pFile, unsigned int Cmd, unsigned long Param)
{
	int Ret = 0;
	/*  */
	unsigned int DebugFlag[3] = {0};
	unsigned int Dapc_Reg[10] = {0};
	struct ISP_REG_IO_STRUCT RegIo;
	struct ISP_WAIT_IRQ_STRUCT IrqInfo;
	struct ISP_CLEAR_IRQ_STRUCT ClearIrq;
	struct ISP_USER_INFO_STRUCT *pUserInfo;
	unsigned int wakelock_ctrl;
	unsigned int module;
	/* FIX to avoid build warning */
	unsigned long flags; /* old: unsigned int flags; */
	int userKey = -1;
	struct ISP_REGISTER_USERKEY_STRUCT RegUserKey;
	int i;

	/*  */
	if (pFile->private_data == NULL) {
		LOG_NOTICE(
			"private_data is NULL,(process, pid, tgid)=(%s, %d, %d)\n",
			current->comm, current->pid, current->tgid);

		return -EFAULT;
	}
	/*  */
	pUserInfo = (struct ISP_USER_INFO_STRUCT *)(pFile->private_data);
	/*  */
	switch (Cmd) {
	case ISP_WAKELOCK_CTRL:
		if (copy_from_user(&wakelock_ctrl, (void *)Param,
				   sizeof(unsigned int)) != 0) {

			LOG_NOTICE("get ISP_WAKELOCK_CTRL from user fail\n");
			Ret = -EFAULT;
		} else {
			if (wakelock_ctrl == 1) { /* Enable wakelock */
				if (g_WaitLockCt) {
					g_WaitLockCt++;

					LOG_DBG("add wakelock cnt(%d)\n",
						g_WaitLockCt);

				} else {
#if IS_ENABLED(CONFIG_PM_WAKELOCKS)
					__pm_stay_awake(isp_wake_lock);
#else
					wake_lock(&isp_wake_lock);
#endif
					g_WaitLockCt++;

					LOG_DBG("wakelock enable!! cnt(%d)\n",
						g_WaitLockCt);
				}
			} else { /* Disable wakelock */
				if (g_WaitLockCt)
					g_WaitLockCt--;

				if (g_WaitLockCt) {

					LOG_DBG("subtract wakelock cnt(%d)\n",
						g_WaitLockCt);

				} else {
#if IS_ENABLED(CONFIG_PM_WAKELOCKS)
					__pm_relax(isp_wake_lock);
#else
					wake_unlock(&isp_wake_lock);
#endif
					LOG_DBG("wakelock disable!! cnt(%d)\n",
						g_WaitLockCt);
				}
			}
		}
		break;
	case ISP_GET_DROP_FRAME:
		if (copy_from_user(&DebugFlag[0], (void *)Param,
				   sizeof(unsigned int)) != 0) {
			LOG_NOTICE("get irq from user fail\n");
			Ret = -EFAULT;
		} else {

			if (DebugFlag[0] < ISP_IRQ_TYPE_INT_CAM_A_ST ||
			    DebugFlag[0] > ISP_IRQ_TYPE_INT_CAMSV_END_ST) {
				LOG_NOTICE("err TG(0x%x)\n", DebugFlag[0]);
				Ret = -EFAULT;
				break;
			}

			spin_lock_irqsave(&(IspInfo.SpinLockIrq[DebugFlag[0]]),
					  flags);

			DebugFlag[1] = FrameStatus[DebugFlag[0]];

			spin_unlock_irqrestore(
				&(IspInfo.SpinLockIrq[DebugFlag[0]]), flags);

			if (copy_to_user((void *)Param, &DebugFlag[1],
					 sizeof(unsigned int)) != 0) {

				LOG_NOTICE("copy to user fail\n");
				Ret = -EFAULT;
			}
		}
		break;
	case ISP_GET_INT_ERR:
		if (copy_to_user((void *)Param, (void *)g_ISPIntStatus,
				 sizeof(struct ISP_RAW_INT_STATUS) *
					 ISP_IRQ_TYPE_AMOUNT) != 0) {
			LOG_NOTICE("get int err fail\n");
		} else {
			memset((void *)g_ISPIntStatus, 0,
			       sizeof(g_ISPIntStatus));
		}
		break;
	case ISP_GET_DMA_ERR:
		if (copy_from_user(&DebugFlag[0], (void *)Param,
				   sizeof(unsigned int)) != 0) {

			LOG_NOTICE("get module fail\n");
			Ret = -EFAULT;
		} else {
			if (DebugFlag[0] >= (ISP_IRQ_TYPE_AMOUNT)) {
				LOG_NOTICE("module error(%d)\n", DebugFlag[0]);
				Ret = -EFAULT;
				break;
			}
			if (copy_to_user((void *)Param,
					 &g_DmaErr_CAM[DebugFlag[0]],
					 sizeof(unsigned int) * _cam_max_) != 0)
				LOG_NOTICE("get dma_err fail\n");
		}
		break;
	case ISP_GET_CUR_SOF:
		if (copy_from_user(&DebugFlag[0], (void *)Param,
				   sizeof(unsigned int)) != 0) {
			LOG_NOTICE("get cur sof from user fail\n");
			Ret = -EFAULT;
		} else {
			if (DebugFlag[0] < ISP_IRQ_TYPE_INT_CAM_A_ST ||
			    DebugFlag[0] >= ISP_IRQ_TYPE_AMOUNT) {

				LOG_NOTICE("cursof: error type(%d)\n",
					   DebugFlag[0]);

				Ret = -EFAULT;
				break;
			}
			DebugFlag[1] = sof_count[DebugFlag[0]];
		}
		if (copy_to_user((void *)Param, &DebugFlag[1],
				 sizeof(unsigned int)) != 0) {
			LOG_NOTICE("copy to user fail\n");
			Ret = -EFAULT;
		}
		break;
	case ISP_RESET_BY_HWMODULE: {
		if (copy_from_user(&module, (void *)Param,
				   sizeof(unsigned int)) != 0) {
			LOG_NOTICE("get hwmodule from user fail\n");
			Ret = -EFAULT;
		} else {
			if ((module >= ISP_DEV_NODE_NUM) ||
				(module < ISP_CAM_A_IDX)) {
				LOG_NOTICE(
				"ISP_RESET_BY_HWMODULE module is invalid\n");
				Ret = -EFAULT;
				break;
			}
			spin_lock(&(IspInfo.SpinLockClock));
			if (G_u4EnableClockCount[module] != 0) {
				spin_unlock(&(IspInfo.SpinLockClock));
				ISP_Reset(module);
			} else {
				spin_unlock(&(IspInfo.SpinLockClock));
				LOG_NOTICE("module is power off(%d)\n", module);
			}
		}
		break;
	}
	case ISP_READ_REGISTER: {
		if (copy_from_user(&RegIo, (void *)Param,
				   sizeof(struct ISP_REG_IO_STRUCT)) == 0) {
			/* 2nd layer behavoir of copy from user is */
			/* implemented in ISP_ReadReg(...) */
			Ret = ISP_ReadReg(&RegIo);
		} else {
			LOG_NOTICE("copy_from_user failed\n");
			Ret = -EFAULT;
		}
		break;
	}
	case ISP_WRITE_REGISTER: {
		if (copy_from_user(&RegIo, (void *)Param,
				   sizeof(struct ISP_REG_IO_STRUCT)) == 0) {
			/* 2nd layer behavoir of copy from user is */
			/* implemented in ISP_WriteReg(...) */
			if ((RegIo.Count * sizeof(struct ISP_REG_STRUCT)) > 0xFFFFF000) {
				Ret = -EFAULT;
				LOG_NOTICE("RegIo.Count error\n");
				goto EXIT;
			}
			Ret = ISP_WriteReg(&RegIo);
		} else {
			LOG_NOTICE("copy_from_user failed\n");
			Ret = -EFAULT;
		}
		break;
	}
	case ISP_WAIT_IRQ: {
		if (copy_from_user(&IrqInfo, (void *)Param,
				   sizeof(struct ISP_WAIT_IRQ_STRUCT)) == 0) {
			/*  */
			if ((IrqInfo.Type >= ISP_IRQ_TYPE_AMOUNT) ||
			    (IrqInfo.Type < 0)) {
				Ret = -EFAULT;
				LOG_NOTICE("invalid type(%d)\n", IrqInfo.Type);
				goto EXIT;
			}

			if ((IrqInfo.EventInfo.St_type >= ISP_IRQ_ST_AMOUNT) ||
			    (IrqInfo.EventInfo.St_type < 0)) {

				LOG_NOTICE(
					"invalid St_type(%d), max(%d), force St_type = 0\n",
					IrqInfo.EventInfo.St_type,
					ISP_IRQ_ST_AMOUNT);

				IrqInfo.EventInfo.St_type = 0;
			}

			if ((IrqInfo.EventInfo.UserKey >= IRQ_USER_NUM_MAX) ||
			    (IrqInfo.EventInfo.UserKey < 0)) {

				LOG_NOTICE(
					"invalid userKey(%d), max(%d), force userkey = 0\n",
					IrqInfo.EventInfo.UserKey,
					IRQ_USER_NUM_MAX);

				IrqInfo.EventInfo.UserKey = 0;
			}
			Ret = ISP_WaitIrq(&IrqInfo);
		} else {
			LOG_NOTICE("copy_from_user failed\n");
			Ret = -EFAULT;
		}
		break;
	}
	case ISP_CLEAR_IRQ: {
		if (copy_from_user(&ClearIrq, (void *)Param,
				   sizeof(struct ISP_CLEAR_IRQ_STRUCT)) == 0) {
			LOG_DBG("ISP_CLEAR_IRQ Type(%d)\n", ClearIrq.Type);

			if ((ClearIrq.Type >= ISP_IRQ_TYPE_AMOUNT) ||
			    (ClearIrq.Type < 0)) {
				Ret = -EFAULT;
				LOG_NOTICE("invalid type(%d)\n", ClearIrq.Type);
				goto EXIT;
			}

			if ((ClearIrq.EventInfo.St_type >= ISP_IRQ_ST_AMOUNT) ||
			    (ClearIrq.EventInfo.St_type < 0)) {

				LOG_NOTICE(
					"invalid St_type(%d), max(%d), force St_type = 0\n",
					ClearIrq.EventInfo.St_type,
					ISP_IRQ_ST_AMOUNT);

				ClearIrq.EventInfo.St_type = 0;
			}

			/*  */
			if ((ClearIrq.EventInfo.UserKey >= IRQ_USER_NUM_MAX) ||
			    (ClearIrq.EventInfo.UserKey < 0)) {

				LOG_NOTICE("errUserEnum(%d)",
					   ClearIrq.EventInfo.UserKey);

				Ret = -EFAULT;
				goto EXIT;
			}
			/*avoid line over 120 char per line */
			i = ClearIrq.EventInfo.UserKey;

			LOG_DBG(
			"ISP_CLEAR_IRQ:Type(%d),Status(0x%x),st_status(%d),IrqStatus(0x%x)\n",
			ClearIrq.Type,
			ClearIrq.EventInfo.Status,
			ClearIrq.EventInfo.St_type,
	IspInfo.IrqInfo.Status[ClearIrq.Type][ClearIrq.EventInfo.St_type][i]);

			spin_lock_irqsave(&(IspInfo.SpinLockIrq[ClearIrq.Type]),
					  flags);

			IspInfo.IrqInfo.Status[ClearIrq.Type]
					      [ClearIrq.EventInfo.St_type]
					      [ClearIrq.EventInfo.UserKey] &=
				(~ClearIrq.EventInfo.Status);

			spin_unlock_irqrestore(
				&(IspInfo.SpinLockIrq[ClearIrq.Type]), flags);
		} else {
			LOG_NOTICE("copy_from_user failed\n");
			Ret = -EFAULT;
		}
		break;
	}
	/*  */
	case ISP_REGISTER_IRQ_USER_KEY:
		if (copy_from_user(
			    &RegUserKey, (void *)Param,
			    sizeof(struct ISP_REGISTER_USERKEY_STRUCT)) == 0) {
			RegUserKey.userName[sizeof(RegUserKey.userName)-1] = '\0';
			userKey = ISP_REGISTER_IRQ_USERKEY(RegUserKey.userName);
			RegUserKey.userKey = userKey;
			if (copy_to_user((void *)Param, &RegUserKey,
					 sizeof(struct
						ISP_REGISTER_USERKEY_STRUCT)) !=
			    0)
				LOG_NOTICE("copy_to_user failed\n");

			if (RegUserKey.userKey < 0) {
				LOG_NOTICE("query irq user key fail\n");
				Ret = -1;
			}
		} else {
			LOG_NOTICE("copy from user fail\n");
		}

		break;
	/*  */
	case ISP_FLUSH_IRQ_REQUEST:
		if (copy_from_user(&IrqInfo, (void *)Param,
				   sizeof(struct ISP_WAIT_IRQ_STRUCT)) == 0) {
			if ((IrqInfo.EventInfo.UserKey >= IRQ_USER_NUM_MAX) ||
			    (IrqInfo.EventInfo.UserKey < 0)) {

				LOG_NOTICE("invalid userKey(%d), max(%d)\n",
					   IrqInfo.EventInfo.UserKey,
					   IRQ_USER_NUM_MAX);

				Ret = -EFAULT;
				break;
			}
			if ((IrqInfo.Type >= ISP_IRQ_TYPE_AMOUNT) ||
			    (IrqInfo.Type < 0)) {
				LOG_NOTICE("invalid type(%d), max(%d)\n",
					   IrqInfo.Type, ISP_IRQ_TYPE_AMOUNT);

				Ret = -EFAULT;
				break;
			}
			if ((IrqInfo.EventInfo.St_type >= ISP_IRQ_ST_AMOUNT) ||
			    (IrqInfo.EventInfo.St_type < 0)) {

				LOG_NOTICE(
					"invalid St_type(%d), max(%d), force St_type = 0\n",
					IrqInfo.EventInfo.St_type,
					ISP_IRQ_ST_AMOUNT);

				IrqInfo.EventInfo.St_type = 0;
			}

			Ret = ISP_FLUSH_IRQ(&IrqInfo);
		}
		break;
	case ISP_DEBUG_FLAG:
		if (copy_from_user(DebugFlag, (void *)Param,
				   sizeof(unsigned int)) == 0) {

			IspInfo.DebugMask = DebugFlag[0];

			/* LOG_DBG("FBC kernel debug level = %x\n", */
			/*          IspInfo.DebugMask); */
		} else {
			LOG_NOTICE("copy_from_user failed\n");
			Ret = -EFAULT;
		}
		break;
	case ISP_BUFFER_CTRL:
		Ret = ISP_Buf_CTRL_FUNC(Param);
		break;
	case ISP_VF_LOG:
		if (copy_from_user(DebugFlag, (void *)Param,
				   sizeof(unsigned int) * 2) == 0) {

			unsigned int vf, module = ISP_IRQ_TYPE_INT_CAM_A_ST;
			unsigned int cam_dmao = 0;

			if (DebugFlag[1] < ISP_CAMSYS_CONFIG_IDX ||
			    DebugFlag[1] > ISP_CAMSV7_IDX) {

				LOG_NOTICE("CAM Index is out of range:%d",
					   DebugFlag[1]);

				Ret = -EFAULT;
				break;
			}

			switch (DebugFlag[1]) {
			case ISP_CAM_A_IDX:
				module = ISP_IRQ_TYPE_INT_CAM_A_ST;
				break;
			case ISP_CAM_B_IDX:
				module = ISP_IRQ_TYPE_INT_CAM_B_ST;
				break;
			case ISP_CAM_C_IDX:
				module = ISP_IRQ_TYPE_INT_CAM_C_ST;
				break;
			case ISP_CAMSV0_IDX:
				module = ISP_IRQ_TYPE_INT_CAMSV_0_ST;
				break;
			case ISP_CAMSV1_IDX:
				module = ISP_IRQ_TYPE_INT_CAMSV_1_ST;
				break;
			case ISP_CAMSV2_IDX:
				module = ISP_IRQ_TYPE_INT_CAMSV_2_ST;
				break;
			case ISP_CAMSV3_IDX:
				module = ISP_IRQ_TYPE_INT_CAMSV_3_ST;
				break;
			case ISP_CAMSV4_IDX:
				module = ISP_IRQ_TYPE_INT_CAMSV_4_ST;
				break;
			case ISP_CAMSV5_IDX:
				module = ISP_IRQ_TYPE_INT_CAMSV_5_ST;
				break;
			case ISP_CAMSV6_IDX:
				module = ISP_IRQ_TYPE_INT_CAMSV_6_ST;
				break;
			case ISP_CAMSV7_IDX:
				module = ISP_IRQ_TYPE_INT_CAMSV_7_ST;
				break;
			}

			switch (DebugFlag[1]) {
			case ISP_CAM_A_IDX:
			case ISP_CAM_B_IDX:
			case ISP_CAM_C_IDX:
				/*0x3b04 */
				vf = ISP_RD32(CAM_REG_TG_VF_CON(DebugFlag[1]));
				break;
			case ISP_CAMSV0_IDX:
			case ISP_CAMSV1_IDX:
			case ISP_CAMSV2_IDX:
			case ISP_CAMSV3_IDX:
			case ISP_CAMSV4_IDX:
			case ISP_CAMSV5_IDX:
			case ISP_CAMSV6_IDX:
			case ISP_CAMSV7_IDX:
				/*0x0134 */
				vf = ISP_RD32(
					CAMSV_REG_TG_VF_CON(DebugFlag[1]));
				break;
			default:
				vf = ISP_RD32(CAM_REG_TG_VF_CON(DebugFlag[1]));
			}

			switch (DebugFlag[0]) {
			case 1: {
				if (sec_on) {
					cam_dmao = lock_reg.CAM_REG_CTL_DMA_EN
							   [DebugFlag[1]];
				} else {
					cam_dmao = ISP_RD32(CAM_REG_CTL_DMA_EN(
						DebugFlag[1]));
				}
				irq3a_print_vf_off[module] = 0;
				LOG_INF(
					"CAM_%d viewFinder is ON (SecOn:0x%x)\n",
					module, sec_on);

				if (vf & 0x1) {
					LOG_NOTICE(
						"CAM_%d: vf already enabled\n",
						module);
				} else {
					ISP_WR32(
						CAM_REG_TG_VF_CON(DebugFlag[1]),
						(vf + 0x1));
					/*For RAWI DMA Err debug*/
					ISP_WR32(CAM_REG_DBG_SET(ISP_CAM_A_IDX),
						0x00040000);
					ISP_WR32(CAM_REG_DBG_SET(ISP_CAM_B_IDX),
						0x00040000);
					ISP_WR32(CAM_REG_DBG_SET(ISP_CAM_C_IDX),
						0x00040000);
				}
				/*SCQ does not support CQ covery */
				cq_recovery[module] = (((ISP_RD32(CAM_REG_CAMCQ_CQ_EN(
						DebugFlag[1])) >> 20)
						& 0x1) ? 0 : 1);

#if (TIMESTAMP_QUEUE_EN == 1)
				memset((void *)&(IspInfo.TstpQInfo[module]), 0,
				       sizeof(struct ISP_TIMESTPQ_INFO_STRUCT));

				g1stSwP1Done[module] = MTRUE;
				gPrevSofTimestp[module] = 0;
#endif
				pstRTBuf[module]->ring_buf[_imgo_].active =
					((cam_dmao & 0x1) ? (MTRUE) : (MFALSE));

				pstRTBuf[module]->ring_buf[_rrzo_].active =
					((cam_dmao & 0x4) ? (MTRUE) : (MFALSE));

				pstRTBuf[module]->ring_buf[_lcso_].active =
					((cam_dmao & 0x2000) ? (MTRUE)
							   : (MFALSE));

				pstRTBuf[module]->ring_buf[_lcesho_].active =
					((cam_dmao & 0x4000) ? (MTRUE)
							   : (MFALSE));
				pstRTBuf[module]->ring_buf[_lmvo_].active =
					((cam_dmao & 0x10000) ? (MTRUE)
							     : (MFALSE));

				pstRTBuf[module]->ring_buf[_rsso_].active =
					((cam_dmao & 0x20000) ? (MTRUE)
							     : (MFALSE));

				/*reset 1st sof flag when vf is enabled */
				g1stSof[module] = MTRUE;
				break;
			}
			case 0: {
				irq3a_print_vf_off[module] = 1;
				LOG_INF("CAM_%d viewFinder is OFF\n", module);

				if (vf & 0x1) {
					ISP_WR32(
						CAM_REG_TG_VF_CON(DebugFlag[1]),
						(vf - 0x1));
					cq_recovery[module] = 0;
				} else {
					LOG_NOTICE(
						"CAM_%d: vf already disabled\n",
						module);
				}
				break;
			}
			/* CAMSV */
			case 11: {
				LOG_INF("CAMSV_%d viewFinder is ON\n", module);

				cam_dmao = (ISP_RD32(CAMSV_REG_MODULE_EN(
						    DebugFlag[1])) &
					    0x10);

				LOG_DBG("CAMSV_%d:[DMA_EN]:0x%x\n", module,
					cam_dmao);

				vf = ISP_RD32(
					CAMSV_REG_TG_VF_CON(DebugFlag[1]));

				if (vf & 0x1) {
					LOG_NOTICE(
					"CAMSV_%d: vf already enabled\n",
					(DebugFlag[1] - ISP_CAMSV_START_IDX));
				} else {
					ISP_WR32(CAMSV_REG_TG_VF_CON(
							 DebugFlag[1]),
						 (vf + 0x1));
				}

				pstRTBuf[module]
					->ring_buf[_camsv_imgo_]
					.active =
					((cam_dmao & 0x10) ? (MTRUE)
							   : (MFALSE));

				/*reset 1st sof flag when vf is enabled */
				g1stSof[module] = MTRUE;
				break;
			}
			case 10: {
				LOG_INF("CAMSV_%d viewFinder is OFF\n",
					(DebugFlag[1] - ISP_CAMSV_START_IDX));

				vf = ISP_RD32(
					CAMSV_REG_TG_VF_CON(DebugFlag[1]));

				if (vf & 0x1) {
					ISP_WR32(CAMSV_REG_TG_VF_CON(
							 DebugFlag[1]),
						 (vf - 0x1));
				} else {
					LOG_NOTICE(
					"CAMSV_%d: vf already disalbed\n",
					(DebugFlag[1] - ISP_CAMSV_START_IDX));
				}
				break;
			}
			}

			switch (DebugFlag[0]) {
			/* CAM */
			case 0:
			case 1:
				LOG_NOTICE("CAM_%d_REG_TG_VF_CON 0x%08x\n",
					   (DebugFlag[1] - ISP_CAM_A_IDX),
					   ISP_RD32(CAM_REG_TG_VF_CON(
						   DebugFlag[1])));
				break;
			/* CAMSV */
			case 10:
			case 11:
				LOG_NOTICE("CAMSV_%d_REG_TG_VF_CON 0x%08x\n",
					   (DebugFlag[1] - ISP_CAMSV_START_IDX),
					   ISP_RD32(CAMSV_REG_TG_VF_CON(
						   DebugFlag[1])));
				break;
			default:
				LOG_NOTICE("No support this debugFlag[0] %d",
					   DebugFlag[0]);
				Ret = -EFAULT;
				break;
			}
		} else {
			LOG_NOTICE("copy_from_user failed\n");
			Ret = -EFAULT;
		}
		break;
	case ISP_GET_START_TIME:
		if (copy_from_user(DebugFlag, (void *)Param,
				   sizeof(unsigned int) * 3) == 0) {
			struct S_START_T Tstp = {
				0,
			};

#if (TIMESTAMP_QUEUE_EN == 1)
			unsigned int dma_id = DebugFlag[1];

			if (_cam_max_ == DebugFlag[1]) {
				/* only for wait timestamp to ready */
				Ret = ISP_WaitTimestampReady(DebugFlag[0],
							     DebugFlag[2]);

				break;
			}

			switch (DebugFlag[0]) {
			case ISP_IRQ_TYPE_INT_CAM_A_ST:
			case ISP_IRQ_TYPE_INT_CAM_B_ST:
				if (ISP_PopBufTimestamp(DebugFlag[0], dma_id,
							Tstp) != 0) {
					LOG_NOTICE(
						"Get Buf sof timestamp fail");
				}
				break;
			case ISP_IRQ_TYPE_INT_CAMSV_0_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_1_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_2_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_3_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_4_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_5_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_6_ST:
			case ISP_IRQ_TYPE_INT_CAMSV_7_ST:
				Tstp.sec = gSTime[DebugFlag[0]].sec;
				Tstp.usec = gSTime[DebugFlag[0]].usec;
				break;
			default:
				LOG_NOTICE("unsupported module:0x%x\n",
					   DebugFlag[0]);

				Ret = -EFAULT;
				break;
			}
			if (Ret != 0)
				break;
#else
			if (DebugFlag[0] < ISP_IRQ_TYPE_INT_CAM_A_ST ||
			    DebugFlag[0] > ISP_IRQ_TYPE_INT_CAMSV_7_ST) {

				LOG_NOTICE("unsupported module:0x%x",
					   DebugFlag[0]);

				Ret = -EFAULT;
				break;
			}

			if (g1stSof[DebugFlag[0]] == MFALSE) {
				Tstp.sec = gSTime[DebugFlag[0]].sec;
				Tstp.usec = gSTime[DebugFlag[0]].usec;
			} else {
				Tstp.sec = Tstp.usec = 0;
			}
#endif

			if (copy_to_user((void *)Param, &Tstp,
					 sizeof(struct S_START_T)) != 0) {
				LOG_NOTICE("copy_to_user failed");
				Ret = -EFAULT;
			}
		}
		break;
	case ISP_GET_SUPPORTED_ISP_CLOCKS: {
		struct ISP_CLK_INFO ispclks;

		memset((void *)&ispclks, 0x0, sizeof(struct ISP_CLK_INFO));

		ispclks.clklevelcnt = (unsigned char)ISP_SetPMQOS(
			E_CLK_SUPPORTED, ISP_IRQ_TYPE_INT_CAM_A_ST,
			(unsigned int *)ispclks.clklevel);

		if (ispclks.clklevelcnt >= ISP_CLK_LEVEL_CNT) {

			LOG_NOTICE(
				"clklevelcnt is exceeded_%d, memory corruption\n",
				ispclks.clklevelcnt);

			Ret = -EFAULT;
			break;
		}

		if (copy_to_user((void *)Param, &ispclks,
				 sizeof(struct ISP_CLK_INFO)) != 0) {
			LOG_NOTICE("copy_to_user failed");
			Ret = -EFAULT;
		}
	} break;
	case ISP_DFS_CTRL: {
		unsigned int dfs_ctrl;

		if (copy_from_user(&dfs_ctrl, (void *)Param,
				   sizeof(unsigned int)) == 0) {
			ISP_SetPMQOS(E_CLK_CLR, ISP_IRQ_TYPE_INT_CAM_A_ST,
				     NULL);
		} else {
			LOG_NOTICE("ISP_DFS_CTRL copy_from_user failed\n");
			Ret = -EFAULT;
		}
	} break;
	case ISP_DFS_UPDATE: {
		unsigned int dfs_update;

		if (copy_from_user(&dfs_update, (void *)Param,
				   sizeof(unsigned int)) == 0) {
			ISP_SetPMQOS(E_CLK_UPDATE, ISP_IRQ_TYPE_INT_CAM_A_ST,
				     &dfs_update);
		} else {
			LOG_NOTICE("ISP_DFS_UPDATE copy_from_user failed\n");
			Ret = -EFAULT;
		}
	} break;
	case ISP_GET_CUR_ISP_CLOCK: {
		struct ISP_GET_CLK_INFO getclk;
		unsigned int clk[2] = {0};

		ISP_SetPMQOS(E_CLK_CUR, ISP_IRQ_TYPE_INT_CAM_A_ST, clk);
		getclk.curClk = clk[0];
		getclk.targetClk = clk[1];
		if (copy_to_user((void *)Param, &getclk,
				 sizeof(struct ISP_GET_CLK_INFO)) != 0) {

			LOG_NOTICE("copy_to_user failed");
			Ret = -EFAULT;
		}
	} break;
	case ISP_GET_GLOBAL_TIME: {
#ifdef TS_BOOT_T
#define TS_TYPE (2)
#else
#define TS_TYPE (1)
#endif
		u64 hwTickCnt[TS_TYPE], globalTime[TS_TYPE];

		if (copy_from_user(hwTickCnt, (void *)Param,
				   sizeof(u64) * TS_TYPE) == 0) {
#ifdef ENABLE_TIMESYNC_HANDLE
			globalTime[0] =
				archcounter_timesync_to_monotonic(hwTickCnt[0]);
			do_div(globalTime[0], 1000); /* ns to us */
#ifdef TS_BOOT_T
			globalTime[1] =
				archcounter_timesync_to_boot(hwTickCnt[0]);
			do_div(globalTime[1], 1000); /* ns to us */
#endif
#else
			/* Todo: need to get the global time. */
			globalTime[0] = 0;
			globalTime[1] = 0;
#endif
			if (copy_to_user((void *)Param, globalTime,
					 sizeof(u64) * TS_TYPE) != 0) {
				LOG_NOTICE(
					"ISP_GET_GLOBAL_TIME copy_to_user failed");
				Ret = -EFAULT;
			}
		} else {
			LOG_NOTICE(
				"ISP_GET_GLOBAL_TIME copy_from_user failed\n");
			Ret = -EFAULT;
		}
	} break;
	case ISP_SET_PM_QOS_INFO: {
		struct ISP_PM_QOS_INFO_STRUCT pm_qos_info;

		if (copy_from_user(&pm_qos_info, (void *)Param,
				   sizeof(struct ISP_PM_QOS_INFO_STRUCT)) ==
		    0) {
			ISP_SetPMQOS(E_BW_UPDATE, pm_qos_info.module,
				     (unsigned int *)pm_qos_info.port_bw);
		} else {
			LOG_NOTICE(
				"ISP_SET_PM_QOS_INFO copy_from_user failed\n");

			Ret = -EFAULT;
		}
	} break;
	case ISP_SET_PM_QOS:
		if (copy_from_user(DebugFlag, (void *)Param,
				   sizeof(unsigned int) * 2) == 0) {
			ISP_SetPMQOS(E_BW_CLR, DebugFlag[1], DebugFlag);
		} else {
			LOG_NOTICE("ISP_SET_PM_QOS copy_from_user failed\n");
			Ret = -EFAULT;
		}
		break;
	case ISP_GET_VSYNC_CNT:
		if (copy_from_user(&DebugFlag[0], (void *)Param,
				   sizeof(unsigned int)) != 0) {
			LOG_NOTICE("get cur sof from user fail");
			Ret = -EFAULT;
		} else {

			if (DebugFlag[0] < ISP_IRQ_TYPE_INT_CAM_A_ST ||
			    DebugFlag[0] >= ISP_IRQ_TYPE_AMOUNT) {

				LOG_NOTICE("err TG(0x%x)\n", DebugFlag[0]);
				Ret = -EFAULT;
				break;
			}

			DebugFlag[1] = Vsync_cnt[DebugFlag[0]];
		}
		if (copy_to_user((void *)Param, &DebugFlag[1],
				 sizeof(unsigned int)) != 0) {
			LOG_NOTICE("copy to user fail");
			Ret = -EFAULT;
		}
		break;
	case ISP_RESET_VSYNC_CNT: {
		enum ISP_IRQ_TYPE_ENUM i = ISP_IRQ_TYPE_INT_CAM_A_ST;

		for (i = ISP_IRQ_TYPE_INT_CAM_A_ST; i < ISP_IRQ_TYPE_AMOUNT;
		     i++)
			Vsync_cnt[i] = 0;
	} break;
	case SV_SET_PM_QOS:
		if (copy_from_user(DebugFlag, (void *)Param,
				   sizeof(unsigned int) * 2) == 0) {
			SV_SetPMQOS(E_BW_CLR, DebugFlag[1], DebugFlag);
		} else {
			LOG_NOTICE("ISP_SET_PM_QOS copy_from_user failed\n");
			Ret = -EFAULT;
		}
		break;
	case SV_SET_PM_QOS_INFO: {
		struct ISP_PM_QOS_INFO_STRUCT pm_qos_info;

		if (copy_from_user(&pm_qos_info, (void *)Param,
				   sizeof(struct ISP_PM_QOS_INFO_STRUCT)) ==
		    0) {
			SV_SetPMQOS(E_BW_UPDATE, pm_qos_info.module,
				    (unsigned int *)pm_qos_info.port_bw);
		} else {
			LOG_NOTICE(
				"ISP_SET_PM_QOS_INFO copy_from_user failed\n");
			Ret = -EFAULT;
		}
	} break;
	case SV_DFS_UPDATE: {
		unsigned int dfs_update;

		if (copy_from_user(&dfs_update, (void *)Param,
				   sizeof(unsigned int)) == 0) {
			SV_SetPMQOS(E_CLK_UPDATE, ISP_IRQ_TYPE_INT_CAMSV_START_ST,
				    &dfs_update);
		} else {
			LOG_NOTICE("SV_DFS_UPDATE copy_from_user failed\n");
			Ret = -EFAULT;
		}
	} break;
	case SV_GET_CUR_ISP_CLOCK: {
		struct ISP_GET_CLK_INFO getclk;
		unsigned int clk[2] = {0};

		SV_SetPMQOS(E_CLK_CUR, ISP_IRQ_TYPE_INT_CAMSV_START_ST, clk);
		getclk.curClk = clk[0];
		getclk.targetClk = clk[1];
		if (copy_to_user((void *)Param, &getclk,
				 sizeof(struct ISP_GET_CLK_INFO)) != 0) {
			LOG_NOTICE("copy_to_user failed");
			Ret = -EFAULT;
		}
	} break;
	case SV_GET_SUPPORTED_ISP_CLOCKS: {
		struct ISP_CLK_INFO ispclks;

		memset((void *)&ispclks, 0x0, sizeof(struct ISP_CLK_INFO));

		ispclks.clklevelcnt = (unsigned char)SV_SetPMQOS(
			E_CLK_SUPPORTED, ISP_IRQ_TYPE_INT_CAMSV_START_ST,
			(unsigned int *)ispclks.clklevel);

		if (ispclks.clklevelcnt >= ISP_CLK_LEVEL_CNT) {

			LOG_NOTICE(
				"clklevelcnt is exceeded_%d, memory corruption\n",
				ispclks.clklevelcnt);

			Ret = -EFAULT;
			break;
		}

		if (copy_to_user((void *)Param, &ispclks,
				 sizeof(struct ISP_CLK_INFO)) != 0) {
			LOG_NOTICE("copy_to_user failed");
			Ret = -EFAULT;
		}
	} break;
	case ISP_NOTE_CQTHR0_BASE: {
		struct ISP_CQ0_NOTE_INFO cq0_note;
		unsigned int index = 0;
		memset((void *)&cq0_note, 0x0, sizeof(struct ISP_CQ0_NOTE_INFO));

		if (copy_from_user(&cq0_note, (void *)Param,
			sizeof(struct ISP_CQ0_NOTE_INFO)) != 0) {
			LOG_NOTICE("copy to user fail");
			Ret = -EFAULT;
			break;
		}

		index = cq0_note.cq0_data[CAM_A][0] - ISP_CAM_A_IDX;
		if (index <= (ISP_CAM_C_IDX - ISP_CAM_A_IDX)) {
			if (cq0_note.cq0_data[CAM_A][1] != 0) {
				g_cqBaseAddr[index][0] = cq0_note.cq0_data[CAM_A][1];
				g_cq0NextBA[index][0] = cq0_note.cq0_data[CAM_A][2];
				/*LOG_NOTICE("(CAM A)CQ0 pa 0x%x, 0x%x, 0x%x",
				 *cq0_note.cq0_data[CAM_A][0], cq0_note.cq0_data[CAM_A][1],
				 * cq0_note.cq0_data[CAM_A][2]);
				 */
			}
		}
		index = cq0_note.cq0_data[CAM_B][0] - ISP_CAM_A_IDX;
		if (index <= (ISP_CAM_C_IDX - ISP_CAM_A_IDX)) {
			if (cq0_note.cq0_data[CAM_B][1] != 0) {
				g_cqBaseAddr[index][0] = cq0_note.cq0_data[CAM_B][1];
				g_cq0NextBA[index][0] = cq0_note.cq0_data[CAM_B][2];
				/*LOG_NOTICE("(CAM B)CQ0 pa 0x%x, 0x%x, 0x%x",
				 *cq0_note.cq0_data[CAM_B][0], cq0_note.cq0_data[CAM_B][1],
				 *cq0_note.cq0_data[CAM_B][2]);
				 */
			}
		}

		if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
			index = cq0_note.cq0_data[CAM_C][0] - ISP_CAM_A_IDX;
			if (index <= (ISP_CAM_C_IDX - ISP_CAM_A_IDX)) {
				if (cq0_note.cq0_data[CAM_C][1] != 0) {
					g_cqBaseAddr[index][0] = cq0_note.cq0_data[CAM_C][1];
					g_cq0NextBA[index][0] = cq0_note.cq0_data[CAM_C][2];
					/*LOG_NOTICE("(CAM C)CQ0 pa 0x%x, 0x%x, 0x%x",
					 *cq0_note.cq0_data[CAM_C][0], cq0_note.cq0_data[CAM_C][1],
					 *cq0_note.cq0_data[CAM_C][2]);
					 */
				}
			}
		}

	} break;

	case ISP_CQ_SW_PATCH: {
		static struct ISP_MULTI_RAW_CONFIG multiRAWConfig = {
			0,
		};
		static uintptr_t Addr[ISP_IRQ_TYPE_INT_CAMSV_START_ST] = {
			0,
		};

		if (copy_from_user(&multiRAWConfig, (void *)Param,
				   sizeof(struct ISP_MULTI_RAW_CONFIG)) == 0) {
			if (multiRAWConfig.HWmodule < 0 ||
			    multiRAWConfig.HWmodule >
				    (ISP_IRQ_TYPE_INT_CAMSV_START_ST - 1)) {

				LOG_NOTICE("Wrong HWmodule:%d",
					   multiRAWConfig.HWmodule);

				Ret = -EFAULT;
				break;
			}

			Addr[multiRAWConfig.HWmodule] =
				multiRAWConfig.cq_base_pAddr;
		}

		switch (multiRAWConfig.slave_cam_num) {
		case 1: /* 2 RAWS to 1 tg for static twin mode */
			if (multiRAWConfig.master_module ==
				    ISP_IRQ_TYPE_INT_CAM_A_ST &&
			    multiRAWConfig.twin_module ==
				    ISP_IRQ_TYPE_INT_CAM_C_ST) {
				if ((Addr[ISP_IRQ_TYPE_INT_CAM_A_ST] != 0) &&
				    (Addr[ISP_IRQ_TYPE_INT_CAM_C_ST] != 0)) {
					unsigned long flags;

					spin_lock(&(IspInfo.SpinLockClock));
					if (G_u4EnableClockCount[ISP_CAM_A_IDX] != 0 &&
						G_u4EnableClockCount[ISP_CAM_C_IDX] != 0) {
						spin_unlock(&(IspInfo.SpinLockClock));

						spin_lock_irqsave(
							&IspInfo.SpinLockIrq[0], flags);

						ISP_WR32(CAM_REG_CTL_CD_DONE_SEL(
								 ISP_CAM_A_IDX),
							 0x10000);

						ISP_WR32(CAM_REG_CQ_THR0_BASEADDR(
								 ISP_CAM_C_IDX),
							 Addr[2]);

						ISP_WR32(CAM_REG_CQ_THR0_BASEADDR(
								 ISP_CAM_A_IDX),
							 Addr[0]);

						Addr[ISP_IRQ_TYPE_INT_CAM_A_ST] = Addr
							[ISP_IRQ_TYPE_INT_CAM_C_ST] = 0;

						spin_unlock_irqrestore(
							&IspInfo.SpinLockIrq[0], flags);
					} else {
						spin_unlock(&(IspInfo.SpinLockClock));
						LOG_NOTICE("hw module is power off\n");
					}
				}
			} else if (multiRAWConfig.master_module ==
					   ISP_IRQ_TYPE_INT_CAM_B_ST &&
				   multiRAWConfig.twin_module ==
					   ISP_IRQ_TYPE_INT_CAM_C_ST) {

				if ((Addr[ISP_IRQ_TYPE_INT_CAM_B_ST] != 0) &&
				    (Addr[ISP_IRQ_TYPE_INT_CAM_C_ST] != 0)) {
					unsigned long flags;

					spin_lock(&(IspInfo.SpinLockClock));
					if (G_u4EnableClockCount[ISP_CAM_B_IDX] != 0 &&
						G_u4EnableClockCount[ISP_CAM_C_IDX] != 0) {
						spin_unlock(&(IspInfo.SpinLockClock));

						spin_lock_irqsave(
							&IspInfo.SpinLockIrq[1], flags);

						ISP_WR32(CAM_REG_CTL_CD_DONE_SEL(
								 ISP_CAM_B_IDX),
							 0x10000);

						ISP_WR32(CAM_REG_CQ_THR0_BASEADDR(
								 ISP_CAM_C_IDX),
							 Addr[2]);

						ISP_WR32(CAM_REG_CQ_THR0_BASEADDR(
								 ISP_CAM_B_IDX),
							 Addr[1]);

						Addr[ISP_IRQ_TYPE_INT_CAM_B_ST] = Addr
							[ISP_IRQ_TYPE_INT_CAM_C_ST] = 0;

						spin_unlock_irqrestore(
							&IspInfo.SpinLockIrq[1], flags);
					} else {
						spin_unlock(&(IspInfo.SpinLockClock));
						LOG_NOTICE("module is power off\n");
					}
				}
			} else {
				if ((Addr[ISP_IRQ_TYPE_INT_CAM_A_ST] != 0) &&
				    (Addr[ISP_IRQ_TYPE_INT_CAM_B_ST] != 0)) {
					unsigned long flags;

					spin_lock(&(IspInfo.SpinLockClock));
					if (G_u4EnableClockCount[ISP_CAM_A_IDX] != 0 &&
						G_u4EnableClockCount[ISP_CAM_B_IDX] != 0) {
						spin_unlock(&(IspInfo.SpinLockClock));

						spin_lock_irqsave(
							&IspInfo.SpinLockIrq[0], flags);

						ISP_WR32(CAM_REG_CTL_CD_DONE_SEL(
								 ISP_CAM_A_IDX),
							 0x10000);

						ISP_WR32(CAM_REG_CQ_THR0_BASEADDR(
								 ISP_CAM_B_IDX),
							 Addr[1]);

						ISP_WR32(CAM_REG_CQ_THR0_BASEADDR(
								 ISP_CAM_A_IDX),
							 Addr[0]);

						Addr[ISP_IRQ_TYPE_INT_CAM_A_ST] = Addr
							[ISP_IRQ_TYPE_INT_CAM_B_ST] = 0;

						spin_unlock_irqrestore(
							&IspInfo.SpinLockIrq[0], flags);
					} else {
						spin_unlock(&(IspInfo.SpinLockClock));
						LOG_NOTICE("module is power off\n");
					}
				}
			}
			break;
		case 2: /* 3 RAWS to 1 tg for static twin mode */
			if ((Addr[ISP_IRQ_TYPE_INT_CAM_A_ST] != 0) &&
			    (Addr[ISP_IRQ_TYPE_INT_CAM_B_ST] != 0) &&
			    (Addr[ISP_IRQ_TYPE_INT_CAM_C_ST] != 0)) {
				unsigned long flags;

				spin_lock(&(IspInfo.SpinLockClock));
				if (G_u4EnableClockCount[ISP_CAM_A_IDX] == 0 ||
					G_u4EnableClockCount[ISP_CAM_B_IDX] == 0 ||
					G_u4EnableClockCount[ISP_CAM_C_IDX] == 0) {
					spin_unlock(&(IspInfo.SpinLockClock));
					LOG_NOTICE("module is power off\n");
					break;
				}
				spin_unlock(&(IspInfo.SpinLockClock));

				spin_lock_irqsave(
					&IspInfo.SpinLockIrq
						 [ISP_IRQ_TYPE_INT_CAM_A_ST],
					flags);

				ISP_WR32(CAM_REG_CTL_CD_DONE_SEL(ISP_CAM_A_IDX),
					 0x10000);

				ISP_WR32(
					CAM_REG_CQ_THR0_BASEADDR(ISP_CAM_C_IDX),
					Addr[2]);

				ISP_WR32(
					CAM_REG_CQ_THR0_BASEADDR(ISP_CAM_B_IDX),
					Addr[1]);

				ISP_WR32(
					CAM_REG_CQ_THR0_BASEADDR(ISP_CAM_A_IDX),
					Addr[0]);

				Addr[ISP_IRQ_TYPE_INT_CAM_A_ST] =
					Addr[ISP_IRQ_TYPE_INT_CAM_B_ST] = 0;
				Addr[ISP_IRQ_TYPE_INT_CAM_C_ST] = 0;

				spin_unlock_irqrestore(
					&IspInfo.SpinLockIrq
						 [ISP_IRQ_TYPE_INT_CAM_A_ST],
					flags);
			}
			break;
		};
	} break;
#if (SMI_LARB_MMU_CTL == 1)
	case ISP_LARB_MMU_CTL: {

		struct ISP_LARB_MMU_STRUCT larbInfo;

		if (copy_from_user(&larbInfo, (void *)Param,
				   sizeof(struct ISP_LARB_MMU_STRUCT)) != 0) {
			LOG_NOTICE("copy_from_user LARB_MMU_CTL failed\n");
			Ret = -EFAULT;
			goto EXIT;
		}

		switch (larbInfo.LarbNum) {
		case 13:
		case 14:
		case 16:
		case 17:
		case 18:
			break;
		default:
			LOG_NOTICE("Wrong SMI_LARB port=%d\n",
				   larbInfo.LarbNum);

			Ret = -EFAULT;
			goto EXIT;
		}

		if ((SMI_LARB_BASE[larbInfo.LarbNum] == NULL) ||
		    (larbInfo.regOffset >= 0x1000)) {
			LOG_NOTICE(
				"Wrong SMI_LARB port=%d base addr=%pK offset=0x%x\n",
				larbInfo.LarbNum,
				SMI_LARB_BASE[larbInfo.LarbNum],
				larbInfo.regOffset);

			Ret = -EFAULT;
			goto EXIT;
		}

		*(unsigned int *)((char *)SMI_LARB_BASE[larbInfo.LarbNum] +
				  larbInfo.regOffset) = larbInfo.regVal;

	} break;
#endif
	case ISP_SET_SEC_DAPC_REG:
		if (copy_from_user(Dapc_Reg, (void *)Param,
				   sizeof(unsigned int) * 10) != 0) {
			LOG_NOTICE("get ISP_SET_SEC_DAPC_REG from user fail\n");
			Ret = -EFAULT;
		} else {
			if (Dapc_Reg[0] < ISP_CAM_A_IDX ||
			    Dapc_Reg[0] >= ISP_CAMSV_START_IDX) {
				LOG_NOTICE("module index(0x%x) error\n",
					   Dapc_Reg[0]);
				Ret = -EFAULT;
				break;
			}

			spin_lock(&(IspInfo.SpinLockClock));
			if (G_u4EnableClockCount[Dapc_Reg[0]] == 0) {
				spin_unlock(&(IspInfo.SpinLockClock));
				LOG_NOTICE("module is power off(0x%x)\n",
					   Dapc_Reg[0]);
				Ret = -EFAULT;
				break;
			}
			spin_unlock(&(IspInfo.SpinLockClock));

			if (Dapc_Reg[1] == MTRUE) {
				sec_on = Dapc_Reg[1];
				lock_reg.CAM_REG_CTL_EN[Dapc_Reg[0]] =
					Dapc_Reg[2];
				lock_reg.CAM_REG_CTL_EN2[Dapc_Reg[0]] =
					Dapc_Reg[3];
				lock_reg.CAM_REG_CTL_EN3[Dapc_Reg[0]] =
					Dapc_Reg[4];
				//lock_reg.CAM_REG_CTL_EN4[Dapc_Reg[0]] =
				//	Dapc_Reg[5];
				lock_reg.CAM_REG_CTL_DMA_EN[Dapc_Reg[0]] =
					Dapc_Reg[5];
				lock_reg.CAM_REG_CTL_DMA2_EN[Dapc_Reg[0]] =
					Dapc_Reg[6];
				lock_reg.CAM_REG_CTL_SEL[Dapc_Reg[0]] =
					Dapc_Reg[7];
				lock_reg.CAM_REG_CTL_SEL2[Dapc_Reg[0]] =
					Dapc_Reg[8];
				LOG_INF(
				"[DAPC]EN:0x%x EN2:0x%x EN3:0x%x\n",
					lock_reg.CAM_REG_CTL_EN[Dapc_Reg[0]],
					lock_reg.CAM_REG_CTL_EN2[Dapc_Reg[0]],
					lock_reg.CAM_REG_CTL_EN3[Dapc_Reg[0]]);
				LOG_INF(
				"[DAPC]DMA:0x%x DMA2:0x%x SEL:0x%x SEL2:0x%x\n",
					lock_reg.CAM_REG_CTL_DMA_EN
						[Dapc_Reg[0]],
					lock_reg.CAM_REG_CTL_DMA2_EN
						[Dapc_Reg[0]],
					lock_reg.CAM_REG_CTL_SEL[Dapc_Reg[0]],
					lock_reg.CAM_REG_CTL_SEL2[Dapc_Reg[0]]);
			} else {
				LOG_NOTICE("get wrong sec status (0x%x)\n",
					   Dapc_Reg[1]);
				Ret = -EFAULT;
			}
		}
		break;
	case ISP_GET_CUR_HWP1DONE:
		if (copy_from_user(&DebugFlag[0], (void *)Param,
				   sizeof(unsigned int)) != 0) {
			LOG_NOTICE("get cur hw p1 done from user fail\n");
			Ret = -EFAULT;
		} else {
			if (DebugFlag[0] < ISP_IRQ_TYPE_INT_CAM_A_ST ||
			    DebugFlag[0] >= ISP_IRQ_TYPE_AMOUNT) {

				LOG_NOTICE("cur hw p1 done: error type(%d)\n",
					   DebugFlag[0]);

				Ret = -EFAULT;
				break;
			}
			DebugFlag[1] = HwP1Done_cnt[DebugFlag[0]];
		}
		if (copy_to_user((void *)Param, &DebugFlag[1],
				 sizeof(unsigned int)) != 0) {
			LOG_NOTICE("copy to user fail\n");
			Ret = -EFAULT;
		}
		break;
	case ISP_SET_VIR_CQCNT: {
		unsigned int _cq_cnt[2] = {0};

		if (copy_from_user(&_cq_cnt, (void *)Param,
			sizeof(unsigned int) * 2) == 0) {
			LOG_DBG("hw_module:%d VirCQ count from user: %d\n",
				_cq_cnt[0], _cq_cnt[1]);

			if (_cq_cnt[0] <= (ISP_IRQ_TYPE_INT_CAM_C_ST -
				ISP_IRQ_TYPE_INT_CAM_A_ST))
				g_virtual_cq_cnt[_cq_cnt[0]] = _cq_cnt[1];
			else
				LOG_NOTICE("invalid HW module(%d)\n",
					_cq_cnt[0]);
		} else {
			LOG_NOTICE(
				"Virtual CQ count copy_from_user failed\n");
			Ret = -EFAULT;
		}
	} break;
	case ISP_POWER_CTRL: {
		unsigned int pwrctl[2] = {0};
		unsigned int i = 0;

		if (copy_from_user(&pwrctl, (void *)Param,
			sizeof(unsigned int) * 2) == 0) {
			if ((pwrctl[0] >= ISP_DEV_NODE_NUM) ||
				(pwrctl[0] < ISP_CAM_A_IDX)) {
				LOG_NOTICE(
					"module index is invalid module(%d)!", pwrctl[0]);
				Ret = -EFAULT;
				break;
			}

			LOG_NOTICE("isp pwr_ctrl(%d,%d) enableClkCnt(%d)\n",
				pwrctl[0], pwrctl[1], G_u4EnableClockCount[pwrctl[0]]);

			if (pwrctl[1]) {
				/* Enable clock */
				ISP_EnableClock(pwrctl[0], MTRUE);

				spin_lock(&(IspInfo.SpinLockClock));
				if (G_u4EnableClockCount[pwrctl[0]] == 1) {
					spin_unlock(&(IspInfo.SpinLockClock));
					ISP_ConfigDMAControl(pwrctl[0]);
				} else {
					spin_unlock(&(IspInfo.SpinLockClock));
				}
			} else {
				if (pwrctl[0] >= ISP_CAMSV_START_IDX)
					ISP_StopSVHW(pwrctl[0]);
				else if (pwrctl[0] < ISP_CAM_C_IDX)
					ISP_StopHW(pwrctl[0]);

				/* Disable clock.
				 *  1. clkmgr: G_u4EnableClockCount=0, call clk_enable/disable
				 *  2. CCF: call clk_enable/disable every time
				 *     -> when IspInfo.UserCount, disable all ISP clk
				 */
				spin_lock(&(IspInfo.SpinLockClock));
				i = G_u4EnableClockCount[pwrctl[0]];
				spin_unlock(&(IspInfo.SpinLockClock));
				while (i > 0) {
					ISP_EnableClock(pwrctl[0], MFALSE);
					i--;
				}
			}
		} else {
			LOG_NOTICE(
				"ISP_POWER_CTRL copy_from_user failed\n");
			Ret = -EFAULT;
		}
	} break;
	default: {
		LOG_NOTICE("Unknown Cmd(%d)\n", Cmd);
		Ret = -EPERM;
		break;
	}
	}
/*  */
EXIT:
	if (Ret != 0)
		LOG_NOTICE(
			"Fail, Cmd(%d), Pid(%d), (process, pid, tgid)=(%s, %d, %d)\n",
			Cmd, pUserInfo->Pid, current->comm, current->pid,
			current->tgid);
	/*  */
	return Ret;
}

#ifdef CONFIG_COMPAT

/*******************************************************************************
 *
 ******************************************************************************/
static int compat_get_isp_read_register_data(
	struct compat_ISP_REG_IO_STRUCT __user *data32,
	struct ISP_REG_IO_STRUCT __user *data)
{
	compat_uint_t count;
	compat_uptr_t uptr;
	int err = 0;

	err = get_user(uptr, &data32->pData);
	err |= put_user(compat_ptr(uptr), &data->pData);
	err |= get_user(count, &data32->Count);
	err |= put_user(count, &data->Count);
	return err;
}

static int compat_put_isp_read_register_data(
	struct compat_ISP_REG_IO_STRUCT __user *data32,
	struct ISP_REG_IO_STRUCT __user *data)
{
	compat_uint_t count;
	/*      compat_uptr_t uptr; */
	int err = 0;
	/* Assume data pointer is unchanged. */
	/* err = get_user(compat_ptr(uptr),     &data->pData); */
	/* err |= put_user(uptr, &data32->pData); */
	err |= get_user(count, &data->Count);
	err |= put_user(count, &data32->Count);
	return err;
}

static int compat_get_isp_buf_ctrl_struct_data(
	struct compat_ISP_BUFFER_CTRL_STRUCT __user *data32,
	struct ISP_BUFFER_CTRL_STRUCT __user *data)
{
	compat_uint_t tmp, tmp2, tmp3;
	compat_uptr_t uptr;
	int err = 0;

	err = get_user(tmp, &data32->ctrl);
	err |= put_user(tmp, &data->ctrl);
	err |= get_user(tmp2, &data32->module);
	err |= put_user(tmp2, &data->module);
	err |= get_user(tmp3, &data32->buf_id);
	err |= put_user(tmp3, &data->buf_id);
	err |= get_user(uptr, &data32->data_ptr);
	err |= put_user(compat_ptr(uptr), &data->data_ptr);
	err |= get_user(uptr, &data32->ex_data_ptr);
	err |= put_user(compat_ptr(uptr), &data->ex_data_ptr);
	err |= get_user(uptr, &data32->pExtend);
	err |= put_user(compat_ptr(uptr), &data->pExtend);

	return err;
}

static int compat_put_isp_buf_ctrl_struct_data(
	struct compat_ISP_BUFFER_CTRL_STRUCT __user *data32,
	struct ISP_BUFFER_CTRL_STRUCT __user *data)
{
	compat_uint_t tmp, tmp2, tmp3;
	/*      compat_uptr_t uptr; */
	int err = 0;

	err = get_user(tmp, &data->ctrl);
	err |= put_user(tmp, &data32->ctrl);
	err |= get_user(tmp2, &data->module);
	err |= put_user(tmp2, &data32->module);
	err |= get_user(tmp3, &data->buf_id);
	err |= put_user(tmp3, &data32->buf_id);
	/* Assume data pointer is unchanged. */
	/* err |= get_user(compat_ptr(uptr), &data->data_ptr); */
	/* err |= put_user(uptr, &data32->data_ptr); */
	/* err |= get_user(compat_ptr(uptr), &data->ex_data_ptr); */
	/* err |= put_user(uptr, &data32->ex_data_ptr); */
	/* err |= get_user(compat_ptr(uptr), &data->pExtend); */
	/* err |= put_user(uptr, &data32->pExtend); */

	return err;
}

static int compat_get_isp_ref_cnt_ctrl_struct_data(
	struct compat_ISP_REF_CNT_CTRL_STRUCT __user *data32,
	struct ISP_REF_CNT_CTRL_STRUCT __user *data)
{
	compat_uint_t tmp, tmp2;
	compat_uptr_t uptr;
	int err = 0;

	err = get_user(tmp, &data32->ctrl);
	err |= put_user(tmp, &data->ctrl);
	err |= get_user(tmp2, &data32->id);
	err |= put_user(tmp2, &data->id);
	err |= get_user(uptr, &data32->data_ptr);
	err |= put_user(compat_ptr(uptr), &data->data_ptr);

	return err;
}

static int compat_put_isp_ref_cnt_ctrl_struct_data(
	struct compat_ISP_REF_CNT_CTRL_STRUCT __user *data32,
	struct ISP_REF_CNT_CTRL_STRUCT __user *data)
{
	compat_uint_t tmp, tmp2;
	/*      compat_uptr_t uptr; */
	int err = 0;

	err = get_user(tmp, &data->ctrl);
	err |= put_user(tmp, &data32->ctrl);
	err |= get_user(tmp2, &data->id);
	err |= put_user(tmp2, &data32->id);
	/* Assume data pointer is unchanged. */
	/* err |= get_user(compat_ptr(uptr), &data->data_ptr); */
	/* err |= put_user(uptr, &data32->data_ptr); */

	return err;
}

static long ISP_ioctl_compat(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	long ret = 0;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_ISP_READ_REGISTER: {
		struct compat_ISP_REG_IO_STRUCT __user *data32;
		struct ISP_REG_IO_STRUCT __user *data;

		int err = 0;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_isp_read_register_data(data32, data);
		if (err) {
			LOG_INF("compat_get_isp_read_register_data error!!!\n");
			return err;
		}

		ret = filp->f_op->unlocked_ioctl(filp, ISP_READ_REGISTER,
						 (unsigned long)data);

		err = compat_put_isp_read_register_data(data32, data);
		if (err) {
			LOG_INF("compat_put_isp_read_register_data error!!!\n");
			return err;
		}
		return ret;
	}
	case COMPAT_ISP_WRITE_REGISTER: {
		struct compat_ISP_REG_IO_STRUCT __user *data32;
		struct ISP_REG_IO_STRUCT __user *data;

		int err = 0;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_isp_read_register_data(data32, data);
		if (err) {
			LOG_INF("COMPAT_ISP_WRITE_REGISTER error!!!\n");
			return err;
		}
		ret = filp->f_op->unlocked_ioctl(filp, ISP_WRITE_REGISTER,
						 (unsigned long)data);

		return ret;
	}
	case COMPAT_ISP_BUFFER_CTRL: {
		struct compat_ISP_BUFFER_CTRL_STRUCT __user *data32;
		struct ISP_BUFFER_CTRL_STRUCT __user *data;

		int err = 0;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_isp_buf_ctrl_struct_data(data32, data);
		if (err)
			return err;

		if (err) {
			LOG_INF(
				"compat_get_isp_buf_ctrl_struct_data error!!!\n");
			return err;
		}
		ret = filp->f_op->unlocked_ioctl(filp, ISP_BUFFER_CTRL,
						 (unsigned long)data);

		err = compat_put_isp_buf_ctrl_struct_data(data32, data);

		if (err) {
			LOG_INF(
				"compat_put_isp_buf_ctrl_struct_data error!!!\n");
			return err;
		}
		return ret;
	}
	case COMPAT_ISP_REF_CNT_CTRL: {
		struct compat_ISP_REF_CNT_CTRL_STRUCT __user *data32;
		struct ISP_REF_CNT_CTRL_STRUCT __user *data;

		int err = 0;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_isp_ref_cnt_ctrl_struct_data(data32, data);
		if (err) {
			LOG_INF(
				"compat_get_isp_ref_cnt_ctrl_struct_data error!!!\n");
			return err;
		}

		err = compat_put_isp_ref_cnt_ctrl_struct_data(data32, data);
		if (err) {
			LOG_INF(
				"compat_put_isp_ref_cnt_ctrl_struct_data error!!!\n");
			return err;
		}
		return ret;
	}
	case COMPAT_ISP_DEBUG_FLAG: {
		/* compat_ptr(arg) will convert the arg */
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_DEBUG_FLAG, (unsigned long)compat_ptr(arg));

		return ret;
	}
	case COMPAT_ISP_GET_DMA_ERR: {
		/* compat_ptr(arg) will convert the arg */
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_GET_DMA_ERR, (unsigned long)compat_ptr(arg));
		return ret;
	}
	case COMPAT_ISP_WAKELOCK_CTRL: {
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_WAKELOCK_CTRL,
			(unsigned long)compat_ptr(arg));

		return ret;
	}
	case COMPAT_ISP_GET_DROP_FRAME: {
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_GET_DROP_FRAME,
			(unsigned long)compat_ptr(arg));

		return ret;
	}
	case COMPAT_ISP_GET_CUR_SOF: {
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_GET_CUR_SOF, (unsigned long)compat_ptr(arg));

		return ret;
	}
	case COMPAT_ISP_RESET_BY_HWMODULE: {
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_RESET_BY_HWMODULE,
			(unsigned long)compat_ptr(arg));

		return ret;
	}
	case COMPAT_ISP_VF_LOG: {
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_VF_LOG, (unsigned long)compat_ptr(arg));

		return ret;
	}
	case COMPAT_ISP_GET_START_TIME: {
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_GET_START_TIME,
			(unsigned long)compat_ptr(arg));

		return ret;
	}
	case COMPAT_ISP_NOTE_CQTHR0_BASE: {
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_NOTE_CQTHR0_BASE, (unsigned long)compat_ptr(arg));

		return ret;
	}
	case COMPAT_ISP_SET_VIR_CQCNT: {
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_SET_VIR_CQCNT, (unsigned long)compat_ptr(arg));

		return ret;
	}
	case COMPAT_ISP_POWER_CTRL: {
		ret = filp->f_op->unlocked_ioctl(
			filp, ISP_POWER_CTRL, (unsigned long)compat_ptr(arg));
		return ret;
	}
	case ISP_GET_DUMP_INFO:
	case ISP_WAIT_IRQ:
	case ISP_CLEAR_IRQ: /* structure (no pointer) */
	case ISP_REGISTER_IRQ_USER_KEY:
	case ISP_FLUSH_IRQ_REQUEST:
	case ISP_GET_VSYNC_CNT:
	case ISP_RESET_VSYNC_CNT:
	case ISP_ION_IMPORT:
	case ISP_ION_FREE:
	case ISP_ION_FREE_BY_HWMODULE:
	case ISP_CQ_SW_PATCH:
	case ISP_LARB_MMU_CTL:
	case ISP_DFS_CTRL:
	case ISP_DFS_UPDATE:
	case ISP_GET_SUPPORTED_ISP_CLOCKS:
	case ISP_GET_CUR_ISP_CLOCK:
	case ISP_GET_GLOBAL_TIME:
	case ISP_SET_PM_QOS_INFO:
	case ISP_SET_PM_QOS:
	case ISP_GET_INT_ERR:
	case SV_DFS_UPDATE:
	case SV_GET_CUR_ISP_CLOCK:
	case SV_SET_PM_QOS_INFO:
	case SV_SET_PM_QOS:
	case ISP_SET_SEC_DAPC_REG:
	case ISP_NOTE_CQTHR0_BASE:
	case ISP_GET_CUR_HWP1DONE:
		return filp->f_op->unlocked_ioctl(filp, cmd, arg);
	default:
		return -ENOIOCTLCMD;
		/* return ISP_ioctl(filep, cmd, arg); */
	}
}

#endif

/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_open(struct inode *pInode, struct file *pFile)
{
	int Ret = 0;
	unsigned int i, j;
	int q = 0, p = 0;
	struct ISP_USER_INFO_STRUCT *pUserInfo;

	mutex_lock(&open_isp_mutex);
	LOG_DBG("- E. UserCount: %d.\n", IspInfo.UserCount);

	/*  */
	spin_lock(&(IspInfo.SpinLockIspRef));

	pFile->private_data = NULL;

	pFile->private_data =
		kmalloc(sizeof(struct ISP_USER_INFO_STRUCT), GFP_ATOMIC);

	if (pFile->private_data == NULL) {
		LOG_DBG(
			"ERROR: kmalloc failed, (process, pid, tgid)=(%s, %d, %d)\n",
			current->comm, current->pid, current->tgid);

		Ret = -ENOMEM;
	} else {
		pUserInfo = (struct ISP_USER_INFO_STRUCT *)pFile->private_data;
		pUserInfo->Pid = current->pid;
		pUserInfo->Tid = current->tgid;
	}
	/*  */
	if (IspInfo.UserCount > 0) {
		IspInfo.UserCount++;
		spin_unlock(&(IspInfo.SpinLockIspRef));

		LOG_DBG(
			"Curr UserCount(%d), (process, pid, tgid)=(%s, %d, %d), users exist\n",
			IspInfo.UserCount,
			current->comm, current->pid, current->tgid);

		goto EXIT;
	} else {
		IspInfo.UserCount++;
		spin_unlock(&(IspInfo.SpinLockIspRef));

/* kernel log limit to (current+150) lines per second */
#ifndef EP_NO_K_LOG_ADJUST
		pr_detect_count = get_detect_count();
		i = pr_detect_count + 150;
		set_detect_count(i);

		LOG_DBG(
			"Curr UserCount(%d), (process, pid, tgid)=(%s, %d, %d), log_limit_line(%d), first user\n",
			IspInfo.UserCount,
			current->comm, current->pid, current->tgid, i);
#else
		LOG_DBG(
			"Curr UserCount(%d), (process, pid, tgid)=(%s, %d, %d), first user\n",
			IspInfo.UserCount,
			current->comm, current->pid, current->tgid);
#endif
	}

	/* do wait queue head init when re-enter in camera */
	/*  */
	for (i = 0; i < IRQ_USER_NUM_MAX; i++) {
		FirstUnusedIrqUserKey = 1;

		strncpy((void *)IrqUserKey_UserInfo[i].userName,
			"DefaultUserNametoAllocMem", USERKEY_USERNAME_STR_LEN);

		IrqUserKey_UserInfo[i].userKey = -1;
	}

	IspInfo.BufInfo.Read.pData = kmalloc(ISP_BUF_SIZE, GFP_ATOMIC);
	IspInfo.BufInfo.Read.Size = ISP_BUF_SIZE;
	IspInfo.BufInfo.Read.Status = ISP_BUF_STATUS_EMPTY;
	if (IspInfo.BufInfo.Read.pData == NULL) {
		LOG_DBG("ERROR: BufRead kmalloc failed\n");
		Ret = -ENOMEM;
		goto EXIT;
	}

	/*  */
	for (i = 0; i < ISP_IRQ_TYPE_AMOUNT; i++) {
		for (j = 0; j < ISP_IRQ_ST_AMOUNT; j++) {
			for (q = 0; q < IRQ_USER_NUM_MAX; q++) {
				IspInfo.IrqInfo.Status[i][j][q] = 0;
				IspInfo.IrqInfo.MarkedFlag[i][j][q] = 0;
				for (p = 0; p < 32; p++) {
					IspInfo.IrqInfo
						.MarkedTime_sec[i][p][q] = 0;

					IspInfo.IrqInfo
						.MarkedTime_usec[i][p][q] = 0;

					IspInfo.IrqInfo
						.PassedBySigCnt[i][p][q] = 0;

					IspInfo.IrqInfo
						.LastestSigTime_sec[i][p] = 0;

					IspInfo.IrqInfo
						.LastestSigTime_usec[i][p] = 0;
				}
			}
		}
	}
	/* reset backup regs */
	memset(g_BkReg, 0, sizeof(struct _isp_bk_reg_t) * ISP_IRQ_TYPE_AMOUNT);

#ifdef ENABLE_TIMESYNC_HANDLE
	archcounter_timesync_init(MTRUE); /* Global timer enable */
#endif

#ifdef KERNEL_LOG
	IspInfo.DebugMask = (ISP_DBG_INT);
#endif
/*  */
EXIT:
	if (Ret < 0) {
		if (IspInfo.BufInfo.Read.pData != NULL) {
			kfree(IspInfo.BufInfo.Read.pData);
			IspInfo.BufInfo.Read.pData = NULL;
		}
	}

	LOG_INF("- X. Ret: %d. UserCount: %d\n", Ret, IspInfo.UserCount);

	mutex_unlock(&open_isp_mutex);
	return Ret;
}


/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_release(struct inode *pInode, struct file *pFile)
{
	struct ISP_USER_INFO_STRUCT *pUserInfo;
	unsigned int Reg;
	unsigned int i = 0;

	mutex_lock(&open_isp_mutex);
	LOG_DBG("- E. UserCount: %d.\n", IspInfo.UserCount);

	/*  */
	/* LOG_DBG("UserCount(%d)",IspInfo.UserCount); */
	/*  */
	if (pFile->private_data != NULL) {
		pUserInfo = (struct ISP_USER_INFO_STRUCT *)pFile->private_data;
		kfree(pFile->private_data);
		pFile->private_data = NULL;
	}
	/*      */
	spin_lock(&(IspInfo.SpinLockIspRef));
	IspInfo.UserCount--;
	if (IspInfo.UserCount > 0) {
		spin_unlock(&(IspInfo.SpinLockIspRef));

		LOG_DBG("Curr UserCount(%d), (process, pid, tgid)="
			"(%s, %d, %d),	users exist",
			IspInfo.UserCount, current->comm, current->pid,
			current->tgid);

		goto EXIT;
	} else {
		spin_unlock(&(IspInfo.SpinLockIspRef));
	}

/* kernel log limit back to default */
#ifndef EP_NO_K_LOG_ADJUST
	set_detect_count(pr_detect_count);
#endif
	/*      */
	LOG_DBG("Curr UserCount(%d), (proc,pid,tgid)=(%s,%d,%d), log_limit_line(%d), last user\n",
		IspInfo.UserCount, current->comm, current->pid, current->tgid,
		pr_detect_count);

	for (i = ISP_CAM_A_IDX; i < ISP_CAMSV_START_IDX; i++) {
		int clkcnt = 0;
		int j = 0;

		spin_lock(&(IspInfo.SpinLockClock));
		if (G_u4EnableClockCount[i] == 0) {
			spin_unlock(&(IspInfo.SpinLockClock));
			LOG_DBG("G_u4EnableClockCount[%d] already be 0, cannot r/w reg\n", i);
			continue;
		}
		clkcnt = G_u4EnableClockCount[i];
		spin_unlock(&(IspInfo.SpinLockClock));

		/* Close VF when ISP_release */
		/* reason of close vf is to make sure */
		/* camera can serve regular after previous abnormal exit */
		Reg = ISP_RD32(CAM_REG_TG_VF_CON(i));
		Reg &= 0xfffffffE; /* close Vfinder */
		ISP_WR32(CAM_REG_TG_VF_CON(i), Reg);

		/* Set DMX_SEL = 0 when ISP_release */
		/* Reson: If twin is enabled, the twin module's DMX_SEL will
		 *        be set to 1.
		 *        It will encounter err when run single path and other
		 *        module dmx_sel = 1
		 */
		if (!sec_on) {
			Reg = ISP_RD32(CAM_REG_CTL_SEL(i));
			Reg &= 0xfffffff8; /* set dmx to 0 */
			ISP_WR32(CAM_REG_CTL_SEL(i), Reg);
		}

		/* Reset Twin status.
		 *  If previous camera run in twin mode,
		 *  then mediaserver died, no one clear this status.
		 *  Next camera runs in single mode, and it will not update CQ0
		 */
		ISP_WR32(CAM_REG_CTL_TWIN_STATUS(i), 0x0);

		LOG_INF("dev(%d): Disable all clk, cnt(%d)\n", i, clkcnt);
		for (j = 0; j < clkcnt; j++)
			ISP_EnableClock(i, MFALSE);
	}

	for (i = RUNTIME_ISP_CAMSV_START_IDX; i <= ISP_CAMSV_END_IDX; i++) {
		int clkcnt = 0;
		int j = 0;

		spin_lock(&(IspInfo.SpinLockClock));
		if (G_u4EnableClockCount[i] == 0) {
			spin_unlock(&(IspInfo.SpinLockClock));
			LOG_DBG("G_u4EnableClockCount[%d] already be 0, cannot r/w reg\n", i);
			continue;
		}
		clkcnt = G_u4EnableClockCount[i];
		spin_unlock(&(IspInfo.SpinLockClock));

		Reg = ISP_RD32(CAMSV_REG_TG_VF_CON(i));
		Reg &= 0xfffffffE; /* close Vfinder */
		ISP_WR32(CAMSV_REG_TG_VF_CON(i), Reg);

		LOG_INF("dev(%d): Disable all clk, cnt(%d)\n", i, clkcnt);
		for (j = 0; j < clkcnt; j++)
			ISP_EnableClock(i, MFALSE);
	}

	/* why i add this wake_unlock here, */
	/* because the Ap is not expected to be dead. */
	/* The driver must releae the wakelock, */
	/* otherwise the system will not enter */
	/* the power-saving mode */
	if (g_WaitLockCt) {
		LOG_INF("wakelock disable!! cnt(%d)\n", g_WaitLockCt);
#if IS_ENABLED(CONFIG_PM_WAKELOCKS)
		__pm_relax(isp_wake_lock);
#else
		wake_unlock(&isp_wake_lock);
#endif
		g_WaitLockCt = 0;
	}
	/* reset */
	/*      */
	for (i = 0; i < IRQ_USER_NUM_MAX; i++) {
		FirstUnusedIrqUserKey = 1;

		strncpy((void *)IrqUserKey_UserInfo[i].userName,
			"DefaultUserNametoAllocMem", USERKEY_USERNAME_STR_LEN);

		IrqUserKey_UserInfo[i].userKey = -1;
	}
	if (IspInfo.BufInfo.Read.pData != NULL) {
		kfree(IspInfo.BufInfo.Read.pData);
		IspInfo.BufInfo.Read.pData = NULL;
		IspInfo.BufInfo.Read.Size = 0;
		IspInfo.BufInfo.Read.Status = ISP_BUF_STATUS_EMPTY;
	}

	/* reset backup regs */
	memset(g_BkReg, 0, sizeof(struct _isp_bk_reg_t) * ISP_IRQ_TYPE_AMOUNT);

	/* reset secure cam info */
	if (sec_on) {
		memset(&lock_reg, 0, sizeof(struct isp_sec_dapc_reg));
		sec_on = 0;
	}


EXIT:

	LOG_INF("- X. UserCount: %d.", IspInfo.UserCount);
	mutex_unlock(&open_isp_mutex);
	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_mmap(struct file *pFile, struct vm_area_struct *pVma)
{
	unsigned long length = 0;
	unsigned long pfn = 0x0;

	/*LOG_DBG("- E."); */
	length = (pVma->vm_end - pVma->vm_start);
	/*  */
	pVma->vm_page_prot = pgprot_noncached(pVma->vm_page_prot);
	pfn = pVma->vm_pgoff << PAGE_SHIFT;

	/* LOG_INF("ISP_mmap: vm_pgoff(0x%lx),pfn(0x%x),phy(0x%lx),
	 * vm_start(0x%lx),vm_end(0x%lx),length(0x%lx)\n",
	 * pVma->vm_pgoff, pfn, pVma->vm_pgoff << PAGE_SHIFT, pVma->vm_start,
	 * pVma->vm_end, length);
	 */

	switch (pfn) {
	case CAM_A_BASE_HW:
	case CAM_B_BASE_HW:
	case CAM_C_BASE_HW:
	case CAM_A_INNER_BASE_HW:
	case CAM_B_INNER_BASE_HW:
	case CAM_C_INNER_BASE_HW:
		if (length > ISP_REG_RANGE) {
			LOG_NOTICE("range err:mod:0x%x len:0x%x RANGE:0x%x\n",
				  (unsigned int)pfn, (unsigned int)length,
				   (unsigned int)ISP_REG_RANGE);
			return -EAGAIN;
		}
		break;
	case CAMSV_0_BASE_HW:
	case CAMSV_1_BASE_HW:
	case CAMSV_2_BASE_HW:
	case CAMSV_3_BASE_HW:
	case CAMSV_4_BASE_HW:
	case CAMSV_5_BASE_HW:
	case CAMSV_6_BASE_HW:
	case CAMSV_7_BASE_HW:
		if (length > ISPSV_REG_RANGE) {
			LOG_NOTICE("range err:mod:0x%x len:0x%x RANGE:0x%x\n",
				  (unsigned int)pfn, (unsigned int)length,
				   (unsigned int)ISPSV_REG_RANGE);
			return -EAGAIN;
		}
		break;
	default:
		LOG_NOTICE("Illegal starting HW addr for mmap!\n");
		return -EAGAIN;
	}
	if (remap_pfn_range(pVma, pVma->vm_start, pVma->vm_pgoff,
			    pVma->vm_end - pVma->vm_start, pVma->vm_page_prot))
		return -EAGAIN;

	/*  */
	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/

static dev_t IspDevNo;
static struct cdev *pIspCharDrv;
static struct class *pIspClass;

static const struct file_operations IspFileOper = {
	.owner = THIS_MODULE,
	.open = ISP_open,
	.release = ISP_release,
	/* .flush       = mt_isp_flush, */
	.mmap = ISP_mmap,
	.unlocked_ioctl = ISP_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ISP_ioctl_compat,
#endif
};

/*******************************************************************************
 *
 ******************************************************************************/
static inline void ISP_UnregCharDev(void)
{
	LOG_DBG("- E.");
	/*      */
	/* Release char driver */
	if (pIspCharDrv != NULL) {
		cdev_del(pIspCharDrv);
		pIspCharDrv = NULL;
	}
	/*      */
	unregister_chrdev_region(IspDevNo, 1);
}

/*******************************************************************************
 *
 ******************************************************************************/
static inline int ISP_RegCharDev(void)
{
	int Ret = 0;

	/*  */
	LOG_DBG("- E.\n");
	/*  */
	Ret = alloc_chrdev_region(&IspDevNo, 0, 1, ISP_DEV_NAME);
	if ((Ret) < 0) {
		LOG_NOTICE("alloc_chrdev_region failed, %d\n", Ret);
		return Ret;
	}
	/* Allocate driver */
	pIspCharDrv = cdev_alloc();
	if (pIspCharDrv == NULL) {
		LOG_NOTICE("cdev_alloc failed\n");
		Ret = -ENOMEM;
		goto EXIT;
	}
	/* Attatch file operation. */
	cdev_init(pIspCharDrv, &IspFileOper);
	/*  */
	pIspCharDrv->owner = THIS_MODULE;
	/* Add to system */
	Ret = cdev_add(pIspCharDrv, IspDevNo, 1);
	if ((Ret) < 0) {
		LOG_NOTICE("Attatch file operation failed, %d\n", Ret);
		goto EXIT;
	}
/*  */
EXIT:
	if (Ret < 0)
		ISP_UnregCharDev();

	LOG_DBG("- X.\n");
	return Ret;
}

/*******************************************************************************
 *
 ******************************************************************************/
static unsigned int NodeName_to_DevIdx(const char *name)
{
	/* Note:
	 * The following compared string should be the same as the node name
	 * defined in dts.
	 */
	if (strncmp(name, "camisp_legacy", strlen("camisp_legacy")) == 0)
		return ISP_CAMSYS_CONFIG_IDX;
	else if (strncmp(name, "camsys_rawa_legacy", strlen("camsys_rawa_legacy")) == 0)
		return ISP_CAMSYS_RAWA_CONFIG_IDX;
	else if (strncmp(name, "camsys_rawb_legacy", strlen("camsys_rawb_legacy")) == 0)
		return ISP_CAMSYS_RAWB_CONFIG_IDX;
	else if (strncmp(name, "camsys_rawc_legacy", strlen("camsys_rawc_legacy")) == 0)
		return ISP_CAMSYS_RAWC_CONFIG_IDX;
	else if (strncmp(name, "cam1_inner_legacy", strlen("cam1_inner_legacy")) == 0)
		return ISP_CAM_A_INNER_IDX;
	else if (strncmp(name, "cam2_inner_legacy", strlen("cam2_inner_legacy")) == 0)
		return ISP_CAM_B_INNER_IDX;
	else if (strncmp(name, "cam3_inner_legacy", strlen("cam3_inner_legacy")) == 0)
		return ISP_CAM_C_INNER_IDX;
	else if (strncmp(name, "cam1_legacy", strlen("cam1_legacy")) == 0)
		return ISP_CAM_A_IDX;
	else if (strncmp(name, "cam2_legacy", strlen("cam2_legacy")) == 0)
		return ISP_CAM_B_IDX;
	else if (strncmp(name, "cam3_legacy", strlen("cam3_legacy")) == 0)
		return ISP_CAM_C_IDX;
	else if (strncmp(name, "camsv1_legacy", strlen("camsv1_legacy")) == 0)
		return ISP_CAMSV0_IDX;
	else if (strncmp(name, "camsv2_legacy", strlen("camsv2_legacy")) == 0)
		return ISP_CAMSV1_IDX;
	else if (strncmp(name, "camsv3_legacy", strlen("camsv3_legacy")) == 0)
		return ISP_CAMSV2_IDX;
	else if (strncmp(name, "camsv4_legacy", strlen("camsv4_legacy")) == 0)
		return ISP_CAMSV3_IDX;
	else if (strncmp(name, "camsv5_legacy", strlen("camsv5_legacy")) == 0)
		return ISP_CAMSV4_IDX;
	else if (strncmp(name, "camsv6_legacy", strlen("camsv6_legacy")) == 0)
		return ISP_CAMSV5_IDX;
	else if (strncmp(name, "camsv7_legacy", strlen("camsv7_legacy")) == 0)
		return ISP_CAMSV6_IDX;
	else if (strncmp(name, "camsv8_legacy", strlen("camsv8_legacy")) == 0)
		return ISP_CAMSV7_IDX;
	else
		return ISP_DEV_NODE_NUM;

}
#ifdef USE_MTK_SMI_LARB_API
/*******************************************************************************
 *
 ******************************************************************************/
/*
 * Before common kernel iommu's device link ready.
 * we need to use SMI API to power on bus directly.
 */
void ISP_get_larb(struct platform_device *pDev, unsigned int dev_idx, int larb_num)
{
	struct device_node *node;
	struct platform_device *larb_pdev;
	unsigned int larb_id = 0;
	int i = 0;

	if ((larb_num > 2) || (larb_num < 1)) {
		LOG_NOTICE("larb num is invalid. Not to get larb.\n");
		return;
	}
	for (i = 0; i < larb_num; i++) {
		if (larb_num > 1)
			node = of_parse_phandle(pDev->dev.of_node, "mediatek,larbs", i);
		else
			node = of_parse_phandle(pDev->dev.of_node, "mediatek,larb", i);
		if (!node) {
			LOG_NOTICE("%s: no mediatek,larb found\n",
				pDev->dev.of_node->name);
			return;
		}
		larb_pdev = of_find_device_by_node(node);
		if (of_property_read_u32(node, "mediatek,larb-id", &larb_id))
			LOG_NOTICE("Error: get larb id from DTS fail!!\n");
		else
			LOG_NOTICE("%s gets larb_id=%d\n",
				pDev->dev.of_node->name, larb_id);
		of_node_put(node);
		if (!larb_pdev) {
			LOG_NOTICE("%s: no mediatek,larb device found\n",
				pDev->dev.of_node->name);
			return;
		}
		if (i == 0)
			isp_devs[dev_idx].larb = &larb_pdev->dev;
		else
			isp_devs[dev_idx].larb_2nd = &larb_pdev->dev;
		LOG_NOTICE("%s: get %s\n", pDev->dev.of_node->name,
			larb_pdev->dev.of_node->name);
	}
}
#endif
/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_probe(struct platform_device *pDev)
{
	int Ret = 0;
	/*    struct resource *pRes = NULL; */
	int i = 0, j = 0;
	unsigned char n;
	unsigned int irq_info[3]; /* Record interrupts info from device tree */
	unsigned int dev_idx = (unsigned int)ISP_DEV_NODE_NUM;

#ifdef CONFIG_OF
	struct device *dev = NULL;
	int camsv_shift = 0;
#endif

	LOG_INF("- E. ISP driver probe.\n");

/* Get platform_device parameters */
#ifdef CONFIG_OF
	if (pDev == NULL) {
		LOG_NOTICE("Error: pDev is NULL\n");
		return -ENXIO;
	}

	atomic_inc(&G_u4DevNodeCt);

	dev_idx = NodeName_to_DevIdx(pDev->dev.of_node->name);
	if ((dev_idx >= ISP_DEV_NODE_NUM) || (dev_idx < ISP_CAMSYS_CONFIG_IDX)) {
		LOG_NOTICE(
			"Error: device idx is wrong. G_u4DevNodeCt=%d, devnode(%s).\n",
			atomic_read(&G_u4DevNodeCt), pDev->dev.of_node->name);
		return -ENODEV;
	}
	LOG_INF("device idx= %d.\n", dev_idx);

	/* Assign the device into isp_devs[] by the order of dev_idx (defined
	 * in ISP_DEV_NODE_ENUM).
	 */
	isp_devs[dev_idx].dev = &pDev->dev;

	/* iomap registers */
	isp_devs[dev_idx].regs = of_iomap(pDev->dev.of_node, 0);

	if (!(isp_devs[dev_idx].regs)) {
		LOG_NOTICE(
			"Error: Unable to ioremap registers, of_iomap fail, G_u4DevNodeCt=%d, devnode(%s).\n",
			atomic_read(&G_u4DevNodeCt), pDev->dev.of_node->name);

		return -ENOMEM;
	}

#ifdef CONFIG_MTK_IOMMU_PGTABLE_EXT
#if (CONFIG_MTK_IOMMU_PGTABLE_EXT > 32)
	*(isp_devs[dev_idx].dev->dma_mask) = (u64)DMA_BIT_MASK(34);
	isp_devs[dev_idx].dev->coherent_dma_mask = (u64)DMA_BIT_MASK(34);
#endif
#endif
	LOG_INF("G_u4DevNodeCt=%d, devnode(%s), map_addr=0x%lx\n",
		atomic_read(&G_u4DevNodeCt),
		pDev->dev.of_node->name,
		(unsigned long)isp_devs[dev_idx].regs);
#ifndef EP_NO_CLKMGR /* MTCMOS */
	if ((strncmp(pDev->dev.of_node->name, "cam1_legacy", strlen("cam1_legacy")) == 0) ||
		(strncmp(pDev->dev.of_node->name, "cam2_legacy", strlen("cam2_legacy")) == 0) ||
		(strncmp(pDev->dev.of_node->name, "cam3_legacy", strlen("cam3_legacy")) == 0) ||
		(strncmp(pDev->dev.of_node->name, "camisp_legacy", strlen("camisp_legacy")) == 0))
		pm_runtime_enable(&pDev->dev);
#endif
	/* get IRQ ID and request IRQ */
	isp_devs[dev_idx].irq = irq_of_parse_and_map(pDev->dev.of_node, 0);

	if (isp_devs[dev_idx].irq > 0) {
		/* Get IRQ Flag from device node */
		if (of_property_read_u32_array(pDev->dev.of_node, "interrupts",
					       irq_info,
					       ARRAY_SIZE(irq_info))) {

			LOG_NOTICE("Error: get irq flags from DTS fail!!\n");
			return -ENODEV;
		}

		for (i = 0; i < ISP_IRQ_TYPE_AMOUNT; i++) {
			if ((strcmp(pDev->dev.of_node->name,
				    IRQ_CB_TBL[i].device_name) == 0) &&
			    (IRQ_CB_TBL[i].isr_fp != NULL)) {

				if(RUNTIME_ISP_CAMSV_START_ST == 0 && i >= ISP_IRQ_TYPE_INT_CAMSV_START_ST){
					RUNTIME_ISP_CAMSV_START_ST = i;
					camsv_shift = RUNTIME_ISP_CAMSV_START_ST - ISP_IRQ_TYPE_INT_CAMSV_START_ST;
					RUNTIME_ISP_CAMSV_START_IDX = ISP_CAMSV_START_IDX + camsv_shift;

					LOG_INF("RUNTIME_ISP_CAMSV_START_ST=%d, RUNTIME_ISP_CAMSV_START_IDX=%d, dev_name=%s\n",
						RUNTIME_ISP_CAMSV_START_ST, RUNTIME_ISP_CAMSV_START_IDX, IRQ_CB_TBL[i].device_name);

				}

				Ret = request_irq(
					isp_devs[dev_idx].irq,
					(irq_handler_t)IRQ_CB_TBL[i].isr_fp,
					irq_info[2],
					(const char *)IRQ_CB_TBL[i].device_name,
					NULL);

				if (Ret) {
					LOG_NOTICE(
					"Error: request_irq fail, G_u4DevNodeCt=%d, devnode(%s), irq=%d, ISR: %s\n",
					atomic_read(&G_u4DevNodeCt),
					pDev->dev.of_node->name,
					isp_devs[dev_idx].irq,
					IRQ_CB_TBL[i].device_name);

					return Ret;
				}

				LOG_INF(
				"G_u4DevNodeCt=%d, devnode(%s), irq=%d, ISR: %s\n",
					atomic_read(&G_u4DevNodeCt), pDev->dev.of_node->name,
					isp_devs[dev_idx].irq,
					IRQ_CB_TBL[i].device_name);

				break;
			}
		}

		if (i >= ISP_IRQ_TYPE_AMOUNT)
			LOG_INF("No ISR: G_u4DevNodeCt=%d, node(%s), irq=%d\n",
				atomic_read(&G_u4DevNodeCt), pDev->dev.of_node->name,
				isp_devs[dev_idx].irq);

	} else {
		LOG_INF("No IRQ!!: G_u4DevNodeCt=%d, devnode(%s), irq=%d\n",
			atomic_read(&G_u4DevNodeCt),
			pDev->dev.of_node->name,
			isp_devs[dev_idx].irq);
	}

#ifdef USE_MTK_SMI_LARB_API
	if ((strncmp(pDev->dev.of_node->name, "cam1_legacy", strlen("cam1_legacy")) == 0) ||
		(strncmp(pDev->dev.of_node->name, "cam2_legacy", strlen("cam2_legacy")) == 0) ||
		(strncmp(pDev->dev.of_node->name, "cam3_legacy", strlen("cam3_legacy")) == 0))
		ISP_get_larb(pDev, dev_idx, 1);
	else if (strncmp(pDev->dev.of_node->name, "camisp_legacy",
		strlen("camisp_legacy")) == 0)
		ISP_get_larb(pDev, dev_idx, 2);
#endif

	/* Only register char driver in the 1st time */
	if (atomic_read(&G_u4DevNodeCt) == 1) {
		/* Register char driver */
		Ret = ISP_RegCharDev();
		if ((Ret)) {
			LOG_NOTICE("Error: register char failed");
			return Ret;
		}

		/* Create class register */
		pIspClass = class_create(THIS_MODULE, "ispdrv");
		if (IS_ERR(pIspClass)) {
			Ret = PTR_ERR(pIspClass);
			LOG_NOTICE("Unable to create class, err = %d\n", Ret);
			goto EXIT;
		}
		dev = device_create(pIspClass, NULL, IspDevNo, NULL,
				    ISP_DEV_NAME);

		if (IS_ERR(dev)) {
			Ret = PTR_ERR(dev);
			LOG_NOTICE(
				"Error: Failed to create device: /dev/%s, err = %d",
				ISP_DEV_NAME, Ret);

			goto EXIT;
		}
#endif

		/* Init spinlocks */
		spin_lock_init(&(IspInfo.SpinLockIspRef));
		spin_lock_init(&(IspInfo.SpinLockIsp));
		for (n = 0; n < ISP_IRQ_TYPE_AMOUNT; n++) {
			spin_lock_init(&(IspInfo.SpinLockIrq[n]));
			spin_lock_init(&(IspInfo.SpinLockIrqCnt[n]));
		}
		spin_lock_init(&(IspInfo.SpinLockRTBC));
		spin_lock_init(&(IspInfo.SpinLockClock));

		spin_lock_init(&(SpinLock_UserKey));

#ifndef EP_NO_CLKMGR /* CCF */

		isp_clk.CAMSYS_LARB13_CGPDN =
			devm_clk_get(&pDev->dev, "CAMSYS_LARB13_CGPDN");

		isp_clk.CAMSYS_LARB14_CGPDN =
			devm_clk_get(&pDev->dev, "CAMSYS_LARB14_CGPDN");

		if (IS_MT6893(g_platform_id)) {
			isp_clk.CAMSYS_LARB15_CGPDN =
				devm_clk_get(&pDev->dev, "CAMSYS_LARB15_CGPDN");
		}

		isp_clk.CAMSYS_SENINF_CGPDN =
			devm_clk_get(&pDev->dev, "CAMSYS_SENINF_CGPDN");

		if (IS_MT6873(g_platform_id) || IS_MT6853(g_platform_id)) {
			isp_clk.CAMSYS_CAM2MM_GALS_CGPDN =
				devm_clk_get(&pDev->dev,
					"CAMSYS_MAIN_CAM2MM_GALS_CGPDN");
		}

		isp_clk.CAMSYS_TOP_MUX_CCU =
			devm_clk_get(&pDev->dev, "TOPCKGEN_TOP_MUX_CCU");

		isp_clk.CAMSYS_CCU0_CGPDN =
			devm_clk_get(&pDev->dev, "CAMSYS_CCU0_CGPDN");

		isp_clk.ISP_CAM_CAMSYS =
			devm_clk_get(&pDev->dev, "CAMSYS_CAM_CGPDN");

		isp_clk.ISP_CAM_CAMTG =
			devm_clk_get(&pDev->dev, "CAMSYS_CAMTG_CGPDN");

		isp_clk.ISP_CAM_CAMSV0 =
			devm_clk_get(&pDev->dev, "CAMSYS_CAMSV0_CGPDN");

		isp_clk.ISP_CAM_CAMSV1 =
			devm_clk_get(&pDev->dev, "CAMSYS_CAMSV1_CGPDN");

		isp_clk.ISP_CAM_CAMSV2 =
			devm_clk_get(&pDev->dev, "CAMSYS_CAMSV2_CGPDN");

		if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
			isp_clk.ISP_CAM_CAMSV3 =
				devm_clk_get(&pDev->dev, "CAMSYS_CAMSV3_CGPDN");
		}

		isp_clk.ISP_CAM_LARB16_RAWA =
			devm_clk_get(&pDev->dev, "CAMSYS_RAWALARB16_CGPDN");
		isp_clk.ISP_CAM_SUBSYS_RAWA =
			devm_clk_get(&pDev->dev, "CAMSYS_RAWACAM_CGPDN");
		isp_clk.ISP_CAM_TG_RAWA =
			devm_clk_get(&pDev->dev, "CAMSYS_RAWATG_CGPDN");
		isp_clk.ISP_CAM_LARB17_RAWB =
			devm_clk_get(&pDev->dev, "CAMSYS_RAWBLARB17_CGPDN");
		isp_clk.ISP_CAM_SUBSYS_RAWB =
			devm_clk_get(&pDev->dev, "CAMSYS_RAWBCAM_CGPDN");
		isp_clk.ISP_CAM_TG_RAWB =
			devm_clk_get(&pDev->dev, "CAMSYS_RAWBTG_CGPDN");

		if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
			isp_clk.ISP_CAM_LARB18_RAWC =
				devm_clk_get(&pDev->dev, "CAMSYS_RAWCLARB18_CGPDN");
			isp_clk.ISP_CAM_SUBSYS_RAWC =
				devm_clk_get(&pDev->dev, "CAMSYS_RAWCCAM_CGPDN");
			isp_clk.ISP_CAM_TG_RAWC =
				devm_clk_get(&pDev->dev, "CAMSYS_RAWCTG_CGPDN");
		}

		isp_clk.ISP_TOP_MUX_CAMTM =
			devm_clk_get(&pDev->dev, "TOPCKGEN_TOP_MUX_CAMTM");

		if (IS_ERR(isp_clk.CAMSYS_LARB13_CGPDN)) {
			LOG_NOTICE("cannot get CAMSYS_LARB13_CGPDN clock\n");
			return PTR_ERR(isp_clk.CAMSYS_LARB13_CGPDN);
		}
		if (IS_ERR(isp_clk.CAMSYS_LARB14_CGPDN)) {
			LOG_NOTICE("cannot get CAMSYS_LARB14_CGPDN clock\n");
			return PTR_ERR(isp_clk.CAMSYS_LARB14_CGPDN);
		}

		if (IS_MT6893(g_platform_id)) {
			if (IS_ERR(isp_clk.CAMSYS_LARB15_CGPDN)) {
				LOG_NOTICE("cannot get CAMSYS_LARB15_CGPDN clock\n");
				return PTR_ERR(isp_clk.CAMSYS_LARB15_CGPDN);
			}
		}

		if (IS_ERR(isp_clk.CAMSYS_SENINF_CGPDN)) {
			LOG_NOTICE("cannot get CAMSYS_SENINF_CGPDN clock\n");
			return PTR_ERR(isp_clk.CAMSYS_SENINF_CGPDN);
		}

		if (IS_MT6873(g_platform_id) || IS_MT6853(g_platform_id)) {
			if (IS_ERR(isp_clk.CAMSYS_CAM2MM_GALS_CGPDN)) {
				LOG_NOTICE(
					"cannot get CAMSYS_CAM2MM_GALS_CGPDN clock\n");
				return PTR_ERR(isp_clk.CAMSYS_CAM2MM_GALS_CGPDN);
			}
		}

		if (IS_ERR(isp_clk.CAMSYS_TOP_MUX_CCU)) {
			LOG_NOTICE("cannot get CAMSYS_TOP_MUX_CCU clock\n");
			return PTR_ERR(isp_clk.CAMSYS_TOP_MUX_CCU);
		}
		if (IS_ERR(isp_clk.CAMSYS_CCU0_CGPDN)) {
			LOG_NOTICE("cannot get CAMSYS_CCU0_CGPDN clock\n");
			return PTR_ERR(isp_clk.CAMSYS_CCU0_CGPDN);
		}
		if (IS_ERR(isp_clk.ISP_CAM_CAMSYS)) {
			LOG_NOTICE("cannot get ISP_CAM_CAMSYS clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_CAMSYS);
		}
		if (IS_ERR(isp_clk.ISP_CAM_CAMTG)) {
			LOG_NOTICE("cannot get ISP_CAM_CAMTG clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_CAMTG);
		}
		if (IS_ERR(isp_clk.ISP_CAM_CAMSV0)) {
			LOG_NOTICE("cannot get ISP_CAM_CAMSV0 clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_CAMSV0);
		}
		if (IS_ERR(isp_clk.ISP_CAM_CAMSV1)) {
			LOG_NOTICE("cannot get ISP_CAM_CAMSV1 clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_CAMSV1);
		}
		if (IS_ERR(isp_clk.ISP_CAM_CAMSV2)) {
			LOG_NOTICE("cannot get ISP_CAM_CAMSV2 clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_CAMSV2);
		}

		if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
			if (IS_ERR(isp_clk.ISP_CAM_CAMSV3)) {
				LOG_NOTICE("cannot get ISP_CAM_CAMSV3 clock\n");
				return PTR_ERR(isp_clk.ISP_CAM_CAMSV3);
			}
		}

		if (IS_ERR(isp_clk.ISP_CAM_LARB16_RAWA)) {
			LOG_NOTICE("cannot get ISP_CAM_LARB16_RAWA clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_LARB16_RAWA);
		}
		if (IS_ERR(isp_clk.ISP_CAM_SUBSYS_RAWA)) {
			LOG_NOTICE("cannot get ISP_CAM_SUBSYS_RAWA clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_SUBSYS_RAWA);
		}
		if (IS_ERR(isp_clk.ISP_CAM_TG_RAWA)) {
			LOG_NOTICE("cannot get ISP_CAM_TG_RAWA clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_TG_RAWA);
		}
		if (IS_ERR(isp_clk.ISP_CAM_LARB17_RAWB)) {
			LOG_NOTICE("cannot get ISP_CAM_LARB17_RAWB clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_LARB17_RAWB);
		}
		if (IS_ERR(isp_clk.ISP_CAM_SUBSYS_RAWB)) {
			LOG_NOTICE("cannot get ISP_CAM_SUBSYS_RAWB clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_SUBSYS_RAWB);
		}
		if (IS_ERR(isp_clk.ISP_CAM_TG_RAWB)) {
			LOG_NOTICE("cannot get ISP_CAM_TG_RAWB clock\n");
			return PTR_ERR(isp_clk.ISP_CAM_TG_RAWB);
		}

		if (IS_MT6873(g_platform_id) || IS_MT6893(g_platform_id)) {
			if (IS_ERR(isp_clk.ISP_CAM_LARB18_RAWC)) {
				LOG_NOTICE("cannot get ISP_CAM_LARB18_RAWC clock\n");
				return PTR_ERR(isp_clk.ISP_CAM_LARB18_RAWC);
			}
			if (IS_ERR(isp_clk.ISP_CAM_SUBSYS_RAWC)) {
				LOG_NOTICE("cannot get ISP_CAM_SUBSYS_RAWC clock\n");
				return PTR_ERR(isp_clk.ISP_CAM_SUBSYS_RAWC);
			}
			if (IS_ERR(isp_clk.ISP_CAM_TG_RAWC)) {
				LOG_NOTICE("cannot get ISP_CAM_TG_RAWC clock\n");
				return PTR_ERR(isp_clk.ISP_CAM_TG_RAWC);
			}
		}

		if (IS_ERR(isp_clk.ISP_TOP_MUX_CAMTM)) {
			LOG_NOTICE("cannot get ISP_TOP_MUX_CAMTM clock\n");
			return PTR_ERR(isp_clk.ISP_TOP_MUX_CAMTM);
		}
#endif
		/*  */
		for (i = 0; i < ISP_IRQ_TYPE_AMOUNT; i++)
			init_waitqueue_head(&IspInfo.WaitQueueHead[i]);
		for (i = 0; i < CAM_AMOUNT; i++) {
			for (j = 0; j < ISP_WAITQ_HEAD_IRQ_AMOUNT; j++) {
				init_waitqueue_head(
					&IspInfo.WaitQHeadCam[i][j]);
			}
		}
		for (i = 0; i < CAMSV_AMOUNT; i++) {
			for (j = 0; j < ISP_WAITQ_HEAD_IRQ_SV_AMOUNT; j++) {
				init_waitqueue_head(
					&IspInfo.WaitQHeadCamsv[i][j]);
			}
		}

#if IS_ENABLED(CONFIG_PM_WAKELOCKS)
	isp_wake_lock = wakeup_source_register(&pDev->dev, "isp_lock_wakelock");
#else
	wake_lock_init(&isp_wake_lock, WAKE_LOCK_SUSPEND, "isp_lock_wakelock");
#endif

#if (ISP_BOTTOMHALF_WORKQ == 1)
		for (i = 0; i < ISP_IRQ_TYPE_AMOUNT; i++) {
			isp_workque[i].module = i;
			memset((void *)&(isp_workque[i].isp_bh_work), 0,
			       sizeof(isp_workque[i].isp_bh_work));
			INIT_WORK(&(isp_workque[i].isp_bh_work),
				  ISP_BH_Workqueue);
		}

		isr_log_wq = alloc_ordered_workqueue("ISP_BH_Workqueue",
			WQ_UNBOUND | __WQ_ORDERED | __WQ_ORDERED_EXPLICIT);
		if (!isr_log_wq)
			LOG_NOTICE("Error: allocate workqueue failed!\n");
#endif

		/* Init IspInfo */
		spin_lock(&(IspInfo.SpinLockIspRef));
		IspInfo.UserCount = 0;
		spin_unlock(&(IspInfo.SpinLockIspRef));
		/*  */
		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAM_A_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM;

		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAM_B_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM;

		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAM_C_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM;

		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAMSV_0_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV;

		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAMSV_1_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV;
		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAMSV_2_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV;

		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAMSV_3_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV;

		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAMSV_4_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV;

		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAMSV_5_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV;

		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAMSV_6_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV;
		IspInfo.IrqInfo.Mask[ISP_IRQ_TYPE_INT_CAMSV_7_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV;

		/* only cam have warning mask */
		IspInfo.IrqInfo
			.WarnMask[ISP_IRQ_TYPE_INT_CAM_A_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM_WARN;

		IspInfo.IrqInfo
			.WarnMask[ISP_IRQ_TYPE_INT_CAM_B_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM_WARN;

		IspInfo.IrqInfo
			.WarnMask[ISP_IRQ_TYPE_INT_CAM_C_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM_WARN;

		IspInfo.IrqInfo
			.Warn2Mask[ISP_IRQ_TYPE_INT_CAM_A_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM_WARN_2;
		IspInfo.IrqInfo
			.Warn2Mask[ISP_IRQ_TYPE_INT_CAM_B_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM_WARN_2;

		IspInfo.IrqInfo
			.Warn2Mask[ISP_IRQ_TYPE_INT_CAM_C_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM_WARN_2;
		/*  */
		IspInfo.IrqInfo.ErrMask[ISP_IRQ_TYPE_INT_CAM_A_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM_ERR;

		IspInfo.IrqInfo.ErrMask[ISP_IRQ_TYPE_INT_CAM_B_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM_ERR;

		IspInfo.IrqInfo.ErrMask[ISP_IRQ_TYPE_INT_CAM_C_ST][SIGNAL_INT] =
			INT_ST_MASK_CAM_ERR;

		IspInfo.IrqInfo
			.ErrMask[ISP_IRQ_TYPE_INT_CAMSV_0_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV_ERR;

		IspInfo.IrqInfo
			.ErrMask[ISP_IRQ_TYPE_INT_CAMSV_1_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV_ERR;

		IspInfo.IrqInfo
			.ErrMask[ISP_IRQ_TYPE_INT_CAMSV_2_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV_ERR;

		IspInfo.IrqInfo
			.ErrMask[ISP_IRQ_TYPE_INT_CAMSV_3_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV_ERR;

		IspInfo.IrqInfo
			.ErrMask[ISP_IRQ_TYPE_INT_CAMSV_4_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV_ERR;

		IspInfo.IrqInfo
			.ErrMask[ISP_IRQ_TYPE_INT_CAMSV_5_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV_ERR;

		IspInfo.IrqInfo
			.ErrMask[ISP_IRQ_TYPE_INT_CAMSV_6_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV_ERR;

		IspInfo.IrqInfo
			.ErrMask[ISP_IRQ_TYPE_INT_CAMSV_7_ST][SIGNAL_INT] =
			INT_ST_MASK_CAMSV_ERR;

		/* Init IrqCntInfo */
		for (i = 0; i < ISP_IRQ_TYPE_AMOUNT; i++) {
			for (j = 0; j < ISP_ISR_MAX_NUM; j++) {
				IspInfo.IrqCntInfo.m_err_int_cnt[i][j] = 0;
				IspInfo.IrqCntInfo.m_warn_int_cnt[i][j] = 0;
			}
			IspInfo.IrqCntInfo.m_err_int_mark[i] = 0;
			IspInfo.IrqCntInfo.m_warn_int_mark[i] = 0;

			IspInfo.IrqCntInfo.m_int_usec[i] = 0;
		}

EXIT:
		if (Ret < 0)
			ISP_UnregCharDev();
	}

	LOG_INF("- X. ISP driver probe.\n");

	return Ret;
}

/*******************************************************************************
 * Called when the device is being detached from the driver
 ******************************************************************************/
static int ISP_remove(struct platform_device *pDev)
{
	/*    struct resource *pRes; */
	int IrqNum;
	/*  */
	LOG_DBG("- E.");
#ifndef EP_NO_CLKMGR /* CCF */
	if ((strncmp(pDev->dev.of_node->name, "cam1_legacy", strlen("cam1_legacy")) == 0) ||
		(strncmp(pDev->dev.of_node->name, "cam2_legacy", strlen("cam2_legacy")) == 0) ||
		(strncmp(pDev->dev.of_node->name, "cam3_legacy", strlen("cam3_legacy")) == 0) ||
		(strncmp(pDev->dev.of_node->name, "camisp_legacy", strlen("camisp_legacy")) == 0))
		pm_runtime_disable(&pDev->dev);
#endif
	/* unregister char driver. */
	ISP_UnregCharDev();

	/* Release IRQ */
	disable_irq(IspInfo.IrqNum);
	IrqNum = platform_get_irq(pDev, 0);
	free_irq(IrqNum, NULL);

	/*  */
	device_destroy(pIspClass, IspDevNo);
	/*  */
	class_destroy(pIspClass);
	pIspClass = NULL;

#if (ISP_BOTTOMHALF_WORKQ == 1)
	destroy_workqueue(isr_log_wq);
#endif
	return 0;
}

static int ISP_suspend(struct platform_device *pDev, pm_message_t Mesg)
{
	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_resume(struct platform_device *pDev)
{
	return 0;
}

/*---------------------------------------------------------------------------*/
#ifdef CONFIG_PM
/*---------------------------------------------------------------------------*/
int ISP_pm_suspend(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	WARN_ON(pdev == NULL);

	/*pr_debug("calling %s()\n", __func__); */

	return ISP_suspend(pdev, PMSG_SUSPEND);
}

int ISP_pm_resume(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);

	WARN_ON(pdev == NULL);

	/*pr_debug("calling %s()\n", __func__); */

	return ISP_resume(pdev);
}

/* extern void mt_irq_set_sens(unsigned int irq, unsigned int sens); */
/* extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity); */
int ISP_pm_restore_noirq(struct device *device)
{
/*pr_debug("calling %s()\n", __func__); */
#ifndef CONFIG_OF
	mt_irq_set_sens(CAM0_IRQ_BIT_ID, MT_LEVEL_SENSITIVE);
	mt_irq_set_polarity(CAM0_IRQ_BIT_ID, MT_POLARITY_LOW);
#endif
	return 0;
}

/*---------------------------------------------------------------------------*/
#else /*CONFIG_PM */
/*---------------------------------------------------------------------------*/
#define ISP_pm_suspend NULL
#define ISP_pm_resume NULL
#define ISP_pm_restore_noirq NULL
/*---------------------------------------------------------------------------*/
#endif /*CONFIG_PM */
/*---------------------------------------------------------------------------*/

const struct dev_pm_ops ISP_pm_ops = {
	.suspend = ISP_pm_suspend,
	.resume = ISP_pm_resume,
	.freeze = ISP_pm_suspend,
	.thaw = ISP_pm_resume,
	.poweroff = ISP_pm_suspend,
	.restore = ISP_pm_resume,
	.restore_noirq = ISP_pm_restore_noirq,
};

/*******************************************************************************
 *
 ******************************************************************************/
static struct platform_driver IspDriver = {.probe = ISP_probe,
					   .remove = ISP_remove,
					   .suspend = ISP_suspend,
					   .resume = ISP_resume,
					   .driver = {
						   .name = ISP_DEV_NAME,
						   .owner = THIS_MODULE,
#ifdef CONFIG_OF
						   .of_match_table = isp_of_ids,
#endif
#ifdef CONFIG_PM
						   .pm = &ISP_pm_ops,
#endif
					   } };

#if IS_ENABLED(CONFIG_PM)
/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_pm_event_suspend(void)
{
	unsigned int regVal;
	enum ISP_IRQ_TYPE_ENUM IrqType = ISP_IRQ_TYPE_AMOUNT;
	int ret = 0;
	unsigned int i = 0;
	unsigned int regTGSt, loopCnt;
	struct ISP_WAIT_IRQ_STRUCT waitirq;
	ktime_t time;
	unsigned long long sec = 0, m_sec = 0;
	unsigned long long timeoutMs = 500000000; /*500ms */


	for (i = ISP_CAM_A_IDX; i < ISP_DEV_NODE_NUM; i++) {
		if (i == ISP_CAM_C_IDX) {
			if (IS_MT6853(g_platform_id))
				continue;
		}

		/* Check clock counter instead of check IspInfo.UserCount
		 *  for ensuring current clocks are on or off
		 */
		spin_lock(&(IspInfo.SpinLockClock));
		if (!G_u4EnableClockCount[i]) {
			spin_unlock(&(IspInfo.SpinLockClock));

			LOG_INF("dev(%d) - X. UserCount=%d,Cnt:%d\n",
				i, IspInfo.UserCount,
				G_u4EnableClockCount[i]);

			continue;
		}
		spin_unlock(&(IspInfo.SpinLockClock));

		switch (i) {
		case ISP_CAM_A_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAM_A_ST;
			break;
		case ISP_CAM_B_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAM_B_ST;
			break;
		case ISP_CAM_C_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAM_C_ST;
			break;
		case ISP_CAMSV0_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAMSV_0_ST;
			break;
		case ISP_CAMSV1_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAMSV_1_ST;
			break;
		case ISP_CAMSV2_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAMSV_2_ST;
			break;
		case ISP_CAMSV3_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAMSV_3_ST;
			break;
		case ISP_CAMSV4_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAMSV_4_ST;
			break;
		case ISP_CAMSV5_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAMSV_5_ST;
			break;
		case ISP_CAMSV6_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAMSV_6_ST;
			break;
		case ISP_CAMSV7_IDX:
			IrqType = ISP_IRQ_TYPE_INT_CAMSV_7_ST;
			break;
		default:
			IrqType = ISP_IRQ_TYPE_AMOUNT;
			break;
		}

		regVal = ISP_RD32(CAMX_REG_TG_VF_CON(i));
		if (regVal & 0x01) {
			LOG_INF("dev(%d) suspend,disable VF,wakelock:%d,clk:%d,devct:%d\n",
				i, g_WaitLockCt, G_u4EnableClockCount[i]);

			SuspnedRecord[i] = 1;
			/* disable VF */
			ISP_WR32(CAMX_REG_TG_VF_CON(i), (regVal & (~0x01)));

			/* wait TG idle */
			loopCnt = 3;
			waitirq.Type = IrqType;
			waitirq.EventInfo.Clear = ISP_IRQ_CLEAR_WAIT;
			waitirq.EventInfo.Status = VS_INT_ST;
			waitirq.EventInfo.St_type = SIGNAL_INT;
			waitirq.EventInfo.Timeout = 0x100;
			waitirq.EventInfo.UserKey = 0x0;
			waitirq.bDumpReg = 0;

			do {
				regTGSt = (ISP_RD32(CAMX_REG_TG_INTER_ST(i)) &
					   0x00003F00) >> 8;

				if (regTGSt == 1)
					break;

				LOG_INF("dev(%d): wait 1VD (%d)\n", i, loopCnt);
				ret = ISP_WaitIrq(&waitirq);
				/* first wait is clear wait, */
				/* others are non-clear wait */
				waitirq.EventInfo.Clear = ISP_IRQ_CLEAR_NONE;
			} while (--loopCnt);

			if (-ERESTARTSYS == ret) {
				LOG_INF("dev(%d): interrupt by system signal, wait idle\n", i);

				/* timer */
				time = ktime_get();
				m_sec = time;

				while (regTGSt != 1) {

					regTGSt = (ISP_RD32(CAMX_REG_TG_INTER_ST(
							   i)) & 0x00003F00) >> 8;

					/*timer */
					time = ktime_get();
					sec = time;
					/* wait time>timeoutMs, break */
					if ((sec - m_sec) > timeoutMs)
						break;
				}
				if (regTGSt == 1) {
					LOG_INF("dev(%d): wait idle done\n", i);
				} else {
					LOG_INF("dev(%d): wait idle timeout(%lld)\n",
						i, (sec - m_sec));
				}
			}

			/* backup: frame CNT
			 * After VF enable, The frame count will be 0 at next VD;
			 * if it has P1_DON after set vf disable, g_BkReg no need
			 * to add 1
			 */
			regTGSt = ISP_RD32_TG_CAMX_FRM_CNT(IrqType, i);
			g_BkReg[IrqType].CAM_TG_INTER_ST = regTGSt;
			regVal = ISP_RD32(CAMX_REG_TG_SEN_MODE(i));
			ISP_WR32(CAMX_REG_TG_SEN_MODE(i), (regVal & (~0x01)));
		} else {
			LOG_INF("dev(%d) suspend,wakelock:%d,clk:%d,devct:%d\n", i,
				g_WaitLockCt, G_u4EnableClockCount[i]);

			SuspnedRecord[i] = 0;
		}

		spin_lock(&(IspInfo.SpinLockClock));
		loopCnt = G_u4EnableClockCount[i];
		spin_unlock(&(IspInfo.SpinLockClock));

		LOG_INF("dev(%d) - X. wakelock:%d, last dev node,disable clk:%d\n",
			i, g_WaitLockCt, loopCnt);
		while (loopCnt > 0) {
			ISP_EnableClock(i, MFALSE);
			loopCnt--;
		}
	}

	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void ISP_pm_event_resume(void)
{
}

/*******************************************************************************
 *
 ******************************************************************************/
static int ISP_suspend_pm_event(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_HIBERNATION_PREPARE:
		return NOTIFY_DONE;
	case PM_RESTORE_PREPARE:
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION:
		return NOTIFY_DONE;
	case PM_SUSPEND_PREPARE: /* before enter suspend */
		ISP_pm_event_suspend();
		return NOTIFY_DONE;
	case PM_POST_SUSPEND: /* after resume */
		ISP_pm_event_resume();
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block ISP_suspend_pm_notifier_func = {
	.notifier_call = ISP_suspend_pm_event,
	.priority = 0,
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
static ssize_t ISP_DumpRegToProc(struct file *pFile, char *pStart, size_t off,
				 loff_t *Count)
{
	LOG_NOTICE("ISP_DumpRegToProc_Func: Not implement");
	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static ssize_t ISP_RegDebug(struct file *pFile, const char *pBuffer,
			    size_t Count, loff_t *pData)
{
	LOG_NOTICE("ISP_RegDebug_Func: Not implement");
	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static ssize_t CAMIO_DumpRegToProc(struct file *pFile, char *pStart, size_t off,
				   loff_t *Count)
{
	LOG_NOTICE("CAMIO_DumpRegToProc_Func: Not implement");
	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static ssize_t CAMIO_RegDebug(struct file *pFile, const char *pBuffer,
			      size_t Count, loff_t *pData)
{
	LOG_NOTICE("CAMIO_RegDebug_Func: Not implement");
	return 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
static const struct proc_ops fcameraisp_proc_fops = {
	.proc_read = ISP_DumpRegToProc,
	.proc_write = ISP_RegDebug,
};

static const struct proc_ops fcameraio_proc_fops = {
	.proc_read = CAMIO_DumpRegToProc,
	.proc_write = CAMIO_RegDebug,
};

/*******************************************************************************
 *
 ******************************************************************************/

static int __init ISP_Init(void)
{
	int Ret = 0, i, j, k;
	void *tmp;
//	struct device_node *isp_dev_node = NULL;

#if (SMI_LARB_MMU_CTL == 1)
	struct device_node *node = NULL;
#endif
	/*  */
	LOG_DBG("- E.");
	mutex_init(&open_isp_mutex);

	/*  */
	atomic_set(&G_u4DevNodeCt, 0);
	/*  */

	g_platform_id = GET_PLATFORM_ID("mediatek,camisp_legacy");
	if (g_platform_id == 0) {
		LOG_NOTICE("get platform id failed\n");
		return -ENODEV;
	}

	Ret = platform_driver_register(&IspDriver);
	if ((Ret) < 0) {
		LOG_NOTICE("platform_driver_register failed\n");
		return Ret;
	}

/* Use of_find_compatible_node() sensor registers from device tree */
/* Don't use compatitble define in probe(). */
/* Otherwise, probe() of Seninf driver cannot be called. */
#if (SMI_LARB_MMU_CTL == 1)
	do {
		unsigned int larb_id = 0;

		for (i = 0; i < ARRAY_SIZE(SMI_LARB_BASE); i++)
			SMI_LARB_BASE[i] = 0;

		for (i = 0; i < ARRAY_SIZE(SMI_LARB_BASE); i++) {

			if (IS_MT6873(g_platform_id))
				node = of_find_compatible_node(node, NULL, "mediatek,mt6873-smi-larb");
			else if (IS_MT6853(g_platform_id))
				node = of_find_compatible_node(node, NULL, "mediatek,mt6853-smi-larb");
			else if (IS_MT6893(g_platform_id))
				node = of_find_compatible_node(node, NULL, "mediatek,mt6893-smi-larb");

			if (!node) {
				if (i == 0) {
					LOG_NOTICE("find no larb node\n");
					break;
				}

				LOG_NOTICE("Found %d larb and Search done\n", i);
				break;
			}

			if (of_property_read_u32(node, "mediatek,larb-id", &larb_id)) {
				LOG_NOTICE("Error: get larb id from DTS fail!!\n");
				break;
			} else {
				SMI_LARB_BASE[larb_id] = of_iomap(node, 0);

				if (!SMI_LARB_BASE[larb_id]) {
					LOG_NOTICE(
						"unable to map SMI_LARB_BASE registers!!!\n");
					break;
				}
				LOG_INF("SMI_LARB%d_BASE: 0x%p\n", larb_id, SMI_LARB_BASE[larb_id]);
			}
		}
	} while (0);
#endif

	/* FIX-ME: linux-3.10 procfs API changed */
	proc_create("driver/isp_reg", 0000, NULL, &fcameraisp_proc_fops);
	proc_create("driver/camio_reg", 0000, NULL, &fcameraio_proc_fops);

#ifdef CAM_ISP_DBGFS
	camisp_dbg_root = debugfs_create_dir("camera_isp", NULL);
	debugfs_create_x32("platform", 0644,
		camisp_dbg_root, &g_platform_id);
#endif

	for (j = 0; j < ISP_IRQ_TYPE_AMOUNT; j++) {
		switch (j) {
		case ISP_IRQ_TYPE_INT_CAM_A_ST:
		case ISP_IRQ_TYPE_INT_CAM_B_ST:
		case ISP_IRQ_TYPE_INT_CAM_C_ST:
		case ISP_IRQ_TYPE_INT_CAMSV_0_ST:
		case ISP_IRQ_TYPE_INT_CAMSV_1_ST:
		case ISP_IRQ_TYPE_INT_CAMSV_2_ST:
		case ISP_IRQ_TYPE_INT_CAMSV_3_ST:
		case ISP_IRQ_TYPE_INT_CAMSV_4_ST:
		case ISP_IRQ_TYPE_INT_CAMSV_5_ST:
		case ISP_IRQ_TYPE_INT_CAMSV_6_ST:
		case ISP_IRQ_TYPE_INT_CAMSV_7_ST:
			if (sizeof(struct ISP_RT_BUF_STRUCT) >
			    ((RT_BUF_TBL_NPAGES)*PAGE_SIZE)) {
				i = 0;

				while (i < sizeof(struct ISP_RT_BUF_STRUCT))
					i += PAGE_SIZE;

				pBuf_kmalloc[j] =
					kmalloc(i + 2 * PAGE_SIZE, GFP_KERNEL);

				if ((pBuf_kmalloc[j]) == NULL) {
					LOG_NOTICE("mem not enough\n");
					return -ENOMEM;
				}
				memset(pBuf_kmalloc[j], 0x00, i);
				Tbl_RTBuf_MMPSize[j] = (i / PAGE_SIZE) + 2;
			} else {
				pBuf_kmalloc[j] = kmalloc(
					(RT_BUF_TBL_NPAGES + 2) * PAGE_SIZE,
					GFP_KERNEL);

				if ((pBuf_kmalloc[j]) == NULL) {
					LOG_NOTICE("mem not enough\n");
					return -ENOMEM;
				}
				memset(pBuf_kmalloc[j], 0x00,
				       (RT_BUF_TBL_NPAGES + 2) * PAGE_SIZE);

				Tbl_RTBuf_MMPSize[j] = (RT_BUF_TBL_NPAGES + 2);
			}
			/* round it up to the page bondary */
			pTbl_RTBuf[j] =
				(int *)((((unsigned long)pBuf_kmalloc[j]) +
					 PAGE_SIZE - 1) &
					PAGE_MASK);

			pstRTBuf[j] = (struct ISP_RT_BUF_STRUCT *)pTbl_RTBuf[j];
			pstRTBuf[j]->state = ISP_RTBC_STATE_INIT;
			break;
		default:
			pBuf_kmalloc[j] = NULL;
			pTbl_RTBuf[j] = NULL;
			Tbl_RTBuf_MMPSize[j] = 0;
			break;
		}
	}

	/* isr log */
	if (PAGE_SIZE < ((ISP_IRQ_TYPE_AMOUNT * NORMAL_STR_LEN *
			  ((DBG_PAGE + INF_PAGE + ERR_PAGE) + 1)) *
			 LOG_PPNUM)) {

		i = 0;
		while (i < ((ISP_IRQ_TYPE_AMOUNT * NORMAL_STR_LEN *
			     ((DBG_PAGE + INF_PAGE + ERR_PAGE) + 1)) *
			    LOG_PPNUM))
			i += PAGE_SIZE;

	} else {
		i = PAGE_SIZE;
	}
	pLog_kmalloc = kmalloc(i, GFP_KERNEL);
	if ((pLog_kmalloc) == NULL) {
		LOG_NOTICE("mem not enough\n");
		return -ENOMEM;
	}
	memset(pLog_kmalloc, 0x00, i);
	tmp = pLog_kmalloc;
	for (i = 0; i < LOG_PPNUM; i++) {
		for (j = 0; j < ISP_IRQ_TYPE_AMOUNT; j++) {
			gSvLog[j]._str[i][_LOG_DBG] = (char *)tmp;
			/* tmp = (void*) ((unsigned int)tmp + */
			/* (NORMAL_STR_LEN*DBG_PAGE)); */

			tmp = (void *)((char *)tmp +
				       (NORMAL_STR_LEN * DBG_PAGE));

			gSvLog[j]._str[i][_LOG_INF] = (char *)tmp;
			/* tmp = (void*) ((unsigned int)tmp + */
			/* (NORMAL_STR_LEN*INF_PAGE)); */

			tmp = (void *)((char *)tmp +
				       (NORMAL_STR_LEN * INF_PAGE));

			gSvLog[j]._str[i][_LOG_ERR] = (char *)tmp;
			/* tmp = (void*) ((unsigned int)tmp + */
			/* (NORMAL_STR_LEN*ERR_PAGE)); */

			tmp = (void *)((char *)tmp +
				       (NORMAL_STR_LEN * ERR_PAGE));
		}
		/* //log buffer ,in case of overflow */
		/* tmp = (void*) ((unsigned int)tmp + NORMAL_STR_LEN); */

		/* log buffer ,in case of overflow */
		tmp = (void *)((char *)tmp + NORMAL_STR_LEN);
	}
	/* mark the pages reserved , FOR MMAP */
	for (j = 0; j < ISP_IRQ_TYPE_AMOUNT; j++) {
		if (pTbl_RTBuf[j] != NULL) {
			for (i = 0; i < Tbl_RTBuf_MMPSize[j] * PAGE_SIZE;
			     i += PAGE_SIZE) {
				SetPageReserved(virt_to_page(
					((unsigned long)pTbl_RTBuf[j]) + i));
			}
		}
	}

	for (i = 0; i < ISP_DEV_NODE_NUM; i++)
		SuspnedRecord[i] = 0;

	for (k = ISP_IRQ_TYPE_INT_CAM_A_ST; k <= ISP_IRQ_TYPE_INT_CAM_C_ST; k++)
		ISP_SetPMQOS(E_BW_ADD, k, NULL);

	ISP_SetPMQOS(E_CLK_ADD, ISP_IRQ_TYPE_INT_CAM_A_ST, NULL);

	for (j = RUNTIME_ISP_CAMSV_START_ST;
			j <= ISP_IRQ_TYPE_INT_CAMSV_END_ST;
			j++)
		SV_SetPMQOS(E_BW_ADD, j, NULL);

	SV_SetPMQOS(E_CLK_ADD, ISP_IRQ_TYPE_INT_CAMSV_START_ST, NULL);

#if IS_ENABLED(CONFIG_PM)
	Ret = register_pm_notifier(&ISP_suspend_pm_notifier_func);
	if (Ret) {
		LOG_NOTICE("Failed to register PM notifier.\n");
		return Ret;
	}
#endif

	LOG_DBG("- E. Ret: %d.", Ret);
	return Ret;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void __exit ISP_Exit(void)
{
	int i, j;

	LOG_DBG("- B.");
	/*  */
	platform_driver_unregister(&IspDriver);
	/*  */
	for (i = RUNTIME_ISP_CAMSV_START_ST;
			i <= ISP_IRQ_TYPE_INT_CAMSV_END_ST;
			i++)
		SV_SetPMQOS(E_BW_REMOVE, i, NULL);

	SV_SetPMQOS(E_CLK_REMOVE, ISP_IRQ_TYPE_INT_CAMSV_START_ST, NULL);

	for (i = ISP_IRQ_TYPE_INT_CAM_A_ST; i <= ISP_IRQ_TYPE_INT_CAM_C_ST; i++)
		ISP_SetPMQOS(E_BW_REMOVE, i, NULL);

	ISP_SetPMQOS(E_CLK_REMOVE, ISP_IRQ_TYPE_INT_CAM_A_ST, NULL);

	for (j = 0; j < ISP_IRQ_TYPE_AMOUNT; j++) {
		if (pTbl_RTBuf[j] != NULL) {
			/* unreserve the pages */
			for (i = 0; i < Tbl_RTBuf_MMPSize[j] * PAGE_SIZE;
			     i += PAGE_SIZE) {
				ClearPageReserved(virt_to_page(
					((unsigned long)pTbl_RTBuf[j]) + i));
			}
			/* free the memory areas */
			kfree(pBuf_kmalloc[j]);
		}
	}

	/* free the memory areas */
	kfree(pLog_kmalloc);

	LOG_DBG("- E.");
}

void IRQ_INT_ERR_CHECK_CAM(unsigned int WarnStatus, unsigned int ErrStatus,
			   unsigned int warnTwo, enum ISP_IRQ_TYPE_ENUM module)
{
	/* ERR print */
	if (ErrStatus) {
		switch (module) {
		case ISP_IRQ_TYPE_INT_CAM_A_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_A_ST].ispIntErr |=
				(ErrStatus | warnTwo);

			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_A_ST].ispInt5Err |=
				WarnStatus;

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAM_A_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_A_ST]
					.ispIntErr;

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAM_A_ST]
				.ispInt5Err =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_A_ST]
					.ispInt5Err;

			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_ERR,
				"CAM_A:raw_int_err:0x%x, raw_int5_wrn:0x%x,lsci_wrn:0x%x\n",
				ErrStatus, WarnStatus, warnTwo);

			/* DMA ERR print */
			if (ErrStatus & (DMA_ERR_ST | TG_ERR_ST))
				ISP_DumpDmaDeepDbg(module, ErrStatus);

			break;
		case ISP_IRQ_TYPE_INT_CAM_B_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_B_ST].ispIntErr |=
				(ErrStatus | warnTwo);

			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_B_ST].ispInt5Err |=
				(WarnStatus);

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAM_B_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_B_ST]
					.ispIntErr;

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAM_B_ST]
				.ispInt5Err =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_B_ST]
					.ispInt5Err;

			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_ERR,
				"CAM_B:raw_int_err:0x%x, raw_int5_wrn:0x%x,lsci_wrn:0x%x\n",
				ErrStatus, WarnStatus, warnTwo);

			/* DMA ERR print */
			if (ErrStatus & (DMA_ERR_ST | TG_ERR_ST))
				ISP_DumpDmaDeepDbg(module, ErrStatus);

			break;
		case ISP_IRQ_TYPE_INT_CAM_C_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_C_ST].ispIntErr |=
				(ErrStatus | warnTwo);

			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_C_ST].ispInt5Err |=
				WarnStatus;

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAM_C_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_C_ST]
					.ispIntErr;

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAM_C_ST]
				.ispInt5Err =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAM_C_ST]
					.ispInt5Err;

			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_ERR,
				"CAM_C:raw_int_err:0x%x, raw_int5_wrn:0x%x,lsci_wrn:0x%x\n",
				ErrStatus, WarnStatus, warnTwo);

			/* DMA ERR print */
			if (ErrStatus & (DMA_ERR_ST | TG_ERR_ST))
				ISP_DumpDmaDeepDbg(module, ErrStatus);

			break;
		case ISP_IRQ_TYPE_INT_CAMSV_0_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_0_ST].ispIntErr |=
				(ErrStatus | WarnStatus);

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAMSV_0_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_0_ST]
					.ispIntErr;

			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "CAMSV0:int_err:0x%x_0x%x\n", WarnStatus,
				       ErrStatus);

			break;
		case ISP_IRQ_TYPE_INT_CAMSV_1_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_1_ST].ispIntErr |=
				(ErrStatus | WarnStatus);

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAMSV_1_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_1_ST]
					.ispIntErr;

			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "CAMSV1:int_err:0x%x_0x%x\n", WarnStatus,
				       ErrStatus);
			break;
		case ISP_IRQ_TYPE_INT_CAMSV_2_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_2_ST].ispIntErr |=
				(ErrStatus | WarnStatus);

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAMSV_2_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_2_ST]
					.ispIntErr;

			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "CAMSV2:int_err:0x%x_0x%x\n", WarnStatus,
				       ErrStatus);
			break;
		case ISP_IRQ_TYPE_INT_CAMSV_3_ST:

			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_3_ST].ispIntErr |=
				(ErrStatus | WarnStatus);

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAMSV_3_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_3_ST]
					.ispIntErr;

			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "CAMSV3:int_err:0x%x_0x%x\n", WarnStatus,
				       ErrStatus);

			break;
		case ISP_IRQ_TYPE_INT_CAMSV_4_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_4_ST].ispIntErr |=
				(ErrStatus | WarnStatus);

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAMSV_4_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_4_ST]
					.ispIntErr;

			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "CAMSV4:int_err:0x%x_0x%x\n", WarnStatus,
				       ErrStatus);
			break;
		case ISP_IRQ_TYPE_INT_CAMSV_5_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_5_ST].ispIntErr |=
				(ErrStatus | WarnStatus);

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAMSV_5_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_5_ST]
					.ispIntErr;

			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "CAMSV5:int_err:0x%x_0x%x\n", WarnStatus,
				       ErrStatus);
			break;
		case ISP_IRQ_TYPE_INT_CAMSV_6_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_6_ST].ispIntErr |=
				(ErrStatus | WarnStatus);

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAMSV_6_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_6_ST]
					.ispIntErr;

			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "CAMSV6:int_err:0x%x_0x%x\n", WarnStatus,
				       ErrStatus);
			break;
		case ISP_IRQ_TYPE_INT_CAMSV_7_ST:
			g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_7_ST].ispIntErr |=
				(ErrStatus | WarnStatus);

			g_ISPIntStatus_SMI[ISP_IRQ_TYPE_INT_CAMSV_7_ST]
				.ispIntErr =
				g_ISPIntStatus[ISP_IRQ_TYPE_INT_CAMSV_7_ST]
					.ispIntErr;

			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "CAMSV7:int_err:0x%x_0x%x\n", WarnStatus,
				       ErrStatus);
			break;
		default:
			break;
		}
	}
}

enum CAM_FrameST Irq_CAM_FrameStatus(enum ISP_DEV_NODE_ENUM module,
				     enum ISP_IRQ_TYPE_ENUM irq_mod,
				     unsigned int delayCheck, unsigned int *FBCurStat)
{
	static const unsigned int dma_arry_map[_cam_max_] = {
		0,	/* _imgo_     */
		1,	/* _ufeo_	   */
		2,	/* _rrzo_	   */
		3,	/* _ufgo_	   */
		4,  /* _yuvo_	  */
		5,	/* _yuvbo_    */
		6,	/* _yuvco_    */
		_cam_max_,	/* _tsfso_	 */
		_cam_max_,	/* _aao_	 */
		_cam_max_,	/* _aho_	 */
		_cam_max_,	/* _afo_	 */
		_cam_max_,	/* _pdo_	 */
		_cam_max_,	/* _flko_	 */
		7,	/* _lceso_    */
		8,	/* _lcesho_    */
		_cam_max_, /* _ltmso_    */
		9,	/* _lmvo_	   */
		10,	/* _rsso_     */
		11,	/* _rsso_r2_  */
		12,	/* _crzo_     */
		13,	/* _crzbo_    */
		14,	/* _crzo_r2_  */
	};

	unsigned int dma_en, dma2_en;
	union FBC_CTRL_1 fbc_ctrl1[_cam_max_ + 1];
	union FBC_CTRL_2 fbc_ctrl2[_cam_max_ + 1];
	union FBC_CTRL_2 fbc_ctrl2_STT;
	bool bQueMode = MFALSE;
	unsigned int product = 1;
	/* TSTP_V3 unsigned int frmPeriod = */
	/* ((ISP_RD32(CAM_REG_TG_SUB_PERIOD(module)) >> 8) & 0x1F) + 1; */
	unsigned int i;

	if ((module < ISP_CAM_A_IDX) || (module >= ISP_CAMSV_START_IDX)) {
		LOG_NOTICE("unsupported module:0x%x\n", module);
		return CAM_FST_DROP_FRAME;
	}

	if (sec_on) {
		dma_en = lock_reg.CAM_REG_CTL_DMA_EN[module];
		dma2_en = lock_reg.CAM_REG_CTL_DMA2_EN[module];
	} else {
		dma_en = ISP_RD32(CAM_REG_CTL_DMA_EN(module));
		dma2_en = ISP_RD32(CAM_REG_CTL_DMA2_EN(module));
	}

	if (dma_en & _IMGO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_imgo_]].Raw =
			ISP_RD32(CAM_REG_FBC_IMGO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_imgo_]].Raw =
			ISP_RD32(CAM_REG_FBC_IMGO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_imgo_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _IMGO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_imgo_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_imgo_]].Raw = 0x0;
	}

	if (dma_en & _RRZO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_rrzo_]].Raw =
			ISP_RD32(CAM_REG_FBC_RRZO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_rrzo_]].Raw =
			ISP_RD32(CAM_REG_FBC_RRZO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_rrzo_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _RRZO_R1_EN_;
		}

	} else {
		fbc_ctrl1[dma_arry_map[_rrzo_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_rrzo_]].Raw = 0x0;
	}

	if (dma_en & _LCESO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_lcso_]].Raw =
			ISP_RD32(CAM_REG_FBC_LCESO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_lcso_]].Raw =
			ISP_RD32(CAM_REG_FBC_LCESO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_lcso_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _LCESO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_lcso_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_lcso_]].Raw = 0x0;
	}

	if (dma_en & _LCESHO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_lcesho_]].Raw =
			ISP_RD32(CAM_REG_FBC_LCESHO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_lcesho_]].Raw =
			ISP_RD32(CAM_REG_FBC_LCESHO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_lcesho_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _LCESHO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_lcesho_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_lcesho_]].Raw = 0x0;
	}

	if (dma_en & _UFEO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_ufeo_]].Raw =
			ISP_RD32(CAM_REG_FBC_UFEO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_ufeo_]].Raw =
			ISP_RD32(CAM_REG_FBC_UFEO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_ufeo_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _UFEO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_ufeo_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_ufeo_]].Raw = 0x0;
	}

	if (dma_en & _UFGO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_ufgo_]].Raw =
			ISP_RD32(CAM_REG_FBC_UFGO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_ufgo_]].Raw =
			ISP_RD32(CAM_REG_FBC_UFGO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_ufgo_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _UFGO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_ufgo_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_ufgo_]].Raw = 0x0;
	}

	if (dma_en & _RSSO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_rsso_]].Raw =
			ISP_RD32(CAM_REG_FBC_RSSO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_rsso_]].Raw =
			ISP_RD32(CAM_REG_FBC_RSSO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_rsso_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _RSSO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_rsso_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_rsso_]].Raw = 0x0;
	}

	if (dma_en & _LMVO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_lmvo_]].Raw =
			ISP_RD32(CAM_REG_FBC_LMVO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_lmvo_]].Raw =
			ISP_RD32(CAM_REG_FBC_LMVO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_lmvo_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _LMVO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_lmvo_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_lmvo_]].Raw = 0x0;
	}

	if (dma_en & _YUVBO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_yuvbo_]].Raw =
			ISP_RD32(CAM_REG_FBC_YUVBO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_yuvbo_]].Raw =
			ISP_RD32(CAM_REG_FBC_YUVBO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_yuvbo_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _YUVBO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_yuvbo_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_yuvbo_]].Raw = 0x0;
	}

	if (dma_en & _CRZO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_crzo_]].Raw =
			ISP_RD32(CAM_REG_FBC_CRZO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_crzo_]].Raw =
			ISP_RD32(CAM_REG_FBC_CRZO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_crzo_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _CRZO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_crzo_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_crzo_]].Raw = 0x0;
	}

	if (dma_en & _CRZBO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_crzbo_]].Raw =
			ISP_RD32(CAM_REG_FBC_CRZBO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_crzbo_]].Raw =
			ISP_RD32(CAM_REG_FBC_CRZBO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_crzbo_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _CRZBO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_crzbo_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_crzbo_]].Raw = 0x0;
	}

	if (dma_en & _YUVCO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_yuvco_]].Raw =
			ISP_RD32(CAM_REG_FBC_YUVCO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_yuvco_]].Raw =
			ISP_RD32(CAM_REG_FBC_YUVCO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_yuvco_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _YUVCO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_yuvco_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_yuvco_]].Raw = 0x0;
	}

	if (dma_en & _CRZO_R2_EN_) {
		fbc_ctrl1[dma_arry_map[_crzo_r2_]].Raw =
			ISP_RD32(CAM_REG_FBC_CRZO_R2_CTL1(module));

		fbc_ctrl2[dma_arry_map[_crzo_r2_]].Raw =
			ISP_RD32(CAM_REG_FBC_CRZO_R2_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_crzo_r2_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _CRZO_R2_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_crzo_r2_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_crzo_r2_]].Raw = 0x0;
	}

	if (dma_en & _RSSO_R2_EN_) {
		fbc_ctrl1[dma_arry_map[_rsso_r2_]].Raw =
			ISP_RD32(CAM_REG_FBC_RSSO_R2_CTL1(module));

		fbc_ctrl2[dma_arry_map[_rsso_r2_]].Raw =
			ISP_RD32(CAM_REG_FBC_RSSO_R2_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_rsso_r2_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _RSSO_R2_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_rsso_r2_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_rsso_r2_]].Raw = 0x0;
	}

	if (dma_en & _YUVO_R1_EN_) {
		fbc_ctrl1[dma_arry_map[_yuvo_]].Raw =
			ISP_RD32(CAM_REG_FBC_YUVO_CTL1(module));

		fbc_ctrl2[dma_arry_map[_yuvo_]].Raw =
			ISP_RD32(CAM_REG_FBC_YUVO_CTL2(module));

		if (fbc_ctrl2[dma_arry_map[_yuvo_]].Bits.FBC_CNT > 0) {
			(*FBCurStat) = (*FBCurStat) | _YUVO_R1_EN_;
		}
	} else {
		fbc_ctrl1[dma_arry_map[_yuvo_]].Raw = 0x0;
		fbc_ctrl2[dma_arry_map[_yuvo_]].Raw = 0x0;
	}
	/*STT FBC Status record*/
	if (dma_en & _TSFSO_R1_EN_) {

		fbc_ctrl2_STT.Raw =
			ISP_RD32(CAM_REG_FBC_TSFSO_CTL2(module));

		if (fbc_ctrl2_STT.Bits.FBC_CNT > 0)
			(*FBCurStat) = (*FBCurStat) | _TSFSO_R1_EN_;
	}

	if (dma_en & _AAO_R1_EN_) {

		fbc_ctrl2_STT.Raw =
			ISP_RD32(CAM_REG_FBC_AAO_CTL2(module));

		if (fbc_ctrl2_STT.Bits.FBC_CNT > 0)
			(*FBCurStat) = (*FBCurStat) | _AAO_R1_EN_;
	}

	if (dma_en & _AAHO_R1_EN_) {

		fbc_ctrl2_STT.Raw =
			ISP_RD32(CAM_REG_FBC_AAHO_CTL2(module));

		if (fbc_ctrl2_STT.Bits.FBC_CNT > 0)
			(*FBCurStat) = (*FBCurStat) | _AAHO_R1_EN_;
	}

	if (dma_en & _FLKO_R1_EN_) {

		fbc_ctrl2_STT.Raw =
			ISP_RD32(CAM_REG_FBC_FLKO_CTL2(module));

		if (fbc_ctrl2_STT.Bits.FBC_CNT > 0)
			(*FBCurStat) = (*FBCurStat) | _FLKO_R1_EN_;
	}

	if (dma_en & _AFO_R1_EN_) {

		fbc_ctrl2_STT.Raw =
			ISP_RD32(CAM_REG_FBC_AFO_CTL2(module));

		if (fbc_ctrl2_STT.Bits.FBC_CNT > 0)
			(*FBCurStat) = (*FBCurStat) | _AFO_R1_EN_;
	}

	if (dma_en & _PDO_R1_EN_) {

		fbc_ctrl2_STT.Raw =
			ISP_RD32(CAM_REG_FBC_PDO_CTL2(module));

		if (fbc_ctrl2_STT.Bits.FBC_CNT > 0)
			(*FBCurStat) = (*FBCurStat) | _PDO_R1_EN_;
	}

	if (dma_en & _LTMSO_R1_EN_) {

		fbc_ctrl2_STT.Raw =
			ISP_RD32(CAM_REG_FBC_LTMSO_CTL2(module));

		if (fbc_ctrl2_STT.Bits.FBC_CNT > 0)
			(*FBCurStat) = (*FBCurStat) | _LTMSO_R1_EN_;
	}

	for (i = 0; i < _cam_max_; i++) {
		if (dma_arry_map[i] != _cam_max_) {
			if (fbc_ctrl1[dma_arry_map[i]].Raw != 0) {

				bQueMode = fbc_ctrl1[dma_arry_map[i]]
						   .Bits.FBC_MODE;
				break;
			}
		}
	}

	if (bQueMode) {
		for (i = 0; i < _cam_max_; i++) {
			if (dma_arry_map[i] == _cam_max_)
				continue;

			if (fbc_ctrl1[dma_arry_map[i]].Raw != 0) {
				product *=
					fbc_ctrl2[dma_arry_map[i]].Bits.FBC_CNT;
				if (product == 0)
					return CAM_FST_DROP_FRAME;
			}
		}
	} else {
		for (i = 0; i < _cam_max_; i++) {
			if (dma_arry_map[i] == _cam_max_)
				continue;

			if (fbc_ctrl1[dma_arry_map[i]].Raw != 0) {
				product *=
				(fbc_ctrl1[dma_arry_map[i]].Bits.FBC_NUM -
				fbc_ctrl2[dma_arry_map[i]].Bits.FBC_CNT);

				if (product == 0)
					return CAM_FST_DROP_FRAME;
			}
		}
	}

	if (product == 1)
		return CAM_FST_LAST_WORKING_FRAME;
	else
		return CAM_FST_NORMAL;
}

#if ((TIMESTAMP_QUEUE_EN == 1) || (Lafi_WAM_CQ_ERR == 1))
static void ISP_GetDmaPortsStatus(enum ISP_DEV_NODE_ENUM reg_module,
				  unsigned int *DmaPortsStats)
{
	unsigned int dma_en = 0, dma2_en = 0;

	if (sec_on) {
		dma_en = lock_reg.CAM_REG_CTL_DMA_EN[reg_module];
		dma2_en = lock_reg.CAM_REG_CTL_DMA2_EN[reg_module];
	} else {
		dma_en = ISP_RD32(CAM_REG_CTL_DMA_EN(reg_module));
		dma2_en = ISP_RD32(CAM_REG_CTL_DMA2_EN(reg_module));
	}
	/* dma */
	DmaPortsStats[_imgo_] = ((dma_en & _IMGO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_ltmso_] = ((dma_en & _LTMSO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_rrzo_] = ((dma_en & _RRZO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_lcso_] = ((dma_en & _LCESO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_lcesho_] = ((dma_en & _LCESHO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_aao_] = ((dma_en & _AAO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_aaho_] = ((dma_en & _AAHO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_flko_] = ((dma_en & _FLKO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_ufeo_] = ((dma_en & _UFEO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_afo_] = ((dma_en & _AFO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_ufgo_] = ((dma_en & _UFGO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_rsso_] = ((dma_en & _RSSO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_lmvo_] = ((dma_en & _LMVO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_yuvbo_] = ((dma_en & _YUVBO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_tsfso_] = ((dma_en & _TSFSO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_pdo_] = ((dma_en & _PDO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_crzo_] = ((dma_en & _CRZO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_crzbo_] = ((dma_en & _CRZBO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_yuvco_] = ((dma_en & _YUVCO_R1_EN_) ? 1 : 0);
	DmaPortsStats[_crzo_r2_] = ((dma_en & _CRZO_R2_EN_) ? 1 : 0);
	DmaPortsStats[_rsso_r2_] = ((dma_en & _RSSO_R2_EN_) ? 1 : 0);
	DmaPortsStats[_yuvo_] = ((dma_en & _YUVO_R1_EN_) ? 1 : 0);
}

#endif
#if (TIMESTAMP_QUEUE_EN == 1)
static enum CAM_FrameST Irq_CAM_SttFrameStatus(enum ISP_DEV_NODE_ENUM module,
					       enum ISP_IRQ_TYPE_ENUM irq_mod,
					       unsigned int dma_id,
					       unsigned int delayCheck)
{
	static const int dma_arry_map[_cam_max_] = {
		_cam_max_,	 /* _imgo_     */
		_cam_max_,	 /* _ufeo_	   */
		_cam_max_,	 /* _rrzo_	   */
		_cam_max_,	 /* _ufgo_	   */
		_cam_max_,  /* _yuvo_	  */
		_cam_max_,	 /* _yuvbo_    */
		_cam_max_,	 /* _yuvco_    */
		0, /* _tsfso_	 */
		1, /* _aao_	 */
		2, /* _aaho_	 */
		3, /* _afo_	 */
		4, /* _pdo_	 */
		5, /* _flko_	 */
		_cam_max_,	 /* _lceso_    */
		_cam_max_,	 /* _lcesho_    */
		6, /* _ltmso_    */
		_cam_max_,	 /* _lmvo_	   */
		_cam_max_,	 /* _rsso_     */
		_cam_max_,  /* _rsso_r2_  */
		_cam_max_,	 /* _crzo_     */
		_cam_max_,	 /* _crzbo_    */
		_cam_max_,	 /* _crzo_r2_  */
	};

	unsigned int dma_en;
	union FBC_CTRL_1 fbc_ctrl1;
	union FBC_CTRL_2 fbc_ctrl2;
	bool bQueMode = MFALSE;
	unsigned int product = 1;
	/* TSTP_V3 unsigned int     frmPeriod = 1; */

	switch (module) {
	case ISP_CAM_A_IDX:
	case ISP_CAM_B_IDX:
	case ISP_CAM_C_IDX:
		if (dma_id >= _cam_max_) {
			LOG_NOTICE(
				"LINE_%d ERROR: unsupported module:0x%x dma:%d\n",
				__LINE__, module, dma_id);

			return CAM_FST_DROP_FRAME;
		}
		if (dma_arry_map[dma_id] == _cam_max_) {
			LOG_NOTICE(
				"LINE_%d ERROR: unsupported module:0x%x dma:%d\n",
				__LINE__, module, dma_id);

			return CAM_FST_DROP_FRAME;
		}
		break;
	default:
		LOG_NOTICE("LINE_%d ERROR: unsupported module:0x%x dma:%d\n",
			   __LINE__, module, dma_id);

		return CAM_FST_DROP_FRAME;
	}

	fbc_ctrl1.Raw = 0x0;
	fbc_ctrl2.Raw = 0x0;

	if (sec_on)
		dma_en = lock_reg.CAM_REG_CTL_DMA_EN[module];
	else
		dma_en = ISP_RD32(CAM_REG_CTL_DMA_EN(module));

	if (_aao_ == dma_id) {
		if (dma_en & _AAO_R1_EN_) {
			fbc_ctrl1.Raw = ISP_RD32(CAM_REG_FBC_AAO_CTL1(module));
			fbc_ctrl2.Raw = ISP_RD32(CAM_REG_FBC_AAO_CTL2(module));
		}
	}

	if (_aaho_ == dma_id) {
		if (dma_en & _AAHO_R1_EN_) {
			fbc_ctrl1.Raw = ISP_RD32(CAM_REG_FBC_AAHO_CTL1(module));
			fbc_ctrl2.Raw = ISP_RD32(CAM_REG_FBC_AAHO_CTL2(module));
		}
	}

	if (_afo_ == dma_id) {
		if (dma_en & _AFO_R1_EN_) {
			fbc_ctrl1.Raw = ISP_RD32(CAM_REG_FBC_AFO_CTL1(module));
			fbc_ctrl2.Raw = ISP_RD32(CAM_REG_FBC_AFO_CTL2(module));
		}
	}

	if (_pdo_ == dma_id) {
		if (dma_en & _PDO_R1_EN_) {
			fbc_ctrl1.Raw = ISP_RD32(CAM_REG_FBC_PDO_CTL1(module));
			fbc_ctrl2.Raw = ISP_RD32(CAM_REG_FBC_PDO_CTL2(module));
		}
	}

	if (_flko_ == dma_id) {
		if (dma_en & _FLKO_R1_EN_) {
			fbc_ctrl1.Raw = ISP_RD32(CAM_REG_FBC_FLKO_CTL1(module));
			fbc_ctrl2.Raw = ISP_RD32(CAM_REG_FBC_FLKO_CTL2(module));
		}
	}

	if (_tsfso_ == dma_id) {
		if (dma_en & _TSFSO_R1_EN_) {
			fbc_ctrl1.Raw =
				ISP_RD32(CAM_REG_FBC_TSFSO_CTL1(module));

			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_TSFSO_CTL2(module));
		}
	}

	if (_ltmso_ == dma_id) {
		if (dma_en & _LTMSO_R1_EN_) {
			fbc_ctrl1.Raw =
				ISP_RD32(CAM_REG_FBC_LTMSO_CTL1(module));

			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_LTMSO_CTL2(module));
		}
	}

	bQueMode = fbc_ctrl1.Bits.FBC_MODE;

	if (bQueMode) {
		if (fbc_ctrl1.Raw != 0) {
			product *= fbc_ctrl2.Bits.FBC_CNT;

		if (product == 0)
			return CAM_FST_DROP_FRAME;
		} else
			return CAM_FST_DROP_FRAME;
	} else {
		if (fbc_ctrl1.Raw != 0) {
			product *= (fbc_ctrl1.Bits.FBC_NUM -
				    fbc_ctrl2.Bits.FBC_CNT);

			if (product == 0)
				return CAM_FST_DROP_FRAME;
		} else
			return CAM_FST_DROP_FRAME;
	}

	if (product == 1)
		return CAM_FST_LAST_WORKING_FRAME;
	else
		return CAM_FST_NORMAL;
}

static int32_t ISP_PushBufTimestamp(unsigned int module, unsigned int dma_id,
				    unsigned int sec, unsigned int usec,
				    unsigned int frmPeriod)
{
	unsigned int wridx = 0;
	union FBC_CTRL_2 fbc_ctrl2;
	enum ISP_DEV_NODE_ENUM reg_module;

	fbc_ctrl2.Raw = 0x0;

	switch (module) {
	case ISP_IRQ_TYPE_INT_CAM_A_ST:
		reg_module = ISP_CAM_A_IDX;
		break;
	case ISP_IRQ_TYPE_INT_CAM_B_ST:
		reg_module = ISP_CAM_B_IDX;
		break;
	case ISP_IRQ_TYPE_INT_CAM_C_ST:
		reg_module = ISP_CAM_C_IDX;
		break;
	default:
		LOG_NOTICE("Unsupport module:x%x\n", module);
		return -EFAULT;
	}

	switch (module) {
	case ISP_IRQ_TYPE_INT_CAM_A_ST:
	case ISP_IRQ_TYPE_INT_CAM_B_ST:
	case ISP_IRQ_TYPE_INT_CAM_C_ST:
		switch (dma_id) {
		case _imgo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_IMGO_CTL2(reg_module));
			break;
		case _ltmso_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_LTMSO_CTL2(reg_module));
			break;
		case _rrzo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_RRZO_CTL2(reg_module));
			break;
		case _lcso_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_LCESO_CTL2(reg_module));
			break;
		case _lcesho_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_LCESHO_CTL2(reg_module));
			break;
		case _aao_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_AAO_CTL2(reg_module));
			break;
		case _aaho_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_AAHO_CTL2(reg_module));
			break;
		case _flko_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_FLKO_CTL2(reg_module));
			break;
		case _ufeo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_UFEO_CTL2(reg_module));
			break;
		case _afo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_AFO_CTL2(reg_module));
			break;
		case _ufgo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_UFGO_CTL2(reg_module));
			break;
		case _rsso_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_RSSO_CTL2(reg_module));
			break;
		case _lmvo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_LMVO_CTL2(reg_module));
			break;
		case _yuvbo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_YUVBO_CTL2(reg_module));
			break;
		case _tsfso_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_TSFSO_CTL2(reg_module));
			break;
		case _pdo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_PDO_CTL2(reg_module));
			break;
		case _crzo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_CRZO_CTL2(reg_module));
			break;
		case _crzbo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_CRZBO_CTL2(reg_module));
			break;
		case _yuvco_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_YUVCO_CTL2(reg_module));
			break;
		case _crzo_r2_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_CRZO_R2_CTL2(reg_module));
			break;
		case _rsso_r2_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_RSSO_R2_CTL2(reg_module));
			break;
		case _yuvo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_YUVO_CTL2(reg_module));
			break;
		default:
			LOG_NOTICE("Unsupport dma:x%x\n", dma_id);
			return -EFAULT;
		}
		break;
	default:
		return -EFAULT;
	}

	if (frmPeriod > 1) {
		fbc_ctrl2.Bits.WCNT =
			(fbc_ctrl2.Bits.WCNT / frmPeriod) * frmPeriod;
	}
	if (((fbc_ctrl2.Bits.WCNT + frmPeriod) & 63) ==
	    IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt) {
		IRQ_LOG_KEEPER(
			module, m_CurrentPPB, _LOG_INF,
			"Cam:%d dma:%d ignore push wcnt_%d_%d\n", module,
			dma_id,
			IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt,
			fbc_ctrl2.Bits.WCNT);
		return 0;
	}

	wridx = IspInfo.TstpQInfo[module].Dmao[dma_id].WrIndex;

	IspInfo.TstpQInfo[module].Dmao[dma_id].TimeQue[wridx].sec = sec;
	IspInfo.TstpQInfo[module].Dmao[dma_id].TimeQue[wridx].usec = usec;

	if (IspInfo.TstpQInfo[module].Dmao[dma_id].WrIndex >=
	    (ISP_TIMESTPQ_DEPTH - 1))
		IspInfo.TstpQInfo[module].Dmao[dma_id].WrIndex = 0;
	else
		IspInfo.TstpQInfo[module].Dmao[dma_id].WrIndex++;

	IspInfo.TstpQInfo[module].Dmao[dma_id].TotalWrCnt++;

	/* Update WCNT for patch timestamp when SOF ISR missing */
	IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt =
		((IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt + 1) &
		 0x3F);

	return 0;
}

static int32_t ISP_PopBufTimestamp(unsigned int module, unsigned int dma_id,
				   struct S_START_T *pTstp)
{
	switch (module) {
	case ISP_IRQ_TYPE_INT_CAM_A_ST:
	case ISP_IRQ_TYPE_INT_CAM_B_ST:
	case ISP_IRQ_TYPE_INT_CAM_C_ST:
		switch (dma_id) {
		case _imgo_:
		case _ltmso_:
		case _rrzo_:
		case _lcso_:
		case _lcesho_:
		case _aao_:
		case _aaho_:
		case _flko_:
		case _ufeo_:
		case _afo_:
		case _ufgo_:
		case _rsso_:
		case _lmvo_:
		case _yuvbo_:
		case _tsfso_:
		case _pdo_:
		case _crzo_:
		case _crzbo_:
		case _yuvco_:
		case _crzo_r2_:
		case _rsso_r2_:
		case _yuvo_:
			break;
		default:
			LOG_NOTICE("Unsupport dma:x%x\n", dma_id);
			return -EFAULT;
		}
		break;
	case ISP_IRQ_TYPE_INT_CAMSV_0_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_1_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_2_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_3_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_4_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_5_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_6_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_7_ST:
		switch (dma_id) {
		case _camsv_imgo_:
			break;
		default:
			LOG_NOTICE("Unsupport dma:x%x\n", dma_id);
			return -EFAULT;
		}
		break;
	default:
		LOG_NOTICE("Unsupport module:x%x\n", module);
		return -EFAULT;
	}

	if (pTstp) {
		*pTstp = IspInfo.TstpQInfo[module]
				 .Dmao[dma_id]
				 .TimeQue[IspInfo.TstpQInfo[module]
						  .Dmao[dma_id]
						  .RdIndex];
	}
	if (IspInfo.TstpQInfo[module].Dmao[dma_id].RdIndex >=
	    (ISP_TIMESTPQ_DEPTH - 1))
		IspInfo.TstpQInfo[module].Dmao[dma_id].RdIndex = 0;
	else
		IspInfo.TstpQInfo[module].Dmao[dma_id].RdIndex++;

	IspInfo.TstpQInfo[module].Dmao[dma_id].TotalRdCnt++;

	return 0;
}

static int32_t ISP_WaitTimestampReady(unsigned int module, unsigned int dma_id)
{
	unsigned int _timeout = 0;
	unsigned int wait_cnt = 0;

	if (IspInfo.TstpQInfo[module].Dmao[dma_id].TotalWrCnt >
	    IspInfo.TstpQInfo[module].Dmao[dma_id].TotalRdCnt)
		return 0;

	LOG_INF("Wait module:%d dma:%d timestamp ready W/R:%d/%d\n", module,
		dma_id,
		(unsigned int)IspInfo.TstpQInfo[module].Dmao[dma_id].TotalWrCnt,
		(unsigned int)IspInfo.TstpQInfo[module]
			.Dmao[dma_id]
			.TotalRdCnt);

	for (wait_cnt = 3; wait_cnt > 0; wait_cnt--) {
		_timeout = wait_event_interruptible_timeout(
			IspInfo.WaitQueueHead[module],
			(IspInfo.TstpQInfo[module].Dmao[dma_id].TotalWrCnt >
			 IspInfo.TstpQInfo[module].Dmao[dma_id].TotalRdCnt),
			ISP_MsToJiffies(2000));
		/* check if user is interrupted by system signal */
		if ((_timeout != 0) &&
		    (!(IspInfo.TstpQInfo[module].Dmao[dma_id].TotalWrCnt >
		       IspInfo.TstpQInfo[module].Dmao[dma_id].TotalRdCnt))) {
			LOG_INF(
				"interrupted by system signal, return value(%d)\n",
				_timeout);
			return -ERESTARTSYS;
		}

		if (_timeout > 0)
			break;

		LOG_INF("WARNING: cam:%d dma:%d wait left count %d\n", module,
			dma_id, wait_cnt);
	}
	if (wait_cnt == 0) {
		LOG_NOTICE("ERROR: cam:%d dma:%d wait timestamp timeout!!!\n",
			   module, dma_id);

		return -EFAULT;
	}

	return 0;
}

static int32_t ISP_CompensateMissingSofTime(enum ISP_DEV_NODE_ENUM reg_module,
					    unsigned int module,
					    unsigned int dma_id,
					    unsigned int sec, unsigned int usec,
					    unsigned int frmPeriod)
{
	union FBC_CTRL_2 fbc_ctrl2;
	unsigned int delta_wcnt = 0, wridx = 0, wridx_prev1 = 0;
	unsigned int wridx_prev2 = 0, i = 0;
	unsigned int delta_time = 0, max_delta_time = 0;
	struct S_START_T time_prev1, time_prev2;

	/*To shrink error log, only rrzo print error log */
	bool dmao_mask = MFALSE;

	/*
	 * Patch timestamp and WCNT base on current HW WCNT and
	 * previous SW WCNT value, and calculate difference
	 */

	fbc_ctrl2.Raw = 0;

	switch (module) {
	case ISP_IRQ_TYPE_INT_CAM_A_ST:
	case ISP_IRQ_TYPE_INT_CAM_B_ST:
	case ISP_IRQ_TYPE_INT_CAM_C_ST:
		switch (dma_id) {
		case _imgo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_IMGO_CTL2(reg_module));
			break;
		case _ltmso_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_LTMSO_CTL2(reg_module));
			break;
		case _rrzo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_RRZO_CTL2(reg_module));
			dmao_mask = MTRUE;
			break;
		case _lcso_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_LCESO_CTL2(reg_module));
			break;
		case _lcesho_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_LCESHO_CTL2(reg_module));
			break;
		case _aao_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_AAO_CTL2(reg_module));
			break;
		case _aaho_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_AAHO_CTL2(reg_module));
			break;
		case _flko_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_FLKO_CTL2(reg_module));
			break;
		case _ufeo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_UFEO_CTL2(reg_module));
			break;
		case _afo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_AFO_CTL2(reg_module));
			break;
		case _ufgo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_UFGO_CTL2(reg_module));
			break;
		case _rsso_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_RSSO_CTL2(reg_module));
			break;
		case _lmvo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_LMVO_CTL2(reg_module));
			break;
		case _yuvbo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_YUVBO_CTL2(reg_module));
			break;
		case _tsfso_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_TSFSO_CTL2(reg_module));
			break;
		case _pdo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_PDO_CTL2(reg_module));
			break;
		case _crzo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_CRZO_CTL2(reg_module));
			break;
		case _crzbo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_CRZBO_CTL2(reg_module));
			break;
		case _yuvco_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_YUVCO_CTL2(reg_module));
			break;
		case _crzo_r2_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_CRZO_R2_CTL2(reg_module));
			break;
		case _rsso_r2_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_RSSO_R2_CTL2(reg_module));
			break;
		case _yuvo_:
			fbc_ctrl2.Raw =
				ISP_RD32(CAM_REG_FBC_YUVO_CTL2(reg_module));
			break;
		default:
			LOG_NOTICE("Unsupport dma:x%x\n", dma_id);
			return -EFAULT;
		}
		break;
	case ISP_IRQ_TYPE_INT_CAMSV_0_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_1_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_2_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_3_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_4_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_5_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_6_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_7_ST:
	default:
		LOG_NOTICE("Unsupport module:x%x\n", module);
		return -EFAULT;
	}

	if (frmPeriod > 1) {
		fbc_ctrl2.Bits.WCNT =
			(fbc_ctrl2.Bits.WCNT / frmPeriod) * frmPeriod;
	}

	if (((fbc_ctrl2.Bits.WCNT + frmPeriod) & 63) ==
	    IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt) {
		if (dmao_mask)
			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"Cam:%d dma:%d ignore compensate wcnt_%d_%d\n",
				module, dma_id, IspInfo.TstpQInfo[module]
							.Dmao[dma_id]
							.PrevFbcWCnt,
				fbc_ctrl2.Bits.WCNT);
		return 0;
	}

	if (IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt >
	    fbc_ctrl2.Bits.WCNT) {
		delta_wcnt = fbc_ctrl2.Bits.WCNT + 64 -
			     IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt;
	} else {
		delta_wcnt = fbc_ctrl2.Bits.WCNT -
			     IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt;
	}
	if (delta_wcnt > 255) {
		if (dmao_mask)
			LOG_NOTICE("ERROR: Cam:%d dma:%d WRONG WCNT:%d_%d_%d\n",
				   module, dma_id, delta_wcnt,
				   IspInfo.TstpQInfo[module]
					   .Dmao[dma_id]
					   .PrevFbcWCnt,
				   fbc_ctrl2.Bits.WCNT);
		return -EFAULT;
	} else if (delta_wcnt > 6) {
		if (dmao_mask) {
			LOG_NOTICE(
				"WARNING: Cam:%d dma:%d SUSPICIOUS WCNT:%d_%d_%d\n",
				module, dma_id, delta_wcnt,
				IspInfo.TstpQInfo[module]
					.Dmao[dma_id]
					.PrevFbcWCnt,
				fbc_ctrl2.Bits.WCNT);
		}
	} else if (delta_wcnt == 0) {
		return 0;
	}

	/* delta_wcnt *= frmPeriod; */

	/* Patch missing SOF timestamp */
	wridx = IspInfo.TstpQInfo[module].Dmao[dma_id].WrIndex;
	wridx_prev1 = (wridx == 0) ? (ISP_TIMESTPQ_DEPTH - 1) : (wridx - 1);

	wridx_prev2 = (wridx_prev1 == 0) ? (ISP_TIMESTPQ_DEPTH - 1)
					 : (wridx_prev1 - 1);

	time_prev1.sec =
		IspInfo.TstpQInfo[module].Dmao[dma_id].TimeQue[wridx_prev1].sec;

	time_prev1.usec = IspInfo.TstpQInfo[module]
				  .Dmao[dma_id]
				  .TimeQue[wridx_prev1]
				  .usec;

	time_prev2.sec =
		IspInfo.TstpQInfo[module].Dmao[dma_id].TimeQue[wridx_prev2].sec;

	time_prev2.usec = IspInfo.TstpQInfo[module]
				  .Dmao[dma_id]
				  .TimeQue[wridx_prev2]
				  .usec;

	if ((sec > time_prev1.sec) ||
	    ((sec == time_prev1.sec) && (usec > time_prev1.usec))) {
		max_delta_time = ((sec - time_prev1.sec) * 1000000 + usec) -
				 time_prev1.usec;

	} else {
		if (dmao_mask) {
			LOG_NOTICE(
				"ERROR: Cam:%d dma:%d current timestamp: cur: %d.%06d prev1: %d.%06d\n",
				module, dma_id, sec, usec, time_prev1.sec,
				time_prev1.usec);
		}
		max_delta_time = 0;
	}

	if ((time_prev1.sec > time_prev2.sec) ||
	    ((time_prev1.sec == time_prev2.sec) &&
	     (time_prev1.usec > time_prev2.usec))) {

		delta_time = ((time_prev1.sec - time_prev2.sec) * 1000000 +
			      time_prev1.usec) -
			     time_prev2.usec;
	} else {
		if (dmao_mask) {
			LOG_NOTICE(
				"ERROR: Cam:%d dma:%d previous timestamp: prev1: %d.%06d prev2: %d.%06d\n",
				module, dma_id, time_prev1.sec, time_prev1.usec,
				time_prev2.sec, time_prev2.usec);
		}
		delta_time = 0;
	}

	if (delta_time > (max_delta_time / delta_wcnt)) {
		if (dmao_mask)
			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"WARNING: Cam:%d dma:%d delta time too large: cur %dus max %dus patch wcnt: %d\n",
				module, dma_id, delta_time, max_delta_time,
				delta_wcnt);
		delta_time = max_delta_time / delta_wcnt;
	}

	for (i = 0; i < delta_wcnt; i++) {
		time_prev1.usec += delta_time;
		while (time_prev1.usec >= 1000000) {
			time_prev1.usec -= 1000000;
			time_prev1.sec++;
		}
		/* WCNT will be increase in this API */
		ISP_PushBufTimestamp(module, dma_id, time_prev1.sec,
				     time_prev1.usec, frmPeriod);
	}

	if (dmao_mask) {
		IRQ_LOG_KEEPER(
			module, m_CurrentPPB, _LOG_INF,
			"Cam:%d dma:%d wcnt:%d_%d_%d T:%d.%06d_.%06d_%d.%06d\n",
			module, dma_id, delta_wcnt,
			IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt,
			fbc_ctrl2.Bits.WCNT, sec, usec, delta_time,
			time_prev1.sec, time_prev1.usec);
	}
	if (IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt !=
	    fbc_ctrl2.Bits.WCNT) {
		if (dmao_mask) {
			LOG_NOTICE(
				"ERROR: Cam:%d dma:%d strange WCNT SW_HW: %d_%d\n",
				module, dma_id, IspInfo.TstpQInfo[module]
							.Dmao[dma_id]
							.PrevFbcWCnt,
				fbc_ctrl2.Bits.WCNT);
		}
		IspInfo.TstpQInfo[module].Dmao[dma_id].PrevFbcWCnt =
			fbc_ctrl2.Bits.WCNT;
	}

	return 0;
}

#if (TSTMP_SUBSAMPLE_INTPL == 1)
static int32_t ISP_PatchTimestamp(unsigned int module, unsigned int dma_id,
				  unsigned int frmPeriod,
				  unsigned long long refTimestp,
				  unsigned long long prevTimestp)
{
	unsigned long long prev_tstp = prevTimestp, cur_tstp = refTimestp;
	unsigned int target_wridx = 0, curr_wridx = 0, frm_dt = 0;
	unsigned int last_frm_dt = 0, i = 1;

	/* Only sub-sample case needs patch */
	if (frmPeriod <= 1)
		return 0;

	curr_wridx = IspInfo.TstpQInfo[module].Dmao[dma_id].WrIndex;

	if (curr_wridx < frmPeriod)
		target_wridx = (curr_wridx + ISP_TIMESTPQ_DEPTH - frmPeriod);
	else
		target_wridx = curr_wridx - frmPeriod;

	frm_dt = (((unsigned int)(cur_tstp - prev_tstp)) / frmPeriod);
	last_frm_dt = ((cur_tstp - prev_tstp) - frm_dt * (frmPeriod - 1));

	if (frm_dt == 0) {
		LOG_INF("WARNING: timestamp delta too small: %d\n",
			(int)(cur_tstp - prev_tstp));
	}
	i = 0;
	while (target_wridx != curr_wridx) {

		if (i > frmPeriod) {
			LOG_NOTICE(
				"Error: too many intpl in sub-sample period %d_%d\n",
				target_wridx, curr_wridx);

			return -EFAULT;
		}

		IspInfo.TstpQInfo[module]
			.Dmao[dma_id]
			.TimeQue[target_wridx]
			.usec += (frm_dt * i);

		while (IspInfo.TstpQInfo[module]
			       .Dmao[dma_id]
			       .TimeQue[target_wridx]
			       .usec >= 1000000) {

			IspInfo.TstpQInfo[module]
				.Dmao[dma_id]
				.TimeQue[target_wridx]
				.usec -= 1000000;

			IspInfo.TstpQInfo[module]
				.Dmao[dma_id]
				.TimeQue[target_wridx]
				.sec++;
		}

		i++;
		target_wridx++; /* patch from 2nd time */
		if (target_wridx >= ISP_TIMESTPQ_DEPTH)
			target_wridx = 0;
	}

	return 0;
}
#endif

#endif

irqreturn_t ISP_Irq_CAMSV_0(int Irq, void *DeviceId)
{
	return ISP_Irq_CAMSV(ISP_IRQ_TYPE_INT_CAMSV_0_ST, ISP_CAMSV0_IDX,
			     "CAMSV0");
}

irqreturn_t ISP_Irq_CAMSV_1(int Irq, void *DeviceId)
{
	return ISP_Irq_CAMSV(ISP_IRQ_TYPE_INT_CAMSV_1_ST, ISP_CAMSV1_IDX,
			     "CAMSV1");
}

irqreturn_t ISP_Irq_CAMSV_2(int Irq, void *DeviceId)
{
	return ISP_Irq_CAMSV(ISP_IRQ_TYPE_INT_CAMSV_2_ST, ISP_CAMSV2_IDX,
			     "CAMSV2");
}

irqreturn_t ISP_Irq_CAMSV_3(int Irq, void *DeviceId)
{
	return ISP_Irq_CAMSV(ISP_IRQ_TYPE_INT_CAMSV_3_ST, ISP_CAMSV3_IDX,
			     "CAMSV3");
}

irqreturn_t ISP_Irq_CAMSV_4(int Irq, void *DeviceId)
{
	return ISP_Irq_CAMSV(ISP_IRQ_TYPE_INT_CAMSV_4_ST, ISP_CAMSV4_IDX,
			     "CAMSV4");
}

irqreturn_t ISP_Irq_CAMSV_5(int Irq, void *DeviceId)
{
	return ISP_Irq_CAMSV(ISP_IRQ_TYPE_INT_CAMSV_5_ST, ISP_CAMSV5_IDX,
			     "CAMSV5");
}

irqreturn_t ISP_Irq_CAMSV_6(int Irq, void *DeviceId)
{
	return ISP_Irq_CAMSV(ISP_IRQ_TYPE_INT_CAMSV_6_ST, ISP_CAMSV6_IDX,
			     "CAMSV6");
}

irqreturn_t ISP_Irq_CAMSV_7(int Irq, void *DeviceId)
{
	return ISP_Irq_CAMSV(ISP_IRQ_TYPE_INT_CAMSV_7_ST, ISP_CAMSV7_IDX,
			     "CAMSV7");
}

irqreturn_t ISP_Irq_CAMSV(enum ISP_IRQ_TYPE_ENUM irq_module,
			  enum ISP_DEV_NODE_ENUM cam_idx, const char *str)
{
	unsigned int module = irq_module;
	unsigned int reg_module = cam_idx;
	unsigned int i, IrqStatus, ErrStatus, time_stamp, cur_v_cnt = 0;
	unsigned int IrqEnableOrig, IrqEnableNew;

	union FBC_CTRL_1 fbc_ctrl1[2];
	/* */
	union FBC_CTRL_2 fbc_ctrl2[2];

	struct timespec64 time_frmb;
	unsigned long long sec = 0;
	unsigned long usec = 0;
	ktime_t time;

	/* Avoid touch hwmodule when clock is disable. */
	/* DEVAPC will moniter this kind of err */
	if (G_u4EnableClockCount[cam_idx] == 0)
		return IRQ_HANDLED;

	/*  */
	ktime_get_ts64(&time_frmb);
	usec = time_frmb.tv_nsec / 1000;
	sec = time_frmb.tv_sec;

#if (ISP_BOTTOMHALF_WORKQ == 1)
	gSvLog[module]._lastIrqTime.sec = sec;
	gSvLog[module]._lastIrqTime.usec = usec;
#endif

	spin_lock(&(IspInfo.SpinLockIrq[module]));
	IrqStatus = ISP_RD32(CAMSV_REG_INT_STATUS(reg_module));
	spin_unlock(&(IspInfo.SpinLockIrq[module]));

	ErrStatus = IrqStatus & IspInfo.IrqInfo.ErrMask[module][SIGNAL_INT];
	IrqStatus = IrqStatus & IspInfo.IrqInfo.Mask[module][SIGNAL_INT];

	/* Check ERR/WRN ISR times, if it occur too frequently,
	 * mark it for avoding keep enter ISR. It will happen KE
	 */
	for (i = 0; i < ISP_ISR_MAX_NUM; i++) {
		/* Only check irq that un marked yet */
		if (!(IspInfo.IrqCntInfo.m_err_int_mark[module] & (1 << i))) {

			if (ErrStatus & (1 << i))
				IspInfo.IrqCntInfo.m_err_int_cnt[module][i]++;

			if (usec - IspInfo.IrqCntInfo.m_int_usec[module] <
			    INT_ERR_WARN_TIMER_THREAS) {
				if (IspInfo.IrqCntInfo
					    .m_err_int_cnt[module][i] >=
				    INT_ERR_WARN_MAX_TIME)
					IspInfo.IrqCntInfo
						.m_err_int_mark[module] |=
						(1 << i);

			} else {
				IspInfo.IrqCntInfo.m_int_usec[module] = usec;
				IspInfo.IrqCntInfo.m_err_int_cnt[module][i] = 0;
			}
		}
	}

	spin_lock(&(IspInfo.SpinLockIrq[module]));
	IrqEnableOrig = ISP_RD32(CAMSV_REG_INT_EN(reg_module));
	spin_unlock(&(IspInfo.SpinLockIrq[module]));

	IrqEnableNew =
		IrqEnableOrig & ~(IspInfo.IrqCntInfo.m_err_int_mark[module]);
	ISP_WR32(CAMSV_REG_INT_EN(reg_module), IrqEnableNew);
	/*  */
	IRQ_INT_ERR_CHECK_CAM(0, ErrStatus, 0, module);

	fbc_ctrl1[0].Raw = ISP_RD32(CAMSV_REG_FBC_IMGO_CTL1(reg_module));
	fbc_ctrl2[0].Raw = ISP_RD32(CAMSV_REG_FBC_IMGO_CTL2(reg_module));
	time_stamp = ISP_RD32(CAMSV_REG_TG_TIME_STAMP(reg_module));

	/* sof , done order chech . */
	if ((IrqStatus & SV_HW_PASS1_DON_ST) || (IrqStatus & SV_SOF_INT_ST)) {
		/* cur_v_cnt = ((ISP_RD32(CAMSV_REG_TG_INTER_ST( */
		/* reg_module)) & 0x00FF0000) >> 16); */

		cur_v_cnt = ISP_RD32_TG_CAMX_FRM_CNT(module, reg_module);
	}
	if ((IrqStatus & SV_HW_PASS1_DON_ST) && (IrqStatus & SV_SOF_INT_ST)) {
		if (cur_v_cnt != sof_count[module]) {
			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_INF,
				       "isp sof_don block, %d_%d\n", cur_v_cnt,
				       sof_count[module]);
		}
	}

	if (IrqStatus & SV_HW_PASS1_DON_ST) {
		HwP1Done_cnt[module]++;
		if (HwP1Done_cnt[module] > 255)
			HwP1Done_cnt[module] -= 256;
	}

	spin_lock(&(IspInfo.SpinLockIrq[module]));
	if (IrqStatus & SV_SW_PASS1_DON_ST) {
		sec = cpu_clock(0);	  /* ns */
		do_div(sec, 1000);	   /* usec */
		usec = do_div(sec, 1000000); /* sec and usec */
		/* update pass1 done time stamp for eis user */
		/* (need match with the time stamp in image header) */

		IspInfo.IrqInfo.LastestSigTime_usec[module][10] =
			(unsigned int)(usec);

		IspInfo.IrqInfo.LastestSigTime_sec[module][10] =
			(unsigned int)(sec);

		if (IspInfo.DebugMask & ISP_DBG_CAMSV_LOG) {
			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"%s P1_DON_%d(0x%08x_0x%08x) stamp[0x%08x]\n",
				str,
				(sof_count[module]) ? (sof_count[module] - 1)
						    : (sof_count[module]),
				(unsigned int)(fbc_ctrl1[0].Raw),
				(unsigned int)(fbc_ctrl2[0].Raw), time_stamp);
		}

		/* for dbg log only */
		if (pstRTBuf[module]->ring_buf[_camsv_imgo_].active) {
			pstRTBuf[module]->ring_buf[_camsv_imgo_].img_cnt =
				sof_count[module];
		}
	}

	if (IrqStatus & SV_SOF_INT_ST) {
		time = ktime_get(); /* ns */
		sec = time;
		do_div(sec, 1000);	   /* usec */
		usec = do_div(sec, 1000000); /* sec and usec */

		if (IspInfo.DebugMask & ISP_DBG_CAMSV_LOG) {
			static unsigned int m_sec = 0, m_usec;

			if (g1stSof[module]) {
				m_sec = sec;
				m_usec = usec;
				gSTime[module].sec = sec;
				gSTime[module].usec = usec;
			}

			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"%s P1_SOF_%d_%d(0x%08x_0x%08x,0x%08x),int_us: %d, stamp[%d]\n",
				str, sof_count[module], cur_v_cnt,
				(unsigned int)(ISP_RD32(
					CAMSV_REG_FBC_IMGO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAMSV_REG_FBC_IMGO_CTL2(reg_module))),
				ISP_RD32(CAMSV_REG_IMGO_BASE_ADDR(reg_module)),
				(unsigned int)((sec * 1000000 + usec) -
					       (1000000 * m_sec + m_usec)),
				time_stamp);
			/* keep current time */
			m_sec = sec;
			m_usec = usec;

			/* dbg information only */
			if (cur_v_cnt !=
			    ((ISP_RD32(CAMSV_REG_TG_INTER_ST(reg_module)) &
			      0x00FF0000) >>
			     16)) {
				IRQ_LOG_KEEPER(
					module, m_CurrentPPB, _LOG_INF,
					"SW ISR right on next hw p1_done\n");
			}
		}

		/* update SOF time stamp for eis user */
		/* (need match with the time stamp in image header) */
		IspInfo.IrqInfo.LastestSigTime_usec[module][12] =
			(unsigned int)(sec);

		IspInfo.IrqInfo.LastestSigTime_sec[module][12] =
			(unsigned int)(usec);

		/* sw sof counter */
		sof_count[module]++;
		/* for match vsync cnt */
		if (sof_count[module] > 255)
			sof_count[module] -= 256;

		g1stSof[module] = MFALSE;
	}

	for (i = 0; i < IRQ_USER_NUM_MAX; i++) {
		/* 1. update interrupt status to all users */
		IspInfo.IrqInfo.Status[module][SIGNAL_INT][i] |= IrqStatus;

		/* 2. update signal time and passed by signal count */
		if (IspInfo.IrqInfo.MarkedFlag[module][SIGNAL_INT][i] &
		    IspInfo.IrqInfo.Mask[module][SIGNAL_INT]) {

			unsigned int cnt = 0, tmp = IrqStatus;

			while (tmp) {
				if (tmp & 0x1) {
					IspInfo.IrqInfo
						.LastestSigTime_usec[module]
								    [cnt] =
						(unsigned int)time_frmb.tv_nsec
						/ 1000;

					IspInfo.IrqInfo
						.LastestSigTime_sec[module]
								   [cnt] =
						(unsigned int)time_frmb.tv_sec;

					IspInfo.IrqInfo
						.PassedBySigCnt[module][cnt]
							       [i]++;
				}
				tmp = tmp >> 1;
				cnt++;
			}
		} else {
			/* no any interrupt is not marked and */
			/* in read mask in this irq type */
		}
	}
	spin_unlock(&(IspInfo.SpinLockIrq[module]));
	/*  */
	if (IrqStatus & SV_SOF_INT_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCamsv[ISP_GetWaitQCamsvIndex(module)]
					       [ISP_WAITQ_HEAD_IRQ_SV_SOF]);
	}
	if (IrqStatus & SV_SW_PASS1_DON_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCamsv[ISP_GetWaitQCamsvIndex(
				module)][ISP_WAITQ_HEAD_IRQ_SV_SW_P1_DONE]);
	}
	wake_up_interruptible(&IspInfo.WaitQueueHead[module]);

	/* dump log, use workq */
	if (IrqStatus & (SV_SOF_INT_ST | SV_SW_PASS1_DON_ST | SV_VS1_ST)) {
#if (ISP_BOTTOMHALF_WORKQ == 1)
		queue_work(isr_log_wq, &isp_workque[module].isp_bh_work);
#endif
	}

	return IRQ_HANDLED;
}

irqreturn_t ISP_Irq_CAM_A(int Irq, void *DeviceId)
{
	return ISP_Irq_CAM(ISP_IRQ_TYPE_INT_CAM_A_ST, ISP_CAM_A_IDX);
}

irqreturn_t ISP_Irq_CAM_B(int Irq, void *DeviceId)
{
	return ISP_Irq_CAM(ISP_IRQ_TYPE_INT_CAM_B_ST, ISP_CAM_B_IDX);
}

irqreturn_t ISP_Irq_CAM_C(int Irq, void *DeviceId)
{
	return ISP_Irq_CAM(ISP_IRQ_TYPE_INT_CAM_C_ST, ISP_CAM_C_IDX);
}

#if Lafi_WAM_CQ_ERR
void ISP_DumpDbgPort(unsigned int *reg_module_array,
	unsigned int reg_module_count)
{
	unsigned int dbgPort[16];
	unsigned int cq_chksum_base = 0;
	unsigned int i = 0x0, j = 0x0;
	unsigned int tmp_module = 0;

	for (i = 0; i < reg_module_count; i++) {
		tmp_module = reg_module_array[i];
		LOG_NOTICE("dump CAM%d\n", tmp_module);
		ISP_WR32(CAM_REG_DBG_SET(tmp_module), 0xF0);
		/*smi:CQI_R1 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x114);
		dbgPort[0] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*smi:CQI_R2 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x115);
		dbgPort[1] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*chksum:CQI_R1 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x480B);
		dbgPort[2] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*chksum:CQI_R1 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x490B);
		dbgPort[3] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*chksum:CQI_R1 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x4A0B);
		dbgPort[4] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*chksum:CQI_R1 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x4B0B);
		dbgPort[5] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*chksum:CQI_R2 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x4C0B);
		dbgPort[6] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*chksum:CQI_R2 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x4D0B);
		dbgPort[7] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*chksum:CQI_R2 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x4E0B);
		dbgPort[8] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*chksum:CQI_R2 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x4F0B);
		dbgPort[9] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*fifo:CQI_R1 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x20214);
		dbgPort[10] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		/*fifo:CQI_R2 */
		ISP_WR32(CAM_REG_DMA_DEBUG_SEL(tmp_module), 0x20215);
		dbgPort[11] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		LOG_NOTICE("dbgPort smi(CQI_R1,0x%08x)(CQI_R2,0x%08x)\n",
			   dbgPort[0], dbgPort[1]);
		LOG_NOTICE("chksum(CQI_R1,0x%08x,0x%08x,0x%08x,0x%08x)",
			   dbgPort[2], dbgPort[3], dbgPort[4], dbgPort[5]);
		LOG_NOTICE("(CQI_R2,0x%08x,0x%08x,0x%08x,0x%08x)\n",
			   dbgPort[6], dbgPort[7], dbgPort[8], dbgPort[9]);
		LOG_NOTICE("fifo(0x%08x,0x%08x)\n",
			   dbgPort[10], dbgPort[11]);

		ISP_WR32(CAM_REG_CAMCQ_CQ_EN(tmp_module),
			(ISP_RD32(CAM_REG_CAMCQ_CQ_EN(tmp_module)) &
			0xEFFFFFFF));
		ISP_WR32(CAM_REG_DBG_SET(tmp_module), 0x1B);
		/* CQ thread */
		dbgPort[12] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		ISP_WR32(CAM_REG_DBG_SET(tmp_module), 0x11B);
		/* CQ state */
		dbgPort[13] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		LOG_NOTICE("(CQ thread 0x%08x)(CQ state 0x%08x)",
				dbgPort[12], dbgPort[13]);

		for (j = 0; j < 16; j++) {
			cq_chksum_base = 0x1B + (j << 8);
			ISP_WR32(CAM_REG_CAMCQ_CQ_EN(tmp_module),
				 (ISP_RD32(CAM_REG_CAMCQ_CQ_EN(tmp_module)) |
				  0x10000000));
			/* CQ checksum */
			ISP_WR32(CAM_REG_DBG_SET(tmp_module), cq_chksum_base);
			dbgPort[j] = ISP_RD32(CAM_REG_DBG_PORT(tmp_module));
		}
		LOG_NOTICE("CQ chksum(0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,\n",
			   dbgPort[0], dbgPort[1], dbgPort[2], dbgPort[3],
			   dbgPort[4]);
		LOG_NOTICE("0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,\n", dbgPort[5],
			   dbgPort[6], dbgPort[7], dbgPort[8], dbgPort[9]);
		LOG_NOTICE("0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x)\n",
			   dbgPort[10], dbgPort[11], dbgPort[12], dbgPort[13],
			   dbgPort[14], dbgPort[15]);
	}
}

static void Set_CQ_immediate(unsigned int tmp_module)
{
	union CAMCQ_CQ_CTL_ cq_ctrl;

	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR0_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR0_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR1_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR1_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR9_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR9_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR10_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR10_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR11_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR11_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR12_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR12_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR13_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR13_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR14_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR14_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR16_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR16_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR17_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR17_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR20_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR20_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR24_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x1;
		ISP_WR32(CAM_REG_CQ_THR24_CTL(tmp_module), cq_ctrl.Raw);
	}

}

static void Set_CQ_continuous(unsigned int tmp_module)
{
	union CAMCQ_CQ_CTL_ cq_ctrl;

	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR0_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR0_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR1_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR1_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR9_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR9_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR10_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR10_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR11_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR11_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR12_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR12_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR13_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR13_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR14_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR14_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR16_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR16_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR17_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR17_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR20_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR20_CTL(tmp_module), cq_ctrl.Raw);
	}
	cq_ctrl.Raw =
		(unsigned int)ISP_RD32(CAM_REG_CQ_THR24_CTL(tmp_module));
	if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
		cq_ctrl.Bits.CAMCQ_CQ_MODE = 0x2;
		ISP_WR32(CAM_REG_CQ_THR24_CTL(tmp_module), cq_ctrl.Raw);
	}
}

int CQ_Recover_start(enum ISP_IRQ_TYPE_ENUM irq_module,
unsigned int *reg_module, unsigned int *reg_module_array,
unsigned int *reg_module_count)
{
	union CAMCTL_TWIN_STATUS_ twinStatus;
	unsigned int i = 0, tmp_module = 0, index = 0;

	switch (irq_module) {
	case ISP_IRQ_TYPE_INT_CAM_A_ST:
		*reg_module = ISP_CAM_A_IDX;
		break;
	case ISP_IRQ_TYPE_INT_CAM_B_ST:
		*reg_module = ISP_CAM_B_IDX;
		break;
	case ISP_IRQ_TYPE_INT_CAM_C_ST:
		*reg_module = ISP_CAM_C_IDX;
		break;
	default:
		LOG_NOTICE("Wrong IRQ module: %d",
			   (unsigned int)irq_module);
		break;
	}
	LOG_NOTICE("+CQ recover");
	reg_module_array[0] = *reg_module;
	twinStatus.Raw = ISP_RD32(CAM_REG_CTL_TWIN_STATUS(*reg_module));

	if (twinStatus.Bits.TWIN_EN == MTRUE) {
		if (((reg_module_array[0] == ISP_CAM_A_IDX) &&
			(twinStatus.Bits.MASTER_MODULE != CAM_A)) ||
			((reg_module_array[0] == ISP_CAM_B_IDX) &&
			(twinStatus.Bits.MASTER_MODULE != CAM_B)) ||
			((reg_module_array[0] == ISP_CAM_C_IDX) &&
			(twinStatus.Bits.MASTER_MODULE != CAM_C)))
			return -1;
		for (i = 0; i < twinStatus.Bits.SLAVE_CAM_NUM; i++) {
			if (i == 0) {
				switch (twinStatus.Bits.TWIN_MODULE) {
				case CAM_A:
				reg_module_array[1] = ISP_CAM_A_IDX;
				break;
				case CAM_B:
				reg_module_array[1] = ISP_CAM_B_IDX;
				break;
				case CAM_C:
				reg_module_array[1] = ISP_CAM_C_IDX;
					break;
				default:
				LOG_NOTICE(
				"twin module is invalid! recover fail");
				return -1;
				}
			} else if (i == 1) {
				switch (twinStatus.Bits.TRIPLE_MODULE) {
				case CAM_A:
				reg_module_array[2] = ISP_CAM_A_IDX;
				break;
				case CAM_B:
				reg_module_array[2] = ISP_CAM_B_IDX;
				break;
				case CAM_C:
				reg_module_array[2] = ISP_CAM_C_IDX;
				break;
				default:
				LOG_NOTICE(
				"twin module is invalid! recover fail");
				return -1;
				}
			}
		}
		*reg_module_count = twinStatus.Bits.SLAVE_CAM_NUM + 1;
	} else {
		*reg_module_count = 1;
	}

	/* 1. record current FBC value.
	 *    Change to ISP_RecordCQAddr @SOF because FBC may be empty buffer at CQ over vsync.
	 */

	/* 2. turn off TG viewFinder, CMOS */
	ISP_WR32(
		CAM_REG_TG_VF_CON(*reg_module),
		(ISP_RD32(CAM_REG_TG_VF_CON(*reg_module)) & 0xFFFFFFFE));
	ISP_WR32(CAM_REG_TG_SEN_MODE(*reg_module),
		 (ISP_RD32(CAM_REG_TG_SEN_MODE(*reg_module)) &
		  0xFFFFFFFE));
	LOG_NOTICE("disable viewfinder & cmos to do CQ recover");

	/* 3. disable double buffer and enable CQ done control*/
	for (i = 0; i < *reg_module_count; i++) {
		tmp_module = reg_module_array[i];
		ISP_WR32(CAM_REG_CTL_MISC(tmp_module),
			 (ISP_RD32(CAM_REG_CTL_MISC(tmp_module)) & 0xFFFFFFEF));
		LOG_NOTICE(
			"disable double buffer CAM%d to do CQ recover",
			tmp_module);
		ISP_WR32(CAM_REG_CTL_RAW_INT6_EN(tmp_module), 0xFFFFFFFF);
	}
	index = *reg_module - ISP_CAM_A_IDX;
	if (index > (ISP_CAM_C_IDX - ISP_CAM_A_IDX)) {
		LOG_NOTICE(
			"index is invalid! recover fail");
			return -1;
	}
	LOG_NOTICE("start HW recover due to CQ over Vsync ...\n");
	LOG_NOTICE("fbc:imgo:0x%x,rrzo:0x%x,ufeo:0x%x,ufgo:0x%x\n",
		   g_fbc_ctrl2[index][_imgo_].Raw,
		   g_fbc_ctrl2[index][_rrzo_].Raw,
		   g_fbc_ctrl2[index][_ufeo_].Raw,
		   g_fbc_ctrl2[index][_ufgo_].Raw);
	LOG_NOTICE("fbc:rsso:0x%x,lmvo:0x%x,lcso:0x%x,lcesho:0x%x\n",
		   g_fbc_ctrl2[index][_rsso_].Raw,
		   g_fbc_ctrl2[index][_lmvo_].Raw,
		   g_fbc_ctrl2[index][_lcso_].Raw,
		   g_fbc_ctrl2[index][_lcesho_].Raw);
	LOG_NOTICE("fbc:yuvo:0x%x,yuvbo:0x%x,yuvco:0x%x,crzo:0x%x\n",
		   g_fbc_ctrl2[index][_yuvo_].Raw,
		   g_fbc_ctrl2[index][_yuvbo_].Raw,
		   g_fbc_ctrl2[index][_yuvco_].Raw,
		   g_fbc_ctrl2[index][_crzo_].Raw);
	LOG_NOTICE("fbc:aao:0x%x,aaho:0x%x,afo:0x%x,flko:0x%x\n",
		   g_fbc_ctrl2[index][_aao_].Raw,
		   g_fbc_ctrl2[index][_aaho_].Raw,
		   g_fbc_ctrl2[index][_afo_].Raw,
		   g_fbc_ctrl2[index][_flko_].Raw);
	LOG_NOTICE("fbc:pdo:0x%x,tsfso:0x%x,ltmso:0x%x\n",
		   g_fbc_ctrl2[index][_pdo_].Raw,
		   g_fbc_ctrl2[index][_tsfso_].Raw,
		   g_fbc_ctrl2[index][_ltmso_].Raw);
	return 0;
}

int CQ_Recover_middle(unsigned int reg_module,
unsigned int *reg_module_array, unsigned int reg_module_count)
{
	union CAMCQ_CQ_CTL_ cq_ctrl;
	union CAMCTL_START_ en_ctlStart;
	union CAMCTL_INT6_STATUS_ DmaStatus6;
	unsigned int  DmaEnStatus[ISP_CAM_C_IDX-ISP_CAM_A_IDX+1][_cam_max_];
	unsigned long long  sec = 0, usec = 0, m_sec = 0, m_usec = 0;
	unsigned long long  timeoutMs = 500;/*0.5ms*/
	unsigned int i = 0, tmp_module = 0, index = 0;
	unsigned int cq_done = 0;
	int reset_count = 1;
	bool ret = MTRUE;

	do {
		/* Start Reset Flow */
		ret = MTRUE;
		g_cqDoneStatus[0] = 0;
		g_cqDoneStatus[1] = 0;
		g_cqDoneStatus[2] = 0;
		/* 4. HW reset & SW reset including master and slave cam*/
		for (i = 0; i < reg_module_count; i++) {
			tmp_module = reg_module_array[i];
			ISP_WR32(
			CAM_REG_CTL_SW_CTL(tmp_module), 0x0);
			ISP_WR32(
			CAM_REG_CTL_SW_CTL(tmp_module), 0x1); /*SW_RST_TRIG*/

			mdelay(1); /* Wait reset done */
			ISP_WR32(
			CAM_REG_CTL_SW_CTL(tmp_module), 0x4); /*HW_RST*/
			ISP_WR32(
			CAM_REG_CTL_SW_CTL(tmp_module), 0x0);
		}

		/* 5. restore FBC setting, only STT port*/

		for (i = 0; i < reg_module_count; i++) {
			tmp_module = reg_module_array[i];
			index = tmp_module - ISP_CAM_A_IDX;

			if (index > (ISP_CAM_C_IDX - ISP_CAM_A_IDX)) {
				LOG_NOTICE(
					"index is invalid! recover fail");
					return -1;
			}
			ISP_GetDmaPortsStatus(tmp_module,
				&DmaEnStatus[index][0]);
			if (DmaEnStatus[index][_aao_])
				ISP_WR32(CAM_REG_FBC_AAO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_aao_].Raw);

			if (DmaEnStatus[index][_aaho_])
				ISP_WR32(CAM_REG_FBC_AAHO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_aaho_].Raw);

			if (DmaEnStatus[index][_afo_])
				ISP_WR32(CAM_REG_FBC_AFO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_afo_].Raw);

			if (DmaEnStatus[index][_flko_])
				ISP_WR32(CAM_REG_FBC_FLKO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_flko_].Raw);

			if (DmaEnStatus[index][_pdo_])
				ISP_WR32(CAM_REG_FBC_PDO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_pdo_].Raw);
#if 0
			if (DmaEnStatus[index][_imgo_])
				ISP_WR32(CAM_REG_FBC_IMGO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_imgo_].Raw);

			if (DmaEnStatus[index][_rrzo_])
				ISP_WR32(CAM_REG_FBC_RRZO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_rrzo_].Raw);

			if (DmaEnStatus[index][_ufeo_])
				ISP_WR32(CAM_REG_FBC_UFEO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_ufeo_].Raw);

			if (DmaEnStatus[index][_ufgo_])
				ISP_WR32(CAM_REG_FBC_UFGO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_ufgo_].Raw);

			if (DmaEnStatus[index][_rsso_])
				ISP_WR32(CAM_REG_FBC_RSSO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_rsso_].Raw);

			if (DmaEnStatus[index][_lmvo_])
				ISP_WR32(CAM_REG_FBC_LMVO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_lmvo_].Raw);

			if (DmaEnStatus[index][_lcso_])
				ISP_WR32(CAM_REG_FBC_LCESO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_lcso_].Raw);

			if (DmaEnStatus[index][_lcesho_])
				ISP_WR32(CAM_REG_FBC_LCESHO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_lcesho_].Raw);
#endif
			if (DmaEnStatus[index][_ltmso_])
				ISP_WR32(CAM_REG_FBC_LTMSO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_ltmso_].Raw);

			if (DmaEnStatus[index][_tsfso_])
				ISP_WR32(CAM_REG_FBC_TSFSO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_tsfso_].Raw);
#if 0
			if (DmaEnStatus[index][_yuvo_])
				ISP_WR32(CAM_REG_FBC_YUVO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_yuvo_].Raw);

			if (DmaEnStatus[index][_yuvbo_])
				ISP_WR32(CAM_REG_FBC_YUVBO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_yuvbo_].Raw);

			if (DmaEnStatus[index][_yuvco_])
				ISP_WR32(CAM_REG_FBC_YUVCO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_yuvco_].Raw);

			if (DmaEnStatus[index][_crzo_])
				ISP_WR32(CAM_REG_FBC_CRZO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_crzo_].Raw);

			if (DmaEnStatus[index][_crzbo_])
				ISP_WR32(CAM_REG_FBC_CRZBO_CTL2(tmp_module),
					g_fbc_ctrl2[index][_crzbo_].Raw);

			if (DmaEnStatus[index][_crzo_r2_])
				ISP_WR32(CAM_REG_FBC_CRZO_R2_CTL2(tmp_module),
					g_fbc_ctrl2[index][_crzo_r2_].Raw);

			if (DmaEnStatus[index][_rsso_r2_])
				ISP_WR32(CAM_REG_FBC_RSSO_R2_CTL2(tmp_module),
					g_fbc_ctrl2[index][_rsso_r2_].Raw);
#endif
		}

		/* 6. restore CQ base address */
		for (i = 0; i < reg_module_count; i++) {
			tmp_module = reg_module_array[i];
			index = tmp_module - ISP_CAM_A_IDX;

			if (index > (ISP_CAM_C_IDX - ISP_CAM_A_IDX)) {
				LOG_NOTICE(
					"index is invalid! recover fail");
					return -1;
			}
			cq_ctrl.Raw = (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR0_CTL(tmp_module));
			if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
				ISP_WR32(CAM_REG_CQ_THR0_BASEADDR(tmp_module),
					g_cqBaseAddr[index][0]);
				LOG_NOTICE("[%d]CQ0 base: 0x2%x(0x2%x)\n",
					index, g_cqBaseAddr[index][0],
					(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(tmp_module)));
				if ((g_cqBaseAddr[index][0] -
					(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(tmp_module)))
					!= 0) {
					ISP_WR32(CAM_REG_CQ_THR0_BASEADDR(
						tmp_module),
						g_cqBaseAddr[index][0]);
					LOG_NOTICE("restore CQ0 again\n");

				}
			}
		//CQ1
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR1_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR1_BASEADDR(tmp_module),
				g_cqBaseAddr[index][1]);
			LOG_NOTICE("[%d]CQ1 base:0x2%x(0x2%x)\n",
				index, g_cqBaseAddr[index][1],
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR1_BASEADDR(tmp_module)));
		}
		//CQ9(TSFSO)
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR9_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR9_BASEADDR(tmp_module),
				g_cqBaseAddr[index][9]);
			LOG_NOTICE("[%d]CQ9 base:0x2%x(0x2%x)\n", index,
				g_cqBaseAddr[index][9],
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR9_BASEADDR(tmp_module)));
			if ((g_cqBaseAddr[index][9] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR9_BASEADDR(tmp_module))) != 0) {
				ISP_WR32(CAM_REG_CQ_THR9_BASEADDR(tmp_module),
					g_cqBaseAddr[index][9]);
				LOG_NOTICE("restore CQ9 again\n");
			}
		}
		//CQ10(AAO)
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR10_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR10_BASEADDR(tmp_module),
				g_cqBaseAddr[index][10]);
			LOG_NOTICE("[%d]CQ10 base:0x2%x(0x2%x)\n",
				index, g_cqBaseAddr[index][10],
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR10_BASEADDR(tmp_module)));
			if ((g_cqBaseAddr[index][10] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR10_BASEADDR(tmp_module))) != 0) {
				ISP_WR32(CAM_REG_CQ_THR10_BASEADDR(tmp_module),
					g_cqBaseAddr[index][10]);
				LOG_NOTICE("restore CQ10 again\n");
			}
		}
		//CQ11(AAHO)
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR11_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR11_BASEADDR(tmp_module),
				g_cqBaseAddr[index][11]);
			LOG_NOTICE("[%d]CQ11 base:0x2%x(0x2%x)\n",
				index, g_cqBaseAddr[index][11],
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR11_BASEADDR(tmp_module)));
			if ((g_cqBaseAddr[index][11] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR11_BASEADDR(tmp_module))) != 0) {
				ISP_WR32(CAM_REG_CQ_THR11_BASEADDR(tmp_module),
					g_cqBaseAddr[index][11]);
				LOG_NOTICE("restore CQ11 again\n");
			}
		}
		//CQ12(AFO)
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR12_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR12_BASEADDR(tmp_module),
				g_cqBaseAddr[index][12]);
			LOG_NOTICE("[%d]CQ12 base:0x2%x(0x2%x)\n",
				index, g_cqBaseAddr[index][12],
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR12_BASEADDR(tmp_module)));
			if ((g_cqBaseAddr[index][12] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR12_BASEADDR(tmp_module))) != 0) {
				ISP_WR32(CAM_REG_CQ_THR12_BASEADDR(tmp_module),
					g_cqBaseAddr[index][12]);
				LOG_NOTICE("restore CQ12 again\n");
			}
		}
		//CQ13(PDO)
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR13_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR13_BASEADDR(tmp_module),
				g_cqBaseAddr[index][13]);
			LOG_NOTICE("[%d]CQ13 base:0x2%x(0x2%x)\n",
				index, g_cqBaseAddr[index][13],
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR13_BASEADDR(tmp_module)));
			if ((g_cqBaseAddr[index][13] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR13_BASEADDR(tmp_module))) != 0) {
				ISP_WR32(CAM_REG_CQ_THR13_BASEADDR(tmp_module),
					g_cqBaseAddr[index][13]);
				LOG_NOTICE("restore CQ13 again\n");
			}
		}
		//CQ14(FLKO)
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR14_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR14_BASEADDR(tmp_module),
				g_cqBaseAddr[index][14]);
			LOG_NOTICE("[%d]CQ14 base:0x2%x(0x2%x)\n",
				index, g_cqBaseAddr[index][14],
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR14_BASEADDR(tmp_module)));
			if ((g_cqBaseAddr[index][14] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR14_BASEADDR(tmp_module))) != 0) {
				ISP_WR32(CAM_REG_CQ_THR14_BASEADDR(tmp_module),
					g_cqBaseAddr[index][14]);
				LOG_NOTICE("restore CQ14 again\n");
			}
		}
		//CQ16
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR16_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR16_BASEADDR(tmp_module),
				g_cqBaseAddr[index][16]);
			LOG_NOTICE("[%d]CQ16 base:0x%x\n",
					index, g_cqBaseAddr[index][16]);
		}
		//CQ17(LTMSO)
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR17_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR17_BASEADDR(tmp_module),
				g_cqBaseAddr[index][17]);
			LOG_NOTICE("[%d]CQ17 base:0x2%x(0x2%x)\n",
				index, g_cqBaseAddr[index][17],
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR17_BASEADDR(tmp_module)));
			if ((g_cqBaseAddr[index][17] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR17_BASEADDR(tmp_module))) != 0) {
				ISP_WR32(CAM_REG_CQ_THR17_BASEADDR(tmp_module),
					g_cqBaseAddr[index][17]);
				LOG_NOTICE("restore CQ17 again\n");
			}
		}
		//CQ20
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR20_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR20_BASEADDR(tmp_module),
				g_cqBaseAddr[index][20]);
			LOG_NOTICE("CQ20 base:0x%x\n", g_cqBaseAddr[index][20]);
		}
		//CQ24
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR24_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			ISP_WR32(CAM_REG_CQ_THR24_BASEADDR(tmp_module),
				g_cqBaseAddr[index][24]);
			LOG_NOTICE("CQ24 base:0x%x\n", g_cqBaseAddr[index][24]);
		}

		/* 7. set CQ immediate mode */
		Set_CQ_immediate(tmp_module);

		/* 8. CQ immediate trigger */
		timeoutMs = 500; /* 0.5ms */

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR0_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR0_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR0_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR0_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec  - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ0 timeout0x%x,0x%x\n",
				(unsigned int)ISP_RD32(
				CAM_REG_CTL_START_ST(tmp_module)), cq_done);
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR0_DONE_ST;
			}
			LOG_NOTICE("wait CQ0 start 0x%x, base 0x%x\n",
			(unsigned int)ISP_RD32(CAM_REG_CTL_START_ST(
			tmp_module)),
			(unsigned int)ISP_RD32(CAM_REG_CQ_THR0_BASEADDR(
			tmp_module)));
			if ((g_cqBaseAddr[index][0] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR0_BASEADDR(tmp_module))) == 0)
				ret = MFALSE;
		}

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR1_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR1_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR1_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR1_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec  - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ1 timeout\n");
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR1_DONE_ST;
			}
		}

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR9_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR9_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR9_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR9_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec  - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ9 timeout0x%x,0x%x\n",
					(unsigned int)ISP_RD32(
					CAM_REG_CTL_START_ST(tmp_module)),
					cq_done);
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR9_DONE_ST;
			}
			LOG_NOTICE("wait CQ9 start 0x%x, base 0x%x\n",
			(unsigned int)ISP_RD32(CAM_REG_CTL_START_ST(
			tmp_module)),
			(unsigned int)ISP_RD32(CAM_REG_CQ_THR9_BASEADDR(
			tmp_module)));
			if ((g_cqBaseAddr[index][9] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR9_BASEADDR(tmp_module))) == 0)
				ret = MFALSE;
		}

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR10_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
				en_ctlStart.Bits.CQ_THR10_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR10_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR10_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ10 timeout\n");
					ret = MFALSE;
					break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR10_DONE_ST;
			}
		}

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR11_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
				en_ctlStart.Bits.CQ_THR11_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR11_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR11_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ11 timeout\n");
					ret = MFALSE;
					break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR11_DONE_ST;
			}
		}

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR12_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR12_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR12_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR12_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ12 timeout0x%x, 0x%x\n",
					(unsigned int)ISP_RD32(
					CAM_REG_CTL_START_ST(tmp_module)),
					cq_done);
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR12_DONE_ST;
			}
			LOG_NOTICE("wait CQ12 start 0x%x, base 0x%x\n",
			(unsigned int)ISP_RD32(CAM_REG_CTL_START_ST(
			tmp_module)),
			(unsigned int)ISP_RD32(CAM_REG_CQ_THR12_BASEADDR(
			tmp_module)));
			if ((g_cqBaseAddr[index][12] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR12_BASEADDR(tmp_module))) == 0)
				ret = MFALSE;
		}

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR13_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR13_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR13_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR13_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ13 timeout0x%x, 0x%x\n",
					(unsigned int)ISP_RD32(
					CAM_REG_CTL_START_ST(tmp_module)),
					cq_done);
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR13_DONE_ST;
			}
			LOG_NOTICE("wait CQ13 start 0x%x, base 0x%x\n",
			(unsigned int)ISP_RD32(CAM_REG_CTL_START_ST(
			tmp_module)),
			(unsigned int)ISP_RD32(CAM_REG_CQ_THR13_BASEADDR(
			tmp_module)));
			if ((g_cqBaseAddr[index][13] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR13_BASEADDR(tmp_module))) == 0)
				ret = MFALSE;
		}

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR14_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR14_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR14_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR14_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ14 timeout0x%x, 0x%x\n",
					(unsigned int)ISP_RD32(
					CAM_REG_CTL_START_ST(tmp_module)),
					cq_done);
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR14_DONE_ST;
			}
			LOG_NOTICE("wait CQ14 start 0x%x, base 0x%x\n",
			(unsigned int)ISP_RD32(CAM_REG_CTL_START_ST(
			tmp_module)),
			(unsigned int)ISP_RD32(CAM_REG_CQ_THR14_BASEADDR(
			tmp_module)));
			if ((g_cqBaseAddr[index][14] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR14_BASEADDR(tmp_module))) == 0)
				ret = MFALSE;
		}

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR16_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR16_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR16_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR16_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ16 timeout0x%x, 0x%x\n",
					(unsigned int)ISP_RD32(
					CAM_REG_CTL_START_ST(tmp_module)),
					cq_done);
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR16_DONE_ST;
			}
			LOG_NOTICE("wait CQ16 start 0x%x, base 0x%x\n",
			(unsigned int)ISP_RD32(CAM_REG_CTL_START_ST(
			tmp_module)),
			(unsigned int)ISP_RD32(CAM_REG_CQ_THR16_BASEADDR(
			tmp_module)));
			if ((g_cqBaseAddr[index][16] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR16_BASEADDR(tmp_module))) == 0)
				ret = MFALSE;
		}

		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR17_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR17_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR17_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR17_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ17 timeout0x%x, 0x%x\n",
					(unsigned int)ISP_RD32(
					CAM_REG_CTL_START_ST(tmp_module)),
					cq_done);
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR17_DONE_ST;
			}
			LOG_NOTICE("wait CQ17 start 0x%x, base 0x%x\n",
			(unsigned int)ISP_RD32(CAM_REG_CTL_START_ST(
			tmp_module)),
			(unsigned int)ISP_RD32(CAM_REG_CQ_THR17_BASEADDR(
			tmp_module)));
			if ((g_cqBaseAddr[index][17] - (unsigned int)ISP_RD32(
				CAM_REG_CQ_THR17_BASEADDR(tmp_module))) == 0)
				ret = MFALSE;
		}
#if 0
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR20_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR20_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR20_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR20_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ20 timeout0x%x, 0x%x\n",
					(unsigned int)ISP_RD32(
					CAM_REG_CTL_START_ST(tmp_module)),
					cq_done);
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR20_DONE_ST;
			}
		}
#endif
		cq_ctrl.Raw = (unsigned int)ISP_RD32(CAM_REG_CQ_THR24_CTL(
			tmp_module));
		if (cq_ctrl.Bits.CAMCQ_CQ_EN == 0x1) {
			en_ctlStart.Raw = 0x0;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);
			en_ctlStart.Bits.CQ_THR24_START = 0x1;
			ISP_WR32(CAM_REG_CTL_START(tmp_module),
				en_ctlStart.Raw);

			DmaStatus6.Raw = g_cqDoneStatus[index];
			cq_done = DmaStatus6.Bits.CQ_THR24_DONE_ST;
			m_sec = ktime_get(); /* ns */
			do_div(m_sec, 1000); /* usec */
			m_usec = do_div(m_sec, 1000000);/* sec and usec */
			/* wait CQ loading done */
			while ((ISP_RD32(CAM_REG_CTL_START_ST(tmp_module)) &
			en_ctlStart.Bits.CQ_THR24_START) || (cq_done == 0)) {
				sec = ktime_get(); /* ns */
				do_div(sec, 1000); /* usec */
				usec = do_div(sec, 1000000);/* sec and usec */
			if ((usec - m_usec) > timeoutMs) {
				LOG_NOTICE("wait CQ24 timeout0x%x, 0x%x\n",
					(unsigned int)ISP_RD32(
					CAM_REG_CTL_START_ST(tmp_module)),
					cq_done);
				ret = MFALSE;
				break;
			}
				DmaStatus6.Raw = g_cqDoneStatus[index];
				cq_done |= DmaStatus6.Bits.CQ_THR24_DONE_ST;
			}
		}
		}
		reset_count--;
	} while ((ret == MFALSE) && (reset_count > 0));

	return 0;
}

int CQ_Recover_final(unsigned int reg_module,
unsigned int *reg_module_array, unsigned int reg_module_count)
{
	unsigned int i = 0;
	unsigned int tmp_module = ISP_CAM_A_IDX;

	/* 9. set CQ continuous mode */
	/* 10. disable CQ done control and enable double buffer */
	for (i = 0; i < reg_module_count; i++) {
		tmp_module = reg_module_array[i];
		Set_CQ_continuous(tmp_module);
		ISP_WR32(CAM_REG_CTL_RAW_INT6_EN(tmp_module), 0x0);
		ISP_WR32(CAM_REG_CTL_MISC(tmp_module),
			 (ISP_RD32(CAM_REG_CTL_MISC(tmp_module)) | 0x10));
		LOG_NOTICE("en double buf CAM%d for CQ recover", tmp_module);
	}
	/* 11. enable TG CMOS & viewFinder */
	ISP_WR32(CAM_REG_TG_SEN_MODE(reg_module),
		 (ISP_RD32(CAM_REG_TG_SEN_MODE(reg_module)) | 0x1));
	ISP_WR32(CAM_REG_TG_VF_CON(reg_module),
		 (ISP_RD32(CAM_REG_TG_VF_CON(reg_module)) | 0x1));
	LOG_NOTICE(
		"turn on TG VF, CMOS to do CQ recover 0x%x, 0x%x",
		(unsigned int)ISP_RD32(CAM_REG_TG_SEN_MODE(reg_module)),
		(unsigned int)ISP_RD32(CAM_REG_TG_VF_CON(reg_module)));

	return 0;
}
#endif
irqreturn_t ISP_Irq_CAM(
	enum ISP_IRQ_TYPE_ENUM irq_module, enum ISP_DEV_NODE_ENUM cam_idx)
{
	unsigned int module = irq_module;
	unsigned int reg_module = ISP_CAM_A_IDX;
	unsigned int inner_reg_module = ISP_CAM_A_INNER_IDX;
	unsigned int i, cardinalNum = 0, IrqStatus, ErrStatus, WarnStatus;
	unsigned int DmaStatus, WarnStatus_2 = 0, cur_v_cnt = 0;
	unsigned int Dma2Status = 0, DropStatus = 0;
	unsigned int IrqStatusX = 0, Irq2StatusX = 0, Irq3StatusX;
	unsigned int Irq4StatusX, Irq5StatusX;
	unsigned int cqDoneIndex = 0;
	unsigned int dma_en = 0, dma2_en = 0;
	unsigned int int_en = 0, int3_en = 0, int4_en = 0;
	union FBC_CTRL_1 fbc_ctrl1[2];
	union FBC_CTRL_2 fbc_ctrl2[2];

	struct timespec64 time_frmb;
	unsigned long long sec = 0;
	unsigned long usec = 0;
	static unsigned int sec_sof[ISP_IRQ_TYPE_INT_CAMSV_START_ST] = {0};
	static unsigned int usec_sof[ISP_IRQ_TYPE_INT_CAMSV_START_ST] = {0};
	ktime_t time;
	unsigned int IrqEnableOrig, IrqEnableNew;
	union CAMCTL_TWIN_STATUS_ twinStatus;
	unsigned int isStagger = 0, FBC_CurState = 0;

	/* Avoid touch hwmodule when clock is disable. */
	/* DEVAPC will moniter this kind of err */
	if (G_u4EnableClockCount[cam_idx] == 0)
		return IRQ_HANDLED;

	ktime_get_ts64(&time_frmb);
	usec = time_frmb.tv_nsec / 1000;
	sec = time_frmb.tv_sec;

#if (ISP_BOTTOMHALF_WORKQ == 1)
	gSvLog[module]._lastIrqTime.sec = sec;
	gSvLog[module]._lastIrqTime.usec = usec;
#endif

	switch (irq_module) {
	case ISP_IRQ_TYPE_INT_CAM_A_ST:
		reg_module = ISP_CAM_A_IDX;
		inner_reg_module = ISP_CAM_A_INNER_IDX;
		cardinalNum = 0;
		break;
	case ISP_IRQ_TYPE_INT_CAM_B_ST:
		reg_module = ISP_CAM_B_IDX;
		inner_reg_module = ISP_CAM_B_INNER_IDX;
		cardinalNum = 1;
		break;
	case ISP_IRQ_TYPE_INT_CAM_C_ST:
		reg_module = ISP_CAM_C_IDX;
		inner_reg_module = ISP_CAM_C_INNER_IDX;
		cardinalNum = 2;
		break;
	default:
		LOG_NOTICE("Wrong IRQ module: %d", (unsigned int)module);
		return IRQ_HANDLED;
	}

	isStagger = ((ISP_RD32(CAM_REG_TG_SEN_MODE(reg_module)) &
				0x00400000)) >> 22;

	spin_lock(&(IspInfo.SpinLockIrq[module]));
	IrqStatus = ISP_RD32(CAM_REG_CTL_RAW_INT_STATUS(reg_module));
	DmaStatus = ISP_RD32(CAM_REG_CTL_RAW_INT2_STATUS(reg_module));
	cqDoneIndex = reg_module - ISP_CAM_A_IDX;
	if (cqDoneIndex <= (ISP_CAM_C_IDX - ISP_CAM_A_IDX)) {
		g_cqDoneStatus[cqDoneIndex] =
			ISP_RD32(CAM_REG_CTL_RAW_INT6_STATUS(reg_module));
	}
	int_en = IrqStatus;
	int3_en = ISP_RD32(CAM_REG_CTL_RAW_INT3_EN(reg_module));
	int4_en = ISP_RD32(CAM_REG_CTL_RAW_INT4_EN(reg_module));

	Dma2Status = ISP_RD32(CAM_REG_CTL_RAW_INT3_STATUS(reg_module));
	DropStatus = ISP_RD32(CAM_REG_CTL_RAW_INT4_STATUS(reg_module));
	WarnStatus = ISP_RD32(CAM_REG_CTL_RAW_INT5_STATUS(reg_module));

	spin_unlock(&(IspInfo.SpinLockIrq[module]));

	ErrStatus = IrqStatus & IspInfo.IrqInfo.ErrMask[module][SIGNAL_INT];
#if Lafi_WAM_CQ_ERR
	if (((IrqStatus & SOF_INT_ST) == 0) && (IrqStatus & VS_INT_ST)
				&& (cq_recovery[module] == 1)) {
		if ((ISP_RD32(CAMX_REG_TG_VF_CON(reg_module)) == 0x1) &&
		    (g1stSof[module] == MFALSE)) {
			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "cq over vync\n");
			ErrStatus |= CQ_VS_ERR_ST;
		}
	}

	twinStatus.Raw = ISP_RD32(CAM_REG_CTL_TWIN_STATUS(reg_module));

	if ((IrqStatus & SOF_INT_ST) &&
		(((ISP_RD32(CAM_REG_TG_SEN_MODE(reg_module)) &
				0x00040000) >> 18) == 0x1) &&
			(twinStatus.Bits.TWIN_EN == MTRUE) &&
				(cq_recovery[module] == 1)) {
		if ((ISP_RD32(CAM_REG_RRZO_FH_SPARE_2(reg_module))
			!= pre_magic_num) && (CAM_FST_LAST_WORKING_FRAME ==
				Irq_CAM_FrameStatus(reg_module, module, 0, &FBC_CurState))) {
			if (g_cq0NextBA[CAM_A][0] != 0 &&
				twinStatus.Bits.SLAVE_CAM_NUM == 0x1) {
				if (ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
					reg_module)) !=
					g_cq0NextBA[CAM_A][0] ||
					ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
					ISP_CAM_B_IDX)) !=
					g_cq0NextBA[CAM_B][0]) {
					IRQ_LOG_KEEPER(module,
					m_CurrentPPB, _LOG_ERR,
					"[Twin]CQ Pages wrong...CQ Recovery master(0x%08x_0x%08x) slave(0x%08x_0x%08x)\n",
					ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
					reg_module)),
					g_cq0NextBA[CAM_A][0],
					ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
					ISP_CAM_B_IDX)),
					g_cq0NextBA[CAM_B][0]);
					ErrStatus |= CQ_VS_ERR_ST;
				}
			} else if (g_cq0NextBA[CAM_A][0] != 0
					&& twinStatus.Bits.SLAVE_CAM_NUM
								== 0x2) {
				if ((ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
					reg_module)) !=
					g_cq0NextBA[CAM_A][0]) ||
					(ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
					ISP_CAM_B_IDX)) !=
					g_cq0NextBA[CAM_B][0]) ||
					(ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
					ISP_CAM_C_IDX)) !=
					g_cq0NextBA[CAM_C][0])) {
					IRQ_LOG_KEEPER(module,
					m_CurrentPPB, _LOG_ERR,
						"[Triple]CQ Pages wrong...CQ Recovery\n");
					ErrStatus |= CQ_VS_ERR_ST;
				}
			}
		}
	}
#endif
	WarnStatus_2 =
		IrqStatus & IspInfo.IrqInfo.Warn2Mask[module][SIGNAL_INT];

	IrqStatus = IrqStatus & IspInfo.IrqInfo.Mask[module][SIGNAL_INT];

	/* Check ERR/WRN ISR times, if it occur too frequently,
	 * mark it for avoding keep enter ISR
	 * It will happen KE
	 */
	for (i = 0; i < ISP_ISR_MAX_NUM; i++) {
		/* Only check irq that un marked yet */
		if (!((IspInfo.IrqCntInfo.m_err_int_mark[module] & (1 << i)) ||
		      (IspInfo.IrqCntInfo.m_warn_int_mark[module] &
		       (1 << i)))) {

			if (ErrStatus & (1 << i))
				IspInfo.IrqCntInfo.m_err_int_cnt[module][i]++;

			if (WarnStatus & (1 << i))
				IspInfo.IrqCntInfo.m_warn_int_cnt[module][i]++;

			if (usec - IspInfo.IrqCntInfo.m_int_usec[module] <
			    INT_ERR_WARN_TIMER_THREAS) {
				if (IspInfo.IrqCntInfo
					    .m_err_int_cnt[module][i] >=
				    INT_ERR_WARN_MAX_TIME) {
					IspInfo.IrqCntInfo
						.m_err_int_mark[module] |=
						(1 << i);
				}
				if (IspInfo.IrqCntInfo
					    .m_warn_int_cnt[module][i] >=
				    INT_ERR_WARN_MAX_TIME) {
					IspInfo.IrqCntInfo
						.m_warn_int_mark[module] |=
						(1 << i);
				}
			} else {
				IspInfo.IrqCntInfo.m_int_usec[module] = usec;
				IspInfo.IrqCntInfo.m_err_int_cnt[module][i] = 0;

				IspInfo.IrqCntInfo.m_warn_int_cnt[module][i] =
					0;
			}
		}
	}

	spin_lock(&(IspInfo.SpinLockIrq[module]));
	IrqEnableOrig = ISP_RD32(CAM_REG_CTL_RAW_INT_EN(reg_module));
	spin_unlock(&(IspInfo.SpinLockIrq[module]));

	IrqEnableNew = IrqEnableOrig &
		       ~(IspInfo.IrqCntInfo.m_err_int_mark[module] |
			 IspInfo.IrqCntInfo.m_warn_int_mark[module]);

	ISP_WR32(CAM_REG_CTL_RAW_INT_EN(reg_module), IrqEnableNew);

	/*      */
	IRQ_INT_ERR_CHECK_CAM(WarnStatus, ErrStatus, WarnStatus_2, module);

	fbc_ctrl1[0].Raw = ISP_RD32(CAM_REG_FBC_IMGO_CTL1(reg_module));
	fbc_ctrl1[1].Raw = ISP_RD32(CAM_REG_FBC_RRZO_CTL1(reg_module));
	fbc_ctrl2[0].Raw = ISP_RD32(CAM_REG_FBC_IMGO_CTL2(reg_module));
	fbc_ctrl2[1].Raw = ISP_RD32(CAM_REG_FBC_RRZO_CTL2(reg_module));

	if (sec_on) {
		dma_en = lock_reg.CAM_REG_CTL_DMA_EN[reg_module];
		dma2_en = lock_reg.CAM_REG_CTL_DMA2_EN[reg_module];
	} else {
		dma_en = ISP_RD32(CAM_REG_CTL_DMA_EN(reg_module));
		dma2_en = ISP_RD32(CAM_REG_CTL_DMA2_EN(reg_module));
	}

#if defined(ISP_MET_READY)
	if (trace_ISP__Pass1_CAM_enter_enabled()) {
		/*MET:ISP EOF */
		if (IrqStatus & HW_PASS1_DON_ST)
			CAMSYS_MET_Events_Trace(0, reg_module, irq_module);

		if (IrqStatus & SOF_INT_ST)
			CAMSYS_MET_Events_Trace(1, reg_module, irq_module);
	}
#endif

	/* sof , done order chech . */
	if ((IrqStatus & HW_PASS1_DON_ST) || (IrqStatus & SOF_INT_ST))
		/*cur_v_cnt = ((ISP_RD32(CAM_REG_TG_INTER_ST(reg_module)) & */
		/*      0x00FF0000) >> 16); */

		cur_v_cnt = ISP_RD32_TG_CAMX_FRM_CNT(module, reg_module);

	if ((IrqStatus & HW_PASS1_DON_ST) && (IrqStatus & SOF_INT_ST)) {
		if (cur_v_cnt != sof_count[module]) {
			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_INF,
				       "isp sof_don block, %d_%d\n", cur_v_cnt,
				       sof_count[module]);
		}
	}

	if (IrqStatus & HW_PASS1_DON_ST) {
		HwP1Done_cnt[module]++;
		if (HwP1Done_cnt[module] > 255)
			HwP1Done_cnt[module] -= 256;

	/* these INT should read once per frame, correct timing is p1 done */
	spin_lock(&(IspInfo.SpinLockIrq[module]));
	IrqStatusX = ISP_RD32(CAM_REG_CTL_RAW_INT_STATUSX(inner_reg_module));
	Irq2StatusX = ISP_RD32(CAM_REG_CTL_RAW_INT2_STATUSX(inner_reg_module));
	Irq3StatusX = ISP_RD32(CAM_REG_CTL_RAW_INT3_STATUSX(inner_reg_module));
	Irq4StatusX = ISP_RD32(CAM_REG_CTL_RAW_INT4_STATUSX(inner_reg_module));
	Irq5StatusX = ISP_RD32(CAM_REG_CTL_RAW_INT5_STATUSX(inner_reg_module));
	spin_unlock(&(IspInfo.SpinLockIrq[module]));
#if 0
		if (dma2_en & _RAWI_R2_EN_) {
			IRQ_LOG_KEEPER(
			       module, m_CurrentPPB, _LOG_INF,
			       "module:%d, int_st=0x%x,int2_st=0x%x,int3_st=0x%x,int4_st=0x%x,int5_st=0x%x,DmaEn=0x%x,Dma2En=0x%x,int_stx=0x%x,int2_stx=0x%x,int3_stx=0x%x,int4_stx=0x%x,int5_stx=0x%x\n",
			       module, int_en, DmaStatus, Dma2Status,
			       DropStatus, WarnStatus, dma_en, dma2_en,
			       IrqStatusX, Irq2StatusX,
			       Irq3StatusX, Irq4StatusX, Irq5StatusX);

			ISP_DumpDebugData(module, RAWI_R2_DEBUG);
		}
#endif
	}

	if ((IrqStatus & HW_PASS1_DON_ST) && HW_DON_LOG_ENABLE) {
		if (FrameStatus[module] == CAM_FST_DROP_FRAME) {
			/* reduce SMVR case hw p1 done log */
			if (Irq2StatusX != 0) {
				IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_INF,
					"CAM%c P1_HW_DON_%d_%d dma done(0x%x,0x%x,0x%x)int(0x%x,0x%x,0x%x)FLKBA(0x%x,0x%x,0x%x)SPR(9:0x%x,0x%x,0x%x,10:0x%x,0x%x,0x%x)THR14(0x%x,0x%x,0x%x)FBC(0x%x,0x%x,0x%x)\n",
		'A' + cardinalNum,
		(sof_count[module]) ? (sof_count[module] - 1)
		: (sof_count[module]),
		HwP1Done_cnt[module],
		Irq2StatusX,
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT2_STATUSX(
		ISP_CAM_B_INNER_IDX)),
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT2_STATUSX(
		ISP_CAM_C_INNER_IDX)),
		IrqStatusX,
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT_STATUSX(
		ISP_CAM_B_INNER_IDX)),
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT_STATUSX(
		ISP_CAM_C_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_BASE_ADDR(ISP_CAM_A_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_BASE_ADDR(ISP_CAM_B_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_BASE_ADDR(ISP_CAM_C_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_FH_FH_SPARE_9(ISP_CAM_A_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_FH_FH_SPARE_9(ISP_CAM_B_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_FH_FH_SPARE_9(ISP_CAM_C_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_FH_FH_SPARE_10(ISP_CAM_A_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_FH_FH_SPARE_10(ISP_CAM_B_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_FH_FH_SPARE_10(ISP_CAM_C_INNER_IDX)),
		ISP_RD32(CAM_REG_CQ_THR14_BASEADDR(ISP_CAM_A_INNER_IDX)),
		ISP_RD32(CAM_REG_CQ_THR14_BASEADDR(ISP_CAM_B_INNER_IDX)),
		ISP_RD32(CAM_REG_CQ_THR14_BASEADDR(ISP_CAM_C_INNER_IDX)),
		ISP_RD32(CAM_REG_FBC_FLKO_CTL2(ISP_CAM_A_INNER_IDX)),
		ISP_RD32(CAM_REG_FBC_FLKO_CTL2(ISP_CAM_B_INNER_IDX)),
		ISP_RD32(CAM_REG_FBC_FLKO_CTL2(ISP_CAM_C_INNER_IDX)));
			}
		}
	}

	spin_lock(&(IspInfo.SpinLockIrq[module]));
	if (IrqStatus & VS_INT_ST) {
		Vsync_cnt[cardinalNum]++;
		/*LOG_INF("CAMA N3D:0x%x\n", Vsync_cnt[0]); */
	}
	if (IrqStatus & SW_PASS1_DON_ST) {
		time = ktime_get(); /* ns */
		sec = time;
		do_div(sec, 1000);	   /* usec */
		usec = do_div(sec, 1000000); /* sec and usec */
		/* update pass1 done time stamp for eis user */
		/* (need match with the time stamp in image header) */

		IspInfo.IrqInfo.LastestSigTime_usec[module][10] =
			(unsigned int)(usec);

		IspInfo.IrqInfo.LastestSigTime_sec[module][10] =
			(unsigned int)(sec);

		if (IspInfo.DebugMask & ISP_DBG_INT) {
			/*SW p1_don is not reliable */
			if (FrameStatus[module] != CAM_FST_DROP_FRAME) {
				gPass1doneLog[module].module = module;
#ifdef ENABLE_STT_IRQ_LOG
				if (snprintf(gPass1doneLog[module]._str,
				P1DONE_STR_LEN,
				"CAM_%c P1_DON_%d(0x%08x_0x%08x,0x%08x_0x%08x)dma done(0x%x,0x%x,0x%x)int(0x%x,0x%x,0x%x)FLKBA(0x%x,0x%x,0x%x)AAO(0x%x,0x%x,0x%x)THR14(0x%x,0x%x,0x%x)FBC(0x%x,0x%x,0x%x)exe_us:%d ",
		'A' + cardinalNum,
		(sof_count[module])
			? (sof_count[module] - 1)
			: (sof_count[module]),
		(unsigned int)(fbc_ctrl1[0].Raw),
		(unsigned int)(fbc_ctrl2[0].Raw),
		(unsigned int)(fbc_ctrl1[1].Raw),
		(unsigned int)(fbc_ctrl2[1].Raw),
		Irq2StatusX,
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT2_STATUSX(
		ISP_CAM_B_INNER_IDX)),
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT2_STATUSX(
		ISP_CAM_C_INNER_IDX)),
		IrqStatusX,
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT_STATUSX(
		ISP_CAM_B_INNER_IDX)),
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT_STATUSX(
		ISP_CAM_C_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_BASE_ADDR(ISP_CAM_A_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_BASE_ADDR(ISP_CAM_B_INNER_IDX)),
		ISP_RD32(CAM_REG_FLKO_BASE_ADDR(ISP_CAM_C_INNER_IDX)),
		ISP_RD32(CAM_REG_AAO_BASE_ADDR(ISP_CAM_A_INNER_IDX)),
		ISP_RD32(CAM_REG_AAO_BASE_ADDR(ISP_CAM_B_INNER_IDX)),
		ISP_RD32(CAM_REG_AAO_BASE_ADDR(ISP_CAM_C_INNER_IDX)),
		ISP_RD32(CAM_REG_CQ_THR14_BASEADDR(ISP_CAM_A_INNER_IDX)),
		ISP_RD32(CAM_REG_CQ_THR14_BASEADDR(ISP_CAM_B_INNER_IDX)),
		ISP_RD32(CAM_REG_CQ_THR14_BASEADDR(ISP_CAM_C_INNER_IDX)),
		ISP_RD32(CAM_REG_FBC_FLKO_CTL2(ISP_CAM_A_INNER_IDX)),
		ISP_RD32(CAM_REG_FBC_FLKO_CTL2(ISP_CAM_B_INNER_IDX)),
		ISP_RD32(CAM_REG_FBC_FLKO_CTL2(ISP_CAM_C_INNER_IDX)),
		(unsigned int)((sec * 1000000 + usec) -
		(1000000 * sec_sof[module] +
		usec_sof[module]))) < 0) {
					LOG_NOTICE("Error: snprintf fail\n");
				}
#else
				if (snprintf(gPass1doneLog[module]._str,
				P1DONE_STR_LEN,
				"CAM_%c P1_DON_%d(0x%08x_0x%08x,0x%08x_0x%08x)dma done(0x%x,0x%x,0x%x)int(0x%x,0x%x,0x%x)exe_us:%d ",
		'A' + cardinalNum,
		(sof_count[module])
			? (sof_count[module] - 1)
			: (sof_count[module]),
		(unsigned int)(fbc_ctrl1[0].Raw),
		(unsigned int)(fbc_ctrl2[0].Raw),
		(unsigned int)(fbc_ctrl1[1].Raw),
		(unsigned int)(fbc_ctrl2[1].Raw),
		Irq2StatusX,
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT2_STATUSX(
		ISP_CAM_B_INNER_IDX)),
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT2_STATUSX(
		ISP_CAM_C_INNER_IDX)),
		IrqStatusX,
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT_STATUSX(
		ISP_CAM_B_INNER_IDX)),
		(unsigned int)ISP_RD32(
		CAM_REG_CTL_RAW_INT_STATUSX(
		ISP_CAM_C_INNER_IDX)),
		(unsigned int)((sec * 1000000 + usec) -
		(1000000 * sec_sof[module] +
		usec_sof[module]))) < 0) {
					LOG_NOTICE("Error: snprintf fail\n");
				}
#endif
			}
		}
#if (TSTMP_SUBSAMPLE_INTPL == 1)
		if (g1stSwP1Done[module] == MTRUE) {
			unsigned long long cur_timestp =
				(unsigned long long)sec * 1000000 + usec;

			unsigned int frmPeriod =
				((ISP_RD32(CAM_REG_TG_SUB_PERIOD(reg_module)) >>
				  8) &
				 0x1F) +
				1;

			if (frmPeriod > 1) {
				ISP_PatchTimestamp(module, _imgo_, frmPeriod,
						   cur_timestp,
						   gPrevSofTimestp[module]);
				ISP_PatchTimestamp(module, _rrzo_, frmPeriod,
						   cur_timestp,
						   gPrevSofTimestp[module]);
				ISP_PatchTimestamp(module, _ufeo_, frmPeriod,
						   cur_timestp,
						   gPrevSofTimestp[module]);
				ISP_PatchTimestamp(module, _lmvo_, frmPeriod,
						   cur_timestp,
						   gPrevSofTimestp[module]);
				ISP_PatchTimestamp(module, _lcso_, frmPeriod,
						   cur_timestp,
						   gPrevSofTimestp[module]);
				ISP_PatchTimestamp(module, _lcesho_, frmPeriod,
						   cur_timestp,
						   gPrevSofTimestp[module]);
				ISP_PatchTimestamp(module, _ufgo_, frmPeriod,
						   cur_timestp,
						   gPrevSofTimestp[module]);
			}

			g1stSwP1Done[module] = MFALSE;
		}
#endif

		/* for dbg log only */
		if (pstRTBuf[module]->ring_buf[_imgo_].active) {

			pstRTBuf[module]->ring_buf[_imgo_].img_cnt =
				sof_count[module];
		}
		if (pstRTBuf[module]->ring_buf[_rrzo_].active) {
			pstRTBuf[module]->ring_buf[_rrzo_].img_cnt =
				sof_count[module];
		}
		if (pstRTBuf[module]->ring_buf[_ufeo_].active) {
			pstRTBuf[module]->ring_buf[_ufeo_].img_cnt =
				sof_count[module];
		}
		if (pstRTBuf[module]->ring_buf[_ufgo_].active) {
			pstRTBuf[module]->ring_buf[_ufgo_].img_cnt =
				sof_count[module];
		}
	}

	if ((IrqStatus & SOF_INT_ST) && isStagger == 1)
		IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR, "CAM%c DCIF_SOF", 'A' + cardinalNum);

	if (((IrqStatus & SOF_INT_ST) && isStagger == 0) ||
		((IrqStatus & VS_INT_ST) && isStagger == 1 &&
		(ISP_RD32(CAMX_REG_TG_VF_CON(reg_module)) == 0x1))) {
		unsigned int frmPeriod =
			((ISP_RD32(CAM_REG_TG_SUB_PERIOD(reg_module)) >> 8) &
			 0x1F) +
			1;

		unsigned int irqDelay = 0;

		time = ktime_get(); /* ns */
		sec = time;
		do_div(sec, 1000);	   /* usec */
		usec = do_div(sec, 1000000); /* sec and usec */

		cur_v_cnt = ISP_RD32_TG_CAMX_FRM_CNT(module, reg_module);
#if (Lafi_WAM_CQ_ERR == 1)
		if (!(ErrStatus & CQ_VS_ERR_ST))
			ISP_RecordCQAddr(reg_module);
#endif

		if (frmPeriod == 0) {
			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_ERR,
				       "ERROR: Wrong sub-sample period: 0");
			goto LB_CAM_SOF_IGNORE;
		}

		/* chk this frame have EOF or not, dynimic dma port chk */
		FrameStatus[module] =
			Irq_CAM_FrameStatus(reg_module, module, irqDelay, &FBC_CurState);

		if (ErrStatus & CQ_VS_ERR_ST)
			FrameStatus[module] = CAM_FST_BLOCK_FRAME;

		if (FrameStatus[module] == CAM_FST_DROP_FRAME) {
			gLostPass1doneLog[module].module = module;
			if (snprintf(gLostPass1doneLog[module]._str, P1DONE_STR_LEN,
				"CAM%c Lost p1 done_%d (0x%x): ",
				'A' + cardinalNum, sof_count[module],
				cur_v_cnt) < 0) {
				LOG_NOTICE("Error: snprintf fail\n");
			}
			/*
			 *IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_INF,
			 *	       "CAM%c Lost p1 done_%d (0x%x): ",
			 *	       'A' + cardinalNum, sof_count[module],
			 *	       cur_v_cnt);
			 */
		}
		if ((ISP_RD32(CAM_REG_DMA_CQ_COUNTER(reg_module)))
			!= g_virtual_cq_cnt[module]){
			IrqStatus &= ~SOF_INT_ST;
			IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_INF,
				"CAM%c PHY cqcnt:%d != VIR cqcnt:%d, IrqStatus:0x%x\n",
				'A'+cardinalNum,
				ISP_RD32(CAM_REG_DMA_CQ_COUNTER(reg_module)),
				g_virtual_cq_cnt[module],
				IrqStatus);
		}
		/* During SOF, re-enable that err/warn irq had been marked and
		 * reset IrqCntInfo
		 */
		IrqEnableNew = ISP_RD32(CAM_REG_CTL_RAW_INT_EN(reg_module));
		IrqEnableNew |= (IspInfo.IrqCntInfo.m_err_int_mark[module] |
				 IspInfo.IrqCntInfo.m_warn_int_mark[module]);
		ISP_WR32(CAM_REG_CTL_RAW_INT_EN(reg_module), IrqEnableNew);

		IspInfo.IrqCntInfo.m_err_int_mark[module] = 0;
		IspInfo.IrqCntInfo.m_warn_int_mark[module] = 0;
		IspInfo.IrqCntInfo.m_int_usec[module] = 0;

		for (i = 0; i < ISP_ISR_MAX_NUM; i++) {
			IspInfo.IrqCntInfo.m_err_int_cnt[module][i] = 0;
			IspInfo.IrqCntInfo.m_warn_int_cnt[module][i] = 0;
		}

		if (IspInfo.DebugMask & ISP_DBG_INT) {
			static unsigned int m_sec[ISP_IRQ_TYPE_AMOUNT] = {0};
			static unsigned int m_usec[ISP_IRQ_TYPE_AMOUNT] = {0};
			unsigned int magic_num;

			if (pstRTBuf[module]->ring_buf[_imgo_].active) {

				magic_num = ISP_RD32(
					CAM_REG_IMGO_FH_SPARE_2(reg_module));
			} else {
				magic_num = ISP_RD32(
					CAM_REG_RRZO_FH_SPARE_2(reg_module));
			}
			if (g1stSof[module]) {
				m_sec[module] = sec;
				m_usec[module] = usec;
				gSTime[module].sec = sec;
				gSTime[module].usec = usec;
			}
			#if (TIMESTAMP_QUEUE_EN == 1)
			{
				unsigned long long cur_timestp =
				(unsigned long long)sec*1000000 + usec;

				unsigned int subFrm = 0;
				enum CAM_FrameST FrmStat_aao, FrmStat_afo,
					FrmStat_aaho, FrmStat_flko, FrmStat_pdo,
					FrmStat_ltmso;

			ISP_GetDmaPortsStatus(reg_module,
					IspInfo.TstpQInfo[module].DmaEnStatus);

			/* Prevent WCNT increase after
			 * ISP_CompensateMissingSofTime around P1_DON
			 * and FBC_CNT decrease to 0, following drop frame is
			 * checked becomes true,
			 * then SOF timestamp will missing for current frame
			 */
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_aao_]) {
				FrmStat_aao = Irq_CAM_SttFrameStatus(
				reg_module, module, _aao_, irqDelay);
			} else {
				FrmStat_aao = CAM_FST_DROP_FRAME;
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_aaho_]) {
				FrmStat_aaho = Irq_CAM_SttFrameStatus(
				reg_module, module, _aaho_, irqDelay);
			} else {
				FrmStat_aaho = CAM_FST_DROP_FRAME;
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_afo_]) {
				FrmStat_afo = Irq_CAM_SttFrameStatus(
				reg_module, module, _afo_, irqDelay);
			} else {
				FrmStat_afo = CAM_FST_DROP_FRAME;
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_flko_]) {
				FrmStat_flko = Irq_CAM_SttFrameStatus(
				reg_module, module, _flko_, irqDelay);
			} else {
				FrmStat_flko = CAM_FST_DROP_FRAME;
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_pdo_]) {
				FrmStat_pdo = Irq_CAM_SttFrameStatus(
				reg_module, module, _pdo_, irqDelay);
				} else {
					FrmStat_pdo = CAM_FST_DROP_FRAME;
				}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_ltmso_]) {
				FrmStat_ltmso = Irq_CAM_SttFrameStatus(
				reg_module, module, _ltmso_, irqDelay);
				} else {
					FrmStat_ltmso = CAM_FST_DROP_FRAME;
				}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_imgo_])
				ISP_CompensateMissingSofTime(
					reg_module, module, _imgo_,
					sec, usec, frmPeriod);

			if (IspInfo.TstpQInfo[module].DmaEnStatus[_rrzo_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _rrzo_,
				sec, usec, frmPeriod);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_ufeo_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _ufeo_,
				sec, usec, frmPeriod);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_ufgo_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _ufgo_,
				sec, usec, frmPeriod);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_lmvo_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _lmvo_,
				sec, usec, frmPeriod);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_lcso_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _lcso_,
				sec, usec, frmPeriod);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_lcesho_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _lcesho_,
				sec, usec, frmPeriod);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_aao_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _aao_,
				sec, usec, 1);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_aaho_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _aaho_,
				sec, usec, 1);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_afo_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _afo_,
				sec, usec, 1);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_flko_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _flko_,
				sec, usec, 1);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_pdo_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _pdo_,
				sec, usec, 1);
			}
			if (IspInfo.TstpQInfo[module].DmaEnStatus[_ltmso_]) {
				ISP_CompensateMissingSofTime(
				reg_module, module, _ltmso_,
				sec, usec, 1);
			}
			if (FrameStatus[module] != CAM_FST_DROP_FRAME) {
				for (subFrm = 0; subFrm < frmPeriod; subFrm++) {
					/* Current frame is NOT DROP FRAME */
					if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_imgo_]) {
						ISP_PushBufTimestamp(module,
							_imgo_, sec, usec,
							frmPeriod);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_rrzo_]) {
					ISP_PushBufTimestamp(module,
							_rrzo_, sec, usec,
							frmPeriod);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_ufeo_]) {
					ISP_PushBufTimestamp(module,
							_ufeo_, sec, usec,
							frmPeriod);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_ufgo_]) {
					ISP_PushBufTimestamp(module,
							_ufgo_, sec, usec,
							frmPeriod);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_lmvo_]) {
					ISP_PushBufTimestamp(module,
							_lmvo_, sec, usec,
							frmPeriod);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_lcso_]) {
					ISP_PushBufTimestamp(module,
							_lcso_, sec, usec,
							frmPeriod);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus
				[_lcesho_]) {
					ISP_PushBufTimestamp(module,
							_lcesho_, sec, usec,
							frmPeriod);
					}
				}

				/* for slow motion sub-sample */
				/* must after current ISP_PushBufTimestamp() */
				#if (TSTMP_SUBSAMPLE_INTPL == 1)
			if ((frmPeriod > 1) && (g1stSwP1Done[module] ==
			MFALSE)) {
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_imgo_]) {
					ISP_PatchTimestamp(module,
						_imgo_, frmPeriod,
						cur_timestp,
						gPrevSofTimestp
						[module]);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_rrzo_]) {
					ISP_PatchTimestamp(module,
						_rrzo_, frmPeriod,
						cur_timestp,
						gPrevSofTimestp
						[module]);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_ufeo_]) {
					ISP_PatchTimestamp(module,
						_ufeo_, frmPeriod,
						cur_timestp,
						gPrevSofTimestp
						[module]);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_ufgo_]) {
					ISP_PatchTimestamp(module,
						_ufgo_, frmPeriod,
						cur_timestp,
						gPrevSofTimestp
						[module]);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_lmvo_]) {
					ISP_PatchTimestamp(module,
						_lmvo_, frmPeriod,
						cur_timestp,
						gPrevSofTimestp
						[module]);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus[_lcso_]) {
					ISP_PatchTimestamp(module,
						_lcso_, frmPeriod,
						cur_timestp,
						gPrevSofTimestp
						[module]);
					}
				if (
				IspInfo.TstpQInfo[module].DmaEnStatus
				[_lcesho_]) {
					ISP_PatchTimestamp(module,
						_lcesho_, frmPeriod,
						cur_timestp,
						gPrevSofTimestp
						[module]);
					}
				}
#endif
			}

			if (IspInfo.TstpQInfo[module]
				    .DmaEnStatus[_aao_]) {
				if (FrmStat_aao != CAM_FST_DROP_FRAME) {
					ISP_PushBufTimestamp(module,
							     _aao_, sec,
							     usec, 1);
				}
			}
			if (IspInfo.TstpQInfo[module]
				    .DmaEnStatus[_aaho_]) {
				if (FrmStat_aao != CAM_FST_DROP_FRAME) {
					ISP_PushBufTimestamp(module,
							     _aaho_, sec,
							     usec, 1);
				}
			}
			if (IspInfo.TstpQInfo[module]
				    .DmaEnStatus[_afo_]) {
				if (FrmStat_afo != CAM_FST_DROP_FRAME) {
					ISP_PushBufTimestamp(module,
							     _afo_, sec,
							     usec, 1);
				}
			}
			if (IspInfo.TstpQInfo[module]
				    .DmaEnStatus[_flko_]) {
				if (FrmStat_flko !=
				    CAM_FST_DROP_FRAME) {
					ISP_PushBufTimestamp(
						module, _flko_, sec,
						usec, 1);
				}
			}
			if (IspInfo.TstpQInfo[module]
				    .DmaEnStatus[_pdo_]) {
				if (FrmStat_pdo != CAM_FST_DROP_FRAME) {
					ISP_PushBufTimestamp(module,
							     _pdo_, sec,
							     usec, 1);
				}
			}
			if (IspInfo.TstpQInfo[module]
				    .DmaEnStatus[_ltmso_]) {
				if (FrmStat_ltmso != CAM_FST_DROP_FRAME) {
					ISP_PushBufTimestamp(module,
							     _ltmso_, sec,
							     usec, 1);
				}
			}
#if (TSTMP_SUBSAMPLE_INTPL == 1)
				gPrevSofTimestp[module] = cur_timestp;
#endif
			}
#endif /* (TIMESTAMP_QUEUE_EN == 1) */

#if 0
			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"%s,%s,CAM_%c P1_SOF_%d_%d(0x%08x_0x%08x,0x%08x_0x%08x,0x%08x,0x%08x,0x%x),int_us:%d,cq:0x%08x, DMA 0x%x,0x%x, LTMS/FLK/AA/AAH/TSFS(0x%x, 0x%x, 0x%x, 0x%x, 0x%x) pa(0x%x,0x%x,0x%x,0x%x,0x%x),0x%x,0x%x,0x%x)\n",
				gPass1doneLog[module]._str,
				gLostPass1doneLog[module]._str,
				'A' + cardinalNum, sof_count[module], cur_v_cnt,
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_IMGO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_IMGO_CTL2(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_RRZO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_RRZO_CTL2(reg_module))),
				ISP_RD32(CAM_REG_IMGO_BASE_ADDR(reg_module)),
				ISP_RD32(CAM_REG_RRZO_BASE_ADDR(reg_module)),
				magic_num,
				(unsigned int)((sec * 1000000 + usec) -
				(1000000 * m_sec[module] + m_usec[module])),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DMA_EN(reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DMA_EN(
						inner_reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_LTMSO_CTL2(reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_FLKO_CTL2(
						reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_AAO_CTL2(reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_AAHO_CTL2(
						reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_TSFSO_CTL2(
						reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_LTMSO_BASE_ADDR(reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_FLKO_BASE_ADDR(
						reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_AAO_BASE_ADDR(reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_AAHO_BASE_ADDR(
						reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_TSFSO_BASE_ADDR(
						reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_FLKO_FH_BASE_ADDR(
						reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_AAO_FH_BASE_ADDR(
						reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_AAHO_FH_BASE_ADDR(
						reg_module)));
#else
			if (sec_on)
				IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"CAM_%c P1_SOF_%d_%d(0x%08x_0x%08x,0x%08x_0x%08x,0x%08x,0x%08x,0x%x/0x%x),int_us:%d,cq:0x%08x_0x%08x 0x%08x_0x%08x,Don(0x%08x_0x%08x 0x%08x_0x%08x,0x%08x_0x%08x 0x%08x_0x%08x 0x%08x_0x%08x),DMA(0x%x_0x%x,0x%x_0x%x,0x%x_0x%x),CRZO(0x%x_0x%x,0x%x_0x%x,0x%x_0x%x,0x%x_0x%x,0x%x_0x%x,0x%x_0x%x),YUVO(0x%x_0x%x 0x%x_0x%x, 0x%x_0x%x 0x%x_0x%x, 0x%x_0x%x 0x%x_0x%x)\n",
				'A' + cardinalNum, sof_count[module], cur_v_cnt,
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_IMGO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_IMGO_CTL2(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_RRZO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_RRZO_CTL2(reg_module))),
				ISP_RD32(CAM_REG_IMGO_BASE_ADDR(reg_module)),
				ISP_RD32(CAM_REG_RRZO_BASE_ADDR(reg_module)),
				magic_num, pre_magic_num,
				(unsigned int)((sec * 1000000 + usec) -
					       (1000000 * m_sec[module] + m_usec[module])),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(
						ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DONE_SEL(
						ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DONE_SEL(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DONE_SEL(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DONE_SEL(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						reg_module)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_CRZO_CTL2(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_CRZO_CTL2(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_CRZO_CTL2(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_CRZO_CTL2(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CRZO_BASE_ADDR(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CRZO_BASE_ADDR(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CRZO_BASE_ADDR(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CRZO_BASE_ADDR(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CRZO_FH_BASE_ADDR(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CRZO_FH_BASE_ADDR(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CRZO_FH_BASE_ADDR(
						ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CRZO_FH_BASE_ADDR(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_YUVO_CTL2(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_YUVO_CTL2(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_YUVO_CTL2(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_FBC_YUVO_CTL2(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_YUVO_BASE_ADDR(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_YUVO_BASE_ADDR(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_YUVO_BASE_ADDR(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_YUVO_BASE_ADDR(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_YUVO_FH_BASE_ADDR(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_YUVO_FH_BASE_ADDR(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_YUVO_FH_BASE_ADDR(
						ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_YUVO_FH_BASE_ADDR(
						ISP_CAM_C_INNER_IDX)));
			else {
				IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"%s,%s,CAM_%c P1_SOF_%d_%d(0x%08x_0x%08x,0x%08x_0x%08x,0x%08x,0x%08x,0x%x/0x%x),int_us:%d,FBC:0x%08x,cq:0x%08x_0x%08x 0x%08x_0x%08x 0x%08x_0x%08x,Don(0x%08x_0x%08x 0x%08x_0x%08x,0x%08x_0x%08x 0x%08x_0x%08x),DMA(0x%x_0x%x,0x%x_0x%x,0x%x_0x%x,0x%x_0x%x,0x%x_0x%x,0x%x_0x%x),CTL_EN(0x%x_0x%x 0x%x_0x%x 0x%x_0x%x),CTL_EN2(0x%x_0x%x 0x%x_0x%x 0x%x_0x%x)\n",
				gPass1doneLog[module]._str,
				gLostPass1doneLog[module]._str,
				'A' + cardinalNum, sof_count[module], cur_v_cnt,
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_IMGO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_IMGO_CTL2(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_RRZO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_RRZO_CTL2(reg_module))),
				ISP_RD32(CAM_REG_IMGO_BASE_ADDR(reg_module)),
				ISP_RD32(CAM_REG_RRZO_BASE_ADDR(reg_module)),
				magic_num, pre_magic_num,
				(unsigned int)((sec * 1000000 + usec) -
				(1000000 * m_sec[module] + m_usec[module])),
				FBC_CurState,
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
						ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CQ_THR0_BASEADDR(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_MISC(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DONE_SEL(
						ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DONE_SEL(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DONE_SEL(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DONE_SEL(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DMA_EN(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DMA_EN(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DMA_EN(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DMA_EN(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DMA_EN(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_DMA_EN(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_DMA_FRAME_HEADER_EN1(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN(
						ISP_CAM_C_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN2(ISP_CAM_A_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN2(
						ISP_CAM_A_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN2(ISP_CAM_B_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN2(
						ISP_CAM_B_INNER_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN2(ISP_CAM_C_IDX)),
				(unsigned int)ISP_RD32(
					CAM_REG_CTL_EN2(
						ISP_CAM_C_INNER_IDX)));
			}
#endif

			if (snprintf(gPass1doneLog[module]._str, P1DONE_STR_LEN, "\\") < 0)
				LOG_NOTICE("Error: snprintf fail\n");

			if (snprintf(gLostPass1doneLog[module]._str, P1DONE_STR_LEN, "\\") < 0)
				LOG_NOTICE("Error: snprintf fail\n");

#ifdef ENABLE_STT_IRQ_LOG /*STT addr */
			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"CAM_%c_aa(0x%08x_0x%08x_0x%08x)aaho(0x%08x_0x%08x_0x%08x)\n",
				'A' + cardinalNum,
				ISP_RD32(CAM_REG_AAO_BASE_ADDR(reg_module)),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_AAO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_AAO_CTL2(reg_module))),
				ISP_RD32(CAM_REG_AAHO_BASE_ADDR(reg_module)),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_AAHO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_AAHO_CTL2(reg_module))));

			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"CAM_%c_af(0x%08x_0x%08x_0x%08x)pd(0x%08x_0x%08x_0x%08x)\n",
				'A' + cardinalNum,
				ISP_RD32(CAM_REG_AFO_BASE_ADDR(reg_module)),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_AFO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_AFO_CTL2(reg_module))),
				ISP_RD32(CAM_REG_PDO_BASE_ADDR(reg_module)),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_PDO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_PDO_CTL2(reg_module))));

			IRQ_LOG_KEEPER(
				module, m_CurrentPPB, _LOG_INF,
				"CAM_%c_flk(0x%08x_0x%08x_0x%08x)ltms(0x%08x_0x%08x_0x%08x)\n",
				'A' + cardinalNum,
				ISP_RD32(CAM_REG_FLKO_BASE_ADDR(reg_module)),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_FLKO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_FLKO_CTL2(reg_module))),
				ISP_RD32(CAM_REG_LTMSO_BASE_ADDR(reg_module)),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_LTMSO_CTL1(reg_module))),
				(unsigned int)(ISP_RD32(
					CAM_REG_FBC_LTMSO_CTL2(reg_module))));
#endif
			/* keep current time */
			m_sec[module] = sec;
			m_usec[module] = usec;
			sec_sof[module] = sec;
			usec_sof[module] = usec;

			/* dbg information only */
			if (cur_v_cnt !=
			    ISP_RD32_TG_CAMX_FRM_CNT(module, reg_module)) {
				IRQ_LOG_KEEPER(
					module, m_CurrentPPB, _LOG_INF,
					"SW ISR right on next hw p1_done\n");
			}

			/*keep previous magic number */
			pre_magic_num = magic_num;
		}
		/* update SOF time stamp for eis user */
		/* (need match with the time stamp in image header) */
		IspInfo.IrqInfo.LastestSigTime_usec[module][12] =
			(unsigned int)(sec);

		IspInfo.IrqInfo.LastestSigTime_sec[module][12] =
			(unsigned int)(usec);

		sof_count[module] += frmPeriod;
		/* for match vsync cnt */
		if (sof_count[module] > 255)
			sof_count[module] -= 256;

		g1stSof[module] = MFALSE;
	}
LB_CAM_SOF_IGNORE:

#ifdef ENABLE_STT_IRQ_LOG
	if (DmaStatus & (AAO_DONE_ST | AFO_DONE_ST | AAHO_DONE_ST |
		LTMSO_R1_DONE_ST | PDO_DONE_ST)) {
		IRQ_LOG_KEEPER(module, m_CurrentPPB, _LOG_INF,
			       "CAM%c_STT_Done_%d_0x%x\n", 'A' + cardinalNum,
			       (sof_count[module]) ? (sof_count[module] - 1)
						   : (sof_count[module]),
			       DmaStatus);
	}
#endif

	for (i = 0; i < IRQ_USER_NUM_MAX; i++) {
		/* 1. update interrupt status to all users */
		IspInfo.IrqInfo.Status[module][SIGNAL_INT][i] |= IrqStatus;
		IspInfo.IrqInfo.Status[module][DMA_INT][i] |= DmaStatus;

		/* 2. update signal time and passed by signal count */
		if (IspInfo.IrqInfo.MarkedFlag[module][SIGNAL_INT][i] &
		    IspInfo.IrqInfo.Mask[module][SIGNAL_INT]) {
			unsigned int cnt = 0, tmp = IrqStatus;

			while (tmp) {
				if (tmp & 0x1) {
					IspInfo.IrqInfo
						.LastestSigTime_usec[module]
								    [cnt] =
						(unsigned int)time_frmb.tv_nsec
						/ 1000;

					IspInfo.IrqInfo
						.LastestSigTime_sec[module]
								   [cnt] =
						(unsigned int)time_frmb.tv_sec;

					IspInfo.IrqInfo
						.PassedBySigCnt[module][cnt]
							       [i]++;
				}
				tmp = tmp >> 1;
				cnt++;
			}
		} else {
			/* no any interrupt is not marked and */
			/* in read mask in this irq type */
		}
	}
	spin_unlock(&(IspInfo.SpinLockIrq[module]));
	/*  */
	if (IrqStatus & SOF_INT_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_SOF]);
	}
	if (IrqStatus & SW_PASS1_DON_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_SW_P1_DONE]);
	}
	if (IrqStatus & HW_PASS1_DON_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_HW_P1_DONE]);
	}
	if (DmaStatus & AAO_DONE_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_AAO_DONE]);
	}
	if (DmaStatus & AAHO_DONE_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_AAHO_DONE]);
	}
	if (DmaStatus & FLKO_DONE_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_FLKO_DONE]);
	}
	if (DmaStatus & AFO_DONE_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_AFO_DONE]);
	}
	if (DmaStatus & TSFSO_DONE_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_TSFSO_DONE]);
	}
	if (DmaStatus & LTMSO_R1_DONE_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_LTMSO_DONE]);
	}
	if (DmaStatus & PDO_DONE_ST) {
		wake_up_interruptible(
			&IspInfo.WaitQHeadCam[ISP_GetWaitQCamIndex(module)]
					     [ISP_WAITQ_HEAD_IRQ_PDO_DONE]);
	}
	wake_up_interruptible(&IspInfo.WaitQueueHead[module]);

	/* dump log, use workq */
	if ((IrqStatus & (SOF_INT_ST | HW_PASS1_DON_ST | VS_INT_ST)) ||
		ErrStatus) {
#if (ISP_BOTTOMHALF_WORKQ == 1)
		queue_work(isr_log_wq, &isp_workque[module].isp_bh_work);
#endif
	}

	return IRQ_HANDLED;
}

/*******************************************************************************
 *
 ******************************************************************************/
static void SMI_INFO_DUMP(enum ISP_IRQ_TYPE_ENUM irq_module)
{
#ifndef EP_MARK_SMI
	switch (irq_module) {
	case ISP_IRQ_TYPE_INT_CAM_A_ST:
	case ISP_IRQ_TYPE_INT_CAM_B_ST:
	case ISP_IRQ_TYPE_INT_CAM_C_ST:
		if ((g_ISPIntStatus_SMI[irq_module].ispIntErr & DMA_ERR_ST) &&
			!(g_ISPIntStatus_SMI[irq_module].ispIntErr
			& TG_GBERR_ST)){
			if (g_ISPIntStatus_SMI[irq_module].ispInt5Err &
			    INT_ST_MASK_CAM_WARN) {

				LOG_NOTICE("ERR:SMI_DUMP by module:%d\n",
					   irq_module);
#if IS_ENABLED(CONFIG_MTK_SMI_EXT)
		if (smi_debug_bus_hang_detect(false, "camera_isp") != 0)
			LOG_NOTICE("ERR:smi_debug_bus_hang_detect");
#endif
			}
			g_ISPIntStatus_SMI[irq_module].ispIntErr =
				g_ISPIntStatus_SMI[irq_module].ispInt5Err = 0;
		}
		if (g_ISPIntStatus_SMI[irq_module].ispIntErr & CQ_VS_ERR_ST) {
/* sw workaround for CQ byebye */
#if Lafi_WAM_CQ_ERR
			unsigned int reg_module_array[3];
			unsigned int reg_module_count;
			unsigned int reg_module = ISP_CAM_A_IDX;
			unsigned int ret = 0;
			/**/
			ret = CQ_Recover_start(irq_module, &reg_module,
				reg_module_array, &reg_module_count);
			if (ret == -1)
				goto EXIT_CQ_RECOVER;
			//ISP_DumpDbgPort(reg_module_array, reg_module_count);
			ret = CQ_Recover_middle(reg_module,
				reg_module_array, reg_module_count);
			if (ret == -1)
				goto EXIT_CQ_RECOVER;
			ret = CQ_Recover_final(reg_module,
				reg_module_array, reg_module_count);
			if (ret == -1)
				goto EXIT_CQ_RECOVER;
EXIT_CQ_RECOVER:
				LOG_NOTICE("-CQ recover");
#endif
			/*
			 * LOG_NOTICE("ERR:SMI_DUMP by module:%d\n",
			 * irq_module);
			 * if (smi_debug_bus_hang_detect(
			 *              SMI_PARAM_BUS_OPTIMIZATION,
			 *              true, false, true) != 0)
			 *      LOG_NOTICE("ERR:smi_debug_bus_hang_detect");
			 */
			g_ISPIntStatus_SMI[irq_module].ispIntErr =
				g_ISPIntStatus_SMI[irq_module].ispInt5Err = 0;
		}
		break;
	case ISP_IRQ_TYPE_INT_CAMSV_0_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_1_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_2_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_3_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_4_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_5_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_6_ST:
	case ISP_IRQ_TYPE_INT_CAMSV_7_ST:
		if (g_ISPIntStatus_SMI[irq_module].ispIntErr & SV_IMGO_ERR) {
			if (g_ISPIntStatus_SMI[irq_module].ispIntErr &
			    SV_IMGO_OVERRUN) {

				LOG_NOTICE("ERR:SMI_DUMP by module:%d\n",
					   irq_module);

#if IS_ENABLED(CONFIG_MTK_SMI_EXT)
		if (smi_debug_bus_hang_detect(false, "camera_isp_camsv") != 0)
			LOG_NOTICE("ERR:smi_debug_bus_hang_detect");
#endif

			}
			g_ISPIntStatus_SMI[irq_module].ispIntErr = 0;
		}
		break;
	default:
		LOG_NOTICE("error:unsupported module:%d\n", irq_module);
		break;
	}
#endif
}

#if (ISP_BOTTOMHALF_WORKQ == 1)
static void ISP_BH_Workqueue(struct work_struct *pWork)
{
	struct IspWorkqueTable *pWorkTable =
		container_of(pWork, struct IspWorkqueTable, isp_bh_work);

	IRQ_LOG_PRINTER(pWorkTable->module, m_CurrentPPB, _LOG_ERR);
	IRQ_LOG_PRINTER(pWorkTable->module, m_CurrentPPB, _LOG_INF);
	SMI_INFO_DUMP(pWorkTable->module);
}
#endif

/*******************************************************************************
 *
 ******************************************************************************/
module_init(ISP_Init);
module_exit(ISP_Exit);
MODULE_DESCRIPTION("Camera ISP driver");
MODULE_AUTHOR("SW7");
MODULE_LICENSE("GPL");
