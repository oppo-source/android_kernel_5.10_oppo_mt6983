/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#ifndef __LPM_MODULE_H__
#define __LPM_MODULE_H__

#include <linux/arm-smccc.h>

#include <linux/soc/mediatek/mtk_sip_svc.h>

#define lpm_smc_impl(p1, p2, p3, p4, p5, res) \
			arm_smccc_smc(p1, p2, p3, p4\
			, p5, 0, 0, 0, &res)


#define lpm_smc(_funcid, _lp_id, _act, _val1, _val2) ({\
	struct arm_smccc_res res;\
	lpm_smc_impl(_funcid, _lp_id, _act, _val1\
					, _val2, res);\
	res.a0; })


/* behavior */
#define MT_LPM_SMC_ACT_SET		(1<<0UL)
#define MT_LPM_SMC_ACT_CLR		(1<<1UL)
#define MT_LPM_SMC_ACT_GET		(1<<2UL)
#define MT_LPM_SMC_ACT_PUSH		(1<<3UL)
#define MT_LPM_SMC_ACT_POP		(1<<4UL)
#define MT_LPM_SMC_ACT_SUBMIT		(1<<5UL)

/* compatible action for legacy smc from lk */
#define MT_LPM_SMC_ACT_COMPAT		(1<<31UL)


#define MT_LPM_SMC_MAGIC		0xDA000000
#define MT_LPM_SMC_USER_MASK		0xff
#define MT_LPM_SMC_USER_SHIFT		16
#define MT_LPM_SMC_USER_ID_MASK		0x0000ffff

#define PSCI_E_SUCCESS			0

enum mt_lpm_smc_user_id {
	mt_lpm_smc_user_cpu_pm = 0,
	mt_lpm_smc_user_spm_dbg,
	mt_lpm_smc_user_spm,
	mt_lpm_smc_user_cpu_pm_lp,
	mt_lpm_smc_user_max,
};


/* sink user id to smc's user id */
#define MT_LPM_SMC_USER_SINK(user, uid) \
		(((uid & MT_LPM_SMC_USER_ID_MASK)\
		| ((user & MT_LPM_SMC_USER_MASK)\
			<< MT_LPM_SMC_USER_SHIFT))\
		| MT_LPM_SMC_MAGIC)



/* sink cpu pm's smc id */
#define MT_LPM_SMC_USER_ID_CPU_PM(uid) \
	MT_LPM_SMC_USER_SINK(mt_lpm_smc_user_cpu_pm, uid)
/* sink spm debug's smc id */
#define MT_LPM_SMC_USER_ID_SPM_DBG(uid) \
	MT_LPM_SMC_USER_SINK(mt_lpm_smc_user_spm_dbg, uid)
/* sink spm's smc id */
#define MT_LPM_SMC_USER_ID_SPM(uid) \
	MT_LPM_SMC_USER_SINK(mt_lpm_smc_user_spm, uid)
/* sink cpu pm's low power extension smc id */
#define MT_LPM_SMC_USER_ID_CPU_PM_LP(uid) \
	MT_LPM_SMC_USER_SINK(mt_lpm_smc_user_cpu_pm_lp, uid)



/* sink cpu pm's user id */
#define MT_LPM_SMC_USER_CPU_PM(uid)\
			MT_LPM_SMC_USER_ID_CPU_PM(uid)
/* sink spm debug's user id */
#define MT_LPM_SMC_USER_SPM_DBG(uid)\
			MT_LPM_SMC_USER_ID_SPM_DBG(uid)
/* sink spm's user id */
#define MT_LPM_SMC_USER_SPM(uid)\
			MT_LPM_SMC_USER_ID_SPM(uid)
	/* sink spm's user id */
#define MT_LPM_SMC_USER_CPU_PM_LP(uid)\
			MT_LPM_SMC_USER_ID_CPU_PM_LP(uid)



/* cpu pm smc definition */
#define MCUSYS_STATUS_PDN				(1 << 0UL)
#define MCUSYS_STATUS_CPUSYS_PROTECT	(1 << 8UL)
#define MCUSYS_STATUS_MCUSYS_PROTECT	(1 << 9UL)

enum MT_CPU_PM_SMC_UID {
	/* cpu_pm function ID*/
	MCUSYS_STATUS,
	CPC_COMMAND,
	IRQ_REMAIN_LIST_ALLOC,
	IRQ_REMAIN_IRQ_ADD,
	IRQ_REMAIN_IRQ_SUBMIT,
	VALIDATE_PWR_STATE_CTRL,
	CPU_PM_CTRL,
};

#define lpm_smc_cpu_pm(_lp_id, _act, _val1, _val2) ({\
		lpm_smc(MTK_SIP_MTK_LPM_CONTROL,\
				MT_LPM_SMC_USER_CPU_PM(_lp_id),\
				_act, _val1, _val2); })

/* spm debug smc definition */

enum MT_SPM_DBG_SMC_UID {
	/* spm dbg function ID*/
	MT_SPM_DBG_SMC_UID_IDLE_PWR_CTRL,
	MT_SPM_DBG_SMC_UID_IDLE_CNT,
	MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL,
	MT_SPM_DBG_SMC_UID_SUSPEND_DBG_CTRL,
	MT_SPM_DBG_SMC_UID_FS,
	MT_SPM_DBG_SMC_UID_RC_SWITCH,
	MT_SPM_DBG_SMC_UID_RC_CNT,
	MT_SPM_DBG_SMC_UID_COND_CHECK,
	MT_SPM_DBG_SMC_UID_COND_BLOCK,
	MT_SPM_DBG_SMC_UID_BLOCK_LATCH,
	MT_SPM_DBG_SMC_UID_BLOCK_DETAIL,
	MT_SPM_DBG_SMC_UID_RES_NUM,
	MT_SPM_DBG_SMC_UID_RES_REQ,
	MT_SPM_DBG_SMC_UID_RES_USAGE,
	MT_SPM_DBG_SMC_UID_RES_USER_NUM,
	MT_SPM_DBG_SMC_UID_RES_USER_VALID,
	MT_SPM_DBG_SMC_UID_RES_USER_NAME,
	MT_SPM_DBG_SMC_UID_DOE_RESOURCE_CTRL,
	MT_SPM_DBG_SMC_UID_DOE_RC,
	MT_SPM_DBG_SMC_UID_RC_COND_CTRL,
	MT_SPM_DBG_SMC_UID_RC_RES_CTRL,
	MT_SPM_DBG_SMC_UID_RC_RES_INFO,
	MT_SPM_DBG_SMC_UID_RC_BBLPM,
	MT_SPM_DBG_SMC_UID_RC_TRACE,
	MT_SPM_DBG_SMC_UID_RC_TRACE_TIME,
	MT_SPM_DBG_SMC_UID_RC_DUMP_PLL,
};

#define lpm_smc_spm_dbg(_lp_id, _act, _val1, _val2) ({\
		lpm_smc(MTK_SIP_MTK_LPM_CONTROL,\
				MT_LPM_SMC_USER_SPM_DBG(_lp_id),\
				_act, _val1, _val2); })

/* spm smc definition */
#define MT_SPM_STATUS_SUSPEND_SLEEP		(1<<27UL)

enum MT_SPM_SMC_UID {
	/* spm function ID*/
	MT_SPM_SMC_UID_STATUS,
	MT_SPM_SMC_UID_PCM_WDT,
	MT_SPM_SMC_UID_PCM_TIMER,
	MT_SPM_SMC_UID_FW_TYPE,
	MT_SPM_SMC_UID_PHYPLL_MODE,
	MT_SPM_SMC_UID_SET_PENDING_IRQ_INIT,

	/* call from lk load_spm */
	MT_SPM_SMC_UID_FW_INIT = 142857,
};
#define lpm_smc_spm(_lp_id, _act, _val1, _val2) ({\
		lpm_smc(MTK_SIP_MTK_LPM_CONTROL,\
				MT_LPM_SMC_USER_SPM(_lp_id),\
				_act, _val1, _val2); })

enum MT_CPU_PM_LP_SMC_UID {
	LP_CPC_COMMAND,
	IRQS_REMAIN_ALLOC,
	IRQS_REMAIN_CTRL,
	IRQS_REMAIN_IRQ,
	IRQS_REMAIN_WAKEUP_CAT,
	IRQS_REMAIN_WAKEUP_SRC,
};

enum MT_CPU_PM_CTRL {
	BUCK_MODE_CTRL,
	ARMPLL_MODE_CTRL,
	CM_IS_NOTIFIED,
};

#define lpm_smc_cpu_pm_lp(_lp_id, _act, _val1, _val2) ({\
		lpm_smc(MTK_SIP_MTK_LPM_CONTROL,\
				MT_LPM_SMC_USER_CPU_PM_LP(_lp_id),\
				_act, _val1, _val2); })

#endif
