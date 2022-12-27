// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>

#include <mtk_spm_internal.h>
#include <mtk_spm_suspend_internal.h>
#include <mtk_idle_fs/mtk_idle_sysfs.h>
#include <mtk_spm_resource_req_console.h>

/**************************************
 * Macro and Inline
 **************************************/
#define DEFINE_ATTR_RO(_name)			\
	static struct kobj_attribute _name##_attr = {	\
		.attr	= {				\
			.name = #_name,			\
			.mode = 0444,			\
		},					\
		.show	= _name##_show,			\
	}

#define DEFINE_ATTR_RW(_name)			\
	static struct kobj_attribute _name##_attr = {	\
		.attr	= {				\
			.name = #_name,			\
			.mode = 0644,			\
		},					\
		.show	= _name##_show,			\
		.store	= _name##_store,		\
	}

#define __ATTR_OF(_name)	(&_name##_attr.attr)

static char *pwr_ctrl_str[PW_MAX_COUNT] = {
	[PW_PCM_FLAGS] = "pcm_flags",
	[PW_PCM_FLAGS_CUST] = "pcm_flags_cust",
	[PW_PCM_FLAGS_CUST_SET] = "pcm_flags_cust_set",
	[PW_PCM_FLAGS_CUST_CLR] = "pcm_flags_cust_clr",
	[PW_PCM_FLAGS1] = "pcm_flags1",
	[PW_PCM_FLAGS1_CUST] = "pcm_flags1_cust",
	[PW_PCM_FLAGS1_CUST_SET] = "pcm_flags1_cust_set",
	[PW_PCM_FLAGS1_CUST_CLR] = "pcm_flags1_cust_clr",
	[PW_TIMER_VAL] = "timer_val",
	[PW_TIMER_VAL_CUST] = "timer_val_cust",
	[PW_TIMER_VAL_RAMP_EN] = "timer_val_ramp_en",
	[PW_TIMER_VAL_RAMP_EN_SEC] = "timer_val_ramp_en_sec",
	[PW_WAKE_SRC] = "wake_src",
	[PW_WAKE_SRC_CUST] = "wake_src_cust",
	[PW_WAKELOCK_TIMER_VAL] = "wakelock_timer_val",
	[PW_WDT_DISABLE] = "wdt_disable",
	/* SPM_AP_STANDBY_CON */
	[PW_WFI_OP] = "wfi_op",
	[PW_WFI_TYPE] = "wfi_type",
	[PW_MP0_CPUTOP_IDLE_MASK] = "mp0_cputop_idle_mask",
	[PW_MP1_CPUTOP_IDLE_MASK] = "mp1_cputop_idle_mask",
	[PW_MCUSYS_IDLE_MASK] = "mcusys_idle_mask",
	[PW_MM_MASK_B] = "mm_mask_b",
	[PW_MD_DDR_EN_0_DBC_EN] = "md_ddr_en_0_dbc_en",
	[PW_MD_DDR_EN_1_DBC_EN] = "md_ddr_en_1_dbc_en",
	[PW_MD_MASK_B] = "md_mask_b",
	[PW_SSPM_MASK_B] = "sspm_mask_b",
	[PW_SCP_MASK_B] = "scp_mask_b",
	[PW_SRCCLKENI_MASK_B] = "srcclkeni_mask_b",
	[PW_MD_APSRC_1_SEL] = "md_apsrc_1_sel",
	[PW_MD_APSRC_0_SEL] = "md_apsrc_0_sel",
	[PW_CONN_DDR_EN_DBC_EN] = "conn_ddr_en_dbc_en",
	[PW_CONN_MASK_B] = "conn_mask_b",
	[PW_CONN_APSRC_SEL] = "conn_apsrc_sel",
	[PW_CONN_SRCCLKENA_SEL_MASK] = "conn_srcclkena_sel_mask",
	/* SPM_SRC_REQ */
	[PW_SPM_APSRC_REQ] = "spm_apsrc_req",
	[PW_SPM_F26M_REQ] = "spm_f26m_req",
	[PW_SPM_INFRA_REQ] = "spm_infra_req",
	[PW_SPM_VRF18_REQ] = "spm_vrf18_req",
	[PW_SPM_DDREN_REQ] = "spm_ddren_req",
	[PW_SPM_RSV_SRC_REQ] = "spm_rsv_src_req",
	[PW_SPM_DDREN_2_REQ] = "spm_ddren_2_req",
	[PW_CPU_MD_DVFS_SOP_FORCE_ON] = "cpu_md_dvfs_sop_force_on",
	/* SPM_SRC_MASK */
	[PW_CSYSPWREQ_MASK] = "csyspwreq_mask",
	[PW_CCIF0_MD_EVENT_MASK_B] = "ccif0_md_event_mask_b",
	[PW_CCIF0_AP_EVENT_MASK_B] = "ccif0_ap_event_mask_b",
	[PW_CCIF1_MD_EVENT_MASK_B] = "ccif1_md_event_mask_b",
	[PW_CCIF1_AP_EVENT_MASK_B] = "ccif1_ap_event_mask_b",
	[PW_CCIF2_MD_EVENT_MASK_B] = "ccif2_md_event_mask_b",
	[PW_CCIF2_AP_EVENT_MASK_B] = "ccif2_ap_event_mask_b",
	[PW_CCIF3_MD_EVENT_MASK_B] = "ccif3_md_event_mask_b",
	[PW_CCIF3_AP_EVENT_MASK_B] = "ccif3_ap_event_mask_b",
	[PW_MD_SRCCLKENA_0_INFRA_MASK_B] = "md_srcclkena_0_infra_mask_b",
	[PW_MD_SRCCLKENA_1_INFRA_MASK_B] = "md_srcclkena_1_infra_mask_b",
	[PW_CONN_SRCCLKENA_INFRA_MASK_B] = "conn_srcclkena_infra_mask_b",
	[PW_UFS_INFRA_REQ_MASK_B] = "ufs_infra_req_mask_b",
	[PW_SRCCLKENI_INFRA_MASK_B] = "srcclkeni_infra_mask_b",
	[PW_MD_APSRC_REQ_0_INFRA_MASK_B] = "md_apsrc_req_0_infra_mask_b",
	[PW_MD_APSRC_REQ_1_INFRA_MASK_B] = "md_apsrc_req_1_infra_mask_b",
	[PW_CONN_APSRCREQ_INFRA_MASK_B] = "conn_apsrcreq_infra_mask_b",
	[PW_UFS_SRCCLKENA_MASK_B] = "ufs_srcclkena_mask_b",
	[PW_MD_VRF18_REQ_0_MASK_B] = "md_vrf18_req_0_mask_b",
	[PW_MD_VRF18_REQ_1_MASK_B] = "md_vrf18_req_1_mask_b",
	[PW_UFS_VRF18_REQ_MASK_B] = "ufs_vrf18_req_mask_b",
	[PW_GCE_VRF18_REQ_MASK_B] = "gce_vrf18_req_mask_b",
	[PW_CONN_INFRA_REQ_MASK_B] = "conn_infra_req_mask_b",
	[PW_GCE_APSRC_REQ_MASK_B] = "gce_apsrc_req_mask_b",
	[PW_DISP0_APSRC_REQ_MASK_B] = "disp0_apsrc_req_mask_b",
	[PW_DISP1_APSRC_REQ_MASK_B] = "disp1_apsrc_req_mask_b",
	[PW_MFG_REQ_MASK_B] = "mfg_req_mask_b",
	[PW_VDEC_REQ_MASK_B] = "vdec_req_mask_b",
	[PW_MCU_APSRCREQ_INFRA_MASK_B] = "mcu_apsrcreq_infra_mask_b",
	/* SPM_SRC2_MASK */
	[PW_MD_DDR_EN_0_MASK_B] = "md_ddr_en_0_mask_b",
	[PW_MD_DDR_EN_1_MASK_B] = "md_ddr_en_1_mask_b",
	[PW_CONN_DDR_EN_MASK_B] = "conn_ddr_en_mask_b",
	[PW_DDREN_SSPM_APSRC_REQ_MASK_B] = "ddren_sspm_apsrc_req_mask_b",
	[PW_DDREN_SCP_APSRC_REQ_MASK_B] = "ddren_scp_apsrc_req_mask_b",
	[PW_DISP0_DDREN_MASK_B] = "disp0_ddren_mask_b",
	[PW_DISP1_DDREN_MASK_B] = "disp1_ddren_mask_b",
	[PW_GCE_DDREN_MASK_B] = "gce_ddren_mask_b",
	[PW_DDREN_EMI_SELF_REFRESH_CH0_MASK_B] =
		"ddren_emi_self_refresh_ch0_mask_b",
	[PW_DDREN_EMI_SELF_REFRESH_CH1_MASK_B] =
		"ddren_emi_self_refresh_ch1_mask_b",
	[PW_MCU_APSRC_REQ_MASK_B] = "mcu_apsrc_req_mask_b",
	[PW_MCU_DDREN_MASK_B] = "mcu_ddren_mask_b",
	/* SPM_WAKEUP_EVENT_MASK */
	[PW_SPM_WAKEUP_EVENT_MASK] = "spm_wakeup_event_mask",
	/* SPM_WAKEUP_EVENT_EXT_MASK */
	[PW_SPM_WAKEUP_EVENT_EXT_MASK] = "spm_wakeup_event_ext_mask",
	/* SPM_SRC3_MASK */
	[PW_MD_DDR_EN_2_0_MASK_B] = "md_ddr_en_2_0_mask_b",
	[PW_MD_DDR_EN_2_1_MASK_B] = "md_ddr_en_2_1_mask_b",
	[PW_CONN_DDR_EN_2_MASK_B] = "conn_ddr_en_2_mask_b",
	[PW_DDREN2_SSPM_APSRC_REQ_MASK_B] = "ddren2_sspm_apsrc_req_mask_b",
	[PW_DDREN2_SCP_APSRC_REQ_MASK_B] = "ddren2_scp_apsrc_req_mask_b",
	[PW_DISP0_DDREN2_MASK_B] = "disp0_ddren2_mask_b",
	[PW_DISP1_DDREN2_MASK_B] = "disp1_ddren2_mask_b",
	[PW_GCE_DDREN2_MASK_B] = "gce_ddren2_mask_b",
	[PW_DDREN2_EMI_SELF_REFRESH_CH0_MASK_B] =
		"ddren2_emi_self_refresh_ch0_mask_b",
	[PW_DDREN2_EMI_SELF_REFRESH_CH1_MASK_B] =
		"ddren2_emi_self_refresh_ch1_mask_b",
	[PW_MCU_DDREN_2_MASK_B] = "mcu_ddren_2_mask_b",
	/* MP0_CPU0_WFI_EN */
	[PW_MP0_CPU0_WFI_EN] = "mp0_cpu0_wfi_en",
	/* MP0_CPU1_WFI_EN */
	[PW_MP0_CPU1_WFI_EN] = "mp0_cpu1_wfi_en",
	/* MP0_CPU2_WFI_EN */
	[PW_MP0_CPU2_WFI_EN] = "mp0_cpu2_wfi_en",
	/* MP0_CPU3_WFI_EN */
	[PW_MP0_CPU3_WFI_EN] = "mp0_cpu3_wfi_en",
	/* MP0_CPU4_WFI_EN */
	[PW_MP0_CPU4_WFI_EN] = "mp0_cpu4_wfi_en",
	/* MP0_CPU5_WFI_EN */
	[PW_MP0_CPU5_WFI_EN] = "mp0_cpu5_wfi_en",
	/* MP0_CPU6_WFI_EN */
	[PW_MP0_CPU6_WFI_EN] = "mp0_cpu6_wfi_en",
	/* MP0_CPU7_WFI_EN */
	[PW_MP0_CPU7_WFI_EN] = "mp0_cpu7_wfi_en",
};

/**************************************
 * xxx_ctrl_show Function
 **************************************/
/* code gen by spm_pwr_ctrl_atf.pl, need struct pwr_ctrl */
static ssize_t show_pwr_ctrl(int id, const struct pwr_ctrl *pwrctrl
		, char *buf, size_t buf_sz)
{
	char *p = buf;
	size_t mSize = 0;

	mSize += scnprintf(p + mSize, buf_sz - mSize,
				"pcm_flags = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
				"pcm_flags_cust = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS_CUST, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
				"pcm_flags_cust_set = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS_CUST_SET, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
				"pcm_flags_cust_clr = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS_CUST_CLR, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
				"pcm_flags1 = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS1, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
				"pcm_flags1_cust = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS1_CUST, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
				"pcm_flags1_cust_set = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS1_CUST_SET, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
				"pcm_flags1_cust_clr = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS1_CUST_CLR, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"timer_val = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_TIMER_VAL, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"timer_val_cust = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_TIMER_VAL_CUST, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"timer_val_ramp_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_TIMER_VAL_RAMP_EN, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"timer_val_ramp_en_sec = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_TIMER_VAL_RAMP_EN_SEC, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"wake_src = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_WAKE_SRC, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"wake_src_cust = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_WAKE_SRC_CUST, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"wakelock_timer_val = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_WAKELOCK_TIMER_VAL, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"wdt_disable = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_WDT_DISABLE, 0));
	/* SPM_AP_STANDBY_CON */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"wfi_op = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_WFI_OP, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"wfi_type = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_WFI_TYPE, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp0_cputop_idle_mask = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP0_CPUTOP_IDLE_MASK, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp1_cputop_idle_mask = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP1_CPUTOP_IDLE_MASK, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mcusys_idle_mask = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MCUSYS_IDLE_MASK, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mm_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MM_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_ddr_en_0_dbc_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_0_DBC_EN, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_ddr_en_1_dbc_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_1_DBC_EN, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"sspm_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SSPM_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"scp_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SCP_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"srcclkeni_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SRCCLKENI_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_apsrc_1_sel = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_APSRC_1_SEL, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_apsrc_0_sel = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_APSRC_0_SEL, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"conn_ddr_en_dbc_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CONN_DDR_EN_DBC_EN, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"conn_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CONN_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"conn_apsrc_sel = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CONN_APSRC_SEL, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"conn_srcclkena_sel_mask = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CONN_SRCCLKENA_SEL_MASK, 0));
	/* SPM_SRC_REQ */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"spm_apsrc_req = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SPM_APSRC_REQ, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"spm_f26m_req = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SPM_F26M_REQ, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"spm_infra_req = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SPM_INFRA_REQ, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"spm_vrf18_req = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SPM_VRF18_REQ, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"spm_ddren_req = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SPM_DDREN_REQ, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"spm_rsv_src_req= 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SPM_RSV_SRC_REQ, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"spm_ddren_2_req= 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SPM_DDREN_2_REQ, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"cpu_md_dvfs_sop_force_on= 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CPU_MD_DVFS_SOP_FORCE_ON, 0));
	/* SPM_SRC_MASK */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"csyspwreq_mask = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CSYSPWREQ_MASK, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ccif0_md_event_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CCIF0_MD_EVENT_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ccif0_ap_event_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CCIF0_AP_EVENT_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ccif1_md_event_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CCIF1_MD_EVENT_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ccif1_ap_event_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CCIF1_AP_EVENT_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ccif2_md_event_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CCIF2_MD_EVENT_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ccif2_ap_event_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CCIF2_AP_EVENT_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ccif3_md_event_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CCIF3_MD_EVENT_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ccif3_ap_event_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CCIF3_AP_EVENT_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_srcclkena_0_infra_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_SRCCLKENA_0_INFRA_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_srcclkena_1_infra_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_SRCCLKENA_1_INFRA_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"conn_srcclkena_infra_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CONN_SRCCLKENA_INFRA_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ufs_infra_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_UFS_INFRA_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"srcclkeni_infra_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SRCCLKENI_INFRA_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_apsrc_req_0_infra_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_APSRC_REQ_0_INFRA_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_apsrc_req_1_infra_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_APSRC_REQ_1_INFRA_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"conn_apsrcreq_infra_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CONN_APSRCREQ_INFRA_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ufs_srcclkena_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_UFS_SRCCLKENA_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_vrf18_req_0_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_VRF18_REQ_0_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_vrf18_req_1_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_VRF18_REQ_1_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ufs_vrf18_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_UFS_VRF18_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"gce_vrf18_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_GCE_VRF18_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"conn_infra_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CONN_INFRA_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"gce_apsrc_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_GCE_APSRC_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"disp0_apsrc_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DISP0_APSRC_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"disp1_apsrc_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DISP1_APSRC_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mfg_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MFG_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"vdec_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_VDEC_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mcu_apsrcreq_infra_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MCU_APSRCREQ_INFRA_MASK_B, 0));
	/* SPM_SRC2_MASK */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_ddr_en_0_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_0_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_ddr_en_1_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_1_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"conn_ddr_en_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CONN_DDR_EN_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ddren_sspm_apsrc_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DDREN_SSPM_APSRC_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ddren_scp_apsrc_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DDREN_SCP_APSRC_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"disp0_ddren_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DISP0_DDREN_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"disp1_ddren_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DISP1_DDREN_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"gce_ddren_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_GCE_DDREN_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ddren_emi_self_refresh_ch0_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DDREN_EMI_SELF_REFRESH_CH0_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ddren_emi_self_refresh_ch1_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DDREN_EMI_SELF_REFRESH_CH1_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mcu_apsrc_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MCU_APSRC_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mcu_ddren_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MCU_DDREN_MASK_B, 0));
	/* SPM_WAKEUP_EVENT_MASK */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"spm_wakeup_event_mask = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SPM_WAKEUP_EVENT_MASK, 0));
	/* SPM_WAKEUP_EVENT_EXT_MASK */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"spm_wakeup_event_ext_mask = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_SPM_WAKEUP_EVENT_EXT_MASK, 0));
	/* SPM_SRC3_MASK */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_ddr_en_2_0_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_2_0_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"md_ddr_en_2_1_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_2_1_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"conn_ddr_en_2_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_CONN_DDR_EN_2_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ddren2_sspm_apsrc_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DDREN2_SSPM_APSRC_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ddren2_scp_apsrc_req_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DDREN2_SCP_APSRC_REQ_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"disp0_ddren2_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DISP0_DDREN2_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"disp1_ddren2_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DISP1_DDREN2_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"gce_ddren2_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_GCE_DDREN2_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ddren2_emi_self_refresh_ch0_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DDREN2_EMI_SELF_REFRESH_CH0_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"ddren2_emi_self_refresh_ch1_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_DDREN2_EMI_SELF_REFRESH_CH1_MASK_B, 0));
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mcu_ddren_2_mask_b = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MCU_DDREN_2_MASK_B, 0));
	/* MP0_CPU0_WFI_EN */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp0_cpu0_wfi_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP0_CPU1_WFI_EN, 0));
	/* MP0_CPU1_WFI_EN */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp0_cpu1_wfi_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP0_CPU1_WFI_EN, 0));
	/* MP0_CPU2_WFI_EN */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp0_cpu2_wfi_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP0_CPU2_WFI_EN, 0));
	/* MP0_CPU3_WFI_EN */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp0_cpu3_wfi_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP0_CPU3_WFI_EN, 0));
	/* MP0_CPU4_WFI_EN */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp0_cpu4_wfi_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP0_CPU4_WFI_EN, 0));
	/* MP0_CPU5_WFI_EN */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp0_cpu5_wfi_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP0_CPU5_WFI_EN, 0));
	/* MP0_CPU6_WFI_EN */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp0_cpu6_wfi_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP0_CPU6_WFI_EN, 0));
	/* MP0_CPU7_WFI_EN */
	mSize += scnprintf(p + mSize, buf_sz - mSize,
			"mp0_cpu7_wfi_en = 0x%zx\n",
			SMC_CALL(GET_PWR_CTRL_ARGS,
				id, PW_MP0_CPU7_WFI_EN, 0));

	WARN_ON(buf_sz - mSize <= 0);

	return mSize;
}

static ssize_t suspend_ctrl_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return show_pwr_ctrl(SPM_PWR_CTRL_SUSPEND, __spm_suspend.pwrctrl
			, buf, get_mtk_lp_kernfs_bufsz_max());
}

static ssize_t IdleDram_ctrl_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return show_pwr_ctrl(SPM_PWR_CTRL_IDLE_DRAM, &pwrctrl_dram
		, buf, get_mtk_lp_kernfs_bufsz_max());
}

static ssize_t IdleSyspll_ctrl_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return show_pwr_ctrl(SPM_PWR_CTRL_IDLE_SYSPLL, &pwrctrl_syspll
		, buf, get_mtk_lp_kernfs_bufsz_max());
}

static ssize_t IdleBus26m_ctrl_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return show_pwr_ctrl(SPM_PWR_CTRL_IDLE_BUS26M, &pwrctrl_bus26m
		, buf, get_mtk_lp_kernfs_bufsz_max());
}

static ssize_t spmfw_version_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return get_spmfw_version(buf, PAGE_SIZE, NULL);
}

/**************************************
 * xxx_ctrl_store Function
 **************************************/
/* code gen by spm_pwr_ctrl_atf.pl, need struct pwr_ctrl */
static ssize_t store_pwr_ctrl(int id, struct pwr_ctrl *pwrctrl,
	const char *buf, size_t count)
{
	u32 val;
	char cmd[64];

	if (sscanf(buf, "%63s %x", cmd, &val) != 2)
		return -EPERM;

	pr_info("[SPM] pwr_ctrl: cmd = %s, val = 0x%x\n", cmd, val);


	if (!strcmp(cmd,
		pwr_ctrl_str[PW_PCM_FLAGS])) {
		pwrctrl->pcm_flags = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_PCM_FLAGS_CUST])) {
		pwrctrl->pcm_flags_cust = val;
		SMC_CALL(PWR_CTRL_ARGS,
					id, PW_PCM_FLAGS_CUST, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_PCM_FLAGS_CUST_SET])) {
		pwrctrl->pcm_flags_cust_set = val;
		SMC_CALL(PWR_CTRL_ARGS,
					id, PW_PCM_FLAGS_CUST_SET, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_PCM_FLAGS_CUST_CLR])) {
		pwrctrl->pcm_flags_cust_clr = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS_CUST_CLR, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_PCM_FLAGS1])) {
		pwrctrl->pcm_flags1 = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS1, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_PCM_FLAGS1_CUST])) {
		pwrctrl->pcm_flags1_cust = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS1_CUST, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_PCM_FLAGS1_CUST_SET])) {
		pwrctrl->pcm_flags1_cust_set = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS1_CUST_SET, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_PCM_FLAGS1_CUST_CLR])) {
		pwrctrl->pcm_flags1_cust_clr = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_PCM_FLAGS1_CUST_CLR, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_TIMER_VAL])) {
		pwrctrl->timer_val = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_TIMER_VAL, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_TIMER_VAL_CUST])) {
		pwrctrl->timer_val_cust = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_TIMER_VAL_CUST, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_TIMER_VAL_RAMP_EN])) {
		pwrctrl->timer_val_ramp_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_TIMER_VAL_RAMP_EN, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_TIMER_VAL_RAMP_EN_SEC])) {
		pwrctrl->timer_val_ramp_en_sec = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_TIMER_VAL_RAMP_EN_SEC, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_WAKE_SRC])) {
		pwrctrl->wake_src = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_WAKE_SRC, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_WAKE_SRC_CUST])) {
		pwrctrl->wake_src_cust = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_WAKE_SRC_CUST, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_WAKELOCK_TIMER_VAL])) {
		pwrctrl->wakelock_timer_val = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_WAKELOCK_TIMER_VAL, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_WDT_DISABLE])) {
		pwrctrl->wdt_disable = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_WDT_DISABLE, val);
	 /* SPM_AP_STANDBY_CON */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_WFI_OP])) {
		pwrctrl->wfi_op = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_WFI_OP, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_WFI_TYPE])) {
		pwrctrl->wfi_op = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_WFI_TYPE, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP0_CPUTOP_IDLE_MASK])) {
		pwrctrl->mp0_cputop_idle_mask = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP0_CPUTOP_IDLE_MASK, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP1_CPUTOP_IDLE_MASK])) {
		pwrctrl->mp1_cputop_idle_mask = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP1_CPUTOP_IDLE_MASK, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MCUSYS_IDLE_MASK])) {
		pwrctrl->mcusys_idle_mask = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MCUSYS_IDLE_MASK, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MM_MASK_B])) {
		pwrctrl->mm_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MM_MASK_B, val);
	}  else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_DDR_EN_0_DBC_EN])) {
		pwrctrl->md_ddr_en_0_dbc_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_0_DBC_EN, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_DDR_EN_1_DBC_EN])) {
		pwrctrl->md_ddr_en_1_dbc_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_1_DBC_EN, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_MASK_B])) {
		pwrctrl->md_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SSPM_MASK_B])) {
		pwrctrl->sspm_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_SSPM_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SCP_MASK_B])) {
		pwrctrl->scp_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_SCP_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SRCCLKENI_MASK_B])) {
		pwrctrl->srcclkeni_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_SRCCLKENI_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_APSRC_1_SEL])) {
		pwrctrl->md_apsrc_1_sel = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_APSRC_1_SEL, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_APSRC_0_SEL])) {
		pwrctrl->md_apsrc_0_sel = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_APSRC_0_SEL, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CONN_DDR_EN_DBC_EN])) {
		pwrctrl->conn_ddr_en_dbc_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CONN_DDR_EN_DBC_EN, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CONN_MASK_B])) {
		pwrctrl->conn_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CONN_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CONN_APSRC_SEL])) {
		pwrctrl->conn_apsrc_sel = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CONN_APSRC_SEL, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CONN_SRCCLKENA_SEL_MASK])) {
		pwrctrl->conn_apsrc_sel = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CONN_SRCCLKENA_SEL_MASK, val);
	/* SPM_SRC_REQ */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SPM_APSRC_REQ])) {
		unsigned int req = (val == 0) ?
			SPM_RESOURCE_CONSOLE_RELEASE : SPM_RESOURCE_CONSOLE_REQ;
		if (spm_resource_req_console_by_id(id, req
			, _RES_MASK(MTK_SPM_RES_EX_DRAM_S1)) != 0) {
			pwrctrl->spm_apsrc_req = val;
			SMC_CALL(PWR_CTRL_ARGS,
					id, PW_SPM_APSRC_REQ, val);
		}
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SPM_F26M_REQ])) {
		unsigned int req = (val == 0) ?
			SPM_RESOURCE_CONSOLE_RELEASE : SPM_RESOURCE_CONSOLE_REQ;
		if (spm_resource_req_console_by_id(id, req
			, _RES_MASK(MTK_SPM_RES_EX_26M)) != 0) {
			pwrctrl->spm_f26m_req = val;
			SMC_CALL(PWR_CTRL_ARGS,
					id, PW_SPM_F26M_REQ, val);
		}
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SPM_INFRA_REQ])) {
		unsigned int req = (val == 0) ?
			SPM_RESOURCE_CONSOLE_RELEASE : SPM_RESOURCE_CONSOLE_REQ;
		if (spm_resource_req_console_by_id(id, req
			, _RES_MASK(MTK_SPM_RES_EX_AXI_BUS)) != 0) {
			pwrctrl->spm_infra_req = val;
			SMC_CALL(PWR_CTRL_ARGS,
					id, PW_SPM_INFRA_REQ, val);
		}
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SPM_VRF18_REQ])) {
		unsigned int req = (val == 0) ?
			SPM_RESOURCE_CONSOLE_RELEASE : SPM_RESOURCE_CONSOLE_REQ;
		if (spm_resource_req_console_by_id(id, req
			, _RES_MASK(MTK_SPM_RES_EX_MAINPLL)) != 0) {
			pwrctrl->spm_vrf18_req = val;
			SMC_CALL(PWR_CTRL_ARGS,
					id, PW_SPM_VRF18_REQ, val);
		}
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SPM_DDREN_REQ])) {
		unsigned int req = (val == 0) ?
			SPM_RESOURCE_CONSOLE_RELEASE : SPM_RESOURCE_CONSOLE_REQ;
		if (spm_resource_req_console_by_id(id, req
			, _RES_MASK(MTK_SPM_RES_EX_DRAM_S0)) != 0) {
			pwrctrl->spm_ddren_req = val;
			SMC_CALL(PWR_CTRL_ARGS,
					id, PW_SPM_DDREN_REQ, val);
		}
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SPM_RSV_SRC_REQ])) {
		pwrctrl->spm_rsv_src_req = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_SPM_RSV_SRC_REQ, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SPM_DDREN_2_REQ])) {
		pwrctrl->spm_ddren_2_req = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_SPM_DDREN_2_REQ, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CPU_MD_DVFS_SOP_FORCE_ON])) {
		pwrctrl->cpu_md_dvfs_sop_force_on = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CPU_MD_DVFS_SOP_FORCE_ON, val);
	/* SPM_SRC_MASK */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CSYSPWREQ_MASK])) {
		pwrctrl->csyspwreq_mask = val;
		SMC_CALL(PWR_CTRL_ARGS,
			id, PW_CSYSPWREQ_MASK, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CCIF0_MD_EVENT_MASK_B])) {
		pwrctrl->ccif0_md_event_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CCIF0_MD_EVENT_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CCIF0_AP_EVENT_MASK_B])) {
		pwrctrl->ccif0_ap_event_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CCIF0_AP_EVENT_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CCIF1_MD_EVENT_MASK_B])) {
		pwrctrl->ccif1_md_event_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CCIF1_MD_EVENT_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CCIF1_AP_EVENT_MASK_B])) {
		pwrctrl->ccif1_ap_event_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CCIF1_AP_EVENT_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CCIF2_MD_EVENT_MASK_B])) {
		pwrctrl->ccif2_md_event_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CCIF2_MD_EVENT_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CCIF2_AP_EVENT_MASK_B])) {
		pwrctrl->ccif2_ap_event_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CCIF2_AP_EVENT_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CCIF3_MD_EVENT_MASK_B])) {
		pwrctrl->ccif3_md_event_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CCIF3_MD_EVENT_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CCIF3_AP_EVENT_MASK_B])) {
		pwrctrl->ccif3_ap_event_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CCIF3_AP_EVENT_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_SRCCLKENA_0_INFRA_MASK_B])) {
		pwrctrl->md_srcclkena_0_infra_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_SRCCLKENA_0_INFRA_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_SRCCLKENA_1_INFRA_MASK_B])) {
		pwrctrl->md_srcclkena_1_infra_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_SRCCLKENA_1_INFRA_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CONN_SRCCLKENA_INFRA_MASK_B])) {
		pwrctrl->conn_srcclkena_infra_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CONN_SRCCLKENA_INFRA_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_UFS_INFRA_REQ_MASK_B])) {
		pwrctrl->ufs_infra_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_UFS_INFRA_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SRCCLKENI_INFRA_MASK_B])) {
		pwrctrl->srcclkeni_infra_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_SRCCLKENI_INFRA_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_APSRC_REQ_0_INFRA_MASK_B])) {
		pwrctrl->md_apsrc_req_0_infra_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_APSRC_REQ_0_INFRA_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_APSRC_REQ_1_INFRA_MASK_B])) {
		pwrctrl->md_apsrc_req_1_infra_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_APSRC_REQ_1_INFRA_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CONN_APSRCREQ_INFRA_MASK_B])) {
		pwrctrl->conn_apsrcreq_infra_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CONN_APSRCREQ_INFRA_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_UFS_SRCCLKENA_MASK_B])) {
		pwrctrl->ufs_srcclkena_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_UFS_SRCCLKENA_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_VRF18_REQ_0_MASK_B])) {
		pwrctrl->md_vrf18_req_0_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_VRF18_REQ_0_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_VRF18_REQ_1_MASK_B])) {
		pwrctrl->md_vrf18_req_1_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_VRF18_REQ_1_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_UFS_VRF18_REQ_MASK_B])) {
		pwrctrl->ufs_vrf18_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_UFS_VRF18_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_GCE_VRF18_REQ_MASK_B])) {
		pwrctrl->gce_vrf18_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_GCE_VRF18_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CONN_INFRA_REQ_MASK_B])) {
		pwrctrl->conn_infra_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CONN_INFRA_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_GCE_APSRC_REQ_MASK_B])) {
		pwrctrl->gce_apsrc_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_GCE_APSRC_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DISP0_APSRC_REQ_MASK_B])) {
		pwrctrl->disp0_apsrc_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DISP0_APSRC_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DISP1_APSRC_REQ_MASK_B])) {
		pwrctrl->disp1_apsrc_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DISP1_APSRC_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MFG_REQ_MASK_B])) {
		pwrctrl->mfg_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MFG_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_VDEC_REQ_MASK_B])) {
		pwrctrl->vdec_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_VDEC_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MCU_APSRCREQ_INFRA_MASK_B])) {
		pwrctrl->vdec_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MCU_APSRCREQ_INFRA_MASK_B, val);
	/* SPM_SRC2_MASK */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_DDR_EN_0_MASK_B])) {
		pwrctrl->md_ddr_en_0_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_0_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_DDR_EN_1_MASK_B])) {
		pwrctrl->md_ddr_en_1_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_1_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CONN_DDR_EN_MASK_B])) {
		pwrctrl->conn_ddr_en_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CONN_DDR_EN_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DDREN_SSPM_APSRC_REQ_MASK_B])) {
		pwrctrl->ddren_sspm_apsrc_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DDREN_SSPM_APSRC_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DDREN_SCP_APSRC_REQ_MASK_B])) {
		pwrctrl->ddren_scp_apsrc_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DDREN_SCP_APSRC_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DISP0_DDREN_MASK_B])) {
		pwrctrl->disp0_ddren_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DISP0_DDREN_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DISP1_DDREN_MASK_B])) {
		pwrctrl->disp1_ddren_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DISP1_DDREN_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_GCE_DDREN_MASK_B])) {
		pwrctrl->gce_ddren_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_GCE_DDREN_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DDREN_EMI_SELF_REFRESH_CH0_MASK_B])) {
		pwrctrl->ddren_emi_self_refresh_ch0_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DDREN_EMI_SELF_REFRESH_CH0_MASK_B,
					val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DDREN_EMI_SELF_REFRESH_CH1_MASK_B])) {
		pwrctrl->ddren_emi_self_refresh_ch1_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DDREN_EMI_SELF_REFRESH_CH1_MASK_B,
					val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MCU_APSRC_REQ_MASK_B])) {
		pwrctrl->ddren_emi_self_refresh_ch0_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MCU_APSRC_REQ_MASK_B,
					val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MCU_DDREN_MASK_B])) {
		pwrctrl->ddren_emi_self_refresh_ch1_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MCU_DDREN_MASK_B,
					val);
	/* SPM_WAKEUP_EVENT_MASK */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SPM_WAKEUP_EVENT_MASK])) {
		pwrctrl->spm_wakeup_event_mask = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_SPM_WAKEUP_EVENT_MASK, val);
	/* SPM_WAKEUP_EVENT_EXT_MASK */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_SPM_WAKEUP_EVENT_EXT_MASK])) {
		pwrctrl->spm_wakeup_event_ext_mask = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_SPM_WAKEUP_EVENT_EXT_MASK, val);
	/* SPM_SRC3_MASK */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_DDR_EN_2_0_MASK_B])) {
		pwrctrl->md_ddr_en_2_0_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_2_0_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MD_DDR_EN_2_1_MASK_B])) {
		pwrctrl->md_ddr_en_2_1_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MD_DDR_EN_2_1_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_CONN_DDR_EN_2_MASK_B])) {
		pwrctrl->conn_ddr_en_2_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_CONN_DDR_EN_2_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DDREN2_SSPM_APSRC_REQ_MASK_B])) {
		pwrctrl->ddren2_sspm_apsrc_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DDREN2_SSPM_APSRC_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DDREN2_SCP_APSRC_REQ_MASK_B])) {
		pwrctrl->ddren2_scp_apsrc_req_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DDREN2_SCP_APSRC_REQ_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DISP0_DDREN2_MASK_B])) {
		pwrctrl->disp0_ddren2_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DISP0_DDREN2_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DISP1_DDREN2_MASK_B])) {
		pwrctrl->disp1_ddren2_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DISP1_DDREN2_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_GCE_DDREN2_MASK_B])) {
		pwrctrl->gce_ddren2_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_GCE_DDREN2_MASK_B, val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DDREN2_EMI_SELF_REFRESH_CH0_MASK_B])) {
		pwrctrl->ddren2_emi_self_refresh_ch0_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DDREN2_EMI_SELF_REFRESH_CH0_MASK_B,
					val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_DDREN2_EMI_SELF_REFRESH_CH1_MASK_B])) {
		pwrctrl->ddren2_emi_self_refresh_ch1_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_DDREN2_EMI_SELF_REFRESH_CH1_MASK_B,
					val);
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MCU_DDREN_2_MASK_B])) {
		pwrctrl->ddren2_emi_self_refresh_ch1_mask_b = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MCU_DDREN_2_MASK_B,
					val);
	/* MP0_CPU0_WFI_EN */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP0_CPU0_WFI_EN])) {
		pwrctrl->mp0_cpu0_wfi_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP0_CPU0_WFI_EN, val);
	/* MP0_CPU1_WFI_EN */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP0_CPU1_WFI_EN])) {
		pwrctrl->mp0_cpu1_wfi_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP0_CPU1_WFI_EN, val);
	/* MP0_CPU2_WFI_EN */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP0_CPU2_WFI_EN])) {
		pwrctrl->mp0_cpu2_wfi_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP0_CPU2_WFI_EN, val);
	/* MP0_CPU3_WFI_EN */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP0_CPU3_WFI_EN])) {
		pwrctrl->mp0_cpu3_wfi_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP0_CPU3_WFI_EN, val);
	/* MP0_CPU4_WFI_EN */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP0_CPU4_WFI_EN])) {
		pwrctrl->mp0_cpu4_wfi_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP0_CPU4_WFI_EN, val);
	/* MP0_CPU5_WFI_EN */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP0_CPU5_WFI_EN])) {
		pwrctrl->mp0_cpu5_wfi_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP0_CPU5_WFI_EN, val);
	/* MP0_CPU6_WFI_EN */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP0_CPU6_WFI_EN])) {
		pwrctrl->mp0_cpu6_wfi_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP0_CPU6_WFI_EN, val);
	/* MP0_CPU7_WFI_EN */
	} else if (!strcmp(cmd,
			pwr_ctrl_str[PW_MP0_CPU7_WFI_EN])) {
		pwrctrl->mp0_cpu7_wfi_en = val;
		SMC_CALL(PWR_CTRL_ARGS,
				id, PW_MP0_CPU7_WFI_EN, val);
	}

	return count;
}

static ssize_t suspend_ctrl_store(struct kobject *kobj,
	struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	return store_pwr_ctrl(SPM_PWR_CTRL_SUSPEND, __spm_suspend.pwrctrl,
		buf, count);
}

static ssize_t IdleDram_ctrl_store(struct kobject *kobj,
	struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	return store_pwr_ctrl(SPM_PWR_CTRL_IDLE_DRAM
		, &pwrctrl_dram, buf, count);
}

static ssize_t IdleSyspll_ctrl_store(struct kobject *kobj,
	struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	return store_pwr_ctrl(SPM_PWR_CTRL_IDLE_SYSPLL
		, &pwrctrl_syspll, buf, count);
}

static ssize_t IdleBus26m_ctrl_store(struct kobject *kobj,
	struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	return store_pwr_ctrl(SPM_PWR_CTRL_IDLE_BUS26M
		, &pwrctrl_bus26m, buf, count);
}

static ssize_t spmfw_version_store(struct kobject *kobj,
	struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	return 0;
}

/**************************************
 * fm_suspend Function
 **************************************/
static ssize_t fm_suspend_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *p = buf;

	WARN_ON(p - buf >= PAGE_SIZE);
	return p - buf;
}

/**************************************
 * Init Function
 **************************************/
DEFINE_ATTR_RW(suspend_ctrl);
DEFINE_ATTR_RW(IdleDram_ctrl);
DEFINE_ATTR_RW(IdleSyspll_ctrl);
DEFINE_ATTR_RW(IdleBus26m_ctrl);
DEFINE_ATTR_RW(spmfw_version);
DEFINE_ATTR_RO(fm_suspend);

static struct attribute *spm_attrs[] = {
	/* for spm_lp_scen.pwrctrl */
	__ATTR_OF(suspend_ctrl),
	__ATTR_OF(IdleDram_ctrl),
	__ATTR_OF(IdleSyspll_ctrl),
	__ATTR_OF(IdleBus26m_ctrl),
	__ATTR_OF(spmfw_version),
	__ATTR_OF(fm_suspend),

	/* must */
	NULL,
};

static struct attribute_group spm_attr_group = {
	.name = "spm",
	.attrs = spm_attrs,
};

int spm_fs_init(void)
{
	int r;

	/* create /sys/power/spm/xxx */
	r = mtk_idle_sysfs_power_create_group(&spm_attr_group);
	if (r)
		pr_info("[SPM] FAILED TO CREATE /sys/power/spm (%d)\n", r);

	return r;
}

MODULE_DESCRIPTION("SPM-FS Driver v0.1");
MODULE_LICENSE("GPL");
