/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015 MediaTek Inc.
 */

#ifndef __MBOOT_PARAMS_H__
#define __MBOOT_PARAMS_H__

#include <linux/console.h>
#include <linux/pstore.h>

enum AEE_FIQ_STEP_NUM {
	AEE_FIQ_STEP_COMMON_DIE_START = 64,
	AEE_FIQ_STEP_COMMON_DIE_LOCK,
	AEE_FIQ_STEP_COMMON_DIE_KASLR,
	AEE_FIQ_STEP_COMMON_DIE_SCP,
	AEE_FIQ_STEP_COMMON_DIE_TRACE,
	AEE_FIQ_STEP_COMMON_DIE_EMISC,
	AEE_FIQ_STEP_COMMON_DIE_CS,
	AEE_FIQ_STEP_COMMON_DIE_DONE
};

enum AEE_EXP_TYPE_NUM {
	AEE_EXP_TYPE_HWT = 1,
	AEE_EXP_TYPE_KE = 2,
	AEE_EXP_TYPE_NESTED_PANIC = 3,
	AEE_EXP_TYPE_SMART_RESET = 4,
	AEE_EXP_TYPE_HANG_DETECT = 5,
	AEE_EXP_TYPE_BL33_CRASH = 6,
	AEE_EXP_TYPE_BL2_EXT_CRASH = 7,
	AEE_EXP_TYPE_AEE_LK_CRASH = 8,
	AEE_EXP_TYPE_TFA_CRASH = 9,
	AEE_EXP_TYPE_MAX_NUM = 16,
};

#if IS_ENABLED(CONFIG_MTK_AEE_IPANIC)
extern void aee_rr_rec_clk(int id, u32 val);
extern int aee_rr_reboot_reason_show(struct seq_file *m, void *v);
extern int aee_rr_last_fiq_step(void);
extern void aee_rr_rec_exp_type(unsigned int type);
extern unsigned int aee_rr_curr_exp_type(void);
extern void aee_rr_rec_kick(uint32_t kick_bit);
extern void aee_rr_rec_check(uint32_t check_bit);
extern void aee_rr_rec_scp(void);
extern void aee_rr_rec_kaslr_offset(u64 value64);
extern void aee_rr_rec_cpu_dvfs_vproc_big(u8 val);
extern void aee_rr_rec_cpu_dvfs_vproc_little(u8 val);
extern void aee_rr_rec_cpu_dvfs_oppidx(u8 val);
extern void aee_rr_rec_cpu_dvfs_cci_oppidx(u8 val);
extern void aee_rr_rec_cpu_dvfs_status(u8 val);
extern void aee_rr_rec_cpu_dvfs_step(u8 val);
extern void aee_rr_rec_cpu_dvfs_cb(u8 val);
extern void aee_rr_rec_cpufreq_cb(u8 val);
extern u8 aee_rr_curr_cpu_dvfs_oppidx(void);
extern u8 aee_rr_curr_cpu_dvfs_cci_oppidx(void);
extern u8 aee_rr_curr_cpu_dvfs_status(void);
extern u8 aee_rr_curr_cpu_dvfs_step(void);
extern u8 aee_rr_curr_cpu_dvfs_cb(void);
extern u8 aee_rr_curr_cpufreq_cb(void);
extern void aee_rr_rec_ptp_devinfo_0(u32 val);
extern void aee_rr_rec_ptp_devinfo_1(u32 val);
extern void aee_rr_rec_ptp_devinfo_2(u32 val);
extern void aee_rr_rec_ptp_devinfo_3(u32 val);
extern void aee_rr_rec_ptp_devinfo_4(u32 val);
extern void aee_rr_rec_ptp_devinfo_5(u32 val);
extern void aee_rr_rec_ptp_devinfo_6(u32 val);
extern void aee_rr_rec_ptp_devinfo_7(u32 val);
extern void aee_rr_rec_ptp_e0(u32 val);
extern void aee_rr_rec_ptp_e1(u32 val);
extern void aee_rr_rec_ptp_e2(u32 val);
extern void aee_rr_rec_ptp_e3(u32 val);
extern void aee_rr_rec_ptp_e4(u32 val);
extern void aee_rr_rec_ptp_e5(u32 val);
extern void aee_rr_rec_ptp_e6(u32 val);
extern void aee_rr_rec_ptp_e7(u32 val);
extern void aee_rr_rec_ptp_e8(u32 val);
extern void aee_rr_rec_ptp_e9(u32 val);
extern void aee_rr_rec_ptp_e10(u32 val);
extern void aee_rr_rec_ptp_e11(u32 val);
extern void aee_rr_rec_ptp_vboot(u64 val);
extern void aee_rr_rec_ptp_cpu_big_volt(u64 val);
extern void aee_rr_rec_ptp_cpu_big_volt_1(u64 val);
extern void aee_rr_rec_ptp_cpu_big_volt_2(u64 val);
extern void aee_rr_rec_ptp_cpu_big_volt_3(u64 val);
extern void aee_rr_rec_ptp_gpu_volt(u64 val);
extern void aee_rr_rec_ptp_gpu_volt_1(u64 val);
extern void aee_rr_rec_ptp_gpu_volt_2(u64 val);
extern void aee_rr_rec_ptp_gpu_volt_3(u64 val);
extern void aee_rr_rec_ptp_cpu_little_volt(u64 val);
extern void aee_rr_rec_ptp_cpu_little_volt_1(u64 val);
extern void aee_rr_rec_ptp_cpu_little_volt_2(u64 val);
extern void aee_rr_rec_ptp_cpu_little_volt_3(u64 val);
extern void aee_rr_rec_ptp_cpu_2_little_volt(u64 val);
extern void aee_rr_rec_ptp_cpu_2_little_volt_1(u64 val);
extern void aee_rr_rec_ptp_cpu_2_little_volt_2(u64 val);
extern void aee_rr_rec_ptp_cpu_2_little_volt_3(u64 val);
extern void aee_rr_rec_ptp_cpu_cci_volt(u64 val);
extern void aee_rr_rec_ptp_cpu_cci_volt_1(u64 val);
extern void aee_rr_rec_ptp_cpu_cci_volt_2(u64 val);
extern void aee_rr_rec_ptp_cpu_cci_volt_3(u64 val);
extern void aee_rr_rec_ptp_temp(u64 val);
extern void aee_rr_rec_ptp_status(u8 val);
extern void aee_rr_rec_eem_pi_offset(u8 val);
extern u32 aee_rr_curr_ptp_devinfo_0(void);
extern u32 aee_rr_curr_ptp_devinfo_1(void);
extern u32 aee_rr_curr_ptp_devinfo_2(void);
extern u32 aee_rr_curr_ptp_devinfo_3(void);
extern u32 aee_rr_curr_ptp_devinfo_4(void);
extern u32 aee_rr_curr_ptp_devinfo_5(void);
extern u32 aee_rr_curr_ptp_devinfo_6(void);
extern u32 aee_rr_curr_ptp_devinfo_7(void);
extern u32 aee_rr_curr_ptp_e0(void);
extern u32 aee_rr_curr_ptp_e1(void);
extern u32 aee_rr_curr_ptp_e2(void);
extern u32 aee_rr_curr_ptp_e3(void);
extern u32 aee_rr_curr_ptp_e4(void);
extern u32 aee_rr_curr_ptp_e5(void);
extern u32 aee_rr_curr_ptp_e6(void);
extern u32 aee_rr_curr_ptp_e7(void);
extern u32 aee_rr_curr_ptp_e8(void);
extern u32 aee_rr_curr_ptp_e9(void);
extern u32 aee_rr_curr_ptp_e10(void);
extern u32 aee_rr_curr_ptp_e11(void);
extern u64 aee_rr_curr_ptp_vboot(void);
extern u64 aee_rr_curr_ptp_cpu_big_volt(void);
extern u64 aee_rr_curr_ptp_cpu_big_volt_1(void);
extern u64 aee_rr_curr_ptp_cpu_big_volt_2(void);
extern u64 aee_rr_curr_ptp_cpu_big_volt_3(void);
extern u64 aee_rr_curr_ptp_gpu_volt(void);
extern u64 aee_rr_curr_ptp_gpu_volt_1(void);
extern u64 aee_rr_curr_ptp_gpu_volt_2(void);
extern u64 aee_rr_curr_ptp_gpu_volt_3(void);
extern u64 aee_rr_curr_ptp_cpu_little_volt(void);
extern u64 aee_rr_curr_ptp_cpu_little_volt_1(void);
extern u64 aee_rr_curr_ptp_cpu_little_volt_2(void);
extern u64 aee_rr_curr_ptp_cpu_little_volt_3(void);
extern u64 aee_rr_curr_ptp_cpu_2_little_volt(void);
extern u64 aee_rr_curr_ptp_cpu_2_little_volt_1(void);
extern u64 aee_rr_curr_ptp_cpu_2_little_volt_2(void);
extern u64 aee_rr_curr_ptp_cpu_2_little_volt_3(void);
extern u64 aee_rr_curr_ptp_cpu_cci_volt(void);
extern u64 aee_rr_curr_ptp_cpu_cci_volt_1(void);
extern u64 aee_rr_curr_ptp_cpu_cci_volt_2(void);
extern u64 aee_rr_curr_ptp_cpu_cci_volt_3(void);
extern u64 aee_rr_curr_ptp_temp(void);
extern u8 aee_rr_curr_ptp_status(void);
extern unsigned long *aee_rr_rec_mtk_cpuidle_footprint_va(void);
extern unsigned long *aee_rr_rec_mtk_cpuidle_footprint_pa(void);
extern void aee_rr_rec_sodi3_val(u32 val);
extern u32 aee_rr_curr_sodi3_val(void);
extern void aee_rr_rec_sodi_val(u32 val);
extern u32 aee_rr_curr_sodi_val(void);
extern void aee_rr_rec_deepidle_val(u32 val);
extern u32 aee_rr_curr_deepidle_val(void);
extern void aee_rr_rec_spm_suspend_val(u32 val);
extern u32 aee_rr_curr_spm_suspend_val(void);
extern void aee_rr_rec_vcore_dvfs_status(u32 val);
extern u32 aee_rr_curr_vcore_dvfs_status(void);
extern unsigned int *aee_rr_rec_mcdi_wfi(void);
extern void aee_rr_rec_mcdi_val(int id, u32 val);
extern void aee_rr_rec_vcore_dvfs_opp(u32 val);
extern u32 aee_rr_curr_vcore_dvfs_opp(void);
extern void aee_rr_rec_ocp_target_limit(int id, u32 val);
extern u32 aee_rr_curr_ocp_target_limit(int id);
extern void aee_rr_rec_ocp_enable(u8 val);
extern u8 aee_rr_curr_ocp_enable(void);
extern void aee_rr_rec_ppm_cluster_limit(int id, u32 val);
extern void aee_rr_rec_ppm_step(u8 val);
extern void aee_rr_rec_ppm_cur_state(u8 val);
extern void aee_rr_rec_ppm_min_pwr_bgt(u32 val);
extern void aee_rr_rec_ppm_policy_mask(u32 val);
extern void aee_rr_rec_ppm_waiting_for_pbm(u8 val);
extern void aee_rr_rec_gpu_dvfs_vgpu(u8 val);
extern u8 aee_rr_curr_gpu_dvfs_vgpu(void);
extern void aee_rr_rec_gpu_dvfs_oppidx(u8 val);
extern void aee_rr_rec_gpu_dvfs_status(u8 val);
extern u8 aee_rr_curr_gpu_dvfs_status(void);
extern void aee_rr_rec_gpu_dvfs_power_count(int val);
extern void aee_rr_rec_hang_detect_timeout_count(unsigned int val);
extern int aee_rr_curr_fiq_step(void);
extern void aee_rr_rec_fiq_step(u8 i);
extern void aee_rr_rec_last_irq_enter(int cpu, int irq, u64 j);
extern void aee_rr_rec_last_irq_exit(int cpu, int irq, u64 j);
extern void aee_rr_rec_hotplug_footprint(int cpu, u8 fp);
extern void aee_rr_rec_hotplug_cpu_event(u8 val);
extern void aee_rr_rec_hotplug_cb_index(u8 val);
extern void aee_rr_rec_hotplug_cb_fp(unsigned long val);
extern void aee_rr_rec_hotplug_cb_times(unsigned long val);
extern void aee_rr_rec_hps_cb_enter_times(unsigned long long val);
extern void aee_rr_rec_hps_cb_cpu_bitmask(unsigned int val);
extern void aee_rr_rec_hps_cb_footprint(unsigned int val);
extern void aee_rr_rec_hps_cb_fp_times(unsigned long long val);
extern void aee_rr_rec_last_init_func(unsigned long val);
extern void aee_rr_rec_last_sync_func(unsigned long val);
extern void aee_rr_rec_last_async_func(unsigned long val);
extern void aee_rr_rec_set_bit_pmic_ext_buck(int bit, int loc);
extern int aee_rr_init_thermal_temp(int num);
extern int aee_rr_rec_thermal_temp(int index, s8 val);
extern void aee_rr_rec_thermal_status(u8 val);
extern void aee_rr_rec_thermal_ATM_status(u8 val);
extern void aee_rr_rec_thermal_ktime(u64 val);
extern s8 aee_rr_curr_thermal_temp(int index);
extern u8 aee_rr_curr_thermal_status(void);
extern u8 aee_rr_curr_thermal_ATM_status(void);
extern u64 aee_rr_curr_thermal_ktime(void);
extern void aee_rr_rec_cpu_caller(u32 val);
extern void aee_rr_rec_cpu_callee(u32 val);
extern void aee_rr_rec_cpu_up_prepare_ktime(u64 val);
extern void aee_rr_rec_cpu_starting_ktime(u64 val);
extern void aee_rr_rec_cpu_online_ktime(u64 val);
extern void aee_rr_rec_cpu_down_prepare_ktime(u64 val);
extern void aee_rr_rec_cpu_dying_ktime(u64 val);
extern void aee_rr_rec_cpu_dead_ktime(u64 val);
extern void aee_rr_rec_cpu_post_dead_ktime(u64 val);
extern void aee_sram_fiq_log(const char *msg);
extern void aee_rr_rec_wdk_ktime(u64 val);
extern void aee_rr_rec_wdk_systimer_cnt(u64 val);

#else
static inline void aee_rr_rec_clk(int id, u32 val)
{
}

static inline int aee_rr_reboot_reason_show(struct seq_file *m, void *v)
{
	return 0;
}

static inline int aee_rr_last_fiq_step(void)
{
	return 0;
}

static inline void aee_rr_rec_exp_type(unsigned int type)
{
}

static inline unsigned int aee_rr_curr_exp_type(void)
{
	return 0;
}

static inline void aee_rr_rec_kick(uint32_t kick_bit)
{
}

static inline void aee_rr_rec_check(uint32_t check_bit)
{
}

static inline void aee_rr_rec_scp(void)
{
}

static inline void aee_rr_rec_kaslr_offset(u64 value64)
{
}

static inline void aee_rr_rec_cpu_dvfs_vproc_big(u8 val)
{
}

static inline void aee_rr_rec_cpu_dvfs_vproc_little(u8 val)
{
}

static inline void aee_rr_rec_cpu_dvfs_oppidx(u8 val)
{
}

static inline void aee_rr_rec_cpu_dvfs_cci_oppidx(u8 val)
{
}

static inline void aee_rr_rec_cpu_dvfs_status(u8 val)
{
}

static inline void aee_rr_rec_cpu_dvfs_step(u8 val)
{
}

static inline void aee_rr_rec_cpu_dvfs_cb(u8 val)
{
}

static inline void aee_rr_rec_cpufreq_cb(u8 val)
{
}

static inline u8 aee_rr_curr_cpu_dvfs_oppidx(void)
{
	return 0;
}

static inline u8 aee_rr_curr_cpu_dvfs_cci_oppidx(void)
{
	return 0;
}

static inline u8 aee_rr_curr_cpu_dvfs_status(void)
{
	return 0;
}

static inline u8 aee_rr_curr_cpu_dvfs_step(void)
{
	return 0;
}

static inline u8 aee_rr_curr_cpu_dvfs_cb(void)
{
	return 0;
}

static inline u8 aee_rr_curr_cpufreq_cb(void)
{
	return 0;
}

static inline void aee_rr_rec_ptp_devinfo_0(u32 val)
{
}

static inline void aee_rr_rec_ptp_devinfo_1(u32 val)
{
}

static inline void aee_rr_rec_ptp_devinfo_2(u32 val)
{
}

static inline void aee_rr_rec_ptp_devinfo_3(u32 val)
{
}

static inline void aee_rr_rec_ptp_devinfo_4(u32 val)
{
}

static inline void aee_rr_rec_ptp_devinfo_5(u32 val)
{
}

static inline void aee_rr_rec_ptp_devinfo_6(u32 val)
{
}

static inline void aee_rr_rec_ptp_devinfo_7(u32 val)
{
}

static inline void aee_rr_rec_ptp_e0(u32 val)
{
}

static inline void aee_rr_rec_ptp_e1(u32 val)
{
}

static inline void aee_rr_rec_ptp_e2(u32 val)
{
}

static inline void aee_rr_rec_ptp_e3(u32 val)
{
}

static inline void aee_rr_rec_ptp_e4(u32 val)
{
}

static inline void aee_rr_rec_ptp_e5(u32 val)
{
}

static inline void aee_rr_rec_ptp_e6(u32 val)
{
}

static inline void aee_rr_rec_ptp_e7(u32 val)
{
}

static inline void aee_rr_rec_ptp_e8(u32 val)
{
}

static inline void aee_rr_rec_ptp_e9(u32 val)
{
}

static inline void aee_rr_rec_ptp_e10(u32 val)
{
}

static inline void aee_rr_rec_ptp_e11(u32 val)
{
}

static inline void aee_rr_rec_ptp_vboot(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_big_volt(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_big_volt_1(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_big_volt_2(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_big_volt_3(u64 val)
{
}

static inline void aee_rr_rec_ptp_gpu_volt(u64 val)
{
}

static inline void aee_rr_rec_ptp_gpu_volt_1(u64 val)
{
}

static inline void aee_rr_rec_ptp_gpu_volt_2(u64 val)
{
}

static inline void aee_rr_rec_ptp_gpu_volt_3(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_little_volt(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_little_volt_1(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_little_volt_2(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_little_volt_3(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_2_little_volt(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_2_little_volt_1(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_2_little_volt_2(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_2_little_volt_3(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_cci_volt(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_cci_volt_1(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_cci_volt_2(u64 val)
{
}

static inline void aee_rr_rec_ptp_cpu_cci_volt_3(u64 val)
{
}

static inline void aee_rr_rec_ptp_temp(u64 val)
{
}

static inline void aee_rr_rec_ptp_status(u8 val)
{
}

static inline void aee_rr_rec_eem_pi_offset(u8 val)
{
}

static inline u32 aee_rr_curr_ptp_devinfo_0(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_devinfo_1(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_devinfo_2(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_devinfo_3(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_devinfo_4(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_devinfo_5(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_devinfo_6(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_devinfo_7(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e0(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e1(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e2(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e3(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e4(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e5(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e6(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e7(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e8(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e9(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e10(void)
{
	return 0;
}

static inline u32 aee_rr_curr_ptp_e11(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_vboot(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_big_volt(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_big_volt_1(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_big_volt_2(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_big_volt_3(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_gpu_volt(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_gpu_volt_1(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_gpu_volt_2(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_gpu_volt_3(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_little_volt(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_little_volt_1(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_little_volt_2(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_little_volt_3(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_2_little_volt(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_2_little_volt_1(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_2_little_volt_2(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_2_little_volt_3(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_cci_volt(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_cci_volt_1(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_cci_volt_2(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_cpu_cci_volt_3(void)
{
	return 0;
}

static inline u64 aee_rr_curr_ptp_temp(void)
{
	return 0;
}

static inline u8 aee_rr_curr_ptp_status(void)
{
	return 0;
}

static inline unsigned long *aee_rr_rec_mtk_cpuidle_footprint_va(void)
{
	return NULL;
}

static inline unsigned long *aee_rr_rec_mtk_cpuidle_footprint_pa(void)
{
	return NULL;
}

static inline void aee_rr_rec_sodi3_val(u32 val)
{
}

static inline u32 aee_rr_curr_sodi3_val(void)
{
	return 0;
}

static inline void aee_rr_rec_sodi_val(u32 val)
{
}

static inline u32 aee_rr_curr_sodi_val(void)
{
	return 0;
}

static inline void aee_rr_rec_deepidle_val(u32 val)
{
}

static inline u32 aee_rr_curr_deepidle_val(void)
{
	return 0;
}

static inline void aee_rr_rec_spm_suspend_val(u32 val)
{
}

static inline u32 aee_rr_curr_spm_suspend_val(void)
{
	return 0;
}

static inline void aee_rr_rec_vcore_dvfs_status(u32 val)
{
}

static inline u32 aee_rr_curr_vcore_dvfs_status(void)
{
	return 0;
}

static inline unsigned int *aee_rr_rec_mcdi_wfi(void)
{
	return NULL;
}

static inline void aee_rr_rec_mcdi_val(int id, u32 val)
{
}

static inline void aee_rr_rec_vcore_dvfs_opp(u32 val)
{
}

static inline u32 aee_rr_curr_vcore_dvfs_opp(void)
{
	return 0;
}

static inline void aee_rr_rec_ocp_target_limit(int id, u32 val)
{
}

static inline u32 aee_rr_curr_ocp_target_limit(int id)
{
	return 0;
}

static inline void aee_rr_rec_ocp_enable(u8 val)
{
}

static inline u8 aee_rr_curr_ocp_enable(void)
{
	return 0;
}

static inline void aee_rr_rec_ppm_cluster_limit(int id, u32 val)
{
}

static inline void aee_rr_rec_ppm_step(u8 val)
{
}

static inline void aee_rr_rec_ppm_cur_state(u8 val)
{
}

static inline void aee_rr_rec_ppm_min_pwr_bgt(u32 val)
{
}

static inline void aee_rr_rec_ppm_policy_mask(u32 val)
{
}

static inline void aee_rr_rec_ppm_waiting_for_pbm(u8 val)
{
}

static inline void aee_rr_rec_gpu_dvfs_vgpu(u8 val)
{
}

static inline u8 aee_rr_curr_gpu_dvfs_vgpu(void)
{
	return 0;
}

static inline void aee_rr_rec_gpu_dvfs_oppidx(u8 val)
{
}

static inline void aee_rr_rec_gpu_dvfs_status(u8 val)
{
}

static inline u8 aee_rr_curr_gpu_dvfs_status(void)
{
	return 0;
}

static inline void aee_rr_rec_gpu_dvfs_power_count(int val)
{
}

static inline void aee_rr_rec_hang_detect_timeout_count(unsigned int val)
{
}

static inline int aee_rr_curr_fiq_step(void)
{
	return 0;
}

static inline void aee_rr_rec_fiq_step(u8 i)
{
}

static inline void aee_rr_rec_last_irq_enter(int cpu, int irq, u64 j)
{
}

static inline void aee_rr_rec_last_irq_exit(int cpu, int irq, u64 j)
{
}

static inline void aee_rr_rec_hotplug_footprint(int cpu, u8 fp)
{
}

static inline void aee_rr_rec_hotplug_cpu_event(u8 val)
{
}

static inline void aee_rr_rec_hotplug_cb_index(u8 val)
{
}

static inline void aee_rr_rec_hotplug_cb_fp(unsigned long val)
{
}

static inline void aee_rr_rec_hotplug_cb_times(unsigned long val)
{
}

static inline void aee_rr_rec_hps_cb_enter_times(unsigned long long val)
{
}

static inline void aee_rr_rec_hps_cb_cpu_bitmask(unsigned int val)
{
}

static inline void aee_rr_rec_hps_cb_footprint(unsigned int val)
{
}

static inline void aee_rr_rec_hps_cb_fp_times(unsigned long long val)
{
}

static inline void aee_rr_rec_last_init_func(unsigned long val)
{
}

static inline void aee_rr_rec_last_sync_func(unsigned long val)
{
}

static inline void aee_rr_rec_last_async_func(unsigned long val)
{
}

static inline void aee_rr_rec_set_bit_pmic_ext_buck(int bit, int loc)
{
}

static inline int aee_rr_init_thermal_temp(int num)
{
	return 0;
}

static inline int aee_rr_rec_thermal_temp(int index, s8 val)
{
	return 0;
}

static inline void aee_rr_rec_thermal_status(u8 val)
{
}

static inline void aee_rr_rec_thermal_ATM_status(u8 val)
{
}

static inline void aee_rr_rec_thermal_ktime(u64 val)
{
}

static inline s8 aee_rr_curr_thermal_temp(int index)
{
	return 0;
}

static inline u8 aee_rr_curr_thermal_status(void)
{
	return 0;
}

static inline u8 aee_rr_curr_thermal_ATM_status(void)
{
	return 0;
}

static inline u64 aee_rr_curr_thermal_ktime(void)
{
	return 0;
}

static inline void aee_rr_rec_cpu_caller(u32 val)
{
}

static inline void aee_rr_rec_cpu_callee(u32 val)
{
}

static inline void aee_rr_rec_cpu_up_prepare_ktime(u64 val)
{
}

static inline void aee_rr_rec_cpu_starting_ktime(u64 val)
{
}

static inline void aee_rr_rec_cpu_online_ktime(u64 val)
{
}

static inline void aee_rr_rec_cpu_down_prepare_ktime(u64 val)
{
}

static inline void aee_rr_rec_cpu_dying_ktime(u64 val)
{
}

static inline void aee_rr_rec_cpu_dead_ktime(u64 val)
{
}

static inline void aee_rr_rec_cpu_post_dead_ktime(u64 val)
{
}

static inline void aee_sram_fiq_log(const char *msg)
{
}

static inline void aee_rr_rec_wdk_ktime(u64 val)
{
}

static inline void aee_rr_rec_wdk_systimer_cnt(u64 val)
{
}

#endif /* CONFIG_MTK_AEE_IPANIC */

#endif
