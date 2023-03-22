// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/topology.h>

#include "mtk_cpufreq_internal.h"
#include "mtk_cpufreq_hybrid.h"
#include "mtk_cpufreq_platform.h"

#if IS_ENABLED(CONFIG_MTK_CPU_MSSV)
unsigned int __attribute__((weak)) cpumssv_get_state(void) { return 0; }
#endif

unsigned int func_lv_mask;
unsigned int do_dvfs_stress_test;
unsigned int dvfs_power_mode;
unsigned int sched_dvfs_enable;

ktime_t now[NR_SET_V_F];
ktime_t delta[NR_SET_V_F];
ktime_t max[NR_SET_V_F];

enum ppb_power_mode {
	DEFAULT_MODE,		/* normal mode */
	LOW_POWER_MODE,
	JUST_MAKE_MODE,
	PERFORMANCE_MODE,	/* sports mode */
	NUM_PPB_POWER_MODE
};

static const char *power_mode_str[NUM_PPB_POWER_MODE] = {
	"Default(Normal) mode",
	"Low Power mode",
	"Just Make mode",
	"Performance(Sports) mode"
};

char *_copy_from_user_for_proc(const char __user *buffer, size_t count)
{
	char *buf = (char *)__get_free_page(GFP_USER);

	if (!buf)
		return NULL;

	if (count >= PAGE_SIZE)
		goto out;

	if (copy_from_user(buf, buffer, count))
		goto out;

	buf[count] = '\0';

	return buf;

out:
	free_page((unsigned long)buf);

	return NULL;
}

/* cpufreq_debug */
static int cpufreq_debug_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "cpufreq debug (log level) = %d\n", func_lv_mask);

	return 0;
}
#ifndef CONFIG_GENERIC_ARCH_TOPOLOGY
extern struct cpu_topology cpu_topology[NR_CPUS];
#endif
static int arch_get_cluster_id(unsigned int cpu)
{
	struct cpu_topology *cpu_topo = &cpu_topology[cpu];

	return cpu_topo->package_id < 0 ? 0 : cpu_topo->package_id;
}


static ssize_t cpufreq_debug_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int dbg_lv;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	rc = kstrtoint(buf, 10, &dbg_lv);
	if (rc < 0)
		tag_pr_info
		("echo dbg_lv (dec) > /proc/cpufreq/cpufreq_debug\n");
	else
		func_lv_mask = dbg_lv;

	free_page((unsigned long)buf);
	return count;
}

static int cpufreq_power_mode_proc_show(struct seq_file *m, void *v)
{
	unsigned int mode = dvfs_power_mode;

	seq_printf(m, "%s\n",
	mode < NUM_PPB_POWER_MODE ? power_mode_str[mode] : "Unknown");

	return 0;
}

static ssize_t cpufreq_power_mode_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int mode = 0;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &mode) && mode < NUM_PPB_POWER_MODE) {
		dvfs_power_mode = mode;
		tag_pr_debug("%s start\n", power_mode_str[mode]);
	} else {
		tag_pr_info
		("echo 0/1/2/3 > /proc/cpufreq/cpufreq_power_mode\n");
	}

	free_page((unsigned long)buf);
	return count;
}

static int cpufreq_stress_test_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", do_dvfs_stress_test);

	return 0;
}

static ssize_t cpufreq_stress_test_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int do_stress;
	int rc;
	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;
	rc = kstrtoint(buf, 10, &do_stress);
	if (rc < 0)
		tag_pr_info("echo 0/1 > /proc/cpufreq/cpufreq_stress_test\n");
	else {
		do_dvfs_stress_test = do_stress;
#ifdef HYBRID_CPU_DVFS
		cpuhvfs_set_dvfs_stress(do_stress);
#endif
	}

	free_page((unsigned long)buf);
	return count;
}

/* cpufreq_oppidx */
static int cpufreq_oppidx_proc_show(struct seq_file *m, void *v)
{
	struct mt_cpu_dvfs *p = m->private;
	int j;
	unsigned long flags;


	cpufreq_lock(flags);
	seq_printf(m, "[%s/%u]\n", p->name, p->cpu_id);
	seq_printf(m, "cpufreq_oppidx = %d\n", p->idx_opp_tbl);

	for (j = 0; j < p->nr_opp_tbl; j++) {
		seq_printf(m, "\t%-2d (%u, %u)\n",
			      j, cpu_dvfs_get_freq_by_idx(p, j),
			      cpu_dvfs_get_volt_by_idx(p, j));
	}
	cpufreq_unlock(flags);

	return 0;
}

static ssize_t cpufreq_oppidx_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	struct mt_cpu_dvfs *p = PDE_DATA(file_inode(file));
	int oppidx;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	rc = kstrtoint(buf, 10, &oppidx);
	if (rc < 0) {
		p->dvfs_disable_by_procfs = false;
		tag_pr_info("echo oppidx > /proc/cpufreq/%s/cpufreq_oppidx\n",
		p->name);
	} else {
		if (oppidx >= 0 && oppidx < p->nr_opp_tbl) {
			p->dvfs_disable_by_procfs = true;
#ifdef HYBRID_CPU_DVFS
			if (!cpu_dvfs_is(p, MT_CPU_DVFS_CCI))
				cpuhvfs_set_freq(
					arch_get_cluster_id(p->cpu_id),
					cpu_dvfs_get_freq_by_idx(p, oppidx));
#else
			_mt_cpufreq_dvfs_request_wrapper(p, oppidx,
			MT_CPU_DVFS_NORMAL, NULL);
#endif
		} else {
			p->dvfs_disable_by_procfs = false;
			tag_pr_info
			("echo oppidx > /proc/cpufreq/%s/cpufreq_oppidx\n",
			p->name);
		}
	}

	free_page((unsigned long)buf);

	return count;
}

/* cpufreq_freq */
static int cpufreq_freq_proc_show(struct seq_file *m, void *v)
{
	struct mt_cpu_dvfs *p = m->private;
	struct pll_ctrl_t *pll_p = id_to_pll_ctrl(p->Pll_id);

	if (pll_p == NULL)
		return 0;

	seq_printf(m, "%d KHz\n", pll_p->pll_ops->get_cur_freq(pll_p));

	return 0;
}

static ssize_t cpufreq_freq_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	struct mt_cpu_dvfs *p = PDE_DATA(file_inode(file));
	int freq, i, found = 0;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	rc = kstrtoint(buf, 10, &freq);
	if (rc < 0) {
		p->dvfs_disable_by_procfs = false;
		tag_pr_info
		("echo khz > /proc/cpufreq/%s/cpufreq_freq\n", p->name);
	} else {
#if IS_ENABLED(CONFIG_MTK_CPU_MSSV)
		if (!cpumssv_get_state()) {
			for (i = 0; i < p->nr_opp_tbl; i++) {
				if (freq == p->opp_tbl[i].cpufreq_khz) {
					found = 1;
					break;
				}
			}
		} else if (freq > 0)
			found = 1;

		if (found == 1) {
			p->dvfs_disable_by_procfs = true;
  #ifdef HYBRID_CPU_DVFS
			if (!cpu_dvfs_is(p, MT_CPU_DVFS_CCI))
    #ifdef SINGLE_CLUSTER
				cpuhvfs_set_freq(cpufreq_get_cluster_id(
					p->cpu_id), freq);
    #else
				cpuhvfs_set_freq(arch_get_cluster_id(
					p->cpu_id), freq);
    #endif
			else
				cpuhvfs_set_freq(MT_CPU_DVFS_CCI, freq);
  #else
			_mt_cpufreq_dvfs_request_wrapper(p,
					i, MT_CPU_DVFS_NORMAL, NULL);
  #endif
		} else {
			p->dvfs_disable_by_procfs = false;
			tag_pr_info(
		"frequency %dKHz! is not found in CPU opp table\n", freq);
			}
#else
		if (freq < p->opp_tbl[p->nr_opp_tbl - 1].cpufreq_khz) {
			if (freq != 0)
				tag_pr_info
				("frequency should higher than %dKHz!\n",
				p->opp_tbl[p->nr_opp_tbl - 1].cpufreq_khz);

			p->dvfs_disable_by_procfs = false;
		} else {
			for (i = 0; i < p->nr_opp_tbl; i++) {
				if (freq == p->opp_tbl[i].cpufreq_khz) {
					found = 1;
					break;
				}
			}

			if (found == 1) {
				p->dvfs_disable_by_procfs = true;
#ifdef HYBRID_CPU_DVFS
				if (!cpu_dvfs_is(p, MT_CPU_DVFS_CCI))
					cpuhvfs_set_freq(
					arch_get_cluster_id(p->cpu_id),
						cpu_dvfs_get_freq_by_idx(p, i));
#else
				_mt_cpufreq_dvfs_request_wrapper(p,
				i, MT_CPU_DVFS_NORMAL, NULL);
#endif
			} else {
				p->dvfs_disable_by_procfs = false;
				tag_pr_info
			("frequency %dKHz! is not found in CPU opp table\n",
					    freq);
			}
		}
#endif
	}

	free_page((unsigned long)buf);

	return count;
}

/* cpufreq_volt */
static int cpufreq_volt_proc_show(struct seq_file *m, void *v)
{
	struct mt_cpu_dvfs *p = m->private;
	struct buck_ctrl_t *vproc_p = id_to_buck_ctrl(p->Vproc_buck_id);
	struct buck_ctrl_t *vsram_p = id_to_buck_ctrl(p->Vsram_buck_id);
	unsigned long flags;

	if (vproc_p == NULL || vsram_p == NULL)
		return 0;
	cpufreq_lock(flags);
	seq_printf(m, "Vproc: %d uV\n",
		vproc_p->buck_ops->get_cur_volt(vproc_p) * 10);
	seq_printf(m, "Vsram: %d uV\n",
		vsram_p->buck_ops->get_cur_volt(vsram_p) * 10);
	cpufreq_unlock(flags);

	return 0;
}

static ssize_t cpufreq_volt_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned long flags;
	struct mt_cpu_dvfs *p = PDE_DATA(file_inode(file));
#ifndef HYBRID_CPU_DVFS
	struct buck_ctrl_t *vproc_p = id_to_buck_ctrl(p->Vproc_buck_id);
	int ret = 0;
#endif
	int uv;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;
	rc = kstrtoint(buf, 10, &uv);
	if (rc < 0) {
		p->dvfs_disable_by_procfs = false;
		tag_pr_info
		("echo uv > /proc/cpufreq/%s/cpufreq_volt\n", p->name);
	} else {
		p->dvfs_disable_by_procfs = true;
		cpufreq_lock(flags);
#ifdef HYBRID_CPU_DVFS
#if IS_ENABLED(CONFIG_MTK_CPU_MSSV)
		if (!cpu_dvfs_is(p, MT_CPU_DVFS_CCI))
#ifdef SINGLE_CLUSTER
			cpuhvfs_set_volt(
				cpufreq_get_cluster_id(p->cpu_id), uv/10);
#else
			cpuhvfs_set_volt(
				arch_get_cluster_id(p->cpu_id), uv/10);
#endif
#endif
#else
		vproc_p->fix_volt = uv / 10;
		ret = set_cur_volt_wrapper(p, vproc_p->fix_volt);
		if (ret)
			tag_pr_info("%s err to set_cur_volt_wrapper ret = %d\n",
					__func__, ret);
#endif
		cpufreq_unlock(flags);
	}

	free_page((unsigned long)buf);

	return count;
}

/* cpufreq_turbo_mode */
int disable_turbo;
static int cpufreq_turbo_mode_proc_show(struct seq_file *m, void *v)
{
	struct mt_cpu_dvfs *p = m->private;

	seq_printf(m, "turbo_mode(support, disable, loc_opt) = %d, %d, %d\n",
		      turbo_flag, disable_turbo, p->turbo_mode);

	return 0;
}

static ssize_t cpufreq_turbo_mode_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	struct mt_cpu_dvfs *p = PDE_DATA(file_inode(file));
	unsigned int turbo_mode;
	int rc;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;
	rc = kstrtoint(buf, 10, &turbo_mode);
	if (rc < 0)
		tag_pr_info
		("echo 0/1 > /proc/cpufreq/%s/cpufreq_turbo_mode\n", p->name);
	else {
		p->turbo_mode = turbo_mode;
#ifdef HYBRID_CPU_DVFS
		if (turbo_mode == 0) {
			cpuhvfs_set_turbo_disable(1);
			cpuhvfs_set_turbo_mode(0, 0, 0);
			disable_turbo = 1;
		}
#endif
	}

	free_page((unsigned long)buf);

	return count;
}

/* cpufreq_sched_disable */
static int cpufreq_sched_disable_proc_show(struct seq_file *m, void *v)
{
	int r = 1;

#ifdef HYBRID_CPU_DVFS
	r = cpuhvfs_get_sched_dvfs_disable();
#endif

	seq_printf(m, "cpufreq_sched_disable = %d\n", r);

	return 0;
}

static ssize_t cpufreq_sched_disable_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int sched_disable;
	int rc;
	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;
	rc = kstrtoint(buf, 10, &sched_disable);
	if (rc < 0)
		tag_pr_info
		("echo 0/1 > /proc/cpufreq/cpufreq_sched_disable\n");
	else {
#ifdef HYBRID_CPU_DVFS
		cpuhvfs_set_sched_dvfs_disable(sched_disable);
		if (sched_disable)
			sched_dvfs_enable = 0;
		else
			sched_dvfs_enable = 1;
#endif
	}

	free_page((unsigned long)buf);

	return count;
}

/* cpufreq_time_profile */
static int cpufreq_dvfs_time_profile_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < NR_SET_V_F; i++)
		seq_printf(m, "max[%d] = %lld us\n", i, ktime_to_us(max[i]));

#ifdef HYBRID_CPU_DVFS
	cpuhvfs_get_time_profile();
#endif

	return 0;
}

static ssize_t cpufreq_dvfs_time_profile_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int temp;
	int rc;
	int i;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	rc = kstrtoint(buf, 10, &temp);
	if (rc < 0)
		tag_pr_info
		("echo 1 > /proc/cpufreq/cpufreq_dvfs_time_profile\n");
	else {
		if (temp == 1) {
			for (i = 0; i < NR_SET_V_F; i++)
				/* max[i].tv64 = 0; */
				max[i] = ktime_set(0, 0);
		}
	}
	free_page((unsigned long)buf);

	return count;
}

#ifdef CCI_MAP_TBL_SUPPORT
/* cpufreq_cci_map_table */
static int cpufreq_cci_map_table_proc_show(struct seq_file *m, void *v)
{
	int i, j, k;
	unsigned int result;
	unsigned int pt_1 = 0, pt_2 = 0;

#ifdef HYBRID_CPU_DVFS
	for (k = 0; k < NR_CCI_TBL; k++) {
		if (k == 0 && !pt_1) {
			seq_puts(m, "CCI MAP Normal Mode:\n");
			pt_1 = 1;
		} else if (k == 1 && !pt_2) {
			seq_puts(m, "CCI MAP Perf Mode:\n");
			pt_2 = 1;
		}
		for (i = 0; i < NR_FREQ; i++) {
			for (j = 0; j < NR_FREQ; j++) {
				result = cpuhvfs_get_cci_result(i, j, k);
				if (j == 0)
					seq_printf(m, "{%d", result);
				else if (j == (NR_FREQ - 1))
					seq_printf(m, ", %d},", result);
				else
					seq_printf(m, ", %d", result);
			}
			seq_puts(m, "\n");
		}
	}
#endif
	return 0;
}

static ssize_t cpufreq_cci_map_table_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int idx_1, idx_2, result, mode;

	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d %d %d",
		&idx_1, &idx_2, &result, &mode) == 4) {
#ifdef HYBRID_CPU_DVFS
		/* BY_PROC_FS */
		cpuhvfs_update_cci_map_tbl(idx_1,
			idx_2, result, mode, 0);
#endif
	} else
		tag_pr_info(
		"Usage: echo <L_idx> <B_idx> <result> <mode>\n");

	return count;
}
/* cpufreq_cci_mode */
static int cpufreq_cci_mode_proc_show(struct seq_file *m, void *v)
{
	unsigned int mode;

#ifdef HYBRID_CPU_DVFS
	mode = cpuhvfs_get_cci_mode();
	if (mode == 0)
		seq_puts(m, "cci_mode as Normal mode 0\n");
	else if (mode == 1)
		seq_puts(m, "cci_mode as Perf mode 1\n");
	else
		seq_puts(m, "cci_mode as Unknown mode 2\n");
#endif
	return 0;
}

static ssize_t cpufreq_cci_mode_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int mode;
	int rc;
	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	rc = kstrtoint(buf, 10, &mode);

	if (rc < 0)
		tag_pr_info(
		"Usage: echo <mode>(0:Nom 1:Perf)\n");
	else {
#ifdef HYBRID_CPU_DVFS
		/* BY_PROC_FS */
		cpuhvfs_update_cci_mode(mode, 0);
#endif
	}

	free_page((unsigned long)buf);

	return count;
}
#endif

#ifdef IMAX_ENABLE
static int cpufreq_imax_enable_proc_show(struct seq_file *m, void *v)
{
	unsigned int state;

#ifdef HYBRID_CPU_DVFS
	state = cpuhvfs_get_imax_state();
	seq_printf(m, "IMAX MODE = %d, (DISABLE = 0, ENABLE = 1, MPMM = 2)\n",
			state);
#endif
	return 0;
}

static ssize_t cpufreq_imax_enable_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int state;
	int rc;
	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	rc = kstrtoint(buf, 10, &state);

	if (rc < 0)
		tag_pr_info(
		"Usage: echo ON/OFF(0:Disable 1:Enable 2:Enable MPMM)\n");
	else {
#ifdef HYBRID_CPU_DVFS
		/* BY_PROC_FS */
		cpuhvfs_update_imax_state(state);
#endif
	}

	return count;
}

static int cpufreq_imax_reg_dump_proc_show(struct seq_file *m, void *v)
{
	unsigned int reg_val[REG_LEN] = {0};
	unsigned int addr_arr[REG_LEN] = { 0xcd10, 0xce0c, 0xce10, 0xce14 };
	unsigned int i;

#ifdef HYBRID_CPU_DVFS
	cpufreq_get_imax_reg(addr_arr, reg_val);
	for (i = 0; i < REG_LEN; i++)
		seq_printf(m, "IMAX addr 0x%x = 0x%x\n", addr_arr[i],
				reg_val[i]);
#endif
	return 0;
}

static int cpufreq_imax_thermal_protect_proc_show(struct seq_file *m, void *v)
{
	unsigned int state;

#ifdef HYBRID_CPU_DVFS
	state = cpuhvfs_get_imax_thermal_state();
	seq_printf(m, "IMAX THERMAL protect = %d, (DISABLE = 0, ENABLE = 1)\n",
			state);
#endif
	return 0;
}

static ssize_t cpufreq_imax_thermal_protect_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int state;
	int rc;
	char *buf = _copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	rc = kstrtoint(buf, 10, &state);

	if (rc < 0)
		tag_pr_info(
		"Usage: echo ON/OFF(0:Disable 1:Enable)\n");
	else {
#ifdef HYBRID_CPU_DVFS
		/* BY_PROC_FS */
		cpuhvfs_update_imax_thermal_state(state);
#endif
	}

	return count;
}

#endif

PROC_FOPS_RW(cpufreq_debug);
PROC_FOPS_RW(cpufreq_stress_test);
PROC_FOPS_RW(cpufreq_power_mode);
PROC_FOPS_RW(cpufreq_sched_disable);
PROC_FOPS_RW(cpufreq_dvfs_time_profile);
#ifdef CCI_MAP_TBL_SUPPORT
PROC_FOPS_RW(cpufreq_cci_map_table);
PROC_FOPS_RW(cpufreq_cci_mode);
#endif
#ifdef IMAX_ENABLE
PROC_FOPS_RW(cpufreq_imax_enable);
PROC_FOPS_RO(cpufreq_imax_reg_dump);
PROC_FOPS_RW(cpufreq_imax_thermal_protect);
#endif

PROC_FOPS_RW(cpufreq_oppidx);
PROC_FOPS_RW(cpufreq_freq);
PROC_FOPS_RW(cpufreq_volt);
PROC_FOPS_RW(cpufreq_turbo_mode);

int cpufreq_procfs_init(void)
{
	struct proc_dir_entry *dir = NULL;
	struct proc_dir_entry *cpu_dir = NULL;
	struct mt_cpu_dvfs *p = id_to_cpu_dvfs(0);
	int i, j;

	struct pentry {
		const char *name;
		const struct proc_ops *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(cpufreq_debug),
		PROC_ENTRY(cpufreq_stress_test),
		PROC_ENTRY(cpufreq_power_mode),
		PROC_ENTRY(cpufreq_sched_disable),
		PROC_ENTRY(cpufreq_dvfs_time_profile),
#ifdef CCI_MAP_TBL_SUPPORT
		PROC_ENTRY(cpufreq_cci_map_table),
		PROC_ENTRY(cpufreq_cci_mode),
#endif
#ifdef IMAX_ENABLE
		PROC_ENTRY(cpufreq_imax_enable),
		PROC_ENTRY(cpufreq_imax_reg_dump),
		PROC_ENTRY(cpufreq_imax_thermal_protect),
#endif

	};

	const struct pentry cpu_entries[] = {
		PROC_ENTRY(cpufreq_oppidx),
		PROC_ENTRY(cpufreq_freq),
		PROC_ENTRY(cpufreq_volt),
		PROC_ENTRY(cpufreq_turbo_mode),
	};

	dir = proc_mkdir("cpufreq", NULL);

	if (!dir) {
		tag_pr_notice("fail to create /proc/cpufreq @ %s()\n",
								__func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create
		    (entries[i].name, 0664, dir, entries[i].fops))
			tag_pr_notice("%s(), create /proc/cpufreq/%s failed\n",
				__func__, entries[i].name);
	}

	for_each_cpu_dvfs(j, p) {
		cpu_dir = proc_mkdir(p->name, dir);

		if (!cpu_dir) {
			tag_pr_notice
				("fail to create /proc/cpufreq/%s @ %s()\n",
				p->name, __func__);
			return -ENOMEM;
		}

		for (i = 0; i < ARRAY_SIZE(cpu_entries); i++) {
			if (!proc_create_data
			    (cpu_entries[i].name, 0664,
			    cpu_dir, cpu_entries[i].fops, p))
				tag_pr_notice
				("%s(), create /proc/cpufreq/%s/%s failed\n",
				__func__, p->name, entries[i].name);
		}
	}

	return 0;
}
