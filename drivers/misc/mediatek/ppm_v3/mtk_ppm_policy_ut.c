// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>

#include "mtk_ppm_internal.h"


static void ppm_ut_update_limit_cb(void);

/* other members will init by ppm_main */
static struct ppm_policy_data ut_policy = {
	.name			= __stringify(PPM_POLICY_UT),
	.lock			= __MUTEX_INITIALIZER(ut_policy.lock),
	.policy			= PPM_POLICY_UT,
	.priority		= PPM_POLICY_PRIO_HIGHEST,
	.update_limit_cb	= ppm_ut_update_limit_cb,
	.status_change_cb	= NULL,
};

struct ppm_ut_data {
	bool is_freq_idx_fixed;
	bool is_core_num_fixed;

	struct ppm_ut_limit {
		int freq_idx;
		int core_num;
	} *limit;
} ut_data;


static void ppm_ut_update_limit_cb(void)
{
}

static int ppm_ut_fix_core_num_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < ut_policy.req.cluster_num; i++)
		seq_printf(m, "cluster %d fix core num = %d\n",
			i, ut_data.limit[i].core_num);

	return 0;
}

static ssize_t ppm_ut_fix_core_num_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int i = 0;
	int *fix_core;
	bool activated = true;
	bool is_clear = true;
	unsigned int cluster_num = ut_policy.req.cluster_num;
	char *buf, *tok, *tmp;

	if (!ut_data.limit)
		return -EINVAL;

	buf = ppm_copy_from_user_for_proc(buffer, count);
	if (!buf)
		return -ENOMEM;

	fix_core = kcalloc(cluster_num, sizeof(*fix_core), GFP_KERNEL);
	if (!fix_core)
		goto no_mem;

	tmp = buf;
	while ((tok = strsep(&tmp, " ")) != NULL) {
		if (i == cluster_num) {
			ppm_err("number of arguments > %d!\n", cluster_num);
			goto out;
		}

		if (kstrtoint(tok, 10, &fix_core[i])) {
			ppm_err("Invalid input: %s\n", tok);
			goto out;
		} else
			i++;
	}

	if (i < cluster_num) {
		ppm_err("number of arguments < %d!\n", cluster_num);
		goto out;
	}

	for (i = 0; i < cluster_num; i++) {
		if (fix_core[i] > (int)get_cluster_max_cpu_core(i)) {
			ppm_err("@%s: Invalid input! fix_core[%d] = %d\n",
				__func__, i, fix_core[i]);
			goto out;
		} else if (fix_core[i] != -1)
			is_clear = false;
	}

	ppm_lock(&ut_policy.lock);

	if (!ut_policy.is_enabled) {
		ppm_warn("@%s: UT policy is not enabled!\n", __func__);
		ppm_unlock(&ut_policy.lock);
		goto out;
	}

	if (is_clear) {
		ut_data.is_core_num_fixed = false;
		for (i = 0; i < cluster_num; i++) {
			ut_data.limit[i].core_num = -1;
			ut_policy.req.limit[i].min_cpu_core =
				get_cluster_min_cpu_core(i);
			ut_policy.req.limit[i].max_cpu_core =
				get_cluster_max_cpu_core(i);
		}
	} else {
		ut_data.is_core_num_fixed = true;

		for (i = 0; i < cluster_num; i++) {
			ut_data.limit[i].core_num = fix_core[i];
			ut_policy.req.limit[i].min_cpu_core =
				(fix_core[i] == -1)
					? get_cluster_min_cpu_core(i)
					: fix_core[i];
			ut_policy.req.limit[i].max_cpu_core =
				(fix_core[i] == -1)
					? get_cluster_max_cpu_core(i)
					: fix_core[i];
		}
	}

	/* unlimited */
	if (!ut_data.is_core_num_fixed && !ut_data.is_freq_idx_fixed)
		activated = false;

	/* fire ppm HERE!*/
	ut_policy.is_activated = activated;
	ppm_unlock(&ut_policy.lock);
	mt_ppm_main();

out:
	kfree(fix_core);
no_mem:
	free_page((unsigned long)buf);
	return count;
}

static int ppm_ut_fix_freq_idx_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < ut_policy.req.cluster_num; i++)
		seq_printf(m, "cluster %d fix freq idx = %d\n",
			i, ut_data.limit[i].freq_idx);

	return 0;
}

static ssize_t ppm_ut_fix_freq_idx_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int i = 0;
	int *fix_freq;
	bool activated = true;
	bool is_clear = true;
	unsigned int cluster_num = ut_policy.req.cluster_num;
	char *buf, *tok, *tmp;

	if (!ut_data.limit)
		return -EINVAL;

	buf = ppm_copy_from_user_for_proc(buffer, count);
	if (!buf)
		return -ENOMEM;

	fix_freq = kcalloc(cluster_num, sizeof(*fix_freq), GFP_KERNEL);
	if (!fix_freq)
		goto no_mem;

	tmp = buf;
	while ((tok = strsep(&tmp, " ")) != NULL) {
		if (i == cluster_num) {
			ppm_err("number of arguments > %d!\n", cluster_num);
			goto out;
		}

		if (kstrtoint(tok, 10, &fix_freq[i])) {
			ppm_err("Invalid input: %s\n", tok);
			goto out;
		} else
			i++;
	}

	if (i < cluster_num) {
		ppm_err("number of arguments < %d!\n", cluster_num);
		goto out;
	}

	for (i = 0; i < cluster_num; i++) {
		if (fix_freq[i] > (int)get_cluster_min_cpufreq_idx(i)) {
			ppm_err("@%s: Invalid input! fix_freq[%d] = %d\n",
				__func__, i, fix_freq[i]);
			goto out;
		} else if (fix_freq[i] != -1)
			is_clear = false;
	}

	ppm_lock(&ut_policy.lock);

	if (!ut_policy.is_enabled) {
		ppm_warn("@%s: UT policy is not enabled!\n", __func__);
		ppm_unlock(&ut_policy.lock);
		goto out;
	}

	if (is_clear) {
		ut_data.is_freq_idx_fixed = false;
		for (i = 0; i < cluster_num; i++) {
			ut_data.limit[i].freq_idx = -1;
			ut_policy.req.limit[i].min_cpufreq_idx =
				get_cluster_min_cpufreq_idx(i);
			ut_policy.req.limit[i].max_cpufreq_idx =
				get_cluster_max_cpufreq_idx(i);
		}
	} else {
		ut_data.is_freq_idx_fixed = true;

		for (i = 0; i < cluster_num; i++) {
			ut_data.limit[i].freq_idx = fix_freq[i];
			ut_policy.req.limit[i].min_cpufreq_idx =
				(fix_freq[i] == -1)
					? get_cluster_min_cpufreq_idx(i)
					: fix_freq[i];
			ut_policy.req.limit[i].max_cpufreq_idx =
				(fix_freq[i] == -1)
					? get_cluster_max_cpufreq_idx(i)
					: fix_freq[i];
		}
	}

	/* unlimited */
	if (!ut_data.is_core_num_fixed && !ut_data.is_freq_idx_fixed)
		activated = false;

	/* fire ppm HERE!*/
	ut_policy.is_activated = activated;
	ppm_unlock(&ut_policy.lock);
	mt_ppm_main();

out:
	kfree(fix_freq);
no_mem:
	free_page((unsigned long)buf);
	return count;
}

PROC_FOPS_RW(ut_fix_core_num);
PROC_FOPS_RW(ut_fix_freq_idx);

int ppm_ut_policy_init(void)
{
	int i, ret = 0;

	struct pentry {
		const char *name;
		const struct proc_ops *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(ut_fix_core_num),
		PROC_ENTRY(ut_fix_freq_idx),
	};

	FUNC_ENTER(FUNC_LV_POLICY);

	/* create procfs */
	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create(entries[i].name, 0644,
			policy_dir, entries[i].fops)) {
			ppm_err("%s(), create /proc/ppm/policy/%s failed\n",
				__func__, entries[i].name);
			ret = -EINVAL;
			goto out;
		}
	}

	ut_data.limit = kcalloc(ppm_main_info.cluster_num,
		sizeof(*ut_data.limit), GFP_KERNEL);
	if (!ut_data.limit) {
		ret = -ENOMEM;
		goto out;
	}

	/* init ut_data */
	ut_data.is_freq_idx_fixed = false;
	ut_data.is_core_num_fixed = false;
	for (i = 0; i < ppm_main_info.cluster_num; i++) {
		ut_data.limit[i].freq_idx = -1;
		ut_data.limit[i].core_num = -1;
	}

	if (ppm_main_register_policy(&ut_policy)) {
		ppm_err("@%s: UT policy register failed\n", __func__);
		ret = -EINVAL;
		kfree(ut_data.limit);
		goto out;
	}

	ppm_info("@%s: register %s done!\n", __func__, ut_policy.name);

out:
	FUNC_EXIT(FUNC_LV_POLICY);

	return ret;
}

void ppm_ut_policy_exit(void)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	kfree(ut_data.limit);

	ppm_main_unregister_policy(&ut_policy);

	FUNC_EXIT(FUNC_LV_POLICY);
}
