// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include "mtk_ppm_internal.h"

#define MAGIC_NUM	0x1605FAFA

static void ppm_ptpod_update_limit_cb(void);
static void ppm_ptpod_status_change_cb(bool enable);

/* other members will init by ppm_main */
static struct ppm_policy_data ptpod_policy = {
	.name			= __stringify(PPM_POLICY_PTPOD),
	.lock			= __MUTEX_INITIALIZER(ptpod_policy.lock),
	.policy			= PPM_POLICY_PTPOD,
	.priority		= PPM_POLICY_PRIO_HIGHEST,
	.update_limit_cb	= ppm_ptpod_update_limit_cb,
	.status_change_cb	= ppm_ptpod_status_change_cb,
};
unsigned int (*get_ptpod_fix_idx_register)(unsigned int cluster_id);
void mt_ppm_get_cluster_ptpod_fix_freq_idx_register(void *fun)
{
	if (fun != NULL)
		get_ptpod_fix_idx_register = fun;
	return;
}
EXPORT_SYMBOL(mt_ppm_get_cluster_ptpod_fix_freq_idx_register);

void mt_ppm_ptpod_policy_activate(void)
{
	unsigned int i;

	FUNC_ENTER(FUNC_LV_API);

	ppm_lock(&ptpod_policy.lock);

	if (!ptpod_policy.is_enabled) {
		ppm_warn("@%s: ptpod policy is not enabled!\n", __func__);
		ppm_unlock(&ptpod_policy.lock);
		goto end;
	}

	ptpod_policy.is_activated = true;
	for (i = 0; i < ptpod_policy.req.cluster_num; i++) {
		ptpod_policy.req.limit[i].min_cpufreq_idx =
			get_ptpod_fix_idx_register(i);
		ptpod_policy.req.limit[i].max_cpufreq_idx =
			get_ptpod_fix_idx_register(i);
	}
	ppm_unlock(&ptpod_policy.lock);
	mt_ppm_main();

end:
	FUNC_EXIT(FUNC_LV_API);
}
EXPORT_SYMBOL(mt_ppm_ptpod_policy_activate);

void mt_ppm_ptpod_policy_deactivate(void)
{
	unsigned int i;

	FUNC_ENTER(FUNC_LV_API);

	ppm_lock(&ptpod_policy.lock);

	/* deactivate ptpod policy */
	if (ptpod_policy.is_activated) {
		ptpod_policy.is_activated = false;

		/* restore to default setting */
		for (i = 0; i < ptpod_policy.req.cluster_num; i++) {
			ptpod_policy.req.limit[i].min_cpufreq_idx =
				get_cluster_min_cpufreq_idx(i);
			ptpod_policy.req.limit[i].max_cpufreq_idx =
				get_cluster_max_cpufreq_idx(i);
		}

		ppm_unlock(&ptpod_policy.lock);

		mt_ppm_main();
	} else
		ppm_unlock(&ptpod_policy.lock);

	FUNC_EXIT(FUNC_LV_API);
}
EXPORT_SYMBOL(mt_ppm_ptpod_policy_deactivate);

static void ppm_ptpod_update_limit_cb(void)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	FUNC_EXIT(FUNC_LV_POLICY);
}

static void ppm_ptpod_status_change_cb(bool enable)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: ptpod policy status changed to %d\n", __func__, enable);

	FUNC_EXIT(FUNC_LV_POLICY);
}

static int ppm_ptpod_test_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "PTPOD is_activate = %d\n", ptpod_policy.is_activated);

	return 0;
}

static ssize_t ppm_ptpod_test_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int enabled = 0;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (!kstrtouint(buf, 10, &enabled)) {
#ifdef PPM_SSPM_SUPPORT
		ppm_ipi_ptpod_test(enabled);
#else
		if (enabled == MAGIC_NUM)
			mt_ppm_ptpod_policy_activate();
		else
			mt_ppm_ptpod_policy_deactivate();
#endif
	} else
		ppm_err("@%s: Invalid input!\n", __func__);

	free_page((unsigned long)buf);
	return count;
}

PROC_FOPS_RW(ptpod_test);

int ppm_ptpod_policy_init(void)
{
	int i, ret = 0;

	struct pentry {
		const char *name;
		const struct proc_ops *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(ptpod_test),
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

	if (ppm_main_register_policy(&ptpod_policy)) {
		ppm_err("@%s: ptpod policy register failed\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	ppm_info("@%s: register %s done!\n", __func__, ptpod_policy.name);

out:
	FUNC_EXIT(FUNC_LV_POLICY);

	return ret;
}

void ppm_ptpod_policy_exit(void)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_main_unregister_policy(&ptpod_policy);

	FUNC_EXIT(FUNC_LV_POLICY);
}

