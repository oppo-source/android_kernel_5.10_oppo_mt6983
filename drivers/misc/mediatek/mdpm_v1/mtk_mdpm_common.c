// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <mtk_mdpm_common.h>
#include <mtk_mdpm_api.h>
#include <mtk_mdpm.h>

#if MD_POWER_METER_ENABLE
#include "mtk_vcore_dvfs.h"
#include "mtk_ccci_common.h"
#endif

bool mt_mdpm_debug;
static int g_dbm_power[POWER_TYPE_NUM], g_scenario_power[POWER_TYPE_NUM];
#ifdef MD_POWER_UT
u32 fake_share_reg;
u32 fake_share_mem[SHARE_MEM_BLOCK_NUM];
#endif

#if MD_POWER_METER_ENABLE
static bool md1_ccci_ready;
#endif

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[MDPM] " fmt

#define _BIT_(_bit_)		((unsigned int)(1 << (_bit_)))
#define _BITMASK_(_bits_)	\
(((unsigned int)-1>>(31-((1)?_bits_)))&~((1U<<((0)?_bits_))-1))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

u32 __attribute__ ((weak))
spm_vcorefs_get_MD_status(void)
{
	pr_notice_ratelimited("%s not ready\n", __func__);
	return 0;
}

#if MD_POWER_METER_ENABLE
void init_md_section_level(enum pbm_kicker kicker)
{
	u32 *share_mem = NULL;
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
	share_mem =
		(u32 *)get_smem_start_addr(MD_SYS1, SMEM_USER_RAW_DBM, NULL);
	if (share_mem == NULL) {
		pr_notice("ERROR: can't get SMEM_USER_RAW_DBM address");
		return;
	}
#else
	return;
#endif

	if (kicker == KR_MD1) {
		init_md1_section_level(share_mem);
		md1_ccci_ready = 1;
	} else
		pr_warn("unknown MD kicker: %d\n", kicker);
}

int get_md1_power(enum mdpm_power_type power_type, bool need_update)
{
	u32 share_reg, *share_mem;
	unsigned int scenario;
	int scenario_power, dbm_power;

#if !IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
	return 0;
#endif

	if (power_type >= POWER_TYPE_NUM ||
		power_type < 0) {
		pr_notice("[md1_power] invalid power_type=%d\n",
			power_type);
		return 0;
	}

	if (need_update == false)
		return g_scenario_power[MAX_POWER] + g_dbm_power[MAX_POWER];

#ifdef MD_POWER_UT
	share_reg = fake_share_reg;
#else
	if (!md1_ccci_ready)
		return MAX_MD1_POWER;

	share_reg = spm_vcorefs_get_MD_status();
#endif
	scenario = get_md1_scenario(share_reg, power_type);

	scenario_power = get_md1_scenario_power(scenario, power_type);
	g_scenario_power[power_type] = scenario_power;

#ifdef MD_POWER_UT
	share_mem = fake_share_mem;
#else
	share_mem = (u32 *)get_smem_start_addr(MD_SYS1, 0, NULL);
#endif
	dbm_power = get_md1_dBm_power(scenario, share_mem, power_type);
	g_dbm_power[power_type] = dbm_power;

	if (mt_mdpm_debug)
		pr_info("[md1_power] scenario_power=%d dbm_power=%d total=%d\n",
			scenario_power, dbm_power, scenario_power + dbm_power);

	return scenario_power + dbm_power;
}

static int mt_mdpm_debug_proc_show(struct seq_file *m, void *v)
{
	if (mt_mdpm_debug)
		seq_puts(m, "mdpm debug enabled\n");
	else
		seq_puts(m, "mdpm debug disabled\n");

	return 0;
}

/*
 * enable debug message
 */
static ssize_t mt_mdpm_debug_proc_write
(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;
	int debug = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);

	if (copy_from_user(desc, buffer, len))
		return 0;

	len = (len < 0) ? 0 : len;
	desc[len] = '\0';

	/* if (sscanf(desc, "%d", &debug) == 1) { */
	if (kstrtoint(desc, 10, &debug) == 0) {
		if (debug == 0)
			mt_mdpm_debug = 0;
		else if (debug == 1)
			mt_mdpm_debug = 1;
		else
			pr_notice("should be [0:disable,1:enable]\n");
	} else
		pr_notice("should be [0:disable,1:enable]\n");

	return count;
}

static int mt_mdpm_power_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "MAX power: scenario=%dmW dbm=%dmW total=%dmW\n",
		g_scenario_power[MAX_POWER], g_dbm_power[MAX_POWER],
		g_scenario_power[MAX_POWER] + g_dbm_power[MAX_POWER]);

	return 0;
}

#define PROC_FOPS_RW(name)						\
static int mt_ ## name ## _proc_open(struct inode *inode, struct file *file)\
{									\
	return single_open(file, mt_ ## name ## _proc_show, PDE_DATA(inode));\
}									\
static const struct proc_ops  mt_ ## name ## _proc_fops = {	\
	.proc_open		= mt_ ## name ## _proc_open,			\
	.proc_read		= seq_read,					\
	.proc_lseek		= seq_lseek,					\
	.proc_release	= single_release,				\
	.proc_write		= mt_ ## name ## _proc_write,			\
}

#define PROC_FOPS_RO(name)						\
static int mt_ ## name ## _proc_open(struct inode *inode, struct file *file)\
{									\
	return single_open(file, mt_ ## name ## _proc_show, PDE_DATA(inode));\
}									\
static const struct proc_ops  mt_ ## name ## _proc_fops = {	\
	.proc_open		= mt_ ## name ## _proc_open,		\
	.proc_read		= seq_read,				\
	.proc_lseek		= seq_lseek,				\
	.proc_release	= single_release,			\
}

#define PROC_ENTRY(name)	{__stringify(name), &mt_ ## name ## _proc_fops}

PROC_FOPS_RW(mdpm_debug);
PROC_FOPS_RO(mdpm_power);

static int mt_mdpm_create_procfs(void)
{
	struct proc_dir_entry *dir = NULL;
	int i;

	struct pentry {
		const char *name;
		const struct proc_ops  *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(mdpm_debug),
		PROC_ENTRY(mdpm_power),
	};

	dir = proc_mkdir("mdpm", NULL);

	if (!dir) {
		pr_err("fail to create /proc/mdpm @ %s()\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create
		    (entries[i].name, 0664, dir, entries[i].fops))
			pr_err("@%s: create /proc/mdpm/%s failed\n", __func__,
				    entries[i].name);
	}

	return 0;
}

#else /* MD_POWER_METER_ENABLE */
void init_md_section_level(enum pbm_kicker kicker)
{
	pr_notice("MD_POWER_METER_ENABLE:0\n");
}

int get_md1_power(enum mdpm_power_type power_type, bool need_update)
{
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
	return MAX_MD1_POWER;
#else
	return 0;
#endif
}
#endif /* MD_POWER_METER_ENABLE */

static int __init mdpm_module_init(void)
{
	int ret = 0;

#if MD_POWER_METER_ENABLE
	mt_mdpm_create_procfs();

#ifdef MD_POWER_UT
	mt_mdpm_debug = 1;
	init_md1_section_level(fake_share_mem);
	md_power_meter_ut();
#endif
#endif /* MD_POWER_METER_ENABLE */
	return ret;
}

static void __exit mdpm_module_exit(void)
{

}

EXPORT_SYMBOL(get_md1_power);
EXPORT_SYMBOL(init_md_section_level);

module_init(mdpm_module_init);
module_exit(mdpm_module_exit);
MODULE_DESCRIPTION("MDPM Driver v0.1");
MODULE_LICENSE("GPL");
