// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/smp.h>
/* #include <linux/earlysuspend.h> */
#include <linux/clk-provider.h>
#include <linux/io.h>

/* **** */
/* #include <mach/mt_typedefs.h> */
#include <mt-plat/sync_write.h>
/*#include "mt_clkmgr.h"*/
#include "mt6765_clkmgr.h"
/* #include <mach/mt_dcm.h> */
/* #include <mach/mt_idvfs.h> */ /* Fix when idvfs ready */
/*#include "mt_spm.h"*/
/*#include <mach/mt_spm_mtcmos.h>*/
/* #include <mach/mt_spm_sleep.h> */
/* #include <mach/mt_gpufreq.h> */
/* #include <mach/irqs.h> */

/* #include <mach/upmu_common.h> */
/* #include <mach/upmu_sw.h> */
/* #include <mach/upmu_hw.h> */
/*#include "mt_freqhopping_drv.h"*/

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#endif

#ifdef CONFIG_OF
void __iomem *clk_apmixed_base;
void __iomem *clk_mcucfg_base;
#endif


/************************************************
 **********         log debug          **********
 ************************************************/
#define TAG     "[Power/clkmgr] "

#define clk_info(fmt, args...)      \
	pr_info(TAG fmt, ##args)
#define clk_dbg(fmt, args...)       \
	pr_debug(TAG fmt, ##args)

/************************************************
 **********      register access       **********
 ************************************************/

#define clk_readl(addr) \
	readl(addr)
    /* DRV_Reg32(addr) */

#define clk_writel(addr, val)   \
	mt_reg_sync_writel(val, addr)

#define clk_setl(addr, val) \
	mt_reg_sync_writel(clk_readl(addr) | (val), addr)

#define clk_clrl(addr, val) \
	mt_reg_sync_writel(clk_readl(addr) & ~(val), addr)

/************************************************
 **********      struct definition     **********
 ************************************************/
/************************************************
 **********      global variablies     **********
 ************************************************/
/************************************************
 **********      spin lock protect     **********
 ************************************************/
/************************************************
 **********          pll part          **********
 ************************************************/
/************************************************
 **********         subsys part        **********
 ************************************************/
/*ARMPLL1*/
#define ARMPLL_CON0		(clk_apmixed_base + 0x20C)
#define ARMPLL_CON1		(clk_apmixed_base + 0x210)
#define ARMPLL_CON2		(clk_apmixed_base + 0x214)
#define ARMPLL_CON3		(clk_apmixed_base + 0x218)

/*ARMPLL2*/
#define ARMPLL_L_CON0		(clk_apmixed_base + 0x21C)
#define ARMPLL_L_CON1		(clk_apmixed_base + 0x220)
#define ARMPLL_L_CON2		(clk_apmixed_base + 0x224)
#define ARMPLL_L_CON3		(clk_apmixed_base + 0x228)

/*CCIPLL*/
#define CCIPLL_CON0		(clk_apmixed_base + 0x22C)
#define CCIPLL_CON1		(clk_apmixed_base + 0x230)
#define CCIPLL_CON2		(clk_apmixed_base + 0x234)
#define CCIPLL_CON3		(clk_apmixed_base + 0x238)

/*MMPLL*/
#define MMPLL_CON0		(clk_apmixed_base + 0x25C)
#define MMPLL_CON1		(clk_apmixed_base + 0x260)
#define MMPLL_CON2		(clk_apmixed_base + 0x264)
#define MMPLL_CON3		(clk_apmixed_base + 0x268)

/*GPUPLL*/
#define MFGPLL_CON0		(clk_apmixed_base + 0x24C)
#define MFGPLL_CON1		(clk_apmixed_base + 0x250)
#define MFGPLL_CON2		(clk_apmixed_base + 0x254)
#define MFGPLL_CON3		(clk_apmixed_base + 0x258)

/* MCUCFG Register */
#define MP0_PLL_DIV_CFG		(clk_mcucfg_base + 0x7A0) /*ARMPLL_LL*/
#define MP1_PLL_DIV_CFG		(clk_mcucfg_base + 0x7A4) /*ARMPLL_L*/
#define BUS_PLL_DIV_CFG		(clk_mcucfg_base + 0x7C0) /*CCIPLL*/

/************************************************
 **********         cg_clk part        **********
 ************************************************/

/************************************************
 **********       initialization       **********
 ************************************************/

/************************************************
 **********       function debug       **********
 ************************************************/
static void clk_dump(void)
{
	clk_info("[ARMPLL_LL_CON0]=0x%08x\n", clk_readl(ARMPLL_CON0));
	clk_info("[ARMPLL_LL_CON1]=0x%08x\n", clk_readl(ARMPLL_CON1));
	clk_info("[ARMPLL_LL_CON2]=0x%08x\n", clk_readl(ARMPLL_CON2));
	clk_info("[ARMPLL_LL_CON3]=0x%08x\n", clk_readl(ARMPLL_CON3));
	clk_info("[MP0_PLL_DIV_CFG]=0x%08x\n", clk_readl(MP0_PLL_DIV_CFG));
	clk_info("[CCF] ARMPLL(LL): %d\n", mt_get_abist_freq(22));

	clk_info("[ARMPLL_L_CON0]=0x%08x\n", clk_readl(ARMPLL_L_CON0));
	clk_info("[ARMPLL_L_CON1]=0x%08x\n", clk_readl(ARMPLL_L_CON1));
	clk_info("[ARMPLL_L_CON2]=0x%08x\n", clk_readl(ARMPLL_L_CON2));
	clk_info("[ARMPLL_L_CON3]=0x%08x\n", clk_readl(ARMPLL_L_CON3));
	clk_info("[MP1_PLL_DIV_CFG]=0x%08x\n", clk_readl(MP1_PLL_DIV_CFG));
	clk_info("[CCF] ARMPLL(L): %d\n", mt_get_abist_freq(21));

	clk_info("[CCIPLL_CON0]=0x%08x\n", clk_readl(CCIPLL_CON0));
	clk_info("[CCIPLL_CON1]=0x%08x\n", clk_readl(CCIPLL_CON1));
	clk_info("[CCIPLL_CON2]=0x%08x\n", clk_readl(CCIPLL_CON2));
	clk_info("[CCIPLL_CON3]=0x%08x\n", clk_readl(CCIPLL_CON3));
	clk_info("[BUS_PLL_DIV_CFG]=0x%08x\n", clk_readl(BUS_PLL_DIV_CFG));
	clk_info("[CCF] CCIPLL: %d\n", mt_get_abist_freq(49));
}

static int armpll1_fsel_read(struct seq_file *m, void *v)
{
	return 0;
}

static int armpll2_fsel_read(struct seq_file *m, void *v)
{
	return 0;
}

static int ccipll_fsel_read(struct seq_file *m, void *v)
{
	return 0;
}

static int mmpll_fsel_read(struct seq_file *m, void *v)
{
	return 0;
}

static int gpupll_fsel_read(struct seq_file *m, void *v)
{
	return 0;
}

static int pll_div_value_map(int index)
{
	int div = 0x08;

	switch (index) {
	case 1:
		div = 0x08;
	break;

	case 2:
		div = 0x0A;
	break;
	case 4:
		div = 0x0B;
	break;
	case 6:
		div = 0x1D;
	break;
	default:
		div = 0x08;
	break;
	}
	return div;
}
#define SDM_PLL_N_INFO_MASK 0x003FFFFF /*N_INFO[21:0]*/
#define ARMPLL_POSDIV_MASK  0x07000000 /*POSDIV[26:24]*/
#define SDM_PLL_N_INFO_CHG  0x80000000
#define ARMPLL_DIV_MASK	    0xFFE1FFFF /* DIVCK_SEL[21:17] */
#define ARMPLL_DIV_SHIFT    17
static ssize_t armpll1_fsel_write(struct file *file, const char __user *buffer,
				   size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;
	unsigned int ctrl_value = 0;
	int div;
	unsigned int value = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;
	desc[len] = '\0';

	if (sscanf(desc, "%x %x", &div, &value) == 2) {
		clk_dump();
		ctrl_value = clk_readl(ARMPLL_CON1);
		ctrl_value &= ~(SDM_PLL_N_INFO_MASK | ARMPLL_POSDIV_MASK);
		ctrl_value |= value &
			(SDM_PLL_N_INFO_MASK | ARMPLL_POSDIV_MASK);
		ctrl_value |= SDM_PLL_N_INFO_CHG;
		clk_writel(ARMPLL_CON1, ctrl_value);
		udelay(20);
		ctrl_value = clk_readl(MP0_PLL_DIV_CFG);
		ctrl_value &= ARMPLL_DIV_MASK;
		ctrl_value |= (pll_div_value_map(div) << ARMPLL_DIV_SHIFT);
		clk_writel(MP0_PLL_DIV_CFG, ctrl_value);
	}
	clk_dump();
	return count;
}

static ssize_t armpll2_fsel_write(struct file *file, const char __user *buffer,
				  size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;
	unsigned int ctrl_value = 0;
	int div;
	unsigned int value;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;
	desc[len] = '\0';

	if (sscanf(desc, "%x %x", &div, &value) == 2) {
		clk_dump();
		ctrl_value = clk_readl(ARMPLL_L_CON1);
		ctrl_value &= ~(SDM_PLL_N_INFO_MASK | ARMPLL_POSDIV_MASK);
		ctrl_value |= value &
			(SDM_PLL_N_INFO_MASK | ARMPLL_POSDIV_MASK);
		ctrl_value |= SDM_PLL_N_INFO_CHG;
		clk_writel(ARMPLL_L_CON1, ctrl_value);
		udelay(20);
		ctrl_value = clk_readl(MP1_PLL_DIV_CFG);
		ctrl_value &= ARMPLL_DIV_MASK;
		ctrl_value |= (pll_div_value_map(div) << ARMPLL_DIV_SHIFT);
		clk_writel(MP1_PLL_DIV_CFG, ctrl_value);
	}
	clk_dump();
	return count;
}

static ssize_t ccipll_fsel_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;
	unsigned int ctrl_value = 0;
	int div;
	unsigned int value;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;
	desc[len] = '\0';

	if (sscanf(desc, "%x %x", &div, &value) == 2) {
		clk_dump();
		ctrl_value = clk_readl(CCIPLL_CON1);
		ctrl_value &= ~(SDM_PLL_N_INFO_MASK | ARMPLL_POSDIV_MASK);
		ctrl_value |= value &
			(SDM_PLL_N_INFO_MASK | ARMPLL_POSDIV_MASK);
		ctrl_value |= SDM_PLL_N_INFO_CHG;
		clk_writel(CCIPLL_CON1, ctrl_value);
		udelay(20);
		ctrl_value = clk_readl(BUS_PLL_DIV_CFG);
		ctrl_value &= ARMPLL_DIV_MASK;
		ctrl_value |= (pll_div_value_map(div) << ARMPLL_DIV_SHIFT);
		clk_writel(BUS_PLL_DIV_CFG, ctrl_value);
	}
	clk_dump();
	return count;

}

static ssize_t mmpll_fsel_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *data)
{
		char desc[32];
		int len = 0;
		unsigned int con0_value, con1_value;

		len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
			if (copy_from_user(desc, buffer, len))
				return 0;
		desc[len] = '\0';
		if (sscanf(desc, "%x %x", &con1_value, &con0_value) == 2) {
			clk_writel(MMPLL_CON1, con1_value);
			clk_writel(MMPLL_CON0, con0_value);
			udelay(20);
		}
		return count;
}

static ssize_t gpupll_fsel_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *data)
{
		char desc[32];
		int len = 0;
		unsigned int con0_value, con1_value;

		len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
			if (copy_from_user(desc, buffer, len))
				return 0;
		desc[len] = '\0';
		if (sscanf(desc, "%x %x", &con1_value, &con0_value) == 2) {
			clk_writel(MFGPLL_CON1, con1_value);
			clk_writel(MFGPLL_CON0, con0_value);
			udelay(20);
		}
		return count;
}

static int mm_clk_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_abist_freq(27));
	return 0;
}

static int gpupll_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_abist_freq(25));
	return 0;
}

static int univpll_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_abist_freq(24));
	return 0;
}

/************ L ********************/
static int proc_armpll1_fsel_open(struct inode *inode, struct file *file)
{
	clk_info("%s", __func__);

	return single_open(file, armpll1_fsel_read, NULL);
}

static const struct file_operations armpll1_fsel_proc_fops = {
	.owner = THIS_MODULE,
	.open = proc_armpll1_fsel_open,
	.read = seq_read,
	.write = armpll1_fsel_write,
	.release = single_release,
};
/************ LL ********************/
static int proc_armpll2_fsel_open(struct inode *inode, struct file *file)
{
	clk_info("%s", __func__);

	return single_open(file, armpll2_fsel_read, NULL);
}

static const struct file_operations armpll2_fsel_proc_fops = {
	.owner = THIS_MODULE,
	.open = proc_armpll2_fsel_open,
	.read = seq_read,
	.write = armpll2_fsel_write,
	.release = single_release,
};
/************ CCI ********************/
static int proc_ccipll_fsel_open(struct inode *inode, struct file *file)
{
	clk_info("%s", __func__);

	return single_open(file, ccipll_fsel_read, NULL);
}

static const struct file_operations ccipll_fsel_proc_fops = {
	.owner = THIS_MODULE,
	.open = proc_ccipll_fsel_open,
	.read = seq_read,
	.write = ccipll_fsel_write,
	.release = single_release,
};
/************ MM ********************/
static int proc_mmpll_fsel_open(struct inode *inode, struct file *file)
{
	clk_info("%s", __func__);

	return single_open(file, mmpll_fsel_read, NULL);
}

static const struct file_operations mmpll_fsel_proc_fops = {
	.owner = THIS_MODULE,
	.open = proc_mmpll_fsel_open,
	.read = seq_read,
	.write = mmpll_fsel_write,
	.release = single_release,
};
/************ GPU ********************/
static int proc_gpupll_fsel_open(struct inode *inode, struct file *file)
{
	clk_info("%s", __func__);

	return single_open(file, gpupll_fsel_read, NULL);
}

static const struct file_operations gpupll_fsel_proc_fops = {
	.owner = THIS_MODULE,
	.open = proc_gpupll_fsel_open,
	.read = seq_read,
	.write = gpupll_fsel_write,
	.release = single_release,
};
/************ mm_clk ********************/
static int proc_mm_clk_open(struct inode *inode, struct file *file)
{
	return single_open(file, mm_clk_speed_dump_read, NULL);
}

static const struct file_operations mm_fops = {
	.owner = THIS_MODULE,
	.open = proc_mm_clk_open,
	.read = seq_read,
};
/************ gpupll ********************/
static int proc_gpupll_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpupll_speed_dump_read, NULL);
}

static const struct file_operations gpu_fops = {
	.owner = THIS_MODULE,
	.open = proc_gpupll_open,
	.read = seq_read,
};
/************ univpll ********************/
static int proc_univpll_open(struct inode *inode, struct file *file)
{
	return single_open(file, univpll_speed_dump_read, NULL);
}

static const struct file_operations univ_fops = {
	.owner = THIS_MODULE,
	.open = proc_univpll_open,
	.read = seq_read,
};


#ifdef CONFIG_OF
void iomap(void)
{
	struct device_node *node;

/*apmixed*/
	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6765-apmixedsys");
	if (!node)
		pr_info("[CLK_APMIXED] find node failed\n");
	clk_apmixed_base = of_iomap(node, 0);
	if (!clk_apmixed_base)
		pr_info("[CLK_APMIXED] base failed\n");

/*mcucfg*/
	node = of_find_compatible_node(NULL, NULL, "mediatek,mcucfg");
	if (!node)
		pr_info("[CLK_MCUCFG] find node failed\n");
	clk_mcucfg_base = of_iomap(node, 0);
	if (!clk_mcucfg_base)
		pr_info("[CLK_MCUCFG] base failed\n");

}
#endif


static int mt_clkmgr_debug_module_init(void)
{
	iomap();
	// mt_clkmgr_debug_init(); //TODO
	return 0;
}

module_init(mt_clkmgr_debug_module_init);

MODULE_LICENSE("GPL");
