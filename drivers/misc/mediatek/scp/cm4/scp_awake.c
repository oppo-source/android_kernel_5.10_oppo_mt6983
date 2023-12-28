// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/module.h>       /* needed by all modules */
#include <linux/init.h>         /* needed by module macros */
#include <linux/fs.h>           /* needed by file_operations* */
#include <linux/miscdevice.h>   /* needed by miscdevice* */
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/device.h>       /* needed by device_* */
#include <linux/vmalloc.h>      /* needed by vmalloc */
#include <linux/uaccess.h>      /* needed by copy_to_user */
#include <linux/fs.h>           /* needed by file_operations* */
#include <linux/slab.h>         /* needed by kmalloc */
#include <linux/poll.h>         /* needed by poll */
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_fdt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <mt-plat/aee.h>
#include <linux/delay.h>
#include "scp_feature_define.h"
#include "scp_ipi.h"
#include "scp_helper.h"
#include "scp_excep.h"
#include "scp_dvfs.h"

struct mutex scp_awake_mutexs[SCP_CORE_TOTAL];


/*
 * acquire scp lock flag, keep scp awake
 * @param scp_core_id: scp core id
 * return  0 :get lock success
 *        -1 :get lock timeout
 */
int scp_awake_lock(enum scp_core_id scp_id)
{
	unsigned long spin_flags;
	char *core_id;
	int *scp_awake_count;
	int count = 0;
	int ret = -1;
	unsigned int tmp;

	if (scp_id >= SCP_CORE_TOTAL) {
		pr_notice("%s: SCP ID >= SCP_CORE_TOTAL\n", __func__);
		return ret;
	}

	scp_awake_count = (int *)&scp_awake_counts[scp_id];
	core_id = core_ids[scp_id];

	if (is_scp_ready(scp_id) == 0) {
		pr_notice("%s: %s not enabled\n", __func__, core_id);
		return ret;
	}

	/* scp unlock awake */
	spin_lock_irqsave(&scp_awake_spinlock, spin_flags);
	if (*scp_awake_count > 0) {
		*scp_awake_count = *scp_awake_count + 1;
		spin_unlock_irqrestore(&scp_awake_spinlock, spin_flags);
		return 0;
	}

	/*set a direct IPI to awake SCP */
	/*pr_debug("scp_awake_lock: try to awake %s\n", core_id);*/
	writel(0xA0 | (1 << AP_AWAKE_LOCK), INFRA_IRQ_SET);

	count = 0;
	while (++count != SCP_AWAKE_TIMEOUT) {
		if (atomic_read(&scp_reset_status) == RESET_STATUS_START) {
			pr_notice("%s: resetting scp, break\n", __func__);
			break;
		}

		tmp = readl(INFRA_IRQ_SET);
		if ((tmp & 0xf0) != 0xA0) {
			pr_notice("%s: INFRA_IRQ_SET %x\n", __func__, tmp);
			break;
		}
		if (!((tmp & 0x0f) & (1 << AP_AWAKE_LOCK))) {
			ret = 0;
			break;
		}
		udelay(10);
	}
	/* clear status */
	writel(readl(INFRA_IRQ_SET), INFRA_IRQ_CLEAR);

	/* scp lock awake success*/
	if (ret != -1)
		*scp_awake_count = *scp_awake_count + 1;

	if (ret == -1) {
		pr_notice("%s: awake %s fail..\n", __func__, core_id);
		WARN_ON(1);
#if SCP_RECOVERY_SUPPORT
		if (scp_set_reset_status() == RESET_STATUS_STOP) {
			pr_notice("%s: start to reset scp...\n", __func__);
			scp_send_reset_wq(RESET_TYPE_AWAKE);
		} else
			pr_notice("%s: scp resetting\n", __func__);
#endif
	}

	spin_unlock_irqrestore(&scp_awake_spinlock, spin_flags);

	return ret;
}
EXPORT_SYMBOL_GPL(scp_awake_lock);

/*
 * release scp awake lock flag
 * @param scp_core_id: scp core id
 * return  0 :release lock success
 *        -1 :release lock fail
 */
int scp_awake_unlock(enum scp_core_id scp_id)
{
	unsigned long spin_flags;
	int *scp_awake_count;
	char *core_id;
	int count = 0;
	int ret = -1;
	unsigned int tmp;

	if (scp_id >= SCP_CORE_TOTAL) {
		pr_notice("%s: SCP ID >= SCP_CORE_TOTAL\n", __func__);
		return -1;
	}

	scp_awake_count = (int *)&scp_awake_counts[scp_id];
	core_id = core_ids[scp_id];

	if (is_scp_ready(scp_id) == 0) {
		pr_notice("%s: %s not enabled\n", __func__, core_id);
		return -1;
	}

	/* scp unlock awake */
	spin_lock_irqsave(&scp_awake_spinlock, spin_flags);
	if (*scp_awake_count > 1) {
		*scp_awake_count = *scp_awake_count - 1;
		spin_unlock_irqrestore(&scp_awake_spinlock, spin_flags);
		return 0;
	}

	/* WE1: set a direct IPI to release awake SCP */
	/*pr_debug("scp_awake_lock: try to awake %s\n", core_id);*/
	writel(0xA0 | (1 << AP_AWAKE_UNLOCK), INFRA_IRQ_SET);

	count = 0;
	while (++count != SCP_AWAKE_TIMEOUT) {
		tmp = readl(INFRA_IRQ_SET);
		if ((tmp & 0xf0) != 0xA0) {
			pr_notice("%s: INFRA7_IRQ_SET %x\n", __func__, tmp);
			break;
		}
		if (!((tmp & 0x0f) & (1 << AP_AWAKE_UNLOCK))) {
			ret = 0;
			break;
		}
		udelay(10);
	}
	/* clear status */
	writel(readl(INFRA_IRQ_SET), INFRA_IRQ_CLEAR);

	/* scp unlock awake success*/
	if (ret != -1) {
		if (*scp_awake_count <= 0)
			pr_notice("%s:%s awake_count=%d NOT SYNC!\n",
				__func__, core_id, *scp_awake_count);

		if (*scp_awake_count > 0)
			*scp_awake_count = *scp_awake_count - 1;
	}

	/* spinlock context safe */
	spin_unlock_irqrestore(&scp_awake_spinlock, spin_flags);

	return ret;
}
EXPORT_SYMBOL_GPL(scp_awake_unlock);

void scp_enable_sram(void)
{
	uint32_t reg_temp;

	/* enable sram, enable 1 block per time */
	for (reg_temp = 0xffffffff; reg_temp != 0;) {
		reg_temp = reg_temp >> 1;
		writel(reg_temp, SCP_SRAM_PDN);
	}

	/*
	 * l1c sram, 64K / tail sram, +32k
	 * L1_SRAM_PD / d_l1c_SRAM_PD / d_l1c_tag_SRAM_PD
	 * / p_l1c_SRAM_PD / p_l1c_tag_SRAM_PD
	 */
	writel(0, SCP_CLK_CTRL_L1_SRAM_PD);
	/* TCM_TAIL_SRAM_PD */
	writel(0, SCP_CLK_CTRL_TCM_TAIL_SRAM_PD);
}

/*
 * scp_sys_reset, reset scp
 */
#if SCP_RECOVERY_SUPPORT
int scp_sys_full_reset(void)
{
#if SCP_SYSTEM_RESET_SUPPORT
	#define INFRA_SLP_PROT_BITS (SCP_TO_INFRA_BIT | SCP_TO_AUDIO_BIT)
	#define SUBSYS_RESET_BITS (SCP_RESET_BIT | SCP_SEC_RESET_BIT)

	unsigned int reset_timeout;
	u32 tmp;

	pr_notice("[SCP]full reset start\n");

	pr_notice("[SCP]enable sleep protection\n");
	/*enable sleep protection*/
	reset_timeout = SCP_SYS_RESET_TIMEOUT;
	tmp = readl(INFRA_SLP_PROT_SET);
	writel(INFRA_SLP_PROT_BITS | tmp, INFRA_SLP_PROT_SET);
	dsb(SY);
	if ((readl(INFRA_SLP_PROT_STAT) & INFRA_SLP_PROT_BITS)
			!= INFRA_SLP_PROT_BITS) {
		pr_notice("[SCP]waiting sleep protection\n");
		reset_timeout = SCP_SYS_RESET_TIMEOUT;
		while ((reset_timeout > 0) &&
			((readl(INFRA_SLP_PROT_STAT) & INFRA_SLP_PROT_BITS)
					!= INFRA_SLP_PROT_BITS)) {
			mdelay(1);
			reset_timeout--;
		}

		if (reset_timeout == 0)
			pr_notice("[SCP]sleep protection timeout\n");
	}

	pr_notice("[SCP]set subsys reset\n");
	/*shut down scp subsys*/
	reset_timeout = SCP_SYS_RESET_TIMEOUT;
	tmp = readl(MODULE_RESET_STATUS);
	writel(SUBSYS_RESET_BITS | tmp, MODULE_RESET_SET);
	if ((readl(MODULE_RESET_STATUS) & SUBSYS_RESET_BITS)
			!= SUBSYS_RESET_BITS) {
		pr_notice("[SCP] sys reset waiting status...\n");
		while ((reset_timeout > 0) &&
			((readl(MODULE_RESET_STATUS) & SUBSYS_RESET_BITS)
			!= SUBSYS_RESET_BITS)) {
			mdelay(1);
			reset_timeout--;
		}

		if (reset_timeout)
			pr_notice("[SCP]full reset timeout\n");
	}

	/*finish the shutdown process by clearing reset bit*/
	pr_notice("[SCP]clear subsys reset\n");
	tmp = readl(MODULE_RESET_CLR);
	writel(SUBSYS_RESET_BITS | tmp, MODULE_RESET_CLR);

	/*enable scp sram*/
	pr_notice("[SCP]enable sram\n");
	scp_enable_sram();
#endif

	pr_notice("[SCP]copy scp to sram\n");
	/*copy loader to scp sram*/
	memcpy_to_scp(SCP_TCM, (const void *)(size_t)scp_loader_base_virt
		, scp_region_info_copy.ap_loader_size);
	/*set info to sram*/
	memcpy_to_scp(scp_region_info, (const void *)&scp_region_info_copy
		, sizeof(scp_region_info_copy));

#if SCP_SYSTEM_RESET_SUPPORT
	pr_notice("[SCP]disable sleep protection\n");
	/*disable sleep protection*/
	tmp = readl(INFRA_SLP_PROT_SET);
	writel(~INFRA_SLP_PROT_BITS & tmp,
		INFRA_SLP_PROT_SET);
#endif
	pr_notice("[SCP]full reset done\n");
	return 0;
}
#else
int scp_sys_full_reset(void) { return 0; }
#endif

MODULE_LICENSE("GPL");


