// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 * GenieZone (hypervisor-based seucrity platform) enables hardware protected
 * and isolated security execution environment, includes
 * 1. GZ hypervisor
 * 2. Hypervisor-TEE OS (built-in Trusty OS)
 * 3. Drivers (ex: debug, communication and interrupt) for GZ and
 *    hypervisor-TEE OS
 * 4. GZ and hypervisor-TEE and GZ framework (supporting multiple TEE
 *    ecosystem, ex: M-TEE, Trusty, GlobalPlatform, ...)
 */
/*
 * This is IPC driver
 *
 * For communication between client OS and hypervisor-TEE OS, saddIPC driver
 * is provided, including:
 * 1. standard call interface for communication and entering hypervisor-TEE
 * 2. virtio for message/command passing by shared memory
 * 3. IPC driver
 */

/* #define DEBUG */
#include <asm/compiler.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#if IS_ENABLED(CONFIG_MT_GZ_TRUSTY_DEBUGFS)
#include <linux/random.h>
#endif
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <gz-trusty/smcall.h>
#include <gz-trusty/sm_err.h>
#include <gz-trusty/trusty.h>
#include <linux/arm-smccc.h>
#include <uapi/linux/sched/types.h>
#include <linux/string.h>

#define enable_code 0 /*replace #if 0*/

#if enable_code /*#if 0*/
#if IS_ENABLED(CONFIG_MTK_AEE_IPANIC)
#include "trusty-ramconsole.h"
#endif
#endif

/* #define TRUSTY_SMC_DEBUG */

#if IS_ENABLED(TRUSTY_SMC_DEBUG)
#define trusty_dbg(fmt...) dev_dbg(fmt)
#define trusty_info(fmt...) dev_info(fmt)
#define trusty_err(fmt...) dev_info(fmt)
#else
#define trusty_dbg(fmt...)
#define trusty_info(fmt...) dev_info(fmt)
#define trusty_err(fmt...) dev_info(fmt)
#endif

#if IS_ENABLED(CONFIG_ARM64)
#define SMC_ARG0		"x0"
#define SMC_ARG1		"x1"
#define SMC_ARG2		"x2"
#define SMC_ARG3		"x3"
#define SMC_ARCH_EXTENSION	""
#define SMC_REGISTERS_TRASHED	"x4", "x5", "x6", "x7", "x8", "x9", "x10", \
				"x11", "x12", "x13", "x14", "x15", "x16", "x17"
#else
#define SMC_ARG0		"r0"
#define SMC_ARG1		"r1"
#define SMC_ARG2		"r2"
#define SMC_ARG3		"r3"
#define SMC_ARCH_EXTENSION	".arch_extension sec\n"
#define SMC_REGISTERS_TRASHED	"ip"
#endif
static inline ulong smc_asm(ulong r0, ulong r1, ulong r2, ulong r3)
{
	register ulong _r0 asm(SMC_ARG0) = r0;
	register ulong _r1 asm(SMC_ARG1) = r1;
	register ulong _r2 asm(SMC_ARG2) = r2;
	register ulong _r3 asm(SMC_ARG3) = r3;
	struct arm_smccc_res res;

	arm_smccc_smc(_r0, _r1, _r2, _r3, _r0, _r1, _r2, _r3, &res);

	return res.a0;
}

s32 trusty_fast_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	s = platform_get_drvdata(to_platform_device(dev));

	WARN_ON(!s);
	WARN_ON(!SMC_IS_FASTCALL(smcnr));
	WARN_ON(SMC_IS_SMC64(smcnr));

	return smc_asm(smcnr, a0, a1, a2);
}
EXPORT_SYMBOL(trusty_fast_call32);

#if IS_ENABLED(CONFIG_64BIT)
s64 trusty_fast_call64(struct device *dev, u64 smcnr, u64 a0, u64 a1, u64 a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	WARN_ON(!s);
	WARN_ON(!SMC_IS_FASTCALL(smcnr));
	WARN_ON(!SMC_IS_SMC64(smcnr));

	return smc_asm(smcnr, a0, a1, a2);
}
#endif

static inline bool is_busy(int ret)
{
	return (ret == SM_ERR_BUSY || ret == SM_ERR_GZ_BUSY
		|| ret == SM_ERR_NBL_BUSY);
}

static inline bool is_nop_call(u32 smc_nr)
{
	return (smc_nr == MTEE_SMCNR_TID(SMCF_SC_NOP, 0) ||
		smc_nr == MTEE_SMCNR_TID(SMCF_SC_NOP, 1));
}

static ulong trusty_std_call_inner(struct device *dev, ulong smcnr,
				   ulong a0, ulong a1, ulong a2)
{
	ulong ret;
	int retry = 5;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));
	uint32_t smcnr_tru_relast = MTEE_SMCNR_TID(SMCF_SC_RESTART_LAST, 0);
	uint32_t smcnr_nbl_relast = MTEE_SMCNR_TID(SMCF_SC_RESTART_LAST, 1);

	trusty_dbg(dev, "[%s/%s](0x%lx %lx %lx %lx)\n",
		   __func__, get_tee_name(s->tee_id), smcnr, a0, a1, a2);

	while (true) {
		ret = smc_asm(smcnr, a0, a1, a2);

		while ((s32) ret == SM_ERR_FIQ_INTERRUPTED)
			ret = smc_asm(MTEE_SMCNR(SMCF_SC_RESTART_FIQ, dev),
				      0, 0, 0);

		if (!is_busy(ret) || !retry)
			break;

		trusty_dbg(dev,
			   "[%s/%s](0x%lx %lx %lx %lx) ret %ld busy, retry\n",
			   __func__, get_tee_name(s->tee_id), smcnr, a0, a1, a2,
			   ret);
		retry--;
		if (is_nebula_tee(s->tee_id)) {
			if ((s32) ret == SM_ERR_NBL_BUSY &&
			     smcnr == smcnr_tru_relast)
				smcnr = smcnr_nbl_relast;
			else if ((s32) ret == SM_ERR_GZ_BUSY &&
				 smcnr == smcnr_nbl_relast)
				smcnr = smcnr_tru_relast;
		}
	}

	return ret;
}

static ulong trusty_std_call_helper(struct device *dev, ulong smcnr,
				    ulong a0, ulong a1, ulong a2)
{
	ulong ret;
	int sleep_time = 1;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	while (true) {
		local_irq_disable();
		atomic_notifier_call_chain(&s->notifier, TRUSTY_CALL_PREPARE,
					   NULL);
		if (is_nebula_tee(s->tee_id)) {
			/* For debug purpose.
			 * a0 = !is_nop_call(smcnr),
			 * 0 means NOP, 1 means STDCALL
			 * a1 = cpu core (get after local IRQ is disabled)
			 */
			if (smcnr == MTEE_SMCNR_TID(SMCF_SC_RESTART_LAST, 0) ||
			    smcnr == MTEE_SMCNR_TID(SMCF_SC_RESTART_LAST, 1))
				a1 = smp_processor_id();
		}
		ret = trusty_std_call_inner(dev, smcnr, a0, a1, a2);
		atomic_notifier_call_chain(&s->notifier, TRUSTY_CALL_RETURNED,
					   NULL);
		local_irq_enable();
		if (!is_busy(ret))
			break;

		if (sleep_time == 256)
			trusty_info(dev,
				    "%s(0x%lx 0x%lx 0x%lx 0x%lx) ret busy\n",
				    __func__, smcnr, a0, a1, a2);

		trusty_dbg(dev,
			   "%s(0x%lx 0x%lx 0x%lx 0x%lx) busy, wait %d ms\n",
			   __func__, smcnr, a0, a1, a2, sleep_time);
		msleep(sleep_time);
		if (sleep_time < 1000)
			sleep_time <<= 1;

		trusty_dbg(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) retry\n",
			   __func__, smcnr, a0, a1, a2);
	}

	if (sleep_time > 256)
		trusty_info(dev, "%s(0x%lx 0x%lx 0x%lx 0x%lx) busy cleared\n",
			    __func__, smcnr, a0, a1, a2);

	return ret;
}

static void trusty_std_call_cpu_idle(struct trusty_state *s)
{
	int ret;
	unsigned long timeout = HZ / 5; /* 200 ms */

	ret = wait_for_completion_timeout(&s->cpu_idle_completion, timeout);
	if (!ret) {
		pr_info("%s: time out wait cpu idle to clear, retry anyway\n",
			__func__);
	}
}

static int trusty_interrupted_loop(struct trusty_state *s, u32 smcnr, int ret)
{
	while (ret == SM_ERR_INTERRUPTED || ret == SM_ERR_CPU_IDLE) {

		trusty_dbg(s->dev, "[%s/%s] smc:0x%x ret:%d interrupted\n",
			   __func__, get_tee_name(s->tee_id), smcnr, ret);

		if (ret == SM_ERR_CPU_IDLE)
			trusty_std_call_cpu_idle(s);

		ret = trusty_std_call_helper(s->dev,
					MTEE_SMCNR_TID(SMCF_SC_RESTART_LAST, 0),
					0, 0, 0);
	}

	return ret;
}

static int nebula_interrupted_loop(struct trusty_state *s, u32 smcnr, int ret)
{
	while (ret == SM_ERR_GZ_INTERRUPTED || ret == SM_ERR_GZ_CPU_IDLE ||
	       ret == SM_ERR_NBL_INTERRUPTED || ret == SM_ERR_NBL_CPU_IDLE ||
	       ret == SM_ERR_INTERRUPTED || ret == SM_ERR_CPU_IDLE) {

		trusty_dbg(s->dev, "[%s/%s] smc:0x%x ret:%d interrupted\n",
			   __func__, get_tee_name(s->tee_id), smcnr, ret);

		if (ret == SM_ERR_GZ_CPU_IDLE || ret == SM_ERR_NBL_CPU_IDLE ||
		    ret == SM_ERR_CPU_IDLE)
			trusty_std_call_cpu_idle(s);

		if (ret == SM_ERR_NBL_INTERRUPTED || ret == SM_ERR_NBL_CPU_IDLE)
			ret = trusty_std_call_helper(s->dev,
					MTEE_SMCNR_TID(SMCF_SC_RESTART_LAST, 1),
					!is_nop_call(smcnr), 0, 0);
		else if (ret == SM_ERR_GZ_INTERRUPTED ||
			 ret == SM_ERR_GZ_CPU_IDLE)
			ret = trusty_std_call_helper(s->dev,
					SMC_SC_GZ_RESTART_LAST,
					!is_nop_call(smcnr), 0, 0);
		else
			ret = trusty_std_call_helper(s->dev,
					MTEE_SMCNR_TID(SMCF_SC_RESTART_LAST, 0),
					!is_nop_call(smcnr), 0, 0);
	}

	return ret;
}

s32 trusty_std_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	int ret;
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	s = platform_get_drvdata(to_platform_device(dev));

	WARN_ON(SMC_IS_FASTCALL(smcnr));
	WARN_ON(SMC_IS_SMC64(smcnr));

	if (smcnr != MTEE_SMCNR_TID(SMCF_SC_NOP, 0) &&
	    smcnr != MTEE_SMCNR_TID(SMCF_SC_NOP, 1)) {
		mutex_lock(&s->smc_lock);
		reinit_completion(&s->cpu_idle_completion);
	}

	trusty_dbg(dev, "[%s/%s](0x%x %x %x %x) started\n",
		   __func__, get_tee_name(s->tee_id), smcnr, a0, a1, a2);

	ret = trusty_std_call_helper(dev, smcnr, a0, a1, a2);

	if (is_trusty_tee(s->tee_id))	/* For Trusty */
		ret = trusty_interrupted_loop(s, smcnr, ret);
	else if (is_nebula_tee(s->tee_id))	/* For Nebula */
		ret = nebula_interrupted_loop(s, smcnr, ret);

	trusty_dbg(dev, "[%s/%s](0x%x %x %x %x) returned %d\n",
		   __func__, get_tee_name(s->tee_id), smcnr, a0, a1, a2, ret);

	WARN_ONCE(ret == SM_ERR_PANIC, "trusty crashed");

	if (smcnr == MTEE_SMCNR_TID(SMCF_SC_NOP, 0) ||
	    smcnr == MTEE_SMCNR_TID(SMCF_SC_NOP, 1))
		complete(&s->cpu_idle_completion);
	else {
		mutex_unlock(&s->smc_lock);
	}

	return ret;
}
EXPORT_SYMBOL(trusty_std_call32);

int trusty_call_notifier_register(struct device *dev, struct notifier_block *n)
{
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	s = platform_get_drvdata(to_platform_device(dev));
	return atomic_notifier_chain_register(&s->notifier, n);
}
EXPORT_SYMBOL(trusty_call_notifier_register);

int trusty_call_notifier_unregister(struct device *dev,
				    struct notifier_block *n)
{
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	s = platform_get_drvdata(to_platform_device(dev));
	return atomic_notifier_chain_unregister(&s->notifier, n);
}
EXPORT_SYMBOL(trusty_call_notifier_unregister);

int trusty_callback_notifier_register(struct device *dev,
				struct notifier_block *n)
{
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	s = platform_get_drvdata(to_platform_device(dev));
	return blocking_notifier_chain_register(&s->callback, n);
}
EXPORT_SYMBOL(trusty_callback_notifier_register);

int trusty_callback_notifier_unregister(struct device *dev,
				struct notifier_block *n)
{
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	s = platform_get_drvdata(to_platform_device(dev));
	return blocking_notifier_chain_unregister(&s->callback, n);
}
EXPORT_SYMBOL(trusty_callback_notifier_unregister);

int trusty_adjust_task_attr(struct device *dev,
				struct trusty_task_attr *manual_task_attr)
{
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	s = platform_get_drvdata(to_platform_device(dev));

	blocking_notifier_call_chain(&s->callback,
			TRUSTY_CALLBACK_VIRTIO_WQ_ATTR, manual_task_attr);

	return 0;
}
EXPORT_SYMBOL(trusty_adjust_task_attr);

int trusty_dump_systrace(struct device *dev, void *data)
{
#if ENABLE_GZ_TRACE_DUMP
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	s = platform_get_drvdata(to_platform_device(dev));

	blocking_notifier_call_chain(&s->callback,
			TRUSTY_CALLBACK_SYSTRACE, data);
#endif
	return 0;
}
EXPORT_SYMBOL(trusty_dump_systrace);

static int trusty_remove_child(struct device *dev, void *data)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

ssize_t trusty_version_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	return scnprintf(buf, PAGE_SIZE, "%s\n", s->version_str);
}

static ssize_t trusty_version_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t n)
{
	return n;
}
DEVICE_ATTR_RW(trusty_version);

#if enable_code /*#if 0*/
#if IS_ENABLED(CONFIG_MTK_AEE_IPANIC)
static void init_gz_ramconsole(struct device *dev)
{
	u32 low, high;
#ifdef SRAM_TEST

	low = 0x0011E000;
	high = 0x00000000;
#else
	unsigned long *gz_irq_pa;

	gz_irq_pa = aee_rr_rec_gz_irq_pa();
	trusty_info(dev, "ram console: get PA %p\n", gz_irq_pa);

	low = (u32) (((u64) gz_irq_pa << 32) >> 32);
	high = (u32) ((u64) gz_irq_pa >> 32);
#endif

	trusty_info(dev,
		"ram console: send ram console PA from kernel to GZ\n");
	trusty_std_call32(dev, MT_SMC_SC_SET_RAMCONSOLE, low, high, 0);
}
#endif
#endif

const char *trusty_version_str_get(struct device *dev)
{
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return ERR_PTR(-EINVAL);

	s = platform_get_drvdata(to_platform_device(dev));
	return s->version_str;
}
EXPORT_SYMBOL(trusty_version_str_get);

static void trusty_init_version(struct trusty_state *s, struct device *dev)
{
	int ret;
	int i;
	int version_str_len;
	u32 smcnr_version = MTEE_SMCNR(SMCF_FC_GET_VERSION_STR, dev);

	ret = trusty_fast_call32(dev, smcnr_version, -1, 0, 0);
	if (ret <= 0)
		goto err_get_size;

	version_str_len = ret;

	s->version_str = devm_kzalloc(dev, version_str_len + 1, GFP_KERNEL);

	if (!s->version_str)
		goto err_nomem;

	for (i = 0; i < version_str_len; i++) {
		ret = trusty_fast_call32(dev, smcnr_version, i, 0, 0);
		if (ret < 0)
			goto err_get_char;
		s->version_str[i] = ret;
	}
	s->version_str[i] = '\0';

	trusty_info(dev, "trusty version: %s\n", s->version_str);

	ret = device_create_file(dev, &dev_attr_trusty_version);
	if (ret)
		goto err_create_file;

	return;

err_create_file:
err_get_char:
	devm_kfree(dev, s->version_str);
	s->version_str = NULL;
err_nomem:
err_get_size:
	trusty_info(dev, "failed to get version: %d\n", ret);
}

u32 trusty_get_api_version(struct device *dev)
{
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return -EINVAL;

	s = platform_get_drvdata(to_platform_device(dev));
	return s->api_version;
}
EXPORT_SYMBOL(trusty_get_api_version);

static int trusty_init_api_version(struct trusty_state *s, struct device *dev)
{
	u32 api_version;
	uint32_t smcnr_api = MTEE_SMCNR(SMCF_FC_API_VERSION, dev);

	api_version = trusty_fast_call32(dev, smcnr_api,
					 TRUSTY_API_VERSION_CURRENT,
					 s->tee_id, 0);
	if (api_version == SM_ERR_UNDEFINED_SMC)
		api_version = 0;

	if (api_version > TRUSTY_API_VERSION_CURRENT) {
		trusty_info(dev, "unsupported api version %u > %u\n",
			    api_version, TRUSTY_API_VERSION_CURRENT);
		return -EINVAL;
	}

	trusty_info(dev, "selected api version: %u (requested %u)\n",
		    api_version, TRUSTY_API_VERSION_CURRENT);
	s->api_version = api_version;

	return 0;
}

static bool dequeue_nop(struct trusty_state *s, u32 *args, struct list_head *nop_queue)
{
	unsigned long flags;
	struct trusty_nop *nop = NULL;

	spin_lock_irqsave(&s->nop_lock, flags);

	if (!list_empty(nop_queue)) {
		nop = list_first_entry(nop_queue, struct trusty_nop, node);
		list_del_init(&nop->node);
		args[0] = nop->args[0];
		args[1] = nop->args[1];
		args[2] = nop->args[2];
	} else {
		args[0] = 0;
		args[1] = 0;
		args[2] = 0;
	}

	spin_unlock_irqrestore(&s->nop_lock, flags);

	return nop;
}

static void locked_nop_work_func(struct nop_task_info *nop_ti)
{
	int ret;
	struct trusty_state *s = nop_ti->ts;
	u32 smcnr_locked_nop = MTEE_SMCNR_TID(SMCF_SC_LOCKED_NOP, s->tee_id);

	ret = trusty_std_call32(s->dev, smcnr_locked_nop, 0, 0, 0);

	if (ret != 0)
		trusty_info(s->dev, "%s: SMC_SC_LOCKED_NOP failed %d\n",
			    __func__, ret);
}

static void nop_work_func(struct nop_task_info *nop_ti)
{
	struct trusty_state *s = nop_ti->ts;
	bool next;
	enum tee_id_t tee_id = s->tee_id;
	int ret;
	u32 args[3];
	u32 smcnr_nop = MTEE_SMCNR_TID(SMCF_SC_NOP, tee_id);

	trusty_dbg(s->dev, "%s: idx %d\n", __func__, nop_ti->idx);

	dequeue_nop(s, args, &nop_ti->nop_queue);

	do {
		trusty_dbg(s->dev, "%s: %x %x %x\n",
			   __func__, args[0], args[1], args[2]);

		ret = trusty_std_call32(s->dev, smcnr_nop,
					args[0], args[1], args[2]);

		if (ret == SM_ERR_NOP_INTERRUPTED)
			smcnr_nop = MTEE_SMCNR_TID(SMCF_SC_NOP, 0);
		else if (ret == SM_ERR_NBL_NOP_INTERRUPTED)
			smcnr_nop = MTEE_SMCNR_TID(SMCF_SC_NOP, 1);
		else
			smcnr_nop = MTEE_SMCNR_TID(SMCF_SC_NOP, 0);


		next = dequeue_nop(s, args, &nop_ti->nop_queue);

		if (ret == SM_ERR_NBL_NOP_INTERRUPTED ||
		    ret == SM_ERR_GZ_NOP_INTERRUPTED ||
		    ret == SM_ERR_NOP_INTERRUPTED)
			next = true;
		else if ((ret != SM_ERR_GZ_NOP_DONE) &&
			 (ret != SM_ERR_NBL_NOP_DONE) &&
			 (ret != SM_ERR_NOP_DONE))
			trusty_info(s->dev, "%s: tee_id %d, smc failed %d\n",
				    __func__, tee_id, ret);
	} while (next);

	trusty_dbg(s->dev, "%s: done\n", __func__);

	blocking_notifier_call_chain(&s->callback, TRUSTY_CALLBACK_SYSTRACE, NULL);

}

void trusty_enqueue_nop(struct device *dev, struct trusty_nop *nop, int cpu)
{
	unsigned long flags;
	struct nop_task_info *nop_ti;
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return;

	s = platform_get_drvdata(to_platform_device(dev));

	preempt_disable();
	if (cpu_possible(cpu))
		nop_ti = per_cpu_ptr(s->nop_tasks_info, cpu);
	else
		nop_ti = this_cpu_ptr(s->nop_tasks_info);

	if (nop) {
		WARN_ON(s->api_version < TRUSTY_API_VERSION_SMP_NOP);

		spin_lock_irqsave(&s->nop_lock, flags);
		if (list_empty(&nop->node))
			list_add_tail(&nop->node, &nop_ti->nop_queue);
		spin_unlock_irqrestore(&s->nop_lock, flags);
	}

	complete(&nop_ti->run);
	preempt_enable();

	return;
}
EXPORT_SYMBOL(trusty_enqueue_nop);

void trusty_dequeue_nop(struct device *dev, struct trusty_nop *nop)
{
	unsigned long flags;
	struct trusty_state *s;

	if (IS_ERR_OR_NULL(dev))
		return;

	s = platform_get_drvdata(to_platform_device(dev));

	if (WARN_ON(!nop))
		return;

	spin_lock_irqsave(&s->nop_lock, flags);
	if (nop) {
		if (!list_empty(&nop->node))
			list_del_init(&nop->node);
	}
	spin_unlock_irqrestore(&s->nop_lock, flags);
}
EXPORT_SYMBOL(trusty_dequeue_nop);

static int trusty_task_nop(void *data)
{
	struct nop_task_info *nop_ti = (struct nop_task_info *)data;
	struct trusty_state *s;
	long timeout = MAX_SCHEDULE_TIMEOUT;
	int idx;

	if (!nop_ti)
		return -ENOMEM;

	s = nop_ti->ts;
	if (!s)
		return -ENOMEM;

	complete(&nop_ti->rdy);

	idx = nop_ti->idx;

	trusty_info(s->dev, "tee%d/%s_%d ->\n", s->tee_id, __func__, idx);

	while (!kthread_should_stop()) {
		wait_for_completion_interruptible_timeout(&nop_ti->run, timeout);

		if (nop_ti->idx >= 0) {
			nop_ti->nop_func(nop_ti);
			// blocking_notifier_call_chain(
			//	&s->callback, TRUSTY_CALLBACK_SYSTRACE, NULL);
		} else
			break;

		if (unlikely(smp_processor_id() != idx)) {
			/* self migrate */
			if (cpu_online(idx)) {
				struct cpumask cpu_mask;

				cpumask_clear(&cpu_mask);
				cpumask_set_cpu(idx, &cpu_mask);
				set_cpus_allowed_ptr(current, &cpu_mask);
				trusty_info(s->dev, "%s migrate to cpu %d\n",
					    __func__, idx);
			} else
				trusty_info(s->dev, "%s cpu %d is offline\n",
					    __func__, idx);
		}
	}

	trusty_info(s->dev, "tee%d/%s_%d -<\n", s->tee_id, __func__, idx);

	return 0;
}

static int trusty_nop_thread_free(struct trusty_state *s)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct nop_task_info *nop_ti = per_cpu_ptr(s->nop_tasks_info, cpu);
		struct task_struct *ts = per_cpu_ptr(s->nop_tasks_fd, cpu);

		if (IS_ERR_OR_NULL(nop_ti) || IS_ERR_OR_NULL(ts))
			continue;

		nop_ti->idx = -1;
		complete(&nop_ti->run);
		//kthread_stop(ts);
		trusty_info(s->dev, "tee%d %s cpu=%u\n", s->tee_id, __func__, cpu);
	}

	return 0;
}

static int trusty_nop_thread_create(struct trusty_state *s)
{
	unsigned int cpu;
	int ret;

	s->nop_tasks_fd = devm_alloc_percpu(s->dev, struct task_struct);
	if (!s->nop_tasks_fd) {
		trusty_info(s->dev, "Failed to allocate nop_tasks_fd\n");
		return -ENOMEM;
	}

	s->nop_tasks_info = devm_alloc_percpu(s->dev, struct nop_task_info);
	if (!s->nop_tasks_info) {
		trusty_info(s->dev, "Failed to allocate nop_tasks_info\n");
		return -ENOMEM;
	}

	for_each_possible_cpu(cpu) {
		struct task_struct *ts = per_cpu_ptr(s->nop_tasks_fd, cpu);
		struct nop_task_info *nop_ti = per_cpu_ptr(s->nop_tasks_info, cpu);

		nop_ti->idx = cpu;
		nop_ti->ts = s;
		if (s->api_version < TRUSTY_API_VERSION_SMP)
			nop_ti->nop_func = locked_nop_work_func;
		else
			nop_ti->nop_func = nop_work_func;

		init_completion(&nop_ti->run);
		init_completion(&nop_ti->rdy);
		INIT_LIST_HEAD(&nop_ti->nop_queue);

		ts = kthread_create_on_node(trusty_task_nop, (void *)nop_ti,
					    cpu_to_node(cpu), "id%d_trusty_n/%d",
					    s->tee_id, cpu);
		if (IS_ERR(ts)) {
			trusty_info(s->dev, "%s unable create kthread\n", __func__);
			ret = PTR_ERR(ts);
			goto err_thread_create;
		}
		set_user_nice(ts, PRIO_TO_NICE(MAX_USER_RT_PRIO) + 1);
		kthread_bind(ts, cpu);

		wake_up_process(ts);
	}

	for_each_possible_cpu(cpu) {
		struct nop_task_info *nop_ti = per_cpu_ptr(s->nop_tasks_info, cpu);

		ret = wait_for_completion_timeout(&nop_ti->rdy, msecs_to_jiffies(5000));
		if (ret <= 0)
			goto err_thread_rdy;
	}

	return 0;

err_thread_rdy:
err_thread_create:
	trusty_nop_thread_free(s);
	return ret;
}

static int trusty_poll_notify(struct notifier_block *nb, unsigned long action,
		       void *data)
{
	struct trusty_state *s;

	if (action != TRUSTY_CALL_RETURNED)
		return NOTIFY_DONE;

	s = container_of(nb, struct trusty_state, poll_notifier);

	if (s->api_version < TRUSTY_API_VERSION_SMP_NOP)
		return NOTIFY_DONE;

	kthread_queue_work(&s->poll_worker, &s->poll_work);

	return NOTIFY_OK;
}

static void trusty_poll_work(struct kthread_work *work)
{
	struct trusty_state *s = container_of(work, struct trusty_state, poll_work);
	uint32_t cpu_mask;

	cpu_mask = (uint32_t)trusty_fast_call32(s->dev,
						SMC_FC_GZ_GET_CPU_REQUEST,
						0, 0, 0);

	if (cpu_mask == SM_ERR_UNDEFINED_SMC) {
		trusty_info(s->dev, "%s get error cpu mask!\n", __func__);
		return;
	}

	if (cpu_mask > 0) {
		int cpu;

		for_each_online_cpu(cpu) {
			if ((cpu < 32) && (cpu_mask & (1 << cpu))) {
				trusty_info(s->dev, "%s send nop for cpu %d\n",
						__func__, cpu);
				trusty_enqueue_nop(s->dev, NULL, cpu);
			}
		}
	}
}

static int trusty_poll_create(struct trusty_state *s)
{
	// int ret;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	s->poll_notifier.notifier_call = trusty_poll_notify;
	s->poll_notifier.priority = -1;
	/* ret = trusty_call_notifier_register(s->dev, &s->poll_notifier);
	if (ret) {
		trusty_info(s->dev,
			 "%s: failed (%d) to register notifier\n",
			 __func__, ret);
		return ret;
	}
	*/

	kthread_init_work(&s->poll_work, trusty_poll_work);
	kthread_init_worker(&s->poll_worker);
	s->poll_task = kthread_create(kthread_worker_fn, (void *)&s->poll_worker,
				      "trusty_poll_task");
	if (IS_ERR(s->poll_task)) {
		trusty_info(s->dev, "%s: unable create trusty_poll_worker\n",
			    __func__, s->tee_id);
		return PTR_ERR(s->poll_task);
	}

	sched_setscheduler(s->poll_task, SCHED_RR, &param);
	set_user_nice(s->poll_task, PRIO_TO_NICE(100));

	wake_up_process(s->poll_task);

	return 0;
}

static int trusty_poll_free(struct trusty_state *s)
{
	kthread_flush_worker(&s->poll_worker);
	kthread_stop(s->poll_task);

	trusty_call_notifier_unregister(s->dev, &s->poll_notifier);

	return 0;
}

static int trusty_probe(struct platform_device *pdev)
{
	int ret, tee_id = 0;
	struct trusty_state *s;
	struct device_node *node = pdev->dev.of_node;

	if (!node) {
		trusty_info(&pdev->dev, "of_node required\n");
		return -EINVAL;
	}

	/* For multiple TEEs */
	ret = of_property_read_u32(node, "tee-id", &tee_id);
	if (ret != 0 && !is_tee_id(tee_id)) {
		dev_info(&pdev->dev,
			 "[%s] ERROR: tee_id is not set on device tree\n",
			 __func__);
		return -EINVAL;
	}

	dev_info(&pdev->dev, "--- init trusty-smc for MTEE %d ---\n", tee_id);

	s = devm_kzalloc(&pdev->dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	/* set tee_id as early as possible */
	pdev->id = tee_id;
	s->tee_id = tee_id;

	s->dev = &pdev->dev;
	spin_lock_init(&s->nop_lock);

	mutex_init(&s->smc_lock);
	ATOMIC_INIT_NOTIFIER_HEAD(&s->notifier);
	BLOCKING_INIT_NOTIFIER_HEAD(&s->callback);
	init_completion(&s->cpu_idle_completion);
	platform_set_drvdata(pdev, s);

	/* init SMC number table before any SMC call. */
	ret = init_smcnr_table(&pdev->dev, s->tee_id);
	if (ret < 0) {
		trusty_info(&pdev->dev, "Failed to init smc number table\n");
		goto err_smcall_table;
	}

	trusty_init_version(s, &pdev->dev);

	ret = trusty_init_api_version(s, &pdev->dev);
	if (ret < 0)
		goto err_api_version;

	ret = trusty_nop_thread_create(s);
	if (ret < 0)
		goto err_nop_thread_create;

	ret = trusty_poll_create(s);
	if (ret)
		goto err_poll_create;

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret < 0) {
		trusty_info(&pdev->dev, "Failed to add children: %d\n", ret);
		goto err_add_children;
	}

#if IS_ENABLED(CONFIG_MT_GZ_TRUSTY_DEBUGFS)
	mtee_create_debugfs(s, &pdev->dev);
#else
	trusty_info(&pdev->dev, "%s, Not MT_GZ_TRUSTY_DEBUGFS\n", __func__);
#endif

#if enable_code /*#if 0*/
#if IS_ENABLED(CONFIG_MTK_AEE_IPANIC)
	init_gz_ramconsole(&pdev->dev);
#endif
#endif
	return 0;

err_add_children:
	trusty_poll_free(s);
err_poll_create:
	trusty_nop_thread_free(s);
err_nop_thread_create:
err_api_version:
	if (s->version_str) {
		device_remove_file(&pdev->dev, &dev_attr_trusty_version);
	}
err_smcall_table:
	device_for_each_child(&pdev->dev, NULL, trusty_remove_child);
	mutex_destroy(&s->smc_lock);
	return ret;
}

static int trusty_remove(struct platform_device *pdev)
{
	struct trusty_state *s = platform_get_drvdata(pdev);

	device_for_each_child(&pdev->dev, NULL, trusty_remove_child);

	trusty_poll_free(s);

	trusty_nop_thread_free(s);

	mutex_destroy(&s->smc_lock);
	if (s->version_str) {
		device_remove_file(&pdev->dev, &dev_attr_trusty_version);
	}

	return 0;
}

static const struct of_device_id trusty_of_match[] = {
	{.compatible = "android,trusty-smc-v1",},
	{},
};

static struct platform_driver trusty_driver = {
	.probe = trusty_probe,
	.remove = trusty_remove,
	.driver = {
		   .name = "trusty",
		   .owner = THIS_MODULE,
		   .of_match_table = trusty_of_match,
		   },
};

static const struct of_device_id nebula_of_match[] = {
	{.compatible = "android,nebula-smc-v1",},
	{},
};

static struct platform_driver nebula_driver = {
	.probe = trusty_probe,
	.remove = trusty_remove,
	.driver = {
		   .name = "nebula",
		   .owner = THIS_MODULE,
		   .of_match_table = nebula_of_match,
		   },
};

static int __init trusty_driver_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&trusty_driver);
	if (ret)
		goto err_trusty_driver;

	ret = platform_driver_register(&nebula_driver);
	if (ret)
		goto err_nebula_driver;

	return ret;

err_nebula_driver:
err_trusty_driver:
	pr_info("Platform driver register failed\n");
	return -ENODEV;
}

static void __exit trusty_driver_exit(void)
{
	/* remove trusty driver */
	platform_driver_unregister(&trusty_driver);
	/* remove nebula driver */
	platform_driver_unregister(&nebula_driver);
}

//subsys_initcall(trusty_driver_init);
arch_initcall(trusty_driver_init);
module_exit(trusty_driver_exit);

MODULE_LICENSE("GPL");

