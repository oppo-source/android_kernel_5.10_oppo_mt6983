// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 */

#include <linux/io.h>
#if IS_ENABLED(CONFIG_OF_RESERVED_MEM)
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#endif
#if IS_ENABLED(CONFIG_MTK_EMI)
#include <soc/mediatek/emi.h>
#endif
#include "adsp_reserved_mem.h"
#include "adsp_platform.h"
#include "adsp_core.h"


#define ADSP_MEM_RESERVED_KEY "mediatek,reserve-memory-adsp_share"
#define ADSP_RESERVE_MEMORY_BLOCK(xname) \
		{.phys_addr = 0x0, .virt_addr = NULL, \
		 .size = 0, .name = xname}

static struct adsp_reserve_mblock adsp_reserve_mem = {0};

static struct adsp_reserve_mblock adsp_reserve_mblocks[] = {
	[ADSP_A_IPI_DMA_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-ipidma-a"),
	[ADSP_B_IPI_DMA_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-ipidma-b"),
	[ADSP_A_LOGGER_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-logger-a"),
	[ADSP_B_LOGGER_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-logger-b"),
	[ADSP_C2C_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-c2c"),
	[ADSP_A_DEBUG_DUMP_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-dbg-dump-a"),
	[ADSP_B_DEBUG_DUMP_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-dbg-dump-b"),
	[ADSP_A_CORE_DUMP_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-core-dump-a"),
	[ADSP_B_CORE_DUMP_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-core-dump-b"),
#if !IS_ENABLED(CONFIG_FPGA_EARLY_PORTING)
	[ADSP_AUDIO_COMMON_MEM_ID]
		= ADSP_RESERVE_MEMORY_BLOCK("adsp-rsv-audio"),
#endif
};

static struct adsp_reserve_mblock *adsp_get_reserve_mblock(
					enum adsp_reserve_mem_id_t id)
{
	void *va_start = adsp_reserve_mblocks[0].virt_addr;

	if (id >= ADSP_NUMS_MEM_ID) {
		pr_info("%s no reserve memory for %d\n", __func__, id);
		return NULL;
	}
	if (!va_start) {
		pr_info("%s va_start is NULL\n", __func__);
		return NULL;
	}

	return &adsp_reserve_mblocks[id];
}

phys_addr_t adsp_get_reserve_mem_phys(enum adsp_reserve_mem_id_t id)
{
	struct adsp_reserve_mblock *mblk = adsp_get_reserve_mblock(id);

	return mblk ? mblk->phys_addr : 0;
}
EXPORT_SYMBOL(adsp_get_reserve_mem_phys);

void *adsp_get_reserve_mem_virt(enum adsp_reserve_mem_id_t id)
{
	struct adsp_reserve_mblock *mblk = adsp_get_reserve_mblock(id);

	return mblk ? mblk->virt_addr : NULL;
}
EXPORT_SYMBOL(adsp_get_reserve_mem_virt);

size_t adsp_get_reserve_mem_size(enum adsp_reserve_mem_id_t id)
{
	struct adsp_reserve_mblock *mblk = adsp_get_reserve_mblock(id);

	return mblk ? mblk->size : 0;
}
EXPORT_SYMBOL(adsp_get_reserve_mem_size);

void adsp_set_emimpu_shared_region(void)
{
#if IS_ENABLED(CONFIG_MTK_EMI)
	struct emimpu_region_t adsp_region;
	struct adsp_reserve_mblock *mem = &adsp_reserve_mem;
	int ret = 0;

	ret = mtk_emimpu_init_region(&adsp_region,
				     MPU_PROCT_REGION_ADSP_SHARED);
	if (ret < 0)
		pr_info("%s fail to init emimpu region\n", __func__);
	mtk_emimpu_set_addr(&adsp_region, mem->phys_addr,
			    (mem->phys_addr + mem->size - 0x1));
	mtk_emimpu_set_apc(&adsp_region, MPU_PROCT_D0_AP,
			   MTK_EMIMPU_NO_PROTECTION);
	mtk_emimpu_set_apc(&adsp_region, MPU_PROCT_D10_ADSP,
			   MTK_EMIMPU_NO_PROTECTION);
	ret = mtk_emimpu_set_protection(&adsp_region);
	if (ret < 0)
		pr_info("%s fail to set emimpu protection\n", __func__);
	mtk_emimpu_free_region(&adsp_region);
#else
	pr_info("%s(), emi config not enable\n", __func__);
#endif
}

static int adsp_init_reserve_memory(struct platform_device *pdev, struct adsp_reserve_mblock *mem)
{
	struct device_node *node;
	struct reserved_mem *rmem;
	struct device *dev = &pdev->dev;
	u64 mem_info[2];
	int ret;

	/* Reserved memory allocated from lk */
	ret = of_property_read_u64_array(dev->of_node, "shared_memory", mem_info, 2);
	if (!ret) {
		mem->phys_addr = (phys_addr_t)mem_info[0];
		mem->size = (size_t)mem_info[1];
		pr_info("%s(), get \"shared_memory\" property from dts, (%llx, %zx)\n",
				__func__, mem->phys_addr, mem->size);
		goto RSV_IOREMAP;
	}
	/* Otherwise, get reserved memory from reserved node */
	node = of_find_compatible_node(NULL, NULL, ADSP_MEM_RESERVED_KEY);
	if (!node) {
		pr_info("%s(), no node for reserved memory\n", __func__);
		return -ENOMEM;
	}
	rmem = of_reserved_mem_lookup(node);
	if (!rmem) {
		pr_info("%s(), cannot lookup reserved memory\n", __func__);
		return -ENOMEM;
	}

	if (!rmem->base || !rmem->size) {
		pr_info("%s() reserve memory illegal addr:%llx, size:%llx\n",
			__func__, rmem->base, rmem->size);
		return -ENOMEM;
	} else {
		mem->phys_addr = rmem->base;
		mem->size = (size_t)rmem->size;
		/* set mpu of shared memory to emi */
		adsp_set_emimpu_shared_region();
	}

RSV_IOREMAP:
	mem->virt_addr = ioremap_wc(mem->phys_addr, mem->size);
	if (!mem->virt_addr) {
		pr_info("%s() ioremap fail\n", __func__);
		return -ENOMEM;
	}

	return 0;
}

int adsp_mem_device_probe(struct platform_device *pdev)
{
	int ret = 0;
	enum adsp_reserve_mem_id_t id;
	struct adsp_reserve_mblock *mem = &adsp_reserve_mem;
	size_t acc_size = 0;
	u32 size;

	ret = adsp_init_reserve_memory(pdev, mem);
	if (ret)
		return ret;

	for (id = 0; id < ADSP_NUMS_MEM_ID; id++) {
		of_property_read_u32(pdev->dev.of_node,
		      adsp_reserve_mblocks[id].name,
		      &size);
		if (!ret)
			adsp_reserve_mblocks[id].size = (size_t)size;
	}

	/* assign to each memory block */
	for (id = 0; id < ADSP_NUMS_MEM_ID; id++) {
		adsp_reserve_mblocks[id].phys_addr = mem->phys_addr + acc_size;
		adsp_reserve_mblocks[id].virt_addr = mem->virt_addr + acc_size;
		acc_size += ALIGN(adsp_reserve_mblocks[id].size, RSV_BLOCK_ALIGN);
#ifdef MEM_DEBUG
		pr_info("adsp_reserve_mblocks[%d] phys_addr:%llx, size:0x%zx\n",
			id,
			adsp_reserve_mblocks[id].phys_addr,
			adsp_reserve_mblocks[id].size);
#endif
	}

	if (acc_size > mem->size) {
		pr_info("%s(), not enough of memory use(0x%zx) > total(0x%zx)\n",
			__func__, acc_size, mem->size);
		return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL(adsp_mem_device_probe);

ssize_t adsp_reserve_memory_dump(char *buffer, int size)
{
	int n = 0, i = 0;
	struct adsp_reserve_mblock *mem = &adsp_reserve_mem;

	n += scnprintf(buffer + n, size - n,
		"Reserve-memory-all:0x%llx 0x%p 0x%zx\n",
		mem->phys_addr, mem->virt_addr, mem->size);

	for (i = 0; i < ADSP_NUMS_MEM_ID; i++) {
		mem = &adsp_reserve_mblocks[i];
		n += scnprintf(buffer + n, size - n,
			"Reserve-memory-Block[%02d]:0x%llx 0x%p 0x%zx\n",
			i, mem->phys_addr, mem->virt_addr, mem->size);
	}
	return n;
}

void adsp_update_mpu_memory_info(struct adsp_priv *pdata)
{
	struct adsp_mpu_info_t mpu_info;

	mpu_info.share_dram_addr = (u32)adsp_reserve_mem.phys_addr;
	mpu_info.share_dram_size = (u32)adsp_reserve_mem.size;

	pr_info("[ADSP] mpu info=(0x%x, 0x%x)\n",
		 mpu_info.share_dram_addr, mpu_info.share_dram_size);
	adsp_copy_to_sharedmem(pdata, ADSP_SHAREDMEM_MPUINFO,
		&mpu_info, sizeof(struct adsp_mpu_info_t));
}
