// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/list.h>
#include <linux/slab.h>

#include <lpm_module.h>
#include <lpm_internal.h>
#include <lpm_trace.h>


#define LPM_TRACE_SYSRAM_MEMCPY_DEST(_offset, _buf, _sz) ({\
	memcpy(buf, (void *)(lpm_trace_ins.mmu + _offset),\
		_sz); })

#define LPM_TRACE_SYSRAM_MEMCPY_SRC(_offset, _buf, _sz) ({\
	memcpy((void *)(lpm_trace_ins.mmu + _offset), buf,\
		_sz); })


struct LPM_TRACE_INS {
	void __iomem *mmu;
	size_t size;
};

static struct LPM_TRACE_INS lpm_trace_ins;

int __init lpm_trace_parsing(struct device_node *parent)
{
	struct device_node *node;

	if (!parent)
		return -EINVAL;

	node = of_find_compatible_node(parent, NULL,
					"mediatek,lpm-sysram");
	if (node) {
		struct resource res;

		if (!of_address_to_resource(node, 0, &res)) {
			lpm_trace_ins.size = (size_t)resource_size(&res);
			lpm_trace_ins.mmu = ioremap(res.start,
							resource_size(&res));
		}
		of_node_put(node);
	}
	return 0;
}

size_t lpm_trace_sysram_read(unsigned long offset,
					 void *buf, size_t sz)
{
	size_t rSz = 0;

	if ((offset >= lpm_trace_ins.size)
	    || !lpm_trace_ins.mmu)
		return -EINVAL;

	rSz = ((offset + sz) < lpm_trace_ins.size) ?
		sz : (lpm_trace_ins.size - offset);

	LPM_TRACE_SYSRAM_MEMCPY_DEST(offset, buf, rSz);
	return rSz;
}

size_t lpm_trace_sysram_wrtie(unsigned long offset,
					 const void *buf, size_t sz)
{
	size_t rSz = 0;

	if ((offset >= lpm_trace_ins.size)
	    || !lpm_trace_ins.mmu)
		return -EINVAL;

	rSz = ((offset + sz) < lpm_trace_ins.size) ?
		sz : (lpm_trace_ins.size - offset);

	LPM_TRACE_SYSRAM_MEMCPY_SRC(offset, buf, rSz);
	return 0;
}

int lpm_trace_instance_get(int type, struct LPM_PLAT_TRACE *ins)
{
	int ret = 0;

	if (!ins)
		return -EINVAL;

	if (type == LPM_PLAT_TRACE_SYSRAM) {
		ins->read = lpm_trace_sysram_read;
		ins->write = lpm_trace_sysram_wrtie;
	} else
		ret = -EINVAL;

	return ret;
}
