// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: James Liao <jamesjj.liao@mediatek.com>
 */

#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clkdev.h>
#include <linux/mfd/syscon.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

static ATOMIC_NOTIFIER_HEAD(mtk_clk_notifier_list);

int register_mtk_clk_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&mtk_clk_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(register_mtk_clk_notifier);

int unregister_mtk_clk_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&mtk_clk_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_mtk_clk_notifier);

int mtk_clk_notify(struct regmap *regmap, struct regmap *hwv_regmap,
		const char *name, u32 ofs, u32 id, u32 shift, int event_type)
{
	struct clk_event_data clke;

	clke.event_type = event_type;
	clke.regmap = regmap;
	clke.hwv_regmap = hwv_regmap;
	clke.name = name;
	clke.ofs = ofs;
	clke.id = id;
	clke.shift = shift;

	atomic_notifier_call_chain(&mtk_clk_notifier_list, 0, &clke);

	return 0;
}
EXPORT_SYMBOL_GPL(mtk_clk_notify);

struct clk_onecell_data *mtk_alloc_clk_data(unsigned int clk_num)
{
	int i;
	struct clk_onecell_data *clk_data;

	clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		return NULL;

	clk_data->clks = kcalloc(clk_num, sizeof(*clk_data->clks), GFP_KERNEL);
	if (!clk_data->clks)
		goto err_out;

	clk_data->clk_num = clk_num;

	for (i = 0; i < clk_num; i++)
		clk_data->clks[i] = ERR_PTR(-ENOENT);

	return clk_data;
err_out:
	kfree(clk_data);

	return NULL;
}
EXPORT_SYMBOL(mtk_alloc_clk_data);

void mtk_clk_register_fixed_clks(const struct mtk_fixed_clk *clks,
		int num, struct clk_onecell_data *clk_data)
{
	int i;
	struct clk *clk;

	for (i = 0; i < num; i++) {
		const struct mtk_fixed_clk *rc = &clks[i];

		if (clk_data && !IS_ERR_OR_NULL(clk_data->clks[rc->id]))
			continue;

		clk = clk_register_fixed_rate(NULL, rc->name, rc->parent, 0,
					      rc->rate);

		if (IS_ERR(clk)) {
			pr_err("Failed to register clk %s: %ld\n",
					rc->name, PTR_ERR(clk));
			continue;
		}

		if (clk_data)
			clk_data->clks[rc->id] = clk;
	}
}
EXPORT_SYMBOL(mtk_clk_register_fixed_clks);

void mtk_clk_register_factors(const struct mtk_fixed_factor *clks,
		int num, struct clk_onecell_data *clk_data)
{
	int i;
	struct clk *clk;

	for (i = 0; i < num; i++) {
		const struct mtk_fixed_factor *ff = &clks[i];

		if (clk_data && !IS_ERR_OR_NULL(clk_data->clks[ff->id]))
			continue;

		clk = clk_register_fixed_factor(NULL, ff->name, ff->parent_name,
				CLK_SET_RATE_PARENT, ff->mult, ff->div);

		if (IS_ERR(clk)) {
			pr_err("Failed to register clk %s: %ld\n",
					ff->name, PTR_ERR(clk));
			continue;
		}

		if (clk_data)
			clk_data->clks[ff->id] = clk;
	}
}
EXPORT_SYMBOL(mtk_clk_register_factors);

int mtk_clk_register_gates_with_dev(struct device_node *node,
		const struct mtk_gate *clks,
		int num, struct clk_onecell_data *clk_data,
		struct device *dev)
{
	int i;
	struct clk *clk;
	struct regmap *regmap, *hw_voter_regmap;

	if (!clk_data)
		return -ENOMEM;

	regmap = syscon_node_to_regmap(node);
	if (IS_ERR(regmap)) {
		pr_err("Cannot find regmap for %pOF: %ld\n", node,
				PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	hw_voter_regmap = syscon_regmap_lookup_by_phandle(node, "hw-voter-regmap");
	if (IS_ERR(hw_voter_regmap))
		hw_voter_regmap = NULL;

	for (i = 0; i < num; i++) {
		const struct mtk_gate *gate = &clks[i];

		if (!IS_ERR_OR_NULL(clk_data->clks[gate->id]))
			continue;

		if (hw_voter_regmap && gate->flags & CLK_USE_HW_VOTER)
			clk = mtk_clk_register_gate_hwv(gate->name, gate->parent_name,
					regmap,
					hw_voter_regmap,
					gate->regs->set_ofs,
					gate->regs->clr_ofs,
					gate->regs->sta_ofs,
					gate->hwv_regs->set_ofs,
					gate->hwv_regs->clr_ofs,
					gate->hwv_regs->sta_ofs,
					gate->shift, gate->ops, gate->flags, dev);
		else
			clk = mtk_clk_register_gate(gate->name, gate->parent_name,
					regmap,
					gate->regs->set_ofs,
					gate->regs->clr_ofs,
					gate->regs->sta_ofs,
					gate->shift, gate->ops, gate->flags, dev);

		if (IS_ERR(clk)) {
			pr_err("Failed to register clk %s: %ld\n",
					gate->name, PTR_ERR(clk));
			continue;
		}

		clk_data->clks[gate->id] = clk;
	}

	return 0;
}
EXPORT_SYMBOL(mtk_clk_register_gates_with_dev);

int mtk_clk_register_gates(struct device_node *node,
		const struct mtk_gate *clks,
		int num, struct clk_onecell_data *clk_data)
{
	return mtk_clk_register_gates_with_dev(node,
		clks, num, clk_data, NULL);
}
EXPORT_SYMBOL(mtk_clk_register_gates);

struct clk *mtk_clk_register_composite(const struct mtk_composite *mc,
		void __iomem *base, spinlock_t *lock)
{
	struct clk *clk;
	struct clk_mux *mux = NULL;
	struct clk_gate *gate = NULL;
	struct clk_divider *div = NULL;
	struct clk_hw *mux_hw = NULL, *gate_hw = NULL, *div_hw = NULL;
	const struct clk_ops *mux_ops = NULL, *gate_ops = NULL, *div_ops = NULL;
	const char * const *parent_names;
	const char *parent;
	int num_parents;
	int ret;

	if (mc->mux_shift >= 0) {
		mux = kzalloc(sizeof(*mux), GFP_KERNEL);
		if (!mux)
			return ERR_PTR(-ENOMEM);

		mux->reg = base + mc->mux_reg;
		mux->mask = BIT(mc->mux_width) - 1;
		mux->shift = mc->mux_shift;
		mux->lock = lock;
		mux->flags = mc->mux_flags;
		mux_hw = &mux->hw;
		mux_ops = &clk_mux_ops;

		parent_names = mc->parent_names;
		num_parents = mc->num_parents;
	} else {
		parent = mc->parent;
		parent_names = &parent;
		num_parents = 1;
	}

	if (mc->gate_shift >= 0) {
		gate = kzalloc(sizeof(*gate), GFP_KERNEL);
		if (!gate) {
			ret = -ENOMEM;
			goto err_out;
		}

		gate->reg = base + mc->gate_reg;
		gate->bit_idx = mc->gate_shift;
		gate->flags = CLK_GATE_SET_TO_DISABLE;
		gate->lock = lock;

		gate_hw = &gate->hw;
		gate_ops = &clk_gate_ops;
	}

	if (mc->divider_shift >= 0) {
		div = kzalloc(sizeof(*div), GFP_KERNEL);
		if (!div) {
			ret = -ENOMEM;
			goto err_out;
		}

		div->reg = base + mc->divider_reg;
		div->shift = mc->divider_shift;
		div->width = mc->divider_width;
		div->lock = lock;

		div_hw = &div->hw;
		div_ops = &clk_divider_ops;
	}

	clk = clk_register_composite(NULL, mc->name, parent_names, num_parents,
		mux_hw, mux_ops,
		div_hw, div_ops,
		gate_hw, gate_ops,
		mc->flags);

	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto err_out;
	}

	return clk;
err_out:
	kfree(div);
	kfree(gate);
	kfree(mux);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL(mtk_clk_register_composite);

void mtk_clk_register_composites(const struct mtk_composite *mcs,
		int num, void __iomem *base, spinlock_t *lock,
		struct clk_onecell_data *clk_data)
{
	struct clk *clk;
	int i;

	for (i = 0; i < num; i++) {
		const struct mtk_composite *mc = &mcs[i];

		if (clk_data && !IS_ERR_OR_NULL(clk_data->clks[mc->id]))
			continue;

		clk = mtk_clk_register_composite(mc, base, lock);

		if (IS_ERR(clk)) {
			pr_err("Failed to register clk %s: %ld\n",
					mc->name, PTR_ERR(clk));
			continue;
		}

		if (clk_data)
			clk_data->clks[mc->id] = clk;
	}
}
EXPORT_SYMBOL(mtk_clk_register_composites);

void mtk_clk_register_dividers(const struct mtk_clk_divider *mcds,
			int num, void __iomem *base, spinlock_t *lock,
				struct clk_onecell_data *clk_data)
{
	struct clk *clk;
	int i;

	for (i = 0; i <  num; i++) {
		const struct mtk_clk_divider *mcd = &mcds[i];

		if (clk_data && !IS_ERR_OR_NULL(clk_data->clks[mcd->id]))
			continue;

		clk = clk_register_divider(NULL, mcd->name, mcd->parent_name,
			mcd->flags, base +  mcd->div_reg, mcd->div_shift,
			mcd->div_width, mcd->clk_divider_flags, lock);

		if (IS_ERR(clk)) {
			pr_err("Failed to register clk %s: %ld\n",
				mcd->name, PTR_ERR(clk));
			continue;
		}

		if (clk_data)
			clk_data->clks[mcd->id] = clk;
	}
}
EXPORT_SYMBOL(mtk_clk_register_dividers);


int mtk_clk_simple_probe(struct platform_device *pdev)
{
	const struct mtk_clk_desc *mcd;
	struct clk_onecell_data *clk_data;
	struct device_node *node = pdev->dev.of_node;
	int r;

	mcd = of_device_get_match_data(&pdev->dev);
	if (!mcd)
		return -EINVAL;

	clk_data = mtk_alloc_clk_data(mcd->num_clks);
	if (!clk_data)
		return -ENOMEM;

	r = mtk_clk_register_gates(node, mcd->clks, mcd->num_clks, clk_data);
	if (r)
		return r;

	return of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);
}
EXPORT_SYMBOL(mtk_clk_simple_probe);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MTK");
MODULE_AUTHOR("MediaTek Inc.");
