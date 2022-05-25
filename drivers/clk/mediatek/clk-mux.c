// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Owen Chen <owen.chen@mediatek.com>
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/time64.h>
#include <linux/timekeeping.h>
#include <linux/sched/clock.h>

#include "clk-mtk.h"
#include "clk-mux.h"

static unsigned long long profile_time[4];
static bool is_registered;

static inline struct mtk_clk_mux *to_mtk_clk_mux(struct clk_hw *hw)
{
	return container_of(hw, struct mtk_clk_mux, hw);
}

static int mtk_clk_mux_enable(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 mask = BIT(mux->data->gate_shift);

	return regmap_update_bits(mux->regmap, mux->data->mux_ofs,
			mask, ~mask);
}

static void mtk_clk_mux_disable(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 mask = BIT(mux->data->gate_shift);

	regmap_update_bits(mux->regmap, mux->data->mux_ofs, mask, mask);
}

static int mtk_clk_mux_enable_setclr(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	unsigned long flags = 0;

	if (mux->lock)
		spin_lock_irqsave(mux->lock, flags);
	else
		__acquire(mux->lock);

	regmap_write(mux->regmap, mux->data->clr_ofs,
		BIT(mux->data->gate_shift));

	/*
	 * If mux setting restore after vcore resume, it will
	 * not be effective yet. Set the update bit to ensure the mux gets
	 * updated.
	 */
	regmap_write(mux->regmap, mux->data->upd_ofs,
		BIT(mux->data->upd_shift));

	if (mux->lock)
		spin_unlock_irqrestore(mux->lock, flags);
	else
		__release(mux->lock);

	return 0;
}

static void mtk_clk_mux_disable_setclr(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);

	regmap_write(mux->regmap, mux->data->set_ofs,
			BIT(mux->data->gate_shift));
}

static int mtk_clk_mux_is_enabled(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 val = 0;

	if (!is_registered)
		return 0;

	regmap_read(mux->regmap, mux->data->mux_ofs, &val);

	return (val & BIT(mux->data->gate_shift)) == 0;
}

static int mtk_clk_hwv_mux_is_enabled(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 val = 0;

	if (!is_registered)
		return 0;

	regmap_read(mux->hwv_regmap, mux->data->hwv_set_ofs, &val);

	return (val & BIT(mux->data->gate_shift)) != 0;
}

static int mtk_clk_hwv_mux_is_done(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 val = 0;

	regmap_read(mux->hwv_regmap, mux->data->hwv_sta_ofs, &val);

	return (val & BIT(mux->data->gate_shift)) != 0;
}

static int mtk_clk_hwv_mux_enable(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 val, val2;
	int i = 0, j = 0;

	profile_time[2] = 0;
	profile_time[3] = 0;
	regmap_write(mux->hwv_regmap, mux->data->hwv_set_ofs,
			BIT(mux->data->gate_shift));
	profile_time[0] = sched_clock();

	while (!mtk_clk_hwv_mux_is_enabled(hw)) {
		if (i < MTK_WAIT_HWV_PREPARE_CNT)
			udelay(MTK_WAIT_HWV_PREPARE_US);
		else
			goto hwv_prepare_fail;
		i++;
	}

	profile_time[1] = sched_clock();
	i = 0;

	while (1) {
		regmap_read(mux->hwv_regmap, mux->data->hwv_sta_ofs, &val);

		if ((profile_time[2] == 0) && (val & BIT(mux->data->gate_shift)) != 0)
			profile_time[2] = sched_clock();
		else {
			regmap_read(mux->regmap, mux->data->mux_ofs, &val2);
			if ((val2 & BIT(mux->data->gate_shift)) == 0) {
				profile_time[3] = sched_clock();
				break;
			} else if (j  > MTK_WAIT_HWV_STA_CNT)
				goto hwv_sta_fail;
			else
				j++;
		}

		if (i < MTK_WAIT_HWV_DONE_CNT && j < MTK_WAIT_HWV_STA_CNT)
			udelay(MTK_WAIT_HWV_DONE_US);
		else
			goto hwv_done_fail;

		i++;
	}

	return 0;

hwv_sta_fail:
	mtk_clk_notify(mux->regmap, mux->hwv_regmap, NULL,
			mux->data->mux_ofs, (mux->data->hwv_set_ofs / MTK_HWV_ID_OFS),
			mux->data->gate_shift, CLK_EVT_LONG_BUS_LATENCY);
hwv_done_fail:
	regmap_read(mux->regmap, mux->data->mux_ofs, &val);
	regmap_read(mux->hwv_regmap, mux->data->hwv_sta_ofs, &val2);
	pr_err("%s mux enable timeout(%x %x)\n", clk_hw_get_name(hw), val, val2);
hwv_prepare_fail:
	regmap_read(mux->regmap, mux->data->hwv_sta_ofs, &val);
	pr_err("%s mux prepare timeout(%x)\n", clk_hw_get_name(hw), val);

	for (i = 0; i < 4; i++)
		pr_err("[%d]%lld us", i, profile_time[i]);

	mtk_clk_notify(mux->regmap, mux->hwv_regmap, NULL,
			mux->data->mux_ofs, (mux->data->hwv_set_ofs / MTK_HWV_ID_OFS),
			mux->data->gate_shift, CLK_EVT_HWV_CG_TIMEOUT);

	return -EBUSY;
}

static void mtk_clk_hwv_mux_disable(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 val;
	int i = 0;

	regmap_write(mux->hwv_regmap, mux->data->hwv_clr_ofs,
			BIT(mux->data->gate_shift));

	while (mtk_clk_hwv_mux_is_enabled(hw)) {
		if (i < MTK_WAIT_HWV_PREPARE_CNT)
			udelay(MTK_WAIT_HWV_PREPARE_US);
		else
			goto hwv_prepare_fail;
		i++;
	}

	i = 0;

	while (!mtk_clk_hwv_mux_is_done(hw)) {
		if (i < MTK_WAIT_HWV_DONE_CNT)
			udelay(MTK_WAIT_HWV_DONE_US);
		else
			goto hwv_done_fail;
		i++;
	}

	return;

hwv_done_fail:
	regmap_read(mux->regmap, mux->data->mux_ofs, &val);
	pr_err("%s mux disable timeout(%dus)(%x)\n", clk_hw_get_name(hw),
			i * MTK_WAIT_HWV_DONE_US, val);
hwv_prepare_fail:
	pr_err("%s mux unprepare timeout(%dus)\n", clk_hw_get_name(hw),
			i * MTK_WAIT_HWV_PREPARE_US);

	mtk_clk_notify(mux->regmap, mux->hwv_regmap, clk_hw_get_name(hw),
			mux->data->mux_ofs, (mux->data->hwv_set_ofs / MTK_HWV_ID_OFS),
			mux->data->gate_shift, CLK_EVT_HWV_CG_TIMEOUT);
	return;
}

static u8 mtk_clk_mux_get_parent(struct clk_hw *hw)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 mask = GENMASK(mux->data->mux_width - 1, 0);
	u32 val = 0;

	regmap_read(mux->regmap, mux->data->mux_ofs, &val);
	val = (val >> mux->data->mux_shift) & mask;

	return val;
}

static int mtk_clk_mux_set_parent_lock(struct clk_hw *hw, u8 index)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 mask = GENMASK(mux->data->mux_width - 1, 0);
	unsigned long flags = 0;

	if (mux->lock)
		spin_lock_irqsave(mux->lock, flags);
	else
		__acquire(mux->lock);

	regmap_update_bits(mux->regmap, mux->data->mux_ofs, mask,
		index << mux->data->mux_shift);

	if (mux->lock)
		spin_unlock_irqrestore(mux->lock, flags);
	else
		__release(mux->lock);

	return 0;
}

static int mtk_clk_mux_set_parent_setclr_lock(struct clk_hw *hw, u8 index)
{
	struct mtk_clk_mux *mux = to_mtk_clk_mux(hw);
	u32 mask = GENMASK(mux->data->mux_width - 1, 0);
	u32 val = 0, orig = 0;
	unsigned long flags = 0;

	if (mux->lock)
		spin_lock_irqsave(mux->lock, flags);
	else
		__acquire(mux->lock);

	regmap_read(mux->regmap, mux->data->mux_ofs, &orig);
	val = (orig & ~(mask << mux->data->mux_shift))
			| (index << mux->data->mux_shift);

	if (val != orig) {
		regmap_write(mux->regmap, mux->data->clr_ofs,
				mask << mux->data->mux_shift);
		regmap_write(mux->regmap, mux->data->set_ofs,
				index << mux->data->mux_shift);

		if (mux->data->upd_shift >= 0)
			regmap_write(mux->regmap, mux->data->upd_ofs,
					BIT(mux->data->upd_shift));
	}

	if (mux->lock)
		spin_unlock_irqrestore(mux->lock, flags);
	else
		__release(mux->lock);

	return 0;
}

const struct clk_ops mtk_mux_ops = {
	.get_parent = mtk_clk_mux_get_parent,
	.set_parent = mtk_clk_mux_set_parent_lock,
};
EXPORT_SYMBOL(mtk_mux_ops);

const struct clk_ops mtk_mux_clr_set_upd_ops = {
	.get_parent = mtk_clk_mux_get_parent,
	.set_parent = mtk_clk_mux_set_parent_setclr_lock,
};
EXPORT_SYMBOL(mtk_mux_clr_set_upd_ops);

const struct clk_ops mtk_mux_gate_ops = {
	.enable = mtk_clk_mux_enable,
	.disable = mtk_clk_mux_disable,
	.is_enabled = mtk_clk_mux_is_enabled,
	.get_parent = mtk_clk_mux_get_parent,
	.set_parent = mtk_clk_mux_set_parent_lock,
};
EXPORT_SYMBOL(mtk_mux_gate_ops);

const struct clk_ops mtk_mux_gate_clr_set_upd_ops = {
	.enable = mtk_clk_mux_enable_setclr,
	.disable = mtk_clk_mux_disable_setclr,
	.is_enabled = mtk_clk_mux_is_enabled,
	.get_parent = mtk_clk_mux_get_parent,
	.set_parent = mtk_clk_mux_set_parent_setclr_lock,
};
EXPORT_SYMBOL(mtk_mux_gate_clr_set_upd_ops);

const struct clk_ops mtk_hwv_mux_ops = {
	.enable = mtk_clk_hwv_mux_enable,
	.disable = mtk_clk_hwv_mux_disable,
	.is_enabled = mtk_clk_hwv_mux_is_enabled,
	.get_parent = mtk_clk_mux_get_parent,
	.set_parent = mtk_clk_mux_set_parent_setclr_lock,
};
EXPORT_SYMBOL(mtk_hwv_mux_ops);

static struct clk *mtk_clk_register_mux(const struct mtk_mux *mux,
				 struct regmap *regmap,
				 struct regmap *hw_voter_regmap,
				 spinlock_t *lock)
{
	struct mtk_clk_mux *clk_mux;
	struct clk_init_data init = {};
	struct clk *clk;

	clk_mux = kzalloc(sizeof(*clk_mux), GFP_KERNEL);
	if (!clk_mux)
		return ERR_PTR(-ENOMEM);

	init.name = mux->name;
	init.flags = mux->flags | CLK_SET_RATE_PARENT;
	init.parent_names = mux->parent_names;
	init.num_parents = mux->num_parents;
	init.ops = mux->ops;

	clk_mux->regmap = regmap;
	clk_mux->hwv_regmap = hw_voter_regmap;
	clk_mux->data = mux;
	clk_mux->lock = lock;
	clk_mux->hw.init = &init;

	clk = clk_register(NULL, &clk_mux->hw);
	if (IS_ERR(clk)) {
		kfree(clk_mux);
		return clk;
	}

	return clk;
}

int mtk_clk_register_muxes(const struct mtk_mux *muxes,
			   int num, struct device_node *node,
			   spinlock_t *lock,
			   struct clk_onecell_data *clk_data)
{
	struct regmap *regmap, *hw_voter_regmap;
	struct clk *clk;
	int i;

	is_registered = false;

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
		const struct mtk_mux *mux = &muxes[i];

		if (IS_ERR_OR_NULL(clk_data->clks[mux->id])) {
			clk = mtk_clk_register_mux(mux, regmap, hw_voter_regmap, lock);

			if (IS_ERR(clk)) {
				pr_err("Failed to register clk %s: %ld\n",
				       mux->name, PTR_ERR(clk));
				continue;
			}

			clk_data->clks[mux->id] = clk;
		}
	}

	is_registered = true;

	return 0;
}
EXPORT_SYMBOL(mtk_clk_register_muxes);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MUX");
MODULE_AUTHOR("MediaTek Inc.");
