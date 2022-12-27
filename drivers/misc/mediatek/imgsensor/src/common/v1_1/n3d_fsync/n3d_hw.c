// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/spinlock.h>
#include <linux/spinlock_types.h>

#include "n3d_hw.h"
#include "n3d_reg.h"
#include "n3d_util.h"
#include "vsync_recorder.h"

#undef IRQ_PRINT

#ifdef IRQ_PRINT
#define REC_LOG(format, args...) LOG_D(format, ##args)
#else
#define REC_LOG(format, args...)
#endif

static DEFINE_SPINLOCK(read_reg_lock);

static int set_n3d_enable(struct base_reg *regs, int enable)
{
	int en = enable ? 1 : 0;

	/* enable/disable n3d A */
	SENINF_BITS(regs->pseninf_n3d_base[SENINF_N3D_A],
		    SENINF_N3D_A_CTL, DIFF_EN, en);
	SENINF_BITS(regs->pseninf_n3d_base[SENINF_N3D_A],
		    SENINF_N3D_A_CTL, SEN1_OV_VS_INT_EN, en);
	SENINF_BITS(regs->pseninf_n3d_base[SENINF_N3D_A],
		    SENINF_N3D_A_CTL, SEN2_OV_VS_INT_EN, en);
	SENINF_BITS(regs->pseninf_n3d_base[SENINF_N3D_A],
		    SENINF_N3D_A_CTL, N3D_EN, en);

	return 0;
}

int set_n3d_source(struct base_reg *regs,
		   struct sensor_info *sen1,
		   struct sensor_info *sen2)
{
	LOG_D("start sync sensor_idx1(%u), cammux_id1(%u) sensor_idx2(%u), cammux_id2(%u)\n",
	       sen1->sensor_idx, sen1->cammux_id,
	       sen2->sensor_idx, sen2->cammux_id);

	mutex_lock(&regs->reg_mutex);

	/* set source */
	SENINF_BITS(regs->pseninf_top_base,
		    SENINF_TOP_N3D_A_CTRL,
		    RG_N3D_SENINF1_VSYNC_SRC_SEL_A,
		    sen1->cammux_id);
	SENINF_BITS(regs->pseninf_top_base,
		    SENINF_TOP_N3D_A_CTRL,
		    RG_N3D_SENINF2_VSYNC_SRC_SEL_A,
		    sen2->cammux_id);

	/* enable n3d A */
	set_n3d_enable(regs, 1);

	mutex_unlock(&regs->reg_mutex);

	return 0;
}

int disable_n3d(struct base_reg *regs)
{
	LOG_D("E\n");

	mutex_lock(&regs->reg_mutex);

	/* disable n3d A */
	set_n3d_enable(regs, 0);

	mutex_unlock(&regs->reg_mutex);

	return 0;
}

int read_status(struct base_reg *regs)
{
	int result = 0;
	int period = 0;
	int ck_cnt = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&read_reg_lock, flags);
	result = SENINF_READ_REG(regs->pseninf_n3d_base[SENINF_N3D_A],
				 SENINF_N3D_A_INT);
	spin_unlock_irqrestore(&read_reg_lock, flags);

	if (result & 0x10) {
		spin_lock_irqsave(&read_reg_lock, flags);
		period = SENINF_READ_REG(regs->pseninf_n3d_base[SENINF_N3D_A],
					 SENINF_N3D_A_CNT0);
		spin_unlock_irqrestore(&read_reg_lock, flags);
		record_vs1(period);
		REC_LOG("status(0x%x) sen1 period = %u\n", result, period);
	}
	if (result & 0x20) {
		spin_lock_irqsave(&read_reg_lock, flags);
		period = SENINF_READ_REG(regs->pseninf_n3d_base[SENINF_N3D_A],
					 SENINF_N3D_A_CNT1);
		spin_unlock_irqrestore(&read_reg_lock, flags);
		record_vs2(period);
		REC_LOG("status(0x%x) sen2 period = %u\n", result, period);
	}

	if (result & 0x4) {
		spin_lock_irqsave(&read_reg_lock, flags);
		ck_cnt = SENINF_READ_REG(regs->pseninf_n3d_base[SENINF_N3D_A],
					 SENINF_N3D_A_DIFF_CNT);
		spin_unlock_irqrestore(&read_reg_lock, flags);
		record_vs_diff((ck_cnt >> 31) & 0x1, ck_cnt & 0x7FFFFFFF);
		if (ck_cnt & 0x80000000)
			REC_LOG("status(0x%x) vs2 after vs1, diff = %u\n",
				 result, (ck_cnt & 0x7FFFFFFF));
		else
			REC_LOG("status(0x%x) vs1 after vs2, diff = %u\n",
				 result, (ck_cnt & 0x7FFFFFFF));
	}

	/* for irq mis trigger debug */
	if (!result)
		record_mis_trigger_cnt();

#ifdef IRQ_PRINT
	if (result) {
		show_records(0);
		show_records(1);
	}
#endif

	return result;
}

