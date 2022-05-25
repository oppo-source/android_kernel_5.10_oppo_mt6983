/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MT_PWM_HAL_H__
#define __MT_PWM_HAL_H__
#include <linux/types.h>
#include <linux/io.h>
#include <asm/cacheflush.h>

/*
 * Define macros.
 */
#define mt_reg_sync_writel(v, a) \
	do {    \
		__raw_writel((v), (void __force __iomem *)((a)));   \
		mb(); /*make sure register access in order */ \
	} while (0)

/**********************************
 * Global enum data
 */
/******************* Register Manipulations*****************/
#define INREG32(reg)          __raw_readl((void *)reg)
#define OUTREG32(reg, val)      mt_reg_sync_writel(val, (void *)reg)
#define OUTREG32_DMA(reg, val)	((*(long *)(reg)) = (long)(val))
#define SETREG32(reg, val)      OUTREG32(reg, INREG32(reg)|(val))
#define CLRREG32(reg, val)      OUTREG32(reg, INREG32(reg)&~(val))
#define MASKREG32(x, y, z)  OUTREG32(x, (INREG32(x)&~(y))|(z))

enum PWN_NO {
	PWM_MIN,
	PWM1 = PWM_MIN,
	PWM2,
	PWM3,
	PWM4,
	PWM5,
	PWM6,
	PWM_NUM,
	PWM_MAX = PWM_NUM
};

enum TEST_SEL_BIT {
	TEST_SEL_FALSE,
	TEST_SEL_TRUE
};

enum PWM_CON_MODE_BIT {
	PERIOD,
	RAND
};

enum PWM_CON_SRCSEL_BIT {
	PWM_FIFO,
	MEMORY
};

enum PWM_CON_IDLE_BIT {
	IDLE_FALSE,
	IDLE_TRUE,
	IDLE_MAX
};

enum PWM_CON_GUARD_BIT {
	GUARD_FALSE,
	GUARD_TRUE,
	GUARD_MAX
};

enum OLD_MODE_BIT {
	OLDMODE_DISABLE,
	OLDMODE_ENABLE
};

enum PWM_BUF_VALID_BIT {
	BUF0_VALID,
	BUF0_EN_VALID,
	BUF1_VALID,
	BUF1_EN_VALID,
	BUF_EN_MAX
};

enum CLOCK_SRC {
	CLK_BLOCK,
	CLK_BLOCK_BY_1625_OR_32K
};

enum PWM_CLK_DIV {
	CLK_DIV_MIN,
	CLK_DIV1 = CLK_DIV_MIN,
	CLK_DIV2,
	CLK_DIV4,
	CLK_DIV8,
	CLK_DIV16,
	CLK_DIV32,
	CLK_DIV64,
	CLK_DIV128,
	CLK_DIV_MAX
};

enum PWM_INT_ENABLE_BITS {
	PWM1_INT_FINISH_EN,
	PWM1_INT_UNDERFLOW_EN,
	PWM2_INT_FINISH_EN,
	PWM2_INT_UNDERFLOW_EN,
	PWM3_INT_FINISH_EN,
	PWM3_INT_UNDERFLOW_EN,
	PWM4_INT_FINISH_EN,
	PWM4_INT_UNDERFLOW_EN,
	PWM_INT_ENABLE_BITS_MAX,
};

enum PWM_INT_STATUS_BITS {
	PWM1_INT_FINISH_ST,
	PWM1_INT_UNDERFLOW_ST,
	PWM2_INT_FINISH_ST,
	PWM2_INT_UNDERFLOW_ST,
	PWM3_INT_FINISH_ST,
	PWM3_INT_UNDERFLOW_ST,
	PWM4_INT_FINISH_ST,
	PWM4_INT_UNDERFLOW_ST,
	PWM_INT_STATUS_BITS_MAX,
};

enum PWM_INT_ACK_BITS {
	PWM1_INT_FINISH_ACK,
	PWM1_INT_UNDERFLOW_ACK,
	PWM2_INT_FINISH_ACK,
	PWM2_INT_UNDERFLOW_ACK,
	PWM3_INT_FINISH_ACK,
	PWM3_INT_UNDERFLOW_ACK,
	PWM4_INT_FINISH_ACK,
	PWM4_INT_UNDERFLOW_ACK,
	PWM_INT_ACK_BITS_MAX,
};

enum PWM_CLOCK_SRC_ENUM {
	PWM_CLK_SRC_MIN,
	PWM_CLK_OLD_MODE_BLOCK = PWM_CLK_SRC_MIN,
	PWM_CLK_OLD_MODE_32K,
	PWM_CLK_NEW_MODE_BLOCK,
	PWM_CLK_NEW_MODE_BLOCK_DIV_BY_1625,
	PWM_CLK_SRC_NUM,
	PWM_CLK_SRC_INVALID,
};

enum PWM_MODE_ENUM {
	PWM_MODE_MIN,
	PWM_MODE_OLD = PWM_MODE_MIN,
	PWM_MODE_FIFO,
	PWM_MODE_MEMORY,
	PWM_MODE_RANDOM,
	PWM_MODE_DELAY,
	PWM_MODE_INVALID,
};

enum INFRA_CLK_SRC_CTRL {
	CLK_32K = 0x00,
	CLK_26M = 0x01,
	CLK_78M = 0x2,
	CLK_SEL_TOPCKGEN = 0x3,
};

#define PWM_NEW_MODE_DUTY_TOTAL_BITS 64

void mt_set_pwm_3dlcm_enable_hal(u8 enable);
void mt_set_pwm_3dlcm_inv_hal(u32 pwm_no, u8 inv);
void mt_set_pwm_3dlcm_base_hal(u32 pwm_no);

void mt_pwm_26M_clk_enable_hal(u32 enable);

#endif
