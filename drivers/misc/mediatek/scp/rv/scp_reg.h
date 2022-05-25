/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __SCP_REG_H
#define __SCP_REG_H

#define SCP_SEMAPHORE			(scpreg.cfg + 0x0018)
#define SCP_3WAY_SEMAPHORE		(scpreg.cfg + 0x001c)
#define SCP_SCP2SPM_VOL_LV		(scpreg.cfg + 0x0020)

/* SCP to SPM IPC clear */
#define SCP_TO_SPM_REG			(scpreg.cfg + 0x0094)

#define R_GIPC_IN_SET			(scpreg.cfg + 0x0098)
#define R_GIPC_IN_CLR			(scpreg.cfg + 0x009c)
	#define B_GIPC0_SETCLR_0	(1 << 0)
	#define B_GIPC0_SETCLR_1	(1 << 1)
	#define B_GIPC0_SETCLR_2	(1 << 2)
	#define B_GIPC0_SETCLR_3	(1 << 3)
	#define B_GIPC1_SETCLR_0	(1 << 4)
	#define B_GIPC1_SETCLR_1	(1 << 5)
	#define B_GIPC1_SETCLR_2	(1 << 6)
	#define B_GIPC1_SETCLR_3	(1 << 7)
	#define B_GIPC2_SETCLR_0	(1 << 8)
	#define B_GIPC2_SETCLR_1	(1 << 9)
	#define B_GIPC2_SETCLR_2	(1 << 10)
	#define B_GIPC2_SETCLR_3	(1 << 11)
	#define B_GIPC3_SETCLR_0	(1 << 12)
	#define B_GIPC3_SETCLR_1	(1 << 13)
	#define B_GIPC3_SETCLR_2	(1 << 14)
	#define B_GIPC3_SETCLR_3	(1 << 15)
	#define B_GIPC4_SETCLR_0	(1 << 16)
	#define B_GIPC4_SETCLR_1	(1 << 17)
	#define B_GIPC4_SETCLR_2	(1 << 18)
	#define B_GIPC4_SETCLR_3	(1 << 19)

#define R_CORE0_SW_RSTN_CLR	(scpreg.cfg_core0 + 0x0000)
#define R_CORE1_SW_RSTN_CLR	(scpreg.cfg_core1 + 0x0000)

#define R_CORE0_SW_RSTN_SET	(scpreg.cfg_core0 + 0x0004)
#define R_CORE1_SW_RSTN_SET	(scpreg.cfg_core1 + 0x0004)


#define R_CORE0_DBG_CTRL	(scpreg.cfg_core0 + 0x0010)
#define R_CORE1_DBG_CTRL	(scpreg.cfg_core1 + 0x0010)
	#define M_CORE_TBUF_DBG_SEL	(7 << 4)
	#define S_CORE_TBUF_DBG_SEL	(4)
	#define M_CORE_TBUF_DBG_SEL_RV55	(0xfff0ff0f)
	#define S_CORE_TBUF_S		(4)
	#define S_CORE_TBUF1_S		(16)
#define R_CORE0_WDT_IRQ		(scpreg.cfg_core0 + 0x0030)
#define R_CORE1_WDT_IRQ		(scpreg.cfg_core1 + 0x0030)
	#define B_WDT_IRQ	(1 << 0)

#define R_CORE0_WDT_CFG		(scpreg.cfg_core0 + 0x0034)
#define R_CORE1_WDT_CFG		(scpreg.cfg_core1 + 0x0034)
	#define V_INSTANT_WDT	0x80000000

#define R_CORE0_STATUS			(scpreg.cfg_core0 + 0x0070)
	#define B_CORE_GATED		(1 << 0)
	#define B_CORE_HALT		(1 << 1)
#define R_CORE0_MON_PC			(scpreg.cfg_core0 + 0x0080)
#define R_CORE0_MON_LR			(scpreg.cfg_core0 + 0x0084)
#define R_CORE0_MON_SP			(scpreg.cfg_core0 + 0x0088)
#define R_CORE0_TBUF_WPTR		(scpreg.cfg_core0 + 0x008c)

#define R_CORE0_MON_PC_LATCH		(scpreg.cfg_core0 + 0x00d0)
#define R_CORE0_MON_LR_LATCH		(scpreg.cfg_core0 + 0x00d4)
#define R_CORE0_MON_SP_LATCH		(scpreg.cfg_core0 + 0x00d8)

#define R_CORE0_T1_MON_PC		(scpreg.cfg_core0 + 0x0160)
#define R_CORE0_T1_MON_LR		(scpreg.cfg_core0 + 0x0164)
#define R_CORE0_T1_MON_SP		(scpreg.cfg_core0 + 0x0168)

#define R_CORE0_T1_MON_PC_LATCH		(scpreg.cfg_core0 + 0x0170)
#define R_CORE0_T1_MON_LR_LATCH		(scpreg.cfg_core0 + 0x0174)
#define R_CORE0_T1_MON_SP_LATCH		(scpreg.cfg_core0 + 0x0178)

#define R_CORE1_STATUS			(scpreg.cfg_core1 + 0x0070)
#define R_CORE1_MON_PC			(scpreg.cfg_core1 + 0x0080)
#define R_CORE1_MON_LR			(scpreg.cfg_core1 + 0x0084)
#define R_CORE1_MON_SP			(scpreg.cfg_core1 + 0x0088)
#define R_CORE1_TBUF_WPTR		(scpreg.cfg_core1 + 0x008c)

#define R_CORE1_MON_PC_LATCH		(scpreg.cfg_core1 + 0x00d0)
#define R_CORE1_MON_LR_LATCH		(scpreg.cfg_core1 + 0x00d4)
#define R_CORE1_MON_SP_LATCH		(scpreg.cfg_core1 + 0x00d8)

#define R_CORE1_T1_MON_PC		(scpreg.cfg_core1 + 0x0160)
#define R_CORE1_T1_MON_LR		(scpreg.cfg_core1 + 0x0164)
#define R_CORE1_T1_MON_SP		(scpreg.cfg_core1 + 0x0168)

#define R_CORE1_T1_MON_PC_LATCH		(scpreg.cfg_core1 + 0x0170)
#define R_CORE1_T1_MON_LR_LATCH		(scpreg.cfg_core1 + 0x0174)
#define R_CORE1_T1_MON_SP_LATCH		(scpreg.cfg_core1 + 0x0178)

#define R_CORE0_TBUF_DATA31_0		(scpreg.cfg_core0 + 0x00e0)
#define R_CORE0_TBUF_DATA63_32		(scpreg.cfg_core0 + 0x00e4)
#define R_CORE0_TBUF_DATA95_64		(scpreg.cfg_core0 + 0x00e8)
#define R_CORE0_TBUF_DATA127_96		(scpreg.cfg_core0 + 0x00ec)

#define R_CORE0_TBUF1_DATA31_0		(scpreg.cfg_core0 + 0x00f0)
#define R_CORE0_TBUF1_DATA63_32		(scpreg.cfg_core0 + 0x00f4)
#define R_CORE0_TBUF1_DATA95_64		(scpreg.cfg_core0 + 0x00f8)
#define R_CORE0_TBUF1_DATA127_96	(scpreg.cfg_core0 + 0x00fc)

#define R_CORE1_TBUF_DATA31_0		(scpreg.cfg_core1 + 0x00e0)
#define R_CORE1_TBUF_DATA63_32		(scpreg.cfg_core1 + 0x00e4)
#define R_CORE1_TBUF_DATA95_64		(scpreg.cfg_core1 + 0x00e8)
#define R_CORE1_TBUF_DATA127_96		(scpreg.cfg_core1 + 0x00ec)

#define R_CORE1_TBUF1_DATA31_0		(scpreg.cfg_core1 + 0x00f0)
#define R_CORE1_TBUF1_DATA63_32		(scpreg.cfg_core1 + 0x00f4)
#define R_CORE1_TBUF1_DATA95_64		(scpreg.cfg_core1 + 0x00f8)
#define R_CORE1_TBUF1_DATA127_96	(scpreg.cfg_core1 + 0x00fc)

#define SCP_A_GENERAL_REG0       (scpreg.cfg_core0 + 0x0040)
/* DRAM reserved address and size */
#define SCP_A_GENERAL_REG1       (scpreg.cfg_core0 + 0x0044)
#define DRAM_RESV_ADDR_REG	 SCP_A_GENERAL_REG1
#define SCP_A_GENERAL_REG2       (scpreg.cfg_core0 + 0x0048)
#define DRAM_RESV_SIZE_REG	 SCP_A_GENERAL_REG2
/*EXPECTED_FREQ_REG*/
#define SCP_A_GENERAL_REG3       (scpreg.cfg_core0 + 0x004C)
#define EXPECTED_FREQ_REG        SCP_A_GENERAL_REG3
/*CURRENT_FREQ_REG*/
#define SCP_A_GENERAL_REG4       (scpreg.cfg_core0 + 0x0050)
#define CURRENT_FREQ_REG         SCP_A_GENERAL_REG4
/*SCP_GPR_CM4_A_REBOOT*/
#define SCP_A_GENERAL_REG5		(scpreg.cfg_core0 + 0x0054)
#define SCP_GPR_CORE0_REBOOT		SCP_A_GENERAL_REG5
	#define CORE_RDY_TO_REBOOT	0x34
	#define CORE_REBOOT_OK		0x1
#define SCP_GPR_CORE1_REBOOT		(scpreg.cfg_core1 + 0x0054)

#define SCP_A_GENERAL_REG6		(scpreg.cfg_core0 + 0x0058)
#define SCP_A_GENERAL_REG7		(scpreg.cfg_core0 + 0x005C)
/* bus tracker reg */
#define SCP_BUS_DBG_CON			(scpreg.bus_tracker)
#define SCP_BUS_DBG_AR_TRACK0_L		(scpreg.bus_tracker + 0x100)
#define SCP_BUS_DBG_AR_TRACK1_L		(scpreg.bus_tracker + 0x108)
#define SCP_BUS_DBG_AR_TRACK2_L		(scpreg.bus_tracker + 0x110)
#define SCP_BUS_DBG_AR_TRACK3_L		(scpreg.bus_tracker + 0x118)
#define SCP_BUS_DBG_AR_TRACK4_L		(scpreg.bus_tracker + 0x120)
#define SCP_BUS_DBG_AR_TRACK5_L		(scpreg.bus_tracker + 0x128)
#define SCP_BUS_DBG_AR_TRACK6_L		(scpreg.bus_tracker + 0x130)
#define SCP_BUS_DBG_AR_TRACK7_L		(scpreg.bus_tracker + 0x138)
#define SCP_BUS_DBG_AR_TRACK8_L		(scpreg.bus_tracker + 0x140)
#define SCP_BUS_DBG_AR_TRACK9_L		(scpreg.bus_tracker + 0x148)
#define SCP_BUS_DBG_AR_TRACK10_L	(scpreg.bus_tracker + 0x150)
#define SCP_BUS_DBG_AR_TRACK11_L	(scpreg.bus_tracker + 0x158)
#define SCP_BUS_DBG_AR_TRACK12_L	(scpreg.bus_tracker + 0x160)
#define SCP_BUS_DBG_AR_TRACK13_L	(scpreg.bus_tracker + 0x168)
#define SCP_BUS_DBG_AR_TRACK14_L	(scpreg.bus_tracker + 0x170)
#define SCP_BUS_DBG_AR_TRACK15_L	(scpreg.bus_tracker + 0x178)
#define SCP_BUS_DBG_AR_TRACK16_L	(scpreg.bus_tracker + 0x180)
#define SCP_BUS_DBG_AR_TRACK17_L	(scpreg.bus_tracker + 0x188)
#define SCP_BUS_DBG_AR_TRACK18_L	(scpreg.bus_tracker + 0x190)
#define SCP_BUS_DBG_AR_TRACK19_L	(scpreg.bus_tracker + 0x198)
#define SCP_BUS_DBG_AR_TRACK20_L	(scpreg.bus_tracker + 0x1a0)
#define SCP_BUS_DBG_AR_TRACK21_L	(scpreg.bus_tracker + 0x1a8)
#define SCP_BUS_DBG_AR_TRACK22_L	(scpreg.bus_tracker + 0x1b0)
#define SCP_BUS_DBG_AR_TRACK23_L	(scpreg.bus_tracker + 0x1b8)
#define SCP_BUS_DBG_AR_TRACK24_L	(scpreg.bus_tracker + 0x1c0)
#define SCP_BUS_DBG_AR_TRACK25_L	(scpreg.bus_tracker + 0x1c8)
#define SCP_BUS_DBG_AR_TRACK26_L	(scpreg.bus_tracker + 0x1d0)
#define SCP_BUS_DBG_AR_TRACK27_L	(scpreg.bus_tracker + 0x1d8)
#define SCP_BUS_DBG_AR_TRACK28_L	(scpreg.bus_tracker + 0x1e0)
#define SCP_BUS_DBG_AR_TRACK29_L	(scpreg.bus_tracker + 0x1e8)
#define SCP_BUS_DBG_AR_TRACK30_L	(scpreg.bus_tracker + 0x1f0)
#define SCP_BUS_DBG_AR_TRACK31_L	(scpreg.bus_tracker + 0x1f8)

#define SCP_BUS_DBG_AW_TRACK0_L		(scpreg.bus_tracker + 0x300)
#define SCP_BUS_DBG_AW_TRACK1_L		(scpreg.bus_tracker + 0x308)
#define SCP_BUS_DBG_AW_TRACK2_L		(scpreg.bus_tracker + 0x310)
#define SCP_BUS_DBG_AW_TRACK3_L		(scpreg.bus_tracker + 0x318)
#define SCP_BUS_DBG_AW_TRACK4_L		(scpreg.bus_tracker + 0x320)
#define SCP_BUS_DBG_AW_TRACK5_L		(scpreg.bus_tracker + 0x328)
#define SCP_BUS_DBG_AW_TRACK6_L		(scpreg.bus_tracker + 0x330)
#define SCP_BUS_DBG_AW_TRACK7_L		(scpreg.bus_tracker + 0x338)
#define SCP_BUS_DBG_AW_TRACK8_L		(scpreg.bus_tracker + 0x340)
#define SCP_BUS_DBG_AW_TRACK9_L		(scpreg.bus_tracker + 0x348)
#define SCP_BUS_DBG_AW_TRACK10_L	(scpreg.bus_tracker + 0x350)
#define SCP_BUS_DBG_AW_TRACK11_L	(scpreg.bus_tracker + 0x358)
#define SCP_BUS_DBG_AW_TRACK12_L	(scpreg.bus_tracker + 0x360)
#define SCP_BUS_DBG_AW_TRACK13_L	(scpreg.bus_tracker + 0x368)
#define SCP_BUS_DBG_AW_TRACK14_L	(scpreg.bus_tracker + 0x370)
#define SCP_BUS_DBG_AW_TRACK15_L	(scpreg.bus_tracker + 0x378)
#define SCP_BUS_DBG_AW_TRACK16_L	(scpreg.bus_tracker + 0x380)
#define SCP_BUS_DBG_AW_TRACK17_L	(scpreg.bus_tracker + 0x388)
#define SCP_BUS_DBG_AW_TRACK18_L	(scpreg.bus_tracker + 0x390)
#define SCP_BUS_DBG_AW_TRACK19_L	(scpreg.bus_tracker + 0x398)
#define SCP_BUS_DBG_AW_TRACK20_L	(scpreg.bus_tracker + 0x3a0)
#define SCP_BUS_DBG_AW_TRACK21_L	(scpreg.bus_tracker + 0x3a8)
#define SCP_BUS_DBG_AW_TRACK22_L	(scpreg.bus_tracker + 0x3b0)
#define SCP_BUS_DBG_AW_TRACK23_L	(scpreg.bus_tracker + 0x3b8)
#define SCP_BUS_DBG_AW_TRACK24_L	(scpreg.bus_tracker + 0x3c0)
#define SCP_BUS_DBG_AW_TRACK25_L	(scpreg.bus_tracker + 0x3c8)
#define SCP_BUS_DBG_AW_TRACK26_L	(scpreg.bus_tracker + 0x3d0)
#define SCP_BUS_DBG_AW_TRACK27_L	(scpreg.bus_tracker + 0x3d8)
#define SCP_BUS_DBG_AW_TRACK28_L	(scpreg.bus_tracker + 0x3e0)
#define SCP_BUS_DBG_AW_TRACK29_L	(scpreg.bus_tracker + 0x3e8)
#define SCP_BUS_DBG_AW_TRACK30_L	(scpreg.bus_tracker + 0x3f0)
#define SCP_BUS_DBG_AW_TRACK31_L	(scpreg.bus_tracker + 0x3f8)

/* clk reg*/
#define SCP_A_SLEEP_DEBUG_REG		(scpreg.clkctrl + 0x0028)
#define SCP_CLK_CTRL_L1_SRAM_PD		(scpreg.clkctrl + 0x002C)
#define SCP_CLK_HIGH_CORE_CG		(scpreg.clkctrl + 0x005C)
#define SCP_CPU0_SRAM_PD		(scpreg.clkctrl + 0x0080)
#define SCP_CPU1_SRAM_PD		(scpreg.clkctrl + 0x0084)
#define SCP_CLK_CTRL_TCM_TAIL_SRAM_PD	(scpreg.clkctrl + 0x0094)
#define SCP_POWER_STATUS				(scpreg.clkctrl + 0x0124)
#define SCP_SLP_PWR_CTRL				(scpreg.clkctrl + 0x0128)

#define CLK_SW_SEL					(scpreg.clkctrl + 0x0)
#define CLK_SW_SEL_O_BIT			8
#define CLK_SW_SEL_O_MASK			0xf
#define CLK_SW_SEL_O_ULPOSC_CORE	0x4
#define CLK_SW_SEL_O_ULPOSC_PERI	0x8

#define CLK_ENABLE				(scpreg.clkctrl + 0x0004)

#define CLK_SAFE_ACK			(scpreg.clkctrl + 0x0008)
#define CLK_SYS_SAFE_ACK_BIT	0
#define CLK_HIGH_SAFE_ACK_BIT	1

#define CLK_HIGH_CORE			(scpreg.clkctrl + 0x005C)
#define HIGH_CORE_CG_BIT		1

#define CLK_ON_CTRL				(scpreg.clkctrl + 0x006C)
#define HIGH_AO_BIT				0
#define HIGH_CG_AO_BIT			2
#define HIGH_CORE_AO_BIT		4
#define HIGH_CORE_DIS_SUB_BIT	5
#define HIGH_CORE_CG_AO_BIT		6
#define HIGH_FINAL_VAL_BIT		8
#define HIGH_FINAL_VAL_MASK		0x1f

#define R_SEC_CTRL			(scpreg.cfg_sec + 0x0000)
	#define B_CORE0_CACHE_DBG_EN	(1 << 28)
	#define B_CORE1_CACHE_DBG_EN	(1 << 29)

#define R_CORE0_CACHE_RAM		(scpreg.l1cctrl + 0x00000)
#define R_CORE1_CACHE_RAM		(scpreg.l1cctrl + 0x20000)

/* scp region info */
#define SCP_DRAM_RESV_EMBED		(0x7)
#define SCP_DRAM_RESV_NO_EMBED(val)	(val & ~(SCP_DRAM_RESV_EMBED))

/* INFRA_IRQ (always on register) */
#define INFRA_IRQ_SET			(scpreg.scpsys + 0x0B14)
	#define AP_AWAKE_LOCK		(0)
	#define AP_AWAKE_UNLOCK		(1)
	#define CONNSYS_AWAKE_LOCK	(2)
	#define CONNSYS_AWAKE_UNLOCK	(3)
#define INFRA_IRQ_CLEAR			(scpreg.scpsys + 0x0B18)
#define SCP_SYS_INFRA_MON		(scpreg.scpsys + 0x0D50)

#endif
