// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015 MediaTek Inc.
 */

#include "ccci_core.h"
#include "ccci_platform.h"

#include "md_sys1_platform.h"
#include "cldma_reg.h"
#include "modem_reg_base.h"
#include "modem_secure_base.h"
#include "ap_md_reg_dump.h"

#define TAG "mcd"

#define RAnd2W(a, b, c)  ccci_write32(a, b, (ccci_read32(a, b)&c))

static size_t mdreg_write32(size_t reg_id, size_t value)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_KERNEL_CCCI_CONTROL, MD_DBGSYS_REG_DUMP, reg_id,
		value, 0, 0, 0, 0, &res);

	return res.a0;
}

/*
 * This file is generated.
 * From 20190924_MT6885_MDReg_remap.xlsx
 * With ap_md_reg_dump_code_gentool.py v0.1
 * Date 2019-10-04 09:48:58.843141
 */
void internal_md_dump_debug_register(unsigned int md_index)
{
	void __iomem *dump_reg0;

	/* dump AP_MDSRC_REQ */
	dump_reg0 = ioremap_wc(0x10006434, 0x4);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x4 bytes from 0x10006434\n");
		return;
	}
	/* Dump 0x1000_6434 - 0x1000_6437 */
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"md_dbg_sys: 0x%X\n", ccci_read32(dump_reg0, 0x0));
	iounmap(dump_reg0);

	/* Set DBGSYS Time out */
	dump_reg0 = ioremap_wc(0x0D10111C, 0x4);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x4 bytes from 0x0D10111C\n");
		return;
	}
	/* Set DBGSYS Time Out Config */
	mdreg_write32(MD_REG_SET_DBGSYS_TIME_OUT_ADDR, 0x8001E848); /*D10111C*/
	/* 0xA060_111C – 0xA060_111F */
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"md_dbg_sys time out: 0x%X\n", ccci_read32(dump_reg0, 0x0));
	iounmap(dump_reg0);

	/* PC Monitor */
	dump_reg0 = ioremap_wc(0x0D11C000, 0x21B0);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x21B0 bytes from 0x0D11C000\n");
		return;
	}
	/* Stop PCMon */
	mdreg_write32(MD_REG_PC_MONITOR_ADDR, 0x2222); /* addr 0xD11DC00 */
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD PC monitor\n");
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"common: 0x0D11DC00\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00001C00), 0x100);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00001D00), 0x100);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00001E00), 0xB0);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00001F00), 0xB0);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00002000), 0xB0);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00002100), 0xB0);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"core0/1/2/3: [0]0x0D11C000, [1]0x0D11C700, [2]0x0D11CE00, [3]0x0D11D500\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0x700);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000700), 0x700);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000E00), 0x700);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00001500), 0x700);
	/* Re-Start PCMon */
	mdreg_write32(MD_REG_PC_MONITOR_ADDR, 0x1111); /* addr 0xD11DC00 */
	iounmap(dump_reg0);

	/* PLL reg (clock control) */
	dump_reg0 = ioremap_wc(0x0D103800, 0x248A0);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x248A0 bytes from 0x0D103800\n");
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD PLL\n");
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"CLKSW: [0]0x0D116000, [1]0x0D116400, [2]0x0D116F00\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00012800), 0x150);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00012A00), 0x30);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00012B00), 0x10);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00012C00), 0x20);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00013700), 0x8);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"PLLMIXED:[0]0x0D114000,[1]0x0D114100,[2]0x0D114200,[3]0x0D114300,[4]0x0D114400,[5]0x0D114500,[6]0x0D114C00,[7]0x0D114D00,[8]0x0D114F00\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00010800), 0xA4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00010900), 0x44);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00010A00), 0x4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00010B00), 0x20);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00010C00), 0xC0);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00010D00), 0x30);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00011000), 0x34);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00011400), 0x44);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00011500), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00011700), 0x14);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"CLKCTL: [0]0x0D103800, [1]0x0D103910\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000110), 0x20);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"GLOBAL CON: [0]0x0D115000, [1]0x0D115090, [2]0x0D115200, [3]0x0D115600, [4]0x0D115700, [5]0x0D115800, [6]0x0D115900, [7]0x0D115D00, [8]0x0D115F00\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00011800), 0x4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00011890), 0x80);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00011A00), 0x344);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00011E00), 0xD8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00011F00), 0xC0);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00012000), 0x70);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00012100), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00012500), 0x4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00012700), 0x8);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"AO CONFIG: [0]0x0D128098\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00024898), 0x8);
	iounmap(dump_reg0);

	/* BUS */
	dump_reg0 = ioremap_wc(0x0D102000, 0x37140);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x37140 bytes from 0x0D102000\n");
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD Bus status: [0]0x0D102000, [1]0x0D102100, [2]0x0D102600, [3]0x0D11F000, [4]0x0D139000, [5]0x0D109000, [6]0x0D128000\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0x90);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000100), 0x320);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000600), 0x110);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0001D000), 0x70);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00037000), 0x140);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00007000), 0x330);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00026000), 0x1A0);
	iounmap(dump_reg0);

	/* BUSMON */
	dump_reg0 = ioremap_wc(0x0D108000, 0x30F24);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x30F24 bytes from 0x0D108000\n");
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD Bus REC: [0]0x0D138000, [1]0x0D108000\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030000), 0x108);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030200), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030220), 0x34);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030280), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x000302A0), 0x34);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030400), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030500), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030700), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030820), 0x4C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030900), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030A00), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030B00), 0x424);
	/* [Pre-Action] Disable bus his rec & select entry 0 */
	mdreg_write32(MD_REG_BUSMON_ADDR_0, 0x0); /* addr 0xD138408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030860), 0xC);
	/* [Pre-Action] Select entry 1 */
	mdreg_write32(MD_REG_BUSMON_ADDR_0, 0x100010); /* addr 0xD138408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030860), 0xC);
	/* [Pre-Action] Select entry 2 */
	mdreg_write32(MD_REG_BUSMON_ADDR_0, 0x200020); /* addr 0xD138408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030860), 0xC);
	/* [Pre-Action] Select entry 3 */
	mdreg_write32(MD_REG_BUSMON_ADDR_0, 0x300030); /* addr 0xD138408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030860), 0xC);
	/* [Pre-Action] Select entry 4 */
	mdreg_write32(MD_REG_BUSMON_ADDR_0, 0x400040); /* addr 0xD138408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030860), 0xC);
	/* [Pre-Action] Select entry 5 */
	mdreg_write32(MD_REG_BUSMON_ADDR_0, 0x500050); /* addr 0xD138408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030860), 0xC);
	/* [Pre-Action] Select entry 6 */
	mdreg_write32(MD_REG_BUSMON_ADDR_0, 0x600060); /* addr 0xD138408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030860), 0xC);
	/* [Pre-Action] Select entry 7 */
	mdreg_write32(MD_REG_BUSMON_ADDR_0, 0x700070); /* addr 0xD138408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00030860), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0x108);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000200), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000220), 0x34);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000280), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x000002A0), 0x34);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000400), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000500), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000700), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000820), 0x4C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000900), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000A00), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000B00), 0x424);
	/* [Pre-Action] Disable bus his rec & select entry 0 */
	mdreg_write32(MD_REG_BUSMON_ADDR_1, 0x0); /* addr 0xD108408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000860), 0xC);
	/* [Pre-Action] Select entry 1 */
	mdreg_write32(MD_REG_BUSMON_ADDR_1, 0x100010); /* addr 0xD108408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000860), 0xC);
	/* [Pre-Action] Select entry 2 */
	mdreg_write32(MD_REG_BUSMON_ADDR_1, 0x200020); /* addr 0xD108408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000860), 0xC);
	/* [Pre-Action] Select entry 3 */
	mdreg_write32(MD_REG_BUSMON_ADDR_1, 0x300030); /* addr 0xD108408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000860), 0xC);
	/* [Pre-Action] Select entry 4 */
	mdreg_write32(MD_REG_BUSMON_ADDR_1, 0x400040); /* addr 0xD108408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000860), 0xC);
	/* [Pre-Action] Select entry 5 */
	mdreg_write32(MD_REG_BUSMON_ADDR_1, 0x500050); /* addr 0xD108408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000860), 0xC);
	/* [Pre-Action] Select entry 6 */
	mdreg_write32(MD_REG_BUSMON_ADDR_1, 0x600060); /* addr 0xD108408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000860), 0xC);
	/* [Pre-Action] Select entry 7 */
	mdreg_write32(MD_REG_BUSMON_ADDR_1, 0x700070); /* addr 0xD108408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000830), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000860), 0xC);
	iounmap(dump_reg0);

	/* ECT */
	dump_reg0 = ioremap_wc(0x0D101100, 0xCF48);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0xCF48 bytes from 0x0D101100\n");
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD ECT: [0]0x0D10C130, [1]0x0D10C134, [2]0x0D10D130, [3]0x0D10D134, [4]0x0D10E000, [5]0x0D10E030, [6]0x0D10E040, [7]0x0D101100\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0000B030), 0x4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0000B034), 0x4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0000C030), 0x4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0000C034), 0x4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0000CF00), 0x18);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0000CF30), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0000CF40), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0x10);
	iounmap(dump_reg0);

	/* TOPSM reg */
	dump_reg0 = ioremap_wc(0x0D110000, 0x8E8);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x8E8 bytes from 0x0D110000\n");
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD TOPSM status: 0x0D110000\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0x8E8);
	iounmap(dump_reg0);

	/* MD RGU reg */
	dump_reg0 = ioremap_wc(0x0D112100, 0x25C);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x25C bytes from 0x0D112100\n");
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD RGU: [0]0x0D112100, [1]0x0D112300\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0xDD);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000200), 0x5C);
	iounmap(dump_reg0);

	/* OST status */
	dump_reg0 = ioremap_wc(0x0D111000, 0x20C);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x20C bytes from 0x0D111000\n");
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD OST status: [0]0x0D111000, [1]0x0D111200\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0xF4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000200), 0xC);
	iounmap(dump_reg0);

	/* CSC reg */
	dump_reg0 = ioremap_wc(0x0D113000, 0x224);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x224 bytes from 0x0D113000\n");
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD CSC: 0x0D113000\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0x224);
	iounmap(dump_reg0);

	/* ELM reg */
	dump_reg0 = ioremap_wc(0x20350000, 0x721);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x721 bytes from 0x20350000\n");
		return;
	}
#if defined(__MD_DEBUG_DUMP__)
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD ELM: 0x20350000\n");
#endif
#if defined(__MD_DEBUG_DUMP__)
	/* This dump might cause bus hang so enable it only when needed */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0x721);
#endif
	iounmap(dump_reg0);

	/* USIP */
	dump_reg0 = ioremap_wc(0x0D104400, 0x52478);
	if (dump_reg0 == NULL) {
		CCCI_MEM_LOG_TAG(md_index, TAG,
			"Dump MD failed to ioremap 0x52478 bytes from 0x0D104400\n");
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump MD USIP: [0]0x0D104400, [1]0x0D104610, [2]0x0D105400, [3]0x0D105610, [4]0x0D13A000, [5]0x0D13A058\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000000), 0x100);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00000210), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00001000), 0x100);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00001210), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C00), 0x100);
	/* [Pre-Action] config usip bus dbg sel 1 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0x10000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 2 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0x20000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 3 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0x30000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 4 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0x40000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 5 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0x50000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 6 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0x60000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 7 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0x70000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 8 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0x80000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 9 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0x90000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 10 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0xA0000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 11 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0xB0000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 12 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0xC0000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 13 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0xD0000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 14 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0xE0000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] config usip bus dbg sel 15 */
	mdreg_write32(MD_REG_USIP_ADDR_0, 0xF0000000); /* addr 0xD13A00C */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00035C58), 0x4);
	/* [Pre-Action] force-on vcore hram bus clock */
	mdreg_write32(MD_REG_USIP_ADDR_1, 0x1); /* addr 0xD153068 */
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore th0 reg: [0]0x0D148000\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043C00), 0x120);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore th1 reg: [0]0x0D148400\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00044000), 0x120);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore th2 reg: [0]0x0D148800\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00044400), 0x120);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore th3 reg: [0]0x0D148C00\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00044800), 0x120);
	/* [Pre-Action] force-on mcore bus clock */
	mdreg_write32(MD_REG_USIP_ADDR_2, 0xAA5); /* addr 0xD148094 */
	/*  */
	mdreg_write32(MD_REG_USIP_ADDR_3, 0xA8300418); /* addr 0xD1480DC */
	/*  */
	mdreg_write32(MD_REG_USIP_ADDR_4, 0x1); /* addr 0xD148218 */
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore hram reg: [0]0x0D153000\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004EC00), 0x7C);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore th0 reg: [0]0x0D150000\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004BC00), 0x120);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore th1 reg: [0]0x0D150400\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004C000), 0x120);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore th2 reg: [0]0x0D150800\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004C400), 0x120);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore th3 reg: [0]0x0D150C00\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004C800), 0x120);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore and vcore PC trace: [0]0x0D14A000, [1]0x0D152000\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00045C00), 0x800);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004DC00), 0x800);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore0 dbus recorder: [0]0x0D152800, [1]0x0D152890, [2]0x0D152900\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004E400), 0x24);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004E490), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004E500), 0x200);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore peri dbus recorder: [0]0x0D155000, [1]0x0D155090, [2]0x0D155100\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00050C00), 0x24);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00050C90), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00050D00), 0x200);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore pmuck abus: [0]0x0D154C30\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00050830), 0x4);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore vcoreck abus: [0]0x0D154430\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00050030), 0x10);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump hram hrambusck abus: [0]0x0D156830\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00052430), 0x48);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump hram hramck abus: [0]0x0D156430\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00052030), 0x4);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore peri dbus: [0]0x0D155800\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00051400), 0x1B0);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump vcore0 internal dbus: [0]0x0D151024\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0004CC24), 0x654);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore0 dbus recorder: [0]0x0D14A800, [1]0x0D14A890, [2]0x0D14A900\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00046400), 0x24);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00046490), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00046500), 0x200);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore0 internal dbus: [0]0x0D149024\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00044C24), 0x5D8);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcoresys dbus recorder 0: [0]0x0D145000, [1]0x0D145090, [2]0x0D145100\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00040C00), 0x24);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00040C90), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00040D00), 0x200);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore peri dbus1: [0]0x0D146000\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00041C00), 0x2E0);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore peri mcoreck abus: [0]0x0D143030, [1]0x0D143080, [2]0x0D1430A4\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0003EC30), 0x30);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0003EC80), 0x4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0003ECA4), 0xC0);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore peri busck abus: [0]0x0D142830, [1]0x0D1428A4\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0003E430), 0x38);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0003E4A4), 0x94);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump Mcoresys Bus REC: [0]0x0D147000, [1]0x0D147B00\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00042C00), 0x108);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00042E00), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00042E20), 0x34);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00042E80), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00042EA0), 0x34);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043000), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043100), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043300), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043420), 0x4C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043500), 0x1C);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043600), 0x8);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043700), 0x424);
	/* [Pre-Action] Disable bus his rec & select entry 0 */
	mdreg_write32(MD_REG_USIP_ADDR_5, 0x0); /* addr 0xD147408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043430), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043460), 0xC);
	/* [Pre-Action] Select entry 1 */
	mdreg_write32(MD_REG_USIP_ADDR_5, 0x100010); /* addr 0xD147408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043430), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043460), 0xC);
	/* [Pre-Action] Select entry 2 */
	mdreg_write32(MD_REG_USIP_ADDR_5, 0x200020); /* addr 0xD147408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043430), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043460), 0xC);
	/* [Pre-Action] Select entry 3 */
	mdreg_write32(MD_REG_USIP_ADDR_5, 0x300030); /* addr 0xD147408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043430), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043460), 0xC);
	/* [Pre-Action] Select entry 4 */
	mdreg_write32(MD_REG_USIP_ADDR_5, 0x400040); /* addr 0xD147408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043430), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043460), 0xC);
	/* [Pre-Action] Select entry 5 */
	mdreg_write32(MD_REG_USIP_ADDR_5, 0x500050); /* addr 0xD147408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043430), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043460), 0xC);
	/* [Pre-Action] Select entry 6 */
	mdreg_write32(MD_REG_USIP_ADDR_5, 0x600060); /* addr 0xD147408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043430), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043460), 0xC);
	/* [Pre-Action] Select entry 7 */
	mdreg_write32(MD_REG_USIP_ADDR_5, 0x700070); /* addr 0xD147408 */
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043430), 0xC);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00043460), 0xC);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore peri perick abus: [0]0x0D143830, [1]0x0D143870\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0003F430), 0x4);
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x0003F470), 0x4);
	CCCI_MEM_LOG_TAG(md_index, TAG,
		"Dump mcore peri dbus2: [0]0x0D144800\n");
	ccci_util_mem_dump(md_index, CCCI_DUMP_MEM_DUMP,
		(dump_reg0 + 0x00040400), 0xB0);
	iounmap(dump_reg0);
}

#define MD_REG_DUMP_MAGIC   (0x44554D50) /* DUMP */

static int md_dump_mem_once(unsigned int md_id, const void *buff_src,
		unsigned long dump_len)
{
	int ret;
	unsigned long i;
	unsigned long dump_limit;
	unsigned int tmp_idx;
	char temp_buf[132] = { 0 };

	if (!buff_src || !dump_len) {
		CCCI_ERROR_LOG(md_id, TAG, "%s: buff_src = NULL or dump_len = 0x%lx\n",
			__func__, dump_len);
		return -1;
	}

	for (i = 0; i < dump_len;) {
		dump_limit = ((dump_len - i) < 128) ? (dump_len - i) : 128;
		memcpy_fromio(temp_buf, (buff_src + i), dump_limit);
		temp_buf[dump_limit] = 0;
		tmp_idx = 0;

		while (unlikely(strlen(temp_buf + tmp_idx) == 0) && (tmp_idx < dump_limit)) {
			CCCI_ERROR_LOG(md_id, TAG,
				"%s: strlen return 0, dump_limit:0x%lx, i:0x%lx, temp_buf[%d]:%d\n",
				__func__, dump_limit, i, tmp_idx, temp_buf[tmp_idx]);
			i++;
			tmp_idx++;
		}

		if (tmp_idx < dump_limit)
			ret = ccci_dump_write(md_id, CCCI_DUMP_MEM_DUMP, 0, "%s",
				(temp_buf + tmp_idx));
		else {
			CCCI_ERROR_LOG(md_id, TAG,
				"%s: dump_limit:0x%lx, i:0x%lx, temp_buf[0->%d] are all 0\n",
				__func__, dump_limit, i, tmp_idx);
			continue;
		}

		if (ret < 0) {
			CCCI_ERROR_LOG(md_id, TAG,
				"%s: ccci_dump_write fail %d, 0x%llx, %lu, 0x%llx\n",
				__func__, ret, dump_limit, strlen(temp_buf + tmp_idx), i);
			return -2;
		} else if (!ret) {
			CCCI_ERROR_LOG(md_id, TAG,
				"%s: ccci_dump_write return 0: %d, 0x%lx, %lu, 0x%lx\n",
				__func__, ret, dump_limit, strlen(temp_buf + tmp_idx), i);
			i++;
		}
		i += ret;
	}

	return i;
}

void md_dump_reg(unsigned int md_index)
{
	struct arm_smccc_res res;
	unsigned long long buf_addr, buf_size, total_len = 0;
	void *buff_src;
	int loop_max_cnt = 0, ret;

	/* 1. start to dump, send start command */
	arm_smccc_smc(MTK_SIP_KERNEL_CCCI_CONTROL, MD_DEBUG_DUMP,
		MD_REG_DUMP_START, MD_REG_DUMP_MAGIC, 0, 0, 0, 0, &res);
	buf_addr = res.a1;
	buf_size = res.a2;
	CCCI_NORMAL_LOG(-1, TAG,
		"[%s][MD_REG_DUMP_START] flag_1=0x%llx, flag_2=0x%llx, flag_3=0x%llx, flag_4=0x%llx\n",
		__func__, res.a0, res.a1, res.a2, res.a3);
	/* go kernel debug red dump,fix me,we need make it more compatible later */
	if ((res.a0 & 0xffff0000) != 0) {
		CCCI_NORMAL_LOG(-1, TAG, "[%s] go kernel md reg dump\n", __func__);
		internal_md_dump_debug_register(md_index);
		return;
	}

	if (buf_addr <= 0 || buf_size <= 0)
		return;
	/* get read buffer, remap */
	buff_src = ioremap_wt(buf_addr, buf_size);
	if (buff_src == NULL) {
		CCCI_ERROR_LOG(md_index, TAG,
			"Dump MD failed to ioremap 0x%llx bytes from 0x%llx\n",
			buf_size, buf_addr);
		return;
	}
	CCCI_MEM_LOG_TAG(md_index, TAG, "MD register dump start, 0x%llx\n",
		(unsigned long long)buff_src);
	/* 2. get dump data, send dump_stage command to get */
	do {
		loop_max_cnt++;
		arm_smccc_smc(MTK_SIP_KERNEL_CCCI_CONTROL, MD_DEBUG_DUMP,
			MD_REG_DUMP_STAGE, 0, 0, 0, 0, 0, &res);
		CCCI_DEBUG_LOG(-1, TAG,
			"[%s][MD_REG_DUMP_STAGE] flag_0=0x%llx, flag_1=0x%llx, flag_2=0x%llx, flag_3=0x%llx\n",
			__func__, res.a0, res.a1, res.a2, res.a3);
		switch (res.a2) {
		case DUMP_FINISHED: /* go through */
		case DUMP_UNFINISHED:
			if (!res.a1)
				goto DUMP_END;
			ret = md_dump_mem_once(md_index, buff_src, res.a1);
			if (ret < 0)
				goto DUMP_END;
			total_len += ret;
			break;
		case DUMP_DELAY_us:
			/* delay a3 */
			break;
		default:
			break;
		}
	} while (res.a2);

DUMP_END:
	CCCI_NORMAL_LOG(-1, TAG,
		"[%s]MD register dump end 0x%x, 0x%llx, 0x%llx\n",
		__func__, loop_max_cnt, buf_size, total_len);
	CCCI_MEM_LOG_TAG(-1, TAG,
		"[%s]MD register dump end 0x%x, 0x%llx, 0x%llx\n",
		__func__, loop_max_cnt, buf_size, total_len);
	iounmap(buff_src);
}

void md_dump_register_6873(unsigned int md_index)
{
	internal_md_dump_debug_register(md_index);
}
