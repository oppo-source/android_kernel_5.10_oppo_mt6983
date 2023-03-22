// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#include "cmdq_helper_ext.h"
#include "cmdq_reg.h"
#include "cmdq_device.h"
#include "cmdq_virtual.h"
#include <linux/seq_file.h>
#ifdef CMDQ_CG_M4U_LARB0
#include "m4u.h"
#endif
#ifdef CONFIG_MTK_SMI_EXT
#include "smi_public.h"
#endif

static struct cmdqCoreFuncStruct gFunctionPointer;

u64 cmdq_virtual_flag_from_scenario_default(enum CMDQ_SCENARIO_ENUM scn)
{
	u64 flag = 0;

	switch (scn) {
	case CMDQ_SCENARIO_TRIGGER_LOOP:
		/* Trigger loop does not related to any HW by itself. */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_USER_SPACE:
		/* user space case, engine flag is passed seprately */
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_DEBUG:
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		flag = 0LL;
		break;

	case CMDQ_SCENARIO_USER_DISP_COLOR:
		/* color path */
		flag = 0LL;
		break;

	default:
		if (scn >= CMDQ_MAX_SCENARIO_COUNT) {
			/* Error status print */
			CMDQ_ERR("Unknown scenario type %d\n", scn);
		}
		flag = 0LL;
		break;
	}

	return flag;
}

const char *cmdq_virtual_parse_module_from_reg_addr_legacy(u32 reg_addr)
{
	const u32 addr_base_and_page = (reg_addr & 0xFFFFF000);

	/* for well-known base, we check them with 12-bit mask
	 * defined in mt_reg_base.h
	 * TODO: comfirm with SS if IO_VIRT_TO_PHYS workable when enable device
	 * tree?
	 */
	switch (addr_base_and_page) {
	case 0x14000000:
		return "MMSYS";
	case 0x14001000:
		return "MDP_RDMA0";
	case 0x14002000:
		return "MDP_RDMA1";
	case 0x14003000:
		return "MDP_RSZ0";
	case 0x14004000:
		return "MDP_RSZ1";
	case 0x14005000:
		return "MDP_RSZ2";
	case 0x14006000:
		return "MDP_WDMA";
	case 0x14007000:
		return "MDP_WROT0";
	case 0x14008000:
		return "MDP_WROT1";
	case 0x14009000:
		return "MDP_TDSHP0";
	case 0x1400A000:
		return "MDP_TDSHP1";
	case 0x1400B000:
		return "MDP_CROP";
	case 0x1400C000:
		return "DISP_OVL0";
	case 0x1400D000:
		return "DISP_OVL1";
	case 0x14013000:
		return "COLOR0";
	case 0x14014000:
		return "COLOR1";
	case 0x14015000:
		return "AAL";
	case 0x14016000:
		return "GAMA";
	case 0x14020000:
		return "MMSYS_MUTEX";
	case 0x18000000:
		return "VENC_GCON";
	case 0x18002000:
		return "VENC";
	case 0x18003000:
		return "JPGENC";
	case 0x18004000:
		return "JPGDEC";
	}

	/* for other register address we rely on GCE subsys to group them
	 * with 16-bit mask.
	 */
	return cmdq_core_parse_subsys_from_reg_addr(reg_addr);
}

/*
 * GCE capability
 */
u32 cmdq_virtual_get_subsys_LSB_in_arg_a(void)
{
	return 16;
}

/* HW thread related */
bool cmdq_virtual_is_a_secure_thread(const s32 thread)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	if ((thread >= CMDQ_MIN_SECURE_THREAD_ID) &&
		(thread < CMDQ_MIN_SECURE_THREAD_ID +
		CMDQ_MAX_SECURE_THREAD_COUNT)) {
		return true;
	}
#endif
	return false;
}

/**
 * Scenario related
 *
 */
bool cmdq_virtual_is_disp_scenario(const enum CMDQ_SCENARIO_ENUM scenario)
{
	bool dispScenario = false;

	switch (scenario) {
	case CMDQ_SCENARIO_TRIGGER_LOOP:
	case CMDQ_SCENARIO_USER_DISP_COLOR:
		dispScenario = true;
		break;
	default:
		break;
	}
	/* freely dispatch */
	return dispScenario;
}

bool cmdq_virtual_is_dynamic_scenario(
	const enum CMDQ_SCENARIO_ENUM scenario)
{
	bool dynamic_thread;

	switch (scenario) {
	case CMDQ_SCENARIO_USER_SPACE:
	case CMDQ_SCENARIO_USER_MDP:
	case CMDQ_SCENARIO_DEBUG_MDP:
		dynamic_thread = true;
		break;
	default:
		dynamic_thread = false;
		break;
	}

	return dynamic_thread;
}

int cmdq_virtual_disp_thread(enum CMDQ_SCENARIO_ENUM scenario)
{
	switch (scenario) {
	/* HACK: force debug into 0/1 thread */
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		/* primary config: thread 0 */
		return 0;

	case CMDQ_SCENARIO_USER_DISP_COLOR:
		return 4;
	case CMDQ_SCENARIO_TRIGGER_LOOP:
		return 7;
	default:
		/* freely dispatch */
		return CMDQ_INVALID_THREAD;
	}
	/* freely dispatch */
	return CMDQ_INVALID_THREAD;
}

int cmdq_virtual_get_thread_index(enum CMDQ_SCENARIO_ENUM scenario,
	const bool secure)
{
	if (!secure)
		return cmdq_get_func()->dispThread(scenario);

	/* dispatch secure thread according to scenario */
	switch (scenario) {
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		/* CMDQ_MIN_SECURE_THREAD_ID */
		return CMDQ_THREAD_SEC_PRIMARY_DISP;
	case CMDQ_SCENARIO_USER_MDP:
	case CMDQ_SCENARIO_USER_SPACE:
	case CMDQ_SCENARIO_DEBUG:
	case CMDQ_SCENARIO_DEBUG_MDP:
		/* because there is one input engine for MDP, reserve one
		 * secure thread is enough
		 */
		return CMDQ_THREAD_SEC_MDP;
	case CMDQ_SCENARIO_ISP_FDVT:
	case CMDQ_SCENARIO_ISP_FDVT_OFF:
		return CMDQ_THREAD_SEC_ISP;
	default:
		CMDQ_ERR("no dedicated secure thread for scenario:%d\n",
			scenario);
		return CMDQ_INVALID_THREAD;
	}
}

enum CMDQ_HW_THREAD_PRIORITY_ENUM cmdq_virtual_priority_from_scenario(
	enum CMDQ_SCENARIO_ENUM scenario)
{
	switch (scenario) {
	case CMDQ_SCENARIO_USER_DISP_COLOR:
		/* currently, a prefetch thread is always in high priority. */
		return CMDQ_THR_PRIO_DISPLAY_CONFIG;

		/* HACK: force debug into 0/1 thread */
	case CMDQ_SCENARIO_DEBUG_PREFETCH:
		return CMDQ_THR_PRIO_DISPLAY_CONFIG;

	default:
		/* other cases need exta logic, see below. */
		break;
	}

	if (cmdq_get_func()->is_disp_loop(scenario))
		return CMDQ_THR_PRIO_DISPLAY_TRIGGER;
	else
		return CMDQ_THR_PRIO_NORMAL;
}

bool cmdq_virtual_is_disp_loop(enum CMDQ_SCENARIO_ENUM scenario)
{
	bool is_disp_loop = false;

	if (scenario == CMDQ_SCENARIO_TRIGGER_LOOP)
		is_disp_loop = true;

	return is_disp_loop;
}

const char *cmdq_virtual_parse_module_from_reg_addr(u32 reg_addr)
{
	const u32 addr_base_and_page = (reg_addr & 0xFFFFF000);

#ifdef CMDQ_USE_LEGACY
	return cmdq_virtual_parse_module_from_reg_addr_legacy(reg_addr);
#else
	/* for well-known base, we check them with 12-bit mask
	 * defined in mt_reg_base.h
	 * TODO: comfirm with SS if IO_VIRT_TO_PHYS workable when enable
	 * device tree?
	 */
	switch (addr_base_and_page) {
	case 0x14001000: /* MDP_RDMA0 */
	case 0x14002000: /* MDP_RDMA1 */
	case 0x14003000: /* MDP_RSZ0 */
	case 0x14004000: /* MDP_RSZ1 */
	case 0x14005000: /* MDP_RSZ2 */
	case 0x14006000: /* MDP_WDMA */
	case 0x14007000: /* MDP_WROT0 */
	case 0x14008000: /* MDP_WROT1 */
	case 0x14009000: /* MDP_TDSHP */
		return "MDP";
	case 0x14014000: /* DISP_COLOR0 */
	case 0x14015000: /* DISP_COLOR1 */
		return "COLOR";
	case 0x14016000: /* DISP_COLOR0 */
	case 0x14017000: /* DISP_COLOR1 */
		return "CCORR";
	case 0x1400B000: /* DISP_OVL0 */
	case 0x1400D000: /* DISP_OVL0_2L */
		return "OVL0";
	case 0x1400C000: /* DISP_OVL1 */
	case 0x1400E000: /* DISP_OVL1_2L */
		return "OVL1";
	case 0x14018000: /* DISP_AAL0 */
	case 0x14019000: /* DISP_AAL1 */
	case 0x1401a000: /* DISP_GAMMA0 */
	case 0x1401b000: /* DISP_GAMMA1 */
		return "AAL";
	case 0x17020000: /* VENC */
		return "VENC";
	case 0x17030000: /* JPGENC */
		return "JPGENC";
	case 0x17040000: /* JPGDEC */
		return "JPGDEC";
	}

	/* for other register address we rely on GCE subsys to group them
	 * with 16-bit mask.
	 */
	return cmdq_core_parse_subsys_from_reg_addr(reg_addr);
#endif
}

ssize_t cmdq_virtual_print_status_clock(char *buf)
{
	s32 length = 0;
	char *pBuffer = buf;

#ifdef CMDQ_PWR_AWARE
	/* MT_CG_DISP0_MUTEX_32K is removed in this platform */
	pBuffer += sprintf(pBuffer, "MT_CG_INFRA_GCE: %d\n",
		cmdq_dev_gce_clock_is_enable());

	pBuffer += sprintf(pBuffer, "\n");
#endif

	length = pBuffer - buf;
	return length;
}

void cmdq_virtual_print_status_seq_clock(struct seq_file *m)
{
#ifdef CMDQ_PWR_AWARE
	/* MT_CG_DISP0_MUTEX_32K is removed in this platform */
	seq_printf(m, "MT_CG_INFRA_GCE: %d", cmdq_dev_gce_clock_is_enable());

	seq_puts(m, "\n");
#endif
}

void cmdq_virtual_enable_common_clock_locked(bool enable)
{
#ifdef CMDQ_PWR_AWARE
	if (enable) {
		CMDQ_VERBOSE("[CLOCK] Enable SMI & LARB0 Clock\n");
		/* Use SMI clock API */
#ifdef CONFIG_MTK_SMI_EXT
		smi_bus_prepare_enable(SMI_LARB0, "CMDQ");
#endif
	} else {
		CMDQ_VERBOSE("[CLOCK] Disable SMI & LARB0 Clock\n");
		/* disable, reverse the sequence */
#ifdef CONFIG_MTK_SMI_EXT
		smi_bus_disable_unprepare(SMI_LARB0, "CMDQ");
#endif
	}
#endif				/* CMDQ_PWR_AWARE */
}

void cmdq_virtual_enable_gce_clock_locked(bool enable)
{
#ifdef CMDQ_PWR_AWARE
	if (enable) {
		CMDQ_VERBOSE("[CLOCK] Enable CMDQ(GCE) Clock\n");
		cmdq_dev_enable_gce_clock(enable);
	} else {
		CMDQ_VERBOSE("[CLOCK] Disable CMDQ(GCE) Clock\n");
		cmdq_dev_enable_gce_clock(enable);
	}
#endif
}


const char *cmdq_virtual_parse_handle_error_module_by_hwflag_impl(
	const struct cmdqRecStruct *pHandle)
{
	const char *module = NULL;

	if (cmdq_get_func()->isDispScenario(pHandle->scenario))
		module = "DISP";
	else
		module = cmdq_mdp_parse_handle_error_module_by_hwflag(pHandle);

	/* other case, we need to analysis instruction for more detail */
	return module;
}

const char *cmdq_virtual_parse_error_module_by_hwflag_impl(
	const struct cmdqRecStruct *task)
{
	const char *module = NULL;

	/* TODO: fill in correct dispatch module */
	if (cmdq_get_func()->isDispScenario(task->scenario))
		module = "DISP";
	else
		module = cmdq_mdp_parse_handle_error_module_by_hwflag(task);
	/* other case, we need to analysis instruction for more detail */
	return module;
}

/**
 * Debug
 *
 */
int cmdq_virtual_dump_smi(const int showSmiDump)
{
	int isSMIHang = 0;

#if defined(CONFIG_MTK_SMI_EXT) && !defined(CONFIG_FPGA_EARLY_PORTING) && \
	!defined(CONFIG_MTK_SMI_VARIANT)
	isSMIHang = smi_debug_bus_hang_detect(showSmiDump, "CMDQ");
	CMDQ_ERR("SMI Hang? = %d\n", isSMIHang);
#else
	CMDQ_LOG("[WARNING]not enable SMI dump now\n");
#endif

	return isSMIHang;
}

void cmdq_virtual_dump_gpr(void)
{
	int i = 0;
	long offset = 0;
	u32 value = 0;

	CMDQ_LOG("========= GPR dump =========\n");
	for (i = 0; i < 16; i++) {
		offset = CMDQ_GPR_R32(i);
		value = CMDQ_REG_GET32(offset);
		CMDQ_LOG("[GPR %2d]+0x%lx = 0x%08x\n", i, offset, value);
	}
	CMDQ_LOG("========= GPR dump =========\n");
}


/**
 * Event backup
 *
 */
struct cmdq_backup_event_struct {
	enum cmdq_event EventID;
	u32 BackupValue;
};

static struct cmdq_backup_event_struct g_cmdq_backup_event[] = {
#ifdef CMDQ_EVENT_NEED_BACKUP
	{CMDQ_SYNC_TOKEN_VENC_EOF, 0,},
	{CMDQ_SYNC_TOKEN_VENC_INPUT_READY, 0,},
#endif				/* CMDQ_EVENT_NEED_BACKUP */
};


void cmdq_virtual_event_backup(void)
{
	int i;
	int array_size = (sizeof(g_cmdq_backup_event) /
		sizeof(struct cmdq_backup_event_struct));

	for (i = 0; i < array_size; i++) {
		if (g_cmdq_backup_event[i].EventID < 0 ||
			g_cmdq_backup_event[i].EventID >= CMDQ_SYNC_TOKEN_MAX)
			continue;

		g_cmdq_backup_event[i].BackupValue = cmdqCoreGetEvent(
			g_cmdq_backup_event[i].EventID);
		CMDQ_MSG("[backup event] event: %s, value: %d\n",
			cmdq_core_get_event_name_enum(
			g_cmdq_backup_event[i].EventID),
			g_cmdq_backup_event[i].BackupValue);
	}
}

void cmdq_virtual_event_restore(void)
{
	int i;
	int array_size = (sizeof(g_cmdq_backup_event) /
		sizeof(struct cmdq_backup_event_struct));

	for (i = 0; i < array_size; i++) {
		if (g_cmdq_backup_event[i].EventID < 0 ||
			g_cmdq_backup_event[i].EventID >= CMDQ_SYNC_TOKEN_MAX)
			continue;

		CMDQ_MSG("[restore event] event: %s, value: %d\n",
			cmdq_core_get_event_name_enum(
			g_cmdq_backup_event[i].EventID),
			g_cmdq_backup_event[i].BackupValue);

		if (g_cmdq_backup_event[i].BackupValue == 1)
			cmdqCoreSetEvent(g_cmdq_backup_event[i].EventID);
		else if (g_cmdq_backup_event[i].BackupValue == 0)
			cmdqCoreClearEvent(g_cmdq_backup_event[i].EventID);
	}
}

/**
 * Test
 *
 */
void cmdq_virtual_test_setup(void)
{
	/* unconditionally set CMDQ_SYNC_TOKEN_CONFIG_ALLOW and mutex
	 * STREAM_DONE so that DISPSYS scenarios may pass check.
	 */
}

void cmdq_virtual_test_cleanup(void)
{
	/* do nothing */
}

void cmdq_virtual_init_module_PA_stat(void)
{
}

void cmdq_virtual_function_setting(void)
{
	struct cmdqCoreFuncStruct *pFunc;

	pFunc = &(gFunctionPointer);

	/*
	 * GCE capability
	 */
	pFunc->getSubsysLSBArgA = cmdq_virtual_get_subsys_LSB_in_arg_a;

	/* HW thread related */
	pFunc->isSecureThread = cmdq_virtual_is_a_secure_thread;

	/**
	 * Scenario related
	 *
	 */
	pFunc->isDispScenario = cmdq_virtual_is_disp_scenario;
	pFunc->isDynamic = cmdq_virtual_is_dynamic_scenario;
	pFunc->dispThread = cmdq_virtual_disp_thread;
	pFunc->getThreadID = cmdq_virtual_get_thread_index;
	pFunc->priority = cmdq_virtual_priority_from_scenario;
	pFunc->is_disp_loop = cmdq_virtual_is_disp_loop;

	/**
	 * Module dependent
	 *
	 */
	pFunc->parseModule = cmdq_virtual_parse_module_from_reg_addr;
	pFunc->printStatusClock = cmdq_virtual_print_status_clock;
	pFunc->printStatusSeqClock = cmdq_virtual_print_status_seq_clock;
	pFunc->enableGCEClockLocked = cmdq_virtual_enable_gce_clock_locked;
	pFunc->parseErrorModule =
		cmdq_virtual_parse_error_module_by_hwflag_impl;
	pFunc->parseHandleErrorModule =
		cmdq_virtual_parse_handle_error_module_by_hwflag_impl;

	/**
	 * Debug
	 *
	 */
	pFunc->dumpSMI = cmdq_virtual_dump_smi;
	pFunc->dumpGPR = cmdq_virtual_dump_gpr;

	/**
	 * Event backup
	 *
	 */
	pFunc->eventBackup = cmdq_virtual_event_backup;
	pFunc->eventRestore = cmdq_virtual_event_restore;

	/**
	 * Test
	 *
	 */
	pFunc->testSetup = cmdq_virtual_test_setup;
	pFunc->testCleanup = cmdq_virtual_test_cleanup;
	pFunc->initModulePAStat = cmdq_virtual_init_module_PA_stat;
}

struct cmdqCoreFuncStruct *cmdq_get_func(void)
{
	return &gFunctionPointer;
}
