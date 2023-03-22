// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015 MediaTek Inc.
 */
#include <linux/kernel.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include "ccci_config.h"
#include "ccci_common_config.h"
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#include <mt-plat/aee.h>
#endif
#include "mdee_dumper_v1.h"

#ifndef DB_OPT_DEFAULT
#define DB_OPT_DEFAULT    (0)	/* Dummy macro define to avoid build error */
#endif

#ifndef DB_OPT_FTRACE
#define DB_OPT_FTRACE   (0)	/* Dummy macro define to avoid build error */
#endif

static void ccci_aed_v1(struct ccci_fsm_ee *mdee, unsigned int dump_flag,
	const char *aed_str, int db_opt)
{
	void *ex_log_addr = NULL;
	int ex_log_len = 0;
	void *md_img_addr = NULL;
	int md_img_len = 0;
	int info_str_len = 0;
	char *buff;		/*[AED_STR_LEN]; */
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
	char buf_fail[] = "Fail alloc mem for exception\n";
#endif
	char *img_inf = NULL;
	int md_id = mdee->md_id;
	struct mdee_dumper_v1 *dumper = mdee->dumper_obj;
	struct ccci_smem_region *mdss_dbg =
		ccci_md_get_smem_by_user_id(mdee->md_id,
			SMEM_USER_RAW_MDSS_DBG);
	struct ccci_per_md *per_md_data = ccci_get_per_md_data(mdee->md_id);
	int md_dbg_dump_flag = per_md_data->md_dbg_dump_flag;

	buff = kmalloc(AED_STR_LEN, GFP_ATOMIC);
	if (buff == NULL) {
		CCCI_ERROR_LOG(md_id, FSM, "Fail alloc Mem for buff, %d!\n",
			md_dbg_dump_flag);
		goto err_exit1;
	}
	img_inf = ccci_get_md_info_str(md_id);
	if (img_inf == NULL)
		img_inf = "";
	info_str_len = strlen(aed_str);
	info_str_len += strlen(img_inf);

	if (info_str_len > AED_STR_LEN)
		/* Cut string length to AED_STR_LEN */
		buff[AED_STR_LEN - 1] = '\0';

	scnprintf(buff, AED_STR_LEN, "md%d:%s%s", md_id + 1, aed_str, img_inf);
	/* MD ID must sync with aee_dump_ccci_debug_info() */
 err_exit1:
	if (dump_flag & CCCI_AED_DUMP_CCIF_REG) {
		/* check this first, as we overwrite share memory here */
		ex_log_addr = mdss_dbg->base_ap_view_vir;
		ex_log_len = mdss_dbg->size;
		ccci_md_dump_info(mdee->md_id,
			DUMP_FLAG_CCIF | DUMP_FLAG_CCIF_REG,
			mdss_dbg->base_ap_view_vir + CCCI_EE_OFFSET_CCIF_SRAM,
			CCCI_EE_SIZE_CCIF_SRAM);
	}
	if (dump_flag & CCCI_AED_DUMP_EX_MEM) {
		ex_log_addr = mdss_dbg->base_ap_view_vir;
		ex_log_len = mdss_dbg->size;
	}
	if (dump_flag & CCCI_AED_DUMP_EX_PKT) {
		ex_log_addr = (void *)&dumper->ex_info;
		ex_log_len = sizeof(struct ex_log_t);
	}
	if (buff == NULL) {
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
		if (md_dbg_dump_flag & (1U << MD_DBG_DUMP_SMEM))
			aed_md_exception_api(ex_log_addr, ex_log_len,
				md_img_addr, md_img_len, buf_fail, db_opt);
		else
			aed_md_exception_api(NULL, 0, md_img_addr, md_img_len,
				buf_fail, db_opt);
#endif
	} else {
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
		if (md_dbg_dump_flag & (1 << MD_DBG_DUMP_SMEM))
			aed_md_exception_api(ex_log_addr, ex_log_len,
				md_img_addr, md_img_len, buff, db_opt);
		else
			aed_md_exception_api(NULL, 0, md_img_addr, md_img_len,
				buff, db_opt);
#endif
		kfree(buff);
	}
}
static void mdee_dumper_info_dump_v1(struct ccci_fsm_ee *mdee)
{
	struct mdee_dumper_v1 *dumper = mdee->dumper_obj;
	int md_id = mdee->md_id;
	char *ex_info = NULL;/* [EE_BUF_LEN] = ""; */
	char *ex_info_temp = NULL;
	/* [EE_BUF_LEN] = "\n[Others] May I-Bit dis too long\n"; */
	char *i_bit_ex_info = NULL;
	char buf_fail[] = "Fail alloc mem for exception\n";
	int db_opt = (DB_OPT_DEFAULT | DB_OPT_FTRACE);
	int dump_flag = 0;
	struct debug_info_t *debug_info = &dumper->debug_info;
	unsigned char c;
	int md_state = ccci_fsm_get_md_state(mdee->md_id);
	struct ccci_smem_region *mdccci_dbg =
		ccci_md_get_smem_by_user_id(mdee->md_id,
			SMEM_USER_RAW_MDCCCI_DBG);
	struct ccci_smem_region *mdss_dbg =
		ccci_md_get_smem_by_user_id(mdee->md_id,
			SMEM_USER_RAW_MDSS_DBG);
	struct rtc_time tm;
	struct timespec64 time_spec64;
	struct timespec64 tv = { 0 };
	struct timespec64 tv_android = { 0 };
	struct rtc_time tm_android;
	struct ccci_per_md *per_md_data =
			ccci_get_per_md_data(mdee->md_id);
	int md_dbg_dump_flag = per_md_data->md_dbg_dump_flag;

	ktime_get_real_ts64(&time_spec64);
	tv.tv_sec = time_spec64.tv_sec;
	tv.tv_nsec = time_spec64.tv_nsec;
	tv_android = tv;
	rtc_time64_to_tm(tv.tv_sec, &tm);
	tv_android.tv_sec -= (time64_t)sys_tz.tz_minuteswest * 60;
	rtc_time64_to_tm(tv_android.tv_sec, &tm_android);
	CCCI_ERROR_LOG(md_id, FSM,
	"Sync:%d%02d%02d %02d:%02d:%02d.%u(%02d:%02d:%02d.%03d(TZone))\n",
	tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
	tm.tm_hour, tm.tm_min, tm.tm_sec,
	(unsigned int)tv.tv_nsec,
	tm_android.tm_hour, tm_android.tm_min, tm_android.tm_sec,
	(unsigned int)tv_android.tv_nsec);

	ex_info = kmalloc(EE_BUF_LEN, GFP_ATOMIC);
	if (ex_info == NULL) {
		CCCI_ERROR_LOG(md_id, FSM, "Fail alloc Mem for ex_info!\n");
		goto err_exit;
	}
	CCCI_ERROR_LOG(md_id, FSM, "exception type(%d):%s\n",
	debug_info->type, debug_info->name ? : "Unknown");

	switch (debug_info->type) {
	case MD_EX_TYPE_ASSERT_DUMP:
		/* Fall through */
	case MD_EX_TYPE_ASSERT:
		CCCI_NORMAL_LOG(md_id, FSM, "filename = %s\n",
			debug_info->assert.file_name);
		CCCI_NORMAL_LOG(md_id, FSM, "line = %d\n",
			debug_info->assert.line_num);
		CCCI_NORMAL_LOG(md_id, FSM,
				"para0 = %d, para1 = %d, para2 = %d\n",
				debug_info->assert.parameters[0],
				debug_info->assert.parameters[1],
				debug_info->assert.parameters[2]);
		scnprintf(ex_info, EE_BUF_LEN,
			"\n[%s] file:%s line:%d\np1:0x%08x\np2:0x%08x\np3:0x%08x\n",
			debug_info->name,
			debug_info->assert.file_name,
			debug_info->assert.line_num,
			debug_info->assert.parameters[0],
			debug_info->assert.parameters[1],
			debug_info->assert.parameters[2]);
		break;
	case MD_EX_TYPE_UNDEF:
		/* Fall through */
	case MD_EX_TYPE_SWI:
		/* Fall through */
	case MD_EX_TYPE_PREF_ABT:
		/* Fall through */
	case MD_EX_TYPE_DATA_ABT:
		/* Fall through */
	case MD_EX_TYPE_FATALERR_BUF:
		/* Fall through */
	case MD_EX_TYPE_FATALERR_TASK:
		/* Fall through */
	case MD_EX_TYPE_C2K_ERROR:
		CCCI_NORMAL_LOG(md_id, FSM, "fatal error code 1 = %d\n",
			debug_info->fatal_error.err_code1);
		CCCI_NORMAL_LOG(md_id, FSM, "fatal error code 2 = %d\n",
			debug_info->fatal_error.err_code2);
		CCCI_NORMAL_LOG(md_id, FSM, "fatal error offender %s\n",
			debug_info->fatal_error.offender);
		if (debug_info->fatal_error.offender[0] != '\0') {
			scnprintf(ex_info, EE_BUF_LEN,
			"\n[%s] err_code1:%d err_code2:%d\nMD Offender:%s\n",
			debug_info->name, debug_info->fatal_error.err_code1,
			debug_info->fatal_error.err_code2,
			debug_info->fatal_error.offender);
		} else {
			scnprintf(ex_info, EE_BUF_LEN,
			"\n[%s] err_code1:%d err_code2:%d\n", debug_info->name,
			debug_info->fatal_error.err_code1,
			debug_info->fatal_error.err_code2);
		}
		break;
	case CC_MD1_EXCEPTION:
		CCCI_NORMAL_LOG(md_id, FSM, "fatal error code 1 = %d\n",
		debug_info->fatal_error.err_code1);
		CCCI_NORMAL_LOG(md_id, FSM, "fatal error code 2 = %d\n",
		debug_info->fatal_error.err_code2);
		scnprintf(ex_info, EE_BUF_LEN,
		"\n[%s] err_code1:%d err_code2:%d\n",
		debug_info->name,
		debug_info->fatal_error.err_code1,
		debug_info->fatal_error.err_code2);
		break;
	case MD_EX_TYPE_EMI_CHECK:
		CCCI_NORMAL_LOG(md_id, FSM,
		"md_emi_check: %08X, %08X, %02d, %08X\n",
		debug_info->data.data0, debug_info->data.data1,
		debug_info->data.channel, debug_info->data.reserved);
		scnprintf(ex_info, EE_BUF_LEN,
		"\n[emi_chk] %08X, %08X, %02d, %08X\n",
		debug_info->data.data0, debug_info->data.data1,
		debug_info->data.channel, debug_info->data.reserved);
		break;
	case DSP_EX_TYPE_ASSERT:
		CCCI_NORMAL_LOG(md_id, FSM, "filename = %s\n",
			debug_info->dsp_assert.file_name);
		CCCI_NORMAL_LOG(md_id, FSM, "line = %d\n",
			debug_info->dsp_assert.line_num);
		CCCI_NORMAL_LOG(md_id, FSM, "exec unit = %s\n",
			debug_info->dsp_assert.execution_unit);
		CCCI_NORMAL_LOG(md_id, FSM,
			"para0 = %d, para1 = %d, para2 = %d\n",
			debug_info->dsp_assert.parameters[0],
			debug_info->dsp_assert.parameters[1],
			debug_info->dsp_assert.parameters[2]);
		scnprintf(ex_info, EE_BUF_LEN,
			"\n[%s] file:%s line:%d\nexec:%s\np1:%d\np2:%d\np3:%d\n",
			debug_info->name, debug_info->assert.file_name,
			debug_info->assert.line_num,
			debug_info->dsp_assert.execution_unit,
			debug_info->dsp_assert.parameters[0],
			debug_info->dsp_assert.parameters[1],
			debug_info->dsp_assert.parameters[2]);
		break;
	case DSP_EX_TYPE_EXCEPTION:
		CCCI_NORMAL_LOG(md_id, FSM,
			"exec unit = %s, code1:0x%08x\n",
			debug_info->dsp_exception.execution_unit,
			debug_info->dsp_exception.code1);
		scnprintf(ex_info, EE_BUF_LEN,
			"\n[%s] exec:%s code1:0x%08x\n", debug_info->name,
			debug_info->dsp_exception.execution_unit,
			debug_info->dsp_exception.code1);
		break;
	case DSP_EX_FATAL_ERROR:
		CCCI_NORMAL_LOG(md_id, FSM, "exec unit = %s\n",
			debug_info->dsp_fatal_err.execution_unit);
		CCCI_NORMAL_LOG(md_id, FSM,
			"err_code0 = 0x%08x, err_code1 = 0x%08x\n",
			debug_info->dsp_fatal_err.err_code[0],
			debug_info->dsp_fatal_err.err_code[1]);

		scnprintf(ex_info, EE_BUF_LEN,
			"\n[%s] exec:%s err_code1:0x%08x err_code2:0x%08x\n",
			debug_info->name,
			debug_info->dsp_fatal_err.execution_unit,
			debug_info->dsp_fatal_err.err_code[0],
			debug_info->dsp_fatal_err.err_code[1]);
		break;

	default:		/* Only display exception name */
		scnprintf(ex_info, EE_BUF_LEN, "\n[%s]\n", debug_info->name);
		break;
	}

	/* Add additional info */
	ex_info_temp = kmalloc(EE_BUF_LEN, GFP_ATOMIC);
	if (ex_info_temp == NULL) {
		CCCI_ERROR_LOG(md_id, FSM,
			"Fail alloc Mem for ex_info_temp!\n");
		goto err_exit;
	}
	scnprintf(ex_info_temp, EE_BUF_LEN, "%s", ex_info);
	switch (dumper->more_info) {
	case MD_EE_CASE_ONLY_SWINT:
		scnprintf(ex_info, EE_BUF_LEN, "%s%s", ex_info_temp,
			"\nOnly SWINT case\n");
		break;
	case MD_EE_CASE_ONLY_EX:
		scnprintf(ex_info, EE_BUF_LEN, "%s%s", ex_info_temp,
			"\nOnly EX case\n");
		break;
	case MD_EE_CASE_NO_RESPONSE:
		/* use strcpy, otherwise if this happens after a MD EE,
		 * the former EE info will be printed out
		 */
		if (scnprintf(ex_info, EE_BUF_LEN,
			"\n[Others] MD long time no response\n") < 0)
			ex_info[0] = 0;
		db_opt |= (unsigned int)DB_OPT_FTRACE;
		break;
	case MD_EE_CASE_WDT:
		strncpy(ex_info, "\n[Others] MD watchdog timeout interrupt\n",
			EE_BUF_LEN);
		break;
	default:
		break;
	}

	/* get ELM_status field from MD side */
	scnprintf(ex_info_temp, EE_BUF_LEN, "%s", ex_info);
	c = dumper->ex_info.envinfo.ELM_status;
	CCCI_ERROR_LOG(md_id, FSM, "ELM_status: %x\n", c);
	switch (c) {
	case 0xFF:
		scnprintf(ex_info, EE_BUF_LEN, "%s%s", ex_info_temp,
			"\nno ELM info\n");
		break;
	case 0xAE:
		scnprintf(ex_info, EE_BUF_LEN, "%s%s", ex_info_temp,
			"\nELM rlat:FAIL\n");
		break;
	case 0xBE:
		scnprintf(ex_info, EE_BUF_LEN, "%s%s", ex_info_temp,
			"\nELM wlat:FAIL\n");
		break;
	case 0xDE:
		scnprintf(ex_info, EE_BUF_LEN, "%s%s", ex_info_temp,
			"\nELM r/wlat:PASS\n");
		break;
	default:
		break;
	}
err_exit:
	/* Dump MD EE info */
	CCCI_MEM_LOG_TAG(md_id, FSM, "Dump MD EX log\n");
	if ((md_id == MD_SYS3)
		|| (dumper->more_info == MD_EE_CASE_NORMAL
		&& md_state == BOOT_WAITING_FOR_HS1)) {
		ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP,
			&dumper->ex_info, sizeof(struct ex_log_t));
	} else if (md_dbg_dump_flag & (1 << MD_DBG_DUMP_SMEM)) {
		ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP,
			mdccci_dbg->base_ap_view_vir, mdccci_dbg->size);
		ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP,
			mdss_dbg->base_ap_view_vir, mdss_dbg->size);
	}

	if (dumper->more_info == MD_EE_CASE_NORMAL
		&& md_state == BOOT_WAITING_FOR_HS1) {
		/* MD will not fill in share memory
		 * before we send runtime data
		 */
		dump_flag = CCCI_AED_DUMP_EX_PKT;
	} else {
		/*
		 * otherwise always dump whole share memory,
		 * as MD will fill debug log into
		 * its 2nd 1K region after bootup
		 */
		dump_flag = CCCI_AED_DUMP_EX_MEM;
		if (dumper->more_info == MD_EE_CASE_NO_RESPONSE)
			dump_flag |= CCCI_AED_DUMP_CCIF_REG;
	}
	/* update here to maintain handshake stage
	 * info during exception handling
	 */
	if (debug_info->type == MD_EX_TYPE_C2K_ERROR
		&& debug_info->fatal_error.err_code1 == MD_EX_C2K_FATAL_ERROR)
		CCCI_ERROR_LOG(md_id, FSM, "C2K EE, No need trigger DB\n");
	else if (debug_info->type == CC_MD1_EXCEPTION)
		CCCI_ERROR_LOG(md_id, FSM, "MD1 EE, No need trigger DB\n");
	else if (ex_info == NULL)
		ccci_aed_v1(mdee, dump_flag, buf_fail, db_opt);
	else
		ccci_aed_v1(mdee, dump_flag, ex_info, db_opt);
	kfree(ex_info);
	kfree(ex_info_temp);
	kfree(i_bit_ex_info);
}

/*
 * copy raw data (struct ex_log_t) received from md into struct debug_info_t
 */
static void mdee_dumper_info_prepare_v1(struct ccci_fsm_ee *mdee)
{
	struct ex_log_t *ex_info = NULL;
	int ee_type, ee_case;
	struct mdee_dumper_v1 *dumper = mdee->dumper_obj;
	int md_id = mdee->md_id;
	struct debug_info_t *debug_info = &dumper->debug_info;
	struct ccci_smem_region *mdss_dbg =
		ccci_md_get_smem_by_user_id(mdee->md_id,
			SMEM_USER_RAW_MDSS_DBG);

	if (debug_info == NULL)
		return;

	if ((md_id == MD_SYS3) ||
	(dumper->more_info == MD_EE_CASE_NORMAL
	&& ccci_fsm_get_md_state(mdee->md_id)
	== BOOT_WAITING_FOR_HS1)) {
		ex_info = &dumper->ex_info;
		CCCI_DEBUG_LOG(md_id, FSM,
			"Parse ex info from ccci packages\n");
	} else {
		ex_info = (struct ex_log_t *)mdss_dbg->base_ap_view_vir;
		CCCI_DEBUG_LOG(md_id, FSM,
			"Parse ex info from shared memory\n");
	}
	ee_case = dumper->more_info;

	memset(debug_info, 0, sizeof(struct debug_info_t));/* need review */
	ee_type = ex_info->header.ex_type;
	debug_info->type = ee_type;
	mdee->ex_type = ee_type;

	if (*((char *)ex_info + CCCI_EXREC_OFFSET_OFFENDER) != 0xCC) {
		memcpy(debug_info->fatal_error.offender,
		(char *)ex_info + CCCI_EXREC_OFFSET_OFFENDER,
		sizeof(debug_info->fatal_error.offender) - 1); /*need review */
		debug_info->fatal_error.offender[
			sizeof(debug_info->fatal_error.offender) - 1] = '\0';
	} else {
		debug_info->fatal_error.offender[0] = '\0';
	}

	switch (ee_type) {
	case MD_EX_TYPE_INVALID:
		debug_info->name = "INVALID";
		break;

	case MD_EX_TYPE_UNDEF:
		debug_info->name = "Fatal error (undefine)";
		debug_info->fatal_error.err_code1 =
			ex_info->content.fatalerr.error_code.code1;
		debug_info->fatal_error.err_code2 =
			ex_info->content.fatalerr.error_code.code2;
		break;

	case MD_EX_TYPE_SWI:
		debug_info->name = "Fatal error (swi)";
		debug_info->fatal_error.err_code1 =
			ex_info->content.fatalerr.error_code.code1;
		debug_info->fatal_error.err_code2 =
			ex_info->content.fatalerr.error_code.code2;
		break;

	case MD_EX_TYPE_PREF_ABT:
		debug_info->name = "Fatal error (prefetch abort)";
		debug_info->fatal_error.err_code1 =
			ex_info->content.fatalerr.error_code.code1;
		debug_info->fatal_error.err_code2 =
			ex_info->content.fatalerr.error_code.code2;
		break;

	case MD_EX_TYPE_DATA_ABT:
		debug_info->name = "Fatal error (data abort)";
		debug_info->fatal_error.err_code1 =
			ex_info->content.fatalerr.error_code.code1;
		debug_info->fatal_error.err_code2 =
			ex_info->content.fatalerr.error_code.code2;
		break;

	case MD_EX_TYPE_ASSERT:
		debug_info->name = "ASSERT";
		if (md_id == MD_SYS3) {
			scnprintf(debug_info->assert.file_name,
				sizeof(debug_info->assert.file_name),
				ex_info->content.c2k_assert.filename);
			debug_info->assert.line_num =
				ex_info->content.c2k_assert.linenumber;
			debug_info->assert.parameters[0] =
				ex_info->content.c2k_assert.parameters[0];
			debug_info->assert.parameters[1] =
				ex_info->content.c2k_assert.parameters[1];
			debug_info->assert.parameters[2] =
				ex_info->content.c2k_assert.parameters[2];
		} else {
			scnprintf(debug_info->assert.file_name,
				sizeof(debug_info->assert.file_name),
				ex_info->content.assert.filename);
			debug_info->assert.line_num =
				ex_info->content.assert.linenumber;
			debug_info->assert.parameters[0] =
				ex_info->content.assert.parameters[0];
			debug_info->assert.parameters[1] =
				ex_info->content.assert.parameters[1];
			debug_info->assert.parameters[2] =
				ex_info->content.assert.parameters[2];
		}
		break;

	case MD_EX_TYPE_FATALERR_TASK:
		debug_info->name = "Fatal error (task)";
		debug_info->fatal_error.err_code1 =
			ex_info->content.fatalerr.error_code.code1;
		debug_info->fatal_error.err_code2 =
			ex_info->content.fatalerr.error_code.code2;
		break;
	case MD_EX_TYPE_C2K_ERROR:
		debug_info->name = "Fatal error (C2K_EXP)";
		debug_info->fatal_error.err_code1 =
			ex_info->content.fatalerr.error_code.code1;
		debug_info->fatal_error.err_code2 =
			ex_info->content.fatalerr.error_code.code2;
		break;
	case CC_MD1_EXCEPTION:
		debug_info->name = "Fatal error (LTE_EXP)";
		debug_info->fatal_error.err_code1 =
			ex_info->content.fatalerr.error_code.code1;
		debug_info->fatal_error.err_code2 =
			ex_info->content.fatalerr.error_code.code2;
		break;
	case MD_EX_TYPE_FATALERR_BUF:
		debug_info->name = "Fatal error (buff)";
		debug_info->fatal_error.err_code1 =
			ex_info->content.fatalerr.error_code.code1;
		debug_info->fatal_error.err_code2 =
			ex_info->content.fatalerr.error_code.code2;
		break;

	case MD_EX_TYPE_LOCKUP:
		debug_info->name = "Lockup";
		break;

	case MD_EX_TYPE_ASSERT_DUMP:
		debug_info->name = "ASSERT DUMP";
		if (md_id == MD_SYS3) {
			scnprintf(debug_info->assert.file_name,
				sizeof(debug_info->assert.file_name),
				ex_info->content.c2k_assert.filename);
			debug_info->assert.line_num =
				ex_info->content.c2k_assert.linenumber;
		} else {
			scnprintf(debug_info->assert.file_name,
				sizeof(debug_info->assert.file_name),
				ex_info->content.assert.filename);
			debug_info->assert.line_num =
				ex_info->content.assert.linenumber;
		}
		break;

	case DSP_EX_TYPE_ASSERT:
		debug_info->name = "MD DMD ASSERT";
		if (md_id == MD_SYS3) {
			scnprintf(debug_info->dsp_assert.file_name,
				sizeof(debug_info->dsp_assert.file_name),
				ex_info->content.c2k_assert.filename);
			debug_info->dsp_assert.line_num =
				ex_info->content.c2k_assert.linenumber;
			scnprintf(debug_info->dsp_assert.execution_unit,
				sizeof(debug_info->dsp_assert.execution_unit),
				ex_info->envinfo.execution_unit);
			debug_info->dsp_assert.parameters[0] =
				ex_info->content.c2k_assert.parameters[0];
			debug_info->dsp_assert.parameters[1] =
				ex_info->content.c2k_assert.parameters[1];
			debug_info->dsp_assert.parameters[2] =
				ex_info->content.c2k_assert.parameters[2];

		} else {
			scnprintf(debug_info->dsp_assert.file_name,
				sizeof(debug_info->dsp_assert.file_name),
				ex_info->content.assert.filename);
			debug_info->dsp_assert.line_num =
				ex_info->content.assert.linenumber;
			scnprintf(debug_info->dsp_assert.execution_unit,
				sizeof(debug_info->dsp_assert.execution_unit),
				ex_info->envinfo.execution_unit);
			debug_info->dsp_assert.parameters[0] =
				ex_info->content.assert.parameters[0];
			debug_info->dsp_assert.parameters[1] =
				ex_info->content.assert.parameters[1];
			debug_info->dsp_assert.parameters[2] =
				ex_info->content.assert.parameters[2];
		}
		break;

	case DSP_EX_TYPE_EXCEPTION:
		debug_info->name = "MD DMD Exception";
		scnprintf(debug_info->dsp_exception.execution_unit,
			sizeof(debug_info->dsp_exception.execution_unit),
			ex_info->envinfo.execution_unit);
		debug_info->dsp_exception.code1 =
			ex_info->content.fatalerr.error_code.code1;
		break;

	case DSP_EX_FATAL_ERROR:
		debug_info->name = "MD DMD FATAL ERROR";
		scnprintf(debug_info->dsp_fatal_err.execution_unit,
			sizeof(debug_info->dsp_fatal_err.execution_unit),
			ex_info->envinfo.execution_unit);
		debug_info->dsp_fatal_err.err_code[0] =
			ex_info->content.fatalerr.error_code.code1;
		debug_info->dsp_fatal_err.err_code[1] =
			ex_info->content.fatalerr.error_code.code2;
		break;

	default:
		debug_info->name = "UNKNOWN Exception";
		break;
	}

	debug_info->ext_mem = ex_info;
	debug_info->ext_size = sizeof(struct ex_log_t);
}
static void mdee_dumper_v1_set_ee_pkg(struct ccci_fsm_ee *mdee,
	char *data, int len)
{
	struct mdee_dumper_v1 *dumper = mdee->dumper_obj;
	int cpy_len =
		len > sizeof(struct ex_log_t) ? sizeof(struct ex_log_t) : len;

	memcpy(&dumper->ex_info, data, cpy_len);
}

static void mdee_dumper_v1_dump_ee_info(struct ccci_fsm_ee *mdee,
	enum MDEE_DUMP_LEVEL level, int more_info)
{
	struct mdee_dumper_v1 *dumper = mdee->dumper_obj;
	int md_id = mdee->md_id;
	struct ccci_smem_region *mdccci_dbg =
		ccci_md_get_smem_by_user_id(mdee->md_id,
			SMEM_USER_RAW_MDCCCI_DBG);
	struct ccci_smem_region *mdss_dbg =
		ccci_md_get_smem_by_user_id(mdee->md_id,
			SMEM_USER_RAW_MDSS_DBG);
	char ex_info[EE_BUF_LEN] = {0};
	int md_state = ccci_fsm_get_md_state(mdee->md_id);
	struct ccci_per_md *per_md_data = ccci_get_per_md_data(mdee->md_id);
	int md_dbg_dump_flag = per_md_data->md_dbg_dump_flag;

	dumper->more_info = more_info;
	if (level == MDEE_DUMP_LEVEL_BOOT_FAIL) {
		if (md_state == BOOT_WAITING_FOR_HS1) {
			scnprintf(ex_info, EE_BUF_LEN,
				"\n[Others] MD_BOOT_UP_FAIL(HS%d)\n", 1);
			/* Handshake 1 fail */
			ccci_aed_v1(mdee,
			CCCI_AED_DUMP_CCIF_REG,
			ex_info, DB_OPT_DEFAULT);
		} else if (md_state == BOOT_WAITING_FOR_HS2) {
			scnprintf(ex_info, EE_BUF_LEN,
				"\n[Others] MD_BOOT_UP_FAIL(HS%d)\n", 2);
			/* Handshake 2 fail */
			CCCI_MEM_LOG_TAG(md_id, FSM, "Dump MD EX log\n");
			if (md_dbg_dump_flag & (1U << MD_DBG_DUMP_SMEM)) {
				ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP,
					mdccci_dbg->base_ap_view_vir,
					mdccci_dbg->size);
				ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP,
					mdss_dbg->base_ap_view_vir,
					mdss_dbg->size);
			}

			ccci_aed_v1(mdee,
			CCCI_AED_DUMP_CCIF_REG | CCCI_AED_DUMP_EX_MEM,
			ex_info, DB_OPT_FTRACE);
		}
	} else if (level == MDEE_DUMP_LEVEL_STAGE1) {
		if (md_dbg_dump_flag & (1 << MD_DBG_DUMP_SMEM)) {
			CCCI_MEM_LOG_TAG(md_id, FSM, "Dump MD exp smem_log\n");
			ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP,
				mdccci_dbg->base_ap_view_vir, mdccci_dbg->size);
			ccci_util_mem_dump(md_id, CCCI_DUMP_MEM_DUMP,
				mdss_dbg->base_ap_view_vir, mdss_dbg->size);
		}
	} else if (level == MDEE_DUMP_LEVEL_STAGE2) {
		mdee_dumper_info_prepare_v1(mdee);
		mdee_dumper_info_dump_v1(mdee);
	} else {

	}
}

static struct md_ee_ops mdee_ops_v1 = {
	.dump_ee_info = &mdee_dumper_v1_dump_ee_info,
	.set_ee_pkg = &mdee_dumper_v1_set_ee_pkg,
};
int mdee_dumper_v1_alloc(struct ccci_fsm_ee *mdee)
{
	struct mdee_dumper_v1 *dumper;
	int md_id = mdee->md_id;

	/* Allocate port_proxy obj and set all member zero */
	dumper = kzalloc(sizeof(struct mdee_dumper_v1), GFP_KERNEL);
	if (dumper == NULL) {
		CCCI_ERROR_LOG(md_id, FSM,
			"%s:alloc mdee_parser_v1 fail\n", __func__);
		return -1;
	}
	mdee->dumper_obj = dumper;
	mdee->ops = &mdee_ops_v1;
	return 0;
}

