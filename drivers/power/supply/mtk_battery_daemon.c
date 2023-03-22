// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author Wy Chuang<wy.chuang@mediatek.com>
 */

#include <linux/netlink.h>	/* netlink */
#ifdef CONFIG_OF
#include <linux/of_fdt.h>	/*of_dt API*/
#endif
#include <linux/platform_device.h>
#include <linux/reboot.h>	/*kernel_power_off*/
#include <linux/skbuff.h>	/* netlink */
#include <linux/socket.h>	/* netlink */
#include <net/sock.h>		/* netlink */
#include "mtk_battery_daemon.h"
#include "mtk_battery.h"

#if IS_ENABLED(CONFIG_PMIC_LBAT_SERVICE)
#include <pmic_lbat_service.h>
#endif



static int interpolation(int i1, int b1, int i2, int b2, int i)
{
	int ret;

	ret = (b2 - b1) * (i - i1) / (i2 - i1) + b1;
	return ret;
}

/* ============================================================ */
/* Customized function */
/* ============================================================ */
int get_customized_aging_factor(int orig_af)
{
	return (orig_af + 100);
}

int get_customized_d0_c_soc(int origin_d0_c_soc)
{
	int val;

	val = (origin_d0_c_soc + 0);
	return val;
}

int get_customized_d0_v_soc(int origin_d0_v_soc)
{
	int val;

	val = (origin_d0_v_soc + 0);
	return val;
}

int get_customized_uisoc(int origin_uisoc)
{
	int val;

	val = (origin_uisoc + 0);

	return val;
}

int fg_get_system_sec(void)
{
	ktime_t ctime = 0;
	struct timespec64 tmp_time;

	ctime = ktime_get_boottime();
	tmp_time = ktime_to_timespec64(ctime);

	return (int)tmp_time.tv_sec;
}

int fg_get_imix(struct mtk_battery *gm)
{
	int imix = 1;

	if (gm->imix != 0)
		return gm->imix;

	return imix;
}

int gauge_set_nag_en(struct mtk_battery *gm, int nafg_zcv_en)
{
	if (gm->disableGM30)
		return 0;

	if (gm->disable_nafg_int == false)
		gauge_set_property(GAUGE_PROP_NAFG_EN, nafg_zcv_en);

	bm_debug(
		"%s = %d\n",
		__func__,
		nafg_zcv_en);

	return 0;
}

int gauge_get_average_current(struct mtk_battery *gm, bool *valid)
{
	int iavg = 0;
	int ver;

	ver = gauge_get_int_property(GAUGE_PROP_HW_VERSION);
	if (gm->disableGM30)
		iavg = 0;
	else {
		if (ver >= GAUGE_HW_V1000 &&
			ver < GAUGE_HW_V2000)
			iavg = gm->sw_iavg;
		else {
			*valid = gm->gauge->fg_hw_info.current_avg_valid;
			iavg =
			gauge_get_int_property(GAUGE_PROP_AVERAGE_CURRENT);
		}
	}

	return iavg;
}

void mtk_battery_send_to_user(struct mtk_battery *gm,
	int seq, struct fgd_nl_msg_t *reply_msg)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	u32 pid = 0;
	int size = reply_msg->fgd_data_len + FGD_NL_HDR_LEN;
	int len = NLMSG_SPACE(size);
	void *data;
	int ret = -1;

	if (gm == NULL)
		gm = get_mtk_battery();

	if (gm != NULL)
		pid = gm->fgd_pid;

	reply_msg->identity = FGD_NL_MAGIC;

	if (in_interrupt())
		skb = alloc_skb(len, GFP_ATOMIC);
	else
		skb = alloc_skb(len, GFP_KERNEL);

	if (!skb)
		return;

	nlh = nlmsg_put(skb, pid, seq, 0, size, 0);
	data = NLMSG_DATA(nlh);
	memcpy(data, reply_msg, size);
	NETLINK_CB(skb).portid = 0;	/* from kernel */
	NETLINK_CB(skb).dst_group = 0;	/* unicast */

	if (gm->mtk_battery_sk != NULL)
		ret = netlink_unicast
			(gm->mtk_battery_sk, skb, pid, MSG_DONTWAIT);

	if (ret < 0) {
		bm_err("[%s]send failed ret=%d pid=%d\n", __func__, ret, pid);
		return;
	}
}

void fg_cmd_check(struct fgd_nl_msg_t *msg)
{
	while (msg->fgd_subcmd == 0 &&
		msg->fgd_subcmd_para1 != FGD_NL_HDR_LEN) {
		bm_err("fuel gauge version error cmd:%d %d\n",
			msg->fgd_cmd,
			msg->fgd_subcmd);
		msleep(10000);
		break;
	}
}

void fg_daemon_send_data(struct mtk_battery *gm,
	int cmd, char *rcv, char *ret)
{
	struct fgd_cmd_param_t_4 *prcv;
	struct fgd_cmd_param_t_4 *pret;

	prcv = (struct fgd_cmd_param_t_4 *)rcv;
	pret = (struct fgd_cmd_param_t_4 *)ret;

	bm_debug("%s type:%d, tsize:%d size:%d idx:%d\n",
		__func__,
		cmd,
		prcv->total_size,
		prcv->size,
		prcv->idx);

		pret->total_size = prcv->total_size;
		pret->size = prcv->size;
		pret->idx = prcv->idx;


	switch (cmd) {
	case FG_DAEMON_CMD_SEND_CUSTOM_TABLE:
		{
			char *ptr;

			if (sizeof(struct fgd_cmd_param_t_custom)
				!= prcv->total_size) {
				bm_err("%s size is different %d %d\n",
				__func__,
				(int)sizeof(
				struct fgd_cmd_param_t_custom),
				prcv->total_size);
			}

			ptr = (char *)&gm->fg_data;
			memcpy(&ptr[prcv->idx],
				prcv->input,
				prcv->size);

			bm_debug(
				"FG_DATA_TYPE_TABLE type:%d size:%d %d idx:%d\n",
				FG_DAEMON_CMD_SEND_CUSTOM_TABLE,
				prcv->total_size,
				prcv->size,
				prcv->idx);
		}
		break;
	case FG_DAEMON_CMD_SEND_VERSION_CONTROL:
		{
			char *ptr;

			if (sizeof(struct VersionControl)
				!= prcv->total_size) {
				bm_err("%s vc size is different %d %d\n",
				__func__,
				(int)sizeof(
				struct VersionControl),
				prcv->total_size);
			}

			ptr = (char *)&gm->fg_version;
			memcpy(&ptr[prcv->idx],
				prcv->input,
				prcv->size);

			bm_debug(
				"VersionControl type:%d size:%d %d idx:%d\n",
				FG_DAEMON_CMD_SEND_VERSION_CONTROL,
				prcv->total_size,
				prcv->size,
				prcv->idx);
			bm_debug(
				"VersionControl %d %d %d %d %d\n",
				gm->fg_version.androidVersion,
				gm->fg_version.daemon_cmds,
				gm->fg_version.kernel_cmds,
				gm->fg_version.custom_data_len,
				gm->fg_version.custom_table_len);

			if (gm->fg_version.daemon_cmds != FG_DAEMON_CMD_FROM_USER_NUMBER ||
				gm->fg_version.kernel_cmds != FG_KERNEL_CMD_FROM_USER_NUMBER ||
				gm->fg_version.custom_data_len !=
					sizeof(struct fuel_gauge_custom_data) ||
				gm->fg_version.custom_table_len !=
					sizeof(struct fuel_gauge_table_custom_data)) {

				gm->algo.active = true;
				battery_algo_init(gm);
				bm_err("[%s]: %d %d,%d %d,%d %d,%d %d, enable Kernel mode Gauge\n",
					__func__,
					gm->fg_version.daemon_cmds, FG_DAEMON_CMD_FROM_USER_NUMBER,
					gm->fg_version.kernel_cmds, FG_KERNEL_CMD_FROM_USER_NUMBER,
					gm->fg_version.custom_data_len,
					sizeof(struct fuel_gauge_custom_data),
					gm->fg_version.custom_table_len,
					sizeof(struct fuel_gauge_table_custom_data));
			}
		}
		break;
	default:
		bm_err("%s bad cmd 0x%x\n",
			__func__, cmd);
		break;
	}
}

void fg_daemon_get_data(int cmd,
	char *rcv,
	char *ret)
{
	struct fgd_cmd_param_t_4 *prcv;
	struct fgd_cmd_param_t_4 *pret;
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	prcv = (struct fgd_cmd_param_t_4 *)rcv;
	pret = (struct fgd_cmd_param_t_4 *)ret;

	bm_debug("%s type:%d, tsize:%d size:%d idx:%d\n",
		__func__,
		cmd,
		prcv->total_size,
		prcv->size,
		prcv->idx);

		pret->total_size = prcv->total_size;
		pret->size = prcv->size;
		pret->idx = prcv->idx;


	switch (cmd) {
	case FG_DAEMON_CMD_GET_CUSTOM_SETTING:
	{
		char *ptr;

		if (sizeof(struct fuel_gauge_custom_data)
			!= prcv->total_size) {
			bm_err("%s size is different %d %d\n",
			__func__,
			(int)sizeof(
			struct fuel_gauge_custom_data),
			prcv->total_size);
		}

		ptr = (char *)&gm->fg_cust_data;
		memcpy(pret->input, &ptr[prcv->idx], pret->size);
		bm_trace(
			"FG_DATA_TYPE_TABLE type:%d size:%d %d idx:%d data:%d %d %d %d\n",
			FG_DAEMON_CMD_GET_CUSTOM_SETTING,
			pret->total_size,
			pret->size,
			pret->idx,
			pret->input[0],
			pret->input[1],
			pret->input[2],
			pret->input[3]);

	}
	break;
	case FG_DAEMON_CMD_GET_CUSTOM_TABLE:
		{
			char *ptr;

			if (sizeof(struct fuel_gauge_table_custom_data)
				!= prcv->total_size) {
				bm_err("%s size is different %d %d\n",
				__func__,
				(int)sizeof(
				struct fuel_gauge_table_custom_data),
				prcv->total_size);
			}

			ptr = (char *)&gm->fg_table_cust_data;
			memcpy(pret->input, &ptr[prcv->idx],
				pret->size);
			bm_trace(
				"FG_DATA_TYPE_TABLE type:%d size:%d %d idx:%d\n",
				FG_DAEMON_CMD_GET_CUSTOM_TABLE,
				prcv->total_size,
				prcv->size,
				prcv->idx);
		}
		break;
	default:
		bm_err("%s bad cmd:0x%x\n",
			__func__, cmd);
		break;

	}

}

void fg_ocv_query_soc(struct mtk_battery *gm, int ocv)
{
	if (ocv > 50000 || ocv <= 0)
		return;

	gm->algo_req_ocv = ocv;
	wakeup_fg_algo_cmd(gm,
		FG_INTR_KERNEL_CMD, FG_KERNEL_CMD_REQ_ALGO_DATA,
		ocv);

	bm_trace("[%s] ocv:%d\n",
		__func__, ocv);
}

void fg_custom_data_check(struct mtk_battery *gm)
{
	struct fuel_gauge_custom_data *p;
	struct fuel_gauge_table_custom_data *fg_table_cust_data;

	p = &gm->fg_cust_data;
	fg_table_cust_data = &gm->fg_table_cust_data;
	fgauge_get_profile_id();

	bm_err("FGLOG MultiGauge0[%d] BATID[%d] pmic_min_vol[%d,%d,%d,%d,%d]\n",
		p->multi_temp_gauge0, gm->battery_id,
		fg_table_cust_data->fg_profile[0].pmic_min_vol,
		fg_table_cust_data->fg_profile[1].pmic_min_vol,
		fg_table_cust_data->fg_profile[2].pmic_min_vol,
		fg_table_cust_data->fg_profile[3].pmic_min_vol,
		fg_table_cust_data->fg_profile[4].pmic_min_vol);
	bm_err("FGLOG pon_iboot[%d,%d,%d,%d,%d] qmax_sys_vol[%d %d %d %d %d]\n",
		fg_table_cust_data->fg_profile[0].pon_iboot,
		fg_table_cust_data->fg_profile[1].pon_iboot,
		fg_table_cust_data->fg_profile[2].pon_iboot,
		fg_table_cust_data->fg_profile[3].pon_iboot,
		fg_table_cust_data->fg_profile[4].pon_iboot,
		fg_table_cust_data->fg_profile[0].qmax_sys_vol,
		fg_table_cust_data->fg_profile[1].qmax_sys_vol,
		fg_table_cust_data->fg_profile[2].qmax_sys_vol,
		fg_table_cust_data->fg_profile[3].qmax_sys_vol,
		fg_table_cust_data->fg_profile[4].qmax_sys_vol);

}

void Intr_Number_to_Name(char *intr_name, unsigned int intr_no)
{
	switch (intr_no) {
	case FG_INTR_0:
		sprintf(intr_name, "FG_INTR_INIT");
		break;

	case FG_INTR_TIMER_UPDATE:
		sprintf(intr_name, "FG_INTR_TIMER_UPDATE");
		break;

	case FG_INTR_BAT_CYCLE:
		sprintf(intr_name, "FG_INTR_BAT_CYCLE");
		break;

	case FG_INTR_CHARGER_OUT:
		sprintf(intr_name, "FG_INTR_CHARGER_OUT");
		break;

	case FG_INTR_CHARGER_IN:
		sprintf(intr_name, "FG_INTR_CHARGER_IN");
		break;

	case FG_INTR_FG_TIME:
		sprintf(intr_name, "FG_INTR_FG_TIME");
		break;

	case FG_INTR_BAT_INT1_HT:
		sprintf(intr_name, "FG_INTR_COULOMB_HT");
		break;

	case FG_INTR_BAT_INT1_LT:
		sprintf(intr_name, "FG_INTR_COULOMB_LT");
		break;

	case FG_INTR_BAT_INT2_HT:
		sprintf(intr_name, "FG_INTR_UISOC_HT");
		break;

	case FG_INTR_BAT_INT2_LT:
		sprintf(intr_name, "FG_INTR_UISOC_LT");
		break;

	case FG_INTR_BAT_TMP_HT:
		sprintf(intr_name, "FG_INTR_BAT_TEMP_HT");
		break;

	case FG_INTR_BAT_TMP_LT:
		sprintf(intr_name, "FG_INTR_BAT_TEMP_LT");
		break;

	case FG_INTR_BAT_TIME_INT:
		sprintf(intr_name, "FG_INTR_BAT_TIME_INT");
		break;

	case FG_INTR_NAG_C_DLTV:
		sprintf(intr_name, "FG_INTR_NAFG_VOLTAGE");
		break;

	case FG_INTR_FG_ZCV:
		sprintf(intr_name, "FG_INTR_FG_ZCV");
		break;

	case FG_INTR_SHUTDOWN:
		sprintf(intr_name, "FG_INTR_SHUTDOWN");
		break;

	case FG_INTR_RESET_NVRAM:
		sprintf(intr_name, "FG_INTR_RESET_NVRAM");
		break;

	case FG_INTR_BAT_PLUGOUT:
		sprintf(intr_name, "FG_INTR_BAT_PLUGOUT");
		break;

	case FG_INTR_IAVG:
		sprintf(intr_name, "FG_INTR_IAVG");
		break;

	case FG_INTR_VBAT2_L:
		sprintf(intr_name, "FG_INTR_VBAT2_L");
		break;

	case FG_INTR_VBAT2_H:
		sprintf(intr_name, "FG_INTR_VBAT2_H");
		break;

	case FG_INTR_CHR_FULL:
		sprintf(intr_name, "FG_INTR_CHR_FULL");
		break;

	case FG_INTR_DLPT_SD:
		sprintf(intr_name, "FG_INTR_DLPT_SD");
		break;

	case FG_INTR_BAT_TMP_C_HT:
		sprintf(intr_name, "FG_INTR_BAT_TMP_C_HT");
		break;

	case FG_INTR_BAT_TMP_C_LT:
		sprintf(intr_name, "FG_INTR_BAT_TMP_C_LT");
		break;

	case FG_INTR_BAT_INT1_CHECK:
		sprintf(intr_name, "FG_INTR_COULOMB_C");
		break;

	case FG_INTR_KERNEL_CMD:
		sprintf(intr_name, "FG_INTR_KERNEL_CMD");
		break;

	case FG_INTR_BAT_INT2_CHECK:
		sprintf(intr_name, "FG_INTR_BAT_INT2_CHECK");
		break;

	default:
		sprintf(intr_name, "FG_INTR_UNKNOWN");
		bm_err("[%s] unknown intr %d\n", __func__, intr_no);
		break;
	}
}

void exec_BAT_EC(int cmd, int param)
{
	int i;
	struct mtk_battery *gm;
	struct BAT_EC_Struct *ec;
	struct fuel_gauge_custom_data *fg_cust_data;
	struct fuel_gauge_table_custom_data *fg_table_cust_data;


	gm = get_mtk_battery();
	ec = &gm->Bat_EC_ctrl;
	fg_cust_data = &gm->fg_cust_data;
	fg_table_cust_data = &gm->fg_table_cust_data;

	bm_err("exe_BAT_EC cmd %d, param %d\n", cmd, param);
	switch (cmd) {
	case 101:
		{
			/* Force Temperature, force_get_tbat*/
			if (param == 0xff) {
				ec->fixed_temp_en = 0;
				ec->fixed_temp_value = 25;
				bm_err("exe_BAT_EC cmd %d, param %d, disable\n",
					cmd, param);

			} else {
				ec->fixed_temp_en = 1;
				if (param >= 100)
					ec->fixed_temp_value =
						0 - (param - 100);
				else
					ec->fixed_temp_value = param;
				bm_err("exe_BAT_EC cmd %d, param %d, enable\n",
					cmd, param);
			}
		}
		break;
	case 105:
		{
			/* force interrupt trigger */
			switch (param) {
			case 1:
				{
					wakeup_fg_algo(gm,
						FG_INTR_TIMER_UPDATE);
				}
				break;
			case 4096:
				{
					wakeup_fg_algo(gm, FG_INTR_NAG_C_DLTV);
				}
				break;
			case 8192:
				{
					wakeup_fg_algo(gm, FG_INTR_FG_ZCV);
				}
				break;
			case 32768:
				{
					wakeup_fg_algo(gm, FG_INTR_RESET_NVRAM);
				}
				break;
			case 65536:
				{
					wakeup_fg_algo(gm, FG_INTR_BAT_PLUGOUT);
				}
				break;


			default:
				{

				}
				break;
			}
			bm_err(
				"exe_BAT_EC cmd %d, param %d, force interrupt\n",
				cmd, param);
		}
		break;
	case 108:
		{
			/* Set D0_C_CUST */
			if (param == 0xff) {
				ec->debug_d0_c_en = 0;
				ec->debug_d0_c_value = 0;
				bm_err(
					"exe_BAT_EC cmd %d, param %d, disable\n",
					cmd, param);

			} else {
				ec->debug_d0_c_en = 1;
				ec->debug_d0_c_value = param;
				bm_err(
					"exe_BAT_EC cmd %d, param %d, enable\n",
					cmd, param);
			}
		}
		break;
	case 109:
		{
			/* Set D0_V_CUST */
			if (param == 0xff) {
				ec->debug_d0_v_en = 0;
				ec->debug_d0_v_value = 0;
				bm_err(
					"exe_BAT_EC cmd %d, param %d, disable\n",
					cmd, param);

			} else {
				ec->debug_d0_v_en = 1;
				ec->debug_d0_v_value = param;
				bm_err(
					"exe_BAT_EC cmd %d, param %d, enable\n",
					cmd, param);
			}
		}
		break;
	case 110:
		{
			/* Set UISOC_CUST */
			if (param == 0xff) {
				ec->debug_uisoc_en = 0;
				ec->debug_uisoc_value = 0;
				bm_err(
					"exe_BAT_EC cmd %d, param %d, disable\n",
					cmd, param);

			} else {
				ec->debug_uisoc_en = 1;
				ec->debug_uisoc_value = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, enable\n",
				cmd, param);
			}
		}
		break;
	case 600:
		{
			fg_cust_data->aging_diff_max_threshold = param;
			bm_err(
				"exe_BAT_EC cmd %d, aging_diff_max_threshold:%d\n",
				cmd, param);
		}
		break;
	case 601:
		{
			fg_cust_data->aging_diff_max_level = param;
			bm_err(
				"exe_BAT_EC cmd %d, aging_diff_max_level:%d\n",
				cmd, param);
		}
		break;
	case 602:
		{
			fg_cust_data->aging_factor_t_min = param;
			bm_err(
				"exe_BAT_EC cmd %d, aging_factor_t_min:%d\n",
				cmd, param);
		}
		break;
	case 603:
		{
			fg_cust_data->cycle_diff = param;
			bm_err(
				"exe_BAT_EC cmd %d, cycle_diff:%d\n",
				cmd, param);
		}
		break;
	case 604:
		{
			fg_cust_data->aging_count_min = param;
			bm_err(
				"exe_BAT_EC cmd %d, aging_count_min:%d\n",
				cmd, param);
		}
		break;
	case 605:
		{
			fg_cust_data->default_score = param;
			bm_err(
				"exe_BAT_EC cmd %d, default_score:%d\n",
				cmd, param);
		}
		break;
	case 606:
		{
			fg_cust_data->default_score_quantity = param;
			bm_err(
				"exe_BAT_EC cmd %d, default_score_quantity:%d\n",
				cmd, param);
		}
		break;
	case 607:
		{
			fg_cust_data->fast_cycle_set = param;
			bm_err(
				"exe_BAT_EC cmd %d, fast_cycle_set:%d\n",
				cmd, param);
		}
		break;
	case 608:
		{
			fg_cust_data->level_max_change_bat = param;
			bm_err(
				"exe_BAT_EC cmd %d, level_max_change_bat:%d\n",
				cmd, param);
		}
		break;
	case 609:
		{
			fg_cust_data->diff_max_change_bat = param;
			bm_err(
				"exe_BAT_EC cmd %d, diff_max_change_bat:%d\n",
				cmd, param);
		}
		break;
	case 610:
		{
			fg_cust_data->aging_tracking_start = param;
			bm_err(
				"exe_BAT_EC cmd %d, aging_tracking_start:%d\n",
				cmd, param);
		}
		break;
	case 611:
		{
			fg_cust_data->max_aging_data = param;
			bm_err(
				"exe_BAT_EC cmd %d, max_aging_data:%d\n",
				cmd, param);
		}
		break;
	case 612:
		{
			fg_cust_data->max_fast_data = param;
			bm_err(
				"exe_BAT_EC cmd %d, max_fast_data:%d\n",
				cmd, param);
		}
		break;
	case 613:
		{
			fg_cust_data->fast_data_threshold_score = param;
			bm_err(
				"exe_BAT_EC cmd %d, fast_data_threshold_score:%d\n",
				cmd, param);
		}
		break;
	case 614:
		{
			bm_err(
				"exe_BAT_EC cmd %d,FG_KERNEL_CMD_AG_LOG_TEST=%d\n",
				cmd, param);
			/* timo todo */
			/* fg_test_ag_cmd(99); */

		}
		break;
	case 701:
		{
			fg_cust_data->pseudo1_en = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, pseudo1_en\n",
				cmd, param);
		}
		break;
	case 702:
		{
			fg_cust_data->pseudo100_en = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, pseudo100_en\n",
				cmd, param);
		}
		break;
	case 703:
		{
			fg_cust_data->qmax_sel = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, qmax_sel\n",
				cmd, param);
		}
		break;
	case 704:
		{
			fg_cust_data->iboot_sel = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, iboot_sel\n",
				cmd, param);
		}
		break;
	case 705:
		{
			for (i = 0;
				i < fg_table_cust_data->active_table_number;
				i++) {
				fg_table_cust_data->fg_profile[i].pmic_min_vol =
					param * UNIT_TRANS_10;
			}
			bm_err(
				"exe_BAT_EC cmd %d, param %d, pmic_min_vol\n",
				cmd, param);
		}
		break;
	case 706:
		{
			for (i = 0;
				i < fg_table_cust_data->active_table_number;
				i++) {
				fg_table_cust_data->fg_profile[i].pon_iboot =
				param * UNIT_TRANS_10;
			}
			bm_err("exe_BAT_EC cmd %d, param %d, poweron_system_iboot\n"
				, cmd, param * UNIT_TRANS_10);
		}
		break;
	case 707:
		{
			fg_cust_data->shutdown_system_iboot = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, shutdown_system_iboot\n",
				cmd, param);
		}
		break;
	case 708:
		{
			fg_cust_data->com_fg_meter_resistance = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, com_fg_meter_resistance\n",
				cmd, param);
		}
		break;
	case 709:
		{
			fg_cust_data->r_fg_value = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, r_fg_value\n",
				cmd, param);
		}
		break;
	case 710:
		{
			fg_cust_data->q_max_sys_voltage = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, q_max_sys_voltage\n",
				cmd, param);
		}
		break;
	case 711:
		{
			fg_cust_data->loading_1_en = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, loading_1_en\n",
				cmd, param);
		}
		break;
	case 712:
		{
			fg_cust_data->loading_2_en = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, loading_2_en\n",
				cmd, param);
		}
		break;
	case 713:
		{
			fg_cust_data->aging_temp_diff = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, aging_temp_diff\n",
				cmd, param);
		}
		break;
	case 714:
		{
			fg_cust_data->aging1_load_soc = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, aging1_load_soc\n",
				cmd, param);
		}
		break;
	case 715:
		{
			fg_cust_data->aging1_update_soc = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, aging1_update_soc\n",
				cmd, param);
		}
		break;
	case 716:
		{
			fg_cust_data->aging_100_en = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, aging_100_en\n",
				cmd, param);
		}
		break;
	case 717:
		{
			fg_table_cust_data->active_table_number = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, additional_battery_table_en\n",
				cmd, param);
		}
		break;
	case 718:
		{
			fg_cust_data->d0_sel = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, d0_sel\n",
				cmd, param);
		}
		break;
	case 719:
		{
			fg_cust_data->zcv_car_gap_percentage = param;
			bm_err(
				"exe_BAT_EC cmd %d, param %d, zcv_car_gap_percentage\n",
				cmd, param);
		}
		break;
	case 720:
		{
			fg_cust_data->multi_temp_gauge0 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->multi_temp_gauge0=%d\n",
				cmd, param);
		}
		break;
	case 721:
		{
			fg_custom_data_check(gm);
			bm_err("exe_BAT_EC cmd %d", cmd);
		}
		break;
	case 724:
		{
			fg_table_cust_data->fg_profile[0].pseudo100 =
				UNIT_TRANS_100 * param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pseudo100_t0=%d\n",
				cmd, param);
		}
		break;
	case 725:
		{
			fg_table_cust_data->fg_profile[1].pseudo100 =
				UNIT_TRANS_100 * param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pseudo100_t1=%d\n",
				cmd, param);
		}
		break;
	case 726:
		{
			fg_table_cust_data->fg_profile[2].pseudo100 =
				UNIT_TRANS_100 * param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pseudo100_t2=%d\n",
				cmd, param);
		}
		break;
	case 727:
		{
			fg_table_cust_data->fg_profile[3].pseudo100 =
				UNIT_TRANS_100 * param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pseudo100_t3=%d\n",
				cmd, param);
		}
		break;
	case 728:
		{
			fg_table_cust_data->fg_profile[4].pseudo100 =
				UNIT_TRANS_100 * param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pseudo100_t4=%d\n",
				cmd, param);
		}
		break;
	case 729:
		{
			fg_cust_data->keep_100_percent = UNIT_TRANS_100 * param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->keep_100_percent=%d\n",
				cmd, param);
		}
		break;
	case 730:
		{
			fg_cust_data->ui_full_limit_en = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_en=%d\n",
				cmd, param);
		}
		break;
	case 731:
		{
			fg_cust_data->ui_full_limit_soc0 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_soc0=%d\n",
				cmd, param);
		}
		break;
	case 732:
		{
			fg_cust_data->ui_full_limit_ith0 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_ith0=%d\n",
				cmd, param);
		}
		break;
	case 733:
		{
			fg_cust_data->ui_full_limit_soc1 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_soc1=%d\n",
				cmd, param);
		}
		break;
	case 734:
		{
			fg_cust_data->ui_full_limit_ith1 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_ith1=%d\n",
				cmd, param);
		}
		break;
	case 735:
		{
			fg_cust_data->ui_full_limit_soc2 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_soc2=%d\n",
				cmd, param);
		}
		break;
	case 736:
		{
			fg_cust_data->ui_full_limit_ith2 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_ith2=%d\n",
				cmd, param);
		}
		break;
	case 737:
		{
			fg_cust_data->ui_full_limit_soc3 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_soc3=%d\n",
				cmd, param);
		}
		break;
	case 738:
		{
			fg_cust_data->ui_full_limit_ith3 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_ith3=%d\n",
				cmd, param);
		}
		break;
	case 739:
		{
			fg_cust_data->ui_full_limit_soc4 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_soc4=%d\n",
				cmd, param);
		}
		break;
	case 740:
		{
			fg_cust_data->ui_full_limit_ith4 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_ith4=%d\n",
				cmd, param);
		}
		break;
	case 741:
		{
			fg_cust_data->ui_full_limit_time = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->ui_full_limit_time=%d\n",
				cmd, param);
		}
		break;
	case 743:
		{
			fg_table_cust_data->fg_profile[0].pmic_min_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pmic_min_vol_t0=%d\n",
				cmd, param);
		}
		break;
	case 744:
		{
			fg_table_cust_data->fg_profile[1].pmic_min_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pmic_min_vol_t1=%d\n",
				cmd, param);
		}
		break;
	case 745:
		{
			fg_table_cust_data->fg_profile[2].pmic_min_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pmic_min_vol_t2=%d\n",
				cmd, param);
		}
		break;
	case 746:
		{
			fg_table_cust_data->fg_profile[3].pmic_min_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pmic_min_vol_t3=%d\n",
				cmd, param);
		}
		break;
	case 747:
		{
			fg_table_cust_data->fg_profile[4].pmic_min_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pmic_min_vol_t4=%d\n",
				cmd, param);
		}
		break;
	case 748:
		{
			fg_table_cust_data->fg_profile[0].pon_iboot = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pon_iboot_t0=%d\n",
				cmd, param);
		}
		break;
	case 749:
		{
			fg_table_cust_data->fg_profile[1].pon_iboot = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pon_iboot_t1=%d\n",
				cmd, param);
		}
		break;
	case 750:
		{
			fg_table_cust_data->fg_profile[2].pon_iboot = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pon_iboot_t2=%d\n",
				cmd, param);
		}
		break;
	case 751:
		{
			fg_table_cust_data->fg_profile[3].pon_iboot = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pon_iboot_t3=%d\n",
				cmd, param);
		}
		break;
	case 752:
		{
			fg_table_cust_data->fg_profile[4].pon_iboot = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->pon_iboot_t4=%d\n",
				cmd, param);
		}
		break;
	case 753:
		{
			fg_table_cust_data->fg_profile[0].qmax_sys_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->qmax_sys_vol_t0=%d\n",
				cmd, param);
		}
		break;
	case 754:
		{
			fg_table_cust_data->fg_profile[1].qmax_sys_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->qmax_sys_vol_t1=%d\n",
				cmd, param);
		}
		break;
	case 755:
		{
			fg_table_cust_data->fg_profile[2].qmax_sys_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->qmax_sys_vol_t2=%d\n",
				cmd, param);
		}
		break;
	case 756:
		{
			fg_table_cust_data->fg_profile[3].qmax_sys_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->qmax_sys_vol_t3=%d\n",
				cmd, param);
		}
		break;
	case 757:
		{
			fg_table_cust_data->fg_profile[4].qmax_sys_vol = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->qmax_sys_vol_t4=%d\n",
				cmd, param);
		}
		break;
	case 758:
		{
			fg_cust_data->nafg_ratio = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->nafg_ratio=%d\n",
				cmd, param);
		}
		break;
	case 759:
		{
			fg_cust_data->nafg_ratio_en = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->nafg_ratio_en=%d\n",
				cmd, param);
		}
		break;
	case 760:
		{
			fg_cust_data->nafg_ratio_tmp_thr = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->nafg_ratio_tmp_thr=%d\n",
				cmd, param);
		}
		break;
	case 761:
		{
			wakeup_fg_algo(gm, FG_INTR_BAT_CYCLE);
			bm_err(
				"exe_BAT_EC cmd %d, bat_cycle intr\n", cmd);
		}
		break;
	case 762:
		{
			fg_cust_data->difference_fgc_fgv_th1 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->difference_fgc_fgv_th1=%d\n",
				cmd, param);
		}
		break;
	case 763:
		{
			fg_cust_data->difference_fgc_fgv_th2 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->difference_fgc_fgv_th1=%d\n",
				cmd, param);
		}
		break;
	case 764:
		{
			fg_cust_data->difference_fgc_fgv_th3 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->difference_fgc_fgv_th3=%d\n",
				cmd, param);
		}
		break;
	case 765:
		{
			fg_cust_data->difference_fullocv_ith = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->difference_fullocv_ith=%d\n",
				cmd, param);
		}
		break;
	case 766:
		{
			fg_cust_data->difference_fgc_fgv_th_soc1 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->difference_fgc_fgv_th_soc1=%d\n",
				cmd, param);
		}
		break;
	case 767:
		{
			fg_cust_data->difference_fgc_fgv_th_soc2 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->difference_fgc_fgv_th_soc2=%d\n",
				cmd, param);
		}
		break;
	case 768:
		{
			fg_cust_data->embedded_sel = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->embedded_sel=%d\n",
				cmd, param);
		}
		break;
	case 769:
		{
			fg_cust_data->car_tune_value = param * UNIT_TRANS_10;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->car_tune_value=%d\n",
				cmd, param * UNIT_TRANS_10);
		}
		break;
	case 770:
		{
			fg_cust_data->shutdown_1_time = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->shutdown_1_time=%d\n",
				cmd, param);
		}
		break;
	case 771:
		{
			fg_cust_data->tnew_told_pon_diff = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->tnew_told_pon_diff=%d\n",
				cmd, param);
		}
		break;
	case 772:
		{
			fg_cust_data->tnew_told_pon_diff2 = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->tnew_told_pon_diff2=%d\n",
				cmd, param);
		}
		break;
	case 773:
		{
			fg_cust_data->swocv_oldocv_diff = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->swocv_oldocv_diff=%d\n",
				cmd, param);
		}
		break;
	case 774:
		{
			fg_cust_data->hwocv_oldocv_diff = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->hwocv_oldocv_diff=%d\n",
				cmd, param);
		}
		break;
	case 775:
		{
			fg_cust_data->hwocv_swocv_diff = param;
			bm_err(
				"exe_BAT_EC cmd %d, fg_cust_data->hwocv_swocv_diff=%d\n",
				cmd, param);
		}
		break;
	case 776:
		{
			ec->debug_kill_daemontest = param;
			bm_err(
				"exe_BAT_EC cmd %d, debug_kill_daemontest=%d\n",
				cmd, param);
		}
		break;
	case 777:
		{
			fg_cust_data->swocv_oldocv_diff_emb = param;
			bm_err("exe_BAT_EC cmd %d, swocv_oldocv_diff_emb=%d\n",
				cmd, param);
		}
		break;
	case 778:
		{
			fg_cust_data->vir_oldocv_diff_emb_lt = param;
			bm_err(
				"exe_BAT_EC cmd %d, vir_oldocv_diff_emb_lt=%d\n",
				cmd, param);
		}
		break;
	case 779:
		{
			fg_cust_data->vir_oldocv_diff_emb_tmp = param;
			bm_err(
				"exe_BAT_EC cmd %d, vir_oldocv_diff_emb_tmp=%d\n",
				cmd, param);
		}
		break;
	case 780:
		{
			fg_cust_data->vir_oldocv_diff_emb = param;
			bm_err("exe_BAT_EC cmd %d, vir_oldocv_diff_emb=%d\n",
				cmd, param);
		}
		break;
	case 781:
		{
			ec->debug_kill_daemontest = 1;
			fg_cust_data->dod_init_sel = 12;
			bm_err("exe_BAT_EC cmd %d,force goto dod_init12=%d\n",
				cmd, param);
		}
		break;
	case 782:
		{
			fg_cust_data->charge_pseudo_full_level = param;
			bm_err(
				"exe_BAT_EC cmd %d,fg_cust_data->charge_pseudo_full_level=%d\n",
				cmd, param);
		}
		break;
	case 783:
		{
			fg_cust_data->full_tracking_bat_int2_multiply = param;
			bm_err(
				"exe_BAT_EC cmd %d,fg_cust_data->full_tracking_bat_int2_multiply=%d\n",
				cmd, param);
		}
		break;
	case 784:
		{
			bm_err(
				"exe_BAT_EC cmd %d,DAEMON_PID=%d\n",
				cmd, gm->fgd_pid);
		}
		break;
	case 785:
		{
			gm->bat_cycle_thr = param;
			bm_err(
				"exe_BAT_EC cmd %d,thr=%d\n",
				cmd, gm->bat_cycle_thr);
		}
		break;
	case 786:
		{
			gm->bat_cycle_ncar = param;

			bm_err(
				"exe_BAT_EC cmd %d,thr=%d\n",
				cmd, gm->bat_cycle_ncar);
		}
		break;
	case 787:
		{
			fg_ocv_query_soc(gm, param);
			bm_err(
				"exe_BAT_EC cmd %d,[fg_ocv_query_soc]ocv=%d\n",
				cmd, param);
		}
		break;
	case 788:
		{
			fg_cust_data->record_log = param;

			bm_err(
				"exe_BAT_EC cmd %d,record_log=%d\n",
				cmd, fg_cust_data->record_log);
		}
		break;
	case 789:
		{
			int zcv_avg_current = 0;

			zcv_avg_current = param;

			gauge_set_property(GAUGE_PROP_ZCV_INTR_EN, 0);
			gauge_set_property(GAUGE_PROP_ZCV_INTR_THRESHOLD,
				zcv_avg_current);
			gauge_set_property(GAUGE_PROP_ZCV_INTR_EN, 1);

			bm_err(
				"exe_BAT_EC cmd %d,zcv_avg_current =%d\n",
				cmd, param);
		}
		break;
	case 790:
		{
			bm_err(
				"exe_BAT_EC cmd %d,force DLPT shutdown", cmd);

			wakeup_fg_algo(gm, FG_INTR_DLPT_SD);
		}
		break;
	case 791:
		{
			gm->enable_tmp_intr_suspend = param;
			bm_err(
				"exe_BAT_EC cmd %d,gm->enable_tmp_intr_suspend =%d\n",
				cmd, param);
		}
		break;
	case 792:
		{
			wakeup_fg_algo_cmd(gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_BUILD_SEL_BATTEMP,
				param);

			bm_err(
				"exe_BAT_EC cmd %d,req bat table temp =%d\n",
				cmd, param);

		}
		break;
	case 793:
		{

			wakeup_fg_algo_cmd(gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_UPDATE_AVG_BATTEMP,
				param);

			bm_err(
				"exe_BAT_EC cmd %d,update mavg temp\n",
				cmd);

		}
		break;
	case 794:
		{
			wakeup_fg_algo_cmd(gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_SAVE_DEBUG_PARAM,
				param);

			bm_err(
				"exe_BAT_EC cmd %d,FG_KERNEL_CMD_SAVE_DEBUG_PARAM\n",
				cmd);
		}
		break;
	case 795:
		{
			bm_err(
				"exe_BAT_EC cmd %d,change aging to=%d\n",
				cmd, param);
			wakeup_fg_algo_cmd(gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_REQ_CHANGE_AGING_DATA,
				param * 100);
		}
		break;
	case 796:
		{
			bm_err(
				"exe_BAT_EC cmd %d,FG_KERNEL_CMD_AG_LOG_TEST=%d\n",
				cmd, param);
			wakeup_fg_algo_cmd(gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_AG_LOG_TEST, param);
		}
		break;
	case 797:
		{
			bm_err(
				"exe_BAT_EC cmd %d,GAUGE_PROP_CAR_TUNE_VALUE, current=%d\n",
				cmd, param);
			gauge_set_property(GAUGE_PROP_CAR_TUNE_VALUE,
				param);
		}
		break;
	case 798:
		{
			bm_err(
				"exe_BAT_EC cmd %d,FG_KERNEL_CMD_CHG_DECIMAL_RATE=%d\n",
				cmd, param);

			gm->soc_decimal_rate = param;

			wakeup_fg_algo_cmd(gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_CHG_DECIMAL_RATE, param);
		}
		break;
	case 799:
		{
			bm_err(
				"exe_BAT_EC cmd %d, Send INTR_CHR_FULL to daemon, force_full =%d\n",
				cmd, param);

			gm->is_force_full = param;
			wakeup_fg_algo(gm, FG_INTR_CHR_FULL);
		}
		break;
	case 800:
		{
			bm_err(
				"exe_BAT_EC cmd %d, charge_power_sel =%d\n",
				cmd, param);

			set_charge_power_sel(param);
		}
		break;
	case 801:
		{
			bm_err(
				"exe_BAT_EC cmd %d, charge_power_sel =%d\n",
				cmd, param);

			dump_pseudo100(param);
		}
		break;
	case 802:
		{
			gm->fg_cust_data.zcv_com_vol_limit = param;

			bm_err(
				"exe_BAT_EC cmd %d,zcv_com_vol_limit =%d\n",
				cmd, param);
		}
		break;
	case 803:
		{
			gm->fg_cust_data.sleep_current_avg = param;

			bm_err(
				"exe_BAT_EC cmd %d,sleep_current_avg =%d\n",
				cmd, param);
		}
		break;
	case 804:
		{
			wakeup_fg_algo(gm, FG_INTR_FG_TIME);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT1_HT);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT1_LT);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT2_HT);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT2_LT);
			wakeup_fg_algo(gm, FG_INTR_BAT_TIME_INT);
			wakeup_fg_algo(gm, FG_INTR_IAVG);
			wakeup_fg_algo(gm, FG_INTR_VBAT2_L);
			wakeup_fg_algo(gm, FG_INTR_VBAT2_H);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT1_CHECK);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT2_CHECK);
			gauge_set_nag_en(gm, 1);
			battery_update(gm);

			bm_err("exe_BAT_EC cmd %d for SET\n", cmd);
		}
		break;
	case 805:
		{
			gm->algo.active = true;
			battery_algo_init(gm);
			wakeup_fg_algo(gm, FG_INTR_FG_TIME);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT1_HT);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT1_LT);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT2_HT);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT2_LT);
			wakeup_fg_algo(gm, FG_INTR_BAT_TIME_INT);
			wakeup_fg_algo(gm, FG_INTR_IAVG);
			wakeup_fg_algo(gm, FG_INTR_VBAT2_L);
			wakeup_fg_algo(gm, FG_INTR_VBAT2_H);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT1_CHECK);
			wakeup_fg_algo(gm, FG_INTR_BAT_INT2_CHECK);
			bm_err("exe_BAT_EC cmd %d for SET\n", cmd);
			battery_update(gm);
		}
		break;


	default:
		bm_err(
			"exe_BAT_EC cmd %d, param %d, default\n",
			cmd, param);
		break;
	}

}


/*=========================*/
/* adb */
/*=========================*/

static ssize_t Battery_Temperature_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_err("%s, %d %d\n", __func__, gm->bs_data.bat_batt_temp,
		gm->fixed_bat_tmp);

	return sprintf(buf, "%d\n", gm->fixed_bat_tmp);
}

static ssize_t Battery_Temperature_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	signed int temp;
	int curr_bat_temp;
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	if (kstrtoint(buf, 10, &temp) == 0) {

		gm->fixed_bat_tmp = temp;
		if (gm->fixed_bat_tmp == 0xffff)
			fg_bat_temp_int_internal(gm);
		else {
			gauge_set_property(GAUGE_PROP_EN_BAT_TMP_LT, 0);
			gauge_set_property(GAUGE_PROP_EN_BAT_TMP_HT, 0);
			wakeup_fg_algo(gm, FG_INTR_BAT_TMP_C_HT);
			wakeup_fg_algo(gm, FG_INTR_BAT_TMP_HT);
		}

		curr_bat_temp = force_get_tbat(gm, true);
		bm_err(
			"%s: fixed_bat_tmp:%d ,tmp:%d!\n",
			__func__,
			temp, curr_bat_temp);

			gm->bs_data.bat_batt_temp = curr_bat_temp;
			battery_update(gm);
	} else {
		bm_err("%s: format error!\n", __func__);
	}
	return size;
}



static ssize_t UI_SOC_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_err("%s: %d %d\n",
		__func__,
		gm->ui_soc, gm->fixed_uisoc);
	return sprintf(buf, "%d\n", gm->fixed_uisoc);

}

static ssize_t UI_SOC_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{

	struct mtk_battery *gm;
	signed int temp;

	gm = get_mtk_battery();
	if (kstrtoint(buf, 10, &temp) == 0) {
		gm->fixed_uisoc = temp;

		bm_err("%s: %d %d\n",
			__func__,
			gm->ui_soc, gm->fixed_uisoc);

		battery_update(gm);
	}

	return size;
}

static ssize_t uisoc_update_type_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_trace("[FG]%s:%d\n", __func__,
		gm->fg_cust_data.uisoc_update_type);

	return sprintf(buf, "%d\n", gm->fg_cust_data.uisoc_update_type);
}

static ssize_t uisoc_update_type_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_err("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		bm_err("[%s] buf is %s\n",
			__func__, buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			bm_err(
				"[%s] val is %d ??\n",
				__func__,
				(int)val);
			val = 0;
		}

		if (val >= 0 && val <= 2) {
			gm->fg_cust_data.uisoc_update_type = val;
			wakeup_fg_algo_cmd(
				gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_UISOC_UPDATE_TYPE,
				val);
			bm_err(
				"[%s] type = %d\n",
				__func__,
				(int)val);
		} else
			bm_err(
			"[%s] invalid type:%d\n",
			__func__,
			(int)val);
	}

	return size;
}

static ssize_t disable_nafg_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_trace("%s,[FG] show nafg disable : %d\n", __func__,
		gm->cmd_disable_nafg);
	return sprintf(buf, "%d\n", gm->cmd_disable_nafg);
}

static ssize_t disable_nafg_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_err("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		bm_err("[%s] buf is %s\n",
			__func__, buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			bm_err(
				"[%s] val is %d ??\n",
				__func__,
				(int)val);
			val = 0;
		}

		if (val == 0)
			gm->cmd_disable_nafg = false;
		else
			gm->cmd_disable_nafg = true;

		wakeup_fg_algo_cmd(
			gm, FG_INTR_KERNEL_CMD,
			FG_KERNEL_CMD_DISABLE_NAFG, val);

		bm_err(
			"[%s] FG_nafg_disable = %d\n",
			__func__,
			(int)val);
	}

	return size;
}


static ssize_t ntc_disable_nafg_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_trace("[FG]%s: %d\n", __func__, gm->ntc_disable_nafg);
	return sprintf(buf, "%d\n", gm->ntc_disable_nafg);
}

static ssize_t ntc_disable_nafg_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_err("[%s]\n",  __func__);

	if (buf != NULL && size != 0) {
		bm_err("[%s] buf is %s\n", __func__, buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			bm_err(
				"[%s] val is %d ??\n",  __func__,
				(int)val);
			val = 0;
		}

		if (val == 0) {
			gm->ntc_disable_nafg = false;
			gm->Bat_EC_ctrl.fixed_temp_en = 0;
			gm->Bat_EC_ctrl.fixed_temp_value = 25;
		} else if (val == 1) {
			gm->Bat_EC_ctrl.fixed_temp_en = 1;
			gm->Bat_EC_ctrl.fixed_temp_value =
				gm->fg_cust_data.battery_tmp_to_disable_nafg;
			gm->ntc_disable_nafg = true;
			wakeup_fg_algo(gm, FG_INTR_NAG_C_DLTV);
		}

		bm_err(
			"[%s]val=%d, temp:%d %d, %d, BATTERY_TMP_TO_DISABLE_NAFG:%d\n",
			 __func__,
			(int)val,
			gm->Bat_EC_ctrl.fixed_temp_en,
			gm->Bat_EC_ctrl.fixed_temp_value,
			gm->ntc_disable_nafg,
			gm->fg_cust_data.battery_tmp_to_disable_nafg);
	}

	return size;
}

static ssize_t FG_meter_resistance_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_trace("[FG] show com_fg_meter_resistance : %d\n",
		gm->fg_cust_data.com_fg_meter_resistance);

	return sprintf(buf, "%d\n", gm->fg_cust_data.com_fg_meter_resistance);
}

static ssize_t FG_meter_resistance_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;

	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_err("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		bm_err("[%s] buf is %s\n",
			__func__, buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			bm_err(
				"[%s] val is %d ??\n",
				__func__,
				(int)val);
			val = 0;
		} else
			gm->fg_cust_data.com_fg_meter_resistance = val;

		bm_err(
			"[%s] com FG_meter_resistance = %d\n",
			__func__,
			(int)val);
	}

	return size;
}

static ssize_t FG_daemon_log_level_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;
	static int loglevel_count;

	gm = get_mtk_battery();

	loglevel_count++;
	if (loglevel_count % 5 == 0)
		bm_err(
		"[FG] show FG_daemon_log_level : %d\n",
		gm->fg_cust_data.daemon_log_level);

	return sprintf(buf, "%d\n", gm->fg_cust_data.daemon_log_level);
}

static ssize_t FG_daemon_log_level_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{

	unsigned long val = 0;
	int ret;
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_err("[FG_daemon_log_level]\n");

	if (buf != NULL && size != 0) {
		bm_err("[FG_daemon_log_level] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			bm_err(
				"[FG_daemon_log_level] val is %d ??\n",
				(int)val);
			val = 0;
		}

		if (val < 10 && val >= 0) {
			gm->fg_cust_data.daemon_log_level = val;
			wakeup_fg_algo_cmd(
				gm,
				FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_CHANG_LOGLEVEL,
				val
			);
			gm->log_level = val;
		}

		bm_err(
			"[FG_daemon_log_level]fg_cust_data.daemon_log_level=%d\n",
			gm->fg_cust_data.daemon_log_level);
	}
	return size;
}

static ssize_t FG_daemon_disable_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_trace("%s[FG] show FG disable : %d\n", __func__, gm->disableGM30);
	return sprintf(buf, "%d\n", gm->disableGM30);
}

static ssize_t FG_daemon_disable_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_err("[disable FG daemon]\n");
	disable_fg(gm);

	if (gm->disableGM30 == true)
		gm->bs_data.bat_capacity = 50;
	battery_update(gm);

	return size;
}

static ssize_t FG_Battery_CurrentConsumption_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret_value = 8888;

	ret_value = gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT) * 100;
	bm_err("%s[EM] FG_Battery_CurrentConsumption : %d .1mA\n", __func__,
		ret_value);

	return sprintf(buf, "%d\n", ret_value);
}

static ssize_t FG_Battery_CurrentConsumption_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	bm_err("%s[EM] Not Support Write Function\n", __func__);
	return size;
}

static ssize_t Power_On_Voltage_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = 3450;
	bm_err("[EM] Power_On_Voltage : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);

}

static ssize_t Power_On_Voltage_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	bm_err("%s[EM] Not Support Write Function\n", __func__);
	return size;
}

static ssize_t Power_Off_Voltage_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret_value = 1;

	ret_value = 3400;
	bm_err("[EM] Power_Off_Voltage : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t Power_Off_Voltage_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	bm_err("%s[EM] Not Support Write Function\n", __func__);
	return size;
}

static ssize_t shutdown_condition_enable_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	bm_err("%s\n", __func__);
	gm = get_mtk_battery();

	bm_trace(
		"[FG] %s : %d\n",
		__func__,
		get_shutdown_cond_flag(gm));
	return sprintf(buf, "%d\n", get_shutdown_cond_flag(gm));
}

static ssize_t shutdown_condition_enable_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret = 0;
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_err("[%s]\n", __func__);
	if (buf != NULL && size != 0) {
		bm_err("[%s] buf is %s\n",
			__func__, buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			bm_err(
				"[%s] val is %d ??\n",
				__func__,
				(int)val);
			val = 0;
		}
		set_shutdown_cond_flag(gm, val);
		bm_err(
			"[%s] shutdown_cond_enabled=%d\n",
			__func__,
			get_shutdown_cond_flag(gm));
	}

	return size;
}

static ssize_t reset_battery_cycle_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();
	bm_trace("[FG] %s : %d\n",
		__func__,
		gm->is_reset_battery_cycle);
	return sprintf(buf, "%d\n", gm->is_reset_battery_cycle);

}

static ssize_t reset_battery_cycle_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_err("[%s]\n", __func__);
	if (buf != NULL && size != 0) {
		bm_err("[%s] buf is %s\n",
			__func__, buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			bm_err(
				"[%s] val is %d ??\n",
				__func__,
				(int)val);
			val = 0;
		}
		if (val == 0)
			gm->is_reset_battery_cycle = false;
		else {
			gm->is_reset_battery_cycle = true;
			wakeup_fg_algo(gm, FG_INTR_BAT_CYCLE);
		}
		bm_err(
			"%s=%d\n",
			__func__,
			gm->is_reset_battery_cycle);
	}

	return size;
}

static ssize_t reset_aging_factor_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_trace("[FG] %s : %d\n",
		__func__,
		gm->is_reset_aging_factor);
	return sprintf(buf, "%d\n", gm->is_reset_aging_factor);
}

static ssize_t reset_aging_factor_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_err("[%s]\n", __func__);
	if (buf != NULL && size != 0) {
		bm_err("[%s] buf is %s\n",
			__func__, buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			bm_err(
				"[%s] val is %d ??\n",
				__func__,
				(int)val);
			val = 0;
		}
		if (val == 0)
			gm->is_reset_aging_factor = false;
		else {
			gm->is_reset_aging_factor = true;
			wakeup_fg_algo_cmd(gm, FG_INTR_KERNEL_CMD,
				FG_KERNEL_CMD_RESET_AGING_FACTOR, 0);
		}
		bm_err(
			"%s=%d\n",
			__func__,
			gm->is_reset_aging_factor);
	}

	return size;
}


static ssize_t BAT_EC_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mtk_battery *gm;

	bm_err("%s\n", __func__);
	gm = get_mtk_battery();


	return sprintf(buf, "%d:%d\n", gm->BAT_EC_cmd, gm->BAT_EC_param);
}

static ssize_t BAT_EC_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	int ret1 = 0, ret2 = 0;
	char cmd_buf[4], param_buf[16];
	struct mtk_battery *gm;

	bm_err("%s\n", __func__);
	gm = get_mtk_battery();
	cmd_buf[3] = '\0';
	param_buf[15] = '\0';

	if (size < 4 || size > 20) {
		bm_err("%s error, size mismatch\n",
			__func__);
		return -1;
	}

	if (buf != NULL && size != 0) {
		bm_err("buf is %s\n", buf);
		cmd_buf[0] = buf[0];
		cmd_buf[1] = buf[1];
		cmd_buf[2] = buf[2];
		cmd_buf[3] = '\0';

		if ((size - 4) > 0) {
			strncpy(param_buf, buf + 4, size - 4);
			param_buf[size - 4 - 1] = '\0';
			bm_err("[FG_IT]cmd_buf %s, param_buf %s\n",
				cmd_buf, param_buf);
			ret2 = kstrtouint(param_buf, 10, &gm->BAT_EC_param);
		}

		ret1 = kstrtouint(cmd_buf, 10, &gm->BAT_EC_cmd);

		if (ret1 != 0 || ret2 != 0) {
			bm_err("ERROR! not valid number! %d %d\n",
				ret1, ret2);
			return -1;
		}
		bm_err("CMD is:%d, param:%d\n",
			gm->BAT_EC_cmd, gm->BAT_EC_param);
	}


	exec_BAT_EC(gm->BAT_EC_cmd, gm->BAT_EC_param);

	return size;
}


static ssize_t BAT_HEALTH_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	bm_err("%s\n", __func__);

	return 0;
}


static ssize_t BAT_HEALTH_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	char copy_str[7], buf_str[350];
	char *s = buf_str, *pch;
	/* char *ori = buf_str; */
	int chr_size = 0;
	int i = 0, j = 0, count = 0, value[50];
	struct mtk_battery *gm;

	gm = get_mtk_battery();

	bm_err("%s, size =%d, str=%s\n", __func__, size, buf);

	if (size < 90 || size > 350) {
		bm_err("%s error, size mismatch\n", __func__);
		return -1;
	}

	if (size >= 90 && size <= 350) {
		for (i = 0; i < strlen(buf); i++) {
			if (buf[i] == ',')
				j++;
		}
		if (j != 46) {
			bm_err("%s error, invalid input\n", __func__);
			return -1;
		}
	}

	strncpy(buf_str, buf, size);
	bm_err("%s, copy str=%s\n", __func__, buf_str);

	if (buf != NULL && size != 0) {
		pch = strchr(s, ',');
		while (pch != NULL) {
			memset(copy_str, 0, 7);
			copy_str[6] = '\0';

			chr_size = pch - s;
			if (count == 0)
				strncpy(copy_str, s, chr_size);
			else
				strncpy(copy_str, s+1, chr_size-1);

			kstrtoint(copy_str, 10, &value[count]);
			/* bm_err("::%s::count:%d,%d\n", copy_str, count, value[count]); */
			s = pch;
			pch = strchr(pch + 1, ',');
			count++;
		}
	}

	if (count == 46) {
		for (i = 0; i < 43; i++)
			gm->bh_data.data[i] = value[i];

		for (i = 0; i < 3; i++)
			gm->bh_data.times[i].tv_sec = value[i+43];

	bm_err("%s count=%d,serial=%d,source=%d,42:%d, value43:[%d, %ld],value45[%d %ld]\n",
		__func__,
		count, gm->bh_data.data[0], gm->bh_data.data[1],
		gm->bh_data.data[42],
		value[43], gm->bh_data.times[0].tv_sec,
		value[45], gm->bh_data.times[2].tv_sec);

		wakeup_fg_algo_cmd(gm, FG_INTR_KERNEL_CMD,
			FG_KERNEL_CMD_SEND_BH_DATA, 0);

		mdelay(4);
		bm_err("%s wakeup DONE~~~\n", __func__);
	} else
		bm_err("%s count=%d, number not match\n", __func__, count);


	return size;
}



static DEVICE_ATTR_RW(Battery_Temperature);
static DEVICE_ATTR_RW(UI_SOC);
static DEVICE_ATTR_RW(uisoc_update_type);
static DEVICE_ATTR_RW(disable_nafg);
static DEVICE_ATTR_RW(ntc_disable_nafg);
static DEVICE_ATTR_RW(FG_meter_resistance);
static DEVICE_ATTR_RW(FG_daemon_log_level);
static DEVICE_ATTR_RW(FG_daemon_disable);
static DEVICE_ATTR_RW(FG_Battery_CurrentConsumption);
static DEVICE_ATTR_RW(Power_On_Voltage);
static DEVICE_ATTR_RW(Power_Off_Voltage);
static DEVICE_ATTR_RW(shutdown_condition_enable);
static DEVICE_ATTR_RW(reset_battery_cycle);
static DEVICE_ATTR_RW(reset_aging_factor);
static DEVICE_ATTR_RW(BAT_EC);
static DEVICE_ATTR_RW(BAT_HEALTH);


static int mtk_battery_setup_files(struct platform_device *pdev)
{
	int ret = 0;

	ret = device_create_file(&(pdev->dev), &dev_attr_Battery_Temperature);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_UI_SOC);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_uisoc_update_type);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_disable_nafg);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_ntc_disable_nafg);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_FG_meter_resistance);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_FG_daemon_log_level);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_FG_daemon_disable);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_BAT_EC);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev),
		&dev_attr_FG_Battery_CurrentConsumption);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_Power_On_Voltage);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_Power_Off_Voltage);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev),
		&dev_attr_shutdown_condition_enable);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_reset_battery_cycle);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_BAT_HEALTH);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_reset_aging_factor);
	if (ret)
		goto _out;


_out:
	return ret;
}






static void mtk_battery_daemon_handler(struct mtk_battery *gm, void *nl_data,
	struct fgd_nl_msg_t *ret_msg)
{
	struct fgd_nl_msg_t *msg;
	static int ptim_vbat, ptim_i;
	int int_value;
	static int badcmd;
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct power_supply *chg_psy = NULL;
	union power_supply_propval prop;
#endif

	if (gm == NULL)
		gm = get_mtk_battery();

	msg = nl_data;
	ret_msg->nl_cmd = msg->nl_cmd;
	ret_msg->fgd_cmd = msg->fgd_cmd;

	switch (msg->fgd_cmd) {
	case FG_DAEMON_CMD_IS_BAT_EXIST:
	{
		int is_bat_exist = 0;

		gauge_get_property(GAUGE_PROP_BATTERY_EXIST, &is_bat_exist);
		ret_msg->fgd_data_len += sizeof(is_bat_exist);
		memcpy(ret_msg->fgd_data,
			&is_bat_exist, sizeof(is_bat_exist));

		bm_debug(
			"[K]FG_DAEMON_CMD_IS_BAT_EXIST=%d\n",
			is_bat_exist);
	}
	break;
	case FG_DAEMON_CMD_FGADC_RESET:
	{
		bm_debug("[K]FG_DAEMON_CMD_FGADC_RESET\n");
		battery_set_property(BAT_PROP_FG_RESET, 0);
	}
	break;
	case FG_DAEMON_CMD_GET_INIT_FLAG:
	{
		ret_msg->fgd_data_len += sizeof(gm->init_flag);
		memcpy(ret_msg->fgd_data,
			&gm->init_flag, sizeof(gm->init_flag));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_INIT_FLAG=%d\n",
			gm->init_flag);
	}
	break;
	case FG_DAEMON_CMD_SET_INIT_FLAG:
	{
		memcpy(&gm->init_flag,
			&msg->fgd_data[0], sizeof(gm->init_flag));

		if (gm->init_flag == 1)
			gauge_set_property(GAUGE_PROP_SHUTDOWN_CAR, -99999);

		bm_debug(
			"[K]FG_DAEMON_CMD_SET_INIT_FLAG=%d\n",
			gm->init_flag);
	}
	break;
	case FG_DAEMON_CMD_GET_TEMPERTURE:	/* fix me */
		{
			bool update;
			int temperture = 0;

			memcpy(&update, &msg->fgd_data[0], sizeof(update));
			temperture = force_get_tbat(gm, true);
			bm_debug("[K]FG_DAEMON_CMD_GET_TEMPERTURE update=%d tmp:%d\n",
				update, temperture);
			ret_msg->fgd_data_len += sizeof(temperture);
			memcpy(ret_msg->fgd_data,
				&temperture, sizeof(temperture));
			/* gFG_temp = temperture; */
		}
	break;
	case FG_DAEMON_CMD_GET_RAC:
	{
		int rac;

		rac = gauge_get_int_property(
				GAUGE_PROP_PTIM_RESIST);
		ret_msg->fgd_data_len += sizeof(rac);
		memcpy(ret_msg->fgd_data, &rac, sizeof(rac));
		bm_debug("[K]FG_DAEMON_CMD_GET_RAC=%d\n", rac);
	}
	break;
	case FG_DAEMON_CMD_GET_DISABLE_NAFG:
	{
		int ret = 0;

		if (gm->ntc_disable_nafg || gm->vbat0_flag)
			ret = 1;
		else
			ret = 0;
		ret_msg->fgd_data_len += sizeof(ret);
		memcpy(ret_msg->fgd_data, &ret, sizeof(ret));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_DISABLE_NAFG=%d, ntc_disable_nafg:%d vbat0_flag:%d\n",
			ret, gm->ntc_disable_nafg, gm->vbat0_flag);
	}
	break;
	case FG_DAEMON_CMD_GET_PTIM_VBAT:
	{
		unsigned int ptim_bat_vol = 0;
		signed int ptim_R_curr = 0, ptim_before = 0;
		struct power_supply *psy_gauge;
		struct power_supply *psy_bat;
		union power_supply_propval val, ptim_val;


		psy_gauge = gm->gauge->psy;
		psy_bat = gm->bs_data.psy;

		if (psy_gauge == NULL) {
			bm_err("[%s]psy is not rdy\n", __func__);
			psy_gauge = gm->bs_data.psy;
		}

		if (gm->init_flag == 1) {
			power_supply_get_property(psy_gauge,
				POWER_SUPPLY_PROP_CURRENT_NOW, &ptim_val);
			ptim_before = ptim_val.intval;

			ptim_bat_vol = gauge_get_int_property(
				GAUGE_PROP_PTIM_BATTERY_VOLTAGE);
			power_supply_get_property(psy_bat,
				POWER_SUPPLY_PROP_CURRENT_NOW, &val);
			power_supply_get_property(psy_gauge,
				POWER_SUPPLY_PROP_CURRENT_NOW, &ptim_val);

			ptim_R_curr = ptim_val.intval;

			bm_err("[K]PTIM inscur:%d PTIM V:%d I:[bef:%d af:%d]\n",
				val.intval, ptim_bat_vol, ptim_before,
				ptim_R_curr);
		} else {
			ptim_bat_vol = gm->ptim_lk_v;
			ptim_R_curr = gm->ptim_lk_i;
			if (ptim_bat_vol == 0) {
				ptim_bat_vol = gauge_get_int_property(
					GAUGE_PROP_PTIM_BATTERY_VOLTAGE);
				power_supply_get_property(psy_bat,
					POWER_SUPPLY_PROP_CURRENT_NOW, &val);
				power_supply_get_property(psy_gauge,
					POWER_SUPPLY_PROP_CURRENT_NOW,
					&ptim_val);
				ptim_R_curr = ptim_val.intval;
			}
			bm_err("[K]PTIM_LK V %d:%d I %d:%d, inscur:%d\n",
				gm->ptim_lk_v, ptim_bat_vol,
				gm->ptim_lk_i, ptim_R_curr, val.intval);
		}
		ptim_vbat = ptim_bat_vol;
		ptim_i = ptim_R_curr;
		ret_msg->fgd_data_len += sizeof(ptim_vbat);
		memcpy(ret_msg->fgd_data,
			&ptim_vbat, sizeof(ptim_vbat));
	}
	break;
	case FG_DAEMON_CMD_GET_PTIM_I:
	{
		ret_msg->fgd_data_len += sizeof(ptim_i);
		memcpy(ret_msg->fgd_data, &ptim_i, sizeof(ptim_i));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_PTIM_I=%d\n", ptim_i);
	}
	break;
	case FG_DAEMON_CMD_IS_CHARGER_EXIST:
	{
		/* todo */
		int is_charger_exist = 0;

#ifndef OPLUS_FEATURE_CHG_BASIC
		if (gm->bs_data.bat_status == POWER_SUPPLY_STATUS_CHARGING)
			is_charger_exist = false;
		else
			is_charger_exist = true;
#else
		chg_psy = devm_power_supply_get_by_phandle(&gm->gauge->pdev->dev,
						       "charger");
		if (IS_ERR_OR_NULL(chg_psy)) {
			bm_err("%s Couldn't get chg_psy\n", __func__);
			is_charger_exist = false;
		} else {
			power_supply_get_property(chg_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
					bm_err("%s get chg_psy\n", __func__);
			if (prop.intval)
				is_charger_exist = true;
			else
				is_charger_exist = false;
		}
#endif /*OPLUS_FEATURE_CHG_BASIC*/
		ret_msg->fgd_data_len += sizeof(is_charger_exist);
		memcpy(ret_msg->fgd_data,
			&is_charger_exist, sizeof(is_charger_exist));

		bm_debug(
			"[K]FG_DAEMON_CMD_IS_CHARGER_EXIST=%d\n",
			is_charger_exist);
	}
	break;
	case FG_DAEMON_CMD_GET_HW_OCV:
	{
		int voltage = 0;

		gm->bs_data.bat_batt_temp = force_get_tbat(gm, true);
		voltage = gauge_get_int_property(
				GAUGE_PROP_BOOT_ZCV);
		gm->gauge->hw_status.hw_ocv = voltage;

		ret_msg->fgd_data_len += sizeof(voltage);
		memcpy(ret_msg->fgd_data, &voltage, sizeof(voltage));
		bm_debug("[K]FG_DAEMON_CMD_GET_HW_OCV=%d\n", voltage);

#ifdef GM_SIMULATOR
		gm->log.phone_state = 1;
		gm->log.ps_logtime = fg_get_log_sec();
		gm->log.ps_system_time = fg_get_system_sec();
		gm3_log_dump(true);
#endif
	}
	break;
	case FG_DAEMON_CMD_GET_FG_SW_CAR_NAFG_CNT:
	{
		int cnt = 0;
		int update;

		memcpy(&update, &msg->fgd_data[0], sizeof(update));

		if (update == 1)
			cnt = gauge_get_int_property(GAUGE_PROP_NAFG_CNT);
		else
			cnt = gm->gauge->hw_status.nafg_cnt;

		ret_msg->fgd_data_len += sizeof(cnt);
		memcpy(ret_msg->fgd_data, &cnt, sizeof(cnt));

		bm_debug(
			"[K]BATTERY_METER_CMD_GET_SW_CAR_NAFG_CNT=%d\n",
			cnt);
	}
	break;
	case FG_DAEMON_CMD_GET_FG_SW_CAR_NAFG_DLTV:
	{
		int dltv = 0;
		int update;

		memcpy(&update, &msg->fgd_data[0], sizeof(update));

		if (update == 1)
			dltv = gauge_get_int_property(GAUGE_PROP_NAFG_DLTV);
		else
			dltv = gm->gauge->hw_status.nafg_dltv;

		ret_msg->fgd_data_len += sizeof(dltv);
		memcpy(ret_msg->fgd_data, &dltv, sizeof(dltv));

		bm_debug(
			"[K]BATTERY_METER_CMD_GET_SW_CAR_NAFG_DLTV=%d\n",
			dltv);
	}
	break;
	case FG_DAEMON_CMD_GET_FG_SW_CAR_NAFG_C_DLTV:
	{
		int c_dltv = 0;
		int update;

		memcpy(&update, &msg->fgd_data[0], sizeof(update));

		if (update == 1)
			c_dltv = gauge_get_int_property(GAUGE_PROP_NAFG_C_DLTV);
		else
			c_dltv = gm->gauge->hw_status.nafg_c_dltv;

		ret_msg->fgd_data_len += sizeof(c_dltv);
		memcpy(ret_msg->fgd_data, &c_dltv, sizeof(c_dltv));

		bm_debug(
			"[K]BATTERY_METER_CMD_GET_SW_CAR_NAFG_C_DLTV=%d\n",
			c_dltv);
	}
	break;
	case FG_DAEMON_CMD_GET_FG_HW_CAR:
	{
		int fg_coulomb = 0;

		fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
		ret_msg->fgd_data_len += sizeof(fg_coulomb);
		memcpy(ret_msg->fgd_data,
			&fg_coulomb, sizeof(fg_coulomb));

		bm_debug(
			"[K]BATTERY_METER_CMD_GET_FG_HW_CAR=%d\n",
			fg_coulomb);
	}
	break;
	case FG_DAEMON_CMD_SET_SW_OCV:
	{
		int _sw_ocv;

		memcpy(&_sw_ocv, &msg->fgd_data[0], sizeof(_sw_ocv));
		gm->gauge->hw_status.sw_ocv = _sw_ocv;
		bm_debug("[K]FG_DAEMON_CMD_SET_SW_OCV=%d\n", _sw_ocv);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_TIME:
	{
		int secs;
		ktime_t ctime = 0, ktime = 0;
		struct timespec64 tmp_time, end_time;

		memcpy(&secs, &msg->fgd_data[0], sizeof(secs));

		if (secs != 0 && secs > 0) {
			ctime = ktime_get_boottime();
			tmp_time = ktime_to_timespec64(ctime);
			end_time.tv_sec = tmp_time.tv_sec + secs;
			end_time.tv_nsec = 0;
			ktime = ktime_set(end_time.tv_sec, end_time.tv_nsec);

			if (msg->fgd_subcmd_para1 == 0)
				alarm_start(&gm->tracking_timer, ktime);
			else
				alarm_start(&gm->one_percent_timer, ktime);
		} else {
			if (msg->fgd_subcmd_para1 == 0)
				alarm_cancel(&gm->tracking_timer);
			else
				alarm_cancel(&gm->one_percent_timer);
		}

		bm_debug("[K]FG_DAEMON_CMD_SET_FG_TIME=%d cmd:%d %d\n",
			secs,
			msg->fgd_subcmd, msg->fgd_subcmd_para1);
	}
	break;
	case FG_DAEMON_CMD_GET_FG_TIME:
	{
		int now_time_secs;

		now_time_secs = fg_get_system_sec();
		ret_msg->fgd_data_len += sizeof(now_time_secs);
		memcpy(ret_msg->fgd_data,
			&now_time_secs, sizeof(now_time_secs));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_NOW_TIME=%d\n",
			now_time_secs);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_BAT_INT1_GAP:
	{
		int fg_coulomb = 0;

		fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);

		memcpy(&gm->coulomb_int_gap,
			&msg->fgd_data[0], sizeof(gm->coulomb_int_gap));

		gm->coulomb_int_ht = fg_coulomb + gm->coulomb_int_gap;
		gm->coulomb_int_lt = fg_coulomb - gm->coulomb_int_gap;
		gauge_coulomb_start(gm, &gm->coulomb_plus, gm->coulomb_int_gap);
		gauge_coulomb_start(gm, &gm->coulomb_minus, -gm->coulomb_int_gap);

		bm_debug(
			"[K]FG_DAEMON_CMD_SET_FG_BAT_INT1_GAP=%d car:%d\n",
			gm->coulomb_int_gap, fg_coulomb);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_BAT_INT2_HT_GAP:
	{
		memcpy(&gm->uisoc_int_ht_gap,
			&msg->fgd_data[0], sizeof(gm->uisoc_int_ht_gap));
		gauge_coulomb_start(gm, &gm->uisoc_plus, gm->uisoc_int_ht_gap);
		bm_debug(
			"[K]FG_DAEMON_CMD_SET_FG_BAT_INT2_HT_GAP=%d\n",
			gm->uisoc_int_ht_gap);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_BAT_INT2_LT_GAP:
	{
		memcpy(&gm->uisoc_int_lt_gap,
			&msg->fgd_data[0], sizeof(gm->uisoc_int_lt_gap));
		gauge_coulomb_start(gm, &gm->uisoc_minus, -gm->uisoc_int_lt_gap);
		bm_debug(
			"[K]FG_DAEMON_CMD_SET_FG_BAT_INT2_LT_GAP=%d\n",
			gm->uisoc_int_lt_gap);
	}
	break;
	case FG_DAEMON_CMD_ENABLE_FG_BAT_INT2_HT:
	{
		int ver;
		ktime_t ktime;

		ver = gauge_get_int_property(GAUGE_PROP_HW_VERSION);
		memcpy(&gm->uisoc_int_ht_en,
			&msg->fgd_data[0], sizeof(gm->uisoc_int_ht_en));
		if (gm->uisoc_int_ht_en == 0) {
			if (ver != GAUGE_NO_HW)
				gauge_coulomb_stop(gm, &gm->uisoc_plus);
			else if (gm->soc != gm->ui_soc) {
				ktime = ktime_set(60, 0);
				alarm_start(&gm->sw_uisoc_timer, ktime);
			}
		}
		bm_debug(
			"[K]FG_DAEMON_CMD_ENABLE_FG_BAT_INT2_HT=%d\n",
			gm->uisoc_int_ht_en);
	}
	break;
	case FG_DAEMON_CMD_ENABLE_FG_BAT_INT2_LT:
	{
		memcpy(&gm->uisoc_int_lt_en,
			&msg->fgd_data[0], sizeof(gm->uisoc_int_lt_en));
		if (gm->uisoc_int_lt_en == 0)
			gauge_coulomb_stop(gm, &gm->uisoc_minus);

		bm_debug(
			"[K]FG_DAEMON_CMD_ENABLE_FG_BAT_INT2_LT=%d\n",
			gm->uisoc_int_lt_en);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_BAT_TMP_C_GAP:
	{
		int tmp = force_get_tbat(gm, true);

		memcpy(&gm->bat_tmp_c_int_gap,
			&msg->fgd_data[0], sizeof(gm->bat_tmp_c_int_gap));

		gm->bat_tmp_c_ht = tmp + gm->bat_tmp_c_int_gap;
		gm->bat_tmp_c_lt = tmp - gm->bat_tmp_c_int_gap;

		bm_debug(
			"[K]FG_DAEMON_CMD_SET_FG_BAT_TMP_C_GAP=%d ht:%d lt:%d\n",
			gm->bat_tmp_c_int_gap,
			gm->bat_tmp_c_ht,
			gm->bat_tmp_c_lt);
	}
	break;

	case FG_DAEMON_CMD_SET_FG_BAT_TMP_GAP:
	{
		int tmp = force_get_tbat(gm, true);

		memcpy(
			&gm->bat_tmp_int_gap, &msg->fgd_data[0],
			sizeof(gm->bat_tmp_int_gap));

		gm->bat_tmp_ht = tmp + gm->bat_tmp_int_gap;
		gm->bat_tmp_lt = tmp - gm->bat_tmp_int_gap;

		bm_debug(
			"[K]FG_DAEMON_CMD_SET_FG_BAT_TMP_GAP=%d ht:%d lt:%d\n",
			gm->bat_tmp_int_gap,
			gm->bat_tmp_ht, gm->bat_tmp_lt);

	}
	break;
	case FG_DAEMON_CMD_IS_BAT_PLUGOUT:
	{
		int is_bat_plugout = 0;

		is_bat_plugout = gm->gauge->hw_status.is_bat_plugout;
		ret_msg->fgd_data_len += sizeof(is_bat_plugout);
		memcpy(ret_msg->fgd_data,
			&is_bat_plugout, sizeof(is_bat_plugout));

		bm_debug(
			"[K]BATTERY_METER_CMD_GET_BOOT_BATTERY_PLUG_STATUS=%d\n",
			is_bat_plugout);
	}
	break;
	case FG_DAEMON_CMD_GET_BAT_PLUG_OUT_TIME:
	{
		unsigned int time = 0;

		time = gm->gauge->hw_status.bat_plug_out_time;
		ret_msg->fgd_data_len += sizeof(time);
		memcpy(ret_msg->fgd_data, &time, sizeof(time));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_BAT_PLUG_OUT_TIME=%d\n",
			time);
	}
	break;
	case FG_DAEMON_CMD_IS_BAT_CHARGING:
	{
		int is_bat_charging = 0;
		int bat_current = 0;

		/* BAT_DISCHARGING = 0 */
		/* BAT_CHARGING = 1 */

		bat_current = gauge_get_int_property(
			GAUGE_PROP_BATTERY_CURRENT);
		if (bat_current > 0)
			is_bat_charging = 1;
		else
			is_bat_charging = 0;

		ret_msg->fgd_data_len += sizeof(is_bat_charging);
		memcpy(ret_msg->fgd_data,
			&is_bat_charging, sizeof(is_bat_charging));

		bm_debug(
			"[K]FG_DAEMON_CMD_IS_BAT_CHARGING=%d\n",
			is_bat_charging);
	}
	break;
	case FG_DAEMON_CMD_GET_CHARGER_STATUS:
	{
		int charger_status = 0;

		/* charger status need charger API */
		/* CHR_ERR = -1 */
		/* CHR_NORMAL = 0 */
#ifndef OPLUS_FEATURE_CHG_BASIC
		if (gm->bs_data.bat_status ==
			POWER_SUPPLY_STATUS_NOT_CHARGING)
			charger_status = -1;
		else
			charger_status = 0;
#else
		chg_psy = devm_power_supply_get_by_phandle(&gm->gauge->pdev->dev,
						       "charger");
		if (IS_ERR_OR_NULL(chg_psy)) {
			bm_err("%s Couldn't get chg_psy\n", __func__);
			charger_status = -1;
		} else {
			power_supply_get_property(chg_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
					bm_err("%s get chg_psy\n", __func__);
			if (prop.intval)
				charger_status = 0;
			else
				charger_status = -1;
		}
#endif /*OPLUS_FEATURE_CHG_BASIC*/
		ret_msg->fgd_data_len += sizeof(charger_status);
		memcpy(ret_msg->fgd_data,
			&charger_status, sizeof(charger_status));

		bm_debug(
			"[K]FG_DAEMON_CMD_GET_CHARGER_STATUS=%d\n",
			charger_status);
	}
	break;
	case FG_DAEMON_CMD_CHECK_FG_DAEMON_VERSION:
	{
		/* todo */
		bm_debug(
			"[K]FG_DAEMON_CMD_CHECK_FG_DAEMON_VERSION\n");
	}
	break;
	case FG_DAEMON_CMD_GET_SHUTDOWN_DURATION_TIME:
	{
		signed int time = 0;

		time = gm->pl_shutdown_time;

		ret_msg->fgd_data_len += sizeof(time);
		memcpy(ret_msg->fgd_data, &time, sizeof(time));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_SHUTDOWN_DURATION_TIME=%d\n",
			time);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_RESET_RTC_STATUS:
	{
		int fg_reset_rtc;

		memcpy(&fg_reset_rtc, &msg->fgd_data[0], sizeof(fg_reset_rtc));
		gauge_set_property(GAUGE_PROP_RESET_FG_RTC, 0);
		bm_debug(
			"[K]BATTERY_METER_CMD_SET_FG_RESET_RTC_STATUS=%d\n",
			fg_reset_rtc);
	}
	break;
	case FG_DAEMON_CMD_SET_IS_FG_INITIALIZED:
	{
		int fg_reset;

		memcpy(&fg_reset, &msg->fgd_data[0], sizeof(fg_reset));
		gauge_set_property(GAUGE_PROP_GAUGE_INITIALIZED, fg_reset);
		bm_debug(
			"[K]BATTERY_METER_CMD_SET_FG_RESET_STATUS=%d\n",
			fg_reset);
	}
	break;
	case FG_DAEMON_CMD_GET_IS_FG_INITIALIZED:
	{
		int fg_reset;

		fg_reset = gauge_get_int_property(GAUGE_PROP_GAUGE_INITIALIZED);
		ret_msg->fgd_data_len += sizeof(fg_reset);
		memcpy(ret_msg->fgd_data, &fg_reset, sizeof(fg_reset));
		bm_debug(
			"[K]BATTERY_METER_CMD_GET_FG_RESET_STATUS=%d\n",
			fg_reset);
	}
	break;
	case FG_DAEMON_CMD_IS_HWOCV_UNRELIABLE:
	{
		int is_hwocv_unreliable;

		is_hwocv_unreliable =
			gm->gauge->hw_status.flag_hw_ocv_unreliable;
		ret_msg->fgd_data_len += sizeof(is_hwocv_unreliable);
		memcpy(ret_msg->fgd_data,
			&is_hwocv_unreliable, sizeof(is_hwocv_unreliable));
		bm_debug(
			"[K]FG_DAEMON_CMD_IS_HWOCV_UNRELIABLE=%d\n",
			is_hwocv_unreliable);
	}
	break;
	case FG_DAEMON_CMD_GET_FG_CURRENT_AVG:
	{
		int fg_current_iavg = gm->sw_iavg;
		bool valid;
		int ver;

		ver = gauge_get_int_property(GAUGE_PROP_HW_VERSION);

		if (ver >= GAUGE_HW_V2000)
			fg_current_iavg =
				gauge_get_average_current(gm, &valid);

		ret_msg->fgd_data_len += sizeof(fg_current_iavg);
		memcpy(ret_msg->fgd_data,
			&fg_current_iavg, sizeof(fg_current_iavg));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_FG_CURRENT_AVG=%d %d v:%d\n",
			fg_current_iavg, gm->sw_iavg, ver);
	}
	break;
	case FG_DAEMON_CMD_GET_FG_CURRENT_IAVG_VALID:
	{
		bool valid = false;
		int iavg_valid = true;
		int ver;

		ver = gauge_get_int_property(GAUGE_PROP_HW_VERSION);
		if (ver >= GAUGE_HW_V2000) {
			gauge_get_average_current(gm, &valid);
			iavg_valid = valid;
		}

		ret_msg->fgd_data_len += sizeof(iavg_valid);
		memcpy(ret_msg->fgd_data, &iavg_valid, sizeof(iavg_valid));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_FG_CURRENT_IAVG_VALID=%d\n",
			iavg_valid);
	}
	break;
	case FG_DAEMON_CMD_GET_ZCV:
	{
		int zcv = 0;

		zcv = gauge_get_int_property(GAUGE_PROP_ZCV);
		ret_msg->fgd_data_len += sizeof(zcv);
		memcpy(ret_msg->fgd_data, &zcv, sizeof(zcv));
		bm_debug("[K]FG_DAEMON_CMD_GET_ZCV=%d\n", zcv);
	}
	break;
	case FG_DAEMON_CMD_SET_NAG_ZCV_EN:
	{
		int nafg_zcv_en;

		memcpy(&nafg_zcv_en, &msg->fgd_data[0], sizeof(nafg_zcv_en));

		gauge_set_nag_en(gm, nafg_zcv_en);

		bm_debug(
			"[K]FG_DAEMON_CMD_SET_NAG_ZCV_EN=%d\n",
			nafg_zcv_en);
	}
	break;

	case FG_DAEMON_CMD_SET_NAG_ZCV:
	{
		int nafg_zcv;

		memcpy(&nafg_zcv, &msg->fgd_data[0], sizeof(nafg_zcv));

		gauge_set_property(GAUGE_PROP_NAFG_ZCV, nafg_zcv);
		gm->log.nafg_zcv = nafg_zcv;
		gm->gauge->hw_status.nafg_zcv = nafg_zcv;

		bm_debug("[K]BATTERY_METER_CMD_SET_NAG_ZCV=%d\n", nafg_zcv);
	}
	break;
	case FG_DAEMON_CMD_SET_NAG_C_DLTV:
	{
		int nafg_c_dltv;

		memcpy(&nafg_c_dltv, &msg->fgd_data[0], sizeof(nafg_c_dltv));
		gm->gauge->hw_status.nafg_c_dltv_th = nafg_c_dltv;
		gauge_set_property(GAUGE_PROP_NAFG_C_DLTV, nafg_c_dltv);
		gauge_set_nag_en(gm, 1);

		bm_debug(
			"[K]BATTERY_METER_CMD_SET_NAG_C_DLTV=%d\n",
			nafg_c_dltv);
	}
	break;
	case FG_DAEMON_CMD_SET_IAVG_INTR:
	{
		bm_err("[K]FG_DAEMON_CMD_SET_IAVG_INTR is removed\n");
	}
	break;
	case FG_DAEMON_CMD_SET_BAT_PLUGOUT_INTR:
	{
		int fg_bat_plugout_en;

		memcpy(&fg_bat_plugout_en,
			&msg->fgd_data[0], sizeof(fg_bat_plugout_en));
		gauge_set_property(GAUGE_PROP_BAT_PLUGOUT_EN,
			fg_bat_plugout_en);
		bm_debug(
			"[K]BATTERY_METER_CMD_SET_BAT_PLUGOUT_INTR_EN=%d\n",
			fg_bat_plugout_en);
	}
	break;
	case FG_DAEMON_CMD_SET_ZCV_INTR:
	{
		int fg_zcv_current;

		memcpy(&fg_zcv_current,
			&msg->fgd_data[0], sizeof(fg_zcv_current));

		gauge_set_property(GAUGE_PROP_ZCV_INTR_THRESHOLD,
			fg_zcv_current);
		gauge_set_property(GAUGE_PROP_ZCV_INTR_EN, 1);

		bm_debug(
			"[K]BATTERY_METER_CMD_SET_ZCV_INTR=%d\n",
			fg_zcv_current);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_QUSE:/* tbd */
	{
		int fg_quse;

		memcpy(&fg_quse, &msg->fgd_data[0], sizeof(fg_quse));

		bm_debug("[K]FG_DAEMON_CMD_SET_FG_QUSE=%d\n", fg_quse);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_DC_RATIO:/* tbd */
	{
		int fg_dc_ratio;

		memcpy(&fg_dc_ratio, &msg->fgd_data[0], sizeof(fg_dc_ratio));

		bm_debug(
			"[K]BATTERY_METER_CMD_SET_FG_DC_RATIO=%d\n",
			fg_dc_ratio);
	}
	break;
	case FG_DAEMON_CMD_SOFF_RESET:
	{
		gauge_set_property(GAUGE_PROP_SOFF_RESET, 1);
		bm_debug("[K]BATTERY_METER_CMD_SOFF_RESET\n");
	}
	break;
	case FG_DAEMON_CMD_NCAR_RESET:
	{
		gauge_set_property(GAUGE_PROP_NCAR_RESET, 1);
		bm_debug("[K]BATTERY_METER_CMD_NCAR_RESET\n");
	}
	break;
	case FG_DAEMON_CMD_SET_BATTERY_CYCLE_THRESHOLD:
	{
		memcpy(&gm->bat_cycle_thr,
			&msg->fgd_data[0], sizeof(gm->bat_cycle_thr));

		gauge_set_property(GAUGE_PROP_BAT_CYCLE_INTR_THRESHOLD,
			gm->bat_cycle_thr);

		bm_debug(
			"[K]FG_DAEMON_CMD_SET_BATTERY_CYCLE_THRESHOLD=%d\n",
			gm->bat_cycle_thr);

		fg_sw_bat_cycle_accu(gm);
	}
	break;
	case FG_DAEMON_CMD_GET_IMIX:
	{
		int imix = UNIT_TRANS_10 * fg_get_imix(gm);

		ret_msg->fgd_data_len += sizeof(imix);
		memcpy(ret_msg->fgd_data, &imix, sizeof(imix));
		bm_debug("[K]FG_DAEMON_CMD_GET_IMIX=%d\n", imix);
	}
	break;
	case FG_DAEMON_CMD_IS_BATTERY_CYCLE_RESET:
	{
		int reset = gm->is_reset_battery_cycle;

		ret_msg->fgd_data_len += sizeof(reset);
		memcpy(ret_msg->fgd_data, &reset, sizeof(reset));
		bm_debug(
			"[K]FG_DAEMON_CMD_IS_BATTERY_CYCLE_RESET = %d\n",
			reset);
		gm->is_reset_battery_cycle = false;
	}
	break;
	case FG_DAEMON_CMD_GET_AGING_FACTOR_CUST:
	{
		int aging_factor_cust = 0;
		int origin_aging_factor;

		memcpy(&origin_aging_factor,
			&msg->fgd_data[0], sizeof(origin_aging_factor));
		aging_factor_cust =
			get_customized_aging_factor(origin_aging_factor);

		ret_msg->fgd_data_len += sizeof(aging_factor_cust);
		memcpy(ret_msg->fgd_data,
			&aging_factor_cust, sizeof(aging_factor_cust));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_AGING_FACTOR_CUST = %d\n",
			aging_factor_cust);
	}
	break;
	case FG_DAEMON_CMD_GET_D0_C_SOC_CUST:
	{
		int d0_c_cust = 0;
		int origin_d0_c;

		memcpy(&origin_d0_c, &msg->fgd_data[0], sizeof(origin_d0_c));
		d0_c_cust = get_customized_d0_c_soc(origin_d0_c);

		ret_msg->fgd_data_len += sizeof(d0_c_cust);
		memcpy(ret_msg->fgd_data, &d0_c_cust, sizeof(d0_c_cust));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_D0_C_CUST = %d\n",
			d0_c_cust);
	}
	break;

	case FG_DAEMON_CMD_GET_D0_V_SOC_CUST:
	{
		int d0_v_cust = 0;
		int origin_d0_v;

		memcpy(&origin_d0_v, &msg->fgd_data[0], sizeof(origin_d0_v));
		d0_v_cust = get_customized_d0_v_soc(origin_d0_v);

		ret_msg->fgd_data_len += sizeof(d0_v_cust);
		memcpy(ret_msg->fgd_data, &d0_v_cust, sizeof(d0_v_cust));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_D0_V_CUST = %d\n",
			d0_v_cust);
	}
	break;

	case FG_DAEMON_CMD_GET_UISOC_CUST:
	{
		int uisoc_cust = 0;
		int origin_uisoc;

		memcpy(&origin_uisoc, &msg->fgd_data[0], sizeof(origin_uisoc));
		uisoc_cust = get_customized_uisoc(origin_uisoc);

		ret_msg->fgd_data_len += sizeof(uisoc_cust);
		memcpy(ret_msg->fgd_data, &uisoc_cust, sizeof(uisoc_cust));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_UISOC_CUST = %d\n",
			uisoc_cust);
	}
	break;
	case FG_DAEMON_CMD_IS_KPOC:
	{
		int is_kpoc = is_kernel_power_off_charging();

		ret_msg->fgd_data_len += sizeof(is_kpoc);
		memcpy(ret_msg->fgd_data, &is_kpoc, sizeof(is_kpoc));
		bm_debug(
			"[K]FG_DAEMON_CMD_IS_KPOC = %d\n", is_kpoc);
	}
	break;
	case FG_DAEMON_CMD_GET_NAFG_VBAT:
	{
		int nafg_vbat;

		nafg_vbat = gauge_get_int_property(GAUGE_PROP_NAFG_VBAT);
		ret_msg->fgd_data_len += sizeof(nafg_vbat);
		memcpy(ret_msg->fgd_data, &nafg_vbat, sizeof(nafg_vbat));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_NAFG_VBAT = %d\n",
			nafg_vbat);
	}
	break;
	case FG_DAEMON_CMD_GET_HW_INFO:
	{
		gauge_set_property(GAUGE_PROP_HW_INFO, 1);
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_HW_INFO\n");
	}
	break;
	case FG_DAEMON_CMD_SET_KERNEL_SOC:
	{
		int daemon_soc;
		int soc_type;

		soc_type = msg->fgd_subcmd_para1;

		memcpy(&daemon_soc, &msg->fgd_data[0], sizeof(daemon_soc));
		if (soc_type == 0)
			gm->soc = (daemon_soc + 50) / 100;

		bm_debug(
		"[K]FG_DAEMON_CMD_SET_KERNEL_SOC = %d %d, type:%d\n",
		daemon_soc, gm->soc, soc_type);

	}
	break;
	case FG_DAEMON_CMD_SET_KERNEL_UISOC:
	{
		int daemon_ui_soc;
		int old_uisoc;
		struct timespec64 diff_time;
		ktime_t ctime = 0, dtime = 0;

		memcpy(&daemon_ui_soc, &msg->fgd_data[0],
			sizeof(daemon_ui_soc));

		if (daemon_ui_soc < 0) {
			bm_debug("FG_DAEMON_CMD_SET_KERNEL_UISOC error,daemon_ui_soc:%d\n",
				daemon_ui_soc);
			daemon_ui_soc = 0;
		}

		gm->fg_cust_data.ui_old_soc = daemon_ui_soc;
		old_uisoc = gm->ui_soc;

		if (gm->disableGM30 == true)
			gm->ui_soc = 50;
		else
			gm->ui_soc = (daemon_ui_soc + 50) / 100;

		/* when UISOC changes, check the diff time for smooth */
		if (old_uisoc != gm->ui_soc) {
			ctime = ktime_get_boottime();
			dtime = ktime_sub(ctime, gm->uisoc_oldtime);
			diff_time = ktime_to_timespec64(dtime);

			bm_err("[K]FG_DAEMON_CMD_SET_KERNEL_UISOC = %d %d GM3:%d old:%d diff=%ld\n",
				daemon_ui_soc, gm->ui_soc,
				gm->disableGM30, old_uisoc, diff_time.tv_sec);
			gm->uisoc_oldtime = ctime;

			gm->bs_data.bat_capacity = gm->ui_soc;
			battery_update(gm);
		} else {
			bm_debug("[K]FG_DAEMON_CMD_SET_KERNEL_UISOC = %d %d GM3:%d\n",
				daemon_ui_soc, gm->ui_soc, gm->disableGM30);
			/* ac_update(&ac_main); */
			gm->bs_data.bat_capacity = gm->ui_soc;
			battery_update(gm);
		}
	}
	break;
	case FG_DAEMON_CMD_SET_KERNEL_INIT_VBAT:
	{
		int daemon_init_vbat;

		memcpy(&daemon_init_vbat, &msg->fgd_data[0],
			sizeof(daemon_init_vbat));
		bm_debug(
			"[K]FG_DAEMON_CMD_SET_KERNEL_INIT_VBAT = %d\n",
			daemon_init_vbat);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_SHUTDOWN_COND:
	{
		int shutdown_cond;

		memcpy(&shutdown_cond, &msg->fgd_data[0],
			sizeof(shutdown_cond));
		set_shutdown_cond(gm, shutdown_cond);
		bm_debug(
			"[K]FG_DAEMON_CMD_SET_FG_SHUTDOWN_COND = %d\n",
			shutdown_cond);
	}
	break;
	case FG_DAEMON_CMD_GET_FG_SHUTDOWN_COND:
	{
		unsigned int shutdown_cond = get_shutdown_cond(gm);

		ret_msg->fgd_data_len += sizeof(shutdown_cond);
		memcpy(ret_msg->fgd_data,
			&shutdown_cond, sizeof(shutdown_cond));

		bm_debug("[K] shutdown_cond = %d\n", shutdown_cond);
	}
	break;
	case FG_DAEMON_CMD_ENABLE_FG_VBAT_L_INT:
	{
		int fg_vbat_l_en;

		memcpy(&fg_vbat_l_en, &msg->fgd_data[0], sizeof(fg_vbat_l_en));
		gauge_set_property(GAUGE_PROP_EN_LOW_VBAT_INTERRUPT,
			fg_vbat_l_en);
		bm_debug(
			"[K]FG_DAEMON_CMD_ENABLE_FG_VBAT_L_INT = %d\n",
			fg_vbat_l_en);
	}
	break;

	case FG_DAEMON_CMD_ENABLE_FG_VBAT_H_INT:
	{
		int fg_vbat_h_en;

		memcpy(&fg_vbat_h_en, &msg->fgd_data[0], sizeof(fg_vbat_h_en));
		gauge_set_property(GAUGE_PROP_EN_HIGH_VBAT_INTERRUPT,
			fg_vbat_h_en);
		bm_debug(
			"[K]FG_DAEMON_CMD_ENABLE_FG_VBAT_H_INT = %d\n",
			fg_vbat_h_en);
	}
	break;

	case FG_DAEMON_CMD_SET_FG_VBAT_L_TH:
	{
		int fg_vbat_l_thr;

		memcpy(&fg_vbat_l_thr,
			&msg->fgd_data[0], sizeof(fg_vbat_l_thr));
		gauge_set_property(GAUGE_PROP_VBAT_LT_INTR_THRESHOLD,
		fg_vbat_l_thr);
		set_shutdown_vbat_lt(gm,
			fg_vbat_l_thr, gm->fg_cust_data.vbat2_det_voltage1);
		bm_debug(
			"[K]FG_DAEMON_CMD_SET_FG_VBAT_L_TH = %d\n",
			fg_vbat_l_thr);
	}
	break;
	case FG_DAEMON_CMD_SET_FG_VBAT_H_TH:
	{
		int fg_vbat_h_thr;

		memcpy(&fg_vbat_h_thr, &msg->fgd_data[0],
			sizeof(fg_vbat_h_thr));
		gauge_set_property(GAUGE_PROP_VBAT_HT_INTR_THRESHOLD,
		fg_vbat_h_thr);

		bm_debug(
			"[K]FG_DAEMON_CMD_SET_FG_VBAT_H_TH=%d\n",
			fg_vbat_h_thr);
	}
	break;
	case FG_DAEMON_CMD_SET_CAR_TUNE_VALUE:
	{
		signed int cali_car_tune;

		memcpy(
			&cali_car_tune,
			&msg->fgd_data[0],
			sizeof(cali_car_tune));
		bm_err("[K] cali_car_tune = %d, default = %d, Use [cali_car_tune]\n",
			cali_car_tune, gm->fg_cust_data.car_tune_value);
		gm->fg_cust_data.car_tune_value = cali_car_tune;
	}
	break;
	case FG_DAEMON_CMD_GET_RTC_UI_SOC:
	{
		int rtc_ui_soc;

		rtc_ui_soc = gauge_get_int_property(GAUGE_PROP_RTC_UI_SOC);

		ret_msg->fgd_data_len += sizeof(rtc_ui_soc);
		memcpy(ret_msg->fgd_data, &rtc_ui_soc,
			sizeof(rtc_ui_soc));
		bm_debug("[K]FG_DAEMON_CMD_GET_RTC_UI_SOC = %d\n",
			rtc_ui_soc);
	}
	break;

	case FG_DAEMON_CMD_SET_RTC_UI_SOC:
	{
		int rtc_ui_soc;

		memcpy(&rtc_ui_soc, &msg->fgd_data[0], sizeof(rtc_ui_soc));

		if (rtc_ui_soc < 0) {
			bm_err("[K]FG_DAEMON_CMD_SET_RTC_UI_SOC error,rtc_ui_soc=%d\n",
				rtc_ui_soc);

			rtc_ui_soc = 0;
		}
		gauge_set_property(GAUGE_PROP_RTC_UI_SOC, rtc_ui_soc);
		bm_debug(
			"[K] BATTERY_METER_CMD_SET_RTC_UI_SOC=%d\n",
			rtc_ui_soc);
	}
	break;
	case FG_DAEMON_CMD_SET_CON0_SOC:
	{
		int _soc = 0;

		memcpy(&_soc, &msg->fgd_data[0], sizeof(_soc));
		gauge_set_property(GAUGE_PROP_CON0_SOC, _soc);
		bm_debug("[K]FG_DAEMON_CMD_SET_CON0_SOC = %d\n", _soc);
	}
	break;

	case FG_DAEMON_CMD_GET_CON0_SOC:
	{
		int _soc = 0;

		_soc = gauge_get_int_property(GAUGE_PROP_CON0_SOC);
		ret_msg->fgd_data_len += sizeof(_soc);
		memcpy(ret_msg->fgd_data, &_soc, sizeof(_soc));
		bm_debug("[K]FG_DAEMON_CMD_GET_CON0_SOC = %d\n", _soc);
	}
	break;
	case FG_DAEMON_CMD_GET_NVRAM_FAIL_STATUS:
	{
		int flag = 0;

		flag = gauge_get_int_property(GAUGE_PROP_IS_NVRAM_FAIL_MODE);
		ret_msg->fgd_data_len += sizeof(flag);
		memcpy(ret_msg->fgd_data, &flag, sizeof(flag));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_NVRAM_FAIL_STATUS = %d\n",
			flag);
	}
	break;
	case FG_DAEMON_CMD_SET_NVRAM_FAIL_STATUS:
	{
		int flag = 0;

		memcpy(&flag, &msg->fgd_data[0], sizeof(flag));
		gauge_set_property(GAUGE_PROP_IS_NVRAM_FAIL_MODE, flag);
		bm_debug(
			"[K]FG_DAEMON_CMD_SET_NVRAM_FAIL_STATUS = %d\n",
			flag);
	}
	break;
	case FG_DAEMON_CMD_GET_RTC_TWO_SEC_REBOOT:
	{
		int two_sec_reboot_flag;

		two_sec_reboot_flag = gm->pl_two_sec_reboot;
		ret_msg->fgd_data_len += sizeof(two_sec_reboot_flag);
		memcpy(ret_msg->fgd_data,
			&two_sec_reboot_flag,
			sizeof(two_sec_reboot_flag));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_RTC_TWO_SEC_REBOOT = %d\n",
			two_sec_reboot_flag);
	}
	break;
	case FG_DAEMON_CMD_GET_RTC_INVALID:
	{
		int rtc_invalid;

		rtc_invalid = gm->gauge->hw_status.rtc_invalid;
		ret_msg->fgd_data_len += sizeof(rtc_invalid);
		memcpy(ret_msg->fgd_data, &rtc_invalid, sizeof(rtc_invalid));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_RTC_INVALID = %d\n",
			rtc_invalid);
	}
	break;
	case FG_DAEMON_CMD_SET_DAEMON_PID:
	{
		fg_cmd_check(msg);
		/* check is daemon killed case*/
		if (gm->fgd_pid == 0) {
			memcpy(&gm->fgd_pid, &msg->fgd_data[0],
				sizeof(gm->fgd_pid));
			bm_err("[K]FG_DAEMON_CMD_SET_DAEMON_PID = %d(first launch)\n",
				gm->fgd_pid);
		} else {
			memcpy(&gm->fgd_pid, &msg->fgd_data[0],
				sizeof(gm->fgd_pid));
			bm_err("[K]FG_DAEMON_CMD_SET_DAEMON_PID=%d,kill daemon:%d init_flag:%d (re-launch)\n",
				gm->fgd_pid,
				gm->Bat_EC_ctrl.debug_kill_daemontest,
				gm->init_flag);
			if (gm->Bat_EC_ctrl.debug_kill_daemontest != 1 &&
				gm->init_flag == 1)
				gm->fg_cust_data.dod_init_sel = 14;
		}
	}
	break;
	case FG_DAEMON_CMD_GET_VBAT:
	{
		unsigned int vbat = 0;

		if (gm->disableGM30)
			vbat = 4000 * 10;
		else
			vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE) * 10;

		ret_msg->fgd_data_len += sizeof(vbat);
		memcpy(ret_msg->fgd_data, &vbat, sizeof(vbat));
		bm_debug("[K]FG_DAEMON_CMD_GET_VBAT = %d\n", vbat);
	}
	break;
	case FG_DAEMON_CMD_PRINT_LOG:
	{
		fg_cmd_check(msg);
		bm_err("%s", &msg->fgd_data[0]);
	}
	break;
	case FG_DAEMON_CMD_SEND_VERSION_CONTROL:
	{
		bm_err("[K]FG_DAEMON_CMD_SEND_VERSION_CONTROL\n");
	}
	case FG_DAEMON_CMD_SEND_CUSTOM_TABLE:
	{
		fg_daemon_send_data(gm, msg->fgd_cmd,
			&msg->fgd_data[0],
			&ret_msg->fgd_data[0]);
	}
	break;
	case FG_DAEMON_CMD_GET_CUSTOM_SETTING:
	case FG_DAEMON_CMD_GET_CUSTOM_TABLE:
	{
		fg_cmd_check(msg);
		bm_debug("[K]FG_DAEMON_CMD_GET_DATA\n");
		fg_daemon_get_data(msg->fgd_cmd, &msg->fgd_data[0],
			&ret_msg->fgd_data[0]);
		ret_msg->fgd_data_len =
			sizeof(struct fgd_cmd_param_t_4);
	}
	break;
	case FG_DAEMON_CMD_DUMP_LOG:
	{
		bm_debug("[K]FG_DAEMON_CMD_DUMP_LOG %d %d %d\n",
			msg->fgd_subcmd, msg->fgd_subcmd_para1,
			(int)strlen(&msg->fgd_data[0]));
	}
	break;
	case FG_DAEMON_CMD_GET_SHUTDOWN_CAR:
	{
		int shutdown_car_diff = 0;
		int tmp_cardiff = 0;

		shutdown_car_diff = gauge_get_int_property(
			GAUGE_PROP_SHUTDOWN_CAR);

		if (abs(shutdown_car_diff) > 1000) {
			tmp_cardiff = shutdown_car_diff;
			shutdown_car_diff = 0;
		}

		ret_msg->fgd_data_len += sizeof(shutdown_car_diff);
		memcpy(ret_msg->fgd_data, &shutdown_car_diff,
			sizeof(shutdown_car_diff));
		bm_debug(
			"[K]FG_DAEMON_CMD_GET_SHUTDOWN_CAR = %d, tmp=%d\n",
			shutdown_car_diff, tmp_cardiff);
	}
	break;
	case FG_DAEMON_CMD_GET_NCAR:
	{
		int ver;

		ver = gauge_get_int_property(GAUGE_PROP_HW_VERSION);
		if (ver >= GAUGE_HW_V1000 &&
			ver < GAUGE_HW_V2000)
			memcpy(ret_msg->fgd_data, &gm->bat_cycle_ncar,
				sizeof(gm->bat_cycle_ncar));
		else {
			gauge_set_property(GAUGE_PROP_HW_INFO, 1);
			memcpy(ret_msg->fgd_data, &gm->gauge->fg_hw_info.ncar,
				sizeof(gm->gauge->fg_hw_info.ncar));
		}
		bm_debug("[K]FG_DAEMON_CMD_GET_NCAR version:%d ncar:%d %d\n",
			ver, gm->bat_cycle_ncar, gm->gauge->fg_hw_info.ncar);
	}
	break;
	case FG_DAEMON_CMD_GET_CURR_1:
	{
		gauge_set_property(GAUGE_PROP_HW_INFO, 1);

		memcpy(ret_msg->fgd_data, &gm->gauge->fg_hw_info.current_1,
			sizeof(gm->gauge->fg_hw_info.current_1));

		bm_debug("[K]FG_DAEMON_CMD_GET_CURR_1 %d\n",
			gm->gauge->fg_hw_info.current_1);
	}
	break;
	case FG_DAEMON_CMD_GET_CURR_2:
	{
		gauge_set_property(GAUGE_PROP_HW_INFO, 1);

		memcpy(ret_msg->fgd_data, &gm->gauge->fg_hw_info.current_2,
			sizeof(gm->gauge->fg_hw_info.current_2));

		bm_debug("[K]FG_DAEMON_CMD_GET_CURR_2 %d\n",
			gm->gauge->fg_hw_info.current_2);

	}
	break;
	case FG_DAEMON_CMD_GET_REFRESH:
	{
		gauge_set_property(GAUGE_PROP_HW_INFO, 1);
		bm_debug("[K]FG_DAEMON_CMD_GET_REFRESH\n");
	}
	break;
	case FG_DAEMON_CMD_GET_IS_AGING_RESET:
	{
		int reset = gm->is_reset_aging_factor;

		memcpy(ret_msg->fgd_data, &reset,
			sizeof(reset));

		gm->is_reset_aging_factor = 0;
		bm_debug("[K]FG_DAEMON_CMD_GET_IS_AGING_RESET %d %d\n",
			reset, gm->is_reset_aging_factor);
	}
	break;
	case FG_DAEMON_CMD_SET_SOC:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->soc = (int_value + 50) / 100;
		bm_debug("[K]FG_DAEMON_CMD_SET_SOC %d\n",
			gm->soc);
	}
	break;
	case FG_DAEMON_CMD_SET_C_D0_SOC:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->fg_cust_data.c_old_d0 = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_C_D0_SOC %d\n",
		gm->fg_cust_data.c_old_d0);
	}
	break;
	case FG_DAEMON_CMD_SET_V_D0_SOC:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->fg_cust_data.v_old_d0 = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_V_D0_SOC %d\n",
		gm->fg_cust_data.v_old_d0);
	}
	break;
	case FG_DAEMON_CMD_SET_C_SOC:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->fg_cust_data.c_soc = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_C_SOC %d\n",
		gm->fg_cust_data.c_soc);
	}
	break;
	case FG_DAEMON_CMD_SET_V_SOC:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->fg_cust_data.v_soc = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_V_SOC %d\n",
		gm->fg_cust_data.v_soc);
	}
	break;
	case FG_DAEMON_CMD_SET_QMAX_T_AGING:
	{
		/* todo */
		bm_debug("[K]FG_DAEMON_CMD_SET_QMAX_T_AGING\n");
	}
	break;
	case FG_DAEMON_CMD_SET_SAVED_CAR:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->d_saved_car = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_SAVED_CAR %d\n",
		gm->d_saved_car);
	}
	break;
	case FG_DAEMON_CMD_SET_AGING_FACTOR:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->aging_factor = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_AGING_FACTOR %d\n",
		gm->aging_factor);
	}
	break;
	case FG_DAEMON_CMD_SET_QMAX:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->algo_qmax = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_QMAX %d\n",
		gm->algo_qmax);
	}
	break;
	case FG_DAEMON_CMD_SET_BAT_CYCLES:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->bat_cycle = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_BAT_CYCLES %d\n",
		gm->bat_cycle);
	}
	break;
	case FG_DAEMON_CMD_SET_NCAR:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->bat_cycle_ncar = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_NCAR %d\n",
		gm->bat_cycle_ncar);
	}
	break;
	case FG_DAEMON_CMD_SET_OCV_MAH:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->algo_ocv_to_mah = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_OCV_MAH %d\n",
		gm->algo_ocv_to_mah);
	}
	break;
	case FG_DAEMON_CMD_SET_OCV_VTEMP:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->algo_vtemp = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_OCV_VTEMP %d\n",
		gm->algo_vtemp);
	}
	break;
	case FG_DAEMON_CMD_SET_OCV_SOC:
	{
		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));
		gm->algo_ocv_to_soc = int_value;
		bm_debug("[K]FG_DAEMON_CMD_SET_OCV_SOC %d\n",
		gm->algo_ocv_to_soc);
	}
	break;
	case FG_DAEMON_CMD_SET_CON0_SOFF_VALID:
	{
		int ori_value = 0;

		memcpy(&int_value, &msg->fgd_data[0], sizeof(int_value));

		ori_value = gauge_get_int_property(
			GAUGE_PROP_MONITOR_SOFF_VALIDTIME);
		gauge_set_property(GAUGE_PROP_MONITOR_SOFF_VALIDTIME,
			int_value);

		bm_debug("[K]FG_DAEMON_CMD_SET_CON0_SOFF_VALID ori:%d, new:%d\n",
			ori_value, int_value);
	}
	break;

	case FG_DAEMON_CMD_SET_AGING_INFO:
		bm_debug("[K]FG_DAEMON_CMD_SET_AGING_INFO\n");
	break;
	case FG_DAEMON_CMD_GET_SOC_DECIMAL_RATE:
	{
		int decimal_rate = gm->soc_decimal_rate;

		ret_msg->fgd_data_len += sizeof(decimal_rate);
		memcpy(ret_msg->fgd_data, &decimal_rate,
			sizeof(decimal_rate));
		bm_debug("[K]FG_DAEMON_CMD_GET_SOC_DECIMAL_RATE %d\n",
			decimal_rate);
	}
	break;
	case FG_DAEMON_CMD_GET_VERSION_CONTROL:
	{
		int mode = gm->algo.active;

		ret_msg->fgd_data_len += sizeof(mode);
		memcpy(ret_msg->fgd_data, &mode,
			sizeof(mode));
		bm_debug("[K]FG_DAEMON_CMD_GET_VERSION_CONTROL %d\n",
			mode);
	}
	break;
	case FG_DAEMON_CMD_GET_DIFF_SOC_SET:
	{
		int soc_setting = 1;

		ret_msg->fgd_data_len += sizeof(soc_setting);
		memcpy(ret_msg->fgd_data, &soc_setting,
			sizeof(soc_setting));

		bm_debug("[K]FG_DAEMON_CMD_GET_DIFF_SOC_SET %d\n",
			soc_setting);
	}
	break;
	case FG_DAEMON_CMD_SET_ZCV_INTR_EN:
		bm_debug("[K]FG_DAEMON_CMD_SET_ZCV_INTR_EN\n");
	break;
	case FG_DAEMON_CMD_GET_IS_FORCE_FULL:
	{
		/* 1 = trust customer full condition */
		/* 0 = using gauge ori full flow */
		int force_full = gm->is_force_full;

		bm_debug("[K]FG_DAEMON_CMD_GET_IS_FORCE_FULL, %d\n",
			force_full);
		ret_msg->fgd_data_len += sizeof(force_full);
		memcpy(ret_msg->fgd_data,
			&force_full, sizeof(force_full));
	}
	break;
//timotodo
	case FG_DAEMON_CMD_GET_ZCV_INTR_CURR:
	{
		int zcv_cur = 0;
		zcv_cur = gauge_get_int_property(GAUGE_PROP_ZCV_CURRENT);

		bm_debug("[K]FG_DAEMON_CMD_GET_ZCV_INTR_CURR, %d\n", zcv_cur);

		ret_msg->fgd_data_len += sizeof(zcv_cur);
		memcpy(ret_msg->fgd_data,
			&zcv_cur, sizeof(zcv_cur));

	}
	break;
	case FG_DAEMON_CMD_GET_CHARGE_POWER_SEL:
	{
		int charge_power_sel = gm->charge_power_sel;

		bm_debug("[K]FG_DAEMON_CMD_GET_CHARGE_POWER_SEL,%d\n",
			charge_power_sel);
		ret_msg->fgd_data_len += sizeof(charge_power_sel);
		memcpy(ret_msg->fgd_data,
			&charge_power_sel, sizeof(charge_power_sel));

	}
	break;
	case FG_DAEMON_CMD_COMMUNICATION_INT:
	{
		bm_debug("[K]FG_DAEMON_CMD_COMMUNICATION_INT\n");
	}
	break;
	case FG_DAEMON_CMD_SET_BATTERY_CAPACITY:
	{
		struct fgd_cmd_param_t_8 param;
		char *rcv;
		struct fgd_cmd_param_t_4 *prcv;

		rcv = &msg->fgd_data[0];
		prcv = (struct fgd_cmd_param_t_4 *)rcv;
		memcpy(&param, prcv->input, sizeof(struct fgd_cmd_param_t_8));
#ifdef OPLUS_FEATURE_CHG_BASIC
		gm->prev_batt_fcc = param.data[4];
		gm->prev_batt_remaining_capacity = param.data[4] /10 * param.data[6] / 10000;
#endif /* OPLUS_FEATURE_CHG_BASIC */
		bm_err("[fr] FG_DAEMON_CMD_SET_BATTERY_CAPACITY = %d %d %d %d %d %d %d %d %d %d RM:%d\n",
				param.data[0],
				param.data[1],
				param.data[2],
				param.data[3],
				param.data[4],
				param.data[5],
				param.data[6],
				param.data[7],
				param.data[8],
				param.data[9],
				param.data[4] * param.data[6] / 10000);
	}
	break;
	case FG_DAEMON_CMD_GET_BH_DATA:
	{
		bm_debug("[K]FG_DAEMON_CMD_GET_BH_DATA\n");

		ret_msg->fgd_data_len += sizeof(struct ag_center_data_st);
		memcpy(ret_msg->fgd_data,
			&gm->bh_data, sizeof(struct ag_center_data_st));

	}
	break;



	default:
		badcmd++;
		bm_err("[%s]bad cmd:0x%x times:%d\n", __func__,
			msg->fgd_cmd, badcmd);
		break;
	}
}

void mtk_battery_netlink_handler(struct sk_buff *skb)
{
	u32 pid;
	kuid_t uid;
	int seq;
	void *data;
	struct nlmsghdr *nlh;
	struct fgd_nl_msg_t *fgd_msg, *fgd_ret_msg;
	int size = 0;
	static struct mtk_battery *gm;

	if (gm == NULL)
		gm = get_mtk_battery();

	nlh = (struct nlmsghdr *)skb->data;
	pid = NETLINK_CREDS(skb)->pid;
	uid = NETLINK_CREDS(skb)->uid;
	seq = nlh->nlmsg_seq;

	data = NLMSG_DATA(nlh);

	fgd_msg = (struct fgd_nl_msg_t *)data;

	bm_debug("rcv:%d %d %d %d %d %d\n",
		fgd_msg->nl_cmd,
		fgd_msg->fgd_cmd,
		fgd_msg->fgd_subcmd,
		fgd_msg->fgd_subcmd_para1,
		fgd_msg->fgd_data_len,
		fgd_msg->identity);

	if (fgd_msg->identity != FGD_NL_MAGIC) {
		bm_err("[%s]not correct MTKFG netlink packet!%d\n",
			__func__, fgd_msg->identity);
		return;
	}

	if (gm->fgd_pid != pid &&
		fgd_msg->fgd_cmd > FG_DAEMON_CMD_SET_DAEMON_PID) {
		bm_err("[%s]drop rev netlink pid:%d:%d  cmd:%d:%d\n",
			__func__,
			pid,
			gm->fgd_pid,
			fgd_msg->fgd_cmd,
			FG_DAEMON_CMD_SET_DAEMON_PID);
		return;
	}

	size = fgd_msg->fgd_ret_data_len + FGD_NL_HDR_LEN;

	if (size > (PAGE_SIZE << 1))
		fgd_ret_msg = vmalloc(size);
	else {
		if (in_interrupt())
			fgd_ret_msg = kmalloc(size, GFP_ATOMIC);
		else
			fgd_ret_msg = kmalloc(size, GFP_KERNEL);
	}

	if (fgd_ret_msg == NULL) {
		if (size > PAGE_SIZE)
			fgd_ret_msg = vmalloc(size);

		if (fgd_ret_msg == NULL)
			return;
	}

	if (!fgd_ret_msg)
		return;

	memset(fgd_ret_msg, 0, size);
	mtk_battery_daemon_handler(gm, data, fgd_ret_msg);
	mtk_battery_send_to_user(gm, seq, fgd_ret_msg);

	kvfree(fgd_ret_msg);
}


unsigned int TempConverBattThermistor(struct mtk_battery *gm, int temp)
{
	int RES1 = 0, RES2 = 0;
	int TMP1 = 0, TMP2 = 0;
	int i;
	unsigned int TBatt_R_Value = 0xffff;
	struct fg_temp *ptable;

	ptable = gm->tmp_table;


	if (temp >= ptable[20].BatteryTemp) {
		TBatt_R_Value = ptable[20].TemperatureR;
	} else if (temp <= ptable[0].BatteryTemp) {
		TBatt_R_Value = ptable[0].TemperatureR;
	} else {
		RES1 = ptable[0].TemperatureR;
		TMP1 = ptable[0].BatteryTemp;

		for (i = 0; i <= 20; i++) {
			if (temp <= ptable[i].BatteryTemp) {
				RES2 = ptable[i].TemperatureR;
				TMP2 = ptable[i].BatteryTemp;
				break;
			}
			{	/* hidden else */
				RES1 = ptable[i].TemperatureR;
				TMP1 = ptable[i].BatteryTemp;
			}
		}


		TBatt_R_Value = interpolation(TMP1, RES1, TMP2, RES2, temp);
	}

	bm_trace(
		"[%s] [%d] %d %d %d %d %d\n",
		__func__,
		TBatt_R_Value, TMP1,
		RES1, TMP2, RES2, temp);

	return TBatt_R_Value;
}

unsigned int TempToBattVolt(struct mtk_battery *gm, int temp, int update)
{
	unsigned int R_NTC = TempConverBattThermistor(gm, temp);
	long long Vin = 0;
	long long V_IR_comp = 0;
	/*int vbif28 = pmic_get_auxadc_value(AUXADC_LIST_VBIF);*/
	int vbif28 = gm->rbat.rbat_pull_up_volt;
	static int fg_current_temp;
	static bool fg_current_state;
	int fg_r_value = gm->fg_cust_data.com_r_fg_value;
	int fg_meter_res_value = 0;

	if (gm->no_bat_temp_compensate == 0)
		fg_meter_res_value = gm->fg_cust_data.com_fg_meter_resistance;
	else
		fg_meter_res_value = 0;

#ifdef RBAT_PULL_UP_VOLT_BY_BIF
	vbif28 = gauge_get_int_property(GAUGE_PROP_BIF_VOLTAGE);
#endif
	Vin = (long long)R_NTC * vbif28 * 10;	/* 0.1 mV */

#if defined(__LP64__) || defined(_LP64)
	do_div(Vin, (R_NTC + gm->rbat.rbat_pull_up_r));
#else
	Vin = div_s64(Vin, (R_NTC + gm->rbat.rbat_pull_up_r));
#endif

	if (update == true) {
		fg_current_temp = gauge_get_int_property(
			GAUGE_PROP_BATTERY_CURRENT);
		if (fg_current_temp <= 0)
			fg_current_state = false;
		else
			fg_current_state = true;
	}

	if (fg_current_state == true) {
		V_IR_comp = Vin;
		V_IR_comp +=
			((fg_current_temp *
			(fg_meter_res_value + fg_r_value)) / 10000);
	} else {
		V_IR_comp = Vin;
		V_IR_comp -=
			((fg_current_temp *
			(fg_meter_res_value + fg_r_value)) / 10000);
	}

	bm_trace("[%s] temp %d R_NTC %d V(%lld %lld) I %d CHG %d\n",
		__func__,
		temp, R_NTC, Vin, V_IR_comp, fg_current_temp, fg_current_state);

	return (unsigned int) V_IR_comp;
}


void fg_bat_temp_int_internal(struct mtk_battery *gm)
{
	int tmp = 0;
	int fg_bat_new_ht, fg_bat_new_lt;

	if (gm->disableGM30) {
		gm->bs_data.bat_batt_temp = 25;
		battery_update(gm);
		return;
	}

#if defined(CONFIG_MTK_DISABLE_GAUGE) || defined(FIXED_TBAT_25)
	gm->bs_data.bat_batt_temp = 25;
	battery_update(gm);
	tmp = 1;
	fg_bat_new_ht = 1;
	fg_bat_new_lt = 1;
	return;
#else
	tmp = force_get_tbat(gm, true);

	gauge_set_property(GAUGE_PROP_EN_BAT_TMP_LT, 0);
	gauge_set_property(GAUGE_PROP_EN_BAT_TMP_HT, 0);


//	if (get_ec()->fixed_temp_en == 1)
//		tmp = get_ec()->fixed_temp_value;

	if (tmp >= gm->bat_tmp_c_ht)
		wakeup_fg_algo(gm, FG_INTR_BAT_TMP_C_HT);
	else if (tmp <= gm->bat_tmp_c_lt)
		wakeup_fg_algo(gm, FG_INTR_BAT_TMP_C_LT);

	if (tmp >= gm->bat_tmp_ht)
		wakeup_fg_algo(gm, FG_INTR_BAT_TMP_HT);
	else if (tmp <= gm->bat_tmp_lt)
		wakeup_fg_algo(gm, FG_INTR_BAT_TMP_LT);

	fg_bat_new_ht = TempToBattVolt(gm, tmp + 1, 1);
	fg_bat_new_lt = TempToBattVolt(gm, tmp - 1, 0);

	if (gm->fixed_bat_tmp == 0xffff) {
		gauge_set_property(GAUGE_PROP_BAT_TMP_LT_THRESHOLD,
			fg_bat_new_lt);
		gauge_set_property(GAUGE_PROP_EN_BAT_TMP_LT, 1);

		gauge_set_property(GAUGE_PROP_BAT_TMP_HT_THRESHOLD,
			fg_bat_new_ht);
		gauge_set_property(GAUGE_PROP_EN_BAT_TMP_HT, 1);
	}
	bm_debug("[%s][FG_TEMP_INT] T[%d] V[%d %d] C[%d %d] h[%d %d]\n",
		__func__,
		tmp, gm->bat_tmp_ht,
		gm->bat_tmp_lt, gm->bat_tmp_c_ht,
		gm->bat_tmp_c_lt,
		fg_bat_new_lt, fg_bat_new_ht);

	gm->bs_data.bat_batt_temp = tmp;
	battery_update(gm);
#endif
}

void fg_bat_temp_int_sw_check(struct mtk_battery *gm)
{
	int tmp = force_get_tbat(gm, true);

	if (gm->disableGM30)
		return;

	bm_debug(
		"[%s] tmp %d lt %d ht %d\n",
		__func__,
		tmp, gm->bat_tmp_lt,
		gm->bat_tmp_ht);

	if (tmp >= gm->bat_tmp_ht)
		fg_bat_temp_int_internal(gm);
	else if (tmp <= gm->bat_tmp_lt)
		fg_bat_temp_int_internal(gm);
}

void fg_int_event(struct mtk_battery *gm, enum gauge_event evt)
{
	if (gm->is_probe_done == false) {
		bm_err("[%s]battery is not rdy:%d\n",
			__func__, gm->is_probe_done);
		return;
	}

	if (evt != EVT_INT_NAFG_CHECK)
		fg_bat_temp_int_sw_check(gm);

	gauge_set_property(GAUGE_PROP_EVENT, evt);
}

/* ============================================================ */
/* check bat plug out  */
/* ============================================================ */
void sw_check_bat_plugout(struct mtk_battery *gm)
{
	int is_bat_exist;

	if (gm->disable_plug_int && gm->disableGM30 != true) {
		is_bat_exist = gauge_get_int_property(GAUGE_PROP_BATTERY_EXIST);
		/* fg_bat_plugout_int_handler(); */
		if (is_bat_exist == 0) {
			bm_err(
				"[swcheck_bat_plugout]g_disable_plug_int=%d, is_bat_exist %d, is_fg_disable %d\n",
				gm->disable_plug_int,
				is_bat_exist,
				gm->disableGM30);

			gm->bs_data.bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
			wakeup_fg_algo(gm, FG_INTR_BAT_PLUGOUT);
			battery_update(gm);
			kernel_power_off();
		}
	}
}

/* ============================================================ */
/* sw low battery interrupt handler */
/* ============================================================ */
void fg_update_sw_low_battery_check(unsigned int thd)
{
	int vbat;
	int thd1, thd2, thd3;
	struct mtk_battery *gm;
	struct fuel_gauge_custom_data *fg_cust_data;
	int version = 0;

	gm = get_mtk_battery();
	fg_cust_data = &gm->fg_cust_data;

	thd1 = fg_cust_data->vbat2_det_voltage1 / 10;
	thd2 = fg_cust_data->vbat2_det_voltage2 / 10;
	thd3 = fg_cust_data->vbat2_det_voltage3 / 10;

	gauge_get_property(GAUGE_PROP_HW_VERSION, &version);

	if (version >= GAUGE_HW_V2000)
		return;

	if (gm->disableGM30)
		vbat = 4000 * 10;
	else
		vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE) * 10;

	bm_err("[%s]vbat:%d %d ht:%d %d lt:%d %d\n",
		__func__,
		thd, vbat,
		gm->sw_low_battery_ht_en,
		gm->sw_low_battery_ht_threshold,
		gm->sw_low_battery_lt_en,
		gm->sw_low_battery_lt_threshold);

	if (gm->sw_low_battery_ht_en == 1 && thd == thd3) {
		mutex_lock(&gm->sw_low_battery_mutex);
		gm->sw_low_battery_ht_en = 0;
		gm->sw_low_battery_lt_en = 0;
		mutex_unlock(&gm->sw_low_battery_mutex);
		disable_shutdown_cond(gm, LOW_BAT_VOLT);
		wakeup_fg_algo(gm, FG_INTR_VBAT2_H);
	}
	if (gm->sw_low_battery_lt_en == 1 && (thd == thd1 || thd == thd2)) {
		mutex_lock(&gm->sw_low_battery_mutex);
		gm->sw_low_battery_ht_en = 0;
		gm->sw_low_battery_lt_en = 0;
		mutex_unlock(&gm->sw_low_battery_mutex);
		wakeup_fg_algo(gm, FG_INTR_VBAT2_L);
	}
}


/* ============================================================ */
/* hw low battery interrupt handler */
/* ============================================================ */
static int sw_vbat_h_irq_handler(struct mtk_battery *gm)
{
	bm_debug("[%s]\n", __func__);
	gauge_set_property(GAUGE_PROP_EN_HIGH_VBAT_INTERRUPT, 0);
	gauge_set_property(GAUGE_PROP_EN_LOW_VBAT_INTERRUPT, 0);
	disable_shutdown_cond(gm, LOW_BAT_VOLT);
	wakeup_fg_algo(gm, FG_INTR_VBAT2_H);

	fg_int_event(gm, EVT_INT_VBAT_H);
	sw_check_bat_plugout(gm);
	return 0;
}

static irqreturn_t vbat_h_irq(int irq, void *data)
{
	struct mtk_battery *gm = data;

	if (fg_interrupt_check(gm) == false)
		return IRQ_HANDLED;
	sw_vbat_h_irq_handler(gm);
	return IRQ_HANDLED;
}

static int sw_vbat_l_irq_handler(struct mtk_battery *gm)
{
	bm_debug("[%s]\n", __func__);
	gauge_set_property(GAUGE_PROP_EN_HIGH_VBAT_INTERRUPT, 0);
	gauge_set_property(GAUGE_PROP_EN_LOW_VBAT_INTERRUPT, 0);
	wakeup_fg_algo(gm, FG_INTR_VBAT2_L);

	fg_int_event(gm, EVT_INT_VBAT_L);
	sw_check_bat_plugout(gm);
	return 0;
}

static irqreturn_t vbat_l_irq(int irq, void *data)
{
	struct mtk_battery *gm = data;

	if (fg_interrupt_check(gm) == false)
		return IRQ_HANDLED;
	sw_vbat_l_irq_handler(gm);

	return IRQ_HANDLED;
}

/* ============================================================ */
/* nafg interrupt handler */
/* ============================================================ */
static int nafg_irq_handler(struct mtk_battery *gm)
{
	struct mtk_gauge *gauge;
	int nafg_en = 0;
	signed int nafg_cnt = 0;
	signed int nafg_dltv = 0;
	signed int nafg_c_dltv = 0;
	int vbat = 0;

	gauge = gm->gauge;
	/* 1. Get SW Car value */
	fg_int_event(gm, EVT_INT_NAFG_CHECK);

	nafg_cnt = gauge_get_int_property(GAUGE_PROP_NAFG_CNT);
	nafg_dltv = gauge_get_int_property(GAUGE_PROP_NAFG_DLTV);
	nafg_c_dltv = gauge_get_int_property(GAUGE_PROP_NAFG_C_DLTV);

	gauge->hw_status.nafg_cnt = nafg_cnt;
	gauge->hw_status.nafg_dltv = nafg_dltv;
	gauge->hw_status.nafg_c_dltv = nafg_c_dltv;

	if (gm->disableGM30)
		vbat = 4000;
	else
		vbat = gauge_get_int_property(GAUGE_PROP_BATTERY_VOLTAGE);

	bm_err(
		"[%s][cnt:%d dltv:%d cdltv:%d cdltvt:%d zcv:%d vbat:%d]\n",
		__func__,
		nafg_cnt,
		nafg_dltv,
		nafg_c_dltv,
		gauge->hw_status.nafg_c_dltv_th,
		gauge->hw_status.nafg_zcv,
		vbat);
	/* battery_dump_nag(); */

	/* 2. Stop HW interrupt*/
	gauge_set_nag_en(gm, nafg_en);

	fg_int_event(gm, EVT_INT_NAFG);

	/* 3. Notify fg daemon */
	wakeup_fg_algo(gm, FG_INTR_NAG_C_DLTV);
	gm->last_nafg_update_time = ktime_get_boottime();
	return 0;
}

static irqreturn_t nafg_irq(int irq, void *data)
{
	struct mtk_battery *gm = data;

	if (fg_interrupt_check(gm) == false)
		return IRQ_HANDLED;
	nafg_irq_handler(gm);
	return IRQ_HANDLED;
}

/* ============================================================ */
/* battery plug out interrupt handler */
/* ============================================================ */
static irqreturn_t bat_plugout_irq(int irq, void *data)
{
	struct mtk_battery *gm = data;
	int is_bat_exist;

	is_bat_exist = gauge_get_int_property(GAUGE_PROP_BATTERY_EXIST);

	bm_err("[%s]is_bat %d miss:%d\n",
		__func__,
		is_bat_exist, gm->plug_miss_count);

	if (fg_interrupt_check(gm) == false)
		return IRQ_HANDLED;

	/* avoid battery plug status mismatch case*/
	if (is_bat_exist == 1) {
		fg_int_event(gm, EVT_INT_BAT_PLUGOUT);
		gm->plug_miss_count++;

		bm_err("[%s]is_bat %d miss:%d\n",
			__func__,
			is_bat_exist, gm->plug_miss_count);

		if (gm->plug_miss_count >= 3) {
			disable_gauge_irq(gm->gauge, BAT_PLUGOUT_IRQ);
			bm_err("[%s]disable FG_BAT_PLUGOUT\n",
				__func__);
			gm->disable_plug_int = 1;
		}
	}

	if (is_bat_exist == 0) {
		gm->bs_data.bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
		wakeup_fg_algo(gm, FG_INTR_BAT_PLUGOUT);
		battery_update(gm);
		fg_int_event(gm, EVT_INT_BAT_PLUGOUT);
		kernel_power_off();
	}
	return IRQ_HANDLED;
}

/* ============================================================ */
/* zcv interrupt handler */
/* ============================================================ */
static irqreturn_t zcv_irq(int irq, void *data)
{
	struct mtk_battery *gm = data;
	int fg_coulomb = 0;
	int zcv_intr_en = 0;
	int zcv_intr_curr = 0;
	int zcv;

	if (fg_interrupt_check(gm) == false)
		return IRQ_HANDLED;

	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
	zcv_intr_curr = gauge_get_int_property(GAUGE_PROP_ZCV_CURRENT);
	zcv = gauge_get_int_property(GAUGE_PROP_ZCV);
	bm_err("[%s] car:%d zcv_curr:%d zcv:%d, slp_cur_avg:%d\n",
		__func__,
		fg_coulomb, zcv_intr_curr, zcv,
		gm->fg_cust_data.sleep_current_avg);

	if (abs(zcv_intr_curr) < gm->fg_cust_data.sleep_current_avg) {
		wakeup_fg_algo(gm, FG_INTR_FG_ZCV);
		zcv_intr_en = 0;
		gauge_set_property(GAUGE_PROP_ZCV_INTR_EN, zcv_intr_en);
	}

	fg_int_event(gm, EVT_INT_ZCV);
	sw_check_bat_plugout(gm);
	return IRQ_HANDLED;
}

/* ============================================================ */
/* battery cycle interrupt handler */
/* ============================================================ */
void fg_sw_bat_cycle_accu(struct mtk_battery *gm)
{
	int diff_car = 0, tmp_car = 0, tmp_ncar = 0, tmp_thr = 0;
	int fg_coulomb = 0, gauge_version;

	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
	tmp_car = gm->bat_cycle_car;
	tmp_ncar = gm->bat_cycle_ncar;

	diff_car = fg_coulomb - gm->bat_cycle_car;

	if (diff_car > 0) {
		bm_err("[%s]drop diff_car\n", __func__);
		gm->bat_cycle_car = fg_coulomb;
	} else {
		gm->bat_cycle_ncar = gm->bat_cycle_ncar + abs(diff_car);
		gm->bat_cycle_car = fg_coulomb;
	}

	gauge_set_property(GAUGE_PROP_HW_INFO, 0);
	bm_err("[%s]car[o:%d n:%d],diff_car:%d,ncar[o:%d n:%d hw:%d] thr %d\n",
		__func__,
		tmp_car, fg_coulomb, diff_car,
		tmp_ncar, gm->bat_cycle_ncar, gm->gauge->fg_hw_info.ncar,
		gm->bat_cycle_thr);

	gauge_version = gauge_get_int_property(GAUGE_PROP_HW_VERSION);
	if (gauge_version >= GAUGE_HW_V1000 &&
		gauge_version < GAUGE_HW_V2000) {
		if (gm->bat_cycle_thr > 0 &&
			gm->bat_cycle_ncar >= gm->bat_cycle_thr) {
			tmp_ncar = gm->bat_cycle_ncar;
			tmp_thr = gm->bat_cycle_thr;

			gm->bat_cycle_ncar = 0;
			wakeup_fg_algo(gm, FG_INTR_BAT_CYCLE);
			bm_err("[fg_cycle_int_handler] ncar:%d thr:%d\n",
				tmp_ncar, tmp_thr);
		}
	}
}

static irqreturn_t cycle_irq(int irq, void *data)
{
	struct mtk_battery *gm = data;

	if (fg_interrupt_check(gm) == false)
		return IRQ_HANDLED;

	disable_gauge_irq(gm->gauge, FG_N_CHARGE_L_IRQ);
	wakeup_fg_algo(gm, FG_INTR_BAT_CYCLE);
	fg_int_event(gm, EVT_INT_BAT_CYCLE);
	sw_check_bat_plugout(gm);

	return IRQ_HANDLED;
}

/* ============================================================ */
/* hw iavg interrupt handler */
/* ============================================================ */
static irqreturn_t iavg_h_irq(int irq, void *data)
{
	struct mtk_battery *gm = data;

	if (fg_interrupt_check(gm) == false)
		return IRQ_HANDLED;
	gm->gauge->hw_status.iavg_intr_flag = 0;
	disable_gauge_irq(gm->gauge, FG_IAVG_H_IRQ);
	disable_gauge_irq(gm->gauge, FG_IAVG_L_IRQ);

	bm_debug("[%s]iavg_intr_flag %d\n",
		__func__,
		gm->gauge->hw_status.iavg_intr_flag);
	wakeup_fg_algo(gm, FG_INTR_IAVG);
	fg_int_event(gm, EVT_INT_IAVG);
	sw_check_bat_plugout(gm);
	return IRQ_HANDLED;
}

static irqreturn_t iavg_l_irq(int irq, void *data)
{
	struct mtk_battery *gm = data;

	if (fg_interrupt_check(gm) == false)
		return IRQ_HANDLED;
	gm->gauge->hw_status.iavg_intr_flag = 0;
	disable_gauge_irq(gm->gauge, FG_IAVG_H_IRQ);
	disable_gauge_irq(gm->gauge, FG_IAVG_L_IRQ);

	bm_debug("[%s]iavg_intr_flag %d\n",
		__func__,
		gm->gauge->hw_status.iavg_intr_flag);
	wakeup_fg_algo(gm, FG_INTR_IAVG);
	fg_int_event(gm, EVT_INT_IAVG);
	sw_check_bat_plugout(gm);
	return IRQ_HANDLED;
}

/* ============================================================ */
/* bat temp interrupt handler */
/* ============================================================ */
static irqreturn_t bat_temp_irq(int irq, void *data)
{
	struct mtk_battery *gm = data;

	if (fg_interrupt_check(gm) == false)
		return IRQ_HANDLED;

	bm_debug("[%s]\n", __func__);
	fg_bat_temp_int_internal(gm);

	return IRQ_HANDLED;
}

/* ============================================================ */
/* charge full interrupt handler */
/* ============================================================ */
void notify_fg_chr_full(struct mtk_battery *gm)
{
	ktime_t ctime = 0, dtime = 0, pre_time = 0;
	struct timespec64 difftime;

	ctime = ktime_get_boottime();
	pre_time = timespec64_to_ktime(gm->chr_full_handler_time);
	dtime = ktime_sub(ctime, pre_time);
	difftime = ktime_to_timespec64(dtime);

	if (ctime <= 10 || difftime.tv_sec >= 10) {
		gm->chr_full_handler_time = ktime_to_timespec64(ctime);
		bm_err("[fg_chr_full_int_handler]\n");
		wakeup_fg_algo(gm, FG_INTR_CHR_FULL);
		fg_int_event(gm, EVT_INT_CHR_FULL);
	}
}


/* ============================================================ */
/* sw iavg */
/* ============================================================ */
static void sw_iavg_init(struct mtk_battery *gm)
{
	int bat_current = 0;

	gm->sw_iavg_time = ktime_get_boottime();

	gm->sw_iavg_car = gauge_get_int_property(GAUGE_PROP_COULOMB);

	/* BAT_DISCHARGING = 0 */
	/* BAT_CHARGING = 1 */
	bat_current = gauge_get_int_property(GAUGE_PROP_BATTERY_CURRENT);

	gm->sw_iavg = bat_current;

	gm->sw_iavg_ht = gm->sw_iavg + gm->sw_iavg_gap;
	gm->sw_iavg_lt = gm->sw_iavg - gm->sw_iavg_gap;

	bm_err("%s %d, current:%d\n", __func__, gm->sw_iavg, bat_current);
}

void fg_update_sw_iavg(struct mtk_battery *gm)
{
	int fg_coulomb;
	int version;
	ktime_t ctime = 0, dtime = 0;
	struct timespec64 diff;

	ctime = ktime_get_boottime();
	dtime = ktime_sub(ctime, gm->sw_iavg_time);
	diff = ktime_to_timespec64(dtime);

	bm_debug("[%s]diff time:%ld\n", __func__, diff.tv_sec);
	if (diff.tv_sec >= 60) {
		fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
#if defined(__LP64__) || defined(_LP64)
		gm->sw_iavg = (fg_coulomb - gm->sw_iavg_car)
			* 3600 / diff.tv_sec;
#else
		if (diff.tv_sec < 65535)
			gm->sw_iavg = (fg_coulomb - gm->sw_iavg_car)
			* 3600 / (int)(diff.tv_sec);
		else {
			gm->sw_iavg = 0;
			bm_err("[%s]diff.tv_sec:%d\n", __func__, diff.tv_sec);
		}
#endif
		gm->sw_iavg_time = ctime;
		gm->sw_iavg_car = fg_coulomb;
		version = gauge_get_int_property(GAUGE_PROP_HW_VERSION);
		if (gm->sw_iavg >= gm->sw_iavg_ht
			|| gm->sw_iavg <= gm->sw_iavg_lt) {
			gm->sw_iavg_ht = gm->sw_iavg + gm->sw_iavg_gap;
			gm->sw_iavg_lt = gm->sw_iavg - gm->sw_iavg_gap;
			if (version < GAUGE_HW_V2000)
				wakeup_fg_algo(gm, FG_INTR_IAVG);
		}
		bm_debug("[%s]time:%ld car:%d %d iavg:%d ht:%d lt:%d gap:%d\n",
			__func__,
			diff.tv_sec, fg_coulomb, gm->sw_iavg_car, gm->sw_iavg,
			gm->sw_iavg_ht, gm->sw_iavg_lt, gm->sw_iavg_gap);
	}

}

int wakeup_fg_daemon(unsigned int flow_state, int cmd, int para1)
{
	static struct mtk_battery *gm;
	char intr_name[32];

	if (gm == NULL)
		gm = get_mtk_battery();

	if (gm->fgd_pid != 0) {
		struct fgd_nl_msg_t *fgd_msg;
		int size = FGD_NL_HDR_LEN + sizeof(flow_state);

		if (size > (PAGE_SIZE << 1))
			fgd_msg = vmalloc(size);
		else {
			if (in_interrupt())
				fgd_msg = kmalloc(size, GFP_ATOMIC);
			else
				fgd_msg = kmalloc(size, GFP_KERNEL);
		}

		if (fgd_msg == NULL) {
			if (size > PAGE_SIZE)
				fgd_msg = vmalloc(size);

			if (fgd_msg == NULL)
				return -1;
		}
		Intr_Number_to_Name(intr_name, flow_state);

		bm_debug("[%s] malloc size=%d pid=%d cmd:[%s],0x%x,cmd:%d,para1:%d\n",
			__func__,
			size, gm->fgd_pid, intr_name, flow_state, cmd, para1);
		memset(fgd_msg, 0, size);
		fgd_msg->fgd_cmd = FG_DAEMON_CMD_NOTIFY_DAEMON;
		fgd_msg->fgd_subcmd = cmd;
		fgd_msg->fgd_subcmd_para1 = para1;
		memcpy(fgd_msg->fgd_data, &flow_state, sizeof(flow_state));
		fgd_msg->fgd_data_len += sizeof(flow_state);
		mtk_battery_send_to_user(gm, 0, fgd_msg);

		kvfree(fgd_msg);

		return 0;
	} else
		return -1;

}

void fg_drv_update_daemon(struct mtk_battery *gm)
{

	int fg_current_iavg = 0;
	bool valid = 0;

	fg_update_sw_iavg(gm);
	fg_nafg_monitor(gm);

	fg_current_iavg = gauge_get_average_current(gm, &valid);

	bm_err("[%s]ui_ht_gap:%d ui_lt_gap:%d sw_iavg:%d %d %d nafg_m:%d %d %d\n",
		__func__,
		gm->uisoc_int_ht_gap, gm->uisoc_int_lt_gap,
		gm->sw_iavg, fg_current_iavg, valid,
		gm->last_nafg_cnt, gm->is_nafg_broken, gm->disable_nafg_int);

	wakeup_fg_algo_cmd(
		gm,
		FG_INTR_KERNEL_CMD,
		FG_KERNEL_CMD_DUMP_REGULAR_LOG,
		0);
}

static void mtk_battery_shutdown(struct mtk_battery *gm)
{
	int fg_coulomb = 0;
	int shut_car_diff = 0;
	int verify_car;

	fg_coulomb = gauge_get_int_property(GAUGE_PROP_COULOMB);
	if (gm->d_saved_car != 0) {
		shut_car_diff = fg_coulomb - gm->d_saved_car;
		gauge_set_property(GAUGE_PROP_SHUTDOWN_CAR, shut_car_diff);
		/* ready for writing to PMIC_RG */
	}
	verify_car = gauge_get_int_property(GAUGE_PROP_SHUTDOWN_CAR);

	bm_err("******** %s!! car=[o:%d,new:%d,diff:%d v:%d]********\n",
		__func__,
		gm->d_saved_car, fg_coulomb, shut_car_diff, verify_car);

}

void enable_bat_temp_det(bool en)
{
	gauge_set_property(GAUGE_PROP_BAT_TEMP_FROZE_EN, !en);
}

static int mtk_battery_suspend(struct mtk_battery *gm, pm_message_t state)
{
	int version;

	bm_err("******** %s!! iavg=%d ***GM3 disable:%d %d %d %d tmp_intr:%d***\n",
		__func__,
		gm->gauge->hw_status.iavg_intr_flag,
		gm->disableGM30,
		gm->fg_cust_data.disable_nafg,
		gm->ntc_disable_nafg,
		gm->cmd_disable_nafg,
		gm->enable_tmp_intr_suspend);

	if (gm->enable_tmp_intr_suspend == 0) {
		gauge_set_property(GAUGE_PROP_EN_BAT_TMP_LT, 0);
		gauge_set_property(GAUGE_PROP_EN_BAT_TMP_HT, 0);
		enable_bat_temp_det(0);
	}

	version = gauge_get_int_property(GAUGE_PROP_HW_VERSION);

	if (version >= GAUGE_HW_V2000
		&& gm->gauge->hw_status.iavg_intr_flag == 1) {
		disable_gauge_irq(gm->gauge, FG_IAVG_H_IRQ);
		if (gm->gauge->hw_status.iavg_lt > 0)
			disable_gauge_irq(gm->gauge, FG_IAVG_L_IRQ);
	}
	return 0;
}

static int mtk_battery_resume(struct mtk_battery *gm)
{
	int version;

	bm_err("******** %s!! iavg=%d ***GM3 disable:%d %d %d %d***\n",
		__func__,
		gm->gauge->hw_status.iavg_intr_flag,
		gm->disableGM30,
		gm->fg_cust_data.disable_nafg,
		gm->ntc_disable_nafg,
		gm->cmd_disable_nafg);

	version = gauge_get_int_property(GAUGE_PROP_HW_VERSION);
	if (version >=
		GAUGE_HW_V2000
		&& gm->gauge->hw_status.iavg_intr_flag == 1) {
		enable_gauge_irq(gm->gauge, FG_IAVG_H_IRQ);
		if (gm->gauge->hw_status.iavg_lt > 0)
			enable_gauge_irq(gm->gauge, FG_IAVG_L_IRQ);
	}
	/* reset nafg monitor time to avoid suspend for too long case */
	gm->last_nafg_update_time = ktime_get_boottime();

	fg_update_sw_iavg(gm);

	if (gm->enable_tmp_intr_suspend == 0) {
		gauge_set_property(GAUGE_PROP_EN_BAT_TMP_LT, 1);
		gauge_set_property(GAUGE_PROP_EN_BAT_TMP_HT, 1);
	enable_bat_temp_det(1);
	}

	return 0;
}

bool is_daemon_support(struct mtk_battery *gm)
{
	bool is_support = true;

	pr_notice("%s: is_support = %d\n", __func__, is_support);
	return is_support;
}

int mtk_battery_daemon_init(struct platform_device *pdev)
{
	int ret;
	int hw_version;
	struct mtk_battery *gm;
	struct mtk_gauge *gauge;
	struct fuel_gauge_custom_data *fg_cust_data;

	gauge = dev_get_drvdata(&pdev->dev);
	gm = gauge->gm;

	if (is_daemon_support(gm) == false)
		return -EIO;

	if (gm->mtk_battery_sk == NULL) {
		bm_err("[%s]netlink_kernel_create error\n", __func__);
		return -EIO;
	}

	gm->pl_two_sec_reboot = gauge_get_int_property(GAUGE_PROP_2SEC_REBOOT);
	gauge_set_property(GAUGE_PROP_2SEC_REBOOT, 0);

	hw_version = gauge_get_int_property(GAUGE_PROP_HW_VERSION);

	gm->shutdown = mtk_battery_shutdown;
	gm->suspend = mtk_battery_suspend;
	gm->resume = mtk_battery_resume;
	fg_cust_data = &gm->fg_cust_data;

	if (hw_version == GAUGE_NO_HW) {
		gm->gauge->sw_nafg_irq = nafg_irq_handler;
		gm->gauge->sw_vbat_h_irq = sw_vbat_h_irq_handler;
		gm->gauge->sw_vbat_l_irq = sw_vbat_l_irq_handler;
	}

	if (hw_version >= GAUGE_HW_V0500) {
		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[ZCV_IRQ],
		NULL, zcv_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_gauge_zcv",
		gm);

		if (hw_version == GAUGE_HW_V0500) {
			ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
			gm->gauge->irq_no[VBAT_H_IRQ],
			NULL, vbat_h_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
			"mtk_gauge_vbat_high",
			gm);

			ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
			gm->gauge->irq_no[VBAT_L_IRQ],
			NULL, vbat_l_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
			"mtk_gauge_vbat_low",
			gm);
		}
	}

	if (hw_version >= GAUGE_HW_V1000) {
		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[NAFG_IRQ],
		NULL, nafg_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_gauge_nafg",
		gm);
		if (hw_version < GAUGE_HW_V2000) {
#if IS_ENABLED(CONFIG_PMIC_LBAT_SERVICE)
			gm->lowbat_service =
			lbat_user_register("fuel gauge",
			fg_cust_data->vbat2_det_voltage3 / 10,
			fg_cust_data->vbat2_det_voltage1 / 10,
			fg_cust_data->vbat2_det_voltage2 / 10,
			fg_update_sw_low_battery_check);
			bm_err("[%s]lbat user_register done, %d,%d,%d\n",
				__func__, fg_cust_data->vbat2_det_voltage3,
				fg_cust_data->vbat2_det_voltage1,
				fg_cust_data->vbat2_det_voltage2);

			if (gm->lowbat_service != NULL) {
				lbat_user_set_debounce(gm->lowbat_service,
				fg_cust_data->vbat2_det_time * 1000,
				fg_cust_data->vbat2_det_counter,
				fg_cust_data->vbat2_det_time * 1000,
				fg_cust_data->vbat2_det_counter);

				bm_err("[%s]lbat set_debounce,time:%d,counter:%d\n",
					__func__, fg_cust_data->vbat2_det_time,
					fg_cust_data->vbat2_det_counter);
			} else
				bm_err("[%s]lowbat_service null\n", __func__);

#endif /* end of CONFIG_PMIC_LBAT_SERVICE */

			/* sw bat_cycle_car init, gm25 should start from 0 */
			gm->bat_cycle_car =
				gauge_get_int_property(GAUGE_PROP_COULOMB);
			if (gm->bat_cycle_car < 0)
				gm->bat_cycle_car = 0;
		}
	}

	if (hw_version >= GAUGE_HW_V2000) {
		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[BAT_PLUGOUT_IRQ],
		NULL, bat_plugout_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_gauge_bat_plugout",
		gm);

		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[FG_N_CHARGE_L_IRQ],
		NULL, cycle_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_cycle_zcv",
		gm);

		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[FG_IAVG_H_IRQ],
		NULL, iavg_h_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_gauge_iavg_h",
		gm);

		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[FG_IAVG_L_IRQ],
		NULL, iavg_l_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_gauge_iavg_l",
		gm);

		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[BAT_TMP_H_IRQ],
		NULL, bat_temp_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_bat_tmp_h",
		gm);

		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[BAT_TMP_L_IRQ],
		NULL, bat_temp_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_bat_tmp_l",
		gm);

		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[VBAT_H_IRQ],
		NULL, vbat_h_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_gauge_vbat_high",
		gm);

		ret = devm_request_threaded_irq(&gm->gauge->pdev->dev,
		gm->gauge->irq_no[VBAT_L_IRQ],
		NULL, vbat_l_irq,
		IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
		"mtk_gauge_vbat_low",
		gm);

	}

	sw_iavg_init(gm);
	mtk_battery_setup_files(pdev);
	return 0;
}

