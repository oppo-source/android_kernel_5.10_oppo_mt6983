// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include "ccci_auxadc.h"
#include "ccci_fsm_internal.h"
#include "ccci_platform.h"
#include "modem_sys.h"
#include "md_sys1_platform.h"

signed int __weak battery_get_bat_voltage(void)
{
	pr_debug("[ccci/dummy] %s is not supported!\n", __func__);
	return 0;
}

#ifdef CCCI_KMODULE_ENABLE
int switch_sim_mode(int id, char *buf, unsigned int len)
{
	pr_debug("[ccci/dummy] %s is not supported!\n", __func__);
	return 0;
}

unsigned int get_sim_switch_type(void)
{
	pr_debug("[ccci/dummy] %s is not supported!\n", __func__);
	return 0;
}
#endif

static int fsm_md_data_ioctl(int md_id, unsigned int cmd, unsigned long arg)
{
	int ret = 0, retry = 0;
	int data = 0;
	char buffer[64] = {0};
	unsigned int sim_slot_cfg[4] = {0};
	char ap_platform[5] = {0};
	int md_gen = 0;
	struct device_node *node = NULL;
	struct ccci_per_md *per_md_data = ccci_get_per_md_data(md_id);
	struct ccci_per_md *other_per_md_data
			= ccci_get_per_md_data(GET_OTHER_MD_ID(md_id));

	node = of_find_compatible_node(NULL, NULL,
		"mediatek,mddriver");
	of_property_read_u32(node,
		"mediatek,md_generation", &md_gen);

	switch (cmd) {
	case CCCI_IOC_GET_MD_PROTOCOL_TYPE:
		snprintf(buffer, sizeof(buffer), "%d",
			md_gen);
		snprintf((void *)ap_platform, sizeof(ap_platform), "%d",
			md_gen);
		if (copy_to_user((void __user *)arg,
			ap_platform, sizeof(ap_platform))) {
			CCCI_ERROR_LOG(md_id, FSM,
				"CCCI_IOC_GET_MD_PROTOCOL_TYPE: copy_from_user fail\n");
			return -EFAULT;
		}
		break;
	case CCCI_IOC_SEND_BATTERY_INFO:
		data = (int)battery_get_bat_voltage();
		CCCI_NORMAL_LOG(md_id, FSM, "get bat voltage %d\n", data);
		ret = ccci_port_send_msg_to_md(md_id, CCCI_SYSTEM_TX,
				MD_GET_BATTERY_INFO, data, 1);
		break;
	case CCCI_IOC_GET_EXT_MD_POST_FIX:
		if (copy_to_user((void __user *)arg,
				per_md_data->img_post_fix, IMG_POSTFIX_LEN)) {
			CCCI_ERROR_LOG(md_id, FSM,
				"CCCI_IOC_GET_EXT_MD_POST_FIX: copy_to_user fail\n");
			ret = -EFAULT;
		}
		break;

	case CCCI_IOC_DL_TRAFFIC_CONTROL:
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(unsigned int)))
			CCCI_ERROR_LOG(md_id, FSM,
				"CCCI_IOC_DL_TRAFFIC_CONTROL: copy_from_user fail\n");
		if (data == 1)
			;/* turn off downlink queue */
		else if (data == 0)
			;/* turn on donwlink queue */
		else
			;
		ret = 0;
		break;
	case CCCI_IOC_SET_MD_BOOT_MODE:
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(unsigned int))) {
			CCCI_ERROR_LOG(md_id, FSM,
				"CCCI_IOC_SET_MD_BOOT_MODE: copy_from_user fail\n");
			ret = -EFAULT;
		} else {
			CCCI_NORMAL_LOG(md_id, FSM,
				"set MD boot mode to %d\n", data);
			per_md_data->md_boot_mode = data;
			if (other_per_md_data)
				other_per_md_data->md_boot_mode = data;
		}
		break;
	case CCCI_IOC_GET_MD_BOOT_MODE:
		ret = put_user((unsigned int)per_md_data->md_boot_mode,
				(unsigned int __user *)arg);
		break;
	case CCCI_IOC_GET_MD_INFO:
		ret = put_user(
		(unsigned int)per_md_data->img_info[IMG_MD].img_info.version,
		(unsigned int __user *)arg);
		break;
	case CCCI_IOC_SET_BOOT_DATA:
		if (copy_from_user(&per_md_data->md_boot_data,
			(void __user *)arg,
			sizeof(per_md_data->md_boot_data))) {
			CCCI_ERROR_LOG(md_id, FSM,
				"CCCI_IOC_SET_BOOT_DATA: copy_from_user fail\n");
			ret = -EFAULT;
		} else {
			if (per_md_data->md_boot_data[MD_CFG_DUMP_FLAG]
				!= MD_DBG_DUMP_INVALID
				&&
				(per_md_data->md_boot_data[MD_CFG_DUMP_FLAG]
				& 1 << MD_DBG_DUMP_PORT)) {
				/*port traffic use 0x6000_000x
				 * as port dump flag
				 */
				ccci_port_set_traffic_flag(md_id,
				per_md_data->md_boot_data[MD_CFG_DUMP_FLAG]);
				per_md_data->md_boot_data[MD_CFG_DUMP_FLAG]
					= MD_DBG_DUMP_INVALID;
			}
			ret = ccci_md_set_boot_data(md_id,
					per_md_data->md_boot_data,
					ARRAY_SIZE(per_md_data->md_boot_data));
			if (ret < 0) {
				CCCI_ERROR_LOG(md_id, FSM,
					"ccci_set_md_boot_data return fail %d\n",
					ret);
				ret = -EFAULT;
			}
		}
		break;
	case CCCI_IOC_SIM_SWITCH:
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(unsigned int))) {
			CCCI_BOOTUP_LOG(md_id, FSM,
				"CCCI_IOC_SIM_SWITCH: copy_from_user fail\n");
			ret = -EFAULT;
		} else {
			switch_sim_mode(md_id, (char *)&data, sizeof(data));
			CCCI_BOOTUP_LOG(md_id, FSM,
				"CCCI_IOC_SIM_SWITCH(%x): %d\n", data, ret);
		}
		break;
	case CCCI_IOC_SIM_SWITCH_TYPE:
		data = get_sim_switch_type();
		CCCI_BOOTUP_LOG(md_id, FSM,
			"CCCI_IOC_SIM_SWITCH_TYPE: 0x%x\n", data);
		ret = put_user(data, (unsigned int __user *)arg);
		break;
	case CCCI_IOC_GET_SIM_TYPE:
		if (per_md_data->sim_type == 0xEEEEEEEE)
			CCCI_BOOTUP_LOG(md_id, FSM,
				"md has not send sim type yet(0x%x)",
				per_md_data->sim_type);
		else
			CCCI_BOOTUP_LOG(md_id, FSM,
				"md has send sim type(0x%x)",
				per_md_data->sim_type);
		ret = put_user(per_md_data->sim_type,
				(unsigned int __user *)arg);
		break;
	case CCCI_IOC_ENABLE_GET_SIM_TYPE:
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(unsigned int))) {
			CCCI_NORMAL_LOG(md_id, FSM,
				"CCCI_IOC_ENABLE_GET_SIM_TYPE: copy_from_user fail\n");
			ret = -EFAULT;
		} else {
			CCCI_NORMAL_LOG(md_id, FSM,
				"CCCI_IOC_ENABLE_GET_SIM_TYPE: 0x%x\n", data);
			ret = ccci_port_send_msg_to_md(md_id,
					CCCI_SYSTEM_TX, MD_SIM_TYPE, data, 1);
		}
		break;
	case CCCI_IOC_RELOAD_MD_TYPE:
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(unsigned int))) {
			CCCI_ERROR_LOG(md_id, FSM,
				"CCCI_IOC_RELOAD_MD_TYPE: copy_from_user fail\n");
			ret = -EFAULT;
		} else {
			ret = set_soc_md_rt_rat_by_idx(md_id, (unsigned int)data);
			CCCI_NORMAL_LOG(md_id, FSM,
				"CCCI_IOC_RELOAD_MD_TYPE: %d ret:%d\n", data, ret);
		}
		break;
	case CCCI_IOC_SET_MD_IMG_EXIST:
		if (copy_from_user(&per_md_data->md_img_exist,
				(void __user *)arg,
				sizeof(per_md_data->md_img_exist))) {
			CCCI_ERROR_LOG(md_id, FSM,
				"CCCI_IOC_SET_MD_IMG_EXIST: copy_from_user fail\n");
			ret = -EFAULT;
		}
		per_md_data->md_img_type_is_set = 1;
		CCCI_BOOTUP_LOG(md_id, FSM,
			"CCCI_IOC_SET_MD_IMG_EXIST: set done!\n");
		break;
	case CCCI_IOC_GET_MD_IMG_EXIST:
		data = get_md_img_type(md_id);
		if (data) {
			memset(&per_md_data->md_img_exist, 0,
				sizeof(per_md_data->md_img_exist));
			per_md_data->md_img_exist[0] = data;
			CCCI_BOOTUP_LOG(md_id, FSM,
				"LK md_type: %d, image num:1\n", data);
		} else {
			CCCI_BOOTUP_LOG(md_id, FSM,
				"CCCI_IOC_GET_MD_IMG_EXIST: waiting set\n");
			while (per_md_data->md_img_type_is_set == 0)
				msleep(200);
		}
		CCCI_BOOTUP_LOG(md_id, FSM,
			"CCCI_IOC_GET_MD_IMG_EXIST: waiting set done!\n");
		if (copy_to_user((void __user *)arg,
			&per_md_data->md_img_exist,
			sizeof(per_md_data->md_img_exist))) {
			CCCI_ERROR_LOG(md_id, FSM,
			"CCCI_IOC_GET_MD_IMG_EXIST: copy_to_user fail!\n");
			ret = -EFAULT;
		}
		break;
	case CCCI_IOC_GET_MD_TYPE:
		data = get_md_img_type(md_id);
		if (!data)
			data = 3; //MT6580 using this
		else {
			retry = 6000;
			do {
				data = get_soc_md_rt_rat_idx(md_id);
				if (data)
					break;
				msleep(50);
				retry--;
			} while (retry);
		}
		CCCI_NORMAL_LOG(md_id, FSM,
			"CCCI_IOC_GET_MD_TYPE: %d!\n", data);
		ret = put_user((unsigned int)data,
			(unsigned int __user *)arg);
		break;
	case CCCI_IOC_STORE_MD_TYPE:
		if (copy_from_user(&data, (void __user *)arg,
			sizeof(unsigned int))) {
			CCCI_BOOTUP_LOG(md_id, FSM,
			"CCCI_IOC_STORE_MD_TYPE: copy_from_user fail\n");
			ret = -EFAULT;
			break;
		}
		per_md_data->config.load_type_saving = data;

		CCCI_BOOTUP_LOG(md_id, FSM,
			"storing md type(%d) in kernel space!\n",
			per_md_data->config.load_type_saving);
		if (per_md_data->config.load_type_saving >= 1
			&& per_md_data->config.load_type_saving
			<= MAX_IMG_NUM) {
			if (per_md_data->config.load_type_saving
				!= per_md_data->config.load_type)
				CCCI_BOOTUP_LOG(md_id, FSM,
					"Maybe Wrong: md type storing not equal with current setting!(%d %d)\n",
					per_md_data->config.load_type_saving,
					per_md_data->config.load_type);
		} else {
			CCCI_BOOTUP_LOG(md_id, FSM,
				"store md type fail: invalid md type(0x%x)\n",
				per_md_data->config.load_type_saving);
			ret = -EFAULT;
		}
		if (ret == 0)
			fsm_monitor_send_message(md_id,
				CCCI_MD_MSG_STORE_NVRAM_MD_TYPE, 0);
		break;
	case CCCI_IOC_GET_MD_TYPE_SAVING:
		ret = put_user(per_md_data->config.load_type_saving,
				(unsigned int __user *)arg);
		break;
	case CCCI_IOC_SEND_ICUSB_NOTIFY:
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(unsigned int))) {
			CCCI_NORMAL_LOG(md_id, FSM,
				"CCCI_IOC_SEND_ICUSB_NOTIFY: copy_from_user fail\n");
			ret = -EFAULT;
		} else {
			ret = ccci_port_send_msg_to_md(md_id,
			CCCI_SYSTEM_TX, MD_ICUSB_NOTIFY, data, 1);
		}
		break;
	case CCCI_IOC_UPDATE_SIM_SLOT_CFG:
		if (copy_from_user(&sim_slot_cfg, (void __user *)arg,
				sizeof(sim_slot_cfg))) {
			CCCI_NORMAL_LOG(md_id, FSM,
				"CCCI_IOC_UPDATE_SIM_SLOT_CFG: copy_from_user fail\n");
			ret = -EFAULT;
		} else {
			int need_update;

			data = get_sim_switch_type();
			CCCI_NORMAL_LOG(md_id, FSM,
				"CCCI_IOC_UPDATE_SIM_SLOT_CFG get s0:%d s1:%d s2:%d s3:%d\n",
				sim_slot_cfg[0], sim_slot_cfg[1],
				sim_slot_cfg[2], sim_slot_cfg[3]);
			need_update = sim_slot_cfg[0];
			per_md_data->sim_setting.sim_mode = sim_slot_cfg[1];
			per_md_data->sim_setting.slot1_mode = sim_slot_cfg[2];
			per_md_data->sim_setting.slot2_mode = sim_slot_cfg[3];
			data = (((unsigned int)data << 16)
					| per_md_data->sim_setting.sim_mode);
			switch_sim_mode(md_id, (char *)&data, sizeof(data));
			fsm_monitor_send_message(md_id,
			CCCI_MD_MSG_CFG_UPDATE, need_update);
			ret = 0;
		}
		break;
	case CCCI_IOC_STORE_SIM_MODE:
		if (copy_from_user(&data, (void __user *)arg,
			sizeof(unsigned int))) {
			CCCI_NORMAL_LOG(md_id, FSM,
			"CCCI_IOC_STORE_SIM_MODE: copy_from_user fail\n");
			ret = -EFAULT;
			break;
		}
		CCCI_NORMAL_LOG(md_id, FSM,
		"store sim mode(%x) in kernel space!\n", data);
		if (per_md_data->sim_setting.sim_mode != data) {
			per_md_data->sim_setting.sim_mode = data;
			fsm_monitor_send_message(md_id,
			CCCI_MD_MSG_CFG_UPDATE, 1);
		} else {
			CCCI_ERROR_LOG(md_id, FSM,
			"same sim mode as last time(0x%x)\n", data);
		}
		break;
	case CCCI_IOC_GET_SIM_MODE:
		CCCI_NORMAL_LOG(md_id, FSM,
		"get sim mode ioctl called by %s\n", current->comm);
		ret = put_user(per_md_data->sim_setting.sim_mode,
		(unsigned int __user *)arg);
		break;
	case CCCI_IOC_GET_CFG_SETTING:
		if (copy_to_user((void __user *)arg,
			&per_md_data->sim_setting,
			sizeof(struct ccci_sim_setting))) {
			CCCI_NORMAL_LOG(md_id, FSM,
			"CCCI_IOC_GET_CFG_SETTING: copy_to_user fail\n");
			ret = -EFAULT;
		}
		break;
	case CCCI_IOC_GET_AT_CH_NUM:
		{
			unsigned int at_ch_num = 4; /*default value*/
			struct ccci_runtime_feature *rt_feature = NULL;

			rt_feature = ccci_md_get_rt_feature_by_id(md_id,
				AT_CHANNEL_NUM, 1);
			if (rt_feature)
				ret = ccci_md_parse_rt_feature(md_id,
				rt_feature, &at_ch_num, sizeof(at_ch_num));
			else
				CCCI_ERROR_LOG(md_id, FSM,
					"get AT_CHANNEL_NUM fail\n");

			CCCI_NORMAL_LOG(md_id, FSM,
				"get at_ch_num = %u\n", at_ch_num);
			ret = put_user(at_ch_num,
					(unsigned int __user *)arg);
			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}

long ccci_fsm_ioctl(int md_id, unsigned int cmd, unsigned long arg)
{
	struct ccci_fsm_ctl *ctl = fsm_get_entity_by_md_id(md_id);
	int ret = 0;
	enum MD_STATE_FOR_USER state_for_user;
	unsigned int data;
	char *VALID_USER = "ccci_mdinit";

	if (!ctl)
		return -EINVAL;

	switch (cmd) {
	case CCCI_IOC_GET_MD_STATE:
		state_for_user = ccci_fsm_get_md_state_for_user(md_id);
		ret = put_user((unsigned int)state_for_user,
				(unsigned int __user *)arg);

		break;
	case CCCI_IOC_GET_OTHER_MD_STATE:
		state_for_user =
		ccci_fsm_get_md_state_for_user(GET_OTHER_MD_ID(md_id));
		ret = put_user((unsigned int)state_for_user,
				(unsigned int __user *)arg);
		break;
	case CCCI_IOC_MD_RESET:
		CCCI_NORMAL_LOG(md_id, FSM,
			"MD reset ioctl called by %s\n", current->comm);
		ret = fsm_monitor_send_message(ctl->md_id,
			CCCI_MD_MSG_RESET_REQUEST, 0);
		fsm_monitor_send_message(GET_OTHER_MD_ID(ctl->md_id),
			CCCI_MD_MSG_RESET_REQUEST, 0);
		inject_md_status_event(md_id, MD_STA_EV_RESET_REQUEST,
					current->comm);
		break;
	case CCCI_IOC_FORCE_MD_ASSERT:
		CCCI_NORMAL_LOG(md_id, FSM,
			"MD force assert ioctl called by %s\n", current->comm);
		ret = ccci_md_force_assert(md_id,
			MD_FORCE_ASSERT_BY_USER_TRIGGER, NULL, 0);
		inject_md_status_event(md_id, MD_STA_EV_F_ASSERT_REQUEST,
					current->comm);
		break;
	case CCCI_IOC_SEND_STOP_MD_REQUEST:
		CCCI_NORMAL_LOG(md_id, FSM,
			"MD stop request ioctl called by %s\n", current->comm);
		ret = fsm_monitor_send_message(ctl->md_id,
			CCCI_MD_MSG_FORCE_STOP_REQUEST, 0);
		fsm_monitor_send_message(GET_OTHER_MD_ID(ctl->md_id),
			CCCI_MD_MSG_FORCE_STOP_REQUEST, 0);
		inject_md_status_event(md_id, MD_STA_EV_STOP_REQUEST,
					current->comm);
		break;
	case CCCI_IOC_SEND_START_MD_REQUEST:
		CCCI_NORMAL_LOG(md_id, FSM,
			"MD start request ioctl called by %s\n", current->comm);
		ret = fsm_monitor_send_message(ctl->md_id,
			CCCI_MD_MSG_FORCE_START_REQUEST, 0);
		fsm_monitor_send_message(GET_OTHER_MD_ID(ctl->md_id),
			CCCI_MD_MSG_FORCE_START_REQUEST, 0);
		inject_md_status_event(md_id, MD_STA_EV_START_REQUEST,
					current->comm);
		break;
	case CCCI_IOC_DO_START_MD:
		/* add check whether the user call md start ioctl is valid */
		if (strncmp(current->comm,
			VALID_USER, strlen(VALID_USER)) == 0) {
			CCCI_NORMAL_LOG(md_id, FSM,
				"MD start ioctl called by %s\n", current->comm);
			ret = fsm_append_command(ctl, CCCI_COMMAND_START, 0);
		} else {
			CCCI_ERROR_LOG(md_id, FSM,
			"drop invalid user:%s call MD start ioctl\n",
			current->comm);
		}
		break;
	case CCCI_IOC_DO_STOP_MD:
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(unsigned int))) {
			CCCI_ERROR_LOG(md_id, FSM,
				"CCCI_IOC_DO_STOP_MD: copy_from_user fail\n");
			ret = -EFAULT;
		} else {
			CCCI_NORMAL_LOG(md_id, FSM,
				"MD stop ioctl called by %s %d\n",
				current->comm, data);
			ret = fsm_append_command(ctl, CCCI_COMMAND_STOP,
					(data ? MD_FLIGHT_MODE_ENTER
					: MD_FLIGHT_MODE_NONE)
					== MD_FLIGHT_MODE_ENTER ?
					FSM_CMD_FLAG_FLIGHT_MODE : 0);
		}
		break;
	case CCCI_IOC_ENTER_DEEP_FLIGHT:
		CCCI_NORMAL_LOG(md_id, FSM,
		"MD enter flight mode ioctl called by %s\n", current->comm);
		ret = fsm_monitor_send_message(ctl->md_id,
				CCCI_MD_MSG_FLIGHT_STOP_REQUEST, 0);
		inject_md_status_event(md_id, MD_STA_EV_ENTER_FLIGHT_REQUEST,
					current->comm);
		break;
	case CCCI_IOC_LEAVE_DEEP_FLIGHT:
		CCCI_NORMAL_LOG(md_id, FSM,
		"MD leave flight mode ioctl called by %s\n", current->comm);
		__pm_wakeup_event(ctl->wakelock, jiffies_to_msecs(10 * HZ));
		ret = fsm_monitor_send_message(ctl->md_id,
				CCCI_MD_MSG_FLIGHT_START_REQUEST, 0);
		inject_md_status_event(md_id, MD_STA_EV_LEAVE_FLIGHT_REQUEST,
					current->comm);
		break;
	case CCCI_IOC_ENTER_DEEP_FLIGHT_ENHANCED:
		CCCI_NORMAL_LOG(md_id, FSM,
		"MD enter flight mode enhanced ioctl called by %s\n",
		current->comm);
		ret = fsm_monitor_send_message(ctl->md_id,
				CCCI_MD_MSG_FLIGHT_STOP_REQUEST, 0);
		fsm_monitor_send_message(GET_OTHER_MD_ID(ctl->md_id),
			CCCI_MD_MSG_FLIGHT_STOP_REQUEST, 0);
		inject_md_status_event(md_id, MD_STA_EV_ENTER_FLIGHT_E_REQUEST,
					current->comm);
		break;
	case CCCI_IOC_LEAVE_DEEP_FLIGHT_ENHANCED:
		CCCI_NORMAL_LOG(md_id, FSM,
		"MD leave flight mode enhanced ioctl called by %s\n",
		current->comm);
		__pm_wakeup_event(ctl->wakelock, jiffies_to_msecs(10 * HZ));
		ret = fsm_monitor_send_message(ctl->md_id,
				CCCI_MD_MSG_FLIGHT_START_REQUEST, 0);
		fsm_monitor_send_message(GET_OTHER_MD_ID(ctl->md_id),
			CCCI_MD_MSG_FLIGHT_START_REQUEST, 0);
		inject_md_status_event(md_id, MD_STA_EV_LEAVE_FLIGHT_E_REQUEST,
					current->comm);
		break;
	/* RILD nodify ccci power off md */
	case CCCI_IOC_RILD_POWER_OFF_MD:
		CCCI_NORMAL_LOG(md_id, FSM,
				"MD will power off ioctl called by %s\n",
				current->comm);
		inject_md_status_event(md_id, MD_STA_EV_RILD_POWEROFF_START,
				current->comm);
		break;
	case CCCI_IOC_SET_EFUN:
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(unsigned int))) {
			CCCI_ERROR_LOG(md_id, FSM,
				"set efun fail: copy_from_user fail\n");
			ret = -EFAULT;
			break;
		}
		CCCI_NORMAL_LOG(md_id, FSM, "EFUN set to %d\n", data);
		if (data == 0)
			ccci_md_soft_stop(md_id, data);
		else if (data != 0)
			ccci_md_soft_start(md_id, data);
		break;
	case CCCI_IOC_MDLOG_DUMP_DONE:
		CCCI_NORMAL_LOG(md_id, FSM,
		"MD logger dump done ioctl called by %s\n", current->comm);
		ctl->ee_ctl.mdlog_dump_done = 1;
		break;
	case CCCI_IOC_RESET_MD1_MD3_PCCIF:
		ccci_md_reset_pccif(md_id);
		break;
	case CCCI_IOC_GET_MD_EX_TYPE:
		ret = put_user((unsigned int)ctl->ee_ctl.ex_type,
				(unsigned int __user *)arg);
		CCCI_NORMAL_LOG(md_id, FSM,
			"get modem exception type=%d ret=%d\n",
			ctl->ee_ctl.ex_type, ret);
		break;
	default:
		ret = fsm_md_data_ioctl(md_id, cmd, arg);
		break;
	}
	return ret;
}

