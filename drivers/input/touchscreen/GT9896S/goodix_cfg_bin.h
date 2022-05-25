/*
 * Goodix Firmware Update Driver.
 *
 * Copyright (C) 2019 - 2020 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#ifndef _GOODIX_CFG_BIN_H_
#define _GOODIX_CFG_BIN_H_

#include "goodix_ts_core.h"

#define TS_DEFAULT_FIRMWARE  "gt9896s_firmware_"
#define TS_DEFAULT_CFG_BIN   "gt9896s_cfg_"
extern const char *gt9896s_firmware_buf;
extern const char *gt9896s_config_buf;
extern int gt9896s_find_touch_node;
extern char panel_firmware_buf[];
extern char panel_config_buf[];

#define TS_BIN_VERSION_START_INDEX	5
#define TS_BIN_VERSION_LEN	4
#define TS_CFG_BIN_HEAD_RESERVED_LEN	6
#define TS_CFG_OFFSET_LEN	2
#define TS_IC_TYPE_NAME_MAX_LEN	15
#define TS_CFG_BIN_HEAD_LEN (sizeof(struct gt9896s_cfg_bin_head) + TS_CFG_BIN_HEAD_RESERVED_LEN)
#define TS_PKG_CONST_INFO_LEN  (sizeof(struct gt9896s_cfg_pkg_const_info))
#define TS_PKG_REG_INFO_LEN (sizeof(struct gt9896s_cfg_pkg_reg_info))
#define TS_PKG_HEAD_LEN (TS_PKG_CONST_INFO_LEN + TS_PKG_REG_INFO_LEN)

/*cfg block definitin*/
#define TS_CFG_BLOCK_PID_LEN	8
#define TS_CFG_BLOCK_VID_LEN	8
#define TS_CFG_BLOCK_FW_MASK_LEN	9
#define TS_CFG_BLOCK_FW_PATCH_LEN	4
#define TS_CFG_BLOCK_RESERVED_LEN	9

#define TS_NORMAL_CFG 0x01
#define TS_HIGH_SENSE_CFG 0x03
#define TS_RQST_FW_RETRY_TIMES 2

#pragma pack(1)
struct gt9896s_cfg_pkg_reg {
	u16 addr;
	u8 reserved1;
	u8 reserved2;
};

struct gt9896s_cfg_pkg_const_info {
	u32 pkg_len;
	u8 ic_type[TS_IC_TYPE_NAME_MAX_LEN];
	u8 cfg_type;
	u8 sensor_id;
	u8 hw_pid[TS_CFG_BLOCK_PID_LEN];
	u8 hw_vid[TS_CFG_BLOCK_VID_LEN];
	u8 fw_mask[TS_CFG_BLOCK_FW_MASK_LEN];
	u8 fw_patch[TS_CFG_BLOCK_FW_PATCH_LEN];
	u16 x_res_offset;
	u16 y_res_offset;
	u16 trigger_offset;
};

struct gt9896s_cfg_pkg_reg_info {
	struct gt9896s_cfg_pkg_reg cfg_send_flag;
	struct gt9896s_cfg_pkg_reg version_base;
	struct gt9896s_cfg_pkg_reg pid;
	struct gt9896s_cfg_pkg_reg vid;
	struct gt9896s_cfg_pkg_reg sensor_id;
	struct gt9896s_cfg_pkg_reg fw_mask;
	struct gt9896s_cfg_pkg_reg fw_status;
	struct gt9896s_cfg_pkg_reg cfg_addr;
	struct gt9896s_cfg_pkg_reg esd;
	struct gt9896s_cfg_pkg_reg command;
	struct gt9896s_cfg_pkg_reg coor;
	struct gt9896s_cfg_pkg_reg gesture;
	struct gt9896s_cfg_pkg_reg fw_request;
	struct gt9896s_cfg_pkg_reg proximity;
	u8 reserved[TS_CFG_BLOCK_RESERVED_LEN];
};

struct gt9896s_cfg_bin_head {
	u32 bin_len;
	u8 checksum;
	u8 bin_version[TS_BIN_VERSION_LEN];
	u8 pkg_num;
};

#pragma pack()

struct gt9896s_cfg_package {
	struct gt9896s_cfg_pkg_const_info cnst_info;
	struct gt9896s_cfg_pkg_reg_info reg_info;
	const u8 *cfg;
	u32 pkg_len;
};



struct gt9896s_cfg_bin {
	unsigned char *bin_data;
	unsigned int bin_data_len;
	struct gt9896s_cfg_bin_head head;
	struct gt9896s_cfg_package *cfg_pkgs;
};

extern struct gt9896s_module gt9896s_modules;

int gt9896s_ts_stage2_init(struct gt9896s_ts_core *core_data);

int gt9896s_ts_core_release(struct gt9896s_ts_core *core_data);

#endif
