/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __IMGSENSOR_H__
#define __IMGSENSOR_H__

#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/atomic.h>

#include "imgsensor_common.h"
#include "imgsensor_i2c.h"
#include "imgsensor_hw.h"

#define IMGSENSOR_FEATURE_PARA_LEN_MAX 128000

struct IMGSENSOR_STATUS {
	u32 reserved:31;
	u32 oc:1;
};

struct IMGSENSOR {
	dev_t dev_no;
	struct cdev *pcdev;
	struct class *pclass;

	struct IMGSENSOR_STATUS       status;
	struct IMGSENSOR_HW           hw;
	struct IMGSENSOR_SENSOR       sensor[IMGSENSOR_SENSOR_IDX_MAX_NUM];
	struct IMGSENSOR_SENSOR_LIST *psensor_list[MAX_NUM_OF_SUPPORT_SENSOR];

	atomic_t imgsensor_open_cnt;
	enum IMGSENSOR_RETURN (*mclk_set_drive_current)
		(void *pinstance,
		enum IMGSENSOR_SENSOR_IDX sensor_idx,
		enum ISP_DRIVING_CURRENT_ENUM drive_current);

#if defined(CONFIG_MTK_CAM_SECURITY_SUPPORT)
	unsigned long long imgsensor_sec_flag;
#endif
};

MINT32
imgsensor_sensor_open(struct IMGSENSOR_SENSOR *psensor);

MINT32
imgsensor_sensor_close(struct IMGSENSOR_SENSOR *psensor);

MUINT32
imgsensor_sensor_get_info(
		struct IMGSENSOR_SENSOR *psensor,
		MUINT32 ScenarioId,
		MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
		MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);

MUINT32
imgsensor_sensor_get_resolution(
		struct IMGSENSOR_SENSOR *psensor,
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);

MUINT32
imgsensor_sensor_feature_control(
		struct IMGSENSOR_SENSOR *psensor,
		MSDK_SENSOR_FEATURE_ENUM FeatureId,
		MUINT8 *pFeaturePara,
		MUINT32 *pFeatureParaLen);

MUINT32
imgsensor_sensor_control(
		struct IMGSENSOR_SENSOR *psensor,
		enum MSDK_SCENARIO_ID_ENUM ScenarioId);

extern struct IMGSENSOR_HW_CFG imgsensor_custom_config[];

#endif

