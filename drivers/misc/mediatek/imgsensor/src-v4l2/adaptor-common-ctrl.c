// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021 MediaTek Inc.

#include "kd_imgsensor_define_v4l2.h"
#include "imgsensor-user.h"
#include "adaptor.h"
#include "adaptor-common-ctrl.h"

#define USER_DESC_TO_IMGSENSOR_ENUM(DESC) \
				(DESC - VC_STAGGER_NE + \
				IMGSENSOR_STAGGER_EXPOSURE_LE)
#define IS_HDR_STAGGER(DESC) \
				((DESC >= VC_STAGGER_NE) && \
				 (DESC <= VC_STAGGER_SE))

int g_stagger_info(struct adaptor_ctx *ctx,
						  int scenario,
						  struct mtk_stagger_info *info)
{
	int ret = 0;
	struct mtk_mbus_frame_desc fd = {0};
	int hdr_cnt = 0;
	unsigned int i = 0;

	if (!info)
		return 0;

	if (info->scenario_id != SENSOR_SCENARIO_ID_NONE)
		scenario = info->scenario_id;

	// dev_info(ctx->dev, " %s scenario %d %d\n", __func__, scenario, info->scenario_id);

	ret = subdrv_call(ctx, get_frame_desc, scenario, &fd);

	if (!ret) {
		for (i = 0; i < fd.num_entries; ++i) {
			u16 udd =
				fd.entry[i].bus.csi2.user_data_desc;

			if (IS_HDR_STAGGER(udd)) {
				hdr_cnt++;
				info->order[i] = USER_DESC_TO_IMGSENSOR_ENUM(udd);
			}
		}
	}

	info->count = hdr_cnt;
	// dev_info(ctx->dev, " %s after %d %d\n", __func__, info->count, info->scenario_id);

	return ret;
}

u32 g_scenario_exposure_cnt(struct adaptor_ctx *ctx, int scenario)
{
	u32 result = 1, len = 0;
	union feature_para para;
	struct mtk_stagger_info info = {0};
	int ret = 0;

	para.u64[0] = scenario;
	para.u64[1] = 0;
	subdrv_call(ctx, feature_control,
		    SENSOR_FEATURE_GET_EXPOSURE_COUNT_BY_SCENARIO,
		    para.u8, &len);
	if (para.u64[1]) {
		result = (u32) para.u64[1];
		adaptor_logd(ctx, "scenario exp count = %u\n", result);
		return result;
	}
	info.scenario_id = SENSOR_SCENARIO_ID_NONE;
	ret = g_stagger_info(ctx, scenario, &info);
	if (!ret) {
		/* non-stagger mode, the info count would be 0, it's same as 1 */
		if (info.count == 0)
			info.count = 1;
		result = info.count;
	}
	adaptor_logd(ctx, "exp count by stagger info = %u\n", result);
	return result;
}

int g_stagger_scenario(struct adaptor_ctx *ctx,
							  int scenario,
							  struct mtk_stagger_target_scenario *info)
{
	int ret = 0;
	union feature_para para;
	u32 len;

	if (!ctx || !info)
		return 0;

	para.u64[0] = scenario;
	para.u64[1] = (u64)info->exposure_num;
	para.u64[2] = SENSOR_SCENARIO_ID_NONE;

	subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO,
		para.u8, &len);

	info->target_scenario_id = (u32)para.u64[2];

	return ret;
}

int g_max_exposure(struct adaptor_ctx *ctx,
				   int scenario,
				   struct mtk_stagger_max_exp_time *info)
{
	u32 len = 0;
	union feature_para para;

	para.u64[0] = scenario;
	para.u64[1] = (u64)info->exposure;
	para.u64[2] = 0;

	subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME,
		para.u8, &len);

	info->max_exp_time = (u32)para.u64[2];

	return 0;
}

int g_max_exposure_line(struct adaptor_ctx *ctx,
				   int scenario,
				   struct mtk_max_exp_line *info)
{
	u32 len = 0;
	union feature_para para;

	para.u64[0] = scenario;
	para.u64[1] = (u64)info->exposure;
	para.u64[2] = 0;

	subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_MAX_EXP_LINE,
		para.u8, &len);

	info->max_exp_line = (u32)para.u64[2];

	return 0;
}

u32 g_sensor_margin(struct adaptor_ctx *ctx, unsigned int scenario)
{
	union feature_para para;
	struct mtk_stagger_info info = {0};
	u32 len = 0;
	u32 mode_exp_cnt = 1;

	// para.u64[0] = ctx->cur_mode->id;
	para.u64[0] = scenario;
	para.u64[1] = 0;
	para.u64[2] = 0;
	subdrv_call(ctx, feature_control,
		    SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO,
		    para.u8, &len);
	info.scenario_id = SENSOR_SCENARIO_ID_NONE;

	// if (!g_stagger_info(ctx, ctx->cur_mode->id, &info))
	if (!g_stagger_info(ctx, scenario, &info))
		mode_exp_cnt = info.count;

	/* no vc info case, it is 1 exposure */
	if (mode_exp_cnt == 0)
		mode_exp_cnt = 1;

	return (para.u64[2] * mode_exp_cnt);
}

u32 g_sensor_fine_integ_line(struct adaptor_ctx *ctx,
	const unsigned int scenario)
{
	union feature_para para;
	u32 fine_integ_line = 0;
	u32 len = 0;

	// para.u64[0] = ctx->cur_mode->id;
	para.u64[0] = scenario;
	para.u64[1] = (u64)&fine_integ_line;

	subdrv_call(ctx, feature_control,
		SENSOR_FEATURE_GET_FINE_INTEG_LINE_BY_SCENARIO,
		para.u8, &len);

	return fine_integ_line;
}
