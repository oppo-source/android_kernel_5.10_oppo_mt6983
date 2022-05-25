// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DRIVER_NAME "gt9772b"
#define GT9772B_I2C_SLAVE_ADDR 0x18

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define GT9772B_NAME				"gt9772b"
#define GT9772B_MAX_FOCUS_POS			1023
#define GT9772B_ORIGIN_FOCUS_POS		0
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define GT9772B_FOCUS_STEPS			1
#define GT9772B_SET_POSITION_ADDR		0x03

#define GT9772B_CMD_DELAY			0xff
#define GT9772B_CTRL_DELAY_US			5000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define GT9772B_MOVE_STEPS			100
#define GT9772B_MOVE_DELAY_US			5000

/* gt9772b device structure */
struct gt9772b_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
};

static inline struct gt9772b_device *to_gt9772b_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct gt9772b_device, ctrls);
}

static inline struct gt9772b_device *sd_to_gt9772b_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct gt9772b_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};


static int gt9772b_set_position(struct gt9772b_device *gt9772b, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gt9772b->sd);

	return i2c_smbus_write_word_data(client, GT9772B_SET_POSITION_ADDR,
					 swab16(val));
}

static int gt9772b_release(struct gt9772b_device *gt9772b)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;

	diff_dac = GT9772B_ORIGIN_FOCUS_POS - gt9772b->focus->val;

	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		GT9772B_MOVE_STEPS;

	val = gt9772b->focus->val;

	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (GT9772B_MOVE_STEPS*(-1)) : GT9772B_MOVE_STEPS);

		ret = gt9772b_set_position(gt9772b, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(GT9772B_MOVE_DELAY_US,
			     GT9772B_MOVE_DELAY_US + 1000);
	}

	// last step to origin
	ret = gt9772b_set_position(gt9772b, GT9772B_ORIGIN_FOCUS_POS);
	if (ret) {
		LOG_INF("%s I2C failure: %d",
			__func__, ret);
		return ret;
	}

	LOG_INF("-\n");

	return 0;
}

static int gt9772b_init(struct gt9772b_device *gt9772b)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gt9772b->sd);
	int ret = 0;

	LOG_INF("+\n");

	client->addr  = GT9772B_I2C_SLAVE_ADDR >> 1;
	ret = i2c_smbus_read_byte_data(client, 0x00);

	LOG_INF("Check HW version: %x\n", ret);

	ret = i2c_smbus_write_byte_data(client, 0xed, 0xab);

	LOG_INF("-\n");

	return 0;
}

/* Power handling */
static int gt9772b_power_off(struct gt9772b_device *gt9772b)
{
	int ret;

	LOG_INF("%s\n", __func__);

	ret = gt9772b_release(gt9772b);
	if (ret)
		LOG_INF("gt9772b release failed!\n");

	ret = regulator_disable(gt9772b->vin);
	if (ret)
		return ret;

	ret = regulator_disable(gt9772b->vdd);
	if (ret)
		return ret;

	if (gt9772b->vcamaf_pinctrl && gt9772b->vcamaf_off)
		ret = pinctrl_select_state(gt9772b->vcamaf_pinctrl,
					gt9772b->vcamaf_off);

	return ret;
}

static int gt9772b_power_on(struct gt9772b_device *gt9772b)
{
	int ret;

	LOG_INF("%s\n", __func__);

	ret = regulator_enable(gt9772b->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(gt9772b->vdd);
	if (ret < 0)
		return ret;

	if (gt9772b->vcamaf_pinctrl && gt9772b->vcamaf_on)
		ret = pinctrl_select_state(gt9772b->vcamaf_pinctrl,
					gt9772b->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(GT9772B_CTRL_DELAY_US, GT9772B_CTRL_DELAY_US + 100);

	ret = gt9772b_init(gt9772b);
	if (ret < 0)
		goto fail;

	return 0;

fail:
	regulator_disable(gt9772b->vin);
	regulator_disable(gt9772b->vdd);
	if (gt9772b->vcamaf_pinctrl && gt9772b->vcamaf_off) {
		pinctrl_select_state(gt9772b->vcamaf_pinctrl,
				gt9772b->vcamaf_off);
	}

	return ret;
}

static int gt9772b_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct gt9772b_device *gt9772b = to_gt9772b_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = gt9772b_set_position(gt9772b, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops gt9772b_vcm_ctrl_ops = {
	.s_ctrl = gt9772b_set_ctrl,
};

static int gt9772b_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct gt9772b_device *gt9772b = sd_to_gt9772b_vcm(sd);

	LOG_INF("%s\n", __func__);

	ret = gt9772b_power_on(gt9772b);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int gt9772b_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gt9772b_device *gt9772b = sd_to_gt9772b_vcm(sd);

	LOG_INF("%s\n", __func__);

	gt9772b_power_off(gt9772b);

	return 0;
}

static const struct v4l2_subdev_internal_ops gt9772b_int_ops = {
	.open = gt9772b_open,
	.close = gt9772b_close,
};

static const struct v4l2_subdev_ops gt9772b_ops = { };

static void gt9772b_subdev_cleanup(struct gt9772b_device *gt9772b)
{
	v4l2_async_unregister_subdev(&gt9772b->sd);
	v4l2_ctrl_handler_free(&gt9772b->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&gt9772b->sd.entity);
#endif
}

static int gt9772b_init_controls(struct gt9772b_device *gt9772b)
{
	struct v4l2_ctrl_handler *hdl = &gt9772b->ctrls;
	const struct v4l2_ctrl_ops *ops = &gt9772b_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	gt9772b->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, GT9772B_MAX_FOCUS_POS, GT9772B_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	gt9772b->sd.ctrl_handler = hdl;

	return 0;
}

static int gt9772b_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gt9772b_device *gt9772b;
	int ret;

	LOG_INF("%s\n", __func__);

	gt9772b = devm_kzalloc(dev, sizeof(*gt9772b), GFP_KERNEL);
	if (!gt9772b)
		return -ENOMEM;

	gt9772b->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(gt9772b->vin)) {
		ret = PTR_ERR(gt9772b->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	gt9772b->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(gt9772b->vdd)) {
		ret = PTR_ERR(gt9772b->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	gt9772b->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(gt9772b->vcamaf_pinctrl)) {
		ret = PTR_ERR(gt9772b->vcamaf_pinctrl);
		gt9772b->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		gt9772b->vcamaf_on = pinctrl_lookup_state(
			gt9772b->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(gt9772b->vcamaf_on)) {
			ret = PTR_ERR(gt9772b->vcamaf_on);
			gt9772b->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		gt9772b->vcamaf_off = pinctrl_lookup_state(
			gt9772b->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(gt9772b->vcamaf_off)) {
			ret = PTR_ERR(gt9772b->vcamaf_off);
			gt9772b->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&gt9772b->sd, client, &gt9772b_ops);
	gt9772b->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gt9772b->sd.internal_ops = &gt9772b_int_ops;

	ret = gt9772b_init_controls(gt9772b);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&gt9772b->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	gt9772b->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&gt9772b->sd);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	gt9772b_subdev_cleanup(gt9772b);
	return ret;
}

static int gt9772b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gt9772b_device *gt9772b = sd_to_gt9772b_vcm(sd);

	LOG_INF("%s\n", __func__);

	gt9772b_subdev_cleanup(gt9772b);

	return 0;
}

static const struct i2c_device_id gt9772b_id_table[] = {
	{ GT9772B_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, gt9772b_id_table);

static const struct of_device_id gt9772b_of_table[] = {
	{ .compatible = "mediatek,gt9772b" },
	{ },
};
MODULE_DEVICE_TABLE(of, gt9772b_of_table);

static struct i2c_driver gt9772b_i2c_driver = {
	.driver = {
		.name = GT9772B_NAME,
		.of_match_table = gt9772b_of_table,
	},
	.probe_new  = gt9772b_probe,
	.remove = gt9772b_remove,
	.id_table = gt9772b_id_table,
};

module_i2c_driver(gt9772b_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("GT9772B VCM driver");
MODULE_LICENSE("GPL v2");
