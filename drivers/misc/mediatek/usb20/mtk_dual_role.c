// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/usb/class-dual-role.h>
#include "musb_core.h"
#include "mtk_musb.h"

#if IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)
static enum dual_role_property mt_dual_role_props[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
	/* DUAL_ROLE_PROP_VCONN_SUPPLY, */
};

static int mt_dual_role_get_prop(struct dual_role_phy_instance *dual_role,
			enum dual_role_property prop, unsigned int *val)
{
	struct musb *musb = dev_get_drvdata(dual_role->dev.parent);
	int ret = 0;
	int mode, pr, dr;

	dev_info(musb->controller, "prop=%d, power=%d, is_host=%d",
			 prop, musb->power, is_host_active(musb));

	if (musb->power) {
		if (is_host_active(musb)) {
			mode = DUAL_ROLE_PROP_MODE_DFP;
			pr = DUAL_ROLE_PROP_PR_SRC;
			dr = DUAL_ROLE_PROP_DR_HOST;
		} else {
			mode = DUAL_ROLE_PROP_MODE_UFP;
			pr = DUAL_ROLE_PROP_PR_SNK;
			dr = DUAL_ROLE_PROP_DR_DEVICE;
		}
	} else {
		mode = DUAL_ROLE_PROP_MODE_NONE;
		pr = DUAL_ROLE_PROP_PR_NONE;
		dr = DUAL_ROLE_PROP_DR_NONE;
	}

	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
		*val = mode;
		break;
	case DUAL_ROLE_PROP_PR:
		*val = pr;
		break;
	case DUAL_ROLE_PROP_DR:
		*val = dr;
		break;
	case DUAL_ROLE_PROP_VCONN_SUPPLY:
		*val = DUAL_ROLE_PROP_VCONN_SUPPLY_NO;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int mt_dual_role_prop_is_writeable(
	struct dual_role_phy_instance *dual_role, enum dual_role_property prop)
{
	/* not support writeable */
	return 0;
}

static int mt_dual_role_set_prop(struct dual_role_phy_instance *dual_role,
			enum dual_role_property prop, const unsigned int *val)
{
	/* do nothing */
	return 0;
}

int mt_usb_dual_role_changed(struct musb *musb)
{
	if (musb->dr_usb)
		dual_role_instance_changed(musb->dr_usb);

	return 0;
}

int mt_usb_dual_role_init(struct musb *musb)
{
	struct dual_role_phy_desc *dual_desc;

	dual_desc = devm_kzalloc(musb->controller, sizeof(*dual_desc),
				GFP_KERNEL);

	if (!dual_desc)
		return -ENOMEM;

	dual_desc->name = "dual-role-usb20";
	dual_desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
	dual_desc->properties = mt_dual_role_props;
	dual_desc->num_properties = ARRAY_SIZE(mt_dual_role_props);
	dual_desc->get_property = mt_dual_role_get_prop;
	dual_desc->set_property = mt_dual_role_set_prop;
	dual_desc->property_is_writeable = mt_dual_role_prop_is_writeable;

	musb->dr_usb = devm_dual_role_instance_register(musb->controller,
							dual_desc);
	if (IS_ERR(musb->dr_usb)) {
		dev_info(musb->controller, "fail to register dual role usb\n");
		return -EINVAL;
	}
	return 0;
}
#endif /* CONFIG_DUAL_ROLE_USB_INTF */
