// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author Wy Chuang<wy.chuang@mediatek.com>
 */

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>

#include "adapter_class.h"

static struct class *adapter_class;

static ssize_t name_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct adapter_device *adapter_dev = to_adapter_device(dev);

	return snprintf(buf, 20, "%s\n",
		       adapter_dev->props.alias_name ?
		       adapter_dev->props.alias_name : "anonymous");
}

static void adapter_device_release(struct device *dev)
{
	struct adapter_device *adapter_dev = to_adapter_device(dev);

	kfree(adapter_dev);
}

int adapter_dev_get_property(struct adapter_device *adapter_dev,
	enum adapter_property sta)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->get_property)
		return adapter_dev->ops->get_property(adapter_dev, sta);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_get_property);

int adapter_dev_get_status(struct adapter_device *adapter_dev,
	struct adapter_status *sta)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->get_status)
		return adapter_dev->ops->get_status(adapter_dev, sta);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_get_status);

int adapter_dev_get_output(struct adapter_device *adapter_dev, int *mV, int *mA)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->get_output)
		return adapter_dev->ops->get_output(adapter_dev, mV, mA);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_get_output);

int adapter_dev_set_cap(struct adapter_device *adapter_dev,
	enum adapter_cap_type type,
	int mV, int mA)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->set_cap)
		return adapter_dev->ops->set_cap(adapter_dev, type, mV, mA);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_set_cap);


int adapter_dev_get_cap(struct adapter_device *adapter_dev,
	enum adapter_cap_type type,
	struct adapter_power_cap *cap)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
		adapter_dev->ops->get_cap)
		return adapter_dev->ops->get_cap(adapter_dev, type, cap);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_get_cap);

int adapter_dev_authentication(struct adapter_device *adapter_dev,
			       struct adapter_auth_data *data)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->authentication)
		return adapter_dev->ops->authentication(adapter_dev, data);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_authentication);

int adapter_dev_is_cc(struct adapter_device *adapter_dev, bool *cc)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->is_cc)
		return adapter_dev->ops->is_cc(adapter_dev, cc);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_is_cc);

int adapter_dev_set_wdt(struct adapter_device *adapter_dev, u32 ms)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->set_wdt)
		return adapter_dev->ops->set_wdt(adapter_dev, ms);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_set_wdt);

int adapter_dev_enable_wdt(struct adapter_device *adapter_dev, bool en)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->enable_wdt)
		return adapter_dev->ops->enable_wdt(adapter_dev, en);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_enable_wdt);

int adapter_dev_sync_volt(struct adapter_device *adapter_dev, u32 mV)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->sync_volt)
		return adapter_dev->ops->sync_volt(adapter_dev, mV);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_sync_volt);

int adapter_dev_send_hardreset(struct adapter_device *adapter_dev)
{
	if (adapter_dev != NULL && adapter_dev->ops != NULL &&
	    adapter_dev->ops->send_hardreset)
		return adapter_dev->ops->send_hardreset(adapter_dev);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(adapter_dev_send_hardreset);

static DEVICE_ATTR_RO(name);

static struct attribute *adapter_class_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};

static const struct attribute_group adapter_group = {
	.attrs = adapter_class_attrs,
};

static const struct attribute_group *adapter_groups[] = {
	&adapter_group,
	NULL,
};

int register_adapter_device_notifier(struct adapter_device *adapter_dev,
				struct notifier_block *nb)
{
	int ret = 0;

	if (!adapter_dev)
		return -ENODEV;

	ret = srcu_notifier_chain_register(&adapter_dev->evt_nh, nb);
	return ret;
}
EXPORT_SYMBOL(register_adapter_device_notifier);

int unregister_adapter_device_notifier(struct adapter_device *adapter_dev,
				struct notifier_block *nb)
{
	if (!adapter_dev)
		return -ENODEV;

	return srcu_notifier_chain_unregister(&adapter_dev->evt_nh, nb);
}
EXPORT_SYMBOL(unregister_adapter_device_notifier);

/**
 * adapter_device_register - create and register a new object of
 *   adapter_device class.
 * @name: the name of the new object
 * @parent: a pointer to the parent device
 * @devdata: an optional pointer to be stored for private driver use.
 * The methods may retrieve it by using adapter_get_data(adapter_dev).
 * @ops: the charger operations structure.
 *
 * Creates and registers new charger device. Returns either an
 * ERR_PTR() or a pointer to the newly allocated device.
 */
struct adapter_device *adapter_device_register(const char *name,
		struct device *parent, void *devdata,
		const struct adapter_ops *ops,
		const struct adapter_properties *props)
{
	struct adapter_device *adapter_dev;
	static struct lock_class_key key;
	struct srcu_notifier_head *head;
	int rc;

	pr_notice("%s: name=%s\n", __func__, name);
	adapter_dev = kzalloc(sizeof(*adapter_dev), GFP_KERNEL);
	if (!adapter_dev)
		return ERR_PTR(-ENOMEM);

	head = &adapter_dev->evt_nh;
	srcu_init_notifier_head(head);
	/* Rename srcu's lock to avoid LockProve warning */
	lockdep_init_map(&(&head->srcu)->dep_map, name, &key, 0);
	mutex_init(&adapter_dev->ops_lock);
	adapter_dev->dev.class = adapter_class;
	adapter_dev->dev.parent = parent;
	adapter_dev->dev.release = adapter_device_release;
	dev_set_name(&adapter_dev->dev, name);
	dev_set_drvdata(&adapter_dev->dev, devdata);

	/* Copy properties */
	if (props) {
		memcpy(&adapter_dev->props, props,
		       sizeof(struct adapter_properties));
	}
	rc = device_register(&adapter_dev->dev);
	if (rc) {
		kfree(adapter_dev);
		return ERR_PTR(rc);
	}
	adapter_dev->ops = ops;
	return adapter_dev;
}
EXPORT_SYMBOL(adapter_device_register);

/**
 * adapter_device_unregister - unregisters a switching charger device
 * object.
 * @adapter_dev: the switching charger device object to be unregistered
 * and freed.
 *
 * Unregisters a previously registered via adapter_device_register object.
 */
void adapter_device_unregister(struct adapter_device *adapter_dev)
{
	if (!adapter_dev)
		return;

	mutex_lock(&adapter_dev->ops_lock);
	adapter_dev->ops = NULL;
	mutex_unlock(&adapter_dev->ops_lock);
	device_unregister(&adapter_dev->dev);
}
EXPORT_SYMBOL(adapter_device_unregister);


static int adapter_match_device_by_name(struct device *dev,
	const void *data)
{
	const char *name = data;

	return strcmp(dev_name(dev), name) == 0;
}

struct adapter_device *get_adapter_by_name(const char *name)
{
	struct device *dev;

	if (!name)
		return (struct adapter_device *)NULL;
	dev = class_find_device(adapter_class, NULL, name,
				adapter_match_device_by_name);

	return dev ? to_adapter_device(dev) : NULL;

}
EXPORT_SYMBOL(get_adapter_by_name);

static void __exit adapter_class_exit(void)
{
	class_destroy(adapter_class);
}

static int __init adapter_class_init(void)
{
	adapter_class = class_create(THIS_MODULE, "Charging Adapter");
	if (IS_ERR(adapter_class)) {
		pr_notice("Unable to create Charging Adapter class; errno = %ld\n",
			PTR_ERR(adapter_class));
		return PTR_ERR(adapter_class);
	}
	adapter_class->dev_groups = adapter_groups;
	return 0;
}

module_init(adapter_class_init);
module_exit(adapter_class_exit);

MODULE_DESCRIPTION("Adapter Class Device");
MODULE_AUTHOR("Wy Chuang <wy.chuang@mediatek.com>");
MODULE_LICENSE("GPL");
