// SPDX-License-Identifier: GPL-2.0+
/*
 * Constant flag: always has the same value; hard-coded in the device tree
 *
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY UCLASS_CROS_VBOOT_FLAG

#include <common.h>
#include <dm.h>
#include <cros/vboot_flag.h>

DECLARE_GLOBAL_DATA_PTR;

/**
 * Private data for this driver
 *
 * @value: Value of the flag
 */
struct flag_const_priv {
	bool value;
};

static int flag_const_read(struct udevice *dev)
{
	struct flag_const_priv *priv = dev_get_priv(dev);

	return priv->value;
}

static int flag_const_ofdata_to_platdata(struct udevice *dev)
{
	struct flag_const_priv *priv = dev_get_priv(dev);
	u32 value;
	int ret;

	ret = dev_read_u32(dev, "value", &value);
	if (ret) {
		log_warning("Missing flag value in '%s'", dev->name);
		return ret;
	}
	priv->value = value != 0;

	return 0;
}

static const struct vboot_flag_ops flag_const_ops = {
	.read	= flag_const_read,
};

static const struct udevice_id flag_const_ids[] = {
	{ .compatible = "google,const-flag" },
	{ }
};

U_BOOT_DRIVER(flag_const_drv) = {
	.name		= "flag_const",
	.id		= UCLASS_CROS_VBOOT_FLAG,
	.of_match	= flag_const_ids,
	.ofdata_to_platdata	= flag_const_ofdata_to_platdata,
	.ops		= &flag_const_ops,
	.priv_auto_alloc_size	= sizeof(struct flag_const_priv),
};
