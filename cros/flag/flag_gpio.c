// SPDX-License-Identifier: GPL-2.0+
/*
 * GPIO flag: read a GPIO to determine a flag value
 *
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY UCLASS_CROS_VBOOT_FLAG

#include <common.h>
#include <dm.h>
#include <asm/gpio.h>
#include <cros/vboot_flag.h>

/**
 * Private data for this driver
 *
 * @desc: GPIO containing the flag value
 */
struct flag_gpio_priv {
	struct gpio_desc desc;
};

static int flag_gpio_read(struct udevice *dev)
{
	struct flag_gpio_priv *priv = dev_get_priv(dev);

	return dm_gpio_get_value(&priv->desc);
}

static int flag_gpio_probe(struct udevice *dev)
{
	struct flag_gpio_priv *priv = dev_get_priv(dev);
	int ret;

	ret = gpio_request_by_name(dev, "gpio", 0, &priv->desc, GPIOD_IS_IN);
	if (ret)
		return ret;
#ifdef CONFIG_SANDBOX
	u32 value;

	if (!dev_read_u32(dev, "sandbox-value", &value)) {
		sandbox_gpio_set_value(priv->desc.dev, priv->desc.offset,
				       value);
		log_info("Sandbox gpio %s/%d = %d\n", dev->name,
			 priv->desc.offset, value);
	}
#endif

	return 0;
}

static const struct vboot_flag_ops flag_gpio_ops = {
	.read	= flag_gpio_read,
};

static const struct udevice_id flag_gpio_ids[] = {
	{ .compatible = "google,gpio-flag" },
	{ }
};

U_BOOT_DRIVER(flag_gpio_drv) = {
	.name		= "flag_gpio",
	.id		= UCLASS_CROS_VBOOT_FLAG,
	.of_match	= flag_gpio_ids,
	.probe		= flag_gpio_probe,
	.ops		= &flag_gpio_ops,
	.priv_auto_alloc_size	= sizeof(struct flag_gpio_priv),
};
