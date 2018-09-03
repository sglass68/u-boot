// SPDX-License-Identifier: GPL-2.0+
/*
 * A vboot flag controlled by a keypress (for use with sandbox)
 *
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY UCLASS_CROS_VBOOT_FLAG

#include <common.h>
#include <dm.h>
#include <log.h>
#include <asm/sdl.h>
#include <cros/vboot_flag.h>

/**
 * Private data for this driver
 *
 * @key: Key code to press on start-up to set this flag to 1. The key codes are
 *	defined in include/linux/input.h (e.g. KEY_MINUS)
 */
struct flag_key_priv {
	int key;
};

static int flag_key_read(struct udevice *dev)
{
	struct flag_key_priv *priv = dev_get_priv(dev);

	return sandbox_sdl_key_pressed(priv->key);
}

static int flag_key_probe(struct udevice *dev)
{
	struct flag_key_priv *priv = dev_get_priv(dev);
	u32 value;

	if (dev_read_u32(dev, "key", &value)) {
		log_warning("Missing 'key' property for '%s'\n", dev->name);

		return -EINVAL;
	}
	priv->key = value;

	return 0;
}

static const struct vboot_flag_ops flag_key_ops = {
	.read	= flag_key_read,
};

static const struct udevice_id flag_key_ids[] = {
	{ .compatible = "google,key-flag" },
	{ }
};

U_BOOT_DRIVER(flag_key_drv) = {
	.name		= "flag_key",
	.id		= UCLASS_CROS_VBOOT_FLAG,
	.of_match	= flag_key_ids,
	.probe		= flag_key_probe,
	.ops		= &flag_key_ops,
	.priv_auto_alloc_size	= sizeof(struct flag_key_priv),
};
