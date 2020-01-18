// SPDX-License-Identifier: GPL-2.0
/*
 * max98357a.c -- MAX98357A Audio driver
 *
 * Copyright 2019 Google LLC
 * Parts taken from coreboot
 */

#include <common.h>
#include <acpigen.h>
#include <acpi_device.h>
#include <audio_codec.h>
#include <dm.h>
#include <sound.h>
#include <asm-generic/gpio.h>
#include <dm/acpi.h>

struct max97357a_priv {
	struct gpio_desc sdmode_gpio;
};

static int max97357a_ofdata_to_platdata(struct udevice *dev)
{
	struct max97357a_priv *priv = dev_get_priv(dev);

	gpio_request_by_name(dev, "sdmode-gpios", 0, &priv->sdmode_gpio,
			     GPIOD_IS_IN);

	return 0;
}

static int max97357a_acpi_fill_ssdt(const struct udevice *dev,
				    struct acpi_ctx *ctx)
{
	struct max97357a_priv *priv = dev_get_priv(dev);
	char scope[ACPI_PATH_MAX];
	char name[ACPI_NAME_MAX];
	char path[ACPI_PATH_MAX];
	struct acpi_dp *dp;
	int ret;

	ret = acpi_device_scope(dev, scope, sizeof(scope));
	if (ret)
		return log_msg_ret("scope", ret);
	ret = acpi_device_name(dev, name);
	if (ret)
		return log_msg_ret("name", ret);

	/* Device */
	acpigen_write_scope(ctx, scope);
	acpigen_write_device(ctx, name);
	acpigen_write_name_string(ctx, "_HID", dev_read_string(dev, "acpi,hid"));
	acpigen_write_name_integer(ctx, "_UID", 0);
	acpigen_write_name_string(ctx, "_DDN", dev_read_string(dev, "acpi,desc"));
	acpigen_write_sta(ctx, acpi_device_status(dev));

	ret = acpi_device_write_gpio_desc(ctx, &priv->sdmode_gpio);
	if (ret)
		return log_msg_ret("gpio", ret);

	/* Resources */
	acpigen_write_name(ctx, "_CRS");
	acpigen_write_resourcetemplate_header(ctx);
	ret = acpi_device_write_gpio_desc(ctx, &priv->sdmode_gpio);
	if (ret)
		return log_msg_ret("gpio", ret);
	acpigen_write_resourcetemplate_footer(ctx);

	/* _DSD for devicetree properties */
	/* This points to the first pin in the first gpio entry in _CRS */
	ret = acpi_device_path(dev, path, sizeof(path));
	if (ret)
		return log_msg_ret("path", ret);
	dp = acpi_dp_new_table("_DSD");
	acpi_dp_add_gpio(dp, "sdmode-gpio", path, 0, 0,
			 priv->sdmode_gpio.flags & GPIOD_ACTIVE_LOW ?
			 ACPI_GPIO_ACTIVE_LOW : ACPI_GPIO_ACTIVE_HIGH);
	acpi_dp_add_integer(dp, "sdmode-delay",
			    dev_read_u32_default(dev, "sdmode-delay", 0));
	acpi_dp_write(ctx, dp);

	acpigen_pop_len(ctx); /* Device */
	acpigen_pop_len(ctx); /* Scope */

	return 0;
}

struct acpi_ops max97357a_acpi_ops = {
	.fill_ssdt	= max97357a_acpi_fill_ssdt,
};

static const struct audio_codec_ops max98357a_ops = {
};

static const struct udevice_id max98357a_ids[] = {
	{ .compatible = "maxim,max98357a" },
	{ }
};

U_BOOT_DRIVER(max98357a) = {
	.name		= "max98357a",
	.id		= UCLASS_AUDIO_CODEC,
	.of_match	= max98357a_ids,
	.ofdata_to_platdata	= max97357a_ofdata_to_platdata,
	.ops		= &max98357a_ops,
	acpi_ops_ptr(&max97357a_acpi_ops)
};
