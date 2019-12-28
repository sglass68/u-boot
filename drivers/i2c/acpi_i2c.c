// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 Google LLC
 */

#include <common.h>
#include <acpi.h>
#include <dm.h>
#include <i2c.h>
#include <asm-generic/gpio.h>
#include <asm/acpigen.h>
#include <asm/acpi_device.h>

static bool acpi_i2c_add_gpios_to_crs(struct acpi_i2c_priv *priv)
{
	/*
	 * Return false if:
	 * 1. Request to explicitly disable export of GPIOs in CRS, or
	 * 2. Both reset and enable GPIOs are not provided.
	 */
	if (priv->disable_gpio_export_in_crs ||
	    (!dm_gpio_is_valid(&priv->reset_gpio) &&
	     !dm_gpio_is_valid(&priv->enable_gpio)))
		return false;

	return true;
}

static int acpi_i2c_write_gpio(struct gpio_desc *gpio, int *curindex)
{
	int ret;

	if (!dm_gpio_is_valid(gpio))
		return -ENOENT;

	acpi_device_write_gpio_desc(gpio);
	ret = *curindex;
	(*curindex)++;

	return ret;
}

int acpi_i2c_fill_ssdt(struct udevice *dev, struct acpi_ctx *ctx)
{
	struct acpi_i2c_priv *priv = dev_get_priv(dev);
	char scope[ACPI_DEVICE_PATH_MAX];
	struct acpi_dp *dsd = NULL;
	int curindex = 0;
	int reset_gpio_index = -1, enable_gpio_index = -1, irq_gpio_index = -1;
	int ret;

	printf("%s: start %s\n", __func__, dev->name);
	ret = acpi_device_scope(dev, scope, sizeof(scope));
	if (ret)
		return log_msg_ret("scope", ret);

	/* Device */
	acpigen_write_scope(scope);
	acpigen_write_device(priv->name);
	acpigen_write_name_string("_HID", priv->hid);
	if (priv->cid)
		acpigen_write_name_string("_CID", priv->cid);
	acpigen_write_name_integer("_UID", priv->uid);
	acpigen_write_name_string("_DDN", priv->desc);
	acpigen_write_sta(acpi_device_status(dev));

	/* Resources */
	acpigen_write_name("_CRS");
	acpigen_write_resourcetemplate_header();
	acpi_device_write_i2c_dev(dev);

	/* Use either Interrupt() or GpioInt() */
	if (dm_gpio_is_valid(&priv->irq_gpio)) {
		irq_gpio_index = acpi_i2c_write_gpio(&priv->irq_gpio,
						     &curindex);
	} else {
		ret = acpi_device_write_interrupt_irq(&priv->irq);
		if (ret)
			return log_msg_ret("irq", ret);
	}

	if (acpi_i2c_add_gpios_to_crs(priv)) {
		reset_gpio_index = acpi_i2c_write_gpio(&priv->reset_gpio,
							&curindex);
		enable_gpio_index = acpi_i2c_write_gpio(&priv->enable_gpio,
							&curindex);
	}
	acpigen_write_resourcetemplate_footer();

	/* Wake capabilities */
	if (priv->wake) {
		acpigen_write_name_integer("_S0W", 4);
		acpigen_write_prw(priv->wake, 3);
	}

	/* DSD */
	if (priv->probed || priv->property_count || priv->compat_string ||
	    reset_gpio_index >= 0 || enable_gpio_index >= 0 ||
	    irq_gpio_index >= 0) {
		const char *path = acpi_device_path(dev);

		printf("** path='%s\n", path);
		dsd = acpi_dp_new_table("_DSD");
		if (priv->compat_string)
			acpi_dp_add_string(dsd, "compatible",
					   priv->compat_string);
		if (priv->probed)
			acpi_dp_add_integer(dsd, "linux,probed", 1);
		if (irq_gpio_index >= 0)
			acpi_dp_add_gpio(dsd, "irq-gpios", path,
					 irq_gpio_index, 0,
					 priv->irq_gpio.flags &
					 GPIOD_ACTIVE_LOW ?
					 ACPI_GPIO_ACTIVE_LOW : 0);
		if (reset_gpio_index >= 0)
			acpi_dp_add_gpio(dsd, "reset-gpios", path,
					 reset_gpio_index, 0,
					 priv->reset_gpio.flags &
					 GPIOD_ACTIVE_LOW ?
					 ACPI_GPIO_ACTIVE_LOW : 0);
		if (enable_gpio_index >= 0)
			acpi_dp_add_gpio(dsd, "enable-gpios", path,
					 enable_gpio_index, 0,
					 priv->enable_gpio.flags &
					 GPIOD_ACTIVE_LOW ?
					 ACPI_GPIO_ACTIVE_LOW : 0);
		/* Add generic property list (not supported) */
		assert(!priv->property_count);
		acpi_dp_add_property_list(dsd, NULL, priv->property_count);
		acpi_dp_write(dsd);
	}

	/* Power Resource */
	if (priv->has_power_resource) {
		struct acpi_gpio reset_gpio, enable_gpio, stop_gpio;

		acpi_device_from_gpio_desc(&priv->reset_gpio, &reset_gpio);
		acpi_device_from_gpio_desc(&priv->enable_gpio, &enable_gpio);
		acpi_device_from_gpio_desc(&priv->stop_gpio, &stop_gpio);
		const struct acpi_power_res_params power_res_params = {
			&reset_gpio,
			priv->reset_delay_ms,
			priv->reset_off_delay_ms,
			&enable_gpio,
			priv->enable_delay_ms,
			priv->enable_off_delay_ms,
			&stop_gpio,
			priv->stop_delay_ms,
			priv->stop_off_delay_ms
		};
		acpi_device_add_power_res(&power_res_params);
	}

	acpigen_pop_len(); /* Device */
	acpigen_pop_len(); /* Scope */

	return 0;
}

int acpi_i2c_ofdata_to_platdata(struct udevice *dev)
{
	struct acpi_i2c_priv *priv = dev_get_priv(dev);
	int ret;

	gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset_gpio,
			     GPIOD_IS_OUT);
	gpio_request_by_name(dev, "enable-gpios", 0, &priv->enable_gpio,
			     GPIOD_IS_OUT);
	gpio_request_by_name(dev, "irq-gpios", 0, &priv->irq_gpio, GPIOD_IS_IN);
	gpio_request_by_name(dev, "stop-gpios", 0, &priv->stop_gpio,
			     GPIOD_IS_OUT);
	irq_get_by_index(dev, 0, &priv->irq);
	priv->hid = dev_read_string(dev, "acpi,hid");
	if (!priv->hid)
		return log_msg_ret("hid", -EINVAL);
	priv->cid = dev_read_string(dev, "acpi,cid");
	dev_read_u32(dev, "acpi,uid", &priv->uid);
	priv->desc = dev_read_string(dev, "acpi,desc");
	ret = acpi_device_name(dev, priv->name);
	if (ret)
		return log_msg_ret("name", ret);
	dev_read_u32(dev, "acpi,wake", &priv->wake);
	priv->probed = dev_read_bool(dev, "acpi,probed");
	priv->compat_string = dev_read_string(dev, "acpi,compatible");
	priv->has_power_resource = dev_read_bool(dev, "acpi,has-power-resource");

	return 0;
}

/* Use name specified in priv or build one from I2C address */
static int acpi_i2c_get_name(const struct udevice *dev, char *out_name)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	struct acpi_i2c_priv *priv = dev_get_priv(dev);

	if (priv->name)
		return acpi_return_name(out_name, priv->name);

	snprintf(out_name, ACPI_DEVICE_NAME_MAX, "D%03X", chip->chip_addr);

	return 0;
}

struct acpi_ops acpi_i2c_ops = {
	.fill_ssdt_generator	= acpi_i2c_fill_ssdt,
	.get_name		= acpi_i2c_get_name,
};
