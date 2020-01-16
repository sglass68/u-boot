// SPDX-License-Identifier: GPL-2.0
/*
 * Generation of tables for particular device types
 *
 * Copyright 2019 Google LLC
 * Mostly taken from coreboot file of the same name
 */

#include <common.h>
#include <acpi_device.h>
#include <acpigen.h>
#include <dm.h>
#include <irq.h>
#include <usb.h>
#include <asm-generic/gpio.h>
#include <dm/acpi.h>

#define ACPI_DP_UUID		"daffd814-6eba-4d8c-8a91-bc9bbf4aa301"
#define ACPI_DP_CHILD_UUID	"dbb8e3e6-5886-4ba6-8795-1319f52a966b"
#define ACPI_DSM_I2C_HID_UUID	"3cdff6f7-4267-4555-ad05-b30a3d8938de"

/**
 * acpi_device_write_zero_len() - Write a placeholder word value
 *
 * @return pointer to the zero word (for fixing up later)
 */
static void *acpi_device_write_zero_len(struct acpi_ctx *ctx)
{
	char *p = acpigen_get_current(ctx);

	acpigen_emit_word(ctx, 0);

	return p;
}

/**
 * acpi_device_fill_from_len() - Fill in a length value
 *
 * This calculated the number of bytes since the provided @start and writes it
 * to @ptr, which was previous returned by acpi_device_write_zero_len().
 *
 * @ptr: Word to update
 * @start: Start address to count from to calculated the length
 */
static void acpi_device_fill_from_len(struct acpi_ctx *ctx, char *ptr,
				      char *start)
{
	uint16_t len = acpigen_get_current(ctx) - start;

	ptr[0] = len & 0xff;
	ptr[1] = (len >> 8) & 0xff;
}

/**
 * acpi_device_fill_len() - Fill in a length value, excluding the length itself
 *
 * Fill in the length field with the value calculated from after the 16bit
 * field to acpigen current. This is useful since the length value does not
 * include the length field itself.
 *
 * This calls acpi_device_fill_from_len() passing @ptr + 2 as @start
 *
 * @ptr: Word to update.
 */
static void acpi_device_fill_len(struct acpi_ctx *ctx, void *ptr)
{
	acpi_device_fill_from_len(ctx, ptr, ptr + sizeof(uint16_t));
}

int acpi_device_name(const struct udevice *dev, char *name)
{
	int ret;

	ret = acpi_get_name(dev, name);
	if (ret)
		return log_msg_ret("name", ret);

	return 0;
}

/**
 * acpi_device_path_fill() - Find the root device and build a path from there
 *
 * This recursively reaches back to the root device and progressively adds path
 * elements until the device is reached.
 *
 * @dev: Device to return path of
 * @buf: Buffer to hold the path
 * @buf_len: Length of buffer
 * @cur: Current position in the buffer
 * @return new position in buffer after adding @dev, or -ve on error
 */
static int acpi_device_path_fill(const struct udevice *dev, char *buf,
				 size_t buf_len, int cur)
{
	char name[ACPI_NAME_MAX];
	int next = 0;
	int ret;

	ret = acpi_device_name(dev, name);
	if (ret)
		return ret;

	/*
	 * Make sure this name segment will fit, including the path segment
	 * separator and possible NUL terminator if this is the last segment.
	 */
	if (!dev || (cur + strlen(name) + 2) > buf_len)
		return cur;

	/* Walk up the tree to the root device */
	if (dev_get_parent(dev)) {
		next = acpi_device_path_fill(dev_get_parent(dev), buf, buf_len,
					     cur);
		if (next < 0)
			return next;
	}

	/* Fill in the path from the root device */
	next += snprintf(buf + next, buf_len - next, "%s%s",
			 dev_get_parent(dev) && *name ? "." : "", name);

	return next;
}

int acpi_device_path(const struct udevice *dev, char *buf, int maxlen)
{
	int ret;

	ret = acpi_device_path_fill(dev, buf, maxlen, 0);
	if (ret < 0)
		return ret;

	return 0;
}

int acpi_device_scope(const struct udevice *dev, char *scope, int maxlen)
{
	int ret;

	if (!dev_get_parent(dev))
		return log_msg_ret("noparent", -EINVAL);

	ret = acpi_device_path_fill(dev_get_parent(dev), scope, maxlen, 0);
	if (ret < 0)
		return log_msg_ret("fill", ret);

	return 0;
}

int acpi_device_status(const struct udevice *dev)
{
	return ACPI_STATUS_DEVICE_ALL_ON;
}

/* ACPI 6.1 section 6.4.3.8.1 - GPIO Interrupt or I/O */
int acpi_device_write_gpio(struct acpi_ctx *ctx, const struct acpi_gpio *gpio)
{
	void *start, *desc_length;
	void *pin_table_offset, *vendor_data_offset, *resource_offset;
	uint16_t flags = 0;
	int pin;

	if (!gpio || gpio->type > ACPI_GPIO_TYPE_IO)
		return -EINVAL;

	start = acpigen_get_current(ctx);

	/* Byte 0: Descriptor Type */
	acpigen_emit_byte(ctx, ACPI_DESCRIPTOR_GPIO);

	/* Byte 1-2: Length (fill in later) */
	desc_length = acpi_device_write_zero_len(ctx);

	/* Byte 3: Revision ID */
	acpigen_emit_byte(ctx, ACPI_GPIO_REVISION_ID);

	/* Byte 4: GpioIo or GpioInt */
	acpigen_emit_byte(ctx, gpio->type);

	/*
	 * Byte 5-6: General Flags
	 *   [15:1]: 0 => Reserved
	 *      [0]: 1 => ResourceConsumer
	 */
	acpigen_emit_word(ctx, 1 << 0);

	switch (gpio->type) {
	case ACPI_GPIO_TYPE_INTERRUPT:
		/*
		 * Byte 7-8: GPIO Interrupt Flags
		 *   [15:5]: 0 => Reserved
		 *      [4]: Wake     (0=NO_WAKE   1=WAKE)
		 *      [3]: Sharing  (0=EXCLUSIVE 1=SHARED)
		 *    [2:1]: Polarity (0=HIGH      1=LOW     2=BOTH)
		 *      [0]: Mode     (0=LEVEL     1=EDGE)
		 */
		if (gpio->irq.mode == ACPI_IRQ_EDGE_TRIGGERED)
			flags |= 1 << 0;
		if (gpio->irq.shared == ACPI_IRQ_SHARED)
			flags |= 1 << 3;
		if (gpio->irq.wake == ACPI_IRQ_WAKE)
			flags |= 1 << 4;

		switch (gpio->irq.polarity) {
		case ACPI_IRQ_ACTIVE_HIGH:
			flags |= 0 << 1;
			break;
		case ACPI_IRQ_ACTIVE_LOW:
			flags |= 1 << 1;
			break;
		case ACPI_IRQ_ACTIVE_BOTH:
			flags |= 2 << 1;
			break;
		}
		break;

	case ACPI_GPIO_TYPE_IO:
		/*
		 * Byte 7-8: GPIO IO Flags
		 *   [15:4]: 0 => Reserved
		 *      [3]: Sharing  (0=EXCLUSIVE 1=SHARED)
		 *      [2]: 0 => Reserved
		 *    [1:0]: IO Restriction
		 *           0 => IoRestrictionNone
		 *           1 => IoRestrictionInputOnly
		 *           2 => IoRestrictionOutputOnly
		 *           3 => IoRestrictionNoneAndPreserve
		 */
		flags |= gpio->io_restrict & 3;
		if (gpio->io_shared)
			flags |= 1 << 3;
		break;
	}
	acpigen_emit_word(ctx, flags);

	/*
	 * Byte 9: Pin Configuration
	 *  0x01 => Default (no configuration applied)
	 *  0x02 => Pull-up
	 *  0x03 => Pull-down
	 *  0x04-0x7F => Reserved
	 *  0x80-0xff => Vendor defined
	 */
	acpigen_emit_byte(ctx, gpio->pull);

	/* Byte 10-11: Output Drive Strength in 1/100 mA */
	acpigen_emit_word(ctx, gpio->output_drive_strength);

	/* Byte 12-13: Debounce Timeout in 1/100 ms */
	acpigen_emit_word(ctx, gpio->interrupt_debounce_timeout);

	/* Byte 14-15: Pin Table Offset, relative to start */
	pin_table_offset = acpi_device_write_zero_len(ctx);

	/* Byte 16: Reserved */
	acpigen_emit_byte(ctx, 0);

	/* Byte 17-18: Resource Source Name Offset, relative to start */
	resource_offset = acpi_device_write_zero_len(ctx);

	/* Byte 19-20: Vendor Data Offset, relative to start */
	vendor_data_offset = acpi_device_write_zero_len(ctx);

	/* Byte 21-22: Vendor Data Length */
	acpigen_emit_word(ctx, 0);

	/* Fill in Pin Table Offset */
	acpi_device_fill_from_len(ctx, pin_table_offset, start);

	/* Pin Table, one word for each pin */
	for (pin = 0; pin < gpio->pin_count; pin++)
		acpigen_emit_word(ctx, gpio->pins[pin]);

	/* Fill in Resource Source Name Offset */
	acpi_device_fill_from_len(ctx, resource_offset, start);

	/* Resource Source Name String */
	acpigen_emit_string(ctx, gpio->resource);

	/* Fill in Vendor Data Offset */
	acpi_device_fill_from_len(ctx, vendor_data_offset, start);

	/* Fill in GPIO Descriptor Length (account for len word) */
	acpi_device_fill_len(ctx, desc_length);

	return 0;
}

int acpi_device_write_gpio_desc(struct acpi_ctx *ctx,
				const struct gpio_desc *desc)
{
	struct acpi_gpio gpio;
	int ret;

	ret = gpio_get_acpi(desc, &gpio);
	if (ret)
		return log_msg_ret("desc", ret);
	ret = acpi_device_write_gpio(ctx, &gpio);
	if (ret)
		return log_msg_ret("gpio", ret);

	return 0;
}

/* ACPI 6.1 section 6.4.3.8.2.1 - I2cSerialBus() */
static void acpi_device_write_i2c(struct acpi_ctx *ctx,
				  const struct acpi_i2c *i2c)
{
	void *desc_length, *type_length;

	/* Byte 0: Descriptor Type */
	acpigen_emit_byte(ctx, ACPI_DESCRIPTOR_SERIAL_BUS);

	/* Byte 1+2: Length (filled in later) */
	desc_length = acpi_device_write_zero_len(ctx);

	/* Byte 3: Revision ID */
	acpigen_emit_byte(ctx, ACPI_I2C_SERIAL_BUS_REVISION_ID);

	/* Byte 4: Resource Source Index is Reserved */
	acpigen_emit_byte(ctx, 0);

	/* Byte 5: Serial Bus Type is I2C */
	acpigen_emit_byte(ctx, ACPI_SERIAL_BUS_TYPE_I2C);

	/*
	 * Byte 6: Flags
	 *  [7:2]: 0 => Reserved
	 *    [1]: 1 => ResourceConsumer
	 *    [0]: 0 => ControllerInitiated
	 */
	acpigen_emit_byte(ctx, 1 << 1);

	/*
	 * Byte 7-8: Type Specific Flags
	 *   [15:1]: 0 => Reserved
	 *      [0]: 0 => 7bit, 1 => 10bit
	 */
	acpigen_emit_word(ctx, i2c->mode_10bit);

	/* Byte 9: Type Specific Revision ID */
	acpigen_emit_byte(ctx, ACPI_I2C_TYPE_SPECIFIC_REVISION_ID);

	/* Byte 10-11: I2C Type Data Length */
	type_length = acpi_device_write_zero_len(ctx);

	/* Byte 12-15: I2C Bus Speed */
	acpigen_emit_dword(ctx, i2c->speed);

	/* Byte 16-17: I2C Slave Address */
	acpigen_emit_word(ctx, i2c->address);

	/* Fill in Type Data Length */
	acpi_device_fill_len(ctx, type_length);

	/* Byte 18+: ResourceSource */
	acpigen_emit_string(ctx, i2c->resource);

	/* Fill in I2C Descriptor Length */
	acpi_device_fill_len(ctx, desc_length);
}

/* ACPI 6.1 section 6.4.3.8.2.2 - SpiSerialBus() */
void acpi_device_write_spi(struct acpi_ctx *ctx, const struct acpi_spi *spi)
{
	void *desc_length, *type_length;
	uint16_t flags = 0;

	/* Byte 0: Descriptor Type */
	acpigen_emit_byte(ctx, ACPI_DESCRIPTOR_SERIAL_BUS);

	/* Byte 1+2: Length (filled in later) */
	desc_length = acpi_device_write_zero_len(ctx);

	/* Byte 3: Revision ID */
	acpigen_emit_byte(ctx, ACPI_SPI_SERIAL_BUS_REVISION_ID);

	/* Byte 4: Resource Source Index is Reserved */
	acpigen_emit_byte(ctx, 0);

	/* Byte 5: Serial Bus Type is SPI */
	acpigen_emit_byte(ctx, ACPI_SERIAL_BUS_TYPE_SPI);

	/*
	 * Byte 6: Flags
	 *  [7:2]: 0 => Reserved
	 *    [1]: 1 => ResourceConsumer
	 *    [0]: 0 => ControllerInitiated
	 */
	acpigen_emit_byte(ctx, 1 << 1);

	/*
	 * Byte 7-8: Type Specific Flags
	 *   [15:2]: 0 => Reserveda
	 *      [1]: 0 => ActiveLow, 1 => ActiveHigh
	 *      [0]: 0 => FourWire, 1 => ThreeWire
	 */
	if (spi->wire_mode == SPI_3_WIRE_MODE)
		flags |= 1 << 0;
	if (spi->device_select_polarity == SPI_POLARITY_HIGH)
		flags |= 1 << 1;
	acpigen_emit_word(ctx, flags);

	/* Byte 9: Type Specific Revision ID */
	acpigen_emit_byte(ctx, ACPI_SPI_TYPE_SPECIFIC_REVISION_ID);

	/* Byte 10-11: SPI Type Data Length */
	type_length = acpi_device_write_zero_len(ctx);

	/* Byte 12-15: Connection Speed */
	acpigen_emit_dword(ctx, spi->speed);

	/* Byte 16: Data Bit Length */
	acpigen_emit_byte(ctx, spi->data_bit_length);

	/* Byte 17: Clock Phase */
	acpigen_emit_byte(ctx, spi->clock_phase);

	/* Byte 18: Clock Polarity */
	acpigen_emit_byte(ctx, spi->clock_polarity);

	/* Byte 19-20: Device Selection */
	acpigen_emit_word(ctx, spi->device_select);

	/* Fill in Type Data Length */
	acpi_device_fill_len(ctx, type_length);

	/* Byte 21+: ResourceSource String */
	acpigen_emit_string(ctx, spi->resource);

	/* Fill in SPI Descriptor Length */
	acpi_device_fill_len(ctx, desc_length);
}

/* PowerResource() with Enable and/or Reset control */
int acpi_device_add_power_res(struct acpi_ctx *ctx,
			      const struct acpi_power_res_params *params)
{
	static const char *power_res_dev_states[] = { "_PR0", "_PR3" };
	unsigned int reset_gpio = params->reset_gpio->pins[0];
	unsigned int enable_gpio = params->enable_gpio->pins[0];
	unsigned int stop_gpio = params->stop_gpio->pins[0];
	int ret;

	if (!reset_gpio && !enable_gpio && !stop_gpio)
		return -EINVAL;

	/* PowerResource (PRIC, 0, 0) */
	acpigen_write_power_res(ctx, "PRIC", 0, 0, power_res_dev_states,
				ARRAY_SIZE(power_res_dev_states));

	/* Method (_STA, 0, NotSerialized) { Return (0x1) } */
	acpigen_write_sta(ctx, 0x1);

	/* Method (_ON, 0, Serialized) */
	acpigen_write_method_serialized(ctx, "_ON", 0);
	if (reset_gpio) {
		ret = acpigen_enable_tx_gpio(ctx, params->reset_gpio);
		if (ret)
			return log_msg_ret("reset1", ret);
	}
	if (enable_gpio) {
		ret = acpigen_enable_tx_gpio(ctx, params->enable_gpio);
		if (ret)
			return log_msg_ret("enable1", ret);
		if (params->enable_delay_ms)
			acpigen_write_sleep(ctx, params->enable_delay_ms);
	}
	if (reset_gpio) {
		ret = acpigen_disable_tx_gpio(ctx, params->reset_gpio);
		if (ret)
			return log_msg_ret("reset2", ret);
		if (params->reset_delay_ms)
			acpigen_write_sleep(ctx, params->reset_delay_ms);
	}
	if (stop_gpio) {
		ret = acpigen_disable_tx_gpio(ctx, params->stop_gpio);
		if (ret)
			return log_msg_ret("stop1", ret);
		if (params->stop_delay_ms)
			acpigen_write_sleep(ctx, params->stop_delay_ms);
	}
	acpigen_pop_len(ctx);		/* _ON method */

	/* Method (_OFF, 0, Serialized) */
	acpigen_write_method_serialized(ctx, "_OFF", 0);
	if (stop_gpio) {
		ret = acpigen_enable_tx_gpio(ctx, params->stop_gpio);
		if (ret)
			return log_msg_ret("stop2", ret);
		if (params->stop_off_delay_ms)
			acpigen_write_sleep(ctx, params->stop_off_delay_ms);
	}
	if (reset_gpio) {
		ret = acpigen_enable_tx_gpio(ctx, params->reset_gpio);
		if (ret)
			return log_msg_ret("reset3", ret);
		if (params->reset_off_delay_ms)
			acpigen_write_sleep(ctx, params->reset_off_delay_ms);
	}
	if (enable_gpio) {
		ret = acpigen_disable_tx_gpio(ctx, params->enable_gpio);
		if (ret)
			return log_msg_ret("enable2", ret);
		if (params->enable_off_delay_ms)
			acpigen_write_sleep(ctx, params->enable_off_delay_ms);
	}
	acpigen_pop_len(ctx);		/* _OFF method */

	acpigen_pop_len(ctx);		/* PowerResource PRIC */

	return 0;
}

static void acpi_dp_write_array(struct acpi_ctx *ctx,
				const struct acpi_dp *array);

static void acpi_dp_write_value(struct acpi_ctx *ctx,
				const struct acpi_dp *prop)
{
	switch (prop->type) {
	case ACPI_DP_TYPE_INTEGER:
		acpigen_write_integer(ctx, prop->integer);
		break;
	case ACPI_DP_TYPE_STRING:
	case ACPI_DP_TYPE_CHILD:
		acpigen_write_string(ctx, prop->string);
		break;
	case ACPI_DP_TYPE_REFERENCE:
		acpigen_emit_namestring(ctx, prop->string);
		break;
	case ACPI_DP_TYPE_ARRAY:
		acpi_dp_write_array(ctx, prop->array);
		break;
	default:
		break;
	}
}

/* Package (2) { "prop->name", VALUE } */
static void acpi_dp_write_property(struct acpi_ctx *ctx,
				   const struct acpi_dp *prop)
{
	acpigen_write_package(ctx, 2);
	acpigen_write_string(ctx, prop->name);
	acpi_dp_write_value(ctx, prop);
	acpigen_pop_len(ctx);
}

/* Write array of Device Properties */
static void acpi_dp_write_array(struct acpi_ctx *ctx,
				const struct acpi_dp *array)
{
	const struct acpi_dp *dp;
	char *pkg_count;

	/* Package element count determined as it is populated */
	pkg_count = acpigen_write_package(ctx, 0);

	/*
	 * Only acpi_dp of type DP_TYPE_TABLE is allowed to be an array.
	 * DP_TYPE_TABLE does not have a value to be written. Thus, start
	 * the loop from next type in the array.
	 */
	for (dp = array->next; dp; dp = dp->next) {
		acpi_dp_write_value(ctx, dp);
		(*pkg_count)++;
	}

	acpigen_pop_len(ctx);
}

static void acpi_dp_free(struct acpi_dp *dp)
{
	while (dp) {
		struct acpi_dp *p = dp->next;

		switch (dp->type) {
		case ACPI_DP_TYPE_CHILD:
			acpi_dp_free(dp->child);
			break;
		case ACPI_DP_TYPE_ARRAY:
			acpi_dp_free(dp->array);
			break;
		default:
			break;
		}

		free(dp);
		dp = p;
	}
}

int acpi_dp_write(struct acpi_ctx *ctx, struct acpi_dp *table)
{
	struct acpi_dp *dp, *prop;
	char *dp_count, *prop_count = NULL;
	int child_count = 0;
	int ret;

	if (!table || table->type != ACPI_DP_TYPE_TABLE)
		return 0;

	/* Name (name) */
	acpigen_write_name(ctx, table->name);

	/* Device Property list starts with the next entry */
	prop = table->next;

	/* Package (DP), default to assuming no properties or children */
	dp_count = acpigen_write_package(ctx, 0);

	/* Print base properties */
	for (dp = prop; dp; dp = dp->next) {
		if (dp->type == ACPI_DP_TYPE_CHILD) {
			child_count++;
		} else {
			/*
			 * The UUID and package is only added when
			 * we come across the first property.  This
			 * is to avoid creating a zero-length package
			 * in situations where there are only children.
			 */
			if (!prop_count) {
				*dp_count += 2;
				/* ToUUID (ACPI_DP_UUID) */
				ret = acpigen_write_uuid(ctx, ACPI_DP_UUID);
				if (ret)
					return log_msg_ret("touuid", ret);
				/*
				 * Package (PROP), element count determined as
				 * it is populated
				 */
				prop_count = acpigen_write_package(ctx, 0);
			}
			(*prop_count)++;
			acpi_dp_write_property(ctx, dp);
		}
	}

	if (prop_count) {
		/* Package (PROP) length, if a package was written */
		acpigen_pop_len(ctx);
	}

	if (child_count) {
		/* Update DP package count to 2 or 4 */
		*dp_count += 2;
		/* ToUUID (ACPI_DP_CHILD_UUID) */
		ret = acpigen_write_uuid(ctx, ACPI_DP_CHILD_UUID);
		if (ret)
			return log_msg_ret("child uuid", ret);

		/* Print child pointer properties */
		acpigen_write_package(ctx, child_count);

		for (dp = prop; dp; dp = dp->next)
			if (dp->type == ACPI_DP_TYPE_CHILD)
				acpi_dp_write_property(ctx, dp);
		/* Package (CHILD) length */
		acpigen_pop_len(ctx);
	}

	/* Package (DP) length */
	acpigen_pop_len(ctx);

	/* Recursively parse children into separate tables */
	for (dp = prop; dp; dp = dp->next) {
		if (dp->type == ACPI_DP_TYPE_CHILD) {
			ret = acpi_dp_write(ctx, dp->child);
			if (ret)
				return log_msg_ret("dp child", ret);
		}
	}

	/* Clean up */
	acpi_dp_free(table);

	return 0;
}

static struct acpi_dp *acpi_dp_new(struct acpi_dp *dp, enum acpi_dp_type type,
				   const char *name)
{
	struct acpi_dp *new;

	new = malloc(sizeof(struct acpi_dp));
	if (!new)
		return NULL;

	memset(new, '\0', sizeof(*new));
	new->type = type;
	new->name = name;

	if (dp) {
		/* Add to end of property list */
		while (dp->next)
			dp = dp->next;
		dp->next = new;
	}

	return new;
}

struct acpi_dp *acpi_dp_new_table(const char *name)
{
	return acpi_dp_new(NULL, ACPI_DP_TYPE_TABLE, name);
}

size_t acpi_dp_add_property_list(struct acpi_dp *dp,
				 const struct acpi_dp *property_list,
				 size_t property_count)
{
	const struct acpi_dp *prop;
	size_t i, properties_added = 0;

	if (!dp || !property_list)
		return 0;

	for (i = 0; i < property_count; i++) {
		prop = &property_list[i];

		if (prop->type == ACPI_DP_TYPE_UNKNOWN || !prop->name)
			continue;

		switch (prop->type) {
		case ACPI_DP_TYPE_INTEGER:
			acpi_dp_add_integer(dp, prop->name, prop->integer);
			break;
		case ACPI_DP_TYPE_STRING:
			acpi_dp_add_string(dp, prop->name, prop->string);
			break;
		case ACPI_DP_TYPE_REFERENCE:
			acpi_dp_add_reference(dp, prop->name, prop->string);
			break;
		case ACPI_DP_TYPE_ARRAY:
			acpi_dp_add_array(dp, prop->array);
			break;
		case ACPI_DP_TYPE_CHILD:
			acpi_dp_add_child(dp, prop->name, prop->child);
			break;
		default:
			continue;
		}

		++properties_added;
	}

	return properties_added;
}

struct acpi_dp *acpi_dp_add_integer(struct acpi_dp *dp, const char *name,
				    uint64_t value)
{
	struct acpi_dp *new = acpi_dp_new(dp, ACPI_DP_TYPE_INTEGER, name);

	if (new)
		new->integer = value;

	return new;
}

struct acpi_dp *acpi_dp_add_string(struct acpi_dp *dp, const char *name,
				   const char *string)
{
	struct acpi_dp *new = acpi_dp_new(dp, ACPI_DP_TYPE_STRING, name);

	if (new)
		new->string = string;

	return new;
}

struct acpi_dp *acpi_dp_add_reference(struct acpi_dp *dp, const char *name,
				      const char *reference)
{
	if (!dp)
		return NULL;

	struct acpi_dp *new = acpi_dp_new(dp, ACPI_DP_TYPE_REFERENCE, name);

	if (new)
		new->string = reference;

	return new;
}

struct acpi_dp *acpi_dp_add_child(struct acpi_dp *dp, const char *name,
				  struct acpi_dp *child)
{
	struct acpi_dp *new;

	if (!dp || !child || child->type != ACPI_DP_TYPE_TABLE)
		return NULL;

	new = acpi_dp_new(dp, ACPI_DP_TYPE_CHILD, name);
	if (new) {
		new->child = child;
		new->string = child->name;
	}

	return new;
}

struct acpi_dp *acpi_dp_add_array(struct acpi_dp *dp, struct acpi_dp *array)
{
	struct acpi_dp *new;

	if (!dp || !array || array->type != ACPI_DP_TYPE_TABLE)
		return NULL;

	new = acpi_dp_new(dp, ACPI_DP_TYPE_ARRAY, array->name);
	if (new)
		new->array = array;

	return new;
}

struct acpi_dp *acpi_dp_add_integer_array(struct acpi_dp *dp, const char *name,
					  uint64_t *array, int len)
{
	struct acpi_dp *dp_array;
	int i;

	if (!dp || len <= 0)
		return NULL;

	dp_array = acpi_dp_new_table(name);
	if (!dp_array)
		return NULL;

	for (i = 0; i < len; i++)
		if (!acpi_dp_add_integer(dp_array, NULL, array[i]))
			break;

	acpi_dp_add_array(dp, dp_array);

	return dp_array;
}

struct acpi_dp *acpi_dp_add_gpio(struct acpi_dp *dp, const char *name,
				 const char *ref, int index, int pin,
				 bool active_low)
{
	if (!dp)
		return NULL;

	struct acpi_dp *gpio = acpi_dp_new_table(name);

	if (!gpio)
		return NULL;

	/* The device that has _CRS containing GpioIO()/GpioInt() */
	acpi_dp_add_reference(gpio, NULL, ref);

	/* Index of the GPIO resource in _CRS starting from zero */
	acpi_dp_add_integer(gpio, NULL, index);

	/* Pin in the GPIO resource, typically zero */
	acpi_dp_add_integer(gpio, NULL, pin);

	/* Set if pin is active low */
	acpi_dp_add_integer(gpio, NULL, active_low);

	acpi_dp_add_array(dp, gpio);

	return gpio;
}

/**
 * acpi_device_set_i2c() - Set up an ACPI I2C struct from a device
 *
 * @dev: I2C device to convert
 * @i2c: Place to put the new structure
 * @scope: Scope of the I2C device (this is the controller path)
 * @return 0 (always)
 */
static int acpi_device_set_i2c(const struct udevice *dev, struct acpi_i2c *i2c,
			       const char *scope)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	struct udevice *bus = dev_get_parent(dev);

	memset(i2c, '\0', sizeof(i2c));
	i2c->address = chip->chip_addr;
	i2c->mode_10bit = 0;

	/*
	 * i2c_bus->speed_hz is set if this device is probed, but if not we
	 * must use the device tree
	 */
	i2c->speed = dev_read_u32_default(bus, "clock-frequency", 100000);
	i2c->resource = scope;

	return 0;
}

int acpi_device_write_i2c_dev(struct acpi_ctx *ctx, const struct udevice *dev)
{
	char scope[ACPI_PATH_MAX];
	struct acpi_i2c i2c;
	int ret;

	ret = acpi_device_scope(dev, scope, sizeof(scope));
	if (ret)
		return log_msg_ret("scope", ret);
	ret = acpi_device_set_i2c(dev, &i2c, scope);
	if (ret)
		return log_msg_ret("set", ret);
	acpi_device_write_i2c(ctx, &i2c);

	return 0;
}

static void i2c_hid_func0_cb(struct acpi_ctx *ctx, void *arg)
{
	/* ToInteger (Arg1, Local2) */
	acpigen_write_to_integer(ctx, ARG1_OP, LOCAL2_OP);
	/* If (LEqual (Local2, 0x0)) */
	acpigen_write_if_lequal_op_int(ctx, LOCAL2_OP, 0x0);
	/*   Return (Buffer (One) { 0x1f }) */
	acpigen_write_return_singleton_buffer(ctx, 0x1f);
	acpigen_pop_len(ctx);	/* Pop : If */
	/* Else */
	acpigen_write_else(ctx);
	/*   If (LEqual (Local2, 0x1)) */
	acpigen_write_if_lequal_op_int(ctx, LOCAL2_OP, 0x1);
	/*     Return (Buffer (One) { 0x3f }) */
	acpigen_write_return_singleton_buffer(ctx, 0x3f);
	acpigen_pop_len(ctx);	/* Pop : If */
	/*   Else */
	acpigen_write_else(ctx);
	/*     Return (Buffer (One) { 0x0 }) */
	acpigen_write_return_singleton_buffer(ctx, 0x0);
	acpigen_pop_len(ctx);	/* Pop : Else */
	acpigen_pop_len(ctx);	/* Pop : Else */
}

static void i2c_hid_func1_cb(struct acpi_ctx *ctx, void *arg)
{
	struct dsm_i2c_hid_config *config = arg;

	acpigen_write_return_byte(ctx, config->hid_desc_reg_offset);
}

static hid_callback_func i2c_hid_callbacks[2] = {
	i2c_hid_func0_cb,
	i2c_hid_func1_cb,
};

void acpi_device_write_dsm_i2c_hid(struct acpi_ctx *ctx,
				   struct dsm_i2c_hid_config *config)
{
	acpigen_write_dsm(ctx, ACPI_DSM_I2C_HID_UUID, i2c_hid_callbacks,
			  ARRAY_SIZE(i2c_hid_callbacks), config);
}

static const char *acpi_name_from_id(enum uclass_id id)
{
	switch (id) {
	case UCLASS_USB_HUB:
		/* Root Hub */
		return "RHUB";
	/* DSDT: acpi/northbridge.asl */
	case UCLASS_NORTHBRIDGE:
		return "MCHC";
	/* DSDT: acpi/lpc.asl */
	case UCLASS_LPC:
		return "LPCB";
	/* DSDT: acpi/xhci.asl */
	case UCLASS_USB:
		return "XHCI";
	case UCLASS_PWM:
		return "PWM";
	default:
		return NULL;
	}
}

static int acpi_check_seq(const struct udevice *dev)
{
	if (dev->req_seq == -1) {
		log_warning("Device '%s' has no seq\n", dev->name);
		return log_msg_ret("no seq", -ENXIO);
	}

	return dev->req_seq;
}

/* If you change this function, add test cases to dm_test_acpi_get_name() */
int acpi_device_get_name(const struct udevice *dev, char *out_name)
{
	enum uclass_id parent_id = UCLASS_INVALID;
	enum uclass_id id;
	const char *name = NULL;

	id = device_get_uclass_id(dev);
	if (dev_get_parent(dev))
		parent_id = device_get_uclass_id(dev_get_parent(dev));

	if (id == UCLASS_SOUND)
		name = "HDAS";
	else if (id == UCLASS_PCI)
		name = "PCI0";
	else if (device_is_on_pci_bus(dev))
		name = acpi_name_from_id(id);
	if (!name) {
		switch (parent_id) {
		case UCLASS_USB: {
			struct usb_device *udev = dev_get_parent_priv(dev);

			sprintf(out_name, udev->speed >= USB_SPEED_SUPER ?
				"HS%02d" : "FS%02d",
				udev->portnr);
			name = out_name;
			break;
		}
		default:
			break;
		}
	}
	if (!name) {
		int num;

		switch (id) {
		/* DSDT: acpi/lpss.asl */
		case UCLASS_SERIAL:
			num = acpi_check_seq(dev);
			if (num < 0)
				return num;
			sprintf(out_name, "URT%d", num);
			name = out_name;
			break;
		case UCLASS_I2C:
			num = acpi_check_seq(dev);
			if (num < 0)
				return num;
			sprintf(out_name, "I2C%d", num);
			name = out_name;
			break;
		case UCLASS_SPI:
			num = acpi_check_seq(dev);
			if (num < 0)
				return num;
			sprintf(out_name, "SPI%d", num);
			name = out_name;
			break;
		default:
			break;
		}
	}
	if (!name) {
		log_warning("No name for device '%s'\n", dev->name);
		return -ENOENT;
	}
	if (name != out_name)
		memcpy(out_name, name, ACPI_NAME_MAX);

	return 0;
}

int acpi_dp_ofnode_copy_int(ofnode node, struct acpi_dp *dp, const char *prop)
{
	int ret;
	u32 val = 0;

	ret = ofnode_read_u32(node, prop, &val);
	acpi_dp_add_integer(dp, prop, val);

	return ret;
}

int acpi_dp_ofnode_copy_str(ofnode node, struct acpi_dp *dp, const char *prop)
{
	const char *val;

	val = ofnode_read_string(node, prop);
	if (!val)
		return -ENOENT;
	acpi_dp_add_string(dp, prop, val);

	return 0;
}

int acpi_dp_dev_copy_int(const struct udevice *dev, struct acpi_dp *dp,
				const char *prop)
{
	int ret;
	u32 val = 0;

	ret = dev_read_u32(dev, prop, &val);
	acpi_dp_add_integer(dp, prop, val);

	return ret;
}

int acpi_dp_dev_copy_str(const struct udevice *dev, struct acpi_dp *dp,
			 const char *prop)
{
	const char *val;

	val = dev_read_string(dev, prop);
	if (!val)
		return -ENOENT;
	acpi_dp_add_string(dp, prop, val);

	return 0;
}
