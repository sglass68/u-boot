/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Generation of tables for particular device types
 *
 * Copyright 2019 Google LLC
 * Mostly taken from coreboot file of the same name
 */

#ifndef __ACPI_DEVICE_H
#define __ACPI_DEVICE_H

#include <i2c.h>
#include <irq.h>
#include <spi.h>
#include <asm-generic/gpio.h>

struct acpi_ctx;
struct irq;
struct gpio_desc;
struct udevice;

/**
 * enum acpi_dp_type - types of device property objects
 *
 * These refer to the types defined by struct acpi_dp below
 *
 * @ACPI_DP_TYPE_UNKNOWN: Unknown / do not use
 * @ACPI_DP_TYPE_INTEGER: Integer value (u64) in @integer
 * @ACPI_DP_TYPE_STRING: String value in @string
 * @ACPI_DP_TYPE_REFERENCE: Reference to another object, with value in @string
 * @ACPI_DP_TYPE_TABLE: Type for a top-level table which may have children
 * @ACPI_DP_TYPE_ARRAY: Array of items with first item in @array and following
 *	items linked from that item's @next
 * @ACPI_DP_TYPE_CHILD: Child object, with siblings in that child's @next
 */
enum acpi_dp_type {
	ACPI_DP_TYPE_UNKNOWN,
	ACPI_DP_TYPE_INTEGER,
	ACPI_DP_TYPE_STRING,
	ACPI_DP_TYPE_REFERENCE,
	ACPI_DP_TYPE_TABLE,
	ACPI_DP_TYPE_ARRAY,
	ACPI_DP_TYPE_CHILD,
};

/* ACPI descriptor values for common descriptors: SERIAL_BUS means I2C */
#define ACPI_DESCRIPTOR_LARGE		(1 << 7)
#define ACPI_DESCRIPTOR_INTERRUPT	(ACPI_DESCRIPTOR_LARGE | 9)
#define ACPI_DESCRIPTOR_GPIO		(ACPI_DESCRIPTOR_LARGE | 12)
#define ACPI_DESCRIPTOR_SERIAL_BUS	(ACPI_DESCRIPTOR_LARGE | 14)

/*
 * PRP0001 is a special DT namespace link device ID. It provides a means to use
 * existing DT-compatible device identification in ACPI. When this _HID is used
 * by an ACPI device, the ACPI subsystem in OS looks up "compatible" property in
 * the device object's _DSD and will use the value of that property to identify
 * the corresponding device in analogy with the original DT device
 * identification algorithm.
 * More details can be found in Linux kernel documentation:
 * Documentation/acpi/enumeration.txt
 */
#define ACPI_DT_NAMESPACE_HID		"PRP0001"

/* Length of a full path to an ACPI device */
#define ACPI_PATH_MAX		30

/**
 * acpi_device_name() - Locate and return the ACPI name for this device
 *
 * @dev: Device to check
 * @name: Returns the character name, must be at least ACPI_NAME_MAX long
 * @return 0 if OK, -ve on error
 */
int acpi_device_name(const struct udevice *dev, char *name);

/**
 * acpi_device_path() - Get the full path to an ACPI device
 *
 * This gets the full path in the form XXXX.YYYY.ZZZZ where XXXX is the root
 * and ZZZZ is the device. All parent devices are added to the path.
 *
 * @dev: Device to check
 * @buf: Buffer to place the path in (should be ACPI_PATH_MAX long)
 * @maxlen: Size of buffer (typically ACPI_PATH_MAX)
 * @return 0 if OK, -ve on error
 */
int acpi_device_path(const struct udevice *dev, char *buf, int maxlen);

/**
 * acpi_device_scope() - Get the scope of an ACPI device
 *
 * This gets the scope which is the full path of the parent device, as per
 * acpi_device_path().
 *
 * @dev: Device to check
 * @buf: Buffer to place the path in (should be ACPI_PATH_MAX long)
 * @maxlen: Size of buffer (typically ACPI_PATH_MAX)
 * @return 0 if OK, -EINVAL if the device has no parent, other -ve on other
 *	error
 */
int acpi_device_scope(const struct udevice *dev, char *scope, int maxlen);

/**
 * acpi_device_status() - Get the status of a device
 *
 * This currently just returns ACPI_STATUS_DEVICE_ALL_ON. It does not support
 * inactive or hidden devices.
 *
 * @dev: Device to check
 * @return device status, as ACPI_STATUS_DEVICE_...
 */
int acpi_device_status(const struct udevice *dev);

/** enum acpi_irq_mode - edge/level trigger mode */
enum acpi_irq_mode {
	ACPI_IRQ_EDGE_TRIGGERED,
	ACPI_IRQ_LEVEL_TRIGGERED,
};

/**
 * enum acpi_irq_polarity - polarity of interrupt
 *
 * @ACPI_IRQ_ACTIVE_LOW - for ACPI_IRQ_EDGE_TRIGGERED this means falling edge
 * @ACPI_IRQ_ACTIVE_HIGH - for ACPI_IRQ_EDGE_TRIGGERED this means rising edge
 * @ACPI_IRQ_ACTIVE_BOTH - not meaningful for ACPI_IRQ_EDGE_TRIGGERED
 */
enum acpi_irq_polarity {
	ACPI_IRQ_ACTIVE_LOW,
	ACPI_IRQ_ACTIVE_HIGH,
	ACPI_IRQ_ACTIVE_BOTH,
};

/**
 * enum acpi_irq_shared - whether interrupt is shared or not
 *
 * @ACPI_IRQ_EXCLUSIVE: only this device uses the interrupt
 * @ACPI_IRQ_SHARED: other devices may use this interrupt
 */
enum acpi_irq_shared {
	ACPI_IRQ_EXCLUSIVE,
	ACPI_IRQ_SHARED,
};

/** enum acpi_irq_wake - indicates whether this interrupt can wake the device */
enum acpi_irq_wake {
	ACPI_IRQ_NO_WAKE,
	ACPI_IRQ_WAKE,
};

/**
 * struct acpi_irq - representation of an ACPI interrupt
 *
 * @pin: ACPI pin that is monitored for the interrupt
 * @mode: Edge/level triggering
 * @polarity: Interrupt polarity
 * @shared: Whether interrupt is shared or not
 * @wake: Whether interrupt can wake the device from sleep
 */
struct acpi_irq {
	unsigned int pin;
	enum acpi_irq_mode mode;
	enum acpi_irq_polarity polarity;
	enum acpi_irq_shared shared;
	enum acpi_irq_wake wake;
};

/**
 * enum acpi_gpio_type - type of the descriptor
 *
 * @ACPI_GPIO_TYPE_INTERRUPT: GpioInterrupt
 * @ACPI_GPIO_TYPE_IO: GpioIo
 */
enum acpi_gpio_type {
	ACPI_GPIO_TYPE_INTERRUPT,
	ACPI_GPIO_TYPE_IO,
};

/**
 * enum acpi_gpio_pull - pull direction
 *
 * @ACPI_GPIO_PULL_DEFAULT: Use default value for pin
 * @ACPI_GPIO_PULL_UP: Pull up
 * @ACPI_GPIO_PULL_DOWN: Pull down
 * @ACPI_GPIO_PULL_NONE: No pullup/pulldown
 */
enum acpi_gpio_pull {
	ACPI_GPIO_PULL_DEFAULT,
	ACPI_GPIO_PULL_UP,
	ACPI_GPIO_PULL_DOWN,
	ACPI_GPIO_PULL_NONE,
};

/**
 * enum acpi_gpio_io_restrict - controls input/output of pin
 *
 * @ACPI_GPIO_IO_RESTRICT_NONE: no restrictions
 * @ACPI_GPIO_IO_RESTRICT_INPUT: input only (no output)
 * @ACPI_GPIO_IO_RESTRICT_OUTPUT: output only (no input)
 * @ACPI_GPIO_IO_RESTRICT_PRESERVE: preserve settings when driver not active
 */
enum acpi_gpio_io_restrict {
	ACPI_GPIO_IO_RESTRICT_NONE,
	ACPI_GPIO_IO_RESTRICT_INPUT,
	ACPI_GPIO_IO_RESTRICT_OUTPUT,
	ACPI_GPIO_IO_RESTRICT_PRESERVE,
};

/** enum acpi_gpio_polarity - controls the GPIO polarity */
enum acpi_gpio_polarity {
	ACPI_GPIO_ACTIVE_HIGH = 0,
	ACPI_GPIO_ACTIVE_LOW = 1,
};

#define ACPI_GPIO_REVISION_ID		1
#define ACPI_GPIO_MAX_PINS		2

/**
 * struct acpi_gpio - representation of an ACPI GPIO
 *
 * @pin_count: Number of pins represented
 * @pins: List of pins
 * @type: GPIO type
 * @pull: Pullup/pulldown setting
 * @resource: Resource name for this GPIO controller
 * For GpioInt:
 * @interrupt_debounce_timeout: Debounce timeout in units of 10us
 * @irq: Interrupt
 *
 * For GpioIo:
 * @output_drive_strength: Drive strength in units of 10uA
 * @io_shared; true if GPIO is shared
 * @io_restrict: I/O restriction setting
 * @polarity: GPIO polarity
 */
struct acpi_gpio {
	int pin_count;
	uint16_t pins[ACPI_GPIO_MAX_PINS];

	enum acpi_gpio_type type;
	enum acpi_gpio_pull pull;
	char resource[ACPI_PATH_MAX];

	/* GpioInt */
	u16 interrupt_debounce_timeout;
	struct acpi_irq irq;

	/* GpioIo */
	u16 output_drive_strength;
	bool io_shared;
	enum acpi_gpio_io_restrict io_restrict;
	enum acpi_gpio_polarity polarity;
};


/* ACPI Descriptors for Serial Bus interfaces */
#define ACPI_SERIAL_BUS_TYPE_I2C		1
#define ACPI_SERIAL_BUS_TYPE_SPI		2
#define ACPI_I2C_SERIAL_BUS_REVISION_ID		1 /* TODO: upgrade to 2 */
#define ACPI_I2C_TYPE_SPECIFIC_REVISION_ID	1
#define ACPI_SPI_SERIAL_BUS_REVISION_ID		1
#define ACPI_SPI_TYPE_SPECIFIC_REVISION_ID	1

/**
 * struct acpi_gpio - representation of an ACPI I2C device
 *
 * @address: 7-bit or 10-bit I2C address
 * @mode_10bit: Which address size is used
 * @speed: Bus speed in Hz
 * @resource: Resource name for the I2C controller
 */
struct acpi_i2c {
	uint16_t address;
	enum i2c_address_mode mode_10bit;
	enum i2c_speed_rate speed;
	const char *resource;
};

/**
 * struct acpi_spi - representation of an ACPI SPI device
 *
 * @device_select: Chip select used by this device (typically 0)
 * @device_select_polarity: Polarity for the device
 * @wire_mode: Number of wires used for SPI
 * @speed: Bus speed in Hz
 * @data_bit_length: Word length for SPI (typically 8)
 * @clock_phase: Clock phase to capture data
 * @clock_polarity: Bus polarity
 * @resource: Resource name for the SPI controller
 */
struct acpi_spi {
	uint16_t device_select;
	enum spi_polarity device_select_polarity;
	enum spi_wire_mode wire_mode;
	unsigned int speed;
	u8 data_bit_length;
	enum spi_clock_phase clock_phase;
	enum spi_polarity clock_polarity;
	const char *resource;
};

/**
 * struct acpi_power_res_params - power on/off sequence information
 *
 * This provides GPIOs and timing information for powering a device on and off.
 * This can be applied to any device that has power control, so is fairly
 * generic.
 *
 * @reset_gpio: GPIO used to take device out of reset or to put it into reset
 * @reset_delay_ms: Delay to be inserted after device is taken out of reset
 *	(_ON method delay)
 * @reset_off_delay_ms: Delay to be inserted after device is put into reset
 *	(_OFF method delay)
 * @enable_gpio: GPIO used to enable device
 * @enable_delay_ms: Delay to be inserted after device is enabled
 * @enable_off_delay_ms: Delay to be inserted after device is disabled
 *	(_OFF method delay)
 * @stop_gpio: GPIO used to stop operation of device
 * @stop_delay_ms: Delay to be inserted after disabling stop (_ON method delay)
 * @stop_off_delay_ms: Delay to be inserted after enabling stop.
 *	(_OFF method delay)
 */
struct acpi_power_res_params {
	struct acpi_gpio *reset_gpio;
	unsigned int reset_delay_ms;
	unsigned int reset_off_delay_ms;
	struct acpi_gpio *enable_gpio;
	unsigned int enable_delay_ms;
	unsigned int enable_off_delay_ms;
	struct acpi_gpio *stop_gpio;
	unsigned int stop_delay_ms;
	unsigned int stop_off_delay_ms;
};

/**
 * struct acpi_dp - ACPI device properties
 *
 * Writing Device Properties objects via _DSD
 *
 * This provides a structure to handle nested device-specific data which ends
 * up in a _DSD table.
 *
 * https://www.kernel.org/doc/html/latest/firmware-guide/acpi/DSD-properties-rules.html
 * https://uefi.org/sites/default/files/resources/_DSD-device-properties-UUID.pdf
 * https://uefi.org/sites/default/files/resources/_DSD-hierarchical-data-extension-UUID-v1.1.pdf
 *
 * The Device Property Hierarchy can be multiple levels deep with multiple
 * children possible in each level.  In order to support this flexibility
 * the device property hierarchy must be built up before being written out.
 *
 * For example:
 *
 * // Child table with string and integer
 * struct acpi_dp *child = acpi_dp_new_table("CHLD");
 * acpi_dp_add_string(child, "childstring", "CHILD");
 * acpi_dp_add_integer(child, "childint", 100);
 *
 * // _DSD table with integer and gpio and child pointer
 * struct acpi_dp *dsd = acpi_dp_new_table("_DSD");
 * acpi_dp_add_integer(dsd, "number1", 1);
 * acpi_dp_add_gpio(dsd, "gpio", "\_SB.PCI0.GPIO", 0, 0, 1);
 * acpi_dp_add_child(dsd, "child", child);
 *
 * // Write entries into SSDT and clean up resources
 * acpi_dp_write(dsd);
 *
 * Name(_DSD, Package() {
 *   ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301")
 *   Package() {
 *     Package() { "gpio", Package() { \_SB.PCI0.GPIO, 0, 0, 0 } }
 *     Package() { "number1", 1 }
 *   }
 *   ToUUID("dbb8e3e6-5886-4ba6-8795-1319f52a966b")
 *   Package() {
 *     Package() { "child", CHLD }
 *   }
 * }
 * Name(CHLD, Package() {
 *   ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301")
 *   Package() {
 *     Package() { "childstring", "CHILD" }
 *     Package() { "childint", 100 }
 *   }
 * }
 *
 * @type: Table type
 * @name: Name of object, typically _DSD but could be CHLD for a child object
 * @next: Next object in list (next array element or next sibling)
 * @child: Pointer to first child, if @type == ACPI_DP_TYPE_CHILD, else NULL
 * @array: First array element, if @type == ACPI_DP_TYPE_ARRAY, else NULL
 * @integer: Integer value of the property, if @type == ACPI_DP_TYPE_INTEGER
 * @string: String value of the property, if @type == ACPI_DP_TYPE_STRING;
 *	child name if @type == ACPI_DP_TYPE_CHILD;
 *	reference name if @type == ACPI_DP_TYPE_REFERENCE;
 */
struct acpi_dp {
	enum acpi_dp_type type;
	const char *name;
	struct acpi_dp *next;
	union {
		struct acpi_dp *child;
		struct acpi_dp *array;
	};
	union {
		u64 integer;
		const char *string;
	};
};

/**
 * acpi_dp_new_table() - Start a new Device Property table
 *
 * @ref: ACPI reference (e.g. "_DSD")
 * @return pointer to table, or NULL if out of memory
 */
struct acpi_dp *acpi_dp_new_table(const char *ref);

/**
 * acpi_dp_add_integer() - Add integer Device Property
 *
 * A new node is added to the end of the property list of @dp
 *
 * @dp: Table to add this property to
 * @name: Name of property
 * @value: Integer value
 * @return pointer to new node, or NULL if out of memory
 */
struct acpi_dp *acpi_dp_add_integer(struct acpi_dp *dp, const char *name,
				    uint64_t value);

/**
 * acpi_dp_add_string() - Add string Device Property
 *
 * A new node is added to the end of the property list of @dp
 *
 * @dp: Table to add this property to
 * @name: Name of property
 * @string: String value
 * @return pointer to new node, or NULL if out of memory
 */
struct acpi_dp *acpi_dp_add_string(struct acpi_dp *dp, const char *name,
				   const char *string);

/**
 * acpi_dp_add_reference() - Add reference Device Property
 *
 * A new node is added to the end of the property list of @dp
 *
 * @dp: Table to add this property to
 * @name: Name of property
 * @reference: Reference value
 * @return pointer to new node, or NULL if out of memory
 */
struct acpi_dp *acpi_dp_add_reference(struct acpi_dp *dp, const char *name,
				      const char *reference);

/**
 * acpi_dp_add_array() - Add array Device Property
 *
 * A new node is added to the end of the property list of @dp, with the array
 * attached to that.
 *
 * @dp: Table to add this property to
 * @name: Name of property
 * @return pointer to new node, or NULL if out of memory
 */
struct acpi_dp *acpi_dp_add_array(struct acpi_dp *dp, struct acpi_dp *array);

/**
 * acpi_dp_add_integer_array() - Add an array of integers
 *
 * A new node is added to the end of the property list of @dp, with the array
 * attached to that. Each element of the array becomes a new node.
 *
 * @dp: Table to add this property to
 * @name: Name of property
 * @return pointer to new array node, or NULL if out of memory
 */
struct acpi_dp *acpi_dp_add_integer_array(struct acpi_dp *dp, const char *name,
					  uint64_t *array, int len);

/**
 * acpi_dp_add_gpio() - Add a GPIO binding Device Property
 *
 * A new node is added to the end of the property list of @dp, with the
 * GPIO properties added to the the new node
 *
 * @dp: Table to add this property to
 * @name: Name of property
 * @ref: Reference of the device that has _CRS containing GpioIO()/GpioInt()
 * @index: Index of the GPIO resource in _CRS starting from zero
 * @pin: Pin in the GPIO resource, typically zero
 * @active_low: true if pin is active low
 * @return pointer to new node, or NULL if out of memory
 */
struct acpi_dp *acpi_dp_add_gpio(struct acpi_dp *dp, const char *name,
				 const char *ref, int index, int pin,
				 bool active_low);

/**
 * acpi_dp_add_child() - Add a child table of Device Properties
 *
 * A new node is added as a child of @dp
 *
 * @dp: Table to add this child to
 * @name: Name of child
 * @child: Child node to add
 * @return pointer to new array node, or NULL if out of memory
 */
struct acpi_dp *acpi_dp_add_child(struct acpi_dp *dp, const char *name,
				  struct acpi_dp *child);

/**
 * acpi_dp_add_property_list() - Add a list of of Device Properties
 *
 * This adds a list of properties to @dp. Any properties without a name or of
 * type ACPI_DP_TYPE_UNKNOWN are ignored.
 *
 * @dp: Table to add properties to
 * @property_list: List of properties to add
 * @property_count: Number of properties in the list
 * @return number of properties added
 */
size_t acpi_dp_add_property_list(struct acpi_dp *dp,
				 const struct acpi_dp *property_list,
				 size_t property_count);

/**
 * acpi_dp_write() - Write Device Property hierarchy and clean up resources
 *
 * This writes the table using acpigen and then frees it
 *
 * @table: Table to write
 * @return 0 if OK, -ve on error
 */
int acpi_dp_write(struct acpi_ctx *ctx, struct acpi_dp *table);

/**
 * acpi_device_write_i2c_dev() - Write an I2C device to ACPI, including
 * information ACPI needs to use it.
 *
 * This writes a serial bus descriptor for the I2C device so that ACPI can
 */
int acpi_device_write_i2c_dev(struct acpi_ctx *ctx, const struct udevice *dev);

/**
 * struct acpi_i2c_priv - Information read from device tree
 *
 * This is used by devices which want to specify various pieces of ACPI
 * information, including power control. It allows a generic function to
 * generate the information for ACPI, based on device-tree properties.
 *
 * @disable_gpio_export_in_crs: Don't export GPIOs in the CRS
 * @reset_gpio: GPIO used to assert reset to the device
 * @enable_gpio: GPIO used to enable the device
 * @stop_gpio: GPIO used to stop the device
 * @irq_gpio: GPIO used for interrupt (if @irq is not used)
 * @irq: IRQ used for interrupt (if @irq_gpio is not used)
 * @hid: _HID value for device (required)
 * @cid: _CID value for device
 * @uid: _UID value for device
 * @desc: _DDN value for device
 * @wake: Wake event, e.g. GPE0_DW1_15; 0 if none
 * @property_count: Number of other DSD properties (currently always 0)
 * @probed: true set set 'linux,probed' property
 * @compat_string: Device tree compatible string to report through ACPI
 * @has_power_resource: true if this device has a power resource
 * @reset_delay_ms: Delay after de-asserting reset, in ms
 * @reset_off_delay_ms: Delay after asserting reset (during power off)
 * @enable_delay_ms: Delay after asserting enable
 * @enable_off_delay_ms: Delay after de-asserting enable (during power off)
 * @stop_delay_ms: Delay after de-aserting stop
 * @stop_off_delay_ms: Delay after asserting stop (during power off)
 * @hid_desc_reg_offset: HID register offset (for Human Interface Devices)
 */
struct acpi_i2c_priv {
	bool disable_gpio_export_in_crs;
	struct gpio_desc reset_gpio;
	struct gpio_desc enable_gpio;
	struct gpio_desc irq_gpio;
	struct gpio_desc stop_gpio;
	struct irq irq;
	const char *hid;
	const char *cid;
	u32 uid;
	const char *desc;
	u32 wake;
	u32 property_count;
	bool probed;
	const char *compat_string;
	bool has_power_resource;
	u32 reset_delay_ms;
	u32 reset_off_delay_ms;
	u32 enable_delay_ms;
	u32 enable_off_delay_ms;
	u32 stop_delay_ms;
	u32 stop_off_delay_ms;
	u32 hid_desc_reg_offset;
};

/**
 * I2C Human-Inteface Devices configuration
 *
 * @hid_desc_reg_offset: HID register offset
 */
struct dsm_i2c_hid_config {
	uint8_t hid_desc_reg_offset;
};

/**
 * acpi_device_write_gpio() - Write GpioIo() or GpioInt() descriptor
 *
 * @gpio: GPIO information to write
 * @return 0 if OK, -ve on error
 */
int acpi_device_write_gpio(struct acpi_ctx *ctx, const struct acpi_gpio *gpio);

/*
 * acpi_device_add_power_res() - Add a basic PowerResource block for a device
 *
 * This includes GPIOs to control enable, reset and stop operation of the
 * device. Each GPIO is optional, but at least one must be provided.
 *
 * Reset - Put the device into / take the device out of reset.
 * Enable - Enable / disable power to device.
 * Stop - Stop / start operation of device.
 *
 * @return 0 if OK, -ve if at least one GPIO is not provided
 */
int acpi_device_add_power_res(struct acpi_ctx *ctx,
			      const struct acpi_power_res_params *params);

/**
 * gpio_get_acpi() - Convert a GPIO description into an ACPI GPIO
 *
 * At present this is fairly limited. It only supports ACPI_GPIO_TYPE_IO and
 * has hard-coded settings for type, pull, IO restrict and polarity. These
 * could come from pinctrl potentially.
 *
 * @desc: GPIO description to convert
 * @gpio: Place to put ACPI GPIO information
 * @return 0 if OK, -ENOENT if @desc is invalid
 */
int gpio_get_acpi(const struct gpio_desc *desc,
			       struct acpi_gpio *gpio);

/**
 * acpi_device_write_dsm_i2c_hid() - Write a device-specific method for HID
 *
 * This writes a DSM for an I2C Human-Interface Device based on the config
 * provided
 *
 * @config: Config information to write
 */
void acpi_device_write_dsm_i2c_hid(struct acpi_ctx *ctx,
				   struct dsm_i2c_hid_config *config);

/**
 * acpi_dp_ofnode_copy_int() - Copy a property from device tree to DP
 *
 * This copies an integer property from the device tree to the ACPI DP table.
 *
 * @dev: Device to copy from
 * @dp: DP to copy to
 * @prop: Property name to copy
 * @return 0 if OK, -ve on error
 */
int acpi_dp_ofnode_copy_int(ofnode node, struct acpi_dp *dp, const char *prop);

/**
 * acpi_dp_dev_copy_str() - Copy a property from device tree to DP
 *
 * This copies a string property from the device tree to the ACPI DP table.
 *
 * @dev: Device to copy from
 * @dp: DP to copy to
 * @prop: Property name to copy
 * @return 0 if OK, -ve on error
 */
int acpi_dp_ofnode_copy_str(ofnode node, struct acpi_dp *dp, const char *prop);

/**
 * acpi_dp_dev_copy_int() - Copy a property from device tree to DP
 *
 * This copies an integer property from the device tree to the ACPI DP table.
 *
 * @dev: Device to copy from
 * @dp: DP to copy to
 * @prop: Property name to copy
 * @return 0 if OK, -ve on error
 */
int acpi_dp_dev_copy_int(const struct udevice *dev, struct acpi_dp *dp,
			 const char *prop);

/**
 * acpi_dp_dev_copy_str() - Copy a property from device tree to DP
 *
 * This copies a string property from the device tree to the ACPI DP table.
 *
 * @dev: Device to copy from
 * @dp: DP to copy to
 * @prop: Property name to copy
 * @return 0 if OK, -ve on error
 */
int acpi_dp_dev_copy_str(const struct udevice *dev, struct acpi_dp *dp,
			       const char *prop);

int acpi_device_get_name(const struct udevice *dev, char *out_name);

#endif
