/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Core ACPI (Advanced Configuration and Power Interface) support
 *
 * Copyright 2019 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#ifndef __DM_ACPI_H__
#define __DM_ACPI_H__

/* Allow operations to be optional for ACPI */
#if CONFIG_IS_ENABLED(ACPIGEN)
#define acpi_ops_ptr(_ptr)	.acpi_ops	= _ptr,
#else
#define acpi_ops_ptr(_ptr)
#endif

/* Length of an ACPI name string; this is 4 characters plus a nul terminator */
#define ACPI_NAME_MAX	5

#if !defined(__ACPI__)

/**
 * struct acpi_ctx - Context used for writing ACPI tables
 *
 * This contains a few useful pieces of information used when writing
 *
 * @current: Current address for writing
 */
struct acpi_ctx {
	void *current;
};

/**
 * struct acpi_ops - ACPI operations supported by driver model
 */
struct acpi_ops {
	/**
	 * get_name() - Obtain the ACPI name of a device
	 *
	 * @dev: Device to check
	 * @out_name: Place to put the name, must hold at least ACPI_NAME_MAX
	 *	bytes
	 * @return 0 if OK, -ENOENT if no name is available, other -ve value on
	 *	other error
	 */
	int (*get_name)(const struct udevice *dev, char *out_name);

	/**
	 * write_tables() - Write out any tables required by this device
	 *
	 * @dev: Device to write
	 * @ctx: ACPI context to use
	 * @return 0 if OK, -ve on error
	 */
	int (*write_tables)(const struct udevice *dev, struct acpi_ctx *ctx);
};

#define device_get_acpi_ops(dev)	(dev->driver->acpi_ops)

/**
 * acpi_get_name() - Obtain the ACPI name of a device
 *
 * @dev: Device to check
 * @out_name: Place to put the name, must hold at least ACPI_NAME_MAX
 *	bytes
 * @return 0 if OK, -ENOENT if no name is available, other -ve value on
 *	other error
 */
int acpi_get_name(const struct udevice *dev, char *out_name);

/**
 * acpi_return_name() - Copy an ACPI name to an output buffer
 *
 * This convenience function can be used to return a literal string as a name
 * in functions that implement the get_name() method.
 *
 * For example:
 *
 *	static int mydev_get_name(const struct udevice *dev, char *out_name)
 *	{
 *		return acpi_return_name(out_name, "WIBB");
 *	}
 *
 * @out_name: Place to put the name
 * @name: Name to copy
 * @return 0 (always)
 */
int acpi_return_name(char *out_name, const char *name);

/**
 * acpi_write_dev_tables() - Write ACPI tables required by devices
 *
 * This scans through all devices and tells them to write any tables they want
 * to write.
 *
 * @return 0 if OK, -ve if any device returned an error
 */
int acpi_write_dev_tables(struct acpi_ctx *ctx);

#endif /* __ACPI__ */

#endif
