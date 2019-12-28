/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#ifndef __ACPI_H__
#define __ACPI_H__

#if CONFIG_IS_ENABLED(ACPI)
#define acpi_ops_ptr(_ptr)	.acpi_ops	= _ptr,
#else
#define acpi_ops_ptr(_ptr)
#endif

struct acpi_dp;
struct udevice;

struct acpi_ctx {
	ulong current;
	struct acpi_rsdp *rsdp;
};

struct acpi_ops {
	int (*get_name)(const struct udevice *dev, char *out_name);
	int (*write_tables)(struct udevice *dev, struct acpi_ctx *ctx);
	int (*fill_ssdt_generator)(struct udevice *dev, struct acpi_ctx *ctx);
	int (*inject_dsdt_generator)(struct udevice *dev, struct acpi_ctx *ctx);
};

#define device_get_acpi_ops(dev)	(dev->driver->acpi_ops)

int acpi_dev_write_tables(struct acpi_ctx *ctx);

int acpi_get_name(const struct udevice *dev, char *out_name);

int acpi_fill_ssdt_generator(struct acpi_ctx *ctx);

int acpi_inject_dsdt_generator(struct acpi_ctx *ctx);


int acpi_return_name(char *out_name, const char *name);

int acpi_align(struct acpi_ctx *ctx);

void acpi_dump_items(void);

int acpi_dp_add_integer_from_dt(struct udevice *dev, struct acpi_dp *dp,
				const char *prop);
int acpi_dp_add_string_from_dt(struct udevice *dev, struct acpi_dp *dp,
			       const char *prop);

#endif
