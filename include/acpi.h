/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#ifndef __ACPI_H__
#define __ACPI_H__

#if CONFIG_IS_ENABLED(ACPI)
#define acpi_ops_ptr(_ptr)	_ptr,
#else
#define acpi_ops_ptr(_ptr)
#endif

struct udevice;

struct acpi_ctx {
	ulong current;
	struct acpi_rsdp *rsdp;
};

struct acpi_ops {
	int (*get_name)(const struct udevice *dev, char *out_name);
	int (*write_tables)(struct udevice *dev, struct acpi_ctx *ctx);
};

int acpi_return_name(char *out_name, const char *name);

int ctx_align(struct acpi_ctx *ctx);

#endif
