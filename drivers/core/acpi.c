// SPDX-License-Identifier: GPL-2.0+
/*
 * Core driver model support for ACPI table generation
 *
 * Copyright 2019 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEOGRY	LOGC_ACPI

#include <common.h>
#include <dm.h>
#include <dm/acpi.h>
#include <dm/device-internal.h>
#include <dm/root.h>

int acpi_return_name(char *out_name, const char *name)
{
	strcpy(out_name, name);

	return 0;
}

int acpi_get_name(const struct udevice *dev, char *out_name)
{
	struct acpi_ops *aops;

	aops = device_get_acpi_ops(dev);
	if (aops && aops->get_name)
		return aops->get_name(dev, out_name);

	return -ENOSYS;
}

int _acpi_fill_ssdt(struct acpi_ctx *ctx, struct udevice *parent)
{
	struct acpi_ops *aops;
	struct udevice *dev;
	int ret;

	aops = device_get_acpi_ops(parent);
	if (aops && aops->fill_ssdt) {
		log_debug("\n- %s %p\n", parent->name,
			  aops->fill_ssdt);
		ret = device_ofdata_to_platdata(parent);
		if (ret)
			return log_msg_ret("ofdata", ret);
		ret = aops->fill_ssdt(parent, ctx);
		if (ret)
			return log_msg_ret("ssdt", ret);
	}
	device_foreach_child(dev, parent) {
		ret = _acpi_fill_ssdt(ctx, dev);
		if (ret)
			return log_msg_ret("recurse", ret);
	}

	return 0;
}

int acpi_fill_ssdt(struct acpi_ctx *ctx)
{
	int ret;

	log_debug("Writing SSDT tables\n");
	ret = _acpi_fill_ssdt(ctx, dm_root());
	log_debug("Writing SSDT finished, err=%d\n", ret);

	return ret;
}

int _acpi_write_dev_tables(struct acpi_ctx *ctx, const struct udevice *parent)
{
	struct acpi_ops *aops;
	struct udevice *dev;
	int ret;

	aops = device_get_acpi_ops(parent);
	if (aops && aops->write_tables) {
		log_debug("- %s\n", parent->name);
		ret = aops->write_tables(parent, ctx);
		if (ret)
			return ret;
	}
	device_foreach_child(dev, parent) {
		ret = _acpi_write_dev_tables(ctx, dev);
		if (ret)
			return ret;
	}

	return 0;
}

int acpi_write_dev_tables(struct acpi_ctx *ctx)
{
	int ret;

	log_debug("Writing device tables\n");
	ret = _acpi_write_dev_tables(ctx, dm_root());
	log_debug("Writing finished, err=%d\n", ret);

	return ret;
}
