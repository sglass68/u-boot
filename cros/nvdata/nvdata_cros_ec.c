// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY UCLASS_CROS_NVDATA

#include <common.h>
#include <dm.h>
#include <cros_ec.h>
#include <cros/nvdata.h>

static int cros_ec_nvdata_read(struct udevice *dev, uint index, u8 *data,
			       int size)
{
	struct udevice *cros_ec = dev_get_parent(dev);

	if (index != CROS_NV_DATA) {
		log_err("Only CROS_NV_DATA supported (not %x)\n", index);
		return -ENOSYS;
	}

	return cros_ec_read_nvdata(cros_ec, data, size);
}

static int cros_ec_nvdata_write(struct udevice *dev, uint index,
				const u8 *data, int size)
{
	struct udevice *cros_ec = dev_get_parent(dev);

	if (index != CROS_NV_DATA) {
		log_err("Only CROS_NV_DATA supported (not %x)\n", index);
		return -ENOSYS;
	}

	return cros_ec_write_nvdata(cros_ec, data, size);
}

static const struct cros_nvdata_ops cros_ec_nvdata_ops = {
	.read	= cros_ec_nvdata_read,
	.write	= cros_ec_nvdata_write,
};

static const struct udevice_id cros_ec_nvdata_ids[] = {
	{ .compatible = "google,cros-ec-nvdata" },
	{ }
};

U_BOOT_DRIVER(cros_ec_nvdata_drv) = {
	.name		= "cros-ec-nvdata",
	.id		= UCLASS_CROS_NVDATA,
	.of_match	= cros_ec_nvdata_ids,
	.ops		= &cros_ec_nvdata_ops,
};
