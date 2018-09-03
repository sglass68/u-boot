// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <dm.h>
#include <ec_commands.h>
#include <cros/nvdata.h>
#include <cros/vboot.h>

int cros_nvdata_read(struct udevice *dev, uint index, u8 *data, int size)
{
	struct cros_nvdata_ops *ops = cros_nvdata_get_ops(dev);

	if (!ops->read)
		return -ENOSYS;

	return ops->read(dev, index, data, size);
}

int cros_nvdata_write(struct udevice *dev, uint index, const u8 *data,
		      int size)
{
	struct cros_nvdata_ops *ops = cros_nvdata_get_ops(dev);

	if (!ops->write)
		return -ENOSYS;

	return ops->write(dev, index, data, size);
}

int cros_nvdata_setup(struct udevice *dev, uint index, uint attr, uint size,
		      const u8 *nv_policy, int nv_policy_size)
{
	struct cros_nvdata_ops *ops = cros_nvdata_get_ops(dev);

	if (!ops->setup)
		return -ENOSYS;

	return ops->setup(dev, index, attr, size, nv_policy, nv_policy_size);
}

int cros_nvdata_lock(struct udevice *dev, uint index)
{
	struct cros_nvdata_ops *ops = cros_nvdata_get_ops(dev);

	if (!ops->setup)
		return -ENOSYS;

	return ops->lock(dev, index);
}

int cros_nvdata_read_walk(uint index, u8 *data, int size)
{
	struct udevice *dev;
	int ret = -ENOSYS;

	uclass_foreach_dev_probe(UCLASS_CROS_NVDATA, dev) {
		ret = cros_nvdata_read(dev, index, data, size);
		if (!ret)
			break;
	}
	if (ret)
		return ret;

	return 0;
}

int cros_nvdata_write_walk(uint index, const u8 *data, int size)
{
	struct udevice *dev;
	int ret = -ENOSYS;

	uclass_foreach_dev_probe(UCLASS_CROS_NVDATA, dev) {
		log(UCLASS_CROS_NVDATA, LOGL_INFO, "write %s\n", dev->name);
		ret = cros_nvdata_write(dev, index, data, size);
		if (!ret)
			break;
	}
	if (ret)
		return ret;

	return 0;
}

int cros_nvdata_setup_walk(uint index, uint attr, uint size,
			   const u8 *nv_policy, uint nv_policy_size)
{
	struct udevice *dev;
	int ret = -ENOSYS;

	uclass_foreach_dev_probe(UCLASS_CROS_NVDATA, dev) {
		ret = cros_nvdata_setup(dev, index, attr, size, nv_policy,
					nv_policy_size);
		if (!ret)
			break;
	}
	if (ret)
		return ret;

	return 0;
}

int cros_nvdata_lock_walk(uint index)
{
	struct udevice *dev;
	int ret = -ENOSYS;

	uclass_foreach_dev_probe(UCLASS_CROS_NVDATA, dev) {
		ret = cros_nvdata_lock(dev, index);
		if (!ret)
			break;
	}
	if (ret)
		return ret;

	return 0;
}

VbError_t VbExNvStorageRead(u8 *buf)
{
	int ret;

	ret = cros_nvdata_read_walk(CROS_NV_DATA, buf, EC_VBNV_BLOCK_SIZE);
	if (ret)
		return VBERROR_UNKNOWN;
	print_buffer(0, buf, 1, EC_VBNV_BLOCK_SIZE, 0);

	return 0;
}

VbError_t VbExNvStorageWrite(const u8 *buf)
{
	int ret;

	printf("write\n");
	print_buffer(0, buf, 1, EC_VBNV_BLOCK_SIZE, 0);
	ret = cros_nvdata_write_walk(CROS_NV_DATA, buf, EC_VBNV_BLOCK_SIZE);
	if (ret)
		return VBERROR_UNKNOWN;

	return 0;
}

UCLASS_DRIVER(cros_nvdata) = {
	.id		= UCLASS_CROS_NVDATA,
	.name		= "cros_nvdata",
};
