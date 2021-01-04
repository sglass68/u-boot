// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY UCLASS_CROS_NVDATA

#include <common.h>
#include <dm.h>
#include <ec_commands.h>
#include <log.h>
#include <cros/nvdata.h>
#include <cros/vboot.h>

int cros_nvdata_read(struct udevice *dev, enum cros_nvdata_type type, u8 *data,
		     int size)
{
	struct cros_nvdata_ops *ops = cros_nvdata_get_ops(dev);

	if (!ops->read)
		return -ENOSYS;

	return ops->read(dev, type, data, size);
}

int cros_nvdata_write(struct udevice *dev, enum cros_nvdata_type type,
		      const u8 *data, int size)
{
	struct cros_nvdata_ops *ops = cros_nvdata_get_ops(dev);

	if (!ops->write)
		return -ENOSYS;

	return ops->write(dev, type, data, size);
}

int cros_nvdata_setup(struct udevice *dev, enum cros_nvdata_type type,
		      uint attr, uint size,
		      const u8 *nv_policy, int nv_policy_size)
{
	struct cros_nvdata_ops *ops = cros_nvdata_get_ops(dev);

	if (!ops->setup)
		return -ENOSYS;

	return ops->setup(dev, type, attr, size, nv_policy, nv_policy_size);
}

int cros_nvdata_lock(struct udevice *dev, enum cros_nvdata_type type)
{
	struct cros_nvdata_ops *ops = cros_nvdata_get_ops(dev);

	if (!ops->setup)
		return -ENOSYS;

	return ops->lock(dev, type);
}

int cros_nvdata_read_walk(enum cros_nvdata_type type, u8 *data, int size)
{
	struct udevice *dev;
	int ret = -ENOSYS;

	uclass_foreach_dev_probe(UCLASS_CROS_NVDATA, dev) {
		ret = cros_nvdata_read(dev, type, data, size);
		if (!ret)
			break;
	}
	if (ret)
		return ret;

	return 0;
}

int cros_nvdata_write_walk(enum cros_nvdata_type type, const u8 *data, int size)
{
	struct udevice *dev;
	int ret = -ENOSYS;

	log_info("write type %d size %x\n", type, size);
	uclass_foreach_dev_probe(UCLASS_CROS_NVDATA, dev) {
		log_info("   - try %s\n", dev->name);
		ret = cros_nvdata_write(dev, type, data, size);
		log_info("   - %s ret=%d\n", dev->name, ret);
		if (!ret)
			break;
	}
	if (ret) {
		log_warning("Failed to write type %d\n", type);
		return ret;
	}

	return 0;
}

int cros_nvdata_setup_walk(enum cros_nvdata_type type, uint attr, uint size,
			   const u8 *nv_policy, uint nv_policy_size)
{
	struct udevice *dev;
	int ret = -ENOSYS;

	uclass_foreach_dev_probe(UCLASS_CROS_NVDATA, dev) {
		ret = cros_nvdata_setup(dev, type, attr, size, nv_policy,
					nv_policy_size);
		if (!ret)
			break;
	}
	if (ret)
		return ret;

	return 0;
}

int cros_nvdata_lock_walk(enum cros_nvdata_type type)
{
	struct udevice *dev;
	int ret = -ENOSYS;

	uclass_foreach_dev_probe(UCLASS_CROS_NVDATA, dev) {
		ret = cros_nvdata_lock(dev, type);
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
#ifdef DEBUG
	print_buffer(0, buf, 1, EC_VBNV_BLOCK_SIZE, 0);
#endif

	return 0;
}

VbError_t VbExNvStorageWrite(const u8 *buf)
{
	int ret;

#ifdef DEBUG
	print_buffer(0, buf, 1, EC_VBNV_BLOCK_SIZE, 0);
#endif
	vboot_dump_nvdata(buf, EC_VBNV_BLOCK_SIZE);
	ret = cros_nvdata_write_walk(CROS_NV_DATA, buf, EC_VBNV_BLOCK_SIZE);
	if (ret)
		return VBERROR_UNKNOWN;

	return 0;
}

UCLASS_DRIVER(cros_nvdata) = {
	.id		= UCLASS_CROS_NVDATA,
	.name		= "cros_nvdata",
};
