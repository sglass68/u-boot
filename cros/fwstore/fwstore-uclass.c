// SPDX-License-Identifier: GPL-2.0+
/*
 * Interface for accessing the firmware image in storage (e.g. SPI flash)
 *
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <dm.h>
#include <cros/fwstore.h>
#include <dm/device-internal.h>

int cros_fwstore_read(struct udevice *dev, int offset, int count, void *buf)
{
	struct cros_fwstore_ops *ops = cros_fwstore_get_ops(dev);

	if (!ops->read)
		return -ENOSYS;

	return ops->read(dev, offset, count, buf);
}

int fwstore_read_decomp(struct udevice *dev, struct fmap_entry *entry,
			void *buf, int buf_size)
{
	struct cros_fwstore_ops *ops = cros_fwstore_get_ops(dev);
	u8 *start;
	int ret;

	if (!ops->read)
		return -ENOSYS;

	/* Read the data into the buffer */
	if (entry->compress_algo == FMAP_COMPRESS_NONE) {
		start = buf;
	} else {
		if (buf_size < entry->unc_length)
			return -ENOSPC;
		start = buf + (buf_size - entry->unc_length);
	}
	ret = ops->read(dev, entry->offset, entry->length, start);
	if (ret)
		return log_ret(ret);

	/* Decompress if needed */
	if (entry->compress_algo == FMAP_COMPRESS_LZ4) {
		size_t out_size = buf_size;

		ret = ulz4fn(start, entry->length, buf, &out_size);
		if (ret)
			return log_msg_ret("decompress lz4", ret);
	}

	return 0;
}

int fwstore_get_reader_dev(struct udevice *fwstore, int offset, int size,
			   struct udevice **devp)
{
	struct udevice *dev;
	int ret;

	if (device_find_first_inactive_child(fwstore, UCLASS_MISC, &dev)) {
		ret = device_bind_ofnode(fwstore,
					 DM_GET_DRIVER(fwstore_reader),
					 "fwstore_reader", 0, ofnode_null(),
					 &dev);
		if (ret)
			return log_msg_ret("bind failed", ret);
	}
	fwstore_reader_setup(dev, offset, size);
	ret = device_probe(dev);
	if (ret)
		return ret;
	*devp = dev;

	return 0;
}

int fwstore_load_image(struct udevice *dev, struct fmap_entry *entry,
		       u8 **imagep, int *image_sizep)
{
	void *data, *buf;
	size_t buf_size;
	int ret;

	if (!entry->length)
		return log_msg_ret("no image", -ENOENT);
	data = malloc(entry->length);
	if (!data)
		return log_msg_ret("allocate space for image", -ENOMEM);
	ret = cros_fwstore_read(dev, entry->offset, entry->length, data);
	if (ret)
		return log_msg_ret("read image", ret);

	switch (entry->compress_algo) {
	case FMAP_COMPRESS_NONE:
		*imagep = data;
		*image_sizep = entry->length;
		break;
	case FMAP_COMPRESS_LZ4:
		buf_size = entry->unc_length;
		buf = malloc(buf_size);
		if (!buf)
			return log_msg_ret("allocate decomp buf", -ENOMEM);
		ret = ulz4fn(data, entry->length, buf, &buf_size);
		if (ret)
			return log_msg_ret("decompress lz4", ret);
		*imagep = buf;
		*image_sizep = buf_size;
		free(data);
		break;
	}

	return 0;
}

UCLASS_DRIVER(cros_fwstore) = {
	.id		= UCLASS_CROS_FWSTORE,
	.name		= "cros_fwstore",
};
