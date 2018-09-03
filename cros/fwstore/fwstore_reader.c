// SPDX-License-Identifier: GPL-2.0+
/*
 * A misc device that reads from a section of a fwstore
 *
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY UCLASS_CROS_FWSTORE

#include <common.h>
#include <dm.h>
#include <misc.h>
#include <cros/fwstore.h>

/**
 * struct fwstore_reader_platdata - information about a firmware reader
 *
 * @cur: current position within the start/size region
 * @base_offset: start offset of region in fwstore
 * @size: size of region in fwstore
 */
struct fwstore_reader_platdata {
	int cur;
	int base_offset;
	int size;
};

void fwstore_reader_setup(struct udevice *dev, int offset, int size)
{
	struct fwstore_reader_platdata *plat = dev_get_platdata(dev);

	plat->base_offset = offset;
	plat->size = size;
	plat->cur = 0;
	log_debug("'%s': setup, base_offset=%x, size=%x\n", dev->name,
		  plat->base_offset, plat->size);
}

int fwstore_reader_size(struct udevice *dev)
{
	struct fwstore_reader_platdata *plat = dev_get_platdata(dev);

	return plat->size;
}

int fwstore_reader_restrict(struct udevice *dev, int offset, int size)
{
	struct fwstore_reader_platdata *plat = dev_get_platdata(dev);

	if (offset < 0 || offset >= plat->size)
		return -EINVAL;
	if (offset + size > plat->size)
		size = plat->size - offset;

	plat->base_offset += offset;
	plat->size = size;
	plat->cur = 0;
	log_debug("Restricting '%s' to offset=%x, size=%x\n", dev->name,
		  plat->base_offset, plat->size);

	return 0;
}

static int fwstore_reader_read(struct udevice *dev, int offset, void *buf,
			       int size)
{
	struct fwstore_reader_platdata *plat = dev_get_platdata(dev);
	int pos, ret;

	/* Figure out where to read from, a do a range check */
	pos = offset == -1 ? plat->cur : offset;
	log_debug("%s: pos %x, size=%x\n", dev->name, pos, plat->size);
	if (pos < 0 || pos >= plat->size)
		return 0;
	if (pos + size > plat->size)
		size = plat->size - pos;

	/* Read the data and update our current position */
	pos += plat->base_offset;
	ret = cros_fwstore_read(dev_get_parent(dev), pos, size, buf);
	if (ret)
		return ret;
	plat->cur += size;
	log_debug("%s: read %x at %x, offset=%x, size=%x, limit=%x\n",
		  dev->name, size, pos, plat->base_offset, plat->size,
		  plat->base_offset + plat->size);

	return size;
}

static struct misc_ops fwstore_reader_ops = {
	.read	= fwstore_reader_read,
};

U_BOOT_DRIVER(fwstore_reader) = {
	.name		= "fwstore_reader",
	.id		= UCLASS_MISC,
	.platdata_auto_alloc_size = sizeof(struct fwstore_reader_platdata),
	.ops		= &fwstore_reader_ops,
};
