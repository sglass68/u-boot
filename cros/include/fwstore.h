/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Interface for access the firmware image in storage (e.g. SPI flash)
 *
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#ifndef __CROS_FWSTORE_H_
#define __CROS_FWSTORE_H_

#include <dm/of_extra.h>

/**
 * These read or write [count] bytes starting from [offset] of storage into or
 * from the [buf].
 *
 * @return 0 if it succeeds, non-zero if it fails
 */
struct cros_fwstore_ops {
	/**
	 * read() - read data
	 *
	 * @dev:	Device to read from
	 * @offset:	Offset within device to read from in bytes
	 * @count:	Number of bytes to read
	 * @buf:	Buffer to place data
	 * @return 0 if OK, -ve on error
	 */
	int (*read)(struct udevice *dev, ulong offset, ulong count, void *buf);

	/**
	 * write() - write data
	 *
	 * @dev:	Device to write to
	 * @offset:	Offset within device to write to in bytes
	 * @count:	Number of bytes to write
	 * @buf:	Buffer containg data to write
	 * @return 0 if OK, -ve on error
	 */
	int (*write)(struct udevice *dev, ulong offset, ulong count, void *buf);

	/**
	 * sw_wp_enabled() - see if software write protect is enabled
	 *
	 * @dev:	Device to check
	 * @return	1 if sw wp is enabled, 0 if disabled, -ve on error
	 */
	int (*sw_wp_enabled)(struct udevice *dev);
};

#define cros_fwstore_get_ops(dev) \
		((struct cros_fwstore_ops *)(dev)->driver->ops)

/**
 * cros_fwstore_read() - read data
 *
 * @dev:	Device to read from
 * @offset:	Offset within device to read from in bytes
 * @count:	Number of bytes to read
 * @buf:	Buffer to place data
 * @return 0 if OK, -ve on error
 */
int cros_fwstore_read(struct udevice *dev, int offset, int count, void *buf);

/**
 * cros_fwstore_write() - write data
 *
 * @dev:	Device to write to
 * @offset:	Offset within device to write to in bytes
 * @count:	Number of bytes to write
 * @buf:	Buffer containg data to write
 * @return 0 if OK, -ve on error
 */
int cros_fwstore_write(struct udevice *dev, int offset, int count, void *buf);

/**
 * cros_fwstore_get_sw_write_prot() - see if software write protect is enabled
 *
 * @dev:	Device to check
 * @return	1 if sw wp is enabled, 0 if disabled, -ve on error
 */
int cros_fwstore_get_sw_write_prot(struct udevice *dev);

/**
 * fwstore_reader_setup() - Set up an existing reader for SPI flash
 *
 * This sets the platform data for the reader device so that it can operate
 * correctly. The device should be inactive. It is not probed by this function.
 *
 * @dev: Device to set up
 * @offset: Start offset in flash to allow the device to access
 * @size: Size of region to allow the device to access
 */
void fwstore_reader_setup(struct udevice *dev, int offset, int size);

/**
 * fwstore_reader_restrict() - Restrict the boundaries of a device
 *
 * This reduces the size of an fwstore reader to extend only between the new
 * offset and size, relative to the existing size of the device.
 *
 * @dev: reader device to update
 * @offset: Offset within the parent device to start from
 * @size: Number of bytes to present as available
 * @return 0 if OK, -EINVAL if the postion is out of range, -ve on error
 */
int fwstore_reader_restrict(struct udevice *dev, int offset, int size);

/**
 * fwstore_reader_size() - Get the size of a reader
 *
 * @dev: reader device to check
 * @return reader size in bytes
 */
int fwstore_reader_size(struct udevice *dev);

/**
 * fwstore_get_reader_dev() - Create a reader from an fwstore device
 *
 * fwstore: Device to read from (UCLASS_CROS_FWSTORE)
 * @offset: Start offset within device
 * @size: Size within device (starting at @offset)
 * @devp: Returns new reader device (UCLASS_MISC)
 * @return 0 if OK, -ve on error
 */
int fwstore_get_reader_dev(struct udevice *fwstore, int offset, int size,
			   struct udevice **devp);

/**
 * fwstore_load_image() - Allocate and load an image from the firmware store
 *
 * This allocates memory for the image and returns a pointer to it.
 *
 * @dev: Device to load from
 * @entry:	Flash entry to load (provides offset, size, compression,
 *		uncompressed size)
 * @imagep:	Returns a pointer to the data (must be freed by caller)
 * @image_sizep: Returns image size
 * @return 0 if OK, -ENOENT if the image has a zero size, -ENOMEM if there is
 *	not enough memory for the buffer, other error on read failre
 */
int fwstore_load_image(struct udevice *dev, struct fmap_entry *entry,
		       u8 **imagep, int *image_sizep);

/**
 * fwstore_read_decomp: Read and decompress data into a buffer
 *
 * @dev: Device to read from (UCLASS_FWSTORE)
 * @entry: Flashmap entry to read from (indicates position, compression, etc.)
 * @buf: Buffer to read into (make sure it is large enough for the uncompressed
 *	data plus a good margin, to allow for uncompression
 * @buf_size: Size of buffer
 * @return 0 if OK, -ve on error
 */
int fwstore_read_decomp(struct udevice *dev, struct fmap_entry *entry,
			void *buf, int buf_size);

#endif /* __CROS_FWSTORE_H_ */
