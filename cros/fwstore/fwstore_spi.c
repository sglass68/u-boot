// SPDX-License-Identifier: GPL-2.0+
/*
 * Implementation of firmware storage access interface for SPI flash
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY UCLASS_CROS_FWSTORE

#include <common.h>
#include <dm.h>
#include <malloc.h>
#include <spi_flash.h>
#include <cros/cros_common.h>
#include <cros/cros_ofnode.h>
#include <cros/fwstore.h>

DECLARE_GLOBAL_DATA_PTR;

struct fwstore_spi_priv {
	struct udevice *sf;
};

/**
 * border_check() - Check if an offset/count are in range
 *
 * Return 0 if the region is is within range, -ESPIPE if the start offset is
 * beyond the end of the device, -ERANGE if the region extends beyond the
 * device
 */
static int border_check(struct udevice *sf, uint offset, uint count)
{
	struct spi_flash *flash = dev_get_uclass_priv(sf);
	uint max_offset = offset + count;

	if (offset >= flash->size) {
		log_debug("at EOF: offset=%x, size=%x\n", offset, flash->size);
		return -ESPIPE;
	}

	/* max_offset will be less than offset iff overflow occurred */
	if (max_offset < offset || max_offset > flash->size) {
		log_debug("exceed range offset=%x, max_offset=%x, flash->size=%x\n",
			  offset, max_offset, flash->size);
		return -ERANGE;
	}

	return 0;
}

static int fwstore_spi_read(struct udevice *dev, ulong offset, ulong count,
			    void *buf)
{
	struct fwstore_spi_priv *priv = dev_get_priv(dev);
	int ret;

	ret = border_check(priv->sf, offset, count);
	if (ret)
		return ret;

	ret = spi_flash_read_dm(priv->sf, offset, count, buf);
	if (ret) {
		log_debug("SPI read fail (count=%ld, ret=%d)\n", count, ret);
		return ret;
	}

	return 0;
}

/**
 * align_to_sector() - Align offset and size
 *
 * Align the right-exclusive range [*offsetp:*offsetp+*lengthp) with
 * the sector size.
 * After alignment adjustment, both offset and length will be multiple of
 * the sector, and will be larger than or equal to the original range.
 *
 * @sector_size: Sector size in bytes (e.g. 4096)
 * @offsetp: Pointer to offset to update
 * @lengthp: Pointer to length to update
 */
static void align_to_sector(uint sector_size, uint *offsetp, uint *lengthp)
{
	log_debug("before adjustment\n");
	log_debug("offset: 0x%x\n", *offsetp);
	log_debug("length: 0x%x\n", *lengthp);

	/* Adjust if offset is not multiple of sector_size */
	if (*offsetp & (sector_size - 1))
		*offsetp &= ~(sector_size - 1);

	/* Adjust if length is not multiple of sector_size */
	if (*lengthp & (sector_size - 1)) {
		*lengthp &= ~(sector_size - 1);
		*lengthp += sector_size;
	}

	log_debug("after adjustment\n");
	log_debug("offset: 0x%x\n", *offsetp);
	log_debug("length: 0x%x\n", *lengthp);
}

static int fwstore_spi_write(struct udevice *dev, ulong offset, ulong count,
			     void *buf)
{
	struct fwstore_spi_priv *priv = dev_get_priv(dev);
	struct spi_flash *flash = dev_get_uclass_priv(priv->sf);
	u8 static_buf[flash->sector_size];
	u8 *backup_buf;
	uint pos, len;
	int ret;

	/* We will erase <n> bytes starting from <pos> */
	pos = offset;
	len = count;
	align_to_sector(flash->sector_size, &pos, &len);

	log_debug("offset:          %08lx\n", offset);
	log_debug("adjusted offset: %08x\n", pos);
	if (pos > offset) {
		log_debug("align incorrect: %08x > %08lx\n", pos, offset);
		return -EINVAL;
	}

	if (border_check(priv->sf, pos, len))
		return -ERANGE;

	backup_buf = len > sizeof(static_buf) ? malloc(len) : static_buf;
	if (!backup_buf)
		return log_msg_ret("Cannot alloc fwstore tmp buf", -ENOMEM);

	ret = spi_flash_read_dm(priv->sf, pos, len, backup_buf);
	if (ret) {
		log_err("cannot backup data: %d\n", ret);
		goto exit;
	}

	ret = spi_flash_erase_dm(priv->sf, pos, len);
	if (ret) {
		log_err("SPI erase fail: %d\n", ret);
		goto exit;
	}

	/* combine data we want to write and backup data */
	memcpy(backup_buf + (offset - pos), buf, count);

	ret = spi_flash_write_dm(priv->sf, pos, len, backup_buf);
	if (ret) {
		log_err("SPI write fail: %d\n", ret);
		goto exit;
	}

	ret = 0;

exit:
	if (backup_buf != static_buf)
		free(backup_buf);

	return ret;
}

static int fwstore_spi_get_sw_write_prot(struct udevice *dev)
{
	struct fwstore_spi_priv *priv = dev_get_priv(dev);
	int ret;

	ret = spl_flash_get_sw_write_prot(priv->sf);
	if (ret < 0) {
		log_warning("spl_flash_get_write_prot_dm() failed: %d\n", ret);
		return 0;
	}
	log_debug("flash SW WP is %d\n", ret);

	return ret != 0;
}

int fwstore_spi_probe(struct udevice *dev)
{
	struct fwstore_spi_priv *priv = dev_get_priv(dev);
	struct ofnode_phandle_args args;
	int ret;

	log_debug("init %s\n", dev->name);
	ret = dev_read_phandle_with_args(dev, "firmware-storage", NULL, 0, 0,
					 &args);
	if (ret < 0) {
		log_debug("fail to look up phandle for device %s\n", dev->name);
		return ret;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_SPI_FLASH, args.node,
					  &priv->sf);
	if (ret) {
		log_debug("fail to init SPI flash at %s: %s: ret=%d\n",
			  dev->name, ofnode_get_name(args.node), ret);
		return ret;
	}

	return 0;
}

static const struct cros_fwstore_ops fwstore_spi_ops = {
	.read		= fwstore_spi_read,
	.write		= fwstore_spi_write,
	.sw_wp_enabled	= fwstore_spi_get_sw_write_prot,
};

static struct udevice_id fwstore_spi_ids[] = {
	{ .compatible = "cros,fwstore-spi" },
	{ },
};

U_BOOT_DRIVER(fwstore_spi) = {
	.name	= "fwstore_spi",
	.id	= UCLASS_CROS_FWSTORE,
	.of_match = fwstore_spi_ids,
	.ops	= &fwstore_spi_ops,
	.probe	= fwstore_spi_probe,
	.priv_auto_alloc_size = sizeof(struct fwstore_spi_priv),
};
