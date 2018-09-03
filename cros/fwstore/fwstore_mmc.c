// SPDX-License-Identifier: GPL-2.0+
/*
 * Implementation of firmware storage access interface for MMC
 * NOTE THAT THIS CODE IS EFFECTIVELY UNTESTED and is for example only
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY UCLASS_CROS_FWSTORE

#include <common.h>
#include <dm.h>
#include <malloc.h>
#include <mmc.h>
#include <cros/cros_common.h>
#include <cros/cros_ofnode.h>
#include <cros/fwstore.h>

DECLARE_GLOBAL_DATA_PTR;

struct priv {
	struct udevice *dev;
	struct mmc *mmc;
	u32 ro_section_size;	/* Offsets bigger are on partition 2 */
	struct blk_desc *blk;
};

/*
 * Determine which boot partition the access falls within.
 *
 * Return partition number, or -EINVAL if discontinuous
 */
static int check_partition(struct priv *priv, u32 offset, u32 count)
{
	int partition = 1;

	/*
	 * Check continuity
	 * If it starts in partition 1, and ends in partition 2, things
	 * will not go well.
	 */
	if (offset < priv->ro_section_size &&
	    offset + count >= priv->ro_section_size) {
		log_debug("Boot partition access not contiguous\n");
		return -EINVAL;
	}

	/* Offsets not in the RO section must be in partition 2 */
	if (offset >= priv->ro_section_size)
		partition = 2;

	return partition;
}

static int fwstore_mmc_read(struct udevice *dev, ulong offset, ulong count,
			    void *buf)
{
	struct priv *priv = dev_get_priv(dev);
	int partition, err;
	u8 *tmp_buf;
	int n, ret = -1;
	int start_block, start_block_offset, end_block, total_blocks;

	log_debug("offset=%#lx, count=%#lx\n", offset, count);
	partition = check_partition(priv, offset, count);
	if (partition < 0)
		return -1;

	if (partition == 2) {
		log_debug("Reading from partition 2\n");
		offset -= priv->ro_section_size;
	}

	start_block = offset / MMC_MAX_BLOCK_LEN;
	start_block_offset = offset % MMC_MAX_BLOCK_LEN;
	end_block = (offset + count) / MMC_MAX_BLOCK_LEN;

	/* Read start to end, inclusive */
	total_blocks = end_block - start_block + 1;

	log_debug("Reading %d blocks\n", total_blocks);

	tmp_buf = malloc(MMC_MAX_BLOCK_LEN * total_blocks);
	if (!tmp_buf) {
		log_debug("Failed to allocate buffer\n");
		goto out;
	}

	/* Open partition */
	err = mmc_hwpart_access(priv->mmc, partition);
	if (err) {
		log_debug("Failed to open boot partition %d\n", partition);
		goto out_free;
	}

	/* Read data */
	n = blk_dread(priv->blk, start_block, total_blocks, tmp_buf);
	if (n != total_blocks) {
		log_debug("Failed to read blocks\n");
		goto out_close;
	}

	/* Copy to output buffer */
	memcpy(buf, tmp_buf + start_block_offset, count);

	ret = 0;

out_close:
	/* Close partition */
	err = mmc_hwpart_access(priv->mmc, 0);
	if (err) {
		log_debug("Failed to close boot partition\n");
		ret = -1;
	}

out_free:
	free(tmp_buf);
out:
	return ret;
}

/*
 * Does not support unaligned writes.
 * Offset and count must be offset-aligned.
 */
static int fwstore_mmc_write(struct udevice *dev, ulong offset, ulong count,
			     void *buf)
{
	struct priv *priv = dev_get_priv(dev);
	u32 num, start_block, total_blocks;
	int partition, err, ret = 0;

	/* Writes not aligned to block size are unsupported */
	if (offset % MMC_MAX_BLOCK_LEN) {
		log_debug("Offset of %ld bytes not aligned to 512 byte boundary\n",
			  offset);
		return -1;
	}

	if (count % MMC_MAX_BLOCK_LEN) {
		log_debug("Count of %ld bytes not aligned to 512 byte boundary\n",
			  count);
		return -1;
	}

	/* Determine partition */
	partition = check_partition(priv, offset, count);
	if (partition < 0)
		return -1;

	if (partition == 2) {
		log_debug("Writing to partition 2\n");
		offset -= priv->ro_section_size;
	}

	start_block = offset / MMC_MAX_BLOCK_LEN;
	total_blocks = count / MMC_MAX_BLOCK_LEN;

	/* Open partition */
	err = mmc_hwpart_access(priv->mmc, partition);
	if (err) {
		log_debug("Failed to open boot partition %d\n", partition);
		return -1;
	}

	/* Write data */
	num = blk_dwrite(priv->blk, start_block, total_blocks, buf);
	if (num != total_blocks) {
		log_debug("Failed to write blocks\n");
		ret = -1;
		goto out;
	}

out:
	/* Close partition */
	err = mmc_hwpart_access(priv->mmc, 0);
	if (err) {
		log_debug("Failed to close boot partition\n");
		return -1;
	}

	return ret;
}

static int fwstore_mmc_sw_wp_enabled_mmc(struct udevice *dev)
{
	struct priv *priv = dev_get_priv(dev);

	return mmc_get_boot_wp(priv->mmc);
}

static int fwstore_mmc_probe(struct udevice *dev)
{
	struct priv *priv = dev_get_priv(dev);
	struct ofnode_phandle_args args;
	int ret;

	log_debug("init %s\n", dev->name);
	ret = dev_read_phandle_with_args(dev, "firmware-storage", NULL, 0, 0,
					 &args);
	if (ret < 0) {
		log_debug("fail to look up phandle for device %s\n", dev->name);
		return ret;
	}

	ret = uclass_get_device_by_ofnode(UCLASS_MMC, args.node, &priv->dev);
	if (ret) {
		log_debug("fail to init MMC at %s: %s: ret=%d\n",
			  dev->name, ofnode_get_name(args.node), ret);
		return ret;
	}

	/* TODO(sjg@chromium.org): Lookup partition size in binman table */
	priv->ro_section_size = 2 << 20;
	priv->mmc = mmc_get_mmc_dev(priv->dev);
	priv->blk = blk_get_by_device(priv->dev);

	return 0;
}

static const struct cros_fwstore_ops fwstore_mmc_ops = {
	.read		= fwstore_mmc_read,
	.write		= fwstore_mmc_write,
	.sw_wp_enabled	= fwstore_mmc_sw_wp_enabled_mmc,
};

static struct udevice_id fwstore_mmc_ids[] = {
	{ .compatible = "google,fwstore-mmc" },
	{ },
};

U_BOOT_DRIVER(fwstore_mmc) = {
	.name	= "fwstore_mmc",
	.id	= UCLASS_CROS_FWSTORE,
	.of_match = fwstore_mmc_ids,
	.ops	= &fwstore_mmc_ops,
	.probe	= fwstore_mmc_probe,
	.priv_auto_alloc_size = sizeof(struct priv),
};
