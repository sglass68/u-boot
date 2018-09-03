// SPDX-License-Identifier: GPL-2.0+
/*
 * Chromium OS vboot EC uclass, used for vboot operations implemented by an EC
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <dm.h>
#include <cros_ec.h>
#include <cros/vboot.h>
#include <cros/vboot_ec.h>

int cros_ec_vboot_running_rw(struct udevice *dev, int *in_rw)
{
	struct udevice *ec_dev = dev_get_parent(dev);
	enum ec_current_image image;
	int ret;

	ret = cros_ec_read_current_image(ec_dev, &image);
	if (ret < 0)
		return ret;

	switch (image) {
	case EC_IMAGE_RO:
		*in_rw = 0;
		break;
	case EC_IMAGE_RW:
		*in_rw = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int cros_ec_vboot_jump_to_rw(struct udevice *dev)
{
	struct udevice *ec_dev = dev_get_parent(dev);
	int ret;

	ret = cros_ec_reboot(ec_dev, EC_REBOOT_JUMP_RW, 0);
	if (ret < 0)
		return ret;

	return 0;
}

int cros_ec_vboot_disable_jump(struct udevice *dev)
{
	struct udevice *ec_dev = dev_get_parent(dev);
	int ret;

	ret = cros_ec_reboot(ec_dev, EC_REBOOT_DISABLE_JUMP, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static u32 get_vboot_hash_offset(enum VbSelectFirmware_t select)
{
	switch (select) {
	case VB_SELECT_FIRMWARE_READONLY:
		return EC_VBOOT_HASH_OFFSET_RO;
	case VB_SELECT_FIRMWARE_EC_UPDATE:
		return EC_VBOOT_HASH_OFFSET_UPDATE;
	default:
		return EC_VBOOT_HASH_OFFSET_ACTIVE;
	}
}

int cros_ec_vboot_hash_image(struct udevice *dev,
			     enum VbSelectFirmware_t select,
			     const u8 **hashp, int *hash_sizep)
{
	struct udevice *ec_dev = dev_get_parent(dev);
	static struct ec_response_vboot_hash resp;
	u32 hash_offset;
	int ret;

	hash_offset = get_vboot_hash_offset(select);

	ret = cros_ec_read_hash(ec_dev, hash_offset, &resp);
	if (ret)
		return ret;
	*hash_sizep = resp.size;
	*hashp = resp.hash_digest;

	return 0;
}

static int vboot_set_region_protection(struct udevice *ec_dev,
				       enum VbSelectFirmware_t select,
				       int enable)
{
	struct ec_response_flash_protect resp;
	u32 protected_region = EC_FLASH_PROTECT_ALL_NOW;
	u32 mask = EC_FLASH_PROTECT_ALL_NOW | EC_FLASH_PROTECT_ALL_AT_BOOT;
	int ret;

	if (select == VB_SELECT_FIRMWARE_READONLY)
		protected_region = EC_FLASH_PROTECT_RO_NOW;

	/* Update protection */
	ret = cros_ec_flash_protect(ec_dev, mask, enable ? mask : 0, &resp);
	if (ret < 0) {
		log_err("Failed to update EC flash protection\n");
		return ret;
	}

	if (!enable) {
		/* If protection is still enabled, need reboot */
		if (resp.flags & protected_region)
			return -EPERM;

		return 0;
	}

	/*
	 * If write protect and ro-at-boot aren't both asserted, don't expect
	 * protection enabled.
	 */
	if (~resp.flags & (EC_FLASH_PROTECT_GPIO_ASSERTED |
			   EC_FLASH_PROTECT_RO_AT_BOOT))
		return 0;

	/* If flash is protected now, success */
	if (resp.flags & EC_FLASH_PROTECT_ALL_NOW)
		return 0;

	/* If RW will be protected at boot but not now, need a reboot */
	if (resp.flags & EC_FLASH_PROTECT_ALL_AT_BOOT)
		return -EPERM;

	/* Otherwise, it's an error */
	return -EIO;
}

static enum ec_flash_region vboot_to_ec_region(enum VbSelectFirmware_t select)
{
	switch (select) {
	case VB_SELECT_FIRMWARE_READONLY:
		return EC_FLASH_REGION_WP_RO;
	case VB_SELECT_FIRMWARE_EC_UPDATE:
		return EC_FLASH_REGION_UPDATE;
	default:
		return EC_FLASH_REGION_ACTIVE;
	}
}

int cros_ec_vboot_update_image(struct udevice *dev,
			       enum VbSelectFirmware_t select,
			       const u8 *image, int image_size)
{
	struct udevice *ec_dev = dev_get_parent(dev);
	u32 region_offset, region_size;
	enum ec_flash_region region;
	int ret;

	region = vboot_to_ec_region(select);
	ret = vboot_set_region_protection(ec_dev, select, 0);
	if (ret)
		return ret;

	ret = cros_ec_flash_offset(ec_dev, region, &region_offset,
				   &region_size);
	if (ret)
		return ret;
	if (image_size > region_size)
		return -EINVAL;

	/*
	 * Erase the entire region, so that the EC doesn't see any garbage
	 * past the new image if it's smaller than the current image.
	 *
	 * TODO: could optimise this to erase just the current image, since
	 * presumably everything past that is 0xff's.  But would still need to
	 * round up to the nearest multiple of erase size.
	 */
	ret = cros_ec_flash_erase(ec_dev, region_offset, region_size);
	if (ret)
		return ret;

	/* Write the image */
	ret = cros_ec_flash_write(ec_dev, image, region_offset, image_size);
	if (ret)
		return ret;

	/* Verify the image */
	ret = cros_ec_efs_verify(ec_dev, region);
	if (ret)
		return ret;

	return 0;
}

int cros_ec_vboot_protect(struct udevice *dev, enum VbSelectFirmware_t select)
{
	struct udevice *ec_dev = dev_get_parent(dev);

	return vboot_set_region_protection(ec_dev, select, 1);
}

int cros_ec_vboot_entering_mode(struct udevice *dev, enum VbEcBootMode_t mode)
{
	struct udevice *ec_dev = dev_get_parent(dev);
	int ret;

	ret = cros_ec_entering_mode(ec_dev, mode);
	if (ret)
		return ret;

	return 0;
}

int cros_ec_vboot_reboot_to_ro(struct udevice *dev)
{
	return 0;
}

static const struct vboot_ec_ops cros_ec_vboot_ops = {
	.running_rw	= cros_ec_vboot_running_rw,
	.jump_to_rw	= cros_ec_vboot_jump_to_rw,
	.running_rw	= cros_ec_vboot_running_rw,
	.hash_image	= cros_ec_vboot_hash_image,
	.update_image	= cros_ec_vboot_update_image,
	.protect	= cros_ec_vboot_protect,
	.entering_mode	= cros_ec_vboot_entering_mode,
	.reboot_to_ro	= cros_ec_vboot_reboot_to_ro,
};

static const struct udevice_id cros_ec_vboot_ids[] = {
	{ .compatible = "google,cros-ec-vboot" },
	{ }
};

U_BOOT_DRIVER(cros_ec_vboot) = {
	.name		= "cros_ec_vboot",
	.id		= UCLASS_CROS_VBOOT_EC,
	.of_match	= cros_ec_vboot_ids,
	.ops		= &cros_ec_vboot_ops,
};
