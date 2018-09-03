/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Chromium OS alternative firmware, used to update firmware on devices in the
 * system other than those using UCLASS_VBOOT_EC.
 *
 * Copyright 2018 Google LLC
 */

#ifndef __CROS_AUX_FW_H
#define __CROS_AUX_FW_H

enum aux_fw_severity {
	/* no update needed */
	AUX_FW_NO_UPDATE = 0,
	/* update needed, can be done quickly */
	AUX_FW_FAST_UPDATE = 1,
	/* update needed, "this would take a while..." */
	AUX_FW_SLOW_UPDATE = 2,
};

/**
 * struct aux_fw_ops - operations required by update process
 */
struct aux_fw_ops {
	/**
	 * check_hash() - Check the hash of the current firmware
	 *
	 * This sets @severityp after checking whether the current firmware
	 * matches the given hash.
	 *
	 * @dev: UCLASS_CROS_AUX_FW device
	 * @hash: Hash to check against
	 * @hash_size: Size of hash in bytes
	 * @severityp: returns severity value for this device
	 * @return 0 if OK, -ve on error
	 */
	int (*check_hash)(struct udevice *dev, const u8 *hash,
			  size_t hash_size, enum aux_fw_severity *severityp);

	/**
	 * update_image() - Update the firmware on the device
	 *
	 * @dev: UCLASS_CROS_AUX_FW device
	 * @image: Firmware image
	 * @image_size: Size of firmware image in bytes
	 * @return 0 if OK, -ERESTARTSYS to reboot to read-only firmware, other
	 *	-ve value on error
	 */
	int (*update_image)(struct udevice *dev, const u8 *image,
			    size_t image_size);

	/**
	 * get_protect() - Get the protect status of the connection to the EC
	 *
	 * @dev: UCLASS_CROS_AUX_FW device
	 * @return 0 if connection is not protected, 1 if protected, -ve on
	 *	error
	 */
	int (*get_protect)(struct udevice *dev);

	/**
	 * Set_protect() - Set the protect status of the connection to the EC
	 *
	 * @dev: UCLASS_CROS_AUX_FW device
	 * @protect: true to protect, false to unprotect
	 * @return 0 if OK, -ve on error
	 */
	int (*set_protect)(struct udevice *dev, bool protect);
};

#define aux_fw_get_ops(dev)	((struct aux_fw_ops *)(dev)->driver->ops)

/* See above for function comments for these stubs */
int aux_fw_check_hash(struct udevice *dev, const u8 *hash,
		      size_t hash_size, enum aux_fw_severity *severityp);
int aux_fw_update_image(struct udevice *dev, const u8 *image,
			size_t image_size);
int aux_fw_get_protect(struct udevice *dev);
int aux_fw_set_protect(struct udevice *dev, bool protect);

/**
 * aux_fw_get_severity() - Get the update severity recorded for a device
 *
 * This can be called after aux_fw_check_hash() to find out the update severity
 * returned by a device
 *
 * @dev: UCLASS_CROS_AUX_FW device
 */
enum aux_fw_severity aux_fw_get_severity(struct udevice *dev);

#endif /* __CROS_AUX_FW_H */
