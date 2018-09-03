/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Chromium OS vboot EC uclass, used for vboot operations implemented by an EC
 * that uses the Chromium OS code base / messages.
 *
 * Copyright 2018 Google LLC
 */

#ifndef __CROS_VBOOT_EC_H
#define __CROS_VBOOT_EC_H

/**
 * struct vboot_ec_ops - EC operations required by vboot
 *
 * These directly correspond to normal vboot VbExEc... interfaces.
 */
struct vboot_ec_ops {
	/**
	 * running_rw() - Check if the EC is currently running rewriteable code
	 *
	 * @dev: UCLASS_CROS_VBOOT_EC device
	 * @in_rwp: Returns 0 if the EC is in RO code, 1 if not
	 * @return 0 if OK, other value if the current EC image is unknown
	 */
	int (*running_rw)(struct udevice *dev, int *in_rwp);

	/** jump_to_rw() - Request the EC jump to its rewriteable code
	 *
	 * If successful, returns when the EC has booting its RW code far enough
	 * to respond to subsequent commands. Does nothing if the EC is already
	 * in its rewriteable code.
	 *
	 * @dev: UCLASS_CROS_VBOOT_EC device
	 * @return 0 if OK, non-zero on error
	 */
	int (*jump_to_rw)(struct udevice *dev);

	/**
	 * disable_jump() - Tell the EC to refuse another jump until it reboots
	 *
	 * After this is called, subsequent calls to jump_to_rw() in this boot
	 * will fail
	 *
	 * @dev: UCLASS_CROS_VBOOT_EC device
	 * @return 0 if OK, non-zero on error
	 */
	int (*disable_jump)(struct udevice *dev);

	/**
	 * hash_image() - Read the SHA-256 hash of the selected EC image
	 *
	 * @dev: UCLASS_CROS_VBOOT_EC device
	 * @devidx:	Device index. 0: EC, 1: PD
	 * @select:	Image to get hash of. RO or RW
	 * @hash:	Pointer to the hash
	 * @hash_sizep:	Pointer to the hash size
	 *
	 * @return 0 if OK, non-zero on error
	 */
	int (*hash_image)(struct udevice *dev, enum VbSelectFirmware_t select,
			  const u8 **hashp, int *hash_sizep);

	/**
	 * update_image() - Update the selected EC image
	 *
	 * @dev: UCLASS_CROS_VBOOT_EC device
	 * @devidx:	Device index. 0: EC, 1: PD
	 * @select:	Image to get hash of. RO or RW
	 * @image:	Pointer to the image
	 * @image_size:	Size of the image in bytes
	 * @return 0 if OK, non-zero on error
	 */
	int (*update_image)(struct udevice *dev, enum VbSelectFirmware_t select,
			    const u8 *image, int image_size);

	/**
	 * protect() - Lock the selected EC code until the EC is rebooted
	 *
	 * This prevents updates until the EC is rebooted. Subsequent calls to
	 * update_image() with the same region this boot will fail.
	 *
	 * @dev: UCLASS_CROS_VBOOT_EC device
	 * @return 0 if OK, non-zero on error
	 */
	int (*protect)(struct udevice *dev, enum VbSelectFirmware_t select);

	/**
	 * entering_mode() - Inform the EC of the boot mode selected by the AP
	 * mode: Normal, Developer, or Recovery
	 *
	 * @dev: UCLASS_CROS_VBOOT_EC device
	 * @return 0 if OK, non-zero on error
	 */
	int (*entering_mode)(struct udevice *dev, enum VbEcBootMode_t mode);

	/**
	 * reboot_to_ro() Tells the EC to reboot to RO on next AP shutdown
	 *
	 * @dev: UCLASS_CROS_VBOOT_EC device
	 * @return 0 if OK, non-zero on error
	 */
	int (*reboot_to_ro)(struct udevice *dev);
};

#define vboot_ec_get_ops(dev)	((struct vboot_ec_ops *)(dev)->driver->ops)

/* See above for comments for these wrapper functions */
int vboot_ec_running_rw(struct udevice *dev, int *in_rwp);
int vboot_ec_jump_to_rw(struct udevice *dev);
int vboot_ec_disable_jump(struct udevice *dev);
int vboot_ec_hash_image(struct udevice *dev, enum VbSelectFirmware_t select,
			const u8 **hashp, int *hash_sizep);
int vboot_ec_update_image(struct udevice *dev, enum VbSelectFirmware_t select,
			  const u8 *image, int image_size);
int vboot_ec_protect(struct udevice *dev, enum VbSelectFirmware_t select);
int vboot_ec_entering_mode(struct udevice *dev, enum VbEcBootMode_t mode);
int vboot_ec_reboot_to_ro(struct udevice *dev);

#endif /* __CROS_VBOOT_EC_H */
