// SPDX-License-Identifier: GPL-2.0+
/*
 * Implementation of EC callbacks
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY UCLASS_CROS_EC

#include <common.h>
#include <dm.h>
#include <cros_ec.h>
#include <malloc.h>
#include <vb2_api.h>
#include <cros/cros_common.h>
#include <cros/fwstore.h>
#include <cros/vboot.h>
#include <cros/vboot_ec.h>
#include <cros/vboot_flag.h>
#include <cros/aux_fw.h>

DECLARE_GLOBAL_DATA_PTR;

int VbExTrustEC(int devidx)
{
	int gpio_ec_in_rw;
	int okay;

	log_debug("%s: %d\n", __func__, devidx);
	if (devidx != 0)
		return 0;

	/* If we don't have a valid GPIO to read, we can't trust it */
	gpio_ec_in_rw = vboot_flag_read_walk(VBOOT_FLAG_EC_IN_RW);
	if (gpio_ec_in_rw < 0) {
		log_debug("can't find GPIO to read, returning 0\n");
		return 0;
	}

	/* We only trust it if it's NOT in its RW firmware */
	okay = !gpio_ec_in_rw;

	log_debug("value=%d, returning %d\n", gpio_ec_in_rw, okay);

	return okay;
}

static int ec_get(int devidx, struct udevice **devp)
{
	struct udevice *dev;
	int ret;

	log_debug("%s: %d\n", __func__, devidx);
	ret = uclass_get_device_by_seq(UCLASS_CROS_VBOOT_EC, devidx, &dev);
	if (ret) {
		log_err("Get EC %d: err=%d\n", devidx, ret);
		return VBERROR_UNKNOWN;
	}
	log_debug("EC = %s\n", dev->name);
	*devp = dev;

	return 0;
}

VbError_t VbExEcRunningRW(int devidx, int *in_rw)
{
	struct udevice *dev;
	int ret;

	log_debug("%s: %d\n", __func__, devidx);
	ret = ec_get(devidx, &dev);
	if (ret)
		return log_msg_ret("Cannot get EC", ret);

	ret = vboot_ec_running_rw(dev, in_rw);
	if (ret) {
		log_err("Failed, err=%d\n", ret);
		return VBERROR_UNKNOWN;
	}

	return VBERROR_SUCCESS;
}

VbError_t VbExEcJumpToRW(int devidx)
{
	struct udevice *dev;
	int ret;

	log_debug("%s: %d\n", __func__, devidx);
	ret = ec_get(devidx, &dev);
	if (ret)
		return log_msg_ret("Cannot get EC", ret);

	ret = vboot_ec_jump_to_rw(dev);
	if (ret) {
		log_err("Failed, err=%d\n", ret);
		return VBERROR_UNKNOWN;
	}

	return VBERROR_SUCCESS;
}

VbError_t VbExEcDisableJump(int devidx)
{
	struct udevice *dev;
	int ret;

	log_debug("%s: %d\n", __func__, devidx);
	ret = uclass_get_device_by_seq(UCLASS_CROS_VBOOT_EC, devidx, &dev);
	ret = ec_get(devidx, &dev);
	if (ret)
		return log_msg_ret("Cannot get EC", ret);

	ret = vboot_ec_disable_jump(dev);
	if (ret) {
		log_err("Failed, err=%d\n", ret);
		return VBERROR_UNKNOWN;
	}

	return VBERROR_SUCCESS;
}

VbError_t VbExEcHashImage(int devidx, enum VbSelectFirmware_t select,
			  const u8 **hashp, int *hash_sizep)
{
	struct udevice *dev;
	int ret;

	ret = ec_get(devidx, &dev);
	if (ret)
		return log_msg_ret("Cannot get EC", ret);

	ret = vboot_ec_hash_image(dev, select, hashp, hash_sizep);
	if (ret) {
		log_err("Failed, err=%d\n", ret);
		return VBERROR_UNKNOWN;
	}

	return VBERROR_SUCCESS;
}

static struct fmap_entry *get_firmware_entry(struct vboot_info *vboot,
					     int devidx,
					     enum VbSelectFirmware_t select)
{
	struct fmap_firmware_entry *fw;
	struct fmap_firmware_ec *ec;
	struct fmap_entry *entry;

	fw = vboot_is_slot_a(vboot) ? &vboot->fmap.readwrite_a :
		&vboot->fmap.readwrite_b;
	if (devidx < 0 || devidx >= EC_COUNT) {
		log_err("entry not found, slot=%s, devidx=%d, select=%d",
			vboot_slot_name(vboot), devidx, select);
		return NULL;
	}
	ec = &fw->ec[devidx];
	entry = select == VB_SELECT_FIRMWARE_READONLY ? &ec->ro : &ec->rw;
	log_debug("Selected devidx=%d, select=%s\n", devidx,
		  select == VB_SELECT_FIRMWARE_READONLY ? "ro" : "rw");

	return entry;
}

VbError_t VbExEcGetExpectedImage(int devidx, enum VbSelectFirmware_t select,
				 const u8 **imagep, int *image_sizep)
{
	struct vboot_info *vboot = vboot_get();
	struct fmap_entry *entry;
	u8 *image;
	int ret;

	log_debug("%s: %d\n", __func__, devidx);
	entry = get_firmware_entry(vboot, devidx, select);
	if (!entry)
		return VBERROR_UNKNOWN;

	ret = fwstore_load_image(vboot->fwstore, entry, &image, image_sizep);
	if (ret) {
		log_err("Cannot locate image: err=%d\n", ret);
		return VBERROR_UNKNOWN;
	}
	*imagep = image;

	return VBERROR_SUCCESS;
}

VbError_t VbExEcGetExpectedImageHash(int devidx, enum VbSelectFirmware_t select,
				     const u8 **hash, int *hash_size)
{
	struct vboot_info *vboot = vboot_get();
	struct fmap_entry *entry;

	log_debug("%s: %d\n", __func__, devidx);
	entry = get_firmware_entry(vboot, devidx, select);
	if (!entry)
		return VBERROR_UNKNOWN;
	*hash = entry->hash;
	*hash_size = entry->hash_size;

	return VBERROR_SUCCESS;
}

VbError_t VbExEcUpdateImage(int devidx, enum VbSelectFirmware_t select,
			    const u8 *image, int image_size)
{
	struct udevice *dev;
	int ret;

	log_debug("%s: %d\n", __func__, devidx);
	ret = ec_get(devidx, &dev);
	if (ret)
		return log_msg_ret("Cannot get EC", ret);

	ret = vboot_ec_update_image(dev, select, image, image_size);
	if (ret) {
		log_err("Failed, err=%d\n", ret);
		switch (ret) {
		case -EINVAL:
			return VBERROR_INVALID_PARAMETER;
		case -EPERM:
			return VBERROR_EC_REBOOT_TO_RO_REQUIRED;
		case -EIO:
		default:
			return VBERROR_UNKNOWN;
		}
	}

	return VBERROR_SUCCESS;
}

VbError_t VbExEcProtect(int devidx, enum VbSelectFirmware_t select)
{
	struct udevice *dev;
	int ret;

	log_debug("%s: %d\n", __func__, devidx);
	ret = ec_get(devidx, &dev);
	if (ret)
		return log_msg_ret("Cannot get EC", ret);

	ret = vboot_ec_protect(dev, select);
	if (ret) {
		log_err("Failed, err=%d\n", ret);
		return VBERROR_UNKNOWN;
	}

	return VBERROR_SUCCESS;
}

VbError_t VbExEcEnteringMode(int devidx, enum VbEcBootMode_t mode)
{
	struct udevice *dev;
	int ret;

	log_debug("%s: %d\n", __func__, devidx);
	ret = ec_get(devidx, &dev);
	if (ret)
		return log_msg_ret("Cannot get EC", ret);

	ret = vboot_ec_entering_mode(dev, mode);
	if (ret) {
		log_err("Failed, err=%d\n", ret);
		return VBERROR_UNKNOWN;
	}

	return VBERROR_SUCCESS;
}

/* Wait 3 seconds after software sync for EC to clear the limit power flag */
#define LIMIT_POWER_WAIT_TIMEOUT 3000
/* Check the limit power flag every 50 ms while waiting */
#define LIMIT_POWER_POLL_SLEEP 50

VbError_t VbExEcVbootDone(int in_recovery)
{
	struct udevice *dev = board_get_cros_ec_dev();
	int limit_power;

	log_debug("%s\n", __func__);
	/* Ensure we have enough power to continue booting */
	while (1) {
		bool message_printed = false;
		int limit_power_wait_time = 0;
		int ret;

		ret = cros_ec_read_limit_power(dev, &limit_power);
		if (ret == -ENOSYS) {
			limit_power = 0;
		} else if (ret) {
			log_warning("Failed to check EC limit power flag\n");
			return VBERROR_UNKNOWN;
		}

		/*
		 * Do not wait for the limit power flag to be cleared in
		 * recovery mode since we didn't just sysjump.
		 */
		if (!limit_power || in_recovery ||
		    limit_power_wait_time > LIMIT_POWER_WAIT_TIMEOUT)
			break;

		if (!message_printed) {
			log_info("Waiting for EC to clear limit power flag\n");
			message_printed = 1;
		}

		mdelay(LIMIT_POWER_POLL_SLEEP);
		limit_power_wait_time += LIMIT_POWER_POLL_SLEEP;
	}

	if (limit_power) {
		log_info("EC requests limited power usage. Request shutdown\n");
		return VBERROR_SHUTDOWN_REQUESTED;
	}

	bootstage_mark(BOOTSTAMP_VBOOT_EC_DONE);

	return VBERROR_SUCCESS;
}

VbError_t VbExEcBatteryCutOff(void)
{
	struct udevice *dev = board_get_cros_ec_dev();
	int ret;

	log_debug("%s\n", __func__);
	ret = cros_ec_battery_cutoff(dev, EC_BATTERY_CUTOFF_FLAG_AT_SHUTDOWN);
	if (ret) {
		log_err("Failed, err=%d\n", ret);
		return VBERROR_UNKNOWN;
	}

	return VBERROR_SUCCESS;
}
