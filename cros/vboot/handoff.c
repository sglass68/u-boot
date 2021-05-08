// SPDX-License-Identifier: GPL-2.0+
/*
 * Internal vboot data passed through from VPL->SPL->U-Boot
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <bloblist.h>
#include <cros_ec.h>
#include <dm.h>
#include <log.h>
#include <cros/vboot.h>
#include <cros/vboot_flag.h>

#include <vb2_internals_please_do_not_use.h>

/**
 * Sets vboot_handoff based on the information in vb2_shared_data
 *
 * @vboot: vboot info
 * @vboot_handoff: Place to write hand-off info
 * @vb2_sd: Internal vboot information to decode and write
 */
static void fill_handoff(struct vboot_info *vboot,
			 struct vboot_handoff *vboot_handoff,
			 struct vb2_shared_data *vb2_sd)
{
	// TODO
// 	VbSharedDataHeader *vb_sd = NULL;
// 		(VbSharedDataHeader *)vboot_handoff->shared_data;
// 	u32 *oflags = NULL; //&vboot_handoff->init_params.out_flags;

#if 0
	vb_sd->flags |= VBSD_BOOT_FIRMWARE_VBOOT2;

	vboot_handoff->selected_firmware = vb2_sd->fw_slot;

	vb_sd->firmware_index = vb2_sd->fw_slot;

	vb_sd->magic = VB_SHARED_DATA_MAGIC;
	vb_sd->struct_version = VB_SHARED_DATA_VERSION;
	vb_sd->struct_size = sizeof(VbSharedDataHeader);
	vb_sd->data_size = VB_SHARED_DATA_MIN_SIZE;
	vb_sd->data_used = sizeof(VbSharedDataHeader);
	vb_sd->fw_version_tpm = vb2_sd->fw_version_secdata;

	if (vboot_flag_read_walk(VBOOT_FLAG_WRITE_PROTECT) == 1)
		vb_sd->flags |= VBSD_BOOT_FIRMWARE_WP_ENABLED;

	if (vb2_sd->recovery_reason) {
		vb_sd->firmware_index = 0xFF;
		if (vb2_sd->flags & VB2_SD_FLAG_MANUAL_RECOVERY)
			vb_sd->flags |= VBSD_BOOT_REC_SWITCH_ON;
		*oflags |= VB_INIT_OUT_ENABLE_RECOVERY;
		*oflags |= VB_INIT_OUT_CLEAR_RAM;
		*oflags |= VB_INIT_OUT_ENABLE_DISPLAY;
		*oflags |= VB_INIT_OUT_ENABLE_USB_STORAGE;
	}
	if (vb2_sd->flags & VB2_SD_DEV_MODE_ENABLED) {
		*oflags |= VB_INIT_OUT_ENABLE_DEVELOPER;
		*oflags |= VB_INIT_OUT_CLEAR_RAM;
		*oflags |= VB_INIT_OUT_ENABLE_DISPLAY;
		*oflags |= VB_INIT_OUT_ENABLE_USB_STORAGE;
		vb_sd->flags |= VBSD_BOOT_DEV_SWITCH_ON;
		vb_sd->flags |= VBSD_LF_DEV_SWITCH_ON;
	}
	if (!vboot->physical_dev_switch)
		vb_sd->flags |= VBSD_HONOR_VIRT_DEV_SWITCH;
	if (vboot->ec_software_sync) {
		vb_sd->flags |= VBSD_EC_SOFTWARE_SYNC;
		if (vboot->ec_slow_update)
			vb_sd->flags |= VBSD_EC_SLOW_UPDATE;
		if (vboot->ec_efs)
			vb_sd->flags |= VBSD_EC_EFS;
	}
	if (!vboot->physical_rec_switch)
		vb_sd->flags |= VBSD_BOOT_REC_SWITCH_VIRTUAL;
	if (vboot->oprom_matters) {
		vb_sd->flags |= VBSD_OPROM_MATTERS;
		/*
		 * Inform vboot if the display was enabled by dev/rec
		 * mode or was requested by vboot kernel phase.
		 */
		if ((*oflags & VB_INIT_OUT_ENABLE_DISPLAY) ||
		    vboot_wants_oprom(vboot)) {
			vb_sd->flags |= VBSD_OPROM_LOADED;
			*oflags |= VB_INIT_OUT_ENABLE_DISPLAY;
		}
	}

	/*
	 * In vboot1, VBSD_FWB_TRIED is set only if B is booted as explicitly
	 * requested. Therefore, if B is booted because A was found bad, the
	 * flag should not be set. It's better not to touch it if we can only
	 * ambiguously control it
	 *
	 * if (vb2_sd->fw_slot)
	 *	vb_sd->flags |= VBSD_FWB_TRIED;
	 */

	/* copy kernel subkey if it's found */
	if (vb2_sd->workbuf_preamble_size) {
		struct vb2_fw_preamble *fp;
		ulong dst, src;

		log_info("Copying FW preamble\n");
		fp = (struct vb2_fw_preamble *)((ulong)vb2_sd +
				vb2_sd->workbuf_preamble_offset);
		src = (ulong)&fp->kernel_subkey +
				fp->kernel_subkey.key_offset;
		dst = (ulong)vb_sd + sizeof(VbSharedDataHeader);
		assert(dst + fp->kernel_subkey.key_size <=
		       (ulong)vboot_handoff + sizeof(*vboot_handoff));
		memcpy((void *)dst, (void *)src,
		       fp->kernel_subkey.key_size);
		vb_sd->data_used += fp->kernel_subkey.key_size;
		vb_sd->kernel_subkey.key_offset =
				dst - (ulong)&vb_sd->kernel_subkey;
		vb_sd->kernel_subkey.key_size = fp->kernel_subkey.key_size;
		vb_sd->kernel_subkey.algorithm = fp->kernel_subkey.algorithm;
		vb_sd->kernel_subkey.key_version =
				fp->kernel_subkey.key_version;
	}

	vb_sd->recovery_reason = vb2_sd->recovery_reason;
#endif
}

/**
 * log_recovery_mode_switch() - Create a bloblist for the recovery switch
 *
 * This reads the EC events and writes them into a bloblist for later use
 *
 * @vboot: vboot info
 * @return 0 if added, -ENOSPC if the bloblist was out of space, -EEXIST if it
 *	is already there so nothing was done
 */
static int log_recovery_mode_switch(struct vboot_info *vboot)
{
	u64 *events;

	/* Don't add this info if it is already there */
	if (!bloblist_find(BLOBLISTT_EC_HOSTEVENT, sizeof(*events)))
		return -EEXIST;
	events = bloblist_add(BLOBLISTT_EC_HOSTEVENT, sizeof(*events), 0);
	if (!events)
		return -ENOSPC;

	*events = cros_ec_get_events_b(vboot->cros_ec);

	return 0;
}

static int clear_recovery_mode_switch(struct vboot_info *vboot)
{
	log_info("Clearing recovery mode\n");

	/* Clear all host event bits requesting recovery mode */
	return cros_ec_clear_events_b(vboot->cros_ec,
		EC_HOST_EVENT_MASK(EC_HOST_EVENT_KEYBOARD_RECOVERY) |
		EC_HOST_EVENT_MASK(EC_HOST_EVENT_KEYBOARD_RECOVERY_HW_REINIT) |
		EC_HOST_EVENT_MASK(EC_HOST_EVENT_KEYBOARD_FASTBOOT));
}

int vboot_fill_handoff(struct vboot_info *vboot)
{
	struct vb2_context *ctx = vboot_get_ctx(vboot);
	struct vboot_handoff *vh;
	struct vb2_shared_data *sd;

	// TODO
	sd = NULL; //(struct vb2_shared_data *)ctx->workbuf;
// 	sd->workbuf_hash_offset = 0;
// 	sd->workbuf_hash_size = 0;

	log_info("Creating vboot_handoff structure\n");
	vh = bloblist_add(BLOBLISTT_VBOOT_HANDOFF, sizeof(*vh), 0);
	if (!vh)
		return log_msg_ret("failed to alloc vboot_handoff struct\n",
				   -ENOSPC);

	/* needed until we finish transtion to vboot2 for kernel verification */
	fill_handoff(vboot, vh, sd);
	vboot->handoff = vh;

	/* Log the recovery mode switches if required, before clearing them */
	log_recovery_mode_switch(vboot);

	log_warning("flags %llx recovery=%d, EC=%s\n", ctx->flags,
		    (ctx->flags & VB2_CONTEXT_RECOVERY_MODE) != 0,
		    vboot->cros_ec->name);
	if (ctx->flags & VB2_CONTEXT_RECOVERY_MODE)
		clear_recovery_mode_switch(vboot);

	return 0;
}
