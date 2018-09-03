// SPDX-License-Identifier: GPL-2.0+
/*
 * Reading of resources (flash regions) for vboot
 *
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <cros/fwstore.h>
#include <cros/vboot.h>

static int resource_read(struct vboot_info *vboot,
			 enum vb2_resource_index index,
			 u32 offset, void *buf, u32 size)
{
	struct fmap_entry *entry;
	int pos;
	int ret;

	switch (index) {
	case VB2_RES_GBB:
		log_info("GBB: ");
		entry = &vboot->fmap.readonly.gbb;
		break;
	case VB2_RES_FW_VBLOCK:
		log_info("Slot %c: ", 'A' + !vboot_is_slot_a(vboot));
		if (vboot_is_slot_a(vboot))
			entry = &vboot->fmap.readwrite_a.vblock;
		else
			entry = &vboot->fmap.readwrite_b.vblock;
		break;
	default:
		return -EINVAL;
	}

	pos = entry->offset + offset;
	log_info("Reading SPI flash offset=%x, size=%x\n", pos, size);
	ret = cros_fwstore_read(vboot->fwstore, pos, size, buf);

	return log_msg_ret("failed to read resource", ret);
}

int vb2ex_read_resource(struct vb2_context *ctx, enum vb2_resource_index index,
			u32 offset, void *buf, u32 size)
{
	struct vboot_info *vboot = ctx->non_vboot_context;
	int ret;

	ret = resource_read(vboot, index, offset, buf, size);
	if (ret == -EINVAL)
		return VB2_ERROR_EX_READ_RESOURCE_INDEX;
	else if (ret)
		return VB2_ERROR_EX_READ_RESOURCE_SIZE;

	return 0;
}
