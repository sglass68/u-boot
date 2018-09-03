// SPDX-License-Identifier: GPL-2.0+
/*
 * Holds functions that need access to internal vboot data
 *
 * Copyright 2018 Google LLC
 */

#define NEED_VB20_INTERNALS

#include <common.h>
#include <cros/vboot.h>

#include <gbb_header.h>

int vboot_wants_oprom(struct vboot_info *vboot)
{
	struct vb2_context *ctx = vboot_get_ctx(vboot);

	return (ctx->nvdata[VB2_NV_OFFS_BOOT] & VB2_NV_BOOT_OPROM_NEEDED) ? 1 :
		 0;
}

#ifndef CONFIG_SPL_BUILD
u32 vboot_get_gbb_flags(struct vboot_info *vboot)
{
	VbCommonParams *cparams = &vboot->cparams;
	GoogleBinaryBlockHeader *hdr = cparams->gbb_data;

	return hdr->flags;
}
#endif
