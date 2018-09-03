// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <bloblist.h>
#include <dm.h>
#include <cros/vboot.h>

int vboot_spl_init(struct vboot_info *vboot)
{
	struct vboot_blob *blob;
	struct vb2_context *ctx;
	int ret;

	blob = bloblist_find(BLOBLISTT_VBOOT_CTX, sizeof(*blob));
	if (!blob)
		return log_msg_ret("Cannot set up vboot context", -ENOENT);
	vboot->blob = blob;
	ctx = &blob->ctx;
	vboot->ctx = ctx;
	ctx->non_vboot_context = vboot;
	vboot->valid = true;

	ret = uclass_first_device_err(UCLASS_CROS_FWSTORE, &vboot->fwstore);
	if (ret)
		return log_msg_ret("Cannot set up fwstore", ret);
#if defined(CONFIG_SANDBOX) && CONFIG_IS_ENABLED(CROS_EC)
	/* Probe the EC so that it will read and write its state */
	uclass_get_device(UCLASS_CROS_EC, 0, &vboot->cros_ec);
#endif

	return 0;
}
