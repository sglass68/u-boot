// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <cros/vboot.h>

int vboot_ver2_select_fw(struct vboot_info *vboot)
{
	int ret;

	ret = vb2api_fw_phase2(vboot_get_ctx(vboot));
	if (ret) {
		log_info("Reboot requested (%x)\n", ret);
		return VBERROR_REBOOT_REQUIRED;
	}

	return 0;
}
