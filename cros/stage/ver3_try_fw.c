// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <cros/cros_common.h>
#include <cros/vboot.h>

int vboot_ver3_try_fw(struct vboot_info *vboot)
{
	int ret;

	bootstage_mark(BOOTSTAGE_VBOOT_START_VERIFY_SLOT);
	ret = vb2api_fw_phase3(vboot_get_ctx(vboot));
	bootstage_mark(BOOTSTAGE_VBOOT_END_VERIFY_SLOT);
	if (ret) {
		log_info("Reboot reqested (%x)\n", ret);
		return VBERROR_REBOOT_REQUIRED;
	}

	return 0;
}
