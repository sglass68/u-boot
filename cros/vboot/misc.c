// SPDX-License-Identifier: GPL-2.0+
/*
 * Implementation of misc callbacks
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <gpt.h>
#include <vb2_api.h>
#include <cros/cros_common.h>
#include <cros/vboot_flag.h>

u32 VbExIsShutdownRequested(void)
{
	int ret, prev;

	/* if lid is NOT OPEN */
	ret = vboot_flag_read_walk(VBOOT_FLAG_LID_OPEN);
	if (ret == 0) {
		log_debug("Lid-closed is detected.\n");
		return 1;
	}
	/*
	 * If power switch is pressed (but previously was known to be not
	 * pressed), we power off.
	 */
	ret = vboot_flag_read_walk_prev(VBOOT_FLAG_POWER_OFF, &prev);
	if (!ret && prev == 1) {
		log_debug("Power-key-pressed is detected.\n");
		return 1;
	}
	/*
	 * Either the gpios don't exist, or the lid is up and and power button
	 * is not pressed. No-Shutdown-Requested.
	 */
	return 0;
}

u8 VbExOverrideGptEntryPriority(const GptEntry *e)
{
	return 0;
}

/* This should never get called */
u32 VbExGetSwitches(u32 request_mask)
{
	return 0;
}
