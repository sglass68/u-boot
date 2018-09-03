// SPDX-License-Identifier: GPL-2.0+
/*
 * Implementation of callbacks for 'legacy' boot
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <os.h>
#include <cros/vboot.h>

int VbExLegacy(int altfw_num)
{
	/* TODO(sjg@chromium.org): Implement this */
	printf("Legacy boot %d\n", altfw_num);

	return 1;
}

u32 VbExGetAltFwIdxMask(void)
{
	/* TODO(sjg@chromium.org): Implement this */
	return 3 << 1;
}
