// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017 Google, Inc
 */

#include <common.h>
#include <dm.h>
#include <wdt.h>

DECLARE_GLOBAL_DATA_PTR;

#ifndef CONFIG_WATCHDOG_TIMEOUT_MSECS
#define CONFIG_WATCHDOG_TIMEOUT_MSECS	(60 * 1000)
#endif
#define WATCHDOG_TIMEOUT_SECS	(CONFIG_WATCHDOG_TIMEOUT_MSECS / 1000)

int initr_watchdog(void)
{
	u32 timeout = WATCHDOG_TIMEOUT_SECS;

	/*
	 * Init watchdog: This will call the probe function of the
	 * watchdog driver, enabling the use of the device
	 */
	if (uclass_get_device_by_seq(UCLASS_WDT, 0,
				     (struct udevice **)&gd->watchdog_dev)) {
		debug("WDT:   Not found by seq!\n");
		if (uclass_get_device(UCLASS_WDT, 0,
				      (struct udevice **)&gd->watchdog_dev)) {
			printf("WDT:   Not found!\n");
			return 0;
		}
	}

	if (CONFIG_IS_ENABLED(OF_CONTROL)) {
		timeout = dev_read_u32_default(gd->watchdog_dev, "timeout-sec",
					       WATCHDOG_TIMEOUT_SECS);
	}

	wdt_start(gd->watchdog_dev, timeout * 1000, 0);
	gd->flags |= GD_FLG_WDT_READY;
	printf("WDT:   Started with%s servicing (%ds timeout)\n",
	       IS_ENABLED(CONFIG_WATCHDOG) ? "" : "out", timeout);

	return 0;
}
