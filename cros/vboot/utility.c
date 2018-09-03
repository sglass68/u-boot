// SPDX-License-Identifier: GPL-2.0+
/*
 * Implementation of APIs provided by firmware and exported to vboot_reference.
 * They includes debug output, memory allocation, timer and delay, etc.
 *
 * TODO(sjg@chromium.org): Convert to use DM for sound
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <sound.h>
#include <sysreset.h>
#include <cros/cros_common.h>
#include <cros/vboot.h>

DECLARE_GLOBAL_DATA_PTR;

#define TICKS_PER_MSEC		(CONFIG_SYS_HZ / 1000)
#define MAX_MSEC_PER_LOOP	((u32)((UINT32_MAX / TICKS_PER_MSEC) / 2))

static void system_abort(void)
{
	/* Wait for 3 seconds to let users see error messages and reboot */
	VbExSleepMs(3000);
	sysreset_walk_halt(SYSRESET_POWER);
}

void VbExError(const char *format, ...)
{
	va_list ap;

	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
	system_abort();
}

void VbExSleepMs(u32 msec)
{
	u32 delay, start;

	/*
	 * Can't use entire UINT32_MAX range in the max delay, because it
	 * pushes get_timer() too close to wraparound. So use /2.
	 */
	while (msec > MAX_MSEC_PER_LOOP) {
		VbExSleepMs(MAX_MSEC_PER_LOOP);
		msec -= MAX_MSEC_PER_LOOP;
	}

	delay = msec * TICKS_PER_MSEC;
	start = get_timer(0);

	while (get_timer(start) < delay)
		udelay(100);
}

VbError_t VbExBeep(u32 msec, u32 frequency)
{
#if defined CONFIG_SOUND
	if (sound_init(gd->fdt_blob)) {
		log_debug("Failed to initialise sound.\n");
		return VBERROR_NO_SOUND;
	}

	printf("About to beep for %d ms at %d Hz.\n", msec, frequency);
	if (!msec)
		return VBERROR_NO_BACKGROUND_SOUND;

	if (frequency) {
		if (sound_play(msec, frequency)) {
			log_debug("Failed to play beep.\n");
			return VBERROR_NO_SOUND;
		}
	} else {
		VbExSleepMs(msec);
	}

	return VBERROR_SUCCESS;
#else
	VbExSleepMs(msec);
	printf("Beep!\n");
	return VBERROR_NO_SOUND;
#endif
}

u64 VbExGetTimer(void)
{
	return timer_get_us();
}

void vb2ex_printf(const char *func, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (func)
		printf("%s: ", func);
	vprintf(fmt, ap);
	va_end(ap);
}
