// SPDX-License-Identifier: GPL-2.0
/*
 * Jumping from SPL to U-Boot proper
 *
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <os.h>
#include <spl.h>
#include <cros/fwstore.h>
#include <cros/vboot.h>

/*
 * Enable this to read into RAM. If undefined, it read directly from FLASH,
 * which only works if flash is memory-mapped, as on x86.
 */
#define USE_RAM		!IS_ENABLED(CONFIG_X86)

int vboot_jump(struct vboot_info *vboot, struct fmap_entry *entry)
{
	struct spl_image_info *spl_image = vboot->spl_image;
#ifdef USE_RAM
	u32 addr = CONFIG_SYS_TEXT_BASE;
	char *buf = (char *)(ulong)addr;
	int ret;
#else
	u32 addr = entry->offset - CONFIG_ROM_SIZE;
#endif

	log_warning("Reading firmware offset %x (addr %x, size %x)\n",
		    entry->offset, addr, entry->length);
#ifdef USE_RAM
	ret = cros_fwstore_read(vboot->fwstore, entry->offset, entry->length,
				buf);
	if (ret)
		return log_msg_ret("Read fwstore", ret);
#endif
	spl_image->size = entry->length;
	spl_image->entry_point = addr;
	spl_image->load_addr = addr;
	spl_image->os = IH_OS_U_BOOT;
	spl_image->name = "U-Boot";

	return 0;
}
