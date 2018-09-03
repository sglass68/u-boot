// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <cros_ec.h>
#include <ec_commands.h>
#include <mapmem.h>
#include <sysreset.h>
#include <cros/vboot.h>

int vboot_rw_select_kernel(struct vboot_info *vboot)
{
	VbSelectAndLoadKernelParams *kparams = &vboot->kparams;
	fdt_addr_t kaddr;
	fdt_size_t ksize;
	int ret;

	ret = vboot_load_config(vboot);
	if (ret)
		return log_msg_ret("Cannot load config", ret);
	kaddr = ofnode_get_addr_size(vboot->config, "kernel-addr", &ksize);
	if (kaddr == FDT_ADDR_T_NONE)
		return log_msg_ret("Cannot read kernel address", -EINVAL);
	kparams->kernel_buffer = map_sysmem(kaddr, ksize);
	kparams->kernel_buffer_size = ksize;

	if (vboot->detachable_ui) {
		kparams->inflags = VB_SALK_INFLAGS_ENABLE_DETACHABLE_UI;
		if (IS_ENABLED(CONFIG_X86) && CONFIG_IS_ENABLED(CROS_EC)) {
			/*
			 * TODO(sjg@chromium.org): On x86 systems, inhibit power
			 * button pulse from EC.
			 * cros_ec_config_powerbtn(0);
			 * list_insert_after(&x86_ec_powerbtn_cleanup.list_node,
			 *		     &cleanup_funcs);
			 */
		}
	}

	printf("Calling VbSelectAndLoadKernel().\n");
	VbError_t res = VbSelectAndLoadKernel(&vboot->cparams, kparams);

	ret = 0;
	if (res == VBERROR_EC_REBOOT_TO_RO_REQUIRED) {
		printf("EC Reboot requested. Doing cold reboot.\n");

		/* TODO(sjg@chromium.org): Create a sysreset driver for this */
		if (CONFIG_IS_ENABLED(CROS_EC))
			cros_ec_reboot(vboot->cros_ec, EC_REBOOT_COLD, 0);
		sysreset_walk_halt(SYSRESET_COLD);
	} else if (res == VBERROR_EC_REBOOT_TO_SWITCH_RW) {
		printf("Switch EC slot requested. Doing cold reboot.\n");
		if (CONFIG_IS_ENABLED(CROS_EC))
			cros_ec_reboot(vboot->cros_ec, EC_REBOOT_COLD,
				       EC_REBOOT_FLAG_SWITCH_RW_SLOT);
		sysreset_walk_halt(SYSRESET_POWER_OFF);
	} else if (res == VBERROR_SHUTDOWN_REQUESTED) {
		printf("Powering off.\n");
		sysreset_walk_halt(SYSRESET_POWER_OFF);
	} else if (res == VBERROR_REBOOT_REQUIRED) {
		printf("Reboot requested. Doing warm reboot.\n");
		sysreset_walk_halt(SYSRESET_WARM);
	}
	if (res != VBERROR_SUCCESS) {
		printf("VbSelectAndLoadKernel() returned %d, doing a cold reboot.\n",
		       res);
		sysreset_walk_halt(SYSRESET_COLD);
	}
	if (ret)
		return log_msg_ret("Failed to reboot/power off", ret);

	return 0;
}
