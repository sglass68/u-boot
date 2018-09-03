// SPDX-License-Identifier: GPL-2.0+
/*
 * General functions used by vboot implementation
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <mapmem.h>
#include <asm/io.h>
#include <cros_ec.h>
#include <cros/vboot.h>

int vboot_alloc(struct vboot_info **vbootp)
{
	gd->vboot = malloc(sizeof(struct vboot_info));
	if (!gd->vboot) {
		log_err("Cannot allocate vboot %lx\n",
			sizeof(struct vboot_info));
		return -ENOMEM;
	}
	memset(gd->vboot, '\0', sizeof(struct vboot_info));
	*vbootp = gd->vboot;

	return 0;
}

struct vboot_info *vboot_get(void)
{
	struct vboot_info *vboot = gd->vboot;

	return vboot->valid ? vboot : NULL;
}

struct vboot_info *vboot_get_alloc(void)
{
	struct vboot_info *vboot = gd->vboot;

	if (!vboot)
		vboot_alloc(&vboot);

	return vboot;
}

struct vboot_info *vboot_get_nocheck(void)
{
	return gd->vboot;
}

int vboot_load_config(struct vboot_info *vboot)
{
	ofnode node;

	node = cros_ofnode_config_node();
	if (!ofnode_valid(node))
		return -ENOENT;

	vboot->deactivate_tpm = ofnode_read_bool(node, "deactivate-tpm");
	vboot->disable_dev_on_rec = ofnode_read_bool(node,
						     "disable-dev-on-rec");
	vboot->ec_efs = ofnode_read_bool(node, "ec-efs");
	vboot->ec_slow_update = ofnode_read_bool(node, "ec-slow-update");
	vboot->ec_software_sync = ofnode_read_bool(node, "ec-software-sync");
	vboot->has_rec_mode_mrc = ofnode_read_bool(node, "recovery-mode-mrc");
	vboot->meminit_in_ro = ofnode_read_bool(node,
						"meminit-in-readonly-code");
	vboot->oprom_matters = ofnode_read_bool(node, "oprom-matters");
	vboot->physical_dev_switch = ofnode_read_bool(node,
						      "physical-dev-switch");
	vboot->physical_rec_switch = ofnode_read_bool(node,
						      "physical-rec-switch");
	vboot->resume_path_same_as_boot = ofnode_read_bool(node,
						"resume-path-same-as-boot");
#ifndef CONFIG_SPL_BUILD
	vboot->detachable_ui = ofnode_read_bool(node, "detachable-ui");
	vboot->disable_memwipe = ofnode_read_bool(node, "disable-memwipe");
	vboot->disable_lid_shutdown_during_update = ofnode_read_bool(node,
					"disable-lid-shutdown-during-update");
	vboot->disable_power_button_during_update = ofnode_read_bool(node,
					"disable-power-button-during-update");
#endif
	vboot->work_buffer_size = ofnode_read_u32_default(node,
					"vboot2-work-buffer-size", 0x3000);

	vboot->config = node;

	return 0;
}

void vboot_init_cparams(struct vboot_info *vboot, VbCommonParams *cparams)
{
#ifdef CONFIG_SYS_COREBOOT
	cparams->shared_data_blob =
		&((chromeos_acpi_t *)lib_sysinfo.vdat_addr)->vdat;
	cparams->shared_data_size =
		sizeof(((chromeos_acpi_t *)lib_sysinfo.vdat_addr)->vdat);
#else
	/*
	 * TODO(sjg@chromium.org): Implement this
	 *	cparams->shared_data_blob = vboot->vb_shared_data;
	 *	cparams->shared_data_size = VB_SHARED_DATA_REC_SIZE;
	 */
#endif
	log_debug("cparams:\n");
	log_debug("- %-20s: %08x\n", "shared_data_blob",
		  (uint)map_to_sysmem(cparams->shared_data_blob));
	log_debug("- %-20s: %08x\n", "shared_data_size",
		  cparams->shared_data_size);
}

bool vboot_is_slot_a(struct vboot_info *vboot)
{
	struct vb2_context *ctx = vboot_get_ctx(vboot);

	return !(ctx->flags & VB2_CONTEXT_FW_SLOT_B);
}

const char *vboot_slot_name(struct vboot_info *vboot)
{
	return vboot_is_slot_a(vboot) ? "A" : "B";
}

void vboot_set_selected_region(struct vboot_info *vboot,
			       const struct fmap_entry *spl,
			       const struct fmap_entry *u_boot)
{
	vboot->blob->spl_entry = *spl;
	vboot->blob->u_boot_entry = *u_boot;
}

int vboot_platform_is_resuming(void)
{
	/* TODO(sjg@chromium.org): Implement this */

	return 0;
}
