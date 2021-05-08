// SPDX-License-Identifier: GPL-2.0
/*
 * Writing of vboot state into ACPI tables
 *
 * Copyright 2020 Google LLC
 */

#define LOG_CATEGORY	LOGC_VBOOT

#include <common.h>
#include <bloblist.h>
#include <asm/cb_sysinfo.h>
#include <log.h>
#include <smbios.h>
#include <asm/intel_gnvs.h>
#include <cros/fwstore.h>
#include <cros/vboot.h>

#include <vboot_struct.h>

#include <vb2_internals_please_do_not_use.h>

/* vboot firmware indexes */
enum {
	VBSD_RW_A	= 0x0,
	VBSD_RW_B	= 0x1,
	VBSD_RO		= 0xff,
	VBSD_RECOVERY	= 0xff,
	VBSD_UNKNOWN	= 0x100,
};


/**
 * get_firmware_index() - Convert a vboot firmware index to crossystem
 *
 * @vbsd_fw_index: vboot firmware index
 * @return associated index for using in ACPI, or -1 if not found
 */
static int get_firmware_index(int vbsd_fw_index)
{
	const struct entry {
		int vbsd_fw_index;
		int main_fw_index;
	} fw_arr[] = {
		{ VBSD_RW_A, BINF_RW_A },
		{ VBSD_RW_B, BINF_RW_B },
		{ VBSD_RECOVERY, BINF_RECOVERY },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(fw_arr); i++) {
		const struct entry *entry = &fw_arr[i];

		if (vbsd_fw_index == entry->vbsd_fw_index)
			return entry->main_fw_index;
	}

	return -1;
}

int vboot_update_acpi(struct vboot_info *vboot)
{
	struct chromeos_acpi_gnvs *tab;
	struct acpi_global_nvs *gnvs;
	GoogleBinaryBlockHeader *gbb;
	VbSharedDataHeader *vb_sd;
	VbSharedDataHeader *vdat;
	uint size, vb_sd_size;
	ulong fmap_addr;
	int main_fw;
	char *hwid;
	uint chsw;
	int ret;

	log_info("Updating ACPI tables\n");
	gnvs = bloblist_find(BLOBLISTT_ACPI_GNVS, sizeof(*gnvs));
	if (!gnvs) {
		if (!vboot_from_cb(vboot))
			return log_msg_ret("bloblist", -ENOENT);
		gnvs = vboot->sysinfo->acpi_gnvs;
		if (!gnvs)
			return log_msg_ret("gnvs", -ENOKEY);
	}
	tab = &gnvs->chromeos;

	vdat = (VbSharedDataHeader *)&tab->vdat;
	if (vdat->magic != VB_SHARED_DATA_MAGIC) {
		log_err("Bad magic value in vboot shared data header\n");
		return -EBADFD;
	}

	vb_sd = (VbSharedDataHeader *)vboot->handoff->shared_data;
	vb_sd_size = sizeof(vboot->handoff->shared_data);
	if (!vb_sd) {
		log_err("Can't find common params\n");
		return -ENOENT;
	}

	if (vb_sd->magic != VB_SHARED_DATA_MAGIC) {
		log_err("Bad magic value in vboot shared data header\n");
		return -EPERM;
	}

	tab->boot_reason = BOOT_REASON_OTHER;
	main_fw = get_firmware_index(vb_sd->firmware_index);
	if (main_fw < 0) {
		log_err("Invalid firmware index %d\n", vb_sd->firmware_index);
		return -EINVAL;
	}
	tab->main_fw_type = main_fw;

	if (vboot->ec_software_sync) {
		int in_rw = 0;

		if (VbExEcRunningRW(0, &in_rw)) {
			log_err("Couldn't tell if the EC firmware is RW\n");
			return -EPROTO;
		}
		tab->activeec_fw = in_rw ? ACTIVE_ECFW_RW : ACTIVE_ECFW_RO;
	}

	chsw = 0;
	if (vb_sd->flags & VBSD_BOOT_FIRMWARE_WP_ENABLED)
		chsw |= CHSW_FIRMWARE_WP;
	if (vb_sd->flags & VBSD_BOOT_REC_SWITCH_ON)
		chsw |= CHSW_RECOVERY_X86;
	if (vb_sd->flags & VBSD_BOOT_DEV_SWITCH_ON)
		chsw |= CHSW_DEVELOPER_SWITCH;
	tab->switches = chsw;

	gbb = vboot->cparams.gbb_data;
	if (memcmp(gbb->signature, GBB_SIGNATURE, GBB_SIGNATURE_SIZE)) {
		log_err("Bad signature on GBB\n");
		return -EBADSLT;
	}
	hwid = (char *)gbb + gbb->hwid_offset;
	size = min(gbb->hwid_size, sizeof(tab->hwid));
	memcpy(tab->hwid, hwid, size);

	size = min(ID_LEN, sizeof(tab->fwid));
	memcpy(tab->fwid, vboot->firmware_id, size);

	size = min(ID_LEN, sizeof(tab->frid));
	memcpy(tab->frid, vboot->readonly_firmware_id, size);

	if (main_fw == BINF_RECOVERY)
		tab->main_fw_type = FIRMWARE_TYPE_RECOVERY;
	else if (vb_sd->flags & VBSD_BOOT_DEV_SWITCH_ON)
		tab->main_fw_type = FIRMWARE_TYPE_DEVELOPER;
	else
		tab->main_fw_type = FIRMWARE_TYPE_NORMAL;

	tab->recovery_reason = vb_sd->recovery_reason;

	if (!vboot->fmap.readonly.fmap.length) {
		log_err("No FMAP available\n");
		return -ENOTDIR;
	}
	ret = fwstore_entry_mmap(vboot->fwstore, &vboot->fmap.readonly.fmap,
				 &fmap_addr);
	if (!ret)
		tab->fmap_base = fmap_addr;
	else
		log_warning("FMAP address cannot be mapped (err=%d)\n", ret);

	if (IS_ENABLED(CONFIG_GENERATE_SMBIOS_TABLE)) {
		size = min(ID_LEN, sizeof(tab->fwid));
		ret = smbios_update_version(vboot->firmware_id);
		if (ret) {
			log_err("Unable to update SMBIOS type 0 version string\n");
			return log_msg_ret("smbios", ret);
		}
	} else if (vboot_from_cb(vboot)) {
		void *smbios_start;

		smbios_start = (void *)(ulong)vboot->sysinfo->smbios_start;
		if (!smbios_start) {
			log_warning("SMBIOS table not provided\n");
			return log_msg_ret("tab", -ENOENT);
		}
		ret = smbios_update_version_full(smbios_start,
						 vboot->firmware_id);
		if (ret) {
			log_err("Unable to update SMBIOS type 0 version string\n");
			return log_msg_ret("cbsmbios", ret);
		}
	}

	/* Synchronize VbSharedDataHeader from vboot_handoff to acpi vdat */
	memcpy(vdat, vb_sd, vb_sd_size);

	return 0;
}

