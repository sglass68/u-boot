// SPDX-License-Identifier: GPL-2.0+
/*
 * Sets up the read-write vboot portion (which loads the kernel)
 *
 * Copyright 2018 Google LLC
 */

#define NEED_VB20_INTERNALS

#include <common.h>
#include <bloblist.h>
#include <dm.h>
#include <mapmem.h>
#include <cros/cros_ofnode.h>
#include <cros/fwstore.h>
#include <cros/keyboard.h>
#include <cros/nvdata.h>
#include <cros/vboot.h>
#include <cros/vboot_flag.h>
#include <cros/memwipe.h>

#include <gbb_header.h>
#include <vboot_struct.h>

/**
 * gbb_copy_in() - Copy a portion of the GBB into vboot->cparams
 *
 * @vboot:	Vboot info
 * @gbb_offset:	Offset of GBB in vboot->fwstore
 * @offset:	Offset within GBB to read
 * @size:	Number of byte sto read
 * @return 0 if OK, -EINVAL if offset/size invalid, other error if fwstore
 *	fails to read
 */
static int gbb_copy_in(struct vboot_info *vboot, uint gbb_offset, uint offset,
		       uint size)
{
	VbCommonParams *cparams = &vboot->cparams;
	u8 *gbb_copy = cparams->gbb_data;
	int ret;

	if (offset > cparams->gbb_size || offset + size > cparams->gbb_size)
		return log_msg_ret("GBB component not inside the GBB", -EINVAL);
	ret = cros_fwstore_read(vboot->fwstore, gbb_offset + offset, size,
				gbb_copy + offset);
	if (ret)
		return log_msg_ret("Cannot read GBB data", ret);

	return 0;
}

static int gbb_init(struct vboot_info *vboot)
{
	struct fmap_entry *entry = &vboot->fmap.readonly.gbb;
	VbCommonParams *cparams = &vboot->cparams;
	int ret;

	cparams->gbb_size = entry->length;
	cparams->gbb_data = malloc(entry->length);
	if (!cparams->gbb_data)
		return log_msg_ret("Cannot allocate GBB buffer", -ENOMEM);

	memset(cparams->gbb_data, 0, cparams->gbb_size);

	u32 offset = entry->offset;

	GoogleBinaryBlockHeader *hdr = cparams->gbb_data;

	ret = gbb_copy_in(vboot, offset, 0, sizeof(GoogleBinaryBlockHeader));
	if (ret)
		return ret;
	log_info("The GBB signature is at %p and is: ", hdr->signature);
	for (int i = 0; i < GBB_SIGNATURE_SIZE; i++)
		log_info(" %02x", hdr->signature[i]);
	log_info("\n");

	ret = gbb_copy_in(vboot, offset, hdr->hwid_offset, hdr->hwid_size);
	if (ret)
		return ret;

	ret = gbb_copy_in(vboot, offset, hdr->rootkey_offset,
			  hdr->rootkey_size);
	if (ret)
		return ret;

	ret = gbb_copy_in(vboot, offset, hdr->recovery_key_offset,
			  hdr->recovery_key_size);
	if (ret)
		return ret;

	return 0;
}

static int common_params_init(struct vboot_info *vboot, bool clear_shared_data)
{
	VbCommonParams *cparams = &vboot->cparams;
	int ret;

	memset(cparams, '\0', sizeof(*cparams));

	ret = gbb_init(vboot);
	if (ret)
		return log_msg_ret("Cannot init GBB", ret);

	cparams->shared_data_blob = vboot->handoff->shared_data;
	cparams->shared_data_size = ARRAY_SIZE(vboot->handoff->shared_data);
	if (clear_shared_data)
		memset(cparams->shared_data_blob, '\0',
		       cparams->shared_data_size);
	log_info("Found shared_data_blob at %x, size %d\n",
		 map_to_sysmem(cparams->shared_data_blob),
		 cparams->shared_data_size);

	return 0;
}

/* When running under coreboot, use the coreboot tables to find memory */
#if defined(CONFIG_SYS_COREBOOT)
static void setup_arch_unused_memory(struct vboot_info *vboot,
				     struct memwipe *wipe)
{
	int i;

	/* Add ranges that describe RAM */
	for (i = 0; i < lib_sysinfo.n_memranges; i++) {
		struct memrange *range = &lib_sysinfo.memrange[i];

		if (range->type == CB_MEM_RAM) {
			memwipe_add(wipe, range->base,
				    range->base + range->size);
		}
	}
	/*
	 * Remove ranges that don't. These should take precedence, so they're
	 * done last and in their own loop.
	 */
	for (i = 0; i < lib_sysinfo.n_memranges; i++) {
		struct memrange *range = &lib_sysinfo.memrange[i];

		if (range->type != CB_MEM_RAM) {
			memwipe_sub(wipe, range->base,
				    range->base + range->size);
		}
	}
}

#else
static void setup_arch_unused_memory(struct vboot_info *vboot,
				     struct memwipe *wipe)
{
	struct fdt_memory ramoops, lp0;
	int bank;

	for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
		if (!gd->bd->bi_dram[bank].size)
			continue;
		memwipe_add(wipe, gd->bd->bi_dram[bank].start,
			    gd->bd->bi_dram[bank].start +
			    gd->bd->bi_dram[bank].size);
	}

	/* Excludes kcrashmem if in FDT */
	if (cros_ofnode_memory("/ramoops", &ramoops))
		log_debug("RAMOOPS not contained within FDT\n");
	else
		memwipe_sub(wipe, ramoops.start, ramoops.end);

	/* Excludes the LP0 vector; only applicable to tegra platforms */
	if (cros_ofnode_memory("/lp0", &lp0))
		log_debug("LP0 not contained within FDT\n");
	else
		memwipe_sub(wipe, lp0.start, lp0.end);
}
#endif

static uintptr_t get_current_sp(void)
{
#ifdef CONFIG_SANDBOX
	return gd->start_addr_sp;
#else
	uintptr_t addr;

	addr = (uintptr_t)&addr;
	return addr;
#endif
}

static void wipe_unused_memory(struct vboot_info *vboot)
{
	struct memwipe wipe;

	memwipe_init(&wipe);
	setup_arch_unused_memory(vboot, &wipe);

	/* Exclude relocated U-Boot structures */
	memwipe_sub(&wipe, get_current_sp() - memwipe_STACK_MARGIN,
		    gd->ram_top);

	/* Exclude the shared data between bootstub and main firmware */
	memwipe_sub(&wipe, (uintptr_t)vboot->handoff,
		    (uintptr_t)vboot->handoff + sizeof(struct vboot_handoff));

	memwipe_execute(&wipe);
}

static int vboot_do_init_out_flags(struct vboot_info *vboot, u32 out_flags)
{
	if (out_flags & VB_INIT_OUT_CLEAR_RAM) {
		if (vboot->disable_memwipe)
			log_warning("Memory wipe requested but not supported\n");
		else
			wipe_unused_memory(vboot);
	}

	vboot->vboot_out_flags = out_flags;

	return 0;
}

static int vboot_init_handoff(struct vboot_info *vboot)
{
	struct vboot_handoff *handoff;
	VbSharedDataHeader *vdat;
	int ret;

	handoff = bloblist_find(BLOBLISTT_VBOOT_HANDOFF, sizeof(*handoff));
	if (!handoff)
		return log_msg_ret("Failed to find vboot_handoff struct\n",
				   -ENOENT);
	vboot->handoff = handoff;

	/* Set up the common param structure, not clearing shared data */
	ret = common_params_init(vboot, 0);
	if (ret)
		return ret;

	vdat = vboot->cparams.shared_data_blob;
	/*
	 * If the lid is closed, don't count down the boot tries for updates,
	 * since the OS will shut down before it can register success.
	 *
	 * VbInit was already called in TPL, so we need to update the vboot
	 * internal flags ourself.
	 */
	if (vboot_flag_read_walk(VBOOT_FLAG_LID_OPEN) == 0) {
		/* Tell kernel selection to not count down */
		vdat->flags |= VBSD_NOFAIL_BOOT;
	}

	return vboot_do_init_out_flags(vboot, handoff->init_params.out_flags);
}

int vboot_rw_init(struct vboot_info *vboot)
{
	struct vboot_blob *blob;
	struct vb2_context *ctx;
	int ret;

	blob = bloblist_find(BLOBLISTT_VBOOT_CTX, sizeof(*blob));
	if (!blob)
		return log_msg_ret("Cannot find vboot context", -ENOENT);
	vboot->blob = blob;
	ctx = &blob->ctx;
	vboot->ctx = ctx;
	ctx->non_vboot_context = vboot;
	printf("ctx = %p\n", ctx);
	print_buffer(0, ctx->nvdata, 1, sizeof(ctx->nvdata), 0);
	vboot->valid = true;
	log_warning("flags %x %d\n", ctx->flags,
		    ((ctx->flags & VB2_CONTEXT_RECOVERY_MODE) != 0));

	ret = vboot_load_config(vboot);
	if (ret)
		return log_msg_ret("Cannot load config", ret);

	ret = uclass_first_device(UCLASS_TPM, &vboot->tpm);
	if (ret)
		return log_msg_ret("Cannot locate TPM", ret);

	ret = cros_ofnode_flashmap(&vboot->fmap);
	if (ret)
		return log_msg_ret("failed to decode fmap\n", ret);
	cros_ofnode_dump_fmap(&vboot->fmap);

	ret = vboot_keymap_init(vboot);
	if (ret)
		return log_msg_ret("Cannot set up keyboard remap", ret);

	ret = uclass_first_device_err(UCLASS_CROS_FWSTORE, &vboot->fwstore);
	if (ret)
		return log_msg_ret("Cannot set up fwstore", ret);

#if CONFIG_IS_ENABLED(CROS_EC)
	ret = uclass_get_device(UCLASS_CROS_EC, 0, &vboot->cros_ec);
	if (ret)
		return log_msg_ret("Cannot locate Chromium OS EC", ret);
	printf("EC=%p\n", vboot->cros_ec);
#endif
	ret = vboot_init_handoff(vboot);
	if (ret)
		return log_msg_ret("Cannot init vboot handoff", ret);

	return 0;
}
