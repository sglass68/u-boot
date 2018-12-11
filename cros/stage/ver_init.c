// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY LOGC_VBOOT
#define NEED_VB20_INTERNALS

#include <common.h>
#include <bloblist.h>
#include <ec_commands.h>
#include <dm.h>
#include <log.h>
#include <cros/cros_common.h>
#include <cros/nvdata.h>
#include <cros/vboot.h>
#include <cros/vboot_flag.h>

/**
 * vb2_init_blob() - Set up the vboot persistent blob
 *
 * This holds struct vboot_blob which includes things we need to know and also
 * the vb2_context. Set up a work buffer within the vb2_context.
 *
 * The blob is called persistent since it is preserved through each stage of
 * the boot.
 *
 * @blob: Pointer to the persistent blob for vboot
 * @work_buffer_size: Size to use for work buffer
 */
static int vb2_init_blob(struct vboot_blob *blob, int work_buffer_size)
{
	struct vb2_context *ctx = &blob->ctx;

	/* initialise the vb2_context */
	memset(blob, '\0', sizeof(*blob));
	ctx->workbuf_size = work_buffer_size;
	ctx->workbuf = memalign(VBOOT_CONTEXT_ALIGN, ctx->workbuf_size);
	if (!ctx->workbuf)
		return -ENOMEM;
	memset(ctx->workbuf, '\0', ctx->workbuf_size);

	return 0;
}

int vboot_ver_init(struct vboot_info *vboot)
{
	struct vboot_blob *blob;
	struct vb2_context *ctx;
	int ret;

	printf("vboot is at %p, size %lx\n", vboot, (ulong)sizeof(*vboot));
	blob = bloblist_add(BLOBLISTT_VBOOT_CTX, sizeof(struct vboot_blob));
	if (!blob)
		return log_msg_ret("Cannot set up vboot context", -ENOSPC);

	bootstage_mark(BOOTSTAGE_VBOOT_START);

	ret = vboot_load_config(vboot);
	if (ret)
		return log_msg_ret("Cannot load config", ret);

	/* Set up context and work buffer */
	ret = vb2_init_blob(blob, vboot->work_buffer_size);
	if (ret)
		return log_msg_ret("Cannot set up work context", ret);
	vboot->blob = blob;
	ctx = &blob->ctx;
	vboot->ctx = ctx;
	ctx->non_vboot_context = vboot;
	vboot->valid = true;

	ret = uclass_first_device_err(UCLASS_TPM, &vboot->tpm);
	if (ret)
		return log_msg_ret("Cannot find TPM", ret);
	ret = cros_tpm_setup(vboot);
	if (ret) {
		log_err("TPM setup failed (err=%x)\n", ret);
		return -EIO;
	}

	/* initialise and read nvdata from non-volatile storage */
	ret = uclass_first_device_err(UCLASS_CROS_NVDATA, &vboot->nvdata_dev);
	if (ret)
		return log_msg_ret("Cannot find nvdata", ret);
	/* TODO(sjg@chromium.org): Support full-size context */
	ret = cros_nvdata_read_walk(CROS_NV_DATA, ctx->nvdata,
				    EC_VBNV_BLOCK_SIZE);
	if (ret)
		return log_msg_ret("Cannot read nvdata", ret);

#if 0
	/* Force legacy mode */
	ctx->nvdata[VB2_NV_OFFS_HEADER] = VB2_NV_HEADER_SIGNATURE_V1;
	ctx->nvdata[VB2_NV_OFFS_DEV] |= VB2_NV_DEV_FLAG_LEGACY;
	vb2_nv_regen_crc(ctx);
#endif
	print_buffer(0, ctx->nvdata, 1, sizeof(ctx->nvdata), 0);

	ret = cros_ofnode_flashmap(&vboot->fmap);
	if (ret)
		return log_msg_ret("failed to decode fmap\n", ret);
	cros_ofnode_dump_fmap(&vboot->fmap);
	ret = uclass_first_device_err(UCLASS_CROS_FWSTORE, &vboot->fwstore);
	if (ret)
		return log_msg_ret("Cannot set up fwstore", ret);

	if (CONFIG_IS_ENABLED(CROS_EC)) {
		ret = uclass_get_device(UCLASS_CROS_EC, 0, &vboot->cros_ec);
		if (ret)
			return log_msg_ret("Cannot locate Chromium OS EC", ret);
	}
	/*
	 * Set S3 resume flag if vboot should behave differently when selecting
	 * which slot to boot.  This is only relevant to vboot if the platform
	 * does verification of memory init and thus must ensure it resumes with
	 * the same slot that it booted from.
	 */
	if (vboot->resume_path_same_as_boot && !vboot->meminit_in_ro &&
	    vboot_platform_is_resuming())
		ctx->flags |= VB2_CONTEXT_S3_RESUME;

	/*
	 * Read secdata from TPM. initialise TPM if secdata not found. We don't
	 * check the return value here because vb2api_fw_phase1 will catch
	 * invalid secdata and tell us what to do (=reboot).
	 */
	bootstage_mark(BOOTSTAGE_VBOOT_START_TPMINIT);
	ret = cros_nvdata_read_walk(CROS_NV_SECDATA, ctx->secdata,
				    sizeof(ctx->secdata));
	if (ret == -ENOENT)
		ret = cros_tpm_factory_initialise(vboot);
	else if (ret)
		return log_msg_ret("Cannot read secdata", ret);

	bootstage_mark(BOOTSTAGE_VBOOT_END_TPMINIT);

	if (vboot_flag_read_walk(VBOOT_FLAG_DEVELOPER) == 1) {
		ctx->flags |= VB2_CONTEXT_FORCE_DEVELOPER_MODE;
		log_info("Enabled developer mode\n");
	}

	if (vboot_flag_read_walk(VBOOT_FLAG_RECOVERY) == 1) {
		if (vboot->disable_dev_on_rec)
			ctx->flags |= VB2_DISABLE_DEVELOPER_MODE;
	}

	if (vboot_flag_read_walk(VBOOT_FLAG_WIPEOUT) == 1)
		ctx->flags |= VB2_CONTEXT_FORCE_WIPEOUT_MODE;

	if (vboot_flag_read_walk(VBOOT_FLAG_LID_OPEN) == 0)
		ctx->flags |= VB2_CONTEXT_NOFAIL_BOOT;

	return 0;
}
