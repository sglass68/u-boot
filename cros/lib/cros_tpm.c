// SPDX-License-Identifier:     BSD-3-Clause
/*
 * Functions for querying, manipulating and locking rollback indices
 * stored in the TPM NVRAM.
 *
 * Taken from coreboot file secdata_tpm.c
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <log.h>
#include <tpm-common.h>
#include <tpm-v1.h>
#include <tpm-v2.h>
#include <cros/nvdata.h>
#include <cros/cros_common.h>
#include <cros/vboot.h>

/*
 * This is derived from rollback_index.h of vboot_reference. see struct
 * RollbackSpaceKernel for details.
 */
static const u8 secdata_kernel[] = {
	0x02,
	0x4C, 0x57, 0x52, 0x47,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
	0xE8,
};

/*
 * Different sets of NVRAM space attributes apply to the "ro" spaces,
 * i.e. those which should not be possible to delete or modify once
 * the RO exits, and the rest of the NVRAM spaces.
 */
static const enum tpm_index_attrs v2_ro_space_attributes =
	TPMA_NV_PPWRITE |
	TPMA_NV_AUTHREAD |
	TPMA_NV_PPREAD |
	TPMA_NV_PLATFORMCREATE |
	TPMA_NV_WRITE_STCLEAR |
	TPMA_NV_POLICY_DELETE;

static const u32 v2_rw_space_attributes =
	TPMA_NV_PPWRITE |
	TPMA_NV_AUTHREAD |
	TPMA_NV_PPREAD |
	TPMA_NV_PLATFORMCREATE;

/*
 * This policy digest was obtained using TPM2_PolicyPCR
 * selecting only PCR_0 with a value of all zeros.
 */
static const u8 v2_pcr0_unchanged_policy[] = {
	0x09, 0x93, 0x3C, 0xCE, 0xEB, 0xB4, 0x41, 0x11, 0x18, 0x81, 0x1D,
	0xD4, 0x47, 0x78, 0x80, 0x08, 0x88, 0x86, 0x62, 0x2D, 0xD7, 0x79,
	0x94, 0x46, 0x62, 0x26, 0x68, 0x8E, 0xEE, 0xE6, 0x6A, 0xA1};

static const int v1_ro_space_attributes = TPM_NV_PER_GLOBALLOCK |
			TPM_NV_PER_PPWRITE;
static const int v1_rw_space_attributes = TPM_NV_PER_GLOBALLOCK |
			TPM_NV_PER_PPWRITE;
static const u8 v1_pcr0_unchanged_policy[0];

/*
 * This is used to initialise the TPM space for recovery hash after defining
 * it. Since there is no data available to calculate hash at the point where TPM
 * space is defined, initialise it to all 0s.
 */
static const u8 rec_hash_data[REC_HASH_NV_SIZE] = {};

static u32 extend_pcr(struct vboot_info *vboot, int pcr,
		      enum vb2_pcr_digest which_digest)
{
	u8 buffer[VB2_PCR_DIGEST_RECOMMENDED_SIZE];
	u8 out[VB2_PCR_DIGEST_RECOMMENDED_SIZE];
	struct vb2_context *ctx = vboot_get_ctx(vboot);
	u32 size = sizeof(buffer);
	int rv;

	rv = vb2api_get_pcr_digest(ctx, which_digest, buffer, &size);
	if (rv != VB2_SUCCESS)
		return rv;
	if (size < TPM_PCR_MINIMUM_DIGEST_SIZE)
		return VB2_ERROR_UNKNOWN;

	return tpm_extend(vboot->tpm, pcr, buffer, out);
}

int cros_tpm_extend_pcrs(struct vboot_info *vboot)
{
	return extend_pcr(vboot, 0, BOOT_MODE_PCR) ||
	       extend_pcr(vboot, 1, HWID_DIGEST_PCR);
}

static int setup_space(struct udevice *dev, enum cros_nvdata_index index,
		       uint attr, const void *nv_policy, uint nv_policy_size,
		       const void *data, uint size)
{
	int ret;

	ret = cros_nvdata_setup_walk(index, attr, size, nv_policy,
				     nv_policy_size);
	if (ret)
		return ret;
	ret = cros_nvdata_write_walk(index, data, size);
	if (ret)
		return ret;

	return 0;
}

static int setup_spaces(struct vboot_info *vboot)
{
	enum tpm_version version = tpm_get_version(vboot->tpm);
	struct vb2_context *ctx = vboot_get_ctx(vboot);
	int ret;

	/*
	 * Of all NVRAM spaces defined by this function the firmware space
	 * must be defined last, because its existence is considered an
	 * indication that TPM factory initialisation was successfully
	 * completed.
	 */
	ret = setup_space(vboot->tpm, CROS_NV_SECDATAK, version == TPM_V1 ?
			  v1_rw_space_attributes : v2_rw_space_attributes,
			  NULL, 0, secdata_kernel, sizeof(secdata_kernel));
	if (ret)
		return ret;

	if (vboot->has_rec_mode_mrc) {
		ret = setup_space(vboot->tpm, CROS_NV_REC_HASH,
				  version == TPM_V1 ? v1_ro_space_attributes :
				  v2_ro_space_attributes,
				  version == TPM_V1 ? v1_pcr0_unchanged_policy :
				  v2_pcr0_unchanged_policy,
				  version == TPM_V1 ?
				  sizeof(v1_pcr0_unchanged_policy) :
				  sizeof(v2_pcr0_unchanged_policy),
				  rec_hash_data, sizeof(rec_hash_data));
		if (ret)
			return ret;
	}
	vb2api_secdata_create(ctx);
	ret = setup_space(vboot->tpm, CROS_NV_SECDATA, version == TPM_V1 ?
			  v1_rw_space_attributes : v2_rw_space_attributes,
			  NULL, 0, ctx->secdata, VB2_SECDATA_SIZE);
	if (ret)
		return ret;

	return 0;
}

static u32 v2_factory_initialise_tpm(struct vboot_info *vboot)
{
	int ret;

	log_warning("Setting up TPM for first time from factory\n");
	ret = tpm_force_clear(vboot->tpm);
	if (ret)
		return ret;

	return setup_spaces(vboot);
}

static int v1_factory_initialise_tpm(struct vboot_info *vboot)
{
	struct tpm_permanent_flags pflags;
	int ret;

	ret = tpm_get_permanent_flags(vboot->tpm, &pflags);
	if (ret != TPM_SUCCESS)
		return -EIO;

	/*
	 * TPM may come from the factory without physical presence finalised.
	 * Fix if necessary.
	 */
	log_debug("physical_presence_lifetime_lock=%d\n",
		  pflags.physical_presence_lifetime_lock);
	if (!pflags.physical_presence_lifetime_lock) {
		log_debug("Finalising physical presence\n");
		ret = tpm_finalise_physical_presence(vboot->tpm);
		if (ret != TPM_SUCCESS)
			return -EIO;
	}

	/*
	 * The TPM will not enforce the NV authorization restrictions until the
	 * execution of a TPM_NV_DefineSpace with the handle of
	 * TPM_NV_INDEX_LOCK.  Here we create that space if it doesn't already
	 * exist
	 */
	log_debug("nv_locked=%d\n", pflags.nv_locked);
	if (!pflags.nv_locked) {
		log_debug("Enabling NV locking\n");
		ret = tpm_nv_set_locked(vboot->tpm);
		if (ret != TPM_SUCCESS)
			return -EIO;
	}

	/* Clear TPM owner, in case the TPM is already owned for some reason */
	log_debug("TPM: Clearing owner\n");
	ret = tpm_clear_and_reenable(vboot->tpm);
	if (ret != TPM_SUCCESS)
		return -EIO;

	return setup_spaces(vboot);
}

/**
 * Perform one-time initialisations.
 *
 * Create the NVRAM spaces, and set their initial values as needed.  Sets the
 * nvLocked bit and ensures the physical presence command is enabled and
 * locked.
 */
int cros_tpm_factory_initialise(struct vboot_info *vboot)
{
	enum tpm_version version = tpm_get_version(vboot->tpm);
	struct vb2_context *ctx = vboot_get_ctx(vboot);
	int ret;

	/* Defines and sets vb2 secdata space */
	vb2api_secdata_create(ctx);

	log_debug("TPM: factory initialisation\n");

	/*
	 * Do a full test.  This only happens the first time the device is
	 * turned on in the factory, so performance is not an issue.  This is
	 * almost certainly not necessary, but it gives us more confidence
	 * about some code paths below that are difficult to
	 * test---specifically the ones that set lifetime flags, and are only
	 * executed once per physical TPM.
	 */
	ret = tpm_self_test_full(vboot->tpm);
	if (ret)
		return -EIO;

	ret = version == TPM_V1 ?
		v1_factory_initialise_tpm(vboot) :
		v2_factory_initialise_tpm(vboot);
	if (ret)
		return ret;

	log_debug("TPM: factory initialisation successful\n");

	return 0;
}

static u32 tpm_get_flags(struct udevice *dev, bool *disablep,
			 bool *deactivatedp, bool *nvlockedp)
{
	struct tpm_permanent_flags pflags;
	u32 ret = tpm_get_permanent_flags(dev, &pflags);

	if (ret == TPM_SUCCESS) {
		if (disablep)
			*disablep = pflags.disable;
		if (deactivatedp)
			*deactivatedp = pflags.deactivated;
		if (nvlockedp)
			*nvlockedp = pflags.nv_locked;
		log_debug("TPM: flags disable=%d, deactivated=%d, nv_locked=%d\n",
			  pflags.disable, pflags.deactivated, pflags.nv_locked);
	}
	return ret;
}

static u32 tpm1_invoke_state_machine(struct vboot_info *vboot,
				     struct udevice *dev)
{
	bool disable;
	bool deactivated;
	u32 ret = TPM_SUCCESS;

	/* Check that the TPM is enabled and activated */
	ret = tpm_get_flags(dev, &disable, &deactivated, NULL);
	if (ret != TPM_SUCCESS) {
		log_err("TPM: Can't read capabilities\n");
		return ret;
	}

	if (!!deactivated != vboot->deactivate_tpm) {
		log_info("TPM: Unexpected TPM deactivated state; toggling..\n");
		ret = tpm_physical_set_deactivated(dev, !deactivated);
		if (ret != TPM_SUCCESS) {
			log_err("TPM: Can't toggle deactivated state\n");
			return ret;
		}

		deactivated = !deactivated;
		ret = TPM_E_MUST_REBOOT;
	}

	if (disable && !deactivated) {
		log_info("TPM: disabled (%d). Enabling..\n", disable);

		ret = tpm_physical_enable(dev);
		if (ret != TPM_SUCCESS) {
			log_err("TPM: Can't set enabled state\n");
			return ret;
		}

		log_info("TPM: Must reboot to re-enable\n");
		ret = TPM_E_MUST_REBOOT;
	}

	return ret;
}

/*
 * This starts the TPM and establishes the root of trust for the
 * anti-rollback mechanism.  SetupTPM can fail for three reasons.  1 A bug. 2 a
 * TPM hardware failure. 3 An unexpected TPM state due to some attack.  In
 * general we cannot easily distinguish the kind of failure, so our strategy is
 * to reboot in recovery mode in all cases.  The recovery mode calls SetupTPM
 * again, which executes (almost) the same sequence of operations.  There is a
 * good chance that, if recovery mode was entered because of a TPM failure, the
 * failure will repeat itself.  (In general this is impossible to guarantee
 * because we have no way of creating the exact TPM initial state at the
 * previous boot.)  In recovery mode, we ignore the failure and continue, thus
 * giving the recovery kernel a chance to fix things (that's why we don't set
 * bGlobalLock).  The choice is between a knowingly insecure device and a
 * bricked device.
 *
 * As a side note, observe that we go through considerable hoops to avoid using
 * the STCLEAR permissions for the index spaces.  We do this to avoid writing
 * to the TPM flashram at every reboot or wake-up, because of concerns about
 * the durability of the NVRAM.
 */
static u32 do_setup(struct vboot_info *vboot, bool s3flag)
{
	u32 ret;

	ret = tpm_open(vboot->tpm);
	if (ret != TPM_SUCCESS) {
		log_err("TPM: Can't initialise\n");
		goto out;
	}

	/* Handle special init for S3 resume path */
	if (s3flag) {
		ret = tpm_resume(vboot->tpm);
		if (ret == TPM_INVALID_POSTINIT)
			log_info("TPM: Already initialised\n");

		return TPM_SUCCESS;
	}

	ret = tpm_startup(vboot->tpm, TPM_ST_CLEAR);
	if (ret != TPM_SUCCESS) {
		log_err("TPM: Can't run startup command\n");
		goto out;
	}

	ret = tpm_tsc_physical_presence(vboot->tpm,
					TPM_PHYSICAL_PRESENCE_PRESENT);
	if (ret != TPM_SUCCESS) {
		/*
		 * It is possible that the TPM was delivered with the physical
		 * presence command disabled.  This tries enabling it, then
		 * tries asserting PP again.
		 */
		ret = tpm_tsc_physical_presence(vboot->tpm,
					TPM_PHYSICAL_PRESENCE_CMD_ENABLE);
		if (ret != TPM_SUCCESS) {
			log_err("Can't enable physical presence command\n");
			goto out;
		}

		ret = tpm_tsc_physical_presence(vboot->tpm,
				TPM_PHYSICAL_PRESENCE_PRESENT);
		if (ret != TPM_SUCCESS) {
			log_err("Can't assert physical presence\n");
			goto out;
		}
	}

	if (tpm_get_version(vboot->tpm) == TPM_V1) {
		ret = tpm1_invoke_state_machine(vboot, vboot->tpm);
		if (ret != TPM_SUCCESS)
			return ret;
	}

out:
	if (ret != TPM_SUCCESS)
		log_err("TPM: setup failed\n");
	else
		log_info("TPM: setup succeeded\n");

	return ret;
}

int cros_tpm_setup(struct vboot_info *vboot)
{
	struct vb2_context *ctx = vboot_get_ctx(vboot);
	int ret;

	ret = do_setup(vboot, ctx->flags & VB2_CONTEXT_S3_RESUME);
	if (ret == TPM_E_MUST_REBOOT)
		ctx->flags |= VB2_CONTEXT_SECDATA_WANTS_REBOOT;

	return ret;
}

int vb2ex_tpm_clear_owner(struct vb2_context *ctx)
{
	struct vboot_info *vboot = vboot_get();
	u32 rv;

	log_info("Clearing TPM owner\n");
	rv = tpm_clear_and_reenable(vboot->tpm);
	if (rv)
		return VB2_ERROR_EX_TPM_CLEAR_OWNER;

	return VB2_SUCCESS;
}
