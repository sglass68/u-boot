// SPDX-License-Identifier: GPL-2.0+
/*
 * Implementation of TPM callbacks
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <config.h>
#include <tpm-v1.h>
#include <cros/vboot.h>

VbError_t VbExTpmInit(void)
{
	/* tpm_lite lib doesn't call VbExTpmOpen after VbExTpmInit */
	return VbExTpmOpen();
}

VbError_t VbExTpmClose(void)
{
	struct vboot_info *vboot = vboot_get();

	if (tpm_close(vboot->tpm))
		return VBERROR_UNKNOWN;

	return VBERROR_SUCCESS;
}

VbError_t VbExTpmOpen(void)
{
	struct vboot_info *vboot = vboot_get();

	if (tpm_open(vboot->tpm))
		return VBERROR_UNKNOWN;

	return VBERROR_SUCCESS;
}

VbError_t VbExTpmSendReceive(const u8 *request, u32 request_length,
			     u8 *response, u32 *response_length)
{
	struct vboot_info *vboot = vboot_get();
	size_t resp_len = *response_length;
	int ret;

	ret = tpm_xfer(vboot->tpm, request, request_length, response,
		       &resp_len);
	*response_length = resp_len;
	if (ret)
		return VBERROR_UNKNOWN;

	return VBERROR_SUCCESS;
}
