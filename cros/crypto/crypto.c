// SPDX-License-Identifier: GPL-2.0+
/*
 * Stubs for hardware crypto which is not connected up yet - see crypto_algos
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <vb2_api.h>

/* No-op stubs that can be overridden by SoCs with hardware-crypto support */
int vb2ex_hwcrypto_digest_init(enum vb2_hash_algorithm hash_alg,
			       u32 data_size)
{
	return VB2_ERROR_EX_HWCRYPTO_UNSUPPORTED;
}

int vb2ex_hwcrypto_digest_extend(const u8 *buf, u32 size)
{
	return VB2_ERROR_EX_HWCRYPTO_UNSUPPORTED;
}

int vb2ex_hwcrypto_digest_finalize(u8 *digest, u32 digest_size)
{
	return VB2_ERROR_EX_HWCRYPTO_UNSUPPORTED;
}
