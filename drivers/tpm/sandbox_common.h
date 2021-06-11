/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Common features for sandbox TPM1 and TPM2 implementations
 *
 * Copyright 2021 Google LLC
 */

#ifndef __TPM_SANDBOX_COMMON_H
#define __TPM_SANDBOX_COMMON_H

/*
 * These numbers derive from adding the sizes of command fields as shown in
 * the TPM commands manual.
 */
#define TPM_HDR_LEN	10

/* These are the different non-volatile spaces that we emulate */
enum sandbox_nv_space {
	NV_SEQ_ENABLE_LOCKING,
	NV_SEQ_GLOBAL_LOCK,
	NV_SEQ_FIRMWARE,
	NV_SEQ_KERNEL,
	NV_SEQ_BACKUP,
	NV_SEQ_FWMP,
	NV_SEQ_REC_HASH,

	NV_SEQ_COUNT,
};

/* TPM NVRAM location indices */
#define FIRMWARE_NV_INDEX		0x1007
#define KERNEL_NV_INDEX			0x1008
#define BACKUP_NV_INDEX			0x1009
#define FWMP_NV_INDEX			0x100a
#define MRC_REC_HASH_NV_INDEX		0x100b

struct __packed rollback_space_kernel {
	/* Struct version, for backwards compatibility */
	uint8_t struct_version;
	/* Unique ID to detect space redefinition */
	uint32_t uid;
	/* Kernel versions */
	uint32_t kernel_versions;
	/* Reserved for future expansion */
	uint8_t reserved[3];
	/* Checksum (v2 and later only) */
	uint8_t crc8;
};

/* Size of each non-volatile space */
#define NV_DATA_SIZE		0x28

struct nvdata_state {
	bool present;
	int length;
	u8 data[NV_DATA_SIZE];
};

int sb_tpm_index_to_seq(uint index);

void sb_tpm_read_data(const struct nvdata_state nvdata[NV_SEQ_COUNT],
		      enum sandbox_nv_space seq, u8 *recvbuf, int data_ofs,
		      int length);

void sb_tpm_write_data(struct nvdata_state nvdata[NV_SEQ_COUNT],
		       enum sandbox_nv_space seq, const u8 *buf, int data_ofs,
		       int length);

void sb_tpm_define_data(struct nvdata_state nvdata[NV_SEQ_COUNT],
			enum sandbox_nv_space seq, int length);

#endif
