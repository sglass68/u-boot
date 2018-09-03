/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Taken from coreboot file payloads/libpayload/include/archive.h
 *
 * Copyright 2018 Google LLC
 * written by Daisuke Nojiri <dnojiri@chromium.org>
 */

#ifndef __CROS_CB_ARCHIVE_H
#define __CROS_CB_ARCHIVE_H

/*
 * Archive file layout:
 *
 *  +----------------------------------+
 *  |           root header            |
 *  +----------------------------------+
 *  |         file_header[0]           |
 *  +----------------------------------+
 *  |         file_header[1]           |
 *  +----------------------------------+
 *  |              ...                 |
 *  +----------------------------------+
 *  |         file_header[count-1]     |
 *  +----------------------------------+
 *  |         file(0) content          |
 *  +----------------------------------+
 *  |         file(1) content          |
 *  +----------------------------------+
 *  |              ...                 |
 *  +----------------------------------+
 *  |         file(count-1) content    |
 *  +----------------------------------+
 */

#define VERSION		0
#define CBAR_MAGIC	"CBAR"
#define NAME_LENGTH	32

/* Root header */
struct directory {
	char magic[4];
	u32 version;	/* version of the header. little endian */
	u32 size;		/* total size of archive. little endian */
	u32 count;		/* number of files. little endian */
};

/* File header */
struct dentry {
	/* file name. nul-terminated if shorter than NAME_LENGTH */
	char name[NAME_LENGTH];
	/* file offset from the root header, little endian */
	u32 offset;
	/* file size, little endian */
	u32 size;
};

static inline struct dentry *get_first_dentry(const struct directory *dir)
{
	return (struct dentry *)(dir + 1);
}

static inline u32 get_first_offset(const struct directory *dir)
{
	return sizeof(struct directory) + sizeof(struct dentry) * dir->count;
}

#endif /* __CROS_CB_ARCHIVE_H */
