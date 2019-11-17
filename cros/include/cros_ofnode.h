/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Device-tree node-handling code
 *
 * Copyright 2018 Google LLC
 */

#ifndef __CROS_OFNODE_H
#define __CROS_OFNODE_H

#include <dm/ofnode.h>
#include <dm/of_extra.h>

enum cros_compress_t {
	CROS_COMPRESS_NONE,
	CROS_COMPRESS_LZO,
};

/* Structures to hold Chromium OS specific configuration from the FMAP */

/* struct fmap_entry is now defined in fdtdec.h */
#include <fdtdec.h>

/* List of EC images available */
enum ec_index_t {
	EC_MAIN,
	EC_PD,

	EC_COUNT
};

struct fmap_firmware_ec {
	struct fmap_entry ro;
	struct fmap_entry rw;
};

struct fmap_firmware_entry {
	struct fmap_entry all;		/* how big is the whole section? */
	struct fmap_entry spl;
	struct fmap_entry boot;		/* U-Boot */
	struct fmap_entry vblock;
	struct fmap_entry firmware_id;

	/* The offset of the first block of R/W firmware when stored on disk */
	u64 block_offset;

	/* EC RW binary, and RO binary if present */
	struct fmap_firmware_ec ec[EC_COUNT];

	struct fmap_entry gbb;
	struct fmap_entry fmap;

	/* U-Boot recovery */
	struct fmap_entry spl_rec;
	struct fmap_entry boot_rec;
};

/*
 * Only sections that are used during booting are put here. More sections will
 * be added if required.
 * TODO(sjg@chromium.org): Unify readonly into struct fmap_firmware_entry
 */
struct cros_fmap {
	struct fmap_firmware_entry readonly;
	struct fmap_firmware_entry readwrite_a;
	struct fmap_firmware_entry readwrite_b;
	struct fmap_entry readwrite_devkey;
	struct fmap_entry elog;
	u32  flash_base;
};

struct fmap_entry;
struct cros_fmap;

/**
 * cros_ofnode_flashmap() - Decode Chromium OS-specific configuration from fdt
 *
 * @config: Returns decoded FMAP
 * @return 0 if OK, -ve on error
 */
int cros_ofnode_flashmap(struct cros_fmap *config);

/**
 * Return the /chromeos-config ofnode
 *
 * @return ofnode found, of ofnode_null() if not found
 */
ofnode cros_ofnode_config_node(void);

/**
 * Decode a named region within a memory bank of a given type.
 *
 * The properties are looked up in the /chromeos-config node
 *
 * See ofnode_decode_memory_region() for more details.
 *
 * @mem_type:	Type of memory to use, which is a name, such as "u-boot" or
 *		"kernel".
 * @suffix:	String to append to the memory/offset property names
 * @basep:	Returns base of region
 * @sizep:	Returns size of region
 * @return 0 if OK, -ve on error
 */
int cros_ofnode_decode_region(const char *mem_type, const char *suffix,
			      fdt_addr_t *basep, fdt_size_t *sizep);

/**
 * Returns information from the FDT about memory for a given root
 *
 * @name:	Root name of alias to search for
 * @config:	structure to use to return information
 * @return 0 if OK, -ve on error
 */
int cros_ofnode_memory(const char *name, struct fdt_memory *config);

/**
 * cros_ofnode_find_locale() - Find the entry which contains a locale
 *
 * Locate the flashmap entry containing the Chromium OS locale information for
 * a given locale, used for the verified boot screens.
 *
 * @name: Name of locale filename to find (e.g. 'locale_en.bin')
 * @entry: Returns entry containing this locale
 * @return 0 if OK, -EINVAL if no locales could be found, -ENOENT if the
 *	requested locale was not found, other -ve error value on other error
 */
int cros_ofnode_find_locale(const char *name, struct fmap_entry *entry);

/**
 * cros_ofnode_dump_fmap() - Dump the position and side of the fmap regions
 *
 * This dumps out commonly used FMAP regions, showing their offset and size
 *
 * @config: FMAP config to dump
 */
void cros_ofnode_dump_fmap(struct cros_fmap *config);
#endif /* __CROS_OFNODE_H */
