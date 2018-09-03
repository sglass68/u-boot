// SPDX-License-Identifier: GPL-2.0+
/*
 * Keyboard handling, including the implementation of VbExKeyboardRead() and
 * remapping of keys for the Fully-Automated Firmware Test (FAFT).
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <gbb_header.h>
#include <cros/keyboard.h>
#include <cros/cros_ofnode.h>
#include <cros/vboot.h>
#include <dm/ofnode.h>

/* Control Sequence Introducer for arrow keys */
#define CSI_0		0x1B	/* Escape */
#define CSI_1		0x5B	/* '[' */

/* Types of keys to be overridden */
enum key_type_t {
	KEY_TYPE_ASCII,
	KEY_TYPE_SPECIAL,

	KEY_TYPE_COUNT,
};

/* Each fdt key array holds three pairs of keys */
#define KEY_ARRAY_SIZE	(2 * 3)

/**
 * struct remap_key - Keys to remap
 *
 * @array: List of keys to remap, as pairs:
 *		- keycode to remap
 *		- new keycode to use
 * @array_ptr: pointes to @array if it is initialised, elseNULL
 */
struct remap_key {
	u32 array[KEY_ARRAY_SIZE];
	u32 *array_ptr;
};

static struct remap_key remap_keys[KEY_TYPE_COUNT];

int vboot_keymap_init(struct vboot_info *vboot)
{
	ofnode node = vboot->config;

	if (!ofnode_valid(node))
		return -ENOENT;

	if (!ofnode_read_u32_array(node, "faft-key-remap-special",
				   remap_keys[KEY_TYPE_SPECIAL].array,
				   KEY_ARRAY_SIZE)) {
		remap_keys[KEY_TYPE_SPECIAL].array_ptr =
			remap_keys[KEY_TYPE_SPECIAL].array;
	}

	if (!ofnode_read_u32_array(node, "faft-key-remap-ascii",
				   remap_keys[KEY_TYPE_ASCII].array,
				   KEY_ARRAY_SIZE)) {
		remap_keys[KEY_TYPE_ASCII].array_ptr =
			remap_keys[KEY_TYPE_ASCII].array;
	}

	return 0;
}

/**
 * Replace normal ascii keys and special keys if the mainboard
 * fdt has either an ascii or a special key remap array.
 *
 * @keyp:	pointer to key to be checked on entry; on exit the value of the
 *		remapped key on success, or the original key on failure
 * @keytype:	KEY_TYPE_ASCII or KEY_TYPE_SPECIAL
 *
 * @return	0 on success, -EPERM if overriding is not enabled, -ENOENT if
 *		no override found
 */
static int faft_key_remap(int *keyp, enum key_type_t keytype)
{
	struct vboot_info *vboot = vboot_get();
	u32 gbb_flags = vboot_get_gbb_flags(vboot);
	int i;

	if ((gbb_flags & GBB_FLAG_FAFT_KEY_OVERIDE) == 0)
		return -1;

	if (remap_keys[keytype].array_ptr) {
		for (i = 0; i < KEY_ARRAY_SIZE; i += 2) {
			if (*keyp == remap_keys[keytype].array_ptr[i]) {
				*keyp = remap_keys[keytype].array_ptr[i + 1];
				return 0;
			}
		}
	}

	return -ENOENT;
}

u32 VbExKeyboardRead(void)
{
	int ch = 0;

	/* No input available */
	if (!tstc())
		goto out;

	/* Read a non-Escape character or a standalone Escape character */
	ch = getc();
	if (ch != CSI_0 || !tstc()) {
		/* Handle normal asci keys for FAFT keyboard matrix */
		if (faft_key_remap(&ch, KEY_TYPE_ASCII) >= 0)
			goto out;

		/*
		 * Special handle of Ctrl-Enter, which is converted into '\n'
		 * by i8042 driver.
		 */
		if (ch == '\n')
			ch = VB_KEY_CTRL_ENTER;
		goto out;
	}

	/* Filter out non- Escape-[ sequence */
	if (getc() != CSI_1) {
		ch = 0;
		goto out;
	}

	/* Get special keys */
	ch = getc();

	/* Handle special keys for FAFT keyboard matrix */
	if (faft_key_remap(&ch, KEY_TYPE_SPECIAL) >= 0)
		goto out;

	/* Special values for arrow up/down/left/right */
	switch (ch) {
	case 'A':
		ch = VB_KEY_UP;
		break;
	case 'B':
		ch = VB_KEY_DOWN;
		break;
	case 'C':
		ch = VB_KEY_RIGHT;
		break;
	case 'D':
		ch = VB_KEY_LEFT;
		break;
	default:
		/* Filter out speical keys that we do not recognise */
		ch = 0;
		break;
	}

out:
	return ch;
}

u32 VbExKeyboardReadWithFlags(u32 *flags_ptr)
{
	if (flags_ptr)
		/* We trust keyboards on u-boot legacy devices */
		*flags_ptr = VB_KEY_FLAG_TRUSTED_KEYBOARD;
	return VbExKeyboardRead();
}
