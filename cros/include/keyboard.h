/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Keyboard handling
 *
 * Copyright 2018 Google LLC
 */

#ifndef __CROS_KEYBOARD_H
#define __CROS_KEYBOARD_H

/**
 * Initialise the remap_key structs from the mainboard fdt arrays.
 *
 * If the array exist in the fdt, it is copied to a remap_key structure
 * and the pointer set to the beginning of the array. If there isn't a
 * fdt key map, the ptr is NULL.
 *
 * @vboot:	vboot structure
 * @return	0 if OK, -ve on error
 */
int vboot_keymap_init(struct vboot_info *vboot);

#endif /* __CROS_KEYBOARD_H */
