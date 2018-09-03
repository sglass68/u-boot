/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Taken from depthcharge file of the same name
 *
 * Copyright 2018 Google LLC
 */

#ifndef __CROS_SCREENS_H
#define __CROS_SCREENS_H

/**
 * vboot_draw_screen() - Draw a given vboot screen
 *
 * Draws a number of predefined screens for verified boot
 *
 * @screen: Screen number to draw (VB_SCREEN_...)
 * @locale: Locale to use (numbered from 0 to n-1, listed in 'locales' file)
 * @return VBERROR_SUCCESS (0) if OK, other VBERROR_... value on error
 */
int vboot_draw_screen(u32 screen, u32 locale);

/**
 * vboot_draw_ui() - Draw a given menu
 *
 * Draws a number of predefined menus for verified boot
 *
 * @screen: Screen number to draw (VB_SCREEN_...)
 * @locale: Locale to use (numbered from 0 to n-1, listed in 'locales' file)
 * @selected_index: Menu index that is currently selected (0..n-1)
 * @disabled_idx_mask: Mask for menu items that are not shown (bit n = index n)
 * @redraw_base: 1 to do a full screen redraw, 0 to draw on top of the existing
 *	screen
 * @return VBERROR_SUCCESS (0) if OK, other VBERROR_... value on error
 */
int vboot_draw_ui(u32 screen, u32 locale,
		  u32 selected_index, u32 disabled_idx_mask,
		  u32 redraw_base);

/**
 * vboot_get_locale_count() - Return number of supported locales
 *
 * @return number of supported locales
 */
int vboot_get_locale_count(void);

#endif /* __CROS_SCREENS_H */
