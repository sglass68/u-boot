// SPDX-License-Identifier: GPL-2.0+
/*
 * Implementation of vboot display callbacks
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <dm.h>
#include <mapmem.h>
#include <video.h>
#include <video_console.h>
#include <cros/cros_ofnode.h>
#include <cros/cros_common.h>
#include <cros/screens.h>
#include <cros/vboot.h>

DECLARE_GLOBAL_DATA_PTR;

/**
 * Write out a line of characters to the display
 *
 * @ch: Character to output
 * @len: Number of characters to output
 */
static void out_line(struct udevice *console, int ch, int len)
{
	int i;

	for (i = 0; i < len; i++)
		vidconsole_put_char(console, ch);
}

static void out_str(struct udevice *console, const char *msg)
{
	while (*msg)
		vidconsole_put_char(console, *msg++);
}

/* Print the message on the center of display */
static void print_on_center(struct udevice *console, const char *message)
{
	struct vidconsole_priv *priv = dev_get_uclass_priv(console);
	int cols = priv->cols;
	int rows = priv->rows;
	int row, len;

	vidconsole_position_cursor(console, 0, 0);

	for (row = 0; row < (rows - 4) / 2; row++)
		out_line(console, '.', cols);
	out_line(console, ' ', cols);
	out_line(console, ' ', cols);

	/* Center the message on its line */
	len = strlen(message);
	out_line(console, ' ', (cols - len) / 2);
	out_str(console, message);
	out_line(console, ' ', (cols - len + 1) / 2);

	out_line(console, ' ', cols);
	out_line(console, ' ', cols);

	/* Don't write to the last row, since that will cause a scroll */
	for (row += 5; row < rows - 1; row++)
		out_line(console, '.', cols);
}

VbError_t VbExDisplayScreen(u32 screen_type, u32 locale)
{
	struct vboot_info *vboot = vboot_get();
	const char *msg = NULL;

	if (!vboot_draw_screen(screen_type, locale)) {
		video_sync(vboot->video, true);
		return VBERROR_SUCCESS;
	}

	/*
	 * Show the debug messages for development. It is a backup method
	 * when GBB does not contain a full set of bitmaps.
	 */
	switch (screen_type) {
	case VB_SCREEN_BLANK:
		/* clear the screen */
		video_clear(vboot->video);
		break;
	case VB_SCREEN_DEVELOPER_WARNING:
		msg = "developer mode warning";
		break;
	case VB_SCREEN_RECOVERY_INSERT:
		msg = "insert recovery image";
		break;
	case VB_SCREEN_RECOVERY_NO_GOOD:
		msg = "insert image invalid";
		break;
	case VB_SCREEN_RECOVERY_TO_DEV:
		msg = "recovery to dev";
		break;
	case VB_SCREEN_DEVELOPER_TO_NORM:
		msg = "developer to norm";
		break;
	case VB_SCREEN_WAIT:
		msg = "wait for ec update";
		break;
	case VB_SCREEN_TO_NORM_CONFIRMED:
		msg = "to norm confirmed";
		break;
	case VB_SCREEN_OS_BROKEN:
		msg = "os broken";
		break;
	case VB_SCREEN_DEVELOPER_WARNING_MENU:
		msg = "developer warning menu";
		break;
	case VB_SCREEN_DEVELOPER_MENU:
		msg = "developer menu";
		break;
	case VB_SCREEN_RECOVERY_TO_DEV_MENU:
		msg = "recovery to dev menu";
		break;
	case VB_SCREEN_DEVELOPER_TO_NORM_MENU:
		msg = "developer to norm menu";
		break;
	case VB_SCREEN_LANGUAGES_MENU:
		msg = "languages menu";
		break;
	case VB_SCREEN_OPTIONS_MENU:
		msg = "options menu";
		break;
	case VB_SCREEN_ALT_FW_PICK:
		msg = "altfw pick";
		break;
	case VB_SCREEN_ALT_FW_MENU:
		msg = "altfw menu";
		break;
	default:
		log_debug("Not a valid screen type: %08x.\n", screen_type);
		return VBERROR_INVALID_SCREEN_INDEX;
	}

	if (msg)
		print_on_center(vboot->console, msg);

	return VBERROR_SUCCESS;
}

/**
 * show_cdata_string() - Display a prompt followed a checked string
 *
 * This is used to show string information from crossystem_data. if this is
 * not set up correctly then we need to make sure we don't print garbage.
 *
 * @prompt:	Prompt string to show
 * @str:	String to print. If the length if > 200 then we assume it is
 *		corrupted
 */
static void show_cdata_string(struct udevice *console, const char *prompt,
			      const char *str)
{
	out_str(console, prompt);
	if (strlen(str) > 200)
		str = "corrupted";
	out_str(console, str);
	out_str(console, "\n");
}

VbError_t VbExDisplayDebugInfo(const char *info_str)
{
	struct vboot_info *vboot = vboot_get();
	struct udevice *console = vboot->console;

	vidconsole_position_cursor(console, 0, 0);
	out_str(console, info_str);

	show_cdata_string(console, "read-only firmware id: ",
			  vboot->readonly_firmware_id);
	show_cdata_string(console, "active firmware id: ", vboot->firmware_id);

	return VBERROR_SUCCESS;
}

VbError_t VbExGetlocalisationCount(u32 *count)
{
	*count = vboot_get_locale_count();

	return VBERROR_SUCCESS;
}

VbError_t VbExDisplayMenu(u32 screen_type, u32 locale,
			  u32 selected_index, u32 disabled_idx_mask,
			  u32 redraw_base)
{
	return vboot_draw_ui(screen_type, locale, selected_index,
			     disabled_idx_mask, redraw_base);
}
