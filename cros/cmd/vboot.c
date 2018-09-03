// SPDX-License-Identifier: GPL-2.0+
/*
 * Implements the 'vboot' command which provides access to the verified boot
 * flow.
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <command.h>
#include <cros/stages.h>
#include <cros/vboot.h>

/* The next stage of vboot to run (used for repeatable commands) */
static enum vboot_stage_t vboot_next_stage;

int board_run_command(const char *cmd)
{
	struct vboot_info *vboot = vboot_get_alloc();

	printf("Secure boot mode: %s\n", cmd);
	if (!strcmp(cmd, "vboot") || !strcmp(cmd, "vboot_go_auto")) {
		vboot_run_auto(vboot, 0);
		/* Should not return */
	} else {
		printf("Unknown command '%s'\n", cmd);
		panic("board_run_command() failed");
	}

	return 1;
}

static int do_vboot_go(cmd_tbl_t *cmdtp, int flag, int argc,
		       char * const argv[])
{
	struct vboot_info *vboot = vboot_get_alloc();
	const char *stage;
	uint flags = 0;
	int ret;

	/* strip off 'go' */
	argc--;
	argv++;
	if (argc < 1)
		return CMD_RET_USAGE;
	if (!strcmp("-n", argv[0])) {
		flags |= VBOOT_FLAG_CMDLINE;
		argc--;
		argv++;
		if (argc < 1)
			return CMD_RET_USAGE;
	}

	stage = argv[0];
	if (!strcmp(stage, "ro")) {
		ret = vboot_run_stages(vboot, true, flags);
	} else if (!strcmp(stage, "rw")) {
		ret = vboot_run_stages(vboot, false, flags);
	} else if (!strcmp(stage, "auto")) {
		ret = vboot_run_auto(vboot, flags);
	} else {
		enum vboot_stage_t stagenum;

		if (flag & CMD_FLAG_REPEAT) {
			stagenum = vboot_next_stage;
		} else {
			if (!strcmp("start", stage)) {
				stagenum = VBOOT_STAGE_FIRST_VER;
			} else if (!strcmp("start_rw", stage)) {
				stagenum = VBOOT_STAGE_FIRST_RW;
			} else if (!strcmp("next", stage)) {
				stagenum = vboot_next_stage;
			} else {
				stagenum = vboot_find_stage(stage);
				if (stagenum == VBOOT_STAGE_NONE) {
					printf("Umknown stage\n");
					return CMD_RET_USAGE;
				}
			}
		}
		if (stagenum == VBOOT_STAGE_COUNT) {
			printf("All vboot stages are complete\n");
			return 1;
		}

		ret = vboot_run_stage(vboot, stagenum);
		if (!ret)
			vboot_next_stage = stagenum + 1;
	}

	return ret ? 1 : 0;
}

static int do_vboot_list(cmd_tbl_t *cmdtp, int flag, int argc,
			 char * const argv[])
{
	enum vboot_stage_t stagenum;
	const char *name;

	printf("Available stages:\n");
	for (stagenum = VBOOT_STAGE_FIRST_VER; stagenum < VBOOT_STAGE_COUNT;
	     stagenum++) {
		name = vboot_get_stage_name(stagenum);
		printf("   %s\n", name);
	}

	return 0;
}

static cmd_tbl_t cmd_vboot_sub[] = {
	U_BOOT_CMD_MKENT(go, 4, 0, do_vboot_go, "", ""),
	U_BOOT_CMD_MKENT(list, 4, 0, do_vboot_list, "", ""),
};

/* Process a vboot sub-command */
static int do_vboot(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	cmd_tbl_t *c;

	/* Strip off leading 'vboot' command argument */
	argc--;
	argv++;

	c = find_cmd_tbl(argv[0], cmd_vboot_sub, ARRAY_SIZE(cmd_vboot_sub));
	if (c)
		return c->cmd(cmdtp, flag, argc, argv);
	else
		return CMD_RET_USAGE;
}

U_BOOT_CMD(vboot, 4, 1, do_vboot, "Chromium OS Verified boot",
	   "go -n [ro|rw|auto|start|next|<stage>]  Run verified boot stage (repeatable)\n"
	   "vboot list           List verified boot stages");

static int do_vboot_go_auto(cmd_tbl_t *cmdtp, int flag, int argc,
			    char * const argv[])
{
	board_run_command("vboot");

	return 0;
}

U_BOOT_CMD(vboot_go_auto, 4, 1, do_vboot_go_auto, "Chromium OS Verified boot",
	   "      Run full verified boot");
