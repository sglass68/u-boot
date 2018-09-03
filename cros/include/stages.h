/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Verified boot stages, used to sequence the implementation of vboot
 *
 * Copyright 2018 Google LLC
 */

#ifndef __CROS_STAGES_H
#define __CROS_STAGES_H

struct vboot_info;

enum vboot_stage_t {
	/*
	 * These form the 'verification' stage, where we decide which RW version
	 * to use, A or B. This runs in TPL.
	 */
	VBOOT_STAGE_FIRST = 0,
	VBOOT_STAGE_FIRST_VER = VBOOT_STAGE_FIRST,
	VBOOT_STAGE_VER_INIT = VBOOT_STAGE_FIRST,
	VBOOT_STAGE_VER1_VBINIT,
	VBOOT_STAGE_VER2_SELECTFW,
	VBOOT_STAGE_VER3_TRYFW,
	VBOOT_STAGE_VER4_LOCATEFW,
	VBOOT_STAGE_VER_FINISH,
	VBOOT_STAGE_VER_JUMP,

	/*
	 * These form the SPL stage where we set up SDRAM and jump to U-Boot
	 * proper. There are A and B versions of this, which may be different
	 * versions. There is also a read-only version of this used for
	 * recovery.
	 */
	VBOOT_STAGE_FIRST_SPL,
	VBOOT_STAGE_SPL_INIT = VBOOT_STAGE_FIRST_SPL,
	VBOOT_STAGE_SPL_JUMP_U_BOOT,

	/*
	 * This is U-Boot proper, which selects the kernel and jumps to it. It
	 * also handles recovery and developer mode. There are A and B versions
	 * of this, which may be different versions. There is also a read-only
	 * version of this used for recovery.
	 */
	VBOOT_STAGE_FIRST_RW,
	VBOOT_STAGE_RW_INIT = VBOOT_STAGE_FIRST_RW,
	VBOOT_STAGE_RW_SELECTKERNEL,
	VBOOT_STAGE_RW_BOOTKERNEL,

	/* VB2 stages, not yet implemented */
	VBOOT_STAGE_RW_KERNEL_PHASE1,
	VBOOT_STAGE_RW_KERNEL_PHASE2,
	VBOOT_STAGE_RW_KERNEL_PHASE3,
	VBOOT_STAGE_RW_KERNEL_BOOT,

	VBOOT_STAGE_COUNT,
	VBOOT_STAGE_NONE,
};

/* Flags to use for running stages */
enum vboot_stage_flag_t {
	/* drop to cmdline on error (only supported in U-Boot proper) */
	VBOOT_FLAG_CMDLINE	= 1 << 0,
};

/**
 * vboot_get_stage_name() - Get the name of a stage
 *
 * @stagenum: Stage to check
 * @return stage name, or "(unknown)" if the stage has no implementation, or
 *	"(invalid)" if an invalid stage is given
 */
const char *vboot_get_stage_name(enum vboot_stage_t stagenum);

/**
 * vboot_find_stage() - Find a stage by name
 *
 * @name: Stage name to search for
 * @return stage found, or VBOOT_STAGE_NONE if not found
 */
enum vboot_stage_t vboot_find_stage(const char *name);

/**
 * vboot_run_stage() - Run a vboot stage
 *
 * @vboot: vboot struct to use
 * @stage: stage to run
 * @return 0 if OK, VBERROR_REBOOT_REQUIRED if a reboot is needed,
 *	VB2_ERROR_API_PHASE1_RECOVERY if we should reboot into recovery, other
 *	non-zero value for any other error (meaning a reboot is needed)
 */
int vboot_run_stage(struct vboot_info *vboot, enum vboot_stage_t stage);

/**
 * vboot_run_stages() - Run vboot stages starting from a given point
 *
 * Stages are executed one after the other until a stage that jumps to the next
 * phase of U-Boot or the kernel. This function normally does not return.
 *
 * This automatically reboots in the event of an error that requires it.
 *
 * @vboot: vboot struct to use
 * @start: stage to run first
 * @flags: Flags to use (VBOOT_FLAG_...)
 * @return -EPERM if the command-line is requested, otherwise does not return
 */
int vboot_run_stages(struct vboot_info *vboot, enum vboot_stage_t start,
		     uint flags);

/**
 * vboot_run_auto() - Run verified boot automatically
 *
 * This selects the correct stage to start from, and runs through all the
 * stages from then on. The result will normally be jumping to the next phase
 * of U-Boot or the kernel.
 *
 * @return does not return in the normal case, returns -ve value on error
 */
int vboot_run_auto(struct vboot_info *vboot, uint flags);

/* TPL stages */
int vboot_ver_init(struct vboot_info *vboot);
int vboot_ver1_vbinit(struct vboot_info *vboot);
int vboot_ver2_select_fw(struct vboot_info *vboot);
int vboot_ver3_try_fw(struct vboot_info *vboot);
int vboot_ver4_locate_fw(struct vboot_info *vboot);
int vboot_ver5_finish_fw(struct vboot_info *vboot);
int vboot_ver6_jump_fw(struct vboot_info *vboot);

/* SPL stages */
int vboot_spl_init(struct vboot_info *vboot);
int vboot_spl_jump_u_boot(struct vboot_info *vboot);

/* U-Boot-proper stages */
int vboot_rw_init(struct vboot_info *vboot);
int vboot_rw_select_kernel(struct vboot_info *vboot);
int vboot_rw_boot_kernel(struct vboot_info *vboot);

/* VB2 stages, not yet implemented */
int vboot_rw_kernel_phase1(struct vboot_info *vboot);
int vboot_rw_kernel_phase2(struct vboot_info *vboot);
int vboot_rw_kernel_phase3(struct vboot_info *vboot);
int vboot_rw_kernel_boot(struct vboot_info *vboot);

#endif /* __CROS_STAGES_H */
