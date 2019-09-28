/* SPDX-License-Identifier: Intel */
/*
 * Copyright (C) 2015-2016 Intel Corp.
 * (Written by Andrey Petrov <andrey.petrov@intel.com> for Intel Corp.)
 * (Written by Alexandru Gagniuc <alexandrux.gagniuc@intel.com> for Intel Corp.)
 * Mostly taken from coreboot fsp2_0/memory_init.c
 */

#ifndef __ASM_FSP_API_H
#define __ASM_FSP_API_H

enum fsp_phase {
	/* Notification code for post PCI enuermation */
	INIT_PHASE_PCI	= 0x20,
	/* Notification code before transferring control to the payload */
	INIT_PHASE_BOOT	= 0x40
};

struct fsp_notify_params {
	/* Notification phase used for NotifyPhase API */
	enum fsp_phase	phase;
};

/* FspNotify API function prototype */
typedef asmlinkage u32 (*fsp_notify_f)(struct fsp_notify_params *params);

#endif
