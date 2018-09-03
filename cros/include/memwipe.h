/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Allows setting and excluding memory regions that need to be cleared.
 *
 * The following methods must be called in order:
 *   memwipe_init()
 *   memwipe_add()
 *   memwipe_sub()
 *   memwipe_execute()
 *
 * Copyright 2018 Google LLC
 */

#ifndef __CROS_memwipe_H
#define __CROS_memwipe_H

/* The margin to keep extra stack region that not to be wiped */
#define memwipe_STACK_MARGIN		1024

/* A node in a linked list of edges, each at position "pos" */
struct memwipe_edge {
	struct memwipe_edge *next;
	phys_addr_t pos;
};

/*
 * Data describing memory to wipe. Contains a linked list of edges between the
 * regions of memory to wipe and not wipe.
 */
struct memwipe {
	struct memwipe_edge head;
};

/*
 * Initialises the memory region that needs to be cleared
 *
 * @wipe:	Wipe structure to initialise
 */
void memwipe_init(struct memwipe *wipe);

/*
 * Adds a memory region to be cleared
 *
 * @wipe:	Wipe structure to add the region to
 * @start:	The start of the region
 * @end:	The end of the region
 */
void memwipe_add(struct memwipe *wipe, phys_addr_t start, phys_addr_t end);

/*
 * Subtracts a memory region from the area to be wiped
 *
 * @wipe:	Wipe structure to subtract the region from
 * @start:	The start of the region
 * @end:	The end of the region
 */
void memwipe_sub(struct memwipe *wipe, phys_addr_t start, phys_addr_t end);

/*
 * Executes the memory wipe
 *
 * @wipe:	Wipe structure to execute
 */
void memwipe_execute(struct memwipe *wipe);

#endif /* __CROS_memwipe_H */
