// SPDX-License-Identifier: GPL-2.0+
/*
 * Allows setting and excluding memory regions that need to be cleared.
 *
 * Copyright 2018 Google LLC
 */

#define LOG_CATEGORY LOGC_VBOOT

#include <common.h>
#include <inttypes.h>
#include <malloc.h>
#include <physmem.h>
#include <cros/cros_common.h>
#include <cros/memwipe.h>

/*
 * This implementation tracks regions of memory that need to be wiped by
 * filling them with zeroes. It does that by keeping a linked list of the
 * edges between regions where memory should be wiped and not wiped. New
 * regions take precedence over older regions they overlap with. With
 * increasing addresses, the regions of memory alternate between needing to be
 * wiped and needing to be left alone. Edges similarly alternate between
 * starting a wipe region and starting a not-wiped region.
 */

static void memwipe_insert_between(struct memwipe_edge *before,
				   struct memwipe_edge *after, phys_addr_t pos)
{
	struct memwipe_edge *new_edge = malloc(sizeof(*new_edge));

	assert(new_edge);
	assert(before != after);

	new_edge->next = after;
	new_edge->pos = pos;
	before->next = new_edge;
}

void memwipe_init(struct memwipe *wipe)
{
	wipe->head.next = NULL;
	wipe->head.pos = 0;
}

static void memwipe_set_region_to(struct memwipe *wipe_info, phys_addr_t start,
				  phys_addr_t end, int new_wiped)
{
	/* whether the current region was originally going to be wiped */
	int wipe = 0;

	assert(start != end);

	/* prev is never NULL, but cur might be */
	struct memwipe_edge *prev = &wipe_info->head;
	struct memwipe_edge *cur = prev->next;

	/*
	 * Find the start of the new region. After this loop, prev will be
	 * before the start of the new region, and cur will be after it or
	 * overlapping start. If they overlap, this ensures that the existing
	 * edge is deleted and we don't end up with two edges in the same spot.
	 */
	while (cur && cur->pos < start) {
		prev = cur;
		cur = cur->next;
		wipe = !wipe;
	}

	/* Add the 'start' edge between prev and cur, if needed */
	if (new_wiped != wipe) {
		memwipe_insert_between(prev, cur, start);
		prev = prev->next;
	}

	/*
	 * Delete any edges obscured by the new region. After this loop, prev
	 * will be before the end of the new region or overlapping it, and cur
	 * will be after if, if there is a edge after it. For the same
	 * reason as above, we want to ensure that we end up with one edge if
	 * there's an overlap.
	 */
	while (cur && cur->pos <= end) {
		cur = cur->next;
		free(prev->next);
		prev->next = cur;
		wipe = !wipe;
	}

	/* Add the 'end' edge between prev and cur, if needed */
	if (wipe != new_wiped)
		memwipe_insert_between(prev, cur, end);
}

/* Set a region to 'wiped' */
void memwipe_add(struct memwipe *wipe, phys_addr_t start, phys_addr_t end)
{
	log_debug("start=%" PRIx64 ", end=%" PRIx64 "\n", (u64)start,
		  (u64)end);
	memwipe_set_region_to(wipe, start, end, 1);
}

/* Set a region to 'not wiped' */
void memwipe_sub(struct memwipe *wipe, phys_addr_t start, phys_addr_t end)
{
	log_debug("start=%" PRIx64 ", end=%" PRIx64 "\n", (u64)start, (u64)end);
	memwipe_set_region_to(wipe, start, end, 0);
}

/* Actually wipe memory */
void memwipe_execute(struct memwipe *wipe)
{
	struct memwipe_edge *cur;

	log_debug("Wipe memory regions:\n");
	for (cur = wipe->head.next; cur; cur = cur->next->next) {
		phys_addr_t start, end;

		if (!cur->next) {
			log_debug("Odd number of region edges!\n");
			return;
		}

		start = cur->pos;
		end = cur->next->pos;

		log_debug("\t[%#016llx, %#016llx)\n",
			  (unsigned long long)start, (unsigned long long)end);
		arch_phys_memset(start, 0, end - start);
	}
}
