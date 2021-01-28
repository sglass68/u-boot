// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2013 Google, Inc
 */

#include <common.h>
#include <command.h>
#include <console.h>
#include <dm.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <asm/state.h>
#include <dm/root.h>
#include <dm/uclass-internal.h>
#include <test/test.h>
#include <test/test.h>
#include <test/ut.h>

DECLARE_GLOBAL_DATA_PTR;

struct unit_test_state global_dm_test_state;

/**
 * dm_test_run_on_flattree() - Check if we should run a test with flat DT
 *
 * This skips long/slow tests where there is not much value in running a flat
 * DT test in addition to a live DT test.
 *
 * @return true to run the given test on the flat device tree
 */
static bool dm_test_run_on_flattree(struct unit_test *test)
{
	const char *fname = strrchr(test->file, '/') + 1;

	return !strstr(fname, "video") || strstr(test->name, "video_base");
}

static bool test_matches(const char *test_name, const char *find_name)
{
	if (!find_name)
		return true;

	if (!strcmp(test_name, find_name))
		return true;

	/* All tests have this prefix */
	if (!strncmp(test_name, "dm_test_", 8))
		test_name += 8;

	if (!strcmp(test_name, find_name))
		return true;

	return false;
}

int dm_test_run(const char *test_name)
{
	struct unit_test *tests = ll_entry_start(struct unit_test, dm_test);
	const int n_ents = ll_entry_count(struct unit_test, dm_test);
	struct unit_test_state *uts = &global_dm_test_state;
	struct unit_test *test;
	int found;

	uts->fail_count = 0;

	if (!CONFIG_IS_ENABLED(OF_PLATDATA)) {
		/*
		 * If we have no device tree, or it only has a root node, then
		 * these * tests clearly aren't going to work...
		 */
		if (!gd->fdt_blob || fdt_next_node(gd->fdt_blob, 0, NULL) < 0) {
			puts("Please run with test device tree:\n"
			     "    ./u-boot -d arch/sandbox/dts/test.dtb\n");
			ut_assert(gd->fdt_blob);
		}
	}

	if (!test_name)
		printf("Running %d driver model tests\n", n_ents);
	else

	found = 0;
	uts->of_root = gd_of_root();
	for (test = tests; test < tests + n_ents; test++) {
		const char *name = test->name;
		int runs;

		if (!test_matches(name, test_name))
			continue;

		/* Run with the live tree if possible */
		runs = 0;
		if (CONFIG_IS_ENABLED(OF_LIVE)) {
			if (!(test->flags & UT_TESTF_FLAT_TREE)) {
				uts->of_live = true;
				ut_assertok(ut_run_test(uts, test, test->name));
				runs++;
			}
		}

		/*
		 * Run with the flat tree if we couldn't run it with live tree,
		 * or it is a core test.
		 */
		if (!(test->flags & UT_TESTF_LIVE_TREE) &&
		    (!runs || dm_test_run_on_flattree(test))) {
			uts->of_live = false;
			ut_assertok(ut_run_test(uts, test, test->name));
			runs++;
		}
		found++;
	}

	if (test_name && !found)
		printf("Test '%s' not found\n", test_name);
	else
		printf("Failures: %d\n", uts->fail_count);

	/* Put everything back to normal so that sandbox works as expected */
	gd_set_of_root(uts->of_root);
	gd->dm_root = NULL;
	ut_assertok(dm_init(CONFIG_IS_ENABLED(OF_LIVE)));
	dm_scan_plat(false);
	if (!CONFIG_IS_ENABLED(OF_PLATDATA))
		dm_scan_fdt(false);

	return uts->fail_count ? CMD_RET_FAILURE : 0;
}

int do_ut_dm(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	const char *test_name = NULL;

	if (argc > 1)
		test_name = argv[1];

	return dm_test_run(test_name);
}
