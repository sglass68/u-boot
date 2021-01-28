// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <console.h>
#include <dm.h>
#include <asm/state.h>
#include <dm/root.h>
#include <dm/test.h>
#include <dm/uclass-internal.h>
#include <test/test.h>
#include <test/ut.h>

DECLARE_GLOBAL_DATA_PTR;

static struct unit_test_state *cur_test_state;

struct unit_test_state *test_get_state(void)
{
	return cur_test_state;
}

/**
 * dm_test_pre_run() - Get ready to run a driver model test
 *
 * This clears out the driver model data structures. For sandbox it resets the
 * state structure
 *
 * @uts: Test state
 */
static int dm_test_pre_run(struct unit_test_state *uts)
{
	bool of_live = uts->of_live;

	uts->root = NULL;
	uts->testdev = NULL;
	uts->force_fail_alloc = false;
	uts->skip_post_probe = false;
	gd->dm_root = NULL;
	if (!CONFIG_IS_ENABLED(OF_PLATDATA))
		memset(dm_testdrv_op_count, '\0', sizeof(dm_testdrv_op_count));
	state_reset_for_test(state_get_current());

	/* Determine whether to make the live tree available */
	gd_set_of_root(of_live ? uts->of_root : NULL);
	ut_assertok(dm_init(of_live));
	uts->root = dm_root();

	return 0;
}

static int dm_test_post_run(struct unit_test_state *uts)
{
	int id;

	for (id = 0; id < UCLASS_COUNT; id++) {
		struct uclass *uc;

		/*
		 * If the uclass doesn't exist we don't want to create it. So
		 * check that here before we call uclass_find_device().
		 */
		uc = uclass_find(id);
		if (!uc)
			continue;
		ut_assertok(uclass_destroy(uc));
	}

	return 0;
}

/* Ensure all the test devices are probed */
static int do_autoprobe(struct unit_test_state *uts)
{
	struct udevice *dev;
	int ret;

	/* Scanning the uclass is enough to probe all the devices */
	for (ret = uclass_first_device(UCLASS_TEST, &dev);
	     dev;
	     ret = uclass_next_device(&dev))
		;

	return ret;
}

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

static bool test_matches(const char *prefix, const char *test_name,
			 const char *select_name)
{
	if (!select_name)
		return true;

	if (!strcmp(test_name, select_name))
		return true;

	/* All tests have this prefix */
	if (!strncmp(test_name, prefix, strlen(prefix)))
		test_name += strlen(prefix);

	if (!strcmp(test_name, select_name))
		return true;

	return false;
}

int test_pre_run(struct unit_test_state *uts, struct unit_test *test)
{
	if (test->flags & UT_TESTF_DM)
		ut_assertok(dm_test_pre_run(uts));

	ut_set_skip_delays(uts, false);

	uts->start = mallinfo();

	if (test->flags & UT_TESTF_SCAN_PDATA)
		ut_assertok(dm_scan_plat(false));

	if (test->flags & UT_TESTF_PROBE_TEST)
		ut_assertok(do_autoprobe(uts));

	if (!CONFIG_IS_ENABLED(OF_PLATDATA) &&
	    (test->flags & UT_TESTF_SCAN_FDT))
		ut_assertok(dm_extended_scan(false));

	if (test->flags & UT_TESTF_CONSOLE_REC) {
		int ret = console_record_reset_enable();

		if (ret) {
			printf("Skipping: Console recording disabled\n");
			return -EAGAIN;
		}
	}
	ut_silence_console(uts);

	return 0;
}

int test_post_run(struct unit_test_state *uts, struct unit_test *test)
{
	ut_unsilence_console(uts);
	if (test->flags & UT_TESTF_DM)
		ut_assertok(dm_test_post_run(uts));

	return 0;
}

int ut_run_test(struct unit_test_state *uts, struct unit_test *test,
		const char *test_name)
{
	const char *fname = strrchr(test->file, '/') + 1;
	const char *note = "";
	int ret;

	if ((test->flags & UT_TESTF_DM) && !uts->of_live)
		note = " (flat tree)";
	printf("Test: %s: %s%s\n", test_name, fname, note);

	/* Allow access to test state from drivers */
	cur_test_state = uts;

	ret = test_pre_run(uts, test);
	if (ret == -EAGAIN)
		return -EAGAIN;
	if (ret)
		return ret;

	test->func(uts);

	ret = test_post_run(uts, test);
	if (ret)
		return ret;

	cur_test_state = NULL;

	return 0;
}

int ut_run_test_live_flat(struct unit_test_state *uts, struct unit_test *test,
			  const char *name)
{
	int runs;

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
	if (test->flags & UT_TESTF_DM) {
		if (!(test->flags & UT_TESTF_LIVE_TREE) &&
		    (!runs || dm_test_run_on_flattree(test))) {
			uts->of_live = false;
			ut_assertok(ut_run_test(uts, test, test->name));
			runs++;
		}
	}

	return 0;
}

int ut_run_tests(struct unit_test_state *uts, const char *prefix,
		 struct unit_test *tests, int count, const char *select_name)
{
	struct unit_test *test;
	int found = 0;

	for (test = tests; test < tests + count; test++) {
		const char *test_name = test->name;
		int ret;

		if (!test_matches(prefix, test_name, select_name))
			continue;
		ret = ut_run_test_live_flat(uts, test, select_name);
		found++;
		if (ret == -EAGAIN)
			continue;
		if (ret)
			return ret;
	}
	if (select_name && !found)
		return -ENOENT;

	return uts->fail_count ? -EBADF : 0;
}

int ut_run_list(const char *category, const char *prefix,
		struct unit_test *tests, int count, const char *select_name)
{
	struct unit_test_state uts = { .fail_count = 0 };
	int ret;

	if (!select_name)
		printf("Running %d %s tests\n", count, category);

	uts.of_root = gd_of_root();
	ret = ut_run_tests(&uts, prefix, tests, count, select_name);

	if (ret == -ENOENT)
		printf("Test '%s' not found\n", select_name);
	else
		printf("Failures: %d\n", uts.fail_count);

	return ret;
}
