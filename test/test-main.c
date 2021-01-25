// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <console.h>
#include <test/test.h>

int ut_run_tests(struct unit_test_state *uts, const char *prefix,
		 struct unit_test *tests, int count, const char *test_name)
{
	struct unit_test *test;
	int prefix_len = prefix ? strlen(prefix) : 0;
	int found = 0;

	for (test = tests; test < tests + count; test++) {
		const char *name = test->name;

		/* Remove the prefix */
		if (prefix && !strncmp(test_name, prefix, prefix_len))
			name += prefix_len;

		if (test_name && strcmp(test_name, name))
			continue;
		printf("Test: %s\n", name);
		found++;

		if (test->flags & UT_TESTF_CONSOLE_REC) {
			int ret = console_record_reset_enable();

			if (ret) {
				printf("Skipping: Console recording disabled\n");
				continue;
			}
		}

		uts->start = mallinfo();

		test->func(uts);
	}
	if (test_name && !found)
		return -ENOENT;

	return uts->fail_count ? -EBADF : 0;
}

int ut_run_list(const char *category, const char *prefix,
		struct unit_test *tests, int count, const char *test_name)
{
	struct unit_test_state uts = { .fail_count = 0 };
	int ret;

	if (!test_name)
		printf("Running %d %s tests\n", count, category);

	ret = ut_run_tests(&uts, prefix, tests, count, test_name);

	if (ret == -ENOENT)
		printf("Test '%s' not found\n", test_name);
	else
		printf("Failures: %d\n", uts.fail_count);

	return ret;
}
