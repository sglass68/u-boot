// SPDX-License-Identifier: GPL-2.0+
/*
 * Tests for bootm routines
 *
 * Copyright 2020 Google LLC
 */

#include <common.h>
#include <bootm.h>
#include <test/suites.h>
#include <test/test.h>
#include <test/ut.h>

DECLARE_GLOBAL_DATA_PTR;

#define BOOTM_TEST(_name, _flags)	UNIT_TEST(_name, _flags, bootm_test)

enum {
	BUF_SIZE	= 1024,
};

#define CONSOLE_STR	"console=/dev/ttyS0"

/* Test cmdline processing where nothing happens */
static int bootm_test_nop(struct unit_test_state *uts)
{
	char buf[BUF_SIZE];

	*buf = '\0';
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, true));
	ut_asserteq_str("", buf);

	strcpy(buf, "test");
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, true));
	ut_asserteq_str("test", buf);

	return 0;
}
BOOTM_TEST(bootm_test_nop, 0);

/* Test cmdline processing when out of space */
static int bootm_test_nospace(struct unit_test_state *uts)
{
	char buf[BUF_SIZE];

	/* Zero buffer size */
	*buf = '\0';
	ut_asserteq(-ENOSPC, bootm_process_cmdline(buf, 0, true));

	/* Buffer string not terminated */
	memset(buf, 'a', BUF_SIZE);
	ut_asserteq(-ENOSPC, bootm_process_cmdline(buf, BUF_SIZE, true));

	/* Not enough space to copy string */
	memset(buf, '\0', BUF_SIZE);
	memset(buf, 'a', BUF_SIZE / 2);
	ut_asserteq(-ENOSPC, bootm_process_cmdline(buf, BUF_SIZE, true));

	/* Just enough space */
	memset(buf, '\0', BUF_SIZE);
	memset(buf, 'a', BUF_SIZE / 2 - 1);
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, true));

	return 0;
}
BOOTM_TEST(bootm_test_nospace, 0);

/* Test silent processing */
static int bootm_test_silent(struct unit_test_state *uts)
{
	char buf[BUF_SIZE];

	/* 'silent_linux' not set should do nothing */
	env_set("silent_linux", NULL);
	strcpy(buf, CONSOLE_STR);
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SILENT));
	ut_asserteq_str(CONSOLE_STR, buf);

	ut_assertok(env_set("silent_linux", "no"));
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SILENT));
	ut_asserteq_str(CONSOLE_STR, buf);

	ut_assertok(env_set("silent_linux", "yes"));
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SILENT));
	ut_asserteq_str("console=", buf);

	/* Empty buffer should still add the string */
	*buf = '\0';
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SILENT));
	ut_asserteq_str("console=", buf);

	/* Check nothing happens when do_silent is false */
	*buf = '\0';
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, 0));
	ut_asserteq_str("", buf);

	/* Not enough space */
	*buf = '\0';
	ut_asserteq(-ENOSPC, bootm_process_cmdline(buf, 8, BOOTM_CL_SILENT));

	/* Just enough space */
	*buf = '\0';
	ut_assertok(bootm_process_cmdline(buf, 9, BOOTM_CL_SILENT));

	/* add at end */
	strcpy(buf, "something");
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SILENT));
	ut_asserteq_str("something console=", buf);

	/* change at start */
	strcpy(buf, CONSOLE_STR " something");
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SILENT));
	ut_asserteq_str("console= something", buf);

	return 0;
}
BOOTM_TEST(bootm_test_silent, 0);

/* Test substitution processing */
static int bootm_test_subst(struct unit_test_state *uts)
{
	char buf[BUF_SIZE];

	strcpy(buf, "some%Athing");
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SUBST));
	ut_asserteq_str("some%Athing", buf);

	/* Replace with shorter string */
	ut_assertok(env_set("bootargs_A", "a"));
	strcpy(buf, "some%Athing");
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SUBST));
	ut_asserteq_str("someathing", buf);

	/* Replace with same-length string */
	ut_assertok(env_set("bootargs_A", "ab"));
	strcpy(buf, "some%Athing");
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SUBST));
	ut_asserteq_str("someabthing", buf);

	/* Replace with longer string */
	ut_assertok(env_set("bootargs_A", "abc"));
	strcpy(buf, "some%Athing");
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SUBST));
	ut_asserteq_str("someabcthing", buf);

	/* Check it is case sensitive */
	strcpy(buf, "some%athing");
	ut_assertok(bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SUBST));
	ut_asserteq_str("some%athing", buf);

	/* Check too long - need 12 bytes for each string */
	strcpy(buf, "some%Athing");
	ut_asserteq(-ENOSPC,
		    bootm_process_cmdline(buf, 12 * 2 - 1, BOOTM_CL_SUBST));

	/* Check just enough space */
	strcpy(buf, "some%Athing");
	ut_assertok(bootm_process_cmdline(buf, 12 * 2, BOOTM_CL_SUBST));
	ut_asserteq_str("someabcthing", buf);

	/*
	 * Check the substition string being too long. This results in a string
	 * of 12 (13 bytes). Allow one more byte margin.
	 */
	ut_assertok(env_set("bootargs_A", "1234567890"));
	strcpy(buf, "a%Ac");
	ut_asserteq(-ENOSPC,
		    bootm_process_cmdline(buf, 13, BOOTM_CL_SUBST));

	strcpy(buf, "a%Ac");
	ut_asserteq(0, bootm_process_cmdline(buf, 14, BOOTM_CL_SUBST));

	/* Check multiple substitutions */
	ut_assertok(env_set("bootargs_A", "abc"));
	strcpy(buf, "some%Athing%Belse");
	ut_asserteq(0, bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SUBST));
	ut_asserteq_str("someabcthing%Belse", buf);

	/* Check multiple substitutions */
	ut_assertok(env_set("bootargs_B", "123"));
	strcpy(buf, "some%Athing%Belse");
	ut_asserteq(0, bootm_process_cmdline(buf, BUF_SIZE, BOOTM_CL_SUBST));
	ut_asserteq_str("someabcthing123else", buf);

	return 0;
}
BOOTM_TEST(bootm_test_subst, 0);

/* Test silent processing in the bootargs variable */
static int bootm_test_silent_var(struct unit_test_state *uts)
{
	env_set("bootargs", NULL);
	ut_assertok(bootm_process_cmdline_env(BOOTM_CL_SUBST));
	ut_assertnull(env_get("bootargs"));

	ut_assertok(env_set("bootargs", "some%Athing"));
	ut_assertok(bootm_process_cmdline_env(BOOTM_CL_SUBST));
	ut_asserteq_str("some%Athing", env_get("bootargs"));

	return 0;
}
BOOTM_TEST(bootm_test_silent_var, 0);

/* Test substitution processing in the bootargs variable */
static int bootm_test_subst_var(struct unit_test_state *uts)
{
	env_set("bootargs", NULL);
	ut_assertok(bootm_process_cmdline_env(BOOTM_CL_SILENT));
	ut_asserteq_str("console=", env_get("bootargs"));

	ut_assertok(env_set("bootargs", "some%Athing"));
	ut_assertok(bootm_process_cmdline_env(BOOTM_CL_SILENT));
	ut_asserteq_str("some%Athing console=", env_get("bootargs"));

	return 0;
}
BOOTM_TEST(bootm_test_subst_var, 0);

/* Test substitution and silent console processing in the bootargs variable */
static int bootm_test_subst_both(struct unit_test_state *uts)
{
	ut_assertok(env_set("silent_linux", "yes"));
	env_set("bootargs", NULL);
	ut_assertok(bootm_process_cmdline_env(BOOTM_CL_ALL));
	ut_asserteq_str("console=", env_get("bootargs"));

	ut_assertok(env_set("bootargs", "some%Athing " CONSOLE_STR));
	ut_assertok(env_set("bootargs_A", "1234567890"));
	ut_assertok(bootm_process_cmdline_env(BOOTM_CL_ALL));
	ut_asserteq_str("some1234567890thing console=", env_get("bootargs"));

	return 0;
}
BOOTM_TEST(bootm_test_subst_both, 0);

int do_ut_bootm(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	struct unit_test *tests = ll_entry_start(struct unit_test, bootm_test);
	const int n_ents = ll_entry_count(struct unit_test, bootm_test);

	return cmd_ut_category("bootm", "bootm_test_", tests, n_ents,
			       argc, argv);
}
