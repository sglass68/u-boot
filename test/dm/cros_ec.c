// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2021 Google LLC
 */

#include <common.h>
#include <cros_ec.h>
#include <dm.h>
#include <asm/test.h>
#include <dm/test.h>
#include <test/ut.h>

static int dm_test_cros_ec_hello(struct unit_test_state *uts)
{
	struct udevice *dev;
	uint val;

	ut_assertok(uclass_first_device_err(UCLASS_CROS_EC, &dev));

	ut_assertok(cros_ec_hello(dev, NULL));

	val = 0xdead1357;
	ut_assertok(cros_ec_hello(dev, &val));
	ut_asserteq(0xdead1357, val);

	sandbox_cros_ec_set_test_flags(dev, CROSECT_BREAK_HELLO);
	ut_asserteq(-ENOTSYNC, cros_ec_hello(dev, &val));
	ut_asserteq(0x12345678, val);

	return 0;
}
DM_TEST(dm_test_cros_ec_hello, UT_TESTF_SCAN_FDT);
