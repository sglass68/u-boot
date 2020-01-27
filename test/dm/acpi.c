// SPDX-License-Identifier: GPL-2.0+
/*
 * Tests for ACPI table generation
 *
 * Copyright 2019 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <dm.h>
#include <dm/acpi.h>
#include <dm/test.h>
#include <test/ut.h>

#define ACPI_TEST_DEV_NAME	"ABCD"

static int testacpi_get_name(const struct udevice *dev, char *out_name)
{
	return acpi_return_name(out_name, ACPI_TEST_DEV_NAME);
}

struct acpi_ops testacpi_ops = {
	.get_name	= testacpi_get_name,
};

static const struct udevice_id testacpi_ids[] = {
	{ .compatible = "denx,u-boot-acpi-test" },
	{ }
};

U_BOOT_DRIVER(testacpi_drv) = {
	.name	= "testacpi_drv",
	.of_match	= testacpi_ids,
	.id	= UCLASS_TEST_ACPI,
	acpi_ops_ptr(&testacpi_ops)
};

UCLASS_DRIVER(testacpi) = {
	.name		= "testacpi",
	.id		= UCLASS_TEST_ACPI,
};

/* Test ACPI get_name() */
static int dm_test_acpi_get_name(struct unit_test_state *uts)
{
	char name[ACPI_NAME_MAX];
	struct udevice *dev;

	ut_assertok(uclass_first_device_err(UCLASS_TEST_ACPI, &dev));
	ut_assertok(acpi_get_name(dev, name));
	ut_asserteq_str(ACPI_TEST_DEV_NAME, name);

	return 0;
}
DM_TEST(dm_test_acpi_get_name, DM_TESTF_SCAN_PDATA | DM_TESTF_SCAN_FDT);
