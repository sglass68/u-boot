// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Google LLC
 */

#include <common.h>
#include <cpu.h>
#include <dm.h>
#include <asm/cpu_common.h>
#include <asm/cpu_x86.h>
#include <asm/intel_acpi.h>
#include <asm/msr.h>
#include <dm/acpi.h>

static int apl_get_info(const struct udevice *dev, struct cpu_info *info)
{
	return cpu_intel_get_info(info, INTEL_BCLK_MHZ);
}

static int apl_get_count(const struct udevice *dev)
{
	return 4;
}

static int acpi_cpu_fill_ssdt(const struct udevice *dev, struct acpi_ctx *ctx)
{
	struct cpu_platdata *plat = dev_get_parent_platdata(dev);
	int ret;

	/* Trigger off the first CPU */
	if (!plat->cpu_id) {
		ret = generate_cpu_entries(ctx, dev);
		if (ret)
			return log_msg_ret("generate", ret);
	}

	return 0;
}

struct acpi_ops apl_cpu_acpi_ops = {
	.fill_ssdt	= acpi_cpu_fill_ssdt,
};

static const struct cpu_ops cpu_x86_apl_ops = {
	.get_desc	= cpu_x86_get_desc,
	.get_info	= apl_get_info,
	.get_count	= apl_get_count,
	.get_vendor	= cpu_x86_get_vendor,
};

static const struct udevice_id cpu_x86_apl_ids[] = {
	{ .compatible = "intel,apl-cpu" },
	{ }
};

U_BOOT_DRIVER(cpu_x86_apl_drv) = {
	.name		= "cpu_x86_apl",
	.id		= UCLASS_CPU,
	.of_match	= cpu_x86_apl_ids,
	.bind		= cpu_x86_bind,
	.ops		= &cpu_x86_apl_ops,
	acpi_ops_ptr(&apl_cpu_acpi_ops)
	.flags		= DM_FLAG_PRE_RELOC,
};
