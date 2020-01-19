/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018, Bin Meng <bmeng.cn@gmail.com>
 */

#ifndef __ASM_INTEL_ACPI_H__
#define __ASM_INTEL_ACPI_H__

struct acpi_ctx;
struct udevice;

int generate_cpu_entries(struct acpi_ctx *ctx, const struct udevice *device);
void soc_power_states_generation(struct acpi_ctx *ctx,
				 int core_id, int cores_per_package);

void generate_p_state_entries(struct acpi_ctx *ctx, int core,
			      int cores_per_package);
void generate_t_state_entries(struct acpi_ctx *ctx, int core,
			      int cores_per_package);
int southbridge_inject_dsdt(const struct udevice *dev, struct acpi_ctx *ctx);

#endif /* __ASM_INTEL_ACPI_H__ */
