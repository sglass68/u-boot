// SPDX-License-Identifier: GPL-2.0+
/*
 * Tests for ACPI table generation
 *
 * Copyright 2019 Google LLC
 */

#include <common.h>
#include <dm.h>
#include <acpi_table.h>
#include <cpu.h>

int acpi_create_dmar(struct acpi_dmar *dmar, enum dmar_flags flags)
{
	struct acpi_table_header *header = &dmar->header;
	struct cpu_info info;
	struct udevice *cpu;
	int ret;

	ret = uclass_first_device(UCLASS_CPU, &cpu);
	if (ret)
		return log_msg_ret("cpu", ret);
	ret = cpu_get_info(cpu, &info);
	memset((void *)dmar, 0, sizeof(struct acpi_dmar));

	/* Fill out header fields. */
	acpi_fill_header(&dmar->header, "DMAR");
	header->length = sizeof(struct acpi_dmar);
	header->revision = acpi_get_table_revision(ACPITAB_DMAR);

	dmar->host_address_width = info.address_width - 1;
	dmar->flags = flags;

	return 0;
}

int acpi_get_table_revision(enum acpi_tables table)
{
	switch (table) {
	case ACPITAB_FADT:
		return ACPI_FADT_REV_ACPI_3_0;
	case ACPITAB_MADT: /* ACPI 3.0: 2, ACPI 4.0/5.0: 3, ACPI 6.2b/6.3: 5 */
		return 2;
	case ACPITAB_MCFG:
		return 1;
	case ACPITAB_TCPA:
		return 2;
	case ACPITAB_TPM2:
		return 4;
	case ACPITAB_SSDT: /* ACPI 3.0 upto 6.3: 2 */
		return 2;
	case ACPITAB_SRAT: /* ACPI 2.0: 1, ACPI 3.0: 2, ACPI 4.0 upto 6.3: 3 */
		return 1; /* TODO Should probably be upgraded to 2 */
	case ACPITAB_DMAR:
		return 1;
	case ACPITAB_SLIT: /* ACPI 2.0 upto 6.3: 1 */
		return 1;
	case ACPITAB_SPMI: /* IMPI 2.0 */
		return 5;
	case ACPITAB_HPET: /* Currently 1. Table added in ACPI 2.0. */
		return 1;
	case ACPITAB_VFCT: /* ACPI 2.0/3.0/4.0: 1 */
		return 1;
	case ACPITAB_IVRS:
		return IVRS_FORMAT_FIXED;
	case ACPITAB_DBG2:
		return 0;
	case ACPITAB_FACS: /* ACPI 2.0/3.0: 1, ACPI 4.0 upto 6.3: 2 */
		return 1;
	case ACPITAB_RSDT: /* ACPI 1.0 upto 6.3: 1 */
		return 1;
	case ACPITAB_XSDT: /* ACPI 2.0 upto 6.3: 1 */
		return 1;
	case ACPITAB_RSDP: /* ACPI 2.0 upto 6.3: 2 */
		return 2;
	case ACPITAB_HEST:
		return 1;
	case ACPITAB_NHLT:
		return 5;
	case ACPITAB_BERT:
		return 1;
	default:
		return -EINVAL;
	}
}
