/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2017 Intel Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <acpi.h>
#include <cpu.h>
#include <dm.h>
#include <asm/acpi_table.h>
#include <asm/acpigen.h>
#include <asm/cpu.h>
#include <asm/cpu_common.h>
#include <asm/generic_wifi.h>
#include <asm/intel_acpi.h>
#include <asm/ioapic.h>
#include <asm/mpspec.h>
#include <asm/smm.h>
#include <asm/turbo.h>
#include <asm/arch/iomap.h>
#include <asm/arch/pm.h>
#include <asm/arch/systemagent.h>
#include <power/acpi_pmc.h>
/*
#include <arch/acpigen.h>
#include <arch/ioapic.h>
#include <arch/smp/mpspec.h>
#include <bootstate.h>
#include <cbmem.h>
#include <cf9_reset.h>
#include <console/console.h>
#include <cpu/intel/turbo.h>
#include <cpu/x86/msr.h>
#include <cpu/x86/smm.h>
#include <intelblocks/acpi.h>
#include <intelblocks/msr.h>
#include <intelblocks/pmclib.h>
#include <intelblocks/uart.h>
#include <soc/gpio.h>
#include <soc/iomap.h>
#include <soc/nvs.h>
#include <soc/pm.h>
#include <string.h>
*/

__weak ulong acpi_fill_mcfg(ulong current)
{
	/* PCI Segment Group 0, Start Bus Number 0, End Bus Number is 255 */
	current += acpi_create_mcfg_mmconfig((void *)current,
					     CONFIG_MMCONF_BASE_ADDRESS, 0, 0,
					     (CONFIG_SA_PCIEX_LENGTH >> 20) - 1);
	return current;
}

static int acpi_sci_irq(void)
{
	int sci_irq = 9;
	uint scis;
	int ret;

	ret = soc_read_sci_irq_select();
	if (IS_ERR_VALUE(ret))
		return log_msg_ret("sci_irq", ret);
	scis = ret;
	scis &= SCI_IRQ_SEL;
	scis >>= SCI_IRQ_ADJUST;

	/* Determine how SCI is routed. */
	switch (scis) {
	case SCIS_IRQ9:
	case SCIS_IRQ10:
	case SCIS_IRQ11:
		sci_irq = scis - SCIS_IRQ9 + 9;
		break;
	case SCIS_IRQ20:
	case SCIS_IRQ21:
	case SCIS_IRQ22:
	case SCIS_IRQ23:
		sci_irq = scis - SCIS_IRQ20 + 20;
		break;
	default:
		log_warning("Invalid SCI route! Defaulting to IRQ9\n");
		sci_irq = 9;
		break;
	}

	log_debug("SCI is IRQ%d\n", sci_irq);

	return sci_irq;
}

static unsigned long acpi_madt_irq_overrides(unsigned long current)
{
	int sci = acpi_sci_irq();
	uint16_t flags = MP_IRQ_TRIGGER_LEVEL;

	if (sci < 0)
		return log_msg_ret("sci irq", sci);

	/* INT_SRC_OVR */
	current += acpi_create_madt_irqoverride((void *)current, 0, 0, 2, 0);

	flags |= soc_madt_sci_irq_polarity(sci);

	/* SCI */
	current +=
	    acpi_create_madt_irqoverride((void *)current, 0, sci, sci, flags);

	return current;
}

unsigned long acpi_fill_madt(unsigned long current)
{
	/* Local APICs */
	current += acpi_create_madt_lapics(current);

	/* IOAPIC */
	current += acpi_create_madt_ioapic((void *)current, 2, IO_APIC_ADDR, 0);

	return acpi_madt_irq_overrides(current);
}

__weak void soc_fill_fadt(struct acpi_fadt *fadt)
{
}

void intel_acpi_fill_fadt(struct acpi_fadt *fadt)
{
	const uint16_t pmbase = IOMAP_ACPI_BASE;

	/* Use ACPI 3.0 revision. */
	fadt->header.revision = get_acpi_table_revision(FADT);

	fadt->sci_int = acpi_sci_irq();
	fadt->smi_cmd = APM_CNT;
	fadt->acpi_enable = APM_CNT_ACPI_ENABLE;
	fadt->acpi_disable = APM_CNT_ACPI_DISABLE;
	fadt->s4bios_req = 0x0;
	fadt->pstate_cnt = 0;

	fadt->pm1a_evt_blk = pmbase + PM1_STS;
	fadt->pm1b_evt_blk = 0x0;
	fadt->pm1a_cnt_blk = pmbase + PM1_CNT;
	fadt->pm1b_cnt_blk = 0x0;

	fadt->gpe0_blk = pmbase + GPE0_STS;

	fadt->pm1_evt_len = 4;
	fadt->pm1_cnt_len = 2;

	/* GPE0 STS/EN pairs each 32 bits wide. */
	fadt->gpe0_blk_len = 2 * GPE0_REG_MAX * sizeof(uint32_t);

	fadt->flush_size = 0x400;	/* twice of cache size */
	fadt->flush_stride = 0x10;	/* Cache line width  */
	fadt->duty_offset = 1;
	fadt->day_alrm = 0xd;

	fadt->flags = ACPI_FADT_WBINVD | ACPI_FADT_C1_SUPPORTED |
	    ACPI_FADT_C2_MP_SUPPORTED | ACPI_FADT_SLEEP_BUTTON |
	    ACPI_FADT_RESET_REGISTER | ACPI_FADT_SEALED_CASE |
	    ACPI_FADT_S4_RTC_WAKE | ACPI_FADT_PLATFORM_CLOCK;

	fadt->reset_reg.space_id = 1;
	fadt->reset_reg.bit_width = 8;
	fadt->reset_reg.addrl = IO_PORT_RESET;
	fadt->reset_value = RST_CPU | SYS_RST;

	fadt->x_pm1a_evt_blk.space_id = 1;
	fadt->x_pm1a_evt_blk.bit_width = fadt->pm1_evt_len * 8;
	fadt->x_pm1a_evt_blk.addrl = pmbase + PM1_STS;

	fadt->x_pm1b_evt_blk.space_id = 1;

	fadt->x_pm1a_cnt_blk.space_id = 1;
	fadt->x_pm1a_cnt_blk.bit_width = fadt->pm1_cnt_len * 8;
	fadt->x_pm1a_cnt_blk.addrl = pmbase + PM1_CNT;

	fadt->x_pm1b_cnt_blk.space_id = 1;

	fadt->x_gpe1_blk.space_id = 1;

}

int intel_southbridge_write_acpi_tables(struct udevice *dev,
					struct acpi_ctx *ctx)
{
	ctx->current = acpi_write_dbg2_pci_uart(ctx->rsdp, ctx->current,
						gd->cur_serial_dev,
						ACPI_ACCESS_SIZE_DWORD_ACCESS);

	ctx->current = acpi_write_hpet(dev, ctx->current, ctx->rsdp);

	return 0;
}

__weak uint32_t acpi_fill_soc_wake(uint32_t generic_pm1_en,
				   const struct chipset_power_state *ps)
{
	return generic_pm1_en;
}

#if IS_ENABLED(CONFIG_INTEL_ACPI_WAKE_SOURCE)
/*
 * Save wake source information for calculating ACPI _SWS values
 *
 * @pm1:  PM1_STS register with only enabled events set
 * @gpe0: GPE0_STS registers with only enabled events set
 *
 * return the number of registers in the gpe0 array or -1 if nothing
 * is provided by this function.
 */

static int acpi_fill_wake(uint32_t *pm1, uint32_t **gpe0)
{
	struct chipset_power_state *ps;
	static uint32_t gpe0_sts[GPE0_REG_MAX];
	uint32_t pm1_en;
	int i;

	ps = cbmem_find(CBMEM_ID_POWER_STATE);
	if (ps == NULL)
		return -1;

	/*
	 * PM1_EN to check the basic wake events which can happen through
	 * powerbtn or any other wake source like lidopen, key board press etc.
	 */
	pm1_en = ps->pm1_en;

	pm1_en = acpi_fill_soc_wake(pm1_en, ps);

	*pm1 = ps->pm1_sts & pm1_en;

	/* Mask off GPE0 status bits that are not enabled */
	*gpe0 = &gpe0_sts[0];
	for (i = 0; i < GPE0_REG_MAX; i++)
		gpe0_sts[i] = ps->gpe0_sts[i] & ps->gpe0_en[i];

	return GPE0_REG_MAX;
}
#endif

__weak int acpi_create_gnvs(struct acpi_global_nvs *gnvs)
{
	return 0;
}

#if 0
void southbridge_inject_dsdt(struct udevice *dev)
{
	struct global_nvs_t *gnvs;

	gnvs = cbmem_find(CBMEM_ID_ACPI_GNVS);
	if (!gnvs) {
		gnvs = cbmem_add(CBMEM_ID_ACPI_GNVS, sizeof(*gnvs));
		if (gnvs)
			memset(gnvs, 0, sizeof(*gnvs));
	}

	if (gnvs) {
		acpi_create_gnvs(gnvs);
		/* And tell SMI about it */
		smm_setup_structures(gnvs, NULL, NULL);

		/* Add it to DSDT.  */
		acpigen_write_scope("\\");
		acpigen_write_name_dword("NVSA", (uintptr_t) gnvs);
		acpigen_pop_len();
	}
}
#endif

static int calculate_power(int tdp, int p1_ratio, int ratio)
{
	u32 m;
	u32 power;

	/*
	 * M = ((1.1 - ((p1_ratio - ratio) * 0.00625)) / 1.1) ^ 2
	 *
	 * Power = (ratio / p1_ratio) * m * tdp
	 */

	m = (110000 - ((p1_ratio - ratio) * 625)) / 11;
	m = (m * m) / 1000;

	power = ((ratio * 100000 / p1_ratio) / 100);
	power *= (m / 100) * (tdp / 1000);
	power /= 1000;

	return power;
}

static int get_cores_per_package(void)
{
	struct cpuid_result result;
	int cores = 1;

	if (gd->arch.x86_vendor != X86_VENDOR_INTEL)
		return 1;

	result = cpuid_ext(0xb, 1);
	cores = result.ebx & 0xff;

	return cores;
}

static void generate_c_state_entries(void)
{
	struct acpi_cstate *c_state_map;
	size_t entries;

	c_state_map = soc_get_cstate_map(&entries);

	/* Generate C-state tables */
	acpigen_write_cst_package(c_state_map, entries);
}

void generate_p_state_entries(int core, int cores_per_package)
{
	int ratio_min, ratio_max, ratio_turbo, ratio_step;
	int coord_type, power_max, num_entries;
	int ratio, power, clock, clock_max;
	bool turbo;

	coord_type = cpu_get_coord_type();
	ratio_min = cpu_get_min_ratio();
	ratio_max = cpu_get_max_ratio();
	clock_max = (ratio_max * cpu_get_bus_clock()) / 1000;
	turbo = (turbo_get_state() == TURBO_ENABLED);

	/* Calculate CPU TDP in mW */
	power_max = cpu_get_power_max();

	/* Write _PCT indicating use of FFixedHW */
	acpigen_write_empty_pct();

	/* Write _PPC with no limit on supported P-state */
	acpigen_write_ppc_nvs();
	/* Write PSD indicating configured coordination type */
	acpigen_write_psd_package(core, 1, coord_type);

	/* Add P-state entries in _PSS table */
	acpigen_write_name("_PSS");

	/* Determine ratio points */
	ratio_step = PSS_RATIO_STEP;
	do {
		num_entries = ((ratio_max - ratio_min) / ratio_step) + 1;
		if (((ratio_max - ratio_min) % ratio_step) > 0)
			num_entries += 1;
		if (turbo)
			num_entries += 1;
		if (num_entries > PSS_MAX_ENTRIES)
			ratio_step += 1;
	} while (num_entries > PSS_MAX_ENTRIES);

	/* _PSS package count depends on Turbo */
	acpigen_write_package(num_entries);

	/* P[T] is Turbo state if enabled */
	if (turbo) {
		ratio_turbo = cpu_get_max_turbo_ratio();

		/* Add entry for Turbo ratio */
		acpigen_write_pss_package(clock_max + 1,	/* MHz */
					  power_max,		/* mW */
					  PSS_LATENCY_TRANSITION,/* lat1 */
					  PSS_LATENCY_BUSMASTER,/* lat2 */
					  ratio_turbo << 8,	/* control */
					  ratio_turbo << 8);	/* status */
		num_entries -= 1;
	}

	/* First regular entry is max non-turbo ratio */
	acpigen_write_pss_package(clock_max,		/* MHz */
				  power_max,		/* mW */
				  PSS_LATENCY_TRANSITION,/* lat1 */
				  PSS_LATENCY_BUSMASTER,/* lat2 */
				  ratio_max << 8,	/* control */
				  ratio_max << 8);	/* status */
	num_entries -= 1;

	/* Generate the remaining entries */
	for (ratio = ratio_min + ((num_entries - 1) * ratio_step);
	     ratio >= ratio_min; ratio -= ratio_step) {

		/* Calculate power at this ratio */
		power = calculate_power(power_max, ratio_max, ratio);
		clock = (ratio * cpu_get_bus_clock()) / 1000;

		acpigen_write_pss_package(clock,		/* MHz */
					  power,		/* mW */
					  PSS_LATENCY_TRANSITION,/* lat1 */
					  PSS_LATENCY_BUSMASTER,/* lat2 */
					  ratio << 8,		/* control */
					  ratio << 8);		/* status */
	}
	/* Fix package length */
	acpigen_pop_len();
}

__weak struct acpi_tstate *soc_get_tss_table(int *entries)
{
	*entries = 0;

	return NULL;
}

void generate_t_state_entries(int core, int cores_per_package)
{
	struct acpi_tstate *soc_tss_table;
	int entries;

	soc_tss_table = soc_get_tss_table(&entries);
	if (entries == 0)
		return;

	/* Indicate SW_ALL coordination for T-states */
	acpigen_write_tsd_package(core, cores_per_package, SW_ALL);

	/* Indicate FixedHW so OS will use MSR */
	acpigen_write_empty_ptc();

	/* Set NVS controlled T-state limit */
	acpigen_write_tpc("\\TLVL");

	/* Write TSS table for MSR access */
	acpigen_write_tss_package(entries, soc_tss_table);
}

__weak void soc_power_states_generation(int core_id,
						int cores_per_package)
{
}

int generate_cpu_entries(struct udevice *dev, struct acpi_ctx *ctx)
{
	int core_id, cpu_id, pcontrol_blk = ACPI_BASE_ADDRESS;
	int plen = 6;
	int totalcores;
	int cores_per_package;
	int numcpus;
	int ret;

	ret = cpu_get_count(dev);
	if (ret < 0)
		return log_msg_ret("count", ret);
	totalcores = ret;
	cores_per_package = get_cores_per_package();
	numcpus = totalcores / cores_per_package;
	log_debug("Found %d CPU(s) with %d core(s) each.\n", numcpus,
		  cores_per_package);

	for (cpu_id = 0; cpu_id < numcpus; cpu_id++) {
		for (core_id = 0; core_id < cores_per_package; core_id++) {
			if (core_id > 0) {
				pcontrol_blk = 0;
				plen = 0;
			}

			/* Generate processor \_PR.CPUx */
			acpigen_write_processor((cpu_id) * cores_per_package +
						core_id, pcontrol_blk, plen);

			/* Generate C-state tables */
			generate_c_state_entries();

			/* Soc specific power states generation */
			soc_power_states_generation(core_id, cores_per_package);

			acpigen_pop_len();
		}
	}
	/* PPKG is usually used for thermal management
	   of the first and only package. */
// 	acpigen_write_processor_package("PPKG", 0, cores_per_package);

	/* Add a method to notify processor nodes */
// 	acpigen_write_processor_cnot(cores_per_package);

	return 0;
}

#if IS_ENABLED(CONFIG_SOC_INTEL_COMMON_ACPI_WAKE_SOURCE)
/* Save wake source data for ACPI _SWS methods in NVS */
static void acpi_save_wake_source(void *unused)
{
	global_nvs_t *gnvs = cbmem_find(CBMEM_ID_ACPI_GNVS);
	uint32_t pm1, *gpe0;
	int gpe_reg, gpe_reg_count;
	int reg_size = sizeof(uint32_t) * 8;

	if (!gnvs)
		return;

	gnvs->pm1i = -1;
	gnvs->gpei = -1;

	gpe_reg_count = acpi_fill_wake(&pm1, &gpe0);
	if (gpe_reg_count < 0)
		return;

	/* Scan for first set bit in PM1 */
	for (gnvs->pm1i = 0; gnvs->pm1i < reg_size; gnvs->pm1i++) {
		if (pm1 & 1)
			break;
		pm1 >>= 1;
	}

	/* If unable to determine then return -1 */
	if (gnvs->pm1i >= 16)
		gnvs->pm1i = -1;

	/* Scan for first set bit in GPE registers */
	for (gpe_reg = 0; gpe_reg < gpe_reg_count; gpe_reg++) {
		uint32_t gpe = gpe0[gpe_reg];
		int start = gpe_reg * reg_size;
		int end = start + reg_size;

		if (gpe == 0) {
			if (!gnvs->gpei)
				gnvs->gpei = end;
			continue;
		}

		for (gnvs->gpei = start; gnvs->gpei < end; gnvs->gpei++) {
			if (gpe & 1)
				break;
			gpe >>= 1;
		}
	}

	/* If unable to determine then return -1 */
	if (gnvs->gpei >= gpe_reg_count * reg_size)
		gnvs->gpei = -1;

	printk(BIOS_DEBUG, "ACPI _SWS is PM1 Index %lld GPE Index %lld\n",
	       (long long)gnvs->pm1i, (long long)gnvs->gpei);
}

BOOT_STATE_INIT_ENTRY(BS_OS_RESUME, BS_ON_ENTRY, acpi_save_wake_source, NULL);

#endif

#ifdef CONFIG_INTEL_GENERIC_WIFI
static int intel_wifi_acpi_fill_ssdt(struct udevice *dev, struct acpi_ctx *ctx)
{
	struct generic_wifi_config config;
	bool have_config;
	int ret;

	ret = dev_read_u32(dev, "gpe-wake", &config.wake);
	have_config = !ret;
	/* By default, all intel wifi chips wake from S3 */
	config.maxsleep = 3;
	ret = generic_wifi_fill_ssdt(dev, have_config ? &config : NULL);
	if (ret)
		return log_msg_ret("wifi", ret);

	return 0;
}

struct acpi_ops wifi_acpi_ops = {
	.fill_ssdt_generator	= intel_wifi_acpi_fill_ssdt,
};

static const struct udevice_id intel_wifi_ids[] = {
	{ .compatible = "intel,generic-wifi" },
	{ }
};

U_BOOT_DRIVER(intel_wifi) = {
	.name		= "intel_wifi",
	.id		= UCLASS_MISC,
	.of_match	= intel_wifi_ids,
	acpi_ops_ptr(&wifi_acpi_ops)
};
#endif /* CONFIG_INTEL_GENERIC_WIFI */
