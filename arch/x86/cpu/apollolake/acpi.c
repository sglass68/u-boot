/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2016 Intel Corp.
 * Copyright (C) 2017-2019 Siemens AG
 * (Written by Lance Zhao <lijian.zhao@intel.com> for Intel Corp.)
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

#define LOG_CATEGORY LOGC_ACPI
/*
#include <arch/acpi.h>
#include <arch/acpigen.h>
#include <console/console.h>
#include <device/mmio.h>
#include <arch/smp/mpspec.h>
#include <assert.h>
#include <device/pci_ops.h>
#include <cbmem.h>
#include <cpu/x86/smm.h>
#include <gpio.h>
#include <intelblocks/acpi.h>
#include <intelblocks/pmclib.h>
#include <intelblocks/sgx.h>
#include <intelblocks/p2sb.h>
#include <soc/iomap.h>
#include <soc/pm.h>
#include <soc/nvs.h>
#include <soc/pci_devs.h>
#include <soc/systemagent.h>
#include <string.h>

#include "chip.h"
*/
#include <common.h>
#include <acpi.h>
#include <acpi_s3.h>
#include <cpu.h>
#include <dm.h>
#include <p2sb.h>
#include <pci.h>
#include <asm/acpigen.h>
#include <asm/acpi_table.h>
#include <asm/cpu_common.h>
#include <asm/intel_pinctrl.h>
#include <asm/intel_pinctrl_defs.h>
#include <asm/intel_regs.h>
#include <asm/io.h>
#include <asm/mpspec.h>
#include <asm/tables.h>
#include <asm/arch/iomap.h>
#include <asm/arch/global_nvs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pm.h>
#include <asm/arch/soc_config.h>
#include <asm/arch/systemagent.h>
#include <dm/uclass-internal.h>
#include <power/acpi_pmc.h>

#define CSTATE_RES(address_space, width, offset, address)		\
	{								\
	.space_id = address_space,					\
	.bit_width = width,						\
	.bit_offset = offset,						\
	.addrl = address,						\
	}

static struct acpi_cstate cstate_map[] = {
	{
		/* C1 */
		.ctype = 1,		/* ACPI C1 */
		.latency = 1,
		.power = 1000,
		.resource = {
			.space_id = ACPI_ADDRESS_SPACE_FIXED,
		},
	}, {
		.ctype = 2,		/* ACPI C2 */
		.latency = 50,
		.power = 10,
		.resource = {
			.space_id = ACPI_ADDRESS_SPACE_IO,
			.addrl = 0x415,
		},
	}, {
		.ctype = 3,		/* ACPI C3 */
		.latency = 150,
		.power = 10,
		.resource = {
			.space_id = ACPI_ADDRESS_SPACE_IO,
			.addrl = 0x419,
		},
	},
};

int soc_read_sci_irq_select(void)
{
	struct acpi_pmc_upriv *upriv;
	struct udevice *dev;
	int ret;

	ret = uclass_first_device_err(UCLASS_ACPI_PMC, &dev);
	if (ret)
		return log_msg_ret("pmc", ret);
	upriv = dev_get_uclass_priv(dev);

	return readl(upriv->pmc_bar0 + IRQ_REG);
}

int soc_write_sci_irq_select(uint scis)
{
	struct acpi_pmc_upriv *upriv;
	struct udevice *dev;
	int ret;

	ret = uclass_first_device_err(UCLASS_ACPI_PMC, &dev);
	if (ret)
		return log_msg_ret("pmc", ret);
	upriv = dev_get_uclass_priv(dev);
	writel(scis, upriv->pmc_bar0 + IRQ_REG);

	return 0;
}

struct acpi_cstate *soc_get_cstate_map(size_t *entries)
{
	*entries = ARRAY_SIZE(cstate_map);
	return cstate_map;
}

int acpi_create_gnvs(struct acpi_global_nvs *gnvs)
{
	const struct apl_config *cfg = gd->arch.soc_config;
	struct udevice *cpu, *pinctrl;
	uint offset;
	int ret;

	if (!cfg)
		return log_msg_ret("cfg", -EINVAL);

	/* Clear out GNVS. */
	memset(gnvs, '\0', sizeof(*gnvs));

/* port to use log or something
	if (IS_ENABLED(CONFIG_CONSOLE_CBMEM))
		gnvs->cbmc = (uintptr_t) cbmem_find(CBMEM_ID_CONSOLE);
*/
#if 0
	if (IS_ENABLED(CONFIG_CHROMEOS)) {
		/* Initialize Verified Boot data */
		chromeos_init_chromeos_acpi(&gnvs->chromeos);
		gnvs->chromeos.vbt2 = ACTIVE_ECFW_RO;
	}
#endif
	/* Set unknown wake source */
	gnvs->pm1i = ~0ULL;

	/* CPU core count */
	gnvs->pcnt = 1;
	ret = uclass_find_first_device(UCLASS_CPU, &cpu);
	if (cpu) {
		ret = cpu_get_count(cpu);
		if (ret > 0)
			gnvs->pcnt = ret;
	}

	/* Enable DPTF based on mainboard configuration */
	gnvs->dpte = cfg->dptf_enable;

	/* Assign address of PERST_0 if GPIO is defined in devicetree */
	if (cfg->prt0_gpio != GPIO_PRT0_UDEF) {
		ret = intel_pinctrl_get_pad(cfg->prt0_gpio, &pinctrl, &offset);
		if (ret)
			return log_msg_ret("prt0", ret);
		gnvs->prt0 = intel_pinctrl_get_config_reg_addr(pinctrl, offset);
	}

	/* Get sdcard cd GPIO portid if GPIO is defined in devicetree.
	 * Get offset of sdcard cd pin.
	 */
	if (cfg->sdcard_cd_gpio) {
		ret = intel_pinctrl_get_pad(cfg->sdcard_cd_gpio, &pinctrl,
					    &offset);
		if (ret)
			return log_msg_ret("prt0", ret);
		gnvs->scdp = p2sb_get_port_id(pinctrl);
		gnvs->scdo = intel_pinctrl_get_acpi_pin(pinctrl, offset);
	}

	return 0;
}

uint32_t acpi_fill_soc_wake(uint32_t generic_pm1_en)
{
	/*
	 * WAK_STS bit is set when the system is in one of the sleep states
	 * (via the SLP_EN bit) and an enabled wake event occurs. Upon setting
	 * this bit, the PMC will transition the system to the ON state and
	 * can only be set by hardware and can only be cleared by writing a one
	 * to this bit position.
	 */
	generic_pm1_en |= WAK_STS | RTC_EN | PWRBTN_EN;

	return generic_pm1_en;
}

int soc_madt_sci_irq_polarity(int sci)
{
	return MP_IRQ_POLARITY_LOW;
}

void soc_fill_fadt(struct acpi_fadt *fadt)
{
	const struct apl_config *cfg = gd->arch.soc_config;

	if (!cfg) {
		printf("%s: bad\n", __func__);
		panic("no cfg");
// 		return log_msg_ret("cfg", -EINVAL);
	}

	fadt->pm_tmr_blk = IOMAP_ACPI_BASE + PM1_TMR;

	fadt->p_lvl2_lat = ACPI_FADT_C2_NOT_SUPPORTED;
	fadt->p_lvl3_lat = ACPI_FADT_C3_NOT_SUPPORTED;

	fadt->pm_tmr_len = 4;
	fadt->duty_width = 3;

	fadt->iapc_boot_arch = ACPI_FADT_LEGACY_DEVICES | ACPI_FADT_8042;

	fadt->x_pm_tmr_blk.space_id = 1;
	fadt->x_pm_tmr_blk.bit_width = fadt->pm_tmr_len * 8;
	fadt->x_pm_tmr_blk.addrl = IOMAP_ACPI_BASE + PM1_TMR;

	if (cfg->lpss_s0ix_enable)
		fadt->flags |= ACPI_FADT_LOW_PWR_IDLE_S0;
}

void acpi_create_fadt(struct acpi_fadt *fadt, struct acpi_facs *facs,
		      void *dsdt)
{
	const struct apl_config *cfg = gd->arch.soc_config;
	struct acpi_table_header *header = &fadt->header;

	if (!cfg) {
		printf("%s: bad\n", __func__);
		panic("no cfg");
// 		return log_msg_ret("cfg", -EINVAL);
	}
	acpi_fadt_common(fadt, facs, dsdt);
	intel_acpi_fill_fadt(fadt);
	soc_fill_fadt(fadt);
	header->checksum = table_compute_checksum(fadt, header->length);
}

int apl_acpi_fill_dmar(struct acpi_ctx *ctx)
{
	struct udevice *dev, *sa_dev;
	uint64_t gfxvtbar = readq(MCHBAR_REG(GFXVTBAR)) & VTBAR_MASK;
	uint64_t defvtbar = readq(MCHBAR_REG(DEFVTBAR)) & VTBAR_MASK;
	bool gfxvten = readl(MCHBAR_REG(GFXVTBAR)) & VTBAR_ENABLED;
	bool defvten = readl(MCHBAR_REG(DEFVTBAR)) & VTBAR_ENABLED;
	unsigned long tmp;
	int ret;

	printf("apl_acpi_fill_dmar start\n");

	uclass_find_first_device(UCLASS_VIDEO, &dev);
	ret = uclass_first_device_err(UCLASS_NORTHBRIDGE, &sa_dev);
	if (ret)
		return log_msg_ret("no sa", ret);

	/* IGD has to be enabled, GFXVTBAR set and enabled. */
	if (dev && device_active(dev) && gfxvtbar && gfxvten) {
		tmp = ctx->current;

		printf("dev %s %s\n", dev->name, sa_dev->name);
		ctx->current += acpi_create_dmar_drhd(ctx->current, 0, 0,
						      gfxvtbar);
		ctx->current += acpi_create_dmar_ds_pci(ctx->current, 0, 2, 0);
		acpi_dmar_drhd_fixup(tmp, ctx->current);

		/* Add RMRR entry */
		tmp = ctx->current;
		ctx->current += acpi_create_dmar_rmrr(ctx->current, 0,
				sa_get_gsm_base(sa_dev),
				sa_get_tolud_base(sa_dev) - 1);
		ctx->current += acpi_create_dmar_ds_pci(ctx->current, 0, 2, 0);
		acpi_dmar_rmrr_fixup(tmp, ctx->current);
	}

	/* DEFVTBAR has to be set and enabled. */
	if (defvtbar && defvten) {
		struct udevice *p2sb_dev;
		uint16_t ibdf, hbdf;
		uint ioapic, hpet;
		int ret;

		tmp = ctx->current;
		/*
		 * P2SB may already be hidden. There's no clear rule, when.
		 * It is needed to get bus, device and function for IOAPIC and
		 * HPET device which is stored in P2SB device. So unhide it to
		 * get the info and hide it again when done.
		 */
// 		p2sb_unhide();
		ret = uclass_first_device_err(UCLASS_P2SB, &p2sb_dev);
		if (ret)
			return log_msg_ret("p2sb", ret);

		dm_pci_read_config16(p2sb_dev, PCH_P2SB_IBDF, &ibdf);
		ioapic = PCI_TO_BDF(ibdf);
		dm_pci_read_config16(p2sb_dev, PCH_P2SB_HBDF, &hbdf);
		hpet = PCI_TO_BDF(hbdf);
// 		p2sb_hide();

		ctx->current += acpi_create_dmar_drhd(ctx->current,
				DRHD_INCLUDE_PCI_ALL, 0, defvtbar);
		ctx->current += acpi_create_dmar_ds_ioapic(ctx->current,
				2, PCI_BUS(ioapic), PCI_DEV(ioapic),
				PCI_FUNC(ioapic));
		ctx->current += acpi_create_dmar_ds_msi_hpet(ctx->current,
				0, PCI_BUS(hpet), PCI_DEV(hpet),
				PCI_FUNC(hpet));
		acpi_dmar_drhd_fixup(tmp, ctx->current);
	}

	return 0;
}

void soc_power_states_generation(int core_id, int cores_per_package)
{
	/* Generate P-state tables */
	generate_p_state_entries(core_id, cores_per_package);

	/* Generate T-state tables */
	generate_t_state_entries(core_id, cores_per_package);
}

static void acpigen_soc_get_dw0_in_local5(uintptr_t addr)
{
	/*
	 * Store (\_SB.GPC0 (addr), Local5)
	 * \_SB.GPC0 is used to read cfg0 value from dw0. It is defined in
	 * gpiolib.asl.
	 */
	acpigen_write_store();
	acpigen_emit_namestring("\\_SB.GPC0");
	acpigen_write_integer(addr);
	acpigen_emit_byte(LOCAL5_OP);
}

static int acpigen_soc_get_gpio_val(unsigned int gpio_num, uint32_t mask)
{
	assert(gpio_num < TOTAL_PADS);
	struct udevice *dev;
	uintptr_t addr;
	uint offset;
	int ret;

	ret = intel_pinctrl_get_pad(gpio_num, &dev, &offset);
	if (ret)
		return ret;
	addr = intel_pinctrl_get_config_reg_addr(dev, offset);

	acpigen_soc_get_dw0_in_local5(addr);

	/* If (And (Local5, mask)) */
	acpigen_write_if_and(LOCAL5_OP, mask);

	/* Store (One, Local0) */
	acpigen_write_store_ops(ONE_OP, LOCAL0_OP);

	acpigen_pop_len();	/* If */

	/* Else */
	acpigen_write_else();

	/* Store (Zero, Local0) */
	acpigen_write_store_ops(ZERO_OP, LOCAL0_OP);

	acpigen_pop_len();	/* Else */

	return 0;
}

static int acpigen_soc_set_gpio_val(unsigned int gpio_num, uint32_t val)
{
	assert(gpio_num < TOTAL_PADS);
	struct udevice *dev;
	uintptr_t addr;
	uint offset;
	int ret;

	ret = intel_pinctrl_get_pad(gpio_num, &dev, &offset);
	if (ret)
		return ret;
	addr = intel_pinctrl_get_config_reg_addr(dev, offset);

	acpigen_soc_get_dw0_in_local5(addr);

	if (val) {
		/* Or (Local5, PAD_CFG0_TX_STATE, Local5) */
		acpigen_write_or(LOCAL5_OP, PAD_CFG0_TX_STATE, LOCAL5_OP);
	} else {
		/* Not (PAD_CFG0_TX_STATE, Local6) */
		acpigen_write_not(PAD_CFG0_TX_STATE, LOCAL6_OP);

		/* And (Local5, Local6, Local5) */
		acpigen_write_and(LOCAL5_OP, LOCAL6_OP, LOCAL5_OP);
	}

	/*
	 * \_SB.SPC0 (addr, Local5)
	 * \_SB.SPC0 is used to write cfg0 value in dw0. It is defined in
	 * gpiolib.asl.
	 */
	acpigen_emit_namestring("\\_SB.SPC0");
	acpigen_write_integer(addr);
	acpigen_emit_byte(LOCAL5_OP);

	return 0;
}

int acpigen_soc_read_rx_gpio(unsigned int gpio_num)
{
	return acpigen_soc_get_gpio_val(gpio_num, PAD_CFG0_RX_STATE);
}

int acpigen_soc_get_tx_gpio(unsigned int gpio_num)
{
	return acpigen_soc_get_gpio_val(gpio_num, PAD_CFG0_TX_STATE);
}

int acpigen_soc_set_tx_gpio(unsigned int gpio_num)
{
	return acpigen_soc_set_gpio_val(gpio_num, 1);
}

int acpigen_soc_clear_tx_gpio(unsigned int gpio_num)
{
	return acpigen_soc_set_gpio_val(gpio_num, 0);
}