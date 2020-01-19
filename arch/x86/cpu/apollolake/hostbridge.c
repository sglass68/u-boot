// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Google LLC
 * Copyright (C) 2015 - 2017 Intel Corp.
 * Copyright (C) 2017 - 2019 Siemens AG
 * (Written by Alexandru Gagniuc <alexandrux.gagniuc@intel.com> for Intel Corp.)
 * (Written by Andrey Petrov <andrey.petrov@intel.com> for Intel Corp.)
 *
 * Portions from coreboot soc/intel/apollolake/chip.c
 */

#define LOG_CATEGORY UCLASS_NORTHBRIDGE

#include <common.h>
#include <acpi_table.h>
#include <dm.h>
#include <dt-structs.h>
#include <spl.h>
#include <asm/intel_pinctrl.h>
#include <asm/intel_regs.h>
#include <asm/io.h>
#include <asm/pci.h>
#include <asm/arch/acpi.h>
#include <asm/arch/systemagent.h>
#include <dm/acpi.h>

/**
 * struct apl_hostbridge_platdata - platform data for hostbridge
 *
 * @dtplat: Platform data for of-platdata
 * @early_pads: Early pad data to set up, each (pad, cfg0, cfg1)
 * @early_pads_count: Number of pads to process
 * @pciex_region_size: BAR length in bytes
 * @bdf: Bus/device/function of hostbridge
 */
struct apl_hostbridge_platdata {
#if CONFIG_IS_ENABLED(OF_PLATDATA)
	struct dtd_intel_apl_hostbridge dtplat;
#endif
	u32 *early_pads;
	int early_pads_count;
	uint pciex_region_size;
	pci_dev_t bdf;
};

enum {
	PCIEXBAR		= 0x60,
	PCIEXBAR_LENGTH_256MB	= 0,
	PCIEXBAR_LENGTH_128MB,
	PCIEXBAR_LENGTH_64MB,

	PCIEXBAR_PCIEXBAREN	= 1 << 0,

	BGSM			= 0xb4,  /* Base GTT Stolen Memory */
	TSEG			= 0xb8,  /* TSEG base */
	TOLUD			= 0xbc,
};

static int apl_hostbridge_early_init_pinctrl(struct udevice *dev)
{
	struct apl_hostbridge_platdata *plat = dev_get_platdata(dev);
	struct udevice *pinctrl;
	int ret;

	ret = uclass_first_device_err(UCLASS_PINCTRL, &pinctrl);
	if (ret)
		return log_msg_ret("no hostbridge pinctrl", ret);

	return pinctrl_config_pads(pinctrl, plat->early_pads,
				   plat->early_pads_count);
}

static int apl_hostbridge_early_init(struct udevice *dev)
{
	struct apl_hostbridge_platdata *plat = dev_get_platdata(dev);
	u32 region_size;
	ulong base;
	u32 reg;
	int ret;

	/* Set up the MCHBAR */
	pci_x86_read_config(plat->bdf, MCHBAR, &base, PCI_SIZE_32);
	base = MCH_BASE_ADDRESS;
	pci_x86_write_config(plat->bdf, MCHBAR, base | 1, PCI_SIZE_32);

	/*
	 * The PCIEXBAR is assumed to live in the memory mapped IO space under
	 * 4GiB
	 */
	pci_x86_write_config(plat->bdf, PCIEXBAR + 4, 0, PCI_SIZE_32);

	switch (plat->pciex_region_size >> 20) {
	default:
	case 256:
		region_size = PCIEXBAR_LENGTH_256MB;
		break;
	case 128:
		region_size = PCIEXBAR_LENGTH_128MB;
		break;
	case 64:
		region_size = PCIEXBAR_LENGTH_64MB;
		break;
	}

	reg = CONFIG_MMCONF_BASE_ADDRESS | (region_size << 1)
				| PCIEXBAR_PCIEXBAREN;
	pci_x86_write_config(plat->bdf, PCIEXBAR, reg, PCI_SIZE_32);

	/*
	 * TSEG defines the base of SMM range. BIOS determines the base
	 * of TSEG memory which must be at or below Graphics base of GTT
	 * Stolen memory, hence its better to clear TSEG register early
	 * to avoid power on default non-zero value (if any).
	 */
	pci_x86_write_config(plat->bdf, TSEG, 0, PCI_SIZE_32);

	ret = apl_hostbridge_early_init_pinctrl(dev);
	if (ret)
		return log_msg_ret("pinctrl", ret);

	return 0;
}

static int apl_hostbridge_ofdata_to_platdata(struct udevice *dev)
{
	struct apl_hostbridge_platdata *plat = dev_get_platdata(dev);
	struct udevice *pinctrl;
	int ret;

	/*
	 * The host bridge holds the early pad data needed to get through TPL.
	 * This is a small amount of data, enough to fit in TPL, so we keep it
	 * separate from the full pad data, stored in the fsp-s subnode. That
	 * subnode is not present in TPL, to save space.
	 */
	ret = uclass_first_device_err(UCLASS_PINCTRL, &pinctrl);
	if (ret)
		return log_msg_ret("no hostbridge PINCTRL", ret);
#if !CONFIG_IS_ENABLED(OF_PLATDATA)
	int root;

	/* Get length of PCI Express Region */
	plat->pciex_region_size = dev_read_u32_default(dev, "pciex-region-size",
						       256 << 20);

	root = pci_get_devfn(dev);
	if (root < 0)
		return log_msg_ret("Cannot get host-bridge PCI address", root);
	plat->bdf = root;

	ret = pinctrl_read_pads(pinctrl, dev_ofnode(dev), "early-pads",
				&plat->early_pads, &plat->early_pads_count);
	if (ret)
		return log_msg_ret("early-pads", ret);
#else
	struct dtd_intel_apl_hostbridge *dtplat = &plat->dtplat;
	int size;

	plat->pciex_region_size = dtplat->pciex_region_size;
	plat->bdf = pci_ofplat_get_devfn(dtplat->reg[0]);

	/* Assume that if everything is 0, it is empty */
	plat->early_pads = dtplat->early_pads;
	size = ARRAY_SIZE(dtplat->early_pads);
	plat->early_pads_count = pinctrl_count_pads(pinctrl, plat->early_pads,
						    size);

#endif

	return 0;
}

static int apl_hostbridge_probe(struct udevice *dev)
{
	if (spl_phase() == PHASE_TPL)
		return apl_hostbridge_early_init(dev);

	return 0;
}

static int apl_acpi_hb_get_name(const struct udevice *dev, char *out_name)
{
	return acpi_return_name(out_name, "RHUB");
}

static int apl_acpi_hb_write_tables(const struct udevice *dev,
				    struct acpi_ctx *ctx)
{
	struct acpi_table_header *header;
	struct acpi_dmar *dmar;
	u32 val;

	/*
	 * Create DMAR table only if virtualization is enabled. Due to some
	 * constraints on Apollo Lake SoC (some stepping affected), VTD could
	 * not be enabled together with IPU. Doing so will override and disable
	 * VTD while leaving CAPID0_A still reporting that VTD is available.
	 * As in this case FSP will lock VTD to disabled state, we need to make
	 * sure that DMAR table generation only happens when at least DEFVTBAR
	 * is enabled. Otherwise the DMAR header will be generated while the
	 * content of the table will be missing.
	 */

	dm_pci_read_config32(dev, CAPID0_A, &val);
	if ((val & VTD_DISABLE) ||
	    !(readl(MCHBAR_REG(DEFVTBAR)) & VTBAR_ENABLED))
		return 0;

	log_debug("ACPI:    * DMAR\n");
	dmar = (struct acpi_dmar *)ctx->current;
	header = &dmar->header;
	acpi_create_dmar(dmar, DMAR_INTR_REMAP);
	ctx->current += sizeof(struct acpi_dmar);
	apl_acpi_fill_dmar(ctx);

	/* (Re)calculate length and checksum */
	header->length = ctx->current - (void *)dmar;
	header->checksum = acpi_checksum((void *)dmar, header->length);

	acpi_align(ctx);
	acpi_add_table(ctx, dmar);
	acpi_align(ctx);

	return 0;
}

static int apl_hostbridge_remove(struct udevice *dev)
{
	/*
	 * TODO(sjg@chromium.org): Consider adding code from coreboot's
	 * platform_fsp_notify_status()
	 */

	return 0;
}

static ulong sa_read_reg(struct udevice *dev, int reg)
{
	u32 val;

	/* All regions concerned for have 1 MiB alignment */
	dm_pci_read_config32(dev, BGSM, &val);

	return ALIGN_DOWN(val, 1 << 20);
}

ulong sa_get_tolud_base(struct udevice *dev)
{
	return sa_read_reg(dev, TOLUD);
}

ulong sa_get_gsm_base(struct udevice *dev)
{
	return sa_read_reg(dev, BGSM);
}

ulong sa_get_tseg_base(struct udevice *dev)
{
	return sa_read_reg(dev, TSEG);
}

struct acpi_ops apl_hostbridge_acpi_ops = {
	.get_name	= apl_acpi_hb_get_name,
	.write_tables	= apl_acpi_hb_write_tables,
};

static const struct udevice_id apl_hostbridge_ids[] = {
	{ .compatible = "intel,apl-hostbridge" },
	{ }
};

U_BOOT_DRIVER(apl_hostbridge_drv) = {
	.name		= "intel_apl_hostbridge",
	.id		= UCLASS_NORTHBRIDGE,
	.of_match	= apl_hostbridge_ids,
	.ofdata_to_platdata = apl_hostbridge_ofdata_to_platdata,
	.probe		= apl_hostbridge_probe,
	.remove		= apl_hostbridge_remove,
	.platdata_auto_alloc_size = sizeof(struct apl_hostbridge_platdata),
	acpi_ops_ptr(&apl_hostbridge_acpi_ops)
	.flags		= DM_FLAG_OS_PREPARE,
};
