// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Google LLC
 */

#include <common.h>
#include <dm.h>
#include <spl.h>
#include <asm/cpu.h>
#include <asm/cpu_common.h>
#include <asm/intel_regs.h>
#include <asm/io.h>
#include <asm/pci.h>
#include <asm/arch/systemagent.h>

/**
 * struct apl_punit_platdata - platform data for punit
 *
 * @pciex_region_size: BAR length in bytes
 */
struct apl_punit_platdata {
	pci_dev_t bdf;
};

/*
 * Punit Initialization code. This all isn't documented, but
 * this is the recipe.
 */
static int punit_init(struct udevice *dev)
{
	struct apl_punit_platdata *plat = dev_get_platdata(dev);
	struct udevice *cpu;
	u32 reg;
	ulong start;
	int ret;

	/* Thermal throttle activation offset */
	ret = uclass_first_device_err(UCLASS_CPU, &cpu);
	if (ret)
		return log_msg_ret("Cannot find CPU", ret);
	cpu_configure_thermal_target(cpu);

	/*
	 * Software Core Disable Mask (P_CR_CORE_DISABLE_MASK_0_0_0_MCHBAR).
	 * Enable all cores here.
	 */
	writel(0, MCHBAR_REG(CORE_DISABLE_MASK));

	/* P-Unit bring up */
	reg = readl(MCHBAR_REG(BIOS_RESET_CPL));
	if (reg == 0xffffffff) {
		/* P-unit not found */
		debug("Punit MMIO not available\n");
		return -ENOENT;
	}

	/* Set Punit interrupt pin IPIN offset 3D */
	pci_x86_write_config(plat->bdf, PCI_INTERRUPT_PIN, 0x2, PCI_SIZE_8);

	/* Set PUINT IRQ to 24 and INTPIN LOCK */
	writel(PUINT_THERMAL_DEVICE_IRQ_VEC_NUMBER |
	       PUINT_THERMAL_DEVICE_IRQ_LOCK,
	       MCHBAR_REG(PUNIT_THERMAL_DEVICE_IRQ));

	if (!IS_ENABLED(CONFIG_SOC_INTEL_GLK))
		clrsetbits_le32(MCHBAR_REG(0x7818), 0x1fe0, 0x220);

	/* Stage0 BIOS Reset Complete (RST_CPL) */
	enable_bios_reset_cpl();

	/*
	 * Poll for bit 8 to check if PCODE has completed its action in response
	 * to BIOS Reset complete.  We wait here till 1 ms for the bit to get
	 * set.
	 */
	start = get_timer(0);
	while (!(readl(MCHBAR_REG(BIOS_RESET_CPL)) & PCODE_INIT_DONE)) {
		if (get_timer(start) > 1) {
			debug("PCODE Init Done timeout\n");
			return -ETIMEDOUT;
		}
		udelay(100);
	}
	debug("PUNIT init complete\n");

	return 0;
}

static int apl_punit_probe(struct udevice *dev)
{
	if (spl_phase() == PHASE_SPL)
		return punit_init(dev);

	return 0;
}

static int apl_punit_ofdata_to_platdata(struct udevice *dev)
{
	struct apl_punit_platdata *plat = dev_get_platdata(dev);
	int root;

	root = pci_get_devfn(dev);
	if (root < 0)
		return log_msg_ret("Cannot get host-bridge PCI address", root);
	plat->bdf = root;

	return 0;
}

static const struct udevice_id apl_syscon_ids[] = {
	{ .compatible = "intel,apl-punit", .data = X86_SYSCON_PUNIT },
	{ }
};

U_BOOT_DRIVER(syscon_intel_punit) = {
	.name		= "intel_punit_syscon",
	.id		= UCLASS_SYSCON,
	.of_match	= apl_syscon_ids,
	.ofdata_to_platdata = apl_punit_ofdata_to_platdata,
	.probe		= apl_punit_probe,
};
