// SPDX-License-Identifier: GPL-2.0
/*
 * Sandbox P2SB for testing
 *
 * Copyright 2019 Google LLC
 */

#define LOG_CATEGORY UCLASS_P2SB

#include <common.h>
#include <dm.h>
#include <asm/io.h>
#include <p2sb.h>

struct sandbox_p2sb_priv {
	ulong base;
};

static int sandbox_p2sb_probe(struct udevice *dev)
{
	struct p2sb_uc_priv *upriv = dev_get_uclass_priv(dev);

	upriv->mmio_base = dm_pci_read_bar32(dev, 0);
	printf("mmio base %x\n", upriv->mmio_base);

	return 0;
}

static struct p2sb_ops sandbox_p2sb_ops = {
};

static const struct udevice_id sandbox_p2sb_ids[] = {
	{ .compatible = "sandbox,p2sb" },
	{ }
};

U_BOOT_DRIVER(p2sb_sandbox) = {
	.name = "pmic_pm8916",
	.id = UCLASS_P2SB,
	.of_match = sandbox_p2sb_ids,
	.probe = sandbox_p2sb_probe,
	.ops = &sandbox_p2sb_ops,
	.priv_auto_alloc_size = sizeof(struct sandbox_p2sb_priv),
};
