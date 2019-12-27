// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 Google, LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <dm.h>
#include <irq.h>
#include <dm/device-internal.h>

int irq_route_pmc_gpio_gpe(struct udevice *dev, uint pmc_gpe_num)
{
	const struct irq_ops *ops = irq_get_ops(dev);

	if (!ops->route_pmc_gpio_gpe)
		return -ENOSYS;

	return ops->route_pmc_gpio_gpe(dev, pmc_gpe_num);
}

int irq_set_polarity(struct udevice *dev, uint irq, bool active_low)
{
	const struct irq_ops *ops = irq_get_ops(dev);

	if (!ops->set_polarity)
		return -ENOSYS;

	return ops->set_polarity(dev, irq, active_low);
}

int irq_snapshot_polarities(struct udevice *dev)
{
	const struct irq_ops *ops = irq_get_ops(dev);

	if (!ops->snapshot_polarities)
		return -ENOSYS;

	return ops->snapshot_polarities(dev);
}

int irq_restore_polarities(struct udevice *dev)
{
	const struct irq_ops *ops = irq_get_ops(dev);

	if (!ops->restore_polarities)
		return -ENOSYS;

	return ops->restore_polarities(dev);
}

int irq_first_device_type(enum irq_dev_t type, struct udevice **devp)
{
	struct udevice *dev;
	struct uclass *uc;
	int ret;

	ret = uclass_get(UCLASS_IRQ, &uc);
	if (ret)
		return ret;
	uclass_foreach_dev(dev, uc) {
		if (dev_get_driver_data(dev) == type) {
			*devp = dev;

			return device_probe(dev);
		}
	}

	return -ENODEV;
}

UCLASS_DRIVER(irq) = {
	.id		= UCLASS_IRQ,
	.name		= "irq",
};
