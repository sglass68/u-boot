// SPDX-License-Identifier: GPL-2.0+
/*
 * Chromium OS alternative firmware, used to update firmware on devices in the
 * system other than those using UCLASS_VBOOT_EC.
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <dm.h>
#include <cros_ec.h>
#include <cros/vboot.h>
#include <cros/aux_fw.h>

/**
 * struct struct aux_fw_uc_priv - Info the uclass stores about each device
 *
 * @update_severity: Last recorded update severity, updated in
 *	aux_fw_check_hash()
 */
struct aux_fw_uc_priv {
	enum aux_fw_severity update_severity;
};

int aux_fw_check_hash(struct udevice *dev, const u8 *hash,
		      size_t hash_size, enum aux_fw_severity *severityp)
{
	struct aux_fw_ops *ops = aux_fw_get_ops(dev);
	struct aux_fw_uc_priv *uc_priv = dev_get_uclass_priv(dev);
	enum aux_fw_severity severity;
	int ret;

	if (!ops->check_hash)
		return -ENOSYS;

	ret = ops->check_hash(dev, hash, hash_size, &severity);
	if (ret)
		return ret;
	uc_priv->update_severity = severity;
	*severityp = severity;

	return 0;
}

int aux_fw_update_image(struct udevice *dev, const u8 *image,
			size_t image_size)
{
	struct aux_fw_ops *ops = aux_fw_get_ops(dev);

	if (!ops->update_image)
		return -ENOSYS;

	return ops->update_image(dev, image, image_size);
}

int aux_fw_get_protect(struct udevice *dev)
{
	struct aux_fw_ops *ops = aux_fw_get_ops(dev);
	int ret;

	if (!ops->get_protect)
		return -ENOSYS;

	ret = ops->get_protect(dev);
	if (ret < 0)
		return ret;

	return ret;
}

int aux_fw_set_protect(struct udevice *dev, bool protect)
{
	struct aux_fw_ops *ops = aux_fw_get_ops(dev);
	int ret;

	if (!ops->set_protect)
		return -ENOSYS;

	ret = ops->set_protect(dev, protect);
	if (ret)
		return ret;

	return 0;
}

enum aux_fw_severity aux_fw_get_severity(struct udevice *dev)
{
	struct aux_fw_uc_priv *uc_priv = dev_get_uclass_priv(dev);

	return uc_priv->update_severity;
}

UCLASS_DRIVER(cros_aux_fw) = {
	.id		= UCLASS_CROS_AUX_FW,
	.name		= "aux_fw",
	.per_device_auto_alloc_size	= sizeof(struct aux_fw_uc_priv),
};
