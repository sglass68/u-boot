// SPDX-License-Identifier: GPL-2.0+
/*
 * Implementation of verified-boot flags for Chromium OS
 *
 * Copyright 2018 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY UCLASS_CROS_VBOOT_FLAG

#include <common.h>
#include <dm.h>
#include <cros/vboot_flag.h>

/* Names for all the flag we know about */
static const char *const flag_name[] = {
	[VBOOT_FLAG_WRITE_PROTECT] = "write-protect",
	[VBOOT_FLAG_DEVELOPER] = "developer",
	[VBOOT_FLAG_LID_OPEN] = "lid-open",
	[VBOOT_FLAG_POWER_OFF] = "power-off",
	[VBOOT_FLAG_EC_IN_RW] = "ec-in-rw",
	[VBOOT_FLAG_OPROM_LOADED] = "oprom-loaded",
	[VBOOT_FLAG_RECOVERY] = "recovery",
	[VBOOT_FLAG_WIPEOUT] = "wipeout",
};

/**
 * struct vboot_flag_uc_priv - uclass information for each device
 *
 * Each device in this uclass has its own version of this struct.
 *
 * @flag: Flag that this device handles
 */
struct vboot_flag_uc_priv {
	enum vboot_flag_t flag;
};

/**
 * struct vboot_flag_uc_priv - information private to the uclass
 *
 * @value: Last read value for each flag
 *
 */
struct vboot_flag_state {
	int value[VBOOT_FLAG_COUNT];
};

int vboot_flag_read(struct udevice *dev)
{
	struct vboot_flag_ops *ops = vboot_flag_get_ops(dev);

	if (!ops->read)
		return -ENOSYS;

	return ops->read(dev);
}

int vboot_flag_read_walk_prev(enum vboot_flag_t flag, int *prevp)
{
	struct udevice *dev;
	struct uclass *uc;
	int ret;

	uclass_foreach_dev_probe(UCLASS_CROS_VBOOT_FLAG, dev) {
		struct vboot_flag_uc_priv *uc_priv = dev_get_uclass_priv(dev);

		if (uc_priv->flag == flag)
			break;
	}

	if (!dev) {
		log_err("No flag device for %s\n", flag_name[flag]);
		return -ENOENT;
	}

	ret = vboot_flag_read(dev);
	if (ret >= 0 && !uclass_get(device_get_uclass_id(dev), &uc)) {
		struct vboot_flag_state *state = uc->priv;

		if (prevp)
			*prevp = state->value[flag];
		state->value[flag] = ret;
	}

	return ret;
}

int vboot_flag_read_walk(enum vboot_flag_t flag)
{
	return vboot_flag_read_walk_prev(flag, NULL);
}

static int vboot_flag_pre_probe(struct udevice *dev)
{
	struct vboot_flag_uc_priv *uc_priv = dev_get_uclass_priv(dev);
	int i;

	for (i = 0; i < VBOOT_FLAG_COUNT; i++) {
		if (!strcmp(dev->name, flag_name[i]))
			break;
	}
	if (i == VBOOT_FLAG_COUNT) {
		log_warning("Unrecognised flag name '%s'\n", dev->name);
		return -EINVAL;
	}
	uc_priv->flag = i;

	return 0;
}

static int vboot_flag_init(struct uclass *uc)
{
	struct vboot_flag_state *state = uc->priv;
	int i;

	for (i = 0; i < VBOOT_FLAG_COUNT; i++)
		state->value[i] = -1;

	return 0;
}

UCLASS_DRIVER(vboot_flag) = {
	.id		= UCLASS_CROS_VBOOT_FLAG,
	.name		= "vboot_flag",
	.init		= vboot_flag_init,
	.pre_probe	= vboot_flag_pre_probe,
	.priv_auto_alloc_size	= sizeof(struct vboot_flag_state),
	.per_device_auto_alloc_size = sizeof(struct vboot_flag_uc_priv),
};
