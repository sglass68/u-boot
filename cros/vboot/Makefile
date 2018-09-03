# SPDX-License-Identifier:	GPL-2.0+
#
# Copyright 2018 Google LLC
#

obj-$(CONFIG_CHROMEOS) += cros_ec_vboot.o
obj-$(CONFIG_CHROMEOS) += ec.o
obj-$(CONFIG_CHROMEOS) += handoff.o
obj-$(CONFIG_CHROMEOS) += legacy.o
obj-$(CONFIG_CHROMEOS) += misc.o
obj-$(CONFIG_CHROMEOS) += tlcl_stub.o
obj-$(CONFIG_CHROMEOS) += utility.o
obj-$(CONFIG_CHROMEOS) += vboot.o

ifndef CONFIG_SPL_BUILD
obj-$(CONFIG_CHROMEOS) += auxfw.o
obj-$(CONFIG_CHROMEOS) += boot_device.o
obj-$(CONFIG_CHROMEOS) += display.o
obj-$(CONFIG_CHROMEOS) += graphics.o
obj-$(CONFIG_CHROMEOS) += keyboard.o
obj-$(CONFIG_CHROMEOS) += screens.o
endif
