# SPDX-License-Identifier:	GPL-2.0+
#
# Copyright 2018 Google LLC
#

obj-$(CONFIG_CHROMEOS) += aux-fw-uclass.o
obj-$(CONFIG_CHROMEOS) += cros_ofnode.o
obj-$(CONFIG_CHROMEOS) += cros_tpm.o
ifndef CONFIG_SANDBOX
obj-$(CONFIG_CHROMEOS) += jump.o
endif
obj-$(CONFIG_CHROMEOS) += memwipe.o
obj-$(CONFIG_CHROMEOS) += resource.o
obj-$(CONFIG_CHROMEOS) += vboot_internal.o
obj-$(CONFIG_CHROMEOS) += vboot-ec-uclass.o
