# SPDX-License-Identifier:	GPL-2.0+
#
# Copyright 2018 Google LLC
#

obj-$(CONFIG_CHROMEOS) += stages.o

# Verification stages (TPL)
ifneq ($(CONFIG_TPL_BUILD)$(CONFIG_VPL_BUILD),)
obj-$(CONFIG_CHROMEOS) += ver_init.o
obj-$(CONFIG_CHROMEOS) += ver1_vbinit.o
obj-$(CONFIG_CHROMEOS) += ver2_select_fw.o
obj-$(CONFIG_CHROMEOS) += ver3_try_fw.o
obj-$(CONFIG_CHROMEOS) += ver4_locate_fw.o
obj-$(CONFIG_CHROMEOS) += ver5_finish_fw.o
obj-$(CONFIG_CHROMEOS) += ver6_jump_fw.o
else
ifdef CONFIG_SPL_BUILD
# SPL stages
obj-$(CONFIG_CHROMEOS) += spl_init.o
obj-$(CONFIG_CHROMEOS) += spl_jump_u_boot.o

# U-Boot stages
else
obj-$(CONFIG_CHROMEOS) += ver_init.o
obj-$(CONFIG_CHROMEOS) += ver1_vbinit.o
obj-$(CONFIG_CHROMEOS) += ver2_select_fw.o
obj-$(CONFIG_CHROMEOS) += ver3_try_fw.o
obj-$(CONFIG_CHROMEOS) += ver4_locate_fw.o
obj-$(CONFIG_CHROMEOS) += ver5_finish_fw.o
obj-$(CONFIG_CHROMEOS) += ver6_jump_fw.o

obj-$(CONFIG_CHROMEOS) += spl_init.o
obj-$(CONFIG_CHROMEOS) += spl_jump_u_boot.o

obj-$(CONFIG_CHROMEOS) += rw_init.o
obj-$(CONFIG_CHROMEOS) += rw_select_kernel.o
obj-$(CONFIG_CHROMEOS) += rw_boot_kernel.o

endif
endif
