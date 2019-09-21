/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 Google LLC
 */

#ifndef _ASM_ARCH_CPU_H
#define _ASM_ARCH_CPU_H

/* Common Timer Copy (CTC) frequency - 19.2MHz */
#define CTC_FREQ		19200000

/*
 * Set to true to use the fast SPI driver to boot, instead of mapped SPI.
 * You also need to enabled CONFIG_SUPPORT_SPI_FLASHBOOT.
 */
#define BOOT_FROM_FAST_SPI_FLASH	false

/*
 * We need to read well past the end of the region in order for execution from
 * the loaded data to work. It is not clear why.
 */
#define SAFETY_MARGIN	0x4000

#define MAX_PCIE_PORTS		6
#define CLKREQ_DISABLED		0xf

#endif /* _ASM_ARCH_CPU_H */
