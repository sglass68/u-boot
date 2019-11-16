/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2017 Intel Corporation.
 * Take from coreboot project file of the same name
 */

#ifndef _ASM_ARCH_IOMAP_H
#define _ASM_ARCH_IOMAP_H

#define TCO_BASE_ADDRESS	(ACPI_BASE_ADDRESS + 0x60)
#define TCO_BASE_SIZE		0x20

#define R_ACPI_PM1_TMR			0x8

/* Put p2sb at 0xd0000000 in TPL */
#define IOMAP_P2SB_BAR		0xd0000000

#define IOMAP_SPI_BASE		0xfe010000
#define IOMAP_I2C2_BASE		(0xfe020000 + 2 * 0x1000)

/*
 * Use UART2. To use UART1 you need to set '2' to '1', change device tree serial
 * node name and 'reg' property, and update CONFIG_DEBUG_UART_BASE.
 */
#define PCH_DEV_UART		PCI_BDF(0, 0x18, 2)

#define PCH_DEV_LPC		PCI_BDF(0, 0x1f, 0)
#define PCH_DEV_SPI		PCI_BDF(0, 0x0d, 2)

#endif
