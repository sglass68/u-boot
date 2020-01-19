/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#ifndef _ASM_ARCH_SOC_CONFIG_H
#define _ASM_ARCH_SOC_CONFIG_H

#include <i2c.h>

#define INTEL_GSPI_MAX		3
#define INTEL_I2C_DEV_MAX	8
#define MAX_USB2_PORTS		8
#define MAX_PCIE_PORTS		6

enum {
	CHIPSET_LOCKDOWN_FSP = 0, /* FSP handles locking per UPDs */
	CHIPSET_LOCKDOWN_COREBOOT, /* coreboot handles locking */
};

/* Serial IRQ control. SERIRQ_QUIET is the default (0) */
enum serirq_mode {
	SERIRQ_QUIET,
	SERIRQ_CONTINUOUS,
	SERIRQ_OFF,
};

struct gspi_cfg {
	/* Bus speed in MHz */
	u32 speed_mhz;
	/* Bus should be enabled prior to ramstage with temporary base */
	u8 early_init;
};

/*
 * This structure will hold data required by common blocks.
 * These are soc specific configurations which will be filled by soc.
 * We'll fill this structure once during init and use the data in common block.
 */
struct soc_intel_common_config {
	int chipset_lockdown;
	struct gspi_cfg gspi[INTEL_GSPI_MAX];
};

enum pnp_settings {
	PNP_PERF,
	PNP_POWER,
	PNP_PERF_POWER,
};

struct usb2_eye_per_port {
	u8 per_port_tx_pe_half;
	u8 per_port_pe_txi_set;
	u8 per_port_txi_set;
	u8 hs_skew_sel;
	u8 usb_tx_emphasis_en;
	u8 per_port_rxi_set;
	u8 hs_npre_drv_sel;
	u8 override_en;
};

struct apl_config {
	/* Common structure containing soc config data required by common code*/
	struct soc_intel_common_config common_soc_config;

	/*
	 * Mapping from PCIe root port to CLKREQ input on the SOC. The SOC has
	 * four CLKREQ inputs, but six root ports. Root ports without an
	 * associated CLKREQ signal must be marked with "CLKREQ_DISABLED"
	 */
	u8 pcie_rp_clkreq_pin[MAX_PCIE_PORTS];

	/* Enable/disable hot-plug for root ports (0 = disable, 1 = enable) */
	u8 pcie_rp_hotplug_enable[MAX_PCIE_PORTS];

	/* De-emphasis enable configuration for each PCIe root port */
	u8 pcie_rp_deemphasis_enable[MAX_PCIE_PORTS];

	/* [14:8] DDR mode Number of dealy elements.Each = 125pSec.
	 * [6:0] SDR mode Number of dealy elements.Each = 125pSec.
	 */
	u32 emmc_tx_cmd_cntl;

	/* [14:8] HS400 mode Number of dealy elements.Each = 125pSec.
	 * [6:0] SDR104/HS200 mode Number of dealy elements.Each = 125pSec.
	 */
	u32 emmc_tx_data_cntl1;

	/* [30:24] SDR50 mode Number of dealy elements.Each = 125pSec.
	 * [22:16] DDR50 mode Number of dealy elements.Each = 125pSec.
	 * [14:8] SDR25/HS50 mode Number of dealy elements.Each = 125pSec.
	 * [6:0] SDR12/Compatibility mode Number of dealy elements.
	 *       Each = 125pSec.
	 */
	u32 emmc_tx_data_cntl2;

	/* [30:24] SDR50 mode Number of dealy elements.Each = 125pSec.
	 * [22:16] DDR50 mode Number of dealy elements.Each = 125pSec.
	 * [14:8] SDR25/HS50 mode Number of dealy elements.Each = 125pSec.
	 * [6:0] SDR12/Compatibility mode Number of dealy elements.
	 *       Each = 125pSec.
	 */
	u32 emmc_rx_cmd_data_cntl1;

	/* [14:8] HS400 mode 1 Number of dealy elements.Each = 125pSec.
	 * [6:0] HS400 mode 2 Number of dealy elements.Each = 125pSec.
	 */
	u32 emmc_rx_strobe_cntl;

	/* [13:8] Auto Tuning mode Number of dealy elements.Each = 125pSec.
	 * [6:0] SDR104/HS200 Number of dealy elements.Each = 125pSec.
	 */
	u32 emmc_rx_cmd_data_cntl2;

	/* Select the eMMC max speed allowed */
	u32 emmc_host_max_speed;

	/* Specifies on which IRQ the SCI will internally appear */
	u32 sci_irq;

	/* Configure serial IRQ (SERIRQ) line */
	enum serirq_mode serirq_mode;

	/* Configure LPSS S0ix Enable */
	bool lpss_s0ix_enable;

	/* Enable DPTF support */
	bool dptf_enable;

	/* TCC activation offset value in degrees Celsius */
	int tcc_offset;

	/* Configure Audio clk gate and power gate
	 * IOSF-SB port ID 92 offset 0x530 [5] and [3]
	 */
	bool hdaudio_clk_gate_enable;
	bool hdaudio_pwr_gate_enable;
	bool hdaudio_bios_config_lockdown;

	/* SLP S3 minimum assertion width */
	int slp_s3_assertion_width_usecs;

	/* GPIO pin for PERST_0 */
	u32 prt0_gpio;

	/* USB2 eye diagram settings per port */
	struct usb2_eye_per_port usb2eye[MAX_USB2_PORTS];

	/* GPIO SD card detect pin */
	unsigned int sdcard_cd_gpio;

	/*
	 * PRMRR size setting with three options
	 *  0x02000000 - 32MiB
	 *  0x04000000 - 64MiB
	 *  0x08000000 - 128MiB
	 */
	u32 PrmrrSize;

	/*
	 * Enable SGX feature.
	 * Enabling SGX feature is 2 step process,
	 * (1) set sgx_enable = 1
	 * (2) set PrmrrSize to supported size
	 */
	bool sgx_enable;

	/*
	 * Select PNP Settings.
	 * (0) Performance,
	 * (1) Power
	 * (2) Power & Performance
	 */
	enum pnp_settings pnp_settings;

	/*
	 * PMIC PCH_PWROK delay configuration - IPC Configuration
	 * Upd for changing PCH_PWROK delay configuration : I2C_Slave_Address
	 * (31:24) + Register_Offset (23:16) + OR Value (15:8) + AND Value (7:0)
	 */
	u32 PmicPmcIpcCtrl;

	/*
	 * Options to disable XHCI Link Compliance Mode. Default is FALSE to not
	 * disable Compliance Mode. Set TRUE to disable Compliance Mode.
	 * 0:FALSE(Default), 1:True.
	 */
	bool DisableComplianceMode;

	/*
	 * Options to change USB3 ModPhy setting for the Integrated Filter (IF)
	 * value. Default is 0 to not changing default IF value (0x12). Set
	 * value with the range from 0x01 to 0xff to change IF value.
	 */
	u32 ModPhyIfValue;

	/*
	 * Options to bump USB3 LDO voltage. Default is FALSE to not increasing
	 * LDO voltage. Set TRUE to increase LDO voltage with 40mV.
	 * 0:FALSE (default), 1:True.
	 */
	bool ModPhyVoltageBump;

	/*
	 * Options to adjust PMIC Vdd2 voltage. Default is 0 to not adjusting
	 * the PMIC Vdd2 default voltage 1.20v. Upd for changing Vdd2 Voltage
	 * configuration: I2C_Slave_Address (31:23) + Register_Offset (23:16)
	 * + OR Value (15:8) + AND Value (7:0) through BUCK5_VID[3:2]:
	 * 00=1.10v, 01=1.15v, 10=1.24v, 11=1.20v (default).
	 */
	u32 PmicVdd2Voltage;

	/*
	 * Option to enable VTD feature. Default is 0 which disables VTD
	 * capability in FSP. Setting this option to 1 in devicetree will enable
	 * the Upd parameter VtdEnable.
	 */
	bool enable_vtd;
};

#endif
