// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <acpi_s3.h>
#include <binman.h>
#include <dm.h>
#include <irq.h>
#include <mmc.h>
#include <p2sb.h>
#include <usb.h>
#include <acpi_device.h>
#include <asm/acpi_table.h>
#include <asm/intel_pinctrl.h>
#include <asm/io.h>
#include <asm/intel_regs.h>
#include <asm/msr.h>
#include <asm/msr-index.h>
#include <asm/pci.h>
#include <asm/arch/cpu.h>
#include <asm/arch/soc_config.h>
#include <asm/arch/systemagent.h>
#include <asm/arch/fsp/fsp_configs.h>
#include <asm/arch/fsp/fsp_s_upd.h>
#include <dm/uclass-internal.h>

static int get_config(struct udevice *dev, struct apl_config *apl)
{
	const u8 *ptr;
	ofnode node;
	u32 emmc[4];
	int ret;

	memset(apl, '\0', sizeof(*apl));

	node = dev_read_subnode(dev, "fsp-s");
	if (!ofnode_valid(node))
		return log_msg_ret("fsp-s settings", -ENOENT);

	ptr = ofnode_read_u8_array_ptr(node, "pcie-rp-clkreq-pin",
				       MAX_PCIE_PORTS);
	if (!ptr)
		return log_msg_ret("pcie-rp-clkreq-pin", -EINVAL);
	memcpy(apl->pcie_rp_clkreq_pin, ptr, MAX_PCIE_PORTS);

	ret = ofnode_read_u32(node, "prt0-gpio", &apl->prt0_gpio);
	if (ret)
		return log_msg_ret("prt0-gpio", ret);
	ret = ofnode_read_u32(node, "sdcard-cd-gpio", &apl->sdcard_cd_gpio);
	if (ret)
		return log_msg_ret("sdcard-cd-gpio", ret);

	ret = ofnode_read_u32_array(node, "emmc", emmc, ARRAY_SIZE(emmc));
	if (ret)
		return log_msg_ret("emmc", ret);
	apl->emmc_tx_data_cntl1 = emmc[0];
	apl->emmc_tx_data_cntl2 = emmc[1];
	apl->emmc_rx_cmd_data_cntl1 = emmc[2];
	apl->emmc_rx_cmd_data_cntl2 = emmc[3];

	apl->dptf_enable = ofnode_read_bool(node, "dptf-enable");

	apl->hdaudio_clk_gate_enable = ofnode_read_bool(node,
						"hdaudio-clk-gate-enable");
	apl->hdaudio_pwr_gate_enable = ofnode_read_bool(node,
						"hdaudio-pwr-gate-enable");
	apl->hdaudio_bios_config_lockdown = ofnode_read_bool(node,
					     "hdaudio-bios-config-lockdown");
	apl->lpss_s0ix_enable = ofnode_read_bool(node, "lpss-s0ix-enable");

	/* Santa */
	apl->usb2eye[1].per_port_pe_txi_set = 7;
	apl->usb2eye[1].per_port_txi_set = 2;

	return 0;
}

static void apl_fsp_silicon_init_params_cb(struct apl_config *apl,
					   struct fsp_s_config *cfg)
{
	u8 port;

	for (port = 0; port < MAX_USB2_PORTS; port++) {
		if (apl->usb2eye[port].per_port_tx_pe_half)
			cfg->port_usb20_per_port_tx_pe_half[port] =
				apl->usb2eye[port].per_port_tx_pe_half;

		if (apl->usb2eye[port].per_port_pe_txi_set)
			cfg->port_usb20_per_port_pe_txi_set[port] =
				apl->usb2eye[port].per_port_pe_txi_set;

		if (apl->usb2eye[port].per_port_txi_set)
			cfg->port_usb20_per_port_txi_set[port] =
				apl->usb2eye[port].per_port_txi_set;

		if (apl->usb2eye[port].hs_skew_sel)
			cfg->port_usb20_hs_skew_sel[port] =
				apl->usb2eye[port].hs_skew_sel;

		if (apl->usb2eye[port].usb_tx_emphasis_en)
			cfg->port_usb20_i_usb_tx_emphasis_en[port] =
				apl->usb2eye[port].usb_tx_emphasis_en;

		if (apl->usb2eye[port].per_port_rxi_set)
			cfg->port_usb20_per_port_rxi_set[port] =
				apl->usb2eye[port].per_port_rxi_set;

		if (apl->usb2eye[port].hs_npre_drv_sel)
			cfg->port_usb20_hs_npre_drv_sel[port] =
				apl->usb2eye[port].hs_npre_drv_sel;
	}
}

int fsps_update_config(struct udevice *dev, ulong rom_offset,
		       struct fsps_upd *upd)
{
	struct fsp_s_config *cfg = &upd->config;
	struct apl_config *apl;
	struct binman_entry vbt;
	void *buf;
	int ret;

	ret = binman_entry_find("intel-vbt", &vbt);
	if (ret)
		return log_msg_ret("Cannot find VBT", ret);
	vbt.image_pos += rom_offset;
	buf = malloc(vbt.size);
	if (!buf)
		return log_msg_ret("Alloc VBT", -ENOMEM);

	/*
	 * Load VBT before devicetree-specific config. This only supports
	 * memory-mapped SPI at present.
	 */
	bootstage_start(BOOTSTAGE_ID_ACCUM_MMAP_SPI, "mmap_spi");
	memcpy(buf, (void *)vbt.image_pos, vbt.size);
	bootstage_accum(BOOTSTAGE_ID_ACCUM_MMAP_SPI);
	if (*(u32 *)buf != VBT_SIGNATURE)
		return log_msg_ret("VBT signature", -EINVAL);
	cfg->graphics_config_ptr = (ulong)buf;

	apl = malloc(sizeof(*apl));
	if (!apl)
		return log_msg_ret("alloc", -ENOMEM);
	ret = get_config(dev, apl);
	if (ret)
		return log_msg_ret("config", ret);
	gd->arch.soc_config = apl;

	cfg->ish_enable = 0;
	cfg->enable_sata = 0;
	cfg->pcie_root_port_en[2] = 0;
	cfg->pcie_rp_hot_plug[2] = 0;
	cfg->pcie_root_port_en[3] = 0;
	cfg->pcie_rp_hot_plug[3] = 0;
	cfg->pcie_root_port_en[4] = 0;
	cfg->pcie_rp_hot_plug[4] = 0;
	cfg->pcie_root_port_en[5] = 0;
	cfg->pcie_rp_hot_plug[5] = 0;
	cfg->pcie_root_port_en[1] = 0;
	cfg->pcie_rp_hot_plug[1] = 0;
	cfg->usb_otg = 0;
	cfg->i2c6_enable = 0;
	cfg->i2c7_enable = 0;
	cfg->hsuart3_enable = 0;
	cfg->spi1_enable = 0;
	cfg->spi2_enable = 0;
	cfg->sdio_enabled = 0;

	memcpy(cfg->pcie_rp_clk_req_number, apl->pcie_rp_clkreq_pin,
	       sizeof(cfg->pcie_rp_clk_req_number));

	memcpy(cfg->pcie_rp_hot_plug, apl->pcie_rp_hotplug_enable,
	       sizeof(cfg->pcie_rp_hot_plug));

	switch (apl->serirq_mode) {
	case SERIRQ_QUIET:
		cfg->sirq_enable = 1;
		cfg->sirq_mode = 0;
		break;
	case SERIRQ_CONTINUOUS:
		cfg->sirq_enable = 1;
		cfg->sirq_mode = 1;
		break;
	case SERIRQ_OFF:
	default:
		cfg->sirq_enable = 0;
		break;
	}

	if (apl->emmc_tx_cmd_cntl)
		cfg->emmc_tx_cmd_cntl = apl->emmc_tx_cmd_cntl;
	if (apl->emmc_tx_data_cntl1)
		cfg->emmc_tx_data_cntl1 = apl->emmc_tx_data_cntl1;
	if (apl->emmc_tx_data_cntl2)
		cfg->emmc_tx_data_cntl2 = apl->emmc_tx_data_cntl2;
	if (apl->emmc_rx_cmd_data_cntl1)
		cfg->emmc_rx_cmd_data_cntl1 = apl->emmc_rx_cmd_data_cntl1;
	if (apl->emmc_rx_strobe_cntl)
		cfg->emmc_rx_strobe_cntl = apl->emmc_rx_strobe_cntl;
	if (apl->emmc_rx_cmd_data_cntl2)
		cfg->emmc_rx_cmd_data_cntl2 = apl->emmc_rx_cmd_data_cntl2;
	if (apl->emmc_host_max_speed)
		cfg->e_mmc_host_max_speed = apl->emmc_host_max_speed;

	cfg->lpss_s0ix_enable = apl->lpss_s0ix_enable;

	cfg->skip_mp_init = true;

	/* Disable setting of EISS bit in FSP */
	cfg->spi_eiss = 0;

	/* Disable FSP from locking access to the RTC NVRAM */
	cfg->rtc_lock = 0;

	/* Enable Audio clk gate and power gate */
	cfg->hd_audio_clk_gate = apl->hdaudio_clk_gate_enable;
	cfg->hd_audio_pwr_gate = apl->hdaudio_pwr_gate_enable;
	/* Bios config lockdown Audio clk and power gate */
	cfg->bios_cfg_lock_down = apl->hdaudio_bios_config_lockdown;
	apl_fsp_silicon_init_params_cb(apl, cfg);

	cfg->usb_otg = true;
	cfg->vtd_enable = apl->enable_vtd;

	return 0;
}

/* Configure package power limits */
static int set_power_limits(struct udevice *dev)
{
	msr_t rapl_msr_reg, limit;
	u32 power_unit;
	u32 tdp, min_power, max_power;
	u32 pl2_val;
	u32 override_tdp[2];
	int ret;

	/* Get units */
	rapl_msr_reg = msr_read(MSR_PKG_POWER_SKU_UNIT);
	power_unit = 1 << (rapl_msr_reg.lo & 0xf);

	/* Get power defaults for this SKU */
	rapl_msr_reg = msr_read(MSR_PKG_POWER_SKU);
	tdp = rapl_msr_reg.lo & PKG_POWER_LIMIT_MASK;
	pl2_val = rapl_msr_reg.hi & PKG_POWER_LIMIT_MASK;
	min_power = (rapl_msr_reg.lo >> 16) & PKG_POWER_LIMIT_MASK;
	max_power = rapl_msr_reg.hi & PKG_POWER_LIMIT_MASK;

	if (min_power > 0 && tdp < min_power)
		tdp = min_power;

	if (max_power > 0 && tdp > max_power)
		tdp = max_power;

	ret = dev_read_u32_array(dev, "tdp-pl-override-mw", override_tdp,
				 ARRAY_SIZE(override_tdp));
	if (ret)
		return log_msg_ret("tdp-pl-override-mw", ret);

	/* Set PL1 override value */
	if (override_tdp[0])
		tdp = override_tdp[0] * power_unit / 1000;

	/* Set PL2 override value */
	if (override_tdp[1])
		pl2_val = override_tdp[1] * power_unit / 1000;

	/* Set long term power limit to TDP */
	limit.lo = tdp & PKG_POWER_LIMIT_MASK;
	/* Set PL1 Pkg Power clamp bit */
	limit.lo |= PKG_POWER_LIMIT_CLAMP;

	limit.lo |= PKG_POWER_LIMIT_EN;
	limit.lo |= (MB_POWER_LIMIT1_TIME_DEFAULT &
		PKG_POWER_LIMIT_TIME_MASK) << PKG_POWER_LIMIT_TIME_SHIFT;

	/* Set short term power limit PL2 */
	limit.hi = pl2_val & PKG_POWER_LIMIT_MASK;
	limit.hi |= PKG_POWER_LIMIT_EN;

	/* Program package power limits in RAPL MSR */
	msr_write(MSR_PKG_POWER_LIMIT, limit);
	log_info("RAPL PL1 %d.%dW\n", tdp / power_unit,
		 100 * (tdp % power_unit) / power_unit);
	log_info("RAPL PL2 %d.%dW\n", pl2_val / power_unit,
		 100 * (pl2_val % power_unit) / power_unit);

	/*
	 * Sett RAPL MMIO register for Power limits. RAPL driver is using MSR
	 * instead of MMIO, so disable LIMIT_EN bit for MMIO
	 */
	writel(limit.lo & ~PKG_POWER_LIMIT_EN, MCHBAR_REG(MCHBAR_RAPL_PPL));
	writel(limit.hi & ~PKG_POWER_LIMIT_EN, MCHBAR_REG(MCHBAR_RAPL_PPL + 4));

	return 0;
}

int p2sb_unhide(void)
{
	struct udevice *dev;
	int ret;

	ret = uclass_find_first_device(UCLASS_P2SB, &dev);
	if (ret)
		return log_msg_ret("p2sb", ret);
	ret = p2sb_set_hide(dev, false);
	if (ret)
		return log_msg_ret("hide", ret);

	return 0;
}

/* Overwrites the SCI IRQ if another IRQ number is given by device tree */
static void set_sci_irq(void)
{
	/* Skip this for now */
}

int arch_fsps_preinit(void)
{
	struct udevice *itss;
	int ret;

	ret = irq_first_device_type(X86_IRQT_ITSS, &itss);
	if (ret)
		return log_msg_ret("no itss", ret);
	/*
	 * Snapshot the current GPIO IRQ polarities. FSP is setting a default
	 * policy that doesn't honour boards' requirements
	 */
	irq_snapshot_polarities(itss);

	/*
	 * Clear the GPI interrupt status and enable registers. These
	 * registers do not get reset to default state when booting from S5.
	 */
	ret = pinctrl_gpi_clear_int_cfg();
	if (ret)
		return log_msg_ret("gpi_clear", ret);

	return 0;
}

int arch_fsp_init_r(void)
{
#ifdef CONFIG_HAVE_ACPI_RESUME
	bool s3wake = gd->arch.prev_sleep_state == ACPI_S3;
#else
	bool s3wake = false;
#endif
	struct udevice *dev, *itss;
	int ret;

	if (!ll_boot_init())
		return 0;
	/*
	 * This must be called before any devices are probed. Put any probing
	 * into arch_fsps_preinit() above.
	 *
	 * We don't use CONFIG_APL_BOOT_FROM_FAST_SPI_FLASH here since it will
	 * force PCI to be probed.
	 */
	ret = fsp_silicon_init(s3wake, false);
	if (ret)
		return ret;

	ret = irq_first_device_type(X86_IRQT_ITSS, &itss);
	if (ret)
		return log_msg_ret("no itss", ret);
	/* Restore GPIO IRQ polarities back to previous settings */
	irq_restore_polarities(itss);

	/* soc_init() */
	ret = p2sb_unhide();
	if (ret)
		return log_msg_ret("unhide p2sb", ret);

	/* Set RAPL MSR for Package power limits*/
	ret = uclass_first_device_err(UCLASS_NORTHBRIDGE, &dev);
	if (ret)
		return log_msg_ret("Cannot get northbridge", ret);
	set_power_limits(dev);

	/*
	 * FSP-S routes SCI to IRQ 9. With the help of this function you can
	 * select another IRQ for SCI.
	 */
	set_sci_irq();

	return 0;
}
