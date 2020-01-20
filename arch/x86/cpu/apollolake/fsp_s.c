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

#if 0
static const char *name_from_id(enum uclass_id id)
{
	switch (id) {
	case UCLASS_USB_HUB:
		/* Root Hub */
		return "RHUB";
	/* DSDT: acpi/northbridge.asl */
	case UCLASS_NORTHBRIDGE:
		return "MCHC";
	/* DSDT: acpi/lpc.asl */
	case UCLASS_LPC:
		return "LPCB";
	/* DSDT: acpi/xhci.asl */
	case UCLASS_USB:
		return "XHCI";
	/* DSDT: acpi/pch_hda.asl */
	case UCLASS_SOUND:	/* Assume HDA for now */
		return "HDAS";
	case UCLASS_PWM:
		return "PWM";
	/* SDIO is not supported */
	/* PCIe */
	/* TODO(sjg@chromium.org): Get from device tree?
	case PCH_DEVFN_PCIE1:
		return "RP03";
	case PCH_DEVFN_PCIE5:
		return "RP01";
	*/
	default:
		return NULL;
	}
}

int soc_acpi_name(const struct udevice *dev, char *out_name)
{
	enum uclass_id parent_id = UCLASS_INVALID;
	enum uclass_id id;
	const char *name = NULL;

	id = device_get_uclass_id(dev);
	if (dev_get_parent(dev))
		parent_id = device_get_uclass_id(dev_get_parent(dev));
// 	printf("dev '%s', id=%d, parent_id=%d\n", dev->name, id, parent_id);

	name = dev_read_string(dev, "acpi,name");
	if (name)
		;
	else if (id == UCLASS_MMC)
		name = mmc_is_sd(dev) ? "SDCD" : "EMMC";
	else if (id == UCLASS_ROOT)
		name = "\\_SB";
	else if (id == UCLASS_SOUND)
		name = "HDAS";
	else if (device_is_on_pci_bus(dev))
		name = name_from_id(id);
	if (!name && id == UCLASS_PCI) {
		struct pci_controller *hose = dev_get_uclass_priv(dev);

		if (hose->acpi_name)
			name = hose->acpi_name;
		else
			name = "PCI0";
	}
	if (!name) {
		switch (parent_id) {
		case UCLASS_USB: {
			struct usb_device *udev = dev_get_parent_priv(dev);

			sprintf(out_name, udev->speed >= USB_SPEED_SUPER ?
				"HS%02d" : "FS%02d",
				udev->portnr);
			name = out_name;
			break;
		}
		default:
			break;
		}
	}
	if (!name) {
		int num;

		if (dev->req_seq == -1) {
			log_warning("Device '%s' has no seq\n", dev->name);
			return log_msg_ret("no seq", -ENXIO);
		}
		num = dev->req_seq;
		switch (id) {
		/* DSDT: acpi/lpss.asl */
		case UCLASS_SERIAL:
			sprintf(out_name, "URT%d", num);
			name = out_name;
			break;
		case UCLASS_I2C:
			sprintf(out_name, "I2C%d", num);
			name = out_name;
			break;
		case UCLASS_SPI:
			sprintf(out_name, "SPI%d", num);
			name = out_name;
			break;
		default:
			break;
		}
	}
	if (!name) {
		log_warning("No name for device '%s'\n", dev->name);
		return -ENOENT;
	}
	if (name != out_name)
		memcpy(out_name, name, ACPI_NAME_MAX);
// 	printf("acpi_name for '%s': %s\n", dev->name, name);

	return 0;
}
#endif

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

#if 0
static void disable_dev(struct device *dev, FSP_S_CONFIG *silconfig)
{
	switch (dev->path.pci.devfn) {
	case PCH_DEVFN_ISH:
		silconfig->IshEnable = 0;
		break;
	case PCH_DEVFN_SATA:
		silconfig->EnableSata = 0;
		break;
	case PCH_DEVFN_PCIE5:
		silconfig->PcieRootPortEn[0] = 0;
		silconfig->PcieRpHotPlug[0] = 0;
		break;
	case PCH_DEVFN_PCIE6:
		silconfig->PcieRootPortEn[1] = 0;
		silconfig->PcieRpHotPlug[1] = 0;
		break;
	case PCH_DEVFN_PCIE1:
		silconfig->PcieRootPortEn[2] = 0;
		silconfig->PcieRpHotPlug[2] = 0;
		break;
	case PCH_DEVFN_PCIE2:
		silconfig->PcieRootPortEn[3] = 0;
		silconfig->PcieRpHotPlug[3] = 0;
		break;
	case PCH_DEVFN_PCIE3:
		silconfig->PcieRootPortEn[4] = 0;
		silconfig->PcieRpHotPlug[4] = 0;
		break;
	case PCH_DEVFN_PCIE4:
		silconfig->PcieRootPortEn[5] = 0;
		silconfig->PcieRpHotPlug[5] = 0;
		break;
	case PCH_DEVFN_XHCI:
		silconfig->Usb30Mode = 0;
		break;
	case PCH_DEVFN_XDCI:
		silconfig->UsbOtg = 0;
		break;
	case PCH_DEVFN_I2C0:
		silconfig->I2c0Enable = 0;
		break;
	case PCH_DEVFN_I2C1:
		silconfig->I2c1Enable = 0;
		break;
	case PCH_DEVFN_I2C2:
		silconfig->I2c2Enable = 0;
		break;
	case PCH_DEVFN_I2C3:
		silconfig->I2c3Enable = 0;
		break;
	case PCH_DEVFN_I2C4:
		silconfig->I2c4Enable = 0;
		break;
	case PCH_DEVFN_I2C5:
		silconfig->I2c5Enable = 0;
		break;
	case PCH_DEVFN_I2C6:
		silconfig->I2c6Enable = 0;
		break;
	case PCH_DEVFN_I2C7:
		silconfig->I2c7Enable = 0;
		break;
	case PCH_DEVFN_UART0:
		silconfig->Hsuart0Enable = 0;
		break;
	case PCH_DEVFN_UART1:
		silconfig->Hsuart1Enable = 0;
		break;
	case PCH_DEVFN_UART2:
		silconfig->Hsuart2Enable = 0;
		break;
	case PCH_DEVFN_UART3:
		silconfig->Hsuart3Enable = 0;
		break;
	case PCH_DEVFN_SPI0:
		silconfig->Spi0Enable = 0;
		break;
	case PCH_DEVFN_SPI1:
		silconfig->Spi1Enable = 0;
		break;
	case PCH_DEVFN_SPI2:
		silconfig->Spi2Enable = 0;
		break;
	case PCH_DEVFN_SDCARD:
		silconfig->SdcardEnabled = 0;
		break;
	case PCH_DEVFN_EMMC:
		silconfig->eMMCEnabled = 0;
		break;
	case PCH_DEVFN_SDIO:
		silconfig->SdioEnabled = 0;
		break;
	case PCH_DEVFN_SMBUS:
		silconfig->SmbusEnable = 0;
		break;
#if !CONFIG(SOC_INTEL_GLK)
	case SA_DEVFN_IPU:
		silconfig->IpuEn = 0;
		break;
#endif
	case PCH_DEVFN_HDA:
		silconfig->HdaEnable = 0;
		break;
	default:
		printk(BIOS_WARNING, "PCI:%02x.%01x: Could not disable the device\n",
			PCI_SLOT(dev->path.pci.devfn),
			PCI_FUNC(dev->path.pci.devfn));
		break;
	}
}

static void parse_devicetree(FSP_S_CONFIG *silconfig)
{
	struct device *dev = pcidev_path_on_root(SA_DEVFN_ROOT);

	if (!dev) {
		printk(BIOS_ERR, "Could not find root device\n");
		return;
	}
	/* Only disable bus 0 devices. */
	for (dev = dev->bus->children; dev; dev = dev->sibling) {
		if (!dev->enabled)
			disable_dev(dev, silconfig);
	}
}
#endif

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

#if 0
/*
 * If the PCIe root port at function 0 is disabled,
 * the PCIe root ports might be coalesced after FSP silicon init.
 * The below function will swap the devfn of the first enabled device
 * in devicetree and function 0 resides a pci device
 * so that it won't confuse coreboot.
 */
static void pcie_update_device_tree(unsigned int devfn0, int num_funcs)
{
	struct device *func0;
	unsigned int devfn;
	int i;
	unsigned int inc = PCI_DEVFN(0, 1);

	func0 = pcidev_path_on_root(devfn0);
	if (func0 == NULL)
		return;

	/* No more functions if function 0 is disabled. */
	if (pci_read_config32(func0, PCI_VENDOR_ID) == 0xffffffff)
		return;

	devfn = devfn0 + inc;

	/*
	 * Increase funtion by 1.
	 * Then find first enabled device to replace func0
	 * as that port was move to func0.
	 */
	for (i = 1; i < num_funcs; i++, devfn += inc) {
		struct device *dev = pcidev_path_on_root(devfn);
		if (dev == NULL)
			continue;

		if (!dev->enabled)
			continue;
		/* Found the first enabled device in given dev number */
		func0->path.pci.devfn = dev->path.pci.devfn;
		dev->path.pci.devfn = devfn0;
		break;
	}
}

static void pcie_override_devicetree_after_silicon_init(void)
{
	pcie_update_device_tree(PCH_DEVFN_PCIE1, 4);
	pcie_update_device_tree(PCH_DEVFN_PCIE5, 2);
}
#endif

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

	printf("get itss\n");
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

#if 0
static void soc_final(void *data)
{
	/* Disable global reset, just in case */
	pmc_global_reset_enable(0);
	/* Make sure payload/OS can't trigger global reset */
	pmc_global_reset_lock();
}

static void drop_privilege_all(void)
{
	/* Drop privilege level on all the CPUs */
	if (mp_run_on_all_cpus(&cpu_enable_untrusted_mode, NULL) < 0)
		printk(BIOS_ERR, "failed to enable untrusted mode\n");
}

static void configure_xhci_host_mode_port0(void)
{
	uint32_t *cfg0;
	uint32_t *cfg1;
	const struct resource *res;
	uint32_t reg;
	struct stopwatch sw;
	struct device *xhci_dev = PCH_DEV_XHCI;

	printk(BIOS_INFO, "Putting xHCI port 0 into host mode.\n");
	res = find_resource(xhci_dev, PCI_BASE_ADDRESS_0);
	cfg0 = (void *)(uintptr_t)(res->base + DUAL_ROLE_CFG0);
	cfg1 = (void *)(uintptr_t)(res->base + DUAL_ROLE_CFG1);
	reg = read32(cfg0);
	if (!(reg & SW_IDPIN_EN_MASK))
		return;

	reg &= ~(SW_IDPIN_MASK | SW_VBUS_VALID_MASK);
	write32(cfg0, reg);

	stopwatch_init_msecs_expire(&sw, 10);
	/* Wait for the host mode status bit. */
	while ((read32(cfg1) & DRD_MODE_MASK) != DRD_MODE_HOST) {
		if (stopwatch_expired(&sw)) {
			printk(BIOS_ERR, "Timed out waiting for host mode.\n");
			return;
		}
	}

	printk(BIOS_INFO, "xHCI port 0 host switch over took %lu ms\n",
		stopwatch_duration_msecs(&sw));
}

static int check_xdci_enable(void)
{
	struct device *dev = PCH_DEV_XDCI;

	return !!dev->enabled;
}

void platform_fsp_notify_status(enum fsp_notify_phase phase)
{
	if (phase == END_OF_FIRMWARE) {

		/*
		 * Before hiding P2SB device and dropping privilege level,
		 * dump CSE status and disable HECI1 interface.
		 */
		heci_cse_lockdown();

		/* Hide the P2SB device to align with previous behavior. */
		p2sb_hide();

		/*
		 * As per guidelines BIOS is recommended to drop CPU privilege
		 * level to IA_UNTRUSTED. After that certain device registers
		 * and MSRs become inaccessible supposedly increasing system
		 * security.
		 */
		drop_privilege_all();

		/*
		 * When USB OTG is set, GLK FSP enables xHCI SW ID pin and
		 * configures USB-C as device mode. Force USB-C into host mode.
		 */
		if (check_xdci_enable())
			configure_xhci_host_mode_port0();

		/*
		 * Override GLK xhci clock gating register(XHCLKGTEN) to
		 * mitigate usb device suspend and resume failure.
		 */
		if (CONFIG(SOC_INTEL_GLK)) {
			uint32_t *cfg;
			const struct resource *res;
			uint32_t reg;
			struct device *xhci_dev = PCH_DEV_XHCI;

			res = find_resource(xhci_dev, PCI_BASE_ADDRESS_0);
			cfg = (void *)(uintptr_t)(res->base + CFG_XHCLKGTEN);
			reg = SRAMPGTEN | SSLSE | USB2PLLSE | IOSFSTCGE |
				HSTCGE | HSUXDMIPLLSE | SSTCGE | XHCFTCLKSE |
				XHCBBTCGIPISO | XHCUSB2PLLSDLE | SSPLLSUE |
				XHCBLCGE | HSLTCGE | SSLTCGE | IOSFBTCGE |
				IOSFGBLCGE;
			write32(cfg, reg);
		}
	}
}

/*
 * spi_flash init() needs to run unconditionally on every boot (including
 * resume) to allow write protect to be disabled for eventlog and nvram
 * updates. This needs to be done as early as possible in ramstage. Thus, add a
 * callback for entry into BS_PRE_DEVICE.
 */
static void spi_flash_init_cb(void *unused)
{
	fast_spi_init();
}
#endif
