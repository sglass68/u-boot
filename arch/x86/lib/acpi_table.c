// SPDX-License-Identifier: GPL-2.0+
/*
 * Based on acpi.c from coreboot
 *
 * Copyright (C) 2015, Saket Sinha <saket.sinha89@gmail.com>
 * Copyright (C) 2016, Bin Meng <bmeng.cn@gmail.com>
 */

#define DEBUG
#define LOG_CATEGORY LOGC_ACPI

#include <common.h>
#include <cpu.h>
#include <dm.h>
#include <dm/uclass-internal.h>
#include <serial.h>
#include <version.h>
#include <asm/acpi/global_nvs.h>
#include <asm/acpi_device.h>
#include <asm/acpi_table.h>
#include <asm/cpu.h>
#include <asm/ioapic.h>
#include <asm/lapic.h>
#include <asm/mpspec.h>
#include <asm/processor.h>
#include <asm/smm.h>
#include <asm/tables.h>
#include <asm/arch/global_nvs.h>
#include <asm/arch/iomap.h>
#include <asm/arch/pm.h>
#include <power/acpi_pmc.h>

/* TODO(sjg@chromium.org): Figure out how to get compiler revision */
#define ASL_REVISION	0

/*
 * IASL compiles the dsdt entries and writes the hex values
 * to a C array AmlCode[] (see dsdt.c).
 */
extern const unsigned char AmlCode[];

/* ACPI RSDP address to be used in boot parameters */
static ulong acpi_rsdp_addr;

u8 acpi_checksum(u8 *table, u32 length)
{
	u8 ret = 0;
	while (length--) {
		ret += *table;
		table++;
	}
	return -ret;
}

static void acpi_write_rsdp(struct acpi_rsdp *rsdp, struct acpi_rsdt *rsdt,
			    struct acpi_xsdt *xsdt)
{
	memset(rsdp, 0, sizeof(struct acpi_rsdp));

	memcpy(rsdp->signature, RSDP_SIG, 8);
	memcpy(rsdp->oem_id, OEM_ID, 6);

	rsdp->length = sizeof(struct acpi_rsdp);
	rsdp->rsdt_address = (u32)rsdt;

	/*
	 * Revision: ACPI 1.0: 0, ACPI 2.0/3.0/4.0: 2
	 *
	 * Some OSes expect an XSDT to be present for RSD PTR revisions >= 2.
	 * If we don't have an ACPI XSDT, force ACPI 1.0 (and thus RSD PTR
	 * revision 0)
	 */
	if (xsdt == NULL) {
		rsdp->revision = ACPI_RSDP_REV_ACPI_1_0;
	} else {
		rsdp->xsdt_address = (u64)(u32)xsdt;
		rsdp->revision = ACPI_RSDP_REV_ACPI_2_0;
	}

	/* Calculate checksums */
	rsdp->checksum = table_compute_checksum((void *)rsdp, 20);
	rsdp->ext_checksum = table_compute_checksum((void *)rsdp,
			sizeof(struct acpi_rsdp));
}

void acpi_fill_header(struct acpi_table_header *header, char *signature)
{
	memcpy(header->signature, signature, 4);
	memcpy(header->oem_id, OEM_ID, 6);
	memcpy(header->oem_table_id, OEM_TABLE_ID, 8);
	header->oem_revision = U_BOOT_BUILD_DATE;
	memcpy(header->aslc_id, ASLC_ID, 4);
}

static void acpi_write_rsdt(struct acpi_rsdt *rsdt)
{
	struct acpi_table_header *header = &(rsdt->header);

	/* Fill out header fields */
	acpi_fill_header(header, "RSDT");
	header->length = sizeof(struct acpi_rsdt);
	header->revision = 1;

	/* Entries are filled in later, we come with an empty set */

	/* Fix checksum */
	header->checksum = table_compute_checksum((void *)rsdt,
			sizeof(struct acpi_rsdt));
}

static void acpi_write_xsdt(struct acpi_xsdt *xsdt)
{
	struct acpi_table_header *header = &(xsdt->header);

	/* Fill out header fields */
	acpi_fill_header(header, "XSDT");
	header->length = sizeof(struct acpi_xsdt);
	header->revision = 1;

	/* Entries are filled in later, we come with an empty set */

	/* Fix checksum */
	header->checksum = table_compute_checksum((void *)xsdt,
			sizeof(struct acpi_xsdt));
}

int acpi_add_table(struct acpi_rsdp *rsdp, void *table)
{
	int i, entries_num;
	struct acpi_rsdt *rsdt;
	struct acpi_xsdt *xsdt = NULL;

	/* The RSDT is mandatory while the XSDT is not */
	rsdt = (struct acpi_rsdt *)rsdp->rsdt_address;

	if (rsdp->xsdt_address)
		xsdt = (struct acpi_xsdt *)((u32)rsdp->xsdt_address);

	/* This should always be MAX_ACPI_TABLES */
	entries_num = ARRAY_SIZE(rsdt->entry);

	for (i = 0; i < entries_num; i++) {
		if (rsdt->entry[i] == 0)
			break;
	}

	if (i >= entries_num) {
		debug("ACPI: Error: too many tables\n");
		return -E2BIG;
	}

	/* Add table to the RSDT */
	rsdt->entry[i] = (u32)table;

	/* Fix RSDT length or the kernel will assume invalid entries */
	rsdt->header.length = sizeof(struct acpi_table_header) +
				(sizeof(u32) * (i + 1));

	/* Re-calculate checksum */
	rsdt->header.checksum = 0;
	rsdt->header.checksum = table_compute_checksum((u8 *)rsdt,
			rsdt->header.length);

	/*
	 * And now the same thing for the XSDT. We use the same index as for
	 * now we want the XSDT and RSDT to always be in sync in U-Boot
	 */
	if (xsdt) {
		/* Add table to the XSDT */
		xsdt->entry[i] = (u64)(u32)table;

		/* Fix XSDT length */
		xsdt->header.length = sizeof(struct acpi_table_header) +
			(sizeof(u64) * (i + 1));

		/* Re-calculate checksum */
		xsdt->header.checksum = 0;
		xsdt->header.checksum = table_compute_checksum((u8 *)xsdt,
				xsdt->header.length);
	}

	return 0;
}

int acpi_create_mcfg_mmconfig(struct acpi_mcfg_mmconfig *mmconfig, u32 base,
			      u16 seg_nr, u8 start, u8 end)
{
	const int size = sizeof(*mmconfig);

	memset(mmconfig, '\0', size);
	mmconfig->base_address_l = base;
	mmconfig->base_address_h = 0;
	mmconfig->pci_segment_group_number = seg_nr;
	mmconfig->start_bus_number = start;
	mmconfig->end_bus_number = end;

	return size;
}

unsigned long acpi_write_hpet(struct udevice *dev, unsigned long current,
			      struct acpi_rsdp *rsdp)
{
	struct acpi_hpet *hpet;

	/*
	 * We explicitly add these tables later on:
	 */
	log_debug("ACPI:    * HPET\n");

	hpet = (struct acpi_hpet *) current;
	current += sizeof(struct acpi_hpet);
	current = ALIGN(current, 16);
	acpi_create_hpet(hpet);
	acpi_add_table(rsdp, hpet);

	return current;
}

unsigned long acpi_write_dbg2_pci_uart(struct acpi_rsdp *rsdp, ulong current,
				       struct udevice *dev, uint8_t access_size)
{
	struct acpi_dbg2_header *dbg2 = (struct acpi_dbg2_header *)current;
	phys_addr_t addr;
	struct acpi_gen_regaddr address;

	if (!dev) {
		log_err("Device not found\n");
		return current;
	}
	if (device_active(dev)) {
		log_info("Device not enabled\n");
		return current;
	}
	/*
	 * PCI devices don't remember their resource allocation information in
	 * U-Boot at present. We assume that MMIO is used for the UART and that
	 * the address space is 32 bytes: ns16550 uses 8 registers of up to
	 * 32-bits each. This is only for debugging so it is not a big deal.
	 */
	addr = dm_pci_read_bar32(dev, 0);

	memset(&address, '\0', sizeof(address));
	address.space_id = ACPI_ADDRESS_SPACE_MEMORY;
	address.addrl = (uint32_t)addr;
	address.addrh = (uint32_t)((addr >> 32) & 0xffffffff);
	address.access_size = access_size;

	acpi_create_dbg2(dbg2,
			 ACPI_DBG2_SERIAL_PORT,
			 ACPI_DBG2_16550_COMPATIBLE,
			 &address, 32,
			 acpi_device_path(dev));

	if (dbg2->header.length) {
		current += dbg2->header.length;
		current = ALIGN(current, 16);
		acpi_add_table(rsdp, dbg2);
	}

	return current;
}

static void acpi_create_facs(struct acpi_facs *facs)
{
	memset((void *)facs, 0, sizeof(struct acpi_facs));

	memcpy(facs->signature, "FACS", 4);
	facs->length = sizeof(struct acpi_facs);
	facs->hardware_signature = 0;
	facs->firmware_waking_vector = 0;
	facs->global_lock = 0;
	facs->flags = 0;
	facs->x_firmware_waking_vector_l = 0;
	facs->x_firmware_waking_vector_h = 0;
	facs->version = 1;
}

static int acpi_create_madt_lapic(struct acpi_madt_lapic *lapic,
				  u8 cpu, u8 apic)
{
	lapic->type = ACPI_APIC_LAPIC;
	lapic->length = sizeof(struct acpi_madt_lapic);
	lapic->flags = LOCAL_APIC_FLAG_ENABLED;
	lapic->processor_id = cpu;
	lapic->apic_id = apic;

	return lapic->length;
}

int acpi_create_madt_lapics(u32 current)
{
	struct udevice *dev;
	int total_length = 0;

	for (uclass_find_first_device(UCLASS_CPU, &dev);
	     dev;
	     uclass_find_next_device(&dev)) {
		struct cpu_platdata *plat = dev_get_parent_platdata(dev);
		int length = acpi_create_madt_lapic(
				(struct acpi_madt_lapic *)current,
				plat->cpu_id, plat->cpu_id);
		current += length;
		total_length += length;
	}

	return total_length;
}

int acpi_create_madt_ioapic(struct acpi_madt_ioapic *ioapic, u8 id,
			    u32 addr, u32 gsi_base)
{
	ioapic->type = ACPI_APIC_IOAPIC;
	ioapic->length = sizeof(struct acpi_madt_ioapic);
	ioapic->reserved = 0x00;
	ioapic->gsi_base = gsi_base;
	ioapic->ioapic_id = id;
	ioapic->ioapic_addr = addr;

	return ioapic->length;
}

int acpi_create_madt_irqoverride(struct acpi_madt_irqoverride *irqoverride,
				 u8 bus, u8 source, u32 gsirq, u16 flags)
{
	irqoverride->type = ACPI_APIC_IRQ_SRC_OVERRIDE;
	irqoverride->length = sizeof(struct acpi_madt_irqoverride);
	irqoverride->bus = bus;
	irqoverride->source = source;
	irqoverride->gsirq = gsirq;
	irqoverride->flags = flags;

	return irqoverride->length;
}

int acpi_create_madt_lapic_nmi(struct acpi_madt_lapic_nmi *lapic_nmi,
			       u8 cpu, u16 flags, u8 lint)
{
	lapic_nmi->type = ACPI_APIC_LAPIC_NMI;
	lapic_nmi->length = sizeof(struct acpi_madt_lapic_nmi);
	lapic_nmi->flags = flags;
	lapic_nmi->processor_id = cpu;
	lapic_nmi->lint = lint;

	return lapic_nmi->length;
}

static int acpi_create_madt_irq_overrides(u32 current)
{
	struct acpi_madt_irqoverride *irqovr;
	u16 sci_flags = MP_IRQ_TRIGGER_LEVEL | MP_IRQ_POLARITY_HIGH;
	int length = 0;

	irqovr = (void *)current;
	length += acpi_create_madt_irqoverride(irqovr, 0, 0, 2, 0);

	irqovr = (void *)(current + length);
	length += acpi_create_madt_irqoverride(irqovr, 0, 9, 9, sci_flags);

	return length;
}

__weak ulong acpi_fill_madt(ulong current)
{
	current += acpi_create_madt_lapics(current);

	current += acpi_create_madt_ioapic((struct acpi_madt_ioapic *)current,
			io_apic_read(IO_APIC_ID) >> 24, IO_APIC_ADDR, 0);

	current += acpi_create_madt_irq_overrides(current);

	return current;
}

static void acpi_create_madt(struct acpi_madt *madt)
{
	struct acpi_table_header *header = &madt->header;
	u32 current = (u32)madt + sizeof(struct acpi_madt);

	memset((void *)madt, '\0', sizeof(struct acpi_madt));

	/* Fill out header fields */
	acpi_fill_header(header, "APIC");
	header->length = sizeof(struct acpi_madt);
	header->revision = 4;

	madt->lapic_addr = LAPIC_DEFAULT_BASE;
	madt->flags = ACPI_MADT_PCAT_COMPAT;

	current = acpi_fill_madt(current);

	/* (Re)calculate length and checksum */
	header->length = current - (u32)madt;

	header->checksum = table_compute_checksum((void *)madt, header->length);
}

__weak ulong acpi_fill_mcfg(ulong current)
{
	current += acpi_create_mcfg_mmconfig
		((struct acpi_mcfg_mmconfig *)current,
		CONFIG_PCIE_ECAM_BASE, 0x0, 0x0, 255);

	return current;
}

/* MCFG is defined in the PCI Firmware Specification 3.0 */
static void acpi_create_mcfg(struct acpi_mcfg *mcfg)
{
	struct acpi_table_header *header = &(mcfg->header);
	u32 current = (u32)mcfg + sizeof(struct acpi_mcfg);

	memset((void *)mcfg, 0, sizeof(struct acpi_mcfg));

	/* Fill out header fields */
	acpi_fill_header(header, "MCFG");
	header->length = sizeof(struct acpi_mcfg);
	header->revision = 1;

	current = acpi_fill_mcfg(current);

	/* (Re)calculate length and checksum */
	header->length = current - (u32)mcfg;
	header->checksum = table_compute_checksum((void *)mcfg, header->length);
}

__weak u32 acpi_fill_csrt(u32 current)
{
	return current;
}

static void acpi_create_csrt(struct acpi_csrt *csrt)
{
	struct acpi_table_header *header = &(csrt->header);
	u32 current = (u32)csrt + sizeof(struct acpi_csrt);

	memset((void *)csrt, 0, sizeof(struct acpi_csrt));

	/* Fill out header fields */
	acpi_fill_header(header, "CSRT");
	header->length = sizeof(struct acpi_csrt);
	header->revision = 0;

	current = acpi_fill_csrt(current);

	/* (Re)calculate length and checksum */
	header->length = current - (u32)csrt;
	header->checksum = table_compute_checksum((void *)csrt, header->length);
}

static void acpi_create_spcr(struct acpi_spcr *spcr)
{
	struct acpi_table_header *header = &(spcr->header);
	struct serial_device_info serial_info = {0};
	ulong serial_address, serial_offset;
	struct udevice *dev;
	uint serial_config;
	uint serial_width;
	int access_size;
	int space_id;
	int ret = -ENODEV;

	/* Fill out header fields */
	acpi_fill_header(header, "SPCR");
	header->length = sizeof(struct acpi_spcr);
	header->revision = 2;

	/* Read the device once, here. It is reused below */
	dev = gd->cur_serial_dev;
	if (dev)
		ret = serial_getinfo(dev, &serial_info);
	if (ret)
		serial_info.type = SERIAL_CHIP_UNKNOWN;

	/* Encode chip type */
	switch (serial_info.type) {
	case SERIAL_CHIP_16550_COMPATIBLE:
		spcr->interface_type = ACPI_DBG2_16550_COMPATIBLE;
		break;
	case SERIAL_CHIP_UNKNOWN:
	default:
		spcr->interface_type = ACPI_DBG2_UNKNOWN;
		break;
	}

	/* Encode address space */
	switch (serial_info.addr_space) {
	case SERIAL_ADDRESS_SPACE_MEMORY:
		space_id = ACPI_ADDRESS_SPACE_MEMORY;
		break;
	case SERIAL_ADDRESS_SPACE_IO:
	default:
		space_id = ACPI_ADDRESS_SPACE_IO;
		break;
	}

	serial_width = serial_info.reg_width * 8;
	serial_offset = serial_info.reg_offset << serial_info.reg_shift;
	serial_address = serial_info.addr + serial_offset;

	/* Encode register access size */
	switch (serial_info.reg_shift) {
	case 0:
		access_size = ACPI_ACCESS_SIZE_BYTE_ACCESS;
		break;
	case 1:
		access_size = ACPI_ACCESS_SIZE_WORD_ACCESS;
		break;
	case 2:
		access_size = ACPI_ACCESS_SIZE_DWORD_ACCESS;
		break;
	case 3:
		access_size = ACPI_ACCESS_SIZE_QWORD_ACCESS;
		break;
	default:
		access_size = ACPI_ACCESS_SIZE_UNDEFINED;
		break;
	}

	debug("UART type %u @ %lx\n", spcr->interface_type, serial_address);

	/* Fill GAS */
	spcr->serial_port.space_id = space_id;
	spcr->serial_port.bit_width = serial_width;
	spcr->serial_port.bit_offset = 0;
	spcr->serial_port.access_size = access_size;
	spcr->serial_port.addrl = lower_32_bits(serial_address);
	spcr->serial_port.addrh = upper_32_bits(serial_address);

	/* Encode baud rate */
	switch (serial_info.baudrate) {
	case 9600:
		spcr->baud_rate = 3;
		break;
	case 19200:
		spcr->baud_rate = 4;
		break;
	case 57600:
		spcr->baud_rate = 6;
		break;
	case 115200:
		spcr->baud_rate = 7;
		break;
	default:
		spcr->baud_rate = 0;
		break;
	}

	serial_config = SERIAL_DEFAULT_CONFIG;
	if (dev)
		ret = serial_getconfig(dev, &serial_config);

	spcr->parity = SERIAL_GET_PARITY(serial_config);
	spcr->stop_bits = SERIAL_GET_STOP(serial_config);

	/* No PCI devices for now */
	spcr->pci_device_id = 0xffff;
	spcr->pci_vendor_id = 0xffff;

	/* Fix checksum */
	header->checksum = table_compute_checksum((void *)spcr, header->length);
}

void acpi_fadt_common(struct acpi_fadt *fadt, struct acpi_facs *facs,
		      void *dsdt)
{
	struct acpi_table_header *header = &fadt->header;

	memset((void *)fadt, '\0', sizeof(struct acpi_fadt));

	acpi_fill_header(header, "FACP");
	header->length = sizeof(struct acpi_fadt);
	header->revision = 4;
	memcpy(header->oem_id, OEM_ID, 6);
	memcpy(header->oem_table_id, ACPI_TABLE_CREATOR, 8);
	memcpy(header->aslc_id, ASLC_ID, 4);
	header->aslc_revision = 1;

	fadt->firmware_ctrl = (unsigned long) facs;
	fadt->dsdt = (unsigned long) dsdt;

	fadt->x_firmware_ctl_l = (unsigned long)facs;
	fadt->x_firmware_ctl_h = 0;
	fadt->x_dsdt_l = (unsigned long)dsdt;
	fadt->x_dsdt_h = 0;

// 	if (CONFIG(SYSTEM_TYPE_CONVERTIBLE) ||
// 	    CONFIG(SYSTEM_TYPE_LAPTOP))
		fadt->preferred_pm_profile = ACPI_PM_MOBILE;
// 	else if (CONFIG(SYSTEM_TYPE_DETACHABLE) ||
// 		 CONFIG(SYSTEM_TYPE_TABLET))
// 		fadt->preferred_pm_profile = PM_TABLET;
// 	else
// 		fadt->preferred_pm_profile = PM_DESKTOP;

	/* Use ACPI 3.0 revision */
	fadt->header.revision = 4;
}

unsigned long acpi_create_dmar_drhd(unsigned long current, u8 flags,
	u16 segment, u64 bar)
{
	struct dmar_entry *drhd = (struct dmar_entry *)current;

	memset(drhd, 0, sizeof(*drhd));
	drhd->type = DMAR_DRHD;
	drhd->length = sizeof(*drhd); /* will be fixed up later */
	drhd->flags = flags;
	drhd->segment = segment;
	drhd->bar = bar;

	return drhd->length;
}

unsigned long acpi_create_dmar_rmrr(unsigned long current, u16 segment,
				    u64 bar, u64 limit)
{
	struct dmar_rmrr_entry *rmrr = (struct dmar_rmrr_entry *)current;

	memset(rmrr, 0, sizeof(*rmrr));
	rmrr->type = DMAR_RMRR;
	rmrr->length = sizeof(*rmrr); /* will be fixed up later */
	rmrr->segment = segment;
	rmrr->bar = bar;
	rmrr->limit = limit;

	return rmrr->length;
}

void acpi_dmar_drhd_fixup(unsigned long base, unsigned long current)
{
	struct dmar_entry *drhd = (struct dmar_entry *)base;

	drhd->length = current - base;
}

void acpi_dmar_rmrr_fixup(unsigned long base, unsigned long current)
{
	struct dmar_rmrr_entry *rmrr = (struct dmar_rmrr_entry *)base;

	rmrr->length = current - base;
}

int acpi_create_dmar(struct acpi_dmar *dmar, enum dmar_flags flags)
{
	struct acpi_table_header *header = &dmar->header;

	memset((void *)dmar, 0, sizeof(struct acpi_dmar));
	if (!header)
		return -EFAULT;

	/* Fill out header fields. */
	memcpy(header->signature, "DMAR", 4);
	memcpy(header->oem_id, OEM_ID, 6);
	memcpy(header->oem_table_id, ACPI_TABLE_CREATOR, 8);
	memcpy(header->aslc_id, ASLC_ID, 4);

	header->aslc_revision = ASL_REVISION;
	header->length = sizeof(struct acpi_dmar);
	header->revision = get_acpi_table_revision(DMAR);

	dmar->host_address_width = cpu_phys_address_size() - 1;
	dmar->flags = flags;

	return 0;
}

static unsigned long acpi_create_dmar_ds(unsigned long current,
	enum dev_scope_type type, u8 enumeration_id, u8 bus, u8 dev, u8 fn)
{
	/* we don't support longer paths yet */
	const size_t dev_scope_length = sizeof(struct dev_scope) + 2;

	struct dev_scope *ds = (struct dev_scope *)current;
	memset(ds, 0, dev_scope_length);
	ds->type	= type;
	ds->length	= dev_scope_length;
	ds->enumeration	= enumeration_id;
	ds->start_bus	= bus;
	ds->path[0].dev	= dev;
	ds->path[0].fn	= fn;

	return ds->length;
}

unsigned long acpi_create_dmar_ds_pci_br(unsigned long current, u8 bus,
	u8 dev, u8 fn)
{
	return acpi_create_dmar_ds(current,
			SCOPE_PCI_SUB, 0, bus, dev, fn);
}

unsigned long acpi_create_dmar_ds_pci(unsigned long current, u8 bus,
	u8 dev, u8 fn)
{
	return acpi_create_dmar_ds(current,
			SCOPE_PCI_ENDPOINT, 0, bus, dev, fn);
}

unsigned long acpi_create_dmar_ds_ioapic(unsigned long current,
	u8 enumeration_id, u8 bus, u8 dev, u8 fn)
{
	return acpi_create_dmar_ds(current,
			SCOPE_IOAPIC, enumeration_id, bus, dev, fn);
}

unsigned long acpi_create_dmar_ds_msi_hpet(unsigned long current,
	u8 enumeration_id, u8 bus, u8 dev, u8 fn)
{
	return acpi_create_dmar_ds(current,
			SCOPE_MSI_HPET, enumeration_id, bus, dev, fn);
}

/* http://www.intel.com/hardwaredesign/hpetspec_1.pdf */
int acpi_create_hpet(struct acpi_hpet *hpet)
{
	struct acpi_table_header *header = &hpet->header;
	struct acpi_gen_regaddr *addr = &hpet->addr;

	memset((void *)hpet, 0, sizeof(struct acpi_hpet));
	if (!header)
		return -EFAULT;

	/* Fill out header fields. */
	acpi_fill_header(header, "HPET");

	header->aslc_revision = ASL_REVISION;
	header->length = sizeof(struct acpi_hpet);
	header->revision = get_acpi_table_revision(HPET);

	/* Fill out HPET address. */
	addr->space_id = 0; /* Memory */
	addr->bit_width = 64;
	addr->bit_offset = 0;
	addr->addrl = CONFIG_HPET_ADDRESS & 0xffffffff;
	addr->addrh = ((unsigned long long)CONFIG_HPET_ADDRESS) >> 32;

	hpet->id = *(unsigned int *)CONFIG_HPET_ADDRESS;
	hpet->number = 0;
	hpet->min_tick = 0x80; /* HPET_MIN_TICKS */

	header->checksum = acpi_checksum((void *)hpet,
					 sizeof(struct acpi_hpet));

	return 0;
}

void acpi_create_dbg2(struct acpi_dbg2_header *dbg2,
		      int port_type, int port_subtype,
		      struct acpi_gen_regaddr *address, uint32_t address_size,
		      const char *device_path)
{
	uintptr_t current;
	struct acpi_dbg2_device *device;
	uint32_t *dbg2_addr_size;
	struct acpi_table_header *header;
	size_t path_len;
	const char *path;
	char *namespace;

	/* Fill out header fields. */
	current = (uintptr_t)dbg2;
	memset(dbg2, 0, sizeof(struct acpi_dbg2_header));
	header = &dbg2->header;

	if (!header)
		return;

	header->revision = get_acpi_table_revision(DBG2);
	acpi_fill_header(header, "DBG2");
	header->aslc_revision = ASL_REVISION;

	/* One debug device defined */
	dbg2->devices_offset = sizeof(struct acpi_dbg2_header);
	dbg2->devices_count = 1;
	current += sizeof(struct acpi_dbg2_header);

	/* Device comes after the header */
	device = (struct acpi_dbg2_device *)current;
	memset(device, 0, sizeof(struct acpi_dbg2_device));
	current += sizeof(struct acpi_dbg2_device);

	device->revision = 0;
	device->address_count = 1;
	device->port_type = port_type;
	device->port_subtype = port_subtype;

	/* Base Address comes after device structure */
	memcpy((void *)current, address, sizeof(struct acpi_gen_regaddr));
	device->base_address_offset = current - (uintptr_t)device;
	current += sizeof(struct acpi_gen_regaddr);

	/* Address Size comes after address structure */
	dbg2_addr_size = (uint32_t *)current;
	device->address_size_offset = current - (uintptr_t)device;
	*dbg2_addr_size = address_size;
	current += sizeof(uint32_t);

	/* Namespace string comes last, use '.' if not provided */
	path = device_path ? : ".";
	/* Namespace string length includes NULL terminator */
	path_len = strlen(path) + 1;
	namespace = (char *)current;
	device->namespace_string_length = path_len;
	device->namespace_string_offset = current - (uintptr_t)device;
	strncpy(namespace, path, path_len);
	current += path_len;

	/* Update structure lengths and checksum */
	device->length = current - (uintptr_t)device;
	header->length = current - (uintptr_t)dbg2;
	header->checksum = acpi_checksum((uint8_t *)dbg2, header->length);
}

ulong acpi_get_rsdp_addr(void)
{
	return acpi_rsdp_addr;
}

int get_acpi_table_revision(enum acpi_tables table)
{
	switch (table) {
	case FADT:
		return ACPI_FADT_REV_ACPI_3_0;
	case MADT: /* ACPI 3.0: 2, ACPI 4.0/5.0: 3, ACPI 6.2b/6.3: 5 */
		return 2;
	case MCFG:
		return 1;
	case TCPA:
		return 2;
	case TPM2:
		return 4;
	case SSDT: /* ACPI 3.0 upto 6.3: 2 */
		return 2;
	case SRAT: /* ACPI 2.0: 1, ACPI 3.0: 2, ACPI 4.0 upto 6.3: 3 */
		return 1; /* TODO Should probably be upgraded to 2 */
	case DMAR:
		return 1;
	case SLIT: /* ACPI 2.0 upto 6.3: 1 */
		return 1;
	case SPMI: /* IMPI 2.0 */
		return 5;
	case HPET: /* Currently 1. Table added in ACPI 2.0. */
		return 1;
	case VFCT: /* ACPI 2.0/3.0/4.0: 1 */
		return 1;
	case IVRS:
		return IVRS_FORMAT_FIXED;
	case DBG2:
		return 0;
	case FACS: /* ACPI 2.0/3.0: 1, ACPI 4.0 upto 6.3: 2 */
		return 1;
	case RSDT: /* ACPI 1.0 upto 6.3: 1 */
		return 1;
	case XSDT: /* ACPI 2.0 upto 6.3: 1 */
		return 1;
	case RSDP: /* ACPI 2.0 upto 6.3: 2 */
		return 2;
	case HEST:
		return 1;
	case NHLT:
		return 5;
	case BERT:
		return 1;
	default:
		return -EINVAL;
	}
}

/*
 * QEMU's version of write_acpi_tables is defined in drivers/misc/qfw.c
 */
ulong write_acpi_tables(ulong start)
{
	u32 current;
	struct acpi_rsdp *rsdp;
	struct acpi_rsdt *rsdt;
	struct acpi_xsdt *xsdt;
	struct acpi_facs *facs;
	struct acpi_table_header *dsdt;
	struct acpi_fadt *fadt;
	struct acpi_mcfg *mcfg;
	struct acpi_madt *madt;
	struct acpi_csrt *csrt;
	struct acpi_spcr *spcr;
	int i;

	current = start;

	/* Align ACPI tables to 16 byte */
	current = ALIGN(current, 16);

	debug("ACPI: Writing ACPI tables at %lx\n", start);

	/* We need at least an RSDP and an RSDT Table */
	rsdp = (struct acpi_rsdp *)current;
	current += sizeof(struct acpi_rsdp);
	current = ALIGN(current, 16);
	rsdt = (struct acpi_rsdt *)current;
	current += sizeof(struct acpi_rsdt);
	current = ALIGN(current, 16);
	xsdt = (struct acpi_xsdt *)current;
	current += sizeof(struct acpi_xsdt);
	/*
	 * Per ACPI spec, the FACS table address must be aligned to a 64 byte
	 * boundary (Windows checks this, but Linux does not).
	 */
	current = ALIGN(current, 64);

	/* clear all table memory */
	memset((void *)start, 0, current - start);

	acpi_write_rsdp(rsdp, rsdt, xsdt);
	acpi_write_rsdt(rsdt);
	acpi_write_xsdt(xsdt);

	debug("ACPI:    * FACS\n");
	facs = (struct acpi_facs *)current;
	current += sizeof(struct acpi_facs);
	current = ALIGN(current, 16);

	acpi_create_facs(facs);

	debug("ACPI:    * DSDT\n");
	dsdt = (struct acpi_table_header *)current;
	memcpy(dsdt, &AmlCode, sizeof(struct acpi_table_header));
	current += sizeof(struct acpi_table_header);
	memcpy((char *)current,
	       (char *)&AmlCode + sizeof(struct acpi_table_header),
	       dsdt->length - sizeof(struct acpi_table_header));
	current += dsdt->length - sizeof(struct acpi_table_header);
	current = ALIGN(current, 16);

	/* Pack GNVS into the ACPI table area */
	for (i = 0; i < dsdt->length; i++) {
		u32 *gnvs = (u32 *)((u32)dsdt + i);
		if (*gnvs == ACPI_GNVS_ADDR) {
			debug("Fix up global NVS in DSDT to 0x%08x\n", current);
			*gnvs = current;
			break;
		}
	}

	/* Update DSDT checksum since we patched the GNVS address */
	dsdt->checksum = 0;
	dsdt->checksum = table_compute_checksum((void *)dsdt, dsdt->length);

	/* Fill in platform-specific global NVS variables */
	acpi_create_gnvs((struct acpi_global_nvs *)current);
	current += sizeof(struct acpi_global_nvs);
	current = ALIGN(current, 16);

	debug("ACPI:    * FADT\n");
	fadt = (struct acpi_fadt *)current;
	current += sizeof(struct acpi_fadt);
	current = ALIGN(current, 16);
	acpi_create_fadt(fadt, facs, dsdt);
	acpi_add_table(rsdp, fadt);

	debug("ACPI:    * MADT\n");
	madt = (struct acpi_madt *)current;
	acpi_create_madt(madt);
	current += madt->header.length;
	acpi_add_table(rsdp, madt);
	current = ALIGN(current, 16);

	debug("ACPI:    * MCFG\n");
	mcfg = (struct acpi_mcfg *)current;
	acpi_create_mcfg(mcfg);
	current += mcfg->header.length;
	acpi_add_table(rsdp, mcfg);
	current = ALIGN(current, 16);

	debug("ACPI:    * CSRT\n");
	csrt = (struct acpi_csrt *)current;
	acpi_create_csrt(csrt);
	current += csrt->header.length;
	acpi_add_table(rsdp, csrt);
	current = ALIGN(current, 16);

	debug("ACPI:    * SPCR\n");
	spcr = (struct acpi_spcr *)current;
	acpi_create_spcr(spcr);
	current += spcr->header.length;
	acpi_add_table(rsdp, spcr);
	current = ALIGN(current, 16);

	printf("current = %x\n", current);

	acpi_rsdp_addr = (unsigned long)rsdp;
	debug("ACPI: done\n");

	return current;
}
