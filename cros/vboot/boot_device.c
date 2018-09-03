// SPDX-License-Identifier: GPL-2.0+
/*
 * Implements boot devices, typically MMC, used to hold the kernel
 *
 * Copyright 2018 Google LLC
 */

#include <common.h>
#include <bootstage.h>
#include <dm.h>
#include <part.h>
#include <usb.h>
#include <cros/cros_common.h>
#include <cros/vboot.h>

/* Maximum number of devices we can support */
enum {
	MAX_DISK_INFO	= 10,
};

/**
 * Add a device to info array if it matches the supplied disk_flags
 *
 * @name: Peripheral name
 * @dev: Device to check
 * @req_flags: Requested flags which must be present for each device
 * @info: Output array of matching devices
 * @return 0 if added, -ENOENT if not
 */
static int add_matching_device(struct udevice *dev, u32 req_flags,
			       VbDiskInfo *info)
{
	struct blk_desc *bdev = dev_get_uclass_platdata(dev);
	u32 flags;

	/* Ignore zero-length devices */
	if (!bdev->lba)
		return -ENOENT;

	/*
	 * Only add this storage device if the properties of req_flags is a
	 * subset of the properties of flags.
	 */
	flags = bdev->removable ? VB_DISK_FLAG_REMOVABLE : VB_DISK_FLAG_FIXED;
	if ((flags & req_flags) != req_flags)
		return -ENOENT;

	info->handle = (VbExDiskHandle_t)dev;
	info->bytes_per_lba = bdev->blksz;
	info->lba_count = bdev->lba;
	info->flags = flags | VB_DISK_FLAG_EXTERNAL_GPT;
	info->name = dev->name;

	return 0;
}

static int boot_device_usb_start(struct vboot_info *vboot)
{
	int enumerate = 1;

	/*
	 * if the USB devices have already been enumerated, redo it
	 * only if something has been plugged on unplugged.
	 */
	if (vboot->usb_is_enumerated)
		enumerate = usb_detect_change();

	if (enumerate) {
		/*
		 * We should stop all USB devices first. Otherwise we can't
		 * detect any new devices.
		 */
		usb_stop();

		if (usb_init() >= 0)
			vboot->usb_is_enumerated = 1;
	}

	return 0;
}

VbError_t VbExDiskGetInfo(VbDiskInfo **infos_ptr, u32 *count_ptr,
			  u32 disk_flags)
{
	struct vboot_info *vboot = vboot_get();
	VbDiskInfo *infos;
	u32 max_count;	/* maximum devices to scan for */
	u32 count = 0;	/* number of matching devices found */
	struct udevice *dev;

	/* We return as many disk infos as possible */
	max_count = MAX_DISK_INFO;

	infos = calloc(max_count, sizeof(VbDiskInfo));

	bootstage_start(BOOTSTAGE_ACCUM_VBOOT_BOOT_DEVICE_INFO,
			"boot_device_info");

	/* If we are looking for removable disks, scan USB */
	if (disk_flags & VB_DISK_FLAG_REMOVABLE)
		boot_device_usb_start(vboot);

	/* Scan through all the interfaces looking for devices */
	uclass_foreach_dev_probe(UCLASS_BLK, dev) {
		/* Now record the devices that have the required flags */
		if (!add_matching_device(dev, disk_flags, infos + count))
			count++;
		if (count == max_count)
			log_warning("Reached maximum device count\n");
	}

	if (count) {
		*infos_ptr = infos;
		*count_ptr = count;
	} else {
		*infos_ptr = NULL;
		*count_ptr = 0;
		free(infos);
	}
	bootstage_accum(BOOTSTAGE_ACCUM_VBOOT_BOOT_DEVICE_INFO);

	/* The operation itself succeeds, despite scan failure all about */
	return VBERROR_SUCCESS;
}

VbError_t VbExDiskFreeInfo(VbDiskInfo *infos, VbExDiskHandle_t preserve_handle)
{
	/* We do nothing for preserve_handle as we keep all the devices on */
	free(infos);
	return VBERROR_SUCCESS;
}

VbError_t VbExDiskRead(VbExDiskHandle_t handle, u64 lba_start, u64 lba_count,
		       void *buffer)
{
	struct udevice *dev = (struct udevice *)handle;
	struct blk_desc *bdev = dev_get_uclass_platdata(dev);
	u64 blks_read;

	log_debug("lba_start=%u, lba_count=%u\n", (uint)lba_start,
		  (uint)lba_count);

	if (lba_start >= bdev->lba || lba_start + lba_count > bdev->lba)
		return VBERROR_UNKNOWN;

	bootstage_start(BOOTSTAGE_ACCUM_VBOOT_BOOT_DEVICE_READ,
			"boot_device_read");
	blks_read = blk_dread(bdev, lba_start, lba_count, buffer);
	bootstage_accum(BOOTSTAGE_ACCUM_VBOOT_BOOT_DEVICE_READ);
	if (blks_read != lba_count)
		return VBERROR_UNKNOWN;

	return VBERROR_SUCCESS;
}

VbError_t VbExDiskWrite(VbExDiskHandle_t handle, u64 lba_start,
			u64 lba_count, const void *buffer)
{
	struct udevice *dev = (struct udevice *)handle;
	struct blk_desc *bdev = dev_get_uclass_platdata(dev);

	if (lba_start >= bdev->lba || lba_start + lba_count > bdev->lba)
		return VBERROR_UNKNOWN;

	if (blk_dwrite(bdev, lba_start, lba_count, buffer) != lba_count)
		return VBERROR_UNKNOWN;

	return VBERROR_SUCCESS;
}

/*
 * Simple implementation of new streaming APIs.  This turns them into calls to
 * the sector-based disk read/write functions above.  This will be replaced
 * imminently with fancier streaming.  In the meantime, this will allow the
 * vboot_reference change which uses the streaming APIs to commit.
 */

/* The stub implementation assumes 512-byte disk sectors */
#define LBA_BYTES 512

/* Internal struct to simulate a stream for sector-based disks */
struct disk_stream {
	/* Disk handle */
	VbExDiskHandle_t handle;

	/* Next sector to read */
	u64 sector;

	/* Number of sectors left in partition */
	u64 sectors_left;
};

VbError_t VbExStreamOpen(VbExDiskHandle_t handle, u64 lba_start,
			 u64 lba_count, VbExStream_t *stream)
{
	struct disk_stream *s;

	if (!handle) {
		*stream = NULL;
		return VBERROR_UNKNOWN;
	}

	s = malloc(sizeof(*s));
	s->handle = handle;
	s->sector = lba_start;
	s->sectors_left = lba_count;

	*stream = (void *)s;

	return VBERROR_SUCCESS;
}

VbError_t VbExStreamRead(VbExStream_t stream, u32 bytes, void *buffer)
{
	struct disk_stream *s = (struct disk_stream *)stream;
	u64 sectors;
	VbError_t rv;

	if (!s)
		return VBERROR_UNKNOWN;

	/* For now, require reads to be a multiple of the LBA size */
	if (bytes % LBA_BYTES)
		return VBERROR_UNKNOWN;

	/* Fail on overflow */
	sectors = bytes / LBA_BYTES;
	if (sectors > s->sectors_left)
		return VBERROR_UNKNOWN;

	rv = VbExDiskRead(s->handle, s->sector, sectors, buffer);
	if (rv != VBERROR_SUCCESS)
		return rv;

	s->sector += sectors;
	s->sectors_left -= sectors;

	return VBERROR_SUCCESS;
}

void VbExStreamClose(VbExStream_t stream)
{
	struct disk_stream *s = (struct disk_stream *)stream;

	free(s);
}
