# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) 2016 Google, Inc
# Written by Simon Glass <sjg@chromium.org>
#
# Entry-type module for 'u-boot-vpl-nodtb.bin'
#

from entry import Entry
from blob import Entry_blob

class Entry_u_boot_vpl_nodtb(Entry_blob):
    """VPL binary without device tree appended

    Properties / Entry arguments:
        - filename: Filename of vpl/u-boot-vpl-nodtb.bin (default
            'vpl/u-boot-vpl-nodtb.bin')

    This is the U-Boot VPL binary, It does not include a device tree blob at
    the end of it so may not be able to work without it, assuming VPL needs
    a device tree to operation on your platform. You can add a u_boot_vpl_dtb
    entry after this one, or use a u_boot_vpl entry instead (which contains
    both VPL and the device tree).
    """
    def __init__(self, section, etype, node):
        Entry_blob.__init__(self, section, etype, node)

    def GetDefaultFilename(self):
        return 'vpl/u-boot-vpl-nodtb.bin'
