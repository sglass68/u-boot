# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) 2016 Google, Inc
# Written by Simon Glass <sjg@chromium.org>
#
# Entry-type module for 'u-boot-tpl-nodtb.bin'
#

from entry import Entry
from blob import Entry_blob

class Entry_u_boot_tpl_nodtb(Entry_blob):
    """TPL binary without device tree appended

    Properties / Entry arguments:
        - filename: Filename of tpl/u-boot-tpl-nodtb.bin (default
            'tpl/u-boot-tpl-nodtb.bin')

    This is the U-Boot TPL binary, It does not include a device tree blob at
    the end of it so may not be able to work without it, assuming TPL needs
    a device tree to operation on your platform. You can add a u_boot_tpl_dtb
    entry after this one, or use a u_boot_tpl entry instead (which contains
    both TPL and the device tree).
    """
    def __init__(self, section, etype, node):
        Entry_blob.__init__(self, section, etype, node)

    def GetDefaultFilename(self):
        return 'tpl/u-boot-tpl-nodtb.bin'
