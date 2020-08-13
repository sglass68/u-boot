# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

from labman.ptypes.usbboot import Part_usbboot
from labman import work

class Part_at91usb(Part_usbboot):
    """An Atmel AT91 USB connection

    Properties:
    """
    def __init__(self):
        super().__init__()
        self.vendor = 0x03eb
        self.product = 0x6124

    def __str__(self):
        """Convert the object into a string for presentation to the user"""
        return 'at91usb %s' % self.name

    def get_detail(self, port=None):
        return '%s, at91usb' % self._symlink
