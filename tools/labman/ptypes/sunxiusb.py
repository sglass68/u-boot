# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

import re
import time

import collections
from labman.ptypes.usbboot import Part_usbboot
from labman import work

class Part_sunxiusb(Part_usbboot):
    """A sunxi FEL connection

    This allows downloading software to the DUT oevr USB using the boot ROM

    Properties:
        _serial: Serial number of the Ykusb
        _symlink: Symlink to the device
    """
    def __init__(self):
        super().__init__()
        self.vendor = 0x1f3a
        self.product = 0xefe8

    def __str__(self):
        """Convert the object into a string for presentation to the user"""
        return 'sunxi %s' % self.name

    def raise_self(self, msg):
        """Raise an error related to this Tegra RCM connection

        Args:
            msg (str): Message to report

        Raises:
            ValueError: always
        """
        raise ValueError('%s: %s' % (str(self), msg))

    def sunxi_fel(self, *in_args):
        """Perform an operation with the sunxi-fel tool

        Args:
            in_args: Arguments to pass to the tool

        Returns:
            CommandResult: Result from command

        Raises:
            ValueError: if the tool failed
        """
        # This tool prints everything on stderr
        args = ['sunxi-fel'] + list(in_args)
        result = self.lab.run_command(*args)
        if result.return_code:
            self.raise_self("Failed to run '%s'" % ' '.join(args))
        return result.stderr
