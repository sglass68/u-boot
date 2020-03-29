# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

import re
import time

import collections
from labman.ptypes.usbboot import Part_usbboot
from labman import work

class Part_tegrarcm(Part_usbboot):
    """A tegra RCM (Remote Controller and Monitoring) connection

    This allows downloading software to the DUT oevr USB using the boot ROM

    Properties:
        _serial: Serial number of the Ykusb
        _symlink: Symlink to the device
    """
    def __init__(self):
        super().__init__()
        self.vendor = 0x0955

    def load(self, yam):
        """Load the object from a yaml definition

        Args:
            yam (dict): Yaml definition
        """
        super().load(yam)
        self.bct = yam.get('bct')

    def __str__(self):
        """Convert the object into a string for presentation to the user"""
        return 'tegrarcm %s' % self.name

    def raise_self(self, msg):
        """Raise an error related to this Tegra RCM connection

        Args:
            msg (str): Message to report

        Raises:
            ValueError: always
        """
        raise ValueError('%s: %s' % (str(self), msg))

    def tegrarcm(self, *in_args):
        """Perform an operation with the tegrarcm tool

        Args:
            in_args: Arguments to pass to the tool

        Returns:
            CommandResult: Result from command

        Raises:
            ValueError: if the tool failed
        """
        # This tool prints everything on stderr
        args = ['tegrarcm'] + list(in_args)
        result = self.lab.run_command(*args)
        if result.return_code:
            self.raise_self("Failed to run '%s'" % ' '.join(args))
        return result.stderr

    def get_py_class_vars(self, part_ref):
        out = super().get_py_class_vars(part_ref)
        out.update({
            'tegra_bct': self.bct,
            })
        return out
