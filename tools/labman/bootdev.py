# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

"""Implement support for boot devices"""

from abc import ABCMeta, abstractmethod

from labman.ptypes.usbboot import Part_usbboot
from labman.part import Part
from labman import work

class Part_bootdev(Part, metaclass=ABCMeta):
    """Base class for a boot device, e.g. Sdwire
    """
    @abstractmethod
    def select_ts(self):
        pass
