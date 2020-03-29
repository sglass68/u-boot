# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

from abc import ABCMeta, abstractmethod

class Power(metaclass=ABCMeta):
    @abstractmethod
    def set_power(self, power_on: bool, port: int):
        """Port a port on or off

        Args:
            power_on (bool): True to power on, False to power off
            port (int): Port number to change

        Raises:
            ValueError: if it did not respond
        """
        pass
