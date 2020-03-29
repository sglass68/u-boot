# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

import re

import dlipower

from labman.part import Part
from labman.power import Power
from labman import work
from patman import tout

class Part_dli(Part, Power):
    """A Internet-controlled power switch for powering DUTs on/off

    Properties:
        _ip: Serial number of the Ykusb
        _user: User name to log in with
        _password: Password to log in with
    """
    def load(self, yam):
        """Load the object from a yaml definition

        Args:
            yam (dict): Yaml definition
        """
        self._hostname = yam.get('hostname')
        if self._hostname is None:
            self.raise_self("Missing hostname")

        self._user = yam.get('user')
        if self._user is None:
            self.raise_self("Missing user")

        self._password = yam.get('password')
        if self._password is None:
            self.raise_self("Missing password")
        if isinstance(self._password, int):
            self._password = str(self._password)
        self.num_ports = 8
        self.switch = dlipower.PowerSwitch(hostname=self._hostname,
            userid=self._user, password=self._password)

    def __str__(self):
        """Convert the object into a string for presentation to the user"""
        return 'dli %s' % self.name

    def raise_self(self, msg):
        """Raise an error related to this Ykusb

        Args:
            msg (str): Message to report

        Raises:
            ValueError: always
        """
        raise ValueError('%s: %s' % (str(self), msg))

    def get_detail(self, port=None):
        return '%s, %d ports' % (self._hostname, self.num_ports)

    def set_power(self, power_on: bool, port: int):
        """Port a port on or off

        Args:
            port (int): Port number to change (1-8)
            power_on (bool): True to power on, False to power off

        Raises:
            ValueError: if it did not respond
        """
        if power_on:
            tout.Detail('%s-%s: Power on' % (str(self), port))
            self.switch.on(port)
        else:
            tout.Detail('%s-%s: Power off' % (str(self), port))
            self.switch.off(port)

    def check(self):
        """Run a check on a dli to see that it seems to work OK

        Returns:
            work.CheckResult: Result obtained from the check
        """
        if self.switch.verify():
            good = True
            msg = ''
        else:
            msg = 'Cannot contact switch at %s' % self._hostname
            good = False
        return work.CheckResult(self, good, msg)

    def get_py_imports(self):
        return 'from dli import Dli'

    def get_py_base_class(self):
        return 'Dli'

    def get_py_class_vars(self, part_ref):
        return {
            'dli_hostname': self._hostname,
            'dli_outlet': part_ref.port,
            'dli_user': self._user,
            'dli_password': self._password,
            }

    def get_poweron(self):
        return 'self.dli_on()'

    def get_poweroff(self):
        return 'self.dli_off()'

    def get_reset(self):
        return 'self.dli_reset()'

    def get_code(self, prop, prop_list, partref):
        if prop == 'poweron':
            # The 'reset' method does this so we don't need to
            if 'reset' in prop_list:
                return None
            return 'self.dli_on()'
        elif prop == 'poweroff':
            return 'self.dli_off()'
        elif prop == 'reset':
            return 'self.dli_reset()'
        elif prop == 'baseclass':
            return 'Dli'
