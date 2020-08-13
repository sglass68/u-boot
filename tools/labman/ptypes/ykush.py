# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

import re

from labman.part import Part
from labman import work

class Part_ykush(Part):
    """A USB relay which can provide a console connection to a DUT

    Properties:
        name: Name of the uart. This should be short, ideally a nickname.
        lab: Lab associated with this uart, or None if none
        _serial: Serial number of the Ykusb
        _symlink: Symlink to the device
    """
    def load(self, yam):
        """Load the object from a yaml definition

        Args:
            yam (dict): Yaml definition
        """
        self._serial = yam.get('serial')
        if self._serial is None:
            self.raise_self("Missing serial")
        self._symlink = yam.get('symlink')
        self.load_usb_port(yam)
        self.power_hub, self._power_phys_port = self.lookup_usb_port(
            yam, 'hub-power-port')
        self.num_ports = 3

    def __str__(self):
        """Convert the object into a string for presentation to the user"""
        return 'ykush %s' % self.name

    def raise_self(self, msg):
        """Raise an error related to this Ykusb

        Args:
            msg (str): Message to report

        Raises:
            ValueError: always
        """
        raise ValueError('%s: %s' % (str(self), msg))

    def get_detail(self, port=None):
        if self.power_hub.port == port:
            return '%s, power' % self._serial
        elif self.hub.port == port:
            return '%s, %d ports' % (self._serial, self.num_ports)
        else:
            return '(internal port error)'

    def ykush(self, *in_args):
        """Perform an operation with the ykush tool

        The output format from the tool is either:

            Downstream port <port> is <state>

        e.g.

            Downstream port 1 is ON

        or if something goes wrong there are multiple lines, with the first one
        being the error (note it still prints the port status even on error):

            Unable to open device


            Downstream port 15 is ON

        Args:
            in_args: Arguments to pass to the tool

        Returns:
            CommandResult: Result from command

        Raises:
            ValueError: if the tool failed
        """
        # Sadly this tool does not seem to return a useful return code, nor use
        # stderr
        args = ['ykushcmd'] + list(in_args)
        result = self.lab.run_command(*args)
        if result.return_code:
            self.raise_self("Failed to run '%s': %d: %s" %
                   (' '.join(args), result.return_code, result.stderr))
        lines = result.stdout.strip().splitlines()
        if len(lines) > 1:
            self.raise_self(lines[0])
        return lines[0] if lines else ''

    def get_status(self, port):
        """Get the relay status for a port

        Args:
            port (int): Port number to check (1-3)

        Returns:
            True if the port is on, False if off

        Raises:
            ValueError: if it did not respond
        """
        out = self.ykush('-s', self._serial, '-g', str(port))
        rem = re.search(r'Downstream port ([0-9]) is (ON|OFF)', out)
        if not rem:
            self.raise_self("Unable to find status in '%s'" % out)
        check_port = rem.group(1)
        if int(check_port) != port:
            self.raise_self("Expected port %d but port %s returned" %
                            (port, check_port))
        return rem.group(2) == 'ON'

    def set_power(self, power_on: bool, port: int):
        """Port a port on or off

        Args:
            power_on (bool): True to power on, False to power off
            port (int): Port number to change (1-3)

        Raises:
            ValueError: if it did not respond
        """
        self.ykush('-s', self._serial, '-u' if power_on else '-d', str(port))

    def check(self):
        try:
            enabled = self.get_status(1)
            #self.set_power(not enabled, 1)
            #new_enabled = self.get_status(1)
            #if enabled == new_enabled:
                #self.raise_self("Unable to toggle power state for port %d" % 1)
            #self.set_power(enabled, 1)
            good = True
            msg = ''
        except ValueError as exc:
            msg = str(exc).strip()
            good = False
        return work.CheckResult(self, good, msg)

    def emit(self):
        emit_list = []
        emit_list.append(self.emit_udev())
        return emit_list

    def emit_udev(self):
        vals = {
            'name': str(self),
            'serial': self._serial,
            'symlink': self._symlink,
            }
        out = '''# {name}
ACTION=="add|bind" \\
, SUBSYSTEM=="usb" \\
, ATTR{{idProduct}}=="f2f7" \\
, ATTR{{idVendor}}=="04d8" \\
, ATTR{{serial}}=="{serial}" \\
, MODE="0666" \\
, SYMLINK+="{symlink}"

'''.format(**vals)
        return work.EmitResult(self, 'udev/99-labman-yepkit.rules', out,
                               '# Generated by labman')

    def get_py_imports(self):
        return 'from ykush import Ykush'

    def get_py_base_class(self):
        return 'Ykush'

    def get_py_class_vars(self, part_ref):
        return {
            'ykush_serial': self._serial,
            'ykush_port': str(part_ref.port),
            }

    def get_poweron(self):
        return 'self.ykush_on()'

    def get_poweroff(self):
        return 'self.ykush_off()'

    def get_reset(self):
        return 'self.ykush_reset()'

    def get_code(self, prop, prop_list, partref):
        if prop == 'poweron':
            # The 'reset' method does this so we don't need to
            if 'reset' in prop_list:
                return None
            return 'self.ykush_on()'
        elif prop == 'poweroff':
            return 'self.ykush_off()'
        elif prop == 'reset':
            return 'self.ykush_reset()'
        elif prop == 'baseclass':
            return 'Ykush'
        elif prop == 'setpower':
            return 'self.ykush_set_power(True)'
        elif prop == 'clearpower':
            return 'self.ykush_set_power(False)'
        elif prop == 'delay':
            return 'self.ykush_delay()'
