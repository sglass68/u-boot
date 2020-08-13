# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

import re

import collections
from labman.part import Part
from labman.power import Power
from labman import work

UsbrelayPort = collections.namedtuple('UsbrelayPort', 'name,portnum')

class Part_usbrelay(Part, Power):
    """A USB relay which can drive a signal high or low

    Properties:
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
            self.raise_self('Missing serial')
        self.load_usb_port(yam)
        self._symlink = yam.get('symlink')
        self._ports = {}
        self.num_ports = yam.get('num-ports')
        if not self.num_ports:
            self.raise_self('Missing num-ports')
        if 'ports' not in yam:
            self.raise_self("Missing post list")
        for name, port_yam in yam.get('ports').items():
            portnum = port_yam.get('port-number')
            if portnum is None:
                self.raise_self("Missing post-number")
            self._ports[name] = UsbrelayPort(name, portnum)

    def __str__(self):
        """Convert the object into a string for presentation to the user"""
        return 'usbrelay %s' % self.name

    def raise_self(self, msg):
        """Raise an error related to this Usbserial

        Args:
            msg (str): Message to report

        Raises:
            ValueError: always
        """
        raise ValueError('%s: %s' % (str(self), msg))

    def get_detail(self, port=None):
        return '%s, %d ports' % (self._serial, self.num_ports)

    @classmethod
    def usbrelay_cls(cls, lab, raiser, *in_args):
        """Perform an operation with the usbrelay tool

        Args:
            in_args: Arguments to pass to the tool

        Returns:
            CommandResult: Result from command

        Raises:
            ValueError: if the tool failed
        """
        # The old tool prints everything on stderr
        args = ['usbrelay'] + list(in_args)
        result = lab.run_command(*args)
        if result.return_code:
            raiser("Failed to run '%s': %d" %
                   (' '.join(args), result.return_code))
        if not result.stderr:
            args = ['usbrelay', '-d'] + list(in_args)
            result = lab.run_command(*args)
            if result.return_code:
                raiser("Failed to run '%s': %d" %
                       (' '.join(args), result.return_code))
            return result.stdout, result.stderr
        return '', result.stderr

    def usbrelay(self, *in_args):
        """Perform an operation with the usbrelay tool

        Args:
            in_args: Arguments to pass to the tool

        Returns:
            CommandResult: Result from command

        Raises:
            ValueError: if the tool failed
        """
        # The old tool prints everything on stderr
        return self.usbrelay_cls(self.lab, self.raise_self, *in_args)

    def set_power(self, power_on: bool, port: int):
        out = self.usbrelay('%s_%s=%d' % (self._serial, port,
                                          1 if power_on else 0))

    def get_serial(self):
        """Get the serial number as reported by the device

        Sample output on stderr:

            Orig: 6QMBS, Serial: 6QMBS, Relay: 1 State: 0
            Device Found
              type: 16c0 05df
              path: /dev/hidraw3
              serial_number:
              Manufacturer: www.dcttech.com
              Product:      USBRelay8
              Release:      100
              Interface:    0
              Number of Relays = 8
            Serial: 6QMBS, Relay: 1 State: 0
            1 HID Serial: 6QMBS Serial: 6QMBS, Relay: 1 State: 0

            Device Found
              type: 16c0 05df
              path: /dev/hidraw4
              serial_number:
              Manufacturer: www.dcttech.com
              Product:      USBRelay4
              Release:      100
              Interface:    0
              Number of Relays = 4
            Serial: 6QMBS, Relay: 1 State: 0

            Serial: 6QMBS, Relay: 1 State: 0 --- Found

        Sample output for newer version:

            stdout:
                6QMBS_1=0
                6QMBS_2=0
                6QMBS_3=0
                6QMBS_4=0
                6QMBS_5=0
                6QMBS_6=0
                6QMBS_7=0
                6QMBS_8=0
                RAAMZ_1=0
                RAAMZ_2=0
                RAAMZ_3=0
                RAAMZ_4=0
                QAAMZ_1=0
                QAAMZ_2=0
                QAAMZ_3=0
                QAAMZ_4=0

            stderr:
                Orig: 6QMBS_1, Serial: 6QMBS, Relay: 1 State: 0
                Version: 0.7-17-g00dac6a282
                Library Version: 0.7-17-g00dac6a282
                enumerate_relay_boards()Found 3 devices
                Device Found
                  type: 16c0 05df
                  path: /dev/hidraw3
                  serial_number: 6QMBS
                Manufacturer: www.dcttech.com
                  Product:      USBRelay8
                  Release:      100
                  Interface:    0
                  Number of Relays = 8
                  Module_type = 1
                Device Found
                  type: 16c0 05df
                  path: /dev/hidraw6
                  serial_number: RAAMZ
                Manufacturer: www.dcttech.com
                  Product:      USBRelay4
                  Release:      100
                  Interface:    0
                  Number of Relays = 4
                  Module_type = 1
                Device Found
                  type: 16c0 05df
                  path: /dev/hidraw4
                  serial_number: QAAMZ
                Manufacturer: www.dcttech.com
                  Product:      USBRelay4
                  Release:      100
                  Interface:    0
                  Number of Relays = 4
                  Module_type = 1
                main() arg 0 Serial: 6QMBS, Relay: 1 State: 0
                find_board(6QMBS) path /dev/hidraw3
                main() operate: 6QMBS, Relay: 1 State: 0
                find_board(6QMBS) path /dev/hidraw3
                operate_relay(6QMBS,) /dev/hidraw3 path

        Returns:
            True if the port is on, False if off

        Raises:
            ValueError: if it did not respond
        """
        out, err = self.usbrelay('%s_1' % self._serial)
        rem = re.search(r'Serial: (%s), Relay: 1 State: 0 --- Found' %
                        self._serial, err)
        if not rem:
            rem = re.search('operate: (%s), Relay:' % self._serial, err)
            if not rem:
                self.raise_self("Unable to find status in '%s', '%s'" %
                                (out, err))
        serial = rem.group(1)
        return serial

    @classmethod
    def raiser_cls(cls):
        raise ValueError(msg)

    @classmethod
    def get_serial_from_hid(cls, lab, hidraw):
        """Get the serial number of a USB relay given its hid device number

        Args:
            hidraw: HID device number (e.g. 4 for '/dev/hiddev4')

        return:
            Seral number, or None if not found
        """
        _, err = cls.usbrelay_cls(lab, cls.raiser_cls, 'ABCDE_1')
        found = False  # True if we find our HID
        for line in err.splitlines():
            if not line.startswith('  '):
                found = False
            rem = re.search(r'  path: */dev/hidraw%d' % hidraw, line)
            if rem:
                found = True
            if found:
                rem = re.search(r'  serial_number: *(.*)', line)
                if rem:
                    return rem.group(1)
        return None

    def check(self):
        try:
            serial = self.get_serial()
            if serial != self._serial:
                msg = "Expected serial '%s' but got '%s'" % (self._serial,
                                                             serial)
                good = False
            else:
                good = True
                msg = ''
        except ValueError as exc:
            msg = str(exc)
            good = False
        return work.CheckResult(self, good, msg)

    def emit(self):
        emit_list = []
        emit_list.append(self.emit_udev())
        return emit_list

    def emit_udev(self):
        vals = {
            'name': str(self),
            'kernel_phys_port': self._kernel_phys_port,
            'symlink': self._symlink,
            }
        out = '''# {name}
ACTION=="add" \\
, KERNEL=="hidraw*" \\
, SUBSYSTEM=="hidraw" \\
, KERNELS=="{kernel_phys_port}" \\
, SYMLINK+="{symlink}" \\
, MODE="0666"

'''.format(**vals)
        return work.EmitResult(self, 'udev/99-labman-usbrelay.rules', out,
                               '# Generated by labman')

    def get_py_imports(self):
        return 'from usbrelay import Usbrelay'

    def get_py_base_class(self):
        return 'Usbrelay'

    def get_py_class_vars(self, part_ref):
        vars = {
            'usbrelay_name': self._serial,
            }
        if part_ref.prop == 'power-button':
            vars['usbrelay_power_button'] = part_ref.port
        if part_ref.prop == 'reset':
            vars['usbrelay_reset'] = part_ref.port
        if part_ref.prop == 'recovery':
            vars['usbrelay_recovery'] = part_ref.port
        return vars

    def get_poweron(self):
        return 'self.usbrelay_on()'

    def get_poweroff(self):
        return 'self.usbrelay_off()'

    def get_reset(self):
        return 'self.usbrelay_toggle_reset()'

    @classmethod
    def guess_part(cls, lab, phys):
        result = lab.get_usb_files(phys, 'product')
        if not result:
            return
        if result['product'][:8] not in ['USBRelay', 'HIDRelay']:
            return None
        result = lab.get_usb_dir_contents(phys + ':1.0/*', 'hidraw')
        if not result or not result[0].startswith('hidraw'):
            return None
        hidnum = int(result[0][6:])
        serial = cls.get_serial_from_hid(lab, hidnum)
        return serial

    def get_code(self, prop, prop_list, partref):
        if prop == 'poweron':
            # The 'reset' method does this so we don't need to
            if 'reset' in prop_list:
                return None
            return 'self.usbrelay_on()'
        elif prop == 'poweroff':
            return 'self.usbrelay_off()'
        elif prop == 'reset':
            return 'self.usbrelay_toggle_reset()'
        elif prop == 'baseclass':
            return 'Usbrelay'
        elif prop == 'setrecovery':
            return 'self.usbrelay_set_recovery(True)'
        elif prop == 'clearrecovery':
            return 'self.usbrelay_set_recovery(False)'
        elif prop == 'setreset':
            return 'self.usbrelay_set_reset(True)'
        elif prop == 'clearreset':
            return 'self.usbrelay_set_reset(False)'
        elif prop == 'delay':
            return 'self.usbrelay_delay()'
