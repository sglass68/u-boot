# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

# Tests for Sdwire class

import unittest

from labman import sdwire
from patman.command import CommandResult

OLD_SERIAL = 'ABCD'
NEW_SERIAL = '4321'
BAD_SERIAL = '@@'

(TEST_BAD_TOOL, TEST_NO_SDWIRES, TEST_ONE_SDWIRE_FOREVER,
 TEST_ONE_SDWIRE_GONE_5_SECONDS, TEST_ONE_SDWIRE_WRONG_SERIAL,
 TEST_INVALID_SERIAL, TEST_ONE_SDWIRE_SUCCESS) = range(7)

class SdwireTest(unittest.TestCase):
    def setUp(self):
        self._print_lines = []
        self._timer = 0
        self._raise = 0

    def get_response(self, num_devices, serial=None):
        resp = 'Number of FTDI devices found: %d\n' % num_devices;
        if num_devices:
            resp += ('Dev: 0, Manufacturer: SRPOL, Serial: %s, Description: sd-wire\n' %
                     serial)
        return CommandResult(resp)

    def sd_mux_ctl(self, *args):
        # Make sure it can handle an error from sd-mux-ctrl
        if self._raise:
            self._raise -= 1
            if self._raise % 2:
                return CommandResult(stderr='test I/O error', return_code=1)
        if self._state == TEST_BAD_TOOL:
            return CommandResult('', return_code=2)
        elif self._state == TEST_NO_SDWIRES:
            return self.get_response(0)
        elif self._state == TEST_INVALID_SERIAL:
            return self.get_response(1, BAD_SERIAL)
        elif self._state == TEST_ONE_SDWIRE_FOREVER:
            return self.get_response(1, OLD_SERIAL)
        elif self._state == TEST_ONE_SDWIRE_GONE_5_SECONDS:
            if self._timer < 5:
                return self.get_response(1, OLD_SERIAL)
            else:
                return self.get_response(0)
        elif self._state == TEST_ONE_SDWIRE_WRONG_SERIAL:
            if self._timer < 5:
                return self.get_response(1, OLD_SERIAL)
            elif self._timer < 10:
                return self.get_response(0)
            else:
                return self.get_response(1, OLD_SERIAL)
        elif self._state == TEST_ONE_SDWIRE_SUCCESS:
            if self._timer < 5:
                return self.get_response(1, OLD_SERIAL)
            elif self._timer < 10:
                return self.get_response(0)
            else:
                return self.get_response(1, NEW_SERIAL)

        raise ValueError('Internal test error')

    def print(self, *args, **kwargs):
        self._print_lines.append([args, kwargs])

    def sleep(self, seconds):
        self._timer += seconds

    def get_sdwire(self):
        return sdwire.Sdwire('name', None, self.sd_mux_ctl, self.print,
                             self.sleep)

    def testToolFailed(self):
        """sd-mux-ctrl doesn't seem to work"""
        sdw = self.get_sdwire()
        self._state = TEST_BAD_TOOL
        with self.assertRaises(ValueError) as e:
            sdw.provision(NEW_SERIAL)
        self.assertIn("Expected device count in first line",str(e.exception))

    def testNoSdwire(self):
        """No sdwire being connected at the start"""
        sdw = self.get_sdwire()
        self._state = TEST_NO_SDWIRES
        with self.assertRaises(ValueError) as e:
            sdw.provision(NEW_SERIAL)
        self.assertIn("Expected to find one SDwire, found 0",str(e.exception))

    def testBadSerial(self):
        """No sdwire being connected at the start"""
        sdw = self.get_sdwire()
        self._state = TEST_INVALID_SERIAL
        with self.assertRaises(ValueError) as e:
            sdw.provision(NEW_SERIAL)
        self.assertRegex(str(e.exception),
                         "Unable to find serial number.*%s.*" % BAD_SERIAL)


    def testNeverUnplug(self):
        """Sdwire is connected and programmed but never unplugged"""
        sdw = self.get_sdwire()
        self._state = TEST_ONE_SDWIRE_FOREVER
        with self.assertRaises(ValueError) as e:
            sdw.provision(NEW_SERIAL)
        self.assertIn("gave up waiting", str(e.exception))
        self.assertEqual(1, len(self._print_lines))
        args, kwargs = self._print_lines[0]
        self.assertIn('Unplug the SDwire', args[0])

    def testNeverInserted(self):
        """Sdwire is connected, programmed, unplugged but never inserted"""
        sdw = self.get_sdwire()
        self._state = TEST_ONE_SDWIRE_GONE_5_SECONDS
        with self.assertRaises(ValueError) as e:
            sdw.provision(NEW_SERIAL)
        self.assertIn("gave up waiting", str(e.exception))
        self.assertEqual(2, len(self._print_lines))
        args, kwargs = self._print_lines[0]
        self.assertIn('Unplug the SDwire', args[0])
        args, kwargs = self._print_lines[1]
        self.assertIn('Insert the SDwire', args[0])

    def testInsertedWrong(self):
        """Sdwire is provisioned but the serial number is wrong"""
        sdw = self.get_sdwire()
        self._state = TEST_ONE_SDWIRE_WRONG_SERIAL
        with self.assertRaises(ValueError) as e:
            sdw.provision(NEW_SERIAL)
        self.assertIn("Expected serial '%s' but got '%s'" %
                      (NEW_SERIAL, OLD_SERIAL), str(e.exception))

    def testSuccess(self):
        """Sdwire is fully provisioned, even with errors from sd-mux-ctrl"""
        self._raise = 100
        sdw = self.get_sdwire()
        self._state = TEST_ONE_SDWIRE_SUCCESS
        sdw.provision(NEW_SERIAL)
        self.assertEqual(10, len(self._print_lines))
        args, kwargs = self._print_lines[9]
        self.assertIn('Provision complete', args[0])
