# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

# Tests for Console class

import unittest

from labman.console import Console

PORT = '/dev/ttyusb_port0'

class ConsoleTest(unittest.TestCase):
    @classmethod
    def get_yaml(cls):
        return {'type': 'usb-uart', 'port': PORT}

    def testLoad(self):
        """Test loading a yaml description"""
        yam = self.get_yaml()
        con = Console(self)
        con.load(yam)
        self.assertEqual(Console.CTYPE_USB_UART, con._type)
        self.assertEqual(PORT, con._port)
        self.assertEqual('console %s/%s' % (Console.CTYPE_USB_UART, PORT),
                         str(con))
        self.assertEqual(PORT, con.get_uart())

    def Raise(self, msg):
        raise ValueError('test: %s' % msg)

    def testBadConnectionType(self):
        """Test handling an invalid connection type"""
        yam = {'type': 'blather', 'port': PORT}
        con = Console(self)
        with self.assertRaises(ValueError) as e:
            con.load(yam)
        self.assertIn("test: console None/None: Invalid type",
                      str(e.exception))

    def testNoConnectionType(self):
        """Test handling a missing connection type"""
        yam = {'port': PORT}
        con = Console(self)
        with self.assertRaises(ValueError) as e:
            con.load(yam)
        self.assertIn("test: console 0/None: Missing type",
                      str(e.exception))

    def testNoPort(self):
        """Test handling a missing porte"""
        yam = {'type': 'usb-uart'}
        con = Console(self)
        with self.assertRaises(ValueError) as e:
            con.load(yam)
        self.assertIn("test: console %d/None: Missing port" %
                      Console.CTYPE_USB_UART, str(e.exception))
