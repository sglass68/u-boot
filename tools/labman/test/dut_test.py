# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

# Tests for Console class

import unittest

from patman import test_util
from labman.dut import Dut
from labman.console import Console
from labman.test.console_test import ConsoleTest

DUT_NAME = 'my-dut'
DESCRIPTION = 'My description'

class DutTest(unittest.TestCase):
    @staticmethod
    def get_yaml():
        return {
            'desc': DESCRIPTION,
            'console': ConsoleTest.get_yaml()
            }

    def testLoad(self):
        """Test loading a yaml description"""
        dut = Dut(self, DUT_NAME)
        dut.load(self.get_yaml())
        self.assertEqual(DUT_NAME, dut._name)
        self.assertEqual(DESCRIPTION, dut._desc)
        self.assertIsNotNone(dut._cons)
        self.assertEqual('dut %s' % DUT_NAME, str(dut))
        with self.assertRaises(ValueError) as e:
            dut.Raise('not good')
        self.assertIn("test: dut my-dut: not good", str(e.exception))

    def testShow(self):
        """Test the show() method"""
        dut = Dut(self, DUT_NAME)
        dut.load(self.get_yaml())
        with test_util.capture_sys_output() as (stdout, stderr):
            dut.show()
        fields = stdout.getvalue().strip().split(sep=None, maxsplit=1)
        self.assertEqual(2, len(fields))
        self.assertEqual(DUT_NAME, fields[0])
        self.assertEqual(DESCRIPTION, fields[1])

    def Raise(self, msg):
        raise ValueError('test: %s' % msg)

    def testEmit(self):
        dut = Dut(self, DUT_NAME)
        dut.load(self.get_yaml())
        with test_util.capture_sys_output() as (stdout, stderr):
            dut.emit_tbot()
        out = stdout.getvalue()
        self.assertIn('Raspberry Pi 3b', out)

