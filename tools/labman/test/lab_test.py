# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

# Tests for Lab class

import yaml
import tempfile
import unittest

from patman import test_util
from labman.lab import Lab
from labman.test import dut_test

LAB_NAME = 'my-lab'
LAB_DESC = 'My description'
PROVISION_NAME = 'fred'
PROVISION_SERIAL = 'serial1234'

class LabTest(unittest.TestCase):
    def setUp(self):
        self._provision_args = None
        self._provision_name = None

    @staticmethod
    def get_yaml():
        return {
            'name': LAB_NAME,
            'desc': LAB_DESC,
            'duts': {
                dut_test.DUT_NAME: dut_test.DutTest.get_yaml(),
                }
            }

    def testRead(self):
        with tempfile.NamedTemporaryFile() as fp:
            lab = Lab()
            fp.write(yaml.dump(self.get_yaml()).encode('utf-8'))
            fp.seek(0)
            lab.read(fp.name)
            self.assertEqual(LAB_NAME, lab._name)

    def testLoad(self):
        """Test loading a yaml description"""
        lab = Lab()
        lab.load(self.get_yaml())
        self.assertEqual(LAB_NAME, lab._name)
        self.assertEqual(LAB_DESC, lab._desc)

        self.assertEqual('lab %s' % LAB_NAME, str(lab))
        with self.assertRaises(ValueError) as e:
            lab.raise_self('not good')
        self.assertIn("lab my-lab: not good", str(e.exception))

    def testShow(self):
        """Test the show() method"""
        lab = Lab()
        lab.load(self.get_yaml())
        with test_util.capture_sys_output() as (stdout, stderr):
            lab.show()
        fields = stdout.getvalue().splitlines()
        self.assertEqual(2, len(fields))
        self.assertEqual('DUTs:', fields[0].strip())
        self.assertIn(dut_test.DUT_NAME, fields[1])

    def Raise(self, msg):
        raise ValueError('test: %s' % msg)

    def testEmit(self):
        lab = Lab()
        lab.load(self.get_yaml())
        with test_util.capture_sys_output() as (stdout, stderr):
            lab.emit(dut_test.DUT_NAME, 'tbot')
        out = stdout.getvalue()
        self.assertIn(dut_test.DUT_NAME, out)

    def testBadEmit(self):
        lab = Lab()
        lab.load(self.get_yaml())
        with self.assertRaises(ValueError) as e:
            lab.emit('unknown-name', 'tbot')
        self.assertIn("Dut 'unknown-name' not found", str(e.exception))

        with self.assertRaises(ValueError) as e:
            lab.emit(dut_test.DUT_NAME, 'bad-ftype')
        self.assertIn("Invalid ftype 'bad-ftype'", str(e.exception))

    def testBadLoad(self):
        """Test loading a yaml description"""
        lab = Lab()
        yam = self.get_yaml()
        del yam['name']
        with self.assertRaises(ValueError) as e:
            lab.load(yam)
        self.assertIn("Missing name", str(e.exception))

    def get_sdwire(self, name):
        self._provision_name = name
        self.lab = None
        return self

    def provision(self, *args):
        self._provision_args = args

    def testProvision(self):
        lab = Lab()
        lab.provision('sdwire', PROVISION_NAME, PROVISION_SERIAL,
                      test_obj=self.get_sdwire)
        self.assertEqual(PROVISION_NAME, self._provision_name)
        self.assertIsNotNone(self._provision_args)
        self.assertSequenceEqual([PROVISION_SERIAL], self._provision_args)

    def testBadProvision(self):
        lab = Lab()
        with self.assertRaises(ValueError) as e:
            lab.provision('unknown', PROVISION_NAME, PROVISION_SERIAL)
        self.assertIn("Unknown component 'unknown'", str(e.exception))
