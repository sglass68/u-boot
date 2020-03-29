# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

# Tests for Lab class

import tempfile
import unittest
import yaml

from patman import test_util
from labman import control
from labman.control import Labman
from labman.test import dut_test
from labman.test import lab_test

FILENAME = 'test-file'
PROV_COMPONENT = 'sdwire'
PROV_NAME = 'hairy'
PROV_SERIAL = '123'

class objectview(object):
    def __init__(self, args):
        if not 'remote' in args:
            args['remote'] = False
        self.__dict__ = args


class ControlTest(unittest.TestCase):
    def setUp(self):
        self._component = None
        self._dut = None
        self._fname = None
        self._ftype = None
        self._name = None
        self._serial = None
        self._show = None

    def read(self, fname):
        self._fname = fname

    def show(self):
        self._show = True

    def emit(self, dut, ftype):
        self._dut = dut
        self._ftype = ftype

    def provision(self, component, name, serial):
        self._component = component
        self._name = name
        self._serial = serial

    def testRead(self):
        with tempfile.NamedTemporaryFile() as fp:
            fp.write(yaml.dump(
                lab_test.LabTest.get_yaml()).encode('utf-8'))
            fp.seek(0)
            args = objectview({'lab': fp.name, 'cmd': 'ls'})
            with test_util.capture_sys_output() as (stdout, stderr):
                Labman(args)
        self.assertEqual(lab_test.LAB_NAME, control.test_lab._name)

    def testLs(self):
        args = objectview({'lab': FILENAME, 'cmd': 'ls'})
        Labman(args, self)
        self.assertEqual(FILENAME, self._fname)
        self.assertTrue(self._show)

    def testEmit(self):
        args = objectview({
            'lab': FILENAME,
            'cmd': 'emit',
            'dut': dut_test.DUT_NAME,
            'ftype': 'tbot'})
        Labman(args, self)
        self.assertEqual(FILENAME, self._fname)
        self.assertEqual(dut_test.DUT_NAME, self._dut)
        self.assertEqual('tbot', self._ftype)

    def testProv(self):
        args = objectview({
            'lab': FILENAME,
            'cmd': 'prov',
            'component': PROV_COMPONENT,
            'name': PROV_NAME,
            'serial': PROV_SERIAL})
        Labman(args, self)
        self.assertEqual(FILENAME, self._fname)
        self.assertEqual(PROV_COMPONENT, self._component)
        self.assertEqual(PROV_NAME, self._name)
        self.assertEqual(PROV_SERIAL, self._serial)
