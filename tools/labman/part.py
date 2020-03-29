# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

import collections
import glob
import importlib
import os

our_path = os.path.dirname(os.path.realpath(__file__))

class Part:
    """A component of the lab, such as a USB serial adaptor or SDwire

    Properties:
        name: Name of the uart. This should be short, ideally a nickname.
        lab: Lab associated with this uart, or None if none
    """
    # obj (Part)- object that the reference points to
    # port (str) - port number of the object (e.g. '12')
    # prop (str) - property name used to get this partref (e.g. 'bootdev')
    PartRef = collections.namedtuple('PartRef', 'obj,port,prop')

    def __init__(self):
        self.num_ports = None
        self._print = print
        self.hub = None
        self.power_hub = None
        self.partrefs = []
        self._kernel_phys_port = None
        self._power_phys_port = None

    def raise_self(self, msg):
        """Raise an error related to this part

        Args:
            msg (str): Message to report

        Raises:
            ValueError: always
        """
        raise ValueError('%s: %s' % (str(self), msg))

    def check(self):
        """Check if a part if working

        Implement this method in the ptype subclass
        """
        return None

    def emit(self):
        """Emit scripts a part

        Implement this method in the ptype subclass
        """
        return None

    def start(self):
        """Starts a daemon used by the part

        Implement this method in the ptype subclass
        """
        return None

    def stop(self):
        """Stops a daemon used by the part

        Implement this method in the ptype subclass
        """
        return None

    def get_all_detail(self, out):
        pass

    def show_line(self):
        print('%15s  %s' % (self.name, self.get_detail()))

    def show(self):
        print('Name: %s' % self.name)
        print('desc: %s' % self.get_detail())

    def lookup_usb_port(self, yam, prop):
        part_ref = self.lookup_part_ref(yam, prop)
        phys_port = '%s.%s' % (part_ref.obj._kernel_phys_port, part_ref.port)
        dev_port = yam.get('device-phys-port')
        if dev_port is not None:
            phys_port += ':%s' % dev_port
        return part_ref, phys_port

    def load_usb_port(self, yam):
        part_ref, phys_port = self.lookup_usb_port(yam, 'hub-port')
        self._kernel_phys_port = str(phys_port)
        self.hub = part_ref

    def lookup_part_ref(self, yam, prop):
        target_yam = yam.get(prop)
        if not target_yam:
            self.raise_self("Missing '%s' property" % prop)
        if not isinstance(target_yam, dict):
            self.raise_self("Expected * reference in prop '%s' (got '%s')" %
                            (prop, target_yam))
        name = target_yam.get('_anchor')
        if not name:
            self.raise_self("Missing anchor in '%s'" % target_yam)
        part = self.parts.find(name)
        port = target_yam.get('port-number')
        if part.num_ports:
            if port is None:
                self.raise_self("Missing port-number in '%s'" % target_yam)
        else:
            if port is not None:
                self.raise_self("Unexpected port-number in '%s'" % target_yam)
        hub_phys_port = str(port) if port is not None else None
        partref = self.PartRef(part, hub_phys_port, prop)
        self.partrefs.append(partref)
        return partref

    def find_dut_by_uart(self):
        return self.parts.find_dut_by_uart(self)

    def find_dut_by_send_device(self):
        return self.parts.find_dut_by_send_device(self)

    def get_deps(self):
        return set([partref.obj for partref in self.partrefs])

    def get_py_imports(self):
        return None

    def get_py_base_class(self):
        return None

    def get_py_class_vars(self, part_ref):
        return {}

    def get_select_dut(self):
        return None

    def get_select_ts(self):
        return None

    def get_reset(self):
        return None

    def get_poweron(self):
        return None

    def get_poweroff(self):
        return None

    def get_uart(self):
        return None

    def get_console(self):
        return None

    def get_send_device(self):
        return None

    @classmethod
    def guess_part(cls, lab, phys):
        return None

    def get_code(self, prop, prop_list, partref):
        return None
