# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

import collections
import glob
import importlib
import os

from labman.ptypes.hub import Part_hub

our_path = os.path.dirname(os.path.realpath(__file__))

class Parts:
    """A collection of parts in the lab, each a Part object"""

    def __init__(self, lab):
        self.lab = lab
        self._parts = {}
        self._modules = {}
        self._ptype_classes = collections.OrderedDict()

    def lookup(self, fname):
        if '__' in fname:
            return None
        module_name = os.path.splitext(fname)[0]
        module = self._modules.get(module_name)
        # Import the module if we have not already done so.
        if not module:
            try:
                module = importlib.import_module('labman.ptypes.' +
                                                 module_name)
            except ImportError as e:
                raise ValueError("Unknown part '%s', error '%s'" %
                                 (module_name, e))
            self._modules[module_name] = module

        # Look up the expected class name
        return module_name, getattr(module, 'Part_%s' % module_name)

    def scan_ptypes(self):
        """Scan for available parts"""
        glob_list = glob.glob(os.path.join(our_path, 'ptypes', '*'))
        # Load DUTs last as they depend on everything else
        for passnum in range(2):
            for fname in glob_list:
                retval = self.lookup(os.path.basename(fname))
                if retval:
                    cls_name, ptype_cls = retval
                    if not passnum and cls_name == 'dut':
                        continue
                    self._ptype_classes[cls_name] = ptype_cls

    def load_all(self, all_yam):
        def do_load(cls_name, ptype_cls):
            yaml_name = cls_name + 's'
            parts_yam = all_yam.get(yaml_name)
            if parts_yam:
                for name, yam in parts_yam.items():
                    if self._parts.get(name):
                        raise ValueError("Duplicate part name '%s'" % name)
                    if yam.get('active') == False:
                        continue
                    part = self.create(ptype_cls, name)
                    part.load(yam)
                    self._parts[name] = part

        # Load hubs first
        do_load('hub', self._ptype_classes['hub'])
        do_load('ykush', self._ptype_classes['ykush'])
        for cls_name, ptype_cls in self._ptype_classes.items():
            if cls_name not in ['hub', 'ykush']:
                do_load(cls_name, ptype_cls)

    def create(self, ptype_cls, name):
        try:
            obj = ptype_cls()
        except TypeError as exc:
            raise TypeError("Ptype '%s': %s" % (ptype_cls, str(exc)))
        obj.name = name
        obj.lab = self.lab
        obj.parts = self
        return obj

    def get_list(self, filter_list=None):
        if filter_list:
            return [part for name, part in self._parts.items()
                    if name in filter_list]
        return list(self._parts.values())

    def get_parts_by_hub(self, part_list):
        hub_dict = collections.defaultdict(dict)
        for part in part_list:
            if part.hub:
                hub_dict[part.hub.obj][part.hub.port] = part
            if part.power_hub:
                hub_dict[part.power_hub.obj][part.power_hub.port] = part
        return hub_dict

    def find(self, to_find):
        for name, part in self._parts.items():
            if name == to_find:
                return part
        raise ValueError("Could not find name '%s' in %s" %
                         (to_find, self._parts.keys()))

    def find_dut_by_uart(self, find_uart):
        for name, part in self._parts.items():
            cons = part.get_console()
            if cons and cons.obj == find_uart:
                return part
        raise ValueError("Could not find DUT for uart '%s' in %s" %
                         (find_uart.name, self._parts.keys()))

    def find_dut_by_send_device(self, find_fdev):
        for name, part in self._parts.items():
            fdev = part.get_send_device()
            if fdev and fdev.obj == find_fdev:
                return part
        raise ValueError("Could not find DUT for send device '%s' in %s" %
                         (find_fdev.name, self._parts.keys()))

    def scan(self):
        def in_hubs(phys_port, hubs):
            """Check if a physical port is attached to any of the given hubs

            Args:
                phys_port: Port to check (e.g. '1-5.1.2')
                hubs: List of hubs to check (e.g. ['1-5', '2.6'])

            Returns:
                True if the port is attached to any of the hubs
            """
            for hub in hubs:
                if phys_port.startswith(hub):
                    return True
            return False

        # Find all available devices by their physical port address
        result = self.lab.run_command('ls', '/sys/bus/usb/devices')
        phys_ports = set([line for line in result.stdout.splitlines()
                      if line[0].isdigit() and ':' not in line])

        # Get a list of all port numbers we know about
        part_list = self.get_list()
        part_ports = set()
        hub_phys_set = set()
        for part in part_list:
            if isinstance(part, Part_hub):
                hub_phys_set.add(part._kernel_phys_port)
            else:
                if part._kernel_phys_port:
                    part_ports.add(part._kernel_phys_port)
                if part._power_phys_port:
                    part_ports.add(part._power_phys_port)

        # Remove any interfaces or sub-devices of this ports
        for port in part_ports:
            for phys in list(phys_ports):
                if phys.startswith(port):
                    phys_ports.remove(phys)

        # Get a list of all available ports, including unconnected ones
        all_ports = set()
        for part in part_list:
            if isinstance(part, Part_hub):
                all_ports |= set(part.phys_port_list())

        # Find connected ports where we don't have a part in our database
        new_parts = phys_ports & all_ports

        print('%d new part(s) found' % len(new_parts))
        if new_parts:
            print('%15s  %-5s  %-12s  %s' %
                  ('Hub', 'Port', 'Part', 'Description'))
            for hub in part_list:
                if isinstance(hub, Part_hub):
                    for phys in hub.phys_port_list():
                        if phys in new_parts:
                            port = phys[len(hub._kernel_phys_port) + 1:]
                            part_name, part_detail = self.scan_for_guess(
                                self.lab, phys)
                            print('%15s  %-5s  %-12s  %s' %
                                  (hub.name, hub._ports[port],
                                   part_name or '(unknown)', part_detail))

        # Drop ports that are attached to hubs we already know about
        phys_ports = set(port for port in phys_ports
                      if not in_hubs(port, hub_phys_set))

        # Drop ports that are sub-ports of others
        phys_ports = {port for port in phys_ports
                      if not in_hubs(port, phys_ports - {port})}

        for phys in phys_ports:
            part_name, part_detail = self.scan_for_guess(self.lab, phys)
            if part_name:
                print('%-5s %-12s  %s' % (phys, part_name, part_detail))


    def scan_for_guess(self, lab, phys):
        for cls_name, ptype_cls in self._ptype_classes.items():
            details = ptype_cls.guess_part(lab, phys)
            if details:
                return cls_name, details
        return None, ''
