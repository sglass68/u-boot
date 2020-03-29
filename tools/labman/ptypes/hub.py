# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

import re

from labman.part import Part
from labman import work

class Part_hub(Part):
    """A USB hub which can connect multiple USB devices to a single port

    Properties:
    """
    def load(self, yam):
        """Load the object from a yaml definition

        Args:
            yam (dict): Yaml definition
        """
        self._kernel_phys_port = yam.get('kernel-phys-port')
        if self._kernel_phys_port is None:
            self.raise_self("Missing kernel-phys-port")
        self._kernel_phys_port = str(self._kernel_phys_port)
        self.num_ports = yam.get('num-ports')
        if not self.num_ports:
            self.raise_self("Missing num-ports")
        self._ports = {}
        self._ports_by_num = {}
        for name, port_yam in yam.get('ports').items():
            port_num = port_yam.get('port-number')
            if port_num is None:
                self.raise_self("Missing port-number in %s" % port_yam)
            port_num = str(port_num)
            nums = re.findall(r'\d+', str(name))
            if nums:
                num = int(nums[0])
            self._ports[port_num] = num
            self._ports_by_num[num] = port_num

    def __str__(self):
        """Convert the object into a string for presentation to the user"""
        return 'hub %s' % self.name

    def raise_self(self, msg):
        """Raise an error related to this hub

        Args:
            msg (str): Message to report

        Raises:
            ValueError: always
        """
        raise ValueError('%s: %s' % (str(self), msg))

    def get_detail(self, port=None):
        return '%s, %d ports' % (self._kernel_phys_port, self.num_ports)

    def check(self):
        """Run a check on a hub to see that it seems to work OK

        This just checks that it is present according to the kernel

        Returns:
            work.CheckResult: Result obtained from the check
        """
        path = '/sys/bus/usb/devices/%s' % self._kernel_phys_port
        result = self.lab.run_command('ls', path)
        if result.return_code:
            msg = str(result.stderr.strip())
            good = False
        else:
            msg = 'all OK'
            good = True
        return work.CheckResult(self, good, msg)

    @classmethod
    def guess_part(cls, lab, phys):
        result = lab.get_usb_files(phys, 'maxchild', 'product')
        if not result:
            return
        if result['product'] and int(result['maxchild']) > 1:
            return result['product']
        return None

    def get_port_name(self, port_num):
        return self._ports[str(port_num)]

    def phys_port_list(self):
        return ['%s.%s' % (self._kernel_phys_port, phys) for
                phys in self._ports.keys()]
