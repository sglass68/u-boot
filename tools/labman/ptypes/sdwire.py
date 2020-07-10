# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

"""Implement support for the SDwire USB/uSD mux"""

from datetime import datetime, timedelta
import glob
import re
import time

from labman.bootdev import Part_bootdev
from labman import work

class Part_sdwire(Part_bootdev):
    """An Sdwire board which can switch a uSD card between DUT and test system

    Properties:
        name: Name of the sdwire. This should be short, ideally a nickname.
        lab: Lab associated with this sdwire, or None if none
        _serial: Serial number of the SDwire
    """
    ORIG_IDS = ['--vendor=0x0403', '--product=0x6015']
    TIMEOUT_S = 30

    DUT, TS = range(2)

    VENDOR = 0x04e8
    PRODUCT = 0x6001

    def __init__(self):
        super().__init__()
        self._serial = None
        self._symlink = None
        self._verbose = False
        self._sleep = time.sleep

    def __str__(self):
        """Convert the object into a string for presentation to the user"""
        return 'sdwire %s' % self.name

    def load(self, yam):
        """Load the object from a yaml definition

        Args:
            yam (dict): Yaml definition
        """
        self._serial = yam.get('serial')
        if isinstance(self._serial, int):
            self._serial = str(self._serial)
        if self._serial is None:
            self.raise_self("Missing serial")
        self._symlink = yam.get('symlink')
        if self._symlink is None:
            self.raise_self("Missing symlink")
        self._block_symlink = yam.get('block-symlink')
        self.load_usb_port(yam)
        self._mount_uuid = yam.get('mount-uuid')
        self._mount_point = yam.get('mount-point')
        if self._mount_uuid:
            if not self._mount_point:
                self.raise_self("Missing mount-point")
        else:
            if self._mount_point:
                self.raise_self("Unexpected mount-point (or missing mount-uuid)")

    def get_detail(self, port=None):
        return '%s' % self._serial

    def get_all_detail(self, out):
        out['serial'] = self._serial
        out['symlink'] = self._symlink
        if self._block_symlink:
            out['blockdev'] = self._block_symlink
        if self._mount_uuid:
            out['blockdev'] = self._mount_uuid
        return out

    def sdmux_ctrl(self, *in_args, retry=True):
        """Perform an operation with the sd-mux-ctrl tool

        Args:
            in_args: Arguments to pass to the tool
            retry (bool): True to retry if an error is obtained. This can help
                in situations where the device is just plugged in and udev has
                not yet set the permissions such that the device can be accessed

        Returns:
            CommandResult: Result from command
        """
        result = None
        args = ['sd-mux-ctrl'] + list(in_args)
        for _ in range(2):
            result = self.lab.run_command(*args)
            if result.return_code:
                if self._verbose:
                    self._print('Error: %s' % result.stderr)
            else:
                return result
            if not retry:
                break
            self._sleep(1)
        return result

    def select_ts(self):
        """Connect the uSD card to the test system"""
        self.sdmux_ctrl('-e', self._serial, '--ts', retry=False)

    def select_dut(self):
        """Connect the uSD card to the DUT"""
        result = self.sdmux_ctrl('-e', self._serial, '--dut', retry=False)
        if result.return_code:
            self.raise_self("Failed '%s'" % result.stderr.strip())

    def get_status(self):
        """Get the current status of the SDwire

        Returns:
            int:
                TS if connected to TS
                DUT if connected to DUT

        Raises:
            ValueError if the result could not be parsed
        """
        result = self.sdmux_ctrl('-e', self._serial, '--status', retry=False)
        out = result.stdout
        words = out.split()
        if not len(words):
            self.raise_self("Invalid response '%s'" % out)
        connected = words[-1]
        if connected == 'DUT':
            return self.DUT
        if connected == 'TS':
            return self.TS
        self.raise_self("Invalid response '%s'" % out)
        return None

    def parse_serial(self, line):
        """Parse the serial-number line emitted by sd-mux-ctrl

        This line is of the form:
            Dev: 3, Manufacturer: SRPOL, Serial: sdwire-7, Description: sd-wire

        Args:
            line (str): Line to parse

        Returns:
            str: Serial number parsed from the string

        Raises:
            ValueError if the value could not be parsed
        """
        rem = re.search(r'Serial: ([A-Z0-9a-z]+),', line)
        if not rem:
            self.raise_self("Unable to find serial number in '%s'" % line)
        old_serial = rem.group(1)
        return old_serial

    def list_matching(self, ids):
        """Get a list of matching SDwires

        Args:
            ids (list): List of ID-related parameters to pass to sd-mux-ctrl

        Returns:
            tuple:
                (int) Number of SDwires found
                (str) Serial number found, if we found exactly one
        """
        result = self.sdmux_ctrl(*ids, '--list')
        lines = result.stdout.splitlines()
        rem = re.search('Number of FTDI devices found: ([0-9]+)',
                        lines and lines[0] or '')
        if not rem:
            self.raise_self("Expected device count in first line '%s'" % lines)
        found = int(rem.group(1))
        serial = None
        if found == 1:
            serial = self.parse_serial(lines[1])
        return found, serial

    def get_serial(self):
        """Get the serial number of available SDwires

        Returns:
            str: Serial number of the only SDwire found

        Raises:
            ValueError: If zero or more than two SDwires were found
        """
        found, serial = self.list_matching(self.ORIG_IDS)
        if found != 1:
            self.raise_self("Expected to find one SDwire, found %s" % found)
        return serial

    def wait_for(self, req_count, orig=False):
        for _ in range(self.TIMEOUT_S):
            found, serial = self.list_matching(self.ORIG_IDS if orig else [])
            if found == req_count:
                return serial
            if req_count:
                msg = '\rInsert the SDwire...'
            else:
                msg = '\rUnplug all SDwires...%d ' % found
            self._print(msg, end='', flush=True)
            self._sleep(1)
        self.raise_self('gave up waiting')

    def provision(self, new_serial):
        """Provision an SDwire ready for use

        This only works if there is a single SDwire connected.
        """
        # Get the current serial number
        if not new_serial:
            self.raise_self('Please specify a serial number')
        self._verbose = True

        self.wait_for(0, True)
        old_serial = self.wait_for(1, True)

        # Program in the new serial number
        self.sdmux_ctrl('--device-serial=%s' % old_serial, *self.ORIG_IDS,
                        '--device-type=sd-wire', '--set-serial=%s' % new_serial)

        # Wait for the device to re-appear with the new details
        self.wait_for(0, True)
        self.wait_for(1, False)

        if serial != new_serial:
            self.raise_self("Expected serial '%s' but got '%s'" %
                            (new_serial, serial))
        self._print("\nProvision complete for serial '%s'" % new_serial)

    def list_disks(self):
        disks = glob.glob('/dev/sd?')
        return set([name[-1] for name in disks])

    def provision_test(self):
        self.wait_for(0)
        orig_disks = self.list_disks()

        self._serial = self.wait_for(1)
        print("Running provision test on serial '%s'" % self._serial)
        self.select_dut()
        if self.get_status() != self.DUT:
            self.raise_self('Failed to switch to DUT')
        time.sleep(1)

        self.select_ts()
        if self.get_status() != self.TS:
            self.raise_self('Failed to switch to TS')
        time.sleep(1)
        new_disks = self.list_disks().difference(orig_disks)
        if len(new_disks) != 1:
            self.raise_self('Cannot find disk')
        new_disk = new_disks.pop()
        print('Found disk %s' % new_disk)
        self._block_symlink = 'sd%s' % new_disk
        self.check_for_block_symlink()

        self.select_dut()
        if self.get_status() != self.DUT:
            self.raise_self('Failed to switch to DUT')

        bsymlink_path = '/dev/%s' % self._block_symlink
        done = False
        for i in range(5):
            result = self.lab.run_command('dd', 'if=%s' % bsymlink_path,
                                          'of=/dev/null', 'count=1')
            if result.return_code:
                done = True
                break
            time.sleep(1)
        if not done:
            raise ValueError("Cannot remove device '%s'" % bsymlink_path)

    def check_for_block_symlink(self):
        bsymlink_path = '/dev/%s' % self._block_symlink
        done = False
        for i in range(5):
            result = self.lab.run_command('dd', 'if=%s' % bsymlink_path,
                                          'of=/dev/null', 'count=1')
            if not result.return_code:
                done = True
                break
            time.sleep(1)
        if not done:
            raise ValueError("Cannot access device '%s'" % bsymlink_path)

    def check_for_mount(self):
        done = False
        start_time = datetime.now()
        for i in range(5):
            out = ""
            result = self.lab.run_command("mount",
                                          "UUID=%s" % self._mount_uuid)
            done = not result.return_code
            if done:
                break
            if "already mounted" in result.stderr:
                # If it is already mounted, try to unmount it first. It may have
                # been mounted by another user so we won't have the access we
                # need. If this gives an error then it might be transient, e.g.
                # "Operation not permitted" is sometimes returned when there are
                # I/O errors on the device.
                self.lab.run_command("umount", "UUID=%s" % self._mount_uuid)
            if "mount point does not exist" in result.stderr:
                raise ValueError(result.stderr.strip())
            duration = datetime.now() - start_time
            if i and duration > timedelta(seconds=20):
                raise ValueError("Timeout on mount '%s'" % self._mount_uuid)
            time.sleep(1)
        if not done:
            raise ValueError("Cannot access mount '%s'" % self._mount_uuid)
        self.lab.run_command("umount", "UUID=%s" % self._mount_uuid)

    def unmount(self):
        self.lab.run_command("umount", "UUID=%s" % self._mount_uuid)

    def check(self):
        """Run a check on the SDwire to see that it seems to work OK

        Returns:
            work.CheckResult: Result obtained from the check
        """
        try:
            self.select_dut()
            if self.get_status() != self.DUT:
                self.raise_self('Failed to switch to DUT')
            time.sleep(1)
            self.select_ts()
            if self.get_status() != self.TS:
                self.raise_self('Failed to switch to TS')

            symlink_path = '/dev/%s' % self._symlink
            result = self.lab.run_command('head', '-0', symlink_path)
            if result.return_code:
                self.raise_self("Failed to locate '%s'" % symlink_path)

            if self._block_symlink:
                self.check_for_block_symlink()

            if self._mount_uuid:
                self.check_for_mount()

            msg = 'all OK'
            good = True
        except ValueError as exc:
            msg = str(exc)
            good = False
        return work.CheckResult(self, good, msg)

    def emit(self):
        emit_list = []
        emit_list.append(self.emit_udev_sdwire())
        if self._block_symlink:
            emit_list.append(self.emit_udev_sdcard())
        if self._mount_uuid:
            emit_list.append(self.emit_fstab_sdcard())
        return emit_list

    def emit_udev_sdwire(self):
        vals = {
            'name': str(self),
            'serial': self._serial,
            'symlink': self._symlink,
            'product': '%04x' % self.PRODUCT,
            'vendor': '%04x' % self.VENDOR,
            }
        out = '''# {name}
ACTION=="add|bind" \\
, SUBSYSTEM=="usb" \\
, ATTR{{idProduct}}=="{product}" \\
, ATTR{{idVendor}}=="{vendor}" \\
, ATTR{{serial}}=="{serial}" \\
, MODE="0666" \\
, SYMLINK+="{symlink}"

'''.format(**vals)
        return work.EmitResult(self, 'udev/99-labman-sdwire.rules', out,
                               '# Generated by labman')

    def emit_udev_sdcard(self):
        vals = {
            'name': str(self),
            'kernel_phys_port': self._kernel_phys_port,
            'block_symlink': self._block_symlink,
            }
        # The block device is always .1 within the SDwire. The mux itself is
        # .2
        out = '''# {name}
ACTION=="add|change" \\
, KERNEL=="sd*" \\
, SUBSYSTEM=="block" \\
, KERNELS=="{kernel_phys_port}.1" \\
, SYMLINK+="{block_symlink}" \\
, MODE="0666"

'''.format(**vals)
        return work.EmitResult(self, 'udev/99-labman-sdcard.rules', out,
                               '# Generated by labman')

    def emit_fstab_sdcard(self):
        vals = {
            'name': str(self),
            'uuid': self._mount_uuid,
            'mount_point': self._mount_point,
            }
        out = '''# {name}
UUID={uuid}\t\t/media/{mount_point}\tauto\tdefaults,noauto,user

'''.format(**vals)
        return work.EmitResult(self, 'etc/fstab', out, '')

    def get_py_imports(self):
        return 'from sdwire import Sdwire'

    def get_py_base_class(self):
        return 'Sdwire'

    def get_py_class_vars(self, part_ref):
        out = {
            'sdwire_serial': self._serial,
            }
        if self._mount_uuid:
            out.update({
                'mount_point': self._mount_point,
                'mount_uuid': self._mount_uuid,
                })
        if self._block_symlink:
            out.update({
                'block_device': '/dev/%s' % self._block_symlink,
                })
        return out

    def get_select_dut(self):
        return 'self.sdwire_dut()'

    def get_select_ts(self):
        return 'self.sdwire_ts()'

    @classmethod
    def guess_part(cls, lab, phys):
        result = lab.get_usb_files(phys + '.2', 'idProduct', 'idVendor', 'serial')
        if not result:
            return
        if result['idProduct'] == '%04x' % cls.PRODUCT:
            return result['serial']
        return None

    def get_code(self, prop, prop_list, partref):
        if prop == 'bootdev_dut':
            return 'self.sdwire_dut()'
        elif prop == 'bootdev_ts':
            return 'self.sdwire_ts()'
        elif prop == 'baseclass':
            return 'Sdwire'
