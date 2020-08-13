# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC

import collections
import os
from typing import Optional
import re
import time

from labman.part import Part
from labman.ptypes.usbboot import Part_usbboot
from labman import work
from labman.power import Power
from patman import tout

class Part_dut(Part, Power):
    """Individual board in the lab, upon which we want to run tests.

    DUT stands for device-under-test. It means the device that is being tested.

    If a board can be used for different builds, it should still only appear
    once in the lab.

    DUTs have a particular hardware configuration (e.g processor, memory) and
    have connections with in the lab (e.g. for serial UART).

    Properties:
        _parent: Parent object (e.g. a Lab)
        _name: Name of the DUT (e.g. 'rpi_3')
        _desc: Description of the DUT (e.g. 'Raspberry Pi 3b')
        _cons (PartRef): Console to talk to the DUT
    """
    def __init__(self):
        super().__init__()
        self._desc = None
        self._cons = None
        self._bootdev = None
        self._flash_method = None
        self._flash_device = None
        self._power = None
        self._power_button = None
        self._reset = None
        self._recovery = None
        self._recovery_extra = None
        self._blob_dest = None
        self._blob_src = None
        self._prompt = None
        self._send_method = None
        self._send_device = None

    def load(self, yam):
        self._desc = yam.get('desc')
        self._cons = self.lookup_part_ref(yam, 'console')
        self._prompt = yam.get('prompt')
        if 'bootdev' in yam:
            self._bootdev = self.lookup_part_ref(yam, 'bootdev')
            if not self._bootdev:
                self.raise_self("Missing bootdev")
        if 'power' in yam:
            self._power = self.lookup_part_ref(yam, 'power')
            if not self._power:
                self.raise_self("Missing power")
        if 'power-button' in yam:
            self._power_button = self.lookup_part_ref(yam, 'power-button')
            if not self._power_button:
                self.raise_self("Missing power-button")
        if 'reset' in yam:
            self._reset = self.lookup_part_ref(yam, 'reset')
            if not self._reset:
                self.raise_self("Missing reset")
        if 'recovery' in yam:
            self._recovery = self.lookup_part_ref(yam, 'recovery')
            if not self._recovery:
                self.raise_self("Missing recovery")
        if 'recovery-extra' in yam:
            self._recovery_extra = self.lookup_part_ref(yam, 'recovery-extra')
            if not self._recovery_extra:
                self.raise_self("Missing recovery")

        flash = yam.get('flash')
        if flash:
            self._flash_method = flash.get('method')
            if not self._flash_method:
                self.raise_self("Missing method in %s" % flash)
            if 'device' in flash:
                self._flash_device = self.lookup_part_ref(flash, 'device')

        send = yam.get('send')
        if send:
            self._send_method = send.get('method')
            if not self._send_method:
                self.raise_self("Missing method in %s" % send)
            if 'device' in send:
                self._send_device = self.lookup_part_ref(send, 'device')
            if self._send_method == 'edison':
                self._send_device.obj.set_recovery_method(
                    Part_usbboot.Method.RECOVERY_RESET_EXTRA)
            elif self._recovery:
                self._send_device.obj.set_recovery_method(
                    Part_usbboot.Method.RECOVERY_RESET)
            elif self._bootdev:
                self._send_device.obj.set_recovery_method(
                    Part_usbboot.Method.BOOTDEV_TS_RESET)

        self._ether_mac = None
        network = yam.get('network')
        if network:
            self._network_method = network.get('method')
            if not self._network_method:
                self.raise_self("Missing method in %s" % network)
            self._ether_mac = network.get('mac-address')

        self._username = yam.get('username')
        self._password = yam.get('password')

        blobs = yam.get('blobs')
        if blobs:
            self._blob_dest = blobs.get('dest')
            if not self._blob_dest:
                self.raise_self("Missing dest in %s" % blobs)
            self._blob_src = blobs.get('src')
            if not self._blob_src:
                self.raise_self("Missing src in %s" % blobs)

        # Read defconfigs
        builds = yam.get('builds')
        if not builds:
            self.raise_self("Missing builds")
        self._builds = []
        self._build_default = None
        for name, build_yam in builds.items():
            target = build_yam.get('target')
            if not target:
                self.raise_self("Missing target in %s" % build_yam)
            toolchain = build_yam.get('toolchain')
            if not toolchain:
                self.raise_self("Missing toolchain in %s" % build_yam)
            if build_yam.get('default'):
                self._build_default = target
                self._toolchain_default = toolchain
            self._builds.append([target, toolchain])
        if not self._build_default:
            self._build_default, self._toolchain_default = self._builds[0]

    def __str__(self):
        return 'dut %s' % self.name

    def raise_self(self, msg):
        self.lab.raise_self('%s: %s' % (str(self), msg))

    def get_detail(self, port=None):
        return self._desc

    def show(self):
        def show_partref(connection_name, partref):
            if partref:
                part, port = partref
                detail = collections.OrderedDict()
                part.get_all_detail(detail)
                out = ''
                for key, value in detail.items():
                   out += '  %s=%s' % (key, value)
                out_name = part.name
                if port is not None:
                    out_name += ' port %s' % port
                print('   %-15s: %-15s %s' % (connection_name, out_name, out))

        super().show()
        show_partref('Boot device', self._bootdev)
        show_partref('Console', self._cons)
        show_partref('Power', self._power)
        show_partref('Reset', self._reset)
        show_partref('Recovery', self._recovery)

    def set_power(self, power_on: bool, port: Optional[int] = None):
        if not self._power:
            self.raise_self('Power control not supported')
        self._power.obj.set_power(power_on, self._power.port)

    def check(self):
        # This does nothing at present
        return work.CheckResult(self, True, '')

    def emit(self):
        emit_list = []
        emit_list.append(self.emit_tbot())
        return emit_list

    def get_class_var_str(self, *part_refs):
        class_vars = {}
        for part_ref in part_refs:
            if part_ref:
                class_vars.update(part_ref.obj.get_py_class_vars(part_ref))
        vals = []
        for key, val in class_vars.items():
            if isinstance(val, str):
                val_str = '"%s"' % val
            elif val is not None and val > 0x1000:
                val_str= '%#x' % val
            else:
                val_str = str(val)
            vals.append('%s = %s' % (key, val_str))

        classvar_str = '\n    '.join(sorted(vals))
        return classvar_str

    def non_none(self, *partrefs):
        return [partref.obj for partref in partrefs if partref]

    def add_sequence(self, vals, prop_list, *partrefs, indent=8, template=None,
                     var=None, add_comma=False):
        if var is None:
            var = prop_list[0]
        calls = []
        for seq, prop in enumerate(prop_list):
            pcalls = []
            allow_dup = False #prop == 'delay'
            for partref in partrefs:
                if partref:
                    call = partref.obj.get_code(prop, prop_list[seq + 1:],
                                                partref)
                    if call and (allow_dup or call not in pcalls):
                        pcalls.append(call)
            calls += pcalls
        if not calls:
            if template:
                regex = re.compile('{%s}' % var)
                for seq, line in enumerate(template):
                    if regex.search(line):
                        template.pop(seq)
                        break
            else:
                calls.append('pass')
        out = ''
        if len(prop_list) == 1:
            calls = sorted(calls)
        for seq, call in enumerate(calls):
            out += '%s%s%s' % ('\n' + ' ' * indent if seq else '', call,
                               ',' if add_comma else '')
        vals[var] = out

    def emit_tbot(self):
        template = '''import tbot
from tbot.machine import board, channel, connector, linux
from tbot.tc import git, shell, uboot
from flash import Flash
from send import Send
{import_str}

class {class_name}UBootBuilder(uboot.UBootBuilder{builder_superclasses}):
    name = "{name}"
    defconfig = "{defconfig}_defconfig"
    toolchain = "{toolchain}"
    {blob_dest}
    {blob_src}
{add_blobs}

class {class_name}(
    connector.ConsoleConnector,
    board.PowerControl,
    board.Board,
    Flash,
    Send,
    {baseclass}
):
    name = "{name}"
    desc = "{desc}"
    {classvar_str}

    ether_mac = {ether-mac}

    def poweron(self) -> None:
        """Procedure to turn power on."""
        {poweron}

    def poweroff(self) -> None:
        """Procedure to turn power off."""
        {poweroff}

    def connect(self, mach) -> channel.Channel:
        """Connect to the board\'s serial interface."""
        {get_uart}
        return {connect_uart}

    def flash(self, repo: git.GitRepository) -> None:
        {preflash}
        self.flash_{flash_method}(repo)
        {postflash}

    def send(self, repo: git.GitRepository) -> None:
        {presend}
        self.send_{send_method}(repo)
        {postsend}


class {class_name}UBoot(
    board.Connector,
    board.UBootAutobootIntercept,
    board.UBootShell,
):
    prompt = "{prompt}"
    build = {class_name}UBootBuilder()


class {class_name}Linux(
    board.Connector,
    board.LinuxBootLogin,
    linux.Bash,
):
    username = "{username}"
    password = "{password}"


BOARD = {class_name}
UBOOT = {class_name}UBoot
LINUX = {class_name}Linux
'''.splitlines()
        partrefs = [self._bootdev, self._cons, self._power, self._power_button,
                    self._reset, self._recovery, self._send_device]
        todo_objs = self.non_none(*partrefs)
        imports = list(set([obj.get_py_imports() for obj in todo_objs]))
        superclasses = ''
        if self._blob_dest:
            imports.insert(0, 'from blobs import Blobs')
            superclasses = ', Blobs'
        import_str = '\n'.join(sorted([imp for imp in imports if imp]))

        classvar_str = self.get_class_var_str(*partrefs)

        vals = {
            'name': self.name,
            'class_name': self.name.title().replace('-', '_'),
            'import_str': import_str,
            'desc': self._desc,
            'classvar_str': classvar_str,
            'defconfig': self._build_default,
            'flash_method': self._flash_method,
            'send_method': self._send_method,
            'ether-mac': '"%s"' % self._ether_mac if self._ether_mac else None,
            'username': self._username if self._username else '',
            'password': self._password if self._password else '',
            'builder_superclasses': superclasses,
            'toolchain': self._toolchain_default,
            'prompt': self._prompt + ' ' if self._prompt else '=> ',
            }

        self.add_sequence(vals, ['baseclass'], *partrefs, template=template,
                          indent=4, add_comma=True)

        if self._blob_dest:
            vals.update ({
                'blob_dest': 'blob_dest = "%s"' % self._blob_dest,
                'blob_src': 'blob_src = "%s"' % self._blob_src,
                'add_blobs': '''
    def do_patch(self, repo: git.GitRepository) -> None:
        self.add_blobs(repo)
'''})
        else:
            regex = re.compile('{blob_src|blob_dest|add_blobs}')
            template = [line for line in template
                        if not regex.search(line)]

        self.add_sequence(vals, ['bootdev_dut', 'poweron', 'reset'],
                          self._bootdev, self._power, self._reset,
                          var='poweron')
        self.add_sequence(vals, ['poweroff', 'bootdev_ts'],
                          self._bootdev, self._power)
        if self._cons.obj._need_dut_power:
            self.add_sequence(vals, ['poweron', 'get_uart'],
                              self._power, self._cons, template=template,
                              var='get_uart')
        else:
            self.add_sequence(vals, ['get_uart'], self._cons, template=template)
        self.add_sequence(vals, ['preflash', 'bootdev_ts'],
                          self._bootdev, self._power, template=template,
                          var='preflash')
        self.add_sequence(vals, ['bootdev_dut',], self._bootdev,
                          template=template, var='postflash')
        self.add_sequence(vals, ['connect_uart'], self._cons, template=template)

        if (self._send_device and
            self._send_device.obj.recovery_method ==
                Part_usbboot.Method.RECOVERY_RESET):
            if self._reset:
                reset_type = ['setreset', 'clearreset']
            else:
                reset_type = ['clearpower', 'setpower']
            self.add_sequence(vals, ['presend', 'setrecovery',
                                     reset_type[0], 'delay',
                                     reset_type[1], 'delay',
                                     'clearrecovery'],
                              self._reset if self._reset else self._power,
                              self._recovery, template=template,
                              var='presend')
        else:
            self.add_sequence(vals, ['presend', 'bootdev_ts', 'reset'],
                              self._bootdev, self._power, template=template,
                              var='presend')
        self.add_sequence(vals, ['bootdev_dut',], self._bootdev,
                          template=template, var='postsend')

        out = '\n'.join(template).format(**vals)
        return work.EmitResult(self, 'tbot/%s.py' % self.name, out + '\n',
                               '# Generated by labman from %s' % str(self))

    def get_console(self):
        return self._cons

    def get_send_device(self):
        return self._send_device

    def setup_recovery(self, use_reset_method):
        "Enable recovery and assert reset (or power off)"""
        if use_reset_method:
            tout.Detail('%s: Recovery: Using reset/power' % str(self))
            self._recovery.obj.set_power(True, self._recovery.port)
            if self._recovery_extra:
                self._recovery_extra.obj.set_power(True,
                                                   self._recovery_extra.port)
            if self._reset:
                self._reset.obj.set_power(True, self._reset.port)
                time.sleep(.1)
                self._reset.obj.set_power(False, self._reset.port)
                time.sleep(.1)
            else:
                self._power.obj.set_power(False, self._power.port)
                time.sleep(2)
                self._power.obj.set_power(True, self._power.port)
                count = 0
                while not os.path.exists('/dev/usbdev-pcduino3') and count < 10:
                    time.sleep(.1)
                    count += .1
                print('\ncount', count)
                #time.sleep(2)  # pcduino3 takes >1s sometimes
            self._recovery.obj.set_power(False, self._recovery.port)
            if self._recovery_extra:
                self._recovery.obj.set_power(False, self._recovery_extra.port)
        else:
            tout.Detail('%s: Recovery: Using select_ts, power' % str(self))
            if not self._power:
                self.raise_self('No power control')
            if self._bootdev:
                self._bootdev.obj.select_ts()
            self._power.obj.set_power(False, self._power.port)
            time.sleep(1)
            self._power.obj.set_power(True, self._power.port)

    def reset_to_recovery(self, symlink):
        use_reset_method = (self._send_device and
            self._send_device.obj.recovery_method in
            [Part_usbboot.Method.RECOVERY_RESET_EXTRA,
             Part_usbboot.Method.RECOVERY_RESET])
        self.setup_recovery(use_reset_method)
