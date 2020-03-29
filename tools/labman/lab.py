# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

"""Implementation of a lab, a collection of devices for running tests"""

import collections
import io
import os
import subprocess
import time
import yaml

from labman.parts import Parts
from labman.ptypes.hub import Part_hub
from labman.ptypes import sdwire
from labman.work import Work
from patman import command
from patman import terminal
from patman import tout

def join(loader, node):
    seq = loader.construct_sequence(node)
    return ''.join([str(i) for i in seq])

class Lab:
    """A colection of DUTs, builders and the infrastructure to connect them

    Properties:
        _name: Name of the lab. This should be short, ideally a nickname.
        _desc: Description for the lab
        _duts: List of available DUTs
        _sdwires: List of available SWwires
        _remote: Remote lab machine to run commands on (None for local)
    """
    def __init__(self):
        self._name = None
        self._desc = None
        self._remote = None
        self._num_threads = 12
        self._state_dir = '/tmp/labmanst'

    def read(self, fname):
        """Read the lab definition from a file

        Args:
            fname: Filename to read from (in yaml format)
        """
        with open(fname) as inf:
            data = inf.read()
        new_data = self.add_references(data)
        #with open('/tmp/asc', 'w') as fd:
            #fd.write(new_data)
        yam = yaml.load(new_data, Loader=yaml.Loader)
        self.load(yam)

    def load(self, yam):
        """Load the lab definition from a dictionary

        Args:
            yam: Dictionary to read
        """
        self._name = yam.get('name')
        if not self._name:
            self.raise_self('Missing name')
        self._desc = yam.get('desc')
        self._parts = Parts(self)
        self._parts.scan_ptypes()
        self._parts.load_all(yam)

    def add_references(self, yam):
        out = io.StringIO()
        add_name = None
        anchor_indent = 0
        min_indent = 0
        do_add = False
        for line in yam.splitlines():
            indent = len(line) - len(line.lstrip())
            #print('indent', indent, min_indent, file=out)
            if line and indent < min_indent:
                add_name = None
            if do_add and add_name:
                print('%s_anchor: %s' % (' ' * anchor_indent, add_name),
                      file=out)
                do_add = False
            if '&' in line:
                pos = line.find(':')
                anchor_indent = line[:pos].count(' ') + 2
                if line[2] != ' ':
                    add_name = line[:pos].strip()
                    min_indent = anchor_indent - 2
                do_add = True
            print(line, file=out)
        return out.getvalue()

    def show(self, which):
        """Show information about the lab"""
        SHOW_HUB, SHOW_ONE, SHOW_ALL = range(3)

        show_names = {
            'all': SHOW_ALL,
            'one': SHOW_ONE,
            'hubs': SHOW_HUB,
            }
        if which:
            show = show_names.get(which[0])
            if show == SHOW_ONE:
                if len(which) < 2:
                    raise ValueError("Missing 'part' argument")
                part_name = which[0]
            elif show is None:
                show = SHOW_ONE
                part_name = which[0]
        else:
            show = SHOW_ALL

        part_list = self._parts.get_list()
        part_dict = {part.name: part for part in part_list}
        if show == SHOW_ALL:
            print('%15s  %s' % ('Name', 'Description'))
            print('%15s  %s' % ('=' * 15, '=' * 30))
            for name in sorted(part_dict.keys()):
                part_dict[name].show_line()
        elif show == SHOW_ONE:
            part = part_dict.get(part_name)
            if not part:
                raise ValueError("Unknown part '%s'" % part_name)
            part.show()
        elif show == SHOW_HUB:
            hub_dict = self._parts.get_parts_by_hub(part_list)
            print('%15s  %-5s  %-12s  %s' %
                  ('Hub', 'Port', 'Part', 'Description'))
            for name, hub in part_dict.items():
                if isinstance(hub, Part_hub):
                    print('%15s  %-5s  %-12s  %s' % (hub.name, '', '',
                                                     hub.get_detail()))
                    for portnum in sorted(hub._ports_by_num.keys()):
                        port = hub._ports_by_num[portnum]
                        part = hub_dict[hub].get(port)
                        print('%15s  %-5s  %-12s  %s' %
                              ('', portnum, part.name if part else '-',
                               part.get_detail(port) if part else ''))
                    print()

        else:
            raise ValueError("Unknown 'ls' argument '%s' (%s)" %
                             (which[0], ', '.join(show_names.keys())))

    def __str__(self):
        """Produce a string representing the lab

        Returns:
            String that can be used by the user to identify the lab
        """
        return 'lab %s' % self._name

    def raise_self(self, msg):
        """Raise an exception related to this lab

        Args:
            msg: Message to put in the exception
        """
        raise ValueError('%s: %s' % (str(self), msg))

    def set_num_threads(self, num_threads):
        self._num_threads = num_threads

    def set_remote(self, remote):
        """Set the remote host to use for running commands

        Args:
            remote: Remote host (None for local)
        """
        self._remote = remote

    def run_command(self, *args, binary=False):
        """Run a command either on the local or remote host

        The command is run through ssh if using a remote host.

        Args:
            args: List of arguments to the command
        """
        if self._remote:
            args = ['ssh', self._remote] + list(args)
        tout.Detail('Command: %s' % ' '.join(args))
        return command.RunCheck(*args, binary=binary)

    def run_subprocess(self, *args):
        if self._remote:
            args = ['ssh', self._remote] + list(args)
        return subprocess.Popen(args, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)

    def emit(self, outdir, dut_name, ftype):
        """Emit a file from the lab

        The output is write to stdout

        Args:
            outdir: Path to output directory for files
            dut_name: Name of dut to emit
            ftyppe: File type to emit ('tbot')
        """
        emit_list = self._parts.get_list(dut_name)
        work = Work(self._num_threads)
        results = work.run(Work.Oper.OP_EMIT, emit_list)
        out_files = {}
        for obj, emit_results in results.items():
            if emit_results:
                for result in emit_results:
                    if result:
                        if result.fname not in out_files:
                            out_files[result.fname] = result.header + '\n\n'
                        out_files[result.fname] += result.data

        for fname, data in out_files.items():
            path = os.path.join(outdir, fname)
            dirs = os.path.dirname(path)
            os.makedirs(dirs, exist_ok=True)
            with open(path, 'w') as outf:
                outf.write(data)

    def provision(self, component, name, serial, test_obj=None):
        """Provision a new component for the lab

        Args:
            component: String compatible type ('sdwire')
            name: Name to use for component
            serial: Serial number to assign to component
            test_obj: Sdwire object to use for testing
        """
        if component == 'sdwire':
            sdw = test_obj(name) if test_obj else sdwire.Part_sdwire()
            sdw.name = name
            sdw.lab = self
            sdw.parts = None
            sdw.provision(serial)
        else:
            self.raise_self("Unknown component '%s'" % component)

    def check(self, parts=None):
        """Run a check on a lab

        This checks all components to make sure that they are working. It
        produces a list of problems along with the error found
        """
        check_list = self._parts.get_list(parts.split(',') if parts else None)
        all_deps = set()
        for part in check_list:
            deps = part.get_deps()
            all_deps |= deps
        all_deps -= set(check_list)
        if all_deps:
            tout.Info('Adding dependent parts: %s' %
                      ' '.join([part.name for part in all_deps]))
            check_list += list(all_deps)

        work = Work(self._num_threads)
        check_list = list(reversed(check_list))
        results = work.run(Work.Oper.OP_CHECK, check_list, progress=True,
                           allow_defer=True)

        good = 0
        bad = 0
        missing = 0
        for obj, result in results.items():
            if result in (None, False):
                missing += 1
            elif result.good:
                good += 1
            else:
                bad += 1
        terminal.PrintClear()
        print('Good %d, bad %d, not tested %d' % (good, bad, missing))

        # Set up a dict containing all broken hubs as keys. The value is a list
        # of Part objects on that hub
        bad_hub_parts = {}

        # Set up a dict containing all non-broken hubs. The value is a dict:
        #    key: Port number
        #    value: Part
        bad_parts_by_port = {}
        for obj, result in results.items():
            if result and isinstance(obj, Part_hub):
                if result.good:
                    bad_parts_by_port[obj] = {}
                else:
                    bad_hub_parts[obj] = []

        # Now add the list of parts attached to each broken hub
        for obj, result in results.items():
            if result and not result.good:
                if obj.hub:
                    if obj.hub.obj in bad_hub_parts:
                        bad_hub_parts[obj.hub.obj].append(obj)
                    else:
                        if obj.hub.obj in bad_parts_by_port:
                            bad_parts_by_port[obj.hub.obj][obj.hub.port] = obj

        # Print out broken devices. If they are connected to a broken hub,
        # just print them in a list with that hub.
        for obj, result in results.items():
            if result and not result.good:
                if obj.hub and obj.hub.obj in bad_hub_parts:
                    pass
                else:
                    print('   %s: %s' % (obj.name, result.msg))
                    if obj in bad_hub_parts:
                        print('      Resulting failures: %s' %
                              ', '.join(bad.name for bad in bad_hub_parts[obj]))
        if missing:
            print('\nNot tested:')
            for obj, result in results.items():
                if not result:
                    print('   %s' % obj.name)

        # Now print bad objects sorted by hub/port
        for hub, ports in bad_parts_by_port.items():
            if ports:
                print("\nFix list for hub '%s'" % hub.name)
                out = {}
                for port in sorted(ports.keys()):
                    out[hub.get_port_name(port)] = ports[port].name
                for port in sorted(out.keys()):
                    print('   %2s: %s' % (port, out[port]))

    def start_daemons(self):
        """Start daemons used by the lab

        This starts daemons used by the lab, such as servod.
        """
        start_list = self._parts.get_list()
        work = Work(self._num_threads)
        results = work.run(Work.Oper.OP_START, start_list)

        good = 0
        bad = 0
        missing = 0
        for obj, result in results.items():
            if result is None:
                missing += 1
            elif result.good:
                good += 1
            else:
                bad += 1
        print('Good %d, bad %d' % (good, bad))
        for obj, result in results.items():
            if result and not result.good:
                print('   %s: %s' %
                      (obj.name, 'good' if result.good else 'bad'))

        try:
            print('ready - press Ctrl-C to stop')
            while True:
                time.sleep(1)
        except Exception as e:
            print('exception', e)
        finally:
            results = work.run(Work.Oper.OP_STOP, start_list)

    def setup_state_dir(self):
        if not os.path.exists(self._state_dir):
            os.mkdir(self._state_dir)

    def scan(self):
        self._parts.scan()

    def get_usb_files(self, phys, *prop_list):
        args = ['/sys/bus/usb/devices/%s/%s' %
                (phys, prop) for prop in prop_list]
        result = self.run_command('cat', *args)
        lines = result.stdout.splitlines()
        if len(lines) != len(prop_list):
            return None
        props = {}
        for seq, val in enumerate(result.stdout.splitlines()):
            props[prop_list[seq]] = val
        return props

    def get_usb_dir_contents(self, phys, dirname):
        arg = '/sys/bus/usb/devices/%s/%s' % (phys, dirname)
        result = self.run_command('ls', arg)
        lines = result.stdout.splitlines()
        return lines

    def check_usb_exists(self, phys, pattern):
        arg = '/sys/bus/usb/devices/%s/%s' % (phys, pattern)
        result = self.run_command('ls', '-d', arg)
        if result.return_code:
            return None
        return result.stdout.splitlines()[0]
