# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>
#
# Main control for labman
#

from lab import Lab

# Used to access the lab for testing
test_lab = None

def Labman(args, lab=None):
    if args.lab:
        if not lab:
            global test_lab
            lab = Lab()
            test_lab = lab
        lab.read(args.lab)

    if args.single_threaded:
        lab.set_num_threads(0)

    if args.remote:
        lab.set_remote(args.remote)

    lab.setup_state_dir()

    if args.cmd == 'check':
        lab.check(args.parts)
    elif args.cmd == 'emit':
        lab.emit(args.output_dir, args.dut, args.ftype)
    elif args.cmd == 'ls':
        lab.show(args.args)
    elif args.cmd == 'prov':
        lab.provision(args.component, args.name, args.serial)
    elif args.cmd == 'start':
        lab.start_daemons()
    elif args.cmd == 'scan':
        lab.scan()
