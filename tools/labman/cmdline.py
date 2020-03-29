# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>
#
# Command-line parser for labman
#

from argparse import ArgumentParser

def ParseArgs(argv):
    """Parse the labman command-line arguments

    Args:
        argv: List of string arguments
    Returns:
        Tuple (options, args) with the command-line options and arugments.
            options provides access to the options (e.g. option.debug)
            args is a list of string arguments
    """
    epilog = '''Labman managers your hardware lab.'''

    parser = ArgumentParser(epilog=epilog)
    parser.add_argument('-1', '--single-threaded',  action='store_true',
        help="Don't use threading to speed up operation")
    parser.add_argument('-D', '--debug', action='store_true',
        help='Enabling debugging (provides a full traceback on error)')
    parser.add_argument('-H', '--full-help', action='store_true',
        default=False, help='Display the README file')
    parser.add_argument('-l', '--lab', type=str,
        help='Select the lab description to use (yaml file)')
    parser.add_argument('-r', '--remote', type=str,
        help='Select the remote host to use')
    parser.add_argument('--toolpath', type=str, action='append',
        help='Add a path to the directories containing tools')
    parser.add_argument('-v', '--verbosity', default=1,
        type=int, help='Control verbosity: 0=silent, 1=warnings, 2=notices, '
        '3=info, 4=detail, 5=debug')

    subparser = parser.add_subparsers(dest='cmd')

    check_parser = subparser.add_parser('check', help='Check parts')
    check_parser.add_argument(
        '-p', '--parts', type=str,
         help='Select the parts to check (comma-separated)')

    emit_parser = subparser.add_parser('emit', help='Emit scripts')
    emit_parser.add_argument('-d', '--dut', type=str,
                             help='Select the DUT to emit')
    emit_parser.add_argument('-f', '--ftype', type=str,
                             help='Select the script type to emit (only tbot)')
    emit_parser.add_argument('-o', '--output-dir', type=str, default='lab_out',
                             help='Select the output directory')

    ls_parser = subparser.add_parser('ls', help='Show lab info')
    ls_parser.add_argument('args', nargs='*',
                           help="Things to list ('all', 'hubs', <part>)")

    prov_parser = subparser.add_parser('prov', help='Provision a lab component')
    prov_parser.add_argument('-c', '--component', type=str,
                             help='Select the component to provision')
    prov_parser.add_argument('-n', '--name', type=str,
                             help='Name to provision with')
    prov_parser.add_argument('-s', '--serial', type=str,
                             help='Serial number to provision with')

    scan_parser = subparser.add_parser('scan',
                                       help='Scan for unprovisioned devices')

    start_parser = subparser.add_parser('start', help='Start lab daemons')

    test_parser = subparser.add_parser('test', help='Run tests')
    test_parser.add_argument('-P', '--processes', type=int,
        help='set number of processes to use for running tests')
    test_parser.add_argument('-T', '--test-coverage', action='store_true',
        default=False, help='run tests and check for 100%% coverage')
    test_parser.add_argument('-X', '--test-preserve-dirs', action='store_true',
        help='Preserve and display test-created input directories; also '
             'preserve the output directory if a single test is run (pass test '
             'name at the end of the command line')
    test_parser.add_argument('tests', nargs='*',
                             help='Test names to run (omit for all)')

    return parser.parse_args(argv)
