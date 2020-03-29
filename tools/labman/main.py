#!/usr/bin/python3

# SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC

"""Lab manager - configuration and maintenance of a lab"""

import os
import sys
import traceback
import unittest

our_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(our_path, '..'))

from patman import test_util
from patman import tout

import cmdline
import control

def RunTests(debug, verbosity, processes, test_preserve_dirs, args, toolpath):
    """Run the functional tests and any embedded doctests

    Args:
        debug: True to enable debugging, which shows a full stack trace on error
        verbosity: Verbosity level to use
        test_preserve_dirs: True to preserve the input directory used by tests
            so that it can be examined afterwards (only useful for debugging
            tests). If a single test is selected (in args[0]) it also preserves
            the output directory for this test. Both directories are displayed
            on the command line.
        processes: Number of processes to use to run tests (None=same as #CPUs)
        args: List of positional args provided to binman. This can hold a test
            name to execute (as in 'binman test testSections', for example)
        toolpath: List of paths to use for tools
    """
    from labman.test.console_test import ConsoleTest
    from labman.test.control_test import ControlTest
    from labman.test.dut_test import DutTest
    from labman.test.lab_test import LabTest
    from labman.test.sdwire_test import SdwireTest
    import doctest

    result = unittest.TestResult()
    for module in []:
        suite = doctest.DocTestSuite(module)
        suite.run(result)

    sys.argv = [sys.argv[0]]
    if debug:
        sys.argv.append('-D')
    if verbosity:
        sys.argv.append('-v%d' % verbosity)
    if toolpath:
        for path in toolpath:
            sys.argv += ['--toolpath', path]

    test_name = args and args[0] or None
    suite = unittest.TestSuite()
    loader = unittest.TestLoader()
    for module in (ControlTest, ConsoleTest, DutTest, LabTest, SdwireTest):
        # Test the test module about our arguments, if it is interested
        if hasattr(module, 'setup_test_args'):
            setup_test_args = getattr(module, 'setup_test_args')
            setup_test_args(preserve_indir=test_preserve_dirs,
                preserve_outdirs=test_preserve_dirs and test_name is not None,
                toolpath=toolpath, verbosity=verbosity)
        if test_name:
            try:
                suite.addTests(loader.loadTestsFromName(test_name, module))
            except AttributeError:
                continue
        else:
            suite.addTests(loader.loadTestsFromTestCase(module))
    if test_util.use_concurrent and processes != 1:
        concurrent_suite = ConcurrentTestSuite(suite,
                fork_for_tests(processes or multiprocessing.cpu_count()))
        concurrent_suite.run(result)
    else:
        suite.run(result)

    # Remove errors which just indicate a missing test. Since Python v3.5 If an
    # ImportError or AttributeError occurs while traversing name then a
    # synthetic test that raises that error when run will be returned. These
    # errors are included in the errors accumulated by result.errors.
    if test_name:
        errors = []

        for test, err in result.errors:
            if ("has no attribute '%s'" % test_name) not in err:
                errors.append((test, err))
            result.testsRun -= 1
        result.errors = errors

    print(result)
    for test, err in result.errors:
        print(test.id(), err)
    for test, err in result.failures:
        print(err, result.failures)
    if result.skipped:
        print('%d labman test%s SKIPPED:' %
              (len(result.skipped), 's' if len(result.skipped) > 1 else ''))
        for skip_info in result.skipped:
            print('%s: %s' % (skip_info[0], skip_info[1]))
    if result.errors or result.failures:
        print('labman tests FAILED')
        return 1
    return 0

def RunTestCoverage():
    """Run the tests and check that we get 100% coverage"""
    test_util.RunTestCoverage('tools/labman/main.py', None,
            ['tools/patman/*.py', '*test/*', '*main.py'], None)

def RunLabman(args):
    """Main entry point to labman once arguments are parsed

    Args:
        args: Command line arguments Namespace object
    """
    ret_code = 0

    if not args.debug:
        sys.tracebacklimit = 0

    if args.cmd == 'test':
        if args.test_coverage:
            RunTestCoverage()
        else:
            ret_code = RunTests(args.debug, args.verbosity, args.processes,
                                args.test_preserve_dirs, args.tests,
                                args.toolpath)

    else:
        try:
            tout.Init(args.verbosity)
            ret_code = control.Labman(args)
        except Exception as e:
            print('labman: %s' % e)
            if args.debug:
                print()
                traceback.print_exc()
            ret_code = 1
        finally:
            tout.Uninit()
    return ret_code


if __name__ == "__main__":
    args = cmdline.ParseArgs(sys.argv[1:])

    ret_code = RunLabman(args)
    sys.exit(ret_code)
