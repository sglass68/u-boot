## SPDX-License-Identifier: GPL-2.0+
# Copyright 2020 Google LLC
# Written by Simon Glass <sjg@chromium.org>

"""Handling of multi-threaded work"""

import collections
from enum import Enum, auto
import queue
import threading

from patman import terminal

CheckResult = collections.namedtuple('CheckResult', 'obj,good,msg')
EmitResult = collections.namedtuple('EmitResult', 'obj,fname,data,header')
StartResult = collections.namedtuple('StartResult', 'obj,good')

class CheckerThread(threading.Thread):
    """Thread to a run an operation on a lab component"""
    def __init__(self, work, thread_num):
        """Set up a new builder thread"""
        threading.Thread.__init__(self)
        self.work = work
        self.thread_num = thread_num

    def run(self):
        while True:
            obj = self.work.queue.get()
            result = self.work.do_oper(obj)
            self.work.add_result(obj, result)
            self.work.queue.task_done()


class ResultThread(threading.Thread):
    """This thread processes results from chceker threads.

    It simply passes the results on to the Work object. There is only one
    result thread, and this helps to serialise the build output.
    """
    def __init__(self, work):
        """Set up a new result thread

        Args:
            builder: Builder which will be sent each result
        """
        threading.Thread.__init__(self)
        self.work = work

    def run(self):
        """Called to start up the result thread.

        We collect the next result obj and pass it on to the build.
        """
        while True:
            obj, result = self.work.out_queue.get()
            self.work.store_result(obj, result)
            self.work.out_queue.task_done()


class Work:
    """Class to handle queuing and running work

    Properties:
        _threads: Threads to use for running checks
        _num_threads: Number of threads to use (0 = single-tasking)
        queue: Queue of jobs for thread, each an object whose selected method
            needs to be run
        out_queue: Queue of results from threads, each
    """
    class Oper(Enum):
        # Check lab components are connected and working properly
        OP_CHECK = auto()
        # Emit tbot and pytest scripts for the lab
        OP_EMIT = auto()
        # Start daemons used by lab components
        OP_START = auto()
        # Stop daemons used by lab components
        OP_STOP = auto()

    def __init__(self, num_threads):
        self._num_threads = num_threads
        self._threads = []
        self.queue = None
        self.out_queue = None
        self._results = {}
        self._oper = None
        self._good = 0
        self._bad = 0
        self._col = terminal.Color()
        self._progress = False
        self._allow_defer = False
        self._lock = threading.Lock()
        self._waiting = {}

    def run_with_threads(self, check_list):
        """Run the work using threads

        This creates some threads and queues and runs the work through those
        """
        self.queue = queue.Queue()
        for obj in check_list:
            self.queue.put(obj)

        # Output Queue
        self.out_queue = queue.Queue()

        # Create threads
        for i in range(self._num_threads):
            thread = CheckerThread(self, i)
            thread.setDaemon(True)
            thread.start()
            self._threads.append(thread)

        thread = ResultThread(self)
        thread.setDaemon(True)
        thread.start()
        self._threads.append(thread)

        # Wait for all tasks to complete
        self.queue.join()

        self.out_queue.join()

    def do_oper(self, obj):
        if self._allow_defer:
            required_set = obj.get_deps()
            required_set &= self._defer_set
            with self._lock:
                done_set = self._results.keys()
                missing = required_set - done_set
                if missing:
                    self._waiting[obj] = missing
                    return False
        if self._oper is self.Oper.OP_CHECK:
            return obj.check()
        elif self._oper is self.Oper.OP_EMIT:
            return obj.emit()
        elif self._oper is self.Oper.OP_START:
            return obj.start()
        elif self._oper is self.Oper.OP_STOP:
            return obj.stop()

    def store_result(self, obj, result):
        """Store the result from checking a component

        Args:
            result: Result object
        """
        def unblock_waiting(ready_obj):
            for obj in list(self._waiting.keys()):
                if ready_obj in self._waiting[obj]:
                    self._waiting[obj].remove(ready_obj)
                if not self._waiting[obj]:
                    del self._waiting[obj]
                    if self._num_threads:
                        self.queue.put(obj)

        if result is not False:
            with self._lock:
                self._results[obj] = result
                unblock_waiting(obj)

        if self._progress:
            if result:
                if result.good:
                    self._good += 1
                else:
                    self._bad += 1
            line = '\r' + self._col.Color(self._col.GREEN, '%3d' % self._good)
            line += self._col.Color(self._col.YELLOW, '%5d' %
                                    len(self._waiting))
            line += self._col.Color(self._col.RED, '%5d' % self._bad)

            line += ' /%-5d  ' % self._count
            upto = self._good + self._bad
            remaining = self._count - upto
            if remaining:
                line += self._col.Color(self._col.MAGENTA, ' -%-3d  ' %
                                        remaining)
            else:
                line += ' ' * 8
            terminal.PrintClear()
            terminal.Print(line, newline=False, limit_to_line=True)

    def add_result(self, obj, result):
        """Add a result obtained by a thread

        This is called from threads so uses an output queue

        Args:
            result: Result to add
        """
        if self.out_queue:
            self.out_queue.put([obj, result])
        else:
            self.store_result(obj, result)

    def run(self, oper, check_list, progress=False, allow_defer=False):
        """Run through the work provided and collect the results

        Args:
            oper (int): Operation to run

        Returns:
            Dict of Result objects, with Result.obj.name as the key
        """
        self._progress = progress
        self._allow_defer = allow_defer
        self._defer_set = set(check_list)
        self._count = len(check_list)
        self._oper = oper
        self._results = {}
        self._good = 0
        self._bad = 0
        self._waiting = {}
        if self._num_threads:
            self.run_with_threads(check_list)
        else:
            todo = check_list
            while todo:
                redo = []
                for obj in todo:
                    result = self.do_oper(obj)
                    self.add_result(obj, result)
                    if result is False:
                        redo.append(obj)
                todo = redo
        return self._results
