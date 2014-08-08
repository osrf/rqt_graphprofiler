from threading import *
import sys
import time
import types

class CallbackTimer(object):
    """ Callback timer
    time -- in seconds
    """
    def __init__(self, time, callback):
        self._timeLength = time
        self._user_callback = callback
        self._lock = Lock()
        self._timer = None
        # variable that says if we are runnign the callback
        self._running_callback_flag = False

    def restart(self):
        has_lock = False
        while (not has_lock) and (not self._running_callback_flag):
            has_lock = self._lock.acquire(False)
        self._stop()
        t = self._start()
        if has_lock:
            self._lock.release()
        return t

    def start(self):
#         with self._lock:
#             return self._start()
        has_lock = False
        print_once = True
        while not has_lock:
            has_lock = self._lock.acquire(False)
            if (not has_lock) and print_once:
                print_once = False
                print "*********************************************************"
                print "*********************************************************"
                print "*********************************************************"
                print "Waiting to acquire lock"
                sys.stdout.flush()
                time.sleep(0)
        t = self._start()
        self._lock.release()
        return t

    def stop(self):
        with self._lock:
            return self._stop()

    def _start(self):
        if not isinstance(self._timer,types.NoneType):
            raise RuntimeError("%s thread already started."%(str(type(self._timer))))
        self._timer = Timer(self._timeLength, self._callback)
        self._timer.start()
        return time.time()

    def _stop(self):
        if self._timer is not None:
            self._timer.cancel()
        self._timer = None
        return time.time()

    def _callback(self):
        with self._lock:
            self._running_callback_flag = True
            try:
                self._user_callback()
            except:
                import traceback
                traceback.print_exc()
            self._running_callback_flag = False
