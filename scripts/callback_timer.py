from threading import *
import sys
import time

class CallbackTimer(object):
    """ Callback timer
    time -- in seconds
    """
    def __init__(self, time, callback):
        self._timeLength = time
        self._user_callback = callback
        self._lock = RLock()
        self._timer = None

    def restart(self):
        with self._lock:
            self.stop()
            return self.start()

    def start(self):
        with self._lock:
            self._timer = Timer(self._timeLength, self._callback)
            self._timer.start()
            return time.time()

    def stop(self):
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            return time.time()

    def _callback(self):
        with self._lock:
            try:
                self._user_callback()
            except:
                import traceback
                traceback.print_exc()
