from threading import Thread
import time


class IntervalTimer(Thread):
    def __init__(self, worker_func, interval=5):
        super(IntervalTimer, self).__init__()
        self._interval = interval
        self._worker_func = worker_func

    def run(self):
        while True:
            self._worker_func()
            time.sleep(self._interval)
