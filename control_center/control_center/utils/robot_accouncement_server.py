import socket
import select
import Queue
from threading import Thread

SOCKET_QUEUE = Queue.Queue()


class RobotAnnouncementServer(Thread):
    def __init__(self, interval=5):
        super(RobotAnnouncementServer, self).__init__()
        self._interval = interval
        self._worker_func = self.run

    def run(self):
        while True:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            s.bind(('0.0.0.0', 56789))

            r, w, x = select.select([s], [], [])
            for i in r:
                test = i.recvfrom(1024)
                SOCKET_QUEUE.put(test)
