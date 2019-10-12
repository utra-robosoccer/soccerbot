#!/usr/bin/env python2

from time import sleep
from socket import socket, error, AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_BROADCAST, gethostname
import os

if __name__ == '__main__':
    s = socket(AF_INET, SOCK_DGRAM)             # Create UDP socket
    s.bind(('', 0))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)   # This is a broadcast socket

    while True:
        try:
            if 'IN_DOCKER_CONTAINER' in os.environ and os.environ['IN_DOCKER_CONTAINER'] == "true":
                data = "zombie_" + gethostname()
            else:
                data = gethostname()

            s.sendto(data, ('<broadcast>', 56789))
            sleep(5)
        except error as e:
            print 'robot announcement failed with error: {}'.format(e)
            sleep(5)
        except KeyboardInterrupt:
            s.close()
            exit(0)
