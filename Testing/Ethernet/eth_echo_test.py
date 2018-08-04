#import json
import numpy
import random
import socket
import sys
import time

# eth_echo_test.py sends message of given sizes over
# transmission layer protocols UDP or TCP to an echo server
# operating on the corresponding protocol. The time to
# receive the echoed packet is measured, giving the latency
# of transmission between the host PC and echo server.
# Multiple trials, sizes, and numbers of messages to send in
# sequence may be specified. Numerical time data for echoes
# is outputted in a JSON file, and statistical information
# is printed to the stdout console. Numerical data is to be
# later parsed for graphing and analysis.

# The script will be helpful to guage performance improvements on PC<->MCU
# transmission in reponse to config setting changes in CubeMX. It will also
# serve as a measure of how robust the system is (i.e. at
# what message size, and number of simultaneous transmissions
# does the system fail).

def gen_random_string(n):
    return ''.join([str(chr(ord(' ') + (random.randint(1, ord('~') - ord(' ')) % (ord('~') - ord(' '))))) for i in range(n)])

# Test parameters
MESSAGE_SIZES = [1, 10, 80, 100]
MESSAGE_NUMS_TEST_IN_SEQUENCE = [1, 10, 100, 500]
MESSAGE_NUM_TRIALS = 3

RESULTS_TIMES = []
TEST_FILE_NAME = "times"

# Network parameters
PROTOCOL = "TCP"
MCU_IP_ADDRESS = "192.168.0.43"
MCU_PORT = 7
HOST_PC_PORT = 7

# PC system paraeters
BUFFER_SIZE = 4096  # Size in bytes of buffer for PC to receive message
#TIMEOUT = 10000     # Time in ms for PC to timeout and fail a test if MCU hangs
TCP_RECEIVE_BUFFER_SIZE = 16

i_trial = 0
for msg_size in MESSAGE_SIZES:
    for num_echoes in MESSAGE_NUMS_TEST_IN_SEQUENCE:
        for i_trial in range(MESSAGE_NUM_TRIALS):
            sock = None

            if PROTOCOL == "UDP":
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            elif PROTOCOL == "TCP":
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            if PROTOCOL == None:
                sys.stderr.write("error: Must specify a protocol")
                exit(1)

            server_address = (MCU_IP_ADDRESS, MCU_PORT)

            if PROTOCOL == "UDP":
                sock.bind(('', HOST_PC_PORT))
            elif PROTOCOL == "TCP":
                sock.connect(server_address)

            message = gen_random_string(msg_size).encode()
            times = []

            print("\n--- NEW TEST")
            print("Sending \"{}\" (of size {} bytes) over {}".format(message, len(message), PROTOCOL))
            print("Measuring time taken for {} echoes".format(num_echoes))
            print("Trial {}".format(i_trial))

            try:
                if (PROTOCOL == "UDP"):
                    for i in range(num_echoes):
                        t0 = time.perf_counter()
                        sent = sock.sendto(message, server_address)
                        data, server = sock.recvfrom(BUFFER_SIZE)
                        t1 = time.perf_counter()
                        times.append(t1 - t0)
                elif (PROTOCOL == "TCP"):
                    for i in range(num_echoes):
                        t0 = time.perf_counter()
                        sent = sock.sendall(message)
                        amount_received = 0
                        amount_expected = len(message)
                        while amount_received < amount_expected:
                            data = sock.recv(TCP_RECEIVE_BUFFER_SIZE)
                            amount_received += len(data)
                        t1 = time.perf_counter()
                        times.append(t1 - t0)
            except Exception as e:
                sys.stderr.write("error: Exception {} while echoing".format(e))
                exit(1)
            finally:
                sock.close()

            f = open(TEST_FILE_NAME, "a")
            try:
                f.write('\n{}'.format(",".join([str(t) for t in times])))
            except Exception as e:
                sys.stderr.write("error: Exception {} while writing data".format(e))
                exit(1)
            finally:
                f.close()

            RESULTS_TIMES.extend(times)

            times_array = numpy.array(times)

            print('Took {} seconds for {} samples'.format(numpy.sum(times_array), num_echoes))
            print('Average echo time: {} seconds'.format(numpy.average(times_array)))
            print('Standard deviation: {} seconds'.format(numpy.std(times_array)))
            print('Maximum: {} seconds, Minimum: {} seconds'.format(numpy.amax(times_array), numpy.amin(times_array)))

print("Collected {} results".format(len(RESULTS_TIMES)))
