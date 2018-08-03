import json
import numpy
import socket
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

# Test parameters
MESSAGE_NUM_TRIALS = 3
MESSAGE_SIZES = [1, 10, 80, 100]
MESSAGE_NUMS_TEST_IN_SEQUENCE = [1, 10, 100, 500]

RESULTS_TIMES = []
TEST_FILE_NAME = "eth_echo_test_results_times.json"

# Network parameters
PROTOCOL = "UDP"
MCU_IP_ADDRESS = "192.168.0.55"
MCU_PORT = 7
HOST_PC_PORT = 7

# PC system paraeters
BUFFER_SIZE = 4096  # Size in bytes of buffer for PC to receive message
TIMEOUT = 10000     # Time in ms for PC to timeout and fail a test if MCU hangs
