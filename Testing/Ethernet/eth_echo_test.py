from datetime import datetime
import json
import numpy
import random
import socket
import sys
import time

import scheddl

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

ETH_ECHO_TEST = {
    "name": "",
    "config": {
        "message_sizes": "",
        "message_nums_test_in_sequence": "",
        "message_num_trials": "",
        "protocol": "",
        "mcu_ip_address": "",
        "mcu_port": "",
        "host_pc_port": "",
        "buffer_size": "",
        "tcp_receive_buffer_size": "",
        "scheduling_type": "",
        "scheduling_params": {
            "scheddl_setting": "",
            "scheddl_runtime": "",
            "scheddl_deadline": "",
            "scheddl_period": ""
        }
    },
    "tests": []
}

DATE_TIME = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")

# Test parameters
#MESSAGE_SIZES = [1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400]
MESSAGE_SIZES = [80]
MESSAGE_NUMS_TEST_IN_SEQUENCE = [1]
MESSAGE_NUM_TRIALS = 100

# Network parameters
PROTOCOL = "UDP"
MCU_IP_ADDRESS = "10.0.0.43"
MCU_PORT = 7
HOST_PC_PORT = 7

# PC system paraeters
BUFFER_SIZE = 4096  # Size in bytes of buffer for PC to receive message
#TIMEOUT = 10000     # Time in ms for PC to timeout and fail a test if MCU hangs
TCP_RECEIVE_BUFFER_SIZE = 16

# Scheduling parameters
SCHEDDL_SETTING = ""
SCHEDDL_RUNTIME  = 300000000000
SCHEDDL_DEADLINE = 600000000000
SCHEDDL_PERIOD   = 600000000000

ETH_ECHO_TEST["name"] = "eth_echo_test_{}_{}.json".format(PROTOCOL, DATE_TIME)
ETH_ECHO_TEST["config"]["message_sizes"] = str(MESSAGE_SIZES)
ETH_ECHO_TEST["config"]["message_nums_test_in_sequence"] = str(MESSAGE_NUMS_TEST_IN_SEQUENCE)
ETH_ECHO_TEST["config"]["message_num_trials"] = MESSAGE_NUM_TRIALS
ETH_ECHO_TEST["config"]["protocol"] = PROTOCOL
ETH_ECHO_TEST["config"]["mcu_ip_address"] = MCU_IP_ADDRESS
ETH_ECHO_TEST["config"]["mcu_port"] = MCU_PORT
ETH_ECHO_TEST["config"]["host_pc_port"] = HOST_PC_PORT
ETH_ECHO_TEST["config"]["buffer_size"] = BUFFER_SIZE
ETH_ECHO_TEST["config"]["tcp_receive_buffer_size"] = TCP_RECEIVE_BUFFER_SIZE
ETH_ECHO_TEST["config"]["scheduling_type"] = "scheddl_python_module"
ETH_ECHO_TEST["config"]["scheduling_params"]["scheddl_setting"] = SCHEDDL_SETTING
ETH_ECHO_TEST["config"]["scheduling_params"]["scheddl_runtime"] = SCHEDDL_RUNTIME
ETH_ECHO_TEST["config"]["scheduling_params"]["scheddl_deadline"] = SCHEDDL_DEADLINE
ETH_ECHO_TEST["config"]["scheduling_params"]["scheddl_period"] = SCHEDDL_PERIOD

if SCHEDDL_SETTING == "deadline":
    scheddl.set_deadline(SCHEDDL_RUNTIME, SCHEDDL_DEADLINE, SCHEDDL_PERIOD, scheddl.RESET_ON_FORK)
    print("Running with SCHEDDL_SETTING \"deadline\"")

i_trial = 0
for msg_size in MESSAGE_SIZES:
    for num_echoes in MESSAGE_NUMS_TEST_IN_SEQUENCE:
        for i_trial in range(MESSAGE_NUM_TRIALS):
            test = {
                "msg_size": msg_size,
                "num_echoes": num_echoes,
                "trial": i_trial,
                "message": gen_random_string(msg_size),
                "times": ""
            }

            sock = None

            if PROTOCOL == "UDP":
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            elif PROTOCOL == "TCP":
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            else:
                sys.stderr.write("error: Must specify a protocol")
                exit(1)

            server_address = (MCU_IP_ADDRESS, MCU_PORT)

            if PROTOCOL == "UDP":
                sock.bind(('', HOST_PC_PORT))
            elif PROTOCOL == "TCP":
                sock.connect(server_address)

            message = test["message"].encode()
            times = []

            print("---- Running {} test: message_size: {} | num_echoes: {} | trial: {}".format(PROTOCOL, msg_size, num_echoes, i_trial))
            # print("    ---- Sending message of size {} bytes".format(len(message)))

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

            times_string = ",".join([str(dt) for dt in times])
            test["times"] = times_string

            ETH_ECHO_TEST["tests"].append(test)
            
            times_array = numpy.array(times)

            print('    ---- Total time: {} s'.format(numpy.sum(times_array)))
            print('    ---- Average echo time: {} s'.format(numpy.average(times_array)))
            print('    ---- Standard deviation: {} s'.format(numpy.std(times_array)))
            print('    ---- Maximum: {} s, Minimum: {} s'.format(numpy.amax(times_array), numpy.amin(times_array)))
            
            time.sleep(0.05)

print("Collected {} results".format(len(ETH_ECHO_TEST["tests"])))

with open(ETH_ECHO_TEST["name"], "w") as test_results_json:
    json.dump(ETH_ECHO_TEST, test_results_json)


