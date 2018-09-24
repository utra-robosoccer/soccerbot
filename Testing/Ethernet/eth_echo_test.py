# the following are available with Python Anaconda 3
from datetime import datetime
import json
import numpy
import random
import socket
import sys
import time

# scheddl from the fork https://github.com/rfairley/scheddl must be installed
# to your python distribution by yourself
import scheddl

# eth_echo_test.py sends message of given sizes over
# transmission layer protocols UDP or TCP to an echo server
# operating on the corresponding protocol. The time to
# receive the echoed packet is measured, giving the latency
# of transmission between the host PC and echo server.
# Multiple trials, sizes, and numbers of messages to send in
# sequence between thread yields of this program may be specified.
# Numerical time data for echoes is outputted in a JSON file, and
# test progress is printed to the console. Numerical data is to be
# later parsed for graphing and analysis.

# The script will be helpful to guage performance improvements on PC<->MCU
# transmission in reponse to config setting changes in CubeMX. It will also
# serve as a measure of how robust the system is (i.e. at
# what message size, and number of simultaneous transmissions
# does the system fail).
#
# Command line arguments will make it easier to run eth_echo_test.py
# as part of a larger testing script (written in e.g. bash).
#
# TODO: skeleton csv file to include in repo as a method of plotting
# TODO: readme for how to use all the Ethernet testing files

def gen_random_string(n):
    return ''.join([str(chr(ord(' ') + (random.randint(1, ord('~') - ord(' ')) % (ord('~') - ord(' '))))) for i in range(n)])

def clean_name_arg(name):
    clean_name = []
    for i in range(len(name)):
        clean_name.append("_") if name[i] == " " else clean_name.append(name[i])
    return "".join(clean_name)

def get_next_arg(arg, arguments):
    next_arg = ""
    try:
        next_arg = arguments[arguments.index(arg) + 1]
        if name_arg[0] == "-":
            exit(1)
    except:
        print("argument error")
        exit(1)
    finally:
        return next_arg


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
MESSAGE_SIZES = [1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400]
#MESSAGE_SIZES = [80]
MESSAGE_NUMS_TEST_IN_SEQUENCE = [1]
MESSAGE_NUM_TRIALS = 10

# Network parameters
PROTOCOL = "UDP"
# MCU_IP_ADDRESS = "10.0.0.43" # Static IP address set on the MCU
MCU_IP_ADDRESS = "127.0.0.1" # Loopback device - OK for testing without board attached
MCU_PORT = 7
HOST_PC_PORT = 7

# PC system paraeters
BUFFER_SIZE = 4096  # Size in bytes of buffer for PC to receive message
#TIMEOUT = 10000     # Time in ms for PC to timeout and fail a test if MCU hangs
TCP_RECEIVE_BUFFER_SIZE = 16
TRIAL_INTERVAL = 0.05 # milliseconds

# Scheduling parameters
SCHEDDL_SETTING = "rr"
SCHEDDL_RUNTIME  = 300000000000
SCHEDDL_DEADLINE = 600000000000
SCHEDDL_PERIOD   = 600000000000
SCHEDDL_PRIORITY = 1

# Computed parameters
OUTPUT_JSON_NAME = "eth_echo_test_{}_{}.json".format(PROTOCOL, DATE_TIME)

# Overwrite any variables if command line arguments are given
arguments = sys.argv[1:]
for arg in arguments:
    if arg == "-h":
        print("options:\n    -h    show help output\n    -n    test name, also used as test output file name\n    -t    number of trials for each message size\n    -i    interval of time in milliseconds to wait after each trial\n    -p    networking protocol to use (UDP or TCP)\n    --mcu-ipaddr    the IP address of the MCU\n    --mcu-port    port to send packets to on the MCU\n    --pc-port    port on the PC to bind socket to\n")
        exit(0)

args = {}
for arg in arguments:
    if arg == "-n":
        args["name"] = get_next_arg(arg, arguments)
        args["name"] = clean_name_arg(args["name"]) + ".json"
    elif arg == "-s":
        args["sizes"] = get_next_arg(arg, arguments)
        args["sizes"] = args["sizes"].split(",")
    elif arg == "-t":
        args["num_trials"] = get_next_arg(arg, arguments)
        args["num_trials"] = int(args["num_trials"])
    elif arg == "-i":
        args["trial_interval"] = get_next_arg(arg, arguments)
        args["trial_interval"] = float(int(args["trial_interval"])) / 1000
    elif arg == "-p":
        args["protocol"] = get_next_arg(arg, arguments)
    elif arg == "--mcu-ipaddr":
        args["mcu_ipaddr"] = get_next_arg(arg, arguments)
    elif arg == "--mcu-port":
        args["mcu_port"] = get_next_arg(arg, arguments)
        args["mcu_port"] = int(args["mcu_port"])
    # else if arg == "--pc-ipaddr": just keep in mind for now
    elif arg == "--pc-port":
        args["pc_port"] = get_next_arg(arg, arguments)
        args["pc_port"] = int(args["pc_port"])

# XXX: need to actually test passing in all of these from command line
if "name" in args:
    OUTPUT_JSON_NAME = args["name"]
elif "sizes" in args:
    MESSAGE_SIZES = args["sizes"]
elif "num_trials" in args:
    MESSAGE_NUM_TRIALS = args["num_trials"]
elif "trial_interval" in args:
    TRIAL_INTERVAL = args["trial_interval"]
elif "protocol" in args:
    PROTOCOL = args["protocol"]
elif "mcu_ipaddr" in args:
    MCU_IP_ADDRESS = args["mcu_ipaddr"]
elif "mcu_port" in args:
    MCU_PORT = args["mcu_port"]
elif "pc_port" in args:
    HOST_PC_PORT = args["pc_port"]

ETH_ECHO_TEST["name"] = OUTPUT_JSON_NAME
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
elif SCHEDDL_SETTING == "fifo":
    scheddl.set_fifo(SCHEDDL_PRIORITY, scheddl.RESET_ON_FORK)
elif SCHEDDL_SETTING == "rr":
    scheddl.set_rr(SCHEDDL_PRIORITY, scheddl.RESET_ON_FORK)

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

            # TODO: should handle timeouts here, if recv is stuck (using python select), and allow several attempts if timeout occurs
            # TODO: should mark timed out data somehow with e.g. NaN
            # TODO: check each received packet for a "stamp" done by the MCU, to confirm the MCU opened it
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

            print("    ---- done -- time: {:.2e}".format(numpy.sum(times_array)))
            
            # TODO: see if thread_yield can be used here instead
            time.sleep(TRIAL_INTERVAL)

print("Ran {} tests, written to file {}".format(len(ETH_ECHO_TEST["tests"]), OUTPUT_JSON_NAME))

with open(OUTPUT_JSON_NAME, "w") as test_results_json:
    json.dump(ETH_ECHO_TEST, test_results_json)
