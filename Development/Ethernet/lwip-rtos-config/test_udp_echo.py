import socket
import time
import numpy

# This script sends a message to the board, at IP address and port given by
# server_address, using User Datagram Protocol (UDP). The board should be
# programmed to echo back UDP packets sent to it. The time taken for num_samples
# echoes is measured.

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = ('192.168.0.55', 7)
sock.bind(('', 7))

message = 'this is a message of length 80 chars. asdfghjklasdfghjklasdfghjklasdfghjkl ++++'.encode()

num_samples = 500

times = []

try:

    # Send data
    print('Sending "{}"'.format(message))
    print('Measuring time taken for {} echoes'.format(num_samples))

    total_time = 0
    for i in range(num_samples):

        t0 = time.perf_counter()
        sent = sock.sendto(message, server_address)

        # Receive response
        data, server = sock.recvfrom(4096)
        t1 = time.perf_counter()
        dt = t1 - t0
        total_time += dt
        #print('received "{}"'.format(data))
        times.append(dt)

    f = open('times', 'a')
    try:
        f.write('\n')
        for i in range(num_samples):
            f.write('{},'.format(times[i]))
    finally:
        f.close()

    times_array = numpy.array(times)

    print('Took {} seconds for {} samples'.format(total_time, num_samples))
    print('Average echo time: {} seconds'.format(numpy.average(times_array)))
    print('Standard deviation: {} seconds'.format(numpy.std(times_array)))
    print('Maximum: {} seconds, Minimum: {} seconds'.format(numpy.amax(times_array), numpy.amin(times_array)))

finally:
    print('Closing socket')
    sock.close()
