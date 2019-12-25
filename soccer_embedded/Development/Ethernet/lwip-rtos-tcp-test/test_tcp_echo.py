import socket
import time
import numpy

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_address = ('192.168.0.75', 7)
sock.connect(server_address)

message = 'this is a message of length 80 chars. asdfghjklasdfghjklasdfghjklasdfghjkl ++++'.encode()

num_samples = 100000

times = []

try:

    # Send data
    print('Sending "{}"'.format(message))
    print('Measuring time taken for {} echoes'.format(num_samples))

    total_time = 0
    for i in range(num_samples):

        t0 = time.perf_counter()
        sent = sock.sendall(message)

        # Receive response
        # Look for the response
        amount_received = 0
        amount_expected = len(message)

        while amount_received < amount_expected:
            data = sock.recv(16)
            amount_received += len(data)

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
