import socket
import time

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_address = ('192.168.0.55', 7)
sock.bind(('', 7))

message = 'this is a message of length 80 chars. asdfghjklasdfghjklasdfghjklasdfghjkl ++++'.encode()

num_samples = 1000

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
        #print('received "{}"'.format(data))
        dt = t1 - t0
        total_time += dt

    print('Took {} seconds for {} samples'.format(total_time, num_samples))
    print('Average echo time: {} seconds'.format(total_time / num_samples))

finally:
    print('Closing socket')
    sock.close()
