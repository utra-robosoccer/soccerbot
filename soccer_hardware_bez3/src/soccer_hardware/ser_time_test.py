import serial
import time
import glob
import numpy as np
import matplotlib.pyplot as plt

plt.ion()
plt.close('all')

def pplot(x, f=lambda x:x, *args, **kwargs):
    plt.plot(np.sort(x), (np.arange(len(x)) + 0.5) / (len(x)), *args, **kwargs)

if __name__ == '__main__':
    with serial.Serial(glob.glob('/dev/ttyUSB*')[0], 1000000, timeout=None) as ser:
        N = 4
        i = 0
        # ser.timeout = None # hopefully get 1ms granularity
        ts = []
        for _ in range(256):
            t0 = time.time()
            b0 = bytes([i & 0xFF] * N)
            ser.write(b0)
            dt1 = time.time() - t0
            # print('%.3e' % ())
            b = b''
            for r_iters in range(32):
                b += ser.read(N)
                if len(b) == N:
                    # print(b == b0)
                    if b != b0:
                        print(i, b[0])
                    break
            dt2 = time.time() - t0
            ts.append((dt1, dt2, r_iters))
            # print('%.3e' % ())

            i += 1
        ts = np.array(ts)
        print('%.1e +/- %.1e' % (np.mean(ts[:,0]), np.std(ts[:,0])))
        print('%.1e +/- %.1e' % (np.mean(ts[:,1]), np.std(ts[:,1])))
        print(np.sum(ts[:,2]))
        f, (a1, a2) = plt.subplots(1, 2)
        plt.sca(a1)
        pplot(ts[:,0])
        plt.sca(a2)
        pplot(ts[:,1])
        plt.show()