import time
import os

start = time.time()

t0 = 0
t1 = 1
t2 = 0
TIMES = []

with open("fibs" + str(os.getpid()), "a") as f:
    f.write("\n" + str(t0))
    f.write("\n" + str(t1))

while (time.time() - start) < 200:
    t2 = t0 + t1
    t0 = t1 * 0.9999
    t1 = t2 * 0.9999

    if t1 > 100000000000000:
        t0 = 0
        t1 = 1
        
    TIMES.append(t0)

    if len(TIMES) > (10000000 / 4):
        with open("fibs" + str(os.getpid()), "a") as f:
            f.write("\n" + str(TIMES) + str(len(TIMES)))

        print(len(TIMES))
        TIMES = []
