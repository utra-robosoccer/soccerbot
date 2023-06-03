import struct
import time
from enum import Enum
from threading import Lock

from common_motor_util import bytepack, crc32mpeg2, le_crc, unpack6
from utility import log_string

DEBUG = False


class CMD_HEADS(Enum):
    DATA, MASTER_START, SERVO_START, END = [a << 6 for a in range(4)]


class CMDS(Enum):
    POSITION, SPEED, PID_COEFF, PID_STATE, SPEED_FILTER_WIDTH, MAX_DRIVE, INVALID = range(7)


class RWS(Enum):
    READ, WRITE = range(2)


class UART_STATES(Enum):
    START, MASTER_HEADED, MASTER_ENDED, SERVO_HEADED, SERVO_ENDED, ENDED = range(6)


# Obsolete: these are obsolete timing estimation params:
# BAUD = int(1e6)
# EXPECT_BYTE_RX_TIME = 25e-6  # based on servo microcontroller nominal wait width per byte plus margin
# EXPECT_BYTE_TX_TIME = 25e-6

CMD_WIDTHS = [2, 2, 7, 6, 2]
SILENT_READ_TIMEOUT = 20E-3 # time to give the kernel to stall waiting to push out bytes or pull in bytes before giving up, assuming we've missed the frame read
# worst-case catch-all for any corruption that fails the state machine, so we can clean the read buffer and start from fresh.
READ_POLL_RATE = 1E-4 # Make this << OS tick rate, so we can pull from the read buffer with minimal delay when the data is available (without tight-looping and hogging CPU)

ACTIVE_READ_TIMEOUT = 3E-3 # allow this time after we've received the head frame but still haven't seen enough returns.
# ^^ This number is important and based on two factors:
# 1. Fully flushing the read buffer / not missing frames. This guards against if the read buffer is slow to pop the contents.
#    This part of the number is based on roundtrip analyses on Ubuntu 20 on an AMD processor as well as Ubuntu 18 on a Jetson Nano showing quantization to 1ms ticks: in the worst-case scenario, we read the head frame right before one tick and the rest of the packet comes in the next tick.
#    Note! Assumes whole packets fit in one tick.
# 2. Avoiding clobbering read frames from the servos by subsequent send commands.
#    This needs to be at least as long as the packet itself for another reason: we can't have this read routine exit prematurely and give control before the servos have a chance to respond.
#    This is expected to be the lax constraint, unless the OS ticks start getting faster. Currently, with ~10 servos at 40 bits per response (10 bits per frame), margining to twice that amount of time for padding time (very pessmistic) at 1Mb/s, expecting ~1ms for transmission.

MAX_JX_SERVO_IDX = 12
N_SERVOS = 12 # TODO: Centralize this (where's the authority on this number?)

def cmd_print(A):
    log_string("".join([format((a >> 6) & 0x3, "02b") + " " + format(a & 0x3F, "06b") + " | " for a in A]))

MUTEXES = {} # locks of port string vs. mutex.
# # hope that strings aren't pointing to the same port under different paths, tempted to use os abspath to resolve to canonical path
INF = float("inf")

jx_iter = 0

def uart_transact(ser, B, cmd, rw, preflush=True):
    global jx_iter

    if ser.port not in MUTEXES:
        MUTEXES[ser.port] = Lock() # index on Serial port, tempted to canonicalize via os.path.abspath.
        # i don't love globals but i also don't love forcing an external lock wrap around this routine
        # we know two threads accessing the same ser object is dangerous, and even the same port (although less dangerous, given the ser object has lots of state that we are manipulating here). however it's the stronger condition to exclude by port, as long as it's usually canonical. it's not safe to compare by `id()` since these are reused after object destruction so we have to be nuanced about what uniqueness will give us safety.
    
    with MUTEXES[ser.port]:
        # print('IN')
        # do not attempt a timeout, the timeouts of this routine must be tuned to the OS UART tick rates to avoid clobbering and stale data
        time0 = time.time()

        def send(b):
            if DEBUG:
                cmd_print(b)
            return ser.write(b)

        if DEBUG:
            log_string("O\t")

        data = [c for b in B for c in [b & 0x3F, (b >> 6) & 0x3F]]  # note: little endian
        Astr = bytepack(CMD_HEADS.MASTER_START.value | (rw.value << 5) | cmd.value) + bytepack(data)

        crc = le_crc(Astr)

        # try:
        # 	ser.write_timeout = 0 # (len(Astr) + 1) * EXPECT_BYTE_TX_TIME
        nbytes_tx = send(Astr + bytepack(CMD_HEADS.END.value | crc & 0x3F))
        # except serial.SerialTimeoutException as e:
        # 	print(nbytes_tx, len(Astr))
        # 	raise e

        if DEBUG:
            log_string(str(Astr))
            log_string("\t" + format(crc, "08X"))
            log_string("\n")

            if rw == RWS.READ:
                log_string("I\t")

        if preflush:
            ser.timeout = 1E-5 # if this is narrower than the kernel quantum does this make this fail every time?
            ser.read(128) # one last try to flush the read buffer in case of any stale data

        ### UART munching FSM ###
        uart_state = UART_STATES.START
        expect_len = len(Astr) + 1
        r0 = b""
        servo_idx = None
        servo_frames = {}
        master_frame = []

        # obsolete empty frames check replaced by timeout-based fail
        # MAX_EMPTY_FRAMES = 8
        # empty_frames = 0

        time_rx_packet0 = None
        _saw_master_start = False
        poll_iters = 0
        while uart_state != UART_STATES.ENDED:
            # # examine timeout condition
            # if time.time() > time0 + timeout:
            #     return (False, None)
            #
            # if n_rx_frames == 0 and len(r0) > 0:
            #     time_rx_packet0 = time.time()
            # n_rx_frames += len(r0)

            for ri, r in enumerate(r0):
                r_head = r & 0xC0
                r_data = r & 0x3F
                if not _saw_master_start and r_head == CMD_HEADS.MASTER_START.value:
                    # sometimes corrupted frames will transmit a master start while we're collecting data, reject these attempts to move backwards in the state machine
                    uart_state = UART_STATES.MASTER_HEADED
                    _saw_master_start = True
                    time_rx_packet0 = time.time() # start ticking on read frames
                elif r_head == CMD_HEADS.SERVO_START.value:
                    servo_idx = r_data
                    if servo_idx not in servo_frames:
                        servo_frames[servo_idx] = [False, b""]
                        uart_state = UART_STATES.SERVO_HEADED
                elif r_head == CMD_HEADS.END.value:
                    if uart_state == UART_STATES.MASTER_HEADED:
                        if rw == RWS.WRITE:
                            uart_state = UART_STATES.ENDED
                            break
                        elif rw == RWS.READ:
                            uart_state = UART_STATES.MASTER_ENDED

                    elif uart_state == UART_STATES.SERVO_HEADED:
                        uart_state = UART_STATES.SERVO_ENDED

                # record data
                if _saw_master_start:
                    rbyte = struct.pack(
                        "B", r
                    )  # interestingly (read: annoyingly) looping over bytestring makes ints. see stackoverflow.com/questions/14267452
                    if uart_state in (UART_STATES.MASTER_HEADED, UART_STATES.MASTER_ENDED):
                        master_frame += rbyte
                    else:
                        servo_frames[servo_idx][1] += rbyte

                if uart_state == UART_STATES.SERVO_ENDED:
                    # CRC check.
                    # TODO: reconfigure logic so this doesn't have to be split from the other conditions on uart_state above
                    s = servo_frames[servo_idx][1]
                    if len(s) > 1:
                        servo_frames[servo_idx][0] = ((le_crc(s[:-1]) ^ s[-1]) & 0x3F) == 0
                        if not servo_frames[servo_idx][0]:
                            print(servo_idx, s)

                if uart_state == UART_STATES.SERVO_ENDED and (
                    (servo_idx == MAX_JX_SERVO_IDX and servo_frames[servo_idx][0])
                    or len(servo_frames) == N_SERVOS
                ) and _saw_master_start: # allow both seeing the CRC-validated last servo frame, or all of the servo frames (even if the last is corrupted) as the exit condition
                    # exit no matter what, even if there are more bytes to be read, which is pretty weird unless we're a full frame behind
                    uart_state = UART_STATES.ENDED
                    break

            # if uart_state != UART_STATES.START and len(r0) == 0:
            #     empty_frames += 1

            (time0_, read_timeout_) = (time0, SILENT_READ_TIMEOUT) if (uart_state == UART_STATES.START or time_rx_packet0 is None) else (time_rx_packet0, ACTIVE_READ_TIMEOUT)
            if time.time() - time0_ > read_timeout_:
                print(rw, (uart_state == UART_STATES.START or time_rx_packet0 is None))
                break

            if uart_state in (UART_STATES.START, UART_STATES.MASTER_HEADED):
                expect_len = len(Astr) + 1 - len(master_frame)
            elif uart_state in (UART_STATES.SERVO_HEADED, UART_STATES.MASTER_ENDED, UART_STATES.SERVO_ENDED):
                w = CMD_WIDTHS[cmd.value] + 2
                l = len(servo_frames[servo_idx]) if servo_idx in servo_frames else 0
                expect_len = max(0, (w - (l % w)))  # both full and empty frames will trigger a full frame read

            if uart_state != UART_STATES.ENDED:
                try:
                    # ser.timeout = EXPECT_BYTE_RX_TIME * expect_len

                    # if uart_state == UART_STATES.START: # hm. trust the state machine or trust the num read bytes? we don't have a counter for the latter yet
                    #     ser.timeout = 5E-3
                    # else:
                    #     ser.timeout = 1E-3
                    #
                    # r0 = ser.read(1)
                    # this led to very high fail rates (~1/10 transactions). possibly due to OS quantization to the 1ms tick for every read/select with a non-zero timeout?
                    # either way, this doesn't seem viable

                    # time.sleep(0) # READ_POLL_RATE # give up remaining quantum, reduce CPU contention. any non-zero value was causing timeout fails, seems it was too generous before.
                    # ^ even time.sleep(0) might be too dangerous, for how short-lived this routine is, may be better to just run at 100% and let the thread manager balance load
                    ser.timeout = 1E-3
                    # worried about ser.timeout=0, seems like when i didn't have global mutexes this was hanging when i was reading an arbitarily large number of bytes
                    # (could have been here or in the buf clear routine)
                    # possible to miss bytes and something about this causes timeout if expect_len is too big that it waits forever on select
                    # that comes down to how select deals with timeout=0, or if there really was somehow a race condition on the ser.timeout?
                    # i'm honestly mystified about the and that worries me a bit
                    # also, the OS load affects the number of packet drops which also really worries me, at very high load are we going to have bad fallout?
                    # for example, the spellcheck as i'm writing this comment is causing packet fails while i run this test.
                    # that would seem to make the chances of a hang much higher.
                    # i wanted to squash the problem down to a single byte wait, although if something is wrong with `select` that wouldn't make
                    # the problem any better since it could wait on that one byte forever. it's just the semantics of this 0 timeout that worries me a bit.

                    r0 = ser.read(expect_len)
                    poll_iters += 1

                    # i'm seeing hundreds of poll iterations in this tight polling loop if timeout=0. when timeout=1ms, the count drops but is still 24 for the read.
                    # so it seems it's not quantized to 1ms per select with timeout, but it's still clearly expensive since the device doesn't report readiness accurately enough
                    # to munch one byte at a time. or it's just too expensive to keep hitting `select` in general for the 1-byte muncher?
                    # this is much more stable than the one-byte muncher, more like a 0.1% fail rate on timeouts, usually the missed master packet

                    # if state == UART_STATES.START:
                    #     ser.timeout = 0
                    #     r0 = ser.read(1) # ser.read(expect_len)
                    # else:
                    #     ser.timeout = 1E-3

                    # use a packet size much larger than the full packet size, so we can flush the buffer when it's possible. ser.timeout = 0 should make this return instantly.
                except ValueError as e:
                    pass

        # checked_servo_frames = {si: tuple(s) for si, s in servo_frames.items() if len(s) == CMD_WIDTHS[cmd.value] + 2} # (((le_crc(s[:-1]) ^ s[-1]) & 0x3F) == 0, s)

        # print('OUT')
        if jx_iter % 128 == 0:
            print(jx_iter, poll_iters)
        jx_iter += 1

        return (uart_state == UART_STATES.ENDED, servo_frames) # empty_frames < MAX_EMPTY_FRAMES

if __name__ == '__main__':
    import serial
    import numpy as np
    
# def uart_transact(ser, B, cmd, rw, preflush=True):    

    MIN_POT=0x27E
    MAX_POT=0xE64
    with serial.Serial('/dev/ttyUSB0', 1000000, timeout=0) as ser:
        uart_transact(ser, [4000, 1, 90] * 13, CMDS.PID_COEFF,
                                    RWS.WRITE)  # push initial PID gains
        uart_transact(ser, [0x27E] * (MAX_JX_SERVO_IDX + 1), CMDS.POSITION, RWS.WRITE)
        input()
        t0 = time.time()
        T0 = 6
        while True:
            uart_transact(ser, [((-np.cos((time.time() - t0) / T0 * 2 * np.pi) / 2 + 0.5) * (MAX_POT-MIN_POT) + MIN_POT).astype(np.uint16)] * (MAX_JX_SERVO_IDX+1), CMDS.POSITION, RWS.WRITE)
            time.sleep(0.05)
        