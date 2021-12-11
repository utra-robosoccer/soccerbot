import struct
import time
from enum import Enum
from threading import Thread

from receiver import Receiver
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


CMD_WIDTHS = [2, 2, 7, 6, 2]
BAUD = int(1e6)
NUM_SERVOS = 13
LAST_SERVO_IDX = 12  # NUM_SERVOS - 1
EXPECT_BYTE_RX_TIME = 15e-6  # based on servo microcontroller nominal wait width per byte plus margin
EXPECT_BYTE_TX_TIME = 15e-6


def cmd_print(A):
    log_string("".join([format((a >> 6) & 0x3, "02b") + " " + format(a & 0x3F, "06b") + " | " for a in A]))


def crc32mpeg2(buf, crc=0xFFFFFFFF):
    CRC32_POLY = 0x104C11DB7
    for val in buf:
        crc ^= val << 24
        for _ in range(8):
            crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ CRC32_POLY
    return crc


def le_crc(S):
    S += b"\x00" * ((4 - len(S) % 4) % 4)  # pad to align to 32-bit words
    n = int(len(S) / 4)
    return crc32mpeg2(struct.pack(">" + "l" * n, *struct.unpack("<" + "l" * n, S)))


def bytepack(A):
    if isinstance(A, list):
        return struct.pack("B" * len(A), *A)
    else:
        return struct.pack("B", A)


def un6pack(bs, _signed=True):
    # little-endian
    c = 0
    for i, b in enumerate(bs):
        c |= (b & 0x3F) << (i * 6)

    if _signed:
        sign_bit = 1 << (len(bs) * 6 - 1)
        if c & sign_bit:
            c -= sign_bit << 1

    return c


INF = float("inf")


def uart_transact(ser, B, cmd, rw, timeout=INF):
    # note: timeout will be imprecise and quantized to tuned timeouts of this read/write routine (optimized to improve latency)
    # this is done to avoid re-entrancy or thread-unsafe operations: do not put this in a thread and time it out forcibly.
    # it will nevertheless try its best to time out in critical loops.
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

    ### UART munching FSM ###
    uart_state = UART_STATES.START
    expect_len = len(Astr) + 1
    r0 = b""
    servo_idx = None
    servo_frames = {}
    master_frame = []
    empty_frames = 0
    MAX_EMPTY_FRAMES = 8
    _saw_master_start = False
    while uart_state != UART_STATES.ENDED and empty_frames < MAX_EMPTY_FRAMES:
        # examine timeout condition
        # if time.time() > time0 - timeout:
        # 	return (False, None)

        for ri, r in enumerate(r0):
            r_head = r & 0xC0
            r_data = r & 0x3F
            if not _saw_master_start and r_head == CMD_HEADS.MASTER_START.value:
                # sometimes corrupted frames will transmit a master start while we're collecting data, reject these attempts to move backwards in the state machine
                uart_state = UART_STATES.MASTER_HEADED
                _saw_master_start = True
            elif r_head == CMD_HEADS.SERVO_START.value:
                servo_idx = r_data
                if servo_idx not in servo_frames:
                    servo_frames[servo_idx] = b""
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
                    servo_frames[servo_idx] += rbyte
            if uart_state == UART_STATES.SERVO_ENDED and servo_idx == LAST_SERVO_IDX and _saw_master_start:
                # exit no matter what, even if there are more bytes to be read, which is pretty weird unless we're a full frame behind
                uart_state = UART_STATES.ENDED
                break

        if uart_state != UART_STATES.START and len(r0) == 0:
            empty_frames += 1

        if uart_state in (UART_STATES.START, UART_STATES.MASTER_HEADED):
            expect_len = len(Astr) + 1 - len(master_frame)
        elif uart_state in (UART_STATES.SERVO_HEADED, UART_STATES.MASTER_ENDED, UART_STATES.SERVO_ENDED):
            w = CMD_WIDTHS[cmd.value] + 2
            l = len(servo_frames[servo_idx]) if servo_idx in servo_frames else 0
            expect_len = max(0, (w - (l % w)))  # both full and empty frames will trigger a full frame read

        if uart_state != UART_STATES.ENDED:
            ser.timeout = EXPECT_BYTE_RX_TIME * expect_len
            # print(ser.timeout)
            r0 = ser.read(expect_len)

    checked_servo_frames = {si: (((le_crc(s[:-1]) ^ s[-1]) & 0x3F) == 0, s) for si, s in servo_frames.items() if len(s) == CMD_WIDTHS[cmd.value] + 2}

    return (empty_frames < MAX_EMPTY_FRAMES, checked_servo_frames)
