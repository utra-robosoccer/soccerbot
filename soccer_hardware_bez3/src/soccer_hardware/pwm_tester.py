import math
import struct
import sys

import serial


def crc32mpeg2(buf, crc=0xFFFFFFFF):
    CRC32_POLY = 0x104C11DB7
    for val in buf:
        crc ^= val << 24
        for _ in range(8):
            crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ CRC32_POLY
    return crc


def align_n(S, n):
    return S + b"\x00" * ((n - len(S) % n) % n)  # pad to align to 32-bit words


def be_crc(S):
    return crc32mpeg2(align_n(S, 4))


def le_crc(S):
    S = align_n(S, 4)
    n = int(len(S) / 4)
    return crc32mpeg2(struct.pack(">" + "l" * n, *struct.unpack("<" + "l" * n, S)))


def bytepack(A):
    if isinstance(A, list):
        return struct.pack("B" * len(A), *A)
    else:
        return struct.pack("B", A)


def pack6(A):
    b = []
    for a in A:
        b += [a & 0x3F, (a >> 6) & 0x3F]

    return bytepack(b)


with serial.Serial(sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem14203", 230400) as ser:
    while True:
        p = [0x800] * 6
        try:
            s, p_ = [int(a, 0) for a in input().strip().split(" ")]
            p[s] = p_
        except (ValueError, IndexError):
            pass  # p = [0x800] * 6

        bs = pack6(p)
        crc = (1 << 6) | (le_crc(bs) & 0x7F)
        ser.write(bs + bytepack(crc))
        print(crc)
