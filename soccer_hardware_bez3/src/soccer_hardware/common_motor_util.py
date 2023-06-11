import struct

def crc32mpeg2(buf, crc=0xFFFFFFFF):
    CRC32_POLY = 0x104C11DB7
    for val in buf:
        crc ^= val # << 24
        for _ in range(32):
            crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ CRC32_POLY
    return crc


def le_crc(S):
    S += b"\x00" * ((4 - len(S) % 4) % 4)  # pad to align to 32-bit words
    n = int(len(S) / 4)
    return crc32mpeg2(struct.unpack("<" + "l" * n, S))


def bytepack(A):
    if isinstance(A, list):
        return struct.pack("B" * len(A), *A)
    else:
        return struct.pack("B", A)


def unpack6(bs, _signed=True):
    # little-endian
    c = 0
    for i, b in enumerate(bs):
        c |= (b & 0x3F) << (i * 6)

    if _signed:
        sign_bit = 1 << (len(bs) * 6 - 1)
        if c & sign_bit:
            c -= sign_bit << 1

    return c

def constrain(x, a, b):
    return min(max(x, min(a, b)), max(a, b))

def pack6(A):
	b = []
	for a in A:
		b += [a & 0x3F, (a >> 6) & 0x3F]

	return bytepack(b)

