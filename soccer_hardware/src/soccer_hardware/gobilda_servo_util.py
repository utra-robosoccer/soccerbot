from common_motor_util import bytepack, crc32mpeg2, le_crc


def uart_transact(ser, B):
    bs = b"".join([bytepack(b) for b in B])
    crc = 1 << 7 | (le_crc(bs) & 0x7F)
    ser.write(bs + bytepack(crc))
