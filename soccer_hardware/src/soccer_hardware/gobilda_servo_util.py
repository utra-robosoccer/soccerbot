from common_motor_util import bytepack, crc32mpeg2, le_crc, pack6


def uart_transact(ser, B):
    bs = pack6(B)
    crc = (1 << 6) | (le_crc(bs) & 0x3F)
    print(bs, crc)
    ser.write(bs + bytepack(crc))
