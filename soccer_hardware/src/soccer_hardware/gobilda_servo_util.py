from common_motor_util import crc32mpeg2, le_crc, bytepack

def uart_transact(ser, B):
	bs = [bytepack(b) for b in B]
	crc = 1 << 7 | (le_crc(bs) & 0x7F)
	ser.write(bs + bytepack(crc))
