from enum import Enum
# import pygame
# import pygame.locals
import serial
import sys
import struct
import time

DEBUG = False

P = (1024, 2048)
POS = (0xFFF, 0)
CMD_DATA, CMD_MASTER_START, CMD_SERVO_START, CMD_END = [a << 6 for a in range(4)]
CMD_POSITION, CMD_SPEED, CMD_PID_COEFF, CMD_SPEED_FILTER_WIDTH, CMD_PID_STATE, CMD_MAX_DRIVE, CMD_SERVO_IDX, CMD_INVALID = range(8)
RW_READ, RW_WRITE = range(2)
CMD_WIDTHS = [ 2, 2, 7, 6, 2, 1, 2, 1 ]
BAUD = int(1E6)
NUM_SERVOS = 13
EXPECT_BYTE_RX_TIME = 15E-6 # based on servo microcontroller nominal wait width per byte plus margin
EXPECT_BYTE_TX_TIME = 15E-6

def crc32mpeg2(buf, crc=0xffffffff):
	CRC32_POLY = 0x104c11db7
	for val in buf:
		crc ^= val << 24
		for _ in range(8):
			crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ CRC32_POLY
	return crc

def cmd_print(A):
	if DEBUG:
		sys.stdout.write(''.join([format((a >> 6) & 0x3, '02b') + ' ' + format(a & 0x3F, '06b') + ' | ' for a in A]))

def le_crc(S):
	S += (b'\x00' * ((4 - len(S) % 4) % 4)) # pad to align to 32-bit words
	n = int(len(S) / 4)
	return crc32mpeg2(struct.pack('>' + 'l'*n, *struct.unpack('<' + 'l'*n, S)))

def bytepack(A):
	if isinstance(A, list):
		return struct.pack('B' * len(A), *A)
	else:
		return struct.pack('B', A)


def uart_transact(ser, B, cmd, rw):
	def send(b):
		cmd_print(b)
		return ser.write(b)

	if DEBUG:
		sys.stdout.write('O\t')

	data = [c for b in B for c in [b & 0x3F, (b >> 6) & 0x3F]] # note: little endian
	Astr = bytepack(CMD_MASTER_START | (rw << 5) | cmd) + bytepack(data)

	crc = le_crc(Astr)
	
	if True: # try:
		# ser.write_timeout = 0 # (len(Astr) + 1) * EXPECT_BYTE_TX_TIME
		nbytes_tx = send(Astr + bytepack(CMD_END | crc & 0x3F))
# 	except serial.SerialTimeoutException as e:
		# print(nbytes_tx, len(Astr))
# 		raise e
	if DEBUG:
		sys.stdout.write(str(Astr))
		sys.stdout.write('\t' + format(crc, '08X'))

	# while True:
	# 	# TODO make this a full state machine
	# 	b = ser.read()
	# 	if len(b) > 0:
	# 		c = struct.unpack('B', b)[0]
	# 		if (c & 0xC0) == CMD_END:
	# 			break
	
	if DEBUG:
		sys.stdout.write('\n')

	if rw == RW_READ:
		if DEBUG:
			sys.stdout.write('I\t')
		
	class UART_STATES(Enum):
		START=0
		MASTER_HEADED=1
		MASTER_ENDED=2
		SERVO_HEADED=3
		SERVO_ENDED=4
		ENDED=5
	
	### UART munching FSM ###
	LAST_SERVO_IDX = 12 # NUM_SERVOS - 1
	uart_state = UART_STATES.START
	expect_len = len(Astr) + 1
	r0 = b''
	servo_idx = None
	servo_frames = {}
	master_frame = []
	empty_frames = 0
	MAX_EMPTY_FRAMES = 3
	_saw_master_start = False
	while uart_state != UART_STATES.ENDED and empty_frames < MAX_EMPTY_FRAMES:
		for ri, r in enumerate(r0):
			r_head = r & 0xC0
			r_data = r & 0x3F
			if not _saw_master_start and r_head == CMD_MASTER_START:
				# sometimes corrupted frames will transmit a master start while we're collecting data, reject these attempts to move backwards in the state machine
				uart_state = UART_STATES.MASTER_HEADED
				_saw_master_start = True
			elif r_head == CMD_SERVO_START:
				servo_idx = r_data
				if servo_idx not in servo_frames:
					servo_frames[servo_idx] = b''
					uart_state = UART_STATES.SERVO_HEADED
			elif r_head == CMD_END:
				if uart_state == UART_STATES.MASTER_HEADED:
					uart_state = UART_STATES.MASTER_ENDED
				elif uart_state == UART_STATES.SERVO_HEADED:
					uart_state = UART_STATES.SERVO_ENDED

			# record data
			if _saw_master_start:
				rbyte = struct.pack('B', r) # interestingly (read: annoyingly) looping over bytestring makes ints. see stackoverflow.com/questions/14267452
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
			w = CMD_WIDTHS[cmd] + 2 
			l = len(servo_frames[servo_idx]) if servo_idx in servo_frames else 0
			expect_len = max(0, (w - (l % w))) # both full and empty frames will trigger a full frame read

		if uart_state != UART_STATES.ENDED:
			ser.timeout = EXPECT_BYTE_RX_TIME * expect_len
			# print(ser.timeout)
			r0 = ser.read(expect_len)

	checked_servo_frames = { si: (((le_crc(s[:-1]) ^ s[-1]) & 0x3F) != 0, s) for si, s in servo_frames.items() if len(s) == CMD_WIDTHS[cmd] + 2 }
	if rw == RW_READ:
		rw = RW_READ
	return (empty_frames >= MAX_EMPTY_FRAMES, checked_servo_frames)

#		try:
#			NUM_SERVOS = 3
#			a = time.time()
#			
#			ser.read(len(Astr) + 1)
#			rets = {}
#			for _ in range(NUM_SERVOS):
#				din = ser.read((CMD_WIDTHS[cmd] + 1)) # [-(CMD_WIDTHS[cmd] + 1):]
#				rets[din[0] & 0x3F]
#
#				crc = le_crc(ret[:-1])
#				cmd_print(ret)
#				cmd_print(bytepack(crc & 0x3F))
#					
#			if DEBUG:
#				sys.stdout.write(str(Astr))
#				sys.stdout.write('\t|\t')
#				sys.stdout.write(str(ret))
#				# sys.stdout.write('\n')
#				sys.stdout.write('%.3e' % (time.time() - a))
#				sys.stdout.write('\n')
#
#			if DEBUG:
#				sys.stdout.write('\n')
#			
#			return (ret, ((crc ^ ret[-1]) & 0x3F) != 0)
#		except IndexError:
#			return (0, 1)


def test(ser):
	time0 = time.time()
	with open('out.csv', 'w') as fout:
		packets = 0
		packet_fails = 0
		
		# pygame.init()
		SW, SH = 640, 480
		# screen = pygame.display.set_mode((SW, SH))
		# uart_transact(ser, [7], CMD_SERVO_IDX, RW_WRITE)
		uart_transact(ser, [1024, 2, 1] * NUM_SERVOS, CMD_PID_COEFF, RW_WRITE) # int(y / SH * (P[1] - P[0]) + P[0])
		# clock = pygame.time.Clock()
		it = 0
		while True:
			it += 1
			if False:
				for event in pygame.event.get():
					if event.type == pygame.locals.QUIT:
						pygame.quit()
						return
			x, y = (0, 0) # pygame.mouse.get_pos()
			if DEBUG:
				print(x, y)
			
			t0 = time.time()
			# print(time.time() - t0)
			# uart_transact(ser, [int(math.sin((packets) * math.pi / 32 + math.pi / 2 * i / NUM_SERVOS) ** 2 * (POS[1] - POS[0]) + POS[0]) for i in range(NUM_SERVOS)], CMD_POSITION, RW_WRITE)

			# if i == 2 else 0
			# uart_transact(ser, [0x800] * 2 + [0xFFF] + [0x800] * 3, CMD_POSITION, RW_WRITE)
			# print(time.time() - t0)

			rxs = [CMD_SPEED] # , CMD_PID_COEFF, CMD_PID_STATE]
			for rx in rxs:
				# packet_fails += uart_transact(ser, [], rx, RW_READ)[1]
				res_raw = uart_transact(ser, [], rx, RW_READ)
				print(res_raw)
				break

				try:
					res_err, res = res_raw[1][3]
					if not res_err:
						res = (res[1] & 0x3F) | ((res[2] & 0x3F) << 6)
						if res & 0x800:
							res -= 0xFFF
						print(res)
						fout.write('%.5f,%d\n' % (time.time() - time0, res))
				except KeyError:
					pass

				packets += 1

			# print(time.time() - t0)

def set_servo_idx(ser, idx):
	return uart_transact(ser, [idx], CMD_SERVO_IDX, RW_WRITE)

if __name__ == '__main__':
	with serial.Serial(sys.argv[1] if len(sys.argv) > 1 else 'COM3', BAUD, timeout=None, write_timeout=None) as ser:
		# set_servo_idx(ser, int(sys.argv[-1]))
		# test(ser)
		uart_transact(ser, [1600, 1, 15] * 13, CMD_PID_COEFF, RW_WRITE)  # push initial PID gains
		uart_transact(ser, [1300] * 13, CMD_MAX_DRIVE, RW_WRITE)  # push initial maximum drive (out of 4096)
		uart_transact(ser, [0x800] * 13, CMD_POSITION, RW_WRITE)
		# for i in range(13):
		# 	uart_transact(self._ser, [0x800] * i, CMDS.POSITION, RWS.WRITE)
		# 	raw_input()
