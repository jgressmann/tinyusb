#! /usr/bin/env python3


from typing import Sequence, Union
import serial
import sys
import time



import linchpin

def lsb_first_bits(b: int):
	for i in range(8):
		bit = 1 << i
		yield (b & bit) == bit

def pid(id: int) -> int:
	# P0 = ID0 ^ ID1 ^ ID2 ^ ID4
	p0 = int((id & 1) == 1) ^ int((id & 2) == 2) ^ int((id & 4) == 4) ^ int((id & 16) == 16)
	# P1 = ~(ID1 ^ ID3 ^ ID4 ^ ID5)
	p1 = 1 ^ int((id & 2) == 2) ^ int((id & 8) == 8) ^ int((id & 16) == 16) ^ int((id & 32) == 32)

	pid = (p1 << 7) | (p0 << 6) | (id & 63)

	return pid

def checksum(ints: Union[bytes, Sequence[int]]) -> int:
	sum = 0
	for i in ints:
		sum += int(i)
		if sum >= 256:
			sum -= 255

	return sum ^ 255

def data_byte(enc: linchpin.RlewEncoder, oversampling: int, b: int):
	for bit in lsb_first_bits(b):
		for _ in range(oversampling):
			enc.add(int(bit))

# def frame_header(enc: linchpin.RlewEncoder, oversampling: int, inter_byte_space: float):

MASTER_REQUEST_ID = 0x3c


def test_case_2_1(enc: linchpin.RlewEncoder, oversampling: int, inter_byte_space: float, nad: int, offset: int, *ids):

	"""Diagnostic frame 'Master Request', IUT as Slave

	This test verifies if the IUT as Slave is able to receive Master request frames.
	In this test case the diagnostic frame type 'Master Request' is checked, i.e. a frame to send commands and data from the Master to the Slave.
	In this test case the TST_FRAME_7 with the frame ID 0x30 shall be used.(according NCF)."""

	# break
	# sync

	while len(ids) < 4:
		ids.append(0xff)

# 	NAD
# PCI

# SID

# D1
# D2
# D3
# D4
# D5
# start index PID (index) PID (index+1) PID (index+2) PID (index+3)


	for byte in [MASTER_REQUEST_ID, nad, 0x06, 0xB7, offset, ids[0], ids[1], ids[2], ids[3]]:
		data_byte(enc, oversampling, byte)

		for _ in range(int(oversampling * inter_byte_space)):
			enc.add(1)

	# checksum




if __name__ == "__main__":
	import argparse

	parser = argparse.ArgumentParser(description='linChPiN test program')
	parser.add_argument('-f', '--frequency', metavar='F', type=int, help='signal frequency [Hz]')
	parser.add_argument('-c', '--cmd-dev', metavar='DEV', required=True, help='command serial port')
	parser.add_argument('-l', '--lin-dev', metavar='DEV', required=True, help='lin serial port')
	parser.add_argument('-b', '--baud', metavar='BAUD', type=int, default=115200, help='Baud rate')

	args = parser.parse_args()
	cmd_serial = None
	lin_serial = None
	urandom = None
	dec = None
	enc = None
	relais_pin = 71 # PC07
	id = 0x20

	try:
		input = []
		offset = 0

		def store(value) -> int:
			global input
			input.append(value)
			return 0

		def load():
			global offset
			o = offset
			offset += 1
			return input[o]

		cmd_serial = serial.Serial(port=args.cmd_dev, baudrate=115200)
		lin_serial = serial.Serial(port=args.lin_dev, baudrate=115200)
		enc = linchpin.RlewEncoder()
		enc.store_callback = store

		dec = linchpin.RlewDecoder()
		dec.load_callback = load

		def lp_run_cmd(cmd: str) -> str:
			cmd_serial.write(bytes(cmd, 'utf-8'))
			time.sleep(0.1)
			data = cmd_serial.read_all()
			return str(data,'utf-8')

		r = lp_run_cmd('?V\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		print(f'version: {r}')

		# r = lp_run_cmd('?SB\n')
		# (e, r) = linchpin.lp_parse_cmd_reply(r)
		# signal_buffer_size = int(r)
		# print(f'signal buffer size: {signal_buffer_size}')

		r = lp_run_cmd('?E\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		little_endian = linchpin.LP_LITTLE_ENDIAN == r
		print(f'little endian: {little_endian}')
		le_str = "little" if little_endian else "big"

		# stop lin
		r = lp_run_cmd('!lmo\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed to turn off lin')

		# # turn off relais powering lin slave
		# r = lp_run_cmd(f'!p {relais_pin} 0\n')
		# (e, r) = linchpin.lp_parse_cmd_reply(r)
		# if linchpin.LinchpinError.NONE.value != e:
		# 	raise ValueError(f'failed to turn off relais')

		# time.sleep(0.1)

		# turn on relais powering lin slave
		r = lp_run_cmd(f'!p {relais_pin} 1\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed to turn off relais')



		oversampling = 8
		baud = 19200
		break_bits = 13
		inter_byte_space = 0


		r = lp_run_cmd(f'!f {oversampling * baud}\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed to set frequency to {oversampling * baud}')

		# # wakey wakey force bus to low for 250μs to 5 ms,
		# #for _ in range((oversampling * baud * 5) // 1000):
		# for _ in range(break_bits):
		# 	for _ in range(oversampling):
		# 		enc.add(0)

		# # slaves show now detect the rising edge and be ready in max 100ms
		# for _ in range((oversampling * baud * 100) // 1000):
		# 	enc.add(1)

		for a0 in range(4):

			# break field
			for _1 in range(break_bits):
				for _2 in range(oversampling):
					enc.add(0)

			# 1 bit time break delimiter
			for _1 in range(oversampling):
				enc.add(1)

			for _1 in range(inter_byte_space):
				enc.add(1)

			# sync field
			# start bit
			for _ in range(oversampling):
				enc.add(0)

			for bit in lsb_first_bits(0x55):
				for _1 in range(oversampling):
					enc.add(int(bit))

			# stop bit
			for _1 in range(oversampling):
				enc.add(1)

			for _1 in range(inter_byte_space):
				enc.add(1)


			# pid field
			# start bit
			for _1 in range(oversampling):
				enc.add(0)

			for bit in lsb_first_bits(pid(id)):
				for _1 in range(oversampling):
					enc.add(int(bit))

			# stop bit
			for _1 in range(oversampling):
				enc.add(1)

			for _1 in range(inter_byte_space):
				enc.add(1)


			# send recessive placeholder for frame data
			# xxx = 0
			for _1 in range(174):
				for _2 in range(oversampling):
					enc.add(1)
					# xxx += 1

			# print(f"xxx {xxx}")

			# # send recessive delay between frames
			# for _ in range((baud * oversampling) // 10):
			# 	enc.add(1)



		enc.finish()

		print("input")
		for i in input:
			print(f"{i:08x}")

		dec_value = None
		dec_count = 0

		def print_dec_value():
			print(f"{dec_value}: {dec_count}")

		while True:
			value = dec.remove()
			if None is value:
				if None is not dec_value:
					print_dec_value()

				break

			if None is dec_value:
				dec_value = value
				dec_count = 1
			elif dec_value != value:
				print_dec_value()
				dec_value = value
				dec_count = 1
			else:
				dec_count += 1


		# # turn on relais powering lin slave
		# r = lp_run_cmd(f'!p {relais_pin} 1\n')
		# (e, r) = linchpin.lp_parse_cmd_reply(r)
		# if linchpin.LinchpinError.NONE.value != e:
		# 	raise ValueError(f'failed to turn on relais')

		# # wait for slave startup
		# time.sleep(0.5)

		# urandom = open("/dev/urandom", "rb")
		# input = urandom.read(16)
		# lin_serial.write(urandom.read(16))

		print(f"{len(input)} u32 to send")

		for i in input:
			for j in range(32):
				bit = 1 << (31-j)
				value = (i & bit) == bit
				print(f"{int(value)}", end="")

			print()

			b = i.to_bytes(4, le_str)
			w = lin_serial.write(b)
			if w != 4:
				raise ValueError(f'failed send 4 byte to lin')

		print()

		# start lin
		r = lp_run_cmd('!lms\n')

		while True:
			# if lin_serial.in_waiting >= 4:
			output = lin_serial.read(4)
			value = int.from_bytes(output, byteorder=le_str, signed=False)
			print(f"{value:08x}")
			if 0 == value:
				break

		r = lp_run_cmd('?L\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed get lin state')

		if r:
			print(f"lin xmit failed with flags: {r}")
		else:
			print(f"lin OK")

	finally:
		if None is not cmd_serial:
			cmd_serial.close()
		if None is not lin_serial:
			lin_serial.close()
		if None is not urandom:
			urandom.close()
		if None is not dec:
			dec.close()
		if None is not enc:
			enc.close()
