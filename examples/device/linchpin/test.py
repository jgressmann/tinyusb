#! /usr/bin/env python3

import asyncio
import base64
import serial
import serial_asyncio
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



		oversampling = 4
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

		for id in [32, 33, 0x32, 0x33]:

			# break field
			for _ in range(break_bits):
				for _ in range(oversampling):
					enc.add(0)

			# 1 bit time break delimiter
			for _ in range(oversampling):
				enc.add(1)

			for _ in range(inter_byte_space):
				enc.add(1)

			# sync field
			# start bit
			for _ in range(oversampling):
				enc.add(0)

			for bit in lsb_first_bits(0x55):
				for _ in range(oversampling):
					enc.add(int(bit))

			# stop bit
			for _ in range(oversampling):
				enc.add(1)

			for _ in range(inter_byte_space):
				enc.add(1)


			# pid field
			# start bit
			for _ in range(oversampling):
				enc.add(0)

			for bit in lsb_first_bits(pid(id)):
				for _ in range(oversampling):
					enc.add(int(bit))

			# stop bit
			for _ in range(oversampling):
				enc.add(1)

			for _ in range(inter_byte_space):
				enc.add(1)


			# send recessive placeholder for frame data
			for _ in range(100):
				for _ in range(oversampling):
					enc.add(1)


			# # send recessive delay between frames
			# for _ in range((baud * oversampling) // 10):
			# 	enc.add(1)



		enc.finish()

		dec_value = None
		dec_count = 0

		def print_dec_value():
			print(f"{dec_value}: {dec_count}")

		while True:
			value = dec.remove()
			if value < 0:
				if linchpin.RlewDecoder.FLAG_EOS != dec.flags:
					raise ValueError("underflow error")

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
