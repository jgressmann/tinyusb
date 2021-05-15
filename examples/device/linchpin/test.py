#! /usr/bin/env python3

import asyncio
import base64
import serial
import serial_asyncio
import sys
import time



import linchpin

def bits(b: int):
	for i in range(8):
		bit = 1 << i
		yield (b & bit) == bit

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

	try:
		input = []

		def store(value) -> int:
			input.append(value)
			return 0

		cmd_serial = serial.Serial(port=args.cmd_dev, baudrate=115200)
		lin_serial = serial.Serial(port=args.lin_dev, baudrate=115200)
		enc = linchpin.RlewEncoder()
		enc.store_callback = store

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


		oversampling = 16

		# 13 bit time break field
		for _ in range(13):
			for _ in range(oversampling):
				enc.add(0)

		# 1 bit time break delimiter
		for _ in range(1):
			for _ in range(oversampling):
				enc.add(1)

		# sync field
		# start bit
		for _ in range(oversampling):
			enc.add(0)

		for bit in bits(0x55):
			for _ in range(oversampling):
				enc.add(int(bit))

		# stop bit
		for _ in range(oversampling):
			enc.add(1)

		# # send recessive
		# for i in range(1000):
		# 	for j in range(oversampling):
		# 		enc.add(1)

		enc.finish()

		urandom = open("/dev/urandom", "rb")

		# input = urandom.read(16)
		# lin_serial.write(urandom.read(16))
		for i in input:
			b = i.to_bytes(4, le_str)
			lin_serial.write(b)

		# start lin
		r = lp_run_cmd('!lms\n')

		while True:
			# if lin_serial.in_waiting >= 4:
			output = lin_serial.read(4)
			value = int.from_bytes(output, byteorder=le_str, signed=False)
			print(hex(value))
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


	sys.exit(0)



	def blocking_io():
		import sys
		while True:
			try:
				b = serialPort.read()
				sys.stdout.buffer.write(b)
			except serial.SerialException as e:
				break
		# print(repr(serialPort.readall())

	def blocking_write():
		serialPort.write(b'RUN\n')
		x = base64.b64encode(b'\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01\x81\x01')
		print(repr(x))
		serialPort.write(x + b'====')
		serialPort.write(b'?S')

	# connected = False
	# t = None

	# class Output(asyncio.Protocol):
	# 	def connection_made(self, transport):
	# 		self.transport = transport
	# 		print('port opened', transport)
	# 		# transport.serial.rts = False
	# 		# transport.write(b'hello world\n')
	# 		transport.write(b'RUN\n')
	# 		connected = True
	# 		t = transport

	# 	def data_received(self, data):
	# 		print('data received', repr(data))
	# 		# self.transport.close()

	# 	def connection_lost(self, exc):
	# 		print('port closed')
	# 		asyncio.get_event_loop().stop()


	# async def send_foo():
	# 	while not connected:
	# 		await asyncio.sleep(0.1)

	# 	t.write(b'asdfdasfasdfdasf')


	import threading
	t = threading.Thread(target=blocking_io)
	# t.daemon = True
	t.start()

	blocking_write()

	# async def loo():
	# 	await asyncio.gather(
	# 		asyncio.to_thread(blocking_io),
	# 		asyncio.to_thread(blocking_write))

	# serialPort.write(b"?F\n")
	# print(serialPort.read().decode("utf8"))

	# loop = asyncio.get_event_loop()
	# sender = asyncio.create_task(send_foo)
	# coro = serial_asyncio.create_serial_connection(loop, Output, args.device, baudrate=args.baud)
	# asyncio.run(loo())
	# loop.run_until_complete(coro)
	# loop.run_until_complete(sender)
	# loop.run_forever()
	# loop.close()
	t.join()
