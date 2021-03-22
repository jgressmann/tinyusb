#! /usr/bin/env python3

import asyncio
import base64
import fastlz
import serial
import serial_asyncio
import sys
import time



import linchpin

if __name__ == "__main__":
	import argparse

	parser = argparse.ArgumentParser(description='LINchPIN test program')
	parser.add_argument('-f', '--frequency', metavar='F', type=int, help='signal frequency [Hz]')
	parser.add_argument('-d', '--device', metavar='DEV', required=True, help='serial port to open')
	parser.add_argument('-b', '--baud', metavar='BAUD', type=int, default=115200, help='Baud rate')

	args = parser.parse_args()


	serialPort = serial.Serial(
		port=args.device,
		baudrate=115200)

	def lp_run_cmd(cmd: str) -> str:
		serialPort.write(bytes(cmd, 'utf-8'))
		time.sleep(0.1)
		data = serialPort.read_all()
		return str(data,'utf-8')
		# SerialTimeoutException

	r = lp_run_cmd('?V\n')
	(e, r) = linchpin.lp_parse_cmd_reply(r)
	print(f'version: {r}')

	r = lp_run_cmd('?SB\n')
	(e, r) = linchpin.lp_parse_cmd_reply(r)
	signal_buffer_size = int(r)
	print(f'signal buffer size: {signal_buffer_size}')

	r = lp_run_cmd('!R\n')
	(e, r) = linchpin.lp_parse_cmd_reply(r)

	if linchpin.LinchpinError.NONE == e:
		print(f'run started')


	e = linchpin.LinchpinRleEncoder()

	with open("/dev/urandom", "rb") as f:
		# while True:
		input = f.read(int(signal_buffer_size / 4))
		for i in input:
			byte_value = i
			for _ in range(8):
				bit_value = (byte_value & 0x1) == 0x1
				e.push(bit_value)
				byte_value >>= 1

		rle_data = e.read()
		compressed_data = fastlz.compress(bytes(rle_data), level=1)
		decompressed_data = fastlz.decompress(bytes(bytearray(compressed_data)))
		if rle_data != decompressed_data:
			raise ValueError("FastLZ is broken")

		# compressed_data2 = fastlz.compress(bytes(input))
		print(f'compressed {len(compressed_data)} bytes')
		print(f'compressed[0] {compressed_data[0]:02x}')
		# print(compressed_data.hex())
		b64e = base64.b64encode(compressed_data)
		end = b64e[-3:].decode('utf-8')
		print(f'b64 {len(b64e)} bytes end with {end}')
		# print(f'{len(compressed_data2)} bytes')

		w = serialPort.write(b64e)
		print(f'wrote {w} bytes')
		serialPort.write(b'\n')

		input = serialPort.read_all()
			# break
		print(f'input {len(input)} bytes')

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
