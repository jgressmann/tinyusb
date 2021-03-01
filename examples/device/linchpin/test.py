#! /usr/bin/env python3

import asyncio
import base64
import serial
import serial_asyncio



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
