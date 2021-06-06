#! /usr/bin/env python3

import bitstring
import linchpin
import serial
import sys
import time
from typing import Sequence, Union
import unittest








class LinConformanceTest(unittest.TestCase):

	# cmd = None
	# lin = None
	RELAIS_PIN = 71
	MASTER_REQUEST_ID = 0x3C
	SLAVE_RESPONSE_ID = 0x3D
	T_BREAK_FIELD_MIN = 13
	T_BREAK_FIELD_MAX = 26
	T_FRAME_MAX = 174
	OVERSAMPLING_MAX = 32

	@classmethod
	def setUpClass(cls):
		cls.cmd = serial.Serial(port=args.cmd_dev, baudrate=args.baud)
		cls.lin = serial.Serial(port=args.lin_dev, baudrate=args.baud, timeout=0)

		r = cls.lp_run_cmd('?V\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed to read version')

		cls.fw_version = r

		# r = lp_run_cmd('?SB\n')
		# (e, r) = linchpin.lp_parse_cmd_reply(r)
		# signal_buffer_size = int(r)
		# print(f'signal buffer size: {signal_buffer_size}')

		r = cls.lp_run_cmd('?E\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		cls.little_endian = linchpin.LP_LITTLE_ENDIAN == r
		# print(f'little endian: {little_endian}')
		cls.le_str = "little" if cls.little_endian else "big"

		# turn off relais powering lin slave (in case it was on)
		r = cls.lp_run_cmd(f'!P {cls.RELAIS_PIN} 0\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed to turn off relais')

		time.sleep(0.5)

		# turn on relais powering lin slave
		r = cls.lp_run_cmd(f'!P {cls.RELAIS_PIN} 1\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed to turn on relais')

		time.sleep(0.2)

	@classmethod
	def tearDownClass(cls):
		if None is not cls.lin:
			cls.lin.close()
			cls.lin = None

		if None is not cls.cmd:
			try:
				# turn off relais powering lin slave
				cls.lp_run_cmd(f'!p {cls.RELAIS_PIN} 0\n')
			except:
				pass

			cls.cmd.close()
			cls.cmd = None

	@classmethod
	def lp_run_cmd(cls, cmd: str) -> str:
		cls.cmd.write(bytes(cmd, 'utf-8'))
		time.sleep(0.1)
		data = cls.cmd.read_all()
		return str(data,'utf-8')

	def setUp(self) -> None:
		self.lin_input_buffer = []
		self.lin_output_buffer = []
		self.lin_input_string = bitstring.BitArray()
		self.lin_output_string = bitstring.BitArray()
		self.lin_output_offset = 0
		self.oversampling = 16
		self.baud_rate = 19200
		self.inter_byte_space = 0
		self.master_delay_s = 0.01

		def input_store(value) -> None:
			self.lin_input_buffer.append(value)

		def output_load() -> int:
			o = self.lin_output_offset
			self.lin_output_offset += 1
			return self.lin_output_buffer[o]

		self.input_enc = linchpin.RlewEncoder()
		self.input_enc.store_callback = input_store

		self.output_dec = linchpin.RlewDecoder()
		self.output_dec.load_callback = output_load

		# stop lin
		r = self.lp_run_cmd('!lmo\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed to turn off lin')

		self.set_frequency(self.baud_rate * self.oversampling)

		return super().setUp()

	def tearDown(self) -> None:
		if None is not self.input_enc:
			self.input_enc.close()
			self.input_enc = None

		if None is not self.output_dec:
			self.output_dec.close()
			self.output_dec = None

		return super().tearDown()

	# def sync(self):
	# 	for _ in range(int(self.oversampling * inter_byte_space)):
	# 		enc.add(1)

	def set_frequency(self, f: int) -> None:
		r = self.__class__.lp_run_cmd(f'!f {f}\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed to set frequency to {f}')

		self.frequency = f

	def g(self, value: int, len: int) -> None:
		# x = f"{value}"

		for _ in range(len):
			self.lin_input_string.append(f"0x{value:x}")
			# if 0 == len(self.lin_input_string) or self.lin_input_string[-1] != x:
			# 	# self.lin_input_ranges.append((b, 1))

			self.input_enc.add(value)

	def g0(self, len: int) -> None:
		self.g(0, len)

	def g1(self, len: int) -> None:
		self.g(1, len)

	def gb(self, byte: int) -> None:
		# start bit
		self.g0(self.oversampling)

		# byte
		for bit in self.__class__.lsb_first_bits(byte):
			self.g(bit, self.oversampling)

		# stop bit
		self.g1(self.oversampling)

	def gs(self, ints: Union[bytes, Sequence[int]]) -> None:
		for i in ints:
			# value
			self.gb(i)

			# interspace
			self.g1(self.oversampling * self.inter_byte_space)

	@staticmethod
	def lsb_first_bits(b: int):
		for i in range(8):
			bit = 1 << i
			yield (b & bit) == bit

	@staticmethod
	def pid(id: int) -> int:
		# P0 = ID0 ^ ID1 ^ ID2 ^ ID4
		p0 = int((id & 1) == 1) ^ int((id & 2) == 2) ^ int((id & 4) == 4) ^ int((id & 16) == 16)
		# P1 = ~(ID1 ^ ID3 ^ ID4 ^ ID5)
		p1 = 1 ^ int((id & 2) == 2) ^ int((id & 8) == 8) ^ int((id & 16) == 16) ^ int((id & 32) == 32)

		pid = (p1 << 7) | (p0 << 6) | (id & 63)

		return pid

	@staticmethod
	def checksum(ints: Union[bytes, Sequence[int]]) -> int:
		sum = 0
		for i in ints:
			sum += int(i)
			if sum >= 256:
				sum -= 255

		return sum ^ 255

	def start_lin(self):
		r = self.__class__.lp_run_cmd(f'!LMS\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed to start lin')

	def run_lin(self):
		lin_started = False
		bar_out = bytearray()
		bar_in = bytearray()
		offset_out = 0
		offset_in = 0
		sleeps = 0
		usb_sleep_default = 0.001 # 1ms

		for i in self.lin_input_buffer:
			bar_out.extend(i.to_bytes(4, self.__class__.le_str))

		def save_lin_input():
			nonlocal offset_in
			nonlocal bar_in
			result = False

			while offset_in + 4 <= len(bar_in):
				start = offset_in
				offset_in += 4
				end = offset_in
				value = int.from_bytes(bar_in[start:end], byteorder=self.__class__.le_str, signed=False)
				self.lin_output_buffer.append(value)
				result = result or 0 == value

			return result

		while True:
			any = False
			output_done = False

			if offset_out < len(bar_out):
				left = len(bar_out) - offset_out
				w = self.__class__.lin.write(bar_out[offset_out:])

				if w < left:
					if not lin_started:
						lin_started = True
						any = True
						self.start_lin()
				else:
					any = True
					offset_out += w

			else:
				output_done = True

				if not lin_started:
					lin_started = True
					any = True
					self.start_lin()

			bits = self.__class__.lin.read_all()
			if len(bits):
				any = True
				bar_in.extend(bits)
				input_done = save_lin_input()
				if output_done and input_done:
					break

			if any:
				sleeps = 0
			else:
				sleeps += output_done

				if sleeps < 3:
					time.sleep(usb_sleep_default)
				else:
					break


		# read possible trailing zeros
		time.sleep(usb_sleep_default)
		bits = self.__class__.lin.read_all()
		if len(bits):
			bar_in.extend(bits)
			save_lin_input()

		# check result of lin run
		r = self.__class__.lp_run_cmd('?L\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed get lin state')

		self.assertEqual(0, e, msg=r)


	def test_2_1_slave_assign_id_from_range(self):
		pass

	def test_2_2_slave_answer_master_request(self):
		pass

	@unittest.skip('meh')
	def test_2_3_slave_error_in_received_frame(self):
		# break
		self.g0(self.oversampling * self.__class__.T_BREAK_FIELD_MIN)

		# 1 bit time break delimiter
		self.g1(self.oversampling)

		pid_mr = self.__class__.pid(self.__class__.MASTER_REQUEST_ID)
		nad = 0x42
		pci = 0x06
		sid = 0xb2
		supplier_id = 0x0000
		function_id = 0x0008
		variant = 0x0f
		supplier_id_lsb = 0
		supplier_id_msb = 0
		function_id_lsb = 8
		function_id_msb = 0
		identifier = 0
		# classic checksum
		csum = self.__class__.checksum([nad, pci, sid, identifier, supplier_id_lsb, supplier_id_msb, function_id_lsb, function_id_msb])

		self.gs([0x55, pid_mr, nad, pci, sid, identifier, supplier_id_lsb, supplier_id_msb, function_id_lsb, function_id_msb, csum])

		# wait for next slot
		self.g1(int(self.master_delay_s * self.frequency))

		# break
		self.g0(self.oversampling * self.__class__.T_BREAK_FIELD_MIN)

		# 1 bit time break delimiter
		self.g1(self.oversampling)

		# gen response header
		pid_sr = self.__class__.pid(self.__class__.SLAVE_RESPONSE_ID)
		self.gs([0x55, pid_sr, nad])
		bytes = 8
		bits_per_byte = 10
		#self.g1(bytes * (bits_per_byte + self.inter_byte_space) * self.oversampling)
		self.g1(self.__class__.T_FRAME_MAX * self.oversampling)


		self.input_enc.finish()
		for i in self.lin_input_buffer:
			b = i.to_bytes(4, self.__class__.le_str)
			w = self.__class__.lin.write(b)
			if w != 4:
				raise ValueError(f'failed send 4 byte to lin')


		r = self.__class__.lp_run_cmd('!LMS\n')

		while True:
			# if lin_serial.in_waiting >= 4:
			bits = self.__class__.lin.read(4)
			value = int.from_bytes(bits, byteorder=self.__class__.le_str, signed=False)
			self.lin_output_buffer.append(value)
			if 0 == value:
				break

		r = self.__class__.lp_run_cmd('?L\n')
		(e, r) = linchpin.lp_parse_cmd_reply(r)
		if linchpin.LinchpinError.NONE.value != e:
			raise ValueError(f'failed get lin state')

		self.assertEqual(0, e, msg=r)


	def test_data_frame(self):
		# break
		self.g0(self.oversampling * self.__class__.T_BREAK_FIELD_MIN)

		# 1 bit time break delimiter
		self.g1(self.oversampling)

		# header
		pid = self.__class__.pid(0x20)
		self.gs([0x55, pid])

		# wait for data
		self.g1(self.__class__.T_FRAME_MAX * self.oversampling)

		self.input_enc.finish()

		self.run_lin()

		print("output")
		for value in self.lin_output_buffer:
			print(f"{value:08x}")

		while True:
			bit = self.output_dec.remove()
			if None is bit:
				break

			self.lin_output_string.append(f"0x{bit:x}")

		self.assertEqual(0, self.lin_input_string[0])

		print(f"input: {self.lin_input_string}")
		print(f"output: {self.lin_output_string}")

if __name__ == "__main__":
	import argparse
	global args

	parser = argparse.ArgumentParser(description='lin conformance test')
	parser.add_argument('-f', '--frequency', metavar='F', type=int, help='signal frequency [Hz]')
	parser.add_argument('-c', '--cmd-dev', metavar='DEV', required=True, help='command serial port')
	parser.add_argument('-l', '--lin-dev', metavar='DEV', required=True, help='lin serial port')
	parser.add_argument('-b', '--baud', metavar='BAUD', type=int, default=115200, help='Baud rate')

	args, left = parser.parse_known_args()

	sys.argv = sys.argv[:1]+left

	unittest.main()

