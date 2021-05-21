import ctypes
from enum import Enum
import os
import re
from typing import Tuple


class LinchpinError(Enum):
	NONE = 0
	UNKNOWN_CMD = -1
	INVALID_PARAM = -2
	OUT_OF_RANGE = -3
	MALFORMED = -4
	FAILED = -5
	UNKNOWN = -6


LP_LITTLE_ENDIAN = "LE"
LP_BIG_ENDIAN = "BE"

LP_CMD_REPLY_PATTERN = re.compile('^(\\d+)\s+(.*?)\s*$', re.MULTILINE | re.DOTALL)

def lp_parse_cmd_reply(rep: str) -> (int, str):
	if None is rep or not len(rep):
		raise ValueError("'rep' must not be empty")

	m = LP_CMD_REPLY_PATTERN.match(rep)
	if m:
		return (int(m.group(1)), m.group(2))

	return (LinchpinError.UNKNOWN, "reply parse error")




# print(os.environ)
p_uint32 = ctypes.POINTER(ctypes.c_uint32)
rlew32 = ctypes.cdll.LoadLibrary("rlew32.so")



class RlewError(Enum):
	NONE = 0
	FALSE = 0
	TRUE = 1
	UNDERFLOW = (1<<1)
	OVERFLOW = (2<<1)
	EOS = (3<<1)

	@staticmethod
	def is_error(e) -> bool:
		return int(e) > 2



class RlewEncoder:
	def __init__(self):
		self.rle = rlew32.rlew32_enc_new()
		self.store_callback = lambda x: 0

	def add(self, bit):
		while True:
			error = rlew32.rlew32_enc_bit(self.rle, bit)
			if not RlewError.is_error(error):
				break

			# overflow
			assert int(error) == RlewError.OVERFLOW.value

			value = ctypes.c_uint32()
			error = rlew32.rlew32_enc_output_take(self.rle, p_uint32(value))
			if RlewError.NONE.value == int(error):
				self.store_callback(int(value.value))

	# def take(self) -> Tuple[int, int]:
	# 	value = ctypes.c_uint32()
	# 	e = rlew32.rlew32_enc_take(self.rle, p_uint32(value))
	# 	return (int(e), int(value))

	def finish(self):
		while True:
			error = rlew32.rlew32_enc_finish(self.rle)
			if not RlewError.is_error(error):
				break

			# overflow
			assert int(error) == RlewError.OVERFLOW.value

			value = ctypes.c_uint32()
			error = rlew32.rlew32_enc_output_take(self.rle, p_uint32(value))
			if RlewError.NONE.value == int(error):
				self.store_callback(int(value.value))

		for _ in range(3):
			self.store_callback(0)

	def __enter__(self):
		return self

	def __exit__(self, type, value, tb):
		self.close()

	def close(self):
		if None is not self.rle:
			rlew32.rlew32_enc_free(self.rle)
			self.rle = None



class RlewDecoder:
	def __init__(self):
		self.rle = rlew32.rlew32_dec_new()

	def remove(self) -> int:
		return rlew32.rlew32_dec_bit(self.rle)

	def __enter__(self):
		return self

	def __exit__(self, type, value, tb):
		self.close()

	def close(self):
		if None is not self.rle:
			rlew32.rlew32_dec_free(self.rle)
			self.rle = None

