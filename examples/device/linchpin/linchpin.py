import ctypes
from enum import Enum
import os
import re


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

rlew32_p_uint32_type = ctypes.POINTER(ctypes.c_uint32)
rlew32_store_func_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32)
rlew32_load_func_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, rlew32_p_uint32_type)
rlew32 = ctypes.cdll.LoadLibrary("rlew32.so")


class RlewEncoder:
	FLAG_OVERFLOW = 0x1

	def __init__(self):
		self.store_callback = lambda x: 0
		def c(_, value):
			return self.store_callback(value)
		self._c_callback = rlew32_store_func_type(c)
		self.rle = rlew32.rlew32_enc_new()

	def add(self, bit):
		rlew32.rlew32_enc_bit(self.rle, None, self._c_callback, bit)

	def finish(self):
		rlew32.rlew32_enc_finish(self.rle, None, self._c_callback)

	def __enter__(self):
		return self

	def __exit__(self, type, value, tb):
		self.close()

	def close(self):
		if None is not self.rle:
			rlew32.rlew32_enc_free(self.rle)
			self.rle = None

	@property
	def flags(self) -> int:
		return rlew32.rlew32_enc_flags(self.rle)


class RlewDecoder:
	FLAG_UNDERFLOW = 0x1
	FLAG_EOS       = 0x2

#define RLEW_FLAG_DEC_UNDERFLOW 0x01
#define RLEW_FLAG_DEC_EOS       0x02
	def __init__(self):
		self.rle = rlew32.rlew32_dec_new()
		self.load_callback = RlewDecoder._default_load_callback

		def c(_, value_ptr: rlew32_p_uint32_type) -> int:
			r = self.load_callback()
			value_ptr[0] = int(r)
			return 0

		self.c = rlew32_load_func_type(c)

	def remove(self) -> int:
		return rlew32.rlew32_dec_bit(self.rle, None, self.c)

	def __enter__(self):
		return self

	def __exit__(self, type, value, tb):
		self.close()

	def close(self):
		if None is not self.rle:
			rlew32.rlew32_dec_free(self.rle)
			self.rle = None

	@property
	def flags(self) -> int:
		return rlew32.rlew32_dec_flags(self.rle)

	@staticmethod
	def _default_load_callback(_):
		pass

