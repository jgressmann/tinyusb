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

rlew32_store_func_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32)
rlew32_load_func_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32))
rlew32 = ctypes.cdll.LoadLibrary("rlew32.so")


class RlewEncoder:
	def __init__(self):
		self.e = rlew32.rlew32_enc_new()
		self.store_callback = RlewEncoder._default_store_callback

		def s(a, b: int) -> int:
			return self.store_callback(b)

		self.c = rlew32_store_func_type(s)

	def add(self, bit):
		rlew32.rlew32_enc_bit(self.e, None, self.c, bit)

	def finish(self):
		rlew32.rlew32_enc_finish(self.e, None, self.c)



	def __enter__(self):
		return self

	def __exit__(self, type, value, tb):
		self.close()

	def close(self):
		if None is not self.e:
			rlew32.rlew32_enc_free(self.e)
			self.e = None

	@staticmethod
	def _default_store_callback(value: ctypes.c_uint32) -> int:
		return 0


# class RlewDecoder:
# 	def __init__(self):
# 		self.e = rlew32.rlew32_dec_new()
# 		self.store_callback = RlewDecoder._default_load_callback

# 		def s(a, b: int) -> int:
# 			return self.store_callback(b)

# 		self.c = rlew32_store_func_type(s)

# 	def remove(self) -> bool:
# 		return rlew32.rlew32_dec_bit(self.e, None, self.c)

# 	def __enter__(self):
# 		return self

# 	def __exit__(self, type, value, tb):
# 		self.close()

# 	def close(self):
# 		if None is not self.e:
# 			rlew32.rlew32_dec_free(self.e)
# 			self.e = None

# 	@staticmethod
# 	def _default_load_callback(value: int) -> int:
# 		return 1

