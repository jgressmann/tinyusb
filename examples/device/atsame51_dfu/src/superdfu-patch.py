#!/usr/bin/env python3

import argparse
import binascii
import struct
import sys


LITTLE_ENDIAN = 'little'
BIG_ENDIAN = 'big'
SUPER_DFU_HEADER_MARKER = b'SuperDFU AH\0\0\0\0\0'
SUPER_DFU_HEADER_LEN = 0x40
SUPER_DFU_APP_OFFSET = 1024
SUPER_DFU_FOOTER_MARKER = b'SuperDFU AF\0\0\0\0\0'
SUPER_DFU_FOOTER_LEN = 16


try:
	parser = argparse.ArgumentParser(description='patch firmware bin file SuperDFU header')
	parser.add_argument('files', nargs=argparse.REMAINDER)
	parser.add_argument('-e', '--endian', choices=['little','big'], required=True)
	parser.add_argument('--strict', type=bool, default=False)
	args = parser.parse_args()

	header_struct_format = "20sLLL"
	footer_struct_format = "16s"
	if args.endian == LITTLE_ENDIAN:
		header_struct_format = "<" + header_struct_format
	else:
		header_struct_format = ">" + header_struct_format

	for file in args.files:
		changed = False
		with open(file, "rb") as f:
			headers_found = 0
			start_index = 0
			content = bytearray(f.read())

			while start_index < len(content):
				start_index = content.find(SUPER_DFU_HEADER_MARKER, start_index)
				if start_index == -1:
					if 0 == headers_found:
						print(f"WARN: No SuperDFU header found in {file}")
					break

				header = list(struct.unpack(header_struct_format, content[start_index:start_index + struct.calcsize(header_struct_format)]))

				end_index = content.find(SUPER_DFU_FOOTER_MARKER, start_index + SUPER_DFU_APP_OFFSET)
				if end_index == -1:
					print(f"WARN: No SuperDFU footer found in {file}")
					break

				# compute app len & crc and repack
				app_len = end_index - start_index - SUPER_DFU_APP_OFFSET
				app_crc = binascii.crc32(content[start_index+SUPER_DFU_APP_OFFSET:end_index])
				header[1] = 0 # header crc
				header[2] = app_len
				header[3] = app_crc
				struct.pack_into(header_struct_format, content, start_index, *header)
				print(f"SuperDFU application header found @ {start_index:08x}, footer @ {end_index:08x}, app len {app_len:08x}, app crc {app_crc:08x}")


				# compute header crc and repack
				header_crc = binascii.crc32(content[start_index:start_index+SUPER_DFU_HEADER_LEN])
				header = list(struct.unpack(header_struct_format, content[start_index:start_index + struct.calcsize(header_struct_format)]))
				header[1] = header_crc
				struct.pack_into(header_struct_format, content, start_index, *header)
				print(f"SuperDFU application header crc {header_crc:08x}")

				changed = True
				headers_found += 1

				start_index = end_index + SUPER_DFU_FOOTER_LEN



		if changed:
			print(f"Saving changed file to {file}...")
			with open(file, "wb") as f:
				f.write(content)

			print(f"Saved")
		elif args.strict and not changed:
			print(f"ERROR: {file} unchanged")
			sys.exit(2)


except Exception as e:
	print("ERROR: " + str(e))
	sys.exit(1)




