#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
#
# Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
#

import argparse
import binascii
import struct
import sys


SUPER_DFU_HEADER_MARKER = b'SuperDFU AH\0\0\0\0\0'
SUPER_DFU_HEADER_LEN = 0x40
SUPER_DFU_HEADER_BOM = 0x1234
SUPER_DFU_APP_OFFSET = 1024
SUPER_DFU_FOOTER_MARKER = b'SuperDFU AF\0\0\0\0\0'
SUPER_DFU_FOOTER_LEN = 16



try:
	parser = argparse.ArgumentParser(description='patch firmware bin file SuperDFU header')
	parser.add_argument('files', nargs=argparse.REMAINDER)
	parser.add_argument('--strict', type=bool, default=False)
	args = parser.parse_args()


	header_struct_format = "<16sBBBBLLLL"
	footer_struct_format = "16s"

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
				if (header[1] != 3):
					print(f"WARN: SuperDFU header version {header[1]} found in {file} not supported")
					break

				bad_endian = True

				if header[3] == 0x34:
					if header[4] == 0x12:
						bad_endian = False # little endian
				elif header[3] == 0x12:
					if header[4] == 0x34:
						bad_endian = False # big endian
						header_struct_format = ">" + header_struct_format[1:]

				if bad_endian:
					print(f"WARN: Unrecognized BOM {header[3]:02x}{header[4]:02x}h in {file}")
					break

				end_index = content.find(SUPER_DFU_FOOTER_MARKER, start_index + SUPER_DFU_APP_OFFSET)
				if end_index == -1:
					print(f"WARN: No SuperDFU footer found in {file}")
					break

				# compute app len & crc and repack
				app_len = end_index - start_index - SUPER_DFU_APP_OFFSET
				app_crc = binascii.crc32(content[start_index+SUPER_DFU_APP_OFFSET:end_index])
				header[6] = 0 # header crc
				header[7] = app_len
				header[8] = app_crc
				struct.pack_into(header_struct_format, content, start_index, *header)
				print(f"SuperDFU application header found @ {start_index:08x}, footer @ {end_index:08x}, app len {app_len:08x}, app crc {app_crc:08x}")


				# compute header crc and repack
				header_crc = binascii.crc32(content[start_index:start_index+SUPER_DFU_HEADER_LEN])
				header = list(struct.unpack(header_struct_format, content[start_index:start_index + struct.calcsize(header_struct_format)]))
				header[6] = header_crc
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




