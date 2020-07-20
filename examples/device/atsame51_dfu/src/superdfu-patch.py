#!/usr/bin/env python3

import argparse
import crcmod
import struct
import sys


LITTLE_ENDIAN = 'little'
BIG_ENDIAN = 'big'
SUPER_DFU_HEADER_MARKER = b'SuperDFU AHv1\0\0\0'
SUPER_DFU_HEADER_LEN = 1024
SUPER_DFU_FOOTER_MARKER = b'SuperDFU AFv1\0\0\0'
SUPER_DFU_FOOTER_LEN = 16

CRC = crcmod.mkCrcFun(0x1EDB88320, initCrc=0xffffffff, rev=True)
#CRC32 = crcmod.predefined.mkPredefinedCrcFun('crc-32')


try:
	parser = argparse.ArgumentParser(description='patch firmware bin file SuperDFU header')
	parser.add_argument('files', nargs=argparse.REMAINDER)
	parser.add_argument('-e', '--endian', choices=['little','big'], required=True)
	parser.add_argument('--strict', type=bool, default=False)
	args = parser.parse_args()

	#header_struct_format = "16sLLBBBB64s233L"
	header_struct_format = "16sLLBBBB64s"
	footer_struct_format = "16s"
	if args.endian == LITTLE_ENDIAN:
		header_struct_format = "<" + header_struct_format
	else:
		header_struct_format = ">" + header_struct_format

	for file in args.files:
		changed = False
		with open(file, "rb") as f:
			start_index = 0
			content = bytearray(f.read())

			while start_index < len(content):
				start_index = content.find(SUPER_DFU_HEADER_MARKER, start_index)
				if start_index == -1:
					break

				header = list(struct.unpack(header_struct_format, content[start_index:start_index + struct.calcsize(header_struct_format)]))

				end_index = content.find(SUPER_DFU_FOOTER_MARKER, start_index + SUPER_DFU_HEADER_LEN)
				if end_index == -1:
					break

				app_len = end_index - start_index - SUPER_DFU_HEADER_LEN
				app_crc = CRC(content[start_index+SUPER_DFU_HEADER_LEN:end_index], crc=0xffffffff)

				print(f"SuperDFU application header found @ {start_index:08x}, footer @ {end_index:08x}, len {app_len:08x}, crc {app_crc:08x}")
				# print(repr(header))

				header[1] = app_len
				header[2] = app_crc

				struct.pack_into(header_struct_format, content, start_index, *header)
					# header[0],
					# header[1],
					# header[2],
					# header[3],
					# header[4],
					# header[5],
					# header[6],
					# header[7],
					# header[8],
					# header[9],
					# header[10],
					# header[11],
					# header[12],
					# header[13],
					# header[14],
					# header[15],
					# header[16])

				changed = True

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




