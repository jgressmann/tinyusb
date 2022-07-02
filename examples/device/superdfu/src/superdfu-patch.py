#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
#
# Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
#

import argparse
import binascii
import struct
import sys


DFU_APP_TAG_MAGIC_STRING = b'SuperDFU AT\0\0\0\0\0'
DFU_APP_TAG_SIZE = 0x40
DFU_APP_TAG_BOM = 0x1234


def patch_file(file: str, length: int) -> bool:
	header_struct_format = "<16sBBBBLLLL"

	with open(file, "rb") as f:
		content = bytearray(f.read())

		tag_index = content.rfind(DFU_APP_TAG_MAGIC_STRING)
		if -1 == tag_index:
			print(f"WARN: SuperDFU tag not found")
			return False, content, None

		tag = list(struct.unpack(header_struct_format, content[tag_index:tag_index + struct.calcsize(header_struct_format)]))
		if tag[0] != DFU_APP_TAG_MAGIC_STRING:
			print(f"WARN: SuperDFU tag magic string mismatch {tag[0].hex()}")
			return False, content, None

		if (tag[1] != 1):
			print(f"WARN: SuperDFU tag version {tag[1]} not supported")
			return False, content, None

		bad_endian = True

		if tag[3] == 0x34:
			if tag[4] == 0x12:
				bad_endian = False # little endian
		elif tag[3] == 0x12:
			if tag[4] == 0x34:
				bad_endian = False # big endian
				header_struct_format = ">" + header_struct_format[1:]

		if bad_endian:
			print(f"WARN: Unrecognized BOM {tag[3]:02x}{tag[4]:02x}h")
			return False, content, None

		print(f"SuperDFU app tag found @ {tag_index:08x}")
		# compute crc and repack
		app_start = tag_index - length

		print(f"Compute app crc from @ {app_start:08x}, len {length:x}h")
		app_crc = binascii.crc32(content[app_start:tag_index])
		tag[6] = 0 # tag crc
		tag[7] = length
		tag[8] = app_crc
		struct.pack_into(header_struct_format, content, tag_index, *tag)
		print(f"App crc {app_crc:08x}")


		# compute tag crc and repack
		tag_crc = binascii.crc32(content[tag_index:tag_index+DFU_APP_TAG_SIZE])
		tag = list(struct.unpack(header_struct_format, content[tag_index:tag_index + struct.calcsize(header_struct_format)]))
		tag[6] = tag_crc
		struct.pack_into(header_struct_format, content, tag_index, *tag)
		print(f"Tag crc {tag_crc:08x}")

		# extract full tag
		tag = bytes(content[tag_index:tag_index+DFU_APP_TAG_SIZE])

		return True, content, tag


try:
	parser = argparse.ArgumentParser(description='patch firmware bin file SuperDFU tag')
	parser.add_argument('file', metavar='FILE', help="file to patch")
	parser.add_argument('saddr', metavar='SADDR', help="start address of app section (hex)")
	parser.add_argument('eaddr', metavar='EADDR', help="end address of app section (hex)")
	parser.add_argument('--tag', metavar='TAG', required=False, help="file to save tag")
	parser.add_argument('--strict', type=bool, default=False)
	args = parser.parse_args()


	saddr = int(args.saddr, 16)
	eaddr = int(args.eaddr, 16)
	file = args.file

	if saddr < 0:
		print("ERROR: invalid start address")
		sys.exit(1)

	if eaddr <= 0:
		print("ERROR: invalid end address")
		sys.exit(1)

	if eaddr <= saddr:
		print("ERROR: start address must be smaller than end address")
		sys.exit(1)

	changed, content, tag = patch_file(file, eaddr-saddr)

	if changed:
		print(f"Saving changed file to {file}...", end='')
		with open(file, "wb") as f:
			f.write(content)

		print(f"done")

		if args.tag:
			print(f"Saving tag to {args.tag}...", end='')
			with open(args.tag, "wb") as f:
				f.write(tag)

			print(f"done")

	elif args.strict and not changed:
		print(f"ERROR: {file} unchanged")
		sys.exit(2)


except Exception as e:
	print("ERROR: " + str(e))
	sys.exit(1)




