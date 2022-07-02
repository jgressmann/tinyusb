/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#pragma once

#define DFU_MS_OS_20_SUBSET_HEADER_FUNCTION_LEN 0x8
#define DFU_MS_OS_20_SUBSET_HEADER_FUNCTION_DATA(interface_index, section_len) \
	/* function subset header: length, type, first interface, reserved, subset length */ \
	U16_TO_U8S_LE(DFU_MS_OS_20_SUBSET_HEADER_FUNCTION_LEN), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), interface_index, 0, U16_TO_U8S_LE(section_len)

#define DFU_MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN 0x14
#define DFU_MS_OS_20_FEATURE_COMPATBLE_ID_DESC_DATA \
	/* MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID */ \
	U16_TO_U8S_LE(DFU_MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00, \
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 /* sub compatible ID, unused */

#define DFU_MS_OS_20_FEATURE_REG_PROPERTY_DEVICE_GUIDS_DESC_LEN 0x84
#define DFU_MS_OS_20_FEATURE_REG_PROPERTY_DEVICE_GUIDS_DESC_DATA \
	/* MS OS 2.0 Registry property descriptor: length, type */ \
	U16_TO_U8S_LE(DFU_MS_OS_20_FEATURE_REG_PROPERTY_DEVICE_GUIDS_DESC_LEN), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), \
	U16_TO_U8S_LE(0x0007) /* REG_MULTI_SZ */, U16_TO_U8S_LE(0x002A), /* wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16 */ \
	'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, \
	'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, \
	0x00, 0x00, \
	U16_TO_U8S_LE(0x0050), /* wPropertyDataLength */ \
	/* bPropertyData: “{f4ef82e0-dc07-4f21-8660-ae50cb3149ca}\0\0” */ \
	'{', 0x00, 'F', 0x00, '4', 0x00, 'E', 0x00, 'F', 0x00, '8', 0x00, '2', 0x00, 'E', 0x00, '0', 0x00, '-', 0x00, \
	'D', 0x00, 'C', 0x00, '0', 0x00, '7', 0x00, '-', 0x00, '4', 0x00, 'F', 0x00, '2', 0x00, '1', 0x00, '-', 0x00, \
	'8', 0x00, '6', 0x00, '6', 0x00, '0', 0x00, '-', 0x00, 'A', 0x00, 'E', 0x00, '5', 0x00, '0', 0x00, 'C', 0x00, \
	'B', 0x00, '3', 0x00, '1', 0x00, '4', 0x00, '9', 0x00, 'C', 0x00, 'A', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00

#define DFU_MS_OS_20_FEATURE_REG_PROPERTY_DEVICE_GUID_DESC_LEN 0x80
#define DFU_MS_OS_20_FEATURE_REG_PROPERTY_DEVICE_GUID_DESC_DATA \
	/* MS OS 2.0 Registry property descriptor: length, type */ \
	U16_TO_U8S_LE(DFU_MS_OS_20_FEATURE_REG_PROPERTY_DEVICE_GUID_DESC_LEN), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY), \
	U16_TO_U8S_LE(0x0001) /* REG_SZ */, U16_TO_U8S_LE(0x0028), /* wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUID\0" in UTF-16 */ \
	'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, \
	'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 0x00, 0x00, \
	U16_TO_U8S_LE(0x004e), /* wPropertyDataLength */ \
	/* bPropertyData: “{f4ef82e0-dc07-4f21-8660-ae50cb3149ca}\0” */ \
	'{', 0x00, 'F', 0x00, '4', 0x00, 'E', 0x00, 'F', 0x00, '8', 0x00, '2', 0x00, 'E', 0x00, '0', 0x00, '-', 0x00, \
	'D', 0x00, 'C', 0x00, '0', 0x00, '7', 0x00, '-', 0x00, '4', 0x00, 'F', 0x00, '2', 0x00, '1', 0x00, '-', 0x00, \
	'8', 0x00, '6', 0x00, '6', 0x00, '0', 0x00, '-', 0x00, 'A', 0x00, 'E', 0x00, '5', 0x00, '0', 0x00, 'C', 0x00, \
	'B', 0x00, '3', 0x00, '1', 0x00, '4', 0x00, '9', 0x00, 'C', 0x00, 'A', 0x00, '}', 0x00, 0x00, 0x00


#define DFU_VENDOR_REQUEST_MICROSOFT 0x01