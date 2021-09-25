/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020-2021 Jean Gressmann <jean@0x42.de>
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

#include <tusb.h>
#include <class/dfu/dfu_rt_device.h>

#define SC_PACKED __packed
#include <supercan.h>
#include <supercan_m1.h>
#include <supercan_debug.h>
#include <supercan_version.h>
#include <usb_descriptors.h>
#include <mcu.h>
#include <device.h>
#include <usnprintf.h>




static const tusb_desc_device_t device = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x0210,

	.bDeviceClass       = TUSB_CLASS_UNSPECIFIED,
	.bDeviceSubClass    = TUSB_CLASS_UNSPECIFIED,
	.bDeviceProtocol    = TUSB_CLASS_UNSPECIFIED,

	.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

	.idVendor           = 0x1d50,
	.idProduct          = 0x5035,
	//.bcdDevice          = (HWREV << 12) | (SUPERCAN_VERSION_MAJOR << 8) | (SUPERCAN_VERSION_MINOR << 4) | (SUPERCAN_VERSION_PATCH),
	.bcdDevice          = (SUPERCAN_VERSION_MAJOR << 12) | (SUPERCAN_VERSION_MINOR << 8) | ((SUPERCAN_VERSION_PATCH / 10) << 4) | (SUPERCAN_VERSION_PATCH % 10),
	//.bcdDevice          = HWREV << 8,

	.iManufacturer      = 0x01,
	.iProduct           = 0x02,
	.iSerialNumber      = 0x03,

	.bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
	return (uint8_t const *) &device;
}


//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
#if CFG_TUD_DFU_RUNTIME
#	define DFU_DESC_LEN TUD_DFU_RT_DESC_LEN
#	define DFU_INTERFACE_COUNT 1
#else
#	define DFU_DESC_LEN 0
#	define DFU_INTERFACE_COUNT 0
#endif

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + 2*(9+4*7) + DFU_DESC_LEN)
#define DFU_STR_INDEX 6
#define DFU_INTERFACE_INDEX 2
#define INTERFACE_COUNT (2 + DFU_INTERFACE_COUNT)

static uint8_t const desc_configuration[] =
{
	// Config number, interface count, string index, total length, attribute, power in mA
	TUD_CONFIG_DESCRIPTOR(1, INTERFACE_COUNT, 0, CONFIG_TOTAL_LEN, 0, 250),

	9, TUSB_DESC_INTERFACE, 0, 0, 4, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 4,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_CMD0_BULK_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_CMD0_BULK_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_MSG0_BULK_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_MSG0_BULK_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	9, TUSB_DESC_INTERFACE, 1, 0, 4, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 5,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_CMD1_BULK_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_CMD1_BULK_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_MSG1_BULK_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_MSG1_BULK_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,

#if CFG_TUD_DFU_RUNTIME
	// Interface number, string index, attributes, detach timeout, transfer size */
	TUD_DFU_RT_DESCRIPTOR(DFU_INTERFACE_INDEX, DFU_STR_INDEX, (DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_MANIFESTATION_TOLERANT), DFU_USB_RESET_TIMEOUT_MS, MCU_NVM_PAGE_SIZE),
#endif
};


TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN, "descriptor size mismatch");

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
	(void) index; // for multiple configurations
	return desc_configuration;
}

// MS_OS_20_SET_HEADER_DESCRIPTOR	0x00
// MS_OS_20_SUBSET_HEADER_CONFIGURATION	0x01
// MS_OS_20_SUBSET_HEADER_FUNCTION	0x02
// MS_OS_20_FEATURE_COMPATBLE_ID	0x03
// MS_OS_20_FEATURE_REG_PROPERTY	0x04
// MS_OS_20_FEATURE_MIN_RESUME_TIME	0x05
// MS_OS_20_FEATURE_MODEL_ID	0x06
// MS_OS_20_FEATURE_CCGP_DEVICE	0x07
// MS_OS_20_FEATURE_VENDOR_REVISION	0x08

#define MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN 0x14
#define MS_OS_20_FEATURE_MODEL_ID_DESC_LEN 0x14

#if CFG_TUD_DFU_RUNTIME
#	define DFU_MS_OS_20_DESC_LEN (0x08+MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN)
#else
#	define DFU_MS_OS_20_DESC_LEN 0
#endif

#define MS_OS_20_DESC_LEN (0x0A+0x08 + 2 * (0x08+MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN+0x84) + DFU_MS_OS_20_DESC_LEN)

uint8_t const desc_ms_os_20[] =
{
	// Set header: length, type, windows version, total length
	U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

	// U16_TO_U8S_LE(MS_OS_20_FEATURE_MODEL_ID_DESC_LEN), U16_TO_U8S_LE(MS_OS_20_FEATURE_MODEL_ID), 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,


	// Configuration subset header: length, type, configuration index, reserved, configuration total length
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A),

	// Function Subset header: length, type, first interface, reserved, subset length
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), 0, 0, U16_TO_U8S_LE(0x08 + MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN + 0x84),

	// MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
	U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible

	// MS OS 2.0 Registry property descriptor: length, type
	U16_TO_U8S_LE(0x0084), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
	U16_TO_U8S_LE(0x0007) /* REG_MULTI_SZ */, U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
	'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
	'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00,
	0x00, 0x00,
	U16_TO_U8S_LE(0x0050), // wPropertyDataLength
	//bPropertyData: “{f4ef82e0-dc07-4f21-8660-ae50cb3149c9}”.
	'{', 0x00, 'F', 0x00, '4', 0x00, 'E', 0x00, 'F', 0x00, '8', 0x00, '2', 0x00, 'E', 0x00, '0', 0x00, '-', 0x00,
	'D', 0x00, 'C', 0x00, '0', 0x00, '7', 0x00, '-', 0x00, '4', 0x00, 'F', 0x00, '2', 0x00, '1', 0x00, '-', 0x00,
	'8', 0x00, '6', 0x00, '6', 0x00, '0', 0x00, '-', 0x00, 'A', 0x00, 'E', 0x00, '5', 0x00, '0', 0x00, 'C', 0x00,
	'B', 0x00, '3', 0x00, '1', 0x00, '4', 0x00, '9', 0x00, 'C', 0x00, '9', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00,


	// Function Subset header: length, type, first interface, reserved, subset length
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), 1, 0, U16_TO_U8S_LE(0x08 + MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN + 0x84),

	// MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
	U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible

	// MS OS 2.0 Registry property descriptor: length, type
	U16_TO_U8S_LE(0x0084), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
	U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
	'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
	'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
	U16_TO_U8S_LE(0x0050), // wPropertyDataLength
	//bPropertyData: “{f4ef82e0-dc07-4f21-8660-ae50cb3149c9}”.
	'{', 0x00, 'F', 0x00, '4', 0x00, 'E', 0x00, 'F', 0x00, '8', 0x00, '2', 0x00, 'E', 0x00, '0', 0x00, '-', 0x00,
	'D', 0x00, 'C', 0x00, '0', 0x00, '7', 0x00, '-', 0x00, '4', 0x00, 'F', 0x00, '2', 0x00, '1', 0x00, '-', 0x00,
	'8', 0x00, '6', 0x00, '6', 0x00, '0', 0x00, '-', 0x00, 'A', 0x00, 'E', 0x00, '5', 0x00, '0', 0x00, 'C', 0x00,
	'B', 0x00, '3', 0x00, '1', 0x00, '4', 0x00, '9', 0x00, 'C', 0x00, '9', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00,

#if CFG_TUD_DFU_RUNTIME
	// Function Subset header: length, type, first interface, reserved, subset length
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), DFU_INTERFACE_INDEX, 0, U16_TO_U8S_LE(0x08 + MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN),

	// MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
	U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible
#endif
};


TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "descriptor size mismatch");



#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
static uint8_t const bos_desc[] =
{
	// total length, number of device caps
	TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

	// Microsoft OS 2.0 descriptor
	TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT)
};

uint8_t const * tud_descriptor_bos_cb(void)
{
	return bos_desc;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
static char const* string_desc_arr [] =
{
	(const char[]) { 0x09, 0x04 },                          // 0: is supported language is English (0x0409)
	"2guys",                                                // 1: Manufacturer
	BOARD_NAME " " SC_NAME " (%s)",                         // 2: Product
	"%s",                                                   // 3: Serial
	BOARD_NAME " " SC_NAME " (%s) CAN ch0",
	BOARD_NAME " " SC_NAME " (%s) CAN ch1",
#if CFG_TUD_DFU_RUNTIME
	BOARD_NAME " " SC_NAME " (%s) DFU",
#endif
};

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	static uint16_t wstring_buffer[64];

	uint8_t chr_count = 0;

	(void) langid;

	if (index >= TU_ARRAY_SIZE(string_desc_arr) || !string_desc_arr[index]) {
		return NULL;
	}

	switch (index) {
	case 0:
		memcpy(&wstring_buffer[1], string_desc_arr[0], 2);
		chr_count = 1;
		break;
	default: {
		char string_buffer[TU_ARRAY_SIZE(wstring_buffer)-1];
		char zero_padded[32]; // 0-pad serial in case it has leading zeros
		int serial_chars = 0;
		int chars = 0;

		memset(zero_padded, '0', 8);
		serial_chars = usnprintf(&zero_padded[8], 24, "%x", device_identifier);
		SC_DEBUG_ASSERT(serial_chars >= 0 && serial_chars <= 8);

		chars = usnprintf(string_buffer, sizeof(string_buffer), string_desc_arr[index], &zero_padded[serial_chars]);

		// convert to UTF-16
		for (int i = 0; i < chars; ++i) {
			wstring_buffer[i+1] = string_buffer[i];
		}

		chr_count = (uint8_t)chars;
	} break;
	}

	SC_DEBUG_ASSERT((size_t)(chr_count + 1) <= TU_ARRAY_SIZE(wstring_buffer));

	// first byte is length (including header), second byte is string type
	wstring_buffer[0] = (TUSB_DESC_STRING << 8) | ((chr_count + 1) << 1);

	return wstring_buffer;
}
