/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jean Gressmann <jean@0x42.de>
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

#define SC_PACKED __packed
#include <supercan.h>
#include <supercan_m1.h>
#include <usb_descriptors.h>
#include <mcu.h>



static const tusb_desc_device_t device = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x0210,

	// .bDeviceClass       = TUSB_CLASS_MISC,
	// .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
	// .bDeviceProtocol    = MISC_PROTOCOL_IAD,
	.bDeviceClass       = TUSB_CLASS_UNSPECIFIED,
	.bDeviceSubClass    = TUSB_CLASS_UNSPECIFIED,
	.bDeviceProtocol    = TUSB_CLASS_UNSPECIFIED,

	.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

	.idVendor           = 0x4243,
	.idProduct          = 0x0002,
	.bcdDevice          = HWREV << 8,

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
#if CFG_TUD_DFU_RT
#	define DFU_DESC_LEN TUD_DFU_RT_DESC_LEN
#else
#	define DFU_DESC_LEN 0
#endif

#if HWREV == 1
#	define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + (9+4*7) + DFU_DESC_LEN)
#	define DFU_STR_INDEX 5
#	define DFU_INTERFACE_INDEX 1
#else
#	define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + 2*(9+4*7) + DFU_DESC_LEN)
#	define DFU_STR_INDEX 6
#	define DFU_INTERFACE_INDEX 2
#endif

static uint8_t const desc_configuration[] =
{
	// Config number, interface count, string index, total length, attribute, power in mA
	TUD_CONFIG_DESCRIPTOR(1, 3, 0, CONFIG_TOTAL_LEN, 0, 100),

	9, TUSB_DESC_INTERFACE, 0, 0, 4, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 4,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_CMD0_BULK_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_CMD0_BULK_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_MSG0_BULK_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_MSG0_BULK_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
#if HWREV > 1
	9, TUSB_DESC_INTERFACE, 1, 0, 4, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 5,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_CMD1_BULK_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_CMD1_BULK_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_MSG1_BULK_OUT, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
	7, TUSB_DESC_ENDPOINT, SC_M1_EP_MSG1_BULK_IN, TUSB_XFER_BULK, U16_TO_U8S_LE(SC_M1_EP_SIZE), 0,
#endif

#if CFG_TUD_DFU_RT
	9, TUSB_DESC_INTERFACE, DFU_INTERFACE_INDEX, 0, 0, TUD_DFU_APP_CLASS, TUD_DFU_APP_SUBCLASS, DFU_PROTOCOL_RT, DFU_STR_INDEX, \
  	/* Function */
#if 0
	DFU attributes
Bit 7..4: reserved
Bit 3: device will perform a bus
detach-attach sequence when it
receives a DFU_DETACH request.
The host must not issue a USB
Reset. (bitWillDetach)
0 = no
1 = yes
Bit 2: device is able to communicate
via USB after Manifestation phase.
(bitManifestationTolerant)
0 = no, must see bus reset
1 = yes
Bit 1: upload capable (bitCanUpload)
0 = no
1 = yes
Bit 0: download capable
(bitCanDnload)
0 = no
1 = yes
#endif
	9,
	DFU_DESC_FUNCTIONAL,
	(0<<3) | (1<<2) | (0<<1) | (1<<0) /*attrs*/,
	U16_TO_U8S_LE(DFU_USB_RESET_TIMEOUT_MS) /* timeout [ms]*/,
	U16_TO_U8S_LE(MCU_NVM_PAGE_SIZE)/* xfer size*/,
	U16_TO_U8S_LE(0x0101)/*bcdVersion*/,
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


#if HWREV == 1
#	define MS_OS_20_DESC_LEN (0x0A+0x08 + 2*(0x08+0x14) + 0x84)
#else
#	define MS_OS_20_DESC_LEN (0x0A+0x08 + 3*(0x08+0x14) + 2*0x84)
#endif
uint8_t const desc_ms_os_20[] =
{
	// Set header: length, type, windows version, total length
	U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

	// Configuration subset header: length, type, configuration index, reserved, configuration total length
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A),

	// Function Subset header: length, type, first interface, reserved, subset length
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), 0, 0, U16_TO_U8S_LE(0x08 + 0x14 + 0x84),

	// MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
	U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
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
#if HWREV > 1
	// Function Subset header: length, type, first interface, reserved, subset length
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), 1, 0, U16_TO_U8S_LE(0x08 + 0x14 + 0x84),

	// MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
	U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
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
#endif
	// Function Subset header: length, type, first interface, reserved, subset length
	U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), DFU_INTERFACE_INDEX, 0, U16_TO_U8S_LE(0x08 + 0x14),

	// MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
	U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible

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
	(const char[]) { 0x09, 0x04 },   // 0: is supported language is English (0x0409)
	"2guys",                         // 1: Manufacturer
	BOARD_NAME " " SC_NAME,          // 2: Product
	"",                        		 // 3: Serial
	SC_NAME " (ch0)",
#if HWREV > 1
	SC_NAME " (ch1)",
#endif
#if CFG_TUD_DFU_RT
	"USB DFU 1.1",
#endif
};


static uint16_t _desc_str[33];
static const char hex_map[16] = "0123456789abcdef";

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	(void) langid;

	uint8_t chr_count;
	switch (index) {
	case 0:
		memcpy(&_desc_str[1], string_desc_arr[0], 2);
		chr_count = 1;
		break;
	case 3: {
		// Section 9.6, p. 60
		chr_count = 32;
		uint32_t serial_number[4];
		same51_get_serial_number(serial_number);
		TU_LOG2(
			"chip serial number %08lx%08lx%08lx%08lx\n",
			serial_number[0],
			serial_number[1],
			serial_number[2],
			serial_number[3]);
		for (unsigned i = 0, k = 1; i < 4; ++i) {
			uint32_t x = serial_number[i];
			for (unsigned j = 0, shift = 24; j < 4; ++j, shift -= 8, k += 2) {
				uint8_t y = (x >> shift) & 0xff;
				uint8_t hi = (y >> 4) & 0xf;
				uint8_t lo = (y >> 0) & 0xf;
				_desc_str[k] = hex_map[hi];
				_desc_str[k+1] = hex_map[lo];
			}
		}
	} break;
	default: {
		// Convert ASCII string into UTF-16
		if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

		const char* str = string_desc_arr[index];

		// Cap at max char
		chr_count = tu_min8(strlen(str), TU_ARRAY_SIZE(_desc_str) - 1);

		for (uint8_t i = 0; i < chr_count; ++i) {
			_desc_str[1+i] = str[i];
		}
	} break;
	}

	// first byte is length (including header), second byte is string type
	_desc_str[0] = (TUSB_DESC_STRING << 8) | (2*chr_count + 2);

	return _desc_str;
}
