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

#include <usb_descriptors.h>
#include <sam.h>
#include <mcu.h>
#include <version.h>

#ifndef VID
#	error Define VID
#endif
#ifndef PID
#	error Define PID
#endif
#ifndef PRODUCT_NAME
#	error Define PRODUCT_NAME
#endif
#ifndef INTERFACE_NAME
#	error Define INTERFACE_NAME
#endif


static const tusb_desc_device_t device = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x0100,

	.bDeviceClass       = TUSB_CLASS_UNSPECIFIED,
	.bDeviceSubClass    = 0x00,
	.bDeviceProtocol    = 0x00,

	.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

	.idVendor           = VID,
	.idProduct          = PID,
	// .bcdDevice          = HWREV << 8,
	.bcdDevice          = (SUPERDFU_VERSION_MAJOR << 12) | (SUPERDFU_VERSION_MINOR << 8) | ((SUPERDFU_VERSION_PATCH / 10) << 4) | (SUPERDFU_VERSION_PATCH % 10),

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

#define ALT_COUNT   1

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_DFU_DESC_LEN(ALT_COUNT))

static uint8_t const desc_configuration[] =
{
	// Config number, interface count, string index, total length, attribute, power in mA
	TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0, 100),

#if 0
	9, TUSB_DESC_INTERFACE, 0, 0, 0, TUD_DFU_APP_CLASS, TUD_DFU_APP_SUBCLASS, DFU_PROTOCOL_DFU, 4, \
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
		(DFU_WILL_DETACH<<3) | (DFU_MANIFESTATION_TOLERANT<<2) | (0<<1) | (1<<0) /*attrs*/,
		U16_TO_U8S_LE(DFU_USB_TIMEOUT_MS) /* timeout [ms]*/,
		U16_TO_U8S_LE(MCU_NVM_PAGE_SIZE)/* xfer size*/,
		U16_TO_U8S_LE(0x0101)/*bcdVersion*/
#endif
	// Interface number, Alternate count, starting string index, attributes, detach timeout, transfer size
	TUD_DFU_DESCRIPTOR(0, ALT_COUNT, 4, (DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_MANIFESTATION_TOLERANT), 1000, CFG_TUD_DFU_XFER_BUFSIZE),
};


uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
	(void) index; // for multiple configurations
	return desc_configuration;
}


//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
static char const* string_desc_arr [] =
{
	(const char[]) { 0x09, 0x04 },            // 0: is supported language is English (0x0409)
	"Jean Gressmann",                         // 1: Manufacturer
	PRODUCT_NAME,                             // 2: Product
	"",                                       // 3: Serial
	INTERFACE_NAME,
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
	case 4: {
		const uint8_t BANK_LEN = 7;
		uint8_t i = 0;
		const char* str = string_desc_arr[index];

		chr_count = tu_min8(strlen(str), TU_ARRAY_SIZE(_desc_str) - 1 - BANK_LEN);

		for (; i < chr_count; ++i) {
			_desc_str[1+i] = str[i];
		}

		_desc_str[1+i++] = ' ';
		_desc_str[1+i++] = 'B';
		_desc_str[1+i++] = 'a';
		_desc_str[1+i++] = 'n';
		_desc_str[1+i++] = 'k';
		_desc_str[1+i++] = ' ';
		_desc_str[1+i++] = '0' + mcu_nvm_boot_bank_index();

		chr_count += BANK_LEN;
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
