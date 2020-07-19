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

#include <usb_descriptors.h>
#include <sam.h>
#include <mcu.h>


static const tusb_desc_device_t device = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x0100,

	.bDeviceClass       = TUSB_CLASS_UNSPECIFIED,
	.bDeviceSubClass    = 0x00,
	.bDeviceProtocol    = 0x00,

	.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

	.idVendor           = 0x4243,
	.idProduct          = 0x0001,
	.bcdDevice          = 0x0001,

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



#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN)

static uint8_t const desc_configuration[] =
{
	// Config number, interface count, string index, total length, attribute, power in mA
	TUD_CONFIG_DESCRIPTOR(1, 0, 0, CONFIG_TOTAL_LEN, 0, 100),


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
	(const char[]) { 0x09, 0x04 },   // 0: is supported language is English (0x0409)
	"Jean Gressmann",                // 1: Manufacturer
	"Blinky",	                     // 2: Product
	"",                              // 3: Serial
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
