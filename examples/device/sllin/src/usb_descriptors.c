/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Jean Gressmann <jean@0x42.de>
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

#include <sllin_board.h>
#include <usb_descriptors.h>
#include <usnprintf.h>

#ifndef SLLIN_BOARD_LIN_COUNT
#	error "Define SLLIN_BOARD_LIN_COUNT!"
#endif

#if SLLIN_BOARD_LIN_COUNT > 2
#	error "Only 2 LIN interfaces supported"
#endif

#if SLLIN_BOARD_LIN_COUNT < 0
#	error "SLLIN_BOARD_LIN_COUNT must be positive"
#endif



#define USB_BCD 0x200

static const tusb_desc_device_t device = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = USB_BCD,

	.bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

	.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

	.idVendor           = 0x1d50,
	.idProduct          = 0x5037,
	.bcdDevice          = (SLLIN_VERSION_MAJOR << 12) | (SLLIN_VERSION_MINOR << 8) | ((SLLIN_VERSION_PATCH / 10) << 4) | (SLLIN_VERSION_PATCH % 10),

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

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + (SLLIN_BOARD_LIN_COUNT) * TUD_CDC_DESC_LEN + DFU_DESC_LEN)
#define DFU_STR_INDEX (4 + (SLLIN_BOARD_LIN_COUNT))
#define DFU_INTERFACE_INDEX (SLLIN_BOARD_LIN_COUNT*2)
#define INTERFACE_COUNT ((SLLIN_BOARD_LIN_COUNT*2) + DFU_INTERFACE_COUNT)

static uint8_t const desc_configuration[] =
{
	// Config number, interface count, string index, total length, attribute, power in mA
	TUD_CONFIG_DESCRIPTOR(1, INTERFACE_COUNT, 0, CONFIG_TOTAL_LEN, 0, 100),

	TUD_CDC_DESCRIPTOR(0, 4, 0x81, 8, 0x02, 0x82, 64),

#if SLLIN_BOARD_LIN_COUNT > 1
	TUD_CDC_DESCRIPTOR(2, 5, 0x83, 8, 0x04, 0x84, 64),
#endif

#if CFG_TUD_DFU_RUNTIME
	// Interface number, string index, attributes, detach timeout, transfer size */
	TUD_DFU_RT_DESCRIPTOR(DFU_INTERFACE_INDEX, DFU_STR_INDEX, (DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_MANIFESTATION_TOLERANT), DFU_USB_RESET_TIMEOUT_MS, MCU_NVM_PAGE_SIZE),
#endif
};


TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN, "descriptor size mismatch");

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
	(const char[]) { 0x09, 0x04 },                          // 0: is supported language is English (0x0409)
	SLLIN_BOARD_USB_MANUFACTURER_STRING,                    // 1: Manufacturer
	SLLIN_BOARD_NAME " " SLLIN_NAME " (%s)",                // 2: Product
	"%s",                                                   // 3: Serial
	SLLIN_BOARD_NAME " " SLLIN_NAME " (%s) LIN ch0",
#if SLLIN_BOARD_LIN_COUNT > 1
	SLLIN_BOARD_NAME " " SLLIN_NAME " (%s) LIN ch1",
#endif
#if CFG_TUD_DFU_RUNTIME
	SLLIN_BOARD_NAME " " SLLIN_NAME " (%s) DFU",
#endif
};

void usb_get_desc_string(uint8_t index, char *ptr, size_t* in_out_capa_len)
{
	char zero_padded[32]; // 0-pad serial in case it has leading zeros
	int serial_chars = 0;
	size_t capacity;

	SLLIN_DEBUG_ASSERT(ptr);
	SLLIN_DEBUG_ASSERT(in_out_capa_len);

	capacity = *in_out_capa_len;

	memset(zero_padded, '0', 8);
	serial_chars = usnprintf(&zero_padded[8], 24, "%x", sllin_board_identifier());
	SLLIN_DEBUG_ASSERT(serial_chars >= 0 && serial_chars <= 8);

	*in_out_capa_len = usnprintf(ptr, capacity, string_desc_arr[index], &zero_padded[serial_chars]);
}

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
		size_t capa_len = TU_ARRAY_SIZE(wstring_buffer) / 2;
		char *string_buffer = (char *)&wstring_buffer[TU_ARRAY_SIZE(wstring_buffer) - capa_len];

		usb_get_desc_string(index, string_buffer, &capa_len);

		// convert to UTF-16
		for (size_t i = 0; i < capa_len; ++i) {
			wstring_buffer[i+1] = string_buffer[i];
		}

		chr_count = (uint8_t)capa_len;
	} break;
	}

	SLLIN_DEBUG_ASSERT((size_t)(chr_count + 1) <= TU_ARRAY_SIZE(wstring_buffer));

	// first byte is length (including header), second byte is string type
	wstring_buffer[0] = (TUSB_DESC_STRING << 8) | ((chr_count + 1) << 1);

	return wstring_buffer;
}
