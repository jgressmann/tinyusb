/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
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



static const tusb_desc_device_t device = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
#if CFG_TUD_DFU_RUNTIME
	.bcdUSB             = 0x0210,
#else
	.bcdUSB             = 0x0200,
#endif

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

#if CFG_TUD_DFU_RUNTIME

#define MS_OS_20_SET_HEADER_DESCRIPTOR_LEN 0x0a
#define MS_OS_20_SUBSET_HEADER_CONFIGURATION_LEN 0x08
#define DFU_MS_OS_20_DESC_LEN (DFU_MS_OS_20_SUBSET_HEADER_FUNCTION_LEN+DFU_MS_OS_20_FEATURE_COMPATBLE_ID_DESC_LEN+DFU_MS_OS_20_FEATURE_REG_PROPERTY_DEVICE_GUIDS_DESC_LEN)
#define MS_OS_20_DESC_LEN (MS_OS_20_SET_HEADER_DESCRIPTOR_LEN + 4 + MS_OS_20_SUBSET_HEADER_CONFIGURATION_LEN + DFU_MS_OS_20_DESC_LEN)


uint8_t const desc_ms_os_20[] =
{
	// Set header: length, type, windows version, total length
	U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR_LEN), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

	// treat as composite
	U16_TO_U8S_LE(0x0004), U16_TO_U8S_LE(MS_OS_20_FEATURE_CCGP_DEVICE),

	// Configuration subset header: length, type, configuration index, reserved, configuration total length
	U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION_LEN), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - MS_OS_20_SET_HEADER_DESCRIPTOR_LEN - 4),

	DFU_MS_OS_20_SUBSET_HEADER_FUNCTION_DATA(DFU_INTERFACE_INDEX, DFU_MS_OS_20_DESC_LEN),
	DFU_MS_OS_20_FEATURE_COMPATBLE_ID_DESC_DATA,
	DFU_MS_OS_20_FEATURE_REG_PROPERTY_DEVICE_GUIDS_DESC_DATA,
};


TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "descriptor size mismatch");



#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
static uint8_t const bos_desc[] =
{
	// total length, number of device caps
	TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

	// Microsoft OS 2.0 descriptor
	TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, DFU_VENDOR_REQUEST_MICROSOFT)
};

uint8_t const * tud_descriptor_bos_cb(void)
{
	return bos_desc;
}
#endif

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
