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

#include <peak_usb_fd.h>

#define HIGH_SPEED_BULK_EP_SIZE 512
#define FULL_SPEED_BULK_EP_SIZE 64


static const tusb_desc_device_t device = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x0200,

	.bDeviceClass       = TUSB_CLASS_UNSPECIFIED,
	.bDeviceSubClass    = 0,
	.bDeviceProtocol    = 0,

	.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

	.idVendor           = 0x0c72,
	.idProduct          = 0x0012,
	.bcdDevice          = 0x0000,

	.iManufacturer      = 0x01,
	.iProduct           = 0x02,
	.iSerialNumber      = 0x00,

	.bNumConfigurations = 0x01
};

static const tusb_desc_device_qualifier_t qualifier = {
  .bLength            = sizeof(tusb_desc_device_qualifier_t),
  .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = TUSB_CLASS_UNSPECIFIED,
  .bDeviceSubClass    = 0,
  .bDeviceProtocol    = 0,
  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .bNumConfigurations = 1,
  .bReserved          = 0
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
	return (uint8_t const *) &device;
}

// uint8_t const * tud_descriptor_device_qualifier_cb(void)
// {
// 	return (uint8_t const *) &qualifier;
// }


//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+


// v3 no error linux
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + 1*9 + 6*7)
static uint8_t const desc_configuration[] =
{
	// Config number, interface count, string index, total length, attribute, power in mA
	TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0, 170),

	9, TUSB_DESC_INTERFACE, 0, 0, 6, TUSB_CLASS_UNSPECIFIED, 0x00, 0x00, 0,

	/* BULK OUT CMD */
  	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_OUT_CMD, TUSB_XFER_BULK, U16_TO_U8S_LE(FULL_SPEED_BULK_EP_SIZE), 0,
	/* BULK IN CMD */
  	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_IN_CMD, TUSB_XFER_BULK, U16_TO_U8S_LE(FULL_SPEED_BULK_EP_SIZE), 0,
	/* BULK OUT MSG CH0 */
  	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_OUT_MSG_CH0, TUSB_XFER_BULK, U16_TO_U8S_LE(FULL_SPEED_BULK_EP_SIZE), 0,
	/* INT IN MSG CH0 */
  	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_IN_MSG_CH0, TUSB_XFER_BULK, U16_TO_U8S_LE(FULL_SPEED_BULK_EP_SIZE), 0,
	/* BULK OUT MSG CH1 */
  	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_OUT_MSG_CH1, TUSB_XFER_BULK, U16_TO_U8S_LE(FULL_SPEED_BULK_EP_SIZE), 0,
	/* INT IN MSG CH1 */
  	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_IN_MSG_CH1, TUSB_XFER_BULK, U16_TO_U8S_LE(FULL_SPEED_BULK_EP_SIZE), 0,

};


// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
	(void) index; // for multiple configurations
	return desc_configuration;
}

// #define OTHER_SPEED_CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + 1*9 + 4*7)
// static uint8_t const other_speed_configuration[] =
// {
// 	// Config number, interface count, string index, total length, attribute, power in mA
// 	TUD_OTHER_SPEED_CONFIG_DESCRIPTOR(1, 1, 0, OTHER_SPEED_CONFIG_TOTAL_LEN, 0, 170),

// 	9, TUSB_DESC_INTERFACE, 0, 0, 4, TUSB_CLASS_UNSPECIFIED, 0x00, 0x00, 0,

// 	/* BULK OUT CMD */
//   	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_OUT_CMD, TUSB_XFER_BULK, U16_TO_U8S_LE(HIGH_SPEED_BULK_EP_SIZE), 0,
// 	/* BULK IN CMD */
//   	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_IN_CMD, TUSB_XFER_BULK, U16_TO_U8S_LE(HIGH_SPEED_BULK_EP_SIZE), 0,
// 	/* BULK OUT MSG */
//   	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_OUT_MSG, TUSB_XFER_BULK, U16_TO_U8S_LE(HIGH_SPEED_BULK_EP_SIZE), 0,
// 	/* INT IN MSG */
//   	7, TUSB_DESC_ENDPOINT, PEAK_USB_FD_EP_BULK_IN_MSG, TUSB_XFER_BULK, U16_TO_U8S_LE(HIGH_SPEED_BULK_EP_SIZE), 0,


// };

// uint8_t const * tud_descriptor_other_speed_configuration_cb(uint8_t index)
// {
// 	(void) index; // for multiple configurations
// 	return other_speed_configuration;
// }


//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr [] =
{
	(const char[]) { 0x09, 0x04 }, 		// 0: is supported language is English (0x0409)
	//  iManufacturer           1 PEAK-System Technik GmbH
//  iProduct                2 PCAN-USB FD

	"J. Gressmann, R. Riedel",       	// 1: Manufacturer
	"PCAN-USB FD PRO",				 		// 2: Product
	// "PEAK-USB-FD-CAN Device",			// 3: Interface
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	(void) langid;

	uint8_t chr_count;

	if ( index == 0) {
	  memcpy(&_desc_str[1], string_desc_arr[0], 2);
	  chr_count = 1;
	} else {
	  // Convert ASCII string into UTF-16

	  if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

	  const char* str = string_desc_arr[index];

	  // Cap at max char
	  chr_count = strlen(str);
	  if ( chr_count > 31 ) chr_count = 31;

	  for(uint8_t i=0; i<chr_count; i++)
	  {
	    _desc_str[1+i] = str[i];
	  }
	}

	// first byte is length (including header), second byte is string type
	_desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

	return _desc_str;
}
