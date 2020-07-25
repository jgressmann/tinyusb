#include <string.h>
#include <tusb.h>
#include <usb_descriptors.h>

#if defined(CFG_TUD_VENDOR_CUSTOM) && CFG_TUD_VENDOR_CUSTOM


static inline const char* recipient_str(tusb_request_recipient_t r)
{
	switch (r) {
	case TUSB_REQ_RCPT_DEVICE:
		return "device (0)";
	case TUSB_REQ_RCPT_INTERFACE:
		return "interface (1)";
	case TUSB_REQ_RCPT_ENDPOINT:
		return "endpoint (2)";
	case TUSB_REQ_RCPT_OTHER:
		return "other (3)";
	default:
		return "???";
	}
}

static inline const char* type_str(tusb_request_type_t value)
{
	switch (value) {
	case TUSB_REQ_TYPE_STANDARD:
		return "standard (0)";
	case TUSB_REQ_TYPE_CLASS:
		return "class (1)";
	case TUSB_REQ_TYPE_VENDOR:
		return "vendor (2)";
	case TUSB_REQ_TYPE_INVALID:
		return "invalid (3)";
	default:
		return "???";
	}
}

static inline const char* dir_str(tusb_dir_t value)
{
	switch (value) {
	case TUSB_DIR_OUT:
		return "out (0)";
	case TUSB_DIR_IN:
		return "in (1)";
	default:
		return "???";
	}
}

void vendord_init(void)
{

}

void vendord_reset(uint8_t rhport)
{
	TU_LOG2("vendord_reset\n");
}

bool vendord_open(uint8_t rhport, tusb_desc_interface_t const * desc_intf, uint16_t *p_len)
{
	TU_LOG2("vendord_open\n");
	(void) rhport;

	// this function get's called for _non_ vendor interfaces
	TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == desc_intf->bInterfaceClass);
	// TU_VERIFY(itf_desc->bInterfaceSubClass == TUSB_CLASS_VENDOR_SPECIFIC);
	// TU_VERIFY(itf_desc->bInterfaceProtocol == DFU_PROTOCOL_RT);

	// uint8_t const * p_desc = tu_desc_next(itf_desc);
	// (*p_len) = sizeof(tusb_desc_interface_t);

	// if (TUSB_DESC_FUNCTIONAL == tu_desc_type(p_desc)) {
	// 	(*p_length) += p_desc[DESC_OFFSET_LEN];
	// 	p_desc = tu_desc_next(p_desc);
	// }
	*p_len = 9;

	return true;
}

bool vendord_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
	TU_LOG2("vendord_xfer_cb\n");
	return false;
}


bool tud_vendor_control_request_cb(uint8_t rhport, tusb_control_request_t const * request)
{
	// TU_LOG2("port %u req\n", rhport);

	TU_LOG2("req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
		request->bmRequestType,
		recipient_str(request->bmRequestType_bit.recipient),
		type_str(request->bmRequestType_bit.type),
		dir_str(request->bmRequestType_bit.direction),
		request->bRequest, request->wValue, request->wIndex,
		request->wLength);

	switch (request->bRequest) {
	case VENDOR_REQUEST_MICROSOFT:
		if (request->wIndex == 7) {
			// Get Microsoft OS 2.0 compatible descriptor
			uint16_t total_len;
			memcpy(&total_len, desc_ms_os_20+8, 2);
			return tud_control_xfer(rhport, request, (void*)desc_ms_os_20, total_len);
		}
		break;
	default:
		break;
	}

	// stall unknown request
	return false;
}

bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const *request)
{
	(void) rhport;
	(void) request;

	return true;
}


#endif