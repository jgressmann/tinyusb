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

#include <string.h>
#include <inttypes.h>


#include <bsp/board.h>
#include <tusb.h>

#include <sam.h>
#include <hal/include/hal_gpio.h>

#include <usb_descriptors.h>
#include <mcu.h>
#include <usb_dfu_1_1.h>
#include <dfu_ram.h>
#include <dfu_app.h>
#include <dfu_debug.h>


static void led_task(void);

#if TU_BIG_ENDIAN == TU_BYTE_ORDER
static inline uint16_t le16_to_cpu(uint16_t value) { return __builtin_bswap16(value); }
static inline uint32_t le32_to_cpu(uint32_t value) { return __builtin_bswap32(value); }
static inline uint16_t cpu_to_le16(uint16_t value) { return __builtin_bswap16(value); }
static inline uint32_t cpu_to_le32(uint32_t value) { return __builtin_bswap32(value); }
static inline uint16_t be16_to_cpu(uint16_t value) { return value; }
static inline uint32_t be32_to_cpu(uint32_t value) { return value; }
static inline uint16_t cpu_to_be16(uint16_t value) { return value; }
static inline uint32_t cpu_to_be32(uint32_t value) { return value; }
#else
static inline uint16_t le16_to_cpu(uint16_t value) { return value; }
static inline uint32_t le32_to_cpu(uint32_t value) { return value; }
static inline uint16_t cpu_to_le16(uint16_t value) { return value; }
static inline uint32_t cpu_to_le32(uint32_t value) { return value; }
static inline uint16_t be16_to_cpu(uint16_t value) { return __builtin_bswap16(value); }
static inline uint32_t be32_to_cpu(uint32_t value) { return __builtin_bswap32(value); }
static inline uint16_t cpu_to_be16(uint16_t value) { return __builtin_bswap16(value); }
static inline uint32_t cpu_to_be32(uint32_t value) { return __builtin_bswap32(value); }
#endif

#ifndef likely
#define likely(x) __builtin_expect(!!(x),1)
#endif

#ifndef unlikely
#define unlikely(x) __builtin_expect(!!(x),0)
#endif



#define NVM_BOOTLOADER_BLOCKS (MCU_BOOTLOADER_SIZE / MCU_NVM_BLOCK_SIZE)
#define NVM_PROG_BLOCKS (MCU_NVM_SIZE / 2 - NVM_BOOTLOADER_BLOCKS)





static struct dfu {
	struct dfu_get_status_reply status;
	uint32_t download_size;
	uint32_t prog_offset;
	uint16_t cleared_pages_left;
	uint32_t page_buffer[MCU_NVM_PAGE_SIZE / 4];
} dfu;

struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_HDR_SECTION_NAME)));

static inline bool nvm_erase_block(void *addr)
{
	LOG("erase block %p\n", addr);

	// while (!NVMCTRL->STATUS.bit.READY); // wait for nvmctrl to be ready

	NVMCTRL->ADDR.reg = NVMCTRL_ADDR_ADDR((uintptr_t)addr);

	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY;

	while (!NVMCTRL->STATUS.bit.READY);

	// clear done flag
	NVMCTRL->INTFLAG.reg = NVMCTRL_INTFLAG_DONE;

	// LOG("INTFLAG %#08lx\n", (uint32_t)NVMCTRL->INTFLAG.reg);

	// check for errors
	if (unlikely(NVMCTRL->INTFLAG.reg)) {
		return false;
	}

	return true;
}

static inline bool nvm_write_main_page(void *addr, void const *ptr)
{
	LOG("write main page @ %p\n", addr);

	// while (!NVMCTRL->STATUS.bit.READY); // wait for nvmctrl to be ready

	// // clear flags
	// NVMCTRL->INTFLAG.reg = NVMCTRL->INTFLAG.reg;

	// // clear page buffer
	// NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_PBC | NVMCTRL_CTRLB_CMDEX_KEY;
	// 	// if (!NVMCTRL->STATUS.bit.READY) {
	// 	// 	LOG("erasing page buffer\n");
	// 	// }
	// // wait for erase
	// while (!NVMCTRL->STATUS.bit.READY);

	// // clear done flag
	// NVMCTRL->INTFLAG.bit.DONE = 1;

	// // LOG("INTFLAG %#08lx\n", (uint32_t)NVMCTRL->INTFLAG.reg);

	// // check for errors
	// if (unlikely(NVMCTRL->INTFLAG.reg)) {
	// 	return false;
	// }

	memcpy(addr, ptr, MCU_NVM_PAGE_SIZE);

	NVMCTRL->ADDR.reg = NVMCTRL_ADDR_ADDR((uintptr_t)addr);

	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_WP | NVMCTRL_CTRLB_CMDEX_KEY;

	while (!NVMCTRL->STATUS.bit.READY);

	// clear done flag
	NVMCTRL->INTFLAG.reg = NVMCTRL_INTFLAG_DONE;

	// LOG("INTFLAG %#08lx\n", (uint32_t)NVMCTRL->INTFLAG.reg);

	// check for errors
	if (unlikely(NVMCTRL->INTFLAG.reg)) {
		return false;
	}

	return true;
}

static void start_app_prepare(void);
static void start_app_prepare(void)
{
#if 0
	// Make sure, the CPU is in privileged mode.

	if( CONTROL_nPRIV_Msk & __get_CONTROL( ) ) {  /* not in privileged mode */
		 EnablePrivilegedMode( ) ;
	}

	// The function EnablePrivilegedMode( ) triggers a SVC, and enters handler mode (which can only run in privileged mode). The nPRIV bit in the CONTROL register is cleared which can only be done in privileged mode. See ARM: How to write an SVC function about implementing SVC functions.
#endif

	// Disable all enabled interrupts in NVIC.
	for (size_t i = 0; i < TU_ARRAY_SIZE(NVIC->ICER); ++i) {
		NVIC->ICER[i] = ~0;
	}

	// Disable all enabled peripherals which might generate interrupt requests, and clear all pending interrupt flags in those peripherals. Because this is device-specific, refer to the device datasheet for the proper way to clear these peripheral interrupts.
	// Clear all pending interrupt requests in NVIC.
	for (size_t i = 0; i < TU_ARRAY_SIZE(NVIC->ICPR); ++i) {
		NVIC->ICPR[i] = ~0;
	}

	// Disable SysTick and clear its exception pending bit, if it is used in the bootloader, e. g. by the RTX.
	SysTick->CTRL = 0;
	SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;

#if 0
	// Disable individual fault handlers if the bootloader used them.

	SCB->SHCSR &= ~( SCB_SHCSR_USGFAULTENA_Msk | \
					 SCB_SHCSR_BUSFAULTENA_Msk | \
					 SCB_SHCSR_MEMFAULTENA_Msk );

	// Activate the MSP, if the core is found to currently run with the PSP. As the compiler might still uses the stack, the PSP needs to be copied to the MSP before this.

	if (CONTROL_SPSEL_Msk & __get_CONTROL()) {
		  /* MSP is not active */
		__set_MSP( __get_PSP( ) ) ;
		__set_CONTROL( __get_CONTROL( ) & ~CONTROL_SPSEL_Msk ) ;
	}
#endif
}


// adapted from http://www.keil.com/support/docs/3913.htm
__attribute__((naked, noreturn)) static void app_jump(uint32_t sp, uint32_t rh)
{
	(void)sp;
	(void)rh;
	__asm__(
		"MSR MSP, r0\n"
		"BX  r1\n"
	);
}

__attribute__((noreturn)) static void start_app_jump(uint32_t addr)
{
	// Load the vector table address of the user application into SCB->VTOR register. Make sure the address meets the alignment requirements.

	SCB->VTOR = addr;

	// A few device families, like the NXP 4300 series, will also have a "shadow pointer" to the VTOR, which also needs to be updated with the new address. Review the device datasheet to see if one exists.
	// The final part is to set the MSP to the value found in the user application vector table and then load the PC with the reset vector value of the user application. This can't be done in C, as it is always possible, that the compiler uses the current SP. But that would be gone after setting the new MSP. So, a call to a small assembler function is done.

	uint32_t* base = (uint32_t*)(uintptr_t)addr;
	// LOG("stack @ %p reset @ %p\n", (void*)base[0], (void*)base[1]);
	app_jump(base[0], base[1]);
}

static inline void watchdog_timeout(uint8_t seconds_in, uint8_t* wdt_reg, uint8_t* seconds_out)
{
	if (seconds_in <= 1) {
		*wdt_reg = WDT_CONFIG_PER_CYC1024_Val;
		*seconds_out = 1;
	} else if (seconds_in <= 2) {
		*wdt_reg = WDT_CONFIG_PER_CYC2048_Val;
		*seconds_out = 2;
	} else if (seconds_in <= 4) {
		*wdt_reg = WDT_CONFIG_PER_CYC4096_Val;
		*seconds_out = 4;
	} else if (seconds_in <= 8) {
		*wdt_reg = WDT_CONFIG_PER_CYC8192_Val;
		*seconds_out = 8;
	} else {
		*wdt_reg = WDT_CONFIG_PER_CYC16384_Val;
		*seconds_out = 16;
	}
}

#define SUPERDFU_VERSION_MAJOR 0
#define SUPERDFU_VERSION_MINOR 2
#define SUPERDFU_VERSION_PATCH 2
#define STR2(x) #x
#define STR(x) STR2(x)
#define NAME "SuperDFU"



int main(void)
{
	board_init();

	LOG(NAME " v" STR(SUPERDFU_VERSION_MAJOR) "." STR(SUPERDFU_VERSION_MINOR) "." STR(SUPERDFU_VERSION_PATCH) " starting...\n");

	dfu.status.bStatus = DFU_ERROR_OK;
	dfu.status.bState = DFU_STATE_DFU_IDLE;
#if SUPERDFU_DEBUG
	dfu.status.bwPollTimeout = cpu_to_le32(5);
#else
	dfu.status.bwPollTimeout = cpu_to_le32(1);
#endif

	bool should_start_app = true;
	struct dfu_app_hdr const *app_hdr = (struct dfu_app_hdr const *)(uintptr_t)MCU_BOOTLOADER_SIZE;

	LOG(NAME " checking for bootloader signature @ %p ... ", dfu_hdr_ptr());
	if (0 == memcmp(DFU_RAM_HDR_MAGIC_STRING, dfu_hdr_ptr()->magic, sizeof(dfu_hdr_ptr()->magic))) {
		LOG("found\n");
		should_start_app = (dfu_hdr_ptr()->flags & DFU_RAM_HDR_FLAG_DFU_REQ) != DFU_RAM_HDR_FLAG_DFU_REQ;
		LOG(NAME " bootloader start requested: %d\n", !should_start_app);
		// clear bootloader start flag
		dfu_hdr_ptr()->flags &= ~DFU_RAM_HDR_FLAG_DFU_REQ;
	} else {
		LOG("not found\n");
		// initialize header
		memset(dfu_hdr_ptr(), 0, sizeof(struct dfu_hdr));
		memcpy(dfu_hdr_ptr()->magic, DFU_RAM_HDR_MAGIC_STRING, sizeof(dfu_hdr_ptr()->magic));
		dfu_hdr_ptr()->version = DFU_RAM_HDR_VERSION;
	}

	if (should_start_app) {
		LOG(NAME " checking app header @ %p\n", app_hdr);
		int error = dfu_app_hdr_validate(app_hdr);
		if (error) {
			dfu.status.bState = DFU_STATE_DFU_ERROR;
			dfu.status.bStatus = DFU_ERROR_FIRMWARE;
			should_start_app = false;

			switch (error) {
			case DFU_APP_ERROR_MAGIC_MISMATCH:
				LOG(NAME " magic mismatch\n");
				break;
			case DFU_APP_ERROR_UNSUPPORED_HDR_VERSION:
				LOG(NAME " unsupported version %u\n", app_hdr->hdr_version);
				break;
			case DFU_APP_ERROR_INVALID_SIZE:
				LOG(NAME " invalid size %lu [bytes]\n", app_hdr->app_size);
				break;
			case DFU_APP_ERROR_CRC_CALC_FAILED:
				LOG(NAME " crc calc failed\n");
				break;
			case DFU_APP_ERROR_CRC_APP_HEADER_MISMATCH:
				LOG(NAME " app header crc verification failed %08lx\n", app_hdr->hdr_crc);
				break;
			case DFU_APP_ERROR_CRC_APP_DATA_MISMATCH:
				LOG(NAME " app data crc verification failed %08lx\n", app_hdr->app_crc);
				break;
			default:
				LOG(NAME " unknown error %d\n", error);
				break;
			}
		} else {
			LOG(
				NAME " found %s v%u.%u.%u\n",
				app_hdr->app_name,
				app_hdr->app_version_major,
				app_hdr->app_version_minor,
				app_hdr->app_version_patch);
		}
	}

	if (should_start_app) {
		// check if stable
		should_start_app = dfu_hdr_ptr()->counter < 3;
		LOG(NAME " stable counter %lu\n", dfu_hdr_ptr()->counter);
	}

	if (should_start_app) {
		// increment counter in case the app crashes and resets the device
		LOG(NAME " incrementing stable counter\n");
		++dfu_hdr_ptr()->counter;

		start_app_prepare();

		// Setup watchdog timer in case app hangs, we'll now
		// by the counter value.
		uint8_t per, timeout;
		watchdog_timeout(app_hdr->app_watchdog_timeout_s, &per, &timeout);
		LOG(NAME " setting %u [s] watchdog timer\n", timeout);
		WDT->CONFIG.bit.PER = per;
		WDT->CTRLA.bit.ENABLE = 1;

		board_uart_write(NAME " gl hf, starting ", -1);
		board_uart_write(app_hdr->app_name, -1);
		board_uart_write("...\n", -1);

		// go go go
		start_app_jump(MCU_BOOTLOADER_SIZE + MCU_VECTOR_TABLE_ALIGNMENT);
		return 0; // never reached
	}

	board_uart_write("SuperDFU v" STR(SUPERDFU_VERSION_MAJOR) "." STR(SUPERDFU_VERSION_MINOR) "." STR(SUPERDFU_VERSION_PATCH) " running\n", -1);

	// set app stable counter
	dfu_hdr_ptr()->counter = 0;

	tusb_init();

	while (1) {
		led_task();
		tud_task();
	}

	return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	LOG("mounted\n");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	LOG("unmounted\n");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	LOG("suspend\n");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	LOG("resume\n");
}

#if SUPERDFU_DEBUG
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
#endif

#if CFG_TUD_DFU_RT_CUSTOM


void dfu_rtd_init(void)
{
	LOG("dfu_rtd_init\n");
}

void dfu_rtd_reset(uint8_t rhport)
{
	(void)rhport;
	LOG("dfu_rtd_reset\n");
	switch (dfu.status.bState) {
	case DFU_STATE_DFU_MANIFEST:
	case DFU_STATE_DFU_MANIFEST_SYNC:
	case DFU_STATE_DFU_MANIFEST_WAIT_RESET:
		NVIC_SystemReset();
		break;
	}


	if (dfu.download_size) { // hack for dfu-util, use with -R
		LOG("reset\n");
		NVIC_SystemReset();
	}
}

bool dfu_rtd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_length)
{
	(void)rhport;
	LOG("dfu_rtd_open\n");
	TU_VERIFY(itf_desc->bInterfaceSubClass == TUD_DFU_APP_SUBCLASS);
	TU_VERIFY(itf_desc->bInterfaceProtocol == DFU_PROTOCOL_DFU);

	*p_length = sizeof(tusb_desc_interface_t) + TUD_DFU_RT_DESC_LEN;

	return true;
}

static bool dfu_state_download_sync_complete(tusb_control_request_t const *request)
{
	if (!dfu.cleared_pages_left) {
		LOG("> clearing block @ %#08lx\n", dfu.prog_offset);
		if (nvm_erase_block((void*)dfu.prog_offset)) {
			dfu.cleared_pages_left = MCU_NVM_BLOCK_SIZE / MCU_NVM_PAGE_SIZE;
		} else {
			LOG("\tclearing failed for block @ %#08lx\n", dfu.prog_offset);
			dfu.status.bStatus = DFU_ERROR_ERASE;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
			return false;
		}
	}

	LOG("> write page @ %p\n", (void*)dfu.prog_offset);
	if (nvm_write_main_page((void*)dfu.prog_offset, dfu.page_buffer)) {
		if (0 == memcmp(dfu.page_buffer, (void*)dfu.prog_offset, sizeof(dfu.page_buffer))) {
			LOG("> verify page @ %p\n", (void*)dfu.prog_offset);
			dfu.download_size += request->wLength;
			dfu.prog_offset += sizeof(dfu.page_buffer);
			--dfu.cleared_pages_left;
			dfu.status.bState = DFU_STATE_DFU_DNLOAD_IDLE;
		} else {
			LOG("> target content\n");
			TU_LOG1_MEM(dfu.page_buffer, MCU_NVM_PAGE_SIZE, 2);
			LOG("> actual content\n");
			TU_LOG1_MEM((void*)dfu.prog_offset, MCU_NVM_PAGE_SIZE, 2);
			LOG("> verification failed for page @ %p\n", (void*)dfu.prog_offset);
			dfu.status.bStatus = DFU_ERROR_VERIFY;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
			return false;
		}
	} else {
		LOG("> write failed for page @ %p\n", (void*)dfu.prog_offset);
		dfu.status.bStatus = DFU_ERROR_WRITE;
		dfu.status.bState = DFU_STATE_DFU_ERROR;
		return false;
	}

	return true;
}

bool dfu_rtd_control_complete(uint8_t rhport, tusb_control_request_t const * request)
{
	(void)rhport;
	// LOG("complete req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
	// 	request->bmRequestType,
	// 	recipient_str(request->bmRequestType_bit.recipient),
	// 	type_str(request->bmRequestType_bit.type),
	// 	dir_str(request->bmRequestType_bit.direction),
	// 	request->bRequest, request->wValue, request->wIndex,
	// 	request->wLength);

	// LOG("DFU state=%u, status=%u\n", dfu.status.bState, dfu.status.bStatus);

	// LOG("dfu_rtd_control_complete\n");

	switch (dfu.status.bState) {
	case DFU_STATE_DFU_DNLOAD_SYNC:
		return dfu_state_download_sync_complete(request);
	default:
		break;
	// default:
	// 	LOG("> UNHANDLED STATE\n");
	// 	break;
	}

	return true;
}


static inline bool dfu_state_manifest_wait_reset(tusb_control_request_t const *request)
{
	const int port = 0;
	LOG("DFU_STATE_DFU_MANIFEST_WAIT_RESET\n");
	switch (request->bRequest) {
	case DFU_REQUEST_GETSTATUS:
		LOG("> DFU_REQUEST_GETSTATUS\n");
		if (unlikely(!tud_control_xfer(port, request, &dfu.status, sizeof(dfu.status)))) {
			dfu.status.bStatus = DFU_ERROR_UNKNOWN;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
		}
		break;
	default:
		LOG("> UNHANDLED REQUEST %02x\n", request->bRequest);
		break;
	}

	return true;
}

static inline bool dfu_state_download_idle(tusb_control_request_t const *request);
static inline bool dfu_start_download(tusb_control_request_t const *request)
{
	LOG("> DFU_REQUEST_DNLOAD\n");
	dfu.prog_offset = MCU_BOOTLOADER_SIZE;
	dfu.download_size = 0;
	dfu.cleared_pages_left = 0;

	dfu.status.bStatus = DFU_ERROR_OK;
	dfu.status.bState = DFU_STATE_DFU_DNLOAD_IDLE;
	return dfu_state_download_idle(request);
}

static inline bool dfu_state_error(tusb_control_request_t const *request)
{
	const int port = 0;
	LOG("DFU_STATE_DFU_ERROR\n");
	switch (request->bRequest) {
	case DFU_REQUEST_CLRSTATUS:
		LOG("> DFU_REQUEST_CLRSTATUS\n");
		dfu.status.bStatus = DFU_ERROR_OK;
		dfu.status.bState = DFU_STATE_DFU_IDLE;
		if (!tud_control_xfer(port, request, NULL, 0)) {
			dfu.status.bStatus = DFU_ERROR_UNKNOWN;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
			return false;
		}
		break;
	case DFU_REQUEST_DNLOAD: // dfu-tool (fwupd 1.2.5) doesn't clear status
		return dfu_start_download(request);
	default:
		LOG("> UNHANDLED REQUEST %02x\n", request->bRequest);
		break;
	}

	return true;
}

static inline bool dfu_state_download_sync(tusb_control_request_t const *request)
{
	const int port = 0;
	LOG("DFU_STATE_DFU_DNLOAD_SYNC\n");
	switch (request->bRequest) {
	case DFU_REQUEST_ABORT:
		LOG("> DFU_REQUEST_ABORT\n");
		dfu.status.bStatus = DFU_ERROR_OK;
		dfu.status.bState = DFU_STATE_DFU_IDLE;
		break;
	default:
		LOG("> UNHANDLED REQUEST %02x\n", request->bRequest);
		break;
	}

	if (!tud_control_xfer(port, request, NULL, 0)) {
		dfu.status.bStatus = DFU_ERROR_UNKNOWN;
		dfu.status.bState = DFU_STATE_DFU_ERROR;
		return false;
	}

	return true;
}

static inline bool dfu_state_download_idle(tusb_control_request_t const *request)
{
	const int port = 0;
	LOG("DFU_STATE_DFU_DNLOAD_IDLE\n");
	switch (request->bRequest) {
	case DFU_REQUEST_DNLOAD:
		LOG("> DFU_REQUEST_DNLOAD l=%u\n", request->wLength);
		if (request->wLength > 0) {
			LOG("> download_size=%#lx\n", dfu.download_size);
			LOG("> prog_offset=%#lx\n", dfu.prog_offset);

			TU_ASSERT(request->wLength <= sizeof(dfu.page_buffer));

			if (dfu.download_size + request->wLength > NVM_PROG_BLOCKS * MCU_NVM_BLOCK_SIZE) {
				dfu.status.bStatus = DFU_ERROR_ADDRESS;
				dfu.status.bState = DFU_STATE_DFU_ERROR;
				return false;
			}

			memset(dfu.page_buffer, 0, sizeof(dfu.page_buffer));
			if (!tud_control_xfer(port, request, dfu.page_buffer, sizeof(dfu.page_buffer))) {
				dfu.status.bStatus = DFU_ERROR_UNKNOWN;
				dfu.status.bState = DFU_STATE_DFU_ERROR;
				return false;
			}

			dfu.status.bState = DFU_STATE_DFU_DNLOAD_SYNC;
		} else {
			if (dfu.download_size) {
				//dfu.status.bState = DFU_STATE_DFU_MANIFEST_WAIT_RESET;
				// dfu.status.bState = DFU_STATE_DFU_IDLE;
				dfu.status.bState = DFU_STATE_DFU_MANIFEST_SYNC;
			} else {
				LOG("> no data received\n");
				dfu.status.bStatus = DFU_ERROR_NOTDONE;
				dfu.status.bState = DFU_STATE_DFU_ERROR;
			}

			if (!tud_control_xfer(port, request, NULL, 0)) {
				dfu.status.bStatus = DFU_ERROR_UNKNOWN;
				dfu.status.bState = DFU_STATE_DFU_ERROR;
				return false;
			}
			return true;
		}
		break;
	default:
		LOG("> UNHANDLED REQUEST %02x\n", request->bRequest);
		return false;
	}

	return true;
}

static inline bool dfu_state_idle(tusb_control_request_t const *request)
{
	LOG("DFU_STATE_DFU_IDLE\n");

	switch (request->bRequest) {
	case DFU_REQUEST_DNLOAD:
		return dfu_start_download(request);
	default:
		LOG("> UNHANDLED REQUEST %02x\n", request->bRequest);
		return false;
	}

	return true;
}

bool dfu_rtd_control_request(uint8_t rhport, tusb_control_request_t const *request)
{
	LOG("req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
		request->bmRequestType,
		recipient_str(request->bmRequestType_bit.recipient),
		type_str(request->bmRequestType_bit.type),
		dir_str(request->bmRequestType_bit.direction),
		request->bRequest, request->wValue, request->wIndex,
		request->wLength);

	TU_VERIFY(request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);
	TU_VERIFY(request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE);

	LOG("DFU state=%u, status=%u\n", dfu.status.bState, dfu.status.bStatus);

	switch (request->bRequest) {
	case DFU_REQUEST_GETSTATUS:
		LOG("> DFU_REQUEST_GETSTATUS\n");
		switch (dfu.status.bState) {
		case DFU_STATE_DFU_MANIFEST_SYNC:
			dfu.status.bState = DFU_STATE_DFU_MANIFEST;
			break;
		case DFU_STATE_DFU_MANIFEST:
			dfu.status.bState = DFU_STATE_DFU_IDLE;
			break;
		}


		if (unlikely(!tud_control_xfer(rhport, request, &dfu.status, sizeof(dfu.status)))) {
			dfu.status.bStatus = DFU_ERROR_UNKNOWN;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
		}
		return true;
	case DFU_REQUEST_DETACH: // dfu-util sends this
		if (unlikely(!tud_control_xfer(rhport, request, NULL, 0))) {
			dfu.status.bStatus = DFU_ERROR_UNKNOWN;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
			return false;
		}
		return true;
	}


	switch (dfu.status.bState) {
	case DFU_STATE_DFU_IDLE:
		return dfu_state_idle(request);
	case DFU_STATE_DFU_DNLOAD_IDLE:
		return dfu_state_download_idle(request);
	case DFU_STATE_DFU_DNLOAD_SYNC:
		return dfu_state_download_sync(request);
	case DFU_STATE_DFU_ERROR:
		return dfu_state_error(request);
	default:
		LOG("> UNHANDLED STATE\n");
		break;
	}

	return false;
}

bool dfu_rtd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
	LOG("dfu_rtd_xfer_cb\n");
	(void) rhport;
	(void) ep_addr;
	(void) result;
	(void) xferred_bytes;
	return true;
}
#endif

static void led_task(void)
{
	static uint32_t last_millis = 0;
	static bool led_state = false;

	uint32_t now = board_millis();
	uint32_t millis_elapsed = now - last_millis;
	if (millis_elapsed >= 100) {
		last_millis = now;
		led_state = !led_state;
		board_led_write(led_state);
	}
}