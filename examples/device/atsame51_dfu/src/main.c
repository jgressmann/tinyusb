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

#include <stdlib.h>
#include <stdio.h>
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


struct led {
#if CFG_TUSB_DEBUG > 0
	const char* name;
#endif
	uint16_t time_ms;
	uint16_t left;
	uint8_t blink;
	uint8_t pin;
};

#if CFG_TUSB_DEBUG > 0
#define LED_STATIC_INITIALIZER(name, pin) \
	{ name, 0, 0, 0, pin }
#else
#define LED_STATIC_INITIALIZER(name, pin) \
	{ 0, 0, 0, pin }
#endif

static struct led leds[] = {
	LED_STATIC_INITIALIZER("debug", PIN_PA02),
	LED_STATIC_INITIALIZER("red1", PIN_PB14),
	LED_STATIC_INITIALIZER("orange1", PIN_PB15),
	LED_STATIC_INITIALIZER("green1", PIN_PA12),
	LED_STATIC_INITIALIZER("red2", PIN_PA13),
	LED_STATIC_INITIALIZER("orange2", PIN_PA14),
	LED_STATIC_INITIALIZER("green2", PIN_PA15),
};

enum {
	LED_DEBUG,
	LED_RED1,
	LED_ORANGE1,
	LED_GREEN1,
	LED_RED2,
	LED_ORANGE2,
	LED_GREEN2
};

#define USB_TRAFFIC_LED LED_ORANGE1
#define USB_TRAFFIC_BURST_DURATION_MS 8
#define USB_TRAFFIC_DO_LED led_burst(USB_TRAFFIC_LED, USB_TRAFFIC_BURST_DURATION_MS)


static void led_init(void);
static void led_task(void);
static inline void led_set(uint8_t index, bool on)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = 0;
	gpio_set_pin_level(leds[index].pin, on);
}

static inline void led_toggle(uint8_t index)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = 0;
	gpio_toggle_pin_level(leds[index].pin);
}

static inline void led_blink(uint8_t index, uint16_t delay_ms)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = delay_ms;
	leds[index].blink = 1;
}

static inline void led_burst(uint8_t index, uint16_t duration_ms)
{
	TU_ASSERT(index < TU_ARRAY_SIZE(leds), );
	leds[index].time_ms = duration_ms;
	leds[index].blink = 0;
	gpio_set_pin_level(leds[index].pin, 1);
}



static struct usb {
	// CFG_TUSB_MEM_ALIGN uint8_t cmd_rx_buffers[2][CMD_BUFFER_SIZE];
	// CFG_TUSB_MEM_ALIGN uint8_t cmd_tx_buffers[2][CMD_BUFFER_SIZE];
	// uint16_t cmd_tx_offsets[2];
	// uint8_t cmd_rx_bank;
	// uint8_t cmd_tx_bank;
	uint8_t port;
	bool mounted;
} usb;

// DEBUG=1 LOG=2 ~24K (text)
// DEBUG=1 LOG=0 ~14K (text)
// DEBUG=0 LOG=0 ~12K (text)
// #if !defined(BOOTLOADER_SIZE) || BOOTLOADER_SIZE <= 0
// #error Define BOOTLOADER_SIZE to a multiple of 2
// #endif
#define BOOTLOADER_SIZE (1u<<15)

// #define NVM_REGIONS 32
// #define NVM_REGION_SIZE (1ul<<14)
// #define NVM_APP_REGION_OFFSET (((1ul<<19) - BOOTLOADER_SIZE) / NVM_REGION_SIZE)
#define NVM_BOOTLOADER_BLOCKS (BOOTLOADER_SIZE / MCU_NVM_BLOCK_SIZE + (BOOTLOADER_SIZE % MCU_NVM_BLOCK_SIZE ? 1 : 0))
#define NVM_PROG_BLOCKS (MCU_NVM_SIZE / 2 - NVM_BOOTLOADER_BLOCKS)



struct dfu_get_status_reply {
	uint32_t bStatus : 8;
	uint32_t bwPollTimeout : 24;
	uint8_t bState;
	uint8_t iString;
} __packed;

static struct dfu {
	struct dfu_get_status_reply status;
	uint32_t download_size;
	uint32_t prog_offset;
	uint16_t cleared_pages_left : 15;
	uint16_t current_block_cleared : 1;
	uint32_t page_buffer[MCU_NVM_PAGE_SIZE / 4];
} dfu;

struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_SECTION_NAME)));

static inline bool nvm_erase_block(void *addr)
{
	TU_LOG2("erase block %p\n", addr);

	while (!NVMCTRL->STATUS.bit.READY); // wait for nvmctrl to be ready

	NVMCTRL->ADDR.reg = NVMCTRL_ADDR_ADDR((uintptr_t)addr);

	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY;

	// if (!NVMCTRL->STATUS.bit.READY) {
	// 	TU_LOG2("erasing block at %lx\n", other_bank_offset);
	// }


	while (!NVMCTRL->STATUS.bit.READY);

	// clear done flag
	NVMCTRL->INTFLAG.bit.DONE = 1;

	TU_LOG2("INTFLAG %#08lx\n", (uint32_t)NVMCTRL->INTFLAG.reg);

	// check for errors
	if (unlikely(NVMCTRL->INTFLAG.reg)) {
		return false;
	}

	return true;
}

static inline bool nvm_write_main_page(void *addr, void const *ptr)
{
	TU_LOG2("write main page @ %p\n", addr);

	while (!NVMCTRL->STATUS.bit.READY); // wait for nvmctrl to be ready

	// clear flags
	NVMCTRL->INTFLAG.reg = NVMCTRL->INTFLAG.reg;

	// clear page buffer
	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_PBC | NVMCTRL_CTRLB_CMDEX_KEY;
		// if (!NVMCTRL->STATUS.bit.READY) {
		// 	TU_LOG2("erasing page buffer\n");
		// }
	// wait for erase
	while (!NVMCTRL->STATUS.bit.READY);

	// clear done flag
	NVMCTRL->INTFLAG.bit.DONE = 1;

	TU_LOG2("INTFLAG %#08lx\n", (uint32_t)NVMCTRL->INTFLAG.reg);

	// check for errors
	if (unlikely(NVMCTRL->INTFLAG.reg)) {
		return false;
	}

	memcpy(addr, ptr, MCU_NVM_PAGE_SIZE);

	NVMCTRL->ADDR.reg = NVMCTRL_ADDR_ADDR((uintptr_t)addr);
	// TU_LOG2("ADDR after memcpy %#08lx\n", NVMCTRL->ADDR.reg);

	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_WP | NVMCTRL_CTRLB_CMDEX_KEY;

	while (!NVMCTRL->STATUS.bit.READY);

	// clear done flag
	NVMCTRL->INTFLAG.bit.DONE = 1;

	TU_LOG2("INTFLAG %#08lx\n", (uint32_t)NVMCTRL->INTFLAG.reg);

	// check for errors
	if (unlikely(NVMCTRL->INTFLAG.reg)) {
		return false;
	}

	return true;
}

static inline bool nvm_see_flush(void)
{
	TU_LOG2("SmartEEPROM flush\n");
	while (!NVMCTRL->STATUS.bit.READY); // wait for nvmctrl to be ready

	// clear flags
	NVMCTRL->INTFLAG.reg = NVMCTRL->INTFLAG.reg;

	// flush page buffer
	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_SEEFLUSH | NVMCTRL_CTRLB_CMDEX_KEY;

	while (!NVMCTRL->STATUS.bit.READY);

	// clear done flag
	NVMCTRL->INTFLAG.bit.DONE = 1;

	TU_LOG2("INTFLAG %#08lx\n", (uint32_t)NVMCTRL->INTFLAG.reg);

	return !NVMCTRL->INTFLAG.reg;
}

// adapted from http://www.keil.com/support/docs/3913.htm
__attribute__((naked, noreturn)) static void app_jump(uint32_t sp, uint32_t rh)
{
	(void)sp;
	(void)rh;
	__asm__("MSR MSP, r0");
	__asm__("BX  r1");
}

__attribute__((noreturn)) static void start_app(uint32_t addr)
{
	// Make sure, the CPU is in privileged mode.

	//   if( CONTROL_nPRIV_Msk & __get_CONTROL( ) )
	//   {  /* not in privileged mode */
	//	 EnablePrivilegedMode( ) ;
	//   }

	// The function EnablePrivilegedMode( ) triggers a SVC, and enters handler mode (which can only run in privileged mode). The nPRIV bit in the CONTROL register is cleared which can only be done in privileged mode. See ARM: How to write an SVC function about implementing SVC functions.
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

	// Disable individual fault handlers if the bootloader used them.

	SCB->SHCSR &= ~( SCB_SHCSR_USGFAULTENA_Msk | \
					 SCB_SHCSR_BUSFAULTENA_Msk | \
					 SCB_SHCSR_MEMFAULTENA_Msk );

	// Activate the MSP, if the core is found to currently run with the PSP. As the compiler might still uses the stack, the PSP needs to be copied to the MSP before this.

	if (CONTROL_SPSEL_Msk & __get_CONTROL())
	{
		  /* MSP is not active */
		__set_MSP( __get_PSP( ) ) ;
		__set_CONTROL( __get_CONTROL( ) & ~CONTROL_SPSEL_Msk ) ;
	}

	// Load the vector table address of the user application into SCB->VTOR register. Make sure the address meets the alignment requirements.

	SCB->VTOR = BOOTLOADER_SIZE;

	// A few device families, like the NXP 4300 series, will also have a "shadow pointer" to the VTOR, which also needs to be updated with the new address. Review the device datasheet to see if one exists.
	// The final part is to set the MSP to the value found in the user application vector table and then load the PC with the reset vector value of the user application. This can't be done in C, as it is always possible, that the compiler uses the current SP. But that would be gone after setting the new MSP. So, a call to a small assembler function is done.

	uint32_t* base = (uint32_t*)BOOTLOADER_SIZE;
	app_jump(base[0], base[1]);
}

#define SUPERDFU_VERSION_MAJOR 0
#define SUPERDFU_VERSION_MINOR 1
#define SUPERDFU_VERSION_PATCH 0
#define STR2(x) #x
#define STR(x) STR2(x)



int main(void)
{
	board_init();

	TU_LOG2("SuperDFU v" STR(SUPERDFU_VERSION_MAJOR) "." STR(SUPERDFU_VERSION_MINOR) "." STR(SUPERDFU_VERSION_PATCH) " starting...\n");

	TU_LOG2("SmartEEPROM supported: %u\n", NVMCTRL->PARAM.bit.SEE);
	if (NVMCTRL->PARAM.bit.SEE) {
		TU_LOG2("SmartEEPROM page size: %u\n", NVMCTRL->SEESTAT.bit.PSZ);
		TU_LOG2("SmartEEPROM block size: %u\n", NVMCTRL->SEESTAT.bit.SBLK);
		TU_LOG2("SmartEEPROM register locked: %u\n", NVMCTRL->SEESTAT.bit.RLOCK);
		TU_LOG2("SmartEEPROM section locked: %u\n", NVMCTRL->SEESTAT.bit.LOCK);
		TU_LOG2("SmartEEPROM active sector: %u\n", NVMCTRL->SEESTAT.bit.ASEES);
	}
	TU_LOG2("Page size: %u\n", 8 << NVMCTRL->PARAM.bit.PSZ);
	TU_LOG2("Page count: %u\n", NVMCTRL->PARAM.bit.NVMP);
	uint32_t bootload_protected_bytes = (15-NVMCTRL->STATUS.bit.BOOTPROT) * (1u<<13u);
	(void)bootload_protected_bytes;
	TU_LOG2("Bootloader protected bytes: %lu\n", bootload_protected_bytes);
	TU_LOG2("Bootloader protection enabled: %u\n", !NVMCTRL->STATUS.bit.BPDIS);
	TU_LOG2("Bank A is mapped at 0x00000000: %u\n", !NVMCTRL->STATUS.bit.AFIRST);
	TU_LOG2("NVM write mode? %u\n", NVMCTRL->CTRLA.bit.WMODE);
	TU_LOG2("NVM status ready? %u\n", NVMCTRL->STATUS.bit.READY);
	TU_LOG2("NVM region locks? %08lx\n", NVMCTRL->RUNLOCK.reg);
	TU_LOG2("NVM user? %#08lx\n", NVMCTRL_USER);
	TU_LOG2("Bootloader size? %#lx\n", (unsigned long)BOOTLOADER_SIZE);

	bool should_start_app = true;
	struct dfu_app_hdr const *app_hdr = (struct dfu_app_hdr const *)(uintptr_t)BOOTLOADER_SIZE;

	if (0 == memcmp(DFU_RAM_MAGIC_STRING, dfu_hdr.magic, sizeof(dfu_hdr.magic))) {
		should_start_app = (dfu_hdr.flags & DFU_RAM_FLAG_DFU_REQ) == DFU_RAM_FLAG_DFU_REQ;
		TU_LOG2("Bootloader start requested: %d\n", should_start_app);
	} else {
		TU_LOG2("Bootloader ram section dfuram @ %p not initalized\n", &dfu_hdr);
		// initialize header
		memset(&dfu_hdr, 0, sizeof(struct dfu_hdr));
		memcpy(dfu_hdr.magic, DFU_RAM_MAGIC_STRING, sizeof(dfu_hdr.magic));
	}

	if (should_start_app) {
		TU_LOG2("Checking app header @ %p\n", app_hdr);
		int error = dfu_app_validate(app_hdr);
		if (error) {
			should_start_app = false;
			switch (error) {
			case DFU_APP_ERROR_MAGIC_MISMATCH:
				TU_LOG2("magic mismatch\n");
				break;
			case DFU_APP_ERROR_UNSUPPORED_HDR_VERSION:
				TU_LOG2("unsupported version %u\n", app_hdr->dfu_app_hdr_version);
				break;
			case DFU_APP_ERROR_INVALID_SIZE:
				TU_LOG2("invalid size %lu [bytes]\n", app_hdr->app_size);
				break;
			case DFU_APP_ERROR_CRC_CALC_FAILED:
				TU_LOG2("crc calc failed\n");
				break;
			case DFU_APP_ERROR_CRC_VERIFICATION_FAILED:
				TU_LOG2("app crc verification failed %08lx\n", app_hdr->app_crc);
				break;
			default:
				TU_LOG2("unknown error %d\n", error);
				break;
			}
		} else {
			TU_LOG2(
				"Found %s v%u.%u.%u\n",
				app_hdr->app_name,
				app_hdr->app_version_major,
				app_hdr->app_version_minor,
				app_hdr->app_version_patch);
		}
	}

	if (should_start_app) {
		// check if stable
		should_start_app = dfu_hdr.counter < 3;
	}

	if (should_start_app && false) {
		// increment counter in case the app crashes and resets the device
		++dfu_hdr.counter;
		start_app(BOOTLOADER_SIZE + sizeof(*app_hdr));
		return 0; // never reached
	}

	TU_LOG2("SuperDFU v" STR(SUPERDFU_VERSION_MAJOR) "." STR(SUPERDFU_VERSION_MINOR) "." STR(SUPERDFU_VERSION_PATCH) " running\n");

	dfu.status.bStatus = DFU_ERROR_OK;
	dfu.status.bState = DFU_STATE_DFU_IDLE;
	dfu.status.bwPollTimeout = cpu_to_le32(100);
	dfu.status.iString = 0;

	led_init();
	tusb_init();

	led_blink(0, 2000);
	led_set(LED_RED1, 1);

	while (1) {
		tud_task();
		led_task();
	}

	return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	TU_LOG2("mounted\n");
	led_blink(0, 250);
	usb.mounted = true;

}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	TU_LOG2("unmounted\n");
	led_blink(0, 1000);
	usb.mounted = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	TU_LOG2("suspend\n");
	usb.mounted = false;
	led_blink(0, 500);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	TU_LOG2("resume\n");
	usb.mounted = true;
	led_blink(0, 250);
}


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

#if CFG_TUD_DFU_RT_CUSTOM


void dfu_rtd_init(void)
{
}

void dfu_rtd_reset(uint8_t rhport)
{
	usb.port = rhport;
}

bool dfu_rtd_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t *p_length)
{
	if (unlikely(rhport != usb.port)) {
		return false;
	}


	// Ensure this is DFU Runtime
	TU_VERIFY(itf_desc->bInterfaceSubClass == TUD_DFU_APP_SUBCLASS);
	TU_VERIFY(itf_desc->bInterfaceProtocol == DFU_PROTOCOL_RT);

	uint8_t const * p_desc = tu_desc_next(itf_desc);
	(*p_length) = sizeof(tusb_desc_interface_t);

	if (TUSB_DESC_FUNCTIONAL == tu_desc_type(p_desc)) {
		*p_length += p_desc[DESC_OFFSET_LEN];
		// p_desc = tu_desc_next(p_desc);
	}

	return true;
}

static bool dfu_state_download_sync_complete(tusb_control_request_t const *request)
{
	// TU_LOG2("> page buffer\n");
	// TU_LOG1_MEM(dfu.page_buffer, sizeof(dfu.page_buffer), 2);

	if (!dfu.current_block_cleared) {
		TU_LOG2("> clearing block @ %#08lx\n", dfu.prog_offset);
		if (nvm_erase_block((void*)dfu.prog_offset)) {
			// TU_LOG2_MEM((void*)dfu.prog_offset, MCU_NVM_BLOCK_SIZE, 2);
			// for (unsigned i = 0; i < MCU_NVM_BLOCK_SIZE / 4; ++i) {
			// 	TU_LOG2("mem @ %#08lx[%u]: %08x\n", dfu.prog_offset, i * 4, *(uint32_t*)(dfu.prog_offset + i * 4));
			// }

			dfu.current_block_cleared = 1;
			dfu.cleared_pages_left = MCU_NVM_BLOCK_SIZE / MCU_NVM_PAGE_SIZE;
		} else {
			TU_LOG1("\tclearing failed for block @ %#08lx\n", dfu.prog_offset);
			dfu.status.bStatus = DFU_ERROR_ERASE;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
			return false;
		}
	}

	TU_LOG2("> write page @ %p\n", (void*)dfu.prog_offset);
	if (nvm_write_main_page((void*)dfu.prog_offset, dfu.page_buffer)) {
		if (0 == memcmp(dfu.page_buffer, (void*)dfu.prog_offset, sizeof(dfu.page_buffer))) {
			TU_LOG2("> verify page @ %p\n", (void*)dfu.prog_offset);
			dfu.download_size += request->wLength;
			dfu.prog_offset += sizeof(dfu.page_buffer);
			--dfu.cleared_pages_left;
			dfu.current_block_cleared = dfu.cleared_pages_left > 0;
			dfu.status.bState = DFU_STATE_DFU_DNLOAD_IDLE;
		} else {
			TU_LOG1("> target content\n");
			TU_LOG1_MEM(dfu.page_buffer, MCU_NVM_PAGE_SIZE, 2);
			TU_LOG1("> actual content\n");
			TU_LOG1_MEM((void*)dfu.prog_offset, MCU_NVM_PAGE_SIZE, 2);
			TU_LOG1("> verification failed for page @ %p\n", (void*)dfu.prog_offset);
			dfu.status.bStatus = DFU_ERROR_VERIFY;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
			return false;
		}
	} else {
		TU_LOG1("> write failed for page @ %p\n", (void*)dfu.prog_offset);
		dfu.status.bStatus = DFU_ERROR_WRITE;
		dfu.status.bState = DFU_STATE_DFU_ERROR;
		return false;
	}

	return true;
}

bool dfu_rtd_control_complete(uint8_t rhport, tusb_control_request_t const * request)
{
	if (unlikely(rhport != usb.port)) {
		TU_LOG1("USB port mismatch (expected %u, have %u)\n", usb.port, rhport);
		return false;
	}

	TU_LOG2("complete req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
		request->bmRequestType,
		recipient_str(request->bmRequestType_bit.recipient),
		type_str(request->bmRequestType_bit.type),
		dir_str(request->bmRequestType_bit.direction),
		request->bRequest, request->wValue, request->wIndex,
		request->wLength);

	TU_VERIFY(request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);
	TU_VERIFY(request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE);

	TU_LOG2("DFU state=%u, status=%u\n", dfu.status.bState, dfu.status.bStatus);

	// TU_LOG2("dfu_rtd_control_complete\n");

	switch (dfu.status.bState) {
	case DFU_STATE_DFU_DNLOAD_SYNC:
		return dfu_state_download_sync_complete(request);
	default:
		break;
	// default:
	// 	TU_LOG2("> UNHANDLED STATE\n");
	// 	break;
	}

	return true;
}

static bool dfu_state_manifest_sync(tusb_control_request_t const *request)
{
	TU_LOG2("DFU_STATE_DFU_MANIFEST_SYNC\n");
	switch (request->bRequest) {
	case DFU_REQUEST_GETSTATUS:
		TU_LOG2("> DFU_REQUEST_GETSTATUS\n");
		if (unlikely(!tud_control_xfer(usb.port, request, &dfu.status, sizeof(dfu.status)))) {
			dfu.status.bStatus = DFU_ERROR_UNKNOWN;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
		}
		// manifest
		TU_LOG2("> clear bootloader blocks\n");
		for (uint8_t i = 0; i < NVM_BOOTLOADER_BLOCKS; ++i) {
			if (nvm_erase_block((void*)(MCU_NVM_SIZE / 2 + i * MCU_NVM_BLOCK_SIZE))) {

			} else {
				dfu.status.bStatus = DFU_ERROR_ERASE;
				dfu.status.bState = DFU_STATE_DFU_ERROR;
				return false;
			}
		}

		TU_LOG2("> copy bootloader pages\n");
		for (uint32_t i = 0; i < NVM_BOOTLOADER_BLOCKS * (MCU_NVM_BLOCK_SIZE / MCU_NVM_PAGE_SIZE); ++i) {
			void *src = (void*)(i * MCU_NVM_PAGE_SIZE);
			void *dst = (void*)(MCU_NVM_SIZE / 2 + i * MCU_NVM_PAGE_SIZE);
			if (nvm_write_main_page(dst, src)) {
				if (0 == memcmp(dst, src, MCU_NVM_PAGE_SIZE)) {

				} else {
					dfu.status.bStatus = DFU_ERROR_VERIFY;
					dfu.status.bState = DFU_STATE_DFU_ERROR;
					return false;
				}
			} else {
				dfu.status.bStatus = DFU_ERROR_WRITE;
				dfu.status.bState = DFU_STATE_DFU_ERROR;
				return false;
			}
		}

		TU_LOG2("> swap banks\n");
		while (!NVMCTRL->STATUS.bit.READY); // wait for nvmctrl to be ready

		// clear flags
		NVMCTRL->INTFLAG.reg = NVMCTRL->INTFLAG.reg;

		// clear page buffer
		NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_BKSWRST | NVMCTRL_CTRLB_CMDEX_KEY;
			// if (!NVMCTRL->STATUS.bit.READY) {
			// 	TU_LOG2("erasing page buffer\n");
			// }
		// wait for erase
		while (!NVMCTRL->STATUS.bit.READY);
		break;
	case DFU_REQUEST_ABORT:
		TU_LOG2("> DFU_REQUEST_ABORT\n");
		dfu.status.bStatus = DFU_ERROR_OK;
		dfu.status.bState = DFU_STATE_DFU_IDLE;
		break;
	default:
		TU_LOG2("> UNHANDLED REQUEST %02x\n", request->bRequest);
		break;
	}

	return true;
}

static bool dfu_state_error(tusb_control_request_t const *request)
{
	TU_LOG2("DFU_STATE_DFU_ERROR\n");
	switch (request->bRequest) {
	case DFU_REQUEST_GETSTATUS:
		TU_LOG2("> DFU_REQUEST_GETSTATUS\n");
		if (unlikely(!tud_control_xfer(usb.port, request, &dfu.status, sizeof(dfu.status)))) {
			dfu.status.bStatus = DFU_ERROR_UNKNOWN;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
		}
		break;
	case DFU_REQUEST_CLRSTATUS:
		TU_LOG2("> DFU_REQUEST_CLRSTATUS\n");
		dfu.status.bStatus = DFU_ERROR_OK;
		dfu.status.bState = DFU_STATE_DFU_IDLE;
		break;
	default:
		TU_LOG2("> UNHANDLED REQUEST %02x\n", request->bRequest);
		break;
	}

	return true;
}

static bool dfu_state_download_sync(tusb_control_request_t const *request)
{
	TU_LOG2("DFU_STATE_DFU_DNLOAD_SYNC\n");
	switch (request->bRequest) {
	case DFU_REQUEST_GETSTATUS:
		TU_LOG2("> DFU_REQUEST_GETSTATUS\n");
		if (unlikely(!tud_control_xfer(usb.port, request, &dfu.status, sizeof(dfu.status)))) {
			dfu.status.bStatus = DFU_ERROR_UNKNOWN;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
		}
		break;
	case DFU_REQUEST_ABORT:
		TU_LOG2("> DFU_REQUEST_ABORT\n");
		dfu.status.bStatus = DFU_ERROR_OK;
		dfu.status.bState = DFU_STATE_DFU_IDLE;
		break;
	default:
		TU_LOG2("> UNHANDLED REQUEST %02x\n", request->bRequest);
		break;
	}

	return true;
}

static bool dfu_state_download_idle(tusb_control_request_t const *request)
{
	TU_LOG2("DFU_STATE_DFU_DNLOAD_IDLE\n");
	switch (request->bRequest) {
	case DFU_REQUEST_DNLOAD:
		TU_LOG2("> DFU_REQUEST_DNLOAD l=%u\n", request->wLength);
		if (request->wLength > 0) {
			TU_LOG2("> download_size=%#lx\n", dfu.download_size);
			TU_LOG2("> prog_offset=%#lx\n", dfu.prog_offset);

			TU_ASSERT(request->wLength <= sizeof(dfu.page_buffer));

			if (dfu.download_size + request->wLength > NVM_PROG_BLOCKS * MCU_NVM_BLOCK_SIZE) {
				dfu.status.bStatus = DFU_ERROR_ADDRESS;
				dfu.status.bState = DFU_STATE_DFU_ERROR;
				return false;
			}

			memset(dfu.page_buffer, 0, sizeof(dfu.page_buffer));
			if (!tud_control_xfer(usb.port, request, dfu.page_buffer, sizeof(dfu.page_buffer))) {
				dfu.status.bStatus = DFU_ERROR_UNKNOWN;
				dfu.status.bState = DFU_STATE_DFU_ERROR;
				return false;
			}

			dfu.status.bState = DFU_STATE_DFU_DNLOAD_SYNC;
		} else {
			if (dfu.download_size) {
				dfu.status.bState = DFU_STATE_DFU_MANIFEST_SYNC;
			} else {
				TU_LOG1("> no data received\n");
				dfu.status.bStatus = DFU_ERROR_NOTDONE;
				dfu.status.bState = DFU_STATE_DFU_ERROR;
				return false;
			}
		}
		break;
	case DFU_REQUEST_GETSTATUS:
		TU_LOG2("> DFU_REQUEST_GETSTATUS\n");
		if (unlikely(!tud_control_xfer(usb.port, request, &dfu.status, sizeof(dfu.status)))) {
			dfu.status.bStatus = DFU_ERROR_UNKNOWN;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
		}
		break;
	default:
		TU_LOG2("> UNHANDLED REQUEST %02x\n", request->bRequest);
		return false;
	}

	return true;
}

static bool dfu_state_idle(tusb_control_request_t const *request)
{
	TU_LOG2("DFU_STATE_DFU_IDLE\n");

	switch (request->bRequest) {
	case DFU_REQUEST_DNLOAD:
		TU_LOG2("> DFU_REQUEST_DNLOAD\n");
		dfu.prog_offset = MCU_NVM_SIZE / 2 + NVM_BOOTLOADER_BLOCKS * MCU_NVM_BLOCK_SIZE;
		dfu.current_block_cleared = 0;
		dfu.download_size = 0;
		// dfu.page_buffer_size = 0;
		dfu.cleared_pages_left = 0;
		// memset(dfu.page_buffer_copy, 0, sizeof(dfu.page_buffer_copy));

		dfu.status.bStatus = DFU_ERROR_OK;
		dfu.status.bState = DFU_STATE_DFU_DNLOAD_IDLE;
		return dfu_state_download_idle(request);

	case DFU_REQUEST_GETSTATUS:
		TU_LOG2("> DFU_REQUEST_GETSTATUS\n");
		if (unlikely(!tud_control_xfer(usb.port, request, &dfu.status, sizeof(dfu.status)))) {
			dfu.status.bStatus = DFU_ERROR_UNKNOWN;
			dfu.status.bState = DFU_STATE_DFU_ERROR;
		}
		break;
	default:
		TU_LOG2("> UNHANDLED REQUEST %02x\n", request->bRequest);
		return false;
	}


	return true;
}

bool dfu_rtd_control_request(uint8_t rhport, tusb_control_request_t const *request)
{
	if (unlikely(rhport != usb.port)) {
		TU_LOG1("USB port mismatch (expected %u, have %u)\n", usb.port, rhport);
		return false;
	}

	TU_LOG2("req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
		request->bmRequestType,
		recipient_str(request->bmRequestType_bit.recipient),
		type_str(request->bmRequestType_bit.type),
		dir_str(request->bmRequestType_bit.direction),
		request->bRequest, request->wValue, request->wIndex,
		request->wLength);

	TU_VERIFY(request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);
	TU_VERIFY(request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE);

	TU_LOG2("DFU state=%u, status=%u\n", dfu.status.bState, dfu.status.bStatus);

	switch (dfu.status.bState) {
	case DFU_STATE_DFU_IDLE:
		return dfu_state_idle(request);
	case DFU_STATE_DFU_DNLOAD_IDLE:
		return dfu_state_download_idle(request);
	case DFU_STATE_DFU_DNLOAD_SYNC:
		return dfu_state_download_sync(request);
	case DFU_STATE_DFU_ERROR:
		return dfu_state_error(request);
	case DFU_STATE_DFU_MANIFEST_SYNC:
		return dfu_state_manifest_sync(request);
	default:
		TU_LOG2("> UNHANDLED STATE\n");
		break;
	}

	return false;
}

bool dfu_rtd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
	TU_LOG2("dfu_rtd_xfer_cb\n");
	(void) rhport;
	(void) ep_addr;
	(void) result;
	(void) xferred_bytes;
	return true;
}
#endif

//--------------------------------------------------------------------+
// LED TASK
//--------------------------------------------------------------------+
static void led_init(void)
{
	PORT->Group[1].DIRSET.reg = PORT_PB14; /* Debug-LED */
	PORT->Group[1].DIRSET.reg = PORT_PB15; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA12; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA13; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA14; /* Debug-LED */
	PORT->Group[0].DIRSET.reg = PORT_PA15; /* Debug-LED */
}

static void led_task(void)
{
	static uint32_t last_ms = 0;
	uint32_t now = board_millis();
	uint32_t elapsed = now - last_ms;
	last_ms = now;

	if (elapsed) {
		for (size_t i = 0; i < TU_ARRAY_SIZE(leds); ++i) {
			uint16_t t = leds[i].time_ms;
			if (!t) {
				continue;
			}

			if (leds[i].blink) {
				if (elapsed >= leds[i].left) {
					// TU_LOG2("led %s (%u) toggle\n", leds[i].name, i);
					gpio_toggle_pin_level(leds[i].pin);
					leds[i].left = t - (elapsed - leds[i].left);
				} else {
					leds[i].left -= elapsed;
				}
			} else {
				// burst
				if (leds[i].left) {
					uint16_t sub = tu_min16(elapsed, leds[i].left);
					leds[i].left -= sub;
					if (!leds[i].left) {
						gpio_set_pin_level(leds[i].pin, 0);
					}
				}
			}
		}
	}
}
