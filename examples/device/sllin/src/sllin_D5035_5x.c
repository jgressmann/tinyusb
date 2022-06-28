/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */


#include <sllin_board.h>

#if D5035_50 || D5035_51

#include <tusb.h>
#include <class/dfu/dfu_rt_device.h>


#if SUPERDFU_APP
struct dfu_hdr dfu_hdr __attribute__((section(DFU_RAM_HDR_SECTION_NAME)));
static struct dfu_app_tag dfu_app_tag __attribute__((used,section(DFU_APP_TAG_SECTION_NAME))) = {
	.tag_magic = DFU_APP_TAG_MAGIC_STRING,
	.tag_version = DFU_APP_TAG_VERSION,
	.tag_bom = DFU_APP_TAG_BOM,
	.tag_dev_id = SUPERDFU_DEV_ID,
	.app_version_major = SLLIN_VERSION_MAJOR,
	.app_version_minor = SLLIN_VERSION_MINOR,
	.app_version_patch = SLLIN_VERSION_PATCH,
	.app_watchdog_timeout_s = 1,
	.app_name = SLLIN_NAME,
};

static struct dfu_app_tag const * const dfu_app_tag_ptr __attribute__((used,section(DFU_APP_TAG_PTR_SECTION_NAME))) = &dfu_app_tag;


void dfu_timer_expired(void)
{
	LOG("DFU detach timer expired\n");

	/*
	 * This is wrong. DFU 1.1. specification wants us to wait for USB reset.
	 * However, without these lines, the device can't be updated on Windows using
	 * dfu-util. Updating from Linux works either way. All hail compatibility (sigh).
	 */
	dfu_request_dfu(1);
	NVIC_SystemReset();
}

#if CFG_TUD_DFU_RUNTIME
void tud_dfu_runtime_reboot_to_dfu_cb(uint16_t ms)
{
	LOG("DFU set detach timer to %u [ms]\n", ms);

	/* The timer seems to be necessary, else dfu-util
	 * will fail spurriously with EX_IOERR (74).
	 */
	dfu_timer_start(ms);
}
#endif // #if CFG_TUD_DFU_RT

extern void dfu_init_begin(void)
{
	LOG(
		"%s v%u.%u.%u starting...\n",
		dfu_app_tag.app_name,
		dfu_app_tag.app_version_major,
		dfu_app_tag.app_version_minor,
		dfu_app_tag.app_version_patch);

	dfu_request_dfu(0); // no bootloader request
}

#endif


#endif // #if D5035_50 || D5035_51
