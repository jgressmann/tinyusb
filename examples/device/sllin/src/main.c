/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */


#include <FreeRTOS.h>
#include <task.h>


#include <tusb.h>
#include <sllin_board.h>
#include <usb_descriptors.h>
#include <leds.h>


enum {
	SLLIN_ERROR_TERMINATOR = 7,
	SLLIN_OK_TERMINATOR = 13,
	SLLIN_QUEUE_ELEMENT_TYPE_TIME_STAMP = SLLIN_QUEUE_ELEMENT_TYPE_COUNT,
};


SLLIN_RAMFUNC static void tusb_device_task(void* param);
SLLIN_RAMFUNC static void lin_usb_task(void *param);



static struct usb {
	StackType_t usb_device_stack[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t usb_device_stack_mem;
	uint8_t port;
} usb;

static struct lin {
	StackType_t usb_task_stack[configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t usb_task_mem;
	TaskHandle_t usb_task_handle;
	sllin_queue_element rx_fifo[8];
	uint8_t rx_sl_buffer[129]; // power of 2 + 1 to include terminating 0
	uint8_t tx_sl_buffer[129]; // power of 2 + 1 to include terminating 0

	sllin_conf conf;
	int led_state;
	uint8_t rx_sl_offset;
	uint8_t tx_sl_offset;
	uint8_t rx_fifo_gi; // not an index, uses full range of type
	uint8_t rx_fifo_pi; // not an index, uses full range of type

	bool tx_full;
	uint8_t rx_full;
	bool enabled;
	bool report_time_per_frame;
	bool report_time_periodically;

#if SLLIN_DEBUG
	int32_t last_ts;
#endif
} lins[SLLIN_BOARD_LIN_COUNT];

struct sllin_frame_data sllin_frame_data[SLLIN_BOARD_LIN_COUNT];


static inline void clear_rx_fifo(uint8_t index)
{
	struct lin *lin = NULL;

	lin = &lins[index];

	uint8_t pi = __atomic_load_n(&lin->rx_fifo_pi, __ATOMIC_ACQUIRE);
	__atomic_store_n(&lin->rx_fifo_gi, pi, __ATOMIC_RELAXED);
}


static void reset_channel_state(uint8_t index)
{
	struct lin *lin = NULL;
	struct sllin_frame_data *fd = NULL;

	lin = &lins[index];
	fd = &sllin_frame_data[index];

	lin->rx_sl_offset = 0;
	lin->tx_sl_offset = 0;
	lin->enabled = false;
	lin->tx_full = false;
	lin->rx_full = 0;
	lin->report_time_per_frame = false;
	lin->report_time_periodically = false;
#if SLLIN_DEBUG
	lin->last_ts = -1;
#endif

	clear_rx_fifo(index);

	// clear frame data
	memset(fd->len, 0, sizeof(fd->len));
}


int main(void)
{
	// no uart here :(
	sllin_board_init_begin();
	LOG("sllin_board_init_begin\n");

	LOG("led_init\n");
	led_init();


	LOG("tusb_init\n");
	tusb_init();

	(void) xTaskCreateStatic(
		&tusb_device_task,
		"tusb",
		TU_ARRAY_SIZE(usb.usb_device_stack),
		NULL,
		SLLIN_TASK_PRIORITY,
		usb.usb_device_stack,
		&usb.usb_device_stack_mem);
	(void) xTaskCreateStatic(
		&led_task,
		"led",
		TU_ARRAY_SIZE(led_task_stack),
		NULL,
		SLLIN_TASK_PRIORITY,
		led_task_stack,
		&led_task_mem);


	sllin_board_init_end();
	LOG("sllin_board_init_end\n");

	for (unsigned i = 0; i < SLLIN_BOARD_LIN_COUNT; ++i) {
		struct lin *lin = &lins[i];

		lin->usb_task_handle = xTaskCreateStatic(
								&lin_usb_task,
								NULL,
								TU_ARRAY_SIZE(lin->usb_task_stack),
								(void*)(uintptr_t)i,
								SLLIN_TASK_PRIORITY,
								lin->usb_task_stack,
								&lin->usb_task_mem);

		lin->conf.master = false;
		lin->conf.sleep_timeout_ms = 4000;
		lin->conf.bitrate = 19200;

		reset_channel_state(i);
	}

	LOG("vTaskStartScheduler\n");
	vTaskStartScheduler();

	LOG("sllin_board_reset\n");
	sllin_board_reset();


	return 0;
}

SLLIN_RAMFUNC static inline void set_led(uint8_t index, uint8_t state)
{
	struct lin *lin = NULL;

	lin = &lins[index];

	if (unlikely(lin->led_state != state)) {
		lin->led_state = state;
		sllin_board_led_lin_status_set(index, state);
	}
}

SLLIN_RAMFUNC static inline uint8_t char_to_nibble(char c)
{
	if (likely(c >= '0' && c <= '9')) {
		return c - '0';
	}

	if (c >= 'a' && c <= 'f') {
		return (c - 'a') + 0xa;
	}

	return ((c - 'A') & 0xf) + 0xa;

}

SLLIN_RAMFUNC static inline char nibble_to_char(uint8_t nibble)
{
	return "0123456789abcdef"[nibble & 0xf];
}

static inline void channel_off(uint8_t index)
{
	sllin_board_lin_uninit(index);
	reset_channel_state(index);
	set_led(index, SLLIN_LIN_LED_STATUS_DISABLED);
}

static inline void channels_off(void)
{
	for (size_t i = 0; i < TU_ARRAY_SIZE(lins); ++i) {
		channel_off((uint8_t)i);
	}
}

SLLIN_RAMFUNC static inline void sllin_make_time_stamp_string(char *buffer, size_t size, uint16_t time_stamp_ms)
{
	SLLIN_DEBUG_ASSERT(size >= 5);
	(void) size;

	buffer[4] = 0;
	buffer[3] = nibble_to_char(time_stamp_ms);
	time_stamp_ms >>= 4;
	buffer[2] = nibble_to_char(time_stamp_ms);
	time_stamp_ms >>= 4;
	buffer[1] = nibble_to_char(time_stamp_ms);
	time_stamp_ms >>= 4;
	buffer[0] = nibble_to_char(time_stamp_ms);
}

SLLIN_RAMFUNC static inline void sllin_store_tx_queue_full_error_response(uint8_t index)
{
	static const char can_crc_error_frame[] = "\x07T2000000480002000000000000"; // CAN_ERR_CRTL_TX_OVERFLOW
	struct lin *lin = NULL;

	lin = &lins[index];
	lin->tx_sl_offset = sizeof(can_crc_error_frame) - 1;
	memcpy(lin->tx_sl_buffer, can_crc_error_frame, lin->tx_sl_offset);

	if (lin->report_time_per_frame) {
		uint16_t ts = sllin_time_stamp_ms();

		sllin_make_time_stamp_string((char*)&lin->tx_sl_buffer[lin->tx_sl_offset], sizeof(lin->tx_sl_buffer) - lin->tx_sl_offset, ts);
		lin->tx_sl_offset += 4;
	}

	lin->tx_sl_buffer[lin->tx_sl_offset++] = SLLIN_OK_TERMINATOR;
}

SLLIN_RAMFUNC static void sllin_process_command(uint8_t index)
{
	// http://www.can232.com/docs/canusb_manual.pdf
	struct lin *lin = &lins[index];
	struct sllin_frame_data *fd = &sllin_frame_data[index];

	lin->tx_sl_buffer[0] = SLLIN_OK_TERMINATOR;
	lin->tx_sl_offset = 1;

	// LOG("ch%u rx: %s\n", index, lin->rx_sl_buffer);

	switch (lin->rx_sl_buffer[0]) {
	case '\n':
		lin->tx_sl_offset = 0;
		break;
	case 'S': // CAN bitrate, values 0-8
		if (lin->enabled) {
			LOG("ch%u refusing to configure LIN when open\n", index);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			if (unlikely(lin->rx_sl_offset < 2)) {
				LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				unsigned seconds = lin->rx_sl_buffer[1] - '0';

				seconds = tu_min8(seconds, 6);

				lin->conf.sleep_timeout_ms = (4 + seconds) * 1000;
			}
		}
		break;
	case 's': // BTR0/BTR1 used to set bitrate
		if (unlikely(lin->rx_sl_offset < 4)) {
			LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			if (lin->enabled) {
				LOG("ch%u refusing to configure LIN when open\n", index);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			} else {
				uint16_t bitrate = (char_to_nibble(lin->rx_sl_buffer[1]) << 12)
								| (char_to_nibble(lin->rx_sl_buffer[2]) << 8)
								| (char_to_nibble(lin->rx_sl_buffer[3]) << 4)
								| (char_to_nibble(lin->rx_sl_buffer[4]) << 0);

				if (lin->conf.bitrate <= 0)  {
					LOG("ch%u zero bitrate is invalid\n", index);
					lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
				} else {
					lin->conf.bitrate = bitrate;
					LOG("ch%u bitrate=%u\n", index, lin->conf.bitrate);
				}
			}
		}
		break;
	case 'L': // slave
	case 'O': // master
		clear_rx_fifo(index);
		lin->enabled = true;
		lin->conf.master = lin->rx_sl_buffer[0] == 'O';
		set_led(index, SLLIN_LIN_LED_STATUS_ON_BUS_AWAKE_PASSIVE);
		sllin_board_lin_init(index, &lin->conf);
		LOG("ch%u open %s bitrate=%u sleep timeout=%u [ms]\n", index, lin->conf.master ? "master" : "slave", lin->conf.bitrate, lin->conf.sleep_timeout_ms);
		break;
	case 'C':
		LOG("ch%u close\n", index);
		lin->enabled = false;
		tud_cdc_n_write_clear(index);
		sllin_board_lin_uninit(index);
		set_led(index, SLLIN_LIN_LED_STATUS_ENABLED_OFF_BUS);
		break;
	case 'r': // master: tx header
		if (likely(lin->enabled)) {
			if (likely(lin->rx_sl_offset >= 5)) {
				if (likely(lin->conf.master)) {
					uint_least16_t can_id = 0;

					can_id <<= 4;
					can_id |= char_to_nibble(lin->rx_sl_buffer[1]);
					can_id <<= 4;
					can_id |= char_to_nibble(lin->rx_sl_buffer[2]);
					can_id <<= 4;
					can_id |= char_to_nibble(lin->rx_sl_buffer[3]);

					// LOG("ch%u %s -> id=%x len=%u flags=%x\n", index, lin->rx_sl_buffer, id, frame_len, flags);

					if (SLLIN_ID_FLAG_BUS_BREAK & can_id) {
						if (likely(sllin_board_lin_master_break(index))) {
							lin->tx_sl_buffer[0] = 'z';
							lin->tx_sl_buffer[1] = SLLIN_OK_TERMINATOR;
							lin->tx_sl_offset = 2;
						} else {
							sllin_store_tx_queue_full_error_response(index);
						}
					} else {
						uint8_t const id = can_id & 0x3f;

						if (likely(sllin_board_lin_master_request(index, id))) {
							lin->tx_sl_buffer[0] = 'z';
							lin->tx_sl_buffer[1] = SLLIN_OK_TERMINATOR;
							lin->tx_sl_offset = 2;
						} else {
							sllin_store_tx_queue_full_error_response(index);
						}
					}
				} else {
					LOG("ch%u refusing to transmit in slave mode\n", index);
					lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
				}
			} else {
				LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			}
		} else {
			LOG("ch%u refusing to transmit / store reponses when closed\n", index);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		}
		break;
	case 'T': {
		const unsigned MIN_LEN = 10;

		if (likely(lin->enabled)) {
			if (likely(lin->rx_sl_offset >= 9)) {
				uint32_t eff_id = 0;
				uint8_t len = 0;
				uint8_t lin_id = 0;

				// setup proper response
				lin->tx_sl_buffer[0] = 'Z';
				lin->tx_sl_buffer[1] = SLLIN_OK_TERMINATOR;
				lin->tx_sl_offset = 2;

				eff_id |= char_to_nibble(lin->rx_sl_buffer[1]);
				eff_id <<= 4;
				eff_id |= char_to_nibble(lin->rx_sl_buffer[2]);
				eff_id <<= 4;
				eff_id |= char_to_nibble(lin->rx_sl_buffer[3]);
				eff_id <<= 4;
				eff_id |= char_to_nibble(lin->rx_sl_buffer[4]);
				eff_id <<= 4;
				eff_id |= char_to_nibble(lin->rx_sl_buffer[5]);
				eff_id <<= 4;
				eff_id |= char_to_nibble(lin->rx_sl_buffer[6]);
				eff_id <<= 4;
				eff_id |= char_to_nibble(lin->rx_sl_buffer[7]);
				eff_id <<= 4;
				eff_id |= char_to_nibble(lin->rx_sl_buffer[8]);

				lin_id = eff_id & 0x3f;

				len = char_to_nibble(lin->rx_sl_buffer[9]);

				if (eff_id & SLLIN_ID_FLAG_FRAME_STORE) {
					if (likely(len <= 8)) {
						if (likely(len * 2 + MIN_LEN >= lin->rx_sl_offset)) {
							unsigned const crc_comp = (eff_id >> SLLIN_ID_FLAG_FRAME_CRC_COMP_SHIFT) & SLLIN_ID_FLAG_FRAME_CRC_COMP_MASK;
							uint8_t *data = fd->data[lin_id];

							for (unsigned i = 0, j = MIN_LEN; i < len; ++i, j += 2) {
								uint8_t byte = (char_to_nibble(lin->rx_sl_buffer[j]) << 4) | char_to_nibble(lin->rx_sl_buffer[j+1]);

								data[i] = byte;
							}

							fd->len[lin_id] = len;
							LOG("ch%u id=%x len=%u store data=%s\n", index, lin_id, len, &lin->rx_sl_buffer[MIN_LEN]);

							switch (crc_comp) {
							case SLLIN_ID_FLAG_FRAME_CRC_COMP_NONE:
								fd->crc[lin_id] = (eff_id >> SLLIN_ID_FLAG_CRC_SHIFT) & SLLIN_ID_FLAG_CRC_MASK;
								LOG("ch%u id=%x store crc=%x\n", index, lin_id, fd->crc[lin_id]);
								break;
							case SLLIN_ID_FLAG_FRAME_CRC_COMP_CLASSIC: {
								sllin_crc_t crc = sllin_crc_start();
								crc = sllin_crc_update(crc, data, len);
								fd->crc[lin_id] = sllin_crc_finalize(crc);
								LOG("ch%u id=%x compute classic crc=%x\n", index, lin_id, fd->crc[lin_id]);
							} break;
							case SLLIN_ID_FLAG_FRAME_CRC_COMP_ENHANCED: {
								sllin_crc_t crc = sllin_crc_start();
								crc = sllin_crc_update1(crc, sllin_id_to_pid(lin_id));
								crc = sllin_crc_update(crc, data, len);
								fd->crc[lin_id] = sllin_crc_finalize(crc);
								LOG("ch%u id=%x compute enhanced crc=%x\n", index, lin_id, fd->crc[lin_id]);
							} break;
							}
						} else {
							LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
							lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
							lin->tx_sl_offset = 1;
						}
					} else {
						LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
						lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
						lin->tx_sl_offset = 1;
					}
				}

				sllin_board_lin_slave_respond(index, lin_id, (eff_id & SLLIN_ID_FLAG_FRAME_ENABLE) == SLLIN_ID_FLAG_FRAME_ENABLE);
				LOG("ch%u id=%x response %s\n", index, lin_id, (eff_id & SLLIN_ID_FLAG_FRAME_ENABLE) == SLLIN_ID_FLAG_FRAME_ENABLE ? "enabled" : "disabled");
			} else {
				LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
				lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
				lin->tx_sl_offset = 1;
			}
		} else {
			LOG("ch%u refusing to accept frame meta data when closed\n", index);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
			lin->tx_sl_offset = 1;
		}
	} break;
	case 'F': {
		uint8_t flags = 0;
		uint8_t rx_full = 0;
#if defined(HAVE_ATOMIC_COMPARE_EXCHANGE) && HAVE_ATOMIC_COMPARE_EXCHANGE
		// NOT supported on Cortex-M0
		rx_full = __atomic_exchange_n(&lin->rx_full, 0, __ATOMIC_ACQ_REL);
#else
		taskENTER_CRITICAL();
		rx_full = lin->rx_full;
		lin->rx_full = 0;
		taskEXIT_CRITICAL();
#endif
		if (rx_full) {
			flags |= 0x1;
		}

		if (lin->tx_full) {
			lin->tx_full = false;
			flags |= 0x2;
		}

		lin->tx_sl_buffer[0] = 'F';
		lin->tx_sl_buffer[1] = nibble_to_char(flags >> 4);
		lin->tx_sl_buffer[2] = nibble_to_char(flags);
		lin->tx_sl_buffer[3] = SLLIN_OK_TERMINATOR;
		lin->tx_sl_offset = 4;
	} break;
	case 'V': { // HW version?
		lin->tx_sl_buffer[0] = 'V';
		lin->tx_sl_buffer[1] = '0';
		lin->tx_sl_buffer[2] = '1';
		lin->tx_sl_buffer[3] = '0';
		lin->tx_sl_buffer[4] = '0';
		lin->tx_sl_buffer[5] = SLLIN_OK_TERMINATOR;
		lin->tx_sl_offset = 6;
	} break;
	case 'v': { // firmware version
		lin->tx_sl_offset = usnprintf((char*)lin->tx_sl_buffer, sizeof(lin->tx_sl_buffer), "v%s%c", SLLIN_VERSION_STR, SLLIN_OK_TERMINATOR);
	} break;
	case 'n': {
		size_t capa_len = sizeof(lin->tx_sl_buffer) - 2;

		usb_get_desc_string(4 + index, (char*)&lin->tx_sl_buffer[1], &capa_len);

		LOG("ch%u name: %s\n", index, &lin->tx_sl_buffer[1]);

		lin->tx_sl_buffer[0] = 'n';
		lin->tx_sl_buffer[1 + capa_len] = SLLIN_OK_TERMINATOR;
		lin->tx_sl_offset = 2 + capa_len;
	} break;
	case 'N': {
		uint32_t id = sllin_board_identifier();
		id = ((id >> 16) & 0xffff) ^ (id & 0xffff);
		lin->tx_sl_offset = usnprintf((char *)lin->tx_sl_buffer, sizeof(lin->tx_sl_buffer), "N%x%c", id, SLLIN_OK_TERMINATOR);
	} break;
	case 'Z':
		if (lin->enabled) {
			LOG("ch%u refusing to change time stamp setting when enabled\n", index);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else if (lin->rx_sl_offset < 2) {
			LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			lin->report_time_per_frame = !(lin->rx_sl_buffer[1] == '0');
			LOG("ch%u report time stamp per frame %u\n", index, lin->report_time_per_frame);
		}
		break;
	case 'Y':
		if (lin->rx_sl_offset < 2) {
			LOG("ch%u malformed command '%s'\n", index, lin->rx_sl_buffer);
			lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		} else {
			lin->report_time_periodically = !(lin->rx_sl_buffer[1] == '0');
			LOG("ch%u report time stamp periodically %u\n", index, lin->report_time_periodically);
		}
		break;
	default:
		LOG("ch%u unhandled command '%s'\n", index, lin->rx_sl_buffer);
		lin->tx_sl_buffer[0] = SLLIN_ERROR_TERMINATOR;
		break;
	}
}

SLLIN_RAMFUNC static inline void process_rx_frame(uint8_t index, sllin_queue_element *e)
{
	struct lin *lin = &lins[index];
	uint32_t id = e->frame.id;
	uint8_t led_state = 0;

	SLLIN_DEBUG_ASSERT(e->frame.len <= 8);

	if (id & SLLIN_ID_FLAG_BUS_STATE_FLAG) {
		switch ((id & SLLIN_ID_FLAG_BUS_STATE_MASK) >> SLLIN_ID_FLAG_BUS_STATE_SHIFT) {
		case SLLIN_ID_FLAG_BUS_STATE_ASLEEP:
			led_state = SLLIN_LIN_LED_STATUS_ON_BUS_SLEEPING;
			break;
		case SLLIN_ID_FLAG_BUS_STATE_AWAKE:
			led_state = SLLIN_LIN_LED_STATUS_ON_BUS_AWAKE_PASSIVE;
			break;
		default:
			led_state = SLLIN_LIN_LED_STATUS_ERROR;
			break;
		}
	} else {
		led_state = SLLIN_LIN_LED_STATUS_ON_BUS_AWAKE_ACTIVE;
	}

	set_led(index, led_state);

	// EFF
	unsigned bytes = 0;

	lin->tx_sl_buffer[lin->tx_sl_offset++] = 'T';
	bytes = e->frame.len;


	lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 28);
	lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 24);
	lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 20);
	lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 16);
	lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 12);
	lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 8);
	lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id >> 4);
	lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(id);
	lin->tx_sl_buffer[lin->tx_sl_offset++] = '0' + e->frame.len;

	for (unsigned i = 0; i < bytes; ++i) {
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(e->frame.data[i] >> 4);
		lin->tx_sl_buffer[lin->tx_sl_offset++] = nibble_to_char(e->frame.data[i]);
	}

	if (lin->report_time_per_frame) {
		sllin_make_time_stamp_string((char*)&lin->tx_sl_buffer[lin->tx_sl_offset], sizeof(lin->tx_sl_buffer) - lin->tx_sl_offset, e->time_stamp_ms);
		lin->tx_sl_offset += 4;
	}

	lin->tx_sl_buffer[lin->tx_sl_offset++] = SLLIN_OK_TERMINATOR;
}

SLLIN_RAMFUNC static inline void process_queue(uint8_t index, sllin_queue_element *e)
{
	struct lin *lin = &lins[index];

	switch (e->type) {
	case SLLIN_QUEUE_ELEMENT_TYPE_FRAME:
		process_rx_frame(index, e);
		break;
	case SLLIN_QUEUE_ELEMENT_TYPE_TIME_STAMP: {
		if (lin->report_time_periodically) {
			lin->tx_sl_buffer[lin->tx_sl_offset++] = 'Y';
			sllin_make_time_stamp_string((char*)&lin->tx_sl_buffer[lin->tx_sl_offset], sizeof(lin->tx_sl_buffer) - lin->tx_sl_offset, e->time_stamp_ms);
			lin->tx_sl_offset += 4;
			lin->tx_sl_buffer[lin->tx_sl_offset++] = SLLIN_OK_TERMINATOR;
		}
	} break;
	default:
		LOG("unhandled queue element type=%x\n", e->type);
		SLLIN_ASSERT(false && "unhandled queue element type");
		break;
	}
}

SLLIN_RAMFUNC static void tusb_device_task(void* param)
{
	(void) param;

	while (1) {
		LOG("tud_task\n");
		tud_task();
	}
}

SLLIN_RAMFUNC static void lin_usb_task(void* param)
{
	const uint8_t index = (uint8_t)(uintptr_t)param;
	SLLIN_ASSERT(index < TU_ARRAY_SIZE(lins));

	LOG("ch%u task start\n", index);

	struct lin *lin = &lins[index];
	bool written = false;
	bool connected = false;

	while (42) {
		bool yield = false;

		(void)ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		for (bool done = false; !done; ) {
			done = true;

			if (tud_cdc_n_connected(index)) {
				// int count = tud_cdc_n_available(index);
				// LOG("ch%u count=%d\n", index, count);
				int c = tud_cdc_n_read_char(index);
				uint8_t gi = lin->rx_fifo_gi;
				uint8_t pi = __atomic_load_n(&lin->rx_fifo_pi, __ATOMIC_ACQUIRE);

				connected = true;

				if (-1 == c) {
					yield = true;
				} else {
					done = false;

					if (SLLIN_OK_TERMINATOR == c && lin->rx_sl_offset) {
						lin->rx_sl_buffer[lin->rx_sl_offset] = 0;
						sllin_process_command(index);
						lin->rx_sl_offset = 0;

						if (lin->tx_sl_offset) {
							uint32_t w = tud_cdc_n_write(index, lin->tx_sl_buffer, lin->tx_sl_offset);

							if (unlikely(w != lin->tx_sl_offset)) {
								lin->tx_full = true;
								LOG("ch%u TXF\n", index);
							}

							lin->tx_sl_offset = 0;

							if (!written) {
								xTaskNotifyGive(lin->usb_task_handle);
								written = true;
							}
						}
					} else {
						lin->rx_sl_buffer[lin->rx_sl_offset] = c;
						lin->rx_sl_offset = (lin->rx_sl_offset + 1) % (TU_ARRAY_SIZE(lin->rx_sl_buffer) - 1);
					}
				}

				if (pi != gi) {
					uint8_t fifo_index = gi % TU_ARRAY_SIZE(lin->rx_fifo);
					sllin_queue_element *e = &lin->rx_fifo[fifo_index];

					process_queue(index, e);

					if (likely(lin->tx_sl_offset)) {
						uint32_t w = tud_cdc_n_write(index, lin->tx_sl_buffer, lin->tx_sl_offset);

						if (unlikely(w != lin->tx_sl_offset)) {
							lin->tx_full = true;
							LOG("ch%u TXF\n", index);
						}

						lin->tx_sl_offset = 0;

						if (!written) {
							xTaskNotifyGive(lin->usb_task_handle);
							written = true;
						}
					}

					__atomic_store_n(&lin->rx_fifo_gi, gi + 1, __ATOMIC_RELEASE);

					done = false;
				}

				if (yield && written) {
					written = false;
					tud_cdc_n_write_flush(index);
					// LOG("ch%u tx flush\n", index);
				}
			} else {
				if (connected) {
					LOG("ch%u TT\n", index);

					channel_off(index);

					// can't call this if disconnected
					// tud_cdc_n_read_flush(index);
				} else {
					yield = true;
				}

				connected = false;

				// may still get pending events such as the time stamp event
				clear_rx_fifo(index);
			}
		}

		if (yield) {
			// yield to prevent this task from eating up the CPU
			// when the USB buffers are full/busy.
			yield = false;
			// taskYIELD();
			vTaskDelay(pdMS_TO_TICKS(1)); // 1ms for USB FS
			// LOG("+");
		}
	}
}

SLLIN_RAMFUNC void tud_cdc_rx_cb(uint8_t index)
{
	struct lin *lin = &lins[index];

	// LOG("ch%u tud_cdc_rx_cb\n", index);

	xTaskNotifyGive(lin->usb_task_handle);
}


SLLIN_RAMFUNC void tud_cdc_line_state_cb(uint8_t index, bool dtr, bool rts)
{
	struct lin *lin = &lins[index];

	(void) dtr;
	(void) rts;

	// LOG("ch%u tud_cdc_line_state_cb\n", index);

	xTaskNotifyGive(lin->usb_task_handle);
}

SLLIN_RAMFUNC extern void sllin_lin_task_queue(uint8_t index, sllin_queue_element const *element)
{
	struct lin *lin = &lins[index];
	uint8_t pi = __atomic_load_n(&lin->rx_fifo_pi, __ATOMIC_RELAXED);
	uint8_t gi = __atomic_load_n(&lin->rx_fifo_gi, __ATOMIC_ACQUIRE);
	uint8_t used = pi - gi;

	if (likely(used < TU_ARRAY_SIZE(lin->rx_fifo))) {
		uint8_t fifo_index = pi % TU_ARRAY_SIZE(lin->rx_fifo);

		lin->rx_fifo[fifo_index] = *element;

		__atomic_store_n(&lin->rx_fifo_pi, pi + 1, __ATOMIC_RELEASE);
	} else {
#if defined(HAVE_ATOMIC_COMPARE_EXCHANGE) && HAVE_ATOMIC_COMPARE_EXCHANGE
		__atomic_store_n(&lin->rx_full, 1, __ATOMIC_RELEASE);
#else
		taskENTER_CRITICAL();
		lin->rx_full = 1;
		taskEXIT_CRITICAL();
#endif
	}
}


SLLIN_RAMFUNC extern void sllin_lin_task_notify_def(uint8_t index, uint32_t count)
{
	struct lin *lin = &lins[index];

	for (uint32_t i = 0; i < count; ++i) {
		xTaskNotifyGive(lin->usb_task_handle);
	}
}

SLLIN_RAMFUNC extern void sllin_lin_task_notify_isr(uint8_t index, uint32_t count)
{
	struct lin *lin = &lins[index];
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (likely(count)) {
		for (uint32_t i = 1; i < count; ++i) {
			vTaskNotifyGiveFromISR(lin->usb_task_handle, NULL);
		}

		// LOG("ch%u notify\n", index);
		vTaskNotifyGiveFromISR(lin->usb_task_handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

uint16_t _sllin_time_stamp_ms;

SLLIN_RAMFUNC extern void vApplicationTickHook(void)
{
	_Static_assert(1000 == configTICK_RATE_HZ, "fix this function");

	uint16_t now = __atomic_load_n(&_sllin_time_stamp_ms, __ATOMIC_ACQUIRE);

	if (unlikely((now % 1024) == 0)) {
		sllin_queue_element e;
		e.type = SLLIN_QUEUE_ELEMENT_TYPE_TIME_STAMP;
		e.time_stamp_ms = now;

		for (uint8_t i = 0; i < TU_ARRAY_SIZE(lins); ++i) {
			sllin_lin_task_queue(i, &e);
			sllin_lin_task_notify_isr(i, 1);
		}
	}

	// update, wrap around at 1 [s] (60000 [ms])
	if (unlikely(now == 0xEA5F)) {
		__atomic_store_n(&_sllin_time_stamp_ms, 0, __ATOMIC_RELEASE);
	} else {
		__atomic_store_n(&_sllin_time_stamp_ms, now + 1, __ATOMIC_RELEASE);
	}
}


void tud_mount_cb(void)
{
	LOG("mounted\n");
	led_blink(0, 250);
}

void tud_umount_cb(void)
{
	LOG("unmounted\n");
	led_blink(0, 1000);
	channels_off();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	LOG("suspend\n");
	led_blink(0, 500);
	channels_off();
}

void tud_resume_cb(void)
{
	LOG("resume\n");
	led_blink(0, 250);
}

#if CFG_TUD_DFU_RUNTIME

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

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
	LOG("port=%u stage=%u\n", rhport, stage);

	switch (stage) {
	case CONTROL_STAGE_SETUP: {
		switch (request->bRequest) {
		case DFU_VENDOR_REQUEST_MICROSOFT:
			if (request->wIndex == 7) {
				// Get Microsoft OS 2.0 compatible descriptor
				LOG("send MS OS 2.0 compatible descriptor\n");
				uint16_t total_len;
				memcpy(&total_len, desc_ms_os_20 + DFU_MS_OS_20_SUBSET_HEADER_FUNCTION_LEN, 2);
				total_len = tu_le16toh(total_len);
				return tud_control_xfer(rhport, request, (void*)desc_ms_os_20, total_len);
			}
			break;
		default:
			LOG("req type 0x%02x (reci %s type %s dir %s) req 0x%02x, value 0x%04x index 0x%04x reqlen %u\n",
				request->bmRequestType,
				recipient_str(request->bmRequestType_bit.recipient),
				type_str(request->bmRequestType_bit.type),
				dir_str(request->bmRequestType_bit.direction),
				request->bRequest, request->wValue, request->wIndex,
				request->wLength);
			break;
		}
	} break;
	case CONTROL_STAGE_DATA:
	case CONTROL_STAGE_ACK:
		switch (request->bRequest) {
		case DFU_VENDOR_REQUEST_MICROSOFT:
			return true;
		default:
			break;
		}
	default:
		break;
	}

	// stall unknown request
	return false;
}

#endif // CFG_TUD_DFU_RUNTIME
