/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>


#include <bsp/board.h>
#include <tusb.h>
#include <device/usbd_pvt.h>
#include <device/dcd.h>

#include <supercan_board.h>
#include <supercan_m1.h>
#include <usb_descriptors.h>
#include <leds.h>


enum {
	CAN_STATUS_FIFO_SIZE = 256,
};

#define SPAM 0
// #define XFER dcd_edpt_xfer
// #define OPEN dcd_edpt_open
#define XFER(...) \
	do { \
		bool r = usbd_edpt_xfer(__VA_ARGS__); \
		SC_DEBUG_ASSERT(r); \
		(void)r; \
	} while (0)


#define OPEN usbd_edpt_open


extern void sc_can_log_bit_timing(sc_can_bit_timing const *c, char const* name)
{
	(void)c;
	(void)name;

	LOG("%s clock[mhz]=%u brp=%u sjw=%u tseg1=%u tseg2=%u bitrate=%lu sp=%u/1000\n",
		name, SC_BOARD_CAN_CLK_HZ / 1000000, c->brp, c->sjw, c->tseg1, c->tseg2,
		sc_bitrate(c->brp, c->tseg1, c->tseg2),
		((1 + c->tseg1) * 1000) / (1 + c->tseg1 + c->tseg2)
	);
}




// FIX ME: move to struct usb
static StackType_t usb_device_stack[(3*configMINIMAL_SECURE_STACK_SIZE)/2];
static StaticTask_t usb_device_task_mem;

static void tusb_device_task(void* param);



struct usb_can {
	CFG_TUSB_MEM_ALIGN uint8_t tx_buffers[2][MSG_BUFFER_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t rx_buffers[2][MSG_BUFFER_SIZE];
	StaticSemaphore_t mutex_mem;
	SemaphoreHandle_t mutex_handle;
	uint16_t tx_offsets[2];
	uint8_t tx_bank;
	uint8_t rx_bank;
	uint8_t pipe;
};

struct usb_cmd {
	CFG_TUSB_MEM_ALIGN uint8_t tx_buffers[2][CMD_BUFFER_SIZE];
	CFG_TUSB_MEM_ALIGN uint8_t rx_buffers[2][CMD_BUFFER_SIZE];
	uint16_t tx_offsets[2];
	uint8_t tx_bank;
	uint8_t rx_bank;
	uint8_t pipe;
};


static struct usb {
	struct usb_cmd cmd[SC_BOARD_CAN_COUNT];
	struct usb_can can[SC_BOARD_CAN_COUNT];
	uint8_t port;
	bool mounted;
} usb;

static struct can {
	sc_can_status status_fifo[CAN_STATUS_FIFO_SIZE];
	StackType_t usb_task_stack_mem[3*configMINIMAL_SECURE_STACK_SIZE];
	StaticTask_t usb_task_mem;
	TaskHandle_t usb_task_handle;

	uint16_t features;
	uint16_t tx_dropped; // summed for until next SC_MSG_CAN_STATUS
	uint16_t rx_lost;    // summed for until next SC_MSG_CAN_STATUS
	uint16_t status_get_index; // NOT an index, uses full range of type
	uint16_t status_put_index; // NOT an index, uses full range of type
	uint8_t int_comm_flags;
	bool enabled;
	bool desync;
} cans[SC_BOARD_CAN_COUNT];


SC_RAMFUNC extern void sc_can_status_queue(uint8_t index, sc_can_status const *status)
{
	struct can *can = &cans[index];
	uint16_t pi = can->status_put_index;
	uint16_t gi = __atomic_load_n(&can->status_get_index, __ATOMIC_ACQUIRE);
	uint16_t used = pi - gi;

	if (likely(used < CAN_STATUS_FIFO_SIZE)) {
		uint16_t fifo_index = pi & (CAN_STATUS_FIFO_SIZE-1);
		can->status_fifo[fifo_index] = *status;
		__atomic_store_n(&can->status_put_index, pi + 1, __ATOMIC_RELEASE);
	} else {
#if defined(HAVE_ATOMIC_COMPARE_EXCHANGE) && HAVE_ATOMIC_COMPARE_EXCHANGE
		__sync_or_and_fetch(&can->int_comm_flags, SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL);
#else
		taskENTER_CRITICAL();
		can->int_comm_flags |= SC_CAN_STATUS_FLAG_IRQ_QUEUE_FULL;
		taskEXIT_CRITICAL();
#endif
	}
}

static inline void can_state_reset(uint8_t index)
{
	struct can *can = &cans[index];
	struct usb_can *usb_can = &usb.can[index];

	can->desync = false;
	can->tx_dropped = 0;
	can->rx_lost = 0;
	__atomic_store_n(&can->status_get_index, __atomic_load_n(&can->status_put_index, __ATOMIC_ACQUIRE), __ATOMIC_RELAXED);
	__atomic_store_n(&can->int_comm_flags, 0, __ATOMIC_RELAXED);

	usb_can->tx_offsets[0] = 0;
	usb_can->tx_offsets[1] = 0;
}

static inline void can_state_initial(uint8_t index)
{
	struct can *can = &cans[index];
	struct usb_cmd *cmd = &usb.cmd[index];

	cmd->tx_offsets[0] = 0;
	cmd->tx_offsets[1] = 0;

	can->enabled = false;
	can->features = sc_board_can_feat_perm(index);

	can_state_reset(index);
	sc_board_can_reset(index);
	sc_board_led_can_status_set(index, SC_CAN_LED_STATUS_ENABLED_OFF_BUS);
}

static inline bool sc_cmd_bulk_in_ep_ready(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.cmd));
	struct usb_cmd *cmd = &usb.cmd[index];
	return 0 == cmd->tx_offsets[!cmd->tx_bank];
}

static inline void sc_cmd_bulk_in_submit(uint8_t index)
{
	SC_DEBUG_ASSERT(sc_cmd_bulk_in_ep_ready(index));
	struct usb_cmd *cmd = &usb.cmd[index];
	SC_DEBUG_ASSERT(cmd->tx_offsets[cmd->tx_bank] > 0);
	SC_DEBUG_ASSERT(cmd->tx_offsets[cmd->tx_bank] <= CMD_BUFFER_SIZE);
	XFER(usb.port, 0x80 | cmd->pipe, cmd->tx_buffers[cmd->tx_bank], cmd->tx_offsets[cmd->tx_bank]);
	cmd->tx_bank = !cmd->tx_bank;
}

SC_RAMFUNC static inline bool sc_can_bulk_in_ep_ready(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	struct usb_can *can = &usb.can[index];
	return 0 == can->tx_offsets[!can->tx_bank];
}

#if SUPERCAN_DEBUG
static inline void dump_rx_tx_timestamps(uint8_t index)
{
	struct usb_can *can = &usb.can[index];
	uint8_t const * const sptr = can->tx_buffers[can->tx_bank];
	uint8_t const * const eptr = sptr + can->tx_offsets[can->tx_bank];
	uint8_t const *ptr = sptr;
	unsigned rx_offset = 0;
	unsigned tx_offset = 0;

	for (; ptr + SC_MSG_HEADER_LEN <= eptr; ) {
		struct sc_msg_header *hdr = (struct sc_msg_header *)ptr;

		switch (hdr->id) {
		case SC_MSG_CAN_RX: {
			struct sc_msg_can_rx const *msg = (struct sc_msg_can_rx const *)hdr;

			LOG("ch%u rx msg index=%02u ts=%08x\n", index, rx_offset, msg->timestamp_us);
			++rx_offset;
		} break;
		}

		ptr += hdr->len;
	}

	ptr = sptr;

	for (; ptr + SC_MSG_HEADER_LEN <= eptr; ) {
		struct sc_msg_header *hdr = (struct sc_msg_header *)ptr;

		switch (hdr->id) {
		case SC_MSG_CAN_TXR: {
			struct sc_msg_can_txr const *msg = (struct sc_msg_can_txr const *)hdr;

			LOG("ch%u txr msg index=%02u ts=%08x\n", index, tx_offset, msg->timestamp_us);
			++tx_offset;
		} break;
		}

		ptr += hdr->len;
	}
}
#endif

SC_RAMFUNC static inline void sc_can_bulk_in_submit(uint8_t index, char const *func)
{
	SC_DEBUG_ASSERT(sc_can_bulk_in_ep_ready(index));
	struct usb_can *can = &usb.can[index];
	SC_DEBUG_ASSERT(can->tx_bank < 2);
	SC_DEBUG_ASSERT(can->tx_offsets[can->tx_bank] > 0);

	(void)func;

	// LOG("ch%u %s: %u bytes\n", index, func, can->tx_offsets[can->tx_bank]);
	//LOG("%c", '>' + index);

#if SUPERCAN_DEBUG

	uint32_t rx_ts_last = 0;
	uint32_t tx_ts_last = 0;
	unsigned rx_offset = 0;
	unsigned tx_offset = 0;


	// LOG("ch%u %s: send %u bytes\n", index, func, can->tx_offsets[can->tx_bank]);
	if (can->tx_offsets[can->tx_bank] > MSG_BUFFER_SIZE) {
		LOG("ch%u %s: msg buffer size %u out of bounds\n", index, func, can->tx_offsets[can->tx_bank]);
		SC_DEBUG_ASSERT(false);
		can->tx_offsets[can->tx_bank] = 0;
		return;
	}

	uint8_t const *sptr = can->tx_buffers[can->tx_bank];
	uint8_t const *eptr = sptr + can->tx_offsets[can->tx_bank];
	uint8_t const *ptr = sptr;

	for (; ptr + SC_MSG_HEADER_LEN <= eptr; ) {
		struct sc_msg_header *hdr = (struct sc_msg_header *)ptr;

		if (!hdr->id || !hdr->len) {
			LOG("ch%u %s msg offset %u zero id/len msg\n", index, func, ptr - sptr);
			// sc_dump_mem(sptr, eptr - sptr);
			SC_DEBUG_ASSERT(false);
			can->tx_offsets[can->tx_bank] = 0;
			return;
		}

		if (hdr->len < SC_MSG_HEADER_LEN) {
			LOG("ch%u %s msg offset %u msg header len %u\n", index, func, ptr - sptr, hdr->len);
			SC_DEBUG_ASSERT(false);
			can->tx_offsets[can->tx_bank] = 0;
			return;
		}

		if (ptr + hdr->len > eptr) {
			LOG("ch%u %s msg offset=%u len=%u exceeds buffer len=%u\n", index, func, (unsigned)(ptr - sptr), hdr->len, MSG_BUFFER_SIZE);
			SC_DEBUG_ASSERT(false);
			can->tx_offsets[can->tx_bank] = 0;
			return;
		}

		if (hdr->len % SC_MSG_CAN_LEN_MULTIPLE) {
			LOG("ch%u %s msg offset=%u len=%u is not a multiple of %u\n", index, func, ptr - sptr, hdr->len, SC_MSG_CAN_LEN_MULTIPLE);
			SC_DEBUG_ASSERT(false);
			can->tx_offsets[can->tx_bank] = 0;
			return;
		}

		switch (hdr->id) {
		case SC_MSG_CAN_STATUS:
			break;
		case SC_MSG_CAN_RX: {
			struct sc_msg_can_rx const *msg = (struct sc_msg_can_rx const *)hdr;
			uint32_t ts = msg->timestamp_us;
			if (rx_ts_last) {
				uint32_t delta = (ts - rx_ts_last) & SC_TS_MAX;
				bool rx_ts_ok = delta <= SC_TS_MAX / 4;
				if (unlikely(!rx_ts_ok)) {
					taskENTER_CRITICAL();
					LOG("ch%u rx msg index=%u ts=%lx prev=%lx\n", index, rx_offset, ts, rx_ts_last);
					dump_rx_tx_timestamps(index);
					SC_ASSERT(false);
					can->tx_offsets[can->tx_bank] = 0;
					taskEXIT_CRITICAL();
					return;
				}
			}
			rx_ts_last = ts;
			++rx_offset;
		} break;
		case SC_MSG_CAN_TXR: {
			struct sc_msg_can_txr const *msg = (struct sc_msg_can_txr const *)hdr;
			uint32_t ts = msg->timestamp_us;
			if (tx_ts_last) {
				uint32_t delta = (ts - tx_ts_last) & SC_TS_MAX;
				bool tx_ts_ok = delta <= SC_TS_MAX / 4;
				if (unlikely(!tx_ts_ok)) {
					taskENTER_CRITICAL();
					LOG("ch%u tx msg index=%u ts=%lx prev=%lx\n", index, tx_offset, ts, tx_ts_last);
					dump_rx_tx_timestamps(index);
					SC_ASSERT(false);
					can->tx_offsets[can->tx_bank] = 0;
					taskEXIT_CRITICAL();
					return;
				}
			}
			tx_ts_last = ts;
			++tx_offset;
		} break;
		case SC_MSG_CAN_ERROR:
			break;
		case 0x42:
			break;
		default:
			LOG("ch%u %s msg offset %u non-device msg id %#02x\n", index, func, ptr - sptr, hdr->id);
			can->tx_offsets[can->tx_bank] = 0;
			return;
		}

		ptr += hdr->len;
	}
#endif

	// Required to immediately send URBs when buffer size > endpoint size
	// and the transfer size is a multiple of the endpoint size, and the
	// tranfer size is smaller than the buffer size, we can either send a zlp
	// or increase the payload size.
	if (MSG_BUFFER_SIZE > SC_M1_EP_SIZE) { // FIX ME -> move to board file
		uint16_t offset = can->tx_offsets[can->tx_bank];
		bool need_to_send_zlp = offset < MSG_BUFFER_SIZE && 0 == (offset % SC_M1_EP_SIZE);

		if (need_to_send_zlp) {
			// LOG("zlpfix\n");
			memset(&can->tx_buffers[can->tx_bank][offset], 0, 4);
			can->tx_offsets[can->tx_bank] += 4;
		}
	}

	// uint16_t rem = can->tx_offsets[can->tx_bank] % 64;

	// if (rem) {
	// 	unsigned bytes = 64 - rem;
	// 	memset(&can->tx_buffers[can->tx_bank][can->tx_offsets[can->tx_bank]], 0, bytes);
	// 	can->tx_offsets[can->tx_bank] += bytes;
	// 	SC_DEBUG_ASSERT(!(can->tx_offsets[can->tx_bank] % 64));
	// 	LOG("ch%u %s align bytes=%u\n", index, func, can->tx_offsets[can->tx_bank]);
	// }

	// LOG("ch%u %s exit bytes=%u\n", index, func, can->tx_offsets[can->tx_bank]);
	XFER(usb.port, 0x80 | can->pipe, can->tx_buffers[can->tx_bank], can->tx_offsets[can->tx_bank]);
	can->tx_bank = !can->tx_bank;
	SC_DEBUG_ASSERT(!can->tx_offsets[can->tx_bank]);

#if SUPERCAN_DEBUG
	memset(can->tx_buffers[can->tx_bank], 0xff, MSG_BUFFER_SIZE);
#endif

	// LOG("ch%u %s sent\n", index, func);
}

static void sc_cmd_bulk_out(uint8_t index, uint32_t xferred_bytes);
static void sc_cmd_bulk_in(uint8_t index);
SC_RAMFUNC static void sc_can_bulk_out(uint8_t index, uint32_t xferred_bytes);
SC_RAMFUNC static void sc_can_bulk_in(uint8_t index);
static void sc_cmd_place_error_reply(uint8_t index, int8_t error);

static void sc_cmd_bulk_out(uint8_t index, uint32_t xferred_bytes)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.cmd));
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans));

	struct usb_cmd *usb_cmd = &usb.cmd[index];
	struct usb_can *usb_can = &usb.can[index];
	struct can *can = &cans[index];


	uint8_t const *in_ptr = usb_cmd->rx_buffers[usb_cmd->rx_bank];
	uint8_t const * const in_end = in_ptr + xferred_bytes;

	// setup next transfer
	usb_cmd->rx_bank = !usb_cmd->rx_bank;
	XFER(usb.port, usb_cmd->pipe, usb_cmd->rx_buffers[usb_cmd->rx_bank], CMD_BUFFER_SIZE);

	// process messages
	while (in_ptr + SC_MSG_HEADER_LEN <= in_end) {
		struct sc_msg_header const *msg = (struct sc_msg_header const *)in_ptr;

		if (in_ptr + msg->len > in_end) {
			LOG("ch%u malformed msg\n", index);
			break;
		}

		if (!msg->len) {
			break;
		}

		in_ptr += msg->len;

		switch (msg->id) {
		case SC_MSG_EOF: {
			LOG("ch%u SC_MSG_EOF\n", index);
			in_ptr = in_end;
		} break;

		case SC_MSG_HELLO_DEVICE: {
			LOG("ch%u SC_MSG_HELLO_DEVICE\n", index);

			// _With_ usb lock, since this isn't the interrupt handler
			// and neither a higher priority task.
			while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));

			can_state_initial(index);

			xSemaphoreGive(usb_can->mutex_handle);

			// transmit empty buffers (clear whatever was in there before)
			XFER(usb.port, 0x80 | usb_can->pipe, usb_can->tx_buffers[usb_can->tx_bank], usb_can->tx_offsets[usb_can->tx_bank]);

			// reset tx buffer
			uint8_t len = sizeof(struct sc_msg_hello);
			usb_cmd->tx_offsets[usb_cmd->tx_bank] = len;
			struct sc_msg_hello *rep = (struct sc_msg_hello *)&usb_cmd->tx_buffers[usb_cmd->tx_bank][0];
			rep->id = SC_MSG_HELLO_HOST;
			rep->len = len;
			rep->proto_version = SC_VERSION;
#if TU_BIG_ENDIAN == TU_BYTE_ORDER
			rep->byte_order = SC_BYTE_ORDER_BE;
#else
			rep->byte_order = SC_BYTE_ORDER_LE;
#endif
			rep->cmd_buffer_size = tu_htons(CMD_BUFFER_SIZE);

			// don't process any more messages
			in_ptr = in_end;

			// assume in token is available
		} break;
		case SC_MSG_DEVICE_INFO: {
			LOG("ch%u SC_MSG_DEVICE_INFO\n", index);
			uint8_t bytes = sizeof(struct sc_msg_dev_info);

			uint8_t *out_ptr;
			uint8_t *out_end;

send_dev_info:
			out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
			out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
			if (out_end - out_ptr >= bytes) {
				struct sc_msg_dev_info *rep = (struct sc_msg_dev_info *)out_ptr;
				const uint32_t device_identifier = sc_board_identifier();

				usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;

				rep->id = SC_MSG_DEVICE_INFO;
				rep->len = bytes;
				rep->feat_perm = sc_board_can_feat_perm(index);
				rep->feat_conf = sc_board_can_feat_conf(index);
				rep->fw_ver_major = SUPERCAN_VERSION_MAJOR;
				rep->fw_ver_minor = SUPERCAN_VERSION_MINOR;
				rep->fw_ver_patch = SUPERCAN_VERSION_PATCH;
				static const char dev_name[] = SC_BOARD_NAME " " SC_NAME " chX";
				rep->name_len = tu_min8(sizeof(dev_name)-1, sizeof(rep->name_bytes));
				memcpy(rep->name_bytes, dev_name, rep->name_len);
				if (rep->name_len <= TU_ARRAY_SIZE(rep->name_bytes)) {
					rep->name_bytes[rep->name_len-1] = '0' + index;
				}

				rep->sn_bytes[0] = (device_identifier >> 24) & 0xff;
				rep->sn_bytes[1] = (device_identifier >> 16) & 0xff;
				rep->sn_bytes[2] = (device_identifier >> 8) & 0xff;
				rep->sn_bytes[3] = (device_identifier >> 0) & 0xff;
				rep->sn_len = 4;
			} else {
				if (sc_cmd_bulk_in_ep_ready(index)) {
					sc_cmd_bulk_in_submit(index);
					goto send_dev_info;
				} else {
					LOG("no space for device info reply\n");
				}
			}
		} break;
		case SC_MSG_CAN_INFO: {
			LOG("ch%u SC_MSG_CAN_INFO\n", index);
			uint8_t bytes = sizeof(struct sc_msg_can_info);

			uint8_t *out_ptr;
			uint8_t *out_end;

send_can_info:
			out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
			out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
			if (out_end - out_ptr >= bytes) {
				struct sc_msg_can_info *rep = (struct sc_msg_can_info *)out_ptr;
				sc_can_bit_timing_range const *nm_bt = sc_board_can_nm_bit_timing_range(index);
				sc_can_bit_timing_range const *dt_bt = sc_board_can_dt_bit_timing_range(index);

				SC_DEBUG_ASSERT(nm_bt);

				if (!dt_bt) {
					dt_bt = nm_bt;
				}

				usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;

				rep->id = SC_MSG_CAN_INFO;
				rep->len = bytes;
				rep->can_clk_hz = SC_BOARD_CAN_CLK_HZ;
				rep->nmbt_brp_min = nm_bt->min.brp;
				rep->nmbt_brp_max = nm_bt->max.brp;
				rep->nmbt_sjw_max = nm_bt->max.sjw;
				rep->nmbt_tseg1_min = nm_bt->min.tseg1;
				rep->nmbt_tseg1_max = nm_bt->max.tseg1;
				rep->nmbt_tseg2_min = nm_bt->min.tseg2;
				rep->nmbt_tseg2_max = nm_bt->max.tseg2;
				rep->dtbt_brp_min = dt_bt->min.brp;
				rep->dtbt_brp_max = dt_bt->max.brp;
				rep->dtbt_sjw_max = dt_bt->max.sjw;
				rep->dtbt_tseg1_min = dt_bt->min.tseg1;
				rep->dtbt_tseg1_max = dt_bt->max.tseg1;
				rep->dtbt_tseg2_min = dt_bt->min.tseg2;
				rep->dtbt_tseg2_max = dt_bt->max.tseg2;
				rep->tx_fifo_size = SC_BOARD_CAN_TX_FIFO_SIZE;
				rep->rx_fifo_size = SC_BOARD_CAN_RX_FIFO_SIZE;
				rep->msg_buffer_size = MSG_BUFFER_SIZE;

				LOG("ch%u clk=%u ", index, SC_BOARD_CAN_CLK_HZ);
				LOG("nm brp=%u..%u sjw=%u..%u tseg1=%u..%u tseg2=%u..%u\n",
					nm_bt->min.brp, nm_bt->max.brp, 1, nm_bt->max.sjw, nm_bt->min.tseg1, nm_bt->max.tseg1, nm_bt->min.tseg2, nm_bt->max.tseg2);
				LOG("dt brp=%u..%u sjw=%u..%u tseg1=%u..%u tseg2=%u..%u\n",
					dt_bt->min.brp, dt_bt->max.brp, 1, dt_bt->max.sjw, dt_bt->min.tseg1, dt_bt->max.tseg1, dt_bt->min.tseg2, dt_bt->max.tseg2);
			} else {
				if (sc_cmd_bulk_in_ep_ready(index)) {
					sc_cmd_bulk_in_submit(index);
					goto send_can_info;
				} else {
					LOG("no space for can info reply\n");
				}
			}
		} break;
		case SC_MSG_NM_BITTIMING: {
			LOG("ch%u SC_MSG_NM_BITTIMING\n", index);
			int8_t error = SC_CAN_ERROR_NONE;
			struct sc_msg_bittiming const *tmsg = (struct sc_msg_bittiming const *)msg;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				error = SC_ERROR_SHORT;
			} else {
				sc_can_bit_timing_range const *nm_bt = sc_board_can_nm_bit_timing_range(index);
				sc_can_bit_timing bt_target;

				// clamp
				bt_target.brp = tu_max16(nm_bt->min.brp, tu_min16(tmsg->brp, nm_bt->max.brp));
				bt_target.sjw = tu_max8(nm_bt->min.sjw, tu_min8(tmsg->sjw, nm_bt->max.sjw));
				bt_target.tseg1 = tu_max16(nm_bt->min.tseg1, tu_min16(tmsg->tseg1, nm_bt->max.tseg1));
				bt_target.tseg2 = tu_max8(nm_bt->min.tseg2, tu_min8(tmsg->tseg2, nm_bt->max.tseg2));

				LOG("ch%u ", index);
				sc_can_log_bit_timing(&bt_target, "nominal");

				sc_board_can_nm_bit_timing_set(index, &bt_target);
			}

			sc_cmd_place_error_reply(index, error);
		} break;
		case SC_MSG_DT_BITTIMING: {
			LOG("ch%u SC_MSG_DT_BITTIMING\n", index);
			int8_t error = SC_CAN_ERROR_NONE;
			struct sc_msg_bittiming const *tmsg = (struct sc_msg_bittiming const *)msg;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				error = SC_ERROR_SHORT;
			} else {
				sc_can_bit_timing_range const *dt_bt = sc_board_can_dt_bit_timing_range(index);
				sc_can_bit_timing bt_target;

				// clamp
				bt_target.brp = tu_max16(dt_bt->min.brp, tu_min16(tmsg->brp, dt_bt->max.brp));
				bt_target.sjw = tu_max8(dt_bt->min.sjw, tu_min8(tmsg->sjw, dt_bt->max.sjw));
				bt_target.tseg1 = tu_max16(dt_bt->min.tseg1, tu_min16(tmsg->tseg1, dt_bt->max.tseg1));
				bt_target.tseg2 = tu_max8(dt_bt->min.tseg2, tu_min8(tmsg->tseg2, dt_bt->max.tseg2));

				LOG("ch%u ", index);
				sc_can_log_bit_timing(&bt_target, "data");

				sc_board_can_dt_bit_timing_set(index, &bt_target);
			}

			sc_cmd_place_error_reply(index, error);
		} break;
		case SC_MSG_FEATURES: {
			LOG("ch%u SC_MSG_FEATURES\n", index);
			struct sc_msg_features const *tmsg = (struct sc_msg_features const *)msg;
			int8_t error = SC_ERROR_NONE;
			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ch%u ERROR: msg too short\n", index);
				error = SC_ERROR_SHORT;
			} else {
				const uint16_t perm = sc_board_can_feat_perm(index);
				const uint16_t conf = sc_board_can_feat_conf(index);

				switch (tmsg->op) {
				case SC_FEAT_OP_CLEAR:
					can->features = perm;
					LOG("ch%u CLEAR features to %#x\n", index, can->features);
					break;
				case SC_FEAT_OP_OR: {
					uint32_t mode_bits = tmsg->arg & (SC_FEATURE_FLAG_MON_MODE | SC_FEATURE_FLAG_RES_MODE | SC_FEATURE_FLAG_EXT_LOOP_MODE);

					if (__builtin_popcount(mode_bits) > 1) {
						error = SC_ERROR_PARAM;
						LOG("ch%u ERROR: attempt to activate more than one mode %08lx\n", index, mode_bits);
					} else if (tmsg->arg & ~(perm | conf)) {
						error = SC_ERROR_UNSUPPORTED;
						LOG("ch%u ERROR: unsupported features %08lx\n", index, tmsg->arg);
					} else {
						can->features |= tmsg->arg;
						LOG("ch%u OR features to %#x\n", index, can->features);
					}
				} break;
				}
			}
			sc_cmd_place_error_reply(index, error);
		} break;
		case SC_MSG_BUS: {
			LOG("ch%u SC_MSG_BUS\n", index);
			struct sc_msg_config const *tmsg = (struct sc_msg_config const *)msg;
			int8_t error = SC_ERROR_NONE;

			if (unlikely(msg->len < sizeof(*tmsg))) {
				LOG("ERROR: msg too short\n");
				error = SC_ERROR_SHORT;
			} else {
				bool was_enabled = can->enabled;
				bool is_enabled = tmsg->arg != 0;

				LOG("ch%u go bus=%d\n", index, is_enabled);

				if (was_enabled != is_enabled) {
					LOG("ch%u go bus %s -> %s\n", index, (was_enabled ? "online" : "offline"), (is_enabled ? "online" : "offline"));
					// _With_ usb lock, since this isn't the interrupt handler
					// and neither a higher priority task.
					while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));

					if (is_enabled) {
						can_state_reset(index);
						sc_board_can_feat_set(index, can->features);
						sc_board_can_go_bus(index, is_enabled);
						sc_board_led_can_status_set(index, SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE);
					} else {
						sc_board_can_go_bus(index, is_enabled);
						can_state_reset(index);
						sc_board_led_can_status_set(index, SC_CAN_LED_STATUS_ENABLED_OFF_BUS);
#if SUPERCAN_DEBUG
						uint8_t *ptr_begin = usb_can->tx_buffers[usb_can->tx_bank];
						uint8_t *ptr_end = ptr_begin + TU_ARRAY_SIZE(usb_can->tx_buffers[usb_can->tx_bank]);
						int error_retrieve = sc_board_can_retrieve(index, ptr_begin, ptr_end);
						(void)error_retrieve;
						// SC_ASSERT(-1 == error_retrieve); // expected impl to not have any messages queued
#endif
						SC_DEBUG_ASSERT(0 == usb_can->tx_offsets[0]);
						SC_DEBUG_ASSERT(0 == usb_can->tx_offsets[1]);
					}

					can->enabled = is_enabled;

					xSemaphoreGive(usb_can->mutex_handle);

					// notify task to make sure it knows
					xTaskNotifyGive(can->usb_task_handle);
				}
			}

			sc_cmd_place_error_reply(index, error);
		} break;
		default:
			TU_LOG2_MEM(msg, msg->len, 2);
			sc_cmd_place_error_reply(index, SC_ERROR_UNSUPPORTED);
			break;
		}
	}

	if (usb_cmd->tx_offsets[usb_cmd->tx_bank] > 0 && sc_cmd_bulk_in_ep_ready(index)) {
		sc_cmd_bulk_in_submit(index);
	}
}

SC_RAMFUNC static void sc_process_msg_can_tx(uint8_t index, struct sc_msg_header const *msg)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	SC_DEBUG_ASSERT(msg);
	SC_DEBUG_ASSERT(SC_MSG_CAN_TX == msg->id);

	struct can *can = &cans[index];
	struct usb_can *usb_can = &usb.can[index];

	// LOG("SC_MSG_CAN_TX %lx\n", __atomic_load_n(&can->sync_tscv, __ATOMIC_ACQUIRE));
	struct sc_msg_can_tx const *tmsg = (struct sc_msg_can_tx const *)msg;
	if (unlikely(msg->len < sizeof(*tmsg))) {
		LOG("ch%u ERROR: SC_MSG_CAN_TX msg too short\n", index);
		return;
	}

	const uint8_t can_frame_len = dlc_to_len(tmsg->dlc);
	if (!(tmsg->flags & SC_CAN_FRAME_FLAG_RTR)) {
		if (unlikely(msg->len < sizeof(*tmsg) + can_frame_len)) {
			LOG("ch%u ERROR: SC_MSG_CAN_TX msg too short\n", index);
			return;
		}
	}

	if (unlikely(tmsg->track_id >= SC_BOARD_CAN_TX_FIFO_SIZE)) {
		LOG("ch%u ERROR: SC_MSG_CAN_TX track ID %u out of bounds [0-%u)\n", index, tmsg->track_id, SC_BOARD_CAN_TX_FIFO_SIZE);
		return;
	}

	if (unlikely(!sc_board_can_tx_queue(index, tmsg))) {
		uint8_t *tx_beg = NULL;
		uint8_t *tx_end = NULL;
		uint8_t *tx_ptr = NULL;
		struct sc_msg_can_txr* rep = NULL;

		sc_board_can_ts_request(index);
		++can->tx_dropped;

send_txr:
		tx_beg = usb_can->tx_buffers[usb_can->tx_bank];
		tx_end = tx_beg + TU_ARRAY_SIZE(usb_can->tx_buffers[usb_can->tx_bank]);
		tx_ptr = tx_beg + usb_can->tx_offsets[usb_can->tx_bank];

		if ((size_t)(tx_end - tx_ptr) >= sizeof(*rep)) {
			usb_can->tx_offsets[usb_can->tx_bank] += sizeof(*rep);

			rep = (struct sc_msg_can_txr*)tx_ptr;
			rep->id = SC_MSG_CAN_TXR;
			rep->len = sizeof(*rep);
			rep->track_id = tmsg->track_id;
			rep->flags = SC_CAN_FRAME_FLAG_DRP;
			uint32_t ts = sc_board_can_ts_wait(index);
			rep->timestamp_us = ts;
		} else {
			if (sc_can_bulk_in_ep_ready(index)) {
				sc_can_bulk_in_submit(index, __func__);
				goto send_txr;
			} else {
				LOG("ch%u: desync\n", index);
				can->desync = true;
			}
		}
	}
}

SC_RAMFUNC static void sc_can_bulk_out(uint8_t index, uint32_t xferred_bytes)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans));

	struct usb_can *usb_can = &usb.can[index];
	struct can *can = &cans[index];
	const uint8_t rx_bank = usb_can->rx_bank;
	(void)rx_bank;
	uint8_t *in_beg = usb_can->rx_buffers[usb_can->rx_bank];
	uint8_t *in_ptr = in_beg;
	uint8_t *in_end = in_ptr + xferred_bytes;


	// start new transfer right away
	usb_can->rx_bank = !usb_can->rx_bank;
	XFER(usb.port, usb_can->pipe, usb_can->rx_buffers[usb_can->rx_bank], MSG_BUFFER_SIZE);

	if (unlikely(!xferred_bytes)) {
		return;
	}

	while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));


	// guard against CAN frames when disabled
	if (likely(can->enabled)) {
#if SUPERCAN_DEBUG
		bool dump = false;
#endif
		// process messages
		while (in_ptr + SC_MSG_HEADER_LEN <= in_end) {
			struct sc_msg_header const *msg = (struct sc_msg_header const *)in_ptr;

			if (in_ptr + msg->len > in_end) {
				LOG("ch%u offset=%u len=%u exceeds buffer size=%u\n", index, (unsigned)(in_ptr - in_beg), msg->len, (unsigned)xferred_bytes);
#if SUPERCAN_DEBUG
				dump = true;
#endif
				break;
			}

			if (!msg->id || !msg->len) {
				// Allow empty message to work around having to zend ZLP
				in_ptr = in_end;
				break;
			}

			in_ptr += msg->len;

			switch (msg->id) {
			case SC_MSG_CAN_TX:
				sc_process_msg_can_tx(index, msg);
				break;

			default:
#if SUPERCAN_DEBUG
				LOG("ch%u unknown msg id=%02x len=%02x\n", index, msg->id, msg->len);
				dump = true;
#endif
				break;
			}
		}

#if SUPERCAN_DEBUG
		if (unlikely(dump)) {
			LOG("msg buffer\n");
			sc_dump_mem(in_beg, xferred_bytes);
		}
#endif

		if (sc_can_bulk_in_ep_ready(index) && usb_can->tx_offsets[usb_can->tx_bank]) {
			sc_can_bulk_in_submit(index, __func__);
		}
	}

	xSemaphoreGive(usb_can->mutex_handle);
}

static void sc_cmd_bulk_in(uint8_t index)
{
	// LOG("< cmd%u IN token\n", index);

	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.cmd));

	struct usb_cmd *usb_cmd = &usb.cmd[index];

	usb_cmd->tx_offsets[!usb_cmd->tx_bank] = 0;

	if (usb_cmd->tx_offsets[usb_cmd->tx_bank]) {
		sc_cmd_bulk_in_submit(index);
	}
}

SC_RAMFUNC static void sc_can_bulk_in(uint8_t index)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.can));
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(cans));

	struct can *can = &cans[index];
	struct usb_can *usb_can = &usb.can[index];

	while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));

	usb_can->tx_offsets[!usb_can->tx_bank] = 0;

	if (usb_can->tx_offsets[usb_can->tx_bank]) {
		sc_can_bulk_in_submit(index, __func__);
	}

	xTaskNotifyGive(can->usb_task_handle);

	xSemaphoreGive(usb_can->mutex_handle);
}

static void sc_cmd_place_error_reply(uint8_t index, int8_t error)
{
	SC_DEBUG_ASSERT(index < TU_ARRAY_SIZE(usb.cmd));

	struct usb_cmd *usb_cmd = &usb.cmd[index];
	uint8_t bytes = sizeof(struct sc_msg_error);
	uint8_t *out_ptr;
	uint8_t *out_end;

send:
	out_ptr = usb_cmd->tx_buffers[usb_cmd->tx_bank] + usb_cmd->tx_offsets[usb_cmd->tx_bank];
	out_end = usb_cmd->tx_buffers[usb_cmd->tx_bank] + CMD_BUFFER_SIZE;
	if (out_end - out_ptr >= bytes) {
		usb_cmd->tx_offsets[usb_cmd->tx_bank] += bytes;
		struct sc_msg_error *rep = (struct sc_msg_error *)out_ptr;
		rep->id = SC_MSG_ERROR;
		rep->len = sizeof(*rep);
		rep->error = error;
	} else {
		if (sc_cmd_bulk_in_ep_ready(index)) {
			sc_cmd_bulk_in_submit(index);
			goto send;
		} else {
			LOG("ch%u: no space for error reply\n", index);
		}
	}
}

SC_RAMFUNC static void can_usb_task(void* param);

static inline void can_usb_disconnect(void)
{
	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		can_state_initial(i);
		sc_board_led_can_status_set(i, SC_CAN_LED_STATUS_DISABLED);
	}
}

int main(void)
{
	// no uart here :(
	sc_board_init_begin();
	LOG("sc_board_init_begin\n");

	LOG("led_init\n");
	led_init();

	LOG("tusb_init\n");
	tusb_init();

	(void) xTaskCreateStatic(&tusb_device_task, "tusb", TU_ARRAY_SIZE(usb_device_stack), NULL, SC_TASK_PRIORITY, usb_device_stack, &usb_device_task_mem);
	(void) xTaskCreateStatic(&led_task, "led", TU_ARRAY_SIZE(led_task_stack), NULL, SC_TASK_PRIORITY, led_task_stack, &led_task_mem);

	usb.cmd[0].pipe = SC_M1_EP_CMD0_BULK_OUT;
	usb.can[0].pipe = SC_M1_EP_MSG0_BULK_OUT;

#if SC_BOARD_CAN_COUNT > 1
	usb.cmd[1].pipe = SC_M1_EP_CMD1_BULK_OUT;
	usb.can[1].pipe = SC_M1_EP_MSG1_BULK_OUT;
#endif

	for (unsigned i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct can *can = &cans[i];
		struct usb_can *usb_can = &usb.can[i];

		usb_can->mutex_handle = xSemaphoreCreateMutexStatic(&usb_can->mutex_mem);
		can->usb_task_handle = xTaskCreateStatic(&can_usb_task, NULL, TU_ARRAY_SIZE(can->usb_task_stack_mem), (void*)(uintptr_t)i, SC_TASK_PRIORITY, can->usb_task_stack_mem, &can->usb_task_mem);
	}


	LOG("can_usb_disconnect\n");
	can_usb_disconnect();

	LOG("sc_board_init_end\n");
	sc_board_init_end();

	LOG("vTaskStartScheduler\n");
	vTaskStartScheduler();

	__unreachable();

	configASSERT(0 && "should not be reachable");

	LOG("sc_board_reset\n");
	sc_board_reset();

	return 0;
}


//--------------------------------------------------------------------+
// USB DEVICE TASK
//--------------------------------------------------------------------+
SC_RAMFUNC static void tusb_device_task(void* param)
{
	(void) param;



	while (1) {
		LOG("tud_task\n");
		tud_task();
	}
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	LOG("mounted\n");

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));
	}

	led_blink(0, 250);
	usb.mounted = true;

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		xSemaphoreGive(usb_can->mutex_handle);
	}
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	LOG("unmounted\n");

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));
	}

	led_blink(0, 1000);
	usb.mounted = false;

	can_usb_disconnect();

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		xSemaphoreGive(usb_can->mutex_handle);
	}
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	LOG("suspend\n");

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));
	}

	usb.mounted = false;
	led_blink(0, 500);

	can_usb_disconnect();

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		xSemaphoreGive(usb_can->mutex_handle);
	}
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	LOG("resume\n");

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));
	}


	usb.mounted = true;
	led_blink(0, 250);

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		xSemaphoreGive(usb_can->mutex_handle);
	}
}


//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+
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


static void sc_usb_init(void)
{
	LOG("SC init\n");
}

static void sc_usb_reset(uint8_t rhport)
{
	LOG("SC port %u reset\n", rhport);

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));
	}

	usb.port = rhport;
	led_blink(0, 1000);
	usb.mounted = false;

	can_usb_disconnect();

	for (uint8_t i = 0; i < SC_BOARD_CAN_COUNT; ++i) {
		struct usb_can *usb_can = &usb.can[i];

		xSemaphoreGive(usb_can->mutex_handle);
	}
}

static uint16_t sc_usb_open(uint8_t rhport, tusb_desc_interface_t const * desc_intf, uint16_t max_len)
{
	const uint8_t eps = 4;
	const uint16_t len_required = 9+eps*7;

	LOG("vendor port %u open\n", rhport);

	if (unlikely(rhport != usb.port)) {
		return 0;
	}

	if (unlikely(max_len < len_required)) {
		return 0;
	}

	TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == desc_intf->bInterfaceClass);

	if (unlikely(desc_intf->bInterfaceNumber >= TU_ARRAY_SIZE(usb.can))) {
		return 0;
	}


	struct usb_cmd *usb_cmd = &usb.cmd[desc_intf->bInterfaceNumber];
	struct usb_can *usb_can = &usb.can[desc_intf->bInterfaceNumber];

	uint8_t const *ptr = (void const *)desc_intf;

	ptr += 9;



	for (uint8_t i = 0; i < eps; ++i) {
		tusb_desc_endpoint_t const *ep_desc = (tusb_desc_endpoint_t const *)(ptr + i * 7);
		LOG("! ep %02x open\n", ep_desc->bEndpointAddress);
		bool success = OPEN(rhport, ep_desc);
		SC_ASSERT(success);
	}

	// bool success_cmd = XFER(rhport, usb_cmd->pipe, usb_cmd->rx_buffers[usb_cmd->rx_bank], CMD_BUFFER_SIZE);
	// bool success_can = XFER(rhport, usb_can->pipe, usb_can->rx_buffers[usb_can->rx_bank], MSG_BUFFER_SIZE);
	// SC_ASSERT(success_cmd);
	// SC_ASSERT(success_can);

	XFER(rhport, usb_cmd->pipe, usb_cmd->rx_buffers[usb_cmd->rx_bank], CMD_BUFFER_SIZE);
	XFER(rhport, usb_can->pipe, usb_can->rx_buffers[usb_can->rx_bank], MSG_BUFFER_SIZE);

	// // Required to immediately send URBs when buffer size > endpoint size
	// // and transfers are multiple of enpoint size.
	// if (MSG_BUFFER_SIZE > SC_M1_EP_SIZE) {
	// 	dcd_auto_zlp(rhport, usb_can->pipe | 0x80, true);
	// }

	return len_required;
}

SC_RAMFUNC static bool sc_usb_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
	(void)result; // always success

	if (unlikely(rhport != usb.port)) {
		return false;
	}

	sc_board_led_usb_burst();


	switch (ep_addr) {
	case SC_M1_EP_CMD0_BULK_OUT:
		sc_cmd_bulk_out(0, xferred_bytes);
		break;
	case SC_M1_EP_CMD0_BULK_IN:
		sc_cmd_bulk_in(0);
		break;
	case SC_M1_EP_MSG0_BULK_OUT:
		sc_can_bulk_out(0, xferred_bytes);
		break;
	case SC_M1_EP_MSG0_BULK_IN:
		sc_can_bulk_in(0);
		break;
#if SC_BOARD_CAN_COUNT > 1
	case SC_M1_EP_CMD1_BULK_OUT:
		sc_cmd_bulk_out(1, xferred_bytes);
		break;
	case SC_M1_EP_CMD1_BULK_IN:
		sc_cmd_bulk_in(1);
		break;
	case SC_M1_EP_MSG1_BULK_OUT:
		sc_can_bulk_out(1, xferred_bytes);
		break;
	case SC_M1_EP_MSG1_BULK_IN:
		sc_can_bulk_in(1);
		break;
#endif
	default:
		LOG("port %u ep %02x result %d bytes %u\n", rhport, ep_addr, result, (unsigned)xferred_bytes);
		return false;
	}

	return true;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
	LOG("port=%u stage=%u\n", rhport, stage);

	sc_board_led_usb_burst();

	if (unlikely(rhport != usb.port)) {
		return false;
	}

	switch (stage) {
	case CONTROL_STAGE_SETUP: {
		switch (request->bRequest) {
		case VENDOR_REQUEST_MICROSOFT:
			if (request->wIndex == 7) {
				// Get Microsoft OS 2.0 compatible descriptor
				uint16_t total_len;
				memcpy(&total_len, desc_ms_os_20+8, 2);
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
		case VENDOR_REQUEST_MICROSOFT:
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



static const usbd_class_driver_t sc_usb_driver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "SC",
#endif
	.init = &sc_usb_init,
	.reset = &sc_usb_reset,
	.open = &sc_usb_open,
	/* TinyUSB doesn't call this callback for vendor requests
	 * but tud_vendor_control_xfer_cb. Sigh :/
	 */
	.control_xfer_cb = NULL,
	.xfer_cb = &sc_usb_xfer_cb,
	.sof = NULL,
};

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count)
{
	SC_ASSERT(driver_count);
	*driver_count = 1;
	return &sc_usb_driver;
}

SC_RAMFUNC extern void sc_can_notify_task_def(uint8_t index, uint32_t count)
{
	struct can *can = &cans[index];

	for (uint32_t i = 0; i < count; ++i) {
		xTaskNotifyGive(can->usb_task_handle);
	}
}

SC_RAMFUNC extern void sc_can_notify_task_isr(uint8_t index, uint32_t count)
{
	struct can *can = &cans[index];
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (likely(count)) {
		for (uint32_t i = 0; i < count - 1; ++i) {
			vTaskNotifyGiveFromISR(can->usb_task_handle, NULL);
		}

		// LOG("CAN%u notify\n", index);
		vTaskNotifyGiveFromISR(can->usb_task_handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

//--------------------------------------------------------------------+
// CAN TASK
//--------------------------------------------------------------------+
#if SPAM
SC_RAMFUNC static void can_usb_task(void *param)
{
	const uint8_t index = (uint8_t)(uintptr_t)param;
	SC_ASSERT(index < TU_ARRAY_SIZE(cans.can));
	SC_ASSERT(index < TU_ARRAY_SIZE(usb.can));

	LOG("ch%u task start\n", index);

	struct can *can = &cans.can[index];
	struct usb_can *usb_can = &usb.can[index];

	unsigned next_dlc = 0;
	uint32_t counter = 0;
	uint32_t can_id = 0x42;


	while (42) {
		// LOG("CAN%u task wait\n", index);
		(void)ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		// LOG("CAN%u task loop\n", index);
		if (unlikely(!usb.mounted)) {
			continue;
		}

		if (unlikely(!can->enabled)) {
			next_dlc = 0;
			counter = 0;
			LOG("ch%u usb state reset\n", index);
			continue;
		}

		led_burst(can->led_traffic, LED_BURST_DURATION_MS);

		while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));

		uint8_t * const tx_beg = usb_can->tx_buffers[usb_can->tx_bank];
		uint8_t * const tx_end = tx_beg + TU_ARRAY_SIZE(usb_can->tx_buffers[usb_can->tx_bank]);
		uint8_t *tx_ptr = tx_beg + usb_can->tx_offsets[usb_can->tx_bank];

		for (;;) {

			// consume all input
			__atomic_store_n(&can->rx_get_index, __atomic_load_n(&can->rx_put_index, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);

			uint8_t bytes = sizeof(struct sc_msg_can_rx);


			uint8_t dlc = next_dlc & 0xf;
			if (!dlc) {
				++dlc;
			}
			uint8_t can_frame_len = dlc_to_len(dlc);
			bytes += can_frame_len;
			if (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1)) {
				bytes += SC_MSG_CAN_LEN_MULTIPLE - (bytes & (SC_MSG_CAN_LEN_MULTIPLE-1));
			}

			if ((size_t)(tx_end - tx_ptr) >= bytes) {
				counter_1MHz_request_current_value_lazy();
				struct sc_msg_can_rx *msg = (struct sc_msg_can_rx *)tx_ptr;
				usb_can->tx_offsets[usb_can->tx_bank] += bytes;
				tx_ptr += bytes;


				msg->id = SC_MSG_CAN_RX;
				msg->len = bytes;
				msg->dlc = dlc;
				msg->flags = SC_CAN_FRAME_FLAG_FDF | SC_CAN_FRAME_FLAG_BRS;
				msg->can_id = can_id;
				memset(msg->data, 0, can_frame_len);
				memcpy(&msg->data, &counter, sizeof(counter));
				msg->timestamp_us = counter_1MHz_wait_for_current_value();
				// LOG("ts=%lx\n", msg->timestamp_us);

				++next_dlc;
				++counter;

			} else {
				break;
			}
		}

		if (sc_can_bulk_in_ep_ready(index) && usb_can->tx_offsets[usb_can->tx_bank]) {
			sc_can_bulk_in_submit(index, __func__);
		}

		xSemaphoreGive(usb_can->mutex_handle);
	}
}
#else
SC_RAMFUNC static void can_usb_task(void *param)
{
	const uint8_t index = (uint8_t)(uintptr_t)param;

	SC_ASSERT(index < TU_ARRAY_SIZE(cans));
	SC_ASSERT(index < TU_ARRAY_SIZE(usb.can));

	LOG("ch%u task start\n", index);

	struct can *can = &cans[index];
	struct usb_can *usb_can = &usb.can[index];
// #if SUPERCAN_DEBUG
// 	uint32_t rx_ts_last = 0;
// 	uint32_t tx_ts_last = 0;
// #endif
	uint8_t current_bus_status = SC_CAN_STATUS_ERROR_ACTIVE;
	uint8_t previous_led_state = SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE;
	uint8_t current_led_state = SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE;
	uint8_t tx_errors = 0;
	uint8_t rx_errors = 0;
	TickType_t bus_activity_ts = 0;
	TickType_t error_ts = 0;
	TickType_t status_ts = 0;
	bool send_can_status = 0;
	bool yield = false;


	while (42) {
		// LOG("CAN%u task wait\n", index);
		const uint32_t pre = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		if (likely(pre > 0)) {

			// LOG("CAN%u task loop\n", index);
			while (pdTRUE != xSemaphoreTake(usb_can->mutex_handle, portMAX_DELAY));

			if (unlikely(!usb.mounted || !can->enabled)) {
				const TickType_t no_error_ts = xTaskGetTickCount() - pdMS_TO_TICKS(SC_BUS_ACTIVITY_TIMEOUT_MS);
				current_bus_status = SC_CAN_STATUS_ERROR_ACTIVE;
				bus_activity_ts = no_error_ts;
				error_ts = no_error_ts;
				tx_errors = 0;
				rx_errors = 0;
				previous_led_state = SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE;
				current_led_state = SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE;
	// #if SUPERCAN_DEBUG
	// 			rx_ts_last = 0;
	// 			tx_ts_last = 0;
	// #endif

				// clear out notifications
				ulTaskNotifyTake(pdTRUE, 0);

				// ensure we don't spin b/c e.g. a CAN interrupt handler generates events
				yield = true;

				LOG("ch%u usb state reset\n", index);
			} else {

				for (bool done = false; !done; ) {
					done = true;

					const TickType_t now = xTaskGetTickCount();
					send_can_status = send_can_status || now - status_ts >= pdMS_TO_TICKS(128); // some boards don't report periodically

					uint8_t * const tx_beg = usb_can->tx_buffers[usb_can->tx_bank];
					uint8_t * const tx_end = tx_beg + TU_ARRAY_SIZE(usb_can->tx_buffers[usb_can->tx_bank]);
					uint8_t *tx_ptr = tx_beg + usb_can->tx_offsets[usb_can->tx_bank];
					int retrieved = 0;

					if (send_can_status) {
						struct sc_msg_can_status *msg = NULL;

						if ((size_t)(tx_end - tx_ptr) >= sizeof(*msg)) {
							done = false;
							send_can_status = 0;
							status_ts = now;
							sc_board_can_ts_request(index);

							msg = (struct sc_msg_can_status *)tx_ptr;
							usb_can->tx_offsets[usb_can->tx_bank] += sizeof(*msg);
							tx_ptr += sizeof(*msg);

							// sc_board_can_status_fill(index, msg);

							uint16_t tx_dropped = can->tx_dropped;
							can->tx_dropped = 0;
							uint16_t rx_lost = can->rx_lost;
							can->rx_lost = 0;

							msg->id = SC_MSG_CAN_STATUS;
							msg->len = sizeof(*msg);
							msg->bus_status = current_bus_status;
							msg->tx_dropped = tx_dropped;
							msg->rx_lost = rx_lost;
							msg->tx_fifo_size = 0;
							msg->rx_fifo_size = 0;
							msg->tx_errors = tx_errors;
							msg->rx_errors = rx_errors;
#if defined(HAVE_ATOMIC_COMPARE_EXCHANGE) && HAVE_ATOMIC_COMPARE_EXCHANGE
							msg->flags = __sync_fetch_and_and(&can->int_comm_flags, 0);
#else
							taskENTER_CRITICAL();
							msg->flags = can->int_comm_flags;
							can->int_comm_flags = 0;
							taskEXIT_CRITICAL();
#endif

							if (can->desync) {
								msg->flags |= SC_CAN_STATUS_FLAG_TXR_DESYNC;
							}

							msg->timestamp_us = sc_board_can_ts_wait(index);


							// LOG("status store %u bytes\n", (unsigned)sizeof(*msg));
							// sc_dump_mem(msg, sizeof(*msg));
						} else {
							if (sc_can_bulk_in_ep_ready(index)) {
								done = false;
								sc_can_bulk_in_submit(index, __func__);
								continue;
							} else {
								yield = true;
							}
						}
					}

					uint16_t status_put_index = __atomic_load_n(&can->status_put_index, __ATOMIC_ACQUIRE);
					if (can->status_get_index != status_put_index) {
						uint16_t fifo_index = can->status_get_index % TU_ARRAY_SIZE(can->status_fifo);
						sc_can_status *s = &can->status_fifo[fifo_index];

						sc_board_led_can_traffic_burst(index);

						switch (s->type) {
						case SC_CAN_STATUS_FIFO_TYPE_BUS_STATUS: {
							bus_activity_ts = now;
							current_bus_status = s->bus_state;
							LOG("ch%u bus status %#x\n", index, current_bus_status);
							send_can_status = 1;
							done = false;
						} break;
						case SC_CAN_STATUS_FIFO_TYPE_BUS_ERROR: {
							struct sc_msg_can_error *msg = NULL;

							bus_activity_ts = now;

							if (likely(s->bus_error.code)) {
								error_ts = now;
							}

							if ((size_t)(tx_end - tx_ptr) >= sizeof(*msg)) {
								msg = (struct sc_msg_can_error *)tx_ptr;
								usb_can->tx_offsets[usb_can->tx_bank] += sizeof(*msg);
								tx_ptr += sizeof(*msg);


								msg->id = SC_MSG_CAN_ERROR;
								msg->len = sizeof(*msg);
								msg->error = s->bus_error.code;
								msg->timestamp_us = s->timestamp_us;
								msg->flags = 0;
								if (s->bus_error.tx) {
									msg->flags |= SC_CAN_ERROR_FLAG_RXTX_TX;
								}
								if (s->bus_error.data_part) {
									msg->flags |= SC_CAN_ERROR_FLAG_NMDT_DT;
								}

								done = false;
							} else {
								if (sc_can_bulk_in_ep_ready(index)) {
									done = false;
									sc_can_bulk_in_submit(index, __func__);
									continue;
								} else {
									// LOG("ch%u dropped CAN bus error msg\n", index);
									yield = true;
								}
							}
						} break;
						case SC_CAN_STATUS_FIFO_TYPE_RXTX_ERRORS: {
							rx_errors = s->counts.rx;
							tx_errors = s->counts.tx;
							send_can_status = 1;
							done = false;
						} break;
						case SC_CAN_STATUS_FIFO_TYPE_TXR_DESYNC:
							bus_activity_ts = now;
							can->desync = true;
							send_can_status = 1;
							done = false;
							break;
						case SC_CAN_STATUS_FIFO_TYPE_RX_LOST:
							can->rx_lost += s->rx_lost;
							send_can_status = 1;
							done = false;
							break;
						default:
							LOG("ch%u unhandled CAN status message type %#02x\n", index, s->type);
							break;
						}

						__atomic_store_n(&can->status_get_index, can->status_get_index+1, __ATOMIC_RELEASE);
					}

					retrieved = sc_board_can_retrieve(index, tx_ptr, tx_end);
					// LOG("r=%d\n", retrieved);

					switch (retrieved) {
					case -1:
						break;
					case 0:
						sc_board_led_can_traffic_burst(index);

						if (sc_can_bulk_in_ep_ready(index)) {
							done = false;
							sc_can_bulk_in_submit(index, __func__);
						} else {
							yield = true;
						}
						break;
					default:
						//LOG("%c", '^' + index);
						sc_board_led_can_traffic_burst(index);

						done = false;
						SC_DEBUG_ASSERT(retrieved > 0);
						SC_DEBUG_ASSERT((size_t)retrieved + usb_can->tx_offsets[usb_can->tx_bank] <= TU_ARRAY_SIZE(usb_can->tx_buffers[usb_can->tx_bank]));
						usb_can->tx_offsets[usb_can->tx_bank] += retrieved;
						tx_ptr += retrieved;
						bus_activity_ts = now;
						break;
					}
				}


				if (usb_can->tx_offsets[usb_can->tx_bank]) {
					if (sc_can_bulk_in_ep_ready(index)) {
						sc_can_bulk_in_submit(index, __func__);
					} else {
						yield = true;
					}
				}


				const bool has_bus_activity = xTaskGetTickCount() - bus_activity_ts < pdMS_TO_TICKS(SC_BUS_ACTIVITY_TIMEOUT_MS);
				const bool has_bus_error = xTaskGetTickCount() - error_ts < pdMS_TO_TICKS(SC_BUS_ACTIVITY_TIMEOUT_MS);

				switch (current_bus_status) {
				case SC_CAN_STATUS_BUS_OFF:
					current_led_state = SC_CAN_LED_STATUS_ENABLED_ON_BUS_BUS_OFF;
					break;
				case SC_CAN_STATUS_ERROR_WARNING:
				case SC_CAN_STATUS_ERROR_PASSIVE:
					current_led_state = has_bus_activity ? SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_ACTIVE : SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_PASSIVE;
					break;
				case SC_CAN_STATUS_ERROR_ACTIVE:
					if (has_bus_error) {
						current_led_state = has_bus_activity ? SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_ACTIVE : SC_CAN_LED_STATUS_ENABLED_ON_BUS_ERROR_PASSIVE;
					} else {
						current_led_state = has_bus_activity ? SC_CAN_LED_STATUS_ENABLED_ON_BUS_ACTIVE : SC_CAN_LED_STATUS_ENABLED_ON_BUS_PASSIVE;
					}
					break;
				}

				if (current_led_state != previous_led_state) {
					sc_board_led_can_status_set(index, current_led_state);

					previous_led_state = current_led_state;
				}
			}

			xSemaphoreGive(usb_can->mutex_handle);
		} else {
			yield = true;
		}

		// LOG("|");

		if (yield) {
			// yield to prevent this task from eating up the CPU
			// when the USB buffers are full/busy.
			yield = false;
			// taskYIELD();
			vTaskDelay(pdMS_TO_TICKS(1)); // 1ms for USB FS
		}
	}
}
#endif // !SPAM
