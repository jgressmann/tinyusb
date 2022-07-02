/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2020-2022 Jean Gressmann <jean@0x42.de>
 *
 */
#include <dfu_debug.h>

#if SUPERDFU_DEBUG
char dfu_log_buffer[SUPERDFU_DEBUG_LOG_BUFFER_SIZE];
#endif


// https://www.segger.com/products/debug-probes/j-link/tools/j-link-swo-viewer/

#ifndef __ARM_ARCH_ISA_ARM
	#define __ARM_ARCH_ISA_ARM 0
#endif

#if __ARM_ARCH_ISA_ARM

/*********************************************************************
*
*       Defines for Cortex-M debug unit
*/
#define ITM_STIM_U32 (*(volatile unsigned int*)0xE0000000)    // Stimulus Port Register word acces
#define ITM_STIM_U8  (*(volatile         char*)0xE0000000)    // Stimulus Port Register byte acces
#define ITM_ENA      (*(volatile unsigned int*)0xE0000E00)    // Trace Enable Ports Register
#define ITM_TCR      (*(volatile unsigned int*)0xE0000E80)    // Trace control register

/*********************************************************************
*
*       SWO_PrintChar()
*
* Function description
*   Checks if SWO is set up. If it is not, return,
*    to avoid program hangs if no debugger is connected.
*   If it is set up, print a character to the ITM_STIM register
*    in order to provide data for SWO.
* Parameters
*   c:    The Chacracter to be printed.
* Notes
*   Additional checks for device specific registers can be added.
*/
static inline void SWO_PrintChar(char c) {
  //
  // Check if ITM_TCR.ITMENA is set
  //
  if ((ITM_TCR & 1) == 0) {
    return;
  }
  //
  // Check if stimulus port is enabled
  //
  if ((ITM_ENA & 1) == 0) {
    return;
  }
  //
  // Wait until STIMx is ready,
  // then send data
  //
  while ((ITM_STIM_U8 & 1) == 0);
  ITM_STIM_U8 = c;
}

#else
	#define SWO_PrintChar(x)
#endif

static inline void write_chars(char const *msg, int count)
{
	if (count == -1) {
		while (*msg) {
			char c = *msg++;

			SWO_PrintChar(c);
			board_uart_write(&c, 1);
		}
	} else {
		for (int i = 0; i < count; ++i) {
			SWO_PrintChar(msg[i]);
		}

		board_uart_write(msg, count);
	}
}


extern void dfu_dump_mem(void const * _ptr, size_t count)
{
	char buf[8];
	int chars = 0;
	uint8_t const *ptr = (uint8_t const *)_ptr;

	for (size_t i = 0; i < count; i += 16) {
		chars = usnprintf(buf, sizeof(buf), "%03X  ", (unsigned)i);

		write_chars(buf, chars);

		size_t end = i + 16;
		if (end > count) {
			end = count;
		}

		for (size_t j = i; j < end; ++j) {
			chars = usnprintf(buf, sizeof(buf), "%02X ", ptr[j]);

			write_chars(buf, chars);
		}

		write_chars("\n", 1);
	}
}

