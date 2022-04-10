/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#include <stdint.h>

void usb_get_desc_string(uint8_t index, char *ptr, size_t* in_out_capa_len);

