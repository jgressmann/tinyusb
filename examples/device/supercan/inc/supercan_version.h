/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2023 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#define SUPERCAN_STR2(x) #x
#define SUPERCAN_STR(x) SUPERCAN_STR2(x)

#define SUPERCAN_VERSION_MAJOR 0
#define SUPERCAN_VERSION_MINOR 5
#define SUPERCAN_VERSION_PATCH 13

#define SUPERCAN_VERSION_STR SUPERCAN_STR(SUPERCAN_VERSION_MAJOR) "." SUPERCAN_STR(SUPERCAN_VERSION_MINOR) "." SUPERCAN_STR(SUPERCAN_VERSION_PATCH)
