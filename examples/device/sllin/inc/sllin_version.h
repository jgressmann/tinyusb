/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2021-2022 Jean Gressmann <jean@0x42.de>
 *
 */

#pragma once

#define SLLIN_STR2(x) #x
#define SLLIN_STR(x) SLLIN_STR2(x)

#define SLLIN_VERSION_MAJOR 0
#define SLLIN_VERSION_MINOR 4
#define SLLIN_VERSION_PATCH 1

#define SLLIN_VERSION_STR SLLIN_STR(SLLIN_VERSION_MAJOR) "." SLLIN_STR(SLLIN_VERSION_MINOR) "." SLLIN_STR(SLLIN_VERSION_PATCH)
