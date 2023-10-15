#pragma once

#include <stdlib.h>
#include "gd32c10x.h"

#define USE_USB_FS 1
#define USB_FS_CORE 1


// in 4 byte terms

// 1152 bytes
// #define RX_FIFO_FS_SIZE                         128
// #define TX0_FIFO_FS_SIZE                        16
// #define TX1_FIFO_FS_SIZE                        16
// #define TX2_FIFO_FS_SIZE                        128
// #define TX3_FIFO_FS_SIZE                        0

// fits in RAM
#define RX_FIFO_FS_SIZE                         96
#define TX0_FIFO_FS_SIZE                        16
#define TX1_FIFO_FS_SIZE                        16
#define TX2_FIFO_FS_SIZE                        96
#define TX3_FIFO_FS_SIZE                        0


#define USB_SOF_OUTPUT              0
#define USB_LOW_POWER               0
#define USB_ISO                     0

//#define VBUS_SENSING_ENABLED

//#define USE_HOST_MODE
#define USE_DEVICE_MODE
//#define USE_OTG_MODE

#ifndef USE_DEVICE_MODE
    #ifndef USE_HOST_MODE
        #error  "USE_DEVICE_MODE or USE_HOST_MODE should be defined!"
    #endif
#endif

#define __ALIGN_BEGIN
#define __ALIGN_END

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__GNUC__)         /* GNU Compiler */
    #ifndef __packed
        #define __packed __attribute__ ((__packed__))
    #endif
#elif defined (__TASKING__)    /* TASKING Compiler */
    #define __packed __unaligned
#endif /* __CC_ARM */


