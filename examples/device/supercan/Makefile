VID = 1d50
PID = 5035

DEPS_SUBMODULES += lib/FreeRTOS-Kernel

include ../../../tools/top.mk
include ../../make.mk

FREERTOS_SRC = lib/FreeRTOS-Kernel

INC += \
	inc \
	src \
	$(TOP)/hw \
	$(TOP)/lib/misc/inc \
	$(TOP)/$(FREERTOS_SRC)/include \
	$(TOP)/$(FREERTOS_SRC)/portable/GCC/$(FREERTOS_PORT)

SRC_C += $(TOP)/lib/misc/src/usnprintf.c
SAM_SRC_C += $(TOP)/lib/misc/src/sam_crc32.c

ifeq ($(D5035_01),1)
INC += $(TOP)/examples/device/superdfu/inc
SRC_C += $(SAM_SRC_C)
endif


ifeq ($(SAME54XPLAINEDPRO),1)
INC += $(TOP)/examples/device/superdfu/inc
CONF_CPU_FREQUENCY = 120000000
SRC_C += $(SAM_SRC_C)
endif



CFLAGS += -Wno-error

SECTION_CFLAGS = -DUSNPRINTF_SECTION_NAME=RAMFUNC_SECTION_NAME -DSC_RAMFUNC_SECTION_NAME=RAMFUNC_SECTION_NAME

ifeq ($(BOARD),feather_m4_can_express)
CFLAGS += $(SECTION_CFLAGS) -DFEATHER_M4_CAN_EXPRESS=1
INC += $(TOP)/examples/device/superdfu/inc
SRC_C += $(SAM_SRC_C)
endif


ifeq ($(BOARD),longan_canbed_m4)
CFLAGS += $(SECTION_CFLAGS) -DLONGAN_CANBED_M4=1
INC += $(TOP)/examples/device/superdfu/inc
SRC_C += $(SAM_SRC_C)
endif



ifdef MIMXRT10XX
ifeq ($(MIMXRT10XX),0)
CFLAGS += $(SECTION_CFLAGS)
else
endif
else
CFLAGS += $(SECTION_CFLAGS)
endif

ifeq ($(BOARD),stm32f303disco)
CFLAGS += $(SECTION_CFLAGS) -UUSNPRINTF_SECTION_NAME -USC_RAMFUNC_SECTION_NAME -DBOARD_NAME="\"STM32F303DISCOVERY\""
endif


ifdef SUPERCAN_DEBUG
  CFLAGS += -DSUPERCAN_DEBUG=$(SUPERCAN_DEBUG)
else
  CFLAGS += -DSUPERCAN_DEBUG=0
endif

ifdef APP
ifneq ($(APP),0)
  CFLAGS += -DSUPERDFU_APP=1
else
  CFLAGS += -DSUPERDFU_APP=0
endif
else
  CFLAGS += -DSUPERDFU_APP=0
endif

# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)
SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))

# FreeRTOS source, all files in port folder
SRC_C += \
	$(FREERTOS_SRC)/list.c \
	$(FREERTOS_SRC)/queue.c \
	$(FREERTOS_SRC)/tasks.c \
	$(FREERTOS_SRC)/timers.c \
	$(subst ../../../,,$(wildcard ../../../$(FREERTOS_SRC)/portable/GCC/$(FREERTOS_PORT)/*.c))

# FreeRTOS (lto + Os) linker issue
LDFLAGS += -Wl,--undefined=vTaskSwitchContext

include ../../rules.mk
include $(TOP)/examples/device/superdfu/src/dfu.mk

