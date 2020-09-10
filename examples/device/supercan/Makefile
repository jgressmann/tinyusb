VID=0x1d50
PID=0x5035
BOARD=d5035-01

include ../../../tools/top.mk
include ../../make.mk

FREERTOS_SRC = lib/FreeRTOS/FreeRTOS/Source

INC += \
	inc \
	src \
	$(TOP)/hw \
	$(TOP)/src/device \
	$(TOP)/$(FREERTOS_SRC)/include \
	$(TOP)/$(FREERTOS_SRC)/portable/GCC/$(FREERTOS_PORT) \
	$(TOP)/examples/device/atsame51_dfu/inc \


CFLAGS += -Wno-error
CFLAGS += -DVID=$(VID) -DPID=$(PID)

ifdef SUPERCAN_DEBUG
  CFLAGS += -DSUPERCAN_DEBUG=$(SUPERCAN_DEBUG)
else
  CFLAGS += -DSUPERCAN_DEBUG=0
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
include $(TOP)/examples/device/atsame51_dfu/src/dfu.mk
