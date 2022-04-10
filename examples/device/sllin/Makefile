VID = 1d50
PID = 5037

DEPS_SUBMODULES += lib/FreeRTOS-Kernel

include ../../../tools/top.mk
include ../../make.mk

FREERTOS_SRC = lib/FreeRTOS-Kernel

INC += \
	inc \
	src \
	$(TOP)/hw \
	$(TOP)/$(FREERTOS_SRC)/include \
	$(TOP)/$(FREERTOS_SRC)/portable/GCC/$(FREERTOS_PORT)


ifeq ($(BOARD),trinket_m0)
CFLAGS += -DRAMFUNC_SECTION_NAME="\".ramfunc\""
else
CFLAGS += -DHAVE_ATOMIC_COMPARE_EXCHANGE
endif

ifeq ($(BOARD),d5035_02)
INC += $(TOP)/examples/device/atsame51_dfu/inc
CFLAGS += -DRAMFUNC_SECTION_NAME="\".ramfunc\"" -DD5035_02=1
endif


# ifeq ($(SAME54XPLAINEDPRO),1)
# INC += $(TOP)/examples/device/atsame51_dfu/inc
# CONF_CPU_FREQUENCY = 120000000
# endif



CFLAGS += -Wno-error

SECTION_CFLAGS = -DUSNPRINTF_SECTION_NAME=RAMFUNC_SECTION_NAME -DSLLIN_RAMFUNC_SECTION_NAME=RAMFUNC_SECTION_NAME



ifdef SLLIN_DEBUG
  CFLAGS += -DSLLIN_DEBUG=$(SLLIN_DEBUG)
else
  CFLAGS += -DSLLIN_DEBUG=0
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
include $(TOP)/examples/device/atsame51_dfu/src/dfu.mk

