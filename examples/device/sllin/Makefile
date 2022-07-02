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
	$(TOP)/lib/misc/inc \
	$(TOP)/$(FREERTOS_SRC)/include \
	$(TOP)/$(FREERTOS_SRC)/portable/GCC/$(FREERTOS_PORT)


SRC_C += $(TOP)/lib/misc/src/usnprintf.c
SAM_SRC_C += $(TOP)/lib/misc/src/sam_crc32.c

SECTION_CFLAGS = -DUSNPRINTF_SECTION_NAME=RAMFUNC_SECTION_NAME -DSLLIN_RAMFUNC_SECTION_NAME=RAMFUNC_SECTION_NAME


ifeq ($(BOARD),trinket_m0)
CFLAGS += -DRAMFUNC_SECTION_NAME="\".ramfunc\"" -DTRINKET_M0=1 $(SECTION_CFLAGS)
SRC_C += $(SAM_SRC_C)
endif

ifeq ($(BOARD),d5035_50)
INC += $(TOP)/examples/device/superdfu/inc
CFLAGS += -DD5035_50=1 -DHAVE_ATOMIC_COMPARE_EXCHANGE $(SECTION_CFLAGS)
SRC_C += $(SAM_SRC_C)
endif

ifeq ($(BOARD),d5035_51)
INC += $(TOP)/examples/device/superdfu/inc
CFLAGS += -DD5035_51=1
SRC_C += $(SAM_SRC_C)
endif


# ifeq ($(SAME54XPLAINEDPRO),1)
# INC += $(TOP)/examples/device/superdfu/inc
# CONF_CPU_FREQUENCY = 120000000
# endif



CFLAGS += -Wno-error



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
include $(TOP)/examples/device/superdfu/src/dfu.mk

