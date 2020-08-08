include ../../../tools/top.mk
include ../../make.mk

CFLAGS += -Wno-error
CFLAGS += -DVID=0x$(VID) -DPID=0x$(PID)


INC += \
	inc \
	src \
	$(TOP)/hw \
	$(TOP)/src/device \
	$(TOP)/examples/device/atsame51_dfu/inc \

# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)
SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))

include ../../rules.mk
include $(TOP)/examples/device/atsame51_dfu/src/dfu.mk