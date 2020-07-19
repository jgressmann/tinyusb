include ../../../tools/top.mk
include ../../make.mk

CFLAGS += -Wno-error

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
