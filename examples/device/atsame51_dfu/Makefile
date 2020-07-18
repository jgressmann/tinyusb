include ../../../tools/top.mk
include ../../make.mk

CFLAGS += -Wno-error

INC += \
	inc \
	src \
	$(TOP)/hw \
	$(TOP)/src/device

# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)
SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))

include ../../rules.mk
