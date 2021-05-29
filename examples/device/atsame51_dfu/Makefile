include ../../../tools/top.mk
include ../../make.mk


INC += \
	inc \
	src \
	$(TOP)/hw \
	$(TOP)/src/device

ifdef VID
  CFLAGS += -DVID=$(VID)
else
  $(error VID is not set)
endif

ifdef PID
  CFLAGS += -DPID=$(PID)
else
  $(error PID is not set)
endif

PRODUCT_NAME ?= SuperDFU
INTERFACE_NAME ?= USB DFU 1.1

CFLAGS += -DPRODUCT_NAME="\"$(PRODUCT_NAME)\"" -DINTERFACE_NAME="\"$(INTERFACE_NAME)\""

ifdef SUPERDFU_DEBUG
  CFLAGS += -DSUPERDFU_DEBUG=$(SUPERDFU_DEBUG)
else
  CFLAGS += -DSUPERDFU_DEBUG=0
endif

# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)
SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))

include ../../rules.mk



