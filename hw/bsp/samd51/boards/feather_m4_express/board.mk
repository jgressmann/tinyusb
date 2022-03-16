CFLAGS += -D__SAME51J19A__

LD_FILE = $(BOARD_PATH)/$(BOARD).ld

# For flash-jlink target
JLINK_DEVICE = ATSAME51J19

flash: flash-bossac
