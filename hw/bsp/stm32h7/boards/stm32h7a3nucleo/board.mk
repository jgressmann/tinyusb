CFLAGS += -DSTM32H7A3xx -DHSE_VALUE=25000000 -DRAMFUNC_SECTION_NAME="\".RamFunc\""

# Default is full speed port
PORT = 0
# HS core with FS phy
FS_PHY_ON_HS_CORE = 1

SRC_S += $(ST_CMSIS)/Source/Templates/gcc/startup_stm32h7a3xx.s
LD_FILE = $(BOARD_PATH)/stm32h7a3zitxq_flash.ld

# For flash-jlink target
JLINK_DEVICE = stm32h7a3zi

# flash target using on-board stlink
flash: flash-stlink

# FreeRTOS recommendation for non r0p1 cores
FREERTOS_PORT = ARM_CM4F

