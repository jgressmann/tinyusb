ST_FAMILY = g0
DEPS_SUBMODULES += lib/CMSIS_5 hw/mcu/st/cmsis_device_$(ST_FAMILY)
# hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

ST_CMSIS = hw/mcu/st/cmsis_device_$(ST_FAMILY)
# ST_HAL_DRIVER = hw/mcu/st/stm32$(ST_FAMILY)xx_hal_driver

CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m0plus \
  -nostdlib -nostartfiles \
  -DSTM32G0B1xx \
  -DCFG_TUSB_MCU=OPT_MCU_STM32G0 \
  -DRAMFUNC_SECTION_NAME="\".RamFunc\""



# LDFLAGS += -latomic

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/STM32G0B1CETX_FLASH.ld

SRC_C += \
  src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c \

SRC_S += \
  $(ST_CMSIS)/Source/Templates/gcc/startup_stm32g0b1xx.s

INC += \
  $(TOP)/lib/CMSIS_5/CMSIS/Core/Include \
  $(TOP)/$(ST_CMSIS)/Include \
  $(TOP)/hw/bsp/$(BOARD)

  # $(TOP)/$(ST_HAL_DRIVER)/Inc \


# For freeRTOS port source
FREERTOS_PORT = ARM_CM0

# For flash-jlink target
JLINK_DEVICE = STM32G0B1CE

# # flash target using on-board stlink
# flash: flash-jlink

# flash target ROM bootloader
flash: $(BUILD)/$(PROJECT).bin
	dfu-util -R -a 0 --dfuse-address 0x08000000 -D $<
