CFLAGS += \
  -mthumb \
  -mabi=aapcs \
  -mlong-calls \
  -mcpu=cortex-m4 \
  -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 \
  -nostdlib -nostartfiles \
  -D__SAME51J19A__ \
  -DCONF_CPU_FREQUENCY=120000000 \
  -DCONF_GCLK_USB_FREQUENCY=48000000 \
  -DCFG_TUSB_MCU=OPT_MCU_SAME51

# compiler options for Atmel Studio's 'Debug' configuration
#-O2 -ffunction-sections -mlong-calls -g3 -Wall -Wextra -mcpu=cortex-m4 -c -std=gnu99 -mfloat-abi=softfp -mfpu=fpv4-sp-d16

# linker options for Atmel Studio's 'Debug' configuration
#-mthumb -Wl,-Map="mcba_clone.map" --specs=nano.specs -Wl,--start-group -lm  -Wl,--end-group -L"..\\Device_Startup"  -Wl,--gc-sections -mcpu=cortex-m4 -Tsame51j19a_flash.ld

CFLAGS += -Wno-error=undef

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/same51j19a_flash.ld

SRC_C += \
	hw/mcu/microchip/same/asf4/same51/gcc/gcc/startup_same51.c \
	hw/mcu/microchip/same/asf4/same51/gcc/system_same51.c \
	hw/mcu/microchip/same/asf4/same51/hpl/gclk/hpl_gclk.c \
	hw/mcu/microchip/same/asf4/same51/hpl/mclk/hpl_mclk.c \
	hw/mcu/microchip/same/asf4/same51/hpl/osc32kctrl/hpl_osc32kctrl.c \
	hw/mcu/microchip/same/asf4/same51/hpl/oscctrl/hpl_oscctrl.c \
	hw/mcu/microchip/same/asf4/same51/hal/src/hal_atomic.c \
	hw/mcu/microchip/same/asf4/same51/hal/utils/src/utils_syscalls.c

INC += \
	$(TOP)/hw/mcu/microchip/same/asf4/same51/ \
	$(TOP)/hw/mcu/microchip/same/asf4/same51/config \
	$(TOP)/hw/mcu/microchip/same/asf4/same51/include \
	$(TOP)/hw/mcu/microchip/same/asf4/same51/hal/include \
	$(TOP)/hw/mcu/microchip/same/asf4/same51/hal/utils/include \
	$(TOP)/hw/mcu/microchip/same/asf4/same51/hpl/port \
	$(TOP)/hw/mcu/microchip/same/asf4/same51/hri \
	$(TOP)/hw/mcu/microchip/same/asf4/same51/CMSIS/Core/Include

# For TinyUSB port source
VENDOR = microchip
CHIP_FAMILY = same

# For freeRTOS port source
FREERTOS_PORT = ARM_CM4F

# For flash-jlink target
JLINK_DEVICE = ATSAME51J19
JLINK_IF = swd

# flash using jlink
flash: flash-jlink
