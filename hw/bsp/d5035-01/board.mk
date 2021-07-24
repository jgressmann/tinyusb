CFLAGS += \
  -mthumb \
  -mabi=aapcs \
  -mlong-calls \
  -mcpu=cortex-m4 \
  -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 \
  -nostdlib -nostartfiles \
  -D__SAME51J19A__ \
  -DCONF_CPU_FREQUENCY=80000000 \
  -DCONF_GCLK_USB_FREQUENCY=48000000 \
  -DCFG_TUSB_MCU=OPT_MCU_SAME51 \
  -DD5035_01=1 \
  -DBOARD_NAME="\"D5035-01\""

CFLAGS += -Wno-error=undef

ifdef HWREV
  CFLAGS += -DHWREV=$(HWREV)
endif

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/same51j19a_flash.ld
ifdef APP
ifneq ($(APP),0)
  # All source paths should be relative to the top level.
  LD_FILE = hw/bsp/$(BOARD)/same51j19a_flash_app.ld
  CFLAGS += -DSUPERDFU_APP=1
else
  CFLAGS += -DSUPERDFU_APP=0
endif
else
  CFLAGS += -DSUPERDFU_APP=0
endif

ifdef BOOTLOADER
ifneq ($(BOOTLOADER),0)
  # All source paths should be relative to the top level.
  LD_FILE = hw/bsp/$(BOARD)/same51j19a_flash_bootloader.ld
endif
endif

SRC_C += \
	hw/mcu/microchip/same/asf4/same51/gcc/gcc/startup_same51.c \
  hw/mcu/microchip/same/asf4/same51/gcc/system_same51.c \

ifdef SYSCALLS
ifneq ($(SYSCALLS),0)
  SRC_C += hw/mcu/microchip/same/asf4/same51/hal/utils/src/utils_syscalls.c
endif
endif

ifdef LOG
ifneq ($(LOG),0)
  SRC_C += hw/mcu/microchip/same/asf4/same51/hal/utils/src/utils_syscalls.c
endif
endif

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

# flash using edbg from https://github.com/ataradov/edbg
flash-edbg: $(BUILD)/$(BOARD)-firmware.bin
	edbg --verbose -t same51 -pv -f $<
