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


# compiler options for Atmel Studio's 'Debug' configuration
#-O2 -ffunction-sections -mlong-calls -g3 -Wall -Wextra -mcpu=cortex-m4 -c -std=gnu99 -mfloat-abi=softfp -mfpu=fpv4-sp-d16

# Create various files
#"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O binary "Inbetriebnahme.elf" "Inbetriebnahme.bin"
#"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "Inbetriebnahme.elf" "Inbetriebnahme.hex"
#"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O binary "Inbetriebnahme.elf" "Inbetriebnahme.eep" || exit 0
#"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objdump.exe" -h -S "Inbetriebnahme.elf" > "Inbetriebnahme.lss"
#"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "Inbetriebnahme.elf" "Inbetriebnahme.srec"
#"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-size.exe" "Inbetriebnahme.elf"






# SRC_C += \
# 	hw/mcu/microchip/same/asf4/same51/gcc/gcc/startup_same51.c \
# 	hw/mcu/microchip/same/asf4/same51/gcc/system_same51.c \
# 	hw/mcu/microchip/same/asf4/same51/hpl/gclk/hpl_gclk.c \
# 	hw/mcu/microchip/same/asf4/same51/hpl/mclk/hpl_mclk.c \
# 	hw/mcu/microchip/same/asf4/same51/hpl/osc32kctrl/hpl_osc32kctrl.c \
# 	hw/mcu/microchip/same/asf4/same51/hpl/oscctrl/hpl_oscctrl.c \
# 	hw/mcu/microchip/same/asf4/same51/hal/src/hal_atomic.c \
# 	hw/mcu/microchip/same/asf4/same51/hal/utils/src/utils_syscalls.c

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
