CHIP ?= SAME51J18A
ifdef CHIP
  ifeq ($(CHIP),SAME51J18A)
    CFLAGS += -D__SAME51J18A__
    CHIP_ROM_SIZE = 0x00040000
    CHIP_RAM_SIZE = 0x00020000
    JLINK_DEVICE = ATSAME51J18
  else
    ifeq ($(CHIP),SAME51J19A)
      CFLAGS += -D__SAME51J19A__
      CHIP_ROM_SIZE = 0x00080000
      CHIP_RAM_SIZE = 0x00030000
      JLINK_DEVICE = ATSAME51J19
    else
      ifeq ($(CHIP),SAME51J20A)
        CFLAGS += -D__SAME51J20A__
        JLINK_DEVICE = ATSAME51J20
        CHIP_ROM_SIZE = 0x00100000
        CHIP_RAM_SIZE = 0x00040000
      else
        $(error Unknown chip: $(CHIP)))
      endif
    endif
  endif
else
  # smallest of the ATSAME51JXXA series
  CFLAGS += -D__SAME51J18A__
  CHIP_ROM_SIZE = 0x00040000
  CHIP_RAM_SIZE = 0x00020000
endif


CFLAGS += \
  -mthumb \
  -mabi=aapcs \
  -mlong-calls \
  -mcpu=cortex-m4 \
  -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 \
  -nostdlib -nostartfiles \
  -DCONF_CPU_FREQUENCY=80000000 \
  -DCONF_GCLK_USB_FREQUENCY=48000000 \
  -DCFG_TUSB_MCU=OPT_MCU_SAME51 \
  -DD5035_01=1 \
  -DBOARD_NAME="\"D5035-01\""

CFLAGS += -Wno-error=undef

ifdef HWREV
  CFLAGS += -DHWREV=$(HWREV)
endif



LD_FILE_IN = $(TOP)/hw/bsp/$(BOARD)/same51jxxa_flash.ld
LINKER_SCRIPT = $(BUILD)/$(BOARD).ld
ifdef APP
ifneq ($(APP),0)
  LD_FILE_IN = $(TOP)/hw/bsp/$(BOARD)/same51jxxa_flash_app.ld
  CFLAGS += -DSUPERDFU_APP=1
else
  CFLAGS += -DSUPERDFU_APP=0
endif
else
  CFLAGS += -DSUPERDFU_APP=0
endif

ifdef BOOTLOADER
ifneq ($(BOOTLOADER),0)
  LD_FILE_IN = $(TOP)/hw/bsp/$(BOARD)/same51jxxa_flash_bootloader.ld
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
JLINK_IF = swd

$(BUILD)/$(BOARD).ld:
	@mkdir -p $(BUILD) 2>/dev/null
	@cat "$(LD_FILE_IN)" | $(SED) 's/CHIP_ROM_SIZE/$(CHIP_ROM_SIZE)/g; s/CHIP_RAM_SIZE/$(CHIP_RAM_SIZE)/g;' >$@

# flash using jlink
flash: flash-jlink

# flash using edbg from https://github.com/ataradov/edbg
flash-edbg: $(BUILD)/$(BOARD)-firmware.bin
	edbg --verbose -t same51 -pv -f $<
