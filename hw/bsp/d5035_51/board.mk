DEPS_SUBMODULES += hw/mcu/microchip

DEVID = 0xD503551

CHIP ?= SAMDA1G16
ifdef CHIP
  ifeq ($(CHIP),SAMDA1G16)
    CFLAGS += -D__SAMD21G16A__

    CHIP_ROM_SIZE = 0x00010000
    CHIP_RAM_SIZE = 0x00002000
    JLINK_DEVICE = ATSAMDA1G16
  else
    $(error Unknown chip: $(CHIP)))
  endif
endif

BOOTLOADER_SIZE = 0x2000
SUPERDFU_APP_TAG_PTR_OFFSET = 0xFC

CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m0plus \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_SAMD21 \
  -DSUPERDFU_APP_TAG_PTR_OFFSET=$(SUPERDFU_APP_TAG_PTR_OFFSET) \
  -DSUPERDFU_BOOTLOADER_SIZE=$(BOOTLOADER_SIZE) \
  -DSUPERDFU_DEV_ID=$(DEVID) \
  -DRAMFUNC_SECTION_NAME="\".ramfunc\""

CFLAGS += -Wno-error=undef -Wno-error=type-limits

HWREV ?= 1
CFLAGS += -DHWREV=$(HWREV)

LD_FILE_PREFIX = samdxx
LD_FILE_IN = $(TOP)/hw/bsp/$(BOARD)/$(LD_FILE_PREFIX)_flash.ld
LINKER_SCRIPT = $(CURDIR)/$(BUILD)/$(BOARD).ld
ifdef APP
ifneq ($(APP),0)
  LD_FILE_IN = $(TOP)/hw/bsp/$(BOARD)/$(LD_FILE_PREFIX)_flash_app.ld
  CFLAGS += -DSUPERDFU_APP=1
else
  CFLAGS += -DSUPERDFU_APP=0
endif
else
  CFLAGS += -DSUPERDFU_APP=0
endif

ifdef BOOTLOADER
ifneq ($(BOOTLOADER),0)
  LD_FILE_IN = $(TOP)/hw/bsp/$(BOARD)/$(LD_FILE_PREFIX)_flash_bootloader.ld
endif
endif


$(LINKER_SCRIPT): $(OBJ_DIRS) $(LD_FILE_IN)
	@cat "$(LD_FILE_IN)" | $(SED) 's/CHIP_ROM_SIZE/$(CHIP_ROM_SIZE)/g; s/CHIP_RAM_SIZE/$(CHIP_RAM_SIZE)/g; s/SUPERDFU_BOOTLOADER_SIZE/$(BOOTLOADER_SIZE)/g; s/SUPERDFU_APP_TAG_PTR_OFFSET/$(SUPERDFU_APP_TAG_PTR_OFFSET)/g;' >$@


LINKER_SCRIPT_TARGET = $(LINKER_SCRIPT)

SRC_C += \
  src/portable/microchip/samd/dcd_samd.c \
	hw/mcu/microchip/samd21/gcc/gcc/startup_samd21.c \
	hw/mcu/microchip/samd21/gcc/system_samd21.c

ifdef LOG
  ifneq ($(LOG),0)
    SRC_C += hw/mcu/microchip/samd21/hal/utils/src/utils_syscalls.c
  endif
endif

INC += \
	$(TOP)/hw/mcu/microchip/samd21/ \
	$(TOP)/hw/mcu/microchip/samd21/config \
	$(TOP)/hw/mcu/microchip/samd21/include \
	$(TOP)/hw/mcu/microchip/samd21/hal/include \
	$(TOP)/hw/mcu/microchip/samd21/hal/utils/include \
	$(TOP)/hw/mcu/microchip/samd21/hpl/port \
	$(TOP)/hw/mcu/microchip/samd21/hri \
	$(TOP)/hw/mcu/microchip/samd21/CMSIS/Include \
  $(TOP)/hw/mcu/microchip/samd21/hri \
  $(TOP)/hw/mcu/microchip/samd21/hpl/pm \

# For freeRTOS port source
FREERTOS_PORT = ARM_CM0

# For flash-jlink target
JLINK_IF = swd

# flash using jlink
flash: flash-jlink

# flash using edbg from https://github.com/ataradov/edbg
flash-edbg: $(BUILD)/$(BOARD)-firmware.bin
	edbg --verbose -t samd21 -pv -f $<
