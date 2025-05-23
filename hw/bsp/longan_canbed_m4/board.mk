DEPS_SUBMODULES += hw/mcu/microchip

DEVID = 0x3558B383
CHIP_ROM_SIZE = 0x00080000
CHIP_RAM_SIZE = 0x00030000
JLINK_DEVICE = ATSAME51G19A
CFLAGS += -D__SAME51J19A__

BOOTLOADER_SIZE = 0x2000
SUPERDFU_APP_TAG_PTR_OFFSET = 0x3FC

CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m4 \
  -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_SAME5X \
  -DSUPERDFU_APP_TAG_PTR_OFFSET=$(SUPERDFU_APP_TAG_PTR_OFFSET) \
  -DSUPERDFU_BOOTLOADER_SIZE=$(BOOTLOADER_SIZE) \
  -DSUPERDFU_DEV_ID=$(DEVID) \
  -DBOARD_NAME="\"Longan CANBED M4\"" \
  -DRAMFUNC_SECTION_NAME="\".ramfunc\"" \
  -DSVC_Handler=SVCall_Handler

CFLAGS += -Wno-error=undef -Wno-error=type-limits


LD_FILE_IN = $(TOP)/hw/bsp/$(BOARD)/same51jxxa_flash.ld
LINKER_SCRIPT = $(CURDIR)/$(BUILD)/$(BOARD).ld
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


$(LINKER_SCRIPT): $(OBJ_DIRS) $(LD_FILE_IN)
	@cat "$(LD_FILE_IN)" | $(SED) 's/CHIP_ROM_SIZE/$(CHIP_ROM_SIZE)/g; s/CHIP_RAM_SIZE/$(CHIP_RAM_SIZE)/g; s/SUPERDFU_BOOTLOADER_SIZE/$(BOOTLOADER_SIZE)/g; s/SUPERDFU_APP_TAG_PTR_OFFSET/$(SUPERDFU_APP_TAG_PTR_OFFSET)/g;' >$@

LINKER_SCRIPT_TARGET = $(LINKER_SCRIPT)

SRC_C += \
  src/portable/microchip/samd/dcd_samd.c \
  hw/mcu/microchip/same51/gcc/gcc/startup_same51.c \
  hw/mcu/microchip/same51/gcc/system_same51.c

INC += \
	$(TOP)/hw/mcu/microchip/same51/ \
	$(TOP)/hw/mcu/microchip/same51/config \
	$(TOP)/hw/mcu/microchip/same51/include \
	$(TOP)/hw/mcu/microchip/same51/hal/include \
	$(TOP)/hw/mcu/microchip/same51/hal/utils/include \
	$(TOP)/hw/mcu/microchip/same51/hpl/port \
	$(TOP)/hw/mcu/microchip/same51/hri \
	$(TOP)/hw/mcu/microchip/same51/CMSIS/Include

# For freeRTOS port source
FREERTOS_PORT = ARM_CM4F

# For flash-jlink target
JLINK_IF = swd

# flash using jlink
flash: flash-jlink

# flash using edbg from https://github.com/ataradov/edbg
flash-edbg: $(BUILD)/$(BOARD)-firmware.bin
	edbg --verbose -t same51 -pv -f $<
