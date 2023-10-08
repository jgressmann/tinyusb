


CMSIS_PATH = $(TOP)/$(BOARD_PATH)/mcu/Firmware/CMSIS
CMSIS_GD32C10X_PATH = $(CMSIS_PATH)/GD/GD32C10x
PER_GD32C10X_PATH = $(TOP)/$(BOARD_PATH)/mcu/Firmware/GD32C10x_standard_peripheral
USB_GD32C10X_PATH = $(TOP)/$(BOARD_PATH)/mcu/Firmware/GD32C10x_usbfs_library

CFLAGS += \
	-mthumb \
	-mabi=aapcs \
	-mcpu=cortex-m4 \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-nostdlib -nostartfiles \
	-DCFG_TUSB_MCU=OPT_MCU_GD32C10X \
	-DGD32C10X \
	-DRAMFUNC_SECTION_NAME="\".ramfunc\""


#-flto \

CFLAGS += -Wno-error=undef -Wno-error=type-limits -Wno-error=unused-parameter




HWREV ?= 1
CFLAGS += -DHWREV=$(HWREV)


# build automagically adds startup_gd32c10x.c to SRC_C
SRC_C += \
	src/portable/st/synopsys/dcd_synopsys.c

INC += \
	$(TOP)/$(BOARD_PATH) \
	$(CMSIS_PATH) \
	$(CMSIS_GD32C10X_PATH)/Include \
	$(PER_GD32C10X_PATH)/Include \
	$(TOP)/$(BOARD_PATH)/mcu/Template \
	$(TOP)/src/portable/st/synopsys





LD_FILE = hw/bsp/$(BOARD)/gd32c10x.ld

# For freeRTOS port source
FREERTOS_PORT = ARM_CM4F

# For flash-jlink target
JLINK_IF = swd
JLINK_DEVICE = GD32E103CB

#GD32F305RB

# flash using jlink
flash: flash-jlink


# flash target ROM bootloader
dfu-upload: $(BUILD)/$(PROJECT).bin
	sudo dfu-util -R -a 0 --dfuse-address 0x08000000 -D $<