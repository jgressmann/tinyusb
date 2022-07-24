
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
	-DDOWNLOAD_MODE=DOWNLOAD_MODE_FLASHXIP \
	-DGD32C10X -DGD32C103R_START \
	-DRAMFUNC_SECTION_NAME="\".ramfunc\""

CFLAGS += -Wno-error=undef -Wno-error=type-limits -Wno-error=unused-parameter

#-flto \


HWREV ?= 1
CFLAGS += -DHWREV=$(HWREV)


SRC_C += \
	$(TOP)/$(BOARD_PATH)/startup_gd32c10x.c \
	src/portable/st/synopsys/dcd_synopsys.c \
	$(CMSIS_GD32C10X_PATH)/Source/system_gd32c10x.c



	# $(GD32VF103_SDK_DRIVER)/gd32vf103_gpio.c \
	# $(GD32VF103_SDK_DRIVER)/Usb/gd32vf103_usb_hw.c \
	# $(GD32VF103_SDK_DRIVER)/gd32vf103_usart.c


INC += \
	$(TOP)/$(BOARD_PATH) \
	$(CMSIS_PATH) \
	$(CMSIS_GD32C10X_PATH)/Include \
	$(PER_GD32C10X_PATH)/Include \
	$(USB_GD32C10X_PATH)/driver/Include \
	$(USB_GD32C10X_PATH)/ustd/common \
	$(TOP)/$(BOARD_PATH)/mcu/Template


LD_FILE = hw/bsp/$(BOARD)/gd32c10x.ld

# For freeRTOS port source
FREERTOS_PORT = ARM_CM4F

# For flash-jlink target
JLINK_IF = swd
JLINK_DEVICE = GD32F305RB

# flash using jlink
flash: flash-jlink


# flash target ROM bootloader
dfu-upload: $(BUILD)/$(PROJECT).bin
	dfu-util -R -a 0 --dfuse-address 0x08000000 -D $<