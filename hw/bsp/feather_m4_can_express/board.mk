UF2_FAMILY_ID = 0x55114460
DEPS_SUBMODULES += hw/mcu/microchip


CFLAGS += \
  -mthumb \
  -mabi=aapcs \
  -mlong-calls \
  -mcpu=cortex-m4 \
  -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_SAME5X \
  -D__SAME51J19A__ \
  -DRAMFUNC_SECTION_NAME="\".ramfunc\"" \
  -DSVC_Handler=SVCall_Handler

CFLAGS += -Wno-error=undef -Wno-error=type-limits


LD_FILE = hw/bsp/$(BOARD)/$(BOARD).ld

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

# flash using bossac at least version 1.8
# can be found in arduino15/packages/arduino/tools/bossac/
# Add it to your PATH or change BOSSAC variable to match your installation
BOSSAC = bossac

flash-bossac: $(BUILD)/$(PROJECT).bin
	@:$(call check_defined, SERIAL, example: SERIAL=/dev/ttyACM0)
	$(BOSSAC) --port=$(SERIAL) -U -i --offset=0x4000 -e -w $^ -R

flash: flash-bossac
