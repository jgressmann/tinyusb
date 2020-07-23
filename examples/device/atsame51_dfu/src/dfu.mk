$(BUILD)/$(BOARD)-firmware.superdfu.elf: $(BUILD)/$(BOARD)-firmware.elf
	@echo CREATE $@
	@$(CP) $^ $@
	@python3 $(TOP)/examples/device/atsame51_dfu/src/superdfu-patch.py --strict 1 -e little $@

$(BUILD)/$(BOARD)-firmware.superdfu.hex: $(BUILD)/$(BOARD)-firmware.superdfu.elf
	@echo CREATE $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BOARD)-firmware.superdfu.bin: $(BUILD)/$(BOARD)-firmware.superdfu.elf
	@echo CREATE $@
	@$(OBJCOPY) -O binary $^ $@

$(BUILD)/$(BOARD)-firmware.dfu: $(BUILD)/$(BOARD)-firmware.superdfu.bin
	@echo CREATE $@
	$(CP) $^ $@
	dfu-suffix -v $(VID) -p $(PID) -a $@

dfu: $(BUILD)/$(BOARD)-firmware.dfu

dfu-upload: $(BUILD)/$(BOARD)-firmware.dfu
	sudo dfu-util -D $(BUILD)/$(BOARD)-firmware.dfu

flash-dfu: $(BUILD)/$(BOARD)-firmware.superdfu.hex
	@echo halt > $(BUILD)/$(BOARD).superdfu.jlink
	@echo r > $(BUILD)/$(BOARD).superdfu.jlink
	@echo loadfile $^ >> $(BUILD)/$(BOARD).superdfu.jlink
	@echo r >> $(BUILD)/$(BOARD).superdfu.jlink
	@echo go >> $(BUILD)/$(BOARD).superdfu.jlink
	@echo exit >> $(BUILD)/$(BOARD).superdfu.jlink
	$(JLINKEXE) -device $(JLINK_DEVICE) -if $(JLINK_IF) -JTAGConf -1,-1 -speed auto -CommandFile $(BUILD)/$(BOARD).superdfu.jlink

diff: $(BUILD)/$(BOARD)-firmware.superdfu.elf
	xxd $(BUILD)/$(BOARD)-firmware.elf >$(BUILD)/$(BOARD)-firmware.xxd
	xxd $(BUILD)/$(BOARD)-firmware.superdfu.elf >$(BUILD)/$(BOARD)-firmware.superdfu.xxd
#meld $(BUILD)/$(BOARD)-firmware.xxd $(BUILD)/$(BOARD)-firmware.superdfu.xxd
