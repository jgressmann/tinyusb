$(BUILD)/$(BOARD)-firmware.superdfu.elf: $(BUILD)/$(BOARD)-firmware.elf
	@echo CREATE $@
	@$(CP) $^ $@
	python3 $(TOP)/examples/device/atsame51_dfu/src/superdfu-patch.py --strict 1 -e little $@

$(BUILD)/$(BOARD)-firmware.superdfu.hex: $(BUILD)/$(BOARD)-firmware.superdfu.elf
	@echo CREATE $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BOARD)-firmware.superdfu.bin: $(BUILD)/$(BOARD)-firmware.superdfu.elf
	@echo CREATE $@
	@$(OBJCOPY) -O binary $^ $@

flash-dfu: $(BUILD)/$(BOARD)-firmware.superdfu.hex
	@echo halt > $(BUILD)/$(BOARD).superdfu.jlink
	@echo r > $(BUILD)/$(BOARD).superdfu.jlink
	@echo loadfile $^ >> $(BUILD)/$(BOARD).superdfu.jlink
	@echo r >> $(BUILD)/$(BOARD).superdfu.jlink
	@echo go >> $(BUILD)/$(BOARD).superdfu.jlink
	@echo exit >> $(BUILD)/$(BOARD).superdfu.jlink
	$(JLINKEXE) -device $(JLINK_DEVICE) -if $(JLINK_IF) -JTAGConf -1,-1 -speed auto -CommandFile $(BUILD)/$(BOARD).superdfu.jlink


$(BUILD)/$(BOARD)-firmware.dfu: $(BUILD)/$(BOARD)-firmware.superdfu.bin
	@echo CREATE $@
	dfu-tool convert dfu $^ $@

# depend on dfu and hex so we can have both in one build
dfu: $(BUILD)/$(BOARD)-firmware.dfu $(BUILD)/$(BOARD)-firmware.superdfu.hex

dfu-upload: $(BUILD)/$(BOARD)-firmware.dfu
	sudo dfu-tool write $(BUILD)/$(BOARD)-firmware.dfu