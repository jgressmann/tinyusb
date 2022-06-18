VID ?= ffff
PID ?= ffff

OUT_BASE_NAME=$(PROJECT)
ELF_FILE=$(OUT_BASE_NAME).elf
DFUED_ELF_FILE=$(OUT_BASE_NAME).superdfu.elf
DFUED_HEX_FILE=$(OUT_BASE_NAME).superdfu.hex
DFUED_BIN_FILE=$(OUT_BASE_NAME).superdfu.bin
DFU_TAG_FILE=$(OUT_BASE_NAME).tag
DFU_BIN_FILE=$(OUT_BASE_NAME).bin
DFU_DFU_FILE=$(OUT_BASE_NAME).dfu

$(BUILD)/$(DFUED_ELF_FILE): $(BUILD)/$(ELF_FILE)
	@echo CREATE $@
	@$(CP) $^ $@.tmp
	python3 $(TOP)/examples/device/atsame51_dfu/src/superdfu-patch.py --strict 1 --tag $(BUILD)/$(DFU_TAG_FILE) $@.tmp
	@mv $@.tmp $@

$(BUILD)/$(DFUED_HEX_FILE): $(BUILD)/$(DFUED_ELF_FILE)
	@echo CREATE $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(DFUED_BIN_FILE): $(BUILD)/$(DFUED_ELF_FILE)
	@echo CREATE $@
	@$(OBJCOPY) -O binary $^ $@

flash-dfu: $(BUILD)/$(DFUED_HEX_FILE)
	@echo halt > $(BUILD)/$(BOARD).superdfu.jlink
	@echo r > $(BUILD)/$(BOARD).superdfu.jlink
	@echo loadfile $^ >> $(BUILD)/$(BOARD).superdfu.jlink
	@echo r >> $(BUILD)/$(BOARD).superdfu.jlink
	@echo go >> $(BUILD)/$(BOARD).superdfu.jlink
	@echo exit >> $(BUILD)/$(BOARD).superdfu.jlink
	$(JLINKEXE) -device $(JLINK_DEVICE) -if $(JLINK_IF) -JTAGConf -1,-1 -speed auto -CommandFile $(BUILD)/$(BOARD).superdfu.jlink


$(BUILD)/$(DFU_DFU_FILE): $(BUILD)/$(DFUED_ELF_FILE)
	@echo CREATE $@
	@$(OBJCOPY) -O binary $^ $(BUILD)/$(DFU_BIN_FILE)
	cat $(BUILD)/$(DFU_TAG_FILE) $(BUILD)/$(DFU_BIN_FILE) >$@
	dfu-suffix -v $(VID) -p $(PID) -a $@

# depend on dfu and hex so we can have both in one build
dfu: $(BUILD)/$(DFU_DFU_FILE) $(BUILD)/$(DFUED_BIN_FILE)

dfu-upload: $(BUILD)/$(DFU_DFU_FILE)
	sudo dfu-util -d $(VID):$(PID) -R -D $^

# flash using edbg from https://github.com/ataradov/edbg
edbg-dfu: $(BUILD)/$(DFUED_BIN_FILE)
	edbg --verbose -t same51 -pv -o $(OFFSET) -f $<