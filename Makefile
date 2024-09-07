
PROGRAM=msp430
SOURCE= \
  src/$(PROGRAM).v \
  src/eeprom.v \
  src/memory_bus.v \
  src/peripherals.v \
  src/ram.v \
  src/rom.v \
  src/spi.v

default:
	yosys -q -p "synth_ice40 -top $(PROGRAM) -json $(PROGRAM).json" $(SOURCE)
	nextpnr-ice40 -r --hx8k --json $(PROGRAM).json --package cb132 --asc $(PROGRAM).asc --opt-timing --pcf icefun.pcf
	icepack $(PROGRAM).asc $(PROGRAM).bin

program:
	iceFUNprog $(PROGRAM).bin

blink:
	naken_asm -l -type bin -o rom.bin test/blink.asm
	python3 tools/bin2txt.py rom.bin > rom.txt

lcd:
	naken_asm -l -type bin -o rom.bin test/lcd.asm
	python3 tools/bin2txt.py rom.bin > rom.txt

simple:
	naken_asm -l -type bin -o rom.bin test/simple.asm
	python3 tools/bin2txt.py rom.bin > rom.txt

memory:
	naken_asm -l -type bin -o rom.bin test/memory.asm
	python3 tools/bin2txt.py rom.bin > rom.txt

branch:
	naken_asm -l -type bin -o rom.bin test/branch.asm
	python3 tools/bin2txt.py rom.bin > rom.txt

clean:
	@rm -f $(PROGRAM).bin $(PROGRAM).json $(PROGRAM).asc *.lst
	@rm -f blink.bin load_byte.bin store_byte.bin test_subroutine.bin
	@rm -f button.bin
	@echo "Clean!"

