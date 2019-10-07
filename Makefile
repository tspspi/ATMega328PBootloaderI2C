CPUFREQ=16000000L
FLASHDEV=/dev/ttyU0
I2CADR=0x14

all: atmega328pi2c.hex clean

atmega328pi2c.elf: atmega328pi2c.c

	#avr-gcc -Wall -Wno-unused-but-set-variable -Os -mrelax -mmcu=atmega328p -DF_CPU=$(CPUFREQ) -DI2CADR=$(I2CADR) -Wl,--section-start=.text=0x7e00 -Wl,--section-start=.version=0x7ffe -Wl,--relax -nostartfiles -o atmega328pi2c.elf atmega328pi2c.c
	avr-gcc -Wall -Wno-unused-but-set-variable -Os -mrelax -mmcu=atmega328p -DF_CPU=$(CPUFREQ) -DI2CADR=$(I2CADR) -Wl,--section-start=.text=0x7800 -Wl,--section-start=.version=0x7ffe -Wl,--relax -nostartfiles -o atmega328pi2c.elf atmega328pi2c.c

atmega328pi2c.hex: atmega328pi2c.elf

	avr-size -t atmega328pi2c.elf
	avr-objcopy -j .text -j .data -j .version --set-section-flags .version=alloc,load -O ihex atmega328pi2c.elf atmega328pi2c.hex
	avr-objdump -h -S atmega328pi2c.elf > atmega328pi2c.lst

flash: atmega328pi2c.hex

	sudo chmod 666 $(FLASHDEV)
	# Setting fuse bits
	#avrdude -v -p atmega328p -c stk500v1 -P $(FLASHDEV) -b 19200 -e -U lock:w:0x3F:m -U efuse:w:0xFD:m -U hfuse:w:0xDA:m -U lfuse:w:0xDF:m
	# hfuse selects 2048 words bootloader size (ToDo: Make smaller ...)
	avrdude -v -p atmega328p -c stk500v1 -P $(FLASHDEV) -b 19200 -e -U lock:w:0x3F:m -U efuse:w:0xFD:m -U hfuse:w:0xDE:m -U lfuse:w:0xDF:m

	# Flashing bootloader
	avrdude -v -p atmega328p -c stk500v1 -P $(FLASHDEV) -b 19200 -U flash:w:atmega328pi2c.hex:i -Ulock:w:0x0F:m

clean:
	-rm *.elf

cleanall: clean

	-rm *.lst
	-rm *.hex

.PHONY: all clean cleanall
