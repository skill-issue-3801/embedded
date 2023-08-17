CC = avr-gcc
OBJCPY = avr-objcopy
SIZE = avr-size
MCU = atmega328p
F_CPU = 8000000
PROGRAMMER = usbasp

INC = ./inc

CFLAGS = \
	-mmcu=$(MCU) \
	-DF_CPU=$(F_CPU)UL \
	-Os \
	-std=gnu99 \
	-Werror \
	-ffunction-sections \
	-fdata-sections \
	-DAVR_USE_HW_I2C 
LDFLAGS = \
	-Wl,--gc-sections \
	-mmcu=$(MCU)
AVRDUDE=avrdude


include filelist.mk


FUSES= -U lfuse:w:0x62:m -U hfuse:w:0xD9:m -U efuse:w:0xFF:m

OBJ = $(SRC:.c=.o)

main.hex: main.elf
	$(OBJCPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature main.elf main.hex

main.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) -o $@

size: main.elf
	$(SIZE) --mcu=$(MCU) --format=avr main.elf

%.o: ./src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) main.elf main.hex

upload: main.hex
	$(AVRDUDE) -F -V -c $(PROGRAMMER) -p $(MCU) -s -U flash:w:main.hex $(FUSES)
