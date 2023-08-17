# AVR Programming
This is simple framework for programming AVRs directly from the command line.

## Tools
To install the required tools to use this framework call this command
```
sudo apt-get install gcc-avr binutils-avr avr-libc gdb-avr avrdude
```

## Editing source files
.c files go in the `/src` directory, .h files go in the `/inc` directory. 

## Bulding code
To build the code return to the `/avr` directory and call:
```
make
```
This will compile all the source files, then build the main uploadable file in the form of a `.hex` file. 

## Uploading code
To upload code, plug in the usbASP programmer to your laptop and connect it to the MCU. Connect it to the chip via SPI. If you wish to change the programmer this can be updated in the `Makefile` by changing the `$(PROGRAMMER)` varriable. One the programmer is pluged in to the chip call:
```
make upload
```
This will upload the code into the eeprom of the chip. Make sure this is only called when the chip is powered to avoid any unnessisary brownouts or programming issues. 
