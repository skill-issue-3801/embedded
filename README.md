# SkillIssue Embedded
This repository is to contain any and all code related to the implementation of the 
embedded systems required for the project.

## AVR
Folder to contain any code used for AVR based microcontrollers.

## STM32
This folder is used as the directory for any projects completed in the STM32CubeIDE. This
microcontrollers utilised ARM based MCUs.

This is where the majority of the current implementation is taking place. The STM32 board
interfaces with GPIO to provide user input, a NFC dev board for RFID based tag identification,
and serial communication to send commands to a listening python script running
on the Raspberry Pi that is displaying the website.

## Pi_serial
Contains any python scripts to be run on the Raspberry Pi to communicate with the embedded systems
microcontrollers that are handling user input.