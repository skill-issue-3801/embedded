# pi_serial
This section contains all Python scripts to be run on the Raspberry Pi to communicate with the embedded systems' microcontrollers that are handling user input. This implementation was written using Python V3.8.

## Rasberry Pi Setup (For Prototype)
Download the pi_serial directory in this repository on the Callender Rasberry Pi computer. The following instructions assume the raspberry Pi is running Ubuntu 22.04 Desktop version.

Save the directory in an easy-to-access repository.

Open this directory in the terminal and install python and pip
```
sudo apt install python3.8 && sudo python3.8 -m pip install pip
```
Then download all dependencies
```
sudo python3 pip install Serial && sudo python3 pip install Keybinds
```

## Rasberry Pi Execution
Open the folder the directory is saved in in terminal if you haven't already and execute the following command.
```
sudo python3 serial_comms.py
```

## Development Setup
You may set up any python development environment that supports Python 3.8 that you would like. Just clone into the repository, download the dependencies, and start development.

## Dependencies
* [Python 3.8](https://www.python.org/downloads/release/python-380/)
* [Serial](https://pyserial.readthedocs.io/en/latest/pyserial.html)
* [Keyboard](https://pypi.org/project/keybind/](https://pypi.org/project/keyboard/)https://pypi.org/project/keyboard/)

## Hardware Requirements
* [The system from the stm32 repository](https://github.com/skill-issue-3801/embedded/tree/main/stm32)
