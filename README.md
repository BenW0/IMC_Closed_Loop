# IMC Closed Loop Controller
This program modifies and extends the original template Intelligent Motor Control code (see https://github.com/BenW0/IMC-Axis-Simple) to implement closed-loop control on a CNC axis. It is designed to work in tandem with the IMC Master variant of the Marlin 3D printer firmware https://github.com/BenW0/IMC_Master_Marlin, and to be separately controlled by a PC using the Controller Configuration GUI https://github.com/BenW0/Ctrl_Design_GUI. For further details, see https://sites.google.com/site/benweisspublic/projects/imc-closed-loop-control.

# Platform
This code runs on a PJRC.com Teensy 3.0/3.1. Connections can be found in the Schematic directory, or by looking at the comments in main.c.

# Dependencies & Building
This code is built using the ARM compiler bundled with Teensyduino, but compiles separately from C using a Makefile.

Modify the TEENSY_PATH variable in the top of the Makefile to point to the Teensyduino Tools path (generally under Arduino/hardware/tools). You will need to use tools/teensy.exe to flash the chip.

All other dependencies are included locally in the teensy-include folder, except for one header file from Freescale which does not have a redistribution license. The header file, MK20DZ10.h, contains an expanded list of constants used by the code, and can be downloaded from Freescale at http://cache.freescale.com/files/microcontrollers/software/device_drivers/Kinetis_Header_File_(RevA_20111104).zip

If that link is broken, you are looking for the Kinetis MCU C/C++ Header Files, Rev A. See the list at http://www.freescale.com/webapp/software-center/library.jsp?tid=SwCHbpKINETIS#/home/t748_t755/~query~/c=c201,c201_c173,c201_c173_c161,c201_c173_c161_c164/popularity/0

Once downloaded, the header file MK20DZ10.h should be placed in the teensy-include folder.

## Building
To build, run "make main.hex". If in Windows, MinGW works fine.


# Using in an IMC Network
At this time, the IMC CLC does not listen for I2C commands from a Master node at boot. To enter IMC mode, connect the Teensy to a computer over USB and use the Controller Configuration GUI to connect to it, enter a control mode (Unity, PID, etc) and then enter IMC Mode. The device is now ready to listen for commands over the IMC/I2C bus.

# Integration with IMC Simple Stepper
This code attempts to make only minimal modifications to the IMC template code available at <<<IMC ME599 Ref>>>. All of the code, including slight modifications, is stored in the IMC directory. Re-merging with the IMC repository should be fairly straightforward, but has not yet been attempted.

# License
See COPYING file for details.