Repository for code for Rasberry Pi Pico to emulate the xmos-xs1 style timing box for the Heart SPIM system.
This repository can be cloned and then imported as a project in the Raspberry Pi Pico extension for VS Code. 


# Install

The code attached here was developed around the Raspberry Pi Pico plugin for VS Code. The pdf attached to the hyperlink "(Getting Started with the Pi-Pico Seris)[https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html]" is worth a quick read and includes instructions for installing the few dependencies and the Pi Pico Extension for VS Code. 
To actually edit and run the Pi Pico Code in VSCode, clone the repository (`git clone`) and import the project within the VS Code plugin. Pressing compile does the double duty of first running a `cmake build .` commnd and compiling the C code. To flash this code onto the pico, connect it to a PC using the USB port on the pico and press the BOOTSEL button at the same time. Release the BOOTSEL button and press RUN within VSCODE. Alternatively, you can flash the program memory of the pico in terminal by copying the .uf2 file into the pico after mounting it as a drive. If successful it should automatically unmount itself. This is also covered in the previously menthioned pdf.



# Integration into SPIM GUI

This version of the timing box was built to backward compatible with the xmos style timing box in terms of how it interacts with software and hardware and should be compatible with existing config.plist's for timing box types > 2

# Some Techical Documentation

## Clocks, and scheduling pianola events
This version of the timing box emulates the clock system of the xmos board where the current time is stored as a 24bit unsigned integer incrementing every 2.56us, wrapping around at 0xFFFFFF. This is implemented using one of the 4 available state machines in the Pi Pico's pio block and is customisable with the limits of storing time as a 32bit unsigned integer and incrementing every 80ns for the Pico 1 or 75ns for the Pico 2 (equivalent to 10 clock cycles of the CPU). For the Pico 1 the CPU clock rate is 125MHz (8ns per tick) and the Pico 2 is 133MHz (7.5ns per tick).
The scheduling of future events relies on checking if the requested clock time is after the current clock time - when the clock wraps around every 2^24 $\approx$ 30 seconds this becomes slightly footery. Again, we just emulate what the Xmos board does in its `timerafter` function. For a 24 bit clock a time,t2, is considered to be in the past w.r.t. the current clock time, t1 if it falls within the range of {t1 - 2^23, t1 - 1}. A time falling outwith this range is then a valid future time!

## Communications Protocol

Communication through the SPIM gui is over UART using byte arrays to transmit data from the host to the timing box. The zeroth byte sent out is always the command byte, and supplementary data is then encoded with the MSB being the 1st and the last byte is the LSB.


### Command set
* SET_Pianola => `0x01`
* SET_PianolaFinalPos => `0x02`
* SET_PianolaRepeatFrom => `0x03`
* SET_PianolaRepeating => `0x04`
* RUN_Pianola => `0x05`
* SET_PianolaFireTime => `0x06`
* IRQ_StopAndReset => `0x7`
* GET_CurrentPianolaTime => `0x08`
* SET_PinSource => `0x09`
* GET_PinSource => `0xA`
* SET_CameraClk => `0xB`
* SET_PIVParams => `0xC`
* SET_ClockDivisor => `0xAB`
* GET_FirmwareVersion => `0xFD`
* IRQ_DumpLog => `0xFE`
* IRQ_HARDRESET => `0xFF`

### Command Description

Below each command in the protocol is defined by what it does and the format of data sent to the timing box and responses to be received. Formats are listed with the zeroth entry of the char array being listed first.

#### SET_Pianola => 0x01
Set the address of the pianola with the mask of outputs to drive high (0) or (1) and the duration to drive high for.
Call : CMD, pianola address, pianola pin mask, pianola duration (3bytes) = 6 bytes
Response : none

#### SET_PianolaFinalPos => 0x02
Sets the address of the final instruction in the pianola.
Call : CMD, pianola address = 2 bytes
Response : none

#### SET_PianolaRepeatFrom => `0x03`
Sets pianola repeating from given address. Repeating can be stopped using the SET_PPianolaRepeating command.
Call : CMD, repreat address = 2 bytes
Response : none

#### SET_PianolaRepeating => `0x04`
Sets flag to repeat pianola. (0x01) to repeat until heat death of the universe and (0x00) to stop repeating.
Call : CMD, repeat flag = 2 bytes


#### RUN_Pianola => `0x05`
Sets pianola running and runs once.
Call : CMD = 1 byte
Response : pianola clock time = 3 bytes


#### SET_PianolaFireTime => `0x06`
Configure pianola to run once at a future time. Returns pianola time and a flag to signify if pianola will fire in the future.
Call : CMD, future fire time = 4 bytes
Response : fire flag, pianola time = 4 bytes

#### IRQ_StopAndReset => `0x7`
Stops pianola running if it is and cancels any scheduled fire times. Pianola instructions remain intact.
Call : CMD
Response : none

#### GET_CurrentPianolaTime => `0x08`
Gets the current clock time from the timing box.
Call : CMD
Response : current pianola time = 3 bytes

#### SET_PinSource => `0x09`
Sets which pins are outputs and intputs. This is mainly for older version of the timing box with configurable inputs and outputs but is implemented for posterity.
Call: CMD, pin index to set, bit index that pin is mapped to, flag if that bit is to be inverted = 3 bytes
Respnse : none

#### GET_PinSource => `0xA`
Get the configuraion for a queried pin.
Call : CMD, pin index to query = 2 bytes
Response : Bit index its mapped to, flag indicating if the output should be inverted = 2 bytes

#### SET_CameraClk => `0xB`
Sets the clock rate for the additional clocks in terms of the half period in pianola ticks (2.56us).
Call : index of clock, clock period (3 bytes) = 4 bytes
Response : none

#### SET_PIVParams => `0xC`
Configures the various parameters for PIV.
Call: camera index,
      interval in ticks betweeen laser pulse pairs (four bytes),
      duration in ticks of laser pulse pairs (four bytes),
      interval in ticks between the end of the first laser pulse and end of first camera exposure (four bytes),
      duration of camera exposures (four bytes),
      Bit index of the camera exposuer mask to be montiored as part of the PIV pulse sequencing
      = 18 bytes
Response : none

#### SET_ClockDivisor => `0xAB`

Sets the tick rate of the triggering clock by dividing the cpu clock speed (125MHz) by a fixed-point integer with 16 integral bits and 8 fractional bits.
The pianola clock divisor is set as this value divided by 10 - clock takes 10 cpu ticks to decrement. There is a miniscule loss of precision on certain value but this should be fine.
TO DO - this should maybe be rolled into the SET_CameraCLK command?
Call : CMD,  clock divisor integral (2 bytes), clock divisor fraction = 4 bytes
Response : none

#### GET_FirmwareVersion => `0xFD`
Returns the current firmware version and the earliest firmware version that is backwards compatible.
Call : CMD = 1 byte
Response : current firmware version, earliest compatible version = 2 bytes


#### IRQ_DumpLog => `0xFE`
Returns the contents of the debug buffer. This is of variable length and dependent on what it is you want to debug.
TO DO: maybe instead of returning CHAR arrays it should instead print to the serial monitor?

Call : CMD = 1 byte
Response : how longs a piece of string?


#### IRQ_HARDRESET => `0xFF`

Like the IRQ_StopAndReset command but the nuclear option. Clears any pending fires, the pianola instructions, and resets other flags / values to their defaults

Call : CMD = 1 bytes
Response : none




TO DO:
  - Write a test harness and upload seperately
  - Separate folders into C projects for the pico, test harness, and an example standalone bit of C code for driving timing box

  - 
