# BeepBoop Project

## Requirements

Uses [this](https://github.com/sudar/Arduino-Makefile) project to allow for Makefile building. This requires the Arduino IDE to be installed, but once installed the user does not need to open it.

### Mac Quick Install

```
brew tap sudar/arduino-mk
brew install arduino-mk
python3 -m pip install pyserial
```

You may need to update the Makefile for your system, modify the necessary variables.

## Building

To build and check that the code compiles, run `make`. To upload to the board, run `make upload`. 

NOTE: ensure the bluetooth module is NOT connected when uploading, otherwise it will fail. Nothing can be connected to the Tx/Rx pins.

## Resources

- [Main Wiki Page For Kit](https://www.uctronics.com/wiki/K0072)
- [Library for Kit and Examples](https://github.com/UCTRONICS/Smart-Robot-Car-Arduino)
- [Makefile reference](https://github.com/sudar/Arduino-Makefile)