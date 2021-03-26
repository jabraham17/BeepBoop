
# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

ARDUINO_DIR= /Applications/Arduino.app/Contents/Java
ARDMK_DIR= /usr/local/Cellar/arduino-mk/1.6.0/
AVR_TOOLS_DIR= $(ARDUINO_DIR)/hardware/tools/avr
MONITOR_PORT= $(shell /dev/cu.usbmodem*)
BOARD_TAG= uno
USER_LIB_PATH = lib
OBJDIR = bin
TARGET = BeepBoop

include $(ARDMK_DIR)/Arduino.mk