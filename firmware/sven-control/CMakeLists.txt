cmake_minimum_required(VERSION 3.5)
set(ARDUINO_SDK_PATH "/home/alexander/workspace/tools/arduino-1.8.13/")
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME sven_control)

set(${PROJECT_NAME}_BOARD nano)
set(ARDUINO_CPU atmega328)
project(${PROJECT_NAME} C CXX ASM)


set(CMAKE_VERBOSE_MAKEFILE ON)

# Define additional source and header files or default arduino sketch files
# These files are now added to the sketch file, if the sketch is defined.
# set(${PROJECT_NAME}_SRCS)
# set(${PROJECT_NAME}_HDRS)

set(sven_control_SRCS
        src/launch.cpp
        src/i2cmaster.S
        src/buffers.cpp
        src/PT2313.cpp
#        /usr/share/arduino/libraries/Wire/utility/twi.c
        lib/encoder/EncButton.h
)
set(sven_control_HDRS
        src/i2cmaster.h
        src/PT2313.h
#        /usr/share/arduino/libraries/Wire/utility/twi.h
        lib/encoder/EncButton.h
)
set(sven_control_BOARD nano)


### Additional static libraries to include in the target.
# set(${PROJECT_NAME}_LIBS)

### Main sketch file
#set(${PROJECT_NAME}_SKETCH src/launch.cpp)

### Add project directories into the build
# add_subdirectory()

### Additional settings to add non-standard or your own Arduino libraries.
# For this example (libs will contain additional arduino libraries)
# An Arduino library my_lib will contain files in libs/my_lib/: my_lib.h, my_lib.cpp + any other cpp files
#link_directories(${CMAKE_CURRENT_SOURCE_DIR}/)

# This is no longer need but left in for those occasions where automatics do not work out. 
# Current Arduino.cmake will recurse into any libraries which contain source files in src/ subdirectory. 
# Files in subdirectories whose name starts with cmake-build or examples are excluded
# For nested library sources replace ${LIB_NAME} with library name for each library
# set(_RECURSE true)

#### Additional settings for programmer. From programmers.txt
set(${PROJECT_NAME}_PROGRAMMER avrispmkii)

set(${PROJECT_NAME}_PORT /dev/ttyUSB0)
#set(${PROJECT_NAME}_PORT /tmp/simavr-uart0)

set(nano.upload.speed 19200)

# https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
set(nano.menu.cpu.atmega328.build.mcu atmega328p)

set(CMAKE_EXE_LINKER_FLAGS -mmcu=${nano.menu.cpu.atmega328.build.mcu})

## Verbose build process
set(${PROJECT_NAME}_AFLAGS -v)


generate_arduino_firmware(${PROJECT_NAME})


print_board_list()
print_programmer_list()
