#
# Copyright (C) 2014 Jacob McIntosh <nacitar at ubercpp dot com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

include(cmake/build_utils.cmake)
###
# Build Values
###
#
# C++ sources: list(APPEND CXX_SOURCES ...)
# C sources:   list(APPEND C_SOURCES ...)
# C/C++ flags: Append(CC_FLAGS ...)
# C++ flags:   Append(CXX_FLAGS ...)
# C flags:     Append(C_FLAGS ...)
# Link flags:  Append(LINK_FLAGS ...)
#
# Clear all:   ClearBuildValues()
#
###

project (nac-firmware CXX C)
cmake_minimum_required(VERSION 2.6.2)

# target name
set(TARGET "${CMAKE_PROJECT_NAME}")
# the program binary's format
set(FORMAT "ihex")
# Debounce time (see keyswitch spec) 5ms is good for cherry mx switches
set(DEBOUNCE_TIME "5")
# LED brightness multiplier (0.0 is off, 1.0 is max)
set(LED_BRIGHTNESS "0.5")
# processor type (for teensy 2.0); must match real life
set(MCU "atmega32u4")
set(BOARD "teensy-2.0")
set(KEYBOARD "ergodox")
set(LAYOUT "workman-p-kinesis-mod")
# processor Hz, must match initialization in source
set(F_CPU "16000000")

# Clang support
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_COMPILER_IS_CLANG "1")
endif()

# GCC support
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG)
  set(GCC_COMPAT "1")
endif()

# Compilation settings
if(GCC_COMPAT)
  # pass in hardware settings
  Append(CC_FLAGS "-mmcu=${MCU} -DF_CPU=${F_CPU}")
  # set options
  Append(CC_FLAGS "-DMAKEFILE_BOARD='${BOARD}'")
  Append(CC_FLAGS "-DMAKEFILE_KEYBOARD='${KEYBOARD}'")
  Append(CC_FLAGS "-DMAKEFILE_KEYBOARD_LAYOUT='${LAYOUT}'")
  Append(CC_FLAGS "-DMAKEFILE_DEBOUNCE_TIME='${DEBOUNCE_TIME}'")
  Append(CC_FLAGS "-DMAKEFILE_LED_BRIGHTNESS='${LED_BRIGHTNESS}'")
  # optimize for size
  Append(CC_FLAGS "-Os")
  # enable common warnings
  Append(CC_FLAGS "-Wall")
  # pack all structure members together without holes
  Append(CC_FLAGS "-fpack-struct")
  # allocate to an 'enum' type only as many bytes as it needs for the declared
  # range of possible values"
  Append(CC_FLAGS "-fshort-enums")
  # place each function or data into its own section in the output file if the
  # target supports arbitrary sections.  for linker optimizations, and
  # discarding unused code.
  Append(CC_FLAGS "-ffunction-sections -fdata-sections")

  # Set language specific flags for cmake
  Append(C_FLAGS "-std=gnu99 -Wstrict-prototypes")
  Append(CXX_FLAGS "-std=gnu++11")

  # Create LINK_FLAGS from our CC_FLAGS (needed for gcc's -flto if we use it) 
  set(LINK_FLAGS "${CC_FLAGS}")
  # generate a link map, with a cross reference table
  Append(LINK_FLAGS "-Wl,-Map=${TARGET}.map,--cref")
  # make sure the map is cleaned
  AddCleanFiles("${TARGET}.map")
  # for some linker optimizations
  Append(LINK_FLAGS "-Wl,--relax")
  # discard unused functions and data
  Append(LINK_FLAGS "-Wl,--gc-sections")

  # Add extra CC_FLAGS we don't want in LINK_FLAGS
  # enable generating dep files
  Append(CC_FLAGS "-MMD -MP -MF ${TARGET}.dep")
  # make sure the dep is cleaned
  AddCleanFiles("${TARGET}.dep")
endif()

# sources
list(APPEND CXX_SOURCES "src/main.cc")

# build the elf
AddProgram("${TARGET}.elf")

# create the hex, eep, and print the summary output (including .hex/.eep sizes)
add_custom_target(run ALL
    COMMAND  # .hex
    "${CMAKE_OBJCOPY}" -O "${FORMAT}"
    -R .eeprom -R .fuse -R .lock -R .signature
    "${TARGET}.elf" "${TARGET}.hex"
    COMMAND  # .eep
    "${CMAKE_OBJCOPY}" -O "${FORMAT}"
    -j .eeprom
    --set-section-flags=.eeprom="alloc,load"
    --change-section-lma .eeprom=0
    --no-change-warnings
    "${TARGET}.elf" "${TARGET}.eep"
    # summary output
    ${ECHO}
    ${ECHO} "---------------------------------------------------------------"
    ${ECHO}
    COMMAND  # sizes
    "${AVR_SIZE_TOOL}" --target="${FORMAT}" "${TARGET}.hex" "${TARGET}.eep"
    ${ECHO}
    ${ECHO} "you can load \"firmware.hex\" and \"firmware.eep\" onto the"
    ${ECHO} "Teensy using the Teensy loader"
    ${ECHO}
    ${ECHO} "---------------------------------------------------------------"
    ${ECHO}
    DEPENDS "${TARGET}.elf")
# make sure the hex and eep are cleaned
AddCleanFiles("${TARGET}.hex" "${TARGET}.eep")
