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

project (nx-firmware CXX)
cmake_minimum_required(VERSION 2.6.2)

# Appends values to a custom-delimited string
macro(AppendDelimited var delimiter)
  foreach(value ${ARGN})
    set("${var}" "${${var}}${delimiter}${value}")
  endforeach()
endmacro()
# Appends values to a space-delimited string
macro(Append var)
  AppendDelimited("${var}" " " ${ARGN})
endmacro()
# Appends values to a semicolon-delimited string
macro(AppendSemi var)
  AppendDelimited("${var}" ";" ${ARGN})
endmacro()

# target name
set(TARGET "nx-firmware")
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

set(EXTRA_OUTPUTS "")
# Compilation settings
if(GCC_COMPAT)
  # Create CCFLAGS (unused by default) to store common C/C++ flags
  set(CCFLAGS "")
  # pass in hardware settings
  Append(CCFLAGS "-mmcu=${MCU} -DF_CPU=${F_CPU}")
  # enable common warnings
  Append(CCFLAGS "-Wall")
  # pack all structure members together without holes
  Append(CCFLAGS "-fpack-struct")
  # allocate to an 'enum' type only as many bytes as it needs for the declared
  # range of possible values"
  Append(CCFLAGS "-fshort-enums")
  # place each function or data into its own section in the output file if the
  # target supports arbitrary sections.  for linker optimizations, and
  # discarding unused code.
  Append(CCFLAGS "-ffunction-sections -fdata-sections")
  # set options
  Append(CCFLAGS "-DMAKEFILE_BOARD='${BOARD}'")
  Append(CCFLAGS "-DMAKEFILE_KEYBOARD='${KEYBOARD}'")
  Append(CCFLAGS "-DMAKEFILE_KEYBOARD_LAYOUT='${LAYOUT}'")
  Append(CCFLAGS "-DMAKEFILE_DEBOUNCE_TIME='${DEBOUNCE_TIME}'")
  Append(CCFLAGS "-DMAKEFILE_LED_BRIGHTNESS='${LED_BRIGHTNESS}'")

  # Set language specific flags for cmake
  Append(CMAKE_C_FLAGS "-std=gnu99 -Wstrict-prototypes")
  Append(CMAKE_CXX_FLAGS "-std=gnu++11")

  # Create LINKFLAGS (unused by default) to store desired linker flags
  set(LINKFLAGS "")
  # generate a link map, with a cross reference table
  Append(LINKFLAGS "-Wl,-Map=${TARGET}.map,--cref")
  AppendSemi(EXTRA_OUTPUTS "${TARGET}.map")
  # for some linker optimizations
  Append(LINKFLAGS "-Wl,--relax")
  # discard unused functions and data
  Append(LINKFLAGS "-Wl,--gc-sections")

  # Create GENDEPFLAGS (unused by default) to enable generating dep files
  set(GENDEPFLAGS "-MMD -MP -MF ${TARGET}.dep")
  AppendSemi(EXTRA_OUTPUTS "${TARGET}.dep")
endif()


# build the elf
add_executable("${TARGET}.elf" "src/main.cc")
set_target_properties("${TARGET}.elf" PROPERTIES COMPILE_FLAGS
    "${CCFLAGS} ${GENDEPFLAGS}")
set_target_properties("${TARGET}.elf" PROPERTIES LINK_FLAGS
    "${CCFLAGS} ${LINKFLAGS}")

# create the hex
add_custom_command(TARGET "${TARGET}.elf" POST_BUILD COMMAND
    "${CMAKE_OBJCOPY}" -O "${FORMAT}"
    -R .eeprom -R .fuse -R .lock -R .signature
    "${TARGET}.elf" "${TARGET}.hex")
AppendSemi(EXTRA_OUTPUTS "${TARGET}.hex")

# create the eep
add_custom_command(TARGET "${TARGET}.elf" POST_BUILD COMMAND
    "${CMAKE_OBJCOPY}" -O "${FORMAT}"
    -j .eeprom
    --set-section-flags=.eeprom="alloc,load"
    --change-section-lma .eeprom=0
    --no-change-warnings
    "${TARGET}.elf" "${TARGET}.eep")
AppendSemi(EXTRA_OUTPUTS "${TARGET}.eep")

# Set the extra outputs so things clean correctly
set_directory_properties(PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${EXTRA_OUTPUTS}")

# Show the sizes
if (CMAKE_SYSTEM_NAME STREQUAL "AVR")
  add_custom_command(TARGET "${TARGET}.elf" POST_BUILD COMMAND
    "${AVR_SIZE_TOOL}" --target="${FORMAT}" "${TARGET}.hex" "${TARGET}.eep")
endif()

