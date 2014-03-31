#!/bin/bash
[ -d 'build' ] && rm -rf build
mkdir build || exit 1
pushd build && cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/avr_toolchain.cmake .. && popd
