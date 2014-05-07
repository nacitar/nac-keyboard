#!/bin/bash

cd "$(dirname "$0")"

root_dir="$PWD"
cmake_repo_dir="$root_dir/external/cmake"

[ -d 'build' ] && rm -rf build
mkdir -p build/avr || exit 1


pushd build/avr &&
  cmake -DCMAKE_TOOLCHAIN_FILE=$cmake_repo_dir/avr_toolchain.cmake $root_dir &&
  popd
