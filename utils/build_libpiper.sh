#!/bin/bash
set -e

echo "--- Building Piper Static Library ---"
# Remove existing directory if it exists to ensure a clean build
rm -rf piper-source

git clone https://github.com/OHF-Voice/piper1-gpl piper-source
cd piper-source/libpiper

cmake -Bbuild-deps \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install" \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON

cmake --build build-deps
cmake --install build-deps

echo "--- Verify install ---"
ls -R "$PWD/install"