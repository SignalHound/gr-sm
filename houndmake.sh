#!/usr/bin/env bash

# Get Signal Hound files
rules=(sh_lib/*.rules)
rule="${rules[0]}"

libs=(sh_lib/lib/libsm_api.so.*)
lib="${libs[0]}"

# Preparation
sudo cp $rule /etc/udev/rules.d
ldconfig -v -n sh_lib/lib
ln -sf $lib sh_lib/lib/libsm_api.so
sudo cp sh_lib/lib/libsm_api.* /usr/local/lib
ldconfig -v -n sh_lib/lib

# Install
mkdir build
cmake
cd build
make
sudo make install
sudo ldconfig
cd ..
