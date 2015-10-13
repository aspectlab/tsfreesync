#!/bin/bash

source /usr/local/oecore-x86_64_orig/environment-setup-armv7ahf-vfp-neon-oe-linux-gnueabi

cmake .. -DCMAKE_TOOLCHAIN_FILE=$OECORE_NATIVE_SYSROOT/usr/share/cmake/OEToolchainConfig.cmake
