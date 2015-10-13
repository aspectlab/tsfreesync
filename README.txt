#####################################################################
Timestamp Free Synchronization

Written by:

A.G.Klein, M.Overdick, J.E.Canfield
#####################################################################

This is a simple instruction file on how to build this project.

1) First make sure you have CMake, the oecore cross compiler, and the UHD API installed on your
machine.

2) For both ./masternode and ./slavenode you need to create a build folder.

3) Inside each build folder, call the following commands:

    source /usr/local/oecore-x86_64/environment-setup-armv7ahf-vfp-neon-oe-linux-gnueabi

    cmake .. -DCMAKE_TOOLCHAIN_FILE=$OECORE_NATIVE_SYSROOT/usr/share/cmake/OEToolchainConfig.cmake

This configures the build environment for cross compiling.

4) In each build directory you can now call "make" to build the master and slave node