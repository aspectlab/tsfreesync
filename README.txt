#####################################################################
Timestamp Free Synchronization

Written by:

A.G.Klein, M.Overdick, J.E.Canfield
#####################################################################

This is a simple instruction file on how to build this project.

1) First make sure you have CMake, the oecore cross compiler, and the UHD API installed on your
machine.

2) For ./masternode, ./slavenode, ./rxnode, and ./txnode folder you need to create a build folder for each program you want to build.

3) Inside each build folder, call the following commands:

    source /usr/local/oecore-x86_64/environment-setup-armv7ahf-vfp-neon-oe-linux-gnueabi

    cmake .. -DCMAKE_TOOLCHAIN_FILE=$OECORE_NATIVE_SYSROOT/usr/share/cmake/OEToolchainConfig.cmake

This configures the build environment for cross compiling.

4) In each build directory you can now call "make" to build the MasterNode, SlaveNode, RXNode, and TXNode

##################################
A little about each program
##################################

MasterNode:
    The masternode is a simple program that listens on one channel for a "ping" from the slave node. When it detecs one, it returns a time reversed response of the "ping" signal it received back to the slave at known delay.
    
SlaveNode:
    The slavenode sends "pings" to the masternode constantly, it uses the time between transmitting and receiving a "ping" to determine the phase of the clock on the masternode, and then adjusts its own clock accordingly.
    
RXNode:
    RXNode is for recording two channels for a configurable amount of time. After the recording is completed, the files are saved to a *.dat file for analysis in applications such as Octave and MATLAB.
    
TXNode:
    TXNode is for transmitting pulses on two channels indefinitely. This program is intended for diagnostics of the USRP hardware to test the phase difference between TX channels on the device and RX channels on a recording device such as the RXNode. 
