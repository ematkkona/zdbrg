#!/bin/bash

source projectnfo
DIR=`pwd`
mkdir -p rpi-zdbrg-buildroot/rpi-zdbrg-tree/datapart/payload
cd ${PROJECT}-main
./build.sh
cd ${DIR}
cd build
./configure.sh && echo OK || exit 1
# Start build
echo "ZdBrg-buildroot for RaspberryPi0W configured."
echo "Starting build in 10 seconds! This can take quite a while ... Hit <CTRL-C> to abort NOW!"
sleep 10s
make && echo OK || exit 1

echo "All done! Image of the build is located in: 'zdbrg/software/build/images/sdcard.img'"
echo "Located in this folder a script: [imgtosd.sh], which can be used to write the image to a SD card. Usage: ./imgtosd.sh <device> (ie. /dev/sda, /dev/mmcblk etc)"
exit 0
