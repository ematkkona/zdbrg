#!/bin/bash

source projectnfo
DIR=`pwd`
mkdir -p rpi-zdbrg-buildroot/rpi-zdbrg-tree/datapart/payload
cd ${PROJECT}-main
./build.sh
cd ${DIR}
cd build
./configure.sh
# Start build
make menuconfig
make && echo OK || echo ERROR

exit 0
