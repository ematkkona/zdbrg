#!/bin/bash

source projectnfo
DIR=`pwd`
echo $DIR
cd ${PROJECT}-main
./build.sh
cd ${DIR}
cd build
./configure.sh
# Start build
make menuconfig
make && echo OK || echo ERROR

exit 0
