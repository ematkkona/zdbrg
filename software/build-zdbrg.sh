#!/bin/bash

source projectnfo
DIR=`pwd`

cd ${PROJECT}-main
./build.sh
cd $DIR/build
./configure
# Start build
make
printf "Done. Start your build:\n cd ${ROOTDIR}/rpi-${PROJECT}-buildroot/build_workdir\n make -j4\n"

exit 0
