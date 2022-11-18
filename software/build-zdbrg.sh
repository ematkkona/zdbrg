#!/bin/bash

source projectnfo
DIR=`pwd`

cd ${PROJECT}-main
./build.sh
cd $DIR/build
./configure
# Start build
make && echo OK || echo ERROR

exit 0
