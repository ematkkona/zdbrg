#!/bin/bash

# Get directory of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Check if Makefile has already been generated
# Do not procede if Makefile has been created
if [ ! -f $DIR/Makefile ]; then
	# Clean output directory
	find $DIR -type f ! -name ".gitignore" ! -name "configure.sh" -delete
	find $DIR -empty -type d -delete
	# Download buildroot if missing
	cd ../rpi-zdbrg-buildroot
	if ! [ -d buildroot ]; then
		mkdir buildroot
		source brver
		wget ${BRDL}
		[ $? != 0 ] && echo "Error downloading buildroot!" && exit 1
		tar -xf ${BRTXZ} --strip-components=1 -C buildroot/
		[ $? != 0 ] && echo "Error unpacking buildroot!" && exit 1
		rm -rf ${BRTXZ}
		cd $DIR
	fi
	# Make zdbrg defconfig (for raspberrypi0w)
	make defconfig BR2_DEFCONFIG=$DIR/../rpi-zdbrg-buildroot/rpi-zdbrg-tree/configs/raspberrypi0w_defconfig BR2_EXTERNAL=$DIR/../rpi-zdbrg-buildroot/rpi-zdbrg-tree/ O=$DIR -C $DIR/../rpi-zdbrg-buildroot/buildroot/
fi

exit 0
