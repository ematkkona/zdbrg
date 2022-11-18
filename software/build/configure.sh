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
	# Setup buidroot external tree
	pwd
	make BR2_EXTERNAL=$DIR/../rpi-zdbrg-buildroot/rpi-zdbrg-tree O=$DIR -C $DIR/../rpi-zdbrg-buildroot/buildroot/ 2> /dev/null > /dev/null
	cp ../rpi-zdbrg-buildroot/rpi-zdbrg-tree/configs/zdbrg.config .config
fi

exit 0
