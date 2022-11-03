#!/bin/bash

ROOTDIR=$(pwd)
IMAGE="${ROOTDIR}/rpi-zdbrg-buildroot/build_workdir/images/sdcard.img"

echo "ZdBrg-Image to SD-card utility"

if [ "${1}" != "" ]; then
	TARGET="${1}"
else
	echo "Use: ${0} [target-device] (fe. /dev/sda)"
	exit 1
fi

if [ -f $IMAGE ]; then
	file $IMAGE
	sudo dd if=$IMAGE of=$TARGET bs=32k conv=fsync status=progress && sync
	if [ $? != 0 ]; then
		echo "Image transfer failed!"
		exit 1
	else
		echo "Success! If automounted, unmount the SD-card before removing it"
	fi
else
	echo "Image file not found!"
	exit 1
fi

echo "Done"
exit 0
	



