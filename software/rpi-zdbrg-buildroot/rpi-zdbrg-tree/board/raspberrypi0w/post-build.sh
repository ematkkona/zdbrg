#!/bin/sh

set -u
set -e

IMAGE_DIR="${BASE_DIR}/images"

# Build external dtb's and install to target (boot/rpi-firmware/overlays)
cd ${BR2_EXTERNAL_RPI_ZDBRG_PATH}/dtbs
make all
cp -f *.dtbo ${IMAGE_DIR}/rpi-firmware/overlays/.
make clean

# Copy over cmdline.txt and config.txt
cp -f ${BR2_EXTERNAL_RPI_ZDBRG_PATH}/board/raspberrypi0w/config_0w.txt ${IMAGE_DIR}/rpi-firmware/config.txt
cp -f ${BR2_EXTERNAL_RPI_ZDBRG_PATH}/board/raspberrypi0w/cmdline_0w.txt ${IMAGE_DIR}/rpi-firmware/cmdline.txt

# Add a console on tty1
if [ -e ${TARGET_DIR}/etc/inittab ]; then
    grep -qE '^tty1::' ${TARGET_DIR}/etc/inittab || \
	sed -i '/GENERIC_SERIAL/a\
tty1::respawn:/sbin/getty -L  tty1 0 vt100 # HDMI console' ${TARGET_DIR}/etc/inittab
fi
