#!/bin/bash

source .envvars || SANE_ENV=1
BRDEFCFG="raspberrypi0w"

function check() {
	# todo...
	return 0
} 

function clean() {
	rm -rf ${ROOTDIR}/payload/dependencies*
	rm -rf rpi-${PROJECT}-buildroot/rpi-${PROJECT}-buildroot/rpi-${PROJECT}-tree/datapart
	rm -rf ${ROOTDIR}/rpi-${PROJECT}-buildroot/rpi-${PROJECT}-tree/overlay/usr/local/zdbrg/zdbrg.py
	rm -rf ${ROOTDIR}/build
	return 0
}

function zdbrg_main() {
	cd ${ROOTDIR}/payload || return 1
	mkdir -p dependencies
	pip download -r requirements.txt -d "./dependencies" || return 1
	tar czf "dependencies-${PROJECT}.tar.gz" dependencies || return 1
	return 0
}

function prepare() {
	cd ${ROOTDIR}
	mkdir -p build || return 1
	mkdir -p payload/dependencies || return 1
	mkdir -p rpi-${PROJECT}-buildroot/rpi-${PROJECT}-tree/datapart/payload || return 1
	return 0
}

function buildroot_prepare() {
	cd ${ROOTDIR}/build || return 1
	if ! [ -d buildroot ]; then
		echo "Downloading buildroot ${BRVER}"
		mkdir -p buildroot || return 1
		wget ${BRDL} || return 1
		[ $? != 0 ] && echo "Error downloading buildroot!" && return 1
		tar -xf ${BRTXZ} --strip-components=1 -C buildroot/
		[ $? != 0 ] && echo "Error unpacking buildroot!" && return 1
		rm ${BRTXZ}
	else
		echo "Buildroot-folder already exists. Run '${} clean' if encountering any problems."
	fi
	return 0
}

function buildroot_configure() {
	echo
	echo "Configuring buldroot: ${PROJECT} for ${BRDEFCFG} ..."
	make defconfig BR2_DEFCONFIG=${ROOTDIR}/rpi-${PROJECT}-buildroot/rpi-${PROJECT}-tree/configs/${BRDEFCFG}_defconfig BR2_EXTERNAL=${ROOTDIR}/rpi-${PROJECT}-buildroot/rpi-${PROJECT}-tree/ O=${ROOTDIR}/build -C ${ROOTDIR}/build/buildroot/ 2> /dev/null > /dev/null || return 1
	return 0
}

function build() {
	cd ${ROOTDIR}/build || return 1
	echo
	echo "Building '${PROJECT} for ${BRDEFCFG}'. Starting build in 10 seconds!"
	echo "Hit <CTRL-C> to cancel!"
	sleep 10s
	make || return 1
	return 0
}

if [ $SANE_ENV ]; then
	echo "Sanity check failed"
	exit 1
fi
echo
echo "'ZdBrg v${SYSVER}' build-script"
echo "'${0} clean' to remove previous build"
if [ "${1}" == "clean" ]; then
	echo
	echo "NEXT: Clean-up"
	clean && echo "SUCCESS: Clean-up" || exit 1
	exit 0
fi
sleep 2
echo
echo "${PROJECT}: buildroot v${BRVER}, target device: ${BRDEFCFG}"
sleep 2
echo
echo "=> Sanity check ..."
check && echo "SUCCESS: Sanity check" || exit 1
echo
echo "=> Initial preparation"
prepare && echo "SUCCESS: Initial preparation" || exit 1
echo
echo "=> ${PROJECT}-payload preparation"
zdbrg_main && echo "SUCCESS: ${PROJECT}-payload preparation" || exit 1
echo
echo "=> Buildroot preparation"
buildroot_prepare && echo "SUCCESS: Buildroot preparation" || exit 1
echo
echo "=> Buildroot configuration"
buildroot_configure && echo "SUCCESS: Buildroot configuration" || exit 1
echo
echo "=> Build"
build && echo "SUCCESS: Build" || exit 1
echo
echo "Resulting (sdard) image: 'build/images/sdcard.img'"

exit 0
