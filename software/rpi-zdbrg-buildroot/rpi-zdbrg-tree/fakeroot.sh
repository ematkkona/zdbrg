#!/bin/sh

echo "fakeroot.sh"

versions() {
	echo "Fetch version info (>> /root/.version):"
	source ${BASE_DIR}/../rpi-zdbrg-buildroot/brver || exit 1
	source ${BASE_DIR}/../../.ver || exit 1
	MAIN=$(cat ${TARGET_DIR}/usr/local/zdbrg/zdbrg.py | grep -m1 Version | cut -d "'" -f2) || exit 1
	VTARGET="${TARGET_DIR}/root/.version"
	[ -f ${VTARGET} ] || rm -rf ${VTARGET}
	touch ${VTARGET} || exit 1
	echo "SYSVER='${SYSVER}'" > ${VTARGET}
	echo "MAINVER='${MAIN}'" >> ${VTARGET}
	echo "BUILDVER='br-${BRVER}'" >> ${VTARGET}
	cat ${VTARGET}
}

echo "Fetch latest main program ..."
cp -f ${BASE_DIR}/../zdbrg-main/zdbrg/zdbrg.py ${TARGET_DIR}/usr/local/zdbrg/zdbrg.py
if [ $? != 0 ]; then
	echo "Err zdbrg.py"
	exit 1
fi
cd ${TARGET_DIR}/etc/init.d
if [ -f S01syslogd ]; then
	rm S01syslogd
	echo "Removed S01syslogd"
fi
if [ -f S02klogd ]; then
	rm S02klogd
	echo "Removed S02klogd"
fi
if [ -f S40network ]; then
	rm S40network
	echo "Removed S40network"
fi
cd ${TARGET_DIR}/etc
if [ -d network ]; then
	rm -rf network
	echo "Removed /etc/network"
fi
versions

echo "fakeroot.sh; done"


exit 0
