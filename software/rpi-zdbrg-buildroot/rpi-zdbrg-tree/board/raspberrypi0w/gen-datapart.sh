#!/bin/bash

set -e

BOARD_DIR="$(dirname $0)"
BOARD_NAME="$(basename ${BOARD_DIR})"
GENIMAGE_CFG="${BR2_EXTERNAL_RPI_ZDBRG_PATH}/board/${BOARD_NAME}/genimage-${BOARD_NAME}-data.cfg"
GENIMAGE_TMP="${BUILD_DIR}/genimage-data.tmp"
ROOTPATH="${BR2_EXTERNAL_RPI_ZDBRG_PATH}"

rm -rf "${GENIMAGE_TMP}"

genimage \
	--rootpath "${ROOTPATH}"   \
	--tmppath "${GENIMAGE_TMP}"    \
	--inputpath "${BINARIES_DIR}"  \
	--outputpath "${BINARIES_DIR}" \
	--config "${GENIMAGE_CFG}"

exit $?
