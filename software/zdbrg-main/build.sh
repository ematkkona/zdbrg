#!/bin/bash

source ../projectnfo

echo "Downloading '${PROJECT}' dependencies ..."
cd ${PROJECT}
mkdir dependencies
pip download -r requirements.txt -d "./dependencies"
tar cvfz "dependencies-${PROJECT}.tar.gz" dependencies
exit 0
