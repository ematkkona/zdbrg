#!/bin/bash

source ../projectnfo

echo "Downloading '${PROJECT}' dependencies ..."
cd ${PROJECT}
mkdir dependencies
pip download -r requirements.txt -d "./dependencies"
tar cvfz "dependencies-${PROJECT}.tar.gz" dependencies
cp dependencies-${PROJECT}.tar.gz ${MTARGET}/dependencies-${PROJECT}.tar.gz
cp ${PROJECT}.py ${MTARGET}/${PROJECT}.py
echo "'${PROJECT}' prepared. Target: ${MTARGET}"

exit 0
