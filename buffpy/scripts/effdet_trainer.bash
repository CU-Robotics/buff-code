#!/bin/bash

EFFDET_SRC="${PROJECT_ROOT}/src/efficientdet-d0-trainer"
RESEARCH_DIR="${EFFDET_SRC}/models/research"
PROTO_DIR="${EFFDET_SRC}/software/protobuf-21.12"

echo ${EFFDET_SRC}
# Rebuild everytime this is run
if [[ -d "${EFFDET_SRC}" ]]; then
    rm -rf ${EFFDET_SRC}
fi

# Clones repo into buff-code/src/
cd "${PROJECT_ROOT}/src" && \
    git clone https://github.com/ethan-wst/efficientdet-d0-trainer.git 

# add protodir to the path 
export PATH="${PATH}:${PROTO_DIR}/bin"

# Installs tensorflow & numpy
pip install --ignore-installed --upgrade tensorflow #==2.11.0
# pip install numpy==1.22.0

# Appends Protobuf to .bashrc (don't mess with bashrc leave that up to users)
# echo $bashrcExport >> ~/.bashrc 
# reset

cd "${RESEARCH_DIR}" && \
    protoc object_detection/protos/*.proto --python_out=.

# Clones COCO repo and makes pythonAPI
# Copies pycocotools in /efficientdet-d0-trainer/models/research/
cd "${EFFDET_SRC}/software" && \
    git clone https://github.com/cocodataset/cocoapi.git
    cd "cocoapi/PythonAPI" && \
    make && \
    cp -r pycocotools $RESEARCH_DIR

# Installs Object Detection API
cd "${RESEARCH_DIR}" && \
    cp object_detection/packages/tf2/setup.py . && \
    python3 -m pip install --use-pep517 .

# Installs Panda Package
# pip install pandas

echo -e "\n"
echo "Recommended that you test the instalations of TensorFlow and the Object"
echo "Detection API with the installation verification instruction at the github:"
echo "https://github.com/ethan-wst/efficientdet-d0-trainer.git"
echo -e "\n"

cd ${PROJECT_ROOT}
