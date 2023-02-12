#!/bin/bash

cd ${PROJECT_ROOT}
    cd "buff-code/src/efficientdet-d0-trainer/models/research/"
    researchDir=$(pwd)
    cd ${PROJECT_ROOT}
    cd $researchDir
    pwd


cd ${PROJECT_ROOT} 
cd buff-code/src
if [[ ! -d "efficientdet-d0-trainer" ]]; then

    # Clones repo into buff-code/src/
    git clone https://github.com/ethan-wst/efficientdet-d0-trainer.git 
    
    cd ${PROJECT_ROOT}
    cd "buff-code/src/efficientdet-d0-trainer/models/research/"
    researchDir=$(pwd)
    
    cd ${PROJECT_ROOT}
    cd "buff-code/src/efficientdet-d0-trainer/software/protobuf-21.12/"
    protoDir=$(pwd)
    
    
    bashrcExport="export PATH=\"${protoDir}/bin:PATH\""

    # Installs tensorflow
    pip install --ignore-installed --upgrade tensorflow==2.10.1
    
    # Appends Protobuf to .bashrc
    cd ${PROJECT_ROOT}
    echo $bashrcExport >> ~/.bashrc 
    reset
    
    cd ${PROJECT_ROOT}
    cd $researchDir
    protoc object_detection/protos/*.proto --python_out=.

    # Clones COCO repo and makes pythonAPI
    cd ${PROJECT_ROOT}
    cd buff-code/src/efficientdet-d0-trainer/software
    git clone https://github.com/cocodataset/cocoapi.git
    cd cocoapi/PythonAPI
    pip install cython
    make
    
    # Copies pycocotools in /efficientdet-d0-trainer/models/research/
    cp -r pycocotools $researchDir

    # Installs Object Detection API
    cd ${PROJECT_ROOT}
    cd $researchDir
    cp object_detection/packages/tf2/setup.py .
    python -m pip install --use-feature=2020-resolver . 

    # Installs Panda Package
    pip install pandas

    echo -e "\n"
    echo "Recommended that you test the instalations of TensorFlow and the Object"
    echo "Detection API with the installation verification instruction at the github:"
    echo "https://github.com/ethan-wst/efficientdet-d0-trainer.git"
    echo -e "\n"
fi
