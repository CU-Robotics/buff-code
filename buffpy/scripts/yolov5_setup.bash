#!/bin/bash


#sudo add-apt-repository ppa:deadsnakes/ppa


cd ${PROJECT_ROOT}/..
git clone https://github.com/ultralytics/yolov5  # clone repo next to buff-code
cd ${PROJECT_ROOT}/../yolov5

pip install torch==1.9.0+cu111 torchvision==0.10.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html
