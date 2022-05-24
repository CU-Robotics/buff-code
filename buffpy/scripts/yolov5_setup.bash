#!/bin/bash

cd ${PROJECT_ROOT}/..
git clone https://github.com/ultralytics/yolov5  # clone repo next to buff-code
cd ${PROJECT_ROOT}/../yolov5
pip install -qr requirements.txt # install dependencies