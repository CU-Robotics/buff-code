#!/bin/bash



cd ${PROJECT_ROOT}/..
if [[ ! -d "yolov5" ]]; then 
	git clone https://github.com/ultralytics/yolov5  # clone repo next to buff-code
	pip install torch==1.9.0+cu111 torchvision==0.10.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html
fi

cd ${PROJECT_ROOT}
