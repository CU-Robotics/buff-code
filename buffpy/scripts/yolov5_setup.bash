#!/bin/bash



cd ${PROJECT_ROOT}/..
if [[ ! -d "yolov5" ]]; then 
	git clone https://github.com/airockchip/yolov5  # clone repo next to buff-code
	pip install torch==1.10.1+cu111 torchvision==0.11.2+cu111 tensorboard -f https://download.pytorch.org/whl/torch_stable.html
fi

pip install tqdm

cd ${PROJECT_ROOT}
