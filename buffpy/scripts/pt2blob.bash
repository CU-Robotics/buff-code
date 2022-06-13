#!/bin/bash

source ${PROJECT_ROOT}/buffpy/scripts/yolov5_setup.bash 
source ${PROJECT_ROOT}/buffpy/scripts/openvino_setup.bash 

cd ${PROJECT_ROOT}/../yolov5

FILENAME=$1
PT_FILE="${PROJECT_ROOT}/buffpy/models/${FILENAME}.pt"

python3 export.py --weights "${PT_FILE}" \
	--img 320 \
	--batch 1 \
	--device cpu \
	--include "onnx" \
	--simplify 

cd ${PROJECT_ROOT}


source /opt/intel/openvino_2021/bin/setupvars.sh
python3 /opt/intel/openvino_2021/deployment_tools/model_optimizer/mo.py \
	--input_model  "${PROJECT_ROOT}/buffpy/models/${FILENAME}.onnx" \
	--model_name "${FILENAME}" \
	--data_type FP16 \
	--output_dir "${PROJECT_ROOT}/buffpy/models/" \
	--input_shape [1,3,320,320] \
	--reverse_input_channel \
	--scale 255


python3 ${PROJECT_ROOT}/buffpy/scripts/onnx2blob.py ${FILENAME}

mv "${PROJECT_ROOT}/buffpy/models/${FILENAME}_openvino_2021.4_6shave.blob" "${PROJECT_ROOT}/buffpy/models/${FILENAME}.blob"