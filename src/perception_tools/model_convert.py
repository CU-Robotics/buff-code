#! /usr/bin/python3

import numpy as np
import os
import cv2
import torch
from rknn.api import RKNN
import torchvision.models as models


QUANTIZE_ON = True
ONNX_MODEL = 'buffnet.onnx'
RKNN_MODEL = 'buffnet.rknn'
DATASET = os.path.join(os.getenv("PROJECT_ROOT"), "buffpy", "lib", "dataset.txt")

if __name__ == '__main__':

	model = os.path.join(os.getenv("PROJECT_ROOT"), "buffpy", "data", "models", ONNX_MODEL)
	# if not os.path.exists(model):
		# export_pytorch_model()

	input_size_list = [[1, 3, 320, 320]]

	# Create RKNN object
	rknn = RKNN(verbose=True)

	# pre-process config
	print('--> Config model')
	rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform='rk3588')
	print('done')

	# Load ONNX model
	print('--> Loading model')
	ret = rknn.load_onnx(model=model)
	if ret != 0:
		print('Load model failed!')
		exit(ret)
	print('done')

	# Build model
	print('--> Building model')
	ret = rknn.build(do_quantization=QUANTIZE_ON, dataset=DATASET)
	if ret != 0:
		print('Build model failed!')
		exit(ret)
	print('done')

	# Export RKNN model
	print('--> Export rknn model')
	ret = rknn.export_rknn(os.path.join(os.getenv("PROJECT_ROOT"), "buffpy", "data", "models", RKNN_MODEL))
	if ret != 0:
		print('Export rknn model failed!')
		exit(ret)
	print('done')