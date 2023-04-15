#! /usr/bin/python3

import numpy as np
import cv2
from rknn.api import RKNN
import torchvision.models as models
import torch
import os


def export_pytorch_model():
    model_path = os.path.join(os.getenv("PROJECT_ROOT"), "buffpy", "data", "models", "buffnet.pt")
    model = torch.hub.load('ultralytics/yolov5', 'custom', model_path)
    model.eval()

    trace_model = torch.jit.script(model, torch.Tensor(1, 3, 320, 320))
    trace_model.save(os.path.join(os.getenv("PROJECT_ROOT"), "buffpy", "data", "models", "buffnet_trace.pt"))


# def show_outputs(output):
#     output_sorted = sorted(output, reverse=True)
#     top5_str = '\n-----TOP 5-----\n'
#     for i in range(5):
#         value = output_sorted[i]
#         index = np.where(output == value)
#         for j in range(len(index)):
#             if (i + j) >= 5:
#                 break
#             if value > 0:
#                 topi = '{}: {}\n'.format(index[j], value)
#             else:
#                 topi = '-1: 0.0\n'
#             top5_str += topi
#     print(top5_str)


# def show_perfs(perfs):
#     perfs = 'perfs: {}\n'.format(perfs)
#     print(perfs)


# def softmax(x):
#     return np.exp(x)/sum(np.exp(x))


if __name__ == '__main__':

    model = os.path.join(os.getenv("PROJECT_ROOT"), "buffpy", "data", "models", "buffnet.pt")
    # if not os.path.exists(model):
        # export_pytorch_model()

    input_size_list = [[1, 3, 320, 320]]

    # Create RKNN object
    rknn = RKNN(verbose=True)

    # Pre-process config
    print('--> Config model')
    rknn.config(mean_values=[123.675, 116.28, 103.53], std_values=[58.395, 58.395, 58.395], target_platform='rk3588s')
    print('done')

    # Load model
    print('--> Loading model')
    ret = rknn.load_pytorch(model=model, input_size_list=input_size_list)
    if ret != 0:
        print('Load model failed!')
        exit(ret)
    print('done')

    # Build model
    print('--> Building model')
    ret = rknn.build(do_quantization=True, dataset='./dataset.txt')
    if ret != 0:
        print('Build model failed!')
        exit(ret)
    print('done')

    # Export rknn model
    print('--> Export rknn model')
    ret = rknn.export_rknn('./buffnet.rknn')
    if ret != 0:
        print('Export rknn model failed!')
        exit(ret)
    print('done')