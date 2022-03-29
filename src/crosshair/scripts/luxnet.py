#! /usr/bin/env python3
"""
	Project:
			buffnet detector
	Author: Mitchell D Scott
	Description:
		Detects and displays images
"""
import os
import sys
import cv2
import yaml
import rospy
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from gdrive_handler import GD_Handler
from std_msgs.msg import Float64MultiArray

import depthai as dai
import time
import numpy as np

# functions taken from https://github.com/ultralytics/yolov5/blob/master/utils/general.py

import torch
import torchvision
import time
import numpy as np


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = torch.zeros_like(x) if isinstance(
        x, torch.Tensor) else np.zeros_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def non_max_suppression(prediction, conf_thres=0.1, iou_thres=0.6, merge=False, classes=None, agnostic=False):
    """Performs Non-Maximum Suppression (NMS) on inference results
    Returns:
         detections with shape: nx6 (x1, y1, x2, y2, conf, cls)
    """
    prediction = torch.from_numpy(prediction)
    if prediction.dtype is torch.float16:
        prediction = prediction.float()  # to FP32

    nc = prediction[0].shape[1] - 5  # number of classes
    xc = prediction[..., 4] > conf_thres  # candidates

    # Settings
    # (pixels) minimum and maximum box width and height
    min_wh, max_wh = 2, 4096
    max_det = 500  # maximum number of detections per image
    time_limit = 10.0  # seconds to quit after
    multi_label = nc > 1  # multiple labels per box (adds 0.5ms/img)

    t = time.time()
    output = [None] * prediction.shape[0]
    for xi, x in enumerate(prediction):  # image index, image inference
        # Apply constraints
        # x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height
        x = x[xc[xi]]  # confidence

        # If none remain process next image
        if not x.shape[0]:
            continue

        # Compute conf
        x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf

        # Box (center x, center y, width, height) to (x1, y1, x2, y2)
        box = xywh2xyxy(x[:, :4])

        # Detections matrix nx6 (xyxy, conf, cls)
        if multi_label:
            i, j = (x[:, 5:] > conf_thres).nonzero(as_tuple=False).T
            x = torch.cat((box[i], x[i, j + 5, None], j[:, None].float()), 1)
        else:  # best class only
            conf, j = x[:, 5:].max(1, keepdim=True)
            x = torch.cat((box, conf, j.float()), 1)[
                conf.view(-1) > conf_thres]

        # Filter by class
        if classes:
            x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

        # Apply finite constraint
        # if not torch.isfinite(x).all():
        #     x = x[torch.isfinite(x).all(1)]

        # If none remain process next image
        n = x.shape[0]  # number of boxes
        if not n:
            continue

        # Sort by confidence
        #x = x[x[:, 4].argsort(descending=True)]

        # Batched NMS
        c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
        # boxes (offset by class), scores
        boxes, scores = x[:, :4] + c, x[:, 4]
        i = torchvision.ops.boxes.nms(boxes, scores, iou_thres)
        if i.shape[0] > max_det:  # limit detections
            i = i[:max_det]

        output[xi] = x[i]
        if (time.time() - t) > time_limit:
            break  # time limit exceeded

    return output


labelMap = [
    'armor',
    'base',
    'car',
    'target',
    'target-blue',
    'target-grey',
    'target-grey-2',
    'target-red',
    'watcher',
    'background'
]

cam_options = ['rgb', 'left', 'right']


def draw_boxes(frame, boxes, total_classes):
    if len(boxes) == 0:
        return frame
    else:

        # define class colors
        colors = boxes[5] * (255 / total_classes)
        colors = colors.astype(np.uint8)
        colors = cv2.applyColorMap(colors, cv2.COLORMAP_HSV)
        colors = np.array(colors)

        for i in range(boxes.shape[0]):
            x1, y1, x2, y2 = int(boxes[i, 0]), int(
                boxes[i, 1]), int(boxes[i, 2]), int(boxes[i, 3])
            conf, cls = boxes[i, 4], int(boxes[i, 5])

            label = f"{labelMap[cls]}: {conf:.2f}"
            color = colors[i, 0, :].tolist()

            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)

            # Get the width and height of label for bg square
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)

            # Shows the text.
            frame = cv2.rectangle(frame, (x1, y1 - 2*h),
                                  (x1 + w, y1), color, -1)
            frame = cv2.putText(frame, label, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    return frame


class BuffNet:
    def __init__(self, config_data=None):
        """
                Define all the parameters of the model here.
                Can be initialized with a config file, a system launch
                or manually from a terminal. will exit if not enough params
                exist.
        """

        if 'DEBUG' in config_data:
            self.debug = True

        model_dir = os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'models')

        cam_source = 'rgb'
        nn_path = os.path.join(model_dir, config_data['MODEL'])
        conf_thresh = 0.5
        iou_thresh = 0.5
        nn_shape = 320

        # Start defining a pipeline

        pipeline = dai.Pipeline()

        pipeline.setOpenVINOVersion(version=dai.OpenVINO.VERSION_2021_4)

        # Define a neural network that will make predictions based on the source frames
        detection_nn = pipeline.create(dai.node.NeuralNetwork)
        detection_nn.setBlobPath(nn_path)

        detection_nn.setNumPoolFrames(4)
        detection_nn.input.setBlocking(False)
        detection_nn.setNumInferenceThreads(2)

        # Define a source - color camera
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(nn_shape, nn_shape)
        cam.setInterleaved(False)
        cam.preview.link(detection_nn.input)
        cam.setFps(30)

        # Create outputs
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("nn_input")
        xout_rgb.input.setBlocking(False)
        detection_nn.passthrough.link(xout_rgb.input)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn")
        xout_nn.input.setBlocking(False)

        detection_nn.out.link(xout_nn.input)

        if not rospy.is_shutdown():
            self.debug = rospy.get_param('/buffbot/DEBUG')
            topics = rospy.get_param('/buffbot/TOPICS')
            self.topics = [topics[t] for t in config_data['TOPICS']]
            # Only spin up image sub if core is running
            if len(self.topics) > 0:

                self.bridge = CvBridge()
                rospy.init_node('buffnet', anonymous=True)
                self.target_pub = rospy.Publisher(
                    self.topics[1], Float64MultiArray, queue_size=1)

                if self.debug and len(self.topics) > 2:
                    self.debug_pub = rospy.Publisher(
                        self.topics[2], Image, queue_size=1)

                self.im_subscriber = rospy.Subscriber(
                    self.topics[0], Image, self.imageCallBack, queue_size=1)

            with dai.Device(pipeline) as device:

                q_nn_input = device.getOutputQueue(
                    name="nn_input", maxSize=4, blocking=False)
                q_nn = device.getOutputQueue(
                    name="nn", maxSize=4, blocking=False)

                start_time = time.time()
                counter = 0
                fps = 0
                layer_info_printed = False

                while True:
                    in_nn_input = q_nn_input.get()
                    in_nn = q_nn.get()

                    frame = in_nn_input.getCvFrame()

                    layers = in_nn.getAllLayers()

                    # get the "output" layer
                    output = np.array(in_nn.getLayerFp16("output"))

                    # reshape to proper format
                    cols = output.shape[0]//6300
                    output = np.reshape(output, (6300, cols))
                    output = np.expand_dims(output, axis=0)

                    total_classes = cols - 5

                    boxes = non_max_suppression(
                        output, conf_thres=conf_thresh, iou_thres=iou_thresh)

                    boxes = np.array(boxes[0])

                    target_msg = Float64MultiArray()
                    target_msg.data = boxes[0]
                    self.target_pub.publish(target_msg)


def main(config_data):

    if config_data is None:
        return

    detector = BuffNet(config_data=config_data)

    if 'TOPICS' in config_data:
        rospy.spin()

    if 'DATA' in config_data:
        # run independantly
        data = bv.load_data(path=os.path.join(os.get_env(
            'PROJECT_ROOT'), 'data', config_data['DATA']))

        for image, labels in data[0:5]:
            detector.detect_and_annotate(image)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f'No Data: BuffNet exiting ...')
    elif '/buffbot' in sys.argv[1]:
        main(rospy.get_param(sys.argv[1]))
    elif '.yaml' in sys.argv[1]:
        with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
            data = yaml.safe_load(f)
        main(data)
