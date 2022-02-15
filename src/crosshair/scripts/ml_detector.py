#! /usr/bin/env python3
"""
	Project:
			Mitch detector
	Author: Mitchell D Scott
	Description:
		Detects and displays images
"""

import os
import cv2
import yaml
import rospy
import torch
import torch.backends.cudnn as cudnn
import numpy as np
import buffvision as bv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import argparse
from PIL import Image
import sys
sys.path.append("..")

from utils.general import non_max_suppression
from utils.augmentations import letterbox
from models.common import DetectMultiBackend

class ML_Detector:
    def __init__(self,
                 weights="/home/cu-robotics/buff-code/weights/best.pt",
                 config="/home/cu-robotics/buff-code/weights/buffdata.yaml",
                 configData=None,
                 device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 orig_shape=(360, 640)
                 ):
        """
                Put params here
                The yolov5 model has an input size of (3, 640, 640)
                so any input gets cropped to that size
        """

        # init model
        self.weights = weights
        self.model = DetectMultiBackend(self.weights, None, False, config)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.model.warmup(imgsz=(1, 3, 640, 640), half=False)

        self.orig_shape = orig_shape

    def init_ros(self):

        # ROS STUFF

        self.debug = rospy.get_param('/buffbot/DEBUG')
        topics = rospy.get_param('/buffbot/TOPICS')

        self.topics = [topics[t] for t in configData['TOPICS']]

        if configData is None:
            return None

        self.steps = None
        if 'ANNOTATION_COLOR' in configData:
            self.ANNOTATION_COLOR = configData['ANNOTATION_COLOR']
        if 'ANNOTATION_THICKNESS' in configData:
            self.ANNOTATION_THICKNESS = configData['ANNOTATION_THICKNESS']

        if len(self.topics) > 0:

            self.bridge = CvBridge()

            rospy.init_node('nn_detector', anonymous=True)

            self.debug_pubs = []
            self.bound_pub = rospy.Publisher(
                self.topics[1], Float64MultiArray, queue_size=1)

            if self.debug and len(self.topics) == 6:
                for topic in self.topics[2:]:
                    self.debug_pubs.append(
                        rospy.Publisher(topic, Image, queue_size=1))

            self.im_subscriber = rospy.Subscriber(
                self.topics[0], Image, self.imageCallBack, queue_size=1)

    def drawLines(self, image, contour):
        line = cv2.fitLine(contour, cv2.DIST_L2, 0, 1, 1)
        # find two points on the line
        x = int(line[0])
        y = int(line[1])
        dx = int(line[2])
        dy = int(line[3])

        return cv2.line(image, (x, y), (x + dx, y + dy), self.ANNOTATION_COLOR, self.ANNOTATION_THICKNESS)

    def handle_imshow():
        # imshow would be nice
        pass

    def detect(self, image):
        """
                Define all image processing operations here
                @PARAMS:
                        image: cv2 image
                @RETURNS:
                        a list of bounding boxes (xyxy, conf, cls)
        """

        # can tune these later
        conf_thres = 0.25
        iou_thres = 0.45

        orig_x, orig_y = image.shape[:2]
        img, ratio, _ = letterbox(image)
        ratio_x, ratio_y = ratio

        img = np.asarray(img)
        img = np.moveaxis(img, -1, 0)
        img = torch.from_numpy(img)
        img = img.float()
        img /= 255  # normalize
        img = img[None]  # add batch axis
        out = self.model(img)
        preds = non_max_suppression(out, conf_thres, iou_thres)

        bounding_boxes = []

        for pred in preds[0]:
            x0, y0, x1, y1, conf, cl = pred
            bounding_boxes.append((x0, y0, x1, y1, conf, cl))

        return bounding_boxes, (ratio_x, ratio_y)

    def xywh2xyxy(self, x):
        # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right

        x0 = x[0] - x[2] / 2  # top left x
        y0 = x[1] - x[3] / 2  # top left y
        x1 = x[0] + x[2] / 2  # bottom right x
        y1 = x[1] + x[3] / 2  # bottom right y

        return (x0, y0, x1, y1)

    def xyxy2xywh(self, x):
        # Convert nx4 boxes from [x1, y1, x2, y2] to [x, y, w, h] where xy1=top-left, xy2=bottom-right
        y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
        y[:, 0] = (x[:, 0] + x[:, 2]) / 2  # x center
        y[:, 1] = (x[:, 1] + x[:, 3]) / 2  # y center
        y[:, 2] = x[:, 2] - x[:, 0]  # width
        y[:, 3] = x[:, 3] - x[:, 1]  # height
        return y

    def label(self, image, bounding_boxes):
        # draw labels on a cv2 img
        offset_x = int((640 - image.shape[0]) / 2)

        for bbox in bounding_boxes:
            if len(bbox) == 6:
                # yolo format
                x0, y0, x1, y1, conf, cl = bbox

                y0 -= offset_x
                y1 -= offset_x

            else:
                # label format
                _, x, y, w, h = bbox

                x0, y0, x1, y1 = self.xywh2xyxy((x, y, w, h))

                x0 *= 640
                x1 *= 640
                y0 *= 360
                y1 *= 360

            x0 = int(x0)
            x1 = int(x1)
            y0 = int(y0)
            y1 = int(y1)

            image = cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 5)
        return image

    def detect_and_compare(self, image, labels):
        """
            show detector predictions vs label
        """

        bounding_boxes, ratio = self.detect(image)

        im_pred = self.label(image, bounding_boxes)
        cv2.imwrite("pred.jpg", im_pred)
        im_label = self.label(image, labels)
        cv2.imwrite("label.jpg", im_label)

    def detect_and_publish(self, image):
        """
                Define all image processing operations here.
                Publishes the bounds to topic instead of returning.
                Maybe in debug also publishes an annotated image.
                @PARAMS:
                        image: an RGB image (should be 640x480)
                @RETURNS:
                        None
        """
        bounding_boxes, ratio = self.detect(image)
        mesg = Float64MultiArray()
        mesg.data = bounding_boxes
        self.bound_pub.publish(mesg)

        if self.debug:
            self.publish_steps()

    def imageCallBack(self, img_msg):
        """
                Callback for in_topic
                @PARAMS:
                        img_msg: the incoming message
                @RETURNS:
                        None
        """
        self.detect_and_publish(self.bridge.imgmsg_to_cv2(img_msg))


def main(configData):

    if configData is None:
        return

    ML_Detector(configData=configData)

    if 'TOPICS' in configData:
        rospy.spin()

    if 'DATA' in configData:
        data = bv.load_data(path=os.path.join(os.get_env(
            'PROJECT_ROOT'), 'data', configData['DATA']))

        for image, labels in data[0:5]:
            detector.detect_and_annotate(image)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--noros', action="store_true", help="run without ros")
    args = parser.parse_args()
    return args


"""
if __name__ == '__main__':
    if len(sys.argv) > 1:
        if '/buffbot' in sys.argv[1]:
            main(rospy.get_param(sys.argv[1]))
        elif sys.argv[1][-5:] == '.yaml':
            path = os.path.join(os.getenv('PROJECT_ROOT'),
                                'config', 'lib', sys.argv[1])
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            main(data)
    else:
        rospy.logerr(
            'Unsupported call: call this with a rosparam component name or a yaml config')
"""
if __name__ == "__main__":
    img = cv2.imread('/home/cu-robotics/buff-code/config/lib/ml_test/86.jpg')
    dets = []
    with open('/home/cu-robotics/buff-code/config/lib/ml_test/86.txt', 'r') as labelfile:
        for line in labelfile:
            c, x, y, w, h = line[:-1].split(' ')
            dets.append((float(c), float(x), float(y), float(w), float(h)))

    det = ML_Detector()

    det.detect_and_compare(img, dets)
