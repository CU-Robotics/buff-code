#! /usr/bin/env python3
"""
	Project:
			Mitch detector
	Author: Mitchell D Scott
	Description:
		Detects and displays images
"""
import os
import sys
import cv2
import yaml
import rospy
import numpy as np
import buffvision as bv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

import sys
sys.path.append("..")

"""

list of packages that the detector needs

pandas
torch
torchvision
pillow
tqdm (maybe)

"""

from lib.models.common import DetectMultiBackend

class ML_Detector:
    def __init__(self,
                 weights="./lib/best.pt",
                 config="./lib/buffdata.yaml",
                 configData=None,
                 device=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 ):
        """
                Put params here
                The yolov5 model has an input size of (3, 640, 640)
                so any input gets cropped to that size
        """

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

        # init model
        self.weights = weights
        self.model = DetectMultiBackend(self.weights, None, False, config)
        self.stride, self.names, _, _, _, _ = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
        self.model.warmup(imgsz=(1, 3, 640, 640), half=False)

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

    def drawEllipse(self, image, contour):
        ellipse = cv2.fitEllipse(contour)
        return cv2.ellipse(image, ellipse, self.ANNOTATION_COLOR, self.ANNOTATION_THICKNESS)

    def detect(self, image):
        """
                Define all image processing operations here
                @PARAMS:
                        image: an RGB image
                @RETURNS:
                        a list of bounding boxes in yolo format (x, y, w, h) 
        """

        image = preprocess_image(image)

        return bounds

    def preprocess_image(self, image):
        pass

    def detect_and_annotate(self, image):
        """
                Displays the steps in the processing line
                @PARAMS:
                        image: an RGB image
                @RETURNS:
                        None
        """
        results = self.detect(image)
        mostly_raw = np.concatenate(steps[:2], axis=1)
        not_so_raw = np.concatenate(steps[2:], axis=1)
        collage = np.concatenate([mostly_raw, not_so_raw])
        bv.buffshow('Steps', collage)

    def publish_steps(self):
        """
                Images processing steps are saved, this will publish them
        """
        for i, pub in enumerate(self.debug_pubs):
            pub.publish(self.bridge.cv2_to_imgmsg(
                self.steps[i], encoding='bgr8'))

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
        results = self.detect(image)
        mesg = Float64MultiArray()
        mesg.data = results

        if not results is None:
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

    MDS_Detector(configData=configData)

    if 'TOPICS' in configData:
        rospy.spin()

    if 'DATA' in configData:
        # run independantly
        data = bv.load_data(path=os.path.join(os.get_env(
            'PROJECT_ROOT'), 'data', configData['DATA']))

        for image, labels in data[0:5]:
            detector.detect_and_annotate(image)


if __name__ == '__main__':
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
