#! /usr/bin/env python3
"""
    publish semi random 2d data at 30fps to test our new tracker implementation
"""
import os
import sys
import cv2
import yaml
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import time


class TrackerDataGenerator:
    def __init__(self, configData=None):
        """
                Define all the parameters of the model here.
                Can be initialized with a config file, a system launch
                or manually from a terminal. will exit if not enough params
                exist.
        """

        self.dt = 1/30
        self.debug = rospy.get_param('/buffbot/DEBUG')
        topics = rospy.get_param('/buffbot/TOPICS')

        self.topics = [topics[t] for t in configData['TOPICS']]
        rospy.logerr('topics: {}'.format(self.topics))

        self.position = (150, 150)
        pub_topic = 'detected_object'

        # Only spin up image sub if core is running

        if len(topics) > 0:

            self.bridge = CvBridge()

            rospy.init_node('tracker_data_generator', anonymous=True)

            self.debug_pubs = []
            self.bound_pub = rospy.Publisher(
                'detected_object', Float64MultiArray, queue_size=1)

            while True:
                self.generate_data()
                msg = Float64MultiArray()
                msg.data = self.position
                self.bound_pub.publish(msg)
                # sending a new bound takes like 1ms. we're not trying to be precise here, just want about 30fps
                time.sleep(self.dt - 0.002)

        else:
            rospy.logerr("len of topics is not zero")

    def generate_data(self):
        rand_1 = np.random.randint(-50, 50)
        rand_2 = np.random.randint(-20, 20)
        prev_1, prev_2 = self.position
        self.position = (prev_1 + rand_1, prev_2 + rand_2)


def main(configData):

    if configData is None:
        return

    detector = TrackerDataGenerator(configData=configData)

    if 'TOPICS' in configData:
        rospy.spin()

    if 'DATA' in configData:
        # run independantly
        data = bv.load_data(path=os.path.join(os.get_env(
            'PROJECT_ROOT'), 'data', configData['DATA']))

        for image, labels in data[0:5]:
            detector.detect_and_annotate(image)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        exit(0)
    if sys.argv[1][-5:] == '.yaml':
        path = os.path.join(os.getenv('PROJECT_ROOT'),
                            'config', 'lib', sys.argv[1])
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        main(data)
    elif '/buffbot' in sys.argv[1]:
        main(rospy.get_param(sys.argv[1]))
    else:
        rospy.logerr(
            'Unsupported call: use this with a rosparam component name or a yaml config')
