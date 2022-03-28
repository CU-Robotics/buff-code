#! /usr/bin/env python3

import os
import sys
import cv2
import yaml
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

import time

"""
    Tracker Demo 2D
    - which data do you want to generate
        - sine wave
        - random noise
        - cu robotics detection data
"""


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
        self.num_demo_frames = configData['NUM_DEMO_FRAMES']
        self.demo = configData['DEMO']
        topics = rospy.get_param('/buffbot/TOPICS')
        self.topics = [topics[t] for t in configData['TOPICS']]

        if not self.demo:
            rospy.logerr('Invalid demo in config file. Exiting.')
            sys.exit(1)

        self.bound_pub = rospy.Publisher(
            'detected_object', Float64MultiArray, queue_size=1)
        self.save_pub = rospy.Publisher(
            'save_data', Bool, queue_size=1
        )
        self.bridge = CvBridge()
        rospy.init_node('tracker_data_generator', anonymous=True)

        if(self.demo == 'random_noise'):

            self.position = (150, 150)

            # Only spin up image sub if core is running

            if len(topics) > 0:

                self.bridge = CvBridge()

                rospy.init_node('tracker_data_generator', anonymous=True)

                self.debug_pubs = []

                for i in range(self.num_demo_frames):
                    self.generate_data()
                    msg = Float64MultiArray()

                    # if(self.debug):
                    #     rospy.loginfo(f'sent position: {self.position}')

                    msg.data = self.position
                    self.bound_pub.publish(msg)
                    # sending a new bound takes like 1ms. we're not trying to be precise here, just want about 30fps
                    time.sleep(self.dt - 0.002)

            else:
                rospy.logerr("len of topics is not zero")

        elif (self.demo == 'sine'):

            self.position = (0, 0)
            # Only spin up image sub if core is running

            if len(topics) > 0:

                # these parameters have been carefully determined by picking arbitrary numbers
                omega = 20  # Scales the range of radians, period = 2 pi / omega
                alpha = 20  # Scales the amplitude ofthe output
                # The timestep for the sample times
                sampleDt = 1 / 30
                sampleT = np.linspace(
                    0, 30/self.num_demo_frames, self.num_demo_frames)  # Times of samples

                # Samples traces a circle defined by omega, alpha and degree_range
                samples = np.array(list(
                    zip(alpha * np.cos(sampleT * omega), alpha * np.sin(sampleT * omega), sampleT)))

                for i in range(50):
                    x, y, t = samples[i]

                    self.position = (x, y)

                    if(self.debug):
                        rospy.loginfo(f'sent position: {self.position}')

                    msg = Float64MultiArray()
                    msg.data = self.position
                    self.bound_pub.publish(msg)
                    # sending a new bound takes like 1ms. we're not trying to be precise here, just want about 30fps
                    time.sleep(self.dt - 0.002)
            else:
                rospy.logerr("demo is incorrect. Exiting...")

        msg = Bool(True)
        rospy.loginfo('Saving and exiting...')
        self.save_pub.publish(msg)

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
