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
        self.bridge = CvBridge()
        rospy.init_node('tracker_data_generator', anonymous=True)
        self.dt = rospy.Rate(30)

        if(self.demo == 'random_noise'):

            self.position = np.array([150, 150, 25, 35, 0, 0], dtype=np.float64)

            # Only spin up image sub if core is running

            if len(topics) > 0:

                self.bridge = CvBridge()
                self.debug_pubs = []

            else:
                rospy.logerr("len of topics is not zero")

        # elif (self.demo == 'sine'):

        #     self.position = (0, 0)
        #     # Only spin up image sub if core is running

        #     if len(topics) > 0:

        #         # these parameters have been carefully determined by picking arbitrary numbers
        #         omega = 20  # Scales the range of radians, period = 2 pi / omega
        #         alpha = 20  # Scales the amplitude ofthe output
        #         # The timestep for the sample times
        #         sampleDt = 1 / 30
        #         sampleT = np.linspace(
        #             0, 30/self.num_demo_frames, self.num_demo_frames)  # Times of samples

        #         # Samples traces a circle defined by omega, alpha and degree_range
        #         samples = np.array(list(
        #             zip(alpha * np.cos(sampleT * omega), alpha * np.sin(sampleT * omega), sampleT)))

        #         for i in range(50):
        #             x, y, t = samples[i]

        #             self.position = (x, y)

        #             if(self.debug):
        #                 rospy.loginfo(f'sent position: {self.position}')

        #             msg = Float64MultiArray()
        #             msg.data = self.position
        #             self.bound_pub.publish(msg)
        #             # sending a new bound takes like 1ms. we're not trying to be precise here, just want about 30fps
        #             time.sleep(self.dt - 0.002)
            # else:
            #     rospy.logerr("demo is incorrect. Exiting...")

    def generate_data(self):
        # Generates a message that imitates a detection [(x,y), (w,h), cf, cl]
        # rand_1 = (np.random.random() - 0.5) * 50
        # rand_2 = (np.random.random() - 0.5) * 20
        # rand_3 = (np.random.random() - 0.5) * 5
        # self.position += np.array([rand_1, rand_2, rand_3 / 5, rand_3, 0, 0])
        # if self.position[0] > 300:
        #     self.position[0] = 300
        # elif self.position[0] < 0:
        #     self.position[1] = 0
        
        # if self.position[1] > 300:
        #     self.position[1] = 300
        # elif self.position[1] < 0:
        #     self.position[1] = 0
        
        # if self.position[2] > 70:
        #     self.position[2] = 70
        # elif self.position[2] < 5:
        #     self.position[2] = 5

        # if self.position[3] > 100:
        #     self.position[3] = 100
        # elif self.position[3] < 15:
        #     self.position[3] = 15
        t = time.time() 
        h =  50 * np.sin(t * 2) + 60
        x =  50 * np.sin(t) + 150
        rospy.loginfo(f'{h}')
        self.position = np.array([x, 150, 10, h, 1, 0])


    def spin(self):
        while not rospy.is_shutdown():
            self.generate_data()
            msg = Float64MultiArray(data=self.position)

            self.bound_pub.publish(msg)
            # sending a new bound takes like 1ms. we're not trying to be precise here, just want about 30fps
            self.dt.sleep()


def main(configData):

    if configData is None:
        return

    detector = TrackerDataGenerator(configData=configData)

    if 'TOPICS' in configData:
        detector.spin()

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
