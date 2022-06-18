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
    def __init__(self):
        """
                Define all the parameters of the model here.
                Can be initialized with a config file, a system launch
                or manually from a terminal. will exit if not enough params
                exist.
        """

        
        self.debug = rospy.get_param('/buffbot/DEBUG')
        topics = rospy.get_param('/buffbot/TOPICS')

        self.bound_pub = rospy.Publisher('image_detection', Float64MultiArray, queue_size=1)
        self.bridge = CvBridge()
        rospy.init_node('tracker_data_generator', anonymous=True)
        self.dt = rospy.Rate(30)

            # Only spin up image sub if core is running


        self.bridge = CvBridge()



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
        h =  2 * np.sin(t) + 20
        x =  320 * np.sin(t) + 150
        self.position = np.array([0, x, 160, h * 1.2, h]) / 320


    def spin(self):
        while not rospy.is_shutdown():
            self.generate_data()
            msg = Float64MultiArray(data=self.position)

            self.bound_pub.publish(msg)
            # sending a new bound takes like 1ms. we're not trying to be precise here, just want about 30fps
            self.dt.sleep()


def main():

    detector = TrackerDataGenerator()

    detector.spin()


if __name__ == '__main__':
    main()
