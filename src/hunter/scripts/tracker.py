#! /usr/bin/env python3
import sys
import time
import rospy
import numpy as np
import buffvision as bv
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import os

"""
    Tracker class
    args:
    - configData: dictionary containing the configuration data
        - data: do you want to save telemetry to a file under data
    - Indicate that you want to save the predict
"""


class Tracker:
    def __init__(self, configData):

        self.plot_count = 0
        self.DEBUG_SAVE_PATH = os.path.join(
            os.getenv('PROJECT_ROOT'),  'data', 'tracker_demo')
        # in case these don't exist, this makes them
        try:
            os.makedirs(self.DEBUG_SAVE_PATH)
        except FileExistsError as e:
            pass

        self.dt = 1/30
        self.t = 0
        self.state = []
        self.error = 0
        self.pred_prev = np.array((0, 0))

        self.num_targets = 0

        self.debug = rospy.get_param('/buffbot/DEBUG')

        # previous groundtruth values. (3, 2 timesteps ago, 1 ago, now) We use these to update our deltas
        self.prev_1 = np.array((0, 0))
        self.prev_2 = np.array((0, 0))
        self.prev_3 = np.array((0, 0))
        self.prev_4 = np.array((0, 0))

        # our predictions for 1 step ahead, 2, 3
        self.pred_1 = np.array((0, 0))
        self.pred_2 = np.array((0, 0))
        self.pred_3 = np.array((0, 0))

        # our deltas that we use to update predictions
        self.d_1 = np.array((0, 0))
        self.d_2 = np.array((0, 0))
        self.d_3 = np.array((0, 0))

        self.debug = rospy.get_param('/buffbot/DEBUG')
        topics = rospy.get_param('/buffbot/TOPICS')

        self.topics = [topics[t] for t in configData['TOPICS']]

        rospy.init_node('target_tracker', anonymous=True)

        self.detect_sub = rospy.Subscriber(
            'detected_object', Float64MultiArray, self.detected_callback, queue_size=1)

        self.save_sub = rospy.Subscriber(
            'save_data', Bool, self.save_callback, queue_size=1
        )

        self.prediction_pub = rospy.Publisher(
            'predictions', Float64MultiArray, queue_size=1)

    def detected_callback(self, msg):
        x, y = msg.data
        # for now. need to figure out how to get accurate time between messages?
        t = self.t

        # do tracker stuff
        pos = np.array((x, y))
        self.update_prev(pos, t)
        self.update_deltas()
        self.update_predictions()
        self.update_error()
        self.save_state()

        self.t += self.dt

        # send out predictions
        predictions_arr = Float64MultiArray()
        predictions = np.array((*self.pred_1, *self.pred_2, *self.pred_3))
        predictions_arr.data = predictions
        if(self.debug):
            rospy.loginfo(
                'current pos: {self.prev_1} pred_1: {self.pred_1} pred_2: {self.pred_2} pred_3: {self.pred_3} error: {self.error} d_1: {self.d_1} num_targets {self.num_targets}'.format(self=self))
        self.prediction_pub.publish(predictions_arr)

    # save data
    def save_callback(self, msg):
        self.plt_save()

    def mse(self, x, y):
        return ((x - y) ** 2).sum()

    def update_error(self):
        if self.prev_1 is not None and self.pred_prev is not None:
            self.error = self.mse(self.prev_1, self.pred_prev)

    def update_prev(self, current_pos, current_t):
        self.prev_4 = self.prev_3
        self.prev_3 = self.prev_2
        self.prev_2 = self.prev_1
        self.prev_1 = current_pos
        self.t = current_t

    def update_deltas(self):
        # Get deltas and pose at t
        if self.prev_2 is not None:
            self.d_1 = (self.prev_1 - self.prev_2) / self.dt

            if self.prev_3 is not None:
                vel_1 = self.d_1
                vel_2 = (self.prev_2 - self.prev_3)

                self.d_2 = (vel_1 - vel_2)

                if self.prev_4 is not None:
                    acc_1 = self.d_2
                    vel_3 = (self.prev_3 - self.prev_4)
                    acc_2 = (vel_2 - vel_3)
                    acc_2 = (acc_1 - acc_2)

                    self.d_3 = (acc_1 - acc_2)

    def save_state(self):
        self.state.append((self.pred_prev, self.prev_1, self.d_1,
                          self.d_2, self.d_3, self.error, self.t))

    def update_predictions(self):

        if self.pred_1 is not None:
            if self.pred_2 is None or self.pred_3 is None:
                self.pred_prev = np.array((0, 0))
            else:
                self.pred_prev = self.pred_1

        if(self.d_1 is not None and self.d_2 is not None and self.d_3 is not None):
            self.pred_1 = self.prev_1 + \
                (self.d_1 * self.dt) + (self.d_2 * (self.dt ** 2) / 2) + \
                (self.d_3 * (self.dt ** 3)) / 6
            self.pred_2 = self.pred_1 + \
                (self.d_1 * self.dt) + (self.d_2 * (self.dt ** 2) / 2) + \
                (self.d_3 * (self.dt ** 3)) / 6
            self.pred_3 = self.pred_2 + \
                (self.d_1 * self.dt) + (self.d_2 * (self.dt ** 2) / 2) + \
                (self.d_3 * (self.dt ** 3)) / 6

        else:
            self.pred_1 = self.prev_1

    def plt_save(self):
        pred, pose, velocity, acceleration, jerk, error, time = list(
            zip(*self.state))
        poseX, poseY = list(zip(*pose))
        predX, predY = list(zip(*pred))

        fig, ax = plt.subplots(2, figsize=(40, 40))

        ax[0].set_title('Sampled vs Predicted x / t')
        ax[0].plot(time, poseX, label='Sample X Pose', marker='x')
        ax[0].plot(time, predX, label='Predicted X Pose', marker='o')
        ax[1].set_title('Sampled vs Predicted y / t')
        ax[1].plot(time, poseY, label='Sample Y Pose', marker='x')
        ax[1].plot(time, predY, label='Predicted Y Pose', marker='o')

        plt.savefig(os.path.join(self.DEBUG_SAVE_PATH,
                    f'plot_{self.plot_count}.png'))
        plt.close()

        if self.debug:
            rospy.loginfo(
                f'Successfully saved at /data/tracker_demo/plot_{self.plot_count}.png. Exiting...')

        sys.exit(1)


def main(configData):

    if configData is None:
        return

    detector = Tracker(configData=configData)

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
        print(f'No Data: BuffNet exiting ...')
    elif '/buffbot' in sys.argv[1]:
        main(rospy.get_param(sys.argv[1]))
    elif '.yaml' in sys.argv[1]:
        with open(os.path.join(os.getenv('PROJECT_ROOT'), 'buffpy', 'config', 'data', sys.argv[1]), 'r') as f:
            data = yaml.safe_load(f)
        main(data)