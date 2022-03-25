#! /usr/bin/env python3
import sys
import time
import rospy
import numpy as np
import buffvision as bv
from std_msgs.msg import Float64MultiArray

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
        self.DEBUG_SAVE_PATH = None

        self.dt = 1/30
        self.t = 0
        self.state = []
        self.error = 0
        self.pred_prev = np.array((0, 0))

        self.debug = True

        # previous groundtruth values. (3, 2 timesteps ago, 1 ago, now) We use these to update our deltas
        self.prev_1 = None
        self.prev_2 = None
        self.prev_3 = None
        self.prev_4 = None

        # our predictions for 1 step ahead, 2, 3
        self.pred_1 = (0, 0)
        self.pred_2 = (0, 0)
        self.pred_3 = (0, 0)

        # our deltas that we use to update predictions
        self.d_1 = None
        self.d_2 = None
        self.d_3 = None

        self.debug = rospy.get_param('/buffbot/DEBUG')
        topics = rospy.get_param('/buffbot/TOPICS')

        rospy.logerr(topics)

        self.topics = [topics[t] for t in configData['TOPICS']]

        rospy.init_node('target_tracker', anonymous=True)

        self.detect_sub = rospy.Subscriber(
            'detected_object', Float64MultiArray, self.callback, queue_size=5)

        self.prediction_pub = rospy.Publisher(
            'predictions', Float64MultiArray, queue_size=1)

    def callback(self, msg):
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

        # send out predictions
        predictions_arr = Float64MultiArray()
        predictions = np.array((*self.pred_1, *self.pred_2, *self.pred_3))
        predictions_arr.data = predictions
        if(self.debug):
            rospy.loginfo(
                'current pos: {self.prev_1} pred_1: {self.pred_1} pred_2: {self.pred_2} pred_3: {self.pred_3} error: {self.error} d_1: {self.d_1}'.format(self=self))
        self.prediction_pub.publish(predictions_arr)

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
            if self.pred_2 is None:
                self.pred_pred = np.array((0, 0))
            else:
                self.pred_prev = self.pred_2

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

    def plot(self, samples):
        sampleX, sampleY, vXActual, vYActual, aXActual, aYActual, sample_t = list(
            zip(*samples))
        pose, vel, accel, jerk, error, t = list(zip(*self.deltas))
        poseX, poseY = list(zip(*pose))

        vX, vY = list(zip(*vel))
        aX, aY = list(zip(*accel))
        jX, jY = list(zip(*jerk))
        eX, eY = list(zip(*error))
        fig, ax = plt.subplots(2, 3, figsize=(20, 20))
        ax[0][0].legend(loc='upper right')
        ax[0][0].set_title('Sampled vs Predicted x / y')
        ax[0][1].plot(t, aX, label='Predicted X Accel', marker='x')
        ax[0][1].plot(t, aY, label='Predicted Y Accel', marker='x')
        ax[0][1].plot(sample_t, aXActual, label='Sample X Accel', marker='o')
        ax[0][1].plot(sample_t, aYActual, label='Sample Y Accel', marker='o')
        ax[0][1].legend(loc='upper right')
        ax[0][1].set_title('Sampled vs Predicted Acceleration / dt')
        ax[0][2].plot(t, vX, label='Predicted X Velocity', marker='x')
        ax[0][2].plot(t, vY, label='Predicted Y Velocity', marker='x')
        ax[0][2].plot(sample_t, vXActual,
                      label='Sample X Velocity', marker='o')
        ax[0][2].plot(sample_t, vYActual,
                      label='Sample Y Velocity', marker='o')
        ax[0][2].legend(loc='upper right')
        ax[0][2].set_title('Sampled vs Predicted Velocity / dt')
        ax[1][0].plot(t, eX, label='X Error', marker='x')
        ax[1][0].plot(t, eY, label='Y Error', marker='x')
        ax[1][0].legend(loc='upper right')
        ax[1][0].set_title('Error over Time')
        ax[1][1].set_title('Sampled vs Predicted x / t')
        ax[1][1].plot(sample_t, sampleX, label='Sample X Pose', marker='o')
        ax[1][1].plot(t, poseX, label='Predicted X Pose', marker='o')
        ax[1][2].set_title('Sampled vs Predicted y / t')
        ax[1][2].plot(sample_t, sampleY, label='Sample Y Pose', marker='o')
        ax[1][2].plot(t, poseY, label='Predicted Y Pose', marker='o')
        plt.savefig(os.path.join(self.DEBUG_SAVE_PATH,
                    f'plot_{self.plot_count}.png'))
        plt.show()


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
