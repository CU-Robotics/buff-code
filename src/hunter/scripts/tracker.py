#! /usr/bin/env python3
import sys
import time
import rospy
import numpy as np
import buffvision as bv
from std_msgs.msg import Float64MultiArray


class DetectionTracer:
    def __init__(self, up_sample=5, t1=0.0):

        self.t1 = 0.0
        self.dt = None
        self.up_sample = up_sample

        # Initialize the tracking parameters
        self.pose = np.zeros(2, dtype=np.float64)
        self.velocity = np.zeros(2, dtype=np.float64)
        self.acceleration = np.zeros(2, dtype=np.float64)
        self.jerk = np.zeros(2, dtype=np.float64)
        self.error = np.zeros(2, dtype=np.float64)
        self.deltas = []  # [[self.pose, self.velocity, self.acceleration, self.jerk, self.error, self.t1]] # State history for ploting
        self.p1 = None  # Last two poses, p1 = pose(t-1), p2 = pose(t-2)
        self.p2 = None
        self.p3 = None

        self.pjerk = None
        self.pacceleration = None
        self.pvelocity = None

        self.debug = rospy.get_param('/buffbot/DEBUG')
        topics = rospy.get_param('/buffbot/TOPICS')

        self.topics = [topics[t] for t in configData['TOPICS']]

        if self.debug:
            self.err_pub = rospy.Publisher(
                self.topics[2], Float64MultiArray, queue_size=1)

        rospy.init_node('target_tracker', anonymous=True)

        self.detect_sub = rospy.Subscriber(
            self.topics[0], Float64MultiArray, self.callback, queue_size=5)

    def process_data(self, data):
        # (w, y, w, h)
        pass

    def on_detection(self, detection):
        # for every detection, we will output a single prediction, for now
        # later, implement upsample
        # for now this is only 2d
        pos = detection.data
        # debug for now
        rospy.loginfo(f'Detection: {pos}')

    def update_pos(self):

        if self.pose is None:
            rospy.logerr("Error: Pose is None")
            return

        arr = Float64MultiArray
        arr.data = self.pose
        self.target_pub.pusblish(arr)

    def info(self):
        print(f'Pose\t\t\tVelocity\t\tAcceleration\t\tJerk\t\t\tError\t\t\tTime\t\t\Deltas')

    for sample in self.deltas:
        print(f'{np.round(sample[0],4)} \t\t{np.round(sample[1],4)} \t\t{np.round(sample[2],4)} \t\t{np.round(sample[3],4)} \t\t{np.round(sample[4],4)} \t\t{np.round(sample[5],4)}')

        pose, vel, accel, jerk, error, t = list(zip(*self.deltas))
        print('pose: {}'.format(len(pose)))

    # no point doing this in buff-code

    """
    def display(self, samples):
        sampleX, sampleY, vXActual, vYActual, aXActual, aYActual, sample_t = list(
            zip(*samples))
        pose, vel, accel, jerk, error, t = list(zip(*self.deltas))
        poseX, poseY = list(zip(*pose))

        vX, vY = list(zip(*vel))
        aX, aY = list(zip(*accel))
        jX, jY = list(zip(*jerk))
        eX, eY = list(zip(*error))
        fig, ax = plt.subplots(2,3, figsize=(20,20))
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
        plt.show()
    """

    def set_state(self, pose=None, velocity=None, acceleration=None, jerk=None, error=None, t=None):
        if not pose is None:
            self.pose = np.array(pose, dtype=np.float64)

        if not velocity is None:
            self.velocity = np.array(velocity, dtype=np.float64)

        if not acceleration is None:
            self.acceleration = np.array(acceleration, dtype=np.float64)

        if not jerk is None:
            self.jerk = np.array(jerk, dtype=np.float64)

        if not error is None:
            self.error = np.array(error, dtype=np.float64)

        if not t is None:
            self.t1 = t

    def save_deltas(self):
        self.deltas.append([self.pose, self.velocity / self.dt, (self.acceleration) / (
            self.dt ** 2), (self.jerk) / (self.dt ** 3), self.error, self.t1])

    def update_state(self, pose_t, dt):

        # Get deltas and pose at t
        if self.p1 is not None:
            if self.p2 is not None:
                if self.p3 is not None:
                    self.jerk = (((pose_t - self.p1) - (self.p2 - self.p1)) -
                                 ((self.p2 - self.p1) - (self.p3 - self.p2))) / (dt**3)

                self.acceleration = ((pose_t - self.p1) -
                                     (self.p2 - self.p1)) / (dt**2)
                self.p3 = self.p2

            self.velocity = (pose_t - self.p1) / dt

        self.p2 = self.p1

        self.p1 = pose_t
        self.pose = pose_t

    def trace(self, samples, sampleDt):

        for x, y, dx, dy, ddx, ddy, t in samples:  # For each measurement generate up_sample prediction steps
            pose_t = np.array([x, y])
            # Get the mse error on every detection
            self.error = mse(pose_t, self.pose)
            self.dt = sampleDt / self.up_sample
            # self.update_state(pose_t, sampleDt)
            # Run up-sampling loop
            for i in range(self.up_sample):
                self.t1 += self.dt
                self.pose = self.pose + (self.velocity * self.dt) / self.up_sample + (self.acceleration * (
                    self.dt**2)) / self.up_sample + (self.jerk * self.dt**3) / self.up_sample
                # self.update_state(self.pose, sampleDt)
                self.save_deltas()

            self.update_state(pose_t, sampleDt)
