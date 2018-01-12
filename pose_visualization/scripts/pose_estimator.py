#!/usr/bin/env python
#
# @brief Estimate position of the rover
#

from __future__ import print_function, division

import rospy
from geometry_msgs.msg import Twist, Vector3Stamped, Pose, Point, Quaternion, PoseWithCovariance, PoseWithCovarianceStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import numpy as np


def euler_to_quaternion(pitch, roll, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)

    w = cy * cr * cp + sy * sr * sp;
    x = cy * sr * cp - sy * cr * sp;
    y = cy * cr * sp + sy * sr * cp;
    z = sy * cr * cp - cy * sr * sp;

    return Quaternion(x, y, z, w)


class KalmanFilter(object):
    """ This class contains all matrices and functions for estimating the next state. """
    def __init__(self):
        self.T = 0.1
        self.Q = np.matrix([
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.5, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.3]
        ])

        self.R = np.matrix([
            [0.002, 0.000, 0.000],
            [0.000, 0.250, 0.000],
            [0.000, 0.000, 0.00017]
        ])

        self.H = np.matrix([
            [0., 0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 0., 1.],
            [0., 0., 0., 1., 0., 0.]
        ])


    def f(self, x, u):
        """ Calculate the next state. """
        T = self.T
        r = np.matrix([
            [x[0,0] + x[4,0] * T * np.cos(x[2,0])],
            [x[1,0] + x[4,0] * T * np.sin(x[2,0])],
            [(x[2,0] + x[3,0] * T) % (2*np.pi)],
            [u[1,0]],
            [x[4,0] + x[5,0] * T],
            [(u[0,0] - x[4,0]) / T]
        ])
        return r

    def F(self, x):
        """ Calculate the Jacobian of f(x, u). """
        T = self.T
        r = np.matrix([
            [1, 0, -T * x[4,0] * np.sin(x[2,0]), 0, T * np.cos(x[2,0]), 0],
            [0, 1, T * x[4,0] * np.cos(x[2,0]), 0, T * np.sin(x[2,0]), 0],
            [0, 0, 1, T, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, T],
            [0, 0, 0, 0, -1/T, 0]
        ])
        return r

    def next_state(self, x, P, z, u):
        """ Calculate the next state and return the updated (x, P) matrices. """
        # Predict
        F = self.F(x)                                    # F(k-1)
        x = self.f(x, u)                                 # x(k, k-1)
        P = F * P * F.T + self.Q                         # P(k, k-1)

        # Update
        y = z - self.H * x                               # y(k)
        S = self.H * P * self.H.T + self.R               # S(k)
        K = P * self.H.T * S.I                           # K(k)
        x = x + K * y                                    # x(k, k)
        P = (np.matrix(np.identity(6)) - K * self.H) * P # P(k, k)

        return x, P


class PoseEstimation(object):
    def __init__(self):
        self.samples = dict()
        self.num_topics = 0

        self.x = np.matrix(np.zeros((6, 1), dtype=np.double)) # x(0, 0)
        self.P = np.matrix(np.zeros((6, 6), dtype=np.double)) # P(0, 0)
        self.u_tm1 = np.matrix([[0], [0]])

        self.filter = KalmanFilter()

        self.pose_est_seq = 0

        self.posepub = rospy.Publisher('/pose_estimation/pose', PoseWithCovarianceStamped, queue_size=10)

        self.register_subscriber('/teleop/cmd_vel', Twist)
        self.register_subscriber('/pi/imu3000/angular_velocity', Vector3Stamped)
        self.register_subscriber('/pi/kxtf9/acceleration', Vector3Stamped)
        self.register_subscriber('/p2os/pose', Odometry)


    def register_subscriber(self, topic, tp):
        self.num_topics += 1

        def callback_fn(data):
            self.samples[topic] = data

            if len(self.samples) == self.num_topics:
                self.next_state(self.samples)
                self.samples = {'/teleop/cmd_vel': self.samples['/teleop/cmd_vel']}

        rospy.Subscriber(topic, tp, callback_fn)


    def next_state(self, sample):
        # Get sensor data (fails sometimes...)
        try:
            z = np.matrix([
                sample['/p2os/pose'].twist.twist.linear.x,
                sample['/pi/kxtf9/acceleration'].vector.x,
                sample['/pi/imu3000/angular_velocity'].vector.z,
            ]).transpose()
        except Exception as e:
            print('next_state:', e)
            return

        # Get control data and store it for the next iteration.
        u = self.u_tm1
        self.u_tm1 = np.matrix([
            sample['/teleop/cmd_vel'].linear.x,
            sample['/teleop/cmd_vel'].angular.z
        ]).transpose()

        self.x, self.P = self.filter.next_state(self.x, self.P, z, u)

        point = Point(self.x[0, 0], self.x[1, 0], 0)
        orientation = euler_to_quaternion(0, 0, self.x[2, 0])

        pose = Pose(point, orientation)

        cov = [0] * 36
        cov[0] = self.P[0, 0]
        cov[1] = self.P[0, 1]
        cov[3] = self.P[0, 2]
        cov[6] = self.P[1, 0]
        cov[7] = self.P[1, 1]
        cov[9] = self.P[1, 2]
        cov[18] = self.P[2, 0]
        cov[19] = self.P[2, 1]
        cov[21] = self.P[2, 2]

        header = Header(self.pose_est_seq, rospy.Time.now(), 'estimate')
        pcv = PoseWithCovariance(pose, cov)
        pcvs = PoseWithCovarianceStamped(header, pcv)
        self.posepub.publish(pcvs)

        self.pose_est_seq += 1

def main():
    rospy.init_node('pose_estimation', anonymous=True)
    pose = PoseEstimation()

    rospy.spin()

if __name__ == '__main__':
    main()
