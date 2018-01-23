#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from itertools import product

from wanderer.utils.helpers import predict_scan_points, select_front_distances


def quat2euler(q):
    a, b, c, d = q[0], q[1], q[2], q[3]

    c11 = a * a + b * b - c * c - d * d
    c21 = 2 * (b * c + a * d)
    c31 = 2 * (b * d - a * c)
    c32 = 2 * (c * d + a * b)
    c33 = a * a - b * b - c * c + d * d

    p = m.asin(-c31)
    r = m.atan2(c32, c33)
    y = m.atan2(c21, c11)

    return r, p, y


class DecisionMaker():

    def __init__(self, pub):
        self.ang_vel = 0.0
        self.lin_vel = 0.1
        self.theta = 0
        self.theta_ls = None
        self.pub = pub
        self.range_w = np.arange(-0.3, 0.35, 0.05)
        self.range_v = np.array([0.1, 0.05, 0])
        self.laser_obs = None
        self.dt = 0.1

    def callback_new(self, data):

        combinations = list(product(self.range_v, self.range_w))

        utilities = None
        for c in combinations:
            u = self.utility(c[0], c[1])
            if utilities is None:
                utilities = u
            else:
                utilities = np.vstack((utilities, u))

        candidate = utilities[np.argmax(utilities[:, 2]), :]

        self.ang_vel = candidate[1]
        self.lin_vel = candidate[0]
        twist = Twist()
        twist.linear.x = self.lin_vel
        twist.angular.z = self.ang_vel
        self.pub.publish(twist)

    def utility(self, lin_vel, ang_vel):
        pos, t = self.predict_rel_pose(lin_vel, ang_vel)
        ak, dk = predict_scan_points(self.laser_obs[:, 0], self.laser_obs[:, 1], pos, t)
        df = select_front_distances(ak, dk)
        util = np.min(df)
        return np.array([[lin_vel, ang_vel, util]])

    def predict_rel_pose(self, v, w):
        tp = w * self.dt  # CHANGE self.dt
        xp = v * self.dt * m.cos(tp)
        yp = v * self.dt * m.sin(tp)
        return [xp, yp], tp

    def callback_laser(self, data):
        ranges, angles = self._convert_data(data)
        self.laser_obs = np.array([angles, ranges]).T

    def _convert_data(self, data):
        ranges = np.array(data.ranges)
        angles = np.arange(len(data.ranges))
        angles = data.angle_min + angles * data.angle_increment
        return ranges, angles


def main():
    pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)
    dec = DecisionMaker(pub)
    rospy.Subscriber('/cps_pe/kfestimate', PoseWithCovarianceStamped,
                     dec.callback_new)
    rospy.Subscriber('/scan', LaserScan, dec.callback_laser)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('wanderer_decision', anonymous=True)

    main()
