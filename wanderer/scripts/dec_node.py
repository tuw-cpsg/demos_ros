#!/usr/bin/env python

import math as m
from itertools import product

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# half angle of the view area in radiant
VIEW_ANGLE = 45./180*np.pi
# threshold for maximal distance to allow linear velocity of 0.0
MAX_THRESHOLD = 3.


def polar2cart(angles, dists):
    x = dists * np.cos(angles)
    y = dists * np.sin(angles)
    return x, y


def cart2polar(x, y):
    d = np.sqrt(x**2 + y**2)
    a = np.arctan2(y, x)
    return a, d


def transform(x, y, t, rot):
    x2 = t[0] + np.cos(rot) * x + np.sin(rot) * y
    y2 = t[1] + np.sin(rot) * x + np.cos(rot) * y
    return x2, y2


def transform_polar(a, d, t, rot):
    x2 = t[0] + np.cos(a + rot) * d
    y2 = t[1] + np.sin(a + rot) * d
    return x2, y2


def predict_scan_points(a, d, t, rot):
    x, y = transform_polar(a, d, t, rot)
    a2, d2 = cart2polar(x, y)
    return a2, d2


def select_front_distances(a, d):
    idx = np.abs(a) <= VIEW_ANGLE
    return d[idx]


class DecisionMaker():

    def __init__(self, pub):
        self.ang_vel = 0.0
        self.lin_vel = 0.1
        self.theta = 0
        self.theta_ls = None
        self.max_dist = None
        self.pub = pub
        self.range_w = np.arange(-0.3, 0.35, 0.05)
        self.range_v = np.array([0.2, 0.1, 0.05])
        self.range_v2 = np.array([0.2, 0.1, 0.05, 0])
        self.laser_obs = None
        self.max_dist = None
        self.dt = 0.1

    def callback_new(self, data):

        if self.laser_obs is None:
            return

        if self.max_dist < MAX_THRESHOLD:
            vs = self.range_v2
        else:
            vs = self.range_v

        combinations = list(product(vs, self.range_w))

        utilities = None
        for c in combinations:
            u = self.utility(c[0], c[1])
            if utilities is None:
                utilities = u
            else:
               utilities = np.vstack((utilities, u))
            
        # print(utilities)
        candidate = utilities[np.argmax(utilities[:, 2]), :]
        print(candidate)
        self.max_dist = candidate[3]

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
        util = np.nanmin(df)
        max_dist = np.nanmax(df)
        return np.array([[lin_vel, ang_vel, util, max_dist]])

    def predict_rel_pose(self, v, w):
        tp = w * self.dt  # CHANGE self.dt
        xp = v * self.dt * m.cos(tp)
        yp = v * self.dt * m.sin(tp)
        return [-xp, -yp], -tp

    def callback_laser(self, data):
        ranges, angles = self._convert_data(data)
        self.laser_obs = np.array([angles, ranges]).T

    def _convert_data(self, data):
        ranges = np.array(data.ranges)
        angles = np.arange(len(data.ranges))
        angles = data.angle_min + angles * data.angle_increment
        idx = ~np.isnan(angles)
        angles = angles[idx]
        ranges = ranges[idx]
        idx2 = ~np.isnan(ranges)
        angles = angles[idx2]
        ranges = ranges[idx2]
        idx3 = ranges <= 100.
        angles = angles[idx3]
        ranges = ranges[idx3]
        return ranges, angles


def main():
    pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)
    dec = DecisionMaker(pub)

    rospy.Subscriber('/p2os/pose', Odometry,
                     dec.callback_new)
    rospy.Subscriber('/scan', LaserScan, dec.callback_laser)
    rospy.spin() 


if __name__ == '__main__':
    rospy.init_node('wanderer_decision', anonymous=True)

    main()
