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
    """
    Converts polar coordinates (e.g. laser range observations) to cartesian coordinates
    :param angles: angles in [rad]
    :param dists: distances in [m]
    :return: (x, y) tuple
    """
    x = dists * np.cos(angles)
    y = dists * np.sin(angles)
    return x, y


def cart2polar(x, y):
    """
    Converts cartesian coordinates (right handed) to polar coordinates
    :param x: coordinate [m]
    :param y: coordinate [m]
    :return: (angles, distances) tuple
    """
    d = np.sqrt(x**2 + y**2)
    a = np.arctan2(y, x)
    return a, d


def transform(x, y, t, rot):
    """
    Transforms cartesian coordinates using similiarty transformation
    :param x: input x coordinates [m]
    :param y: input y coordinates [m]
    :param t: translation list of length 2 [x, y] in [m]
    :param rot: rotation angle [rad]
    :return: transformed coordinates (x, y) tuple
    """
    x2 = t[0] + np.cos(rot) * x + np.sin(rot) * y
    y2 = t[1] + np.sin(rot) * x + np.cos(rot) * y
    return x2, y2


def transform_polar(a, d, t, rot):
    """
    Transforms polar coordinates by translation and rotation parameter
    :param a: angles [rad]
    :param d: distances [m]
    :param t: translation list of length 2 [x, y] in [m]
    :param rot: rotation angle [rad]
    :return: transformed coordinates (x, y) tuple
    """
    x2 = t[0] + np.cos(a + rot) * d
    y2 = t[1] + np.sin(a + rot) * d
    return x2, y2


def predict_scan_points(a, d, t, rot):
    """
    Transforms polar coordinates by using ´transform_polar()´ and returns again polar elements
    :param a: angles [rad]
    :param d: distances [m]
    :param t: translation list of length 2 [x, y] in [m]
    :param rot: rotation angle [rad]
    :return: transformed polar coordinates, (angles, distances) tuple
    """
    x, y = transform_polar(a, d, t, rot)
    a2, d2 = cart2polar(x, y)
    return a2, d2


def select_front_distances(a, d):
    """
    Reduces a point cloud of polar coordinates by the condition of a view angle (defined by global variable VIEW_ANGLE)
    :param a: angles [rad]
    :param d: distances [m]
    :return: distances within 2d-cone of width 2xVIEW_ANGLE
    """
    idx = np.abs(a) <= VIEW_ANGLE
    return d[idx]


class DecisionMaker:
    """
    Class to do simple decision making using the laser range finder environment
    """
    def __init__(self, pub):
        # actual angular velocity command sent to the robot
        self.ang_vel = 0.0
        # actual linear velocitz command sent to the robot
        self.lin_vel = 0.1

        # latest set of point cloud measured by laser range finder
        self.laser_obs = None
        # maximal distance within ROI cone of the point cloud
        self.max_dist = None
        # publisher to publish to the /teleop topic
        self.pub = pub

        # possible commands for angular velocity
        self.range_w = np.arange(-0.3, 0.35, 0.05)
        # possible commands for linear velocity without zero (to use if there is enough space to explore)
        self.range_v = np.array([0.2, 0.1, 0.05])
        # possible commands for linear velocity with zero (to allow rotation at stand if too close to walls)
        self.range_v2 = np.array([0.2, 0.1, 0.05, 0])
        # timespan to predict to
        self.dt = 0.1

    def callback_new(self, data):
        """
        Takes the latest point cloud, does the simple utility based decision making and publishes the result to the
        /teleop to wander around
        :param data: data of the odometry pose (unused)
        :return:
        """
        if self.laser_obs is None:
            # skip if no laser range data available
            return

        if self.max_dist < MAX_THRESHOLD:
            # if there is no region left to explore (maximal distance of laser range finder is smaller than a threshold)
            # use extended set of linear velocity (including 0.0)
            vs = self.range_v2
        else:
            # otherwise force the robot to keep moving
            vs = self.range_v

        # create all possible combinations of linear and angular velocity
        combinations = list(product(vs, self.range_w))

        utilities = None
        for c in combinations:
            # iterate over all combinations and compute the utility
            u = self.utility(c[0], c[1])
            if utilities is None:
                utilities = u
            else:
                utilities = np.vstack((utilities, u))
            
        # select the command set with the maximal expected utility (MEU)
        candidate = utilities[np.argmax(utilities[:, 2]), :]
        print(candidate)
        # store the maximal distance for next epoch
        self.max_dist = candidate[3]

        # store the commands and publish to the topic
        self.ang_vel = candidate[1]
        self.lin_vel = candidate[0]
        twist = Twist()
        twist.linear.x = self.lin_vel
        twist.angular.z = self.ang_vel
        self.pub.publish(twist)

    def utility(self, lin_vel, ang_vel):
        """
        Takes a pair of commands and computes the expected utility
        :param lin_vel: linear velocity
        :param ang_vel: angular velocity
        :return: row vector containing the commands + utility + maximal distance
        """
        # predict the relative pose change introduced by the specified commands
        pos, t = self.predict_rel_pose(lin_vel, ang_vel)
        # predict the point cloud under the assumption that the environment does not change
        ak, dk = predict_scan_points(self.laser_obs[:, 0], self.laser_obs[:, 1], pos, t)
        # subselect the points within the front area
        df = select_front_distances(ak, dk)
        # utility is defined by the minimal distance (nearest obstacle)
        # the further away the nearest obstacle the better
        util = np.nanmin(df)
        # store furthers obstacle
        max_dist = np.nanmax(df)
        return np.array([[lin_vel, ang_vel, util, max_dist]])

    def predict_rel_pose(self, v, w):
        """
        Predict the movement of the robot (corresponding to the kinematic model of the KF estimator)
        :param v: linear velocity [m/s]
        :param w: angular velocity [rad/s]
        :return: transformation parameter
        """
        tp = w * self.dt
        xp = v * self.dt * m.cos(tp)
        yp = v * self.dt * m.sin(tp)
        return [-xp, -yp], -tp

    def callback_laser(self, data):
        """
        Receives the laser range data and converts it to polar coordinates and stores it in a property
        numpy.array with n rows and 2 columns: first containing angles, second containing distances
        :param data: LaserScan from sensor_msg.msgs
        :return: None
        """
        ranges, angles = self._convert_data(data)
        self.laser_obs = np.array([angles, ranges]).T

    def _convert_data(self, data):
        """
        Converts the LaserScan message to polar coordinates
        Removes nans and infs in observations
        :param data: LaserScan from sensor_msg.msgs
        :return: polar tuple (angles, distances)
        """
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
