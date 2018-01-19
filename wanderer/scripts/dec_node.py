#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

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

    def __init__(self):
        self.ang_vel = 0.0
        self.delta_theta = None

    def callbackKF(self, data):
        quat = np.array([data.pose.pose.orientation.w,
                         data.pose.pose.orientation.x,
                         data.pose.pose.orientation.y,
                         data.pose.pose.orientation.z])
        _, _, self.theta = quat2euler(quat)

        range_w = np.arange(-0.3, 0.35, 0.05)
        utility_dtheta = np.array([[0, 0]])
        for w in range_w:
            theta_pi = self.theta + 0.3 * (self.ang_vel + w)
            dtheta = theta_pi - self.theta_ls
            pi2 = m.pi / 2
            utility = (pi2 - m.fabs(dtheta)) / pi2
            utility_dtheta = np.vstack((utility_dtheta,
                                        np.array([[utility, dtheta]])))

        self.ang_vel = utility_dtheta[np.argmax[utility_dtheta[:, 0]], 1] / 0.3

    def callbackLaser(self, data):
        ranges = np.array(data.ranges) 
        max_pos = np.argmax(ranges)
        self.delta_theta = data.angle_min + max_pos * data.angle_increment
        print(self.delta_theta)

    def get_ang_vel(self):
        return self.ang_vel

def main():
    dec = DecisionMaker()
    twist = Twist()
    lin_vel = 0.1

    while True:
        rospy.Subscriber('/cps_pe/kfestimate', PoseWithCovarianceStamped,
                         dec.callbackKF)
        rospy.Subscriber('/scan', LaserScan, dec.callbackLaser)

        twist.linear.x = lin_vel
        twist.angular.z = dec.get_ang_vel()
        pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('wanderer_decision', anonymous=True)
    pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)
    main()
