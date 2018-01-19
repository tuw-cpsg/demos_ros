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

    def callbackKF(self, data):
        quat = np.array([data.pose.pose.orientation.w,
                         data.pose.pose.orientation.x,
                         data.pose.pose.orientation.y,
                         data.pose.pose.orientation.z])
        _, _, yaw = quat2euler(quat)

    def callbackLaser(self, data):
        ranges = np.array(data.ranges) 
        max_pos = np.argmax(ranges)
        self.delta_theta = data.angle_min + max_pos * data.angle_increment
        print(self.delta_theta)

def main():
    twist = Twist()
    dec = DecisionMaker()
    t0 = time.time()

    while True:

        rospy.Subscriber('/cps_pe/kfestimate', PoseWithCovarianceStamped,
                         dec.callbackKF)
        rospy.Subscriber('/scan', LaserScan, dec.callbackLaser)

        if (time.time() - t0) < 7.0:
            twist.linear.x = 0.2
            twist.angular.z = np.deg2rad(10)
        else:
            twist.liner.x = 0.0
            twist.angular.z = 0.0
        pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('wanderer_decision', anonymous=True)
    pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)
    main()
