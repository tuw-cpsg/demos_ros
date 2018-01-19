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

    def __init__(self, pub):
        self.ang_vel = 0.0
        self.theta = 0
        self.theta_ls = None
        self.pub = pub

    def callbackKF(self, data):
        quat = np.array([data.pose.pose.orientation.w,
                         data.pose.pose.orientation.x,
                         data.pose.pose.orientation.y,
                         data.pose.pose.orientation.z])
        _, _, self.theta = quat2euler(quat)
        # self.delta_theta

        

    def callbackLaser(self, data):
        ranges, angles = self._convert_data(data)
        in_front = np.argmin(np.abs(angles))-5, np.argmin(np.abs(angles)) + 5
        # print(ranges[in_front[0]:in_front[1]])
        max_pos = np.argmax(ranges)
        delta_theta = angles[max_pos]
        print(delta_theta)
        self.theta_ls = self.theta + delta_theta 
        

    def get_ang_vel(self):
        return self.ang_vel

    def _convert_data(self, data):
        ranges = np.array(data.ranges) 
        angles = np.arange(len(data.ranges))
        angles = data.angle_min + angles * data.angle_increment
        return ranges, angles


def main():

    pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)
    dec = DecisionMaker(pub)
    twist = Twist()
    #lin_vel = 0.1
    lin_vel = 0

    rospy.Subscriber('/cps_pe/kfestimate', PoseWithCovarianceStamped,
                     dec.callbackKF)
    rospy.Subscriber('/scan', LaserScan, dec.callbackLaser)

    rospy.spin() 

        
        

if __name__ == '__main__':
    rospy.init_node('wanderer_decision', anonymous=True)
    
    main()
