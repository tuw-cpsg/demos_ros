#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

MAX_DIST_THRESHOLD = 1.
DT = 0.1
SPECIAL_MOVEMENT = False


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
        self.max_dist = None
        self.pub = pub
        
        self.MAX_DIST_THRESHOLD = 1.
        self.DT = 0.3
        self.SPECIAL_MOVEMENT = False

    def callbackKF(self, data):
        quat = np.array([data.pose.pose.orientation.w,
                         data.pose.pose.orientation.x,
                         data.pose.pose.orientation.y,
                         data.pose.pose.orientation.z])
        _, _, self.theta = quat2euler(quat)

        if self.theta_ls is None or self.max_dist is None or self.SPECIAL_MOVEMENT:
            return

        if self.max_dist < self.MAX_DIST_THRESHOLD:
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0.3
            self.pub.publish(twist)
            self.SPECIAL_MOVEMENT = True
            time.sleep(np.pi/0.3)
            self.SPECIAL_MOVEMENT = False

        range_w = np.arange(-0.3, 0.35, 0.05)
        utility_dtheta = np.array([[0, 0]])
        for w in range_w:
            # theta_pi = self.theta + 0.3 * (self.ang_vel + w)
            theta_pi = self.theta + self.DT * w
            dtheta = theta_pi - self.theta_ls
            # print('Theta, dtheta: {} | {}'.format(self.theta, dtheta))
            pi2 = m.pi
            utility = (pi2 - m.fabs(dtheta)) / pi2
            utility_dtheta = np.vstack((utility_dtheta,
                                        np.array([[utility, w]])))
        #print(utility_dtheta)
        # self.ang_vel = range_w[np.argmax(utility)]
        # self.ang_vel = utility_dtheta[np.argmax(utility_dtheta[:, 0]), 1]
        # print('{}|{}'.format(self.theta_ls-self.theta, self.ang_vel))
        # twist = Twist()
        # twist.linear.x = self.lin_vel
        # twist.angular.z = self.ang_vel
        # self.pub.publish(twist)

    def callbackLaser(self, data):
        ranges, angles = self._convert_data(data)
        in_front = np.argmin(np.abs(angles))-5, np.argmin(np.abs(angles)) + 5
        max_pos = np.argmax(ranges)
        delta_theta = angles[max_pos]
        print('DTheta: {}'.format(delta_theta/np.pi*180))
        self.theta_ls = self.theta + delta_theta
        self.max_dist = np.nanmax(ranges) 

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

    # rospy.Subscriber('/cps_pe/kfestimate', PoseWithCovarianceStamped,
    #                  dec.callbackKF)
    rospy.Subscriber('/p2os/pose', Odometry,
                     dec.callbackKF)
    rospy.Subscriber('/scan', LaserScan, dec.callbackLaser)

    rospy.spin() 

        
        

if __name__ == '__main__':
    rospy.init_node('wanderer_decision', anonymous=True)
    
    main()
