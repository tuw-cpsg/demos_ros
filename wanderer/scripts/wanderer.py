#!/usr/bin/env python
#
# Let's the rover wander around without any goal.
#

from __future__ import print_function, division

from itertools import product
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped
import time

# Angle left and right of 0 that is considered to be
# in the driving direction of the rover
DRIVING_ANGLE = np.pi / 4
ROVER_WIDTH = 1

# Update time of the laser scanner
PERIOD = 0.1

FALLBACK_DIST = 0.75

# Linear and angular velocities
LINVEL = [0.2, 0.3]
#ANGVEL = [-0.1 * i for i in range(-10, 11)]
ANGVEL = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]
MAX_ANGVEL = 0.3


class Wanderer(object):
    def __init__(self):
        self.cmdPub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)
        self.targetPub = rospy.Publisher('/target', PointStamped, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, callback=self.scan_cb, queue_size=10)
        self.cbCounter = 0
        self.fallbackDirection = 0


    def get_utility(self, angles, ranges, linvel, angvel):
        """
        The utility is defined as the minimal distance in
        driving direction to the next obstacle.
        We calculate the state that we would get when the rover moves
        with the given velocities for one update interval.
        """
        # Integrate the estimated position after the action
        dt = 0.01
        dx = dy = 0
        for i in range(int(PERIOD / dt)):
            theta = i * dt * angvel
            dx = dx + linvel * np.cos(theta) * dt
            dy = dy + linvel * np.sin(theta) * dt

        # Transform the point cloud
        cartX = np.cos(angles) * ranges - dx
        cartY = np.sin(angles) * ranges - dy

        newRanges = np.sqrt(cartX**2 + cartY**2)
        newAngles = np.arctan2(cartY, cartX) - theta

        # New approach
        #cartX = np.cos(angles) * ranges
        #cartY = np.sin(angles) * ranges

        #x1 = dx
        #y1 = dy
        #x2 = dx + np.cos(theta)
        #y2 = dy + np.sin(theta)

        #d = (np.abs((y2 - y1) * cartX - (x2 - x1) * cartY + x2 * y1 - y2 * x1) /
        #        np.sqrt((y2 - y1)**2 + (x2 - x1)**2))

        #indices = np.abs(d) <= ROVER_WIDTH / 2
        #drvPointsX = cartX[indices]
        #drvPointsY = cartY[indices]

        #try:
        #    m = np.min(np.sqrt((drvPointsX - dx)**2 + (drvPointsY - dy)**2))
        #except ValueError:
        #    m = 0

        #return m, (0, 0)

        indices = np.abs(newAngles) <= DRIVING_ANGLE
        drvRanges = newRanges[indices]
        drvAngles = newAngles[indices]

        # Debugging
        try:
            i = np.nanargmin(drvRanges)
            r = drvRanges[i]
            a = drvAngles[i] + theta

            x = np.cos(a) * r + dx
            y = np.sin(a) * r + dy
        except ValueError:
            x = 0
            y = 0

        try:
            m = np.nanmin(drvRanges)
        except ValueError:
            m = np.inf

        return m, (x, y)


    def scan_cb(self, msg):
        #self.cbCounter += 1
        #if self.cbCounter % 10:
        #    return

        #print(msg.angle_min, msg.angle_max, len(msg.ranges))
        #print(time.time())

        # convert to (angle, range) pairs
        inc = (msg.angle_max - msg.angle_min) / len(msg.ranges)
        ranges = np.array(msg.ranges)
        angles = np.array([msg.angle_min + inc * i for i in range(len(msg.ranges))])

        # Get ranges in driving direction
        indices = np.abs(angles) <= DRIVING_ANGLE
        drvRanges = ranges[indices]
        drvAngles = angles[indices]

        # Our actions are defined as pairs of linear and angular velocity,
        # out of an existing set of tuples. Here we calculate the maximum
        # expected utility for these actions.
        lv = None
        av = None
        umax = 0
        px = py = 0
        for linvel, angvel in product(LINVEL, ANGVEL):
            u, (x, y) = self.get_utility(angles, ranges, linvel, angvel)

            if u > umax:
                lv = linvel
                av = angvel
                umax = u
                px = x
                py = y

        # Fallback
        if umax < FALLBACK_DIST:
            lv = 0
            if np.nanmax(ranges) > FALLBACK_DIST:
                i = np.nanargmax(ranges)
                a = angles[i]
                print('Fallback to longest distance:', a)
                if self.fallbackDirection == 0:
                    if a < 0:
                        av = -0.3
                        self.fallbackDirection = -1
                    else:
                        av = 0.3
                        self.fallbackDirection = 1
                else:
                    av = self.fallbackDirection * 0.3

            else:
                print('Fallback to constant angular velocity')
                # constant angular velocity
                av = 0.3
                self.fallbackDirection = 1
        else:
            self.fallbackDirection = 0


        if av < -MAX_ANGVEL:
            av = -MAX_ANGVEL
            lv = 0

        elif av > MAX_ANGVEL:
            av = MAX_ANGVEL
            lv = 0

        cmd = Twist()
        cmd.linear.x = lv
        cmd.angular.z = av
        self.cmdPub.publish(cmd)
        print(lv, av)

        p = PointStamped()
        p.header.frame_id = 'laser'
        p.point.x = px
        p.point.y = py
        self.targetPub.publish(p)


def main():
    rospy.init_node('wanderer', anonymous=True)
    wanderer = Wanderer()
    rospy.spin()

if __name__ == '__main__':
    main()
