#!/usr/bin/env python
#
# Let's the rover wander around without any goal.
#

from __future__ import print_function, division

import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('wanderer', anonymous=True)

    def scan_cb(arg):
        print(arg)

    rospy.Subscriber('/scan', LaserScan, callback=scan_cb, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()
