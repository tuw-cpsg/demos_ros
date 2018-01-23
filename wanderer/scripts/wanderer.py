#!/usr/bin/env python
#
# Let's the rover wander around without any goal.
#

from __future__ import print_function, division

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, PoseWithCovariance, PoseWithCovarianceStamped
from std_msgs.msg import Header


def main():
    rospy.init_node('wanderer', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    main()
