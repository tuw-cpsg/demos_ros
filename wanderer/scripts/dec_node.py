#!/usr/bin/env python

import numpy as np
import time
from geometry_msgs.msg import Twist

def main():
    twist = Twist()
    t0 = time.time()

    while True:
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