#!/usr/bin/env python
#
# @brief Visualize different position estimates
# of the rover. This module subscribes to various
# position sources and publishes markers for rviz.
#

from __future__ import print_function, division

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose2D, PoseWithCovariance, PoseWithCovarianceStamped
from std_msgs.msg import Header

def main():
    rospy.init_node('coordinatetransforms', anonymous=True)

    br = tf.TransformBroadcaster()

    def optitrack_cb(pose2D):
        br.sendTransform(
                (pose2D.x + 0.052, pose2D.y + 0.078, 0),
                tf.transformations.quaternion_from_euler(0, 0, pose2D.theta + 2.22),
                rospy.Time.now(),
                'rover',
                'world'
            )


    rospy.Subscriber('/daisy/ground_pose', Pose2D, optitrack_cb)

    ls = tf.TransformListener()

    ls.waitForTransform('world', 'rover', rospy.Time(0), rospy.Duration(10.0))
    trans, rot = ls.lookupTransform('world', 'rover', rospy.Time(0))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        br.sendTransform(trans, rot, rospy.Time.now(), 'estimate', 'world')
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()
