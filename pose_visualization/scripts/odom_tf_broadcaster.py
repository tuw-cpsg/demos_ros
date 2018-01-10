#!/usr/bin/env python  
import rospy

import tf
import math as m
from transforms3d.euler import quat2euler, euler2quat
from geometry_msgs.msg import PoseStamped

tf_tuples = None


def correct_pose(quat):
   eul = list(quat2euler(quat,'sxyz'))
   eul[2] += m.pi/2
   q_back = euler2quat(eul[0], eul[1], eul[2], 'sxyz')
   return q_back


def check_initial_pose(msg):

   global tf_tuples
   if not tf_tuples:
      # just get the pose once for the initial position in world frame
      tf_tuples = []
      quat = correct_pose((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
      print(quat)
      tf_tuples.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
      tf_tuples.append(quat)
      print(tf_tuples)

   # broadcast the transformation
   br = tf.TransformBroadcaster()
   br.sendTransform(tf_tuples[0],
                    tf_tuples[1],
                    rospy.Time.now(),
                    "odom",
                    "world")


if __name__ == '__main__':
   rospy.init_node('odom_tf_broadcaster')
   rospy.Subscriber('/daisy/pose',
                    PoseStamped,
                    check_initial_pose)
   rospy.spin()
