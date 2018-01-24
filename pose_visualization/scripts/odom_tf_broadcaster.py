#!/usr/bin/env python  
import rospy

import tf
import math as m
from transforms3d.euler import quat2euler, euler2quat
from geometry_msgs.msg import PoseStamped

tf_tuples = None
# ANGLE_CORR = m.pi/2
ANGLE_CORR = 127.7/180.*m.pi
MOUNT = [-0.05, -0.07]


def correct_pose(quat):   
   eul = list(quat2euler(quat,'sxyz'))
   eul[2] = eul[2] + ANGLE_CORR
   q_back = euler2quat(eul[0], eul[1], eul[2], 'sxyz')
   return (q_back[1], q_back[2], q_back[3], q_back[0])


def check_initial_pose(msg):

   global tf_tuples
   if not tf_tuples:
      # just get the pose once for the initial position in world frame
      tf_tuples = []
      quat = correct_pose((msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z))
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

#   br2 = tf.TransformBroadcaster()
#   br2.sendTransform((tf_tuples[0][0]-MOUNT[0], tf_tuples[0][1]-MOUNT[1], tf_tuples[0][2]),
#                    tf_tuples[1],
#                    rospy.Time.now(),
#                    "filter",
#                    "world")


if __name__ == '__main__':
   rospy.init_node('odom_tf_broadcaster')
   rospy.Subscriber('/daisy/pose',
                    PoseStamped,
                    check_initial_pose)
   rospy.spin()
