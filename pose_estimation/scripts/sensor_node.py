#!/usr/bin/env python

import rospy
import numpy as np
import time
from scipy.interpolate import interpolate
from copy import deepcopy
from std_msgs.msg import String, Header
from pose_estimation.msg import InputObs
from geometry_msgs.msg import Vector3Stamped, Twist
from nav_msgs.msg import Odometry

# init node
rospy.init_node('get_sensor_data', anonymous=True)
# publisher sending string-message to kfobs-topic
pub = rospy.Publisher('/cps_pe/kfobs', InputObs, queue_size=10)
t0 = time.time()
# "Buffer"-lists where last 5 sensor data is stored
accB = []
gyrB = []
odoB = []
cmdB = []

def callback(data):
    global accB
    # add acceleration data to list
    accB.append([data.header.stamp.secs + data.header.stamp.nsecs / 1e9, data.vector.y])
    # trim list
    if len(accB) > 5:
        del accB[0]
    # call talker() function as shown in the ROS tutorial
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

def callback2(data):
    global gyrB
    # add gyroscope data to list
    gyrB.append([data.header.stamp.secs + data.header.stamp.nsecs / 1e9, data.vector.z])
    # trim list
    if len(gyrB) > 5:
        del gyrB[0]

    try:
        talker()
    except rospy.ROSInterruptException:
        pass

def callback3(data):
    global odoB
    # get odometry data from subscribed message
    p = data.pose.pose.position
    q = data.pose.pose.orientation
    dp = data.twist.twist.linear
    dq = data.twist.twist.angular
    # add odometry data to list
    odoB.append([data.header.stamp.secs + data.header.stamp.nsecs / 1e9, dp.x, dq.z])
    # trim list
    if len(odoB) > 5:
        del odoB[0]

    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    
def callback_control(data):
    # get current time as timestamp for keyboard commands
    ts = time.time()
    # get odometry data from subscribed message
    du1 = data.linear
    du2 = data.angular
    # add command data to list
    cmdB.append([ts, du1.x, du2.z])
    # trim list
    if len(cmdB) > 5:
        del cmdB[0]

    try:
        talker()
    except rospy.ROSInterruptException:
        pass


def listener():
    # define subcribers getting sensor data
    rospy.Subscriber('/pi/imu3000/angular_velocity', Vector3Stamped, callback2)
    rospy.Subscriber('/pi/kxtf9/acceleration', Vector3Stamped, callback)
    rospy.Subscriber('/p2os/pose', Odometry, callback3)
    rospy.Subscriber('/teleop/cmd_vel', Twist, callback_control)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def talker():
    # talker() publishes synchronized sensor data to kfobs topic
    global t0
    # get current time
    t = time.time()
    # be sure that in every buffer-list are at least 3 data rows
    ctrl_len = (len(accB) > 3) and (len(gyrB) > 3) and (len(odoB) > 3) and (len(cmdB) > 3)
    if ((t - t0) > 0.3) and ctrl_len:
        # set t0 with current time 
        t0 = deepcopy(t)
        npt = np.array([t])
        # extrapolate sensor data to t such that it is published synchronized to timestamp t
        # using sensor data in buffer lists
        int_cmd = list(np.squeeze(interp2t(np.array(cmdB), npt)))
        int_acc = list(np.squeeze(interp2t(np.array(accB), npt)))
        int_gyr = list(np.squeeze(interp2t(np.array(gyrB), npt)))
        int_odo = list(np.squeeze(interp2t(np.array(odoB), npt)))
        # delete timestamp of synchronized acceleration, gyroscope and odometry data
        del int_acc[0]
        del int_gyr[0]
        del int_odo[0]
        del int_cmd[0]
        # publish the summed list of the synchronized data lists as String message
        data = InputObs()
        data.header = Header()
        data.header.stamp = rospy.Time.now()
        data.control = int_cmd
    	data.obs = int_gyr + int_acc + int_odo
        
    	pub.publish(data)
        # old string message
        # data = str(int_cmd + int_gyr + int_acc + int_odo)
        # pub.publish(data[1:-1])

def interp2t(val, t):
    # function to interpolate data
    # val is a matrix which first column is used as x-axis
    # each other column is treated as interpolation problem
    # t indicates at which x-values interpolated data is needed
    ret_vals = np.zeros((t.shape[0], val.shape[1]), dtype=np.float64)
    ret_vals[:,0] = np.squeeze(t)
    for i in range(1, val.shape[1]):
        f = interpolate.interp1d(val[:, 0], val[:, i], fill_value='extrapolate')
        ret_vals[:, i] = f(np.squeeze(t))
    return ret_vals

if __name__ == '__main__':
    listener()
