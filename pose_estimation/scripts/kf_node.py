#!/usr/bin/env python

import numpy as np
import rospy
import math as m
from numpy.linalg import solve
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray, String
from pose_estimation.msg import InputObs

# Mounting Vector
MOUNT = np.array([[-0.05, -0.07]]).T

class KalmanPoseEstimator:
    def __init__(self, pub):
        self._pub = pub
        # dimension of state
        self._dimx = 6
        # dimension of input
        self._dimu = 2
        # dimension of observations
        self._dimz = 4
        # last timestamp
        self._t = None

        # actual and last state
        self.x_k = np.zeros((self._dimx,), dtype=np.float64)
        self.x_j = np.zeros((self._dimx,), dtype=np.float64)

        # actual and last state covariance
        self.P_k = np.zeros((self._dimx, self._dimx), dtype=np.float64)
        self.P_j = np.zeros((self._dimx, self._dimx), dtype=np.float64)

    def get_data(self, data):
        # values = np.fromstring(data.data, dtype=np.float64, sep=',')
        # t = values[0]
        # u = values[1:3]
        # z = values[3:]
        t = data.header.stamp.secs + data.header.stamp.nsecs / 1e9
        u = np.array(data.control)
        z = np.array(data.obs)
        return t, u, z

    def T_x(self, x_j, dt):
        """ compute transition matrix"""
        x_j = np.squeeze(x_j)
        T_mat = np.zeros((self._dimx, self._dimx), dtype=np.float64)
        T_mat[0, :] = np.array([1, 0, np.sin(x_j[2]), 0, dt, 0])
        T_mat[1, :] = np.array([0, 1, np.cos(x_j[2]), 0, dt, 0])
        T_mat[2, :] = np.array([0, 0, 1, dt, 0, 0])
        T_mat[3, :] = np.array([0, 0, 0, 0, 0, 0])
        T_mat[4, :] = np.array([0, 0, 0, 0, 1, dt])
        T_mat[5, :] = np.array([0, 0, 0, 0, -1 / dt, 0])
        return T_mat

    def system_noise(self):
        Q = np.zeros((self._dimx, self._dimx))
        # Q = np.eye(dim_x)
        Q[3, 3] = 5e-6
        Q[5, 5] = 5e-5
        # Q[3, 3] = 1e-8
        # Q[5, 5] = 1e-8
        return Q

    def get_obs_mat(self):
        # z = [rot_gyro, a_acc, v_odo, rot_odo]
        H = np.zeros((self._dimz, self._dimx))
        H[0, 3] = 1
        H[1, 5] = 1
        H[2, 4] = 1
        H[3, 3] = 1

        R = np.eye(4)
        R[0, 0] = 0.015 ** 2
        R[1, 1] = 0.07 ** 2
        R[2, 2] = 0.001 ** 2
        R[3, 3] = 0.003 ** 2

        return H, R

    def system(self, x_j, u_k, dt):
        self.x_k = np.squeeze(np.zeros_like(x_j, dtype=np.float64))
        # x
        self.x_k[0] = x_j[0] + x_j[4] * dt * np.cos(x_j[2])
        # y
        self.x_k[1] = x_j[1] + x_j[4] * dt * np.sin(x_j[2])
        # theta
        self.x_k[2] = x_j[2] + x_j[3] * dt
        # omega
        self.x_k[3] = u_k[1]
        # v
        self.x_k[4] = x_j[4] + x_j[5] * dt
        # a
        self.x_k[5] = (u_k[0] - x_j[4]) / dt

        return self.x_k

    def predict(self, x_j, P_j, dt):
        T = self.T_x(x_j, dt)
        Q = self.system_noise()
        self.P_k = np.dot(T,np.dot(P_j, T.T)) + Q
        return self.P_k

    def update(self, x_k, P_k, z_k):
        H, R = self.get_obs_mat()
        # compute VCM of innovations
        D = R + np.dot(H,np.dot(P_k, H.T))
        # predict observations
        h = np.dot(H, x_k)
        # compute innovations
        d = z_k - h

        # compute Kalman gain
        K = solve(D.T, (np.dot(P_k, H.T)).T).T

        # update
        x_k_hat = x_k + np.dot(K, d)
        # VCM update (Joseph form)
        M = np.eye(self._dimx) - np.dot(K, H)
        m = np.dot(M, np.dot(P_k, M.T)) + np.dot(K, np.dot(R, K.T))
        P_k_hat = 0.5 * (m + m.T)

        return x_k_hat, P_k_hat

    def obs_callback(self, data):        
        t, u, z = self.get_data(data)

        if self._t is None:
            # initialize filter
            self.x_j = np.array([0., 0., 0., 0., 0., 0.])
            self.x_j = applyMounting(self.x_j, inverse=True)
            self.P_j = np.eye(self._dimx)
            self._t = t
        else:
            dt = t - self._t
            self._t = t
            # Predict state
            x_k = self.system(self.x_j, u, dt)
            # predict state covariance
            P_k = self.predict(self.x_j, self.P_j, dt)

            # update
            x_k_hat, P_k_hat = self.update(x_k, P_k, z)
            # proceed with update
            self.x_j = x_k_hat.copy()
            self.P_j = P_k_hat.copy()

            # NEW
            # publishing Pose with Covariance
            # create point message needed in pose message
            x_k_hat = applyMounting(x_k_hat)
            point_msg = Point(x_k_hat[0], x_k_hat[1], 0.0)
            # convert euler angles to quaternion
            quat = euler2quat(np.array([0, 0, x_k_hat[2]]))
            # create quaternion message needed in pose message
            quat_msg = Quaternion(*quat)
            # create pose message needed in PoseWithCovariance message
            pose_msg = Pose()
            pose_msg.position = point_msg
            pose_msg.orientation = quat_msg
            # create PoseWithCovariance message needed in
            # PoseWithCovarianceStamped message
            pwc_msg = PoseWithCovariance()
            pwc_msg.pose = pose_msg
            # set up 6x6 covariance matrix needed in PoseWithCovariance message
            # as z, rot_x and rot_y are not estimated, their variances are
            # assumed
            # var_z = 1e-06 (assumption that z is known with 1mm std
            # var_rot_x = var_rot_y = 3e-06 (assumption that these rotations are
            # known with 0.1 degree std)
            cov_mat = np.array([[P_k_hat[0, 0], P_k_hat[0, 1], 0, 0, 0, P_k_hat[0, 2]],
                                [P_k_hat[1, 0], P_k_hat[1, 1], 0, 0, 0, P_k_hat[1, 2]],
                                [0, 0, 1e-06, 0, 0, 0],
                                [0, 0, 0, 3e-06, 0, 0],
                                [0, 0, 0, 0, 3e-06, 0],
                                [P_k_hat[2, 0], P_k_hat[2, 1], 0, 0, 0, P_k_hat[2, 2]]])
            pwc_msg.covariance = cov_mat.flatten().tolist()
            # create PoseWithCovarianceStamped message
            pwcs_msg = PoseWithCovarianceStamped()
            pwcs_msg.pose = pwc_msg
            pwcs_msg.header = Header()
            pwcs_msg.header.stamp = rospy.Time.now()
            pwcs_msg.header.frame_id = 'odom'

            # publish
            self._pub.publish(pwcs_msg)


def euler2quat(euler):

    r, p, y = euler[0], euler[1], euler[2]
    a = m.cos(r / 2) * m.cos(p / 2) * m.cos(y / 2) + \
        m.sin(r / 2) * m.sin(p / 2) * m.sin(y / 2)
    b = m.sin(r / 2) * m.cos(p / 2) * m.cos(y / 2) - \
        m.cos(r / 2) * m.sin(p / 2) * m.sin(y / 2)
    c = m.cos(r / 2) * m.sin(p / 2) * m.cos(y / 2) + \
        m.sin(r / 2) * m.cos(p / 2) * m.sin(y / 2)
    d = m.cos(r / 2) * m.cos(p / 2) * m.sin(y / 2) - \
        m.sin(r / 2) * m.sin(p / 2) * m.cos(y / 2)

    return b, c, d, a

def applyMounting(x, inverse=False):

    a = x[2]
    R = np.array([[m.cos(a), -m.sin(a)],
                  [m.sin(a), m.cos(a)]])
    if inverse:
        transl = np.dot(R, -MOUNT)
    else:
        transl = np.dot(R, MOUNT)
    x[0] += transl[0, 0]
    x[1] += transl[1, 0]

    return x

def estimation():
    rospy.init_node('kf_pose_estimator', anonymous=True)

    pose_pub = rospy.Publisher('/cps_pe/kfestimate', PoseWithCovarianceStamped,
                               queue_size=10)
    kf = KalmanPoseEstimator(pose_pub)

    rospy.Subscriber('/cps_pe/kfobs', InputObs, kf.obs_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    estimation()
