# -*- coding: utf-8 -*-

import sys
import os
import argparse
import pandas as pd
import numpy as np
import math as m
from matplotlib import pyplot as plt

# Rotation of reference axis of optitrack target related to the movement
# direction of the robot
ROT_MOUNT = np.deg2rad(132.5)

def transform(Xobs, t, r):
    # transform observed x, y-coordinates into the optitrack system
    R = np.array([[m.cos(r), m.sin(r)],
                  [-m.sin(r), m.cos(r)]])

    for i in range(0, np.shape(Xobs)[1]):
        Xobs[0:2, i:i + 1] = t + np.dot(R, Xobs[0:2, i:i + 1])

    return Xobs

def quat2euler(quat):
    # calculate euler angles from quaternion
    a, b, c, d = quat[0], quat[1], quat[2], quat[3]
    c11 = a * a + b * b - c * c - d * d
    c21 = 2 * (b * c + a * d)
    c31 = 2 * (b * d - a * c)
    c32 = 2 * (c * d + a * b)
    c33 = a * a - b * b - c * c + d * d

    r = m.atan2(c32, c33)
    p = m.asin(-c31)
    y = m.atan2(c21, c11)

    return r, p, y

def process_log(odo, est, ref):
    # get coordinates from logged data
    odo_x = np.array([odo['field.pose.pose.position.x']]).T
    odo_y = np.array([odo['field.pose.pose.position.y']]).T
    est_x = np.array([est['field.pose.pose.position.x']]).T
    est_y = np.array([est['field.pose.pose.position.y']]).T
    ref_x = np.array([ref['field.pose.position.x']]).T
    ref_y = np.array([ref['field.pose.position.y']]).T

    # get orientation quaternion from optitrack data
    ori_ref = np.array([ref['field.pose.orientation.w'],
                        ref['field.pose.orientation.x'],
                        ref['field.pose.orientation.y'],
                        ref['field.pose.orientation.z']]).T
    _, _, y = quat2euler(ori_ref[0, :])

    # get time from logged data
    time = np.array(ref['field.header.stamp'])
    # get variances of x, y-coordinates of odometry and EKF estimate
    odo_varx = np.array(odo['field.pose.covariance0'])
    odo_vary = np.array(odo['field.pose.covariance7'])
    est_varx = np.array(est['field.pose.covariance0']) * 1e-4
    est_vary = np.array(est['field.pose.covariance7']) * 1e-4

    # transform coordinates
    odo = transform(np.hstack((odo_x, odo_y)).T,
                    np.array([[ref_x[0], ref_y[0]]]).T, y - ROT_MOUNT)
    est = transform(np.hstack((est_x, est_y)).T,
                    np.array([[ref_x[0], ref_y[0]]]).T, y - ROT_MOUNT)

    # calculate coordinate offsets in the last trajectory point
    dx_e = est[0, -1] - ref_x[-1]
    dy_e = est[1, -1] - ref_y[-1]
    dx_o = odo[0, -1] - ref_x[-1]
    dy_o = odo[1, -1] - ref_y[-1]
    # print results for accuracy and precision of odometry and EKF estimate
    print '--------------------------------------------------------------------'
    print 'accuracy'
    print '--------------------------------------------------------------------'
    print 'offset odometry (dx, dy)[m]: {}'.format(np.array([dx_o, dy_o]).flatten())
    print 'offset EKF estimate (dx, dy)[m]: {}'.format(np.array([dx_e, dy_e]).flatten())
    print '--------------------------------------------------------------------'
    print 'precision'
    print '--------------------------------------------------------------------'
    print 'standard deviation odometry (sx, sy)[m]: {}'.format(np.sqrt(np.array([odo_varx[-1], odo_vary[-1]])))
    print 'standard deviation EKF estimate (sx, sy)[m]: {}'.format(np.sqrt(np.array([est_varx[-1], est_vary[-1]])))
    print '--------------------------------------------------------------------'
    print 'trajectory duration [s]: {}'.format((time[-1] - time[0]) * 1e-9)

    # plot trajectories from odometry, EKF estimate and optitrack
    plt.figure(1)
    plt.plot(odo[0, :], odo[1, :], 'b.-')
    plt.plot(est[0, :], est[1, :], 'g.-')
    plt.plot(ref_x, ref_y, 'm.-')
    plt.axvline(0, color='k')
    plt.axhline(0, color='k')
    plt.axis('equal')
    plt.legend(['odometry', 'EKF estimate', 'optitrack'])

    plt.show()

def main(*argv):
    """
    Main function
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r", "--recording_id", metavar="RECORDING_IDS", type=str,
        nargs="+", help="id(s) of recordings to use", required=True)
    parsed = parser.parse_args(*(argv[1:]))

    # !!!specify folder where recordings are!!!
    log_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'logs'))
    log_path = log_path + '/' + parsed.recording_id[0]
    odo = pd.read_csv(log_path + '/odo.log', delimiter=',')
    kf = pd.read_csv(log_path + '/kf.log', delimiter=',')
    ref = pd.read_csv(log_path + '/ref.log', delimiter=',')

    process_log(odo, kf, ref)

if __name__ == '__main__':
    main(sys.argv)