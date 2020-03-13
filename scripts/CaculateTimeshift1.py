#!/usr/bin/env python2
import os
import argparse
import pandas as pd
import numpy as np
import os
import trajectory_utils as tu
import matplotlib.pyplot as plt
import transformations as tf
import CaculateTimeshift1 as calTime

def computetheta_fortraj(traj):
    rel_rot = []
    traj_size = traj.shape;
    print("===================traj.shape: ", traj_size[0])
    T_c0 = tu.get_rigid_body_trafo(traj[0, 4:], traj[0, 1:4])
    for idx  in range(traj_size[0]):
        T_c1 = tu.get_rigid_body_trafo(traj[idx, 4:], traj[idx, 1:4])
        T_c0_c1 = np.dot(np.linalg.inv(T_c0), T_c1)

        T_rel_rot = np.eye(4)
        T_rel_rot[0:3, 0:3] = T_c0_c1[0:3, 0:3]
        rel_rot.append([traj[idx, 0], tu.compute_angle(T_rel_rot)])
    return np.array(rel_rot)


def computetheta_forimu(imudata):
    rel_rot = []
    imudata_size = imudata.shape;
    print("===================imudata.shape: ", imudata_size[0])
    T_c0 = tu.get_rigid_body_trafo(imudata[0, 1:], np.zeros([1,3]))
    for idx  in range(imudata_size[0]):
        T_c1 = tu.get_rigid_body_trafo(imudata[idx, 1:], np.zeros([1,3]))
        T_c0_c1 = np.dot(np.linalg.inv(T_c0), T_c1)

        T_rel_rot = np.eye(4)
        T_rel_rot[0:3, 0:3] = T_c0_c1[0:3, 0:3]
        rel_rot.append([imudata[idx, 0], tu.compute_angle(T_rel_rot)])
    return np.array(rel_rot)


def findTimeshiftViconImu(vicon_omega, imu_omega):
        print "Estimating time shift lidar to imu:"

        # predict time shift prior
        t = []

        # IMU omega
        omega_measured_norm = []

        # Vicon omega
        omega_predicted_norm = []
        tviconmax = (vicon_omega.max(axis=0))[0]
        tviconmin = (vicon_omega.min(axis=0))[0]
        t_vicon_times = vicon_omega[:,0]
        for idx_imu in range(imu_omega.shape[0]):
            timu_idx = imu_omega[idx_imu,0]
            if timu_idx > tviconmin and timu_idx < tviconmax:
                # get imu measurements and spline from camera
                omega_measured = imu_omega[idx_imu,1]

                # find the closet measurements in vicon
                timediff_vi = t_vicon_times - timu_idx*np.ones(t_vicon_times.shape);
                timediff_vi = np.abs(timediff_vi)
                omega_predicted = vicon_omega[timediff_vi.argmin(axis=0),1]
                # print('Time of omega_measured, omega_predicted: ', timu_idx, '    ', vicon_omega[timediff_vi.argmin(axis=0),0])

                # calc norm
                t = np.hstack((t, timu_idx))
                omega_measured_norm = np.hstack((omega_measured_norm, omega_measured))
                omega_predicted_norm = np.hstack((omega_predicted_norm, omega_predicted))


        # get the time shift
        corr = np.correlate(omega_predicted_norm, omega_measured_norm, "full")
        discrete_shift = corr.argmax() - (np.size(omega_measured_norm) - 1)

        # get cont. time shift
        times = imu_omega[:,0]
        dT = np.mean(np.diff(times))
        shift = -discrete_shift * dT

        # store the timeshift (t_imu = t_cam + timeshiftCamToImuPrior)
        timeshiftlidarToImuPrior = shift

        print "  Time shift lidar to imu (t_imu = t_vicon + shift):"
        print timeshiftlidarToImuPrior
        return timeshiftlidarToImuPrior


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''Analyze trajectories''')
    parser.add_argument('--vicon_dir', help="Folder to vicon poses txt file", default="/media/xinzuo/Disk_PC1T/lic_data_2020/Related_Pkgs/get_timeoffset/mydata/2019-07-03-20-41-16_vicon_tidy.txt")
    parser.add_argument('--imu_dir', help="Folder to imu poses txt file", default="/media/xinzuo/Disk_PC1T/lic_data_2019/LIC_20190703_WithMC/MCLidarIMUCam20hz_2019-07-03-20-41-10_imudata_rot.txt")
    args = parser.parse_args()
    NS1 = np.loadtxt(args.vicon_dir);
    NS2 = np.loadtxt(args.imu_dir);

    # Calculate the norm of angular velocity of vicon
    omega_fromtraj = computetheta_fortraj(NS1)

    # Calculate the norm of angular velocity of IMU
    omega_fromimu = computetheta_forimu(NS2)
    timeshiftlidarToImuPrior = findTimeshiftViconImu(omega_fromtraj, omega_fromimu)

    ##### Plot the rotation normal
    fig = plt.figure(1);
    ax = fig.add_subplot(111);
    p11,p12 = ax.plot(omega_fromtraj[:, 0], omega_fromtraj[:, 1], 'r--', omega_fromimu[:, 0], omega_fromimu[:, 1], 'b--');  # noros_ORBVIO
    ax.legend((p11,p12), ('Form Traj','From IMU'));
    ax.set_xlabel('Time');
    ax.set_ylabel('Omega');
    plt.title('Omega Norm Before Time Shift');

    ##### Plot the rotation normal
    fig2 = plt.figure(2);
    ay = fig2.add_subplot(111);
    corrected_traj_time = omega_fromtraj[:, 0] + timeshiftlidarToImuPrior*np.ones((omega_fromtraj[:, 0]).shape)
    p21, p22 = ay.plot(corrected_traj_time, omega_fromtraj[:, 1], 'r--', omega_fromimu[:, 0], omega_fromimu[:, 1],
                       'b--');  # noros_ORBVIO
    ay.legend((p21, p22), ('Form Traj', 'From IMU'));
    ay.set_xlabel('Time');
    ay.set_ylabel('Omega');
    plt.title('Omega Norm After Time Shift');
    plt.show();




