#!/usr/bin/env python2
import os
import argparse
import numpy as np
import trajectory_utils as tu
import matplotlib.pyplot as plt
import CaculateTimeshift1 as calTime

# timestamp tx ty tz qx qy qz qw
def computeomeganorm_fortraj(traj):
    rel_rot = []
    traj_size = traj.shape;
    print("===================traj.shape: ", traj_size[0])
    for idx  in range(traj_size[0] -1):
        T_c1 = tu.get_rigid_body_trafo(traj[idx, 4:], traj[idx, 1:4])
        T_c2 = tu.get_rigid_body_trafo(traj[idx+1, 4:], traj[idx+1, 1:4])
        T_c1_c2 = np.dot(np.linalg.inv(T_c1), T_c2)

        T_rel_rot = np.eye(4)
        T_rel_rot[0:3, 0:3] = T_c1_c2[0:3, 0:3]
        dt = traj[idx+1, 0] - traj[idx, 0]
        rel_rot.append([traj[idx, 0], tu.compute_angle(T_rel_rot)/dt*np.pi/180.0])
    return np.array(rel_rot)


# timestamp omega_x omega_y omega_z
def computeomeganorm_forimu(imudata):
    rel_rot = []
    imudata_size = imudata.shape;
    print("===================== imudata.shape: ", imudata_size[0])
    for idx in range(imudata_size[0]):
        omeganorm = np.linalg.norm(imudata[idx, 1:])
        rel_rot.append([imudata[idx,0],omeganorm])
    return np.array(rel_rot)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''Analyze trajectories''')
    parser.add_argument('--odom_dir', help="Odom filepath", default="./data/0_odom.txt")
    parser.add_argument('--imu_dir', help="Omega filepath", default="./data/0_omega.txt")
    parser.add_argument('--result_dir', help="Omega filepath", default="./data/result.png")
    args = parser.parse_args()
    NS1 = np.loadtxt(args.odom_dir);   
    NS2 = np.loadtxt(args.imu_dir);

    # Calculate the norm of angular velocity of vicon
    omega_fromtraj = computeomeganorm_fortraj(NS1)

    # Calculate the norm of angular velocity of IMU
    omega_fromimu = computeomeganorm_forimu(NS2)
    
    timeshiftlidarToImuPrior = calTime.findTimeshiftViconImu(omega_fromtraj, omega_fromimu)
    
    ##### Plot the rotation normal
    fig = plt.figure(1);
    fig.set_size_inches(13, 8)
        
    ax = fig.add_subplot(211);
    p11,p12 = ax.plot(omega_fromtraj[:, 0], omega_fromtraj[:, 1], 'r--', omega_fromimu[:, 0], omega_fromimu[:, 1], 'b--');  # noros_ORBVIO
    ax.legend((p11,p12), ('Form Traj','From IMU'));
    ax.set_ylabel('Omega norm');
    plt.title('Omega Norm Before Time Shift');

    ##### Plot the rotation normal
    ay = fig.add_subplot(212);
    corrected_traj_time = omega_fromtraj[:, 0] + timeshiftlidarToImuPrior*np.ones((omega_fromtraj[:, 0]).shape)
    p21, p22 = ay.plot(corrected_traj_time, omega_fromtraj[:, 1], 'r--', omega_fromimu[:, 0], omega_fromimu[:, 1],
                       'b--');  # noros_ORBVIO
    ay.legend((p21, p22), ('Form Traj', 'From IMU'));
    ay.set_xlabel('Time');
    ay.set_ylabel('Omega norm');
    plt.title('Omega Norm After Time Shift: '+str(timeshiftlidarToImuPrior));
    #plt.show();
    fig.savefig(args.result_dir, dpi=100)
