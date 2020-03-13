#include "lidar_odometry.h"

using namespace lidar_odom;


// 点云和map匹配获得位姿
void LiDAROdometry::feed_scan(double timestamp, VPointCloud::Ptr point_cloud) {

    ScanData scan_data;
    Odometry::OdomData odom_data;
    scan_data.seq = point_cloud->header.seq;
    scan_data.cloud = point_cloud;
    odom_data.timestamp = timestamp;
    odom_data.pose = Eigen::Matrix4d::Identity();

    if (!mMapinitialed) {
        addScanAndOdom(scan_data, odom_data);
        updateKeyScan(n_scan()-1);
        mMapinitialed = true;
        return;
    }
    static Eigen::Matrix4d T_cur2last = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d T_last2map = p_odom->getOdomLast().pose;
    Eigen::Matrix4d T_cur2map = Eigen::Matrix4d::Identity();
    T_cur2map = getPose(scan_data, T_last2map.cast<float>() * T_cur2last.cast<float>());
    odom_data.pose = T_cur2map;

    // Constant velocity model
    T_cur2last.block<3,3>(0,0) = (T_last2map.block<3,3>(0,0)).transpose() * T_cur2map.block<3,3>(0,0);
    T_cur2last.block<3,1>(0,3) = (T_last2map.block<3,3>(0,0)).transpose() * (T_cur2map.block<3,1>(0,3) - T_last2map.block<3,1>(0,3));

    // Update
    _pose_accumu = odom_data.pose;
    addScanAndOdom(scan_data, odom_data);
    updateKeyScan(n_scan()-1);
}

bool LiDAROdometry::needUpdate(Odometry::OdomData odomdata)
{
  static Eigen::Matrix4d transformBefore = Eigen::Matrix4d::Identity();
  static double yawBefore = 0, rollBefore = 0, pitchBefore = 0;
  static unsigned int couter = 0;

  const Eigen::Matrix3d initial_rot (odomdata.pose.block<3,3> (0,0));
  const Eigen::Vector3d rot_x (initial_rot*Eigen::Vector3d::UnitX ());
  const Eigen::Vector3d rot_y (initial_rot*Eigen::Vector3d::UnitY ());
  const Eigen::Vector3d rot_z (initial_rot*Eigen::Vector3d::UnitZ ());
  double yawNow = std::atan2 (rot_x[1], rot_x[0]);
  double rollNow = std::atan2 (rot_y[2], rot_y[1]);
  double pitchNow = std::atan2 (rot_z[0], rot_z[2]);

  //      cout << "rpy" << ndt_roll/3.14*180 << " " << ndt_pitch/3.14*180 << " " << ndt_yaw/3.14*180 << " "
  //           << "0 1 2 " << rot_x[0] << " " << rot_x[1] << " " << rot_x[2] << " " << endl;

  if(pow(odomdata.pose(0,3) - transformBefore(0,3), 2) +
          pow(odomdata.pose(1,3) - transformBefore(1,3), 2) +
          pow(odomdata.pose(2,3) - transformBefore(2,3), 2) > 0.04
          //         || step_add_counter > 10
                   || fabs(limitYaw(yawBefore-yawNow))>3.1415926/(180.0/5.0)
                   || fabs(limitYaw(rollBefore-rollNow))>3.1415926/(180.0/5.0)
                   || fabs(limitYaw(pitchBefore-pitchNow))>3.1415926/(180.0/5.0)
          || mKeyScanVector.size() ==0
          )
  {
      //step_add_counter = 0;
      transformBefore = odomdata.pose;
      yawBefore = yawNow;
      rollBefore = rollNow;
      pitchBefore = pitchNow;
      couter++;
      return true;
  }
  return false;
}


bool LiDAROdometry::updateKeyScan(size_t check_idx)
{
    if (needUpdate(p_odom->getOdom(check_idx))) {
      mKeyScanVector.push_back(check_idx);
      VPointCloud tempCloud;
      pcl::transformPointCloud(*(getScan(check_idx).cloud), tempCloud, p_odom->getOdom(check_idx).pose);
      *mSumPointCloud += tempCloud;
      _ndt_omp->setInputTarget(mSumPointCloud);

      //应该需要重新叠加点云 之后再降采样1cm,之后再初始化ndt空间。
        return true;
    }
    else {
        return false;
    }
}

void downsampleCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
                     float in_leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(in_cloud);
  sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
  sor.filter(*out_cloud);
}

Eigen::Matrix4d LiDAROdometry::getPose(LiDAROdometry::ScanData &scan, Eigen::Matrix4f initPose)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    downsampleCloud(scan.cloud, p_filtered_cloud, 0.5);

    _ndt_omp->setInputSource(p_filtered_cloud);
    _ndt_omp->align(output_cloud, initPose);

//    bool converged = _ndt_omp->hasConverged();
//    double fitness_score = _ndt_omp->getFitnessScore();
//    int final_num_iteration = _ndt_omp->getFinalNumIteration();
//    if(!converged) {
//        ROS_WARN("NDT does not converge!!!");
//    }

    return _ndt_omp->getFinalTransformation().cast<double>();
}



