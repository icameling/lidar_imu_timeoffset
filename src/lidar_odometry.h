#ifndef CALIBR_LIDAR_ODOMETRY_H
#define CALIBR_LIDAR_ODOMETRY_H

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pclomp/ndt_omp.h>
#include "vlp_common.h"

namespace lidar_odom {

class Odometry {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Odometry> Ptr;

    struct OdomData {
        /// Timestamp of the reading
        double timestamp;
        Eigen::Matrix4d pose; // cur pose to first pose
    };

    Odometry() {}

    void addOdomData(OdomData odomdata){
      _odom_data.push_back(odomdata);
    }

    void addOdomData(double timestamp, Eigen::Matrix4d pose){
      OdomData data;
      data.timestamp = timestamp;
      data.pose = pose;
      _odom_data.push_back(data);
    }

    OdomData getOdom(int idx) {
      return _odom_data.at(idx);
    }

    OdomData getOdomLast() {
      return _odom_data.back();
    }

    size_t n_odomData() {return _odom_data.size();}

    void updataOdomData(size_t idx, Eigen::Matrix4d pose) {
      assert(idx < _odom_data.size() && "[updataOdomData]: idx out of range.");
      _odom_data.at(idx).pose = pose;
    }

    void save_odom_data(std::string filename_odom) {
      std::ofstream outfile_odom(filename_odom);
      outfile_odom << "#timestamp tx ty tz qx qy qz qw" << std::endl;
      // timestamp
      outfile_odom.precision(6);
      outfile_odom.setf(std::ios::fixed);
      for (OdomData const& data : _odom_data) {

        Eigen::Quaterniond q_LtoM = Eigen::Quaterniond(data.pose.block<3,3>(0,0));
        Eigen::Vector3d p_LinM = data.pose.block<3,1>(0,3);
        outfile_odom<<data.timestamp<<" "
                   <<p_LinM[0]<<" "<<p_LinM[1]<<" "<<p_LinM[2]<<" "
                  <<q_LtoM.x()<<" "<<q_LtoM.y()<<" "<<q_LtoM.z()<<" "<<q_LtoM.w()<<std::endl;
      }
      std::cout << "Save " << _odom_data.size() << " estimated odometry to " << filename_odom << std::endl;
      outfile_odom.close();
    }

private:
    /// Our history of odometry messages
    std::vector<OdomData> _odom_data;

};

class LiDAROdometry {

public:
    typedef std::shared_ptr<LiDAROdometry> Ptr;
    struct ScanData {
        /// Timestamp of the reading
        unsigned int seq;
        VPointCloud::Ptr cloud;
    };

    LiDAROdometry(double ndtResolution = 0.5, double KNearRadius=0.05) {
        _ndt_omp = pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr(
                new pclomp::NormalDistributionsTransform<VPoint, VPoint>());
        _ndt_omp->setResolution(ndtResolution);
        _ndt_omp->setNumThreads(2);
        _ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        _ndt_omp->setTransformationEpsilon(1e-3);
        _ndt_omp->setStepSize(0.01);
        _ndt_omp->setMaximumIterations(50);
        mMapinitialed = false;

        mSumPointCloud = VPointCloud::Ptr(new VPointCloud());
        p_odom = Odometry::Ptr(new Odometry());
        _pose_accumu = Eigen::Matrix4d::Identity();
    }

    void addScanAndOdom(ScanData scandata, Odometry::OdomData odomdata) {
      _scan_data.push_back(scandata);
      p_odom->addOdomData(odomdata);
    }


    void feed_scan(double timestamp, VPointCloud::Ptr point_cloud);

    bool needUpdate(Odometry::OdomData odomdata);
    bool updateKeyScan(size_t check_idx);
    inline double limitYaw(double value)
    {
        if(value>3.1415926)
            value -= 3.1415926;
        if(value < -3.1415926)
            value += 3.1415926;
        return value;
    }

    /// 输入一帧点云和位姿估计值,用Ndt匹配
    Eigen::Matrix4d getPose(LiDAROdometry::ScanData &scan, Eigen::Matrix4f initPose);


    // ==================== get funtions ===================//
    size_t n_scan() {return _scan_data.size();}

    ScanData getScan(int idx) {
      return _scan_data.at(idx);
    }

    inline pcl::PointCloud<pcl::PointXYZI>::Ptr getSumMap(){return mSumPointCloud;}

    Odometry::Ptr getOdom() {return p_odom;}

private:
    /// Our history of Scan messages
    std::vector<ScanData> _scan_data;
    Eigen::Matrix4d _pose_accumu;

    bool mMapinitialed;
    pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr _ndt_omp;
    VPointCloud::Ptr mSumPointCloud;//map点云。
    std::vector<size_t> mKeyScanVector;//关键帧序列

    Odometry::Ptr p_odom;

};


} // namespace Calibr


#endif // CALIBR_LIDAR_ODOMETRY_H
