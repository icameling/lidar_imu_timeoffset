#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "lidar_odometry.h"

using namespace lidar_odom;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_odom");
    ros::NodeHandle nh("~");

    std::string topic_imu, topic_lidar;
    nh.param<std::string>("topic_imu", topic_imu, "/imu0");
    nh.param<std::string>("topic_lidar", topic_lidar, "/velodyne_packets");

    // Location of the ROS bag we want to read in
    std::string path_to_bag;
    nh.param<std::string>("path_bag", path_to_bag, "/home/patrick/datasets/eth/V1_01_easy.bag");
    ROS_INFO("ros bag path is: %s", path_to_bag.c_str());

    // Get our start location and how much of the bag we want to play
    // Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO("bag start: %.1f",bag_start);
    ROS_INFO("bag duration: %.1f",bag_durr);

    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0)? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO("time start = %.6f", time_init.toSec());
    ROS_INFO("time end   = %.6f", time_finish.toSec());
    ROS_INFO("time duration   = %.6f", time_finish.toSec()-time_init.toSec());
    view.addQuery(bag, time_init, time_finish);

    // Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    std::string imu_omega_path;
    nh.param<std::string>("imu_omega_path", imu_omega_path, "./");
    std::ofstream outfile_omega(imu_omega_path);
    outfile_omega << "#timestamp wx wy wz" << std::endl;
    // timestamp
    outfile_omega.precision(6);
    outfile_omega.setf(std::ios::fixed);

    LiDAROdometry::Ptr p_lidar_odom = LiDAROdometry::Ptr(new LiDAROdometry());
    VelodyneCorrection::Ptr p_velodyneConvert = VelodyneCorrection::Ptr(new VelodyneCorrection());
    for (const rosbag::MessageInstance& m : view) {

        // If ros is wants us to stop, break out
        if (!ros::ok())
            return 0;

        // Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
        if (s2 != NULL && m.getTopic() == topic_imu) {
            // convert into correct format
            double timestamp = (*s2).header.stamp.toSec();
            outfile_omega << timestamp << " "
                          << (*s2).angular_velocity.x<< " " << (*s2).angular_velocity.y << " " << (*s2).angular_velocity.z << std::endl;
        }


        // Handle LiDAR measurement after received IMU topic
        velodyne_msgs::VelodyneScan::ConstPtr s3 = m.instantiate<velodyne_msgs::VelodyneScan>();
        if ( (s3 != NULL) && m.getTopic() == topic_lidar) {
          VPointCloud pointcloud;
          // 解析激光packet
          p_velodyneConvert->unpack_scan(s3, pointcloud);
          // Frame to map
          p_lidar_odom->feed_scan((*s3).header.stamp.toSec(), pointcloud.makeShared());
        }
    }
    outfile_omega.close();
    std::string lidar_odom_path;
    nh.param<std::string>("lidar_odom_path", lidar_odom_path, "./");
    p_lidar_odom->getOdom()->save_odom_data(lidar_odom_path);


return 0;
}
