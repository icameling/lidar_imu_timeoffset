#!/usr/bin/env bash

# Reference
#https://github.com/rpng/open_vins/blob/master/ov_msckf/launch/run_ros_eth.sh

# Source our workspace directory to load ENV variables
source ../../../devel/setup.bash
#=============================================================
#=============================================================

#数据包路径
bag_path="/home/ha/lic_dataset"


#数据包名字
bag_name=(
"2020-01-14-10-09-53SyncSystemTime"
"2020-01-14-10-04-16SyncSystemTime"
"2020-01-14-10-02-23SyncSystemTime"
)


#使用的imu topic
imu_topic_name=(
"/imu1/raw/data_withSync"
#"/imu2/raw/data_withSync"
#"/imu3/raw/data_withSync"
#"/imu"
)

bag_start=4
bag_durr=10

for i in "${!bag_name[@]}"; do
        #检查文件夹是否存在
        if [ ! -d "$bag_path/${bag_name[i]}" ];then
          mkdir "$bag_path/${bag_name[i]}"
        fi

		save_path="$bag_path/${bag_name[i]}"
        for j in "${!imu_topic_name[@]}"; do
				echo "path_bag:=$bag_path/${bag_name[i]}.bag"
                echo "topic_imu:=${imu_topic_name[j]}"
				echo "bag_start:=${bag_start}"
				echo "bag_durr:=${bag_durr}"
                echo "========="

                roslaunch lidar_odometry lidar_odom.launch \
						path_bag:="$bag_path/${bag_name[i]}.bag" \
                        topic_imu:="${imu_topic_name[j]}" \
                        bag_start:="${bag_start}" \
                        bag_durr:="${bag_durr}" \
                        lidar_odom_path:="$save_path/${j}_odom.txt" \
                        imu_omega_path:="$save_path/${j}_omega.txt" 
				
				python ../scripts/CaculateTimeshift.py \
						--odom_dir "$save_path/${j}_odom.txt" \
						--imu_dir "$save_path/${j}_omega.txt" \
						--result_dir "$save_path/${j}_result.png"

        done #j in "${!imu_topic_name[@]}"; do
done #for i in "${!bag_name[@]}"; do








