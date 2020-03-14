本工具包实现了一个简单的激光里程计。基于激光里程计结果和IMU原始角速度测量值，计算两者角速度模长序列，最大两条序列的互相关系数，**实现时间偏置的粗略标定**（参考[Kalibr](https://github.com/ethz-asl/kalibr/blob/master/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py#L215:9.)）。

本方法理论上时间偏置的最大标定精度是IMU测量周期，但是由于当前激光里程计比较简陋，激光轨迹准确度不高，时间偏置的标定精度比较依赖激光里程计的精度。本工具包现在主要用于粗略评价激光和IMU硬件同步板是否正确。

本工具包可方便扩展为Vicon和IMU的时间偏置标定。

### Dependencies

**ndt_omp**  用加速版NDT匹配点云。参考https://github.com/koide3/ndt_omp。

### Run

**1. 激光里程计**

```shell
roslaunch lidar_odometry lidar_odom.launch \
            path_bag:="test.bag" \
            topic_imu:="/imu" \
            bag_start:="1" \
            bag_durr:="10" \
            lidar_odom_path:="./odom.txt" \
            imu_omega_path:="./omega.txt" 
```

其中`odom.txt`存储形式为`timestamp tx ty tz qx qy qz qw`，

`omega.txt`存储形式为`timestamp omega_x omega_y omega_z`。

**2. 时间偏置粗略标定**

```shell
python ./scripts/CaculateTimeshift.py \
        --odom_dir "./odom.txt" \
        --imu_dir "./omega.txt" \
        --result_dir "./result.png"
```

**3. 批量标定rosbag数据包**

需要指定上述两个脚本的全部参数。

该脚本会在数据包路径下，新建以数据包名字为名的文件夹，保存数据处理结果。

```shell
./launch/cal_timeshift.sh
```

### Results

`./scripts/data` 文件夹下有两个范例，标定结果如下。

![](./scripts/data/0_result.png)
下图中激光轨迹存在跳变，读者若是有兴趣，可以提高下激光里程计鲁棒性。
![](./scripts/data/1_result.png)