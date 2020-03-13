#ifndef _LIDARCORRECTION_HPP_
#define _LIDARCORRECTION_HPP_

#include <ros/ros.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <angles/angles.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>

namespace lidar_odom {

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

/**
 * @brief The LidarCorrection class
 * velodyne矫正
 */
class VelodyneCorrection
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<VelodyneCorrection> Ptr;

    enum ModelType
    {
        UNINIT,
        VLP_16,
        HDL_32E,
        VLP_32C
    };

    VelodyneCorrection(ModelType modelType = VLP_16)
    {
        m_modelType = modelType;
        setParameters(m_modelType);
        std::cout << "Velodyne model is initialed as VLP_16" << std::endl;
    }

    /**
     * @brief setParameters
     * @param modelType 根据激光雷达型号配置不同的参数
     */
    void setParameters(ModelType modelType)
    {
        m_modelType = modelType;
        m_config.max_range=100;
        m_config.min_range=0.5;
        //m_config.min_angle = 0;
        //m_config.max_angle = 36000;
        m_config.min_angle = 22500;
        m_config.max_angle = 13500;
        // Set up cached values for sin and cos of all the possible headings
        for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
            float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
            cos_rot_table_[rot_index] = cosf(rotation);
            sin_rot_table_[rot_index] = sinf(rotation);
        }

        if(modelType == VLP_16)
        {
            FIRINGS_PER_BLOCK =   2;
            SCANS_PER_FIRING  =  16;
            BLOCK_TDURATION   = 110.592f;   // [µs]
            DSR_TOFFSET       =   2.304f;   // [µs]
            FIRING_TOFFSET    =  55.296f;   // [µs]
            PACKET_TIME = (BLOCKS_PER_PACKET*2*FIRING_TOFFSET);//一个packet的时间12*2*55.296

            /// 不同lidar,这个角度不同,需要看出厂配置文件
            float vert_correction[16]=\
            {
                    -0.2617993877991494,
                    0.017453292519943295,
                    -0.22689280275926285,
                    0.05235987755982989,
                    -0.19198621771937624,
                    0.08726646259971647,
                    -0.15707963267948966,
                    0.12217304763960307,
                    -0.12217304763960307,
                    0.15707963267948966,
                    -0.08726646259971647,
                    0.19198621771937624,
                    -0.05235987755982989,
                    0.22689280275926285,
                    -0.017453292519943295,
                    0.2617993877991494
            };
            for(int i=0; i<16; i++)
            {
                cos_vert_angle_[i] = std::cos(vert_correction[i]);
                sin_vert_angle_[i] = std::sin(vert_correction[i]);
            }
            scan_mapping_16[0]=15;
            scan_mapping_16[1]=7;
            scan_mapping_16[2]=14;
            scan_mapping_16[3]=6;
            scan_mapping_16[4]=13;
            scan_mapping_16[5]=5;
            scan_mapping_16[6]=12;
            scan_mapping_16[7]=4;
            scan_mapping_16[8]=11;
            scan_mapping_16[9]=3;
            scan_mapping_16[10]=10;
            scan_mapping_16[11]=2;
            scan_mapping_16[12]=9;
            scan_mapping_16[13]=1;
            scan_mapping_16[14]=8;
            scan_mapping_16[15]=0;
        }

        if(modelType == HDL_32E)
        {
            FIRINGS_PER_BLOCK =   1;
            SCANS_PER_FIRING  =  32;
            BLOCK_TDURATION   =  46.08f;   // [µs]
            DSR_TOFFSET       =   1.152f;   // [µs]
            FIRING_TOFFSET    =  46.08f;   // [µs]
            PACKET_TIME = (BLOCKS_PER_PACKET*FIRING_TOFFSET);//一个packet的时间12*46.08
            float vert_correction[32]=\
            {//用角度，不用弧度
                    -30.67,
                    -9.33,
                    -29.33,
                    -8,
                    -28,
                    -6.67,
                    -26.67,
                    -5.33,
                    -25.33,
                    -4,
                    -24,
                    -2.67,
                    -22.67,
                    -1.33,
                    -21.33,
                    0.00,
                    -20,
                    1.33,
                    -18.67,
                    2.67,
                    -17.33,
                    4.0,
                    -16,
                    5.33,
                    -14.67,
                    6.67,
                    -13.33,
                    8,
                    -12,
                    9.33,
                    -10.67,
                    10.67
            };
            for(int i=0; i<32; i++)
            {
                cos_vert_angle_[i] = std::cos(vert_correction[i]/180*3.14159);
                sin_vert_angle_[i] = std::sin(vert_correction[i]/180*3.14159);
            }
            scan_mapping_32[0]=31;
            scan_mapping_32[1]=15;
            scan_mapping_32[2]=30;
            scan_mapping_32[3]=14;
            scan_mapping_32[4]=29;
            scan_mapping_32[5]=13;
            scan_mapping_32[6]=28;
            scan_mapping_32[7]=12;
            scan_mapping_32[8]=27;
            scan_mapping_32[9]=11;
            scan_mapping_32[10]=26;
            scan_mapping_32[11]=10;
            scan_mapping_32[12]=25;
            scan_mapping_32[13]=9;
            scan_mapping_32[14]=24;
            scan_mapping_32[15]=8;
            scan_mapping_32[16]=23;
            scan_mapping_32[17]=7;
            scan_mapping_32[18]=22;
            scan_mapping_32[19]=6;
            scan_mapping_32[20]=21;
            scan_mapping_32[21]=5;
            scan_mapping_32[22]=20;
            scan_mapping_32[23]=4;
            scan_mapping_32[24]=19;
            scan_mapping_32[25]=3;
            scan_mapping_32[26]=18;
            scan_mapping_32[27]=2;
            scan_mapping_32[28]=17;
            scan_mapping_32[29]=1;
            scan_mapping_32[30]=16;
            scan_mapping_32[31]=0;
        }
        if(modelType == VLP_32C)
        {
            std::cout << "comming soon" << std::endl;
        }
    }

    /**
     * @brief correction 不使用矫正
     * @param lidarMsg 输入的packet数据
     * @param outPointCloud 输出点云
     */
    void unpack_scan(const velodyne_msgs::VelodyneScan::ConstPtr &lidarMsg, VPointCloud &outPointCloud) {
        if(m_modelType == ModelType::UNINIT) {
            std::cout<< "ERROR! ModelType not set!" << std::endl;
            return ;
        }
        outPointCloud.header = pcl_conversions::toPCL(lidarMsg->header);
        outPointCloud.clear();
        if(m_modelType == ModelType::VLP_16) {
            outPointCloud.height = 16;
            outPointCloud.width = 24*(int)lidarMsg->packets.size();
            outPointCloud.is_dense = false;
            outPointCloud.resize(outPointCloud.height * outPointCloud.width);
        }
        if(m_modelType == ModelType::HDL_32E) {
            outPointCloud.height = 32;
            outPointCloud.width = 12*(int)lidarMsg->packets.size();
            outPointCloud.is_dense = false;
            outPointCloud.resize(outPointCloud.height * outPointCloud.width);
        }

        int block_counter=0;//用于统计整个scan中经过了多少了block
        //uint64_t time_first_us = lidarMsg->header.stamp.toNSec()/1000;//当前packet的第一束激光的时间,这儿是utc时间！
        for (size_t i = 0; i < lidarMsg->packets.size(); ++i) {
            float azimuth;  // 水平面方位角
            float azimuth_diff;
            float last_azimuth_diff=0;
            float azimuth_corrected_f;
            int azimuth_corrected;
            float x, y, z;

            const raw_packet_t *raw = (const raw_packet_t *) &lidarMsg->packets[i].data[0];  // 获得了有效数据的首地址

            for (int block = 0; block < BLOCKS_PER_PACKET; block++, block_counter++) {  //12
                // Calculate difference between current and next block's azimuth angle.
                azimuth = (float)(raw->blocks[block].rotation);  // 先获取当前block的方位角
                //在这儿自适应激光雷达各种转速
                if (block < (BLOCKS_PER_PACKET-1)){
                    azimuth_diff = (float)((36000 + raw->blocks[block+1].rotation - raw->blocks[block].rotation)%36000);
                    last_azimuth_diff = azimuth_diff;
                }
                else {
                    azimuth_diff = last_azimuth_diff;
                }

                for (int firing=0, k=0; firing < FIRINGS_PER_BLOCK; firing++) {
                    for (int dsr=0; dsr < SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE) {//16
                        /** Position Calculation */
                        union two_bytes tmp;
                        tmp.bytes[0] = raw->blocks[block].data[k];
                        tmp.bytes[1] = raw->blocks[block].data[k+1];

                        /** correct for the laser rotation as a function of timing during the firings **/
                        azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*DSR_TOFFSET) + (firing*FIRING_TOFFSET)) / BLOCK_TDURATION);
                        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;  //获得了每一根线的精确角度

                        /*condition added to avoid calculating points which are not
                      in the interesting defined area (min_angle < area < max_angle)*/
                        //ros坐标系下，-y是0，x是9000,范围是0-36000
                        if ((azimuth_corrected >= m_config.min_angle
                             && azimuth_corrected <= m_config.max_angle
                             && m_config.min_angle < m_config.max_angle)
                            || (m_config.min_angle > m_config.max_angle
                                && (azimuth_corrected <= m_config.max_angle
                                    || azimuth_corrected >= m_config.min_angle))) {
                            // convert polar coordinates to Euclidean XYZ
                            float distance = tmp.uint * DISTANCE_RESOLUTION;

                            // 垂直方向的夹角
                            float cos_vert_angle = cos_vert_angle_[dsr];
                            float sin_vert_angle = sin_vert_angle_[dsr];

                            // 水平方向的夹角
                            float cos_rot_angle = cos_rot_table_[azimuth_corrected];
                            float sin_rot_angle = sin_rot_table_[azimuth_corrected];

                            x = distance * cos_vert_angle * sin_rot_angle;
                            y = distance * cos_vert_angle * cos_rot_angle;
                            z = distance * sin_vert_angle;

                            /** Use standard ROS coordinate system (right-hand rule) */
                            float x_coord = y;
                            float y_coord = -x;
                            float z_coord = z;

                            //注意32线坐标不一样，为了和通用驱动一致，需要改变imu的安装方式！
                            float intensity = raw->blocks[block].data[k+2];  // 反射率

                            // 计算出每一个激光点的精确采集时间
                            // uint64_t time_now = time_first_us  + (uint32_t)(((float)dsr*DSR_TOFFSET) + ((float)firing*FIRING_TOFFSET) + (float)block*BLOCK_TDURATION+ (float)i*PACKET_TIME);

                            Eigen::Vector3f out(x_coord, y_coord, z_coord);
                            if (pointInRange(distance))
                            {  //无效值的dis都是0，intensity有大有小
                                // append this point to the cloud
                                VPoint point;
                                point.x = out(0);
                                point.y = out(1);
                                point.z = out(2);
                                point.intensity = intensity;
                                //point.curvature = time_now.float32;  //曲率保存精确的时间，注意float占用4个byte，所以用float存。在utc时间下，这个变量没有意义了！
                                if(m_modelType == ModelType::VLP_16)
                                    outPointCloud.at(2*block_counter+firing, scan_mapping_16[dsr]) = point;
                                if(m_modelType == ModelType::HDL_32E)
                                    outPointCloud.at(block_counter+firing, scan_mapping_32[dsr]) = point;
                            }
                            else
                            {
                                VPoint point;
                                point.x = NAN;
                                point.y = NAN;
                                point.z = NAN;
                                point.intensity = 0;
                                if(m_modelType == ModelType::VLP_16)
                                    outPointCloud.at(2*block_counter+firing, scan_mapping_16[dsr]) = point;
                                if(m_modelType == ModelType::HDL_32E)
                                    outPointCloud.at(block_counter+firing, scan_mapping_32[dsr]) = point;
                            }
                        }
                    }
                }
            }
        }
    }


    static const int RAW_SCAN_SIZE = 3;
    static const int SCANS_PER_BLOCK = 32;
    static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);
    constexpr static const float ROTATION_RESOLUTION = 0.01f;// [deg] 当用很低的转速采集数据，需要修改
    static const uint16_t ROTATION_MAX_UNITS = 36000u; // [deg/100] 当用很低的转速采集数据，需要修改
    constexpr static const float DISTANCE_RESOLUTION = 0.002f; // [m]

    /** @todo make this work for both big and little-endian machines */
    static const uint16_t UPPER_BANK = 0xeeff;
    static const uint16_t LOWER_BANK = 0xddff;

    static const int BLOCKS_PER_PACKET = 12;
    static const int PACKET_STATUS_SIZE = 2;

    int    FIRINGS_PER_BLOCK;
    int    SCANS_PER_FIRING;
    float  BLOCK_TDURATION;
    float  DSR_TOFFSET;
    float  FIRING_TOFFSET;
    float  PACKET_TIME ;//一个packet的时间

    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];
    float cos_vert_angle_[32];//16和32线共用
    float sin_vert_angle_[32];//16和32线共用
    int scan_mapping_16[16];//vlp16线的曝光触发顺序
    int scan_mapping_32[32];//vlp32线的曝光触发顺序
    typedef struct raw_block
    {
        uint16_t header;        ///< UPPER_BANK or LOWER_BANK
        uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
        uint8_t  data[BLOCK_DATA_SIZE];
    } raw_block_t;

    union two_bytes
    {
        uint16_t uint;
        uint8_t  bytes[2];
    };

    union four_bytes
    {
        uint32_t uint32;
        float_t  float32;
    };

    typedef struct raw_packet
    {
        raw_block_t blocks[BLOCKS_PER_PACKET];
        uint32_t revolution;//时间戳,源程序这里是uint16_t有误
        uint8_t status[PACKET_STATUS_SIZE];//状态位,源程序PACKET_STATUS_SIZE=4有误
    } raw_packet_t;

    /** configuration parameters */
    typedef struct {
        double max_range;                ///< maximum range to publish
        double min_range;
        int min_angle;                   ///< minimum angle to publish
        int max_angle;                   ///< maximum angle to publish
    } Config;
    Config m_config;

protected:
    ModelType m_modelType;

    inline bool pointInRange(float range)
    {
        return (range >= m_config.min_range
                && range <= m_config.max_range);
    }

};

}


#endif
