/**
 * @file imu_data_loader.h
 * @author Xiaoteng Yang (xtyang@whu.edu.cn)
 * @brief Xiaoteng Yang received the B.S. degree in 
 * geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. 
 * He is currently a master candidate at the school of Geodesy and Geomatics, Wuhan University.
 * @version 0.1
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef RIVER2_IMU_DATA_LOADER_H
#define RIVER2_IMU_DATA_LOADER_H

#include "sensor_msgs/Imu.h"
#include "sbg_driver/SbgImuData.h"
#include "rosbag/message_instance.h"
#include "sensor/imu.h"

namespace ns_river2 {
    enum class IMUMsgType {
        SENSOR_IMU,
        SBG_IMU
    };

    class IMUDataUnpacker {
    public:
        using Ptr = std::shared_ptr<IMUDataUnpacker>;

    public:
        explicit IMUDataUnpacker() = default;

        static IMUDataUnpacker::Ptr Create();

        static IMUFrame::Ptr Unpack(const sensor_msgs::Imu::ConstPtr &msg);

        static IMUFrame::Ptr Unpack(const sbg_driver::SbgImuData::ConstPtr &msg);
    };

}


#endif //RIVER2_IMU_DATA_LOADER_H
