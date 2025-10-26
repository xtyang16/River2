/**
 * @file imu.cpp
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

#include "sensor/imu.h"

namespace ns_river2 {
    IMUFrameArray::IMUFrameArray(double timestamp, const std::vector<IMUFrame::Ptr> &frames)
            : _timestamp(timestamp), _frames(frames) {}

    IMUFrameArray::Ptr IMUFrameArray::Create(double timestamp, const std::vector<IMUFrame::Ptr> &frames) {
        return std::make_shared<IMUFrameArray>(timestamp, frames);
    }

    double IMUFrameArray::GetTimestamp() const {
        return _timestamp;
    }

    void IMUFrameArray::SetTimestamp(double timestamp) {
        _timestamp = timestamp;
    }

    const std::vector<IMUFrame::Ptr> &IMUFrameArray::GetFrames() const {
        return _frames;
    }

    bool IMUFrameArray::SaveFramesArraysToDisk(const std::string &filename,
                                               const std::vector<IMUFrameArray::Ptr> &arrays,
                                               int precision) {
        std::ofstream file(filename);
        file << std::fixed << std::setprecision(precision);
        cereal::JSONOutputArchive ar(file);
        ar(cereal::make_nvp("imu_arrays", arrays));
        return true;
    }
}
