/**
 * @file imu.h
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

#ifndef RIVER2_IMU_H
#define RIVER2_IMU_H

#include "ctraj/core/imu.h"

namespace ns_river2 {
    using IMUFrame = ns_ctraj::IMUFrame;

    struct IMUFrameArray {
    public:
        using Ptr = std::shared_ptr<IMUFrameArray>;

    private:
        // the timestamp of this array
        double _timestamp;
        std::vector<IMUFrame::Ptr> _frames;

    public:
        explicit IMUFrameArray(double timestamp = INVALID_TIME_STAMP, const std::vector<IMUFrame::Ptr> &frames = {});

        static IMUFrameArray::Ptr
        Create(double timestamp = INVALID_TIME_STAMP, const std::vector<IMUFrame::Ptr> &frames = {});

        [[nodiscard]] double GetTimestamp() const;

        void SetTimestamp(double timestamp);

        [[nodiscard]] const std::vector<IMUFrame::Ptr> &GetFrames() const;

        // save radar frames sequence to disk
        static bool SaveFramesArraysToDisk(const std::string &filename,
                                           const std::vector<IMUFrameArray::Ptr> &arrays,
                                           int precision = 10);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        template<class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("frames", _frames));
        }
    };
}


#endif //RIVER2_IMU_H
