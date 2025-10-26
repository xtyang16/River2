/**
 * @file calib_param_manager.cpp
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

#include "core/calib_param_manager.h"
#include "filesystem"
#include "spdlog/fmt/bundled/color.h"
#include "util/utils.hpp"

namespace ns_river2
{

    CalibParamManager::CalibParamManager() : EXTRI(), TEMPORAL() {}

    CalibParamManager::Ptr CalibParamManager::Create()
    {
        return std::make_shared<CalibParamManager>();
    }

    void CalibParamManager::ShowParamStatus()
    {
        std::stringstream stream;
#define ITEM(name) fmt::format(fmt::emphasis::bold | fmt::fg(fmt::color::green), name)
#define PARAM(name) fmt::format(fmt::emphasis::bold, name)
#define STREAM_PACK(obj) stream << "-- " << obj << std::endl;

        constexpr std::size_t n = 74;

        STREAM_PACK(std::string(25, '-'))
        STREAM_PACK(ITEM("calibration parameters") << " --")
        STREAM_PACK(std::string(n, '-'))

        // -------------------------
        STREAM_PACK(ITEM("EXTRI"))
        // -------------------------
        STREAM_PACK("")

        for (const auto &[imuTopic, _] : CalibParamManager::EXTRI.SO3_BiToBi0)
        {
            STREAM_PACK("IMU: '" << imuTopic << "'")
            const auto EULER_BiToBc = EXTRI.EULER_BiToBc_DEG(imuTopic);
            STREAM_PACK(PARAM("EULER_BiToBc: ") << FormatValueVector<double>(
                            {"Xr", "Yp", "Zy"}, {EULER_BiToBc(2), EULER_BiToBc(1), EULER_BiToBc(0)}))

            const auto POS_BiInBi0 = EXTRI.POS_BiInBi0.at(imuTopic);
            STREAM_PACK(PARAM("  POS_BiInBi0: ") << FormatValueVector<double>(
                            {"Px", "Py", "Pz"}, {POS_BiInBi0(2), POS_BiInBi0(1), POS_BiInBi0(0)}))
            STREAM_PACK("")
        }

        for (const auto &[radarTopic, _] : CalibParamManager::EXTRI.SO3_RjToBi0)
        {
            STREAM_PACK("Radar: '" << radarTopic << "'")
            const auto EULER_RjToBc = EXTRI.EULER_RjToBc_DEG(radarTopic);
            STREAM_PACK(PARAM("EULER_RjToBc: ") << FormatValueVector<double>(
                            {"Xr", "Yp", "Zy"}, {EULER_RjToBc(2), EULER_RjToBc(1), EULER_RjToBc(0)}))

            const auto POS_RjInBi0 = EXTRI.POS_RjInBi0.at(radarTopic);
            STREAM_PACK(PARAM("  POS_RjInBi0: ") << FormatValueVector<double>(
                            {"Px", "Py", "Pz"}, {POS_RjInBi0(2), POS_RjInBi0(1), POS_RjInBi0(0)}))
            STREAM_PACK("")
        }
        STREAM_PACK(std::string(n, '-'))

        // ----------------------------
        STREAM_PACK(ITEM("TEMPORAL"))
        // ----------------------------
        STREAM_PACK("")
        for (const auto &[imuTopic, _] : CalibParamManager::EXTRI.SO3_BiToBi0)
        {
            STREAM_PACK("IMU: '" << imuTopic << "'")
            const auto TIME_OFFSET_BiToBi0 = TEMPORAL.TIME_OFFSET_BiToBi0.at(imuTopic);
            STREAM_PACK(fmt::format("{}: {:+011.6f} (s)", PARAM("TIME_OFFSET_BiToBi0"), TIME_OFFSET_BiToBi0))
            STREAM_PACK("")
        }
        for (const auto &[radarTopic, _] : CalibParamManager::EXTRI.SO3_RjToBi0)
        {
            STREAM_PACK("Radar: '" << radarTopic << "'")
            const auto TIME_OFFSET_RjToBi0 = TEMPORAL.TIME_OFFSET_RjToBi0.at(radarTopic);
            STREAM_PACK(fmt::format("{}: {:+011.6f} (s)", PARAM("TIME_OFFSET_RjToBi0"), TIME_OFFSET_RjToBi0))
            STREAM_PACK("")
        }
        STREAM_PACK(std::string(n, '-'))

        spdlog::info("the detail calibration parameters are below: \n{}", stream.str());

#undef ITEM
#undef PARAM
    }

    void CalibParamManager::VisualizationSensors(ns_viewer::Viewer &viewer) const
    {
        auto SE3_BcToBc = Sophus::SE3f();
        auto centerIMU = ns_viewer::IMU::Create(
            ns_viewer::Posef(SE3_BcToBc.so3().matrix(), SE3_BcToBc.translation()), 0.1,
            ns_viewer::Colour(0.3f, 0.3f, 0.3f, 1.0f));
        viewer.AddEntity(centerIMU);

        for (const auto &[imuTopic, _] : CalibParamManager::EXTRI.SO3_BiToBi0)
        {
            auto BiToBc = EXTRI.SE3_BiToBc(imuTopic).cast<float>();
            auto imu = ns_viewer::IMU::Create(
                ns_viewer::Posef(BiToBc.so3().matrix(), BiToBc.translation()), 0.1);
            auto line = ns_viewer::Line::Create(
                Eigen::Vector3f::Zero(), BiToBc.translation().cast<float>(), ns_viewer::Colour::Black());
            viewer.AddEntity({imu, line});
        }

        for (const auto &[radarTopic, _] : CalibParamManager::EXTRI.SO3_RjToBi0)
        {
            auto RjToBc = EXTRI.SE3_RjToBc(radarTopic).cast<float>();
            auto radar = ns_viewer::Radar::Create(
                ns_viewer::Posef(RjToBc.so3().matrix(), RjToBc.translation()), 0.1, ns_viewer::Colour::Blue());
            auto line = ns_viewer::Line::Create(
                Eigen::Vector3f::Zero(), RjToBc.translation().cast<float>(), ns_viewer::Colour::Black());
            viewer.AddEntity({radar, line});
        }
    }

    CalibParamManager::Ptr
    CalibParamManager::AlignParamToNewSensor(const Sophus::SE3d &SE3_BcToNew, double TF_BcToNew) const
    {
        auto alignedParam = CalibParamManager::Create();
        *alignedParam = *this;
        for (const auto &[imuTopic, _] : alignedParam->EXTRI.SO3_BiToBi0)
        {
            auto alignedSE3 = SE3_BcToNew * alignedParam->EXTRI.SE3_BiToBc(imuTopic);

            alignedParam->EXTRI.SO3_BiToBi0.at(imuTopic) = alignedSE3.so3();
            alignedParam->EXTRI.POS_BiInBi0.at(imuTopic) = alignedSE3.translation();

            alignedParam->TEMPORAL.TIME_OFFSET_BiToBi0.at(imuTopic) += TF_BcToNew;
        }
        for (const auto &[radarTopic, _] : alignedParam->EXTRI.SO3_RjToBi0)
        {
            auto alignedSE3 = SE3_BcToNew * alignedParam->EXTRI.SE3_RjToBc(radarTopic);

            alignedParam->EXTRI.SO3_RjToBi0.at(radarTopic) = alignedSE3.so3();
            alignedParam->EXTRI.POS_RjInBi0.at(radarTopic) = alignedSE3.translation();

            alignedParam->TEMPORAL.TIME_OFFSET_RjToBi0.at(radarTopic) += TF_BcToNew;
        }
        return alignedParam;
    }

    CalibParamManager::Ptr CalibParamManager::AlignParamToNewSensor(const std::string &topic) const
    {
        return AlignParamToNewSensor(
            this->EXTRI.SE3_BiToBc(topic).inverse(), -this->TEMPORAL.TIME_OFFSET_BiToBi0.at(topic));
    }

    void CalibParamManager::Save(const std::string &filename, CerealArchiveType::Enum archiveType) const
    {
        std::ofstream file(filename, std::ios::out);
        auto ar = GetOutputArchiveVariant(file, archiveType);
        SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("CalibParam", *this));
    }

    CalibParamManager::Ptr CalibParamManager::Load(const std::string &filename, CerealArchiveType::Enum archiveType)
    {
        auto calibParamManager = CalibParamManager::Create();
        std::ifstream file(filename, std::ios::in);
        auto ar = GetInputArchiveVariant(file, archiveType);
        SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("CalibParam", *calibParamManager));
        return calibParamManager;
    }
} // namespace ns_river2
