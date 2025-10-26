/**
 * @file calib_param_manager.h
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

#ifndef RIVER2_CALIB_PARAM_MANAGER_H
#define RIVER2_CALIB_PARAM_MANAGER_H

#include "ctraj/view/traj_viewer.h"
#include "cereal/types/map.hpp"
#include "spdlog/spdlog.h"
#include "util/cereal_archive_helper.hpp"
#include "ctraj/utils/sophus_utils.hpp"

namespace ns_river2
{
  struct CalibParamManager
  {
  public:
    using Ptr = std::shared_ptr<CalibParamManager>;

  public:
    // trans radian angle to degree angle
    constexpr static double RAD_TO_DEG = 180.0 / M_PI;
    // trans degree angle to radian angle
    constexpr static double DEG_TO_RAD = M_PI / 180.0;

    // extrinsic
    struct
    {
      std::map<std::string, Sophus::SO3d> SO3_BiToBi0;
      std::map<std::string, Eigen::Vector3d> POS_BiInBi0;

      std::map<std::string, Sophus::SO3d> SO3_RjToBi0;
      std::map<std::string, Eigen::Vector3d> POS_RjInBi0;

      // lie algebra vector space se3
      [[nodiscard]] Sophus::SE3d SE3_RjToBc(const std::string &radarTopic) const
      {
        return {SO3_RjToBi0.at(radarTopic), POS_RjInBi0.at(radarTopic)};
      }

      [[nodiscard]] Sophus::SE3d SE3_BiToBc(const std::string &imuTopic) const
      {
        return {SO3_BiToBi0.at(imuTopic), POS_BiInBi0.at(imuTopic)};
      }

      // quaternion
      [[nodiscard]] Eigen::Quaterniond Q_RjToBc(const std::string &radarTopic) const
      {
        return SO3_RjToBi0.at(radarTopic).unit_quaternion();
      }

      [[nodiscard]] Eigen::Quaterniond Q_BiToBc(const std::string &imuTopic) const
      {
        return SO3_BiToBi0.at(imuTopic).unit_quaternion();
      }

      // the euler angles [radian and degree format]
      [[nodiscard]] Eigen::Vector3d EULER_RjToBc_RAD(const std::string &radarTopic) const
      {
        return Q_RjToBc(radarTopic).toRotationMatrix().eulerAngles(2, 1, 0);
      }

      [[nodiscard]] Eigen::Vector3d EULER_RjToBc_DEG(const std::string &radarTopic) const
      {
        auto euler = EULER_RjToBc_RAD(radarTopic);
        for (int i = 0; i != 3; ++i)
        {
          euler(i) *= CalibParamManager::RAD_TO_DEG;
        }
        return euler;
      }

      [[nodiscard]] Eigen::Vector3d EULER_BiToBc_RAD(const std::string &imuTopic) const
      {
        return Q_BiToBc(imuTopic).toRotationMatrix().eulerAngles(2, 1, 0);
      }

      [[nodiscard]] Eigen::Vector3d EULER_BiToBc_DEG(const std::string &imuTopic) const
      {
        auto euler = EULER_BiToBc_RAD(imuTopic);
        for (int i = 0; i != 3; ++i)
        {
          euler(i) *= CalibParamManager::RAD_TO_DEG;
        }
        return euler;
      }

      std::map<const double *, std::string> GetParamAddressWithDesc()
      {
        std::map<const double *, std::string> infoMap;

#define SAVE_EXTRI_INFO(param) infoMap.insert(std::make_pair(param.data(), #param));

        // imu
        for (const auto &[topic, data] : SO3_BiToBi0)
        {
          infoMap.insert({data.data(), "SO3_BiToBi0[" + topic + "]"});
        }
        for (const auto &[topic, data] : POS_BiInBi0)
        {
          infoMap.insert({data.data(), "POS_BiInBi0[" + topic + "]"});
        }

        // radar
        for (const auto &[topic, data] : SO3_RjToBi0)
        {
          infoMap.insert({data.data(), "SO3_RjToBi0[" + topic + "]"});
        }
        for (const auto &[topic, data] : POS_RjInBi0)
        {
          infoMap.insert({data.data(), "POS_RjInBi0[" + topic + "]"});
        }

#undef SAVE_EXTRI_INFO

        return infoMap;
      }

      std::vector<Sophus::SO3d *> SO3_BiToBi0_AddressVec()
      {
        std::vector<Sophus::SO3d *> addressVec(SO3_BiToBi0.size());
        std::transform(SO3_BiToBi0.begin(), SO3_BiToBi0.end(), addressVec.begin(), [](auto &p)
                       { return &(p.second); });
        return addressVec;
      }

      std::vector<Eigen::Vector3d *> POS_BiInBi0_AddressVec()
      {
        std::vector<Eigen::Vector3d *> addressVec(POS_BiInBi0.size());
        std::transform(POS_BiInBi0.begin(), POS_BiInBi0.end(), addressVec.begin(), [](auto &p)
                       { return &(p.second); });
        return addressVec;
      }

    public:
      // Serialization
      template <class Archive>
      void serialize(Archive &archive)
      {
        archive(CEREAL_NVP(SO3_BiToBi0), CEREAL_NVP(POS_BiInBi0),
                CEREAL_NVP(SO3_RjToBi0), CEREAL_NVP(POS_RjInBi0));
      }

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EXTRI;

    // temporal
    struct
    {
      std::map<std::string, double> TIME_OFFSET_BiToBi0;
      std::map<std::string, double> TIME_OFFSET_RjToBi0;

      std::vector<double *> TIME_OFFSET_BiToBi0_AddressVec()
      {
        std::vector<double *> addressVec(TIME_OFFSET_BiToBi0.size());
        std::transform(TIME_OFFSET_BiToBi0.begin(), TIME_OFFSET_BiToBi0.end(), addressVec.begin(), [](auto &p)
                       { return &(p.second); });
        return addressVec;
      }

      std::map<const double *, std::string> GetParamAddressWithDesc()
      {
        std::map<const double *, std::string> infoMap;

#define SAVE_TEMPORAL_INFO(param) infoMap.insert(std::make_pair(&param, #param));

        for (const auto &[topic, data] : TIME_OFFSET_BiToBi0)
        {
          infoMap.insert({&data, "TIME_OFFSET_BiToBi0[" + topic + "]"});
        }

        for (const auto &[topic, data] : TIME_OFFSET_RjToBi0)
        {
          infoMap.insert({&data, "TIME_OFFSET_RjToBi0[" + topic + "]"});
        }

#undef SAVE_TEMPORAL_INFO

        return infoMap;
      }

    public:
      // Serialization
      template <class Archive>
      void serialize(Archive &ar)
      {
        ar(CEREAL_NVP(TIME_OFFSET_BiToBi0), CEREAL_NVP(TIME_OFFSET_RjToBi0));
      }
    } TEMPORAL;

  public:
    // the constructor
    explicit CalibParamManager();

    // the creator
    static CalibParamManager::Ptr Create();

    // save the parameters to file using cereal library
    template <class CerealArchiveType = CerealArchiveType::YAML>
    void Save(const std::string &filename) const
    {
      std::ofstream file(filename, std::ios::out);
      auto ar = GetOutputArchive<CerealArchiveType>(file);

      (*ar)(cereal::make_nvp("CalibParam", *this));
    }

    // load the parameters from file using cereal library
    template <class CerealArchiveType = CerealArchiveType::YAML>
    static CalibParamManager::Ptr Load(const std::string &filename)
    {
      auto calibParamManager = CalibParamManager::Create();
      std::ifstream file(filename, std::ios::in);
      auto ar = GetInputArchive<CerealArchiveType>(file);

      (*ar)(cereal::make_nvp("CalibParam", *calibParamManager));
      return calibParamManager;
    }

    // save the parameters to file using cereal library
    void Save(const std::string &filename, CerealArchiveType::Enum archiveType) const;

    // load the parameters from file using cereal library
    static CalibParamManager::Ptr Load(const std::string &filename, CerealArchiveType::Enum archiveType);

    // print the parameters in the console
    void ShowParamStatus();

    void VisualizationSensors(ns_viewer::Viewer &viewer) const;

    [[nodiscard]] CalibParamManager::Ptr
    AlignParamToNewSensor(const Sophus::SE3d &SE3_BcToNew, double TF_BcToNew) const;

    [[nodiscard]] CalibParamManager::Ptr AlignParamToNewSensor(const std::string &topic) const;

  public:
    // Serialization
    template <class Archive>
    void serialize(Archive &archive)
    {
      archive(CEREAL_NVP(EXTRI), CEREAL_NVP(TEMPORAL));
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ns_river2

#endif // RIS_CALIB_CALIB_PARAM_MANAGER_H
