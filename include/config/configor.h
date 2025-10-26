/**
 * @file configor.h
 * @author Xiaoteng Yang (xtyang@whu.edu.cn)
 * @brief Xiaoteng Yang received the B.S. degree in 
 *        geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. 
 *        He is currently a master candidate at the school of Geodesy and Geomatics, Wuhan University.
 * @version 0.1
 * @date 2025-10-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef RIVER2_CONFIGOR_H
#define RIVER2_CONFIGOR_H

#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/set.hpp"
#include "cereal/types/vector.hpp"
#include "core/calib_param_manager.h"
#include "sensor/imu_data_loader.h"
#include "sensor/radar_data_loader.h"
#include "util/enum_cast.hpp"
#include "util/utils.hpp"

namespace ns_river2
{
  struct Configor
  {
  public:
    using Ptr = std::shared_ptr<Configor>;

  public:
    struct DataStream
    {
      // [IMU topic, model]
      std::map<std::string, std::string> IMUTopics;

      // [radar topic, model]
      std::map<std::string, std::string> RadarTopics;

      // CalibParamManager CalibParam;
      CalibParamManager CalibParams;

      std::string OutputPath;

    public:
      template <class Archive>
      void serialize(Archive &ar)
      {
        ar(CEREAL_NVP(IMUTopics),
           CEREAL_NVP(RadarTopics),
           CEREAL_NVP(CalibParams), CEREAL_NVP(OutputPath));
      }
    } dataStream;

    struct Prior
    {
      static constexpr int SplineOrder = 3;

      double GravityNorm;

      double SO3SplineKnotDist;
      double VelSplineKnotDist;

      std::map<std::string, double> AcceWeights;
      std::map<std::string, double> AcceBiasRandomWalks;
      std::map<std::string, double> GyroWeights;
      std::map<std::string, double> GyroBiasRandomWalks;
      std::map<std::string, double> RadarWeights;

      std::map<std::string, double> CauchyLossForRadarFactors;

      std::map<std::string, bool> RadarTargetRANSACs;
      std::map<std::string, double> RadarTargetThresholdErrors;
      std::map<std::string, double> RadarTargetThresholdRatios;
      std::map<std::string, int> RadarTargetRAVSACPoints;
      std::map<std::string, double> RadarTargetOutliersProbs;
      std::map<std::string, double> RadarTargetSuccessProbs;

      bool ZeroVelocityUpdate;
      double ZeroVelocityUpdateWeightVel;
      double ZeroVelocityUpdateWeightRot;
      std::map<std::string, double> RadarZeroVelocityThresholds;
      std::map<std::string, double> AcceZeroVelocityThresholds;
      std::map<std::string, double> GyroZeroVelocityThresholds;

    public:
      template <class Archive>
      void serialize(Archive &ar)
      {
        ar(
            CEREAL_NVP(GravityNorm),
            CEREAL_NVP(SO3SplineKnotDist), CEREAL_NVP(VelSplineKnotDist),
            CEREAL_NVP(AcceWeights), CEREAL_NVP(AcceBiasRandomWalks),
            CEREAL_NVP(GyroWeights), CEREAL_NVP(GyroBiasRandomWalks),
            CEREAL_NVP(RadarWeights),
            CEREAL_NVP(CauchyLossForRadarFactors),
            CEREAL_NVP(RadarTargetRANSACs),
            CEREAL_NVP(RadarTargetThresholdErrors),
            CEREAL_NVP(RadarTargetThresholdRatios),
            CEREAL_NVP(RadarTargetRAVSACPoints),
            CEREAL_NVP(RadarTargetOutliersProbs),
            CEREAL_NVP(RadarTargetSuccessProbs),
            CEREAL_NVP(ZeroVelocityUpdate),
            CEREAL_NVP(RadarZeroVelocityThresholds),
            CEREAL_NVP(AcceZeroVelocityThresholds),
            CEREAL_NVP(GyroZeroVelocityThresholds),
            CEREAL_NVP(ZeroVelocityUpdateWeightVel),
            CEREAL_NVP(ZeroVelocityUpdateWeightRot));
      }
    } prior{};

    struct Preference
    {
      /**
       * when the mode is 'DEBUG_MODE', then:
       * 1. the solving information from ceres would be output on the console
       * 2.
       */
      static bool DEBUG_MODE;

      static std::string SO3Spline;
      static std::string VelSpline;

      // these two splines are not used currently
      static std::string BaSpline;
      static std::string BgSpline;

      static std::string PublishTopic;

      static double StatePublishDelay;

      std::map<std::string, std::uint32_t> IMUMsgQueueSizes;
      std::map<std::string, std::uint32_t> RadarMsgQueueSizes;
      std::uint32_t IncrementalOptRate;

      std::string ObjFileForDisplay;

      bool OutputResultsWithTimeAligned;

      bool VisualizeVelocityInBody;

    public:
      template <class Archive>
      void serialize(Archive &ar)
      {
        ar(
            CEREAL_NVP(IMUMsgQueueSizes), CEREAL_NVP(RadarMsgQueueSizes),
            CEREAL_NVP(IncrementalOptRate), CEREAL_NVP(ObjFileForDisplay),
            CEREAL_NVP(OutputResultsWithTimeAligned), CEREAL_NVP(VisualizeVelocityInBody));
      }
    } preference{};

  public:
    Configor();

    ~Configor();

    static Ptr Create();

    // load configure information from the xml file
    static Configor::Ptr
    Load(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

    // load configure information from the xml file
    void Save(const std::string &filename, CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

    // print the main fields
    void PrintMainFields();

  public:
    template <class Archive>
    void serialize(Archive &ar)
    {
      ar(
          cereal::make_nvp("DataStream", dataStream),
          cereal::make_nvp("Prior", prior),
          cereal::make_nvp("Preference", preference));
    }
  };
} // namespace ns_rivers

#endif // RIVER2_CONFIGOR_H
