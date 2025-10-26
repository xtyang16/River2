/**
 * @file configor.cpp
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

#include "config/configor.h"

namespace ns_river2
{
  // -------------------
  // static member field
  // -------------------
  std::string Configor::Preference::SO3Spline = "so3";
  std::string Configor::Preference::VelSpline = "vel";
  std::string Configor::Preference::BaSpline = "ba";
  std::string Configor::Preference::BgSpline = "bg";

  std::string Configor::Preference::PublishTopic = "/river/state";
  double Configor::Preference::StatePublishDelay = 0.05;

  bool Configor::Preference::DEBUG_MODE = false;

  Configor::Configor() = default;

  Configor::~Configor() = default;

  Configor::Ptr Configor::Create()
  {
    return std::make_shared<Configor>();
  }

  Configor::Ptr Configor::Load(const std::string &filename, CerealArchiveType::Enum archiveType)
  {
    auto configor = Configor::Create();
    std::ifstream file(filename, std::ios::in);
    auto ar = GetInputArchiveVariant(file, archiveType);
    SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("Configor", *configor));
    return configor;
  }

  void Configor::Save(const std::string &filename, CerealArchiveType::Enum archiveType)
  {
    std::ofstream file(filename, std::ios::out);
    auto ar = GetOutputArchiveVariant(file, archiveType);
    SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("Configor", *this));
  }

  void Configor::PrintMainFields()
  {
    std::stringstream streamIMUTopics, streamIMUModels, streamRadarTopics, streamRadarModels;
    for (const auto &[imuTopic, imuModel] : dataStream.IMUTopics)
    {
      streamIMUTopics << imuTopic << ' ';
      streamIMUModels << imuTopic << ": " << imuModel << ' ';
    }
    for (const auto &[radarTopic, radarModel] : dataStream.RadarTopics)
    {
      streamRadarTopics << radarTopic << ' ';
      streamRadarModels << radarTopic << ": " << radarModel << ' ';
    }
    std::string IMUTopics = streamIMUTopics.str();
    std::string IMUModels = streamIMUModels.str();
    std::string RadarTopics = streamRadarTopics.str();
    std::string RadarModels = streamRadarModels.str();

    std::stringstream streamIMUMsgQueueSizes, streamRadarMsgQueueSizes;
    {
      for (const auto &[imuTopic, IMUMsgQueueSize] : preference.IMUMsgQueueSizes)
      {
        streamIMUMsgQueueSizes << imuTopic << ": " << IMUMsgQueueSize << ' ';
      }
      for (const auto &[radarTopic, RadarMsgQueueSize] : preference.RadarMsgQueueSizes)
      {
        streamRadarMsgQueueSizes << radarTopic << ": " << RadarMsgQueueSize << ' ';
      }
    }
    std::string IMUMsgQueueSizes = streamIMUMsgQueueSizes.str();
    std::string RadarMsgQueueSizes = streamRadarMsgQueueSizes.str();

    std::stringstream streamAcceWeights, streamAcceBiasRandomWalks, streamGyroWeights,
        streamGyroBiasRandomWalks, streamRadarWeights, streamCauchyLossForRadarFactors;
    {
      for (const auto &[imuTopic, AcceWeight] : prior.AcceWeights)
      {
        streamAcceWeights << imuTopic << ": " << AcceWeight << ' ';
      }
      for (const auto &[imuTopic, AcceBiasRandomWalk] : prior.AcceBiasRandomWalks)
      {
        streamAcceBiasRandomWalks << imuTopic << ": " << AcceBiasRandomWalk << ' ';
      }
      for (const auto &[imuTopic, GyroWeight] : prior.GyroWeights)
      {
        streamGyroWeights << imuTopic << ": " << GyroWeight << ' ';
      }
      for (const auto &[imuTopic, GyroBiasRandomWalk] : prior.GyroBiasRandomWalks)
      {
        streamGyroBiasRandomWalks << imuTopic << ": " << GyroBiasRandomWalk << ' ';
      }
      for (const auto &[radarTopic, RadarWeight] : prior.RadarWeights)
      {
        streamRadarWeights << radarTopic << ": " << RadarWeight << ' ';
      }
      for (const auto &[radarTopic, CauchyLossForRadarFactor] : prior.CauchyLossForRadarFactors)
      {
        streamCauchyLossForRadarFactors << radarTopic << ": " << CauchyLossForRadarFactor << ' ';
      }
    }
    std::string AcceWeights = streamAcceWeights.str();
    std::string AcceBiasRandomWalks = streamAcceBiasRandomWalks.str();
    std::string GyroWeights = streamGyroWeights.str();
    std::string GyroBiasRandomWalks = streamGyroBiasRandomWalks.str();
    std::string RadarWeights = streamRadarWeights.str();
    std::string CauchyLossForRadarFactors = streamCauchyLossForRadarFactors.str();

    std::stringstream streamRadarTargetThresholdErrors, streamRadarTargetThresholdRatios, streamRadarTargetRANSACs,
        streamRadarTargetRAVSACPoints, streamRadarTargetOutliersProbs, streamRadarTargetSuccessProbs;
    {
      for (const auto &[radarTopic, RadarTargetRANSAC] : prior.RadarTargetRANSACs)
      {
        streamRadarTargetRANSACs << radarTopic << ": " << RadarTargetRANSAC << ' ';
      }
      for (const auto &[radarTopic, RadarTargetThresholdError] : prior.RadarTargetThresholdErrors)
      {
        streamRadarTargetThresholdErrors << radarTopic << ": " << RadarTargetThresholdError << ' ';
      }
      for (const auto &[radarTopic, RadarTargetThresholdRatio] : prior.RadarTargetThresholdRatios)
      {
        streamRadarTargetThresholdRatios << radarTopic << ": " << RadarTargetThresholdRatio << ' ';
      }
      for (const auto &[radarTopic, RadarTargetRAVSACPoint] : prior.RadarTargetRAVSACPoints)
      {
        streamRadarTargetRAVSACPoints << radarTopic << ": " << RadarTargetRAVSACPoint << ' ';
      }
      for (const auto &[radarTopic, RadarTargetOutliersProb] : prior.RadarTargetOutliersProbs)
      {
        streamRadarTargetOutliersProbs << radarTopic << ": " << RadarTargetOutliersProb << ' ';
      }
      for (const auto &[radarTopic, RadarTargetSuccessProb] : prior.RadarTargetSuccessProbs)
      {
        streamRadarTargetSuccessProbs << radarTopic << ": " << RadarTargetSuccessProb << ' ';
      }
    }
    std::string RadarTargetRANSACs = streamRadarTargetRANSACs.str();
    std::string RadarTargetThresholdErrors = streamRadarTargetThresholdErrors.str();
    std::string RadarTargetThresholdRatios = streamRadarTargetThresholdRatios.str();
    std::string RadarTargetRAVSACPoints = streamRadarTargetRAVSACPoints.str();
    std::string RadarTargetOutliersProbs = streamRadarTargetOutliersProbs.str();
    std::string RadarTargetSuccessProbs = streamRadarTargetSuccessProbs.str();

    std::stringstream streamRadarZeroVelocityThresholds, streamAcceZeroVelocityThresholds, streamGyroZeroVelocityThresholds;
    {
      for (const auto &[radarTopic, RadarZeroVelocityThreshold] : prior.RadarZeroVelocityThresholds)
      {
        streamRadarZeroVelocityThresholds << radarTopic << ": " << RadarZeroVelocityThreshold << ' ';
      }
      for (const auto &[imuTopic, AcceZeroVelocityThreshold] : prior.AcceZeroVelocityThresholds)
      {
        streamAcceZeroVelocityThresholds << imuTopic << ": " << AcceZeroVelocityThreshold << ' ';
      }
      for (const auto &[imuTopic, GyroZeroVelocityThreshold] : prior.GyroZeroVelocityThresholds)
      {
        streamGyroZeroVelocityThresholds << imuTopic << ": " << GyroZeroVelocityThreshold << ' ';
      }
    }
    std::string RadarZeroVelocityThresholds = streamRadarZeroVelocityThresholds.str();
    std::string AcceZeroVelocityThresholds = streamAcceZeroVelocityThresholds.str();
    std::string GyroZeroVelocityThresholds = streamGyroZeroVelocityThresholds.str();

#define DESC_FIELD(field) #field, field
#define DESC_FORMAT "\n{:>35}: {}"
    spdlog::info(
        "main fields of configor:" DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
            DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
                DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
                    DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT,
        DESC_FIELD(IMUTopics),
        DESC_FIELD(IMUModels),
        DESC_FIELD(RadarTopics),
        DESC_FIELD(RadarModels),
        DESC_FIELD(dataStream.OutputPath),

        DESC_FIELD(prior.SplineOrder),
        DESC_FIELD(prior.GravityNorm),
        DESC_FIELD(prior.SO3SplineKnotDist),
        DESC_FIELD(prior.VelSplineKnotDist),
        DESC_FIELD(AcceWeights),
        DESC_FIELD(AcceBiasRandomWalks),
        DESC_FIELD(GyroWeights),
        DESC_FIELD(GyroBiasRandomWalks),
        DESC_FIELD(RadarWeights),
        DESC_FIELD(CauchyLossForRadarFactors),
        DESC_FIELD(RadarTargetRANSACs),
        DESC_FIELD(RadarTargetThresholdErrors),
        DESC_FIELD(RadarTargetThresholdRatios),
        DESC_FIELD(RadarTargetRAVSACPoints),
        DESC_FIELD(RadarTargetOutliersProbs),
        DESC_FIELD(RadarTargetSuccessProbs),
        DESC_FIELD(prior.ZeroVelocityUpdate),
        DESC_FIELD(prior.ZeroVelocityUpdateWeightVel),
        DESC_FIELD(prior.ZeroVelocityUpdateWeightRot),
        DESC_FIELD(RadarZeroVelocityThresholds),
        DESC_FIELD(AcceZeroVelocityThresholds),
        DESC_FIELD(GyroZeroVelocityThresholds),

        DESC_FIELD(IMUMsgQueueSizes),
        DESC_FIELD(RadarMsgQueueSizes),
        DESC_FIELD(preference.IncrementalOptRate),
        DESC_FIELD(preference.ObjFileForDisplay),
        DESC_FIELD(preference.OutputResultsWithTimeAligned),
        DESC_FIELD(preference.VisualizeVelocityInBody));

    dataStream.CalibParams.ShowParamStatus();

#undef DESC_FIELD
#undef DESC_FORMAT
  }
} // namespace ns_river2
