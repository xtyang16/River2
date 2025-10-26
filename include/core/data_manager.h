/**
 * @file data_manager.h
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

#ifndef RIVER2_DATA_MANAGER_H
#define RIVER2_DATA_MANAGER_H

#include "config/configor.h"
#include "core/status.h"
#include "ros/ros.h"
#include "sensor/imu_data_loader.h"
#include "sensor/radar_data_loader.h"
#include "sensor_msgs/Imu.h"

namespace ns_river2
{
  using MapStringSubscriber = std::map<std::string, ros::Subscriber>;
  using MapStringListIMUFrameptr = std::map<std::string, std::list<IMUFrame::Ptr>>;
  using MapStringListRadarTargetptr = std::map<std::string, std::list<RadarTarget::Ptr>>;
  using MapStringListRadarTargetArrayptr = std::map<std::string, std::list<RadarTargetArray::Ptr>>;
  using MapStringMutex = std::map<std::string, std::mutex>;

  class DataManager
  {
  public:
    using Ptr = std::shared_ptr<DataManager>;

  private:
    // ros node handler used for creating subscribers, i.e., 'imuSuber' and 'radarSuber'
    ros::NodeHandlePtr handler;
    Configor::Ptr configor;

    // ros subscribers to receive imu and radar data
    MapStringSubscriber imuSubers;
    MapStringSubscriber radarSubers;

    // containers to store sensor data
    MapStringListIMUFrameptr imuDataSeqs;
    MapStringListRadarTargetptr radarDataSeqs;

    // the radar target array for initialization, size: 10
    MapStringListRadarTargetArrayptr radarTarAryForInits;

    std::optional<double> RIVER2_TIME_EPOCH;

  public:
    // mutexes employed in multi-thread framework
    MapStringMutex IMUDataSeqMutexs;
    MapStringMutex RadarDataSeqMutexs;
    MapStringMutex RadarInitFramesMutexs;

  public:
    explicit DataManager(const ros::NodeHandlePtr &handler, const Configor::Ptr &configor);

    static Ptr Create(const ros::NodeHandlePtr &handler, const Configor::Ptr &configor);

    [[nodiscard]] const std::optional<double> &GetRiverTimeEpoch() const;

    [[nodiscard]] double GetEndTimeSafely();

    [[nodiscard]] double GetIMUEndTimeSafely();

    [[nodiscard]] double GetRadarEndTimeSafely();

    const MapStringListIMUFrameptr &GetIMUDataSeq() const;

    const MapStringListRadarTargetptr &GetRadarDataSeq() const;

    void ShowDataStatus();

    MapStringListRadarTargetArrayptr GetRadarTarAryForInitSafely();

    MapStringListIMUFrameptr ExtractIMUDataPieceSafely(double start, double end);

    MapStringListIMUFrameptr ExtractIMUDataPieceSafely(double start);

    MapStringListRadarTargetArrayptr ExtractRadarDataPieceSafely(double start, double end);

    MapStringListRadarTargetArrayptr ExtractRadarDataPieceSafely(double start);

    static std::pair<Eigen::Vector3d, Eigen::Matrix3d> AcceMeanVar(const std::list<IMUFrame::Ptr> &data);

    static std::pair<Eigen::Vector3d, Eigen::Matrix3d> GyroMeanVar(const std::list<IMUFrame::Ptr> &data);

    void EraseOldDataPieceSafely(double time);

  protected:
    template <class IMUMsgType>
    void HandleIMUMessage(const typename IMUMsgType::ConstPtr &msg, const std::string &topic)
    {
      auto frame = IMUDataUnpacker::Unpack(msg);

      // try to initialize the time epoch of River
      if (RIVER2_TIME_EPOCH == std::nullopt)
      {
        RIVER2_TIME_EPOCH = frame->GetTimestamp();
        spdlog::info("time epoch of River initialized by imu topic: {}, imu frame: {:.6f}", topic, *RIVER2_TIME_EPOCH);
      }
      frame->SetTimestamp(
          frame->GetTimestamp() - *RIVER2_TIME_EPOCH + configor->dataStream.CalibParams.TEMPORAL.TIME_OFFSET_BiToBi0[topic]);
      {
        std::unique_lock<std::mutex> imuDataLock(IMUDataSeqMutexs[topic]);
        imuDataSeqs[topic].push_back(frame);
      }
    }

    template <class RadarMsgType>
    void HandleRadarMessage(const typename RadarMsgType::ConstPtr &msg, const std::string &topic)
    {
      auto targetAry = RadarDataUnpacker::Unpack(msg);
      // try to initialize the time epoch of River
      if (RIVER2_TIME_EPOCH == std::nullopt)
      {
        RIVER2_TIME_EPOCH = targetAry->GetTimestamp();
        spdlog::info("time epoch of River initialized by radar topic: {}, radar targetAry: {:.6f}", topic, *RIVER2_TIME_EPOCH);
      }
      // aligned the time of targetAry (i.e., radar time) to IMU time
      targetAry->SetTimestamp(
          targetAry->GetTimestamp() - *RIVER2_TIME_EPOCH + configor->dataStream.CalibParams.TEMPORAL.TIME_OFFSET_RjToBi0[topic]);
      for (auto &tar : targetAry->GetTargets())
      {
        tar->SetTimestamp(
            tar->GetTimestamp() - *RIVER2_TIME_EPOCH + configor->dataStream.CalibParams.TEMPORAL.TIME_OFFSET_RjToBi0[topic]);
      }
      {
        std::unique_lock<std::mutex> radarDataLock(RadarDataSeqMutexs[topic]);
        radarDataSeqs[topic].insert(
            radarDataSeqs[topic].end(), targetAry->GetTargets().cbegin(), targetAry->GetTargets().cend());
      }
      auto status = RiverStatus::GetStatusPackSafely();
      if (!RiverStatus::IsWith(RiverStatus::StateManager::Status::HasInitialized, status.StateMagr))
      {
        // spdlog::info("prepare once.");
        OrganizeRadarTarAryForInit(targetAry, topic);
      }
      else
      {
        {
          std::unique_lock<std::mutex> radarInitFramesLock(RadarInitFramesMutexs[topic]);
          radarTarAryForInits[topic].clear();
        }
        if (RiverStatus::IsWith(RiverStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr))
        {
          LOCK_RIVER2_STATUS
          RiverStatus::DataManager::CurStatus ^= RiverStatus::DataManager::Status::RadarTarAryForInitIsReady;
        }
      }
    }

    inline void OrganizeRadarTarAryForInit(const RadarTargetArray::Ptr &rawTarAry, const std::string &topic);
  };

}

#endif // RIVER2_DATA_MANAGER_H
