/**
 * @file data_manager.cpp
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

#include "core/data_manager.h"

namespace ns_river2
{
  /**
   * @brief 创建一个数据处理的工具（判断输入数据类型，配置对应回调函数）
   *
   * @param handler ROS句柄
   * @param configor 配置文件
   */
  DataManager::DataManager(const ros::NodeHandlePtr &handler, const ns_river2::Configor::Ptr &configor)
      : handler(handler), configor(configor), RIVER2_TIME_EPOCH(std::optional<double>())
  {
    spdlog::info("'DataManager' has been booted, thread id: {}.", RIVER2_TO_STR(std::this_thread::get_id()));
    // -------------------------------------------------------
    // create imu message subscriber based on the message type
    // -------------------------------------------------------
    IMUMsgType imuMsgType;
    for (const auto &[IMUTopic, IMUModel] : configor->dataStream.IMUTopics)
    {
      try
      {
        imuMsgType = EnumCast::stringToEnum<IMUMsgType>(IMUModel);
        IMUDataSeqMutexs[IMUTopic];
      }
      catch (...)
      {
        throw Status(
            Status::Flag::WARNING,
            fmt::format(
                "Unsupported IMU Type: '{}'. "
                "Currently supported IMU types are: \n"
                "(1) SENSOR_IMU: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html\n"
                "(2)    SBG_IMU: https://github.com/SBG-Systems/sbg_ros_driver.git\n"
                "...\n"
                "If you need to use other IMU types, "
                "please 'Issues' us on the profile of the github repository.",
                IMUModel));
      }
      /**
       * create subscriber of imu data
       * @topic configor->dataStream.IMUTopic
       * @queue_size configor->preference.IMUMsgQueueSize
       * @callback HandleIMUMessage(...)
       */
      switch (imuMsgType)
      {
      case IMUMsgType::SENSOR_IMU:
        imuSubers[IMUTopic] = handler->subscribe<sensor_msgs::Imu>(
            IMUTopic, configor->preference.IMUMsgQueueSizes[IMUTopic],
            [this, topic = IMUTopic](auto &&PH1)
            {
              HandleIMUMessage<sensor_msgs::Imu>(std::forward<decltype(PH1)>(PH1), topic);
            });
        break;
      case IMUMsgType::SBG_IMU:
        imuSubers[IMUTopic] = handler->subscribe<sbg_driver::SbgImuData>(
            IMUTopic, configor->preference.IMUMsgQueueSizes[IMUTopic],
            [this, topic = IMUTopic](auto &&PH1)
            {
              HandleIMUMessage<sbg_driver::SbgImuData>(std::forward<decltype(PH1)>(PH1), topic);
            });
        break;
      }
    }
    // ---------------------------------------------------------
    // create radar message subscriber based on the message type
    // ---------------------------------------------------------
    RadarMsgType radarMsgType;
    for (const auto &[RadarTopic, RadarModel] : configor->dataStream.RadarTopics)
    {
      try
      {
        radarMsgType = EnumCast::stringToEnum<RadarMsgType>(RadarModel);
        RadarDataSeqMutexs[RadarTopic];
        RadarInitFramesMutexs[RadarTopic];
      }
      catch (...)
      {
        throw Status(
            Status::Flag::WARNING,
            fmt::format(
                "Unsupported Radar Type: '{}'. "
                "Currently supported radar types are: \n"
                "(1)    AWR1843BOOST_RAW: https://github.com/Unsigned-Long/ti_mmwave_rospkg.git\n"
                "(2) AWR1843BOOST_CUSTOM: https://github.com/Unsigned-Long/ti_mmwave_rospkg.git\n"
                "(3)    POINTCLOUD2_POSV: 'sensor_msgs/PointCloud2' with point format: [x, y, z, velocity]\n"
                "(4)   POINTCLOUD2_POSIV: 'sensor_msgs/PointCloud2' with point format: [x, y, z, intensity, velocity]\n"
                "...\n"
                "If you need to use other radar types, "
                "please 'Issues' us on the profile of the github repository.",
                RadarModel));
      }
      /**
       * create subscriber of radar data
       * @topic configor->dataStream.RadarTopic
       * @queue_size configor->preference.RadarMsgQueueSize
       * @callback HandleRadarMessage(...)
       */
      switch (radarMsgType)
      {
      case RadarMsgType::AWR1843BOOST_RAW:
        radarSubers[RadarTopic] = handler->subscribe<ti_mmwave_rospkg::RadarScan>(
            RadarTopic, configor->preference.RadarMsgQueueSizes[RadarTopic],
            [this, topic = RadarTopic](auto &&PH1)
            {
              HandleRadarMessage<ti_mmwave_rospkg::RadarScan>(std::forward<decltype(PH1)>(PH1), topic);
            });
        break;
      case RadarMsgType::AWR1843BOOST_CUSTOM:
        radarSubers[RadarTopic] = handler->subscribe<ti_mmwave_rospkg::RadarScanCustom>(
            RadarTopic, configor->preference.RadarMsgQueueSizes[RadarTopic],
            [this, topic = RadarTopic](auto &&PH1)
            {
              HandleRadarMessage<ti_mmwave_rospkg::RadarScanCustom>(std::forward<decltype(PH1)>(PH1), topic);
            });
        break;
      case RadarMsgType::POINTCLOUD2_XRIO:
        radarSubers[RadarTopic] = handler->subscribe<sensor_msgs::PointCloud2>(
            RadarTopic, configor->preference.RadarMsgQueueSizes[RadarTopic],
            [this, topic = RadarTopic](auto &&PH1)
            {
              HandleRadarMessage<sensor_msgs::PointCloud2>(std::forward<decltype(PH1)>(PH1), topic);
            });
        break;
      }
    }
    spdlog::info("'DataManager' has been finished.");
  }

  DataManager::Ptr DataManager::Create(const ros::NodeHandlePtr &handler, const Configor::Ptr &configor)
  {
    return std::make_shared<DataManager>(handler, configor);
  }

  /**
   * @brief 展示当前数据容器内数据量，开始和结束时间
   *
   */
  void DataManager::ShowDataStatus()
  {
    std::size_t s = 0;
    double st = 0.0, et = 0.0;
    for (const auto &[IMUTopic, imuDataSeq] : imuDataSeqs)
    {
      s = 0, st = 0.0, et = 0.0;
      // lock imu data and obtain its size, start time, and end time quickly
      std::unique_lock<std::mutex> imuDataLock(IMUDataSeqMutexs[IMUTopic]);
      if (!imuDataSeq.empty())
      {
        s = imuDataSeq.size();
        st = imuDataSeq.front()->GetTimestamp();
        et = imuDataSeq.back()->GetTimestamp();
      }
      spdlog::info("imu topic: {}, imu data size: {:02}, time span from '{:.6f}' to '{:.6f}'", IMUTopic, s, st, et);
    }

    for (const auto &[RadarTopic, radarDataSeq] : radarDataSeqs)
    {
      s = 0, st = 0.0, et = 0.0;
      // lock radar data and obtain its size, start time, and end time quickly
      std::unique_lock<std::mutex> radarDataLock(RadarDataSeqMutexs[RadarTopic]);
      if (!radarDataSeq.empty())
      {
        s = radarDataSeq.size();
        st = radarDataSeq.front()->GetTimestamp();
        et = radarDataSeq.back()->GetTimestamp();
      }
      spdlog::info("radar topic: {}, radar data size: {:02}, time span from '{:.6f}' to '{:.6f}'", RadarTopic, s, st, et);
    }
  }

  /**
   * @brief 组织Radar数据用以进行初始化
   *
   * @param rawTarAry 得到的新Radar数据
   */
  void DataManager::OrganizeRadarTarAryForInit(const RadarTargetArray::Ptr &rawTarAry, const std::string &topic)
  {
    // if this system has not been initialization, construct radar rawTarAry arrays
    static std::map<std::string, std::vector<RadarTarget::Ptr>> tarAryMap;
    // spdlog::info("tar size: {:02}", radarTarAryForInits[topic].size());

    tarAryMap[topic].insert(tarAryMap[topic].end(), rawTarAry->GetTargets().cbegin(), rawTarAry->GetTargets().cend());

    static double scanHeadTime = tarAryMap[topic].front()->GetTimestamp();

    if (tarAryMap[topic].back()->GetTimestamp() - scanHeadTime < 0.1)
    {
      return;
    }

    // if time span is larger than 0.1 (s), try to organize these targets as a radar frame
    if (tarAryMap[topic].size() < 3)
    {
      // this radar frame is invalid, clear current status
      tarAryMap[topic].clear();

      std::unique_lock<std::mutex> radarInitFramesLock(RadarInitFramesMutexs[topic]);
      radarTarAryForInits[topic].clear();
      return;
    }

    // this radar frame is valid, compute average time
    double avgTime = 0.0;
    for (const auto &item : tarAryMap[topic])
    {
      avgTime += item->GetTimestamp();
    }
    avgTime /= static_cast<double>(tarAryMap[topic].size());

    {
      std::unique_lock<std::mutex> radarInitFramesLock(RadarInitFramesMutexs[topic]);
      radarTarAryForInits[topic].push_back(RadarTargetArray::Create(avgTime, tarAryMap[topic]));

      if (radarTarAryForInits[topic].size() > 10)
      {
        // keep data that lasting for 1.0 (s), i.e., ten frames, each lasting 0.1 (s)
        radarTarAryForInits[topic].pop_front();

        LOCK_RIVER2_STATUS
        RiverStatus::DataManager::CurStatus |= RiverStatus::DataManager::Status::RadarTarAryForInitIsReady;
      }
    }

    // clear current status
    scanHeadTime = tarAryMap[topic].back()->GetTimestamp();
    tarAryMap[topic].clear();
  }

  /**
   * @brief 获取用于初始化的Radar数据
   *
   * @return std::list<RadarTargetArray::Ptr>
   */
  MapStringListRadarTargetArrayptr DataManager::GetRadarTarAryForInitSafely()
  {
    return radarTarAryForInits;
  }

  /**
   * @brief 取出大于开始时间，大于结束时间的IMU数据段
   *
   * @param start
   * @param end
   * @return std::list<IMUFrame::Ptr>
   */
  MapStringListIMUFrameptr DataManager::ExtractIMUDataPieceSafely(double start, double end)
  {
    MapStringListIMUFrameptr dataSeqs;
    for (const auto &[IMUTopic, imuDataSeq] : imuDataSeqs)
    {
      std::list<IMUFrame::Ptr> dataSeq;
      std::unique_lock<std::mutex> imuDataLock(IMUDataSeqMutexs[IMUTopic]);
      auto sIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [start](const IMUFrame::Ptr &frame)
                                { return frame->GetTimestamp() < start; })
                       .base();
      auto eIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [end](const IMUFrame::Ptr &frame)
                                { return frame->GetTimestamp() < end; })
                       .base();
      std::copy(sIter, eIter, std::back_inserter(dataSeq));
      dataSeqs[IMUTopic] = dataSeq;
    }
    return dataSeqs;
  }

  MapStringListIMUFrameptr DataManager::ExtractIMUDataPieceSafely(double start)
  {
    MapStringListIMUFrameptr dataSeqs;
    for (const auto &[IMUTopic, imuDataSeq] : imuDataSeqs)
    {
      std::list<IMUFrame::Ptr> dataSeq;
      std::unique_lock<std::mutex> imuDataLock(IMUDataSeqMutexs[IMUTopic]);
      auto sIter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [start](const IMUFrame::Ptr &frame)
                                { return frame->GetTimestamp() < start; })
                       .base();
      std::copy(sIter, imuDataSeq.end(), std::back_inserter(dataSeq));
      dataSeqs[IMUTopic] = dataSeq;
    }
    return dataSeqs;
  }

  MapStringListRadarTargetArrayptr DataManager::ExtractRadarDataPieceSafely(double start, double end)
  {
    MapStringListRadarTargetArrayptr tarSeqs;
    for (const auto &[RadarTopic, radarDataSeq] : radarDataSeqs)
    {
      double startTemp = start;
      while (end - startTemp > 0.15)
      {
        std::vector<RadarTarget::Ptr> tarSeq;
        std::unique_lock<std::mutex> radarDataLock(RadarDataSeqMutexs[RadarTopic]);
        auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [startTemp](const RadarTarget::Ptr &tar)
                                  { return tar->GetTimestamp() < startTemp; })
                         .base();
        startTemp += 0.1;
        auto eIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [startTemp](const RadarTarget::Ptr &tar)
                                  { return tar->GetTimestamp() < startTemp; })
                         .base();
        std::copy(sIter, eIter, std::back_inserter(tarSeq));
        double avgTime = 0.0;
        for (const auto &item : tarSeq)
        {
          avgTime += item->GetTimestamp();
        }
        avgTime /= static_cast<double>(tarSeq.size());
        tarSeqs[RadarTopic].push_back(RadarTargetArray::Create(avgTime, tarSeq));
      }
      std::vector<RadarTarget::Ptr> tarSeq;
      std::unique_lock<std::mutex> radarDataLock(RadarDataSeqMutexs[RadarTopic]);
      auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [startTemp](const RadarTarget::Ptr &tar)
                                { return tar->GetTimestamp() < startTemp; })
                       .base();
      auto eIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [end](const RadarTarget::Ptr &tar)
                                { return tar->GetTimestamp() < end; })
                       .base();
      std::copy(sIter, eIter, std::back_inserter(tarSeq));
      double avgTime = 0.0;
      for (const auto &item : tarSeq)
      {
        avgTime += item->GetTimestamp();
      }
      avgTime /= static_cast<double>(tarSeq.size());
      tarSeqs[RadarTopic].push_back(RadarTargetArray::Create(avgTime, tarSeq));
    }
    return tarSeqs;
  }

  MapStringListRadarTargetArrayptr DataManager::ExtractRadarDataPieceSafely(double start)
  {
    MapStringListRadarTargetArrayptr tarSeqs;
    double end = GetRadarEndTimeSafely();
    for (const auto &[RadarTopic, radarDataSeq] : radarDataSeqs)
    {
      double startTemp = start;
      while (end - startTemp > 0.15)
      {
        std::vector<RadarTarget::Ptr> tarSeq;
        std::unique_lock<std::mutex> radarDataLock(RadarDataSeqMutexs[RadarTopic]);
        auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [startTemp](const RadarTarget::Ptr &tar)
                                  { return tar->GetTimestamp() < startTemp; })
                         .base();
        startTemp += 0.1;
        auto eIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [startTemp](const RadarTarget::Ptr &tar)
                                  { return tar->GetTimestamp() < startTemp; })
                         .base();
        std::copy(sIter, eIter, std::back_inserter(tarSeq));
        double avgTime = 0.0;
        for (const auto &item : tarSeq)
        {
          avgTime += item->GetTimestamp();
        }
        avgTime /= static_cast<double>(tarSeq.size());
        tarSeqs[RadarTopic].push_back(RadarTargetArray::Create(avgTime, tarSeq));
      }
      std::vector<RadarTarget::Ptr> tarSeq;
      std::unique_lock<std::mutex> radarDataLock(RadarDataSeqMutexs[RadarTopic]);
      auto sIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [startTemp](const RadarTarget::Ptr &tar)
                                { return tar->GetTimestamp() < startTemp; })
                       .base();
      auto eIter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [end](const RadarTarget::Ptr &tar)
                                { return tar->GetTimestamp() < end; })
                       .base();
      std::copy(sIter, eIter, std::back_inserter(tarSeq));
      double avgTime = 0.0;
      for (const auto &item : tarSeq)
      {
        avgTime += item->GetTimestamp();
      }
      avgTime /= static_cast<double>(tarSeq.size());
      tarSeqs[RadarTopic].push_back(RadarTargetArray::Create(avgTime, tarSeq));
    }
    return tarSeqs;
  }

  double DataManager::GetEndTimeSafely()
  {
    double m1(0.0), m2(0.0);
    for (const auto &[IMUTopic, imuDataSeq] : imuDataSeqs)
    {
      std::unique_lock<std::mutex> imuDataLock(IMUDataSeqMutexs[IMUTopic]);
      if (imuDataSeq.size() == 0)
        continue;
      m1 = std::max(m1, imuDataSeq.back()->GetTimestamp());
    }
    for (const auto &[RadarTopic, radarDataSeq] : radarDataSeqs)
    {
      std::unique_lock<std::mutex> radarDataLock(RadarDataSeqMutexs[RadarTopic]);
      if (radarDataSeq.size() == 0)
        continue;
      m2 = std::max(m2, radarDataSeq.back()->GetTimestamp());
    }
    return std::min(m1, m2);
  }

  const std::optional<double> &DataManager::GetRiverTimeEpoch() const
  {
    return RIVER2_TIME_EPOCH;
  }

  double DataManager::GetIMUEndTimeSafely()
  {
    double m1(0.0);
    for (const auto &[IMUTopic, imuDataSeq] : imuDataSeqs)
    {
      std::unique_lock<std::mutex> imuDataLock(IMUDataSeqMutexs[IMUTopic]);
      m1 = std::max(m1, imuDataSeq.back()->GetTimestamp());
    }
    return m1;
  }

  double DataManager::GetRadarEndTimeSafely()
  {
    double m2(0.0);
    for (const auto &[RadarTopic, radarDataSeq] : radarDataSeqs)
    {
      std::unique_lock<std::mutex> radarDataLock(RadarDataSeqMutexs[RadarTopic]);
      m2 = std::max(m2, radarDataSeq.back()->GetTimestamp());
    }
    return m2;
  }

  /**
   * @brief 从原始IMU数据中获取加速度计数据均值与方差
   *
   * @param data
   * @return std::pair<Eigen::Vector3d, Eigen::Matrix3d>
   */
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> DataManager::AcceMeanVar(const std::list<IMUFrame::Ptr> &data)
  {
    if (data.empty())
    {
      return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
    }

    Eigen::MatrixXd matrix(data.size(), 3);
    int i = 0;
    for (const auto &v : data)
    {
      matrix.row(i++) = v->GetAcce();
    }

    Eigen::Vector3d mean = matrix.colwise().mean();
    Eigen::Matrix3d var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                           (matrix.rowwise() - matrix.colwise().mean())) /
                          static_cast<double>(data.size() - 1);

    return {mean, var};
  }

  /**
   * @brief 从原始IMU数据中获取陀螺仪均值与方差
   *
   * @param data
   * @return std::pair<Eigen::Vector3d, Eigen::Matrix3d>
   */
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> DataManager::GyroMeanVar(const std::list<IMUFrame::Ptr> &data)
  {
    if (data.empty())
    {
      return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
    }

    Eigen::MatrixXd matrix(data.size(), 3);
    int i = 0;
    for (const auto &v : data)
    {
      matrix.row(i++) = v->GetGyro();
    }

    Eigen::Vector3d mean = matrix.colwise().mean();
    Eigen::Matrix3d var = ((matrix.rowwise() - matrix.colwise().mean()).transpose() *
                           (matrix.rowwise() - matrix.colwise().mean())) /
                          static_cast<double>(data.size() - 1);

    return {mean, var};
  }

  /**
   * @brief 删除不需要的数据
   *
   * @param time 标记时间
   */
  void DataManager::EraseOldDataPieceSafely(double time)
  {
    for (auto &[IMUTopic, imuDataSeq] : imuDataSeqs)
    {
      std::unique_lock<std::mutex> imuDataLock(IMUDataSeqMutexs[IMUTopic]);
      auto iter = std::find_if(imuDataSeq.rbegin(), imuDataSeq.rend(), [time](const IMUFrame::Ptr &frame)
                               { return frame->GetTimestamp() < time; })
                      .base();
      imuDataSeq.erase(imuDataSeq.begin(), iter);
    }
    for (auto &[RadarTopic, radarDataSeq] : radarDataSeqs)
    {
      std::unique_lock<std::mutex> radarDataLock(RadarDataSeqMutexs[RadarTopic]);
      auto iter = std::find_if(radarDataSeq.rbegin(), radarDataSeq.rend(), [time](const RadarTarget::Ptr &tar)
                               { return tar->GetTimestamp() < time; })
                      .base();
      radarDataSeq.erase(radarDataSeq.begin(), iter);
    }
  }

  const MapStringListIMUFrameptr &DataManager::GetIMUDataSeq() const
  {
    return imuDataSeqs;
  }

  const MapStringListRadarTargetptr &DataManager::GetRadarDataSeq() const
  {
    return radarDataSeqs;
  }

} // namespace ns_river2
