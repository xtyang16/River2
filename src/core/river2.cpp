/**
 * @file river2.cpp
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

#include "core/river2.h"
#include "cereal/types/utility.hpp"
#include "river2/RiverState.h"

namespace ns_river2
{

  River::River(const Configor::Ptr &configor)
      : handler(new ros::NodeHandle), configor(configor),
        dataMagr(DataManager::Create(handler, configor)),
        stateMagr(StateManager::Create(dataMagr, configor)),
        viewer(Viewer::Create(configor, stateMagr)),
        stateMagrThread(std::make_shared<std::thread>(&StateManager::Run, stateMagr)) ,
        viewerThread(std::make_shared<std::thread>(&Viewer::RunViewer, viewer))
  {
    /**
     * 1. Once the 'dataMagr' is created, two ros::Subscriber, i.e., 'imuSuber' and 'radarSuber' would
     *    start waiting receive ros message from imu and radar topics;
     * 2. The 'StateManager::Run' runs on another thread once 'stateMagrThread' is created;
     * 3. The 'Viewer::RunViewer' runs on another thread once 'viewerThread' is created;
     */
    statePublisher = this->handler->advertise<river2::RiverState>(Configor::Preference::PublishTopic, 100);
  }

  River::Ptr River::Create(const Configor::Ptr &configor)
  {
    return std::make_shared<River>(configor);
  }

  void River::Run()
  {
    spdlog::info("'River::Run' has been booted, thread id: {}.", RIVER2_TO_STR(std::this_thread::get_id()));

    // auto lastTimeShowDataStatus = ros::Time::now().toSec();
    ros::Rate rate(configor->preference.IncrementalOptRate);

    while (ros::ok())
    {
      auto status = RiverStatus::GetStatusPackSafely();

      /// 判断当前状态是否满足停止程序的条件
      if (RiverStatus::IsWith(RiverStatus::StateManager::Status::ShouldQuit, status.StateMagr))
      {
        spdlog::warn("'River::Run' quits normally.");
        break;
      }

      // -------------------
      // publish river state
      // -------------------
      PublishRiverState(status); /// 实现发布解算结果操作

      // --------------
      // erase old data
      // --------------
      if (RiverStatus::IsWith(RiverStatus::StateManager::Status::HasInitialized, status.StateMagr))
      {
        // if you want to evaluate the consistency by comparing inertial quantities, comment out this
        dataMagr->EraseOldDataPieceSafely(status.ValidStateEndTime);
      }

      rate.sleep();
      ros::spinOnce();
    }
    stateMagrThread->join(); /// 函数已经在执行，这里两行保证主程序会一直执行指导线程结束
    viewerThread->join();
  }

  void River::Save()
  {
    // const std::string &tarDir = configor->dataStream.OutputPath + "/river_output";
    const std::string &tarDir = configor->dataStream.OutputPath;
    if (!std::filesystem::exists(tarDir))
    {
      if (!std::filesystem::create_directories(tarDir))
      {
        throw Status(Status::Flag::WARNING, fmt::format(
                                                "the output path for data, i.e., '{}', dose not exist and create failed!", tarDir));
      }
    }
    else
    {
      // std::filesystem::remove_all(tarDir);
      // std::filesystem::create_directories(tarDir);
    }

    // splines
    auto &splines = this->stateMagr->GetSplines();
    auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
    auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
    const double epoch = configor->preference.OutputResultsWithTimeAligned ? 0.0 : *dataMagr->GetRiverTimeEpoch();
    const double velST = velSpline.MinTime(), velET = velSpline.MaxTime();
    const double so3ST = so3Spline.MinTime(), so3ET = so3Spline.MaxTime();

    velSpline.SetStartTime(velST + epoch);
    so3Spline.SetStartTime(so3ST + epoch);

    // splines->Save(tarDir + "/splines.json");

    velSpline.SetStartTime(velST);
    so3Spline.SetStartTime(so3ST);

    // velocity
    const double st = std::max(velST, so3ST), et = std::min(velET, so3ET);

    std::vector<std::pair<double, Eigen::Vector3d>> velocityInW, velocityInB;
    std::vector<std::pair<double, Sophus::SO3d>> quatBtoW;
    std::ofstream file_out(tarDir + "/rivers_out.txt", std::ios::out);
    for (double t = st; t < et;)
    {
      Eigen::Vector3d LIN_VEL_BtoWinW = velSpline.Evaluate(t);
      velocityInW.emplace_back(t + epoch, LIN_VEL_BtoWinW);

      auto SO3_CurToW = so3Spline.Evaluate(t);
      quatBtoW.emplace_back(t + epoch, SO3_CurToW);

      Eigen::Vector3d LIN_VEL_BtoWinB = SO3_CurToW.inverse() * LIN_VEL_BtoWinW;
      velocityInB.emplace_back(t + epoch, LIN_VEL_BtoWinB);

      file_out << std::fixed << std::setprecision(16) << //
          t + epoch << " " <<                            //
          SO3_CurToW.unit_quaternion().w() << " " <<     //
          SO3_CurToW.unit_quaternion().x() << " " <<     //
          SO3_CurToW.unit_quaternion().y() << " " <<     //
          SO3_CurToW.unit_quaternion().z() << " " <<     //
          LIN_VEL_BtoWinW(0) << " " <<                   //
          LIN_VEL_BtoWinW(1) << " " <<                   //
          LIN_VEL_BtoWinW(2) << "\n";

      t += 0.1;
    }
    file_out.close();

    {
      int tab(0);
      for (const auto &[imuTopic, _] : configor->dataStream.IMUTopics)
      {
        spdlog::info("IMU topic: {}", imuTopic);
        std::ofstream file_out_bias_a(tarDir + "/rivers_bias_a_" + std::to_string(tab) + ".txt", std::ios::out);
        std::ofstream file_out_bias_g(tarDir + "/rivers_bias_g_" + std::to_string(tab++) + ".txt", std::ios::out);
        auto acceRecords = stateMagr->GetBaFilter().at(imuTopic)->GetStateRecords();
        for (auto &item : acceRecords)
        {
          file_out_bias_a <<                         //
              std::fixed << std::setprecision(16) << //
              item.time + epoch << " " <<            //
              item.state(0) << " " <<                //
              item.state(1) << " " <<                //
              item.state(2) << "\n";
        }

        auto gyroRecords = stateMagr->GetBgFilter().at(imuTopic)->GetStateRecords();
        for (auto &item : gyroRecords)
        {
          file_out_bias_g <<                         //
              std::fixed << std::setprecision(16) << //
              item.time + epoch << " " <<            //
              item.state(0) << " " <<                //
              item.state(1) << " " <<                //
              item.state(2) << "\n";
        }
        file_out_bias_a.close();
        file_out_bias_g.close();
      }
    }

    {
      // std::ofstream file(tarDir + "/velocity.json", std::ios::out);
      // cereal::JSONOutputArchive ar(file);
      // ar(
      //     cereal::make_nvp("velocity_in_world", velocityInW),
      //     cereal::make_nvp("velocity_in_body", velocityInB));
    }

    // rotation
    {
      // std::ofstream file(tarDir + "/rotation.json", std::ios::out);
      // cereal::JSONOutputArchive ar(file);
      // ar(cereal::make_nvp("quat_body_to_world", quatBtoW));
    }

    // acce and gyro bias
    {
      // for (const auto &[imuTopic, _] : configor->dataStream.IMUTopics)
      // {
      //   std::ofstream file(tarDir + "/bias" + "_" + imuTopic + ".json", std::ios::out);
      //   cereal::JSONOutputArchive ar(file);

      //   auto acceRecords = stateMagr->GetBaFilter().at(imuTopic)->GetStateRecords();
      //   for (auto &item : acceRecords)
      //   {
      //     item.time += epoch;
      //   }

      //   auto gyroRecords = stateMagr->GetBgFilter().at(imuTopic)->GetStateRecords();
      //   for (auto &item : gyroRecords)
      //   {
      //     item.time += epoch;
      //   }

      //   ar(cereal::make_nvp("acce_bias", acceRecords), cereal::make_nvp("gyro_bias", gyroRecords));
      // }
    }

    spdlog::info("outputs of 'River' have been saved to dir '{}'.", tarDir);
  }

  void River::PublishRiverState(const RiverStatus::StatusPack &status)
  {
    /// 判断当前状态是否需要被发布
    if (!RiverStatus::IsWith(RiverStatus::StateManager::Status::NewStateNeedToPublish, status.StateMagr))
    {
      return;
    }

    std::optional<StateManager::StatePack> state = this->stateMagr->GetStatePackSafely(
        status.ValidStateEndTime - Configor::Preference::StatePublishDelay);
    if (state == std::nullopt)
    {
      return;
    }

    river2::RiverState riverState;
    riverState.header.stamp = ros::Time(state->timestamp);
    riverState.header.frame_id = "world";

    riverState.velocity_norm = static_cast<float>(state->LIN_VEL_Bi0ToRefInRef.norm());
    for (const auto &[imuTopic, _] : configor->dataStream.IMUTopics)
    {
      riverState.acce_bias_norm = static_cast<float>(state->bas.at(imuTopic).norm());
      riverState.gyro_bias_norm = static_cast<float>(state->bgs.at(imuTopic).norm());
      break;
    }

    riverState.velocity_in_world.x = state->LIN_VEL_Bi0ToRefInRef(0);
    riverState.velocity_in_world.y = state->LIN_VEL_Bi0ToRefInRef(1);
    riverState.velocity_in_world.z = state->LIN_VEL_Bi0ToRefInRef(2);

    Eigen::Vector3d LIN_VEL_CurToRefInCur = state->LIN_VEL_Bi0ToRefInCur();

    riverState.velocity_in_body.x = LIN_VEL_CurToRefInCur(0);
    riverState.velocity_in_body.y = LIN_VEL_CurToRefInCur(1);
    riverState.velocity_in_body.z = LIN_VEL_CurToRefInCur(2);

    riverState.quaternion.x = state->SO3_Bi0ToRef.unit_quaternion().x();
    riverState.quaternion.y = state->SO3_Bi0ToRef.unit_quaternion().y();
    riverState.quaternion.z = state->SO3_Bi0ToRef.unit_quaternion().z();
    riverState.quaternion.w = state->SO3_Bi0ToRef.unit_quaternion().w();

    statePublisher.publish(riverState);

    {
      LOCK_RIVER2_STATUS
      RiverStatus::StateManager::CurStatus ^= RiverStatus::StateManager::Status::NewStateNeedToPublish; /// 修改状态为不需要发布
    }
  }
} // namespace ns_river2