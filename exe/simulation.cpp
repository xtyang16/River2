/**
 * @file simulation.cpp
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

#include "ros/ros.h"
#include "thread"
#include "core/status.h"
#include "core/river2.h"
#include "view/view_util.h"
#include "ctraj/tiny/multi_viewer.h"
#include "nofree/simulation.hpp"
#include "cereal/types/utility.hpp"

template <class Type>
Type GetParamFromROS(const std::string &param)
{
  Type par;
  if (!ros::param::get(param, par))
  {
    throw ns_river2::Status(
        ns_river2::Status::Flag::CRITICAL,
        fmt::format("the ros param couldn't obtained from '{}'.", param));
  }
  return par;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "river2_simu_node");
  try
  {
    auto outputPath = GetParamFromROS<std::string>("/river2_simu_node/output_path");
    spdlog::info("config output path: '{}'", outputPath);

    if (!std::filesystem::exists(outputPath))
    {
      if (!std::filesystem::create_directories(outputPath))
      {
        throw ns_river2::Status(
            ns_river2::Status::Flag::CRITICAL, "the 'output_path' is not valid.");
      }
    }

    auto staDur = GetParamFromROS<double>("/river2_simu_node/stationary_duration");
    spdlog::info("stationary duration: '{:.6f}'", staDur);

    auto motionDur = GetParamFromROS<double>("/river2_simu_node/motion_duration");
    spdlog::info("motion duration: '{:.6f}'", motionDur);

    auto calibParam = ns_river2::CalibParamManager::Create();

    {
      auto imutopic_1 = GetParamFromROS<std::string>("/river2_simu_node/imutopic_1");
      spdlog::info("spatiotemporal parameters: '{}'", imutopic_1);
      auto extri_imu_1 = GetParamFromROS<std::string>("/river2_simu_node/extri_imutopic_1");
      spdlog::info("spatiotemporal parameters: '{}'", extri_imu_1);
      auto items = ns_river2::SplitString(extri_imu_1, ';');
      if (items.size() != 8)
      {
        throw ns_river2::Status(
            ns_river2::Status::Flag::CRITICAL,
            "wrong spatiotemporal parameters are set, they are should be [qx;qy;qz;qw;px;py;pz;tm].");
      }

      calibParam->EXTRI.SO3_BiToBi0[imutopic_1] = Sophus::SO3d(Eigen::Quaterniond(
          std::stod(items[3]), std::stod(items[0]), std::stod(items[1]), std::stod(items[2])));
      calibParam->EXTRI.POS_BiInBi0[imutopic_1] = Eigen::Vector3d(std::stod(items[4]), std::stod(items[5]), std::stod(items[6]));
      calibParam->TEMPORAL.TIME_OFFSET_BiToBi0[imutopic_1] = std::stod(items[7]);
    }

    {
      auto imutopic_2 = GetParamFromROS<std::string>("/river2_simu_node/imutopic_2");
      spdlog::info("spatiotemporal parameters: '{}'", imutopic_2);
      auto extri_imu_2 = GetParamFromROS<std::string>("/river2_simu_node/extri_imutopic_2");
      spdlog::info("spatiotemporal parameters: '{}'", extri_imu_2);
      auto items = ns_river2::SplitString(extri_imu_2, ';');
      if (items.size() != 8)
      {
        throw ns_river2::Status(
            ns_river2::Status::Flag::CRITICAL,
            "wrong spatiotemporal parameters are set, they are should be [qx;qy;qz;qw;px;py;pz;tm].");
      }

      calibParam->EXTRI.SO3_BiToBi0[imutopic_2] = Sophus::SO3d(Eigen::Quaterniond(
          std::stod(items[3]), std::stod(items[0]), std::stod(items[1]), std::stod(items[2])));
      calibParam->EXTRI.POS_BiInBi0[imutopic_2] = Eigen::Vector3d(std::stod(items[4]), std::stod(items[5]), std::stod(items[6]));
      calibParam->TEMPORAL.TIME_OFFSET_BiToBi0[imutopic_2] = std::stod(items[7]);
    }

    {
      auto radartopic_1 = GetParamFromROS<std::string>("/river2_simu_node/radartopic_1");
      spdlog::info("spatiotemporal parameters: '{}'", radartopic_1);
      auto extri_radartopic_1 = GetParamFromROS<std::string>("/river2_simu_node/extri_radartopic_1");
      spdlog::info("spatiotemporal parameters: '{}'", extri_radartopic_1);
      auto items = ns_river2::SplitString(extri_radartopic_1, ';');
      if (items.size() != 8)
      {
        throw ns_river2::Status(
            ns_river2::Status::Flag::CRITICAL,
            "wrong spatiotemporal parameters are set, they are should be [qx;qy;qz;qw;px;py;pz;tm].");
      }

      calibParam->EXTRI.SO3_RjToBi0[radartopic_1] = Sophus::SO3d(Eigen::Quaterniond(
          std::stod(items[3]), std::stod(items[0]), std::stod(items[1]), std::stod(items[2])));
      calibParam->EXTRI.POS_RjInBi0[radartopic_1] = Eigen::Vector3d(std::stod(items[4]), std::stod(items[5]), std::stod(items[6]));
      calibParam->TEMPORAL.TIME_OFFSET_RjToBi0[radartopic_1] = std::stod(items[7]);
    }

    {
      auto radartopic_2 = GetParamFromROS<std::string>("/river2_simu_node/radartopic_2");
      spdlog::info("spatiotemporal parameters: '{}'", radartopic_2);
      auto extri_radartopic_2 = GetParamFromROS<std::string>("/river2_simu_node/extri_radartopic_2");
      spdlog::info("spatiotemporal parameters: '{}'", extri_radartopic_2);
      auto items = ns_river2::SplitString(extri_radartopic_2, ';');
      if (items.size() != 8)
      {
        throw ns_river2::Status(
            ns_river2::Status::Flag::CRITICAL,
            "wrong spatiotemporal parameters are set, they are should be [qx;qy;qz;qw;px;py;pz;tm].");
      }

      calibParam->EXTRI.SO3_RjToBi0[radartopic_2] = Sophus::SO3d(Eigen::Quaterniond(
          std::stod(items[3]), std::stod(items[0]), std::stod(items[1]), std::stod(items[2])));
      calibParam->EXTRI.POS_RjInBi0[radartopic_2] = Eigen::Vector3d(std::stod(items[4]), std::stod(items[5]), std::stod(items[6]));
      calibParam->TEMPORAL.TIME_OFFSET_RjToBi0[radartopic_2] = std::stod(items[7]);
    }

    calibParam->ShowParamStatus();

    auto frequency = GetParamFromROS<std::string>("/river2_simu_node/frequency");
    spdlog::info("frequency: '{}'", frequency);

    auto freq = ns_river2::SplitString(frequency, ';');
    if (freq.size() != 2)
    {
      throw ns_river2::Status(
          ns_river2::Status::Flag::CRITICAL,
          "wrong frequency are set, they are should be [imu;radar].");
    }
    int imuFreq = std::stoi(freq[0]);
    int radarFreq = std::stoi(freq[1]);
    spdlog::info("imu frequency: '{}'", imuFreq);
    spdlog::info("radar frequency: '{}'", radarFreq);

    auto noise = GetParamFromROS<std::string>("/river2_simu_node/noise");
    auto n = ns_river2::SplitString(noise, ';');
    if (n.size() != 3)
    {
      throw ns_river2::Status(
          ns_river2::Status::Flag::CRITICAL,
          "wrong noises are set, they are should be [acce;gyro;radar].");
    }
    double acceNoise = std::stod(n[0]), gyroNoise = std::stod(n[1]), radarNoise = std::stod(n[2]);
    spdlog::info("accelerator noise: '{}'", acceNoise);
    spdlog::info("gyroscope noise: '{}'", gyroNoise);
    spdlog::info("radar noise: '{}'", radarNoise);

    auto bias = GetParamFromROS<std::string>("/river2_simu_node/bias");
    auto b = ns_river2::SplitString(bias, ';');
    if (b.size() != 2)
    {
      throw ns_river2::Status(
          ns_river2::Status::Flag::CRITICAL,
          "wrong biases are set, they are should be [acce;gyro].");
    }
    double acceBias = std::stod(b[0]), gyroBias = std::stod(b[1]);
    spdlog::info("accelerator bias: '{}'", acceBias);
    spdlog::info("gyroscope bias: '{}'", gyroBias);

    auto simulator = ns_river2::Simulator(outputPath, calibParam, staDur, motionDur);

    simulator.Simulate(imuFreq, radarFreq, acceNoise, gyroNoise, radarNoise, acceBias, gyroBias);

    simulator.GetTraj().GetTrajectory()->Save(outputPath + "/trajectory.json");

    std::vector<std::pair<double, Eigen::Vector3d>> velocityInW, velocityInB;
    std::vector<std::pair<double, Sophus::SO3d>> quatBtoW;
    auto traj = simulator.GetTraj().GetTrajectory();

    std::ofstream file_out(outputPath + "/ref_simu.txt", std::ios::out);
    for (double t = traj->MinTime(); t < traj->MaxTime();)
    {
      Eigen::Vector3d LIN_VEL_BtoWinW = traj->LinearVeloInRef(t);
      velocityInW.emplace_back(t, LIN_VEL_BtoWinW);

      auto SO3_CurToW = traj->Pose(t).so3();
      quatBtoW.emplace_back(t, SO3_CurToW);

      Eigen::Vector3d LIN_VEL_BtoWinB = SO3_CurToW.inverse() * LIN_VEL_BtoWinW;
      velocityInB.emplace_back(t, LIN_VEL_BtoWinB);

      file_out << std::fixed << std::setprecision(16) << //
          t << " " <<                                    //
          SO3_CurToW.unit_quaternion().w() << " " <<     //
          SO3_CurToW.unit_quaternion().x() << " " <<     //
          SO3_CurToW.unit_quaternion().y() << " " <<     //
          SO3_CurToW.unit_quaternion().z() << " " <<     //
          LIN_VEL_BtoWinW(0) << " " <<                   //
          LIN_VEL_BtoWinW(1) << " " <<                   //
          LIN_VEL_BtoWinW(2) << "\n";

      t += 0.01;
    }
    // velocity
    {
      std::ofstream file(outputPath + "/velocity.json", std::ios::out);
      cereal::JSONOutputArchive ar(file);
      ar(
          cereal::make_nvp("velocity_in_world", velocityInW),
          cereal::make_nvp("velocity_in_body", velocityInB));
    }

    // rotation
    {
      std::ofstream file(outputPath + "/rotation.json", std::ios::out);
      cereal::JSONOutputArchive ar(file);
      ar(cereal::make_nvp("quat_body_to_world", quatBtoW));
    }

    simulator.GetTraj().VisualizationDynamic(outputPath);
  }
  catch (const ns_river2::Status &status)
  {
    // if error happened, print it
    switch (status.flag)
    {
    case ns_river2::Status::Flag::FINE:
      // this case usually won't happen
      spdlog::info(status.what);
      break;
    case ns_river2::Status::Flag::WARNING:
      spdlog::warn(status.what);
      break;
    case ns_river2::Status::Flag::ERROR:
      spdlog::error(status.what);
      break;
    case ns_river2::Status::Flag::CRITICAL:
      spdlog::critical(status.what);
      break;
    }
  }
  catch (const std::exception &e)
  {
    // an unknown exception not thrown by this program
    spdlog::critical(e.what());
  }
  ros::shutdown();
  return 0;
}