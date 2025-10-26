/**
 * @file radar.cpp
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

#include "sensor/radar.h"
#include "spdlog/spdlog.h"

#include <utility>

namespace ns_river2
{
  // -----------
  // RadarTarget
  // -----------

  [[nodiscard]] double RadarTarget::GetTimestamp() const
  {
    return _timestamp;
  }

  void RadarTarget::SetTimestamp(double timestamp)
  {
    _timestamp = timestamp;
  }

  const Eigen::Vector3d &RadarTarget::GetTargetXYZ() const
  {
    return _target;
  }

  Eigen::Vector3d RadarTarget::GetTargetRTP() const
  {
    return ns_ctraj::XYZtoRTP<double>(_target);
  }

  double RadarTarget::GetRadialVelocity() const
  {
    return _radialVel;
  }

  RadarTarget::Ptr RadarTarget::Create(double timestamp, const Eigen::Vector4d &rawMes)
  {
    return std::make_shared<RadarTarget>(timestamp, rawMes);
  }

  RadarTarget::RadarTarget(double timestamp, const Eigen::Vector4d &rawMes)
      : _timestamp(timestamp), _target(ns_ctraj::RTPtoXYZ<double>({rawMes(0), rawMes(1), rawMes(2)})),
        _radialVel(rawMes(3)), _range(rawMes(0)), _invRange(1.0 / rawMes(0)) {}

  RadarTarget::Ptr RadarTarget::Create(double timestamp, const Eigen::Vector3d &target, double radialVel)
  {
    return std::make_shared<RadarTarget>(timestamp, target, radialVel);
  }

  RadarTarget::RadarTarget(double timestamp, Eigen::Vector3d target, double radialVel) : _timestamp(timestamp), _target(std::move(target)), _radialVel(radialVel),
                                                                                         _range(_target.norm()), _invRange(1.0 / _target.norm()) {}

  double RadarTarget::GetRange() const
  {
    return _range;
  }

  double RadarTarget::GetInvRange() const
  {
    return _invRange;
  }

  // ----------------
  // RadarTargetArray
  // ----------------

  RadarTargetArray::RadarTargetArray(double timestamp, const std::vector<RadarTarget::Ptr> &targets)
      : _timestamp(timestamp), _targets(targets) {}

  RadarTargetArray::Ptr RadarTargetArray::Create(double timestamp, const std::vector<RadarTarget::Ptr> &targets)
  {
    return std::make_shared<RadarTargetArray>(timestamp, targets);
  }

  double RadarTargetArray::GetTimestamp() const
  {
    return _timestamp;
  }

  const std::vector<RadarTarget::Ptr> &RadarTargetArray::GetTargets() const
  {
    return _targets;
  }

  void RadarTargetArray::SetTimestamp(double timestamp)
  {
    _timestamp = timestamp;
  }

  bool RadarTargetArray::SaveTargetArraysToDisk(const std::string &filename,
                                                const std::vector<RadarTargetArray::Ptr> &arrays,
                                                int precision)
  {
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(precision);
    cereal::JSONOutputArchive ar(file);
    ar(cereal::make_nvp("radar_arrays", arrays));
    return true;
  }

  /**
   * @brief 利用最小二乘的方法计算速度
   *
   * @param SO3_RtoRef
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d RadarTargetArray::RadarVelocityFromStaticTargetArray(const Sophus::SO3d &SO3_RtoRef,
                                                                       const bool &ifPrintError)
  {
    int targetSize = _targets.size();
    Eigen::Matrix3d ROT_RerToR = SO3_RtoRef.inverse().matrix();
    Eigen::VectorXd lVec(targetSize);
    Eigen::MatrixXd BMat(targetSize, 3);
    for (int i = 0; i < targetSize; ++i)
    {
      const auto &tar = _targets.at(i);
      lVec(i) = tar->GetRadialVelocity() * tar->GetTargetXYZ().norm();
      BMat.block<1, 3>(i, 0) = -tar->GetTargetXYZ().transpose() * ROT_RerToR;
    }
    Eigen::Vector3d xVec = (BMat.transpose() * BMat).inverse() * BMat.transpose() * lVec;

    if (ifPrintError)
    {
      // check the result
      Eigen::Matrix3d ROT_RerToR = SO3_RtoRef.inverse().matrix();
      double ErrorOut(.0);
      int num(0);
      for (int i = 0; i < targetSize; ++i)
      {
        const auto &tar = _targets.at(i);
        ErrorOut = (ErrorOut * num +
                    abs((-tar->GetTargetXYZ().transpose() * ROT_RerToR) * xVec -
                        (tar->GetRadialVelocity() * tar->GetTargetXYZ().norm()))) /
                   (num + 1);
        num += 1;
      }
      spdlog::info("targetSize: {:d}, ErrorOut: {:.6f}.", targetSize, ErrorOut);
    }
    return xVec;
  }

  Eigen::Vector3d RadarTargetArray::RadarVelocityFromStaticTargetArray(const Sophus::SO3d &SO3_RtoRef,
                                                                       const std::vector<size_t> &targetIndex)
  {
    int targetSize = targetIndex.size();
    Eigen::Matrix3d ROT_RerToR = SO3_RtoRef.inverse().matrix();
    Eigen::VectorXd lVec(targetSize);
    Eigen::MatrixXd BMat(targetSize, 3);
    for (int i = 0; i < targetSize; i++)
    {
      const auto &tar = _targets.at(targetIndex.at(i));
      lVec(i) = tar->GetRadialVelocity() * tar->GetTargetXYZ().norm();
      BMat.block<1, 3>(i, 0) = -tar->GetTargetXYZ().transpose() * ROT_RerToR;
    }
    Eigen::Vector3d xVec = (BMat.transpose() * BMat).inverse() * BMat.transpose() * lVec;
    return xVec;
  }

  void RadarTargetArray::RadarRemoveOutlier(const Sophus::SO3d &SO3_RtoRef,
                                            const double &errorThreshold,
                                            const double &ratioThreshold,
                                            const double &outlierProb,
                                            const double &successProb,
                                            const int &ransacPoints)
  {
    size_t targetSize = _targets.size();
    if (targetSize > static_cast<size_t>(ransacPoints))
    {
      std::vector<size_t> indexTargetBest(targetSize), indexTargetNow(targetSize);
      size_t retainNumBest(0);
      // start compute
      Eigen::Matrix3d ROT_RerToR = SO3_RtoRef.inverse().matrix();
      // get random vector
      std::vector<size_t> idx(targetSize);
      std::iota(idx.begin(), idx.end(), 0);
      std::random_device rd;
      std::default_random_engine g(rd());
      // eliminate outlier
      size_t ransacIter = std::min(static_cast<size_t>(std::log(1.0 - successProb) /
                                                       std::log(1.0 - std::pow(1.0 - outlierProb, ransacPoints))),
                                   static_cast<size_t>(targetSize));
      for (size_t i = 0; i < ransacIter; ++i)
      {
        std::shuffle(idx.begin(), idx.end(), g);

        std::vector<size_t> targetIndex(ransacPoints);
        std::copy(idx.begin(), idx.begin() + ransacPoints, targetIndex.begin());
        Eigen::Vector3d xVecNow = RadarVelocityFromStaticTargetArray(SO3_RtoRef, targetIndex);

        // check the number retain
        size_t retainNumNow = 0;
        for (size_t j = 0; j < targetSize; ++j)
        {
          const auto &tar = _targets.at(j);
          double errorNow = abs((-tar->GetTargetXYZ().transpose() * ROT_RerToR) * xVecNow -
                                (tar->GetRadialVelocity() * tar->GetTargetXYZ().norm()));
          // spdlog::warn("errorNow: {:.6f}, errorThreshold: {:.6f}", errorNow, abs(tar->GetRadialVelocity() * tar->GetTargetXYZ().norm()));
          // here is the point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          if (errorNow <= errorThreshold || errorNow <= errorThreshold * abs(tar->GetRadialVelocity() * tar->GetTargetXYZ().norm()))
            indexTargetNow[retainNumNow++] = j;
        }
        if (retainNumNow > retainNumBest && retainNumNow > static_cast<size_t>(ransacPoints))
        {
          retainNumBest = retainNumNow;
          std::copy(indexTargetNow.begin(), indexTargetNow.begin() + retainNumBest, indexTargetBest.begin());
          if (retainNumBest >= targetSize * ratioThreshold)
            break;
        }
        // spdlog::warn("\n");
      }

      if (retainNumBest >= static_cast<size_t>(targetSize * (1.0 - outlierProb)))
      {
        for (size_t i = 0; i < retainNumBest; i++)
        {
          _targets[i] = _targets.at(indexTargetBest.at(i));
        }
        _targets.resize(retainNumBest);
      }
      retainNumBest = targetSize;
      spdlog::info("retainNumBest: {:d}, targetSize: {:d}", _targets.size(), targetSize);
    }
  }

  bool RadarTargetArray::IfZeroVelocity(const double &outlierProb,
                                        const double &zeroThreshold)
  {
    size_t targetSize = _targets.size();
    std::vector<double> radialVelVec(targetSize);
    for (size_t i = 0; i < targetSize; i++)
    {
      radialVelVec[i] = _targets.at(i)->GetRadialVelocity();
    }
    std::sort(radialVelVec.begin(), radialVelVec.end());
    size_t vel2SigmaIndex = targetSize * ((1 - outlierProb) > 0.8 ? 0.8 : (1 - outlierProb));
    if (vel2SigmaIndex == targetSize)
      vel2SigmaIndex--;
    double vel2Sigma = radialVelVec[vel2SigmaIndex];
    if (vel2Sigma <= zeroThreshold)
      return true;
    else
      return false;
  }

} // namespace ns_river2
