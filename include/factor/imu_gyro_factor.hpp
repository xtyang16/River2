/**
 * @file imu_gyro_factor.hpp
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

#ifndef RIVER2_IMU_GYRO_FACTOR_HPP
#define RIVER2_IMU_GYRO_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_river2
{
  template <int Order>
  struct IMUGyroFactorWithConstBias
  {
  public:
    using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

  private:
    IMUFrame::Ptr frame;

    double weight;
    double so3DtInv;
    Sophus::SO3d so3_BiToBi0;

    // array offset
    std::pair<std::size_t, double> so3IU;
    std::size_t SO3_OFFSET, BG_OFFSET;

  public:
    IMUGyroFactorWithConstBias(const SplineMetaType &so3Meta, IMUFrame::Ptr imuFrame, double weight,
                               Sophus::SO3d so3_offset)
        : frame(std::move(imuFrame)), weight(weight),
          so3DtInv(1.0 / so3Meta.segments.front().dt),
          so3_BiToBi0(so3_offset)
    {
      // compute knots indexes
      so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);

      // compute knots offset in 'parBlocks'
      SO3_OFFSET = so3IU.first;
      BG_OFFSET = so3Meta.NumParameters();
    }

    static auto
    Create(const SplineMetaType &so3Meta, const IMUFrame::Ptr &frame, double weight,
           Sophus::SO3d so3_offset)
    {
      return new ceres::DynamicAutoDiffCostFunction<IMUGyroFactorWithConstBias>(
          new IMUGyroFactorWithConstBias(so3Meta, frame, weight, so3_offset));
    }

    static std::size_t TypeHashCode()
    {
      return typeid(IMUGyroFactorWithConstBias).hash_code();
    }

  public:
    /**
     * param blocks:
     * [ SO3 | ... | SO3 | BG ]
     */
    template <class T>
    bool operator()(T const *const *parBlocks, T *sResiduals) const
    {
      Sophus::SO3<T> SO3_Bi0ToRef;
      Sophus::SO3Tangent<T> SO3_VEL_Bi0ToRefInBi0;
      ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
          parBlocks + SO3_OFFSET, so3IU.second, so3DtInv,
          &SO3_Bi0ToRef, &SO3_VEL_Bi0ToRefInBi0);

      Eigen::Map<const Eigen::Vector3<T>> BG(parBlocks[BG_OFFSET]);

      Sophus::SO3<T> SO3_BiToRef = SO3_Bi0ToRef * so3_BiToBi0.cast<T>();
      Sophus::SO3Tangent<T> SO3_VEL_Bi0ToRefInRef = SO3_Bi0ToRef * SO3_VEL_Bi0ToRefInBi0;
      Eigen::Vector3<T> pred = SO3_BiToRef.inverse() * SO3_VEL_Bi0ToRefInRef + BG;

      Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
      residuals = pred - frame->GetGyro().cast<T>();
      residuals = T(weight) * residuals;

      return true;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif // RIVER2_IMU_GYRO_FACTOR_HPP
