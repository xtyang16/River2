/**
 * @file radar_factor.hpp
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

#ifndef RIVER2_RADAR_FACTOR_HPP
#define RIVER2_RADAR_FACTOR_HPP

#include "core/calib_param_manager.h"
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"
#include <utility>

namespace ns_river2
{
  template <int Order>
  struct RadarFactor
  {
  public:
    using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

  private:
    RadarTarget::Ptr target;

    double weight;
    double so3DtInv, velDtInv;

    Sophus::SO3d so3_BjToBi0;
    Eigen::Vector3d pos_BjToBi0;

    // compute knots indexes
    std::pair<std::size_t, double> so3IU, velIU;
    std::size_t SO3_OFFSET, VEL_OFFSET;

  public:
    RadarFactor(RadarTarget::Ptr radarTar,
                const SplineMetaType &so3Meta,
                const SplineMetaType &velMeta, double weight,
                Sophus::SO3d so3_offset, Eigen::Vector3d pos_offset)
        : target(std::move(radarTar)), weight(weight),
          so3DtInv(1.0 / so3Meta.segments.front().dt), velDtInv(1.0 / velMeta.segments.front().dt),
          so3_BjToBi0(so3_offset), pos_BjToBi0(pos_offset)
    {
      // compute knots indexes
      so3Meta.template ComputeSplineIndex(target->GetTimestamp(), so3IU.first, so3IU.second);
      velMeta.template ComputeSplineIndex(target->GetTimestamp(), velIU.first, velIU.second);

      // compute knots offset in 'parBlocks'
      SO3_OFFSET = so3IU.first;
      VEL_OFFSET = so3Meta.NumParameters() + velIU.first;
    }

    static auto Create(const RadarTarget::Ptr &radarTar,
                       const SplineMetaType &so3Meta, const SplineMetaType &velMeta, double weight,
                       Sophus::SO3d so3_offset, Eigen::Vector3d pos_offset)
    {
      return new ceres::DynamicAutoDiffCostFunction<RadarFactor>(
          new RadarFactor(radarTar, so3Meta, velMeta, weight, so3_offset, pos_offset));
    }

    static std::size_t TypeHashCode()
    {
      return typeid(RadarFactor).hash_code();
    }

  public:
    /**
     * param blocks:
     * [ SO3 | ... | SO3 | VEL | ... | VEL ]
     */
    template <class T>
    bool operator()(T const *const *parBlocks, T *sResiduals) const
    {
      // compute rotation from {current frame} to {reference frame}
      Sophus::SO3<T> SO3_Bi0ToRef;
      // compute angular velocity of {current frame} with respect to {reference frame} expressed in {current frame}
      Sophus::SO3Tangent<T> SO3_VEL_Bi0ToRefInBi0;
      ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
          parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_Bi0ToRef, &SO3_VEL_Bi0ToRefInBi0);
      Eigen::Vector3<T> SO3_VEL_Bi0ToRefInRef = SO3_Bi0ToRef * SO3_VEL_Bi0ToRefInBi0;

      // compute linear velocity of {current frame} with respect to {reference frame} expressed in {reference frame}
      // note that the zero derivative of velocity spline is exactly the linear velocity
      Eigen::Vector3<T> LIN_VEL_Bi0ToRefInRef;
      ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
          parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_VEL_Bi0ToRefInRef);

      Eigen::Vector3<T> LIN_VEL_BjtoRefInRef =
          -Sophus::SO3<T>::hat(SO3_Bi0ToRef * pos_BjToBi0.cast<T>()) * SO3_VEL_Bi0ToRefInRef +
          LIN_VEL_Bi0ToRefInRef;

      T v1 = -target->GetTargetXYZ().cast<T>().dot(
          so3_BjToBi0.cast<T>().matrix().transpose() * SO3_Bi0ToRef.matrix().transpose() * LIN_VEL_BjtoRefInRef);

      T v2 = static_cast<T>(target->GetRadialVelocity());

      Eigen::Map<Eigen::Vector1<T>> residuals(sResiduals);
      residuals(0, 0) = T(weight) * (target->GetInvRange() * v1 - v2);

      return true;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ns_river2
#endif // RIVER2_RADAR_FACTOR_HPP
