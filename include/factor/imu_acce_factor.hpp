/**
 * @file imu_acce_factor.hpp
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

#ifndef RIVER2_IMU_ACCE_FACTOR_HPP
#define RIVER2_IMU_ACCE_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_river2
{
  template <int Order>
  struct IMUAcceFactorWithConstBias
  {
  public:
    using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

  private:
    IMUFrame::Ptr frame;

    double weight;
    double so3DtInv, velDtInv;
    Sophus::SO3d so3_BiToBi0;
    Eigen::Vector3d pos_BiToBi0;

    std::pair<std::size_t, double> so3IU, velIU;

    std::size_t SO3_OFFSET, VEL_OFFSET, BA_OFFSET, GRAVITY_OFFSET;

  public:
    IMUAcceFactorWithConstBias(const SplineMetaType &so3Meta, const SplineMetaType &velMeta,
                               IMUFrame::Ptr imuFrame, double weight,
                               Sophus::SO3d so3_offset, Eigen::Vector3d pos_offset)
        : frame(std::move(imuFrame)), weight(weight),
          so3DtInv(1.0 / so3Meta.segments.front().dt),
          velDtInv(1.0 / velMeta.segments.front().dt),
          so3_BiToBi0(so3_offset), pos_BiToBi0(pos_offset)
    {
      // compute knots indexes
      so3Meta.template ComputeSplineIndex(frame->GetTimestamp(), so3IU.first, so3IU.second);
      velMeta.template ComputeSplineIndex(frame->GetTimestamp(), velIU.first, velIU.second);

      // compute knots offset in 'parBlocks'
      SO3_OFFSET = so3IU.first;
      VEL_OFFSET = so3Meta.NumParameters() + velIU.first;
      BA_OFFSET = so3Meta.NumParameters() + velMeta.NumParameters();
      GRAVITY_OFFSET = BA_OFFSET + 1;
    }

    static auto Create(const SplineMetaType &so3Meta, const SplineMetaType &velMeta, const IMUFrame::Ptr &frame,
                       double weight,
                       Sophus::SO3d so3_offset, Eigen::Vector3d pos_offset)
    {
      return new ceres::DynamicAutoDiffCostFunction<IMUAcceFactorWithConstBias>(
          new IMUAcceFactorWithConstBias(so3Meta, velMeta, frame, weight, so3_offset, pos_offset));
    }

    static std::size_t TypeHashCode()
    {
      return typeid(IMUAcceFactorWithConstBias).hash_code();
    }

  public:
    /**
     * param blocks:
     * [ SO3 | ... | SO3 | VEL | ... | VEL | BA | GRAVITY ]
     */
    template <class T>
    bool operator()(T const *const *parBlocks, T *sResiduals) const
    {
      // compute rotation from {current frame} to {reference frame}
      Sophus::SO3<T> SO3_Bi0ToRef;
      Sophus::SO3Tangent<T> SO3_VEL_Bi0ToRefInBi0, SO3_ACCE_Bi0ToRefInBi0;
      ns_ctraj::CeresSplineHelper<Order>::template EvaluateLie<T, Sophus::SO3>(
          parBlocks + SO3_OFFSET, so3IU.second, so3DtInv, &SO3_Bi0ToRef,
          &SO3_VEL_Bi0ToRefInBi0, &SO3_ACCE_Bi0ToRefInBi0);
      Sophus::SO3Tangent<T> SO3_VEL_Bi0ToRefInRef = SO3_Bi0ToRef * SO3_VEL_Bi0ToRefInBi0;
      Sophus::SO3Tangent<T> SO3_ACCE_Bi0ToRefInRef = SO3_Bi0ToRef * SO3_ACCE_Bi0ToRefInBi0;

      // compute linear acceleration of {current frame} with
      // respect to {reference frame} expressed in {reference frame}
      // note that the first derivative of velocity spline is exactly the linear acceleration
      Eigen::Vector3<T> LIN_ACCE_Bi0ToRefInRef;
      ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 1>(
          parBlocks + VEL_OFFSET, velIU.second, velDtInv, &LIN_ACCE_Bi0ToRefInRef);

      Eigen::Map<const Eigen::Vector3<T>> BA(parBlocks[BA_OFFSET]);
      Eigen::Map<const Eigen::Vector3<T>> GRAVITY_IN_REF(parBlocks[GRAVITY_OFFSET]);

      Sophus::SO3<T> SO3_BiToRef = SO3_Bi0ToRef * so3_BiToBi0.cast<T>();
      Eigen::Vector3<T> LIN_ACCE_BiToRefInRef =
          -Sophus::SO3<T>::hat(SO3_Bi0ToRef * pos_BiToBi0.cast<T>()) * SO3_ACCE_Bi0ToRefInRef +
          LIN_ACCE_Bi0ToRefInRef -
          Sophus::SO3<T>::hat(SO3_VEL_Bi0ToRefInRef) * Sophus::SO3<T>::hat(SO3_Bi0ToRef * pos_BiToBi0.cast<T>()) * SO3_VEL_Bi0ToRefInRef;

      Eigen::Vector3<T> accePred =
          SO3_BiToRef.inverse() * (LIN_ACCE_BiToRefInRef - GRAVITY_IN_REF) + BA;

      Eigen::Vector3<T> acceResiduals = accePred - frame->GetAcce().cast<T>();

      Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
      residuals = T(weight) * acceResiduals;

      return true;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}

#endif // RIVER2_IMU_ACCE_FACTOR_HPP
