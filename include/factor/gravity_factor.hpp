/**
 * @file gravity_factor.hpp
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

#ifndef RIVER2_GRAVITY_FACTOR_HPP
#define RIVER2_GRAVITY_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_river2
{
  struct GravityFactor
  {
  private:
    const double dt;
    Eigen::Matrix3d DEL_VEL_1;
    Eigen::Vector3d DEL_VEL_2;

    Eigen::Vector3d LIN_VEL_I_Bi0ToRefInRef;
    Eigen::Vector3d LIN_VEL_J_Bi0ToRefInRef;

    Eigen::Vector3d POS_BiInBi0;

    double weight;

  public:
    GravityFactor(double dt, Eigen::Matrix3d deltaVel1, Eigen::Vector3d deltaVel2,
                  Eigen::Vector3d VEL_I_Bi0ToRefInRef, Eigen::Vector3d VEL_J_Bi0ToRefInRef,
                  const Eigen::Vector3d &BiInBi0,
                  double weight)
        : dt(dt), DEL_VEL_1(std::move(deltaVel1)), DEL_VEL_2(std::move(deltaVel2)),
          LIN_VEL_I_Bi0ToRefInRef(std::move(VEL_I_Bi0ToRefInRef)), LIN_VEL_J_Bi0ToRefInRef(std::move(VEL_J_Bi0ToRefInRef)),
          POS_BiInBi0(BiInBi0),
          weight(weight) {}

    static auto
    Create(double dt, Eigen::Matrix3d deltaVel1, Eigen::Vector3d deltaVel2,
           Eigen::Vector3d VEL_I_Bi0ToRefInRef, Eigen::Vector3d VEL_J_Bi0ToRefInRef,
           const Eigen::Vector3d &BiInBi0,
           double weight)
    {
      return new ceres::DynamicAutoDiffCostFunction<GravityFactor>(
          new GravityFactor(dt, deltaVel1, deltaVel2, VEL_I_Bi0ToRefInRef, VEL_J_Bi0ToRefInRef,
                            BiInBi0, weight));
    }

    static std::size_t TypeHashCode()
    {
      return typeid(GravityFactor).hash_code();
    }

  public:
    /**
     * param blocks:
     * [ GRAVITY ]
     */
    template <class T>
    bool operator()(T const *const *sKnots, T *sResiduals) const
    {
      Eigen::Map<const Eigen::Vector3<T>> gravity(sKnots[0]);

      Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);

      Eigen::Vector3<T> pred = LIN_VEL_I_Bi0ToRefInRef - LIN_VEL_J_Bi0ToRefInRef - gravity * dt;
      Eigen::Vector3<T> mes = -DEL_VEL_1.cast<T>() * POS_BiInBi0 + DEL_VEL_2.cast<T>();
      residuals = T(weight) * (pred - mes);

      return true;
    }
  };
}
#endif // RIVER2_GRAVITY_FACTOR_HPP
