/**
 * @file vel_preintegration_factor.hpp
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

#ifndef RIVER2_VEL_PREINTEGRATION_FACTOR_HPP
#define RIVER2_VEL_PREINTEGRATION_FACTOR_HPP

#include <utility>
#include "ctraj/utils/eigen_utils.hpp"
#include "ctraj/utils/sophus_utils.hpp"
#include "sensor/imu.h"

namespace ns_river2
{
    template <int Order>
    struct VelPreintegrationFactor
    {
    public:
        using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

    private:
        Eigen::Matrix3d DEL_VEL_1;
        Eigen::Vector3d DEL_VEL_2;

        double dt;
        double weight;
        double velDtInv;

        Eigen::Vector3d POS_BiInBi0;

        std::pair<std::size_t, double> vel1IU, vel2IU;
        std::size_t VEL1_OFFSET, VEL2_OFFSET, GRAVITY_OFFSET;

    public:
        VelPreintegrationFactor(const SplineMetaType &velMeta,
                                const Eigen::Matrix3d &deltaVel1,
                                const Eigen::Vector3d &deltaVel2,
                                double t1, double t2, double weight,
                                const Eigen::Vector3d &BiInBi0)
            : DEL_VEL_1(std::move(deltaVel1)), DEL_VEL_2(std::move(deltaVel2)),
              dt(t2 - t1), weight(weight), velDtInv(1.0 / velMeta.segments.front().dt),
              POS_BiInBi0(std::move(BiInBi0))
        {
            // compute knots indexes
            velMeta.template ComputeSplineIndex(t1, vel1IU.first, vel1IU.second);
            velMeta.template ComputeSplineIndex(t2, vel2IU.first, vel2IU.second);

            // compute knots offset in 'parBlocks'
            VEL1_OFFSET = vel1IU.first;
            VEL2_OFFSET = vel2IU.first;
            GRAVITY_OFFSET = velMeta.NumParameters();
        }

        static auto Create(const SplineMetaType &velMeta,
                           const Eigen::Matrix3d &deltaVel1,
                           const Eigen::Vector3d &deltaVel2,
                           double t1, double t2, double weight,
                           const Eigen::Vector3d &BiInBi0)
        {
            return new ceres::DynamicAutoDiffCostFunction<VelPreintegrationFactor>(
                new VelPreintegrationFactor(velMeta, deltaVel1, deltaVel2, t1, t2, weight, BiInBi0));
        }

        static std::size_t TypeHashCode()
        {
            return typeid(VelPreintegrationFactor).hash_code();
        }

    public:
        /**
         * param blocks:
         * [ VEL | ... | VEL | GRAVITY ]
         */
        template <class T>
        bool operator()(T const *const *parBlocks, T *sResiduals) const
        {
            // compute linear acceleration of {current frame} with
            // respect to {reference frame} expressed in {reference frame}
            // note that the first derivative of velocity spline is exactly the linear acceleration
            Eigen::Map<const Eigen::Vector3<T>> GRAVITY_IN_REF(parBlocks[GRAVITY_OFFSET]);

            Eigen::Vector3<T> LIN_VEL_I_Bi0ToRefInRef, LIN_VEL_J_Bi0ToRefInRef;
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                parBlocks + VEL1_OFFSET, vel1IU.second, velDtInv, &LIN_VEL_I_Bi0ToRefInRef);
            ns_ctraj::CeresSplineHelper<Order>::template Evaluate<T, 3, 0>(
                parBlocks + VEL2_OFFSET, vel2IU.second, velDtInv, &LIN_VEL_J_Bi0ToRefInRef);

            Eigen::Map<Eigen::Vector3<T>> residuals(sResiduals);
            Eigen::Vector3<T> pred = LIN_VEL_J_Bi0ToRefInRef - LIN_VEL_I_Bi0ToRefInRef - GRAVITY_IN_REF * dt;
            Eigen::Vector3<T> mes = -DEL_VEL_1.cast<T>() * POS_BiInBi0 + DEL_VEL_2.cast<T>();
            residuals = T(weight) * (pred - mes);

            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif // RIVER2_VEL_PREINTEGRATION_FACTOR_HPP
