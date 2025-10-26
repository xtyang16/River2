/**
 * @file view_util.h
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

#ifndef RIVER2_VIEW_UTIL_H
#define RIVER2_VIEW_UTIL_H

#include "ctraj/tiny/viewer.h"
#include "ctraj/core/spline_bundle.h"
#include "config/configor.h"

namespace ns_river2 {
    struct ViewUtil {
    public:
        using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

        static std::string DataOutputPath;

    public:
        static void ShowSO3Spline(const SplineBundleType::So3SplineType &spline, double dt, double st, double et);

        static void ShowSO3SplineWithGravity(const SplineBundleType::So3SplineType &spline,
                                             const Eigen::Vector3d &gravity, double dt, double st, double et);

        static void ShowSO3VelSplineWithGravity(const SplineBundleType::So3SplineType &so3Spline,
                                                const SplineBundleType::RdSplineType &velSpline,
                                                const Eigen::Vector3d &gravity, double dt, double st, double et);
    };
}

#endif //RIVER2_VIEW_UTIL_H
