/**
 * @file estimator.h
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

#ifndef RIVER2_ESTIMATOR_H
#define RIVER2_ESTIMATOR_H

#include "config/configor.h"
#include "core/bias_filter.h"
#include "ctraj/core/spline_bundle.h"
#include "ctraj/core/trajectory_estimator.h"
#include "sensor/imu.h"
#include "sensor/radar.h"

namespace ns_river2
{
  using namespace magic_enum::bitwise_operators;

  struct RiverOptOption
  {
    enum class Option : std::uint32_t
    {
      /**
       * @brief options
       */
      NONE = 1 << 0,
      OPT_SO3 = 1 << 1,
      OPT_VEL = 1 << 2,
      OPT_BA = 1 << 3,
      OPT_BG = 1 << 4,
      OPT_GRAVITY = 1 << 5,
      ALL = OPT_SO3 | OPT_VEL | OPT_BA | OPT_BG | OPT_GRAVITY
    };

    static bool IsOptionWith(Option desired, Option curOption)
    {
      return (desired == (desired & curOption));
    }
  };

  using RiverOpt = RiverOptOption::Option;
  using MapStringVector3dptr = std::map<std::string, std::shared_ptr<Eigen::Vector3d>>;

  class Estimator : public ceres::Problem
  {
  public:
    using Ptr = std::shared_ptr<Estimator>;
    using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;
    using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;

  private:
    Configor::Ptr configor;
    SplineBundleType::Ptr splines;
    std::shared_ptr<Eigen::Vector3d> gravity;
    MapStringVector3dptr bas, bgs;

    // manifolds
    static std::shared_ptr<ceres::EigenQuaternionManifold> QUATER_MANIFOLD;
    static std::shared_ptr<ceres::SphereManifold<3>> GRAVITY_MANIFOLD;

    // involved knots recoder: [spline, knot, count]
    std::map<long, std::map<std::size_t, int>> knotRecoder;

  public:
    Estimator(Configor::Ptr configor, SplineBundleType::Ptr splines,
              const std::shared_ptr<Eigen::Vector3d> &gravity,
              const MapStringVector3dptr &bas,
              const MapStringVector3dptr &bgs);

    static Ptr Create(const Configor::Ptr &configor, const SplineBundleType::Ptr &splines,
                      const std::shared_ptr<Eigen::Vector3d> &gravity,
                      const MapStringVector3dptr &bas,
                      const MapStringVector3dptr &bgs);

    static ceres::Problem::Options DefaultProblemOptions();

    static ceres::Solver::Options
    DefaultSolverOptions(int threadNum = -1, bool toStdout = true, bool useCUDA = false);

    ceres::Solver::Summary Solve(const ceres::Solver::Options &options = Estimator::DefaultSolverOptions());

    Eigen::MatrixXd GetHessianMatrix();

    void ShowKnotStatus() const;

  public:
    void AddGyroMeasurementWithConstBias(const IMUFrame::Ptr &frame, RiverOpt option, double weight,
                                         const std::string &topic);

    void AddVelPIMForGravityRecovery(const Eigen::Matrix3d &deltaVel1,
                                     const Eigen::Vector3d &deltaVel2,
                                     const std::string &imuTopic,
                                     double ti, double tj,
                                     const Eigen::Vector3d &VEL_I_Bi0ToRefInRef,
                                     const Eigen::Vector3d &VEL_J_Bi0ToRefInRef,
                                     RiverOpt option, double weight);

    void AddAcceMeasurementWithConstBias(const IMUFrame::Ptr &frame, RiverOpt option, double weight,
                                         const std::string &topic);

    void AddRadarMeasurement(const RadarTarget::Ptr &radarTar, RiverOpt option, double weight,
                             const std::string &topic);

    ceres::ResidualBlockId AddAcceBiasPriori(const BiasFilter::StatePack &priori, RiverOpt option,
                                             const std::string &topic);

    ceres::ResidualBlockId AddGyroBiasPriori(const BiasFilter::StatePack &priori, RiverOpt option,
                                             const std::string &topic);

    std::vector<ceres::ResidualBlockId> AddVelSplineTailConstraint(RiverOpt option, double weight);

    std::vector<ceres::ResidualBlockId> AddSo3SplineTailConstraint(RiverOpt option, double weight);

    void AddVelocityConstraint(double time, const Eigen::Vector3d &value, RiverOpt option, double weight);

    void AddRotationConstraint(double time, const Sophus::SO3d &value, RiverOpt option, double weight);

    ceres::ResidualBlockId AddVelPreintegrationConstraint(const Eigen::Matrix3d &deltaVel1,
                                                          const Eigen::Vector3d &deltaVel2,
                                                          const std::string &imuTopic,
                                                          double t1, double t2,
                                                          RiverOpt option, double weight);

  protected:
    void AddSo3KnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::So3SplineType &spline,
                         const SplineMetaType &splineMeta, bool setToConst);

    void AddRdKnotsData(std::vector<double *> &paramBlockVec, const SplineBundleType::RdSplineType &spline,
                        const SplineMetaType &splineMeta, bool setToConst);

    static Eigen::MatrixXd CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix);
  };
}

#endif // RIVER2_ESTIMATOR_H
