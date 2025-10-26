/**
 * @file estimator.cpp
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

#include <utility>
#include "core/estimator.h"
#include "factor/imu_gyro_factor.hpp"
#include "factor/gravity_factor.hpp"
#include "factor/imu_acce_factor.hpp"
#include "factor/radar_factor.hpp"
#include "factor/bias_factor.hpp"
#include "factor/tail_factor.hpp"
#include "factor/spline_factor.hpp"
#include "factor/vel_preintegration_factor.hpp"

namespace ns_river2
{
  std::shared_ptr<ceres::EigenQuaternionManifold> Estimator::QUATER_MANIFOLD(new ceres::EigenQuaternionManifold());
  std::shared_ptr<ceres::SphereManifold<3>> Estimator::GRAVITY_MANIFOLD(new ceres::SphereManifold<3>());

  ceres::Problem::Options Estimator::DefaultProblemOptions()
  {
    return ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultProblemOptions();
  }

  ceres::Solver::Options Estimator::DefaultSolverOptions(int threadNum, bool toStdout, bool useCUDA)
  {
    auto defaultSolverOptions = ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultSolverOptions(
        threadNum, toStdout, useCUDA);
    if (!useCUDA)
    {
      defaultSolverOptions.linear_solver_type = ceres::DENSE_SCHUR;
    }
    defaultSolverOptions.trust_region_strategy_type = ceres::DOGLEG;
    if (!toStdout)
    {
      defaultSolverOptions.logging_type = ceres::SILENT;
    }
    return defaultSolverOptions;
  }

  Estimator::Estimator(Configor::Ptr configor, SplineBundleType::Ptr splines,
                       const std::shared_ptr<Eigen::Vector3d> &gravity,
                       const MapStringVector3dptr &bas,
                       const MapStringVector3dptr &bgs)
      : ceres::Problem(DefaultProblemOptions()), configor(std::move(configor)),
        splines(std::move(splines)), gravity(gravity), bas(bas), bgs(bgs) {}

  Estimator::Ptr Estimator::Create(const Configor::Ptr &configor, const SplineBundleType::Ptr &splines,
                                   const std::shared_ptr<Eigen::Vector3d> &gravity,
                                   const MapStringVector3dptr &bas,
                                   const MapStringVector3dptr &bgs)
  {
    return std::make_shared<Estimator>(configor, splines, gravity, bas, bgs);
  }

  ceres::Solver::Summary Estimator::Solve(const ceres::Solver::Options &options)
  {
    ceres::Solver::Summary summary;
    ceres::Solve(options, this, &summary);
    return summary;
  }

  void Estimator::AddRdKnotsData(std::vector<double *> &paramBlockVec,
                                 const Estimator::SplineBundleType::RdSplineType &spline,
                                 const Estimator::SplineMetaType &splineMeta, bool setToConst)
  {
    // for each segment
    for (const auto &seg : splineMeta.segments)
    {
      // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
      auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

      // from the first control point to the last control point
      for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i)
      {
        auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());

        this->AddParameterBlock(data, 3);

        paramBlockVec.push_back(data);
        // set this param block to be constant
        if (setToConst)
        {
          this->SetParameterBlockConstant(data);
        }

        // knot recoder
        knotRecoder[reinterpret_cast<long>(&spline)][i]++;
      }
    }
  }

  void Estimator::AddSo3KnotsData(std::vector<double *> &paramBlockVec,
                                  const Estimator::SplineBundleType::So3SplineType &spline,
                                  const Estimator::SplineMetaType &splineMeta, bool setToConst)
  {
    // for each segment
    for (const auto &seg : splineMeta.segments)
    {
      // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
      auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

      // from the first control point to the last control point
      for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i)
      {
        auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());
        // the local parameterization is very important!!!
        this->AddParameterBlock(data, 4, QUATER_MANIFOLD.get());

        paramBlockVec.push_back(data);
        // set this param block to be constant
        if (setToConst)
        {
          this->SetParameterBlockConstant(data);
        }

        // knot recoder
        knotRecoder[reinterpret_cast<long>(&spline)][i]++;
      }
    }
  }

  /**
   * param blocks:
   * [ GRAVITY ]
   */
  void Estimator::AddVelPIMForGravityRecovery(const Eigen::Matrix3d &deltaVel1,
                                              const Eigen::Vector3d &deltaVel2,
                                              const std::string &imuTopic,
                                              double ti, double tj,
                                              const Eigen::Vector3d &VEL_I_Bi0ToRefInRef,
                                              const Eigen::Vector3d &VEL_J_Bi0ToRefInRef,
                                              RiverOpt option, double weight)
  {
    // approximate
    auto costFunc = GravityFactor::Create(tj - ti,
                                          deltaVel1,
                                          deltaVel2,
                                          VEL_I_Bi0ToRefInRef, VEL_J_Bi0ToRefInRef,
                                          configor->dataStream.CalibParams.EXTRI.POS_BiInBi0[imuTopic],
                                          weight);

    // gravity
    costFunc->AddParameterBlock(3);

    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;
    paramBlockVec.push_back(gravity->data());

    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

    if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_GRAVITY, option))
    {
      this->SetParameterBlockConstant(gravity->data());
    }
  }

  Eigen::MatrixXd Estimator::CRSMatrix2EigenMatrix(ceres::CRSMatrix *jacobian_crs_matrix)
  {
    Eigen::MatrixXd J(jacobian_crs_matrix->num_rows, jacobian_crs_matrix->num_cols);
    J.setZero();

    std::vector<int> jacobian_crs_matrix_rows, jacobian_crs_matrix_cols;
    std::vector<double> jacobian_crs_matrix_values;
    jacobian_crs_matrix_rows = jacobian_crs_matrix->rows;
    jacobian_crs_matrix_cols = jacobian_crs_matrix->cols;
    jacobian_crs_matrix_values = jacobian_crs_matrix->values;

    int cur_index_in_cols_and_values = 0;
    // rows is a num_rows + 1 sized array
    int row_size = static_cast<int>(jacobian_crs_matrix_rows.size()) - 1;
    // outer loop traverse rows, inner loop traverse cols and values
    for (int row_index = 0; row_index < row_size; ++row_index)
    {
      while (cur_index_in_cols_and_values < jacobian_crs_matrix_rows[row_index + 1])
      {
        J(row_index, jacobian_crs_matrix_cols[cur_index_in_cols_and_values]) =
            jacobian_crs_matrix_values[cur_index_in_cols_and_values];
        cur_index_in_cols_and_values++;
      }
    }
    return J;
  }

  Eigen::MatrixXd Estimator::GetHessianMatrix()
  {
    ceres::Problem::EvaluateOptions EvalOpts;
    ceres::CRSMatrix jacobian_crs_matrix;
    this->Evaluate(EvalOpts, nullptr, nullptr, nullptr, &jacobian_crs_matrix);
    Eigen::MatrixXd J = CRSMatrix2EigenMatrix(&jacobian_crs_matrix);
    Eigen::MatrixXd H = J.transpose() * J;
    return H;
  }

  /**
   * param blocks:
   * [ SO3 | ... | SO3 | VEL | ... | VEL ]
   */
  void Estimator::AddRadarMeasurement(const RadarTarget::Ptr &radarTar, RiverOpt option, double weight,
                                      const std::string &topic)
  {
    auto time = radarTar->GetTimestamp();

    if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline))
    {
      // if this frame is not in range
      return;
    }
    if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline))
    {
      // if this frame is not in range
      return;
    }

    // prepare metas for splines
    SplineMetaType so3Meta, velMeta;
    splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
    splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta);

    auto costFunc = RadarFactor<Configor::Prior::SplineOrder>::Create(radarTar, so3Meta, velMeta, weight,
                                                                      configor->dataStream.CalibParams.EXTRI.SO3_RjToBi0[topic],
                                                                      configor->dataStream.CalibParams.EXTRI.POS_RjInBi0[topic]);

    // knots of so3 spline
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i)
    {
      costFunc->AddParameterBlock(4);
    }
    // knots of vel spline
    for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i)
    {
      costFunc->AddParameterBlock(3);
    }

    costFunc->SetNumResiduals(1);

    // organize the param block vector
    std::vector<double *> paramBlockVec;
    paramBlockVec.reserve(so3Meta.NumParameters() + velMeta.NumParameters());

    AddSo3KnotsData(
        paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
        !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option));
    AddRdKnotsData(
        paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
        !RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option));

    this->AddResidualBlock(
        costFunc, new ceres::CauchyLoss(configor->prior.CauchyLossForRadarFactors.at(topic) * weight), paramBlockVec);
  }

  /**
   * param blocks:
   * [ SO3 | ... | SO3 | BG ]
   */
  void Estimator::AddGyroMeasurementWithConstBias(const IMUFrame::Ptr &frame, RiverOpt option, double weight,
                                                  const std::string &topic)
  {
    auto time = frame->GetTimestamp();

    if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline))
    {
      // if this frame is not in range
      return;
    }

    // prepare metas for splines
    SplineMetaType so3Meta;
    splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);

    auto costFunc =
        IMUGyroFactorWithConstBias<Configor::Prior::SplineOrder>::Create(so3Meta, frame, weight,
                                                                         configor->dataStream.CalibParams.EXTRI.SO3_BiToBi0[topic]);

    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i)
    {
      costFunc->AddParameterBlock(4);
    }
    // BIAS OF GYRO
    costFunc->AddParameterBlock(3);

    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;
    paramBlockVec.reserve(so3Meta.NumParameters() + 1);

    AddSo3KnotsData(
        paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
        !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option));
    paramBlockVec.push_back(bgs.at(topic)->data());

    // if (Configor::Preference::DEBUG_MODE)
    // {
    //   spdlog::info("imu topic: {}, Bias gyro: {:.6f}, {:.6f}, {:.6f}.", topic,
    //                bgs.at(topic)->data()[0], bgs.at(topic)->data()[1], bgs.at(topic)->data()[2]);
    // }

    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

    if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_BG, option))
    {
      this->SetParameterBlockConstant(bgs.at(topic)->data());
    }
  }

  /**
   * param blocks:
   * [ SO3 | ... | SO3 | VEL | ... | VEL | BA | GRAVITY ]
   */
  void Estimator::AddAcceMeasurementWithConstBias(const IMUFrame::Ptr &frame, RiverOpt option, double weight,
                                                  const std::string &topic)
  {
    auto time = frame->GetTimestamp();

    if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline))
    {
      // if this frame is not in range
      return;
    }
    if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline))
    {
      // if this frame is not in range
      return;
    }

    // prepare metas for splines
    SplineMetaType so3Meta, velMeta;
    splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);
    splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta);

    auto costFunc = IMUAcceFactorWithConstBias<Configor::Prior::SplineOrder>::Create(
        so3Meta, velMeta, frame, weight,
        configor->dataStream.CalibParams.EXTRI.SO3_BiToBi0[topic],
        configor->dataStream.CalibParams.EXTRI.POS_BiInBi0[topic]);

    // knots of so3 spline
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i)
    {
      costFunc->AddParameterBlock(4);
    }
    // knots of vel spline
    for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i)
    {
      costFunc->AddParameterBlock(3);
    }
    // BIAS OF ACCE
    costFunc->AddParameterBlock(3);
    // gravity
    costFunc->AddParameterBlock(3);

    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;
    paramBlockVec.reserve(so3Meta.NumParameters() + velMeta.NumParameters() + 2);

    // knots of so3 spline
    AddSo3KnotsData(
        paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
        !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option));
    // knots of vel spline
    AddRdKnotsData(
        paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
        !RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option));
    // BIAS OF ACCE
    paramBlockVec.push_back(bas.at(topic)->data());

    // if (Configor::Preference::DEBUG_MODE)
    // {
    //   spdlog::info("imu topic: {}, Bias acce: {:.6f}, {:.6f}, {:.6f}.", topic,
    //                bas.at(topic)->data()[0], bas.at(topic)->data()[1], bas.at(topic)->data()[2]);
    // }
    // gravity
    paramBlockVec.push_back(gravity->data());

    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

    if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_BA, option))
    {
      this->SetParameterBlockConstant(bas.at(topic)->data());
    }
    if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_GRAVITY, option))
    {
      this->SetParameterBlockConstant(gravity->data());
    }
  }

  void Estimator::ShowKnotStatus() const
  {
    for (const auto &[splineAddress, knotInfo] : knotRecoder)
    {
      std::stringstream stream;
      stream << "spline: " << splineAddress << ", ";
      for (const auto &[knotId, count] : knotInfo)
      {
        stream << '[' << knotId << ": " << count << "] ";
      }
      spdlog::info("{}", stream.str());
    }
  }

  ceres::ResidualBlockId Estimator::AddAcceBiasPriori(const BiasFilter::StatePack &priori, RiverOpt option,
                                                      const std::string &topic)
  {
    auto costFunc = BiasFactor::Create(priori.state, priori.var);
    costFunc->AddParameterBlock(3);
    costFunc->SetNumResiduals(3);
    auto id = this->AddResidualBlock(costFunc, nullptr, bas.at(topic)->data());

    if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_BA, option))
    {
      this->SetParameterBlockConstant(bas.at(topic)->data());
    }
    return id;
  }

  ceres::ResidualBlockId Estimator::AddGyroBiasPriori(const BiasFilter::StatePack &priori, RiverOpt option,
                                                      const std::string &topic)
  {
    auto costFunc = BiasFactor::Create(priori.state, priori.var);
    costFunc->AddParameterBlock(3);
    costFunc->SetNumResiduals(3);
    auto id = this->AddResidualBlock(costFunc, nullptr, bgs.at(topic)->data());

    if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_BG, option))
    {
      this->SetParameterBlockConstant(bgs.at(topic)->data());
    }
    return id;
  }

  /**
   * param blocks:
   * [ example for three order spline: VEL | VEL | VEL ]
   */
  std::vector<ceres::ResidualBlockId> Estimator::AddVelSplineTailConstraint(RiverOpt option, double weight)
  {
    auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
    std::vector<ceres::ResidualBlockId> idVec;
    for (int j = 0; j < Configor::Prior::SplineOrder - 2; ++j)
    {
      auto costFunc = RdTailFactor::Create(weight);
      costFunc->AddParameterBlock(3);
      costFunc->AddParameterBlock(3);
      costFunc->AddParameterBlock(3);
      costFunc->SetNumResiduals(3);

      // organize the param block vector
      std::vector<double *> paramBlockVec(3);
      for (int i = 0; i < 3; ++i)
      {
        paramBlockVec.at(i) = velSpline.GetKnot(
                                           j + i + static_cast<int>(velSpline.GetKnots().size()) - Configor::Prior::SplineOrder)
                                  .data();
      }

      auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
      idVec.push_back(id);

      if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option))
      {
        for (auto &knot : paramBlockVec)
        {
          this->SetParameterBlockConstant(knot);
        }
      }
    }
    return idVec;
  }

  /**
   * param blocks:
   * [ SO3 | SO3 | SO3 ]
   */
  std::vector<ceres::ResidualBlockId> Estimator::AddSo3SplineTailConstraint(RiverOpt option, double weight)
  {
    auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
    std::vector<ceres::ResidualBlockId> idVec;
    for (int j = 0; j < Configor::Prior::SplineOrder - 2; ++j)
    {
      auto costFunc = So3TailFactor::Create(weight);
      costFunc->AddParameterBlock(4);
      costFunc->AddParameterBlock(4);
      costFunc->AddParameterBlock(4);
      costFunc->SetNumResiduals(3);

      // organize the param block vector
      std::vector<double *> paramBlockVec(3);
      for (int i = 0; i < 3; ++i)
      {
        paramBlockVec.at(i) = so3Spline.GetKnot(
                                           j + i + static_cast<int>(so3Spline.GetKnots().size()) - Configor::Prior::SplineOrder)
                                  .data();
      }

      auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
      idVec.push_back(id);

      for (const auto &item : paramBlockVec)
      {
        this->SetManifold(item, QUATER_MANIFOLD.get());
      }

      if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option))
      {
        for (auto &knot : paramBlockVec)
        {
          this->SetParameterBlockConstant(knot);
        }
      }
    }
    return idVec;
  }

  /**
   * param blocks:
   * [ VEL | ... | VEL ]
   */
  void Estimator::AddVelocityConstraint(double time, const Eigen::Vector3d &value, RiverOpt option, double weight)
  {
    if (!splines->TimeInRangeForRd(time, Configor::Preference::VelSpline))
    {
      // if this frame is not in range
      return;
    }
    // prepare metas for splines
    SplineMetaType velMeta;
    splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{time, time}}, velMeta);

    auto costFunc = RdSplineFactor<Configor::Prior::SplineOrder>::Create(velMeta, time, value, weight);

    // knots of vel spline
    for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i)
    {
      costFunc->AddParameterBlock(3);
    }

    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;
    paramBlockVec.reserve(velMeta.NumParameters());

    // knots of vel spline
    AddRdKnotsData(
        paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
        !RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option));

    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
  }

  void Estimator::AddRotationConstraint(double time, const Sophus::SO3d &value, RiverOpt option, double weight)
  {
    if (!splines->TimeInRangeForSo3(time, Configor::Preference::SO3Spline))
    {
      // if this frame is not in range
      return;
    }
    // prepare metas for splines
    SplineMetaType so3Meta;
    splines->CalculateSo3SplineMeta(Configor::Preference::SO3Spline, {{time, time}}, so3Meta);

    auto costFunc = So3SplineFactor<Configor::Prior::SplineOrder>::Create(so3Meta, time, value, weight);

    // knots of so3 spline
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i)
    {
      costFunc->AddParameterBlock(4);
    }

    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;
    paramBlockVec.reserve(so3Meta.NumParameters());

    // knots of so3 spline
    AddSo3KnotsData(
        paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3Spline), so3Meta,
        !RiverOptOption::IsOptionWith(RiverOpt::OPT_SO3, option));

    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
  }

  /**
   * param blocks:
   * [ VEL | ... | VEL | GRAVITY ]
   */
  ceres::ResidualBlockId Estimator::AddVelPreintegrationConstraint(const Eigen::Matrix3d &deltaVel1,
                                                                   const Eigen::Vector3d &deltaVel2,
                                                                   const std::string &imuTopic,
                                                                   double t1, double t2,
                                                                   RiverOpt option, double weight)
  {
    if (!splines->TimeInRangeForRd(t1, Configor::Preference::VelSpline) ||
        !splines->TimeInRangeForRd(t2, Configor::Preference::VelSpline))
    {
      // if this frame is not in range
      return nullptr;
    }

    // prepare metas for splines
    SplineMetaType velMeta;
    splines->CalculateRdSplineMeta(Configor::Preference::VelSpline, {{t1, t1}, {t2, t2}}, velMeta);

    auto costFunc =
        VelPreintegrationFactor<Configor::Prior::SplineOrder>::Create(velMeta,
                                                                      deltaVel1, deltaVel2, t1, t2, weight,
                                                                      configor->dataStream.CalibParams.EXTRI.POS_BiInBi0[imuTopic]);

    // knots of vel spline
    for (int i = 0; i < static_cast<int>(velMeta.NumParameters()); ++i)
    {
      costFunc->AddParameterBlock(3);
    }

    // gravity
    costFunc->AddParameterBlock(3);

    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;
    paramBlockVec.reserve(velMeta.NumParameters() + 1);

    // knots of vel spline
    AddRdKnotsData(
        paramBlockVec, splines->GetRdSpline(Configor::Preference::VelSpline), velMeta,
        !RiverOptOption::IsOptionWith(RiverOpt::OPT_VEL, option));
    // gravity
    paramBlockVec.push_back(gravity->data());

    auto id = this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    this->SetManifold(gravity->data(), GRAVITY_MANIFOLD.get());

    if (!RiverOptOption::IsOptionWith(RiverOpt::OPT_GRAVITY, option))
    {
      this->SetParameterBlockConstant(gravity->data());
    }
    return id;
  }
}