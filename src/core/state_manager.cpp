/**
 * @file state_manager.cpp
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

#include "core/state_manager.h"

#include "core/estimator.h"
#include "spdlog/stopwatch.h"
#include "view/view_util.h"
#include <utility>

namespace ns_river2
{

  // -------------------
  // static member field
  // -------------------
  std::mutex StateManager::StatesMutex = {};

  StateManager::StateManager(DataManager::Ptr dataMagr, Configor::Ptr configor)
      : dataMagr(std::move(dataMagr)), configor(std::move(configor)), splines(nullptr),
        gravity(std::make_shared<Eigen::Vector3d>(0.0, 0.0, -this->configor->prior.GravityNorm)),
        margInfo(nullptr)
  {
    spdlog::info("'StateManager' has been booted.");
    for (const auto &[imuTopic, _] : this->configor->dataStream.IMUTopics)
    {
      if (bas.find(imuTopic) == bas.end())
      {
        bas[imuTopic] = std::make_shared<Eigen::Vector3d>(0.0, 0.0, 0.0);
      }
      if (bgs.find(imuTopic) == bgs.end())
      {
        bgs[imuTopic] = std::make_shared<Eigen::Vector3d>(0.0, 0.0, 0.0);
      }
    }
    spdlog::info("'StateManager' has been finished.");
  }

  StateManager::Ptr StateManager::Create(const DataManager::Ptr &dataMagr, const Configor::Ptr &configor)
  {
    return std::make_shared<StateManager>(dataMagr, configor);
  }

  void StateManager::Run()
  {
    spdlog::info("'StateManager::Run' has been booted, thread id: {}.", RIVER2_TO_STR(std::this_thread::get_id()));

    ros::Rate rate(configor->preference.IncrementalOptRate);

    while (ros::ok())
    {
      auto status = RiverStatus::GetStatusPackSafely();
      if (RiverStatus::IsWith(RiverStatus::StateManager::Status::ShouldQuit, status.StateMagr))
      {
        spdlog::warn("'StateManager::Run' quits normally.");
        break;
      }

      if (!RiverStatus::IsWith(RiverStatus::StateManager::Status::HasInitialized, status.StateMagr))
      {
        // if this system has not been initialized
        if (RiverStatus::IsWith(RiverStatus::DataManager::Status::RadarTarAryForInitIsReady, status.DataMagr))
        {
          // if this system is ready for initialization
          {
            // 'StateManager' has known this message
            LOCK_RIVER2_STATUS
            RiverStatus::DataManager::CurStatus ^= RiverStatus::DataManager::Status::RadarTarAryForInitIsReady;
          }
          spdlog::stopwatch sw;
          auto valid = TryPerformInitialization();
          if (valid)
          {
            spdlog::info("total time elapsed in initialization: {} (s).", sw);
            // LOCK_RIVER2_STATUS
            // RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::ShouldQuit;
          }
        }
        else
        {
          spdlog::info("waiting for radar target arrays to perform initialization...");
        }
      }
      else
      {
        spdlog::stopwatch sw;
        // perform incremental optimization
        auto valid = IncrementalOptimization(status);
        if (valid)
        {
          spdlog::info("total time elapsed in current incremental optimization: {} (s).", sw);
        }
      }
      rate.sleep();
      ros::spinOnce();
    }
  }

  /**
   * @brief 进行初始化操作
   *
   * @return true
   * @return false
   */
  bool StateManager::TryPerformInitialization()
  {
    auto radarTarAryForInits = dataMagr->GetRadarTarAryForInitSafely();

    // get start and end time of radar target arrays clocked by the IMU
    double sTimeRadar = static_cast<double>(FLT_MAX);
    double eTimeRadar = (.0);
    for (const auto &[radarTopic, radarTarAryForInit] : radarTarAryForInits)
    {
      if (radarTarAryForInit.size() == 0)
        continue;
      sTimeRadar = std::min(sTimeRadar, radarTarAryForInit.front()->GetTargets().front()->GetTimestamp());
      eTimeRadar = std::max(eTimeRadar, radarTarAryForInit.back()->GetTargets().back()->GetTimestamp());
    }

    auto imuDataSeqs = dataMagr->ExtractIMUDataPieceSafely(sTimeRadar, eTimeRadar);

    // ----------------
    // static condition
    // ----------------
    bool stationary = false;
    for (const auto &[imuTopic, imuDataSeq] : imuDataSeqs)
    {
      if (imuDataSeq.size() == 0)
        continue;
      auto [acceMean, acceVar] = DataManager::AcceMeanVar(imuDataSeq);
      stationary = std::abs(acceMean.norm() - configor->prior.GravityNorm) < 0.01 && acceVar.diagonal().norm() < 0.01;
      if (stationary)
      {
        spdlog::warn(
            "Motion excitation is insufficient!!! IMUTopic: {}, Gravity-removed acceleration average: {:.6f}, variance: {:.6f}"
            ", try to move fast!!!",
            imuTopic,
            std::abs(acceMean.norm() - configor->prior.GravityNorm),
            acceVar.diagonal().norm());
        break;
      }
    }
    if (stationary)
    {
      return false;
    }

    double sTime = static_cast<double>(FLT_MAX);
    double eTime = (.0);
    for (const auto &[_, imuDataSeq] : imuDataSeqs)
    {
      if (imuDataSeq.size() == 0)
        continue;
      sTime = std::min(sTime, imuDataSeq.front()->GetTimestamp());
      eTime = std::max(eTime, imuDataSeq.back()->GetTimestamp());
    }
    sTime = std::max(sTimeRadar, sTime);
    eTime = std::min(eTimeRadar, eTime);

    spdlog::info(
        "'initialization': start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
        sTime, eTime, eTime - sTime);

    if (Configor::Preference::DEBUG_MODE)
    {
      for (const auto &[radarTopic, radarTarAryForInit] : radarTarAryForInits)
      {
        if (radarTarAryForInit.size() == 0)
          continue;
        spdlog::info(
            "radarTopic: {}, 'radarTarAryForInit' size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
            radarTopic, radarTarAryForInit.size(),
            radarTarAryForInit.front()->GetTargets().front()->GetTimestamp(),
            radarTarAryForInit.back()->GetTargets().back()->GetTimestamp(),
            radarTarAryForInit.back()->GetTargets().back()->GetTimestamp() -
                radarTarAryForInit.front()->GetTargets().front()->GetTimestamp());
      }
      for (const auto &[imuTopic, imuDataSeq] : imuDataSeqs)
      {
        if (imuDataSeq.size() == 0)
          continue;
        spdlog::info(
            "imuTopic: {}, involved IMU frame size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
            imuTopic, imuDataSeq.size(),
            imuDataSeq.front()->GetTimestamp(), imuDataSeq.back()->GetTimestamp(),
            imuDataSeq.back()->GetTimestamp() - imuDataSeq.front()->GetTimestamp());
      }
    }

    // create splines
    splines = CreateSplines(sTime, eTime);
    if (Configor::Preference::DEBUG_MODE)
    {
      ShowSplineStatus();
    }

    // init filters
    InitializeBiasFilters(sTime);

    // ---------------------
    // initialize so3 spline
    // ---------------------
    spdlog::stopwatch sw;
    InitializeSO3Spline(imuDataSeqs);
    spdlog::info("initialized rotation spline time elapsed: {} (s).", sw);
    // ViewUtil::ShowSO3Spline(splines->GetSo3Spline(Configor::Preference::SO3Spline), 0.01, sTime, eTime);

    // -------------------------
    // initialize gravity vector
    // -------------------------
    spdlog::stopwatch sw2;
    // the minimum eigen value could represent a good or bad initialization (gravity observability)
    InitializeGravity(radarTarAryForInits, imuDataSeqs);

    // observability condition is good, the gravity has been initialized
    spdlog::info(
        "initialized gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
        (*gravity)(0), (*gravity)(1), (*gravity)(2));
    // ViewUtil::ShowSO3SplineWithGravity(
    //     splines->GetSo3Spline(Configor::Preference::SO3Spline), *gravity, 0.01, sTime, eTime);
    // splines->Save(configor->dataStream.OutputPath + "/splines_so3_init.json");

    // -----------------------
    // recover velocity spline
    // -----------------------
    spdlog::stopwatch sw3;
    auto estimator = InitializeVelSpline(radarTarAryForInits, imuDataSeqs);
    spdlog::info("initialize velocity spline time elapsed: {} (s)", sw3);

    spdlog::info(
        "refined gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
        (*gravity)(0), (*gravity)(1), (*gravity)(2));

    // ------------------
    // update bias filter
    // ------------------
    UpdateBiasFilters(estimator, eTime);

    // -----------------------
    // align states to gravity
    // -----------------------
    AlignInitializedStates();

    spdlog::info(
        "aligned gravity vector: 'gx': {:.6f}, 'gy': {:.6f}, 'gz': {:.6f}",
        (*gravity)(0), (*gravity)(1), (*gravity)(2));

    // ---------------------------------
    // marginalization in initialization
    // ---------------------------------
    spdlog::stopwatch sw4;
    // attention: we need perform marginalization after state alignment,
    // as the old states would be stored in the 'MarginalizationInfo'
    MarginalizationInInit(estimator);
    spdlog::info("marginalization in initialization time elapsed: {} (s)", sw4);

    // splines->Save(configor->dataStream.OutputPath + "/init_splines.json");
    // margInfo->Save(configor->dataStream.OutputPath + "/init_marg_info.json");
    // estimator->ShowKnotStatus();
    // ViewUtil::ShowSO3VelSplineWithGravity(
    //     splines->GetSo3Spline(Configor::Preference::SO3Spline),
    //     splines->GetRdSpline(Configor::Preference::VelSpline), *gravity, 0.01, sTime, eTime);
    // ros::shutdown();

    LOCK_RIVER2_STATUS
    RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::HasInitialized;
    RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::NewStateNeedToDraw;
    RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::NewStateNeedToPublish;
    RiverStatus::StateManager::ValidStateEndTime = eTime;

    return true;
  }

  /**
   * @brief 根据开始和结束时间初始化样条
   *
   * @param sTime
   * @param eTime
   * @return StateManager::SplineBundleType::Ptr
   */
  StateManager::SplineBundleType::Ptr StateManager::CreateSplines(double sTime, double eTime) const
  {
    // create four splines
    auto so3SplineInfo = ns_ctraj::SplineInfo(
        Configor::Preference::SO3Spline, ns_ctraj::SplineType::So3Spline,
        sTime, eTime, configor->prior.SO3SplineKnotDist);
    auto velSplineInfo = ns_ctraj::SplineInfo(
        Configor::Preference::VelSpline, ns_ctraj::SplineType::RdSpline,
        sTime, eTime, configor->prior.VelSplineKnotDist);
    return SplineBundleType::Create({so3SplineInfo, velSplineInfo});
  }

  /**
   * @brief 初始化SO3样条
   *
   * @param imuData
   */
  void StateManager::InitializeSO3Spline(const MapStringListIMUFrameptr &imuDatas)
  {
    // add gyro factors and fit so3 spline
    auto estimator = Estimator::Create(configor, splines, gravity, bas, bgs);
    for (const auto &[imuTopic, imuData] : imuDatas)
    {
      for (const auto &frame : imuData)
      {
        estimator->AddGyroMeasurementWithConstBias(frame, RiverOpt::OPT_SO3,
                                                   configor->prior.GyroWeights[imuTopic], imuTopic);
      }
    }

    // make this problem fun rank
    estimator->SetParameterBlockConstant(
        splines->GetSo3Spline(Configor::Preference::SO3Spline).KnotsFront().data());

    auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

    if (Configor::Preference::DEBUG_MODE)
    {
      spdlog::info("here is the summary of 'InitializeSO3Spline':\n{}\n", sum.BriefReport());
    }
  }

  /**
   * @brief 初始化重力
   *
   * @param radarTarAryVec
   * @param imuData
   */
  void StateManager::InitializeGravity(MapStringListRadarTargetArrayptr &radarTarAryVecs,
                                       const MapStringListIMUFrameptr &imuDatas)
  {
    // obtain the fitted so3 & velocity spline
    const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);

    // timestamp, imu velocity computed from radar measurements
    std::vector<std::pair<double, Eigen::Vector3d>> LIN_VEL_Bi0toRefinRef_VEC;

    // compute the velocity of imu (with respect to the reference frame expressed in the reference frame)
    for (auto &[radarTopic, radarTarAryVec] : radarTarAryVecs)
    {
      // obtain extrinsics
      const auto &SO3_RjtoBi0 = configor->dataStream.CalibParams.EXTRI.SO3_RjToBi0[radarTopic];
      const Eigen::Vector3d &POS_RjInBi0 = configor->dataStream.CalibParams.EXTRI.POS_RjInBi0[radarTopic];

      for (auto &tarAry : radarTarAryVec)
      {
        double t = tarAry->GetTimestamp();

        if (t < so3Spline.MinTime() || t >= so3Spline.MaxTime())
        {
          continue;
        }

        auto SO3_Bi0toRef = so3Spline.Evaluate(t);
        Eigen::Vector3d ANG_VEL_Bi0toRefinRef = SO3_Bi0toRef * so3Spline.VelocityBody(t);
        Sophus::SO3d SO3_RjtoRef = SO3_Bi0toRef * SO3_RjtoBi0;
        if (configor->prior.RadarTargetRANSACs.at(radarTopic))
        {
          tarAry->RadarRemoveOutlier(SO3_RjtoRef,
                                     configor->prior.RadarTargetThresholdErrors.at(radarTopic),
                                     configor->prior.RadarTargetThresholdRatios.at(radarTopic),
                                     configor->prior.RadarTargetOutliersProbs.at(radarTopic),
                                     configor->prior.RadarTargetSuccessProbs.at(radarTopic),
                                     configor->prior.RadarTargetRAVSACPoints.at(radarTopic));
        }
        Eigen::Vector3d LIN_VEL_RjtoRefInRef = tarAry->RadarVelocityFromStaticTargetArray(SO3_RjtoRef, true);
        Eigen::Vector3d LIN_VEL_Bi0toRefinRef =
            LIN_VEL_RjtoRefInRef + Sophus::SO3d::hat(SO3_Bi0toRef * POS_RjInBi0) * ANG_VEL_Bi0toRefinRef;

        LIN_VEL_Bi0toRefinRef_VEC.emplace_back(t, LIN_VEL_Bi0toRefinRef);
      }
    }
    std::sort(LIN_VEL_Bi0toRefinRef_VEC.begin(), LIN_VEL_Bi0toRefinRef_VEC.end(),
              [](const std::pair<double, Eigen::Vector3d> &a, const std::pair<double, Eigen::Vector3d> &b)
              { return a.first < b.first; });

    if (Configor::Preference::DEBUG_MODE)
    {
      for (const auto &[t, vel] : LIN_VEL_Bi0toRefinRef_VEC)
      {
        spdlog::info("'InitializeGravity' time: {:.6f}, LIN_VEL_Bi0toRefinRef: x: {:.6f}, y: {:.6f}, z: {:.6f}.",
                     t, vel.x(), vel.y(), vel.z());
      }
    }

    auto estimator = Estimator::Create(configor, splines, gravity, bas, bgs);
    for (int i = 0; i < static_cast<int>(LIN_VEL_Bi0toRefinRef_VEC.size()) - 1; ++i)
    {
      int j = i + 1;
      const auto &[ti, vi] = LIN_VEL_Bi0toRefinRef_VEC.at(i);
      const auto &[tj, vj] = LIN_VEL_Bi0toRefinRef_VEC.at(j);

      for (const auto &[imuTopic, imuMes] : imuDatas)
      {
        if (imuMes.size() == 0)
          continue;

        auto [sIter, eIter] = ExtractRange(imuMes, ti, tj);
        if ((*sIter)->GetTimestamp() - ti > 0.01 || tj - (*eIter)->GetTimestamp() > 0.01 || tj - ti < 0.01)
          continue;
        std::vector<std::pair<double, Eigen::Matrix3d>> reorganizedSubData1;
        std::vector<std::pair<double, Eigen::Vector3d>> reorganizedSubData2;

        for (auto iter = sIter; iter != eIter; ++iter)
        {
          const auto &frame = *iter;
          double t = frame->GetTimestamp();

          Eigen::Matrix3d m1 = Sophus::SO3d::hat(so3Spline.Evaluate(t) * so3Spline.AccelerationBody(t)).matrix();
          Eigen::Matrix3d m2 = Sophus::SO3d::hat(so3Spline.Evaluate(t) * so3Spline.VelocityBody(t)).matrix();
          Eigen::Matrix3d m3 = (m1 + m2 * m2) * so3Spline.Evaluate(t).matrix();
          reorganizedSubData1.emplace_back(t, m3);

          Eigen::Vector3d v1 =
              so3Spline.Evaluate(t) * configor->dataStream.CalibParams.EXTRI.SO3_BiToBi0[imuTopic] * frame->GetAcce();
          reorganizedSubData2.emplace_back(t, v1);
        }
        estimator->AddVelPIMForGravityRecovery(TrapIntegrationOnce(reorganizedSubData1),
                                               TrapIntegrationOnce(reorganizedSubData2),
                                               imuTopic,
                                               ti, tj, vi, vj, RiverOpt::OPT_GRAVITY, 1.0);
      }
    }
    auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

    if (Configor::Preference::DEBUG_MODE)
    {
      spdlog::info("here is the summary of 'InitializeGravity':\n{}\n", sum.BriefReport());
    }
  }

  /**
   * @brief 初始化速度样条并进行精细化处理
   *
   * @param radarTarAryVec
   * @param imuData
   * @return Estimator::Ptr
   */
  Estimator::Ptr StateManager::InitializeVelSpline(MapStringListRadarTargetArrayptr &radarTarAryVecs,
                                                   const MapStringListIMUFrameptr &imuDatas)
  {
    // ----------------------------------------------------------------------------------------------
    // optimize velocity b-spline using velocity preintegration from linear acceleration measurements
    // ----------------------------------------------------------------------------------------------
    {
      auto estimator = Estimator::Create(configor, splines, gravity, bas, bgs);
      auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
      for (const auto &[imuTopic, imuData] : imuDatas)
      {
        if (imuData.size() == 0)
          continue;

        auto iter1 = imuData.cbegin();
        for (auto iter2 = std::next(iter1); iter2 != imuData.cend(); ++iter2)
        {
          double t1 = (*iter1)->GetTimestamp(), t2 = (*iter2)->GetTimestamp();
          if (t2 - t1 < 0.01)
          {
            continue;
          }
          // velocity preintegration
          std::vector<std::pair<double, Eigen::Matrix3d>> reorganizedSubData1;
          std::vector<std::pair<double, Eigen::Vector3d>> reorganizedSubData2;
          for (auto iter = iter1; iter != std::next(iter2); ++iter)
          {
            const auto &frame = *iter;
            double t = frame->GetTimestamp();

            Eigen::Matrix3d m1 = Sophus::SO3d::hat(so3Spline.Evaluate(t) * so3Spline.AccelerationBody(t)).matrix();
            Eigen::Matrix3d m2 = Sophus::SO3d::hat(so3Spline.Evaluate(t) * so3Spline.VelocityBody(t)).matrix();
            Eigen::Matrix3d m3 = (m1 + m2 * m2) * so3Spline.Evaluate(t).matrix();
            reorganizedSubData1.emplace_back(t, m3);

            Eigen::Vector3d v1 =
                so3Spline.Evaluate(t) * configor->dataStream.CalibParams.EXTRI.SO3_BiToBi0[imuTopic] * frame->GetAcce();
            reorganizedSubData2.emplace_back(t, v1);
          }
          estimator->AddVelPreintegrationConstraint(TrapIntegrationOnce(reorganizedSubData1),
                                                    TrapIntegrationOnce(reorganizedSubData2),
                                                    imuTopic,
                                                    t1, t2, RiverOpt::OPT_VEL, 1.0);
        }
      }
      // maintain observability of the last few control points
      estimator->AddVelSplineTailConstraint(RiverOpt::OPT_VEL, 1000.0);

      // solving
      estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));
      // spdlog::info("'InitializeVelSpline' -----------------------------initial in initial finished-------------------------");
    }

    // add gyro factors and fit so3 spline
    auto estimator = Estimator::Create(configor, splines, gravity, bas, bgs);

    // optimization option in initialization
    RiverOpt option = RiverOpt::OPT_SO3 | RiverOpt::OPT_VEL | RiverOpt::OPT_GRAVITY;

    // acce and gyro factors
    for (const auto &[imuTopic, imuData] : imuDatas)
    {
      for (const auto &frame : imuData)
      {
        estimator->AddGyroMeasurementWithConstBias(frame, option, configor->prior.GyroWeights[imuTopic], imuTopic);

        estimator->AddAcceMeasurementWithConstBias(frame, option, configor->prior.AcceWeights[imuTopic], imuTopic);
      }
    }
    // spdlog::info("'InitializeVelSpline' -----------------------------add IMU raw data.-------------------------");

    // radar factors
    for (const auto &[radarTopic, radarTarAryVec] : radarTarAryVecs)
    {
      for (const auto &ary : radarTarAryVec)
      {
        for (const auto &tar : ary->GetTargets())
        {
          estimator->AddRadarMeasurement(tar, option, configor->prior.RadarWeights.at(radarTopic), radarTopic);
        }
      }
    }
    // spdlog::info("'InitializeVelSpline' -----------------------------add radar targetdata.-------------------------");

    // add a tail constraint to handle the poor observability of the last knot
    auto velTailIdVec = estimator->AddVelSplineTailConstraint(option, 1000.0);
    auto so3TailIdVec = estimator->AddSo3SplineTailConstraint(option, 1000.0);

    // make this problem fun rank
    estimator->SetParameterBlockConstant(
        splines->GetSo3Spline(Configor::Preference::SO3Spline).KnotsFront().data());
    // spdlog::info("'InitializeVelSpline' -----------------------------start solve without ba&bg.-------------------------");

    auto sum = estimator->Solve(Estimator::DefaultSolverOptions(4, Configor::Preference::DEBUG_MODE, false));
    // spdlog::info("'InitializeVelSpline' -----------------------------start solve with ba&bg.-------------------------");

    option |= RiverOpt::OPT_BG | RiverOpt::OPT_BA;
    sum = estimator->Solve(Estimator::DefaultSolverOptions(4, Configor::Preference::DEBUG_MODE, false));

    if (Configor::Preference::DEBUG_MODE)
    {
      spdlog::info("here is the summary of 'InitializeVelSpline':\n{}\n", sum.BriefReport());
    }

    for (const auto &id : velTailIdVec)
    {
      estimator->RemoveResidualBlock(id);
    }
    for (const auto &id : so3TailIdVec)
    {
      estimator->RemoveResidualBlock(id);
    }

    return estimator;
  }

  /**
   * @brief 初始化IMU零偏信息
   *
   * @param sTime
   */
  void StateManager::InitializeBiasFilters(double sTime)
  {
    // ShowSplineStatus();
    {
      // -----------------
      // acceleration bias
      // -----------------
      // the covariance of the initial state is set to 0.01 m/s^2
      for (const auto &[imuTopic, _] : configor->dataStream.IMUTopics)
      {
        if (bas.find(imuTopic) == bas.end())
          break;
        BiasFilter::StatePack initState(sTime, *bas.at(imuTopic), Eigen::Vector3d::Ones() * 0.0001 * 0.0001);
        baFilters[imuTopic] = BiasFilter::Create(initState, configor->prior.AcceBiasRandomWalks[imuTopic]);
        spdlog::info("imu topic: {}, initial state of ba filter: {}", imuTopic, river_to_string(baFilters.at(imuTopic)->GetCurState()));
      }
    }

    {
      // --------------
      // gyroscope bias
      // --------------
      // the covariance of the initial state is set to 0.001 rad/s
      for (const auto &[imuTopic, _] : configor->dataStream.IMUTopics)
      {
        if (bgs.find(imuTopic) == bgs.end())
          break;
        BiasFilter::StatePack initState(sTime, *bgs.at(imuTopic), Eigen::Vector3d::Ones() * 0.00001 * 0.00001);
        bgFilters[imuTopic] = BiasFilter::Create(initState, configor->prior.GyroBiasRandomWalks[imuTopic]);
        spdlog::info("imu topic: {}, initial state of bg filter: {}", imuTopic, river_to_string(bgFilters.at(imuTopic)->GetCurState()));
      }
    }
  }

  /**
   * @brief 计算世界坐标系并进行对齐
   *
   */
  void StateManager::AlignInitializedStates()
  {
    auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
    auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
    // current gravity, velocities, and rotations are expressed in the reference frame
    // align them to the world frame whose negative z axis is aligned with the gravity vector
    auto SO3_RefToW = ObtainAlignedWtoRef(so3Spline.Evaluate(so3Spline.MinTime()), *gravity).inverse();
    *gravity = SO3_RefToW * *gravity;
    for (int i = 0; i < static_cast<int>(so3Spline.GetKnots().size()); ++i)
    {
      so3Spline.GetKnot(i) = SO3_RefToW * so3Spline.GetKnot(i);
    }
    for (int i = 0; i < static_cast<int>(velSpline.GetKnots().size()); ++i)
    {
      velSpline.GetKnot(i) = SO3_RefToW * velSpline.GetKnot(i);
    }
  }

  /**
   * @brief 对固定向量进行边缘化操作
   *
   * @param estimator
   */
  void StateManager::MarginalizationInInit(const Estimator::Ptr &estimator)
  {
    // perform marginalization
    auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
    auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
    int so3KnotSize = static_cast<int>(so3Spline.GetKnots().size());
    int velKnotSize = static_cast<int>(velSpline.GetKnots().size());

    std::set<double *> margParBlocks{};
    margParBlocks.insert(gravity->data());
    /**
     * example, order: 3, knot size: 9, knots from '0'-th to '8'-th
     * | start time for data piece                | end time for data piece
     * [  (O)   (O)   (O)   (O)   (O)   (O)   (O)   (O)   (O) ]
     *     |-----------------|                       | max time for order-4 spline
     *              | the knots which would be marginalized, i.e., from '0'-th to '(kSize - 2 * order)'-th
     *                                                       i.e., from '0'-th to '3'-th
     *  In this way, there would be 5 kept knots.
     */
    for (int i = 0; i < so3KnotSize; ++i)
    {
      if (i <= so3KnotSize - 2 * Configor::Prior::SplineOrder)
      {
        margParBlocks.insert(so3Spline.GetKnot(i).data());
      }
      else
      {
        lastKeepSo3KnotAdd.insert({i, so3Spline.GetKnot(i).data()});
      }
    }
    for (int i = 0; i < velKnotSize; ++i)
    {
      if (i <= velKnotSize - 2 * Configor::Prior::SplineOrder)
      {
        margParBlocks.insert(velSpline.GetKnot(i).data());
      }
      else
      {
        lastKeepVelKnotAdd.insert({i, velSpline.GetKnot(i).data()});
      }
    }

    if (Configor::Preference::DEBUG_MODE)
    {
      spdlog::info("rotation knots: {}, velocity knots: {}", so3KnotSize, velKnotSize);
      spdlog::info(
          "marginalize rotation knots from 0-th to {}-th", so3KnotSize - Configor::Prior::SplineOrder - 1);
      spdlog::info(
          "marginalize velocity knots from 0-th to {}-th", velKnotSize - Configor::Prior::SplineOrder - 1);
    }

    margInfo = ns_ctraj::MarginalizationInfo::Create(estimator.get(), margParBlocks, {}, 2);
  }

  // --------------------------------
  //    top: initialization
  // --------------------------------
  // bottom: incremental optimization
  // --------------------------------

  /**
   * @brief 根据IMU观测数据，对样条进行拓展
   *
   * @param imuData
   */
  void StateManager::PreOptimization(const MapStringListIMUFrameptr &imuDatas)
  {
    auto estimator = Estimator::Create(configor, splines, gravity, bas, bgs);

    // --------------------------
    // optimize rotation b-spline
    // --------------------------
    for (const auto &[imuTopic, imuData] : imuDatas)
    {
      for (const auto &frame : imuData)
      {
        estimator->AddGyroMeasurementWithConstBias(frame, RiverOpt::OPT_SO3, configor->prior.GyroWeights[imuTopic],
                                                   imuTopic);
      }
    }
    // maintain observability of the last few control points
    estimator->AddSo3SplineTailConstraint(RiverOpt::OPT_SO3, 1000.0);

    // solving
    estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

    ns_ctraj::MarginalizationFactor::AddToProblem(estimator.get(), margInfo, 1.0);

    // ----------------------------------------------------------------------------------------------
    // optimize velocity b-spline using velocity preintegration from linear acceleration measurements
    // ----------------------------------------------------------------------------------------------
    auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
    for (const auto &[imuTopic, imuData] : imuDatas)
    {
      auto iter1 = imuData.cbegin();
      for (auto iter2 = std::next(iter1); iter2 != imuData.cend(); ++iter2)
      {
        double t1 = (*iter1)->GetTimestamp(), t2 = (*iter2)->GetTimestamp();
        if (t2 - t1 < 0.01)
        {
          continue;
        }
        // velocity preintegration
        std::vector<std::pair<double, Eigen::Matrix3d>> reorganizedSubData1;
        std::vector<std::pair<double, Eigen::Vector3d>> reorganizedSubData2;
        for (auto iter = iter1; iter != std::next(iter2); ++iter)
        {
          const auto &frame = *iter;
          double t = frame->GetTimestamp();

          Eigen::Matrix3d m1 = Sophus::SO3d::hat(so3Spline.Evaluate(t) * so3Spline.AccelerationBody(t)).matrix();
          Eigen::Matrix3d m2 = Sophus::SO3d::hat(so3Spline.Evaluate(t) * so3Spline.VelocityBody(t)).matrix();
          Eigen::Matrix3d m3 = (m1 + m2 * m2) * so3Spline.Evaluate(t).matrix();
          reorganizedSubData1.emplace_back(t, m3);

          Eigen::Vector3d v1 =
              so3Spline.Evaluate(t) * configor->dataStream.CalibParams.EXTRI.SO3_BiToBi0[imuTopic] * frame->GetAcce();
          reorganizedSubData2.emplace_back(t, v1);
        }
        estimator->AddVelPreintegrationConstraint(TrapIntegrationOnce(reorganizedSubData1),
                                                  TrapIntegrationOnce(reorganizedSubData2),
                                                  imuTopic,
                                                  t1, t2, RiverOpt::OPT_VEL, 1.0);
      }
    }
    // maintain observability of the last few control points
    estimator->AddVelSplineTailConstraint(RiverOpt::OPT_VEL, 1000.0);

    // solving
    estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));
  }

  /**
   * @brief 进行优化操作
   *
   * @param status
   * @return true
   * @return false
   */
  bool StateManager::IncrementalOptimization(const RiverStatus::StatusPack &status)
  {
    // set start time as the end time of last optimization
    const double sTime = status.ValidStateEndTime, eTime = dataMagr->GetEndTimeSafely();

    if (eTime - sTime < 1.0 / configor->preference.IncrementalOptRate)
    {
      return false;
    }

    spdlog::info(
        "'incremental optimization': start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
        sTime, eTime, eTime - sTime);

    // ------------
    // extract data
    // ------------

    auto imuDatas = dataMagr->ExtractIMUDataPieceSafely(sTime, eTime);
    auto radarDatas = dataMagr->ExtractRadarDataPieceSafely(sTime, eTime);
    // spdlog::info("radarDatas size: {d}.", radarDatas.size());

    if (Configor::Preference::DEBUG_MODE)
    {
      for (const auto &[imuTopic, imuData] : imuDatas)
      {
        if (imuData.size() == 0)
          continue;
        spdlog::info(
            "imu topic: {}, 'imuData' size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
            imuTopic,
            imuData.size(), imuData.front()->GetTimestamp(), imuData.back()->GetTimestamp(),
            imuData.back()->GetTimestamp() - imuData.front()->GetTimestamp());
      }
      for (const auto &[radarTopic, radarData] : radarDatas)
      {
        for (const auto &ary : radarData)
        {
          const auto &targets = ary->GetTargets();
          if (targets.size() == 0)
            continue;
          spdlog::info(
              "radar topic: {}, 'radarData' size: {}, start time: {:.6f}, end time: {:.6f}, duration: {:.6f}",
              radarTopic,
              targets.size(), targets.front()->GetTimestamp(), targets.back()->GetTimestamp(),
              targets.back()->GetTimestamp() - targets.front()->GetTimestamp());
        }
      }
    }

    // --------------
    // quit condition
    // --------------
    const double dt = eTime - sTime;
    double imuFreq = (.0);
    double radarFreq = (.0);
    for (const auto &[imuTopic, imuData] : imuDatas)
    {
      imuFreq = std::max(imuFreq, static_cast<double>(imuData.size()) / dt);
    }
    for (const auto &[radarTopic, radarData] : radarDatas)
    {
      int targetSize = 0;
      for (const auto &ary : radarData)
      {
        targetSize += ary->GetTargets().size();
      }
      radarFreq = std::max(radarFreq, static_cast<double>(targetSize) / dt);
    }

    if (Configor::Preference::DEBUG_MODE)
    {
      spdlog::info("imu frequency: {:.6f} hz, radar frequency: {:.6f} hz.", imuFreq, radarFreq);
    }
    if (imuFreq < 50 || radarFreq < 10)
    {
      LOCK_RIVER2_STATUS
      RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::ShouldQuit;
      return false;
    }

    // ----------------
    // static condition
    // ----------------
    LOCK_STATES
    auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
    auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);

    bool stationary(false);
    // double velocity = velSpline.Evaluate(status.ValidStateEndTime - 1E-3).norm();
    for (const auto &[radarTopic, radarData] : radarDatas)
    {
      for (const auto &ary : radarData)
      {
        stationary = stationary && ary->IfZeroVelocity(configor->prior.RadarTargetOutliersProbs.at(radarTopic),
                                                       configor->prior.RadarZeroVelocityThresholds.at(radarTopic));
      }
      if (stationary)
      {
        for (const auto &[imuTopic, imuData] : imuDatas)
        {
          auto [acceMean, acceVar] = DataManager::AcceMeanVar(imuData);
          auto [gyroMean, gyroVar] = DataManager::GyroMeanVar(imuData);
          stationary = acceVar.diagonal().norm() < configor->prior.AcceZeroVelocityThresholds.at(imuTopic) &&
                       gyroVar.diagonal().norm() < configor->prior.GyroZeroVelocityThresholds.at(imuTopic);
          if (stationary)
          {
            spdlog::warn(
                "radar topic: {}, imu topic: {}, Motion excitation is insufficient!!! variance: {:.6f}"
                ", try to move fast!!!",
                radarTopic,
                imuTopic,
                acceVar.diagonal().norm());
            break;
          }
        }
      }
    }
    if (stationary && !configor->prior.ZeroVelocityUpdate)
    {
      // todo: when the body is static, not just quit simply
      // LOCK_RIVER2_STATUS
      // RiverStatus::StateManager::CurStatus ^= RiverStatus::StateManager::Status::HasInitialized;
      return false;
    }

    // --------------
    // extend splines
    // --------------
    int so3KnotOldSize = static_cast<int>(so3Spline.GetKnots().size());
    int velKnotOldSize = static_cast<int>(velSpline.GetKnots().size());

    if (Configor::Preference::DEBUG_MODE)
    {
      ShowSplineStatus();
    }
    // ViewUtil::ShowSO3VelSplineWithGravity(
    //     so3Spline, velSpline, *gravity, 0.01, so3Spline.MinTime(), sTime);

    // --------------------
    // linear extrapolation
    // --------------------
    LinearExtendKnotTo(so3Spline, eTime + 1E-6);
    LinearExtendKnotTo(velSpline, eTime + 1E-6);

    // obtain the updated address
    std::map<double *, double *> updatedLastKeepKnotAdd;
    for (const auto &[idx, add] : lastKeepSo3KnotAdd)
    {
      updatedLastKeepKnotAdd.insert({add, so3Spline.GetKnot(idx).data()});
    }
    for (const auto &[idx, add] : lastKeepVelKnotAdd)
    {
      updatedLastKeepKnotAdd.insert({add, velSpline.GetKnot(idx).data()});
    }
    margInfo->ShiftKeepParBlockAddress(updatedLastKeepKnotAdd);

    if (Configor::Preference::DEBUG_MODE)
    {
      int parShiftCount = 0;
      for (const auto &[oldAdd, newAdd] : updatedLastKeepKnotAdd)
      {
        if (oldAdd != newAdd)
        {
          ++parShiftCount;
        }
      }
      spdlog::error(
          "size of 'updatedLastKeepKnotAdd': {}, count of shifted parameter blocks: {}.",
          updatedLastKeepKnotAdd.size(), parShiftCount);
      // just for debug
      // if (parShiftCount != 0) { ros::shutdown(); }
      ShowSplineStatus();
    }

    // ViewUtil::ShowSO3VelSplineWithGravity(
    //     so3Spline, velSpline, *gravity, 0.01, so3Spline.MinTime(), eTime);

    // ------------------------
    // incremental optimization
    // ------------------------

    PreOptimization(imuDatas);

    // add gyro factors and fit so3 spline
    auto estimator = Estimator::Create(configor, splines, gravity, bas, bgs);

    // marginalization factor
    ns_ctraj::MarginalizationFactor::AddToProblem(estimator.get(), margInfo, 1.0);

    // optimization options for three kinds of factors
    RiverOpt option = RiverOpt::OPT_SO3 | RiverOpt::OPT_VEL | RiverOpt::OPT_BA | RiverOpt::OPT_BG;

    // acce and gyro factors
    std::vector<ceres::ResidualBlockId> bgPrioriIdlist, baPrioriIdlist;
    for (const auto &[imuTopic, imuData] : imuDatas)
    {
      for (const auto &frame : imuData)
      {
        estimator->AddGyroMeasurementWithConstBias(frame, option, configor->prior.GyroWeights[imuTopic], imuTopic);
        estimator->AddAcceMeasurementWithConstBias(frame, option, configor->prior.AcceWeights[imuTopic], imuTopic);
      }
      bgPrioriIdlist.push_back(estimator->AddGyroBiasPriori(bgFilters.at(imuTopic)->Prediction(eTime), option, imuTopic));
      baPrioriIdlist.push_back(estimator->AddAcceBiasPriori(baFilters.at(imuTopic)->Prediction(eTime), option, imuTopic));
    }

    // radar factors
    if (!stationary)
    {
      for (const auto &[radarTopic, radarData] : radarDatas)
      {
        for (const auto &ary : radarData)
        {
          if (configor->prior.RadarTargetRANSACs.at(radarTopic))
          {
            double timetemp = ary->GetTimestamp();
            if (timetemp > so3Spline.MinTime() && timetemp < so3Spline.MaxTime())
            {
              auto SO3_Bi0toRef = so3Spline.Evaluate(timetemp);
              Sophus::SO3d SO3_RjtoRef = SO3_Bi0toRef * configor->dataStream.CalibParams.EXTRI.SO3_RjToBi0[radarTopic];
              ary->RadarRemoveOutlier(SO3_RjtoRef,
                                      configor->prior.RadarTargetThresholdErrors.at(radarTopic),
                                      configor->prior.RadarTargetThresholdRatios.at(radarTopic),
                                      configor->prior.RadarTargetOutliersProbs.at(radarTopic),
                                      configor->prior.RadarTargetSuccessProbs.at(radarTopic),
                                      configor->prior.RadarTargetRAVSACPoints.at(radarTopic));
            }
          }
          for (const auto &tar : ary->GetTargets())
          {
            estimator->AddRadarMeasurement(tar, option, configor->prior.RadarWeights.at(radarTopic), radarTopic);
          }
        }
      }
    }
    else if (stationary && configor->prior.ZeroVelocityUpdate)
    {
      // spdlog::warn("\n");
      // auto rotation = so3Spline.Evaluate(status.ValidStateEndTime - 1E-3);
      // auto velocity = velSpline.Evaluate(status.ValidStateEndTime - 1E-3);
      for (double t = sTime; t <= eTime; t += configor->prior.VelSplineKnotDist)
      {
        estimator->AddVelocityConstraint(t, Eigen::Vector3d::Zero(), option, configor->prior.ZeroVelocityUpdateWeightVel);
      }
      // for (double t = sTime; t <= eTime; t += configor->prior.SO3SplineKnotDist)
      // {
      //   estimator->AddRotationConstraint(t, rotation, option, configor->prior.ZeroVelocityUpdateWeightRot);
      // }
    }

    // ---------------------------------------------------
    // handle the poor observability of the last few knots
    // ---------------------------------------------------
    auto velTailIdVec = estimator->AddVelSplineTailConstraint(option, 1000.0);
    auto so3TailIdVec = estimator->AddSo3SplineTailConstraint(option, 1000.0);

    auto sum = estimator->Solve(Estimator::DefaultSolverOptions(1, Configor::Preference::DEBUG_MODE, false));

    if (Configor::Preference::DEBUG_MODE)
    {
      spdlog::info("here is the summary of 'IncrementalOptimization':\n{}\n", sum.BriefReport());
    }

    // ------------------
    // update bias filter
    // ------------------
    // estimator->ShowKnotStatus();
    UpdateBiasFilters(estimator, eTime);

    // -----------------------
    // perform marginalization
    // -----------------------

    int so3KnotSize = static_cast<int>(so3Spline.GetKnots().size());
    int velKnotSize = static_cast<int>(velSpline.GetKnots().size());
    std::set<double *> margParBlocks{gravity->data()};
    lastKeepSo3KnotAdd.clear(), lastKeepVelKnotAdd.clear();
    /**
     * example, order: 3
     *                 |-------> the knots which would be marginalized this time
     * [  (O)   (O)   (O)   (O)   (O) ... ]
     *     |-----| the knots which had been marginalized last time.
     */
    for (int i = std::max(0, so3KnotOldSize - 2 * Configor::Prior::SplineOrder + 1); i < so3KnotSize; ++i)
    {
      if (i <= so3KnotSize - 2 * Configor::Prior::SplineOrder)
      {
        margParBlocks.insert(so3Spline.GetKnot(i).data());
      }
      else
      {
        lastKeepSo3KnotAdd.insert({i, so3Spline.GetKnot(i).data()});
      }
    }
    for (int i = std::max(0, velKnotOldSize - 2 * Configor::Prior::SplineOrder + 1); i < velKnotSize; ++i)
    {
      if (i <= velKnotSize - 2 * Configor::Prior::SplineOrder)
      {
        margParBlocks.insert(velSpline.GetKnot(i).data());
      }
      else
      {
        lastKeepVelKnotAdd.insert({i, velSpline.GetKnot(i).data()});
      }
    }

    if (Configor::Preference::DEBUG_MODE)
    {
      spdlog::info("old rotation knots: {}, current rotation knots: {}", so3KnotOldSize, so3KnotSize);
      spdlog::info("old velocity knots: {}, current velocity knots: {}", velKnotOldSize, velKnotSize);
      spdlog::info(
          "marginalize rotation knots from {}-th to {}-th",
          so3KnotOldSize - 2 * Configor::Prior::SplineOrder + 1,
          so3KnotSize - 2 * Configor::Prior::SplineOrder);
      spdlog::info(
          "marginalize velocity knots from {}-th to {}-th",
          velKnotOldSize - 2 * Configor::Prior::SplineOrder + 1,
          velKnotSize - 2 * Configor::Prior::SplineOrder);
    }

    // remove bias prior factors before perform 'Marginalization', i.e., remove the prior info for bias
    // as they are treated as random walk process, not constants. In other words, their prior in the next optimization
    // is from the filter, not the marginalization factor
    for (const auto &id : bgPrioriIdlist)
    {
      estimator->RemoveResidualBlock(id);
    }
    for (const auto &id : baPrioriIdlist)
    {
      estimator->RemoveResidualBlock(id);
    }
    // remove linear extrapolation factor as they are not come from real-sensor measurements
    for (const auto &id : velTailIdVec)
    {
      estimator->RemoveResidualBlock(id);
    }
    for (const auto &id : so3TailIdVec)
    {
      estimator->RemoveResidualBlock(id);
    }

    margInfo = ns_ctraj::MarginalizationInfo::Create(estimator.get(), margParBlocks, {}, 1);

    LOCK_RIVER2_STATUS
    RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::NewStateNeedToDraw;
    RiverStatus::StateManager::CurStatus |= RiverStatus::StateManager::Status::NewStateNeedToPublish;
    RiverStatus::StateManager::ValidStateEndTime = eTime;

    return true;
  }

  /**
   * @brief 获取某时刻状态向量
   * std::optional<T> 可以返回T类型，也可以不返回
   *
   * @param t 输入时间
   * @return std::optional<StateManager::StatePack>
   */
  std::optional<StateManager::StatePack> StateManager::GetStatePackSafely(double t) const
  {
    LOCK_STATES
    if (splines == nullptr)
    {
      return {};
    }

    auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
    if (!splines->TimeInRange(t, so3Spline))
    {
      return {};
    }

    auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
    if (!splines->TimeInRange(t, velSpline))
    {
      return {};
    }

    StatePack pack;
    pack.timestamp = t;
    for (const auto &[imuTopic, _] : configor->dataStream.IMUTopics)
    {
      pack.bas[imuTopic] = *bas.at(imuTopic);
      pack.bgs[imuTopic] = *bgs.at(imuTopic);
    }
    pack.gravity = *gravity;
    pack.SO3_Bi0ToRef = so3Spline.Evaluate(t);
    pack.LIN_VEL_Bi0ToRefInRef = velSpline.Evaluate(t);

    return pack;
  }

  std::vector<std::optional<StateManager::StatePack>>
  StateManager::GetStatePackSafely(const std::vector<double> &times) const
  {
    std::vector<std::optional<StateManager::StatePack>> packs(times.size());
    LOCK_STATES
    if (splines == nullptr)
    {
      return packs;
    }
    auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
    auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);

    for (int i = 0; i < static_cast<int>(times.size()); ++i)
    {
      double t = times.at(i);
      if (!splines->TimeInRange(t, so3Spline) || !splines->TimeInRange(t, velSpline))
      {
        packs.at(i) = {};
      }
      else
      {
        StatePack pack;
        pack.timestamp = t;
        for (const auto &[imuTopic, _] : configor->dataStream.IMUTopics)
        {
          pack.bas[imuTopic] = *bas.at(imuTopic);
          pack.bgs[imuTopic] = *bgs.at(imuTopic);
        }
        pack.gravity = *gravity;
        pack.SO3_Bi0ToRef = so3Spline.Evaluate(t);
        pack.LIN_VEL_Bi0ToRefInRef = velSpline.Evaluate(t);

        packs.at(i) = pack;
      }
    }
    return packs;
  }

  const StateManager::SplineBundleType::Ptr &StateManager::GetSplines() const
  {
    return splines;
  }

  void StateManager::ShowSplineStatus() const
  {
    auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3Spline);
    auto &velSpline = splines->GetRdSpline(Configor::Preference::VelSpline);
    spdlog::info(
        "[Rotation Spline] min: '{:.6f}', max: '{:.6f}', dt: '{:.6f}', size: '{}'",
        so3Spline.MinTime(), so3Spline.MaxTime(), so3Spline.GetTimeInterval(), so3Spline.GetKnots().size());
    spdlog::info(
        "[Velocity Spline] min: '{:.6f}', max: '{:.6f}', dt: '{:.6f}', size: '{}'",
        velSpline.MinTime(), velSpline.MaxTime(), velSpline.GetTimeInterval(), velSpline.GetKnots().size());
  }

  /**
   * @brief 取出更新后的零偏
   *
   * @param estimator
   * @param eTime
   */
  void StateManager::UpdateBiasFilters(const Estimator::Ptr &estimator, double eTime)
  {
    for (const auto &[imuTopic, _] : configor->dataStream.IMUTopics)
    {
      auto baCovOpt = ObtainVarMatFromEstimator<3, 3>({bas.at(imuTopic)->data(), bas.at(imuTopic)->data()}, estimator);
      if (baCovOpt != std::nullopt)
      {
        Eigen::Vector3d baCov = baCovOpt->diagonal();
        baFilters.at(imuTopic)->UpdateByEstimator(BiasFilter::StatePack(eTime, *bas.at(imuTopic), baCov));
        spdlog::info("current state of ba filter: {}", river_to_string(baFilters.at(imuTopic)->GetCurState()));
      }

      auto bgCovOpt = ObtainVarMatFromEstimator<3, 3>({bgs.at(imuTopic)->data(), bgs.at(imuTopic)->data()}, estimator);
      if (bgCovOpt != std::nullopt)
      {
        Eigen::Vector3d bgCov = bgCovOpt->diagonal();
        bgFilters.at(imuTopic)->UpdateByEstimator(BiasFilter::StatePack(eTime, *bgs.at(imuTopic), bgCov));
        spdlog::info("current state of bg filter: {}", river_to_string(bgFilters.at(imuTopic)->GetCurState()));
      }
    }
  }

  const std::map<std::string, BiasFilter::Ptr> &StateManager::GetBaFilter() const
  {
    return baFilters;
  }

  const std::map<std::string, BiasFilter::Ptr> &StateManager::GetBgFilter() const
  {
    return bgFilters;
  }

  /**
   * @brief 对样条进行延拓
   *
   * @param spline
   * @param t
   */
  void StateManager::LinearExtendKnotTo(SplineBundleType::RdSplineType &spline, double t)
  {
    Eigen::Vector3d delta = spline.GetKnots().back() - spline.GetKnots().at(spline.GetKnots().size() - 2);
    while ((spline.GetKnots().size() < SplineBundleType::N) || (spline.MaxTime() < t))
    {
      spline.KnotsPushBack(spline.GetKnots().back() + delta);
    }
  }

  /**
   * @brief 对样条进行延拓
   *
   * @param spline
   * @param t
   */
  void StateManager::LinearExtendKnotTo(SplineBundleType::So3SplineType &spline, double t)
  {
    Sophus::SO3d delta =
        spline.GetKnots().at(spline.GetKnots().size() - 2).inverse() * spline.GetKnots().back();
    while ((spline.GetKnots().size() < SplineBundleType::N) || (spline.MaxTime() < t))
    {
      spline.KnotsPushBack(spline.GetKnots().back() * delta);
    }
  }

  StateManager::StatePack::StatePack(double timestamp, const Sophus::SO3d &so3Bi0ToRef,
                                     Eigen::Vector3d linVelBi0ToRefInRef, Eigen::Vector3d gravity,
                                     std::map<std::string, Eigen::Vector3d> bas,
                                     std::map<std::string, Eigen::Vector3d> bgs)
      : timestamp(timestamp), SO3_Bi0ToRef(so3Bi0ToRef), LIN_VEL_Bi0ToRefInRef(std::move(linVelBi0ToRefInRef)),
        gravity(std::move(gravity)), bas(std::move(bas)), bgs(std::move(bgs)) {}

  Eigen::Vector3d StateManager::StatePack::LIN_VEL_Bi0ToRefInCur() const
  {
    return SO3_Bi0ToRef.inverse() * LIN_VEL_Bi0ToRefInRef;
  }

  StateManager::StatePack::StatePack() = default;
} // namespace ns_river2