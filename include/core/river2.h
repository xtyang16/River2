/**
 * @file river2.h
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

#ifndef RIVER2_RIVER2_H
#define RIVER2_RIVER2_H

#include "config/configor.h"
#include "ros/ros.h"
#include "core/state_manager.h"
#include "view/viewer.h"

namespace ns_river2
{
  class River
  {
  public:
    using Ptr = std::shared_ptr<River>;

  private:
    // ros-related members
    ros::NodeHandlePtr handler;
    Configor::Ptr configor;

    ros::Publisher statePublisher;

    DataManager::Ptr dataMagr;
    StateManager::Ptr stateMagr;

    Viewer::Ptr viewer;

    std::shared_ptr<std::thread> stateMagrThread;
    std::shared_ptr<std::thread> viewerThread;

  public:
    explicit River(const Configor::Ptr &configor);

    static Ptr Create(const Configor::Ptr &configor);

    void Run();

    void Save();

  protected:
    void PublishRiverState(const RiverStatus::StatusPack &status);
  };
}
#endif /// RIVER2_RIVER2_H