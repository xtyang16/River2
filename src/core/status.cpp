/**
 * @file status.cpp
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

#include "core/status.h"

namespace ns_river2
{
    // ------------------------
    // static initialized filed
    // ------------------------
    std::mutex RiverStatus::StatusMutex = {};

    RiverStatus::StateManager::Status RiverStatus::StateManager::CurStatus = RiverStatus::StateManager::Status::NONE;
    double RiverStatus::StateManager::ValidStateEndTime = -1.0;

    RiverStatus::DataManager::Status RiverStatus::DataManager::CurStatus = RiverStatus::DataManager::Status::NONE;

    RiverStatus::StatusPack::StatusPack(RiverStatus::StateManager::Status stateMagr, double ValidStateEndTime,
                                        RiverStatus::DataManager::Status dataMagr)
        : StateMagr(stateMagr), ValidStateEndTime(ValidStateEndTime), DataMagr(dataMagr) {}

    RiverStatus::StatusPack RiverStatus::GetStatusPackSafely()
    {
        LOCK_RIVER2_STATUS
        return {StateManager::CurStatus, StateManager::ValidStateEndTime, DataManager::CurStatus};
    }
}