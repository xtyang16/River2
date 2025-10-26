/**
 * @file status.h
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

#ifndef RIVER2_STATUS_H
#define RIVER2_STATUS_H

#include "exception"
#include "string"
#include "util/enum_cast.hpp"
#include "mutex"

namespace ns_river2
{
  struct Status : std::exception
  {
    enum class Flag
    {
      FINE,
      WARNING,
      ERROR,
      CRITICAL
    };

  public:
    Flag flag;
    std::string what;

    Status(Flag flag, std::string what) : flag(flag), what(std::move(what)) {}

    Status() : flag(Flag::FINE), what() {}
  };

  using namespace magic_enum::bitwise_operators;

#define LOCK_RIVER2_STATUS std::unique_lock<std::mutex> statusLock(RiverStatus::StatusMutex);

  class RiverStatus
  {
  public:
    struct StateManager
    {
      enum class Status : std::uint32_t
      {
        /**
         * @brief options
         */
        NONE = 1 << 0,
        HasInitialized = 1 << 1,
        ShouldQuit = 1 << 2,
        NewStateNeedToDraw = 1 << 3,
        NewStateNeedToPublish = 1 << 4,
      };

      static Status CurStatus; // record corrent statu

      static double ValidStateEndTime;
    };

    struct DataManager
    {
      enum class Status : std::uint32_t
      {
        /**
         * @brief options
         */
        NONE = 1 << 0,
        RadarTarAryForInitIsReady = 1 << 1,
      };

      static Status CurStatus;
    };

    struct StatusPack
    {
    public:
      StateManager::Status StateMagr;
      double ValidStateEndTime;

      DataManager::Status DataMagr;

    public:
      StatusPack(StateManager::Status stateMagr, double ValidStateEndTime, DataManager::Status dataMagr);
    };

  public:
    static std::mutex StatusMutex;

  public:
    template <class EnumType>
    static bool IsWith(EnumType desired, EnumType current)
    {
      return (desired == (desired & current));
    }

    static StatusPack GetStatusPackSafely();
  };
}

#endif // RIVER2_STATUS_H
