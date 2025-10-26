/**
 * @file main.cpp
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

#include "ros/ros.h"
#include "thread"
#include "core/status.h"
#include "core/river2.h"
#include "view/view_util.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "river2_prog_node");
    try
    {
        // obtain the path of config file
        std::string configPath;
        if (!ros::param::get("/river2_prog_node/config_path", configPath))
        {
            throw ns_river2::Status(
                ns_river2::Status::Flag::CRITICAL,
                "the configure path couldn't obtained from ros param '/river2_prog_node/config_path'.");
        }
        spdlog::info("loading configure from json file '{}'...", configPath);

        // load the configure file
        auto configor = ns_river2::Configor::Load(configPath);
        configor->PrintMainFields();

        // config the output path of viewer (just for debug)
        ns_river2::ViewUtil::DataOutputPath = configor->dataStream.OutputPath;

        // create the 'River'
        auto river = ns_river2::River::Create(configor);

        // perform solving
        river->Run();

        // save results
        river->Save();
    }
    catch (const ns_river2::Status &status)
    {
        // if error happened, print it
        switch (status.flag)
        {
        case ns_river2::Status::Flag::FINE:
            // this case usually won't happen
            spdlog::info(status.what);
            break;
        case ns_river2::Status::Flag::WARNING:
            spdlog::warn(status.what);
            break;
        case ns_river2::Status::Flag::ERROR:
            spdlog::error(status.what);
            break;
        case ns_river2::Status::Flag::CRITICAL:
            spdlog::critical(status.what);
            break;
        }
    }
    catch (const std::exception &e)
    {
        // an unknown exception not thrown by this program
        spdlog::critical(e.what());
    }
    ros::shutdown();
    return 0;
}