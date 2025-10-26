/**
 * @file viewer.h
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

#ifndef RIVER2_VIEWER_H
#define RIVER2_VIEWER_H

#include "ctraj/tiny/multi_viewer.h"
#include "config/configor.h"
#include "core/state_manager.h"

namespace ns_river2 {
    class Viewer : protected ns_viewer::MultiViewer {
    public:
        using Ptr = std::shared_ptr<Viewer>;

    private:
        Configor::Ptr configor;
        StateManager::Ptr stateMagr;

        std::map<std::string, std::vector<std::size_t>> lastEntities;

        std::vector<std::pair<Eigen::Vector3d, ns_viewer::Colour>> virMarkers;

        static std::string VR_WIN_NAME;
        static std::string SPLINE_WIN_NAME;

    public:
        Viewer(const ns_viewer::MultiViewerConfigor &viewerConfigor, Configor::Ptr configor,
               StateManager::Ptr stateManager);

        static Ptr Create(const Configor::Ptr &configor, const StateManager::Ptr &stateManager);

        void RunViewer();

    protected:
        void InitializeViewer(const RiverStatus::StatusPack &status);

        void UpdateSplineWindow(const RiverStatus::StatusPack &status);

        void UpdateVRWindow(const RiverStatus::StatusPack &status);

        void InitVirtualMarkers(int n, double r);

        bool InitSceneFromObj(const std::string &plyFilename, int n);
    };
}


#endif //RIVER2_VIEWER_H
