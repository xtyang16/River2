/**
 * @file bias_filter.h
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

#ifndef RIVER2_BIAS_FILTER_H
#define RIVER2_BIAS_FILTER_H

#include "ctraj/core/spline_bundle.h"
#include <cereal/types/list.hpp>
#include <ostream>

namespace ns_river2 {
    class BiasFilter {
    public:
        using Ptr = std::shared_ptr<BiasFilter>;

        struct StatePack {
            double time{};
            Eigen::Vector3d state;
            Eigen::Matrix3d var;

            StatePack(double time, Eigen::Vector3d state, const Eigen::Vector3d &varMat);

            StatePack();

            friend std::ostream &operator<<(std::ostream &os, const StatePack &pack);

        public:
            template<class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(time), CEREAL_NVP(state), CEREAL_NVP(var));
            }
        };

    private:
        StatePack curState;
        const double sigma2;

        std::list<StatePack> stateRecords;

    public:
        BiasFilter(StatePack init, double randomWalk);

        static Ptr Create(const StatePack &init, double randomWalk);

        [[nodiscard]] StatePack Prediction(double t) const;

        [[nodiscard]] const StatePack &GetCurState() const;

        [[nodiscard]] const std::list<StatePack> &GetStateRecords() const;

        void Update(const StatePack &mes);

        void UpdateByEstimator(const StatePack &est);
    };
}// namespace ns_river2


#endif//RIVER2_BIAS_FILTER_H
