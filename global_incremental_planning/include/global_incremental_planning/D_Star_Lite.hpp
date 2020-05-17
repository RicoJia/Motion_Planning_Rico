//
// Created by ricojia on 5/8/20.
//

#ifndef MOTION_PLANNING_RICO_D_STAR_LITE_H
#define MOTION_PLANNING_RICO_D_STAR_LITE_H

#include "LPA_Star.hpp"
#include <utility>

namespace global_incremental_planning{
    class D_Star_Lite: public LPA_Star{

    public:
        D_Star_Lite();

        D_Star_Lite(PRM_Grid::GridMap* const map_ptr,
                    const std::vector<double>& start,
                    const std::vector<double>& goal);

        /// \brief compute the shortest path after map update
        /// \return indices of waypoints in map
        std::vector<int> get_shortest_path();


    protected:
        /// \brief update key of a node, based on the current g and rhs, and m_km
        void calc_key(LPA_Star_Node& node);

    private:
        double m_km;

    };
}


#endif //MOTION_PLANNING_RICO_D_STAR_LITE_H
