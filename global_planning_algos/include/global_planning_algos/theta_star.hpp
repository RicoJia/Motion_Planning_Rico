/// \ file 
/// \ brief This is the theta* class

#ifndef MOTION_PLANNING_CODE_THETA_STAR_HPP
#define MOTION_PLANNING_CODE_THETA_STAR_HPP

#include "A_star.hpp"

namespace global_planning_algos{
    class Theta_Star:public A_Star{
    public:
        Theta_Star(std::vector<double>start,
                   std::vector<double>goal,
                   XmlRpc::XmlRpcValue& obstacle_list,
                   double cell_size,
                   double robot_radius,
                   int sample_size,
                   const std::vector<int>& map_x_lims,
                   const std::vector<int>& map_y_lims,
                   int k_nearest
                   );

        std::vector<PRM_Grid::Vertex> get_ordered_waypoints();
    };
}

#endif //MOTION_PLANNING_CODE_THETA_STAR_HPP