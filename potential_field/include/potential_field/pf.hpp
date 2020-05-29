//
// Created by ricojia on 5/23/20.
//

#ifndef MOTION_PLANNING_RICO_PF_H
#define MOTION_PLANNING_RICO_PF_H

#include "prm/PRM.hpp"
#include "prm/rigid2d.hpp"
#include <vector>
#include <stdexcept>


namespace potential_field{

    class pf {
    public:
        pf();

        /// \brief construct the map using real world coordinates (double)
        pf(double cell_size,
           XmlRpc::XmlRpcValue obstacle_list,
           double bounding_r,
           const std::vector<double>& start,
           const std::vector<double>& goal,
           const std::vector<double>& map_x_lims,
           const std::vector<double>& map_y_lims);

        /// \brief calculates next waypoint and returns it
        /// \return waypoint (PRM_Grid::Vertex)
        PRM_Grid::Vertex get_next_waypoint();

    private:
        PRM_Grid::PRM m_prm;
        double m_bounding_r;
        rigid2d::Vector2D m_start;
        rigid2d::Vector2D m_goal;
        std::vector<double> m_map_x_lims;
        std::vector<double> m_map_y_lims;
        rigid2d::Vector2D m_current_pos;

        //parameters
        double m_epsilon;       // xy_tolerance
        double m_ksi;           // step_size and attractive field weight
        double m_eta;           // repusive field strength weight
        double m_q_star;        // obstacle influence radius in repulsive field
        double m_d_star;        // goal radius in attractive field


        /// \brief calculates gradient of the attractive field, from the goal to current robot pos
        /// \return attractive field gradient
        rigid2d::Vector2D get_att_gradient() const;

        /// \brief calculates gradient of the repulsive field, by iterating thru all obstacles,
        /// finding the their closest points to current_pos, and calculating the correspoding .
        /// Then, return the sum of all gradients
        /// \return repulsive field gradient
        rigid2d::Vector2D get_rep_gradient() const;/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include <Eigen/Dense>
#include <math.h>       /* cos */


        /// \brief check if current robot position is within m_epsilon dist from the goal
        bool has_reached_goal() const;

        /// \brief check if the current robot position is in an obstacle
        bool is_in_obstacle() const;

    };

}

#endif //MOTION_PLANNING_RICO_PF_H
