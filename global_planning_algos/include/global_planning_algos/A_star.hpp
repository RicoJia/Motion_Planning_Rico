//
// Created by ricojia on 5/1/20.
//

#ifndef MOTION_PLANNING_CODE_A_STAR_H
#define MOTION_PLANNING_CODE_A_STAR_H

#include "prm/PRM.hpp"
#include <vector>
#include "prm/rigid2d.hpp"
#include <algorithm>
#include <cmath>

namespace global_planning_algos{

    struct A_Star_node{
        int parent_id;     // the id of the parent node
        PRM_Grid::Vertex prm_vertex;
        double g;           // the distance between the current node and the start node
        double h;           // heuristics
        double f;           // total cost
    };

    class A_Star{

    public:
        A_Star();
        A_Star(std::vector<double>start,
                std::vector<double>goal,
                XmlRpc::XmlRpcValue& obstacle_list,
                double cell_size,
                double robot_radius,
                int sample_size,
                const std::vector<int>& map_x_lims,
                const std::vector<int>& map_y_lims,
                int k_nearest
                );
        virtual ~A_Star();

        /// \brief: plan, and return the path waypoints
        /// \return: a vector of waypoints arranged in order goal->path, or empty if no path is available.
        virtual std::vector<PRM_Grid::Vertex> get_ordered_waypoints();

    protected:
        std::vector<A_Star_node> open_list;
        std::vector<A_Star_node> closed_list;       //<unordered_map> is a bit more efficient here, but for the sake of uniformity.
        std::vector<double> goal_position;          // real_world goal position
        std::unordered_map<int, PRM_Grid::Vertex> prm_free_vertices;
        PRM_Grid::PRM prm;                          // used in theta_star
        double bounding_r;

        /// \brief add an A_Star node to the open list. If the node already exists, then modify its parent_id and cost if the existing cost
        /// is greater
        /// \param id - id in prm_free_vertices
        /// \param parent_id - parent id in prm_free_vertices
        /// \param cost - cost to goal.
        void add_to_open_list( const int id, const int parent_id, const double g, const double h, const double f);

        /// \brief calculate heuristics of a node, which is the euclidean distance to the goal
        /// \param id - the current id
        /// \return - heuristics of the current node.
        double get_h(const int id);

        /// \brief calculate g of a node, which is the manhattan distance between the parent and the current node
        /// \param id - the current id
        /// \param parent - parrent A_Star node
        /// \return - g cost of the current node.
        double get_g(const int id, const A_Star_node& parent);

        /// \brief checks if a node is already goal
        /// \param node - node of interest
        /// \return - true if it's goal, else false
        bool is_goal(const A_Star_node& node);
    };
}

#endif //MOTION_PLANNING_CODE_A_STAR_H
