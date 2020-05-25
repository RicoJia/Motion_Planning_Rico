/// \file

#ifndef MOTION_PLANNING_RICO_LPA_STAR_H
#define MOTION_PLANNING_RICO_LPA_STAR_H

#include "prm/grid_map.hpp"
#include "prm/rigid2d.hpp"
#include <utility>
#include <limits>
#include <functional>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace global_incremental_planning{

    struct LPA_Star_Node{
        LPA_Star_Node();
        double g;
        double rhs;
        int data_index;
        std::pair<double, double> key;
        bool is_occupied;
    };

    // operator overloading for comparing infinity.
    constexpr bool is_both_inf(const std::numeric_limits<double>& a, const std::numeric_limits<double>& b);

    /// \brief compare two keys.
    /// \return true if 1. key_1.first < key_2.first; 2. key_1.first == key_2.first; key_1.second < key_2.second
    bool operator<(const std::pair<double, double>& key_1, const std::pair<double, double>& key_2);

    /// \brief predicate for sorting priority queues (min heap).
    /// true if n1 > n2
    bool first_key_is_greater(const LPA_Star_Node& n1, const LPA_Star_Node& n2);


    class LPA_Star {
    public:
        LPA_Star();
        LPA_Star(PRM_Grid::GridMap* const map_ptr,
                 const std::vector<double>& start,
                 const std::vector<double>& goal);

        virtual ~LPA_Star();

        //---------------------------Interface Funcs---------------------------
        /// \brief compute the shortest path after map update
        /// \return indices of waypoints in map
        virtual std::vector<int> get_shortest_path();

        /// \brief  handles map updates, updates associated vertex, then compute shortest path. You should call get_shortest_path() after this
        /// to calculate the shortest path
        /// \param changed_nodes_indices - indices of nodes whose occupancy is changed.
        void update_map(const std::vector<int>& changed_nodes_indices);

        /// \brief returns the index of the current robot position, which is search_end_i;
        int get_current_start_pos();


        //TODO move it back
        /// \brief return the path from the search end point to the search beginning point, by greedily finding the min(g+edge_cost)
        /// For LPA*, search ends at goal. For D*Lite, search ends at start
        /// \return {} if no path is found (g_goal = inf), else, indices of all path waypoints including start and goal.
        std::vector<int> get_path_waypoints() const;

    protected:
        PRM_Grid::GridMap const *map_ptr;       //raw pointer to a GridMap object from the outside. raw pointer is used for pointing to stack_allocated objects, because smart pointer cannot acquire ownership.
        std::vector<LPA_Star_Node> map_data;        // a convenient wrapper of map, each node has the same index in the map.
        std::vector<std::reference_wrapper<LPA_Star_Node> > priority_queue;         //uses reference to reduce run time
        int search_beginning_i;              // start for LPA*, goal for D* Lite. Should not modify after initialization
        int search_end_i;               // goal for LPA*, start for D*Lite. Should not modify after initialization


        //---------------------------Helper Funcs---------------------------
        /// \brief check if a node has the same index as index of the search beginning point.
        /// For LPA*, beginning should be start, for D*Lite, beginning should be goal
        bool is_search_beginning(const LPA_Star_Node & node) const;

        /// \brief calculate heuristics of a node using euclidean distance to the search_end point
        /// For LPA*, search end should be start, for D*Lite, search end  should be goal
        /// \param node_index - index of the node in map_data
        /// \return heuristics
        double get_heuristics(const int node_index) const;

        /// \brief calculate the rhs of a node based on all its neighbors
        /// \param node_index - the index of the node
        /// \return - rhs value of node. -1 if node does not exist. 0 for the search_beginning point.
        double get_rhs(const int node_index) const;

        /// \brief calculate the true edge cost between two nodes (this is undirectional graph)
        /// \params - indices of two nodes
        /// \return - edge cost. inf if node is occupied
        double get_edge_cost(const int node1_index, const int node2_index) const;





        //---------------------------Algorithm Funcs---------------------------
        /// \brief update key of a node, based on the current g and rhs
        virtual void calc_key(LPA_Star_Node& node);

        /// \brief return the reference to the first element in the priority queue, then pop it.
        std::reference_wrapper<LPA_Star_Node> pop();

        /// \brief heapify the p_q, return the smallest key
        /// \return [inf inf] if p_q is empty, else the key of the first element of p_q, which is the smallest
        std::pair<double, double> top_key();

        /// \brief get_rhs for the node, if (g == rhs), remove from U if it's in there; else, update_key, and
        //insert it to priority_queue.
        /// \param node_index
        void update_vertex(const int node_index);

    };

}


#endif //MOTION_PLANNING_RICO_LPA_STAR_H
