/// \file
#include "../include/global_incremental_planning/LPA_Star.hpp"

using std::numeric_limits;
using std::isinf;
using std::vector;
using std::make_heap;
using std::pop_heap;
using std::find_if;
using std::cout;
using std::endl;

namespace global_incremental_planning{

    constexpr bool is_both_inf(const double& a, const double& b){
        return isinf(a)&&isinf(b);
    }

    bool operator<(const std::pair<double, double>& key_1, const std::pair<double, double>& key_2){
        if (key_1.first == key_2.first){
            return key_1.second < key_2.second;
        }
        else{
            return key_1.first < key_2.first - 1e-5;
        }
    }

    /// \brief predicate for sorting priority queues (min heap).
    /// true if n1 > n2
    bool first_key_is_greater(const LPA_Star_Node& n1, const LPA_Star_Node& n2){
        if (n1.key.first == n2.key.first){
            return n1.key.second > n2.key.second;
        }
        else{
            return n1.key.first > n2.key.first;
        }
    }


    LPA_Star_Node::LPA_Star_Node():
        g(numeric_limits<double>::infinity()),
        rhs(numeric_limits<double>::infinity()),
        data_index(-1),
        key{numeric_limits<double>::infinity(), numeric_limits<double>::infinity()},
        is_occupied(true)    {}


    // LPA_Star funcs

    LPA_Star::LPA_Star() {}


    LPA_Star::LPA_Star(PRM_Grid::GridMap* const map_ptr,
                       const std::vector<double>& start,
                       const std::vector<double>& goal):map_ptr(map_ptr)
                       {

        // create an array of LPA_Star_node from raw map data
        vector<int> raw_map_data = map_ptr->get_data();
        for (int i = 0; i < raw_map_data.size(); ++i){
            LPA_Star_Node node;
            node.is_occupied = (bool)(raw_map_data.at(i));
            node.data_index = i;
            this -> map_data.push_back(node);
        }

        // search ends at goal
        this -> search_end_i = map_ptr -> get_index_in_data_array(goal.at(0), goal.at(1));

        // Search starts at start, then find start node, change rhs, key, and push it to the priority queue
        this -> search_beginning_i = map_ptr -> get_index_in_data_array(start.at(0), start.at(1));
        this -> map_data.at(this -> search_beginning_i).rhs = 0;
        calc_key(this -> map_data.at(this -> search_beginning_i));
        this -> priority_queue.push_back(this -> map_data.at(this -> search_beginning_i));
    }

    LPA_Star::~LPA_Star(){}


    //===========================Helper funcs===========================
    /// \brief check if a node has the same index as index of the search beginning point.
    /// For LPA*, beginning should be start, for D*Lite, beginning should be goal
    bool LPA_Star::is_search_beginning(const LPA_Star_Node & node) const{
        return node.data_index == this->search_beginning_i;
    }


    /// \brief calculate heuristics of a node using euclidean distance to the target
    double LPA_Star::get_heuristics(const int node_index) const{
        vector<double> node_xy = this->map_ptr->get_x_y(node_index);
        vector<double> target_xy = this->map_ptr->get_x_y(this -> search_end_i);
        double h = 0.5 * rigid2d::distance(rigid2d::Vector2D(node_xy.at(0), node_xy.at(1)),
                                     rigid2d::Vector2D(target_xy.at(0), target_xy.at(1)));
        //TODO: h must have 0.5 as a multiplier. otherwise regular euclidean distance wouldn't work. Why??
        return h;
    }

    /// \brief return rhs value of node. -1 if node does not exist. 0 for the search_beginning point.
    double LPA_Star::get_rhs(const int node_index) const{

        if (node_index == this->search_beginning_i){
            return 0;
        }
        vector<int>neighbors = this->map_ptr->find_neighbors(node_index, 1);
        if (neighbors.empty()){        //if node_index does not exist
            ROS_WARN_STREAM("Node index "<<node_index<<"does not exist!");
            return -1;
        }

        double min_total_cost = numeric_limits<double>::infinity();

        for (auto neighbor_i: neighbors){
            double edge_cost = this->get_edge_cost(node_index, neighbor_i);
            double total_cost = this->map_data.at(neighbor_i).g + edge_cost;
            if (total_cost < min_total_cost){
                min_total_cost = total_cost;
            }
        }
        return min_total_cost;
    }

    /// \brief calculate the true edge cost between two nodes (this is undirectional graph)
    double LPA_Star::get_edge_cost(const int node1_index, const int node2_index) const{

        if (this->map_data.at(node1_index).is_occupied || this->map_data.at(node2_index).is_occupied){
            return numeric_limits<double>::infinity();
        }
        else{
            vector<double> node1_xy = this->map_ptr->get_x_y(node1_index);
            vector<double> node2_xy = this->map_ptr->get_x_y(node2_index);
            double edge_cost = rigid2d::distance(rigid2d::Vector2D(node1_xy.at(0), node1_xy.at(1)),
                                                 rigid2d::Vector2D(node2_xy.at(0), node2_xy.at(1)));
            return edge_cost;
        }
    }

    /// \brief return the path from the search end point to the search beginning point, by greedily finding the min(g+edge_cost)
   /// For LPA*, search ends at goal. For D*Lite, search ends at start
    std::vector<int> LPA_Star::get_path_waypoints() const{
        vector<int> path_i_vec;
        int current_i = this -> search_end_i;

        if (isinf(this->map_data.at(current_i).g)){ // if no path is found, throws an error.
            ROS_WARN("No path is found!");
            throw (std::runtime_error("No Path is found. The global planning node has quitted"));
//            return path_i_vec;
        }
        else{                                       //there is a path

            path_i_vec.push_back(current_i);    
            do{
                vector<int>neighbors = this->map_ptr->find_neighbors(current_i, 1);
                make_heap(neighbors.begin(), neighbors.end(), [&](const int i1, const int i2){
                    double edge_cost1 = this->get_edge_cost(i1, current_i);
                    double edge_cost2 = this->get_edge_cost(i2, current_i);
                    double g1 = (this->map_data).at(i1).g;
                    double g2 = (this->map_data).at(i2).g;
                    return g1 + edge_cost1 > g2 + edge_cost2;
                });

                current_i = neighbors.at(0);       //gets the next waypoint's index
                path_i_vec.push_back(current_i);

            }while(current_i != this->search_beginning_i);

            return path_i_vec;
        }
    }

    // ===========================Algorithm Funcs===========================
    
    /// \brief update key of a node, based on the current g and rhs
    void LPA_Star::calc_key(LPA_Star_Node& node){
        // <min(g, rhs) + h (s, s_goal), min(g, rhs)>
        node.key.first = std::min(node.g, node.rhs) + this->get_heuristics(node.data_index);
        node.key.second = std::min(node.g, node.rhs);
    }


    /// \brief return the first element in the priority queue, then pop it.
    std::reference_wrapper<LPA_Star_Node> LPA_Star::pop(){
        std::reference_wrapper<LPA_Star_Node> ret = this -> priority_queue.at(0).get();
        pop_heap(this -> priority_queue.begin(), this->priority_queue.end(), first_key_is_greater);
        this -> priority_queue.pop_back();
        return ret;
    }

/// \brief heapify the p_q, return the smallest key
    std::pair<double, double> LPA_Star::top_key(){
        if (this -> priority_queue.empty()){    //no nodes to expand, usually means no path will be found
            return {numeric_limits<double>::infinity(), numeric_limits<double>::infinity()};
        }
        else{
            return this->priority_queue.at(0).get().key;
        }
    }


    /// \brief get_rhs for the node, if (g == rhs), remove from U if it's in there; else, update_key, and
    //insert it to priority_queue.
    void LPA_Star::update_vertex(const int node_index){
        LPA_Star_Node& node_ref = this -> map_data.at(node_index);

        // if node is not search beginning, update its rhs
        if (!(this -> is_search_beginning(node_ref))){
            node_ref.rhs = this -> get_rhs(node_index);
        }

        // remove node_ref from U, if it's in there
        auto search_res = find_if(this->priority_queue.cbegin(), this->priority_queue.cend(),
                                [&](const LPA_Star_Node& n_ref){return n_ref.data_index == node_ref.data_index; });
        if(search_res != this->priority_queue.end()){
            this -> priority_queue.erase(search_res);
            make_heap(this->priority_queue.begin(), this->priority_queue.end(), first_key_is_greater);
        }

        //if node is locally inconsistent, add the node_ref back in priority queue with updated key.
        if(node_ref.g != node_ref.rhs){
            this -> calc_key(node_ref);
            this -> priority_queue.push_back(node_ref);
            push_heap(this->priority_queue.begin(), this->priority_queue.end(), first_key_is_greater);
        }
    }


    /// \brief compute the shortest path after map update
    std::vector<int> LPA_Star::get_shortest_path(){
        while( 1 ){
            LPA_Star_Node& search_end_ref = this -> map_data.at(search_end_i);
            this -> calc_key(search_end_ref);

            // quit when no more inconsistent nodes on priority queue, or goal has been evaluated and becomes consistent
            if(this -> top_key() >= search_end_ref.key && search_end_ref.rhs == search_end_ref.g){
                break;
            }

            LPA_Star_Node& u_ref = this -> pop();

            // if overconsistent (g>rhs), that means the node has never been expanded, update 8 neighbor vertices
            if(u_ref.g > u_ref.rhs){
                u_ref.g = u_ref.rhs;
            }
            //if underconsistent (g < rhs), that means there must be cost changes.
            else if(u_ref.g < u_ref.rhs){
                u_ref.g = numeric_limits<double>::infinity();
                update_vertex( u_ref.data_index );
            }

            // update neighbors
            auto neighbors = this -> map_ptr ->find_neighbors(u_ref.data_index, 1);
            for (int& i: neighbors){
                this -> update_vertex(i);
            }
        }
        auto path_waypoints = this -> get_path_waypoints();
        return path_waypoints;
    }


    /// \brief  handles map updates, updates associated vertex, then compute shortest path. You should call get_shortest_path() after this
    /// to calculate the shortest path
    /// \param changed_nodes_indices - indices of nodes whose occupancy is changed.
    void LPA_Star::update_map(const std::vector<int>& changed_nodes_indices){

        for (int i: changed_nodes_indices){
            // update local map_data
            this->map_data.at(i).is_occupied = true;
            // update associated vertex
            this->map_data.at(i).rhs = numeric_limits<double>::infinity();  //you need to do this explicityly.
            this->update_vertex(i);

            // update vertices connected to the occupied cell
            auto neighbors = this -> map_ptr ->find_neighbors(i, 1);
            for_each(neighbors.begin(), neighbors.end(),
                    [&](const int& neighbor_i){this->map_data.at(i).rhs = this->get_rhs(neighbor_i);});
            for_each(neighbors.begin(), neighbors.end(),
                     [&](const int& neighbor_i){this -> update_vertex(neighbor_i);});
        }
    }

    int LPA_Star::get_current_start_pos() {
        return this -> search_end_i;
    }

}

