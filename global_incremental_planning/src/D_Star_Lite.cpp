//
// Created by ricojia on 5/8/20.
//

#include "../include/global_incremental_planning/D_Star_Lite.hpp"

using std::numeric_limits;
using std::isinf;
using std::vector;
using std::make_heap;
using std::pop_heap;
using std::cout;
using std::endl;

namespace global_incremental_planning{

    D_Star_Lite::D_Star_Lite() {}
    D_Star_Lite::D_Star_Lite(PRM_Grid::GridMap *const map_ptr, const std::vector<double> &start,
                             const std::vector<double> &goal):
                             LPA_Star(), m_km(0) {

        this -> map_ptr = map_ptr;
        // create an array of LPA_Star_node from raw map data
        vector<int> raw_map_data = map_ptr->get_data();
        for (int i = 0; i < raw_map_data.size(); ++i){
            LPA_Star_Node node;
            node.is_occupied = (bool)(raw_map_data.at(i));
            node.data_index = i;
            this -> map_data.push_back(node);
        }
        // search ends at start
        this -> search_end_i = map_ptr -> get_index_in_data_array(start.at(0), start.at(1));

        // Search starts at goal, then find start node, change rhs, key, and push it to the priority queue
        this -> search_beginning_i = map_ptr -> get_index_in_data_array(goal.at(0), goal.at(1));
        this -> map_data.at(this -> search_beginning_i).rhs = 0;
        calc_key(this -> map_data.at(this -> search_beginning_i));
        this -> priority_queue.push_back(this -> map_data.at(this -> search_beginning_i));
    }


    std::vector<int> D_Star_Lite::get_shortest_path(){
        while( 1 ){
            LPA_Star_Node& search_end_ref = this -> map_data.at(search_end_i);
            this -> calc_key(search_end_ref);

            // quit when no more inconsistent nodes on priority queue, or goal has been evaluated and becomes consistent
            auto top_key_ = top_key();

//            if(priority_queue.empty()){
//                break;
//            }     //IF CURRENT HEURISTICS FAIL, THIS IS A SAFE OPTION, THOUGH NOT EFFICIENT.
//
            if(!(this -> top_key() < search_end_ref.key ) && search_end_ref.rhs == search_end_ref.g){
                break;
            }

            LPA_Star_Node& u_ref = this -> pop();
            auto k_old = u_ref.key;

            // if the key of the first vertex of priority_queue has been updated
            LPA_Star_Node u_ref_copy = u_ref;
            this->calc_key(u_ref_copy);

            if(k_old < u_ref_copy.key){
                this -> calc_key(u_ref);
                this -> priority_queue.push_back(u_ref);
                push_heap(this->priority_queue.begin(), this->priority_queue.end(), first_key_is_greater);
            }
            else if(u_ref.g > u_ref.rhs){
                    u_ref.g = u_ref.rhs;
                // if overconsistent (g>=rhs), that means the node has never been expanded, update 8 neighbor vertices
                // update neighbors
                auto neighbors = this -> map_ptr ->find_neighbors(u_ref.data_index, 1);
                for (int& i: neighbors){
                    this -> update_vertex(i);
                }
                }
            else {                //if underconsistent (g < rhs), that means there must be cost changes.
                    u_ref.g = numeric_limits<double>::infinity();
                    update_vertex( u_ref.data_index );
                // update neighbors
                auto neighbors = this -> map_ptr ->find_neighbors(u_ref.data_index, 1);
                for (int& i: neighbors){
                    this -> update_vertex(i);
                }
            }

        }

        auto path_waypoints = this -> get_path_waypoints();

        // update for the next step.
        if(search_end_i != search_beginning_i){
            m_km += this->get_heuristics(path_waypoints.at(1));

            search_end_i = path_waypoints.at(1);

        }

        return path_waypoints;
    }

    //================================helper funcs=============================
    void D_Star_Lite::calc_key(global_incremental_planning::LPA_Star_Node &node) {
        // <min(g, rhs) + h (s, s_goal) + km, min(g, rhs)>
        node.key.first = std::min(node.g, node.rhs) + this->get_heuristics(node.data_index) + m_km;
        node.key.second = std::min(node.g, node.rhs);
    }
}