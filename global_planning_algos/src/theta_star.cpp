/// \file 
/// \brief This is the theta * class

#include "../include/global_planning_algos/theta_star.hpp"

using global_planning_algos::Theta_Star;
using PRM_Grid::Vertex;
using std::cout;
using std::endl;
using PRM_Grid::Vertex;
using PRM_Grid::Edge;
using PRM_Grid::PRM;
using std::vector;

Theta_Star::Theta_Star(std::vector<double> start, std::vector<double> goal, XmlRpc::XmlRpcValue &obstacle_list,
                       double cell_size, double robot_radius, int sample_size, const std::vector<int> &map_x_lims,
                       const std::vector<int> &map_y_lims, int k_nearest) :
                       A_Star(start, goal, obstacle_list, cell_size, robot_radius, sample_size, map_x_lims, map_y_lims, k_nearest)
                       {
}

std::vector<PRM_Grid::Vertex> Theta_Star::get_ordered_waypoints(){
    vector<Vertex> ret_waypoints;
    bool path_is_found = false;

    while (1){
        // check if the open list is empty.
        if(this->open_list.empty()){
            path_is_found = false;
            break;
        }

        // heapify the Ol according to cost, remove the smallest cost item, set it to the current node
        std::make_heap(this->open_list.begin(), this->open_list.end(),
                       [](const A_Star_node& A, const A_Star_node& B){return A.f > B.f;});
        std::pop_heap(this->open_list.begin(), this->open_list.end(),
                      [](const A_Star_node& A, const A_Star_node& B){return A.f > B.f;});
        auto current_node = A_Star_node(this->open_list.back());        //because pop_heap only moves the front node to last element of vector
        this->open_list.pop_back();
        // add it to close list
        this->closed_list.push_back(current_node);

        // check if goal has reached or no path available
        if (this->is_goal(current_node)){
            path_is_found = true;
            break;
        }

        // loop thru all its neighbors,
        std::map<int, Edge> edge_list = current_node.prm_vertex.edge_list;
        for (auto edge_id_pair: edge_list){
            int id = edge_id_pair.first;
            //if one neighbor is on the close list, ignore it.
            auto search_result = std::find_if(this->closed_list.begin(), this->closed_list.end(),
                                              [&id](const A_Star_node& node){return node.prm_vertex.id == id;});
            if (search_result!= this->closed_list.end()){
                continue;
            }

            //add line of sight.
            int last_node_id = current_node.parent_id;
            if (last_node_id!=-1){
                Vertex& last_node = this -> prm_free_vertices[last_node_id];
                Vertex& neighbor_node = this -> prm_free_vertices[id];
                if(!(this->prm.if_edge_collide(last_node, neighbor_node))){
                    if(!(prm.if_edge_too_close_to_obstacle(last_node, this->prm_free_vertices[id], this->bounding_r))){

                        A_Star_node last_A_Star_node = *(std::find_if(closed_list.begin(), closed_list.end(),
                                                                      [&last_node_id](const A_Star_node& node){ return node.prm_vertex.id == last_node_id;}));

                        int parent_id = last_node_id;
                        double g = get_g(id, last_A_Star_node);
                        double h = get_h(id);
                        double f = g + h;
                        add_to_open_list(id, parent_id, g, h, f);
                    }
                    else{
                        goto Add_connected_neighbor;        // The only good use goto is to exit from a nested if statement
                    }
                }
                else{
                    goto Add_connected_neighbor;        // The only good use goto is to exit from a nested if statement
                }
            }
            else{
                //caulculate cost, then put it in open list.
                Add_connected_neighbor:
                // add to open list if the edge is not too close to an obstacle
                if(!(prm.if_edge_too_close_to_obstacle(current_node.prm_vertex, this->prm_free_vertices[id], this->bounding_r))){
                    //caulculate cost, then put it in open list.
                    int parent_id = current_node.prm_vertex.id;
                    double g = get_g(id, current_node);
                    double h = get_h(id);
                    double f = g + h;
                    add_to_open_list(id, parent_id, g, h, f);
                }
            }
        }
    }

    //if path is found, populate it.
    if(path_is_found){
        auto waypoint= this->closed_list.back();
        while(1){
            ret_waypoints.push_back(waypoint.prm_vertex);
            double next_id = waypoint.parent_id;
            waypoint =  *( std::find_if(this->closed_list.begin(), this->closed_list.end(),
                                        [&next_id](const A_Star_node& node){return node.prm_vertex.id == next_id;}) );
            if (waypoint.parent_id == -1){
                ret_waypoints.push_back(waypoint.prm_vertex);
                break;
            }
        }
    }
    else{
        ROS_FATAL_STREAM("NO PATH FOUND");
    }

    return ret_waypoints;
}