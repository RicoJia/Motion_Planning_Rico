/// \brief This file plans path globally using the A star algorithm
/// P
#include "../include/global_planning_algos/A_star.hpp"

using namespace global_planning_algos;
using std::vector;
using std::cout;
using std::endl;
using PRM_Grid::Vertex;
using PRM_Grid::Edge;
using PRM_Grid::PRM;

A_Star::A_Star() {
}

A_Star::~A_Star() {}

A_Star::A_Star(std::vector<double>start,
               std::vector<double>goal,
               XmlRpc::XmlRpcValue& obstacle_list,
               double cell_size,
               double robot_radius,
               int sample_size,
               const std::vector<int>& map_x_lims,
               const std::vector<int>& map_y_lims,
               int k_nearest ):
                                goal_position(goal),
                                bounding_r(robot_radius)
               {
    // build PRM
    prm.add_obstacles_and_normal_vecs(obstacle_list, cell_size);
    prm.add_free_vertices(robot_radius, sample_size, map_x_lims, map_y_lims, cell_size);

    // Add additional points
    std::vector<Vertex> additional_vertices{Vertex(start.at(0), start.at(1)),
                                        Vertex(goal.at(0), goal.at(1))};
    prm.add_additional_free_vertices(additional_vertices, robot_radius);
    // add edges.
    prm.add_edges_to_N_neighbors(k_nearest, robot_radius);
    //store all free vertices from PRM
     this->prm_free_vertices = prm.get_free_map_vertices();
   // add start to open list
    int start_vertex_id = prm.search_free_vertex(start.at(0), start.at(1));
    this->add_to_open_list(start_vertex_id, -1, 0, 0, 0);     //parent_id = -1, g, h are set to 0.
}

//-----------------------------------------Public functions-----------------------------------

std::vector<PRM_Grid::Vertex> A_Star::get_ordered_waypoints(){
    vector<Vertex> ret_waypoints;
    bool path_is_found = false;

    // if the open list is empty, then no path is found.
    while (!this->open_list.empty()){

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

//-----------------------------------------Private functions-----------------------------------

void A_Star::add_to_open_list( const int id, const int parent_id, const double g, const double h, const double f){

    //find if the vertex exists in open list
    auto search_result = std::find_if(this->open_list.begin(), this->open_list.end(),
            [&id](const A_Star_node& node){return id == node.prm_vertex.id; });

    // if not, add it, if so, modify its costs
    if (search_result == this->open_list.end()){
        A_Star_node node{parent_id, this->prm_free_vertices[id], g, h, f};
        this->open_list.push_back(node);
    }
    else{
        if(search_result->g > g){
            search_result->g = g;
            search_result->h = h;
            search_result->f = f;
            search_result->parent_id = parent_id;
        }
    }
}

double A_Star::get_h(const int id){
    double x = this->prm_free_vertices[id].coord.x;
    double y = this->prm_free_vertices[id].coord.y;
    double goal_x = this->goal_position.at(0);
    double goal_y = this->goal_position.at(1);
    return rigid2d::distance(rigid2d::Vector2D(x,y), rigid2d::Vector2D(goal_x, goal_y));
}

double A_Star::get_g(const int id, const A_Star_node& parent) {
    double parent_g = parent.g;
    double x = this->prm_free_vertices[id].coord.x;
    double y = this->prm_free_vertices[id].coord.y;
    double parent_x = parent.prm_vertex.coord.x;
    double parent_y = parent.prm_vertex.coord.y;
    double g_increment = std::abs(parent_x - x) + std::abs(parent_y - y);   //Manhattan distance
    return parent_g + g_increment;
}

bool A_Star::is_goal(const global_planning_algos::A_Star_node &node) {
    return (node.prm_vertex.coord.x == this->goal_position.at(0))
            &&(node.prm_vertex.coord.y == this->goal_position.at(1));
}