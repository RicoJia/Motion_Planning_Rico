/// \file This is the node for visualizing LPA* and D* Lite algorithms
/// \brief this is the main node that builds limited-view map for incremental planning ndoes based on the real map
/// PUBLISH: /global_path (visualization_msgs/MarkerArray)

#define LPA_Star_Select 3
#define D_Star_Lite_Select 4

#include <ros/ros.h>
#include <prm/utils.hpp>
#include "prm/update_grid_map_data.h"
#include "prm/grid_map.hpp"
#include "../include/global_incremental_planning/LPA_Star.hpp"
#include "../include/global_incremental_planning/D_Star_Lite.hpp"
#include <algorithm>


using std::vector;
using std::cout;
using std::endl;
using PRM_Grid::GridMap;



/// \brief update the robot map, return indices of newly occupied cells for updating the grid map  for D* Lite and LPA*
/// note that the visible range is a square.
/// \param true_map_data - the data array true map
/// \param robot_map - the simulated map data robot currently has
/// \params xy_location - current robot position (x,y) in integer coordinates
/// \param scan_length - how much further ahead/back the robot can see. This is 1/2 of a side of the scan square
std::vector<int> update_map_by_robot_location(
                              const std::vector<int>& true_map_data,
                              GridMap& robot_map,
                              const int data_index,
                              const int scan_length)      //a square, return a vector of newly occupied cells
{
    std::vector<int> robot_map_data = robot_map.get_data();
    std::vector<int> neighbors = robot_map.find_neighbors(data_index, scan_length);
    if (!neighbors.empty()){
        neighbors.push_back(data_index);
    }
    // remove indices of neighbors which do not change.
    auto new_end_iter = std::remove_if(neighbors.begin(), neighbors.end(),
                                       [&](const int& neighbor)
                                       {return robot_map_data.at(neighbor)==true_map_data.at(neighbor);});
    std::vector<int> ret_vec (neighbors.begin(), new_end_iter);
    // modify the robot map based on ret_vec
    std::for_each(ret_vec.begin(), ret_vec.end(),
                  [&](const int& index_to_update)
                  {robot_map.modify_data(index_to_update, true_map_data.at(index_to_update)); }  );

    return ret_vec;
}


/// \brief update the robot_map. and return indices of newly occupied cells for updating the grid map by rows, for LPA star
/// \param true_map - the true map data
/// \param robot_map - the simulated map data robot currently has
/// \param row_num - number of rows to increase
/// \return indices of newly occupied cells. When all rows have been updated, it will return {-1}
std::vector<int> update_map_by_clns(
                        const std::vector<int>& true_map_data,
                        GridMap& robot_map,
                        const int clns_to_add_num,
                        const std::vector<int>& map_x_lims, 
                        const std::vector<int>& map_y_lims){
    std::vector<int> ret_vec;

    static int current_x = map_x_lims.at(0), current_y = map_y_lims.at(0);
    static bool is_first_time = true;
    int max_valid_x = map_x_lims.at(1) -1, max_valid_y = map_y_lims.at(1) -1;

    if(is_first_time){
        is_first_time = false;
    }
    else {
        // scan the columns in one shot
        while (1) {
            int data_index = robot_map.get_index_in_data_array(current_x, current_y);
            auto partial_ret_vec = update_map_by_robot_location(true_map_data, robot_map, data_index, clns_to_add_num);
            ret_vec.insert(ret_vec.end(), partial_ret_vec.begin(), partial_ret_vec.end());
            if (current_y == max_valid_y) {
                break;
            }
            current_y = current_y + 2 * clns_to_add_num + 1;
            if (current_y > max_valid_y) {
                current_y = max_valid_y;
            }
        }

        // signal the outside function that all rows have been updated.
        if (current_x == max_valid_x && current_y == max_valid_y) {
            return {};
        }

        //set up x,y for next update.
        current_y = 0;
        if (current_x + clns_to_add_num > max_valid_x) {
            current_x = max_valid_x;
        } else {
            current_x = current_x + 2 * clns_to_add_num + 1;
        }
    }

    return ret_vec;
}



int main(int argc, char**argv){
    ros::init(argc, argv, "global_incremental_planning_node");
    ros::NodeHandle nh, nh2("~");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("global_path", 10, true);


    // Real world parameters for building maps
    XmlRpc::XmlRpcValue obstacle_list;
    int k_nearest;
    double robot_radius;
    int sample_size;
    vector<int> map_x_lims;
    vector<int> map_y_lims;
    double cell_size;
    nh2.getParam("obstacles", obstacle_list);
    nh2.getParam("k_nearest", k_nearest);
    nh2.getParam("robot_radius", robot_radius);
    nh2.getParam("sample_size", sample_size);
    nh2.getParam("map_x_lims", map_x_lims);
    nh2.getParam("map_y_lims", map_y_lims);
    nh2.getParam("cell_size", cell_size);
    vector<double> start, goal;
    nh2.getParam("start", start);
    nh2.getParam("goal", goal);
    PRM_Utils::convert_to_real_world_pos(start, cell_size);
    PRM_Utils::convert_to_real_world_pos(goal, cell_size);



    //set up 2 grid maps: one is the true map, the other is the simulated on
    double safety_distance = robot_radius + cell_size/sqrt(2);
    GridMap true_map(map_x_lims, map_y_lims, cell_size);
    true_map.add_obstacles_and_normal_vecs(obstacle_list, cell_size);
    true_map.add_free_vertices(safety_distance);
    std::vector<int> true_map_data = true_map.get_data();

    GridMap robot_map(map_x_lims, map_y_lims, cell_size);
    robot_map.add_free_vertices(safety_distance);


    // set up the LPA* or D* lite switching mechanism
    int algo_select;
    nh2.getParam("algo_select",algo_select);

    global_incremental_planning::LPA_Star* planner_ptr;
    if (algo_select == LPA_Star_Select){
        planner_ptr = new global_incremental_planning::LPA_Star{&robot_map, start, goal};
        ROS_INFO_STREAM("LPA_Star is in use");
    }
    else{
        planner_ptr = new global_incremental_planning::D_Star_Lite{&robot_map, start, goal};
        ROS_INFO_STREAM("D* Lite is in use");
    }



    ros::ServiceClient update_grid_srv_client;
    update_grid_srv_client = nh.serviceClient<prm::update_grid_map_data>("update_grid_map_data");
    ros::service::waitForService("update_grid_map_data", 2);
    prm::update_grid_map_data srv_msg;


    // loop and map update parameters
    double sleep_time;
    int update_row_num;
    int scan_length;
    nh2.getParam("sleep_time", sleep_time);
    nh2.getParam("update_row_num", update_row_num);
    nh2.getParam("scan_length", scan_length);
    std::vector<int> ordered_waypoints = planner_ptr->get_shortest_path();



    while(ros::ok()){

        try{
            ordered_waypoints = planner_ptr ->get_shortest_path();

            if (algo_select == D_Star_Lite_Select){
                start = robot_map.get_x_y(planner_ptr -> get_current_start_pos());
            }

            // visualization
            //update grid map
            srv_msg.request.grid_map_data = robot_map.get_data();     //show the updated robot map
            update_grid_srv_client.call(srv_msg);       //call the grid map visualization node about the update

            visualization_msgs::MarkerArray marker_arr;

            //Populate end points
            PRM_Utils::populate_end_points(start, goal, "global_incremental_planning", marker_arr, sleep_time);

            //Populate edges
            std::vector<PRM_Grid::Vertex> ordered_waypoint_vertices;
            ordered_waypoint_vertices.reserve(ordered_waypoints.size());    //reserve spots without initialization

            std::for_each(ordered_waypoints.cbegin(), ordered_waypoints.cend(),
                          [&](const int i){auto xy = robot_map.get_x_y(i);
                              ordered_waypoint_vertices.push_back(PRM_Grid::Vertex(xy.at(0), xy.at(1))); });
            PRM_Utils::populate_edges(ordered_waypoint_vertices, marker_arr, "global_incremental_planning", sleep_time);
            marker_pub.publish(marker_arr);     //mark the start and the goal of the robot.


            //update map
            std::vector<int> updated_indices;
            int data_index;
            if (algo_select == LPA_Star_Select){
                // update the map by rows for LPA star
                updated_indices = update_map_by_clns(true_map_data, robot_map, update_row_num, map_x_lims, map_y_lims);
            }

            else if(algo_select == D_Star_Lite_Select){
                data_index = robot_map.get_index_in_data_array(start.at(0), start.at(1));
                updated_indices = update_map_by_robot_location(true_map_data, robot_map, data_index, scan_length);
            }
            planner_ptr ->update_map(updated_indices);
            ros::spinOnce();
            ros::Duration(sleep_time).sleep();
        }

        // catch exception for not finding a path. then break from while loop
        catch(std::exception& e){
            std::cerr<<e.what();
            break;
        }
    }

    return 0;
}

