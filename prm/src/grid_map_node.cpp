/// \brief: the grid mapping node that calculates the 1. occupancy 2. buffer area 3. free space of a map
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "../include/prm/grid_map.hpp"
#include <cmath>

/// \brief: this function populates a grid_msg with occupancy data
void populate_grid_msg(nav_msgs::OccupancyGrid& msg, const int height, const int width, const GridMap& grid_map){
    auto grid_map_data = grid_map.get_data();
    for (int i = 0; i < height * width; ++i){
        msg.data.push_back(grid_map_data.at(i));
    }
}

int main(int argc, char** argv ){
    ros::init(argc, argv, "grid_map_node");
    ros::NodeHandle nh, nh2("~");

    XmlRpc::XmlRpcValue obstacle_list;
    int k_nearest;
    double robot_radius;
    int sample_size;
    std::vector<double> map_x_lims;
    std::vector<double> map_y_lims;
    double cell_size;
    nh2.getParam("obstacles", obstacle_list);
    nh2.getParam("k_nearest", k_nearest);
    nh2.getParam("robot_radius", robot_radius);
    nh2.getParam("sample_size", sample_size);
    nh2.getParam("map_x_lims", map_x_lims);
    nh2.getParam("map_y_lims", map_y_lims);
    nh2.getParam("cell_size", cell_size);

    nav_msgs::OccupancyGrid grid_msg;
    ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 100, true);
    grid_msg.header.frame_id = "map";
    grid_msg.info.resolution = cell_size;
    grid_msg.info.width = (map_x_lims.at(1) - map_x_lims.at(0));
    grid_msg.info.height = (map_y_lims.at(1) - map_y_lims.at(0));

    double safety_distance = robot_radius + cell_size/sqrt(2);
    GridMap grid_map(grid_msg.info.width, grid_msg.info.height);
    grid_map.add_obstacles_and_normal_vecs(obstacle_list, cell_size);
    grid_map.add_free_vertices(safety_distance, sample_size, map_x_lims, map_y_lims, cell_size);
    grid_map.add_edges_to_N_neighbors(k_nearest,safety_distance);

    populate_grid_msg(grid_msg, grid_msg.info.height, grid_msg.info.width, grid_map);
    grid_pub.publish(grid_msg);
    ros::spin();
    return 0;
}

