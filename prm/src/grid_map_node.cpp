/// \brief: This is the visualization of a grid map that has two modes:
///     1. Default grid map using ROS parameter server data to calculate the 1. occupancy 2. buffer area 3. free space of a map.
///         Map can still be updated to a custom one through the ROS service
///     2. Custom grid map (same size as the default) through ROS service /update_grid_map_data
/// PUBLISHES: /map (nav_msgs/OccupancyGrid) the grid world map
/// SERVICE: /update_grid_map_data - Service for visualizing a custom grid map.

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "../include/prm/grid_map.hpp"
#include <cmath>
#include <boost/bind.hpp>
#include "prm/update_grid_map_data.h"

using namespace PRM_Grid;

/// \brief: this function populates a grid_msg with occupancy data
void populate_grid_msg(nav_msgs::OccupancyGrid& msg, const std::vector<int>& grid_map_data ){
    msg.data.clear();
    for (int data: grid_map_data){
        msg.data.push_back(data);
    }
}

/// \brief service callback for updating the grid map data that is routinely updated.
/// \param data - pointer to grid map data vector
/// \req - ROS Service request object that has updated map
/// \return true
bool update_grid_map_data(std::vector<int>* data, const prm::update_grid_map_data::Request& req, const prm::update_grid_map_data::Response& ) {
    *data = req.grid_map_data;
    return true;
}

int main(int argc, char** argv ){
    ros::init(argc, argv, "grid_map_node");
    ros::NodeHandle nh, nh2("~");

    XmlRpc::XmlRpcValue obstacle_list;
    int k_nearest;
    double robot_radius;
    int sample_size;
    std::vector<int> map_x_lims;
    std::vector<int> map_y_lims;
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

    int grid_map_update_rate;
    nh2.getParam("grid_map_update_rate", grid_map_update_rate);
    int use_default_grid;
    nh2.getParam("use_default_grid", use_default_grid);


    std::vector<int> grid_map_data{};

    if (use_default_grid){
        double safety_distance = robot_radius + cell_size/sqrt(2);
        GridMap grid_map(map_x_lims, map_y_lims, cell_size);
        grid_map.add_obstacles_and_normal_vecs(obstacle_list, cell_size);
        grid_map.add_free_vertices(safety_distance);
        grid_map.add_edges_to_N_neighbors(k_nearest,safety_distance);

        grid_map_data= grid_map.get_data(); //global variable for service update_grid_map_data;
    }

    ros::ServiceServer srv = nh.advertiseService<prm::update_grid_map_data::Request, prm::update_grid_map_data::Response>
            ("update_grid_map_data", boost::bind( &update_grid_map_data, &grid_map_data, _1, _2));

    ros::Rate r(grid_map_update_rate);


    while(ros::ok()){

        populate_grid_msg(grid_msg, grid_map_data);
        grid_pub.publish(grid_msg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

