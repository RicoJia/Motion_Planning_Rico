/// \file
/// \brief: This is the node for visualizing Potential Field in ROS
/// PUBLISHES:
///     /pf_path (visualization_msgs/MarkerArray)
/// SERVICES:


#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <ros/ros.h>
#include <vector>
#include "prm/utils.hpp"
#include "../include/potential_field/pf.hpp"


using std::vector;
using std::cout;
using std::endl;

int main(int argc, char** argv){
    ros::init(argc, argv, "pf_node");
    ros::NodeHandle nh, nh2("~");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("pf_path", 100000, true);


    // Real world parameters for building maps
    XmlRpc::XmlRpcValue obstacle_list;
    int k_nearest;
    double robot_radius;
    int sample_size;
    vector<double> map_x_lims;
    vector<double> map_y_lims;
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
    PRM_Utils::convert_to_real_world_pos(map_x_lims, cell_size);
    PRM_Utils::convert_to_real_world_pos(map_y_lims, cell_size);

    potential_field::pf pf(cell_size, obstacle_list, robot_radius, start, goal, map_x_lims, map_y_lims);
    std::vector<PRM_Grid::Vertex> ordered_waypoint_vertices;


    ros::Rate r(20);
    while(ros::ok()){

        try {
            auto next_waypoint = pf.get_next_waypoint();
            cout<<"waypoint: "<<next_waypoint.coord.x<<", "<<next_waypoint.coord.y<<endl;

            //visualization setup
            visualization_msgs::MarkerArray marker_arr;
            double marker_sleep_t = 3600;       //each marker object lives for 3600s
            //Populate end points
            PRM_Utils::populate_end_points(start, goal, "potential_field", marker_arr, marker_sleep_t);
            //Populate edges
            ordered_waypoint_vertices.push_back(next_waypoint);
            PRM_Utils::populate_points(ordered_waypoint_vertices, marker_arr, "potential_field", marker_sleep_t);
            marker_pub.publish(marker_arr);     //mark the start and the goal of the robot.

            ros::spinOnce();
            r.sleep();
        }
        catch (std::exception&){
            break;
        }
    }

    return 0;
}


