/// \ file
/// \brief: this node uses PRM and theta* to generaty a path.
/// \PUBLISH: global_path(visualization_msgs/MarkerArray)

#include "../include/global_planning_algos/theta_star.hpp"
#include "../include/global_planning_algos/A_star.hpp"
#include "prm/PRM.hpp"
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <vector>

using std::vector;
using std::cout;
using std::endl;
using PRM_Grid::Vertex;
using global_planning_algos::A_Star;
using global_planning_algos::Theta_Star;

/// \brief return id for an visualization_msgs::Marker object.
/// \return id of the next marker object (for visualization_msgs::marker) id
int get_id(){
    static int id = 0;
    return id++;
}

/// \brief fill point_list color. if_is_path_vertex: true - dark green, else, brown
void populate_geometric_primitive_list_color(visualization_msgs::Marker& geometric_primitive_list,
        float r,
        float g,
        float b){
    geometric_primitive_list.color.r = r;
    geometric_primitive_list.color.g = g;
    geometric_primitive_list.color.b = b;
    geometric_primitive_list.color.a = 1.0;
}

/// \brief fill in shared info in the geometric_primitive_list
void populate_geometric_primitive_list_basics(visualization_msgs::Marker& geometric_primitive_list){

    geometric_primitive_list.header.frame_id = "/map";
    geometric_primitive_list.header.stamp = ros::Time::now();
    geometric_primitive_list.ns = "global_path";
    geometric_primitive_list.action = visualization_msgs::Marker::ADD;
    geometric_primitive_list.pose.orientation.w = 1.0;
    geometric_primitive_list.lifetime = ros::Duration();
}

/// \brief fill in x, y in a point object
void populate_point_xy(geometry_msgs::Point& p, double x, double y){
    p.x = x; p.y = y;
}

/// \brief: populate the marker array message with line list. Only path vertices and edges from a global planning algorithm is populated here.
void populate_edges(const std::vector<PRM_Grid::Vertex> free_waypoints,
                    visualization_msgs::MarkerArray& marker_array)
{
    std::vector<visualization_msgs::Marker> markers;
    for (unsigned int i = 0; i < free_waypoints.size()-1; ++i){
        geometry_msgs::Point P0, P1;
        populate_point_xy(P0, free_waypoints.at(i).coord.x, free_waypoints.at(i).coord.y);
        populate_point_xy(P1, free_waypoints.at(i+1).coord.x,free_waypoints.at(i+1).coord.y);

        visualization_msgs::Marker line_list, point_list;
        populate_geometric_primitive_list_basics(point_list);
        populate_geometric_primitive_list_basics(line_list);

        point_list.type = visualization_msgs::Marker::POINTS;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        point_list.scale.x = line_list.scale.x = 0.05;
        point_list.scale.y = 0.05;

        populate_geometric_primitive_list_color(line_list, 1.0, 0.412, 0.706);       //pink for path edges
        populate_geometric_primitive_list_color(point_list, 0.01, 0.2, 0.125);        //dark green for path vertices
        point_list.id = get_id();
        line_list.id = get_id();
        point_list.points.push_back(P0);
        point_list.points.push_back(P1);
        line_list.points.push_back(P0);
        line_list.points.push_back(P1);
        markers.push_back(point_list);
        markers.push_back(line_list);

    }
    marker_array.markers = markers;
}

/// \brief mark start and goal in the marker array
void populate_end_points(std::vector<double>start,
                         std::vector<double>goal,
                         visualization_msgs::MarkerArray& marker_array){

    geometry_msgs::Point P0, P1;
    populate_point_xy(P0, start.at(0), start.at(1));
    populate_point_xy(P1, goal.at(0), goal.at(1));
    visualization_msgs::Marker point_list;
    populate_geometric_primitive_list_basics(point_list);
    point_list.type = visualization_msgs::Marker::POINTS;
    point_list.scale.x = 0.15;         point_list.scale.y = 0.15;

    populate_geometric_primitive_list_color(point_list, 0.71, 0.40, 0.12);        //brown for path end points
    point_list.id = get_id();
    point_list.points.push_back(P0);
    point_list.points.push_back(P1);
    std::vector<visualization_msgs::Marker> markers;
    marker_array.markers.push_back(point_list);
}

/// \brief convert coordinates to real world position
void convert_to_real_world_pos(std::vector<double>& point, double cell_size){
    point.at(0) *= cell_size;
    point.at(1) *= cell_size;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh, nh2("~");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("global_path", 10, true);
    visualization_msgs::MarkerArray vis_msgs;

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
    convert_to_real_world_pos(start, cell_size);
    convert_to_real_world_pos(goal, cell_size);

    int algo_select;
    nh2.getParam("algo_select",algo_select);

    A_Star* planner_ptr;
    if (algo_select==0){
        planner_ptr = new Theta_Star{start, goal, obstacle_list, cell_size, robot_radius, sample_size, map_x_lims,map_y_lims, k_nearest};
        ROS_INFO_STREAM("Theta star algorithm is in use");
    }
    else{
        planner_ptr = new A_Star{start, goal, obstacle_list, cell_size, robot_radius, sample_size, map_x_lims,map_y_lims, k_nearest};
        ROS_INFO_STREAM("A star algorithm is in use");
    }

    auto ordered_waypoints = planner_ptr -> get_ordered_waypoints();
    //check if there are no waypoints.
    if (ordered_waypoints.empty()){
        ROS_FATAL_STREAM("NO PATH FOUND");
        return 1;
    }

    populate_edges(ordered_waypoints, vis_msgs);

    populate_end_points(start, goal, vis_msgs);
    marker_pub.publish(vis_msgs);
    ros::spin();
    return 0;
}
