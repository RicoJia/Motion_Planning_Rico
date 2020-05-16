/// \file
/// \brief: Node that visualizes the PRM roadmap on RViz. Red lines for obstacle boundary, Blue lines for edges, green line_list for free wayline_list.
/// PARAMETERS:
/// PUBLISHES:
///     /map_vis (visualization_msgs::MarkerArray)

#include "../include/prm/PRM.hpp"
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <XmlRpcValue.h>

using namespace PRM_Grid;

/// \brief: populate the marker array message with line_list
/// \param: vertex_list: all free edges and obstacle edges from a map
/// \param: visualization_msgs object

void populate_edges(const std::unordered_map<int, Vertex> & vertex_list,
                    const std::unordered_map<int, Vertex> & obstacle_vertex_list,
                    const std::vector< std::vector<int> >& edge_list,
                    const std::vector< std::vector<int> >& obstacle_edge_list,
                    visualization_msgs::MarkerArray& marker_array)
                    {
    std::vector<visualization_msgs::Marker> markers;

    int id = 0;
    for (auto& edge:edge_list){
        geometry_msgs::Point P0, P1;
        P0.x = vertex_list.at(edge.at(0)).coord.x;
        P0.y = vertex_list.at(edge.at(0)).coord.y;
        P1.x = vertex_list.at(edge.at(1)).coord.x;
        P1.y = vertex_list.at(edge.at(1)).coord.y;

        visualization_msgs::Marker line_list, point_list;
        point_list.header.frame_id  = line_list.header.frame_id = "/map";
        point_list.header.stamp = line_list.header.stamp = ros::Time::now();
        point_list.ns = line_list.ns = "free_vertices_edges";
        point_list.action = line_list.action = visualization_msgs::Marker::ADD;
        point_list.pose.orientation.w = line_list.pose.orientation.w = 1.0;
        point_list.lifetime = line_list.lifetime = ros::Duration();

        point_list.type = visualization_msgs::Marker::POINTS;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        point_list.scale.x = line_list.scale.x = 0.05;

        point_list.scale.y = 0.05;
        line_list.color.b = 1.0f;
        line_list.color.a = 1.0;
        point_list.color.g = 1.0f;
        point_list.color.a = 1.0;
        point_list.id = id++;
        line_list.id = id++;

        point_list.points.push_back(P0);
        point_list.points.push_back(P1);
        line_list.points.push_back(P0);
        line_list.points.push_back(P1);
        markers.push_back(point_list);
        markers.push_back(line_list);
    }

    for (auto& edge:obstacle_edge_list){
        geometry_msgs::Point P0, P1;
        for (unsigned int i = 0; i < edge.size()-1; ++i){
            P0.x = obstacle_vertex_list.at(edge.at(i)).coord.x;
            P0.y = obstacle_vertex_list.at(edge.at(i)).coord.y;
            P1.x = obstacle_vertex_list.at(edge.at(i+1)).coord.x;
            P1.y = obstacle_vertex_list.at(edge.at(i+1)).coord.y;

            visualization_msgs::Marker line_list, point_list;
            point_list.header.frame_id  = line_list.header.frame_id = "/map";
            point_list.header.stamp = line_list.header.stamp = ros::Time::now();
            point_list.ns = line_list.ns = "obstacle_vertices_edges";
            point_list.action = line_list.action = visualization_msgs::Marker::ADD;
            point_list.pose.orientation.w = line_list.pose.orientation.w = 1.0;
            point_list.lifetime = line_list.lifetime = ros::Duration();
            point_list.type = visualization_msgs::Marker::POINTS;
            line_list.type = visualization_msgs::Marker::LINE_LIST;
            point_list.scale.x = line_list.scale.x = 0.05;
            point_list.scale.y = 0.05;
            line_list.color.r = 1.0f;
            line_list.color.a = 1.0;
            point_list.color.r = 1.0f;
            point_list.color.g = 1.0f;
            point_list.color.a = 1.0;
            point_list.id = id++;
            line_list.id = id++;
            point_list.points.push_back(P0);
            point_list.points.push_back(P1);
            line_list.points.push_back(P0);
            line_list.points.push_back(P1);
            markers.push_back(point_list);
            markers.push_back(line_list);
        }
    }
    marker_array.markers = markers;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "prm_node");
    ros::NodeHandle nh, nh2("~");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("map_vis", 10, true);
    visualization_msgs::MarkerArray vis_msgs;

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

    int if_show_edges;
    nh2.getParam("if_show_edges", if_show_edges);

    PRM prm;
    prm.add_obstacles_and_normal_vecs(obstacle_list, cell_size);
    if (if_show_edges==1){
        prm.add_free_vertices(robot_radius, sample_size, map_x_lims, map_y_lims, cell_size);
        prm.add_edges_to_N_neighbors(k_nearest, robot_radius);
    }

    auto vertex_list = prm.get_free_map_vertices();
    auto edge_list = prm.get_free_edges();
    auto obstacle_vertex_list = prm.get_obstacle_vertices_list();
    auto obstacle_edge_list = prm.get_obstacle_edges();

    populate_edges(vertex_list, obstacle_vertex_list, edge_list, obstacle_edge_list, vis_msgs);

    marker_pub.publish(vis_msgs);
    ros::spin();
    return 0;
}
