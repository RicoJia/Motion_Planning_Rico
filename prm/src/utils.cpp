//
// Created by ricojia on 5/9/20.
//

#include "../include/prm/utils.hpp"


/// \brief return id for an visualization_msgs::Marker object.
/// \return id of the next marker object (for visualization_msgs::marker) id

namespace PRM_Utils{

    int get_id(){
        static int id = 0;
        return id++;
    }

    /// \brief fill in x, y in a point object
    void populate_point_xy(geometry_msgs::Point& p, double x, double y){
        p.x = x; p.y = y;
    }

    /// \brief convert coordinates to real world position
    void convert_to_real_world_pos(std::vector<double>& point, double cell_size){
        point.at(0) = (point.at(0) +0.5) * cell_size;
        point.at(1) = (point.at(1) +0.5) * cell_size;
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

    //------------------------------------Marker Array functions-----------------------------
    /// \brief fill in shared info in the geometric_primitive_list
    void populate_geometric_primitive_list_basics(visualization_msgs::Marker& geometric_primitive_list, std::string name_space, double life_time){

        geometric_primitive_list.header.frame_id = "/map";
        geometric_primitive_list.header.stamp = ros::Time::now();
        geometric_primitive_list.ns = name_space;
        geometric_primitive_list.action = visualization_msgs::Marker::ADD;
        geometric_primitive_list.pose.orientation.w = 1.0;
        geometric_primitive_list.lifetime = (life_time<=0)?(ros::Duration(life_time)):(ros::Duration(life_time));
    }

    /// \brief mark start and goal in the marker array
    void populate_end_points(std::vector<double>start,
                             std::vector<double>goal,
                             std::string name_space,
                             visualization_msgs::MarkerArray& marker_array,
                             double life_time
                             ){

        geometry_msgs::Point P0, P1;
        PRM_Utils::populate_point_xy(P0, start.at(0), start.at(1));
        PRM_Utils::populate_point_xy(P1, goal.at(0), goal.at(1));
        visualization_msgs::Marker point_list;
        PRM_Utils::populate_geometric_primitive_list_basics(point_list,name_space, life_time);
        point_list.type = visualization_msgs::Marker::POINTS;
        point_list.scale.x = 0.15;         point_list.scale.y = 0.15;

        PRM_Utils::populate_geometric_primitive_list_color(point_list, 0.71, 0.40, 0.12);        //brown for path end points
        point_list.id = PRM_Utils::get_id();
        point_list.points.push_back(P0);
        point_list.points.push_back(P1);
        std::vector<visualization_msgs::Marker> markers;
        marker_array.markers.push_back(point_list);
    }

    /// \brief: populate the marker array message with line list. Only path vertices and edges from a global planning algorithm is populated here.
    void populate_edges(const std::vector<PRM_Grid::Vertex> free_waypoints,
                        visualization_msgs::MarkerArray& marker_array,
                        std::string name_space,
                        double life_time)
    {
        if (free_waypoints.empty()){
            ROS_WARN("Utils: Empty Edge list");
            return;
        }
        for (unsigned int i = 0; i < free_waypoints.size()-1; ++i){
            geometry_msgs::Point P0, P1;
            populate_point_xy(P0, free_waypoints.at(i).coord.x, free_waypoints.at(i).coord.y);
            populate_point_xy(P1, free_waypoints.at(i+1).coord.x,free_waypoints.at(i+1).coord.y);

            visualization_msgs::Marker line_list, point_list;
            populate_geometric_primitive_list_basics(point_list, name_space, life_time);
            populate_geometric_primitive_list_basics(line_list, name_space, life_time);

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
            marker_array.markers.push_back(point_list);
            marker_array.markers.push_back(line_list);
        }
    }

    /// \brief: populate the marker array message with point list. Only path vertices are populated here.
    void populate_points(const std::vector<PRM_Grid::Vertex> free_waypoints,
                        visualization_msgs::MarkerArray& marker_array,
                        std::string name_space,
                        double life_time)
    {
        if (free_waypoints.empty()){
            ROS_WARN("Utils: Empty point list");
            return;
        }
        for (unsigned int i = 0; i < free_waypoints.size(); ++i){
            geometry_msgs::Point P0;
            populate_point_xy(P0, free_waypoints.at(i).coord.x, free_waypoints.at(i).coord.y);

            visualization_msgs::Marker point_list;
            populate_geometric_primitive_list_basics(point_list, name_space, life_time);

            point_list.type = visualization_msgs::Marker::POINTS;

            point_list.scale.x = 0.05;
            point_list.scale.y = 0.05;

            populate_geometric_primitive_list_color(point_list, 0.01, 0.8, 0.125);        //light green for path vertices
            point_list.id = get_id();
            point_list.points.push_back(P0);
            marker_array.markers.push_back(point_list);
        }
    }
};


