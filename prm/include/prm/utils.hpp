/// \file

#ifndef MOTION_PLANNING_RICO_UTILS_H
#define MOTION_PLANNING_RICO_UTILS_H

#include <vector>
#include <string>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "PRM.hpp"

namespace PRM_Utils {

    /// \brief return id for an visualization_msgs::Marker object.
    /// \return id of the next universal marker object (for visualization_msgs::marker) id
    int get_id();

    /// \brief convert coordinates to real world position
    void convert_to_real_world_pos(std::vector<double>& point, double cell_size);

    /// \brief fill in x, y in a point object
    /// \param p - geometry_msgs object
    void populate_point_xy(geometry_msgs::Point& p, double x, double y);


        /// \brief fill point_list color. if_is_path_vertex: true - dark green, else, brown
    /// \param geometric_primitive_list - a marker object, such as POINTS, LINELIEST
    void populate_geometric_primitive_list_color(visualization_msgs::Marker& geometric_primitive_list,
                                                 float r,
                                                 float g,
                                                 float b);

    /// \brief fill in shared info in the geometric_primitive_list
    /// \param geometric_primitive_list - a marker object, such as POINTS, LINELIEST
    /// \param name_space - namespace for the marker object
    void populate_geometric_primitive_list_basics(visualization_msgs::Marker &geometric_primitive_list,
                                                  std::string name_space, double life_time = -1.0);


    /// \brief mark start and goal in the marker array
    /// \param start - starting world position in double
    /// \param goal - goal world position in double
    /// \param name_space - name space of the marker array.
    void populate_end_points(std::vector<double>start,
                             std::vector<double>goal,
                             std::string name_space,
                             visualization_msgs::MarkerArray& marker_array,
                             double life_time = -1.0);

    /// \brief: populate the marker array message with line list. Only path vertices and edges from a global planning algorithm is populated here.
    /// \param free_waypoints - waypoints to be visualized
    /// \param marker_array - marker_array object to be populated
    void populate_edges(const std::vector<PRM_Grid::Vertex> free_waypoints,
                        visualization_msgs::MarkerArray& marker_array,
                        std::string name_space,
                        double life_time);

    /// \brief: populate the marker array message with points.
    /// \param free_waypoints - waypoints to be visualized
    /// \param marker_array - marker_array object to be populated
    /// \param name_space name space of the marker array object
    /// \param life_time life time of the marker objects
    void populate_edges(const std::vector<PRM_Grid::Vertex> free_waypoints,
                        visualization_msgs::MarkerArray& marker_array,
                        std::string name_space,
                        double life_time);


    /// \brief: populate the marker array message with point list. Only path vertices are populated here.
    /// \param free_waypoints - waypoints to be visualized
    /// \param marker_array - marker_array object to be populated
    void populate_points(const std::vector<PRM_Grid::Vertex> free_waypoints,
                         visualization_msgs::MarkerArray& marker_array,
                         std::string name_space,
                         double life_time);
}

#endif //MOTION_PLANNING_RICO_UTILS_H
