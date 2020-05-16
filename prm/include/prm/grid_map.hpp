/// \file
/// \brief: the grid world library

#ifndef MOTION_PLANNING_CODE_GRID_MAP_H
#define MOTION_PLANNING_CODE_GRID_MAP_H

#include "PRM.hpp"
#include "math.h"

namespace PRM_Grid{

    class GridMap: public PRM {
    public:
        GridMap(){}
        GridMap(const std::vector<int>& map_x_lims, const std::vector<int>& map_y_lims, double resolution);

        /// \brief sample, add free vertices to free_node_map and classify data. Data are added following left -> right, down -> up sequence.
        /// \param bounding_r - the bounding readius of the robot
        void add_free_vertices(double bounding_r);

        /// \brief: return data vector
        /// \return: data
        std::vector<int> get_data() const;

        /// \brief: return the (x,y) coordinates of the cell center that corresponds to an index in data
        /// \param: index in the data vector
        /// \return (x y) of a cell given data_index
        std::vector<double> get_x_y(int data_index) const;

        /// \brief return the index of the cell that (x,y) falls into. Points on the upper/right boundary belong to the upper/right cell, points on the lower/left boundary belong to the current cell.
        /// \param x,y of a point
        /// \return index in data[]. if (x,y) is not in the map, then returns -1.
        int get_index_in_data_array(double x, double y) const;

        /// \brief convert the row, column coordinate of a map to data index.
        /// \param integer coordinate x,y of a point
        /// \return index in data[]. if (x,y) is not in the map, then returns -1.
        int get_index_in_data_array(const int x_coord, const int y_coord) const;

                /// \brief return a vector of neighbors' indices, regardless of their occupancy. The scan area is a box (2*scan_range+1) x (2*scan_range+1)
        /// \param data_index: the current cell in the vector
        /// \return vector of neighbor's indices. if the point is out of range, then empty vector is returned
        std::vector<int> find_neighbors(const int data_index, const int scan_range) const;

        /// \brief modify a single member in the data array.
        /// \param data_index - index in the array
        /// \param value_to_write - value to modify the element with
        /// \return true if successful, else false
        bool modify_data(const int data_index, const int value_to_write);
        
    private:
        // the real world origin of the map is [map_x_lims[0] * cell_size, map_y_lims[1] * cell_size]
        std::vector<int> map_x_lims, map_y_lims;          // in grid integer coordinates. [left_or_down, right_or_up]
        double resolution;          // grid cell size
        std::vector<int> data;      // vector width x height. The bottom left corner of the PRM is the the first element of this vector, then the vector expands as you traverse to the left, then up on the PRM.
    };
}

#endif //MOTION_PLANNING_CODE_GRID_MAP_H
