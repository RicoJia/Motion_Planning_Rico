/// \file
/// \brief: the grid world library

#ifndef MOTION_PLANNING_CODE_GRID_MAP_H
#define MOTION_PLANNING_CODE_GRID_MAP_H

#include "PRM.hpp"

namespace PRM_Grid{

    class GridMap: public PRM {
    public:
        GridMap(){}
        GridMap(int width, int height, double resolution);

        /// \brief sample, add free vertices to free_node_map and classify data
        /// \param bounding_r - the bounding readius of the robot
        /// \param map_x_lims - map's [x_min, x_max]
        /// \param map_y_lims - map's [y_min, y_max]
        /// \param map_y_lims - map's [y_min, y_max]
        /// \param cell_size - cell size in the grid
        void add_free_vertices(double bounding_r, const std::vector<double>& map_x_lims, const std::vector<double>& map_y_lims, double cell_size);

        /// \brief: return data vector
        /// \return: data
        std::vector<int> get_data() const;

        /// \brief: return the (x,y) coordinates of the cell center that corresponds to an index in data
        /// \param: index in the data vector
        std::vector<double> get_x_y(int data_index) const;
    private:
        int width, height;          // in grid integer coordinates
        double resolution;          // grid cell size
        std::vector<int> data;      // vector width x height. The bottom left corner of the PRM is the the first element of this vector, then the vector expands as you traverse to the left, then up on the PRM.
    };
}

#endif //MOTION_PLANNING_CODE_GRID_MAP_H
