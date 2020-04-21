/// \file
/// \brief: the grid world library

#ifndef MOTION_PLANNING_CODE_GRID_MAP_H
#define MOTION_PLANNING_CODE_GRID_MAP_H

#include "PRM.hpp"

namespace PRM_Grid{

    class GridMap: public PRM {
    public:
        GridMap(){}
        GridMap(int width, int height, double resolution):
                width(width),
                height(height),
                resolution(resolution)
        {}
//    ~GridMap(){}
        /// \brief sample, add free vertices to free_node_map and classify data
        void add_free_vertices(double bounding_r, const std::vector<double>& map_x_lims, const std::vector<double>& map_y_lims, double cell_size);

        /// \brief: return data vector
        /// \return: data
        std::vector<int> get_data() const;

        /// \brief: return the (x,y) coordinates of the cell center that corresponds to an index in data
        /// \param: index in data
        std::vector<double> get_x_y(int data_index) const;
    private:
        int width, height;
        double resolution;
        std::vector<int> data;
    };
}


#endif //MOTION_PLANNING_CODE_GRID_MAP_H
