//
// Created by ricojia on 4/19/20.
//

#ifndef MOTION_PLANNING_CODE_GRID_MAP_H
#define MOTION_PLANNING_CODE_GRID_MAP_H

#include "PRM.hpp"

class GridMap: public PRM {
public:
    GridMap(){}
    GridMap(int width, int height): width(width), height(height){}
//    ~GridMap(){}
    /// \brief sample, add free vertices to free_node_map and classify data
    void add_free_vertices(double bounding_r, int sample_size, const std::vector<double>& map_x_lims, const std::vector<double>& map_y_lims, double cell_size);

    /// \brief: return data vector
    /// \return: data
    std::vector<int> get_data() const;
private:
    int width, height;
    std::vector<int> data;
};


#endif //MOTION_PLANNING_CODE_GRID_MAP_H
