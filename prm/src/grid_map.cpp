//
// Created by ricojia on 4/19/20.
//

#include "../include/prm/grid_map.hpp"
using std::cout;
using std::endl;

void GridMap::add_free_vertices(double bounding_r,
                                 int sample_size,
                                 const std::vector<double>& map_x_lims,
                                 const std::vector<double>& map_y_lims,
                                 double cell_size){
    for (int y_index = map_y_lims.at(0); y_index < map_y_lims.at(1); ++y_index){
        for (int x_index = map_x_lims.at(0); x_index < map_x_lims.at(1); ++x_index){
            double x = x_index * cell_size + 0.5 * cell_size;
            double y = y_index * cell_size + 0.5 * cell_size;
            Vertex vertex(x,y);

            if(this->if_in_obstacle(vertex)){
                this -> data.push_back(100);    // inside an obstacle
            }
            else if (this -> if_too_close(vertex, bounding_r)){
                this -> data.push_back(20);     // in the buffer area
            }
            else{
                this -> data.push_back(0);
                this->free_node_map.insertVertex(vertex.coord.x, vertex.coord.y);
            }
        }
    }
}

std::vector<int> GridMap::get_data() const {
    return this->data;
}