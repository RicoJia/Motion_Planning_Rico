//
// Created by ricojia on 4/19/20.
//

#include "../include/prm/grid_map.hpp"
using std::cout;
using std::endl;
using namespace PRM_Grid;

void GridMap::add_free_vertices(double bounding_r,
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

std::vector<double> GridMap::get_x_y(int data_index) const{
    int row_num = data_index/width;
    int cln_num = data_index - row_num * width;
    double x = (row_num + 0.5) * this->resolution;
    double y = (cln_num + 0.5) * this->resolution;
    return std::vector<double>{x, y};
}
