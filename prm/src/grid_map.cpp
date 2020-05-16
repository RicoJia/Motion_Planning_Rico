/// \file
#include "../include/prm/grid_map.hpp"

using std::cout;
using std::endl;
using namespace PRM_Grid;

GridMap::GridMap(const std::vector<int>& map_x_lims, const std::vector<int>& map_y_lims, double resolution):
        map_x_lims(map_x_lims),
        map_y_lims(map_y_lims), 
        resolution(resolution){

}

void GridMap::add_free_vertices(double bounding_r){
    for (int y_index = map_y_lims.at(0); y_index < map_y_lims.at(1); ++y_index){
        for (int x_index = map_x_lims.at(0); x_index < map_x_lims.at(1); ++x_index){
            double x = x_index * this->resolution + 0.5 * this->resolution;
            double y = y_index * this->resolution + 0.5 * this->resolution;
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
    if (data_index<0){
        ROS_WARN_STREAM("data_index does not exist: "<<data_index);
        return std::vector<double>();
    }
    int width = map_x_lims.at(1) - map_x_lims.at(0);
    int row_num = data_index/width;
    int cln_num = data_index - row_num * width;
    double x = (cln_num + 0.5) * this->resolution;
    double y = (row_num + 0.5) * this->resolution;
    return std::vector<double>{x, y};
}

int GridMap::get_index_in_data_array(double x, double y) const{
    double x_left_boundary = this->map_x_lims.at(0) * this->resolution;
    double x_right_boundary = this->map_x_lims.at(1) * this->resolution;
    double y_lower_boundary = this->map_y_lims.at(0) * this->resolution;
    double y_upper_boundary = this->map_y_lims.at(1) * this->resolution;
    if (x >=x_right_boundary || x < x_left_boundary || y >=y_upper_boundary ||y < y_lower_boundary){
        return -1;
    }
    else{
        int x_coord = (int)((x-x_left_boundary + 1e-8)/this->resolution);           //1e-8 is for correcting rounding errors. E.g, int(0.6/0.2) = 2, because 0.6/0.2 = 2.999999999, int will simply tructate it to 2.
        int y_coord = (int)((y-y_lower_boundary + 1e-8)/this->resolution);
        int index = this->get_index_in_data_array(x_coord, y_coord);
        return index;
    }
}

int GridMap::get_index_in_data_array(const int x_coord, const int y_coord) const {
    int width = this->map_x_lims.at(1) - this->map_x_lims.at(0);        //map_x_lims.at(1) is not included in the map
    int row_first_index = y_coord * width;
    int index = row_first_index + x_coord;
    return index;
}

std::vector<int> GridMap::find_neighbors(const int data_index, const int scan_range) const {
    if (data_index < 0 || data_index >= (int)this->data.size()){
        return {};
    }
    else{
        int width = this->map_x_lims.at(1) - this->map_x_lims.at(0);
        int row_coord = data_index/width; int column_coord = data_index - row_coord * width;
        int temp[width * width];
        int count = 0;
        for (int cln_i= -1 * scan_range; cln_i< 1 * scan_range + 1; ++cln_i){
            for(int row_j = -1 * scan_range; row_j < 1 * scan_range + 1; ++row_j){
                if (this-> map_x_lims.at(0) <= column_coord + cln_i&& column_coord + cln_i< this->map_x_lims.at(1) &&
                    this-> map_y_lims.at(0) <= row_coord + row_j && row_coord +row_j < this->map_y_lims.at(1) &&
                        (cln_i != 0 || row_j!= 0))
                {
                    temp[count] = data_index + row_j * width + cln_i;
                            ++count;
                }
            }
        }
        return std::vector<int>(temp, temp+count);
    }
}

bool GridMap::modify_data(const int data_index, const int value_to_write) {
    if (data_index < 0 || data_index >= (int) this->data.size()) {
        return false;
    }
    else{
        this->data.at(data_index) = value_to_write;
        return true;
    }
}


//simple test code for find_neighbors
//
//    double safety_distance = robot_radius + cell_size/sqrt(2);
//    GridMap grid_map(map_x_lims, map_y_lims, cell_size);
//    grid_map.add_obstacles_and_normal_vecs(obstacle_list, cell_size);
//    grid_map.add_free_vertices(safety_distance);
//    grid_map.add_edges_to_N_neighbors(k_nearest,safety_distance);
//
//    grid_map_data= grid_map.get_data(); //global variable for service update_grid_map_data;
//
//    int width = map_x_lims.at(1) -map_x_lims.at(0);
//    int height = map_y_lims.at(1) - map_y_lims.at(0);
////    int test_index[] = {0,1,  34,35,  68,69,  70};   //8, 11, /..  24
//    int test_index[] = {width*(height-1),(width)*(height)-1, (width)*(height) };   //8, 11, /..  24
//    for (int i: test_index){
//        std::cout<<"i: "<<i<<" | "<<grid_map.find_neighbors(i, 2).size()<<std::endl;
//    }

