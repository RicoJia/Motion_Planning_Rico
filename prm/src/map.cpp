//
// Created by ricojia on 4/10/20.
//
#include "../include/prm/map.hpp"

using std::vector;
using namespace PRM_Grid;

int Map::search(double x, double y){
    for (auto pair:this->vertex_list){
        auto vertex = pair.second;
        if (vertex.coord.x == x && vertex.coord.y == y){
            return vertex.id;
        }
    }
    return -1;
}

int Map::insertVertex(double x, double y){
    // check if (x,y) is a new point
    int search_index = this->search(x,y);

    if(search_index == -1){
        Vertex vertex(this->next_vertex_id, x, y);          //Make new vertex
        vertex_list.insert({next_vertex_id, vertex});         //insert vertex into vertex_list
        ++this->next_vertex_id;
        return vertex.id;
    }
    else {
        return search_index;
    }
}

bool Map::deleteVertex(int vertex_id){
    auto vertex_map_it = this->vertex_list.find(vertex_id);
    if (vertex_map_it == vertex_list.end()){
        return false;               //Vertex does not exist
    }
    else{
        auto& vertex = (*vertex_map_it).second;
        int to_delete_id = vertex.id; 
        // go to all adjacent nodes, erase the vertex and its edge from its edge list.
        for (auto edge_id_pair: vertex.edge_list){
            int id = edge_id_pair.first;
            auto& target_vertex = (*(this->vertex_list.find(id))).second;
            target_vertex. edge_list.erase(to_delete_id); 
        }
        this->vertex_list.erase(to_delete_id); // erase vertex from vertex list.
        return true;
    }
}

bool Map::insertEdge(int vertex_id1, int vertex_id2){
    // check if both vertices exist in the map
    auto vertex1_map_it = this-> vertex_list.find(vertex_id1);
    auto vertex2_map_it = this-> vertex_list.find(vertex_id2);
    if (vertex1_map_it == this->vertex_list.end() or vertex2_map_it == this->vertex_list.end() ){
        return false;
    }
    else{
        auto& vertex1 = (*vertex1_map_it).second;
        auto& vertex2 = (*vertex2_map_it).second;
        // if the vertex already exists in the id_list. we don't add it
        if (vertex1.edge_list.find(vertex2.id)!= vertex1.edge_list.end()){
            return false;
        }
        else{
            double dist = rigid2d::distance( vertex1.coord, vertex2.coord );
            // make edges
            Edge edge_to_1( vertex1.id, dist );
            Edge edge_to_2( vertex2.id, dist );
            // insert two edges
            vertex1.edge_list.insert({vertex2.id, edge_to_2});
            vertex2.edge_list.insert({vertex1.id, edge_to_1});
            return true;
        }
    }
}

// Do not use delete Edge on obstacles.
bool Map::deleteEdge(int vertex_id1, int vertex_id2){
    auto vertex1_map_it = this-> vertex_list.find(vertex_id1);
    auto vertex2_map_it = this-> vertex_list.find(vertex_id2);
    if (vertex1_map_it == this->vertex_list.end() or vertex2_map_it == this->vertex_list.end() ){
        return false;
    }
    else{
        auto& vertex1 = (*vertex1_map_it).second;
        auto& vertex2 = (*vertex2_map_it).second;
        bool success = vertex1.edge_list.erase(vertex2.id);      // go to vertex 1, delete edge to 2
        vertex2.edge_list.erase(vertex1.id);        // go to vertex 2, delete edge to 1
        return success;         //return size of deleted items
    }
}

void Map::printGraph(bool print_mode){
    vector<vector<int> > all_shapes;
    if (print_mode == 0){
        all_shapes = this -> get_all_shapes();
    }
    else{
        all_shapes = this -> get_all_unique_edges();
    }
    for (auto shape: all_shapes){
        for (int id: shape){
            std::cout<<this->vertex_list.at(id).coord<<"-";
        }
        std::cout<<std::endl;
    }
}


std::vector< std::vector<int> > Map::get_all_shapes(){
    vector< vector<int> > all_shapes;
    for (auto& vertex_id_pair:this->vertex_list){
        vector<int> shape;
        this->add_next_vertices(vertex_id_pair.second.id, shape, vertex_id_pair.second.id);
        if (shape.size()!=1){
            all_shapes.push_back(shape);
        }
    }
    //reset all visited to false
    for (auto& vertex_id_pair:this->vertex_list) {
        vertex_id_pair.second.visited = false;
    }
        return all_shapes;
}

std::vector< std::vector<int> > Map::get_all_unique_edges() {
    vector< vector<int> > all_edges;
    // go to each vertex
    for (auto& vertex_id_pair:this->vertex_list) {
        auto& current_vertex = vertex_id_pair.second;
        //go to every single edge they have
        for (auto& edge_id_pair: current_vertex.edge_list){
            if (this->vertex_list.at(edge_id_pair.first).visited == false){
                all_edges.push_back(vector{current_vertex.id, edge_id_pair.first});
            }
        }
        current_vertex.visited = true;
    }


    //reset all visited to false
    for (auto& vertex_id_pair:this->vertex_list) {
        vertex_id_pair.second.visited = false;
    }

    return all_edges;
}

void Map::add_next_vertices(int current_id, std::vector<int>& shape, int head_id, int last_id){
    shape.push_back(current_id);
    auto vertex_it = this->vertex_list.find(current_id);
    auto& vertex = (*vertex_it).second;
    vertex.visited = true;

    for (auto next_vertex_id_pair: vertex.edge_list){
        int next_id = next_vertex_id_pair.first;
        auto next_vertex_it = this -> vertex_list.find(next_id);
        Vertex& next_vertex = (*next_vertex_it).second;
        if(next_vertex.visited == false ){
            this->add_next_vertices(next_id, shape, head_id, current_id);
        }
        // when the tail meets the head
        if(next_id == head_id && next_id!=last_id){
            shape.push_back(head_id);
        }
    }
}
