
#include "../include/prm/PRM.hpp"

using std::cout;
using std::cin;
using std::endl; 
using std::vector;
using rigid2d::Vector2D;
using rigid2d::length;

//--------------------------------------------pimpl idiom for data hiding------------------------------------------------------------------
struct PRM::impl{
    const double map_x_max = 10;
    const double map_y_max = 10;
    const double bounding_radius = 0.15;      //0.15m
    const double K = 2;
    impl(){}
};

PRM::PRM():pimpl(new impl)  //TODO: add rosnodehandle; Also, why pimpl{} instead of pimpl() is used?
{}
PRM::PRM(PRM&&) = default;
PRM& PRM::operator=(PRM &&) = default;
PRM::~PRM() = default;

//--------------------------------------------Interface functions------------------------------------------------------------------
void PRM::add_obstacles_and_normal_vecs() {
    //TODO: sample obstacles following the counter clockwise rule and the shape closure rule
    vector<vector<double> > x_list= {{0,1,1,0,0}, {10, 20,20,10}};
    vector<vector<double> > y_list = {{0,0,1,1,0}, {0, 0, 1,0}};

    for (unsigned int i=0; i<x_list.size(); ++i){
        //add vertices and edges
        int last_vertex_id = -1;
        for (unsigned int vertex_j = 0; vertex_j < x_list.at(i).size(); ++vertex_j){
            // add vertecies
            int vertex_id = obstacle_map.insertVertex(x_list.at(i).at(vertex_j), y_list.at(i).at(vertex_j));
            if ( last_vertex_id!= -1){
                obstacle_map.insertEdge(vertex_id, last_vertex_id);
            }
            last_vertex_id = vertex_id;
        }
    }

    this -> obstacles_indices_list = obstacle_map.get_all_shapes();

    // have to do the following for loop separately since get_all_shapes do not return a sequence that follows all vertices.
    for(auto& obstacle_indices: obstacles_indices_list){
        int last_vertex_id = -1;
        vector<Vector2D> normal_vecs;
        for (unsigned int i = 0; i<obstacle_indices.size(); ++i){
            int vertex_id = obstacle_indices.at(i);
            //normal vector between the current index and the next index
            if ( last_vertex_id!= -1){
                Vector2D edge_vec = obstacle_map.vertex_list[vertex_id].coord - obstacle_map.vertex_list[last_vertex_id].coord;
                Vector2D normal_vec( edge_vec.y, -1.0* edge_vec.x);
                normal_vec.normalize_vec();
                normal_vecs.push_back(normal_vec);
            }
            last_vertex_id = vertex_id;
        }

        this->normal_vecs_list.push_back(normal_vecs);
    }
}

void PRM::add_free_vertices() {
    //TODO: delete
    vector<Vertex> vertices {Vertex(2, 2),
                             Vertex(1.5, 1.2),
                             Vertex(1.5, 0),
                             Vertex(5,4),
                             Vertex(5,6),
                             Vertex(1,0),
                             Vertex(0.5,0),
                             Vertex(0.5,0.5)};

    //TODO: add sampling
    for (auto vertex:vertices){
        if (!this->if_in_obstacle(vertex)){
            if (!this -> if_too_close(vertex)){
                this->free_node_map.insertVertex(vertex.coord.x, vertex.coord.y);
            }
        }
    }
}

void PRM::add_edges_to_N_neighbors(){
    auto& vertex_list = this->free_node_map.vertex_list;
    // traversing through all vertex combinations. n*(n-1)/2

    for (auto current_vertex_it = vertex_list.begin(); current_vertex_it!=vertex_list.end();++current_vertex_it){
        auto& current_vertex = (*current_vertex_it).second;
        vector<std::pair<int, Vertex> >unexplored_node_list(std::next(current_vertex_it, 1), vertex_list.end());
        //using Lambda Expression for sorting. Getting ascending list of vertices
        std::sort(unexplored_node_list.begin(), unexplored_node_list.end(),
                [&](std::pair<int, Vertex> pair_1, std::pair<int, Vertex> pair_2) -> bool{
                    return rigid2d::distance(pair_1.second.coord, current_vertex.coord) < rigid2d::distance(pair_2.second.coord, current_vertex.coord);
                    });

        for (auto neighbor_vertex_pair: unexplored_node_list){
            auto& neighbor_vertex = neighbor_vertex_pair.second;
            if (current_vertex.edge_list.size() >= this->pimpl->K || neighbor_vertex.edge_list.size() >= this->pimpl->K){
                break;
            }
            if(!this->if_edge_collide(current_vertex, neighbor_vertex)){
                cout<<current_vertex.coord<<" + "<<neighbor_vertex.coord<<endl; //TODO:delete
                this->free_node_map.insertEdge(current_vertex.id, neighbor_vertex.id);
            }
        }
    }
    free_node_map.printGraph(1);
}
//--------------------------------------------helper functions------------------------------------------------------------------

bool PRM::if_in_obstacle(const Vertex& P) const {

    for (unsigned int j = 0; j < this->obstacles_indices_list.size(); ++j){
        auto& obstacle_indices = obstacles_indices_list.at(j);//one obstacle shape
        for (unsigned int i = 0; i<obstacle_indices.size()-1; ++i){
            auto current_vertex = this->obstacle_map.vertex_list.at(obstacle_indices.at(i));
            Vector2D normal_vec = this->normal_vecs_list.at(j).at(i);
            Vector2D AP = P.coord - current_vertex.coord;
            if (AP*normal_vec >0){
                break;
            }
            if (i == obstacle_indices.size()-2){
                return true;
            }
        }
    }
    return false;
}

bool PRM::if_too_close(const Vertex &P) const {
    for(unsigned int j = 0; j < this->obstacles_indices_list.size(); ++j) {
        auto &obstacle_indices = obstacles_indices_list.at(j);
        // get all vertices of an obstacle (the last one is the starting point, thus omitted)
        for (unsigned int i = 0; i < obstacle_indices.size()-1; ++i) {
            Vector2D normal_vec = this->normal_vecs_list.at(j).at(i);
            auto V1 = this->obstacle_map.vertex_list.at(obstacle_indices.at(i));
            auto V2 = this->obstacle_map.vertex_list.at(obstacle_indices.at(i+1));
            Vector2D V1P = P.coord - V1.coord;
            Vector2D V1V2 = V2.coord - V1.coord;
            Vector2D V2P = P.coord - V2.coord;
            double bounding_r= this->pimpl->bounding_radius;

            // check point to line distance
            if (std::abs(V1P * normal_vec )< bounding_r){
                V1V2.normalize_vec();
                double V1P_Projection = V1P * (V1V2);
                // check if line is right above the V1V2 line segment, or it's very close to V1 or V2.  
                if ((V1P_Projection <= 1 && V1P_Projection >=0)
                    || length(V1P) <= bounding_r
                    || length(V2P) <= bounding_r){
//                    cout<<"V1P: "<<length(V1P)<< " V2P: "<<length(V2P)<<" bounding_r: "<<bounding_r<<endl;
                    return true;
                }
            }
        }
    }
    return false;
}

bool PRM::if_edge_collide(const Vertex &P1, const Vertex &P0)const {
    for(unsigned int j = 0; j < this->obstacles_indices_list.size(); ++j){
        auto& obstacle_indices = obstacles_indices_list.at(j);
        double te = 1; double tl = 0;
        // get all vertices of an obstacle (the last one is the starting point, thus omitted)
        for (unsigned int i = 0; i < obstacle_indices.size()-1; ++i){
            //collision condition 1: edge crossing
            Vector2D normal_vec = this->normal_vecs_list.at(j).at(i);
            auto current_vertex = this->obstacle_map.vertex_list.at(obstacle_indices.at(i));
            Vector2D P0V = current_vertex.coord - P0.coord;
            Vector2D P0P1 = P1.coord - P0.coord;
            // case where P0P1 is parallel to edge
            if(P0P1 * normal_vec == 0){
                return false;
            }
            else{
                double ti = (P0V * normal_vec)/(P0P1 * normal_vec);
                //p0p1 "entering" the edge
                if (P0P1 * normal_vec < 0){
                    te = std::min(te, ti);
                }
                else{
                    tl = std::max(tl, ti);
                }
            }
        }
        if (0<=te && te<= tl && tl<=1){
            return true;
        }
        else{
            return false;
        }
    }
}
