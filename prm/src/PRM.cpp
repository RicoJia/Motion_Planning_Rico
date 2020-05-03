
#include "../include/prm/PRM.hpp"


using std::cout;
using std::cin;
using std::endl; 
using std::vector;
using rigid2d::Vector2D;
using rigid2d::length;
using namespace PRM_Grid;
//--------------------------------------------pimpl idiom for data hiding------------------------------------------------------------------
struct PRM::impl{
    const double map_x_max = 10;
    const double map_y_max = 10;
    const double bounding_radius = 0.15;      //0.15m
    const double K = 2;
    impl(){}
};

PRM::PRM():pimpl(new impl)
{}
PRM::PRM(PRM&&) = default;
PRM& PRM::operator=(PRM &&) = default;
PRM::~PRM() = default;


//--------------------------------------------helper functions------------------------------------------------------------------
/// \brief Create a twister for random sampling
/// \returns a twister for random sampling
static std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

bool PRM::if_in_obstacle(const Vertex& P) const {

    for (unsigned int j = 0; j < this->obstacles_indices_list.size(); ++j){
        auto& obstacle_indices = obstacles_indices_list.at(j);//one obstacle shape
        for (unsigned int i = 0; i<obstacle_indices.size()-1; ++i){
            auto current_vertex = this->obstacle_map.vertex_list.at(obstacle_indices.at(i));
            auto next_vertex = this->obstacle_map.vertex_list.at(obstacle_indices.at(i+1));
            Vector2D AP = P.coord - current_vertex.coord;
            Vector2D AB = next_vertex.coord - current_vertex.coord;
            if (AP.x*AB.y - AP.y*AB.x>1e-5){
                break;
            }
            if (i == obstacle_indices.size()-2){
                return true;
            }
        }
    }
    return false;
}

bool PRM::if_too_close(const Vertex &P, double bounding_r) const {
    for(unsigned int j = 0; j < this->obstacles_indices_list.size(); ++j) {
        auto &obstacle_indices = obstacles_indices_list.at(j);
        // get all vertices of an obstacle (the last one is the starting point, thus omitted)
        for (unsigned int i = 0; i < obstacle_indices.size()-1; ++i) {
            auto V1 = this->obstacle_map.vertex_list.at(obstacle_indices.at(i));
            auto V2 = this->obstacle_map.vertex_list.at(obstacle_indices.at(i+1));
            Vector2D V1P = P.coord - V1.coord;
            Vector2D V1V2 = V2.coord - V1.coord;
            Vector2D V2P = P.coord - V2.coord;

            double u = V1P * V1V2/(V1V2 * V1V2);
            if (0<=u && u<= 1){
                auto V = V1.coord + u * V1V2;
                double shortest_dist = rigid2d::distance(V, P.coord);
                if (shortest_dist <= bounding_r){
                    return true;
                }
            }

            if (rigid2d::length(V1P) <= bounding_r || rigid2d::length(V2P) <= bounding_r){
                return true;
            }
        }
    }
    return false;
}


bool PRM::if_edge_too_close_to_obstacle(const Vertex &V1, const Vertex &V2, double bounding_r) const {
    for (auto& obstacle_indices: this->obstacles_indices_list){
        for (int obstacle_index: obstacle_indices){
            auto P = this->obstacle_map.vertex_list.at(obstacle_index);
            Vector2D V1P = P.coord - V1.coord;
            Vector2D V1V2 = V2.coord - V1.coord;
            Vector2D V2P = P.coord - V2.coord;
            double u = V1P * V1V2/(V1V2 * V1V2);
            if (0<=u && u<= 1){
                auto V = V1.coord + u * V1V2;
                double shortest_dist = rigid2d::distance(V, P.coord);
                if (shortest_dist <= bounding_r){
                    return true;
                }
            }
            if (rigid2d::length(V1P) <= bounding_r || rigid2d::length(V2P) <= bounding_r){
                return true;
            }
        }
    }
    return false;
}

bool PRM::if_edge_collide(const Vertex &P1, const Vertex &P0)const {
    for(unsigned int j = 0; j < this->obstacles_indices_list.size(); ++j){
        auto& obstacle_indices = obstacles_indices_list.at(j);
        double te = 0; double tl = 1;
        bool if_break = false;
        // get all vertices of an obstacle (the last one is the starting point, thus omitted)
        for (unsigned int i = 0; i < obstacle_indices.size()-1; ++i){
            //collision condition 1: edge crossing
            Vector2D normal_vec = this->normal_vecs_list.at(j).at(i);
            auto current_vertex = this->obstacle_map.vertex_list.at(obstacle_indices.at(i));
            Vector2D P0V = current_vertex.coord - P0.coord;
            Vector2D P0P1 = P1.coord - P0.coord;
            double D = P0P1 * normal_vec;
            double N = P0V * normal_vec;
            // Segment parallel to edge, and outside of the shape
            if(D == 0){
                if (N<0){
                    if_break = true;
                    break;
                }
            }
            else{
                double ti = N/D;

                //p0p1 "entering" the edge
                if (D < 0){
                    te = std::max(te, ti);
                    if (te > tl){
                        if_break = true;
                        break;
                    }
                }
                else{
                    tl = std::min(tl, ti);
                    if(tl < te){
                        if_break = true;
                        break;
                    }
                }
            }
        }

        // collides with one shape
        if (0<=te && te<= tl && tl<=1 && if_break== false){
            return true;
        }
        if_break = false;

    }
    return false;
}

//--------------------------------------------Interface functions------------------------------------------------------------------

void PRM::add_obstacles_and_normal_vecs( XmlRpc::XmlRpcValue& obstacle_list, double coord_multiplier) {

    for (int i=0; i<obstacle_list.size(); ++i) {
        int last_vertex_id = -1;
        //get into a shape.
        std::vector<int> obstacle_vertices;
        for (int j = 0; j < obstacle_list[i].size(); ++j) {
            int vertex_id = obstacle_map.insertVertex(double(obstacle_list[i][j][0])*coord_multiplier, double(obstacle_list[i][j][1])*coord_multiplier);
            if ( last_vertex_id!= -1){
                obstacle_map.insertEdge(vertex_id, last_vertex_id);
            }
            obstacle_vertices.push_back(vertex_id);
            last_vertex_id = vertex_id;
        }
        this -> obstacles_indices_list.push_back(obstacle_vertices);
    }

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
                normal_vecs.push_back(normal_vec);
            }
            last_vertex_id = vertex_id;
        }
        this->normal_vecs_list.push_back(normal_vecs);
    }
}

void PRM::add_free_vertices(double bounding_r, int sample_size, const std::vector<double>& map_x_lims, const std::vector<double>& map_y_lims,
                            double cell_size){
    std::uniform_real_distribution<> x_gen(map_x_lims.at(0) * cell_size, map_x_lims.at(1)*cell_size);
    std::uniform_real_distribution<> y_gen(map_y_lims.at(0)* cell_size, map_y_lims.at(1)* cell_size);
    for (int i = 0; i<sample_size; ++i){
        double x = x_gen(get_random());
        double y = y_gen(get_random());
        Vertex vertex(x,y);
        if (!this->if_in_obstacle(vertex)){
            if (!this -> if_too_close(vertex, bounding_r)){
                this->free_node_map.insertVertex(vertex.coord.x, vertex.coord.y);
            }
        }
    }
}

bool PRM::add_additional_free_vertices(const std::vector<Vertex>& free_vertices, double bounding_r){
    for (auto vertex: free_vertices){
        if (!this->if_in_obstacle(vertex)) {
            if (!this->if_too_close(vertex, bounding_r)) {
                this->free_node_map.insertVertex(vertex.coord.x, vertex.coord.y);
            }
            else{
                ROS_WARN_STREAM("Invalid Additional Vertex: ("<<vertex.coord.x<<", "<<vertex.coord.y<<")");
                return false;
            }
        }
        else{
            ROS_WARN_STREAM("Invalid Additional Vertex: ("<<vertex.coord.x<<", "<<vertex.coord.y<<")");
            return false;
        }
    }
    return true;
}

void PRM::add_edges_to_N_neighbors(unsigned int K, double bounding_r){
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
            if (current_vertex.edge_list.size() >= K || neighbor_vertex.edge_list.size() >= K){
                break;
            }
            if(!this->if_edge_collide(current_vertex, neighbor_vertex)){
                if(!this->if_edge_too_close_to_obstacle(current_vertex, neighbor_vertex, bounding_r)){
                    this->free_node_map.insertEdge(current_vertex.id, neighbor_vertex.id);
                }
            }
        }
    }
}


std::unordered_map<int, Vertex> PRM::get_free_map_vertices(){
    return this->free_node_map.vertex_list;
}

std::vector< std::vector<int> > PRM::get_free_edges(){
    return this->free_node_map.get_all_unique_edges();
}

std::unordered_map<int, Vertex> PRM::get_obstacle_vertices_list(){
    return this->obstacle_map.vertex_list;
}

std::vector< std::vector<int> > PRM::get_obstacle_edges(){
    return this->obstacle_map.get_all_shapes();
}

int PRM::search_free_vertex(double x, double y){
    return this->free_node_map.search(x, y);
}