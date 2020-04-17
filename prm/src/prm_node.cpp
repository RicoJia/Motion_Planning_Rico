//
// Created by ricojia on 4/15/20.
//
#include "../include/prm/PRM.hpp"
//#include <XmlRpcValue.h>

int main(){
    PRM prm;
    prm.add_obstacles_and_normal_vecs();
    prm.add_free_vertices();
    prm.add_edges_to_N_neighbors();
    return 0;
}
