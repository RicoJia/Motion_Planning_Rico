#ifndef PRM_PRM_H
#define  PRM_PRM_H

/// \brief: The Probablisitic RoadMap Class.
/// Possible optimization:
///     1. you don't need a full graph implemenation for PRM.
///     2. get_all_edges is brute force o(n2) sort.
///     3. sorting for KNN in add_edges_to_N_neighbors is kind of brute force. O(n^2 log n) cuz we are not using tree structure

#include <memory>
#include "map.hpp"
#include <iostream>
#include <vector>
#include <cstdlib>
#include <iterator>
#include <algorithm>
#include <utility>

class PRM{
public:
    PRM();

    /// \brief: move constructor and assignment constructor so that PRM object can be moved (due to Pimpl idiom)
    PRM(PRM&&);
    PRM& operator= (PRM&&);   //TODO: why double &&?
    ///\brief: Expicit destructor so unique pointer can be freed.
    ~PRM();       //TODO: why do we need explicit default constructor? and do default move and copy assignments simple copy everything to the new obejct?

    /// \brief: adding obstacles and outward normal vectors to obstacle_map and normal_vec_list
    void add_obstacles_and_normal_vecs();

    /// \brief sample and add free vertices to free_node_map.
    void add_free_vertices();

    /// \brief: identify K neighbours for each vertex and add edges between them.
    void add_edges_to_N_neighbors();
    //----------------------------------Public interface of PRM--------------------------------
private:
    /// \brief: private data hiding technique called pimpl idiom.
    struct impl;
    std::unique_ptr<impl> pimpl;
    std::vector< std::vector<rigid2d::Vector2D> > normal_vecs_list;     //normal vectors for all obstacle edges
    std::vector< std::vector<int> > obstacles_indices_list;

    Map obstacle_map;          //map that consists of obstacle vertices
    Map free_node_map;         // map that contains free nodes

    /// \brief: check if a vertex is inside an obstacle
    /// \param: Vertex: a vertex
    /// \return: false if not in obstacle, true if it is in obstacle.
    bool if_in_obstacle(const Vertex& P) const;

    /// \brief: check if a vertex is too close to an obstacle (within hte bounding radius)
    /// \param: vertex P
    /// \return: true if P is too close to an obstacle. Else false.
    bool if_too_close(const Vertex& P) const;

    /// \brief: checks if an edge between two vertices will collide with any obstacle
    /// \params: vertex A and B
    /// \return: true if an edge collides with an obstacle, false if not.
    bool if_edge_collide(const Vertex& A, const Vertex& B) const;

};
#endif //PRM_PRM_H