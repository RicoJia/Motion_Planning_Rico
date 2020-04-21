#ifndef PRM_PRM_H
#define  PRM_PRM_H

/// \brief: The Probablisitic RoadMap Class.
/// Possible optimization:
///     1. you don't need a full graph implemenation for PRM.
///     2. get_all_edges is brute force o(n2) sort.
///     3. sorting for KNN in add_edges_to_N_neighbors is kind of brute force. O(n^2 log n) cuz we are not using tree structure
///     4. I implemented some useless features... such as get_all_shapes, which can be subsituted by a single vector.
///     5. Pimpl idiom is not the most convenient to use as it is read only object?
/// Lessons Learned:
///     1. fully understand the math first. My edge collision and occupancy algorithms did not work.
///     2. Be careful with names in params.yaml.
///     3. The search algorithm for map does not return a counter-clockwise cycle. Therefore even though it returns a closed shape, this cannot be used for counter-clockwise shape determination
///     4. tolerance is necessary, as numerical error is inevitable.
///     5. I had deep bugs in my rigid2d code!!!

#include <memory>
#include "map.hpp"
#include <iostream>
#include <vector>
#include <cstdlib>
#include <iterator>
#include <algorithm>
#include <utility>
#include <XmlRpcValue.h>
#include <random>

namespace PRM_Grid{
    class PRM{
    public:
        PRM();

        /// \brief: move constructor and assignment constructor so that PRM object can be moved (due to Pimpl idiom)
        PRM(PRM&&);
        PRM& operator= (PRM&&);   //TODO: why double &&?
        ///\brief: Expicit destructor so unique pointer can be freed.
        virtual ~PRM();       //TODO: why do we need explicit default constructor? and do default move and copy assignments simple copy everything to the new obejct?

        /// \brief: adding obstacles and outward normal vectors to obstacle_map and normal_vec_list
        void add_obstacles_and_normal_vecs(XmlRpc::XmlRpcValue& obstacle_list, double coord_multiplier);

        /// \brief sample and add free vertices to free_node_map.
        virtual void add_free_vertices(double bounding_r, int sample_size, const std::vector<double>& map_x_lims, const std::vector<double>& map_y_lims, double cell_size);

        /// \brief: identify K neighbours for each vertex and add edges between them.
        void add_edges_to_N_neighbors(unsigned int K, double bounding_r);

        /// \brief: return all free vertices
        /// \return: free vertex list
        std::unordered_map<int, Vertex> get_free_map_vertices();

        /// \brief: return all edges with vertex ids.
        /// \return: free edges list
        std::vector< std::vector<int> > get_free_edges();

        /// \brief: return all obstacle vertices
        /// \return: obstacle list
        std::unordered_map<int, Vertex> get_obstacle_vertices_list();

        /// \brief: get all obstacle edges with their ids.
        /// \return obstacle ids
        std::vector< std::vector<int> > get_obstacle_edges();

        /// \brief: checks if an edge between two vertices will collide with any obstacle
        /// \params: vertex A and B
        /// \return: true if an edge collides with an obstacle, false if not.
        bool if_edge_collide(const Vertex& A, const Vertex& B) const;

        //----------------------------------Public interface of PRM--------------------------------
    protected:
        /// \brief: private data hiding technique called pimpl idiom.
        struct impl;
        std::unique_ptr<impl> pimpl;
        std::vector< std::vector<rigid2d::Vector2D> > normal_vecs_list;     //normal vectors for all obstacle edges
        std::vector< std::vector<int> > obstacles_indices_list; //for closed shapes , it should be like [1,2,3,1].

        Map obstacle_map;          //map that consists of obstacle vertices
        Map free_node_map;         // map that contains free nodes

        /// \brief: check if a vertex is inside an obstacle
        /// \param: Vertex: a vertex
        /// \return: false if not in obstacle, true if it is in obstacle.
        bool if_in_obstacle(const Vertex& P) const;

        /// \brief: check if a vertex is too close to an obstacle (within hte bounding radius)
        /// \param: vertex P
        /// \return: true if P is too close to an obstacle. Else false.
        bool if_too_close(const Vertex& P, double bounding_r) const;

        /// \brief: check if an edge between two free vertices is too close to any obstacles
        /// \param: two free vertices
        /// \return: true if not close, else false.
        bool if_edge_too_close_to_obstacle(const Vertex& V1, const Vertex& V2, double bounding_r) const;

    };
}

#endif //PRM_PRM_H
