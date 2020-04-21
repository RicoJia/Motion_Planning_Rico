//
// Created by ricojia on 4/10/20.
//

#ifndef MOTION_PLANNING_CODE_MAP_H
#define MOTION_PLANNING_CODE_MAP_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include "rigid2d.hpp"

namespace PRM_Grid{

    struct Edge{
        int pointing_id;    // id of the vertex the edge is "pointing" to .
        double distance;        //weight
        Edge(): pointing_id(-1), distance(-1.0){}
        Edge(int id, double dist): pointing_id(id), distance(dist){}
    };

    struct Vertex{
        int id;
        rigid2d::Vector2D coord;        //2D coordinate
        std::map<int, Edge> edge_list;     // adjacent nodes
        bool visited;                   // if the node has been visited during traversal

        Vertex(): id(-1), visited(false){}
        Vertex(double x, double y): id(-1), coord(rigid2d::Vector2D(x,y)), visited(false){}
        Vertex(int id, double x, double y): id(id), coord(rigid2d::Vector2D(x,y)), visited(false){}
    };


/// \brief: Undirected map that consists of vertices and edges. This is essentially a representation of connections between vertices
/// this implementation is capable of: printing smallest closed shapes (i.e, if two closed shapes are connected together, both will be printed).
/// All vertices in the map have unique coordinates.
/// Possible optimization: 1. std::vector -> std::list if index does not matter.  2. erase in vector.
/// Improvements: 1. print_graph function can print unique shapes.

    class Map {
    public:
        Map():next_vertex_id(0){};

        /// \brief: insert point (x,y) into the map
        /// \param: x, y: x,y of the point.
        /// \return: vertex id
        int insertVertex(double x, double y);

        /// \brief: delete a vertex based on its id.
        /// \param: vertex id
        /// \return: true for success, false for failure
        bool deleteVertex(int vertex_id);

        /// \brief: insert an edge between two vertices. Distance will be automatically calculated
        /// \param: edge
        /// \return: true for success, false for failure (vertex not found)
        bool insertEdge(int vertex_id1, int vertex_id2);

        /// \brief: delete an edge between two vertices
        /// \param: indices between two edges
        /// \return: true for success, false for failure
        bool deleteEdge(int vertex_id1, int vertex_id2);

        /// \brief: print out the graph connections with node ids.
        /// \param: print_mode: 0 for shapes, 1 for unique edges.
        void printGraph(bool print_mode=0);

        /// \brief: search for (x,y) in the map. Returns the id of the point or -1. O(n)
        /// \param: x, y: x,y of the point.
        /// \return: integer id of the point, or if nothing is found, -1.
        int search(double x, double y);

        /// \brief: get all shapes from the map
        /// \return: vectors of coneected points' indices (in vectors)
        std::vector< std::vector<int> > get_all_shapes();

        /// \brief: get all unique edges from the map (brute force search)
        /// \return: vectors of coneected points' indices (in vectors)
        std::vector< std::vector<int> > get_all_unique_edges() ;

        std::unordered_map<int, Vertex> vertex_list;            // list of vertices, with their id the Vertex

    private:
        int next_vertex_id;                                        // the id for the vertex to be added. this only increases so the id of each vertex is unique.

        /// \brief: Push all next connected points' vertices into shape vector
        /// \param: current_id - id of vertex to be investigated
        /// \param: head_id - the first vertex's id
        /// \param: last_id - last vertex's id
        /// \param: shape - vector that stores all vertices of a shape
        void add_next_vertices(int current_id, std::vector<int>& shape, int head_id, int last_id=-1);
    };

}

#endif //MOTION_PLANNING_CODE_MAP_H
