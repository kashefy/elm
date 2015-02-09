/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_GRAPH_H_
#define _ELM_CORE_GRAPH_GRAPH_H_

#include "elm/core/typedefs_fwd.h"
#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {

class Graph_Impl;
class MatrixGraph;

/**
 * @brief class for exposing public interface of Graph_Impl
 * External face of Graph pimpl.
 */
class Graph
{
public:
    virtual ~Graph();

    Graph();

    /**
     * @brief Construct Graph with reserved no. of verticies.
     * @param nb_vertices no. of vertices
     */
    Graph(int nb_vertices);

    /**
     * @brief get no. of vertices in graph
     * @return no. of vertices in graph
     */
    size_t num_vertices() const;

    /**
     * @brief get edge weight between two vertices
     * @param idx_u index of vertex u
     * @param idx_v index of vertex v
     * @return edge weight between vertices u and v
     * @throws elm::ExceptionBadDims if verticies don't add up to form triangle
     * @throws elm::ExceptionKeyError if a triangle vertex is beyond cloud points
     */
    float operator()(int idx_u, int idx_v) const;

    /**
     * @brief Get adjacency matrix for this graph
     * @param[out] Dense adjacency matrix
     */
    void AdjacencyMat(cv::Mat1f &adj) const;

    /**
     * @brief Get adjacency matrix for this graph
     * @param[out] Sparse adjacency matrix
     */
    void AdjacencyMat(SparseMat1f &adj) const;

#ifdef __WITH_PCL

    /**
     * @brief Construct Graph from triangulated point cloud
     * @param cld point coloud
     * @param t triangulated vertices for point cloud
     */
    Graph(const CloudXYZPtr &cld, const Triangles &t);

#endif // __WITH_PCL

    std::shared_ptr<Graph_Impl> impl;
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPH_H_
