/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_GRAPHMAP_H_
#define _ELM_CORE_GRAPH_GRAPHMAP_H_

#include "elm/core/typedefs_fwd.h"
#include "elm/core/stl/typedefs.h"

namespace elm {

class GraphMap_Impl;

/**
 * @brief class for exposing the public interface of GraphMap_Impl
 * External face of GraphMap pimpl.
 */
class GraphMap
{
public:
    virtual ~GraphMap();

    GraphMap();

    /**
     * @brief Construct Graph from map representing image
     * @param map
     * @param mask for excluding invalid elements
     */
    GraphMap(const cv::Mat1f &map_img, const cv::Mat &mask);

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
     *
     * A new Mat object will only be constructed if not previously alocated
     * or allocated dimensions are not sufficient.
     *
     * @param[out] Dense adjacency matrix
     */
    void AdjacencyMat(cv::Mat1f &adj) const;

    /**
     * @brief get list of vertex ids represented by their primary unique property
     *
     * The ordering matches occurrence in adjacency matrix
     *
     * @return list vertex ids
     */
    VecF VerticesIds() const;

    // members
    std::shared_ptr<GraphMap_Impl> impl;
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPHMAP_H_
