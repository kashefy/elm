/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_GRAPHATTR_H_
#define _ELM_CORE_GRAPH_GRAPHATTR_H_

#include "elm/core/typedefs_fwd.h"
#include "elm/core/stl/typedefs.h"

namespace elm {

class GraphAttr_Impl;

/**
 * @brief class for exposing the public interface of GraphAttr_Impl
 * An attributed graph class
 * where the attributes are encapsulated inside a single-channel OpenCV Mat of floats
 * External face of GraphAttr pimpl.
 */
class GraphAttr
{
public:
    virtual ~GraphAttr();

    GraphAttr();

    /**
     * @brief Construct and attributed Graph from map representing image
     * @param map
     * @param mask for excluding invalid elements
     */
    GraphAttr(const cv::Mat1f &map_img, const cv::Mat1b &mask);

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
    std::shared_ptr<GraphAttr_Impl> impl;
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPHATTR_H_