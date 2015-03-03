/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_GRAPHATTR_H_
#define _ELM_CORE_GRAPH_GRAPHATTR_H_

#include "elm/core/typedefs_sfwd.h"

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

    /**
     * @brief add Attributes to vertex
     * @param vtx_id vertex identifier using primary vertex property
     * @param attr
     * @throws elm::ExceptionKeyError for non-existing vertex id
     */
    void addAttributes(float vtx_id, const cv::Mat1f &attr);

    /**
     * @brief get Attributes to vertex
     * @param vtx_id vertex identifier using primary vertex property
     *
     * @return attributes represented in secondary vertex index property
     *
     * @throws elm::ExceptionKeyError for non-existing vertex id
     */
    cv::Mat1f getAttributes(float vtx_id) const;

    /**
     * @brief apply function to source image map masked by vertex
     * @param func point to function to apply on masked image
     * @param vtx_id for mapping source map image
     * @return result of function application
     * @throws elm::ExceptionKeyError for invalid vertex id
     */
    cv::Mat1f applyVertexToMap(float vtx_id, cv::Mat1f (*func) (const cv::Mat1f &img, const cv::Mat1b &mask)) const;

    /**
     * @brief apply function to source image map masked by vertex for each vertex
     * @param func point to function to apply on masked image
     * @param vtx_id for mapping source map image
     * @return result of function application for each vertex
     */
    VecMat1f applyVerticesToMap(cv::Mat1f (*func) (const cv::Mat1f &img, const cv::Mat1b &mask)) const;

    /**
     * @brief remove edge(s) between two vertices
     * @param vtx_u id for vertex u
     * @param vtx_v id for vertex v
     * @throws elm::ExceptionKeyError for invalid vertex id
     */
    void removeEdges(float vtx_u, float vtx_v);

    /**
     * @brief get the index of a vertex in adjacency matrix
     * @param vtx vertex id
     * @return vertex index
     * @throws elm::ExceptionKeyError for invalid vertex id
     */
    int VertexIndex(float vtx) const;

    /**
     * @brief contract edges/merge two vertices
     *
     * remove (u, v) and (v, u) edges
     * merge all u and v out-edges with common targets
     * merge all u and v in-edges with common sources
     * move the rest of u out-edges to v
     * move the rest of u in-edges to v
     *
     * @param id_u id for vertex u (e.g. color)
     * @param id_v id for vertex v (e.g. color)
     * @return index of merged vertex (idx_v)
     * @throws elm::ExceptionKeyError for invalid vertex index
     */
    float contractEdges(float id_u, float id_v);

    /**
     * @brief Get list of neighboring vertices
     * @param vtx_id vertex id
     * @return list of vertex ids neighboring given vertex
     */
    VecF getNeighbors(float vtx_id) const;

    /**
     * @brief remove a vertex from the graph
     *
     * Sets corresponding pixels in source map image to zero.
     *
     * @param vtx_id id of vertex to remove
     * @throws elm::ExceptionKeyError for invalid vertex id
     */
    void removeVertex(float vtx_id);

    // public members
    std::shared_ptr<GraphAttr_Impl> impl;
};

/**
 * @brief apply function to image after masking it by color (mask := img == color)
 * @param color
 * @param img
 * @param dst
 */
void apply_masked(cv::Mat1f (*func) (const cv::Mat1f &img, const cv::Mat1b &mask),
                  float color,
                  const cv::Mat1f &img,
                  cv::Mat1f &dst);

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPHATTR_H_
