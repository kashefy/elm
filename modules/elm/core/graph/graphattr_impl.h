/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_GRAPHATTR_IMPL_H_
#define _ELM_CORE_GRAPH_GRAPHATTR_IMPL_H_

#include "elm/core/cv/deferredassignhashed_.h"
#include "elm/core/graph/vertexcache.h"

namespace elm {

/**
 * @brief full GraphAttr implementation class
 * Enforces a list of unique vertices accroding to their main property
 * @todo split map image from this class
 */
class GraphAttr_Impl
{
public:
    static const int ID_UNASSIGNED;    ///< reserved for map element not assigned to a vertex

    /** @brief Default constructor
     */
    ~GraphAttr_Impl();

    /** @brief Default constructor
     */
    GraphAttr_Impl();

    /**
     * @brief Construct an attributed Graph from map representing image
     * @param map
     * @param mask for excluding invalid elements (set to false to exclude)
     */
    GraphAttr_Impl(const cv::Mat1i &map_img, const cv::Mat1b &mask);

    /**
     * @brief find Vertex and get its descriptor
     * @param[in] vtx_id id of vertex to find
     * @param[out] vtx_descriptor vertex descriptor
     * @return true on found
     */
    bool findVertex(int vtx_id, VtxDescriptor &vtx_descriptor) const;

    /**
     * @brief remove edges between two vertices while obtaining their descriptors
     * @param[in] vtx_u id for first vertex
     * @param[in] vtx_v id for second vertex
     * @param[out] u descriptor for first vertex
     * @param[out] v descriptor for second vertex
     */
    void removeEdges(int vtx_u, int vtx_v, VtxDescriptor &u, VtxDescriptor &v);

    /**
     * @brief remove edges between two vertices
     * @param u first vertex
     * @param v second vertex
     */
    void removeEdges(const VtxDescriptor &u, const VtxDescriptor &v);

    /**
     * @brief remove Vertex
     * @param vtx_id vertex id to remove
     * @param vtx descriptor of vertex to remove
     */
    void removeVertex(int vtx_id, const VtxDescriptor &vtx);

    /**
     * @brief Keep track of Vertex substitutions
     * Substitutions used for updating map image
     * @param src source vertex
     * @param dst destination vertex (vertex' new id)
     */
    void recordVertexSubstitution(int src, int dst);

    /**
     * @brief Get graph's underlying map image
     * @return map image
     */
    cv::Mat_<VtxColor > MapImg();

    // public members
    GraphAttrType g;    ///< underlying graph object

protected:
    /**
     * @brief retrieves vertex_descriptor of an existing or new vertex
     * @param[in] value primary vertex identifying property (e.g. map img value)
     * @param[out] flag indicating if a new certex was created
     * @return vertex descriptor
     */
    VtxDescriptor retrieveVertex(VtxColor vtx_id, bool &is_new);

    /**
     * @brief update map image to reflect any recent vertex substitutions
     */
    void updateMapImg();

    // members
    cv::Mat_<VtxColor > src_map_img_;         ///< source map image for this graph

    VertexCache vtx_cache_;    ///< cache vertex descriptors

    elm::DeferredAssign_<VtxColor > vertex_subs_; ///< keep track of vertex substitutions

    bool is_map_dirty_;
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPHATTR_IMPL_H_
