/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_GRAPHATTR_IMPL_H_
#define _ELM_CORE_GRAPH_GRAPHATTR_IMPL_H_

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/undirected_graph.hpp>

#include "elm/core/typedefs_fwd.h"

#include <opencv2/core/core.hpp>

namespace elm {

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProp;
typedef boost::property<boost::vertex_color_t, float,
        boost::property<boost::vertex_index2_t, cv::Mat1f > >
        VtxProp;
//typedef boost::undirected_graph<VtxProp, EdgeWeightProp> GraphAttrType;

typedef boost::adjacency_list<boost::setS, boost::listS, boost::undirectedS, VtxProp, EdgeWeightProp> GraphAttrType;

typedef boost::graph_traits<GraphAttrType> GraphAttrTraits;
typedef GraphAttrTraits::edge_iterator edge_iter;
typedef GraphAttrTraits::vertex_descriptor VtxDescriptor;

/**
 * @brief full GraphAttr implementation class
 * Enforces a list of unique vertices accroding to their main property
 */
struct GraphAttr_Impl
{
    static const float ID_UNASSIGNED;    ///< reserved for map element not assigned to a vertex

    /**
     * @brief Default constructor
     */
    GraphAttr_Impl();

    /**
     * @brief Construct an attributed Graph from map representing image
     * @param map
     * @param mask for excluding invalid elements (set to false to exclude)
     */
    GraphAttr_Impl(const cv::Mat1f &map_img, const cv::Mat1b &mask);

    /**
     * @brief find Vertex and get its descriptor
     * @param[in] vtx_id id of vertex to find
     * @param[out] vtx_descriptor vertex descriptor
     * @return true on found
     */
    bool findVertex(float vtx_id, VtxDescriptor &vtx_descriptor) const;

    /**
     * @brief remove edges between two vertices while obtaining their descriptors
     * @param[in] vtx_u id for first vertex
     * @param[in] vtx_v id for second vertex
     * @param[out] u descriptor for first vertex
     * @param[out] v descriptor for second vertex
     */
    void removeEdges(float vtx_u, float vtx_v, VtxDescriptor &u, VtxDescriptor &v);

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
    void removeVertex(float vtx_id, const VtxDescriptor &vtx);

    /**
     * @brief Keep track of Vertex substitutions
     * Substitutions used for updating map image
     * @param src source vertex
     * @param dst destination vertex (vertex' new id)
     */
    void recordVertexSubstitution(float src, float dst);

    /**
     * @brief Get graph's underlying map image
     * @return map image
     */
    cv::Mat1f MapImg();

    // public members
    GraphAttrType g;    ///< underlying graph object

protected:
    // typedefs
    typedef std::map<float, VtxDescriptor > MapVtxDescriptor;   ///< typedef for Map from vertex id to its descriptors
    typedef std::pair<float, float> VtxSubstitution;            ///< typedef for vector of recorded vertex substitutions

    /**
     * @brief retrieves vertex_descriptor of an existing or new vertex
     * @param value primary vertex identifying property (e.g. map img value)
     * @return vertex descriptor
     */
    VtxDescriptor retrieveVertex(float vtx_id);

    /**
     * @brief update map image to reflect any recent vertex substitutions
     */
    void updateMapImg();

    // members
    cv::Mat1f src_map_img_;         ///< source map image for this graph

    MapVtxDescriptor vtx_cache_;    ///< cache vertex descriptors

    std::vector<VtxSubstitution > vertex_subs_; ///< record vertex substitutions
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPHATTR_IMPL_H_
