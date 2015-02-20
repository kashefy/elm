/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_GRAPHMAP_IMPL_H_
#define _ELM_CORE_GRAPH_GRAPHMAP_IMPL_H_

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#include "elm/core/typedefs_fwd.h"

namespace elm {

typedef boost::property<boost::edge_weight_t, int> EdgeWeightProperty;
typedef boost::property<boost::vertex_color_t, float> VtxProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VtxProperty, EdgeWeightProperty> GraphMapType;
typedef boost::graph_traits<GraphMapType> GraphMapTraits;
typedef GraphMapTraits::edge_iterator edge_iter;

/**
 * @brief full GraphMap implementation class
 * Enforces a list of unique vertices accroding to their main property
 */
struct GraphMap_Impl
{
    /**
     * @brief Default constructor
     */
    GraphMap_Impl();

    /**
     * @brief Construct Graph from map representing image
     * @param map
     * @param mask for excluding invalid elements (set to false to exclude)
     */
    GraphMap_Impl(const cv::Mat1f &map_img, const cv::Mat1b &mask);

    // public members
    GraphMapType g;    ///< underlying graph object

protected:
    // typedefs
    typedef GraphMapTraits::vertex_descriptor VtxDescriptor;
    typedef std::map<float, VtxDescriptor > MapVtxDescriptor;

    /**
     * @brief retrieves vertex_descriptor of an existing or new vertex
     * @param value primary vertex identifying property (e.g. map img value)
     * @return vertex descriptor
     */
    VtxDescriptor retrieveVtxDescriptor(float value);

    MapVtxDescriptor vtx_descriptor_cache_;   ///< cache vertex descriptors
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPHMAP_IMPL_H_
