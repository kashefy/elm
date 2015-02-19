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

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> GraphType;
typedef boost::graph_traits<GraphType> GraphTraits;
typedef GraphTraits::edge_iterator edge_iter;

/**
 * @brief full GraphMap implementation class
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
     */
    GraphMap_Impl(const cv::Mat1f &map_img);

    // members
    GraphType g;    ///< underlying graph object
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPHMAP_IMPL_H_
