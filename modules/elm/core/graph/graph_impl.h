/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_GRAPH_GRAPH_IMPL_H_
#define _ELM_CORE_GRAPH_GRAPH_IMPL_H_

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

#ifdef __WITH_PCL

#include "elm/core/typedefs_fwd.h"
#include "elm/core/pcl/typedefs_fwd.h"

#endif // __WITH_PCL

namespace elm {

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> GraphType;
typedef boost::graph_traits<GraphType>::edge_iterator edge_iter;

/**
 * @brief full Graph implementation class
 */
struct Graph_Impl
{
    /**
     * @brief Default constructor
     */
    Graph_Impl();

    /**
     * @brief Alternate consturctor for constructing a graph with reserved no. of vertices
     * @param nb_vertices no. of graph vertices
     */
    Graph_Impl(int nb_vertices);

    size_t num_vertices() const;

#ifdef __WITH_PCL

    /**
     * @brief Construct graph from triangulated point cloud
     * @param cld point cloud making out the grpah vertices
     * @param t triangles making out the edges connecting the points in the point cloud
     */
    Graph_Impl(const CloudXYZPtr &cld, const Triangles &t);

#endif // __WITH_PCL

    GraphType g;    ///< underlying graph object
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPH_IMPL_H_
