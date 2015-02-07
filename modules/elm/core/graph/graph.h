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

#ifdef __WITH_PCL

    /**
     * @brief Construct Graph from triangulated point cloud
     * @param cld point coloud
     * @param t triangulated vertices for point cloud
     */
    Graph(const CloudXYZPtr &cld, const Triangles &t);

#endif // __WITH_PCL

    Graph_Impl *impl;
};

} // namespace elm

#endif // _ELM_CORE_GRAPH_GRAPH_H_
