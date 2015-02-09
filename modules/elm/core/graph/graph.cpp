/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graph.h"
#include "elm/core/debug_utils.h"
#include "elm/core/graph/graph_impl.h"

using namespace elm;

Graph::~Graph()
{
    impl.reset();
}

Graph::Graph()
    : impl(new Graph_Impl())
{
}

Graph::Graph(int nb_vertices)
    : impl(new Graph_Impl(nb_vertices))
{
}

size_t Graph::num_vertices() const
{
    return static_cast<size_t>(boost::num_vertices(impl->g));
}

float Graph::operator ()(int idx_u, int idx_v) const
{
    GraphTraits::vertex_descriptor u(idx_u);
    GraphTraits::vertex_descriptor v(idx_v);

    typename boost::property_map < GraphType, boost::edge_weight_t >::type
            weight = get(boost::edge_weight, impl->g);

    GraphTraits::edge_descriptor e;
    bool found;

    boost::tie(e, found) = edge(u, v, impl->g);
    return found? get(weight, e) : 0.f;
}

#ifdef __WITH_PCL

Graph::Graph(const CloudXYZPtr &cld, const Triangles &t)
    : impl(new Graph_Impl(cld, t))
{
}

#endif // __WITH_PCL
