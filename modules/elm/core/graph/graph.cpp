/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graph.h"

#include <opencv2/core/core.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/graph/graph_impl.h"

using namespace cv;
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

void Graph::AdjacencyMat(Mat1f &adj) const
{
    int nb_vertices = static_cast<int>(num_vertices());
    if(adj.rows < nb_vertices || adj.cols < nb_vertices) {

        adj = Mat1f(nb_vertices, nb_vertices);
    }

    for(int idx_u=0; idx_u<nb_vertices; idx_u++) {

        for(int idx_v=0; idx_v<nb_vertices; idx_v++) {

            adj(idx_u, idx_v) = operator ()(idx_u, idx_v);
        }
    }
}

void Graph::AdjacencyMat(SparseMat1f &adj) const
{
    int nb_vertices = static_cast<int>(num_vertices());

    if((adj.size() == 0) || (nb_vertices < adj.size()[0]) || (nb_vertices < adj.size()[1])) {

        int dims = 2;
        const int _sizes[2] = {nb_vertices, nb_vertices};
        if(nb_vertices > 0) {

            adj = SparseMat1f(dims, _sizes);
        }
    }

    for(int idx_u=0; idx_u<nb_vertices; idx_u++) {

        for(int idx_v=0; idx_v<nb_vertices; idx_v++) {

            adj.ref(idx_u, idx_v) = operator ()(idx_u, idx_v);
        }
    }
}


Graph::Graph(const CloudXYZPtr &cld, const Triangles &t)
    : impl(new Graph_Impl(cld, t))
{
}

#endif // __WITH_PCL
