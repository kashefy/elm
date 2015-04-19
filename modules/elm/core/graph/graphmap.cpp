/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphmap.h"

#include <opencv2/core/core.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/graph/graphmap_impl.h"

using namespace boost;
using namespace cv;
using namespace elm;

GraphMap::~GraphMap()
{
    impl.reset();
}

GraphMap::GraphMap()
    : impl(new GraphMap_Impl())
{
}

GraphMap::GraphMap(const Mat1f &map_img, const Mat1b &mask)
    : impl(new GraphMap_Impl(map_img, mask))
{
}

size_t GraphMap::num_vertices() const
{
    return static_cast<size_t>(boost::num_vertices(impl->g));
}

float GraphMap::operator ()(int idx_u, int idx_v) const
{
    GraphMapTraits::vertex_descriptor u(idx_u);
    GraphMapTraits::vertex_descriptor v(idx_v);

#if _MSC_VER && !__INTEL_COMPILER
#else // #if _MSC_VER && !__INTEL_COMPILER
	typename
#endif // #if _MSC_VER && !__INTEL_COMPILER
		boost::property_map < GraphMapType, boost::edge_weight_t >::type
            weight = get(boost::edge_weight, impl->g);

    GraphMapTraits::edge_descriptor e;
    bool found;

    boost::tie(e, found) = edge(u, v, impl->g);
    return found? get(weight, e) : 0.f;
}

void GraphMap::AdjacencyMat(Mat1f &adj) const
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

VecF GraphMap::VerticesIds() const
{
    VecF vtx_ids(num_vertices());

    property_map<GraphMapType, vertex_color_t>::type
            vertex_color_id = get(vertex_color, impl->g);

    GraphMapTraits::vertex_iterator vi, vi_end, next;
    tie(vi, vi_end) = vertices(impl->g);
    int i=0;
    for (next = vi; vi != vi_end; vi = next) {

        ++next;
        vtx_ids[i++] = vertex_color_id[*vi];
    }

    return vtx_ids;
}

