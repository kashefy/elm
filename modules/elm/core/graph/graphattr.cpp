/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphattr.h"

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/graph/graphattr_impl.h"

using namespace boost;
using namespace cv;
using namespace elm;

GraphAttr::~GraphAttr()
{
    impl.reset();
}

GraphAttr::GraphAttr()
    : impl(new GraphAttr_Impl())
{
}

GraphAttr::GraphAttr(const Mat1f &map_img, const Mat1b &mask)
    : impl(new GraphAttr_Impl(map_img, mask))
{
}

size_t GraphAttr::num_vertices() const
{
    return static_cast<size_t>(boost::num_vertices(impl->g));
}

float GraphAttr::operator ()(int idx_u, int idx_v) const
{
    GraphAttrTraits::vertex_descriptor u(idx_u);
    GraphAttrTraits::vertex_descriptor v(idx_v);

    typename boost::property_map < GraphAttrType, boost::edge_weight_t >::type
            weight = get(boost::edge_weight, impl->g);

    GraphAttrTraits::edge_descriptor e;
    bool found;

    boost::tie(e, found) = edge(u, v, impl->g);
    return found? get(weight, e) : 0.f;
}

void GraphAttr::AdjacencyMat(Mat1f &adj) const
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

VecF GraphAttr::VerticesIds() const
{
    VecF vtx_ids(num_vertices());

    property_map<GraphAttrType, vertex_color_t>::type
            vertex_color_id = get(vertex_color, impl->g);

    GraphAttrTraits::vertex_iterator vi, vi_end, next;
    tie(vi, vi_end) = vertices(impl->g);
    int i=0;
    for (next = vi; vi != vi_end; vi = next) {

        ++next;
        vtx_ids[i++] = vertex_color_id[*vi];
    }

    return vtx_ids;
}

void GraphAttr::addAttributes(float vtx_id, const Mat1f &attr)
{
    VtxDescriptor descriptor;
    if(impl->findVtxDescriptor(vtx_id, descriptor)) {

        property_map<GraphAttrType, vertex_index2_t>::type
                vertex_attributes = get(vertex_index2, impl->g);

        vertex_attributes[descriptor] = attr;
    }
    else {
        std::stringstream s;
        s << "No vertex with id (" << vtx_id << ").";
        ELM_THROW_KEY_ERROR(s.str());
    }
}

Mat1f GraphAttr::getAttributes(float vtx_id) const
{
    VtxDescriptor descriptor;
    if(impl->findVtxDescriptor(vtx_id, descriptor)) {

        property_map<GraphAttrType, vertex_index2_t>::type
                vertex_attributes = get(vertex_index2, impl->g);

        return vertex_attributes[descriptor];
    }
    else {
        std::stringstream s;
        s << "No vertex with id (" << vtx_id << ").";
        ELM_THROW_KEY_ERROR(s.str());
    }
}
