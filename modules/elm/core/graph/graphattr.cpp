/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphattr.h"

#include <boost/thread/thread.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/graph/base_GraphVertexOp.h"
#include "elm/core/graph/graphattr_impl.h"

using namespace boost;
using namespace cv;
using namespace elm;

typedef std::vector<VtxColor > VecColor;

GraphAttr::~GraphAttr()
{
    impl.reset();
}

GraphAttr::GraphAttr()
    : impl(new GraphAttr_Impl())
{
}

GraphAttr::GraphAttr(const Mat1i &map_img, const Mat1b &mask)
    : impl(new GraphAttr_Impl(map_img, mask))
{
}

size_t GraphAttr::num_vertices() const
{
    return static_cast<size_t>(boost::num_vertices(impl->g));
}

float GraphAttr::operator ()(int idx_u, int idx_v) const
{
    VecColor vtx_ids = VerticesIds();
    GraphAttrTraits::vertex_descriptor u, v;

    impl->findVertex(vtx_ids[idx_u], u);
    impl->findVertex(vtx_ids[idx_v], v);

    typename property_map < GraphAttrType, edge_weight_t >::type
            weight = get(edge_weight, impl->g);

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

    // a lot of clutter just to traverse all edges
    typename property_map <GraphAttrType, edge_weight_t >::type
            weight = get(edge_weight, impl->g);

    GraphAttrTraits::vertex_iterator begin, end, next_u;
    tie(begin, end) = vertices(impl->g);

    int idx_u=0;
    for (next_u = begin; next_u != end; ++next_u, idx_u++) {

        GraphAttrTraits::vertex_iterator next_v;
        int idx_v=0;
        for (next_v = begin; next_v != end; ++next_v, idx_v++) {

            GraphAttrTraits::edge_descriptor e;
            bool found;

            tie(e, found) = edge(*next_u, *next_v, impl->g);
            adj(idx_u, idx_v) = found? get(weight, e) : 0.f;
        }
    }
}

VecI GraphAttr::VerticesIds() const
{
    VecI vtx_ids(num_vertices());

    property_map<GraphAttrType, vertex_color_t>::type
            vtx_color_lut = get(vertex_color, impl->g);

    GraphAttrTraits::vertex_iterator vi, vi_end, next;
    tie(vi, vi_end) = vertices(impl->g);
    int i=0;
    for (next = vi; vi != vi_end; vi = next) {

        ++next;
        vtx_ids[i++] = vtx_color_lut[*vi];
    }

    return vtx_ids;
}

void GraphAttr::addAttributes(int vtx_id, const Mat1f &attr)
{
    VtxDescriptor descriptor;
    if(impl->findVertex(vtx_id, descriptor)) {

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

Mat1f GraphAttr::getAttributes(int vtx_id) const
{
    VtxDescriptor descriptor;
    if(impl->findVertex(vtx_id, descriptor)) {

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

Mat1f GraphAttr::applyVertexToMap(int vtx_id, Mat1f (*func)(const Mat1f &img, const Mat1b &mask)) const
{
    Mat1f vtx_result;

    VtxDescriptor vtx_descriptor;
    if(impl->findVertex(vtx_id, vtx_descriptor)) {

        property_map<GraphAttrType, vertex_color_t>::type
                vertex_color_id = get(vertex_color, impl->g);

        int vtx_color = vertex_color_id[vtx_descriptor];

        Mat1f map_img = impl->MapImg();
        Mat1b _mask = map_img == vtx_color;
        vtx_result = func(map_img, _mask);
    }
    else {
        std::stringstream s;
        s << "No vertex with id (" << vtx_id << ").";
        ELM_THROW_KEY_ERROR(s.str());
    }

    return vtx_result;
}

Mat1f GraphAttr::applyVertexOpToMap(int vtx_id, base_GraphVertexOp &vtx_op) const
{
    Mat1f vtx_result;

    VtxDescriptor vtx_descriptor;
    if(impl->findVertex(vtx_id, vtx_descriptor)) {

        property_map<GraphAttrType, vertex_color_t>::type
                vertex_color_id = get(vertex_color, impl->g);

        int vtx_color = vertex_color_id[vtx_descriptor];
        Mat1f map_img = impl->MapImg();
        Mat1b _mask = map_img == vtx_color;
        vtx_op.mutableOpCaller(map_img, _mask, vtx_result);
    }
    else {
        std::stringstream s;
        s << "No vertex with id (" << vtx_id << ").";
        ELM_THROW_KEY_ERROR(s.str());
    }

    return vtx_result;
}

VecMat1f GraphAttr::applyVerticesToMap(Mat1f (*func)(const Mat1f &img, const Mat1b &mask)) const
{
    VecMat1f results(num_vertices());

    property_map<GraphAttrType, vertex_color_t>::type
            vertex_color_id = get(vertex_color, impl->g);

    GraphAttrTraits::vertex_iterator vi, vi_end, next;
    tie(vi, vi_end) = vertices(impl->g);

    thread_group group;

    int i=0;
    for (next = vi; vi != vi_end; vi = ++next) {

        VtxColor vtx_color = vertex_color_id[*vi];

        group.create_thread(
                    boost::bind(&GraphAttr::apply_masked, this,
                                func,
                                vtx_color,
                                boost::ref(results[i++]))
                            );
    }
    group.join_all();

    return results;
}

int GraphAttr::contractEdges(int id_u, int id_v)
{
    VtxDescriptor u;
    VtxDescriptor v;

    // remove (u, v) and (v, u) edges
    impl->removeEdges(id_u, id_v, u, v);

    // TODO: merge all u and v out-edges with common targets
    // TODO: merge all u and v in-edges with common sources
    // move the rest of u out-edges to v
    // move the rest of u in-edges to v
    graph_traits<GraphAttrTraits>::out_edge_iterator e, e_end;
    //graph_traits<GraphAttrTraits>::edge_descriptor e_v;

    typename property_map <GraphAttrType, edge_weight_t >::type
            weight = get(edge_weight, impl->g);

    std::vector< std::pair<VtxDescriptor, VtxDescriptor> > obsolete_edges;

//    property_map<GraphAttrType, vertex_color_t>::type
//            vertex_color_id = get(vertex_color, impl->g);

    for(tie(e, e_end) = out_edges(u, impl->g); e != e_end; ++e) {

        VtxDescriptor src, dst;
        src = source(*e, impl->g);
        dst = target(*e, impl->g);

        // move or merge edge?
        if(src == u) {

//            EdgeWeightProp e_prop = get(weight, *e);
//            bool found;
//            boost::tie(e_v, found) = edge(v, dst, impl->g);

            add_edge(v, dst, get(weight, *e), impl->g); // move edge
        }
        else if(dst == u) {

            add_edge(src, v, get(weight, *e), impl->g); // move edge
        }
        else {

            std::stringstream s;
            s << "Neither edge source nor its target is  vertex u="<<
                 u << "with id=" << id_u << ".";
            ELM_THROW_VALUE_ERROR(s.str());
        }

        // keep track of edges to remove later
        obsolete_edges.push_back(std::make_pair(src, dst));
    }

    // must remove obsolete edges before removing vertex
    VtxDescriptor src, dst;
    for(size_t i=0; i<obsolete_edges.size(); i++) {

        tie(src, dst) = obsolete_edges[i];
        impl->removeEdges(src, dst);
    }

    impl->removeVertex(id_u, u);

    // let map reflect vertex merge
    impl->recordVertexSubstitution(id_u, id_v);

    return id_v;
}

void GraphAttr::removeEdges(int vtx_u, int vtx_v)
{
    VtxDescriptor u, v;
    return impl->removeEdges(vtx_u, vtx_v, u, v);
}

int GraphAttr::VertexIndex(int vtx_id) const
{
    VtxDescriptor v;
    if(!impl->findVertex(vtx_id, v)) {

        std::stringstream s;
        s << "No vertex with id (" << vtx_id << ").";
        ELM_THROW_KEY_ERROR(s.str());
    }

    VecI ids = VerticesIds();
    bool found = false;
    int idx = -1;
    for(int i=0; i < static_cast<int>(ids.size()) && !found; i++) {

        if(ids[i] == vtx_id) {

            idx = i;
            found = true;
        }
    }

    return idx;
}

VecI GraphAttr::getNeighbors(int vtx_id) const
{
    VtxDescriptor u;
    if(!impl->findVertex(vtx_id, u)) {

        std::stringstream s;
        s << "No vertex with id (" << vtx_id << ").";
        ELM_THROW_KEY_ERROR(s.str());
    }

    VecI neigh_ids;

    // Property accessor
    property_map<GraphAttrType, vertex_color_t>::type
            vertex_color_id = get(vertex_color, impl->g);

    graph_traits<GraphAttrTraits>::out_edge_iterator e, e_end;
    for(tie(e, e_end) = out_edges(u, impl->g); e != e_end; ++e) {

        VtxDescriptor src, dst;
        src = source(*e, impl->g);
        dst = target(*e, impl->g);

        // move or merge edge?
        if(src == u) {

            neigh_ids.push_back(vertex_color_id[dst]);
        }
        else if(dst == u) {

            neigh_ids.push_back(vertex_color_id[src]);
        }
        else {

            std::stringstream s;
            s << "Neither edge source nor its target is  vertex u="<<
                 u << "with id=" << vtx_id << ".";
            ELM_THROW_VALUE_ERROR(s.str());
        }
    }

    return neigh_ids;
}

void GraphAttr::removeVertex(int vtx_id)
{
    VtxDescriptor u;

    if(!impl->findVertex(vtx_id, u)) {

        std::stringstream s;
        s << "No vertex with id (" << vtx_id << ").";
        ELM_THROW_KEY_ERROR(s.str());
    }

    graph_traits<GraphAttrTraits>::out_edge_iterator e, e_end;

    std::vector< std::pair<VtxDescriptor, VtxDescriptor> > obsolete_edges;

    for(tie(e, e_end) = out_edges(u, impl->g); e != e_end; ++e) {

        VtxDescriptor src, dst;
        src = source(*e, impl->g);
        dst = target(*e, impl->g);

        // move or merge edge?
        if(src != u && dst != u) {

            std::stringstream s;
            s << "Neither edge source nor its target is vertex u="<<
                 u << "with id=" << vtx_id << ".";
            ELM_THROW_VALUE_ERROR(s.str());
        }

        // keep track of edges to remove later
        obsolete_edges.push_back(std::make_pair(src, dst));
    }

    // must remove obsolete edges before removing vertex
    VtxDescriptor src, dst;
    for(size_t i=0; i<obsolete_edges.size(); i++) {

        tie(src, dst) = obsolete_edges[i];
        impl->removeEdges(src, dst);
    }

    impl->removeVertex(vtx_id, u);

    // let map reflect vertex merge
    impl->recordVertexSubstitution(vtx_id, impl->ID_UNASSIGNED);
}

Mat1i GraphAttr::MapImg() const
{
    return impl->MapImg();
}

void GraphAttr::apply_masked(cv::Mat1f (*func) (const cv::Mat1f &img, const cv::Mat1b &mask),
                             int color,
                             Mat1f &dst) const
{
    Mat1i map_img = impl->MapImg();
    Mat1b _mask = (map_img == color);
    dst = func(map_img, _mask);
}

