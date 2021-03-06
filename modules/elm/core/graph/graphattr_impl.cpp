/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphattr_impl.h"

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/stl/stl_inl.h"

using namespace std;
using namespace cv;
using namespace elm;

const int GraphAttr_Impl::ID_UNASSIGNED = 0;

GraphAttr_Impl::~GraphAttr_Impl()
{
}

GraphAttr_Impl::GraphAttr_Impl()
    : is_map_dirty_(false)
{
}

GraphAttr_Impl::GraphAttr_Impl(const cv::Mat_<VtxColor > &map_img,
                               const Mat1b &mask)
    : src_map_img_(map_img),
      is_map_dirty_(false)
{
    // some validation first
    ELM_THROW_BAD_DIMS_IF(map_img.total()==static_cast<size_t>(1),
                          "Cannot create graph out of single element map.");

    ELM_THROW_BAD_DIMS_IF(!mask.empty() &&
                          (mask.rows != map_img.rows || mask.cols != map_img.cols),
                          "non-empty mask does not match input map.");

    g = GraphAttrType(); // no. of vertices unknown yet

    vtx_cache_.reserve(src_map_img_.total()*2+1); // some sufficiently high number

    bool is_masked = !mask.empty();

    EdgeWeightProp EDGE_CONNECTED = 1.f;

    // Property accessors
    boost::property_map<GraphAttrType, boost::vertex_color_t>::type
            vtx_color_lut = get(boost::vertex_color, g);

    bool is_new_vertex;

    for(int r=0; r<map_img.rows; r++) {

        for(int c=0; c<map_img.cols; c++) {

            if(is_masked && !mask(r, c)) {

                continue; // skip iteration at masked element;
            }

            VtxColor value_cur = map_img(r, c);
            VtxDescriptor cur = retrieveVertex(value_cur, is_new_vertex);

            if(is_new_vertex) {

                vtx_color_lut[cur] = value_cur;
            }

            if(c < map_img.cols-1) {

                if(!is_masked || mask(r, c+1)) {

                    VtxColor value = map_img(r, c+1);
                    VtxDescriptor right = retrieveVertex(value, is_new_vertex);

                    if(cur != right) {

                        if(is_new_vertex) {

                            vtx_color_lut[right] = value;
                        }

                        add_edge(cur, right, EDGE_CONNECTED, g);
                    }
                } // skip right?
            } // not last column

            if(r < map_img.rows-1) {

                if(!is_masked || mask(r+1, c)) {

                    VtxColor value = map_img(r+1, c);
                    VtxDescriptor down = retrieveVertex(value, is_new_vertex);

                    if(cur != down) {

                        if(is_new_vertex) {

                            vtx_color_lut[down] = value;
                        }

                        add_edge(cur, down, EDGE_CONNECTED, g);
                    }
                } // skip down?
            } // not last row
        } // column
    } // row
}

VtxDescriptor GraphAttr_Impl::retrieveVertex(VtxColor vtx_id, bool &is_new)
{
    VtxDescriptor descriptor;

    // if found, return cached descriptor

    if(!findVertex(vtx_id, descriptor)) {

        // otherwise, request new descriptor
        descriptor = boost::add_vertex(vtx_id, g);

        // cache new descriptor
        vtx_cache_.insert(vtx_id, descriptor);

        is_new = true;
    }
    else {
        is_new = false;
    }

    return descriptor;
}

bool GraphAttr_Impl::findVertex(VtxColor vtx_id, VtxDescriptor &vtx_descriptor) const
{
    bool found = vtx_cache_.find(vtx_id, vtx_descriptor);
    return found;
}

void GraphAttr_Impl::removeEdges(const VtxDescriptor &u, const VtxDescriptor &v)
{
    GraphAttrTraits::edge_descriptor e;
    bool found;

    boost::tie(e, found) = edge(u, v, g);

    if(found) {

        remove_edge(e, g);
    }
}

void GraphAttr_Impl::removeEdges(VtxColor vtx_u, VtxColor vtx_v, VtxDescriptor &u, VtxDescriptor &v)
{
    if(!findVertex(vtx_u, u)) {

        std::stringstream s;
        s << "No vertex with id (" << vtx_u << ").";
        ELM_THROW_KEY_ERROR(s.str());
    }

    if(!findVertex(vtx_v, v)) {

        std::stringstream s;
        s << "No vertex with id (" << vtx_v << ").";
        ELM_THROW_KEY_ERROR(s.str());
    }

    return removeEdges(u, v);
}

void GraphAttr_Impl::removeVertex(VtxColor vtx_id, const VtxDescriptor &vtx)
{
    boost::remove_vertex(vtx, g);

    /* Removing a vertex from boost graph resets
     * all descriptors to the [0, V-1]
     * so we need to update the cache
     * to reflect new vertex descriptors
     */

    // Iterate through the vertices and add them to new cache
    vtx_cache_.remove(vtx_id);

    boost::property_map<GraphAttrType, boost::vertex_color_t>::type
            vertex_color_id = get(boost::vertex_color, g);

    GraphAttrTraits::vertex_iterator v, end;
    for(tie(v, end) = vertices(g); v != end; ++v) {

        VtxColor key = vertex_color_id[*v];
        vtx_cache_.insert(key, *v);
    }

    is_map_dirty_ = true;
}

void GraphAttr_Impl::recordVertexSubstitution(VtxColor src, VtxColor dst)
{
    //vertex_subs_.push_back(src, dst);
    vtx_cache_.recordSubstitution(src, dst);
}

Mat_<VtxColor > GraphAttr_Impl::MapImg()
{
    if(is_map_dirty_) {

        updateMapImg();
    }

    return src_map_img_;
}

void GraphAttr_Impl::updateMapImg()
{
    // update map image with any recorded vertex substitutions
    // vertex substitutions could have come from contractEdges()

    if(src_map_img_.isContinuous()) {

        int *mat_data_ptr = reinterpret_cast<int*>(src_map_img_.data);
        const int *end = reinterpret_cast<const int*>(src_map_img_.dataend);

        for(; mat_data_ptr != end; mat_data_ptr++) {

            int tmp = vtx_cache_.Id(*mat_data_ptr);
            *mat_data_ptr = tmp < 0 ? 0 : tmp;
        }
    }
    else {

        const int NB_ELEMENTS = static_cast<int>(src_map_img_.total());
        for(int i=0; i<NB_ELEMENTS; i++) {

            int tmp = vtx_cache_.Id(src_map_img_(i));
            src_map_img_(i) = tmp < 0 ? 0 : tmp;
        }
    }
}
