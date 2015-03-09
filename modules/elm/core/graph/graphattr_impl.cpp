/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphattr_impl.h"

#include <opencv2/core/core.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"
#include "elm/core/stl/stl_inl.h"

using namespace std;
using namespace cv;
using namespace elm;

const float GraphAttr_Impl::ID_UNASSIGNED = 0.f;

GraphAttr_Impl::GraphAttr_Impl()
{
}

GraphAttr_Impl::GraphAttr_Impl(const cv::Mat1f &map_img,
                               const Mat1b &mask)
    : src_map_img_(map_img)
{
    // some validation first
    ELM_THROW_BAD_DIMS_IF(map_img.total()==static_cast<size_t>(1),
                          "Cannot create graph out of single element map.");

    ELM_THROW_BAD_DIMS_IF(!mask.empty() &&
                          (mask.rows != map_img.rows || mask.cols != map_img.cols),
                          "non-empty mask does not match input map.");

    g = GraphAttrType(); // no. of vertices unknown yet

    bool is_masked = !mask.empty();

    EdgeWeightProp EDGE_CONNECTED = 1.f;

    for(int r=0; r<map_img.rows; r++) {

        for(int c=0; c<map_img.cols; c++) {

            if(is_masked && !mask(r, c)) {

                continue; // skip iteration at masked element;
            }

            // Property accessors
            boost::property_map<GraphAttrType, boost::vertex_color_t>::type
                    vertex_color_id = get(boost::vertex_color, g);

            float value_cur = map_img(r, c);
            VtxDescriptor cur = retrieveVertex(value_cur);
            vertex_color_id[cur] = value_cur;

            if(c < map_img.cols-1) {

                if(!is_masked || mask(r, c+1)) {

                    float value = map_img(r, c+1);
                    VtxDescriptor right = retrieveVertex(value);
                    vertex_color_id[right] = value;

                    if(cur != right) {

                        add_edge(cur, right, EDGE_CONNECTED, g);
                    }
                } // skip right?
            } // not last column

            if(r < map_img.rows-1) {

                if(!is_masked || mask(r+1, c)) {

                    float value = map_img(r+1, c);
                    VtxDescriptor down = retrieveVertex(value);
                    vertex_color_id[down] = value;

                    if(cur != down) {

                        add_edge(cur, down, EDGE_CONNECTED, g);
                    }
                } // skip down?
            } // not last row
        } // column
    } // row
}

VtxDescriptor GraphAttr_Impl::retrieveVertex(float vtx_id)
{
    VtxDescriptor descriptor;

    // if found, return cached descriptor
    if(!findVertex(vtx_id, descriptor)) {

        // otherwise, request new descriptor
        descriptor = boost::add_vertex(vtx_id, g);

        // cache new descriptor
        vtx_cache_[vtx_id] = descriptor;
    }

    return descriptor;
}

bool GraphAttr_Impl::findVertex(float vtx_id, VtxDescriptor &vtx_descriptor) const
{
    MapVtxDescriptor::const_iterator itr = vtx_cache_.find(vtx_id);
    bool found = itr != vtx_cache_.end();
    if(found) {

        vtx_descriptor = itr->second; // get cached descriptor
    }

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

void GraphAttr_Impl::removeEdges(float vtx_u, float vtx_v, VtxDescriptor &u, VtxDescriptor &v)
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

void GraphAttr_Impl::removeVertex(float vtx_id, const VtxDescriptor &vtx)
{
    boost::remove_vertex(vtx, g);

    /* Removing a vertex from boost graph resets
     * all descriptors to the [0, V-1]
     * so we need to update the cache
     * to reflect new vertex descriptors
     */

    // Iterate through the vertices and add them to new cache
    vtx_cache_.clear();

    boost::property_map<GraphAttrType, boost::vertex_color_t>::type
            vertex_color_id = get(boost::vertex_color, g);

    GraphAttrTraits::vertex_iterator v, end;
    for(tie(v, end) = vertices(g); v != end; ++v) {

        vtx_cache_[vertex_color_id[*v]] = *v;
    }
}

void GraphAttr_Impl::recordVertexSubstitution(float src, float dst)
{
    vertex_subs_.push_back(std::make_pair(src, dst));
}

Mat1f GraphAttr_Impl::MapImg()
{
    updateMapImg();
    return src_map_img_;
}

void GraphAttr_Impl::updateMapImg()
{
    // update map image with any recorded vertex substitutions
    // vertex substitutions could have come from contractEdges()

    const int nb_subs = static_cast<int>(vertex_subs_.size());

    // traverse backwards in substitution list
    // to short-circuit intermediate substitutions
    for(int i=nb_subs-1; i>=0; i--) {

        float src_i, dst_i;
        boost::tie(src_i, dst_i) = vertex_subs_[i];

        if(src_i != dst_i) {

            for(int j=i-1; j>=0; j--) {

                float src_j, dst_j;
                boost::tie(src_j, dst_j) = vertex_subs_[j];

                if(src_i == dst_j) {

                    vertex_subs_[j].second = dst_i;
                }
            }
        }
    }

    // forward traversal by OR-ing masks with common destination value
    for(size_t i=0; i<vertex_subs_.size(); i++) {

        float src_i, dst_i;
        boost::tie(src_i, dst_i) = vertex_subs_[i];

        if(src_i != dst_i) {

            Mat mask = src_map_img_ == src_i;

            for(size_t j=i+1; j<vertex_subs_.size(); j++) {

                float src_j, dst_j;
                boost::tie(src_j, dst_j) = vertex_subs_[j];

                if(dst_j == dst_i) {

                    Mat mask_j = src_map_img_ == src_j;

                    if(countNonZero(mask_j) > 0) {

                        cv::bitwise_or(mask, mask_j, mask, mask_j);
                    }

                    vertex_subs_[j].second = src_j;
                }
            }

            if(countNonZero(mask) > 0) {

                src_map_img_.setTo(dst_i, mask); // overwrite values with OR'ed mask
            }
        }
    }

    vertex_subs_.clear();
}
