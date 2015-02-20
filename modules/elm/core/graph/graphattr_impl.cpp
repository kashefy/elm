/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphattr_impl.h"

#include <opencv2/core/core.hpp>

#include "elm/core/exception.h"
#include "elm/core/stl/stl.h"

using namespace std;
using namespace cv;
using namespace elm;

GraphAttr_Impl::GraphAttr_Impl()
{
}

GraphAttr_Impl::GraphAttr_Impl(const cv::Mat1f &map_img, const Mat1b &mask)
{
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
            VtxDescriptor cur = retrieveVtxDescriptor(value_cur);
            vertex_color_id[cur] = value_cur;

            if(c < map_img.cols-1) {

                if(!is_masked || mask(r, c+1)) {

                    float value = map_img(r, c+1);
                    VtxDescriptor right = retrieveVtxDescriptor(value);
                    vertex_color_id[right] = value;

                    if(cur != right) {

                        add_edge(cur, right, EDGE_CONNECTED, g);
                    }
                } // skip right?
            } // not last column

            if(r < map_img.rows-1) {

                if(!is_masked || mask(r+1, c)) {

                    float value = map_img(r+1, c);
                    VtxDescriptor down = retrieveVtxDescriptor(value);
                    vertex_color_id[down] = value;

                    if(cur != down) {

                        add_edge(cur, down, EDGE_CONNECTED, g);
                    }
                } // skip down?
            } // not last row
        } // column
    } // row
}

VtxDescriptor GraphAttr_Impl::retrieveVtxDescriptor(float vtx_id)
{
    VtxDescriptor descriptor;

    // if found, return cached descriptor
    if(!findVtxDescriptor(vtx_id, descriptor)) {

        // otherwise, request new descriptor
        descriptor = boost::add_vertex(vtx_id, g);

        // cache new descriptor
        vtx_descriptor_cache_[vtx_id] = descriptor;
    }

    return descriptor;
}

bool GraphAttr_Impl::findVtxDescriptor(float vtx_id, VtxDescriptor &vtx_descriptor) const
{
    MapVtxDescriptor::const_iterator itr = vtx_descriptor_cache_.find(vtx_id);
    bool found = itr != vtx_descriptor_cache_.end();
    if(found) {

        vtx_descriptor = itr->second; // get cached descriptor
    }

    return found;
}
