/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graphmap_impl.h"

#include <opencv2/core/core.hpp>

#include "elm/core/exception.h"
#include "elm/core/stl/stl.h"

using namespace std;
using namespace cv;
using namespace elm;

GraphMap_Impl::GraphMap_Impl()
{
}

GraphMap_Impl::GraphMap_Impl(const cv::Mat1f &map_img, const Mat &mask)
{
    ELM_THROW_BAD_DIMS_IF(map_img.total()==static_cast<size_t>(1),
                          "Cannot create graph out of single element map.");

    ELM_THROW_BAD_DIMS_IF(!mask.empty() &&
                          (mask.rows != map_img.rows || mask.cols != map_img.cols),
                          "non-empty mask does not match input map.");

    g = GraphMapType(); // no. vertices unknown yet

    EdgeWeightProperty EDGE_CONNECTED = 1;

    for(int r=0; r<map_img.rows; r++) {

        for(int c=0; c<map_img.cols; c++) {

            // Property accessors
            boost::property_map<GraphMapType, boost::vertex_color_t>::type
                    vertex_color_id = get(boost::vertex_color, g);

            float value_cur = map_img(r, c);
            VtxDescriptor cur = retrieveVtxDescriptor(value_cur);
            vertex_color_id[cur] = static_cast<int>(value_cur);

            if(c < map_img.cols-1) {

                float value = map_img(r, c+1);
                VtxDescriptor right = retrieveVtxDescriptor(value);
                vertex_color_id[right] = static_cast<int>(value);

                if(cur != right) {

                    add_edge(cur, right, EDGE_CONNECTED, g);
                }
            }

            if(r < map_img.rows-1) {

                float value = map_img(r+1, c);
                VtxDescriptor down = retrieveVtxDescriptor(value);
                vertex_color_id[down] = static_cast<int>(value);

                if(cur != down) {

                    add_edge(cur, down, EDGE_CONNECTED, g);
                }
            }
        } // column
    } // row
}

GraphMap_Impl::VtxDescriptor GraphMap_Impl::retrieveVtxDescriptor(float value)
{
    VtxDescriptor descriptor;
    MapVtxDescriptor::const_iterator itr = vtx_descriptor_cache_.find(value);
    bool found = itr != vtx_descriptor_cache_.end();
    if(found) {

        descriptor = itr->second; // get cached descriptor
    }
    else {
        descriptor = boost::add_vertex(value, g); // get new descriptor

        // cache new descriptor
        vtx_descriptor_cache_[value] = descriptor;
    }

    return descriptor;
}
