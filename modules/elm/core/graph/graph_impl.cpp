/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/graph_impl.h"

#include <opencv2/core.hpp>

#ifdef __WITH_PCL

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#include "elm/core/pcl/triangle_utils.h"

#endif // __WITH_PCL

#include "elm/core/exception.h"

using cv::Mat1f;
using namespace elm;

Graph_Impl::Graph_Impl()
{
}

Graph_Impl::Graph_Impl(int nb_vertices)
    : g(nb_vertices)
{
}

size_t Graph_Impl::num_vertices() const
{
    return static_cast<size_t>(boost::num_vertices(g));
}

#ifdef __WITH_PCL

Graph_Impl::Graph_Impl(const CloudXYZPtr &cld, const Triangles &t)
{
    using namespace pcl;
    uint32_t nb_vertices_uint = static_cast<uint32_t>(cld->size());
    g = GraphType(static_cast<int>(cld->size()));

    for(Triangles::const_iterator itr=t.begin(); itr!=t.end(); ++itr) {

        const Vertices V = *itr;

        if(V.vertices.size() < size_t(3)) {

            ELM_THROW_BAD_DIMS("triangle with less than 3 vertices.");
        }

        //foreach vertex
        uint32_t v0 = V.vertices[0];
        uint32_t v1 = V.vertices[1];
        uint32_t v2 = V.vertices[2];

        if(v0 >= nb_vertices_uint || v1 >= nb_vertices_uint || v2 >= nb_vertices_uint) {

            ELM_THROW_KEY_ERROR("Triangle vertex outside point cloud.");
        }

        Mat1f e_triangle = TriangleEdges(cld->points.at(v0),
                                         cld->points.at(v1),
                                         cld->points.at(v2));

        EdgeWeightProperty e = e_triangle(0);
        add_edge(v0, v1, e, g);

        e = e_triangle(1);
        add_edge(v0, v2, e, g);

        e = e_triangle(2);
        add_edge(v1, v2, e, g);
    }
}

#endif // __WITH_PCL
