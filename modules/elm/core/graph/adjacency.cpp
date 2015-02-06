/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/adjacency.h"

#include <opencv2/core.hpp>

#include "elm/core/debug_utils.h"
#include "elm/core/exception.h"

#ifdef __WITH_PCL

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#include "elm/core/pcl/triangle_utils.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace elm;

void elm::TriangulatedCloudToAdjacencyX(const CloudXYZPtr &cld, const Triangles &t, Mat1f &dst)
{
    int nb_vertices = static_cast<int>(cld->size());
    dst = Mat1f::zeros(nb_vertices, nb_vertices);

    for(Triangles::const_iterator itr=t.begin(); itr!=t.end(); ++itr) {

        const Vertices V = *itr;

        if(V.vertices.size() < size_t(3)) {

            ELM_THROW_BAD_DIMS("triangle with less than 3 vertices.");
        }

        //foreach vertex
        uint32_t v0 = V.vertices[0];
        uint32_t v1 = V.vertices[1];
        uint32_t v2 = V.vertices[2];

        if(v0 >= nb_vertices || v1 >= nb_vertices || v2 >= nb_vertices) {

            ELM_THROW_KEY_ERROR("Triangle vertex outside point cloud.");
        }

        Mat1f e_triangle = TriangleEdges(cld->points.at(v0),
                                         cld->points.at(v1),
                                         cld->points.at(v2));

        dst(v0, v1) = e_triangle(0);
        dst(v0, v2) = e_triangle(1);
        dst(v1, v2) = e_triangle(2);
    }
}

void elm::TriangulatedCloudToAdjacencyX(const CloudXYZPtr &cld, const Triangles &t, SparseMat1f &dst)
{
    int nb_vertices = static_cast<int>(cld->size());

    int dims = 2;
    const int _sizes [2] = {nb_vertices, nb_vertices};
    SparseMat_<float> m(dims, _sizes);

    for(Triangles::const_iterator itr=t.begin(); itr!=t.end(); ++itr) {

        const Vertices V = *itr;

        if(V.vertices.size() < size_t(3)) {

            ELM_THROW_BAD_DIMS("triangle with less than 3 vertices.");
        }

        //foreach vertex
        uint32_t v0 = V.vertices[0];
        uint32_t v1 = V.vertices[1];
        uint32_t v2 = V.vertices[2];

        Mat1f e_triangle = TriangleEdges(cld->points.at(v0),
                                         cld->points.at(v1),
                                         cld->points.at(v2));
        m.ref(v0, v1) = e_triangle(0);
        m.ref(v0, v2) = e_triangle(1);
        m.ref(v1, v2) = e_triangle(2);
    }
}

#endif // __WITH_PCL
