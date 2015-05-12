/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/graph/adjacency.h"

#include <opencv2/core/core.hpp>

#include "elm/core/exception.h"

#ifdef __WITH_PCL

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#endif // __WITH_PCL

#include "elm/core/pcl/triangle_utils.h"

using namespace std;
using namespace cv;
using namespace elm;

extern template class cv::Mat_<float>;

#ifdef __WITH_PCL

extern template class pcl::PointCloud<pcl::PointXYZ >;

using namespace pcl;

void elm::TriangulatedCloudToAdjacency(const CloudXYZPtr &cld, const Triangles &t, Mat1f &dst)
{
    uint32_t nb_vertices_uint = static_cast<uint32_t>(cld->size());
    int nb_vertices = static_cast<int>(nb_vertices_uint);

    if(dst.rows < nb_vertices || dst.cols < nb_vertices) {

        dst = Mat1f::zeros(nb_vertices, nb_vertices);
    }

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

        dst(v0, v1) = dst(v1, v0) = e_triangle(0);
        dst(v0, v2) = dst(v2, v0) = e_triangle(1);
        dst(v1, v2) = dst(v2, v1) = e_triangle(2);
    }
}

void elm::TriangulatedCloudToAdjacency(const CloudXYZPtr &cld, const Triangles &t, SparseMat1f &dst)
{
    uint32_t nb_vertices_uint = static_cast<uint32_t>(cld->size());
    int nb_vertices = static_cast<int>(nb_vertices_uint);

    if((dst.size() == 0) || (nb_vertices < dst.size()[0]) || (nb_vertices < dst.size()[1])) {

        const int DIMS = 2;
        const int _sizes[DIMS] = {nb_vertices, nb_vertices};
        if(nb_vertices > 0) {

            dst = SparseMat1f(DIMS, _sizes);
        }
    }

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

        // force symmetry
        dst.ref(v0, v1) = e_triangle(0);
        dst.ref(v1, v0) = e_triangle(0);

        dst.ref(v0, v2) = e_triangle(1);
        dst.ref(v2, v0) = e_triangle(1);

        dst.ref(v1, v2) = e_triangle(2);
        dst.ref(v2, v1) = e_triangle(2);
    }
}

#endif // __WITH_PCL
