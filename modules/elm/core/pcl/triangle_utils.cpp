/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/core/pcl/triangle_utils.h"

#include <opencv2/core.hpp>

#ifdef __WITH_PCL

#include <pcl/point_types.h>

#include "elm/core/pcl/point_traits.h"

using namespace cv;
using namespace pcl;
using namespace elm;

Mat1f elm::TriangleEdges(const PointXYZ &p0,
                         const PointXYZ &p1,
                         const PointXYZ &p2)
{
    Mat1f edges(1, 3);

    const int nb_coords = elm::PCLPointTraits_<PointXYZ>::NbFloats();

    // Wrap Mat around coordinates of each point
    // Because we're given const points,
    // We can use the const_cast on the point data for constructing the Mat
    // but we need to make it a const Mat to honor the argument's constness
    const Mat1f c0(1, nb_coords, const_cast<float *>(p0.data));
    const Mat1f c1(1, nb_coords, const_cast<float *>(p1.data));

    edges(0) = static_cast<float>(cv::norm(c0-c1));

    const Mat1f c2(1, nb_coords, const_cast<float *>(p2.data));

    edges(1) = static_cast<float>(cv::norm(c0-c2));
    edges(2) = static_cast<float>(cv::norm(c1-c2));

    return edges;
}


#endif // __WITH_PCL
