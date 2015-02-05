/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_TRIANGLE_UTILS_H_
#define _ELM_CORE_TRIANGLE_UTILS_H_


#ifdef __WITH_PCL

#include "elm/core/pcl/typedefs_fwd.h"
#include "elm/core/cv/typedefs_fwd.h"

namespace elm {

cv::Mat1f TriangleEdges(const pcl::PointXYZ &p0,
                        const pcl::PointXYZ &p1,
                        const pcl::PointXYZ &p2);

}

#endif // __WITH_PCL

#endif // _ELM_CORE_TRIANGLE_UTILS_H_
