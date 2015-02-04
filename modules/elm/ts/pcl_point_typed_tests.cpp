/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#include "elm/ts/pcl_point_typed_tests.h"

#ifdef __WITH_PCL // following initializations only applicable with PCL support

ELM_SET_EXPECTED_POINT_ATTR(pcl::PointXYZ,    3,  4);  // x, y, z
ELM_SET_EXPECTED_POINT_ATTR(pcl::Normal,      4,  8);  // n1, n2, n3, curvature
ELM_SET_EXPECTED_POINT_ATTR(pcl::PointNormal, 7, 12);  // PointXYZ + Normal = 4+3

#endif // __WITH_PCL
