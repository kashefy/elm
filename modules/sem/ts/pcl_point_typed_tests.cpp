#include "sem/ts/pcl_point_typed_tests.h"

SEM_SET_EXPECTED_POINT_ATTR(pcl::PointXYZ,    3,  4);  // x, y, z
SEM_SET_EXPECTED_POINT_ATTR(pcl::Normal,      4,  8);  // n1, n2, n3, curvature
SEM_SET_EXPECTED_POINT_ATTR(pcl::PointNormal, 7, 12);  // PointXYZ + Normal = 4+3
