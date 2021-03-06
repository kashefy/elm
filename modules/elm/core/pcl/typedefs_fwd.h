/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/** forward declarations of pcl data types
 */
#ifndef _ELM_CORE_PCL_TYPEDEFS_FWD_H_
#define _ELM_CORE_PCL_TYPEDEFS_FWD_H_

#ifdef __WITH_PCL

#include "elm/core/boost/typedefs_fwd.h"
#include "elm/core/stl/typedefs.h"

namespace pcl {

struct PointXYZ;

struct Normal;

struct PointNormal;

template <typename TPoint> class PointCloud;

struct Vertices;

} // fake namespace pcl for fwd declaration

namespace elm {

typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;

typedef boost::shared_ptr<CloudXYZ > CloudXYZPtr;

typedef pcl::PointCloud<pcl::Normal> CloudNormal;

typedef boost::shared_ptr<CloudNormal > CloudNrmlPtr;

typedef pcl::PointCloud<pcl::PointNormal> CloudPtNrml;

typedef boost::shared_ptr<CloudPtNrml > CloudPtNrmlPtr;

typedef std::vector<pcl::Vertices > VecVertices;

typedef VecVertices Triangles;

} // namespace elm

#endif // __WITH_PCL

#endif // _ELM_CORE_PCL_TYPEDEFS_FWD_H_
