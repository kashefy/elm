/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
/** forward declarations of pcl data types
 */
#ifndef ELM_CORE_PCL_TYPEDEFS_FWD_H_
#define ELM_CORE_PCL_TYPEDEFS_FWD_H_

#ifndef __WITH_PCL
    #warning "Skipping pcl utilities since PCL support is disabled."
#else // __WITH_PCL

#include "elm/core/boost/typedefs_fwd.h"
#include "elm/core/stl/typedefs.h"

namespace pcl {

class PointXYZ;

class Normal;

class PointNormal;

template <typename TPoint> class PointCloud;

class Vertices;

} // fake namespace pcl for fwd declaration

namespace elm {

typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;

typedef boost::shared_ptr<CloudXYZ > CloudXYZPtr;

typedef pcl::PointCloud<pcl::Normal> CloudNormal;

typedef boost::shared_ptr<CloudNormal > CloudNrmlPtr;

typedef pcl::PointCloud<pcl::PointNormal> CloudPtNrml;

typedef boost::shared_ptr<CloudPtNrml > CloudPtNrmlPtr;

typedef std::vector<pcl::Vertices > VecVertices;

} // namespace elm

#endif // __WITH_PCL

#endif // ELM_CORE_PCL_TYPEDEFS_FWD_H_
