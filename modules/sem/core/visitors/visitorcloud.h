#ifndef SEM_CORE_VISITORCLOUD_H_
#define SEM_CORE_VISITORCLOUD_H_

#ifdef __WITH_PCL // the following visitor derived class definition requires PCL support

#include "sem/core/typedefs_fwd.h"
#include "sem/core/pcl/typedefs_fwd.h"
#include "sem/core/pcl/cloud_.h"
#include "sem/core/visitors/visitor_.h"

/**
 * @brief visitor class for converting to pcl point cloud
 * And keeping track of when a heavy conversion (involving deep copy) occured
 *
 * Requires PCL support.
 */
class VisitorCloud :
        public Visitor_<sem::CloudXYZPtr >
{
public:
    void Reset();

    sem::CloudXYZPtr operator()(sem::CloudXYZPtr &c);

    sem::CloudXYZPtr operator()(const sem::VecVertices &vv);

    sem::CloudXYZPtr operator()(float f);

    sem::CloudXYZPtr operator()(int n);

    sem::CloudXYZPtr operator()(uchar c);

    sem::CloudXYZPtr operator()(const sem::Mat_f &m);

protected:
    sem::CloudXYZPtr c_; ///< internal reference for caching most recent conversion result
};

#else // __WITH_PCL
    #warning "Unable to define VisitorCloud visitor without PCL support."
#endif // __WITH_PCL

#endif // SEM_CORE_VISITORCLOUD_H_
