#ifndef SEM_CORE_VISITORCLOUD_H_
#define SEM_CORE_VISITORCLOUD_H_

#ifdef __WITH_PCL // the following visitor derived class definitions requires PCL support

#include "core/pcl_utils.h"
#include "core/visitors/visitor_.h"

/**
 * @brief visitor class for converting to pcl point cloud
 * And keeping track of when a heavy conversion (involving deep copy) occured
 *
 * Requires PCL support.
 */
class VisitorCloud :
        public Visitor_<sem::CloudXYZ::Ptr >
{
public:
    void Reset();

    sem::CloudXYZ::Ptr operator()(sem::CloudXYZ::Ptr &c);

    sem::CloudXYZ::Ptr operator()(const sem::VecVertices &vv);

    sem::CloudXYZ::Ptr operator()(float f);

    sem::CloudXYZ::Ptr operator()(int n);

    sem::CloudXYZ::Ptr operator()(uchar c);

    sem::CloudXYZ::Ptr operator()(const Mat_f &m);

protected:
    sem::CloudXYZ::Ptr c_; ///< internal reference for caching most recent conversion result
};

#else // __WITH_PCL
    #warning "Unable to define VisitorCloud visitor without PCL support."
#endif // __WITH_PCL

#endif // SEM_CORE_VISITORCLOUD_H_
