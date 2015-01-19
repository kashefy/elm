#ifndef SEM_CORE_VISITORCLOUD__H_
#define SEM_CORE_VISITORCLOUD__H_

#ifdef __WITH_PCL // the following visitor derived template class definitions require PCL support

#include "sem/core/pcl/typedefs_fwd.h"
#include "sem/core/cv/typedefs_fwd.h"
#include "sem/core/typedefs_fwd.h"
#include "sem/core/visitors/visitor_.h"

namespace sem {

/**
 * @brief A template visitor class for converting to pcl point clouds,
 * and keeping track of when a heavy conversion (involving deep copy) occured
 *
 * Requires PCL support.
 */
template <class TPoint>
class VisitorCloud_ :
        public Visitor_<boost::shared_ptr<pcl::PointCloud<TPoint > > >
{
public:
    typedef boost::shared_ptr<pcl::PointCloud<TPoint > > CloudTPPtr;

    void Reset();

    CloudTPPtr operator()(CloudTPPtr &c);

    CloudTPPtr operator()(const sem::VecVertices &vv);

    CloudTPPtr operator()(float f);

    CloudTPPtr operator()(int n);

    CloudTPPtr operator()(uchar c);

    CloudTPPtr operator()(const sem::Mat_f &m);

protected:
     CloudTPPtr c_; ///< internal reference for caching most recent conversion result
};

} // namespace sem

#else // __WITH_PCL
    #warning "Unable to define VisitorCloud_ visitor without PCL support."
#endif // __WITH_PCL

#endif // SEM_CORE_VISITORCLOUD__H_
