#ifndef SEM_CORE_VISITORCLOUD__H_
#define SEM_CORE_VISITORCLOUD__H_

#ifdef __WITH_PCL // the following visitor derived template class definitions require PCL support

#include "sem/core/pcl/typedefs_fwd.h"
#include "sem/core/visitors/visitor_.h"

template <class TPoint>
class VisitorCloud_ :
        public Visitor_<boost::shared_ptr<pcl::PointCloud<TPoint > > >
{
public:
    VisitorCloud_();
};

#endif // __WITH_PCL

#endif // SEM_CORE_VISITORCLOUD__H_
