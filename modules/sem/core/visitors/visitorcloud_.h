#ifndef SEM_CORE_VISITORCLOUD__H_
#define SEM_CORE_VISITORCLOUD__H_

#ifdef __WITH_PCL // the following visitor derived template class definitions require PCL support

#include "sem/core/exception.h"
#include "sem/core/pcl/cloud_.h"
#include "sem/core/pcl/typedefs_fwd.h"
#include "sem/core/pcl/point_traits.h"
#include "sem/core/pcl/vertices.h"
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

    void Reset() {

        c_.reset();
    }

    CloudTPPtr operator()(CloudTPPtr &c) {

        if(!bool(c_)) {

            c_ = c;
        }
        return c_;
    }

    /**
     * @todo avoid double deep copy
     */
    CloudTPPtr operator()(const sem::VecVertices &vv) {

        if(!bool(c_)) {

            c_ = Mat2PointCloud_<TPoint >(VecVertices2Mat(vv, false));
        }
        return c_;
    }

    CloudTPPtr operator()(float f) {

        if(PCLPointTraits_<TPoint>::FieldCount() != 1) {

            std::stringstream s;
            s << "Cannot convert scalar to point" <<
                 PCLPointTraits_<TPoint>::FieldCount() << " fields required.";
            SEM_THROW_TYPE_ERROR(s.str());
        }
        SEM_THROW_NOT_IMPLEMENTED;
    }

    CloudTPPtr operator()(int n) {

        if(PCLPointTraits_<TPoint>::FieldCount() != 1) {

            std::stringstream s;
            s << "Cannot convert scalar to point" <<
                 PCLPointTraits_<TPoint>::FieldCount() << " fields required.";
            SEM_THROW_TYPE_ERROR(s.str());
        }
        SEM_THROW_NOT_IMPLEMENTED;
    }

    CloudTPPtr operator()(uchar c) {

        if(PCLPointTraits_<TPoint>::FieldCount() != 1) {

            std::stringstream s;
            s << "Cannot convert scalar to point" <<
                 PCLPointTraits_<TPoint>::FieldCount() << " fields required.";
            SEM_THROW_TYPE_ERROR(s.str());
        }
        SEM_THROW_NOT_IMPLEMENTED;
    }

    CloudTPPtr operator()(const sem::Mat_f &m) {

        if(!bool(c_)) {

            c_ = Mat2PointCloud_<TPoint>(m);
        }
        return c_;
    }

    /**
     * @todo Possible redundant double deep copy. If true, avoid it.
     */
    template <class TPointArg >
    CloudTPPtr operator()(boost::shared_ptr<pcl::PointCloud<TPointArg > > &cld_arg) {

        if(!bool(c_)) {

            c_ = Mat2PointCloud_<TPoint>(PointCloud2Mat_<TPointArg >(cld_arg));
        }
        return c_;
    }

protected:
     CloudTPPtr c_; ///< internal reference for caching most recent conversion result
};

} // namespace sem

#else // __WITH_PCL
    #warning "Unable to define VisitorCloud_ visitor without PCL support."
#endif // __WITH_PCL

#endif // SEM_CORE_VISITORCLOUD__H_
