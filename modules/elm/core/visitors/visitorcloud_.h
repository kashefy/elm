/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, Elm Library Project
// 3-clause BSD License
//
//M*/
#ifndef ELM_CORE_VISITORCLOUD__H_
#define ELM_CORE_VISITORCLOUD__H_

#ifdef __WITH_PCL // the following visitor derived template class definitions require PCL support

#include "elm/core/pcl/cloud_2cloud_.h"
#include "elm/core/pcl/vertices.h"
#include "elm/core/cv/typedefs_fwd.h"
#include "elm/core/typedefs_fwd.h"
#include "elm/core/visitors/visitor_.h"

#include <iostream>

namespace elm {

/**
 * @brief A template visitor class for converting to pcl point clouds,
 * and keeping track of when a heavy conversion (involving deep copy) occured
 *
 * Requires PCL support.
 */
template <class TPointDst>
class VisitorCloud_ :
        public Visitor_<boost::shared_ptr<pcl::PointCloud<TPointDst > > >
{
public:
    typedef boost::shared_ptr<pcl::PointCloud<TPointDst > > CloudTPDstPtr;

    void Reset() {

        c_.reset();
    }

    /**
     * @brief Convert one point cloud to another (see specializations)
     */
    template <class TPointSrc >
    CloudTPDstPtr operator()(boost::shared_ptr<pcl::PointCloud<TPointSrc > > &cld_src) {

        if(!bool(c_)) {

            Cloud_2Cloud_<TPointSrc, TPointDst>::Convert(cld_src, c_);
        }
        return c_;
    }

    CloudTPDstPtr operator()(CloudTPDstPtr &c) {

        if(!bool(c_)) {

            c_ = c;
        }
        return c_;
    }

    /**
     * @todo avoid double deep copy
     */
    CloudTPDstPtr operator()(const elm::VecVertices &vv) {

        if(!bool(c_)) {

            c_ = Mat2PointCloud_<TPointDst >(VecVertices2Mat(vv, false));
        }
        return c_;
    }

    CloudTPDstPtr operator()(float f) {

        if(PCLPointTraits_<TPointDst >::FieldCount() != 1) {

            std::stringstream s;
            s << "Cannot convert scalar to point" <<
                 PCLPointTraits_<TPointDst>::FieldCount() << " fields required.";
            ELM_THROW_TYPE_ERROR(s.str());
        }
        ELM_THROW_NOT_IMPLEMENTED;
    }

    CloudTPDstPtr operator()(int n) {

        if(PCLPointTraits_<TPointDst>::FieldCount() != 1) {

            std::stringstream s;
            s << "Cannot convert scalar to point" <<
                 PCLPointTraits_<TPointDst>::FieldCount() << " fields required.";
            ELM_THROW_TYPE_ERROR(s.str());
        }
        ELM_THROW_NOT_IMPLEMENTED;
    }

    CloudTPDstPtr operator()(uchar c) {

        if(PCLPointTraits_<TPointDst>::FieldCount() != 1) {

            std::stringstream s;
            s << "Cannot convert scalar to point" <<
                 PCLPointTraits_<TPointDst>::FieldCount() << " fields required.";
            ELM_THROW_TYPE_ERROR(s.str());
        }
        ELM_THROW_NOT_IMPLEMENTED;
    }

    CloudTPDstPtr operator()(const elm::Mat_f &m) {

        if(!bool(c_)) {

            c_ = Mat2PointCloud_<TPointDst >(m);
        }
        return c_;
    }

protected:
     CloudTPDstPtr c_; ///< internal reference for caching most recent conversion result
};


} // namespace elm

#else // __WITH_PCL
    #warning "Unable to define VisitorCloud_ visitor without PCL support."
#endif // __WITH_PCL

#endif // ELM_CORE_VISITORCLOUD__H_