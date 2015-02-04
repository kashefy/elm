/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
#ifndef _ELM_CORE_VISITORPOD__H_
#define _ELM_CORE_VISITORPOD__H_

#include "elm/core/visitors/visitor_.h"

#include "elm/core/exception.h"
#include "elm/core/cv/mat_utils.h"
#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/vertices.h"
#include "elm/core/typedefs_fwd.h"

/**
 * @brief template class for scalar POD static visitors
 */
template <typename T>
class VisitorPOD_ :
        public boost::static_visitor<T >,
        public base_ConversionCache
{
public:
    T operator()(const elm::Mat_f &m) const
    {
        size_t n = m.total();
        if(n != 1) {

            std::stringstream s;
            s << "Cannot convert " << n << "-element Mat to Scalar.";
            ELM_THROW_BAD_DIMS(s.str());
        }
        else {
            return static_cast<T>(m(0));
        }
    }

    template <class TPar>
    T operator()(TPar t) const
    {
        return static_cast<T>(t);
    }

#ifdef __WITH_PCL // PCL support required
    template <class TPoint>
    T operator()(boost::shared_ptr<pcl::PointCloud<TPoint > > &c) const
    {
        ELM_THROW_TYPE_ERROR("Cannot convert point cloud to scalar.");
    }

    T operator()(const elm::VecVertices &vv) const
    {
        if(vv.size() == 1) {

            if(vv[0].vertices.size() != 1) {

                ELM_THROW_BAD_DIMS("Can only convert Vertices of size 1 to scalar.");
            }
            else {
                return static_cast<T>(vv[0].vertices[0]);
            }
        }
        else {

            ELM_THROW_BAD_DIMS("Can only convert VecVertices of size 1 to scalar.");
        }
    }
#endif // __WITH_PCL
};

#endif // _ELM_CORE_VISITORPOD__H_
