#ifndef SEM_CORE_VISITORPOD__H_
#define SEM_CORE_VISITORPOD__H_

#include "sem/core/visitors/visitor_.h"

#include "sem/core/exception.h"
#include "sem/core/cv/mat_utils.h"
#include "sem/core/pcl/cloud_.h"
#include "sem/core/pcl/vertices.h"
#include "sem/core/typedefs_fwd.h"

/**
 * @brief template class for scalar POD static visitors
 */
template <typename T>
class VisitorPOD_ :
        public boost::static_visitor<T >,
        public base_ConversionCache
{
public:
    T operator()(const sem::Mat_f &m) const
    {
        size_t n = m.total();
        if(n != 1) {

            std::stringstream s;
            s << "Cannot convert " << n << "-element Mat to Scalar.";
            SEM_THROW_BAD_DIMS(s.str());
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
        SEM_THROW_TYPE_ERROR("Cannot convert point cloud to scalar.");
    }

    T operator()(const sem::VecVertices &vv) const
    {
        if(vv.size() == 1) {

            if(vv[0].vertices.size() != 1) {

                SEM_THROW_BAD_DIMS("Can only convert Vertices of size 1 to scalar.");
            }
            else {
                return static_cast<T>(vv[0].vertices[0]);
            }
        }
        else {

            SEM_THROW_BAD_DIMS("Can only convert VecVertices of size 1 to scalar.");
        }
    }
#endif // __WITH_PCL
};

#endif // SEM_CORE_VISITORPOD__H_
