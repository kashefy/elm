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
#include "elm/core/cv/sparsemat_utils.h"
#include "elm/core/pcl/cloud_.h"
#include "elm/core/pcl/vertices.h"
#include "elm/core/typedefs_sfwd.h"

extern template class cv::Mat_<float>;
extern template class cv::SparseMat_<float>;
extern template class std::vector<cv::Mat_<float> >;

namespace elm {

/**
 * @brief template class for scalar POD static visitors
 */
template <typename T>
class VisitorPOD_ :
        public boost::static_visitor<T >,
        public base_ConversionCache
{
public:
    T operator()(const cv::Mat1f &m) const
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

    T operator()(const elm::SparseMat1f &m) const
    {
        size_t n = elm::total(m);
        if(n != 1) {

            std::stringstream s;
            s << "Cannot convert " << n << "-element Mat to Scalar.";
            ELM_THROW_BAD_DIMS(s.str());
        }
        else {

            T value;
            const int D = m.dims();
            switch(D){

            case 1:
                value = static_cast<T>(m(0));
                break;
            case 2:
                value = static_cast<T>(m(0, 0));
                break;
            case 3:
                value = static_cast<T>(m(0, 0, 0));
                break;
            default:
                int *idx = new int[D];
                for(int i=0; i<D; i++) {
                    idx[i] = 0;
                }

                value = static_cast<T>(m(idx));

                delete[] idx;
            }
            return value;
        }
    }

    T operator()(const elm::VecMat1f &v) const
    {
        size_t n = v.size();
        if(n != 1) {

            std::stringstream s;
            s << "Cannot convert " << n << "-element VecMat1f to Scalar.";
            ELM_THROW_BAD_DIMS(s.str());
        }
        else {
            return static_cast<T>(operator ()(v[0]));
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

} // namespace elm

#endif // _ELM_CORE_VISITORPOD__H_
