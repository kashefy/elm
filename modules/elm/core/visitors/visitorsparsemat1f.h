/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/**
 * @file Define variant visitor classes around SparseMat1f <-> X type conversions
 */
#ifndef _ELM_CORE_VISITORSPARSEMAT1f_H_
#define _ELM_CORE_VISITORSPARSEMAT1f_H_

#include "elm/core/typedefs_sfwd.h"
#include "elm/core/visitors/visitor_.h"

#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {

/**
 * @brief visitor class for converting to a sparse Mat of floats
 */
class VisitorSparseMat1f :
        public Visitor_<elm::SparseMat1f >
{
public:
    elm::SparseMat1f operator()(const elm::SparseMat1f &m) const;

    /**
     * @throws elm::ExceptionBadDims on empty input
     */
    elm::SparseMat1f operator()(const cv::Mat &m) const;

    elm::SparseMat1f operator()(const cv::Mat1f &m) const;

    elm::SparseMat1f operator()(const elm::VecMat1f &v) const;

    elm::SparseMat1f operator()(float f) const;

    elm::SparseMat1f operator()(int n) const;

    elm::SparseMat1f operator()(uchar c) const;

#ifdef __WITH_PCL // this conversion requires PCL support

    elm::SparseMat1f operator()(elm::CloudXYZPtr &c) const;

    elm::SparseMat1f operator()(elm::CloudNrmlPtr &c) const;

    elm::SparseMat1f operator()(elm::CloudPtNrmlPtr &c) const;

    elm::SparseMat1f operator()(const elm::VecVertices &vv) const;

#endif // __WITH_PCL
};

} // namespace elm

#endif // _ELM_CORE_VISITORSPARSEMAT1f_H_
