/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/**
 * @file Define variant visitor classes around VecMat1f <-> X type conversions
 */
#ifndef _ELM_CORE_VISITORVECMAT1f_H_
#define _ELM_CORE_VISITORVECMAT1f_H_

#include "elm/core/typedefs_sfwd.h"
#include "elm/core/visitors/visitor_.h"

#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {
/**
 * @brief visitor class for converting to vector of Mats of floats
 */
class VisitorVecMat1f :
        public Visitor_<VecMat1f >
{
public:
    VecMat1f operator()(const VecMat1f &m) const;

    VecMat1f operator()(const cv::Mat &m) const;

    VecMat1f operator()(const cv::Mat1f &m) const;

    VecMat1f operator()(const elm::SparseMat1f &m) const;

    VecMat1f operator()(float f) const;

    VecMat1f operator()(int n) const;

	VecMat1f operator()(pod::uchar c) const;

#ifdef __WITH_PCL // this conversion requires PCL support

    VecMat1f operator()(elm::CloudXYZPtr &c) const;

    VecMat1f operator()(elm::CloudNrmlPtr &c) const;

    VecMat1f operator()(elm::CloudPtNrmlPtr &c) const;

    VecMat1f operator()(const elm::VecVertices &vv) const;

#endif // __WITH_PCL
};

} // namespace elm

#endif // _ELM_CORE_VISITORVECMAT1f_H_
