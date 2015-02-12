/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/**
 * @file Define variant visitor classes around Mat <-> X type conversions
 * @todo switchelm::Mat_f to basic Mat
 */
#ifndef _ELM_CORE_VISITORMAT__H_
#define _ELM_CORE_VISITORMAT__H_

#include "elm/core/typedefs_fwd.h"
#include "elm/core/visitors/visitor_.h"

#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {
/**
 * @brief visitor class for converting to Mat of floats
 */
class VisitorMat_f :
        public Visitor_<elm::Mat_f >
{
public:
   elm::Mat_f operator()(const cv::Mat &m) const;

   elm::Mat_f operator()(const elm::Mat_f &m) const;

   elm::Mat_f operator()(const elm::SparseMat1f &m) const;

   elm::Mat_f operator()(float f) const;

   elm::Mat_f operator()(int n) const;

   elm::Mat_f operator()(uchar c) const;

#ifdef __WITH_PCL // this conversion requires PCL support

   elm::Mat_f operator()(elm::CloudXYZPtr &c) const;

   elm::Mat_f operator()(elm::CloudNrmlPtr &c) const;

   elm::Mat_f operator()(elm::CloudPtNrmlPtr &c) const;

   elm::Mat_f operator()(const elm::VecVertices &vv) const;

#endif // __WITH_PCL
};

} // namespace elm

#endif // _ELM_CORE_VISITORMAT__H_
