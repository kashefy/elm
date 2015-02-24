/*M///////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, Youssef Kashef
// Copyright (c) 2015, ELM Library Project
// 3-clause BSD License
//
//M*/
/**
 * @file Define variant visitor classes around Mat1f <-> X type conversions
 */
#ifndef _ELM_CORE_VISITORMAT1f_H_
#define _ELM_CORE_VISITORMAT1f_H_

#include "elm/core/typedefs_sfwd.h"
#include "elm/core/visitors/visitor_.h"

#include "elm/core/pcl/typedefs_fwd.h"

namespace elm {
/**
 * @brief visitor class for converting to Mat of floats
 */
class VisitorMat1f :
        public Visitor_<cv::Mat1f >
{
public:
   cv::Mat1f operator()(const cv::Mat &m) const;

   cv::Mat1f operator()(const cv::Mat1f &m) const;

   cv::Mat1f operator()(const elm::SparseMat1f &m) const;

   cv::Mat1f operator()(const elm::VecMat1f &v) const;

   cv::Mat1f operator()(float f) const;

   cv::Mat1f operator()(int n) const;

   cv::Mat1f operator()(uchar c) const;

#ifdef __WITH_PCL // this conversion requires PCL support

   cv::Mat1f operator()(elm::CloudXYZPtr &c) const;

   cv::Mat1f operator()(elm::CloudNrmlPtr &c) const;

   cv::Mat1f operator()(elm::CloudPtNrmlPtr &c) const;

   cv::Mat1f operator()(const elm::VecVertices &vv) const;

#endif // __WITH_PCL
};

} // namespace elm

#endif // _ELM_CORE_VISITORMAT1f_H_
