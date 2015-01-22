/**
 * @file Define variant visitor classes around Mat <-> X type conversions
 * @todo switchsem::Mat_f to basic Mat
 */
#ifndef ELM_CORE_VISITORMAT__H_
#define ELM_CORE_VISITORMAT__H_

#include "sem/core/typedefs_fwd.h"
#include "sem/core/visitors/visitor_.h"

#include "sem/core/pcl/typedefs_fwd.h"

/**
 * @brief visitor class for converting to Mat of floats
 */
class VisitorMat_f :
        public Visitor_<sem::Mat_f >
{
public:
   elm::Mat_f operator()(const cv::Mat &m) const;

   elm::Mat_f operator()(const elm::Mat_f &m) const;

   elm::Mat_f operator()(float f) const;

   elm::Mat_f operator()(int n) const;

   elm::Mat_f operator()(uchar c) const;

#ifdef __WITH_PCL // this conversion requires PCL support

   elm::Mat_f operator()(sem::CloudXYZPtr &c) const;

   elm::Mat_f operator()(sem::CloudNrmlPtr &c) const;

   elm::Mat_f operator()(sem::CloudPtNrmlPtr &c) const;

   elm::Mat_f operator()(const elm::VecVertices &vv) const;

#endif // __WITH_PCL
};

#endif // ELM_CORE_VISITORMAT__H_
