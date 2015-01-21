/**
 * @file Define variant visitor classes around Mat <-> X type conversions
 * @todo switchsem::Mat_f to basic Mat
 */
#ifndef SEM_CORE_VISITORMAT__H_
#define SEM_CORE_VISITORMAT__H_

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
   sem::Mat_f operator()(const cv::Mat &m) const;

   sem::Mat_f operator()(const sem::Mat_f &m) const;

   sem::Mat_f operator()(float f) const;

   sem::Mat_f operator()(int n) const;

   sem::Mat_f operator()(uchar c) const;

#ifdef __WITH_PCL // this conversion requires PCL support

   sem::Mat_f operator()(sem::CloudXYZPtr &c) const;

   sem::Mat_f operator()(sem::CloudNrmlPtr &c) const;

   sem::Mat_f operator()(sem::CloudPtNrmlPtr &c) const;

   sem::Mat_f operator()(const sem::VecVertices &vv) const;

#endif // __WITH_PCL
};

#endif // SEM_CORE_VISITORMAT__H_
