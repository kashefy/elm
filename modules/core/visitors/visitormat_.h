/**
 * @file Define variant visitor classes around Mat <-> X type conversions
 * @todo switch Mat_f to basic Mat
 */
#ifndef SEM_CORE_VISITORMAT__H_
#define SEM_CORE_VISITORMAT__H_

#include "core/cv/typedefs_fwd.h"
#include "core/visitors/visitor_.h"

#include "core/pcl/typedefs_fwd.h"

/**
 * @brief visitor class for converting to Mat of floats
 */
class VisitorMat_f :
        public Visitor_<Mat_f >
{
public:
    Mat_f operator()(const cv::Mat &m) const;

    Mat_f operator()(const Mat_f &m) const;

    Mat_f operator()(float f) const;

    Mat_f operator()(int n) const;

    Mat_f operator()(uchar c) const;

#ifdef __WITH_PCL // this conversion requires PCL support

    Mat_f operator()(sem::CloudXYZPtr &c) const;

    Mat_f operator()(const sem::VecVertices &vv) const;

#endif // __WITH_PCL
};

#endif // SEM_CORE_VISITORMAT__H_
