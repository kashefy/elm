#include "sem/core/visitors/visitormat_.h"

#include <opencv2/core.hpp>

#include "sem/core/pcl/cloud_.h"
#include "sem/core/pcl/vertices.h"

using namespace cv;
using namespace sem;

Mat_f VisitorMat_f::operator()(const Mat &m) const
{
    if((m.type() & CV_MAT_DEPTH_MASK) != (CV_32F & CV_MAT_DEPTH_MASK)) {

        Mat mf;
        m.convertTo(mf, CV_MAKE_TYPE(CV_32F, m.channels()));
        return mf;
    }
    else {
        return m;
    }
}

Mat_f VisitorMat_f::operator()(const Mat_f &m) const
{
    return m;
}

Mat_f VisitorMat_f::operator()(float f) const
{
    return Mat_f(1, 1, f);
}

Mat_f VisitorMat_f::operator()(int n) const
{
    return Mat_f(1, 1, static_cast<float>(n));
}

Mat_f VisitorMat_f::operator()(uchar c) const
{
    return Mat_f(1, 1, static_cast<float>(c));
}

#ifdef __WITH_PCL // definitions below require PCL support

Mat_f VisitorMat_f::operator()(const VecVertices &vv) const
{
    return VecVertices2Mat(vv, false);
}

Mat_f VisitorMat_f::operator()(CloudXYZPtr &c) const
{
    return PointCloud2Mat_<pcl::PointXYZ>(c);
}

Mat_f VisitorMat_f::operator()(CloudNrmlPtr &c) const
{
    return PointCloud2Mat_<pcl::Normal>(c);
}

Mat_f VisitorMat_f::operator()(CloudPtNrmlPtr &c) const
{
    return PointCloud2Mat_<pcl::PointNormal>(c);
}

#endif // __WITH_PCL
