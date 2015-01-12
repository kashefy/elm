#include "core/visitors.h"

using namespace cv;
using namespace sem;

base_ConversionCache::~base_ConversionCache()
{
}

void base_ConversionCache::Reset()
{
}

base_ConversionCache::base_ConversionCache()
{
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

Mat_f VisitorMat_f::operator()(CloudXYZ::Ptr &c) const
{
    return PointCloud2Mat(c);
}

void VisitorCloud::Reset()
{
    c_.reset();
}

CloudXYZ::Ptr VisitorCloud::operator()(CloudXYZ::Ptr &c)
{
    if(!bool(c_)) {

        c_ = c;
    }
    return c_;
}

CloudXYZ::Ptr VisitorCloud::operator()(const VecVertices &vv)
{
    if(!bool(c_)) {

        c_ = Mat2PointCloud(VecVertices2Mat(vv, false));
    }
    return c_;
}

CloudXYZ::Ptr VisitorCloud::operator()(float f)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from float");
}

CloudXYZ::Ptr VisitorCloud::operator()(int n)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from int");
}

CloudXYZ::Ptr VisitorCloud::operator()(uchar c)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from uchar");
}

CloudXYZ::Ptr VisitorCloud::operator()(const Mat_f &m)
{
    if(!bool(c_)) {

        c_ = Mat2PointCloud(m);
    }
    return c_;
}

// imeplemnt VecVertices visitor methods
void VisitorVecVertices::Reset()
{
    vv_.clear();
}

VecVertices VisitorVecVertices::operator()(const VecVertices &vv)
{
    if(vv_.empty()) {

        vv_ = vv;
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(CloudXYZ::Ptr &c)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(PointCloud2Mat(c)); // or keep cache?
    }
    return vv_;
}

VecVertices VisitorVecVertices::operator()(float f)
{
    FromScalar(f);
    return vv_;
}

VecVertices VisitorVecVertices::operator()(int n)
{
    FromScalar(n);
    return vv_;
}

VecVertices VisitorVecVertices::operator()(uchar c)
{
    FromScalar(c);
    return vv_;
}

VecVertices VisitorVecVertices::operator()(const Mat_f &m)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(m);
    }
    return vv_;
}

#endif // __WITH_PCL
