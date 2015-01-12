#include "core/featuredatavisitors.h"

using namespace cv;
using namespace sem;

base_FeatDataConversionCache::~base_FeatDataConversionCache()
{
}

void base_FeatDataConversionCache::Reset()
{
}

base_FeatDataConversionCache::base_FeatDataConversionCache()
{
}

Mat_f FeatDataVisitorMat_f::operator()(const Mat_f &m) const
{
    return m;
}

Mat_f FeatDataVisitorMat_f::operator()(float f) const
{
    return Mat_f(1, 1, f);
}

Mat_f FeatDataVisitorMat_f::operator()(int n) const
{
    return Mat_f(1, 1, static_cast<float>(n));
}

Mat_f FeatDataVisitorMat_f::operator()(uchar c) const
{
    return Mat_f(1, 1, static_cast<float>(c));
}

#ifdef __WITH_PCL // definitions below require PCL support

Mat_f FeatDataVisitorMat_f::operator()(const VecVertices &vv) const
{
    return VecVertices2Mat(vv, false);
}

Mat_f FeatDataVisitorMat_f::operator()(CloudXYZ::Ptr &c) const
{
    return PointCloud2Mat(c);
}

void FeatDataVisitorCloud::Reset()
{
    c_.reset();
}

CloudXYZ::Ptr FeatDataVisitorCloud::operator()(CloudXYZ::Ptr &c)
{
    if(!bool(c_)) {

        c_ = c;
    }
    return c_;
}

CloudXYZ::Ptr FeatDataVisitorCloud::operator()(float f)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from float");
}

CloudXYZ::Ptr FeatDataVisitorCloud::operator()(int n)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from int");
}

CloudXYZ::Ptr FeatDataVisitorCloud::operator()(uchar c)
{
    Reset();
    SEM_THROW_TYPE_ERROR("Cannot create point cloud from uchar");
}

CloudXYZ::Ptr FeatDataVisitorCloud::operator()(const Mat_f &m)
{
    if(!bool(c_)) {

        c_ = Mat2PointCloud(m);
    }
    return c_;
}

// imeplemnt VecVertices visitor methods
void FeatDataVisitorVecVertices::Reset()
{
    vv_.clear();
}

VecVertices FeatDataVisitorVecVertices::operator()(const VecVertices &vv)
{
    if(vv_.empty()) {

        vv_ = vv;
    }
    return vv_;
}

VecVertices FeatDataVisitorVecVertices::operator()(CloudXYZ::Ptr &c)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(PointCloud2Mat(c)); // or keep cache?
    }
    return vv_;
}

VecVertices FeatDataVisitorVecVertices::operator()(float f)
{
    FromScalar(f);
    return vv_;
}

VecVertices FeatDataVisitorVecVertices::operator()(int n)
{
    FromScalar(n);
    return vv_;
}

VecVertices FeatDataVisitorVecVertices::operator()(uchar c)
{
    FromScalar(c);
    return vv_;
}

VecVertices FeatDataVisitorVecVertices::operator()(const Mat_f &m)
{
    if(vv_.empty()) {

        vv_ = Mat2VecVertices(m);
    }
    return vv_;
}

#endif // __WITH_PCL
