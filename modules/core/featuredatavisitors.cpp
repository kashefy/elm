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

#endif // __WITH_PCL
