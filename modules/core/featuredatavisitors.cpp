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

CloudXYZ::Ptr FeatDataVisitorCloud::operator()(const Mat_f &m)
{
    if(!bool(c_)) {

        c_ = Mat2PointCloud(m);
    }
    return c_;
}
