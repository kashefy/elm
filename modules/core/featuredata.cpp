#include "core/featuredata.h"

using namespace std;
using namespace pcl;
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

Mat_<float> FeatDataVisitorMatF::operator()(const Mat_<float> &m) const
{
    return m;
}

Mat_<float> FeatDataVisitorMatF::operator()(CloudXYZ::Ptr &c) const
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

CloudXYZ::Ptr FeatDataVisitorCloud::operator()(const Mat_<float> &m)
{
    if(!bool(c_)) {

        c_ = Mat2PointCloud(m);
    }
    return c_;
}

FeatureData::FeatureData()
{
    Init();
}

// explicit specialization for template method get()
template<>
CloudXYZ::Ptr FeatureData::get()
{
    return var_.apply_visitor(visitor_cloud_);
}

template<>
Mat_<float> FeatureData::get()
{
    return var_.apply_visitor(visitor_mat_);
}

void FeatureData::Init()
{
    Reset();
}

void FeatureData::Reset()
{
    visitor_cloud_.Reset();
    visitor_mat_.Reset();
}
