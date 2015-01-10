#include "core/featuredata.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace sem;

void FeatDataConversionCache::Reset()
{
}

Mat_<float> FeatDataVisitorMatF::operator()(const Mat_<float> &m) const
{
    return m;
}

Mat_<float> FeatDataVisitorMatF::operator()(PointCloudXYZ::Ptr &c) const
{
    return PointCloud2Mat(c);
}

void FeatDataVisitorCloud::Reset()
{
    c_.reset();
}

PointCloudXYZ::Ptr FeatDataVisitorCloud::operator()(PointCloudXYZ::Ptr &c)
{
    if(!bool(c_)) {

        c_ = c;
    }
    return c_;
}

PointCloudXYZ::Ptr FeatDataVisitorCloud::operator()(const Mat_<float> &m)
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
PointCloudXYZ::Ptr FeatureData::get()
{
    return var_.apply_visitor(visitor_cloud_);
}

template<>
Mat_<float> FeatureData::get()
{
    return var_.apply_visitor(visitor_mat_);
}

//// explicit specialization for template conversion method
//template<>
//PointCloudXYZ::Ptr FeatureData::operator PointCloudXYZ::Ptr()
//{
//    return get<PointCloudXYZ>();
//}


void FeatureData::Init()
{
    Reset();
}

void FeatureData::Reset()
{
    visitor_cloud_.Reset();
    visitor_mat_.Reset();
}
