#include "core/featuredata.h"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace sem;

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
