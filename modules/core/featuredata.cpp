#include "core/featuredata.h"

using namespace std;
using namespace cv;
using namespace sem;

FeatureData::FeatureData()
{
    Init();
}

// explicit specialization for template method get()
#ifdef __WITH_PCL // PCL support required
template<>
CloudXYZ::Ptr FeatureData::get()
{
    return var_.apply_visitor(visitor_cloud_);
}
#endif // __WITH_PCL

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
